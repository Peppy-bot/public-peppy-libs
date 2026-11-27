//! A global plan as Nav2 1.5.2's controller server hands it to its
//! controller and its goal checker, through the feasible path handler
//! (nav2_controller plugins/feasible_path_handler.cpp: `setPlan`,
//! `transformToGlobalPlanFrame`, `findPlanSegment`, `transformLocalPlan` and
//! `getTransformedGoal`, with nav2_util's `first_after_integrated_distance`
//! and `min_by` of include/nav2_util/geometry_utils.hpp), its plan in a frame
//! of its own, placed in the cost grid's frame, which the base stands in, with
//! no path inversion or rotation enforced and a plan of one pose taken
//! (`reject_unit_path` false).
//!
//! Each [`GlobalPlan::prune`] reads the base in the plan's frame
//! (feasible_path_handler.cpp:140-161) and finds the plan's pose nearest it
//! among those within `max_robot_pose_search_dist_m` along the plan, never its
//! last pose while it holds two or more (:163-198). It places in the grid's
//! frame the poses from it that lie within `prune_distance_m` along the plan
//! from it, the first past that distance left out as
//! `first_after_integrated_distance` ends the range there
//! (include/nav2_util/geometry_utils.hpp:132-145), up to the first placed off
//! the grid (:201-254), and the plan's goal (:256-272). The poses before the
//! nearest one leave the plan, so the next prune searches from there. Each
//! plan holds its goal checker, made with the plan ([`crate::goal_checker`]).

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::planar_pose::PlanarPose;

use crate::goal_checker::{GoalChecker, GoalCheckerParams};
use crate::grid::GridGeometry;

/// The path handler's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathHandlerParams {
    /// How far along the plan, in meters, the search for the pose nearest
    /// the base reaches (`max_robot_pose_search_dist`).
    pub max_robot_pose_search_dist_m: NonNegativeFinite,
    /// How far along the plan, in meters, from the pose nearest the base
    /// the pruned plan reaches (`prune_distance`).
    pub prune_distance_m: NonNegativeFinite,
}

/// A plan of no pose: the planner gives at least one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("the plan holds no pose: give the planner's path, which holds at least one")]
pub struct EmptyPlan;

/// Why a prune gives no plan.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum PruneError {
    /// The plan's pose nearest the base lies off the grid, as the handler's
    /// `InvalidPath` of no pose in the cost map's frame.
    #[error(
        "the plan's pose nearest the base lies off the cost grid: give a grid that holds the \
         base and the plan ahead of it"
    )]
    OffGrid,
    /// The placement puts the base in the plan's frame, or the plan's goal in
    /// the grid's, at a point that is not finite, as the handler's
    /// `ControllerTFError` of a transform it cannot make.
    #[error(
        "the plan placed in the grid's frame at x {} m, y {} m, yaw {} rad puts the base or the \
         plan's goal at a point that is not finite: give an x and a y that, moved by the \
         base's and the goal's distances from the grid's origin, stay finite",
        .grid_from_plan.x(),
        .grid_from_plan.y(),
        .grid_from_plan.yaw()
    )]
    NotFinite { grid_from_plan: PlanarPose },
}

/// The plan a base follows: the poses it has not passed, the pose the whole
/// plan ends at, and the goal checker that judges it.
#[derive(Debug, PartialEq)]
pub struct GlobalPlan {
    params: PathHandlerParams,
    remaining: Vec<PlanarPose>,
    goal: PlanarPose,
    goal_checker: GoalChecker,
}

/// The poses of a plan ahead of the base, at least one, and the pose the
/// whole plan ends at, in the grid's frame, as the controller and the goal
/// checker read them.
#[derive(Debug, PartialEq)]
pub struct PrunedPlan {
    poses: Vec<PlanarPose>,
    goal: PlanarPose,
}

impl PrunedPlan {
    pub fn poses(&self) -> &[PlanarPose] {
        &self.poses
    }

    /// `getTransformedGoal`: the last pose of the whole plan, in the grid's
    /// frame.
    pub fn goal(&self) -> PlanarPose {
        self.goal
    }
}

impl GlobalPlan {
    /// `setPlannerPath`: the planner's path, in the plan's own frame, set as
    /// the path handler's plan (`setPlan`), with a goal checker of
    /// `goal_checker` that judges the position afresh (`reset`).
    pub fn new(
        poses: Vec<PlanarPose>,
        params: PathHandlerParams,
        goal_checker: GoalCheckerParams,
    ) -> Result<Self, EmptyPlan> {
        let goal = *poses.last().ok_or(EmptyPlan)?;
        Ok(Self {
            params,
            remaining: poses,
            goal,
            goal_checker: GoalChecker::new(goal_checker),
        })
    }

    /// The plan's goal checker, which judges the pruned plans of this plan.
    pub fn goal_checker(&mut self) -> &mut GoalChecker {
        &mut self.goal_checker
    }

    /// `findPlanSegment`, `transformLocalPlan` and `getTransformedGoal` for a
    /// base at `base` on a grid of `geometry`, both in the grid's frame, the
    /// plan's frame placed there at `grid_from_plan`: the base read in the
    /// plan's frame, the pose nearest it found there, and the poses ahead of
    /// the base and the plan's goal placed in the grid's frame, the passed
    /// poses dropped from the plan. Fails when the base in the plan's frame is
    /// not finite, with the plan as it was; when the nearest pose lies off the
    /// grid, after dropping the passed ones; and when the goal in the grid's
    /// frame is not finite, after dropping them.
    pub fn prune(
        &mut self,
        base: PlanarPose,
        grid_from_plan: PlanarPose,
        geometry: &GridGeometry,
    ) -> Result<PrunedPlan, PruneError> {
        let not_finite = PruneError::NotFinite { grid_from_plan };
        let base_in_plan = base.seen_from(grid_from_plan).map_err(|_| not_finite)?;
        let poses = &self.remaining;
        let search_end = first_after_integrated_distance(
            poses,
            0,
            self.params.max_robot_pose_search_dist_m.get(),
        );
        let nearest = nearest_pose(&poses[..search_end], base_in_plan);
        let closest = if poses.len() > 1 && nearest == poses.len() - 1 {
            nearest - 1
        } else {
            nearest
        };
        let pruned_end =
            first_after_integrated_distance(poses, closest, self.params.prune_distance_m.get());
        let ahead: Vec<PlanarPose> = poses[closest..pruned_end]
            .iter()
            .map_while(|&pose| {
                grid_from_plan
                    .compose(pose)
                    .ok()
                    .filter(|placed| geometry.cell_of([placed.x(), placed.y()]).is_some())
            })
            .collect();
        self.remaining.drain(..closest);
        if ahead.is_empty() {
            return Err(PruneError::OffGrid);
        }
        let goal = grid_from_plan.compose(self.goal).map_err(|_| not_finite)?;
        Ok(PrunedPlan { poses: ahead, goal })
    }
}

/// `first_after_integrated_distance` from `start`: the index of the first
/// pose whose distance along the plan from `start` exceeds `limit`, or the
/// plan's length when none does.
fn first_after_integrated_distance(poses: &[PlanarPose], start: usize, limit: f64) -> usize {
    let mut integrated = 0.0;
    for (index, step) in poses[start..].windows(2).enumerate() {
        integrated += step[0].distance_to(step[1]);
        if integrated > limit {
            return start + index + 1;
        }
    }
    poses.len()
}

/// `min_by` of the distance to `pose`: the index of the nearest of `poses`,
/// the last of equally near ones.
fn nearest_pose(poses: &[PlanarPose], pose: PlanarPose) -> usize {
    let mut nearest = 0;
    let mut lowest = pose.distance_to(poses[0]);
    for (index, &candidate) in poses.iter().enumerate().skip(1) {
        let here = pose.distance_to(candidate);
        if here <= lowest {
            lowest = here;
            nearest = index;
        }
    }
    nearest
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    use super::*;
    use control_core::positive_finite::PositiveFinite;

    /// The placement of a plan in the grid's own frame.
    const IDENTITY: PlanarPose = PlanarPose::IDENTITY;

    fn goal_checker() -> GoalCheckerParams {
        let tolerance = NonNegativeFinite::parse(0.25).unwrap();
        GoalCheckerParams {
            xy_goal_tolerance_m: tolerance,
            xy_goal_tolerance_buffer_m: tolerance,
            yaw_goal_tolerance_rad: tolerance,
            path_length_tolerance_m: NonNegativeFinite::parse(1.0).unwrap(),
        }
    }

    fn pose(x: f64, y: f64) -> PlanarPose {
        PlanarPose::new(x, y, 0.0).unwrap()
    }

    fn params(search_m: f64, prune_m: f64) -> PathHandlerParams {
        PathHandlerParams {
            max_robot_pose_search_dist_m: NonNegativeFinite::parse(search_m).unwrap(),
            prune_distance_m: NonNegativeFinite::parse(prune_m).unwrap(),
        }
    }

    fn line(count: usize) -> Vec<PlanarPose> {
        (0..count).map(|i| pose(i as f64 * 0.5, 0.0)).collect()
    }

    fn geometry() -> GridGeometry {
        GridGeometry::new(40, 10, PositiveFinite::parse(0.25).unwrap(), [-1.0, -1.25]).unwrap()
    }

    #[test]
    fn a_plan_of_no_pose_is_refused() {
        assert_eq!(
            GlobalPlan::new(Vec::new(), params(1.0, 1.0), goal_checker()),
            Err(EmptyPlan)
        );
    }

    fn xs(pruned: &PrunedPlan) -> Vec<f64> {
        pruned.poses().iter().map(|p| p.x()).collect()
    }

    // The base at 1.1 m stands nearest the pose at 1.0 m: the pruned plan
    // runs from it through the poses within 1.2 m along the plan, leaving out
    // the pose at 2.5 m, the first past that distance; the poses before the
    // one at 1.0 m leave the plan, so a base back at the start next finds the
    // pose at 1.0 m nearest.
    #[test]
    fn a_prune_drops_the_passed_poses_and_reaches_the_prune_distance() {
        let mut plan = GlobalPlan::new(line(10), params(10.0, 1.2), goal_checker()).unwrap();
        let pruned = plan.prune(pose(1.1, 0.2), IDENTITY, &geometry()).unwrap();
        assert_eq!(xs(&pruned), [1.0, 1.5, 2.0]);
        let again = plan.prune(pose(0.0, 0.0), IDENTITY, &geometry()).unwrap();
        assert_eq!(xs(&again), [1.0, 1.5, 2.0]);
        assert_eq!(again.goal().x(), 4.5);
    }

    // The search reaches only the poses within its distance along the plan,
    // so a base beside the plan's end still takes a pose near the start.
    #[test]
    fn the_search_stops_at_its_distance_along_the_plan() {
        let mut plan = GlobalPlan::new(line(10), params(0.6, 0.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(4.5, 0.0), IDENTITY, &geometry()).unwrap();
        assert_eq!(pruned.poses()[0].x(), 0.5);
    }

    // A base past the plan's end takes the last pose but one, so the pruned
    // plan holds the last two.
    #[test]
    fn the_last_pose_is_never_the_nearest_of_two_or_more() {
        let mut plan = GlobalPlan::new(line(4), params(10.0, 10.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(3.0, 0.0), IDENTITY, &geometry()).unwrap();
        assert_eq!(xs(&pruned), [1.0, 1.5]);
    }

    // Poses past the grid's edge end the pruned plan, and a nearest pose
    // off the grid gives none, its passed poses dropped all the same.
    #[test]
    fn the_pruned_plan_ends_at_the_grid_edge() {
        let far: Vec<PlanarPose> = (0..30).map(|i| pose(i as f64 * 0.5, 0.0)).collect();
        let mut plan = GlobalPlan::new(far, params(100.0, 100.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(0.0, 0.0), IDENTITY, &geometry()).unwrap();
        assert_eq!(pruned.poses().last().unwrap().x(), 8.5);
        assert_eq!(
            plan.prune(pose(12.0, 0.0), IDENTITY, &geometry()),
            Err(PruneError::OffGrid)
        );
        assert_eq!(
            plan.prune(pose(0.0, 0.0), IDENTITY, &geometry()),
            Err(PruneError::OffGrid)
        );
    }

    /// Whether `placed` stands at `x`, `y` and `yaw`, to rounding.
    fn at(placed: PlanarPose, x: f64, y: f64, yaw: f64) -> bool {
        [placed.x() - x, placed.y() - y, placed.yaw() - yaw]
            .iter()
            .all(|difference| difference.abs() < 1e-12)
    }

    // A plan whose frame the grid's places a quarter turn about (1, 2): the
    // base, given in the grid's frame at the plan's (1.1, 0.2), stands nearest
    // the pose at 1.0 m, and the pruned poses and the goal come back in the
    // grid's frame, turned a quarter turn and moved; the grid the plan lies on
    // in its own frame holds none of the placed poses.
    #[test]
    fn a_plan_is_pruned_in_its_own_frame_and_placed_in_the_grids() {
        let quarter = PlanarPose::new(1.0, 2.0, FRAC_PI_2).unwrap();
        let around =
            GridGeometry::new(40, 40, PositiveFinite::parse(0.25).unwrap(), [-5.0; 2]).unwrap();
        let mut plan = GlobalPlan::new(line(10), params(10.0, 1.2), goal_checker()).unwrap();
        let base = quarter.compose(pose(1.1, 0.2)).unwrap();
        let pruned = plan.prune(base, quarter, &around).unwrap();
        assert_eq!(pruned.poses().len(), 3);
        for (placed, x) in pruned.poses().iter().zip([1.0, 1.5, 2.0]) {
            assert!(at(*placed, 1.0, 2.0 + x, FRAC_PI_2), "{placed:?} for {x}");
        }
        assert!(
            at(pruned.goal(), 1.0, 6.5, FRAC_PI_2),
            "{:?}",
            pruned.goal()
        );
        assert_eq!(
            plan.prune(base, quarter, &geometry()),
            Err(PruneError::OffGrid)
        );
    }

    // A placement that carries the base past the finite numbers in the plan's
    // frame is refused, naming the placement, and the plan keeps its poses.
    #[test]
    fn a_placement_past_the_finite_numbers_is_refused_and_drops_nothing() {
        let far = PlanarPose::new(-f64::MAX, -f64::MAX, FRAC_PI_4).unwrap();
        let mut plan = GlobalPlan::new(line(10), params(10.0, 1.2), goal_checker()).unwrap();
        let refused = plan.prune(pose(0.0, 0.0), far, &geometry()).unwrap_err();
        assert_eq!(
            refused,
            PruneError::NotFinite {
                grid_from_plan: far
            }
        );
        assert!(
            refused
                .to_string()
                .starts_with("the plan placed in the grid's frame at x "),
            "{refused}"
        );
        let kept = plan.prune(pose(0.0, 0.0), IDENTITY, &geometry()).unwrap();
        assert_eq!(kept.poses()[0].x(), 0.0);
    }
}
