//! A global plan as Nav2 1.5.2's controller server hands it to its
//! controller and its goal checker, through the feasible path handler
//! (nav2_controller plugins/feasible_path_handler.cpp: `setPlan`,
//! `findPlanSegment`, `transformLocalPlan` and `getTransformedGoal`, with
//! nav2_util's `first_after_integrated_distance` and `min_by` of
//! include/nav2_util/geometry_utils.hpp), its plan, the base and the cost
//! grid in one frame, with no path inversion or rotation enforced and a plan
//! of one pose taken (`reject_unit_path` false).
//!
//! Each [`GlobalPlan::prune`] finds the plan's pose nearest the base among
//! those within `max_robot_pose_search_dist_m` along the plan, never its
//! last pose while it holds two or more, and gives the poses from it that lie
//! within `prune_distance_m` along the plan from it, the first past that
//! distance left out as `first_after_integrated_distance` ends the range
//! there (include/nav2_util/geometry_utils.hpp:132-145), up to the first that
//! lies off the grid. The poses before the nearest one leave the plan,
//! so the next prune searches from there. Each plan holds its goal checker,
//! made with the plan ([`crate::goal_checker`]).

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

/// Why a prune gives no pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "the plan's pose nearest the base lies off the cost grid: give a grid that holds the base \
     and the plan ahead of it"
)]
pub struct PlanOffGrid;

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
/// whole plan ends at, as the controller and the goal checker read them.
#[derive(Debug, PartialEq)]
pub struct PrunedPlan {
    poses: Vec<PlanarPose>,
    goal: PlanarPose,
}

impl PrunedPlan {
    pub fn poses(&self) -> &[PlanarPose] {
        &self.poses
    }

    /// `getTransformedGoal`: the last pose of the whole plan.
    pub fn goal(&self) -> PlanarPose {
        self.goal
    }
}

impl GlobalPlan {
    /// `setPlannerPath`: the planner's path, in the frame of the base and the
    /// grid, set as the path handler's plan (`setPlan`), with a goal checker
    /// of `goal_checker` that judges the position afresh (`reset`).
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

    /// `findPlanSegment` then `transformLocalPlan` for a base at `pose` on a
    /// grid of `geometry`: the poses ahead of the base, the passed ones
    /// dropped from the plan. Fails when the nearest pose lies off the grid,
    /// after dropping the passed ones.
    pub fn prune(
        &mut self,
        pose: PlanarPose,
        geometry: &GridGeometry,
    ) -> Result<PrunedPlan, PlanOffGrid> {
        let poses = &self.remaining;
        let search_end = first_after_integrated_distance(
            poses,
            0,
            self.params.max_robot_pose_search_dist_m.get(),
        );
        let nearest = nearest_pose(&poses[..search_end], pose);
        let closest = if poses.len() > 1 && nearest == poses.len() - 1 {
            nearest - 1
        } else {
            nearest
        };
        let pruned_end =
            first_after_integrated_distance(poses, closest, self.params.prune_distance_m.get());
        let ahead: Vec<PlanarPose> = poses[closest..pruned_end]
            .iter()
            .take_while(|pose| geometry.cell_of([pose.x(), pose.y()]).is_some())
            .copied()
            .collect();
        self.remaining.drain(..closest);
        if ahead.is_empty() {
            return Err(PlanOffGrid);
        }
        Ok(PrunedPlan {
            poses: ahead,
            goal: self.goal,
        })
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
    use super::*;
    use control_core::positive_finite::PositiveFinite;

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
        let pruned = plan.prune(pose(1.1, 0.2), &geometry()).unwrap();
        assert_eq!(xs(&pruned), [1.0, 1.5, 2.0]);
        let again = plan.prune(pose(0.0, 0.0), &geometry()).unwrap();
        assert_eq!(xs(&again), [1.0, 1.5, 2.0]);
        assert_eq!(again.goal().x(), 4.5);
    }

    // The search reaches only the poses within its distance along the plan,
    // so a base beside the plan's end still takes a pose near the start.
    #[test]
    fn the_search_stops_at_its_distance_along_the_plan() {
        let mut plan = GlobalPlan::new(line(10), params(0.6, 0.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(4.5, 0.0), &geometry()).unwrap();
        assert_eq!(pruned.poses()[0].x(), 0.5);
    }

    // A base past the plan's end takes the last pose but one, so the pruned
    // plan holds the last two.
    #[test]
    fn the_last_pose_is_never_the_nearest_of_two_or_more() {
        let mut plan = GlobalPlan::new(line(4), params(10.0, 10.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(3.0, 0.0), &geometry()).unwrap();
        assert_eq!(xs(&pruned), [1.0, 1.5]);
    }

    // Poses past the grid's edge end the pruned plan, and a nearest pose
    // off the grid gives none, its passed poses dropped all the same.
    #[test]
    fn the_pruned_plan_ends_at_the_grid_edge() {
        let far: Vec<PlanarPose> = (0..30).map(|i| pose(i as f64 * 0.5, 0.0)).collect();
        let mut plan = GlobalPlan::new(far, params(100.0, 100.0), goal_checker()).unwrap();
        let pruned = plan.prune(pose(0.0, 0.0), &geometry()).unwrap();
        assert_eq!(pruned.poses().last().unwrap().x(), 8.5);
        assert_eq!(plan.prune(pose(12.0, 0.0), &geometry()), Err(PlanOffGrid));
        assert_eq!(plan.prune(pose(0.0, 0.0), &geometry()), Err(PlanOffGrid));
    }
}
