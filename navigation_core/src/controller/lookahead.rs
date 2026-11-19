//! The carrot of a plan in the base's frame, as nav2_util 1.5.2's
//! `getLookAheadPoint` finds it with `interpolate_after_goal` false, and its
//! `linearInterpolation` (src/controller_utils.cpp).

use control_core::planar_pose::PlanarPose;

use crate::pose::distance;

/// The point `lookahead_dist` along `plan` from its first pose: on the step
/// that reaches that distance, or the last pose when the plan is shorter.
pub(super) fn lookahead_point(lookahead_dist: f64, plan: &[PlanarPose]) -> [f64; 2] {
    let mut path_dist = 0.0;
    for step in plan.windows(2) {
        let (previous, current) = (step[0], step[1]);
        let d = distance(previous, current);
        if path_dist + d >= lookahead_dist {
            return linear_interpolation(previous, current, lookahead_dist - path_dist);
        }
        path_dist += d;
    }
    let last = plan.last().expect("a pruned plan holds a pose");
    [last.x(), last.y()]
}

/// The point `target_dist` from `from` towards `to`.
fn linear_interpolation(from: PlanarPose, to: PlanarPose, target_dist: f64) -> [f64; 2] {
    let dx = to.x() - from.x();
    let dy = to.y() - from.y();
    let target_ratio = target_dist / dx.hypot(dy);
    [from.x() + target_ratio * dx, from.y() + target_ratio * dy]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(x: f64, y: f64) -> PlanarPose {
        PlanarPose::new(x, y, 0.0).unwrap()
    }

    // The carrot lies on the step that reaches the distance, measured along
    // the plan from its first pose.
    #[test]
    fn the_carrot_lies_at_the_distance_along_the_plan() {
        let plan = [pose(0.0, 0.0), pose(1.0, 0.0), pose(1.0, 1.0)];
        assert_eq!(lookahead_point(0.25, &plan), [0.25, 0.0]);
        assert_eq!(lookahead_point(1.5, &plan), [1.0, 0.5]);
        assert_eq!(lookahead_point(1.0, &plan), [1.0, 0.0]);
    }

    // A plan shorter than the distance gives its last pose, and a plan of
    // one pose gives that pose.
    #[test]
    fn a_short_plan_gives_its_last_pose() {
        let plan = [pose(0.0, 0.0), pose(0.5, 0.0)];
        assert_eq!(lookahead_point(2.0, &plan), [0.5, 0.0]);
        assert_eq!(lookahead_point(2.0, &plan[1..]), [0.5, 0.0]);
    }
}
