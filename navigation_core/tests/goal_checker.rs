//! Properties of the goal checker on drawn goals and poses: once the base
//! stands within the tolerance of the plan's goal, the position stays
//! reached, wherever the base goes with no buffer and within the tolerance
//! and the buffer with one, and only the heading must still converge; a
//! new plan or a base past the buffer must reach the position again; a plan
//! ahead longer than the path length tolerance keeps the goal unreached; and
//! each tolerance holds its boundary, as Nav2 compares it.

mod common;

use std::f64::consts::PI;

use common::finite::{non_negative, positive};
use common::poses::planar;
use control_core::planar_pose::PlanarPose;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::goal_checker::GoalCheckerParams;
use navigation_core::grid::GridGeometry;
use navigation_core::path_handler::{GlobalPlan, PathHandlerParams, PrunedPlan};

/// The share of a radius within which a drawn pose lies inside it, and past
/// which one lies outside it: the margin keeps the draws off the boundary
/// the checker compares at.
const INSIDE: f64 = 0.99;
const OUTSIDE: f64 = 1.01;
/// The longest plan ahead, in meters, at which the position can count as reached: Nav2's
/// default.
const PATH_LENGTH_TOLERANCE: f64 = 1.0;
/// A turn past a boundary, in radians, larger than the rounding of `normalize_angle`, which
/// adds and takes away pi.
const BEYOND_ROUNDING: f64 = 1e-12;

/// A grid of 20 m a side about the origin, which holds every drawn pose.
fn geometry() -> GridGeometry {
    GridGeometry::new(200, 200, positive(0.1), [-10.0, -10.0]).expect("a grid")
}

/// The plan of the goal alone, judged by a goal checker of `checker`, and the plan ahead of
/// a base at its goal, of length 0.
fn plan_at(goal: PlanarPose, checker: GoalCheckerParams) -> (GlobalPlan, PrunedPlan) {
    let params = PathHandlerParams {
        max_robot_pose_search_dist_m: non_negative(1.0),
        prune_distance_m: non_negative(1.0),
    };
    let mut plan = GlobalPlan::new(vec![goal], params, checker).expect("one pose");
    let pruned = plan
        .prune(goal, PlanarPose::IDENTITY, &geometry())
        .expect("the goal lies on the grid");
    (plan, pruned)
}

/// A pose `distance` from `center` in a drawn direction, at `yaw`.
fn pose_at(tc: &TestCase, center: PlanarPose, distance: f64, yaw: f64) -> PlanarPose {
    let direction = tc.draw(gs::floats().min_value(-PI).max_value(PI));
    planar(
        center.x() + distance * direction.cos(),
        center.y() + distance * direction.sin(),
        yaw,
    )
}

fn drawn_goal(tc: &TestCase) -> PlanarPose {
    planar(
        tc.draw(gs::floats().min_value(-2.0).max_value(2.0)),
        tc.draw(gs::floats().min_value(-2.0).max_value(2.0)),
        tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
    )
}

fn params(tolerance: f64, buffer: f64, yaw_tolerance: f64) -> GoalCheckerParams {
    GoalCheckerParams {
        xy_goal_tolerance_m: non_negative(tolerance),
        xy_goal_tolerance_buffer_m: non_negative(buffer),
        yaw_goal_tolerance_rad: non_negative(yaw_tolerance),
        path_length_tolerance_m: non_negative(PATH_LENGTH_TOLERANCE),
    }
}

// A base that reached the goal's position keeps it reached: with no buffer
// wherever it goes, with one while it stays within the tolerance and the
// buffer. The goal is then reached exactly when the heading lies within its
// tolerance.
#[hegel::test]
fn a_reached_position_stays_reached(tc: TestCase) {
    let tolerance = tc.draw(gs::floats().min_value(0.05).max_value(1.0));
    let buffer = if tc.draw(gs::booleans()) {
        0.0
    } else {
        tc.draw(gs::floats().min_value(0.01).max_value(0.5))
    };
    let yaw_tolerance = tc.draw(gs::floats().min_value(0.05).max_value(1.0));
    let goal = drawn_goal(&tc);
    let (mut plan, ahead) = plan_at(goal, params(tolerance, buffer, yaw_tolerance));
    let checker = plan.goal_checker();
    let reach = tc.draw(gs::floats().min_value(0.0).max_value(INSIDE * tolerance));
    let first = pose_at(&tc, goal, reach, goal.yaw());
    assert!(
        checker.is_goal_xy_reached(first, &ahead),
        "{first:?} reaches {goal:?}"
    );
    let roam = if buffer == 0.0 {
        5.0
    } else {
        INSIDE * (tolerance + buffer)
    };
    for _ in 0..tc.draw(gs::integers::<usize>().min_value(1).max_value(10)) {
        let distance = tc.draw(gs::floats().min_value(0.0).max_value(roam));
        let within = tc.draw(gs::booleans());
        let offset = if within {
            tc.draw(
                gs::floats()
                    .min_value(0.0)
                    .max_value(INSIDE * yaw_tolerance),
            )
        } else {
            tc.draw(
                gs::floats()
                    .min_value(OUTSIDE * yaw_tolerance)
                    .max_value(3.0),
            )
        };
        let sign = if tc.draw(gs::booleans()) { 1.0 } else { -1.0 };
        let pose = pose_at(&tc, goal, distance, goal.yaw() + sign * offset);
        assert!(
            checker.is_goal_xy_reached(pose, &ahead),
            "{pose:?} keeps the reached position of {goal:?}"
        );
        assert_eq!(
            checker.is_goal_reached(pose, &ahead),
            within,
            "{pose:?} with the heading {offset} rad off the goal's"
        );
    }
}

// Before the base reaches the position, and on a new plan of the same goal after the old
// one's checker reached it, a base beyond the tolerance has not reached it: each plan's goal
// checker judges the position afresh, as Nav2's controller server resets its checker as it
// takes a new plan.
#[hegel::test]
fn a_position_beyond_the_tolerance_must_be_reached_first(tc: TestCase) {
    let tolerance = tc.draw(gs::floats().min_value(0.05).max_value(1.0));
    let goal = drawn_goal(&tc);
    let checker = params(tolerance, 0.0, 0.5);
    let (mut plan, ahead) = plan_at(goal, checker);
    let far = tc.draw(gs::floats().min_value(OUTSIDE * tolerance).max_value(5.0));
    let away = pose_at(&tc, goal, far, goal.yaw());
    assert!(!plan.goal_checker().is_goal_xy_reached(away, &ahead));
    assert!(plan.goal_checker().is_goal_xy_reached(goal, &ahead));
    assert!(plan.goal_checker().is_goal_xy_reached(away, &ahead));
    let (mut replanned, ahead) = plan_at(goal, checker);
    assert!(!replanned.goal_checker().is_goal_xy_reached(away, &ahead));
}

// With a buffer, a base that strays past the tolerance and the buffer
// loses the reached position, and must come back within the tolerance.
#[test]
fn a_base_past_the_buffer_must_reach_the_position_again() {
    let goal = planar(1.0, 1.0, 0.0);
    let (mut plan, ahead) = plan_at(goal, params(0.25, 0.25, 0.5));
    let checker = plan.goal_checker();
    let at = |dx: f64| planar(goal.x() + dx, goal.y(), goal.yaw());
    assert!(checker.is_goal_xy_reached(at(0.2), &ahead));
    assert!(checker.is_goal_xy_reached(at(0.45), &ahead));
    assert!(!checker.is_goal_xy_reached(at(0.55), &ahead));
    assert!(!checker.is_goal_xy_reached(at(0.45), &ahead));
    assert!(checker.is_goal_xy_reached(at(0.2), &ahead));
}

// Each tolerance holds its boundary, as Nav2 compares it: a base exactly the
// tolerance from the goal reaches its position, one exactly the tolerance and
// the buffer from it keeps the reached position, and a heading exactly the yaw
// tolerance off the goal's reaches the goal, and one just past it does not.
// Each distance and turn at a boundary is exact in double precision.
#[test]
fn each_tolerance_holds_its_boundary() {
    let goal = planar(1.0, 1.0, 0.5);
    let (mut plan, ahead) = plan_at(goal, params(0.25, 0.25, 0.25));
    let checker = plan.goal_checker();
    let at = |dx: f64, yaw: f64| planar(goal.x() + dx, goal.y(), yaw);
    assert!(checker.is_goal_xy_reached(at(0.25, goal.yaw()), &ahead));
    assert!(checker.is_goal_xy_reached(at(0.5, goal.yaw()), &ahead));
    assert!(checker.is_goal_reached(at(0.0, 0.25), &ahead));
    assert!(!checker.is_goal_reached(at(0.0, 0.25 - BEYOND_ROUNDING), &ahead));
}

/// The plan from the origin along +X in steps of 0.5 m, `steps` of them, judged by a goal
/// checker of a 5 m tolerance, and the plan pruned for a base at the origin: `steps * 0.5` m
/// long, exactly.
fn plan_of_steps(steps: u32) -> (GlobalPlan, PrunedPlan) {
    let start = planar(0.0, 0.0, 0.0);
    let path: Vec<PlanarPose> = (0..=steps)
        .map(|i| planar(f64::from(i) * 0.5, start.y(), start.yaw()))
        .collect();
    let params_handler = PathHandlerParams {
        max_robot_pose_search_dist_m: non_negative(10.0),
        prune_distance_m: non_negative(10.0),
    };
    let mut plan = GlobalPlan::new(path, params_handler, params(5.0, 0.0, 0.5)).expect("a path");
    let pruned = plan
        .prune(start, PlanarPose::IDENTITY, &geometry())
        .expect("on the grid");
    (plan, pruned)
}

// A plan ahead longer than the path length tolerance keeps the goal unreached, the base
// within the position tolerance of it; a plan exactly the tolerance long lets the position
// decide.
#[test]
fn a_plan_ahead_longer_than_the_tolerance_keeps_the_goal_unreached() {
    let (mut plan, longer) = plan_of_steps(3);
    let goal = longer.goal();
    assert!(!plan.goal_checker().is_goal_xy_reached(goal, &longer));
    assert!(!plan.goal_checker().is_goal_reached(goal, &longer));
    let (mut plan, at_the_tolerance) = plan_of_steps(2);
    assert_eq!(
        at_the_tolerance.poses().len(),
        3,
        "the whole plan, {PATH_LENGTH_TOLERANCE} m long"
    );
    assert!(
        plan.goal_checker()
            .is_goal_xy_reached(at_the_tolerance.goal(), &at_the_tolerance)
    );
}
