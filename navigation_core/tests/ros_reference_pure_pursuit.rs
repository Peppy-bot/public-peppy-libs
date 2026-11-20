//! The controller, the plan pruning and the checkers against Nav2's regulated pure pursuit
//! and controller server (nav2_regulated_pure_pursuit_controller and nav2_controller
//! 1.5.2): `GlobalPlan::prune` ports the feasible path handler, `GoalChecker` and
//! `ProgressChecker` the simple goal and progress checkers, and `PurePursuit::compute`
//! `computeVelocityCommands`, so for the same plan, poses, speeds and cost grid, each step
//! prunes the plan to the same poses, the checkers give the same verdicts, and the forward
//! speed and turn rate lie within 1e-9 of Nav2's, or the command fails as Nav2's does. The
//! sides differ only in the rounding of the base-frame transforms and of their libm's
//! sines and cosines. The evidence: every pure pursuit scenario of the ROS reference
//! harness, run step by step as the controller server runs it.

mod common;

use common::finite::{non_negative, positive};
use common::footprints::padded;
use common::poses::planar;
use control_core::planar_pose::PlanarPose;
use control_core::twist::PlanarTwist;
use navigation_core::PaddedFootprint;
use navigation_core::controller::{
    ControlError, Lookahead, PurePursuit, PurePursuitParams, VelocityScaledLookahead,
};
use navigation_core::goal_checker::GoalCheckerParams;
use navigation_core::grid::{CostGrid, GridGeometry};
use navigation_core::inflation::{Inflation, InflationParams};
use navigation_core::path_handler::{GlobalPlan, PathHandlerParams, PlanOffGrid};
use navigation_core::progress_checker::{ProgressChecker, ProgressCheckerParams};
use ros_reference::pure_pursuit::{Cases, ControllerParameters, Outcome};

/// How far a forward speed or a turn rate may lie from Nav2's.
const BOUND: f64 = 1e-9;

fn pose([x, y, yaw]: [f64; 3]) -> PlanarPose {
    planar(x, y, yaw)
}

/// The port's parameters for a scenario's controller, Nav2's switches the port holds fixed
/// checked first.
fn controller_params(cases: &Cases) -> PurePursuitParams {
    let c: &ControllerParameters = &cases.controller;
    assert!(
        c.use_rotate_to_heading
            && c.use_regulated_linear_velocity_scaling
            && c.use_cost_regulated_linear_velocity_scaling
            && c.use_collision_detection
            && !c.allow_reversing
            && !c.use_fixed_curvature_lookahead
            && !c.interpolate_curvature_after_goal
            && !c.use_dynamic_window
            && !c.allow_obstacle_checking_beyond_goal
            && !c.use_cancel_deceleration
            && cases.track_unknown_space,
        "{}: the scenario runs the controller as the port does: rotation to heading, both \
         regulations and collision detection on; reversing, the fixed curvature lookahead, \
         the dynamic window and the cancel deceleration off; on a grid that tracks unknown \
         cells, as every grid compose makes does",
        cases.name
    );
    let lookahead = if c.use_velocity_scaled_lookahead_dist {
        Lookahead::VelocityScaled(
            VelocityScaledLookahead::parse(
                positive(c.lookahead_time),
                non_negative(c.min_lookahead_dist),
                non_negative(c.max_lookahead_dist),
            )
            .expect("the scenario's lookahead bounds"),
        )
    } else {
        Lookahead::Fixed(positive(c.lookahead_dist))
    };
    PurePursuitParams {
        max_linear_vel_m_s: positive(c.max_linear_vel),
        lookahead,
        rotate_to_heading_angular_vel_rad_s: positive(c.rotate_to_heading_angular_vel),
        rotate_to_heading_min_angle_rad: non_negative(c.rotate_to_heading_min_angle),
        max_angular_accel_rad_s2: positive(c.max_angular_accel),
        min_approach_linear_velocity_m_s: positive(c.min_approach_linear_velocity),
        approach_velocity_scaling_dist_m: non_negative(c.approach_velocity_scaling_dist),
        regulated_linear_scaling_min_radius_m: non_negative(c.regulated_linear_scaling_min_radius),
        regulated_linear_scaling_min_speed_m_s: non_negative(c.regulated_linear_scaling_min_speed),
        cost_scaling_dist_m: positive(c.cost_scaling_dist),
        cost_scaling_gain: non_negative(c.cost_scaling_gain),
        max_allowed_time_to_collision_up_to_carrot_s: non_negative(
            c.max_allowed_time_to_collision_up_to_carrot,
        ),
        min_distance_to_obstacle_m: (c.min_distance_to_obstacle > 0.0)
            .then(|| positive(c.min_distance_to_obstacle)),
        controller_frequency_hz: positive(cases.controller_frequency),
    }
}

/// The inflation of a base of `footprint` at the scenario's scaling factor, from which the
/// controller reads the footprint, for its arc check, and the footprint's inscribed radius
/// and the scaling factor, for its cost regulation. The controller reads no inflation
/// radius, and the scenario's grid comes inflated, so the radius is the least an inflation
/// takes, the inscribed radius.
fn inflation(cases: &Cases, footprint: &PaddedFootprint) -> Inflation {
    Inflation::parse(
        InflationParams {
            inflation_radius_m: non_negative(footprint.inscribed_radius_m()),
            cost_scaling_factor: non_negative(cases.controller.inflation_cost_scaling_factor),
        },
        footprint,
    )
    .expect("an inflation of the inscribed radius")
}

fn goal_checker(cases: &Cases) -> GoalCheckerParams {
    let g = &cases.goal_checker;
    assert!(
        g.stateful && !g.symmetric_yaw_tolerance,
        "{}: the goal checker is stateful, its yaw tolerance one-sided",
        cases.name
    );
    GoalCheckerParams {
        xy_goal_tolerance_m: non_negative(g.xy_goal_tolerance),
        xy_goal_tolerance_buffer_m: non_negative(g.xy_goal_tolerance_buffer),
        yaw_goal_tolerance_rad: non_negative(g.yaw_goal_tolerance),
        path_length_tolerance_m: non_negative(g.path_length_tolerance),
    }
}

fn global_plan(cases: &Cases) -> GlobalPlan {
    let p = &cases.path_handler;
    assert!(
        !p.reject_unit_path && !p.enforce_path_inversion && !p.enforce_path_rotation,
        "{}: the path handler takes a plan of one pose and enforces no inversion or rotation",
        cases.name
    );
    let params = PathHandlerParams {
        max_robot_pose_search_dist_m: non_negative(p.max_robot_pose_search_dist),
        prune_distance_m: non_negative(p.prune_distance),
    };
    GlobalPlan::new(
        cases.plan.iter().copied().map(pose).collect(),
        params,
        goal_checker(cases),
    )
    .expect("the scenario's plan")
}

fn grid(cases: &Cases) -> CostGrid {
    let g = cases.grid;
    let geometry = GridGeometry::new(g.size_x, g.size_y, positive(g.resolution), g.origin)
        .expect("the scenario's grid");
    CostGrid::from_costs(geometry, cases.costs.clone()).expect("a cost a cell")
}

// Each step prunes the plan to Nav2's poses, the goal and progress checkers give Nav2's
// verdicts, and the command lies within the bound of Nav2's or fails as Nav2's does.
#[test]
fn each_step_is_the_controller_servers() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a pure pursuit scenario"
    );
    for cases in &scenarios {
        let footprint = padded(&cases.footprint, 0.0);
        assert_eq!(
            (
                footprint.inscribed_radius_m().to_bits(),
                footprint.circumscribed_radius_m().to_bits()
            ),
            (
                cases.inscribed_radius.to_bits(),
                cases.circumscribed_radius.to_bits()
            ),
            "{}: the footprint's radii are Nav2's",
            cases.name
        );
        let grid = grid(cases);
        let geometry = grid.geometry();
        let mut plan = global_plan(cases);
        let mut progress_checker = ProgressChecker::new(ProgressCheckerParams {
            required_movement_m: non_negative(cases.progress_checker.required_movement_radius),
            movement_time_allowance_s: non_negative(cases.progress_checker.movement_time_allowance),
        });
        let controller = PurePursuit::new(controller_params(cases), &inflation(cases, &footprint));
        for (index, step) in cases.steps.iter().enumerate() {
            let what = format!("{} step {index}", cases.name);
            let base = pose(step.pose);
            let pruned = match plan.prune(base, &geometry) {
                Ok(pruned) => pruned,
                Err(PlanOffGrid) => {
                    assert_eq!(step.outcome, Outcome::PlanOffGrid, "{what}: the prune");
                    continue;
                }
            };
            let first = cases
                .plan
                .iter()
                .position(|p| p[..2] == [pruned.poses()[0].x(), pruned.poses()[0].y()])
                .expect("the pruned plan's first pose is a pose of the plan");
            assert_eq!(
                Some((first, pruned.poses().len())),
                step.pruned,
                "{what}: the pruned plan's first pose and count"
            );
            let reached = plan.goal_checker().is_goal_reached(base, &pruned);
            assert_eq!(Some(reached), step.goal_reached, "{what}: the goal checker");
            if reached {
                assert_eq!(step.outcome, Outcome::GoalReached, "{what}");
                continue;
            }
            let progress = progress_checker.check(base, step.at_ns).is_ok();
            assert_eq!(
                Some(progress),
                step.progress,
                "{what}: the progress checker"
            );
            let [vx, vy, wz] = step.speed;
            let speed = PlanarTwist::new(vx, vy, wz).expect("finite");
            let ours = controller.compute(base, speed, plan.goal_checker(), &pruned, &grid);
            match (ours, step.outcome, step.twist) {
                (Ok(command), Outcome::Command, Some(theirs)) => {
                    let worst = command
                        .axes()
                        .iter()
                        .zip(theirs)
                        .map(|(a, b)| (a - b).abs())
                        .fold(0.0, f64::max);
                    assert!(
                        worst <= BOUND,
                        "{what}: ours {command:?}, Nav2's {theirs:?}, {worst:e} apart"
                    );
                }
                (Err(ControlError::NoValidControl), Outcome::NoValidControl, None)
                | (Err(ControlError::BaseOffGrid), Outcome::BaseOffGrid, None) => {}
                (ours, outcome, theirs) => {
                    panic!("{what}: ours {ours:?}, Nav2's {outcome:?} {theirs:?}")
                }
            }
        }
    }
}
