//! The regulated pure pursuit on drawn and named plans: its forward speed
//! falls as the plan's curvature grows, near obstacles, as the inflation's
//! scaling factor and inscribed radius read the cost, and on the approach
//! to the goal, and a regulation below 0 holds it at its minimum; it turns the
//! base in place towards a heading more than `rotate_to_heading_min_angle_rad`
//! off the base's, and towards a goal's heading of -pi the way tf2 reads it; a
//! contact along the arc ahead, or a footprint vertex off the grid, fails the
//! command, the arc checked farther with a minimum distance to obstacles, up
//! to the longest lookahead; every command is a forward speed and a turn rate;
//! and inverted lookahead bounds are refused.

mod common;

use camera_core::pose::PlanarPose;
use common::finite::{non_negative, positive};
use common::footprints::padded;
use common::poses::planar;
use control_core::twist::Twist;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::PaddedFootprint;
use navigation_core::controller::{
    ControlError, InvertedLookahead, Lookahead, PurePursuit, PurePursuitParams,
    VelocityScaledLookahead,
};
use navigation_core::goal_checker::GoalCheckerParams;
use navigation_core::grid::{CostGrid, FREE, GridGeometry, LETHAL};
use navigation_core::inflation::{Inflation, InflationParams};
use navigation_core::path_handler::{GlobalPlan, PathHandlerParams, PrunedPlan};

/// The slack of a comparison of two computed speeds.
const TOLERANCE: f64 = 1e-9;
/// The forward speed of a straight arc far from obstacles, in m/s.
const MAX_SPEED: f64 = 0.5;
/// The arc radius below which the speed falls with the curvature, in m.
const MIN_RADIUS: f64 = 0.9;

fn params() -> PurePursuitParams {
    PurePursuitParams {
        max_linear_vel_m_s: positive(MAX_SPEED),
        lookahead: Lookahead::Fixed(positive(0.3)),
        rotate_to_heading_angular_vel_rad_s: positive(1.8),
        rotate_to_heading_min_angle_rad: non_negative(0.785),
        max_angular_accel_rad_s2: positive(3.2),
        min_approach_linear_velocity_m_s: positive(0.05),
        approach_velocity_scaling_dist_m: non_negative(0.6),
        regulated_linear_scaling_min_radius_m: non_negative(MIN_RADIUS),
        regulated_linear_scaling_min_speed_m_s: non_negative(0.05),
        cost_scaling_dist_m: positive(0.6),
        cost_scaling_gain: non_negative(1.0),
        max_allowed_time_to_collision_up_to_carrot_s: non_negative(1.0),
        min_distance_to_obstacle_m: None,
        controller_frequency_hz: positive(20.0),
    }
}

/// The inflation of the grids the controller drives on: Nav2's defaults.
fn inflation() -> Inflation {
    inflation_scaled_by(3.0)
}

/// The inflation of Nav2's default radius whose cost falls by `cost_scaling_factor`.
fn inflation_scaled_by(cost_scaling_factor: f64) -> Inflation {
    Inflation::parse(
        InflationParams {
            inflation_radius_m: non_negative(0.55),
            cost_scaling_factor: non_negative(cost_scaling_factor),
        },
        &footprint(),
    )
    .expect("Nav2's radius covers the footprint's inscribed radius")
}

fn footprint() -> PaddedFootprint {
    padded(
        &[[-0.15, -0.1], [0.15, -0.1], [0.15, 0.1], [-0.15, 0.1]],
        0.0,
    )
}

fn goal_checker() -> GoalCheckerParams {
    GoalCheckerParams {
        xy_goal_tolerance_m: non_negative(0.25),
        xy_goal_tolerance_buffer_m: non_negative(0.0),
        yaw_goal_tolerance_rad: non_negative(0.25),
        path_length_tolerance_m: non_negative(1.0),
    }
}

/// A free grid of 12 m a side about the origin, in cells of 5 cm.
fn free_grid() -> CostGrid {
    let geometry = GridGeometry::new(240, 240, positive(0.05), [-6.0, -6.0]).expect("a grid");
    CostGrid::filled(geometry, FREE)
}

/// `grid` with `cost` over the cells whose centers lie in the rectangle
/// from `low` to `high`.
fn with_block(grid: &CostGrid, low: [f64; 2], high: [f64; 2], cost: u8) -> CostGrid {
    let geometry = grid.geometry();
    let costs = geometry
        .cells()
        .map(|cell| {
            let [x, y] = geometry.center_of(cell);
            let inside = (low[0]..=high[0]).contains(&x) && (low[1]..=high[1]).contains(&y);
            if inside { cost } else { grid.cost(cell) }
        })
        .collect();
    CostGrid::from_costs(geometry, costs).expect("the same grid")
}

/// The plan of `poses`, with its goal checker, and the plan pruned for a base at `pose`.
fn pruned(poses: Vec<PlanarPose>, pose: PlanarPose, grid: &CostGrid) -> (GlobalPlan, PrunedPlan) {
    let handler = PathHandlerParams {
        max_robot_pose_search_dist_m: non_negative(3.0),
        prune_distance_m: non_negative(2.0),
    };
    let mut plan = GlobalPlan::new(poses, handler, goal_checker()).expect("a path");
    let ahead = plan.prune(pose, &grid.geometry()).expect("on the grid");
    (plan, ahead)
}

/// A straight plan from the origin along +X, `length` long, in steps of
/// 5 cm.
fn straight(length: f64) -> Vec<PlanarPose> {
    let steps = (length / 0.05).round() as u32;
    (0..=steps)
        .map(|i| planar(f64::from(i) * 0.05, 0.0, 0.0))
        .collect()
}

/// An arc of `radius` from the origin, leaving along +X and turning left,
/// 2.5 m long, in steps of 5 cm.
fn arc(radius: f64) -> Vec<PlanarPose> {
    (0..=50)
        .map(|i| {
            let angle = f64::from(i) * 0.05 / radius;
            planar(radius * angle.sin(), radius * (1.0 - angle.cos()), angle)
        })
        .collect()
}

fn origin() -> PlanarPose {
    planar(0.0, 0.0, 0.0)
}

fn moving(linear: f64, angular: f64) -> Twist {
    Twist::new(linear, 0.0, angular).expect("finite")
}

/// The command of a fresh controller for a base at `pose` moving at
/// `speed`, with `poses` its plan, on `grid`.
fn command(
    poses: Vec<PlanarPose>,
    pose: PlanarPose,
    speed: Twist,
    grid: &CostGrid,
) -> Result<Twist, ControlError> {
    command_of(params(), poses, pose, speed, grid)
}

/// The command of a fresh controller of `params` for a base at `pose`
/// moving at `speed`, with `poses` its plan, on `grid`.
fn command_of(
    params: PurePursuitParams,
    poses: Vec<PlanarPose>,
    pose: PlanarPose,
    speed: Twist,
    grid: &CostGrid,
) -> Result<Twist, ControlError> {
    let (mut plan, ahead) = pruned(poses, pose, grid);
    PurePursuit::new(params, &inflation()).compute(pose, speed, plan.goal_checker(), &ahead, grid)
}

// On arcs whose radius falls, the forward speed never grows, and below the
// minimum radius it falls with the radius, and never below 0, so the base
// never drives backward; a forward speed and a turn rate make every command.
#[hegel::test]
fn the_controller_slows_as_curvature_grows_and_never_below_0(tc: TestCase) {
    let grid = free_grid();
    let wide = tc.draw(gs::floats().min_value(0.4).max_value(5.0));
    let tight = tc.draw(gs::floats().min_value(0.4).max_value(wide));
    let speed = moving(0.3, 0.0);
    let on = |radius: f64| command(arc(radius), origin(), speed, &grid).expect("a free arc");
    let (wide_command, tight_command) = (on(wide), on(tight));
    for command in [wide_command, tight_command] {
        assert_eq!(command.linear_y(), 0.0, "{command:?}");
        assert!(
            command.linear_x() >= 0.0,
            "a forward speed below 0: {command:?}"
        );
        assert!(
            command.angular_z() > 0.0,
            "a left arc turns left: {command:?}"
        );
    }
    assert!(
        tight_command.linear_x() <= wide_command.linear_x() + TOLERANCE,
        "radius {tight} m gives {tight_command:?}, radius {wide} m {wide_command:?}"
    );
    if tight < 0.95 * wide && tight < 0.95 * MIN_RADIUS {
        assert!(
            tight_command.linear_x() < wide_command.linear_x(),
            "radius {tight} m gives {tight_command:?}, radius {wide} m {wide_command:?}"
        );
    }
}

// A lethal cell under the base's center, its outline clear, reads back through a scaling
// factor of 0.01 as an obstacle less than 0 m away, which takes the cost regulation below
// 0: the forward speed holds at `regulated_linear_scaling_min_speed_m_s`, so a regulation
// never drives the base backward.
#[test]
fn a_regulation_below_0_holds_the_speed_at_the_minimum() {
    let pole = with_block(&free_grid(), [-0.03, -0.03], [0.03, 0.03], LETHAL);
    let weak = inflation_scaled_by(0.01);
    let (mut plan, ahead) = pruned(straight(3.0), origin(), &pole);
    let command = PurePursuit::new(params(), &weak)
        .compute(
            origin(),
            moving(0.3, 0.0),
            plan.goal_checker(),
            &ahead,
            &pole,
        )
        .unwrap();
    assert_eq!(command.linear_x(), 0.05, "{command:?}");
}

// A straight plan on a free floor runs at the full speed, straight ahead.
#[test]
fn a_straight_free_plan_runs_at_full_speed() {
    let command = command(straight(3.0), origin(), moving(0.3, 0.0), &free_grid()).unwrap();
    assert!(
        (command.linear_x() - MAX_SPEED).abs() < TOLERANCE,
        "{command:?}"
    );
    assert_eq!(command.angular_z(), 0.0);
}

// The base on an inflated cell near an obstacle drives slower than on a
// free one.
#[test]
fn the_base_slows_near_obstacles() {
    let near = with_block(&free_grid(), [-0.1, -0.1], [0.1, 0.1], 200);
    let command = command(straight(3.0), origin(), moving(0.3, 0.0), &near).unwrap();
    assert!(
        command.linear_x() < MAX_SPEED - 0.05,
        "the base on a cell of cost 200: {command:?}"
    );
}

// A grid inflated with a scaling factor of 0 turns the cost regulation off, as Nav2 turns
// it off for a factor at or below 0: the base on the same inflated cell runs at the full
// speed.
#[test]
fn a_scaling_factor_of_0_leaves_the_speed_near_obstacles() {
    let near = with_block(&free_grid(), [-0.1, -0.1], [0.1, 0.1], 200);
    let unscaled = inflation_scaled_by(0.0);
    let (mut plan, ahead) = pruned(straight(3.0), origin(), &near);
    let command = PurePursuit::new(params(), &unscaled)
        .compute(
            origin(),
            moving(0.3, 0.0),
            plan.goal_checker(),
            &ahead,
            &near,
        )
        .unwrap();
    assert!(
        (command.linear_x() - MAX_SPEED).abs() < TOLERANCE,
        "{command:?}"
    );
}

// The cost under the base reads back as its distance to an obstacle through the inscribed
// radius of the inflation's footprint, 0.1 m: on a cell of cost 200, at the scaling factor
// of 3, that distance is (3 * 0.1 - ln 200 + ln 253) / 3 m, ln 253 in single precision as
// Nav2's `log(253.0f)`, and the speed falls to that distance's share of the 0.6 m
// `cost_scaling_dist_m` (regulation_functions.hpp:72-81).
#[test]
fn the_cost_regulation_reads_the_inflations_inscribed_radius() {
    let near = with_block(&free_grid(), [-0.1, -0.1], [0.1, 0.1], 200);
    let command = command(straight(3.0), origin(), moving(0.3, 0.0), &near).unwrap();
    let log_253_single = f64::from(253f64.ln() as f32);
    let distance_m = (3.0 * 0.1 - 200f64.ln() + log_253_single) / 3.0;
    let expected = MAX_SPEED * distance_m / 0.6;
    assert!(
        (command.linear_x() - expected).abs() < TOLERANCE,
        "{command:?}, the regulated speed {expected}"
    );
}

// Within the approach distance of the plan's end the speed falls in
// proportion to the straight distance to it.
#[test]
fn the_base_slows_on_the_approach_to_the_goal() {
    let command = command(straight(0.4), origin(), moving(0.3, 0.0), &free_grid()).unwrap();
    let expected = MAX_SPEED * 0.4 / 0.6;
    assert!(
        (command.linear_x() - expected).abs() < TOLERANCE,
        "{command:?}"
    );
}

// A plan behind the base turns it in place towards the plan, at no more
// than one control period of angular acceleration from its turn rate.
#[test]
fn the_base_turns_in_place_towards_a_path_behind_it() {
    let behind = planar(origin().x(), origin().y(), 2.5);
    let command = command(straight(3.0), behind, moving(0.0, 0.0), &free_grid()).unwrap();
    assert_eq!(command.linear_x(), 0.0);
    assert!(
        (command.angular_z() + 3.2 / 20.0).abs() < TOLERANCE,
        "a right turn of one period of acceleration: {command:?}"
    );
}

// Once the goal's position is reached, the base turns in place towards the
// plan's last heading.
#[test]
fn a_reached_position_turns_the_base_to_the_goal_heading() {
    let grid = free_grid();
    let mut poses = straight(0.2);
    let end = poses.last_mut().unwrap();
    *end = planar(end.x(), end.y(), 1.0);
    let at_goal = planar(0.15, origin().y(), origin().yaw());
    let command = command(poses, at_goal, moving(0.0, 1.0), &grid).unwrap();
    assert_eq!(command.linear_x(), 0.0);
    assert!(
        command.angular_z() > 0.0,
        "a left turn to 1 rad: {command:?}"
    );
}

// A goal's heading of -pi, the base at 0, turns the base clockwise, as tf2
// reads that heading in the base's frame as -pi.
#[test]
fn a_goal_heading_of_minus_pi_turns_the_base_as_tf2_reads_it() {
    let mut poses = straight(0.2);
    let end = poses.last_mut().unwrap();
    *end = planar(end.x(), end.y(), -std::f64::consts::PI);
    let at_goal = planar(0.15, origin().y(), origin().yaw());
    let command = command(poses, at_goal, moving(0.0, 0.0), &free_grid()).unwrap();
    assert_eq!(command.linear_x(), 0.0);
    assert!(
        command.angular_z() < 0.0,
        "a right turn to -pi: {command:?}"
    );
}

/// A carrot ahead and to the left of the base at exactly
/// `rotate_to_heading_min_angle_rad`, 0.785 rad, Nav2's default: the atan2 of
/// this lateral offset at 1 m ahead rounds to 0.785.
const LATERAL_AT_THE_MIN_ANGLE: f64 = 0.9992039901050427;

// A carrot exactly `rotate_to_heading_min_angle_rad` off the base's heading
// drives the arc, and one past it turns the base in place, as Nav2 turns for
// an angle above the minimum.
#[test]
fn the_base_turns_in_place_only_past_the_minimum_angle() {
    let grid = free_grid();
    let to = |y: f64| {
        let goal = planar(1.0, y, 0.0);
        command(vec![goal], origin(), moving(0.0, 0.0), &grid).unwrap()
    };
    assert_eq!(LATERAL_AT_THE_MIN_ANGLE.atan2(1.0), 0.785);
    let at = to(LATERAL_AT_THE_MIN_ANGLE);
    assert!(at.linear_x() > 0.0, "an arc at the minimum angle: {at:?}");
    let past = to(1.0);
    assert_eq!(past.linear_x(), 0.0, "a turn in place past it: {past:?}");
    assert!(past.angular_z() > 0.0, "{past:?}");
}

// A lethal block across the plan within the carrot's reach fails the
// command, as does one under the base's footprint; one beyond the carrot
// does not.
#[test]
fn an_imminent_collision_is_no_valid_control() {
    let ahead = with_block(&free_grid(), [0.3, -0.5], [0.5, 0.5], LETHAL);
    assert_eq!(
        command(straight(3.0), origin(), moving(0.5, 0.0), &ahead),
        Err(ControlError::NoValidControl)
    );
    let under = with_block(&free_grid(), [-0.2, -0.15], [0.2, 0.15], LETHAL);
    assert_eq!(
        command(straight(3.0), origin(), moving(0.5, 0.0), &under),
        Err(ControlError::NoValidControl)
    );
    let beyond = with_block(&free_grid(), [1.5, -0.5], [1.7, 0.5], LETHAL);
    assert!(command(straight(3.0), origin(), moving(0.5, 0.0), &beyond).is_ok());
}

// A base off the grid that is not turning in place has no cost to regulate
// by.
#[test]
fn a_base_off_the_grid_has_no_command() {
    let grid = free_grid();
    let (mut plan, ahead) = pruned(straight(3.0), origin(), &grid);
    let off = planar(-7.0, origin().y(), origin().yaw());
    let result = PurePursuit::new(params(), &inflation()).compute(
        off,
        moving(0.0, 0.0),
        plan.goal_checker(),
        &ahead,
        &grid,
    );
    assert_eq!(result, Err(ControlError::BaseOffGrid));
}

// A base so far from its plan, 1.3e308 m back on each axis and turned an eighth of a turn,
// that a pose of the plan seen from it lies past the largest double has no command, the
// refusal saying to give both in one frame.
#[test]
fn a_plan_too_far_for_the_base_frame_has_no_command() {
    let grid = free_grid();
    let (mut plan, ahead) = pruned(straight(3.0), origin(), &grid);
    let far = planar(-1.3e308, -1.3e308, std::f64::consts::FRAC_PI_4);
    let refused = PurePursuit::new(params(), &inflation())
        .compute(far, moving(0.0, 0.0), plan.goal_checker(), &ahead, &grid)
        .unwrap_err();
    assert_eq!(refused, ControlError::PlanNotInBaseFrame);
    assert!(
        refused
            .to_string()
            .contains("give the base's pose and the plan in one frame"),
        "{refused}"
    );
}

// A footprint whose front leaves the grid, its center on it, fails the command on a grid of
// no lethal cell, as Nav2's footprint cost reads a vertex off the grid as lethal, and the
// refusal names both causes.
#[test]
fn a_footprint_leaving_the_grid_is_no_valid_control() {
    let near_the_edge = planar(5.9, origin().y(), origin().yaw());
    let ahead = planar(5.95, origin().y(), origin().yaw());
    let refused = command(vec![ahead], near_the_edge, moving(0.0, 0.0), &free_grid()).unwrap_err();
    assert_eq!(refused, ControlError::NoValidControl);
    assert!(
        refused
            .to_string()
            .contains("meets a lethal cell, or a vertex of it lies off the cost grid"),
        "{refused}"
    );
}

/// A lethal block across the plan from `near` m to 1.5 m ahead of the base's center.
fn wall_from(near: f64) -> CostGrid {
    with_block(&free_grid(), [near, -0.5], [1.5, 0.5], LETHAL)
}

/// The command of a controller of `params` for a base at the origin moving
/// at 0.5 m/s along `poses`, on `grid`.
fn driving(
    params: PurePursuitParams,
    poses: Vec<PlanarPose>,
    grid: &CostGrid,
) -> Result<Twist, ControlError> {
    command_of(params, poses, origin(), moving(0.5, 0.0), grid)
}

// With a fixed lookahead, a minimum distance to obstacles checks the arc for at least the
// time the command takes to cover it: a wall 0.1 m ahead of the footprint that a check of
// 0.1 s does not reach fails the command with one of 0.3 m.
#[test]
fn a_minimum_distance_lengthens_the_arc_check() {
    let grid = wall_from(0.25);
    let short = PurePursuitParams {
        lookahead: Lookahead::Fixed(positive(0.6)),
        max_allowed_time_to_collision_up_to_carrot_s: non_negative(0.1),
        ..params()
    };
    assert!(driving(short, straight(3.0), &grid).is_ok());
    let lengthened = PurePursuitParams {
        min_distance_to_obstacle_m: Some(positive(0.3)),
        ..short
    };
    assert_eq!(
        driving(lengthened, straight(3.0), &grid),
        Err(ControlError::NoValidControl)
    );
}

// With a velocity scaled lookahead, a minimum distance to obstacles checks the arc past the
// carrot as far as that distance or the plan's end, whichever is nearer: a wall within
// 0.6 m on a long plan fails the command, and one past the end of a plan 0.4 m long does
// not.
#[test]
fn a_minimum_distance_reaches_past_the_carrot_up_to_the_plans_end() {
    let scaled = PurePursuitParams {
        lookahead: Lookahead::VelocityScaled(
            VelocityScaledLookahead::parse(positive(0.6), non_negative(0.3), non_negative(1.0))
                .expect("ordered bounds"),
        ),
        max_allowed_time_to_collision_up_to_carrot_s: non_negative(0.1),
        min_distance_to_obstacle_m: Some(positive(0.6)),
        ..params()
    };
    let within_reach = wall_from(0.55);
    assert_eq!(
        driving(scaled, straight(3.0), &within_reach),
        Err(ControlError::NoValidControl)
    );
    let unlengthened = PurePursuitParams {
        min_distance_to_obstacle_m: None,
        ..scaled
    };
    assert!(driving(unlengthened, straight(3.0), &within_reach).is_ok());
    let past_the_end = wall_from(0.65);
    let command = driving(scaled, straight(0.4), &past_the_end).expect("the wall lies past");
    assert!(command.linear_x() > 0.0, "{command:?}");
}

// With a velocity scaled lookahead, the arc check that a minimum distance to obstacles
// lengthens reaches no farther than the longest lookahead: a wall 0.55 m ahead, which a
// check of the minimum distance of 0.6 m meets, leaves the command whose longest lookahead
// is 0.3 m, and fails the one whose longest lookahead is 1 m, the carrot at 0.3 m for both.
#[test]
fn a_minimum_distance_reaches_no_farther_than_the_longest_lookahead() {
    let reaching = |max_m: f64| PurePursuitParams {
        lookahead: Lookahead::VelocityScaled(
            VelocityScaledLookahead::parse(positive(0.6), non_negative(0.3), non_negative(max_m))
                .expect("ordered bounds"),
        ),
        min_distance_to_obstacle_m: Some(positive(0.6)),
        ..params()
    };
    let wall = wall_from(0.55);
    assert!(driving(reaching(0.3), straight(3.0), &wall).is_ok());
    assert_eq!(
        driving(reaching(1.0), straight(3.0), &wall),
        Err(ControlError::NoValidControl)
    );
}

// Lookahead bounds whose minimum lies above their maximum are refused, naming what to give;
// equal bounds hold one distance; and the longest lookahead reads back.
#[test]
fn inverted_lookahead_bounds_are_refused() {
    let parse = |min: f64, max: f64| {
        VelocityScaledLookahead::parse(positive(1.5), non_negative(min), non_negative(max))
    };
    let inverted = parse(0.6, 0.3).unwrap_err();
    assert_eq!(
        inverted,
        InvertedLookahead {
            min_m: 0.6,
            max_m: 0.3
        }
    );
    assert!(
        inverted
            .to_string()
            .contains("give the shorter distance as the minimum"),
        "{inverted}"
    );
    assert_eq!(parse(0.3, 0.3).expect("equal bounds").max_m(), 0.3);
    assert_eq!(parse(0.2, 0.7).expect("ordered bounds").max_m(), 0.7);
}
