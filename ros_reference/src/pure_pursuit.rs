//! Pure pursuit. The reference is Nav2's regulated pure pursuit controller
//! (nav2_regulated_pure_pursuit_controller 1.5.2) as Nav2's controller server (nav2_controller
//! 1.5.2) runs it with its feasible path handler, simple goal checker and simple progress
//! checker: on each scenario's cost grid and footprint, following the scenario's plan, it
//! takes each step at its own time with the base's pose and speed, and gives the pruned
//! plan, whether the goal is reached, whether the base makes progress, and the command or
//! the exception that ended the step.

use crate::{Grid, Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a pure pursuit scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "pure_pursuit")]
    PurePursuit,
}

/// The controller's parameters, by Nav2's names: every parameter it declares.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerParameters {
    pub max_linear_vel: f64,
    pub min_linear_vel: f64,
    pub max_angular_vel: f64,
    pub min_angular_vel: f64,
    pub lookahead_dist: f64,
    pub min_lookahead_dist: f64,
    pub max_lookahead_dist: f64,
    pub lookahead_time: f64,
    pub rotate_to_heading_angular_vel: f64,
    pub use_velocity_scaled_lookahead_dist: bool,
    pub min_approach_linear_velocity: f64,
    pub approach_velocity_scaling_dist: f64,
    pub max_allowed_time_to_collision_up_to_carrot: f64,
    pub min_distance_to_obstacle: f64,
    pub use_regulated_linear_velocity_scaling: bool,
    pub use_cost_regulated_linear_velocity_scaling: bool,
    pub cost_scaling_dist: f64,
    pub cost_scaling_gain: f64,
    pub inflation_cost_scaling_factor: f64,
    pub regulated_linear_scaling_min_radius: f64,
    pub regulated_linear_scaling_min_speed: f64,
    pub use_fixed_curvature_lookahead: bool,
    pub curvature_lookahead_dist: f64,
    pub use_rotate_to_heading: bool,
    pub rotate_to_heading_min_angle: f64,
    pub max_linear_accel: f64,
    pub max_linear_decel: f64,
    pub max_angular_accel: f64,
    pub max_angular_decel: f64,
    pub use_cancel_deceleration: bool,
    pub cancel_deceleration: f64,
    pub allow_reversing: bool,
    pub interpolate_curvature_after_goal: bool,
    pub use_collision_detection: bool,
    pub use_dynamic_window: bool,
    pub allow_obstacle_checking_beyond_goal: bool,
}

/// The simple goal checker's parameters, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GoalCheckerParameters {
    pub xy_goal_tolerance: f64,
    pub xy_goal_tolerance_buffer: f64,
    pub yaw_goal_tolerance: f64,
    pub path_length_tolerance: f64,
    pub stateful: bool,
    pub symmetric_yaw_tolerance: bool,
}

/// The simple progress checker's parameters, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgressCheckerParameters {
    pub required_movement_radius: f64,
    pub movement_time_allowance: f64,
}

/// The feasible path handler's parameters, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PathHandlerParameters {
    pub reject_unit_path: bool,
    pub max_robot_pose_search_dist: f64,
    pub prune_distance: f64,
    pub enforce_path_inversion: bool,
    pub enforce_path_rotation: bool,
    pub inversion_xy_tolerance: f64,
    pub inversion_yaw_tolerance: f64,
    pub minimum_rotation_angle: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    grid: Grid,
    /// One row of cells a member, from the first row.
    costs: Vec<Vec<u8>>,
    track_unknown_space: bool,
    footprint: Vec<[f64; 2]>,
    controller_frequency: f64,
    controller: ControllerParameters,
    goal_checker: GoalCheckerParameters,
    progress_checker: ProgressCheckerParameters,
    path_handler: PathHandlerParameters,
    /// Each pose x and y in meters and a heading in radians.
    plan: Vec<[f64; 3]>,
    steps: Vec<StepInput>,
}

/// A step: when it runs, in nanoseconds, the base's pose, and its speed, linear x, linear y
/// and angular z.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StepInput {
    at_ns: i64,
    pose: [f64; 3],
    speed: [f64; 3],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    grid: Grid,
    costs: Vec<Vec<u8>>,
    footprint: Vec<[f64; 2]>,
    inscribed_radius: f64,
    circumscribed_radius: f64,
    plan: Vec<[f64; 3]>,
    steps: Vec<StepRead>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StepRead {
    at_ns: i64,
    pose: [f64; 3],
    speed: [f64; 3],
    pruned: Vec<usize>,
    outcome: Outcome,
    /// Null when the step ended before the check; each field is required.
    #[serde(deserialize_with = "Option::deserialize")]
    goal_reached: Option<bool>,
    #[serde(deserialize_with = "Option::deserialize")]
    progress: Option<bool>,
    twist: Vec<f64>,
}

/// How a step ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    /// The controller gave a command.
    Command,
    /// The goal checker found the goal reached, so no command ran.
    GoalReached,
    /// The controller threw `NoValidControl`: a contact along the arc ahead.
    NoValidControl,
    /// The controller threw `ControllerException`: the base off the grid.
    BaseOffGrid,
    /// The path handler threw `InvalidPath`: no pose of the plan ahead on the grid.
    PlanOffGrid,
}

/// One step as the reference ran it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    pub at_ns: i64,
    /// The base's pose as the scenario gives it; the reference read it back within
    /// 1e-12 rad.
    pub pose: [f64; 3],
    pub speed: [f64; 3],
    /// The index in the plan of the pruned plan's first pose, and its count of poses.
    pub pruned: Option<(usize, usize)>,
    pub outcome: Outcome,
    pub goal_reached: Option<bool>,
    pub progress: Option<bool>,
    /// The command, linear x, linear y and angular z.
    pub twist: Option<[f64; 3]>,
}

/// A pure pursuit scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub grid: Grid,
    /// Every cell's cost, row by row from the first cell.
    pub costs: Vec<u8>,
    pub track_unknown_space: bool,
    pub footprint: Vec<[f64; 2]>,
    /// The footprint's radii as Nav2's layered cost map takes them.
    pub inscribed_radius: f64,
    pub circumscribed_radius: f64,
    pub controller_frequency: f64,
    pub controller: ControllerParameters,
    pub goal_checker: GoalCheckerParameters,
    pub progress_checker: ProgressCheckerParameters,
    pub path_handler: PathHandlerParameters,
    pub plan: Vec<[f64; 3]>,
    pub steps: Vec<Step>,
}

/// How far a heading read back from its quaternion may lie from the heading that made it,
/// in radians.
const HEADING_READ_RAD: f64 = 1e-12;

impl Cases {
    /// Every pure pursuit scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: the reference read the grid, its costs, the
    /// footprint, the plan and each step's pose and speed.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::PurePursuit)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

/// Panics unless `read`, a pose as the reference read it, holds `given`'s position and,
/// within [`HEADING_READ_RAD`], its heading.
fn check_pose(read: [f64; 3], given: [f64; 3], what: &str) {
    assert!(
        read[..2] == given[..2] && (read[2] - given[2]).abs() < HEADING_READ_RAD,
        "{what}: the reference read the pose {given:?} as {read:?}"
    );
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::PurePursuit,
        grid,
        costs,
        track_unknown_space,
        footprint,
        controller_frequency,
        controller,
        goal_checker,
        progress_checker,
        path_handler,
        plan,
        steps,
    } = scenario;
    let Reference {
        kind: ThisKind::PurePursuit,
        grid: grid_read,
        costs: costs_read,
        footprint: footprint_read,
        inscribed_radius,
        circumscribed_radius,
        plan: plan_read,
        steps: steps_read,
    } = reference;
    assert_eq!(grid_read, grid, "{name}: the reference read the grid");
    assert_eq!(costs_read, costs, "{name}: the reference read the costs");
    assert_eq!(
        footprint_read, footprint,
        "{name}: the reference read the footprint"
    );
    assert_eq!(
        plan_read.len(),
        plan.len(),
        "{name}: the reference read the plan"
    );
    for (index, (read, &given)) in plan_read.iter().zip(&plan).enumerate() {
        check_pose(*read, given, &format!("{name} plan pose {index}"));
    }
    assert_eq!(
        steps_read.len(),
        steps.len(),
        "{name}: the reference ran every step"
    );
    let steps = steps
        .iter()
        .zip(steps_read)
        .enumerate()
        .map(|(index, (input, read))| {
            let what = format!("{name} step {index}");
            check_pose(read.pose, input.pose, &what);
            assert_eq!(
                (read.at_ns, read.speed),
                (input.at_ns, input.speed),
                "{what}: the reference read the step"
            );
            let pruned = match read.pruned.as_slice() {
                &[first, count] => Some((first, count)),
                [] => None,
                other => {
                    panic!("{what}: a pruned plan is its first index and count, not {other:?}")
                }
            };
            let twist = match (read.outcome, read.twist.as_slice()) {
                (Outcome::Command, &[x, y, z]) => Some([x, y, z]),
                (Outcome::Command, other) => panic!("{what}: a command of {other:?}"),
                (_, []) => None,
                (outcome, other) => {
                    panic!("{what}: an outcome {outcome:?} with a command {other:?}")
                }
            };
            Step {
                at_ns: input.at_ns,
                pose: input.pose,
                speed: input.speed,
                pruned,
                outcome: read.outcome,
                goal_reached: read.goal_reached,
                progress: read.progress,
                twist,
            }
        })
        .collect();
    Cases {
        name,
        grid,
        costs: costs.into_iter().flatten().collect(),
        track_unknown_space,
        footprint,
        inscribed_radius,
        circumscribed_radius,
        controller_frequency,
        controller,
        goal_checker,
        progress_checker,
        path_handler,
        plan,
        steps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "smac_2d", "grid": {"size_x": 1, "size_y": 1,
            "resolution": 1.0, "origin": [0.0, 0.0]}, "costs": [[0]]}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn every_pure_pursuit_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a pure pursuit scenario"
        );
        for cases in scenarios {
            assert!(
                cases
                    .steps
                    .iter()
                    .any(|step| step.outcome == Outcome::Command),
                "{} gives a command",
                cases.name
            );
            assert_eq!(
                cases.costs.len(),
                cases.grid.size_x as usize * cases.grid.size_y as usize,
                "{} holds a cost a cell",
                cases.name
            );
        }
    }
}
