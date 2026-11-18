//! Smac 2D. The reference is Nav2's Smac 2D planner (nav2_smac_planner 1.5.2): on each
//! scenario's cost grid, it plans each start and goal by `SmacPlanner2D::createPlan`'s
//! steps, once with its smoother off and once smoothed at the scenario's iteration bound,
//! and gives each path's poses, or the failure it throws.

use crate::{Grid, Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a Smac 2D scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "smac_2d")]
    Smac2d,
}

/// The planner's parameters, by Smac 2D's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlannerParameters {
    pub tolerance: f64,
    pub cost_travel_multiplier: f64,
    pub allow_unknown: bool,
    pub max_iterations: u32,
    pub max_on_approach_iterations: u32,
    pub terminal_checking_interval: u32,
    pub max_planning_time: f64,
    pub use_final_approach_orientation: bool,
}

/// The smoother's parameters, by Smac 2D's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmootherParameters {
    pub tolerance: f64,
    pub max_iterations: u32,
    pub w_data: f64,
    pub w_smooth: f64,
    pub do_refinement: bool,
    pub refinement_num: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    grid: Grid,
    /// One row of cells a member, from the first row.
    costs: Vec<Vec<u8>>,
    footprint: Vec<[f64; 2]>,
    planner: PlannerParameters,
    smoother: SmootherParameters,
    plans: Vec<PlanInput>,
}

/// A start and a goal, each x and y in meters and a heading in radians.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PlanInput {
    start: [f64; 3],
    goal: [f64; 3],
}

/// The reference output: the grid, the footprint and the parameters as the reference read
/// them, the planner's tolerance and multiplier in the single precision Smac holds them
/// in, and each plan.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    grid: Grid,
    costs: Vec<Vec<u8>>,
    footprint: Vec<[f64; 2]>,
    planner: PlannerParameters,
    smoother: SmootherParameters,
    plans: Vec<PlanRead>,
}

/// A start and a goal as the reference read them, each heading read back from its
/// quaternion, and the plan of each planner.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PlanRead {
    start: [f64; 3],
    goal: [f64; 3],
    outcome: Outcome,
    raw: Vec<[f64; 3]>,
    smoothed_outcome: Outcome,
    smoothed: Vec<[f64; 3]>,
}

/// How a plan of Smac 2D ended: with a path, or with the exception it threw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    Path,
    StartOccupied,
    GoalOccupied,
    NoValidPath,
    TimedOut,
}

/// One plan: its start and goal as the scenario gives them, how Smac's plan ended, and
/// its poses, each x, y and the heading its quaternion reads back, with the smoother off
/// and smoothed.
pub struct Plan {
    pub start: [f64; 3],
    pub goal: [f64; 3],
    pub outcome: Outcome,
    pub raw: Vec<[f64; 3]>,
    pub smoothed: Vec<[f64; 3]>,
}

/// A Smac 2D scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub grid: Grid,
    /// Every cell's cost, row by row from the first cell.
    pub costs: Vec<u8>,
    pub footprint: Vec<[f64; 2]>,
    pub planner: PlannerParameters,
    pub smoother: SmootherParameters,
    pub plans: Vec<Plan>,
}

/// How far a heading read back from its quaternion may lie from the heading that made
/// it, in radians.
const HEADING_READ_RAD: f64 = 1e-12;

impl Cases {
    /// Every Smac 2D scenario under `scenarios/`, in the order of their names, each paired
    /// with its reference output: the reference read the grid, its costs, the footprint,
    /// the parameters and each start and goal, and its two planners ended each plan alike.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::Smac2d)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

/// `value` as Smac holds a parameter it reads into a float.
fn single(value: f64) -> f64 {
    f64::from(value as f32)
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::Smac2d,
        grid,
        costs,
        footprint,
        planner,
        smoother,
        plans,
    } = scenario;
    let Reference {
        kind: ThisKind::Smac2d,
        grid: grid_read,
        costs: costs_read,
        footprint: footprint_read,
        planner: planner_read,
        smoother: smoother_read,
        plans: plans_read,
    } = reference;
    assert_eq!(grid_read, grid, "{name}: the reference read the grid");
    assert_eq!(costs_read, costs, "{name}: the reference read the costs");
    assert_eq!(
        footprint_read, footprint,
        "{name}: the reference read the footprint"
    );
    assert_eq!(
        planner_read,
        PlannerParameters {
            tolerance: single(planner.tolerance),
            cost_travel_multiplier: single(planner.cost_travel_multiplier),
            ..planner
        },
        "{name}: the reference read the planner's parameters, two of them in single precision"
    );
    assert_eq!(
        smoother_read, smoother,
        "{name}: the reference read the smoother's parameters"
    );
    assert_eq!(
        plans.len(),
        plans_read.len(),
        "{name}: the reference planned every start and goal"
    );
    let plans = plans
        .into_iter()
        .zip(plans_read)
        .enumerate()
        .map(|(index, (input, read))| {
            for (given, read, what) in [
                (input.start, read.start, "start"),
                (input.goal, read.goal, "goal"),
            ] {
                assert!(
                    read[..2] == given[..2] && (read[2] - given[2]).abs() < HEADING_READ_RAD,
                    "{name} plan {index}: the reference read the {what} {given:?} as {read:?}"
                );
            }
            assert_eq!(
                read.outcome, read.smoothed_outcome,
                "{name} plan {index}: Smac's planners ended it alike"
            );
            Plan {
                start: input.start,
                goal: input.goal,
                outcome: read.outcome,
                raw: read.raw,
                smoothed: read.smoothed,
            }
        })
        .collect();
    Cases {
        name,
        grid,
        costs: costs.into_iter().flatten().collect(),
        footprint,
        planner,
        smoother,
        plans,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "occupancy", "grid": {"size_x": 1, "size_y": 1,
            "resolution": 1.0, "origin": [0.0, 0.0]}, "costs": [[0]], "footprint": [],
            "planner": {}, "smoother": {}, "plans": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn every_smac_2d_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(!scenarios.is_empty(), "scenarios/ holds a Smac 2D scenario");
        for cases in scenarios {
            assert!(!cases.plans.is_empty(), "{} holds a plan", cases.name);
            assert_eq!(
                cases.costs.len(),
                cases.grid.size_x as usize * cases.grid.size_y as usize,
                "{} holds a cost a cell",
                cases.name
            );
        }
    }
}
