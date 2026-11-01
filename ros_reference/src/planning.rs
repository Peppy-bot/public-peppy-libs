//! Planning. The reference is OMPL's geometric planners planning through worlds of box
//! obstacles in the unit cube, built, seeded and bounded as `ompl_bridge` builds, seeds
//! and bounds a plan: the status, the ending, the path, its length and its
//! simplification of each plan. A plan of PRM answers its status alone: its path, and so
//! its ending, depend on when its search finds a path.

use crate::{Kind, read_committed_reference, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a planning scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "planning")]
    Planning,
}

/// The planner whose plans answer their status alone.
const STATUS_ALONE: &str = "PRM";

/// The step in which a motion is walked, the worlds, and the plans through them.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    motion_step: f64,
    worlds: Vec<World>,
    plans: Vec<PlanScenario>,
}

/// An axis-aligned box, open: a state on its surface is free.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Obstacle {
    pub min: Vec<f64>,
    pub max: Vec<f64>,
}

/// A world of boxes in the unit cube, with the start and the goal of its plans.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct World {
    pub obstacles: Vec<Obstacle>,
    pub start: Vec<f64>,
    pub goal: Vec<f64>,
}

#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct PlanScenario {
    world: usize,
    planner: String,
    seed: u32,
    iterations: u64,
    simplification: u32,
}

/// The scenario as the reference read it, and the answer of each plan.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    motion_step: f64,
    worlds: Vec<World>,
    plans: Vec<PlanRead>,
}

/// A plan as the reference read it, and its answer. An ending, a path, a length or a
/// simplification that the plan does not have is null, and an answer that leaves one out
/// is refused.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PlanRead {
    world: usize,
    planner: String,
    seed: u32,
    iterations: u64,
    simplification: u32,
    status: PlanStatus,
    #[serde(deserialize_with = "Option::deserialize")]
    ended_by: Option<Ending>,
    #[serde(deserialize_with = "Option::deserialize")]
    path: Option<Vec<Vec<f64>>>,
    #[serde(deserialize_with = "Option::deserialize")]
    length: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    simplified: Option<Vec<Vec<f64>>>,
}

/// What OMPL's planner answered to a plan, by OMPL's name for it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub enum PlanStatus {
    #[serde(rename = "Exact solution")]
    Exact,
    #[serde(rename = "Approximate solution")]
    Approximate,
    #[serde(rename = "Timeout")]
    Timeout,
}

/// What ended a plan's planner, as the C++ program names it.
#[derive(Debug, PartialEq, Deserialize)]
pub enum Ending {
    PlannerReturned,
    IterationBound,
    TimeLimit,
}

/// One plan and what OMPL's planner answered to it.
pub struct PlanCase {
    /// The index of the plan's world in [`Case::worlds`].
    pub world: usize,
    /// The planner, by the name of its class in OMPL.
    pub planner: String,
    pub seed: u32,
    pub iterations: u64,
    /// The iteration bound of the simplification, or 0 for a plan that is not simplified.
    pub simplification: u32,
    pub status: PlanStatus,
    /// What the plan answered beside its status, for every planner but PRM, whose plans
    /// answer their status alone.
    pub planned: Option<Planned>,
}

/// What a plan of a planner other than PRM answered beside its status.
pub struct Planned {
    pub ended_by: Ending,
    /// The path's states, when the planner found an exact path.
    pub path: Option<Vec<Vec<f64>>>,
    /// The path's length, as OMPL measures it.
    pub length: Option<f64>,
    /// The simplified path, when the planner found an exact path and the plan is
    /// simplified.
    pub simplified: Option<Vec<Vec<f64>>>,
}

/// A planning scenario paired with its reference output.
pub struct Case {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub motion_step: f64,
    pub worlds: Vec<World>,
    pub plans: Vec<PlanCase>,
}

impl Case {
    /// Every planning scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: the reference read the scenario's motion step,
    /// worlds and plans, and gave an ending, a path, its length and its simplification
    /// exactly where the plan has them.
    pub fn load_all() -> Vec<Self> {
        load_with(read_reference)
    }

    /// Every planning scenario, as [`Case::load_all`] pairs it, with its committed
    /// reference output, the amd64 image's, whatever directory `ROS_REFERENCE_OUTPUTS`
    /// names.
    pub fn load_committed() -> Vec<Self> {
        load_with(read_committed_reference)
    }
}

/// Every planning scenario, paired with the reference output that `read` reads by name.
fn load_with(read: fn(&str) -> Reference) -> Vec<Case> {
    scenario_names(Kind::Planning)
        .into_iter()
        .map(|name| {
            let scenario = read_scenario(&name);
            let reference = read(&name);
            pair(name, scenario, reference)
        })
        .collect()
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Case {
    let Scenario {
        kind: ThisKind::Planning,
        motion_step,
        worlds,
        plans,
    } = scenario;
    let Reference {
        kind: ThisKind::Planning,
        motion_step: motion_step_read,
        worlds: worlds_read,
        plans: plans_read,
    } = reference;
    assert_eq!(
        motion_step_read, motion_step,
        "{name}: the reference read the motion step"
    );
    assert_eq!(worlds_read, worlds, "{name}: the reference read the worlds");
    assert_eq!(
        plans_read.len(),
        plans.len(),
        "{name}: the reference answered every plan"
    );
    let plans = plans
        .into_iter()
        .zip(plans_read)
        .enumerate()
        .map(|(index, (plan, read))| {
            let what = format!("{name} plan {index}");
            let echoed = PlanScenario {
                world: read.world,
                planner: read.planner.clone(),
                seed: read.seed,
                iterations: read.iterations,
                simplification: read.simplification,
            };
            assert_eq!(echoed, plan, "{what}: the reference read the plan");
            let dimension = worlds[plan.world].start.len();
            let status = read.status;
            let planned = planned(&what, &plan, read, dimension);
            PlanCase {
                world: plan.world,
                planner: plan.planner,
                seed: plan.seed,
                iterations: plan.iterations,
                simplification: plan.simplification,
                status,
                planned,
            }
        })
        .collect();
    Case {
        name,
        motion_step,
        worlds,
        plans,
    }
}

/// What the answer `read` to `plan` holds beside its status: none for a plan of PRM, whose
/// answer holds its status alone, and for every other plan its ending, and a path and its
/// length exactly where the planner found an exact path, simplified exactly where the plan
/// is simplified, each state of `dimension` values.
fn planned(what: &str, plan: &PlanScenario, read: PlanRead, dimension: usize) -> Option<Planned> {
    let PlanRead {
        status,
        ended_by,
        path,
        length,
        simplified,
        ..
    } = read;
    if plan.planner == STATUS_ALONE {
        assert!(
            ended_by.is_none() && path.is_none() && length.is_none() && simplified.is_none(),
            "{what}: a plan of {STATUS_ALONE} answers its status alone"
        );
        return None;
    }
    let ended_by = ended_by.unwrap_or_else(|| panic!("{what}: the plan's ending"));
    let exact = status == PlanStatus::Exact;
    assert!(
        path.is_some() == exact && length.is_some() == exact,
        "{what}: a path and its length exactly where the planner found an exact path"
    );
    assert_eq!(
        simplified.is_some(),
        exact && plan.simplification > 0,
        "{what}: a simplification exactly where a simplified plan found an exact path"
    );
    assert!(
        path.iter()
            .chain(&simplified)
            .flatten()
            .all(|state| state.len() == dimension),
        "{what}: each state of {dimension} values"
    );
    Some(Planned {
        ended_by,
        path,
        length,
        simplified,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const HALF: &str = "0.5";
    const ONE: &str = "1";

    /// A scenario of one world in two dimensions with no obstacle, a plan of RRTConnect
    /// simplified, and a plan of PRM.
    fn scenario() -> Scenario {
        serde_json::from_str(
            r#"{"kind": "planning", "motion_step": 0.5,
                "worlds": [{"obstacles": [], "start": [0.5, 0.5], "goal": [1.0, 1.0]}],
                "plans": [{"world": 0, "planner": "RRTConnect", "seed": 1,
                           "iterations": 10, "simplification": 5},
                          {"world": 0, "planner": "PRM", "seed": 1,
                           "iterations": 10, "simplification": 0}]}"#,
        )
        .expect("parses")
    }

    /// The answer of the plan of RRTConnect of [`scenario`]: `fields`, after what the
    /// reference read.
    fn plan(fields: &str) -> String {
        format!(
            r#"{{"world": 0, "planner": "RRTConnect", "seed": 1, "iterations": 10,
                 "simplification": 5, {fields}}}"#
        )
    }

    /// The answer of the plan of PRM of [`scenario`]: `fields`, after what the reference
    /// read.
    fn prm_plan(fields: &str) -> String {
        format!(
            r#"{{"world": 0, "planner": "PRM", "seed": 1, "iterations": 10,
                 "simplification": 0, {fields}}}"#
        )
    }

    const SOLVED: &str = r#""status": "Exact solution", "ended_by": "PlannerReturned",
        "path": [[0.5, 0.5], [1, 1]], "length": 0.7071067811865476,
        "simplified": [[0.5, 0.5], [1, 1]]"#;

    const PRM_FOUND: &str = r#""status": "Exact solution", "ended_by": null, "path": null,
        "length": null, "simplified": null"#;

    /// The output that read the scenario of [`scenario`], with the answers `plan` and
    /// `prm`, and the start read as `start`.
    fn reference(plan: &str, prm: &str, start: &str) -> Result<Reference, serde_json::Error> {
        serde_json::from_str(&format!(
            r#"{{"kind": "planning", "motion_step": {HALF},
                 "worlds": [{{"obstacles": [], "start": [{start}, {HALF}],
                              "goal": [{ONE}, {ONE}]}}],
                 "plans": [{plan}, {prm}]}}"#
        ))
    }

    fn solved(start: &str) -> Reference {
        reference(&plan(SOLVED), &prm_plan(PRM_FOUND), start).expect("parses")
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let scenario = |kind: &str| {
            format!(r#"{{"kind": "{kind}", "motion_step": 0.5, "worlds": [], "plans": []}}"#)
        };
        assert!(serde_json::from_str::<Scenario>(&scenario("self_distance")).is_err());
        assert!(serde_json::from_str::<Scenario>(&scenario("planning")).is_ok());
    }

    #[test]
    fn a_plan_pairs_with_the_output_that_read_it() {
        let case = pair("scenario".into(), scenario(), solved(HALF));
        let planned = case.plans[0].planned.as_ref().expect("RRTConnect's answer");
        assert_eq!(case.plans[0].status, PlanStatus::Exact);
        assert_eq!(planned.ended_by, Ending::PlannerReturned);
        assert_eq!(planned.path, Some(vec![vec![0.5, 0.5], vec![1.0, 1.0]]));
        assert_eq!(planned.length, Some(f64::from_bits(0x3fe6_a09e_667f_3bcd)));
        assert_eq!(case.plans[1].status, PlanStatus::Exact);
        assert!(
            case.plans[1].planned.is_none(),
            "PRM answers its status alone"
        );
    }

    #[test]
    #[should_panic(expected = "the reference read the worlds")]
    fn an_output_that_read_another_world_does_not_pair() {
        pair("scenario".into(), scenario(), solved(ONE));
    }

    #[test]
    fn an_answer_with_no_path_writes_it_as_null_and_one_that_leaves_it_out_is_refused() {
        let fields = [
            r#""status": "Timeout""#,
            r#""ended_by": "IterationBound""#,
            r#""path": null"#,
            r#""length": null"#,
            r#""simplified": null"#,
        ];
        let unsolved = pair(
            "scenario".into(),
            scenario(),
            reference(&plan(&fields.join(", ")), &prm_plan(PRM_FOUND), HALF).expect("parses"),
        );
        assert_eq!(unsolved.plans[0].status, PlanStatus::Timeout);
        let planned = unsolved.plans[0]
            .planned
            .as_ref()
            .expect("RRTConnect's answer");
        assert!(planned.path.is_none() && planned.simplified.is_none());
        for left_out in 1..fields.len() {
            let mut without = fields.to_vec();
            let field = without.remove(left_out);
            assert!(
                reference(&plan(&without.join(", ")), &prm_plan(PRM_FOUND), HALF).is_err(),
                "an answer without {field} parses"
            );
        }
    }

    #[test]
    #[should_panic(
        expected = "a path and its length exactly where the planner found an exact path"
    )]
    fn a_timeout_with_a_path_does_not_pair() {
        let fields = SOLVED.replace("Exact solution", "Timeout");
        pair(
            "scenario".into(),
            scenario(),
            reference(&plan(&fields), &prm_plan(PRM_FOUND), HALF).expect("parses"),
        );
    }

    #[test]
    #[should_panic(expected = "a plan of PRM answers its status alone")]
    fn a_plan_of_prm_with_a_path_does_not_pair() {
        let prm = SOLVED.replace(r#""ended_by": "PlannerReturned","#, r#""ended_by": null,"#);
        pair(
            "scenario".into(),
            scenario(),
            reference(&plan(SOLVED), &prm_plan(&prm), HALF).expect("parses"),
        );
    }

    #[test]
    #[should_panic(expected = "the plan's ending")]
    fn a_plan_of_another_planner_without_its_ending_does_not_pair() {
        let fields = SOLVED.replace(r#""PlannerReturned""#, "null");
        pair(
            "scenario".into(),
            scenario(),
            reference(&plan(&fields), &prm_plan(PRM_FOUND), HALF).expect("parses"),
        );
    }

    #[test]
    fn every_planning_scenario_pairs_with_its_reference() {
        let cases = Case::load_all();
        assert!(!cases.is_empty(), "scenarios/ holds a planning scenario");
        for case in cases {
            assert!(!case.plans.is_empty(), "{} holds a plan", case.name);
        }
    }
}
