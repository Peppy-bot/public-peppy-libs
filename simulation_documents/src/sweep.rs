//! `sweep/v1`: one workflow revision run in many simulated worlds.
//!
//! A sweep names its workflow, the launcher of its stack, the robot of each
//! role in the world patterns, each world pattern with its seeds and the
//! ending it expects, the faults it injects, the judges of each run, the
//! watch parameters and incidents of its runs, and the thresholds of
//! sign-off. [`SweepDocument::parse`] holds the rules of the document alone,
//! and [`Sweep::new`] those against its workflow and world patterns.

use documents_core::{Document, Kind, Manifest};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use workflow_documents::names::StepId;
use workflow_documents::run::{Incidents, WorkflowRef, binding_refusals};
use workflow_documents::work_cell::{WatchParameter, watch_refusals};
use workflow_documents::{At, Name as WorkflowName, Refusal, Refusals, Thresholds, Workflow};

use crate::stream::Seed;
use crate::{Error, Judge, Name, ParameterName, Physics, RobotName, WorldPattern};

/// The kind of a sweep document.
pub const SWEEP: Kind = Kind::new("simulation_schema", "sweep", 1);

/// The longest push, seconds, as `scene.apply_force` takes it.
pub const MAX_PUSH_S: f64 = 30.0;

/// The fields of a sweep document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepDocument {
    pub manifest: Manifest,
    pub workflow: WorkflowRef,
    /// The launcher of the stack of the sweep, with a simulation.
    pub launcher: String,
    /// The robot of each role in the world patterns.
    pub roles: BTreeMap<WorkflowName, RobotName>,
    pub worlds: Vec<SweptWorld>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub faults: Vec<Fault>,
    pub judges: SweepJudges,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub watch: BTreeMap<ParameterName, WatchParameter>,
    #[serde(default, skip_serializing_if = "Incidents::is_none")]
    pub incidents: Incidents,
    pub thresholds: Thresholds,
}

/// A world pattern of a sweep: the pattern, its seeds and the ending each of
/// its runs expects.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweptWorld {
    pub world: Manifest,
    pub seeds: Seeds,
    pub expect: Expected,
}

/// The seeds from `from` to `to`, both included.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "SeedsFields", into = "SeedsFields")]
pub struct Seeds {
    pub from: Seed,
    pub to: Seed,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "seeds, { from, to }")]
struct SeedsFields {
    from: Seed,
    to: Seed,
}

impl TryFrom<SeedsFields> for Seeds {
    type Error = Error;

    fn try_from(fields: SeedsFields) -> Result<Seeds, Error> {
        if fields.from > fields.to {
            return Err(Error::rule(format!(
                "the seeds from {} to {} end below their start; write `to` at least `from`",
                fields.from, fields.to
            )));
        }
        Ok(Seeds {
            from: fields.from,
            to: fields.to,
        })
    }
}

impl From<Seeds> for SeedsFields {
    fn from(seeds: Seeds) -> SeedsFields {
        SeedsFields {
            from: seeds.from,
            to: seeds.to,
        }
    }
}

impl Seeds {
    /// Each seed, in order.
    pub fn iter(&self) -> impl Iterator<Item = Seed> {
        (self.from.0..=self.to.0).map(Seed)
    }

    pub fn contains(&self, seed: Seed) -> bool {
        self.from <= seed && seed <= self.to
    }

    /// Whether each seed of `other` is one of these.
    pub fn covers(&self, other: &Seeds) -> bool {
        self.contains(other.from) && self.contains(other.to)
    }
}

impl fmt::Display for Seeds {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} to {}", self.from, self.to)
    }
}

/// The ending that each run of a world pattern expects.
#[derive(Clone, Debug, PartialEq)]
pub enum Expected {
    Completed,
    Degraded,
    Failed,
    /// The director refuses the run before the initialization, and names
    /// the assumption.
    Refused {
        assumption: WorkflowName,
    },
}

const EXPECTED_FORM: &str =
    "an expected ending is `completed`, `degraded`, `failed` or `{ refused: <assumption> }`";

impl Serialize for Expected {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Expected::Completed => serializer.serialize_str("completed"),
            Expected::Degraded => serializer.serialize_str("degraded"),
            Expected::Failed => serializer.serialize_str("failed"),
            Expected::Refused { assumption } => {
                BTreeMap::from([("refused", assumption)]).serialize(serializer)
            }
        }
    }
}

impl<'de> Deserialize<'de> for Expected {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Expected, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;
        let refused = value
            .as_object()
            .filter(|fields| fields.len() == 1)
            .and_then(|fields| fields.get("refused")?.as_str())
            .map(WorkflowName::parse);
        let expected = match (value.as_str(), refused) {
            (Some("completed"), _) => Expected::Completed,
            (Some("degraded"), _) => Expected::Degraded,
            (Some("failed"), _) => Expected::Failed,
            (_, Some(Ok(assumption))) => Expected::Refused { assumption },
            (_, Some(Err(error))) => return Err(serde::de::Error::custom(error)),
            _ => return Err(serde::de::Error::custom(EXPECTED_FORM)),
        };
        Ok(expected)
    }
}

/// A fault that a sweep injects into the runs of some seeds of a world
/// pattern, at the first event that `when` names in the run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fault {
    pub world: Manifest,
    pub seeds: Seeds,
    pub when: FaultWhen,
    #[serde(rename = "do")]
    pub action: FaultAction,
    /// The ending of the runs of the fault, in place of that of their world
    /// pattern.
    pub expect: FaultExpected,
}

/// The event of a run at which a fault starts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum FaultWhen {
    StepStarts(StepRef),
    StepEnds(StepRef),
}

impl FaultWhen {
    pub fn step(&self) -> &StepRef {
        match self {
            FaultWhen::StepStarts(step) | FaultWhen::StepEnds(step) => step,
        }
    }
}

/// A step of a workflow, written `<role>.<id>`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StepRef {
    pub role: WorkflowName,
    pub id: StepId,
}

impl fmt::Display for StepRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.role, self.id)
    }
}

impl Serialize for StepRef {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for StepRef {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<StepRef, D::Error> {
        let text = String::deserialize(deserializer)?;
        let refused = || {
            serde::de::Error::custom(format!(
                "`{text}` is no step: write `<role>.<id>`, such as `so101_a.pick_loop`"
            ))
        };
        let (role, id) = text.split_once('.').ok_or_else(refused)?;
        Ok(StepRef {
            role: WorkflowName::parse(role).map_err(|_| refused())?,
            id: StepId::parse(id).map_err(|_| refused())?,
        })
    }
}

/// What a fault does.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum FaultAction {
    Push(Push),
    /// The robot leaves the stack: a robot of the world pattern that no role
    /// binds.
    RemoveRobot(RobotName),
}

/// A constant world-frame push on a free dynamic object, as
/// `scene.apply_force` gives it: the force, newtons, for `duration_s`, more
/// than 0 s and at most 30 s.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "PushFields", into = "PushFields")]
pub struct Push {
    pub object: Name,
    pub force: [f64; 3],
    pub duration_s: f64,
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a push, { object, force, duration_s }"
)]
struct PushFields {
    object: Name,
    force: [f64; 3],
    duration_s: f64,
}

impl TryFrom<PushFields> for Push {
    type Error = Error;

    fn try_from(fields: PushFields) -> Result<Push, Error> {
        let within = fields.duration_s > 0.0 && fields.duration_s <= MAX_PUSH_S;
        if !within {
            return Err(Error::rule(format!(
                "the push lasts {} s; a push lasts more than 0 s and at most {MAX_PUSH_S} s",
                fields.duration_s
            )));
        }
        Ok(Push {
            object: fields.object,
            force: fields.force,
            duration_s: fields.duration_s,
        })
    }
}

impl From<Push> for PushFields {
    fn from(push: Push) -> PushFields {
        PushFields {
            object: push.object,
            force: push.force,
            duration_s: push.duration_s,
        }
    }
}

/// The ending that the runs of a fault expect.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultExpected {
    Degraded,
    Failed,
}

/// The judges of each run: `success` against the `success` of the workflow,
/// `minimum` against its `minimum`, one judge per assumption, and the
/// `unsafe` judges, each an unsafe event when it holds at the end of a run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepJudges {
    pub success: Judge,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minimum: Option<Judge>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub assumptions: BTreeMap<WorkflowName, Judge>,
    #[serde(rename = "unsafe", default, skip_serializing_if = "BTreeMap::is_empty")]
    pub unsafe_events: BTreeMap<WorkflowName, Judge>,
}

impl SweepDocument {
    /// Parses JSON5 text as a sweep document. Refuses, by its place, a blank
    /// launcher, a sweep of no world pattern, a world pattern named twice,
    /// one robot for two roles, and a fault of a world pattern that the sweep
    /// does not name, of seeds that are not of its world pattern, of a
    /// world pattern that expects a refusal, or that removes a robot that a
    /// role binds.
    pub fn parse(text: &str) -> Result<SweepDocument, Error> {
        let document: SweepDocument = Document::parse(text, SWEEP)?.read()?;
        document.check()?;
        Ok(document)
    }

    pub fn document(&self) -> Result<Document, Error> {
        Ok(Document::of(SWEEP, self)?)
    }

    fn check(&self) -> Result<(), Error> {
        if self.launcher.trim().is_empty() {
            return Err(Error::field(
                "launcher",
                "is blank; name the launcher of the stack of the sweep",
            ));
        }
        if self.worlds.is_empty() {
            return Err(Error::field(
                "worlds",
                "names no world pattern; name at least one",
            ));
        }
        for (index, swept) in self.worlds.iter().enumerate() {
            if self.worlds[..index]
                .iter()
                .any(|earlier| earlier.world == swept.world)
            {
                return Err(Error::field(
                    format!("worlds[{index}]"),
                    format!(
                        "names `{}` revision {} again; give each world pattern once",
                        swept.world.name, swept.world.revision
                    ),
                ));
            }
        }
        let mut bound: BTreeMap<&RobotName, &WorkflowName> = BTreeMap::new();
        for (role, robot) in &self.roles {
            if let Some(earlier) = bound.insert(robot, role) {
                return Err(Error::field(
                    format!("roles.{role}"),
                    format!(
                        "`{robot}` is the robot of the role `{earlier}` too: name another robot for each role"
                    ),
                ));
            }
        }
        for (index, fault) in self.faults.iter().enumerate() {
            let at = |field: &str| format!("faults[{index}].{field}");
            let swept = self
                .worlds
                .iter()
                .find(|swept| swept.world == fault.world)
                .ok_or_else(|| {
                    Error::field(
                        at("world"),
                        format!(
                            "names `{}` revision {}, which is no world pattern of `worlds`",
                            fault.world.name, fault.world.revision
                        ),
                    )
                })?;
            if !swept.seeds.covers(&fault.seeds) {
                return Err(Error::field(
                    at("seeds"),
                    format!(
                        "are {}, and the world pattern has the seeds {}; give seeds of the world pattern",
                        fault.seeds, swept.seeds
                    ),
                ));
            }
            if let Expected::Refused { assumption } = &swept.expect {
                return Err(Error::field(
                    at("world"),
                    format!(
                        "expects the refusal of `{assumption}`, so no run starts for a fault to stop; give a world pattern that runs"
                    ),
                ));
            }
            if let FaultAction::RemoveRobot(robot) = &fault.action
                && let Some(role) = bound.get(robot)
            {
                return Err(Error::field(
                    at("do"),
                    format!(
                        "removes `{robot}`, which the role `{role}` binds: the director ends a run interrupted when the lease of a robot of the run ends, so no run can end as the fault expects; remove a robot that no role binds"
                    ),
                ));
            }
        }
        Ok(())
    }
}

/// The refusal of a rule that a sweep breaks at the field `path`.
fn at(path: impl Into<String>, reason: impl Into<String>) -> Refusal {
    Refusal::new(At::field(path), reason)
}

/// A sweep document that fits its workflow and its world patterns.
#[derive(Clone, Debug, PartialEq)]
pub struct Sweep {
    pub document: SweepDocument,
}

impl Sweep {
    /// The sweep of `document`, or the refusal of each rule it breaks
    /// against `workflow` and `patterns`, the world patterns with their
    /// bases folded in: each role of the workflow bound and no other, each
    /// robot of a role and of a `remove_robot` a robot of each world pattern
    /// that names it, each world pattern given, an expected refusal of an
    /// assumption of the workflow, the step of each fault a step of the
    /// workflow, each push on an object that the world pattern does not make
    /// static or give no physics, a judge for each assumption and no other, a
    /// `minimum` judge when the workflow has a `minimum` and none otherwise,
    /// and each watch parameter reading observations of the workflow.
    pub fn new(
        document: SweepDocument,
        workflow: &Workflow,
        patterns: &[WorldPattern],
    ) -> Result<Sweep, Refusals> {
        let mut refusals = binding_refusals(
            workflow,
            &document.workflow,
            &document.roles,
            "name its robot in the world patterns",
        );
        let pattern_of =
            |world: &Manifest| patterns.iter().find(|pattern| pattern.manifest == *world);
        for (index, swept) in document.worlds.iter().enumerate() {
            let Some(pattern) = pattern_of(&swept.world) else {
                refusals.push(at(
                    format!("worlds[{index}].world"),
                    format!(
                        "names `{}` revision {}, which is not among the world patterns given",
                        swept.world.name, swept.world.revision
                    ),
                ));
                continue;
            };
            let robots = pattern.joins();
            for (role, robot) in &document.roles {
                if !robots.contains_key(robot) {
                    refusals.push(at(
                        format!("roles.{role}"),
                        format!(
                            "`{robot}` is no robot of the world pattern `{}` revision {}",
                            swept.world.name, swept.world.revision
                        ),
                    ));
                }
            }
            if let Expected::Refused { assumption } = &swept.expect
                && !workflow.assumptions.contains_key(assumption)
            {
                refusals.push(at(
                    format!("worlds[{index}].expect"),
                    format!("names `{assumption}`, which is no assumption of the workflow"),
                ));
            }
        }
        for (index, fault) in document.faults.iter().enumerate() {
            let step = fault.when.step();
            if !workflow.has_step(&step.role, &step.id) {
                refusals.push(at(
                    format!("faults[{index}].when"),
                    format!("names `{step}`, which is no step of the workflow"),
                ));
            }
            let Some(pattern) = pattern_of(&fault.world) else {
                continue;
            };
            let action_at = format!("faults[{index}].do");
            match &fault.action {
                FaultAction::Push(push) => match pattern.physics_of(&push.object) {
                    None => refusals.push(at(
                        action_at,
                        format!(
                            "pushes `{}`, which is no object of the world pattern",
                            push.object
                        ),
                    )),
                    Some(physics) => {
                        let fixed = physics
                            .iter()
                            .flatten()
                            .find(|physics| **physics != Physics::Dynamic);
                        if let Some(fixed) = fixed {
                            refusals.push(at(
                                action_at,
                                format!(
                                    "pushes `{}`, which the world pattern can give the physics `{}`; a push moves a dynamic object",
                                    push.object,
                                    String::from(*fixed)
                                ),
                            ));
                        }
                    }
                },
                FaultAction::RemoveRobot(robot) => {
                    if !pattern.joins().contains_key(robot) {
                        refusals.push(at(
                            action_at,
                            format!("removes `{robot}`, which is no robot of the world pattern"),
                        ));
                    }
                }
            }
        }
        let judges = &document.judges;
        for name in workflow.assumptions.keys() {
            if !judges.assumptions.contains_key(name) {
                refusals.push(at(
                    "judges.assumptions",
                    format!(
                        "has no judge of the assumption `{name}`; give one judge per assumption"
                    ),
                ));
            }
        }
        for name in judges.assumptions.keys() {
            if !workflow.assumptions.contains_key(name) {
                refusals.push(at(
                    format!("judges.assumptions.{name}"),
                    "is no assumption of the workflow",
                ));
            }
        }
        match (&workflow.minimum, &judges.minimum) {
            (Some(_), None) => refusals.push(at(
                "judges",
                "has no `minimum` judge, and the workflow has a `minimum`; give the judge that checks each degraded ending",
            )),
            (None, Some(_)) => refusals.push(at(
                "judges.minimum",
                "judges a `minimum` that the workflow does not have; leave it out",
            )),
            _ => {}
        }
        refusals.extend(watch_refusals(
            workflow,
            "watch",
            document
                .watch
                .iter()
                .map(|(name, watch)| (name, &watch.observation)),
        ));
        Refusals::or_ok(refusals, Sweep { document })
    }

    /// The seeds of the faults of `world`, which run a fault in place of the
    /// ending that their world pattern expects.
    pub fn fault_seeds(&self, world: &Manifest) -> BTreeSet<Seed> {
        self.document
            .faults
            .iter()
            .filter(|fault| fault.world == *world)
            .flat_map(|fault| fault.seeds.iter())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::RELEASE;

    const TAPE_SWEEP: &str = r#"{
  simulation_schema: "sweep/v1",
  manifest: { name: "tape_packing_cycle_sweep", revision: 1 },
  workflow: { name: "tape_packing_cycle", revision: 3 },
  launcher: "tape_line_simulation",
  roles: { so101_a: "alpha", so101_b: "bravo" },
  worlds: [
    { world: { name: "tape_line", revision: 1 }, seeds: { from: 1, to: 500 }, expect: "completed" },
    { world: { name: "tape_line_no_boxes", revision: 1 }, seeds: { from: 1, to: 50 }, expect: { refused: "spare_boxes" } },
  ],
  faults: [
    { world: { name: "tape_line", revision: 1 }, seeds: { from: 1, to: 10 }, when: { step_starts: "so101_a.pick_loop" }, do: { remove_robot: "charlie" }, expect: "degraded" },
    { world: { name: "tape_line", revision: 1 }, seeds: { from: 11, to: 12 }, when: { step_ends: "so101_b.pick_loop" }, do: { push: { object: "stock_1", force: [5, 0, 0], duration_s: 0.5 } }, expect: "failed" },
  ],
  judges: {
    success: { count: { item_kind: "carton", in: { location: "output_area" }, at_least: 2, each_holding: { item_kind: "tape_roll", at_least: 6 } } },
    minimum: { count: { item_kind: "carton", in: { location: "output_area" }, at_least: 1 } },
    assumptions: { spare_boxes: { count: { item_kind: "carton", in: { region: "box_stock" }, at_least: 2 } } },
    unsafe: { conveyor_tipped: { not: { control: { object: "conveyor", id: "height", at_least: 0.3 } } } },
  },
  watch: { loose_tapes: { unit: "tapes", observation: "state_a.loose" } },
  thresholds: { completed_at_least: 0.95, unsafe_events_at_most: 0, false_successes_at_most: 0.01, false_assumption_checks_at_most: 0 },
}"#;

    fn workflow(minimum: bool) -> Workflow {
        let mut text = serde_json::json!({
            "workflow_schema": "workflow/v1",
            "manifest": { "name": "tape_packing_cycle", "revision": 3 },
            "task": { "objective": "Fill one box per station" },
            "contracts": { "tasks": "task_execution:v1", "packing": "packing:v1" },
            "roles": { "so101_a": {}, "so101_b": {} },
            "observations": { "state_a": { "member": "packing.station_state", "role": "so101_a" } },
            "assumptions": { "spare_boxes": { "says": "At least 2 empty boxes are in the stock", "check": "state_a.spare_boxes >= 2" } },
            "success": "state_a.done",
            "actions": {
                "pickers": {
                    "roles": ["so101_a", "so101_b"],
                    "tree": { "id": "pick_loop", "timeout_s": 600, "call": { "member": "tasks.run_task", "args": { "instruction": "Fill the box", "timeout_s": 60 } } },
                },
            },
        });
        if minimum {
            text["minimum"] = serde_json::json!("state_a.started");
        }
        Workflow::parse(&text.to_string()).unwrap()
    }

    fn pattern(name: &str, stock: &str) -> WorldPattern {
        WorldPattern::parse(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "{name}", revision: 1 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/tape_line" }},
              robots: {{ alpha: {{ join: {{ option: "so101_sim" }}, stand: {{ surface: "station_a" }} }}, bravo: {{ join: {{ option: "so101_sim" }}, stand: {{ surface: "station_b" }} }},
                         charlie: {{ join: {{ option: "so101_sim" }}, stand: {{ surface: "spare_station" }} }} }},
              objects: {{ conveyor: {{ asset: "object/conveyor", pose: {{ position: [0, 0, 0], yaw: 0 }}, physics: "static" }},
                          stock: {{ item_kind: "carton", count: {stock}, pose: {{ in: "box_stock" }} }} }},
              regions: {{ box_stock: {{ centre: [2, 1, 0.5], size: [0.8, 0.6, 1] }} }} }}"#
        ))
        .unwrap()
    }

    fn patterns() -> Vec<WorldPattern> {
        vec![
            pattern("tape_line", "{ range: [3, 6] }"),
            pattern("tape_line_no_boxes", "{ one_of: [0, 1] }"),
        ]
    }

    fn texts(refusals: Refusals) -> Vec<String> {
        refusals.iter().map(Refusal::to_string).collect()
    }

    #[test]
    fn the_tape_sweep_of_the_plan_fits_its_workflow_and_patterns() {
        let document = SweepDocument::parse(TAPE_SWEEP).unwrap();
        assert_eq!(
            document.worlds[1].expect,
            Expected::Refused {
                assumption: WorkflowName::parse("spare_boxes").unwrap()
            }
        );
        assert_eq!(
            document.faults[1].action,
            FaultAction::Push(Push {
                object: Name::parse("stock_1").unwrap(),
                force: [5.0, 0.0, 0.0],
                duration_s: 0.5
            })
        );
        let sweep = Sweep::new(document, &workflow(true), &patterns()).unwrap();
        let fault_seeds = sweep.fault_seeds(&sweep.document.worlds[0].world);
        assert_eq!(fault_seeds.len(), 12);
        let written = sweep.document.document().unwrap().to_json5();
        assert_eq!(SweepDocument::parse(&written).unwrap(), sweep.document);
    }

    #[test]
    fn the_document_alone_refuses_each_rule_at_its_place() {
        for (from, to, expected) in [
            (
                r#"launcher: "tape_line_simulation""#,
                r#"launcher: " ""#,
                "launcher: is blank; name the launcher of the stack of the sweep",
            ),
            (
                r#"so101_b: "bravo""#,
                r#"so101_b: "alpha""#,
                "roles.so101_b: `alpha` is the robot of the role `so101_a` too: name another robot for each role",
            ),
            (
                r#"{ name: "tape_line_no_boxes", revision: 1 }, seeds"#,
                r#"{ name: "tape_line", revision: 1 }, seeds"#,
                "worlds[1]: names `tape_line` revision 1 again; give each world pattern once",
            ),
            (
                "seeds: { from: 11, to: 12 }",
                "seeds: { from: 499, to: 501 }",
                "faults[1].seeds: are 499 to 501, and the world pattern has the seeds 1 to 500; give seeds of the world pattern",
            ),
            (
                "seeds: { from: 1, to: 50 }",
                "seeds: { from: 50, to: 1 }",
                "the seeds from 50 to 1 end below their start",
            ),
            (
                "duration_s: 0.5",
                "duration_s: 31",
                "the push lasts 31 s; a push lasts more than 0 s and at most 30 s",
            ),
            (
                "duration_s: 0.5",
                "duration_s: 0",
                "the push lasts 0 s; a push lasts more than 0 s and at most 30 s",
            ),
            (
                r#"{ step_starts: "so101_a.pick_loop" }"#,
                r#"{ step_starts: "pick_loop" }"#,
                "`pick_loop` is no step: write `<role>.<id>`, such as `so101_a.pick_loop`",
            ),
            (
                r#"remove_robot: "charlie""#,
                r#"remove_robot: "alpha""#,
                "faults[0].do: removes `alpha`, which the role `so101_a` binds: the director ends a run interrupted when the lease of a robot of the run ends, so no run can end as the fault expects; remove a robot that no role binds",
            ),
            (
                r#"expect: "completed""#,
                r#"expect: "done""#,
                "an expected ending is `completed`, `degraded`, `failed` or `{ refused: <assumption> }`",
            ),
        ] {
            let text = TAPE_SWEEP.replace(from, to);
            assert_ne!(text, TAPE_SWEEP, "{from}");
            let refused = SweepDocument::parse(&text).unwrap_err().to_string();
            assert!(refused.contains(expected), "{expected}\n{refused}");
        }
        let refused_world = TAPE_SWEEP.replace(
            r#"{ world: { name: "tape_line", revision: 1 }, seeds: { from: 1, to: 10 }"#,
            r#"{ world: { name: "tape_line_no_boxes", revision: 1 }, seeds: { from: 1, to: 10 }"#,
        );
        assert_eq!(
            SweepDocument::parse(&refused_world)
                .unwrap_err()
                .to_string(),
            "faults[0].world: expects the refusal of `spare_boxes`, so no run starts for a fault to stop; give a world pattern that runs"
        );
    }

    #[test]
    fn a_sweep_against_its_workflow_and_patterns_names_each_misfit() {
        let text = TAPE_SWEEP
            .replace(r#"so101_b: "bravo""#, r#"so101_b: "echo""#)
            .replace(
                r#"{ refused: "spare_boxes" }"#,
                r#"{ refused: "belt_runs" }"#,
            )
            .replace(
                r#"step_ends: "so101_b.pick_loop""#,
                r#"step_ends: "so101_b.close_box""#,
            )
            .replace(r#"object: "stock_1""#, r#"object: "conveyor""#)
            .replace(r#"remove_robot: "charlie""#, r#"remove_robot: "foxtrot""#)
            .replace("spare_boxes: { count", "belt_runs: { count");
        let document = SweepDocument::parse(&text).unwrap();
        let refused = texts(Sweep::new(document, &workflow(false), &patterns()[..1]).unwrap_err());
        assert_eq!(
            refused,
            [
                "roles.so101_b: `echo` is no robot of the world pattern `tape_line` revision 1",
                "worlds[1].world: names `tape_line_no_boxes` revision 1, which is not among the world patterns given",
                "faults[0].do: removes `foxtrot`, which is no robot of the world pattern",
                "faults[1].when: names `so101_b.close_box`, which is no step of the workflow",
                "faults[1].do: pushes `conveyor`, which the world pattern can give the physics `static`; a push moves a dynamic object",
                "judges.assumptions: has no judge of the assumption `spare_boxes`; give one judge per assumption",
                "judges.assumptions.belt_runs: is no assumption of the workflow",
                "judges.minimum: judges a `minimum` that the workflow does not have; leave it out",
            ]
        );
    }

    #[test]
    fn a_sweep_needs_a_minimum_judge_for_a_workflow_with_a_minimum() {
        let text = TAPE_SWEEP.replace(
            r#"    minimum: { count: { item_kind: "carton", in: { location: "output_area" }, at_least: 1 } },
"#,
            "",
        );
        let document = SweepDocument::parse(&text).unwrap();
        assert_eq!(
            texts(Sweep::new(document, &workflow(true), &patterns()).unwrap_err()),
            [
                "judges: has no `minimum` judge, and the workflow has a `minimum`; give the judge that checks each degraded ending"
            ]
        );
    }
}
