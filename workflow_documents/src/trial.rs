//! `trial/v1`: the runs of one bundle on one physical stack, which a person
//! prepares and judges, and which sign off the bundle on that stack when they
//! meet the thresholds of the bundle.

use documents_core::{Document, Kind, Manifest};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

use crate::bundle::Bundle;
use crate::names::{Name, RobotName};
use crate::refusal::{At, Refusal, Refusals};
use crate::run::{
    Attendance, Incidents, RoleBinding, RunDocument, VersionPolicy, WorkflowRef, binding_refusals,
    robots_named_twice, work_cell_role_refusals,
};
use crate::work_cell::{WorkCell, uncovered_work_cell};

/// The kind of a trial document.
pub const TRIAL: Kind = Kind::new("workflow_schema", "trial", 1);

/// The word of `run/v1` that binds a role to the first robot that fits.
const AUTO: &str = "auto";

/// The fields of a trial document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrialDocument {
    pub manifest: Manifest,
    /// The workflow of the bundle.
    pub workflow: WorkflowRef,
    /// The robot of each role: a trial signs off one binding.
    pub roles: BTreeMap<Name, RobotName>,
    /// The values that the operator measured in the work cell.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub work_cell: WorkCell,
    #[serde(default, skip_serializing_if = "Incidents::is_none")]
    pub incidents: Incidents,
    /// The number of runs that count.
    pub runs: u32,
}

impl TrialDocument {
    /// Parses JSON5 text as a trial document. Refuses `"auto"` as the robot
    /// of a role and a trial of no run.
    pub fn parse(text: &str) -> Result<TrialDocument, Refusals> {
        let document: TrialDocument = Document::parse(text, TRIAL)?.read()?;
        let mut refusals: Vec<Refusal> = document
            .roles
            .iter()
            .filter(|(_, robot)| robot.as_str() == AUTO)
            .map(|(role, _)| {
                Refusal::new(
                    At::field(format!("roles.{role}")),
                    "is `\"auto\"`; a trial signs off one binding, so name the robot of each role",
                )
            })
            .collect();
        if document.runs == 0 {
            refusals.push(Refusal::new(
                At::field("runs"),
                "is 0; a trial counts at least one run",
            ));
        }
        Refusals::or_ok(refusals, document)
    }

    pub fn document(&self) -> Document {
        Document::of(TRIAL, self).expect("a trial document serializes to a document")
    }
}

/// A trial document that fits its bundle: the workflow of the bundle, a
/// robot for each role and no other, no robot named for two roles, a bundle
/// with an envelope, each value of the work cell inside it, and a `check`
/// for each assumption of the workflow, which the operator judges.
#[derive(Clone, Debug, PartialEq)]
pub struct Trial {
    pub document: TrialDocument,
}

impl Trial {
    pub fn new(bundle: &Bundle, document: TrialDocument) -> Result<Trial, Refusals> {
        let workflow = &bundle.workflow;
        let mut refusals = binding_refusals(
            workflow,
            &document.workflow,
            &document.roles,
            "name one: a trial binds a robot to each role",
        );
        refusals.extend(robots_named_twice(&document.roles));
        refusals.extend(work_cell_role_refusals(
            workflow,
            "work_cell",
            &document.work_cell,
        ));
        match &bundle.document.validation {
            None => refusals.push(Refusal::new(
                At::Document,
                "the bundle has no envelope; a trial signs off only the bundle of a signed-off sweep",
            )),
            Some(validation) => {
                refusals.extend(validation.work_cell_refusals("work_cell", &document.work_cell));
                let uncovered = uncovered_work_cell(Some(validation), &document.work_cell);
                refusals.extend(uncovered.into_iter().map(|uncovered| {
                    Refusal::new(
                        At::field("work_cell"),
                        format!("{uncovered}; a trial measures each value inside the envelope"),
                    )
                }));
            }
        }
        refusals.extend(
            workflow
                .assumptions
                .iter()
                .filter(|(_, assumption)| assumption.check.is_none())
                .map(|(name, _)| {
                    Refusal::new(
                        At::Document,
                        format!(
                            "the assumption `{name}` of the workflow has no `check`; a trial needs a check of each assumption, for the operator to judge"
                        ),
                    )
                }),
        );
        Refusals::or_ok(refusals, Trial { document })
    }

    /// The run document of each run of the trial: unattended, with the
    /// roles, the work cell and the incidents of the trial, and refused on
    /// any difference from the lock of the bundle.
    pub fn run_document(&self) -> RunDocument {
        let document = &self.document;
        RunDocument {
            workflow: document.workflow.clone(),
            roles: document
                .roles
                .iter()
                .map(|(role, robot)| (role.clone(), RoleBinding::Robot(robot.clone())))
                .collect(),
            attendance: Attendance::Unattended,
            confirmed: BTreeSet::new(),
            incidents: document.incidents.clone(),
            versions: VersionPolicy::Refuse,
            work_cell: document.work_cell.clone(),
            watch: BTreeMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::BundleFile;
    use crate::lock::Lock;
    use crate::run::Run;
    use crate::sign_off::tests::thresholds;
    use crate::work_cell::Validation;
    use crate::workflow::Workflow;

    fn tape_workflow(assumptions: serde_json::Value) -> Workflow {
        let text = serde_json::json!({
            "workflow_schema": "workflow/v1",
            "manifest": { "name": "tape_packing_cycle", "revision": 3 },
            "task": { "objective": "Fill one box" },
            "contracts": { "postures": "postures:v1" },
            "roles": { "so101_a": {}, "so101_b": {} },
            "assumptions": assumptions,
            "success": "true",
            "actions": {
                "so101_a": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } },
                "so101_b": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } },
            },
        });
        Workflow::parse(&text.to_string()).unwrap()
    }

    fn read<T: serde::de::DeserializeOwned>(text: &str) -> T {
        serde_json::from_value(documents_core::parse(text).unwrap()).unwrap()
    }

    fn validation() -> Validation {
        Validation::new(
            thresholds(),
            read(
                r#"{ belt_speed_mps: { unit: "m/s", kind: "control" },
                     belt_top_above_base_m: { unit: "m", kind: "distance", per_role: ["so101_a", "so101_b"] } }"#,
            ),
            read(
                r#"{ belt_speed_mps: [0.05, 0.25],
                     belt_top_above_base_m: { so101_a: [0.10, 0.22], so101_b: [0.10, 0.22] } }"#,
            ),
        )
        .unwrap()
    }

    fn bundle(workflow: &Workflow, validation: Option<Validation>) -> Bundle {
        let files = match validation {
            Some(validation) => {
                Bundle::write_signed_off(workflow, Vec::new(), Lock::default(), validation)
            }
            None => Bundle::write(workflow, Vec::<BundleFile>::new(), Lock::default()),
        };
        Bundle::open(files.unwrap()).unwrap()
    }

    const LINE: &str = r#"{
      workflow_schema: "trial/v1",
      manifest: { name: "line_coordinator", revision: 1 },
      workflow: { name: "tape_packing_cycle", revision: 3 },
      roles: { so101_a: "arm_left", so101_b: "arm_right" },
      work_cell: { belt_speed_mps: 0.20, belt_top_above_base_m: { so101_a: 0.18, so101_b: 0.21 } },
      runs: 20,
    }"#;

    fn texts(refusals: Refusals) -> Vec<String> {
        refusals.iter().map(Refusal::to_string).collect()
    }

    #[test]
    fn a_trial_inside_the_envelope_of_its_bundle_runs_unattended_runs_of_its_binding() {
        let workflow = tape_workflow(
            serde_json::json!({ "belt_runs": { "says": "The belt runs", "check": "true" } }),
        );
        let bundle = bundle(&workflow, Some(validation()));
        let trial = Trial::new(&bundle, TrialDocument::parse(LINE).unwrap()).unwrap();
        let run = trial.run_document();
        assert_eq!(run.attendance, Attendance::Unattended);
        assert_eq!(run.versions, VersionPolicy::Refuse);
        assert_eq!(run.work_cell, trial.document.work_cell);
        assert!(Run::new(&workflow, run).is_ok());
        let written = trial.document.document().to_json5();
        assert_eq!(TrialDocument::parse(&written).unwrap(), trial.document);
    }

    #[test]
    fn a_trial_names_a_robot_for_each_role_and_counts_at_least_one_run() {
        let text = LINE
            .replace("\"arm_right\"", "\"auto\"")
            .replace("runs: 20", "runs: 0");
        assert_eq!(
            texts(TrialDocument::parse(&text).unwrap_err()),
            [
                "roles.so101_b: is `\"auto\"`; a trial signs off one binding, so name the robot of each role",
                "runs: is 0; a trial counts at least one run",
            ]
        );
    }

    #[test]
    fn a_trial_of_a_bundle_without_an_envelope_or_with_a_value_outside_it_is_refused() {
        let workflow =
            tape_workflow(serde_json::json!({ "clean": { "says": "The pan is clean" } }));
        let without = bundle(&workflow, None);
        assert_eq!(
            texts(Trial::new(&without, TrialDocument::parse(LINE).unwrap()).unwrap_err()),
            [
                "the bundle has no envelope; a trial signs off only the bundle of a signed-off sweep",
                "the assumption `clean` of the workflow has no `check`; a trial needs a check of each assumption, for the operator to judge",
            ]
        );
        let with = bundle(&tape_workflow(serde_json::json!({})), Some(validation()));
        let outside = LINE
            .replace("so101_a: 0.18", "so101_a: 0.30")
            .replace("belt_speed_mps: 0.20, ", "");
        assert_eq!(
            texts(Trial::new(&with, TrialDocument::parse(&outside).unwrap()).unwrap_err()),
            [
                "work_cell: `belt_top_above_base_m` of the role `so101_a` is 0.3, outside the tested range [0.1, 0.22]; a trial measures each value inside the envelope",
                "work_cell: `belt_speed_mps` has no value in the work cell block; a trial measures each value inside the envelope",
            ]
        );
    }

    #[test]
    fn a_trial_of_another_workflow_or_binding_is_refused_by_each_rule() {
        let bundle = bundle(&tape_workflow(serde_json::json!({})), Some(validation()));
        let text = LINE
            .replace("revision: 3", "revision: 2")
            .replace(
                "so101_b: \"arm_right\"",
                "so101_b: \"arm_left\", courier: \"cart\"",
            )
            .replace("belt_speed_mps: 0.20", "belt_speed_mps: { so101_a: 0.2 }");
        assert_eq!(
            texts(Trial::new(&bundle, TrialDocument::parse(&text).unwrap()).unwrap_err()),
            [
                "workflow: names `tape_packing_cycle` revision 2, and the workflow is `tape_packing_cycle` revision 3",
                "roles.courier: is no role of the workflow",
                "roles.so101_b: `arm_left` is the robot of the role `so101_a` too: name another robot for each role",
                "work_cell.belt_speed_mps: has one value in the bundle; give one value, not a value per role",
            ]
        );
    }
}
