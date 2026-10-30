//! `run/v1`: one run of a workflow revision, with each role bound to a robot
//! or to `"auto"`.

use documents_core::{Document, Kind, Revision};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::{BTreeMap, BTreeSet};

use crate::names::{Name, NameTag, RobotName};
use crate::refusal::{At, Refusal, Refusals};
use crate::work_cell::{RoleValues, WatchParameter, WorkCell, watch_refusals};
use crate::workflow::Workflow;

/// The kind of a run document.
pub const RUN: Kind = Kind::new("workflow_schema", "run", 1);

/// The workflow revision a run runs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkflowRef {
    pub name: String,
    pub revision: Revision,
}

/// The robot a role binds to: a robot of the fleet listing by its name, or
/// the first present robot that fits.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RoleBinding {
    Robot(RobotName),
    Auto,
}

const AUTO: &str = "auto";

impl Serialize for RoleBinding {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            RoleBinding::Robot(robot) => robot.serialize(serializer),
            RoleBinding::Auto => serializer.serialize_str(AUTO),
        }
    }
}

impl<'de> Deserialize<'de> for RoleBinding {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<RoleBinding, D::Error> {
        let text = String::deserialize(deserializer)?;
        if text == AUTO {
            return Ok(RoleBinding::Auto);
        }
        RobotName::parse(&text)
            .map(RoleBinding::Robot)
            .map_err(serde::de::Error::custom)
    }
}

/// Who decides on incidents: a person, or only the fallbacks of the
/// workflow.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Attendance {
    Attended,
    Unattended,
}

/// The severity of an anomaly that the monitor reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    Low,
    Medium,
    High,
}

/// The member of an allowed contract that looks for anomalies after each
/// call step whose `done_when` holds, and the instance in the stack that
/// serves it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Monitor {
    pub contract: NameTag,
    pub member: Name,
    pub stack_instance: String,
}

/// How a run meets incidents: with no monitor, or with an anomaly monitor
/// and the severities that let an unattended run go on.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum Incidents {
    #[default]
    None,
    Monitored {
        monitor: Monitor,
        continue_on_severity: BTreeSet<Severity>,
    },
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct IncidentFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    monitor: Option<Monitor>,
    #[serde(default, skip_serializing_if = "BTreeSet::is_empty")]
    continue_on_severity: BTreeSet<Severity>,
}

impl Serialize for Incidents {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let fields = match self {
            Incidents::None => IncidentFields {
                monitor: None,
                continue_on_severity: BTreeSet::new(),
            },
            Incidents::Monitored {
                monitor,
                continue_on_severity,
            } => IncidentFields {
                monitor: Some(monitor.clone()),
                continue_on_severity: continue_on_severity.clone(),
            },
        };
        fields.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Incidents {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Incidents, D::Error> {
        let fields = IncidentFields::deserialize(deserializer)?;
        match fields.monitor {
            Some(monitor) => Ok(Incidents::Monitored {
                monitor,
                continue_on_severity: fields.continue_on_severity,
            }),
            None if fields.continue_on_severity.is_empty() => Ok(Incidents::None),
            None => Err(serde::de::Error::custom(
                "`continue_on_severity` needs a `monitor`: the severities are those of its anomalies",
            )),
        }
    }
}

impl Incidents {
    pub fn is_none(&self) -> bool {
        *self == Incidents::None
    }
}

/// What a run does on a difference from the lock of its bundle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VersionPolicy {
    #[default]
    Refuse,
    Warn,
}

/// The fields of a run document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunDocument {
    pub workflow: WorkflowRef,
    pub roles: BTreeMap<Name, RoleBinding>,
    pub attendance: Attendance,
    /// The assumptions without a check that the caller confirms.
    #[serde(default, skip_serializing_if = "BTreeSet::is_empty")]
    pub confirmed: BTreeSet<Name>,
    #[serde(default, skip_serializing_if = "Incidents::is_none")]
    pub incidents: Incidents,
    #[serde(default)]
    pub versions: VersionPolicy,
    /// The measured value of each control and distance parameter of the
    /// work cell, per role for a parameter with one value per role.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub work_cell: WorkCell,
    /// The observations that the director reads through the robots before
    /// the initialization, with those of the bundle.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub watch: BTreeMap<Name, WatchParameter>,
}

impl RunDocument {
    pub fn parse(text: &str) -> Result<RunDocument, Refusals> {
        Ok(Document::parse(text, RUN)?.read()?)
    }

    pub fn document(&self) -> Document {
        Document::of(RUN, self).expect("a run document serializes to a document")
    }

    /// The attended run of `workflow` that binds the roles given, leaves
    /// every other role to `"auto"` and gives the work cell block given.
    pub fn of_roles(
        workflow: &Workflow,
        roles: BTreeMap<Name, RobotName>,
        work_cell: WorkCell,
    ) -> Result<RunDocument, Refusals> {
        let unknown: Vec<Refusal> = roles
            .keys()
            .filter(|role| !workflow.roles.contains_key(*role))
            .map(|role| {
                Refusal::new(
                    At::field(format!("roles.{role}")),
                    "is no role of the workflow",
                )
            })
            .collect();
        Refusals::or_ok(unknown, ())?;
        let roles = workflow
            .roles
            .keys()
            .map(|role| {
                let binding = roles
                    .get(role)
                    .cloned()
                    .map_or(RoleBinding::Auto, RoleBinding::Robot);
                (role.clone(), binding)
            })
            .collect();
        Ok(RunDocument {
            workflow: WorkflowRef {
                name: workflow.manifest.name.clone(),
                revision: workflow.manifest.revision,
            },
            roles,
            attendance: Attendance::Attended,
            confirmed: BTreeSet::new(),
            incidents: Incidents::None,
            versions: VersionPolicy::Refuse,
            work_cell,
            watch: BTreeMap::new(),
        })
    }
}

/// A run document that fits its workflow: the same name and revision, every
/// role bound and no other, no robot named for two roles, `confirmed`
/// naming only assumptions without a check, a value per role of the work
/// cell only for roles of the workflow, and each watch parameter reading
/// observations of the workflow.
#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    pub document: RunDocument,
}

impl Run {
    pub fn new(workflow: &Workflow, document: RunDocument) -> Result<Run, Refusals> {
        let mut refusals = binding_refusals(
            workflow,
            &document.workflow,
            &document.roles,
            "name one, or `\"auto\"`",
        );
        refusals.extend(robots_named_twice(document.roles.iter().filter_map(
            |(role, binding)| match binding {
                RoleBinding::Robot(robot) => Some((role, robot)),
                RoleBinding::Auto => None,
            },
        )));
        for name in &document.confirmed {
            let fits = workflow
                .assumptions
                .get(name)
                .is_some_and(|assumption| assumption.check.is_none());
            if !fits {
                refusals.push(Refusal::new(
                    At::field("confirmed"),
                    format!("names `{name}`, which is no assumption without a `check`"),
                ));
            }
        }
        refusals.extend(work_cell_role_refusals(
            workflow,
            "work_cell",
            &document.work_cell,
        ));
        refusals.extend(watch_refusals(
            workflow,
            "watch",
            document
                .watch
                .iter()
                .map(|(name, watch)| (name, &watch.observation)),
        ));
        Refusals::or_ok(refusals, Run { document })
    }
}

/// The refusals of a document that names the revision `named` of `workflow`
/// and binds `roles`: another name or revision, a role of the workflow that
/// `roles` leaves out (with `hint` to bind it), and a role that is not of
/// the workflow.
pub fn binding_refusals<T>(
    workflow: &Workflow,
    named: &WorkflowRef,
    roles: &BTreeMap<Name, T>,
    hint: &str,
) -> Vec<Refusal> {
    let mut refusals = Vec::new();
    if named.name != workflow.manifest.name || named.revision != workflow.manifest.revision {
        refusals.push(Refusal::new(
            At::field("workflow"),
            format!(
                "names `{}` revision {}, and the workflow is `{}` revision {}",
                named.name, named.revision, workflow.manifest.name, workflow.manifest.revision
            ),
        ));
    }
    for role in workflow.roles.keys() {
        if !roles.contains_key(role) {
            refusals.push(Refusal::new(
                At::field("roles"),
                format!("binds no robot to the role `{role}`; {hint}"),
            ));
        }
    }
    for role in roles.keys() {
        if !workflow.roles.contains_key(role) {
            refusals.push(Refusal::new(
                At::field(format!("roles.{role}")),
                "is no role of the workflow",
            ));
        }
    }
    refusals
}

/// The refusal of each value per role of `work_cell` whose role is no role
/// of `workflow`, at `field.<parameter>.<role>`.
pub(crate) fn work_cell_role_refusals(
    workflow: &Workflow,
    field: &str,
    work_cell: &WorkCell,
) -> Vec<Refusal> {
    work_cell
        .iter()
        .filter_map(|(name, values)| match values {
            RoleValues::PerRole(per_role) => Some((name, per_role)),
            RoleValues::Single(_) => None,
        })
        .flat_map(|(name, per_role)| {
            per_role
                .keys()
                .filter(|role| !workflow.roles.contains_key(*role))
                .map(move |role| {
                    Refusal::new(
                        At::field(format!("{field}.{name}.{role}")),
                        "is no role of the workflow",
                    )
                })
        })
        .collect()
}

/// The refusal of each role that names a robot that a role before it, in
/// the order of `roles`, names too: one robot takes one role. A run and the
/// check of a binding refuse each.
pub fn robots_named_twice<'a>(
    roles: impl IntoIterator<Item = (&'a Name, &'a RobotName)>,
) -> Vec<Refusal> {
    let mut bound: BTreeMap<&RobotName, &Name> = BTreeMap::new();
    roles
        .into_iter()
        .filter_map(|(role, robot)| match bound.get(robot) {
            Some(earlier) => Some(Refusal::new(
                At::field(format!("roles.{role}")),
                format!("`{robot}` is the robot of the role `{earlier}` too: name another robot for each role"),
            )),
            None => {
                bound.insert(robot, role);
                None
            }
        })
        .collect()
}

/// The start of the name of each lease that the workflow director takes for
/// a run.
const RUN_LEASE: &str = "run ";

/// The name of the leases that the workflow director takes for the run
/// `run`: "run r7" for the run r7.
pub fn run_lease_name(run: &str) -> String {
    format!("{RUN_LEASE}{run}")
}

/// The run whose leases bear the name `name`, as [`run_lease_name`] writes
/// it: an id `r<N>`; none for the name of any other lease.
pub fn run_of_lease_name(name: &str) -> Option<&str> {
    name.strip_prefix(RUN_LEASE).filter(|run| {
        run.strip_prefix('r')
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn workflow() -> Workflow {
        let text = json!({
            "workflow_schema": "workflow/v1",
            "manifest": { "name": "home", "revision": 1 },
            "task": { "objective": "Go home" },
            "contracts": { "postures": "postures:v1" },
            "roles": { "arm": {}, "helper": {} },
            "assumptions": {
                "clean": { "says": "The pan is clean" },
                "near": { "says": "The table is near", "check": "true" },
            },
            "success": "true",
            "actions": {
                "arm": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } },
                "helper": { "tree": { "id": "home", "timeout_s": 30, "call": { "member": "postures.move_to_home" } } },
            },
        });
        Workflow::parse(&text.to_string()).unwrap()
    }

    #[test]
    fn each_role_that_names_a_robot_of_an_earlier_role_is_refused() {
        let pair = |role: &str, robot: &str| {
            (Name::parse(role).unwrap(), RobotName::parse(robot).unwrap())
        };
        let roles = BTreeMap::from([
            pair("left", "alpha"),
            pair("right", "alpha"),
            pair("spare", "alpha"),
            pair("tool", "beta"),
        ]);
        let refused: Vec<String> = robots_named_twice(&roles)
            .iter()
            .map(Refusal::to_string)
            .collect();
        assert_eq!(
            refused,
            [
                "roles.right: `alpha` is the robot of the role `left` too: name another robot for each role",
                "roles.spare: `alpha` is the robot of the role `left` too: name another robot for each role",
            ]
        );
        let apart = BTreeMap::from([pair("left", "alpha"), pair("right", "beta")]);
        assert!(robots_named_twice(&apart).is_empty());
    }

    #[test]
    fn the_lease_name_of_a_run_reads_back_as_its_run() {
        for run in ["r1", "r7", "r1024"] {
            assert_eq!(run_of_lease_name(&run_lease_name(run)), Some(run));
        }
        assert_eq!(run_lease_name("r7"), "run r7");
        for other in [
            "run ", "run r", "run 7", "run r7 ", "run r7a", "run  r7", "lease r7", "r7", "",
        ] {
            assert_eq!(run_of_lease_name(other), None, "{other:?}");
        }
    }

    #[test]
    fn roles_given_bind_and_the_others_take_auto() {
        let workflow = workflow();
        let roles = BTreeMap::from([(
            Name::parse("arm").unwrap(),
            RobotName::parse("alpha").unwrap(),
        )]);
        let document = RunDocument::of_roles(&workflow, roles, WorkCell::new()).unwrap();
        assert_eq!(
            document.roles[&Name::parse("helper").unwrap()],
            RoleBinding::Auto
        );
        assert_eq!(document.attendance, Attendance::Attended);
        let text = document.document().to_json5();
        assert!(text.contains("helper: \"auto\""), "{text}");
        assert!(Run::new(&workflow, RunDocument::parse(&text).unwrap()).is_ok());
    }

    #[test]
    fn a_run_that_does_not_fit_its_workflow_is_refused_by_each_rule() {
        let workflow = workflow();
        let text = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 2 },
            roles: { arm: "alpha", visitor: "beta" }, attendance: "attended", confirmed: ["near"] }"#;
        let refused = Run::new(&workflow, RunDocument::parse(text).unwrap())
            .unwrap_err()
            .to_string();
        for part in ["revision 2", "the role `helper`", "roles.visitor", "`near`"] {
            assert!(refused.contains(part), "{part}: {refused}");
        }
        let twice = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "alpha", helper: "alpha" }, attendance: "attended" }"#;
        let refused = Run::new(&workflow, RunDocument::parse(twice).unwrap())
            .unwrap_err()
            .to_string();
        assert!(
            refused.contains("roles.helper: `alpha` is the robot of the role `arm` too: name another robot for each role"),
            "{refused}"
        );
        let both_auto = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "auto", helper: "auto" }, attendance: "attended" }"#;
        assert!(Run::new(&workflow, RunDocument::parse(both_auto).unwrap()).is_ok());
        let monitorless = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "auto", helper: "auto" }, attendance: "unattended", incidents: { continue_on_severity: ["low"] } }"#;
        assert!(
            RunDocument::parse(monitorless)
                .unwrap_err()
                .to_string()
                .contains("needs a `monitor`")
        );
    }

    #[test]
    fn a_run_gives_its_work_cell_and_watch_and_writes_them_back() {
        let workflow = workflow();
        let text = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "alpha", helper: "beta" }, attendance: "unattended",
            work_cell: { belt_speed_mps: 0.2, belt_top_above_base_m: { arm: 0.18, helper: 0.21 } } }"#;
        let document = RunDocument::parse(text).unwrap();
        let top = &document.work_cell[&Name::parse("belt_top_above_base_m").unwrap()];
        assert_eq!(
            *top,
            RoleValues::PerRole(BTreeMap::from([
                (Name::parse("arm").unwrap(), 0.18),
                (Name::parse("helper").unwrap(), 0.21),
            ]))
        );
        let written = document.document().to_json5();
        assert_eq!(RunDocument::parse(&written).unwrap(), document);
        assert!(Run::new(&workflow, document).is_ok());
        let plain = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "alpha", helper: "beta" }, attendance: "attended" }"#;
        let written = RunDocument::parse(plain).unwrap().document().to_json5();
        assert!(
            !written.contains("work_cell") && !written.contains("watch"),
            "{written}"
        );
    }

    #[test]
    fn a_work_cell_value_of_another_role_or_a_watch_of_another_observation_is_refused() {
        let workflow = workflow();
        let text = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "alpha", helper: "beta" }, attendance: "attended",
            work_cell: { belt_top_above_base_m: { arm: 0.18, courier: 0.2 } },
            watch: { loose_items_at_start: { unit: "items", observation: "loose.count" } } }"#;
        let refused: Vec<String> = Run::new(&workflow, RunDocument::parse(text).unwrap())
            .unwrap_err()
            .iter()
            .map(Refusal::to_string)
            .collect();
        assert_eq!(
            refused,
            [
                "work_cell.belt_top_above_base_m.courier: is no role of the workflow",
                "watch.loose_items_at_start: reads `loose`, which is no observation of the workflow",
            ]
        );
        let blank_unit = r#"{ workflow_schema: "run/v1", workflow: { name: "home", revision: 1 },
            roles: { arm: "alpha", helper: "beta" }, attendance: "attended",
            watch: { loose: { unit: "", observation: "loose.count" } } }"#;
        assert!(
            RunDocument::parse(blank_unit)
                .unwrap_err()
                .to_string()
                .contains("the unit is blank"),
        );
    }
}
