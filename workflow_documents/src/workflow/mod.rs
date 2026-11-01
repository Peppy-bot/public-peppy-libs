//! `workflow/v1`: the program of the robots. A workflow names roles and the
//! contracts of their skills, never a robot, a world or an object of one; its
//! steps form one lane per role, which a run binds to a robot.

mod object;
mod rules;
mod step;
#[cfg(test)]
mod tests;

use documents_core::{Digest, Document, Manifest, Revision};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

use crate::expression::Expression;
use crate::names::{Bound, InstanceName, MemberRef, Name, NameTag, StepId};
use crate::refusal::{At, Refusal, Refusals};
use object::{Fields, read_value};
use step::StepReader;

pub use step::{Arg, Call, Param, Policy, SafeCall, Step, StepKind};

/// The kind of a workflow document.
const WORKFLOW: documents_core::Kind = documents_core::Kind::new("workflow_schema", "workflow", 1);

/// The task of a workflow, in prose.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Task {
    pub objective: String,
}

/// A role: a name that a run binds to one robot, with the model id that
/// `robot_identity:v1` reports when the role needs one model.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Role {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
}

/// A named read on the robot of one role.
#[derive(Clone, Debug, PartialEq)]
pub struct Observation {
    pub role: Name,
    pub instance: Option<InstanceName>,
    pub source: ObservationSource,
}

/// What an observation reads: a service called on demand, or the latest
/// message of a topic, which fails when older than its bound.
#[derive(Clone, Debug, PartialEq)]
pub enum ObservationSource {
    Member {
        member: MemberRef,
        args: BTreeMap<String, Arg>,
    },
    Topic {
        topic: MemberRef,
        max_age: Bound,
    },
}

impl Observation {
    /// The member or topic the observation names.
    pub fn named(&self) -> &MemberRef {
        match &self.source {
            ObservationSource::Member { member, .. } => member,
            ObservationSource::Topic { topic, .. } => topic,
        }
    }
}

/// A fact a run needs at its start.
#[derive(Clone, Debug, PartialEq)]
pub struct Assumption {
    pub says: String,
    /// An expression over observations; without it, a person confirms.
    pub check: Option<Expression>,
    /// `kind: "reach"`: the role reaches its work zone.
    pub reach: bool,
}

/// Where a lane comes from: the lane of its role, or a lane group.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LaneOrigin {
    Role,
    Group(Name),
}

/// The tree of steps of one role, with the calls that bring its robot to a
/// safe state when the lane suspends, fails or is aborted.
#[derive(Clone, Debug, PartialEq)]
pub struct Lane {
    pub tree: Step,
    pub safe_state: Vec<SafeCall>,
    pub origin: LaneOrigin,
}

/// A table agreed with the user: one row, a literal, per value of the enum
/// that keys it.
pub type Table = BTreeMap<String, Value>;

/// A workflow that parsed and holds every rule that needs no contract. The
/// workflow director checks the rest against the contracts it names.
#[derive(Clone, Debug, PartialEq)]
pub struct Workflow {
    pub manifest: Manifest,
    pub task: Task,
    pub contracts: BTreeMap<Name, NameTag>,
    pub roles: BTreeMap<Name, Role>,
    /// The locations the workflow names, which the skills of each role know
    /// through locations:v1.
    pub locations: BTreeSet<Name>,
    pub tables: BTreeMap<Name, Table>,
    pub observations: BTreeMap<Name, Observation>,
    pub assumptions: BTreeMap<Name, Assumption>,
    pub success: Expression,
    pub minimum: Option<Expression>,
    /// The preparation tree of each role that has one.
    pub initialization: BTreeMap<Name, Step>,
    /// The lane of each role, lane groups expanded.
    pub lanes: BTreeMap<Name, Lane>,
    document: Document,
}

/// The top-level fields read with serde; steps and lanes are read by hand.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TopFields {
    manifest: Manifest,
    task: Task,
    #[serde(default)]
    contracts: BTreeMap<Name, NameTag>,
    #[serde(default)]
    roles: BTreeMap<Name, Role>,
    #[serde(default)]
    locations: Vec<Name>,
    #[serde(default)]
    tables: BTreeMap<Name, Table>,
    #[serde(default)]
    observations: BTreeMap<Name, Value>,
    #[serde(default)]
    assumptions: BTreeMap<Name, AssumptionFields>,
    success: Expression,
    minimum: Option<Expression>,
    #[serde(default)]
    initialization: BTreeMap<Name, Value>,
    actions: BTreeMap<Name, Value>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct AssumptionFields {
    says: String,
    check: Option<Expression>,
    kind: Option<AssumptionKind>,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum AssumptionKind {
    Reach,
}

impl Workflow {
    /// Parses a workflow and applies every rule that needs no contract; each
    /// refusal names its step as `<role>.<id>` or its field by its path.
    pub fn parse(text: &str) -> Result<Workflow, Refusals> {
        let document = Document::parse(text, WORKFLOW)?;
        Workflow::of_document(document)
    }

    fn of_document(document: Document) -> Result<Workflow, Refusals> {
        let top = document.read::<TopFields>()?;
        let mut refusals = Vec::new();
        if top.roles.is_empty() {
            refusals.push(Refusal::new(At::field("roles"), "names at least one role"));
        }
        let mut locations = BTreeSet::new();
        for location in &top.locations {
            if !locations.insert(location.clone()) {
                refusals.push(Refusal::new(
                    At::field("locations"),
                    format!("names `{location}` twice"),
                ));
            }
        }
        let observations: BTreeMap<Name, Observation> = top
            .observations
            .iter()
            .filter_map(|(name, value)| {
                observation(name, value, &mut refusals)
                    .map(|observation| (name.clone(), observation))
            })
            .collect();
        // A row of a table is named by a value of the enum that keys it, and `$` starts a reference.
        for (table, rows) in &top.tables {
            for key in rows.keys().filter(|key| key.starts_with('$')) {
                refusals.push(Refusal::new(
                    At::field(format!("tables.{table}.{key}")),
                    "names a row with a value of the enum that keys the table; a value never starts with `$`",
                ));
            }
        }
        let assumptions = top
            .assumptions
            .into_iter()
            .map(|(name, fields)| {
                let assumption = Assumption {
                    says: fields.says,
                    check: fields.check,
                    reach: matches!(fields.kind, Some(AssumptionKind::Reach)),
                };
                (name, assumption)
            })
            .collect();
        let initialization = top
            .initialization
            .iter()
            .filter_map(|(role, value)| {
                let mut reader = StepReader {
                    role,
                    params: None,
                    refusals: &mut refusals,
                };
                reader
                    .step(value, &format!("initialization.{role}"))
                    .map(|step| (role.clone(), step))
            })
            .collect();
        let lanes = lanes(&top.actions, &observations, &mut refusals);
        let workflow = Workflow {
            manifest: top.manifest,
            task: top.task,
            contracts: top.contracts,
            roles: top.roles,
            locations,
            tables: top.tables,
            observations,
            assumptions,
            success: top.success,
            minimum: top.minimum,
            initialization,
            lanes,
            document,
        };
        if !refusals.is_empty() {
            return Err(refused(refusals));
        }
        Refusals::or_ok(sorted(rules::check(&workflow)), workflow)
    }

    pub fn document(&self) -> &Document {
        &self.document
    }

    /// Whether the role `role` has a step `id`, in its initialization or its
    /// lane.
    pub fn has_step(&self, role: &Name, id: &StepId) -> bool {
        let mut found = false;
        let trees = self
            .initialization
            .get(role)
            .into_iter()
            .chain(self.lanes.get(role).map(|lane| &lane.tree));
        for tree in trees {
            tree.visit(&mut |step| found |= step.id == *id);
        }
        found
    }

    pub fn digest(&self) -> Digest {
        self.document.digest()
    }

    pub fn to_json5(&self) -> String {
        self.document.to_json5()
    }
}

/// A workflow as a robot session writes it: its manifest has a name and no
/// revision, which the chat gives.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkflowDraft {
    name: String,
    tree: Value,
}

impl WorkflowDraft {
    pub fn parse(text: &str) -> Result<WorkflowDraft, Refusals> {
        let tree = documents_core::parse(text)?;
        let manifest = tree
            .get("manifest")
            .and_then(Value::as_object)
            .ok_or_else(|| {
                Refusals::one(Refusal::new(
                    At::field("manifest"),
                    "is an object with the name of the workflow",
                ))
            })?;
        if manifest.contains_key("revision") {
            return Err(Refusals::one(Refusal::new(
                At::field("manifest.revision"),
                "is given by the chat; leave it out",
            )));
        }
        let name = manifest
            .get("name")
            .and_then(Value::as_str)
            .ok_or_else(|| {
                Refusals::one(Refusal::new(
                    At::field("manifest.name"),
                    "names the workflow",
                ))
            })?
            .to_owned();
        Ok(WorkflowDraft { name, tree })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// The workflow of this draft at `revision`.
    pub fn revised(&self, revision: Revision) -> Result<Workflow, Refusals> {
        let mut tree = self.tree.clone();
        tree["manifest"]["revision"] = Value::from(revision.get());
        let text = serde_json::to_string(&tree).expect("a value tree serializes to JSON");
        Workflow::parse(&text)
    }
}

fn observation(name: &Name, value: &Value, refusals: &mut Vec<Refusal>) -> Option<Observation> {
    let path = format!("observations.{name}");
    let mut refuse = |refusal: Refusal| {
        refusals.push(refusal);
        None
    };
    let fields = match Fields::of(value, &path) {
        Ok(fields) => fields,
        Err(refusal) => return refuse(refusal),
    };
    let role = match fields.read_required::<Name>("role") {
        Ok(role) => role,
        Err(refusal) => return refuse(refusal),
    };
    let instance = match fields.read::<InstanceName>("instance") {
        Ok(instance) => instance,
        Err(refusal) => return refuse(refusal),
    };
    let at = At::field(&path);
    let source = match (fields.get("member"), fields.get("topic")) {
        (Some(member), None) => {
            let member = match read_value::<MemberRef>(member, &fields.at("member")) {
                Ok(member) => member,
                Err(refusal) => return refuse(refusal),
            };
            let anonymous = Name::parse("observation").expect("a name");
            let mut reader = StepReader {
                role: &anonymous,
                params: None,
                refusals,
            };
            let args = reader.args(fields.get("args"), &fields.at("args"), &at)?;
            ObservationSource::Member { member, args }
        }
        (None, Some(topic)) => {
            let topic = match read_value::<MemberRef>(topic, &fields.at("topic")) {
                Ok(topic) => topic,
                Err(refusal) => return refuse(refusal),
            };
            let max_age = match fields.read_required::<Bound>("max_age_s") {
                Ok(max_age) => max_age,
                Err(refusal) => return refuse(refusal),
            };
            ObservationSource::Topic { topic, max_age }
        }
        _ => {
            return refuse(Refusal::new(
                at,
                "an observation names one `member` (a service) or one `topic`",
            ));
        }
    };
    let unknown = fields.unknown();
    if !unknown.is_empty() {
        refusals.extend(unknown);
        return None;
    }
    Some(Observation {
        role,
        instance,
        source,
    })
}

/// A role that a lane of `actions` gives a lane, with the parameters of its
/// lane group and where the lane comes from.
type LaneOfRole = (Name, Option<BTreeMap<Name, Param>>, LaneOrigin);

/// The lane of each role: each lane of `actions` keyed by its role, and one
/// lane per role of each lane group, its parameters put in.
fn lanes(
    actions: &BTreeMap<Name, Value>,
    observations: &BTreeMap<Name, Observation>,
    refusals: &mut Vec<Refusal>,
) -> BTreeMap<Name, Lane> {
    let mut lanes = BTreeMap::new();
    for (key, value) in actions {
        let path = format!("actions.{key}");
        let fields = match Fields::of(value, &path) {
            Ok(fields) => fields,
            Err(refusal) => {
                refusals.push(refusal);
                continue;
            }
        };
        let tree = match fields.require("tree") {
            Ok(tree) => tree,
            Err(refusal) => {
                refusals.push(refusal);
                continue;
            }
        };
        let safe_state = fields.get("safe_state");
        let members: Vec<LaneOfRole> = match fields.get("roles") {
            None => vec![(key.clone(), None, LaneOrigin::Role)],
            Some(roles) => {
                match group_roles(roles, fields.get("params"), observations, &fields, refusals) {
                    Some(members) => members
                        .into_iter()
                        .map(|(role, params)| (role, Some(params), LaneOrigin::Group(key.clone())))
                        .collect(),
                    None => continue,
                }
            }
        };
        refusals.extend(fields.unknown());
        for (role, params, origin) in members {
            let mut reader = StepReader {
                role: &role,
                params: params.as_ref(),
                refusals,
            };
            let lane_tree = reader.step(tree, &fields.at("tree"));
            let lane_safe_state = safe_calls(&mut reader, safe_state, &fields.at("safe_state"));
            let (Some(tree), Some(safe_state)) = (lane_tree, lane_safe_state) else {
                continue;
            };
            if lanes.contains_key(&role) {
                refusals.push(Refusal::new(
                    At::field("actions"),
                    format!("the role `{role}` has two lanes; give each role one lane"),
                ));
                continue;
            }
            lanes.insert(
                role,
                Lane {
                    tree,
                    safe_state,
                    origin,
                },
            );
        }
    }
    lanes
}

/// The roles of a lane group, each with the values of its parameters; every
/// role gives the same parameters.
fn group_roles(
    roles: &Value,
    params: Option<&Value>,
    observations: &BTreeMap<Name, Observation>,
    fields: &Fields,
    refusals: &mut Vec<Refusal>,
) -> Option<Vec<(Name, BTreeMap<Name, Param>)>> {
    let roles: Vec<Name> = match read_value(roles, &fields.at("roles")) {
        Ok(roles) => roles,
        Err(refusal) => {
            refusals.push(refusal);
            return None;
        }
    };
    if roles.is_empty() {
        refusals.push(Refusal::new(
            At::field(fields.at("roles")),
            "names at least one role",
        ));
        return None;
    }
    let given: BTreeMap<Name, BTreeMap<Name, Value>> = match params {
        None => BTreeMap::new(),
        Some(params) => match read_value(params, &fields.at("params")) {
            Ok(params) => params,
            Err(refusal) => {
                refusals.push(refusal);
                return None;
            }
        },
    };
    let names_of = |role: &Name| -> Vec<&Name> {
        given
            .get(role)
            .map(|params| params.keys().collect())
            .unwrap_or_default()
    };
    let first = names_of(&roles[0]);
    let mut members = Vec::new();
    for role in &roles {
        if names_of(role) != first {
            refusals.push(Refusal::new(
                At::field(fields.at("params")),
                format!(
                    "the role `{role}` gives other parameters than `{}`; give each role the same",
                    roles[0]
                ),
            ));
            return None;
        }
        let values = given.get(role).cloned().unwrap_or_default();
        let mut params = BTreeMap::new();
        for (name, value) in values {
            let at = format!("{}.{role}.{name}", fields.at("params"));
            let param = match value {
                Value::Object(ref map) if map.len() == 1 && map.contains_key("$observation") => {
                    let observation = match read_value::<Name>(&map["$observation"], &at) {
                        Ok(observation) => observation,
                        Err(refusal) => {
                            refusals.push(refusal);
                            return None;
                        }
                    };
                    // A lane of the group reads the observations of its own role.
                    let refusal = match observations.get(&observation) {
                        None => Some(format!("`{observation}` is no observation of the workflow")),
                        Some(read) if read.role != *role => Some(format!(
                            "`{observation}` is an observation of the role `{}`; a parameter of `{role}` names one of `{role}`",
                            read.role
                        )),
                        Some(_) => None,
                    };
                    if let Some(message) = refusal {
                        refusals.push(Refusal::new(At::field(at), message));
                        return None;
                    }
                    Param::Observation(observation)
                }
                Value::Object(ref map) if map.len() == 1 && map.contains_key("$location") => {
                    match read_value::<Name>(&map["$location"], &at) {
                        Ok(location) => Param::Location(location),
                        Err(refusal) => {
                            refusals.push(refusal);
                            return None;
                        }
                    }
                }
                Value::Object(ref map) if map.len() == 1 && map.contains_key("$signal") => {
                    match read_value::<Name>(&map["$signal"], &at) {
                        Ok(signal) => Param::Signal(signal),
                        Err(refusal) => {
                            refusals.push(refusal);
                            return None;
                        }
                    }
                }
                Value::Object(ref map) if map.keys().any(|key| key.starts_with('$')) => {
                    refusals.push(Refusal::new(
                        At::field(at),
                        "a parameter is a literal, `{ $observation }`, `{ $location }` or `{ $signal }`",
                    ));
                    return None;
                }
                Value::Null => {
                    refusals.push(Refusal::new(
                        At::field(at),
                        "is null; give the parameter a value",
                    ));
                    return None;
                }
                literal => Param::Literal(literal),
            };
            params.insert(name, param);
        }
        members.push((role.clone(), params));
    }
    let unknown: Vec<&Name> = given.keys().filter(|role| !roles.contains(role)).collect();
    if let Some(role) = unknown.first() {
        refusals.push(Refusal::new(
            At::field(fields.at("params")),
            format!("gives parameters to `{role}`, which is no role of the group"),
        ));
        return None;
    }
    Some(members)
}

fn safe_calls(reader: &mut StepReader, value: Option<&Value>, path: &str) -> Option<Vec<SafeCall>> {
    let Some(value) = value else {
        return Some(Vec::new());
    };
    let Value::Array(items) = value else {
        reader
            .refusals
            .push(Refusal::new(At::field(path), "is a list of calls"));
        return None;
    };
    let calls: Vec<Option<SafeCall>> = items
        .iter()
        .enumerate()
        .map(|(index, item)| safe_call(reader, item, &format!("{path}[{index}]")))
        .collect();
    calls.into_iter().collect()
}

fn safe_call(reader: &mut StepReader, value: &Value, path: &str) -> Option<SafeCall> {
    let fields = match Fields::of(value, path) {
        Ok(fields) => fields,
        Err(refusal) => {
            reader.refusals.push(refusal);
            return None;
        }
    };
    let id = match fields.read_required("id") {
        Ok(id) => id,
        Err(refusal) => {
            reader.refusals.push(refusal);
            return None;
        }
    };
    let at = At::step(reader.role, &id);
    if !fields.has("timeout_s") {
        reader.refusals.push(Refusal::new(
            at,
            "a call of a `safe_state` has its own `timeout_s`",
        ));
        return None;
    }
    let timeout = match fields.read_required::<Bound>("timeout_s") {
        Ok(timeout) => timeout,
        Err(refusal) => {
            reader.refusals.push(refusal);
            return None;
        }
    };
    let call = match fields.require("call") {
        Ok(call) => call,
        Err(refusal) => {
            reader.refusals.push(refusal);
            return None;
        }
    };
    let call = reader.call(call, &fields.at("call"), &at)?;
    if call.result.is_some() || call.done_when.is_some() {
        reader.refusals.push(Refusal::new(
            at,
            "a call of a `safe_state` takes `member`, `instance` and `args` only",
        ));
        return None;
    }
    let unknown = fields.unknown();
    if !unknown.is_empty() {
        reader.refusals.extend(unknown);
        return None;
    }
    Some(SafeCall {
        id,
        timeout,
        member: call.member,
        instance: call.instance,
        args: call.args,
    })
}

fn sorted(mut refusals: Vec<Refusal>) -> Vec<Refusal> {
    refusals.sort();
    refusals.dedup();
    refusals
}

fn refused(refusals: Vec<Refusal>) -> Refusals {
    Refusals::or_ok(sorted(refusals), ()).expect_err("refused with at least one refusal")
}
