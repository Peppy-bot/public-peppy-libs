//! The work cell of a run: the parameters a sweep tested, the envelope it
//! recorded for them, the values an operator measured, and the values that no
//! test covered.
//!
//! A work cell parameter is a number of the work cell: a control value, a
//! distance from a robot base to a surface, or an observation that the
//! workflow director reads through the robots (`watch`). A parameter of the
//! kind `distance` can have one value per role (`per_role`). The envelope
//! gives each parameter, and each role of its `per_role`, the `[min, max]`
//! that a signed-off sweep tested.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::expression::Expression;
use crate::names::Name;
use crate::refusal::{At, Refusal, Refusals};
use crate::sign_off::Thresholds;
use crate::workflow::Workflow;

/// A value of a parameter: one for the work cell, or one per role.
#[derive(Clone, Debug, PartialEq)]
pub enum RoleValues<T> {
    Single(T),
    PerRole(BTreeMap<Name, T>),
}

impl<T: Serialize> Serialize for RoleValues<T> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            RoleValues::Single(value) => value.serialize(serializer),
            RoleValues::PerRole(values) => values.serialize(serializer),
        }
    }
}

/// An object is a value per role, by the name of each role; any other value
/// is the one value of the work cell.
impl<'de, T: DeserializeOwned> Deserialize<'de> for RoleValues<T> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<RoleValues<T>, D::Error> {
        let value = Value::deserialize(deserializer)?;
        let read = match value {
            Value::Object(_) => serde_path_to_error::deserialize(value).map(RoleValues::PerRole),
            single => serde_path_to_error::deserialize(single).map(RoleValues::Single),
        };
        read.map_err(serde::de::Error::custom)
    }
}

/// The measured values of a work cell, by parameter: what a run or a trial
/// gives in its `work_cell` block.
pub type WorkCell = BTreeMap<Name, RoleValues<f64>>;

/// The range of values a sweep tested: `[min, max]`, both included.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TestedRange {
    pub min: f64,
    pub max: f64,
}

impl TestedRange {
    pub fn holds(&self, value: f64) -> bool {
        self.min <= value && value <= self.max
    }
}

impl fmt::Display for TestedRange {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}, {}]", self.min, self.max)
    }
}

impl Serialize for TestedRange {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        [self.min, self.max].serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for TestedRange {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<TestedRange, D::Error> {
        let [min, max] = <[f64; 2]>::deserialize(deserializer)?;
        let ordered = min <= max;
        if !ordered {
            return Err(serde::de::Error::custom(format!(
                "the range [{min}, {max}] ends below its start; write [min, max] with min at most max"
            )));
        }
        Ok(TestedRange { min, max })
    }
}

/// The envelope of a bundle: the range of each parameter, per role for a
/// parameter with `per_role`.
pub type Envelope = BTreeMap<Name, RoleValues<TestedRange>>;

/// A parameter of the work cell: its unit and its kind.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ParameterFields", into = "ParameterFields")]
pub struct WorkCellParameter {
    pub unit: String,
    pub kind: ParameterKind,
}

/// How the value of a parameter is known: an operator measures a control or
/// a distance and gives it in the work cell block; the director reads a
/// watch parameter through the robots.
#[derive(Clone, Debug, PartialEq)]
pub enum ParameterKind {
    Control,
    /// A distance from a robot base, one value per role of `per_role`, or
    /// one value without it.
    Distance {
        per_role: Option<BTreeSet<Name>>,
    },
    Watch {
        observation: Expression,
    },
}

impl ParameterKind {
    /// The roles of a parameter with one value per role.
    pub fn per_role(&self) -> Option<&BTreeSet<Name>> {
        match self {
            ParameterKind::Distance { per_role } => per_role.as_ref(),
            ParameterKind::Control | ParameterKind::Watch { .. } => None,
        }
    }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum KindWord {
    Control,
    Distance,
    Watch,
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a work cell parameter, { unit, kind, per_role, observation }"
)]
struct ParameterFields {
    unit: String,
    kind: KindWord,
    #[serde(skip_serializing_if = "Option::is_none")]
    per_role: Option<Vec<Name>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    observation: Option<Expression>,
}

impl TryFrom<ParameterFields> for WorkCellParameter {
    type Error = String;

    fn try_from(fields: ParameterFields) -> Result<WorkCellParameter, String> {
        let unit = unit_named(fields.unit)?;
        let kind = match (fields.kind, fields.per_role, fields.observation) {
            (KindWord::Control, None, None) => ParameterKind::Control,
            (KindWord::Distance, per_role, None) => ParameterKind::Distance {
                per_role: per_role.map(roles_named_once).transpose()?,
            },
            (KindWord::Watch, None, Some(observation)) => ParameterKind::Watch { observation },
            (KindWord::Watch, _, None) => {
                return Err(
                    "a `watch` parameter names the `observation` that the director reads"
                        .to_owned(),
                );
            }
            _ => {
                return Err(
                    "`per_role` is for a `distance` parameter, and `observation` for a `watch` parameter"
                        .to_owned(),
                );
            }
        };
        Ok(WorkCellParameter { unit, kind })
    }
}

impl From<WorkCellParameter> for ParameterFields {
    fn from(parameter: WorkCellParameter) -> ParameterFields {
        let (kind, per_role, observation) = match parameter.kind {
            ParameterKind::Control => (KindWord::Control, None, None),
            ParameterKind::Distance { per_role } => (
                KindWord::Distance,
                per_role.map(|roles| roles.into_iter().collect()),
                None,
            ),
            ParameterKind::Watch { observation } => (KindWord::Watch, None, Some(observation)),
        };
        ParameterFields {
            unit: parameter.unit,
            kind,
            per_role,
            observation,
        }
    }
}

/// `unit` when it is not blank.
fn unit_named(unit: String) -> Result<String, String> {
    if unit.trim().is_empty() {
        return Err("the unit is blank; name the unit of the value, such as `m`".to_owned());
    }
    Ok(unit)
}

/// The roles of `per_role`, at least one, each named once.
fn roles_named_once(roles: Vec<Name>) -> Result<BTreeSet<Name>, String> {
    if roles.is_empty() {
        return Err(
            "`per_role` names at least one role; leave it out for one value of the work cell"
                .to_owned(),
        );
    }
    let mut once = BTreeSet::new();
    for role in roles {
        if let Some(twice) = once.replace(role) {
            return Err(format!(
                "`per_role` names the role `{twice}` twice; name each role once"
            ));
        }
    }
    Ok(once)
}

/// A parameter that a sweep or a run declares in its field `watch`: an
/// observation that the director reads through the robots.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a watch parameter, { unit, observation }"
)]
pub struct WatchParameter {
    #[serde(deserialize_with = "unit_of_watch")]
    pub unit: String,
    pub observation: Expression,
}

fn unit_of_watch<'de, D: Deserializer<'de>>(deserializer: D) -> Result<String, D::Error> {
    unit_named(String::deserialize(deserializer)?).map_err(serde::de::Error::custom)
}

/// The refusal of each watch parameter whose observation reads a name that
/// is no observation of `workflow`, at `field.<parameter>`.
pub fn watch_refusals<'a>(
    workflow: &Workflow,
    field: &str,
    observations: impl IntoIterator<Item = (&'a Name, &'a Expression)>,
) -> Vec<Refusal> {
    observations
        .into_iter()
        .flat_map(|(name, observation)| {
            observation
                .roots()
                .into_iter()
                .filter(|root| !workflow.observations.contains_key(root))
                .map(move |root| {
                    Refusal::new(
                        At::field(format!("{field}.{name}")),
                        format!("reads `{root}`, which is no observation of the workflow"),
                    )
                })
        })
        .collect()
}

/// What a signed-off sweep gives a bundle: the thresholds that each trial of
/// the bundle meets, the parameters of the work cell and their envelope.
/// Each parameter has a range in the envelope, per role for a parameter with
/// `per_role`, and the envelope names no other.
#[derive(Clone, Debug, PartialEq)]
pub struct Validation {
    pub thresholds: Thresholds,
    pub work_cell_parameters: BTreeMap<Name, WorkCellParameter>,
    pub envelope: Envelope,
}

impl Validation {
    /// The validation of the parameters and envelope given, or the refusal
    /// of each parameter whose envelope does not give it a range, per role
    /// for a parameter with `per_role`, and of each range of no parameter.
    pub fn new(
        thresholds: Thresholds,
        work_cell_parameters: BTreeMap<Name, WorkCellParameter>,
        envelope: Envelope,
    ) -> Result<Validation, Refusals> {
        let mut refusals = Vec::new();
        for (name, parameter) in &work_cell_parameters {
            let at = || At::field(format!("envelope.{name}"));
            match (parameter.kind.per_role(), envelope.get(name)) {
                (_, None) => refusals.push(Refusal::new(
                    At::field("envelope"),
                    format!("gives no range to the parameter `{name}`"),
                )),
                (None, Some(RoleValues::PerRole(_))) => refusals.push(Refusal::new(
                    at(),
                    "gives a range per role to a parameter without `per_role`; give it one range",
                )),
                (Some(_), Some(RoleValues::Single(_))) => refusals.push(Refusal::new(
                    at(),
                    "gives one range to a parameter with `per_role`; give a range to each of its roles",
                )),
                (Some(roles), Some(RoleValues::PerRole(ranges))) => {
                    let given: BTreeSet<&Name> = ranges.keys().collect();
                    let named: BTreeSet<&Name> = roles.iter().collect();
                    if given != named {
                        refusals.push(Refusal::new(
                            at(),
                            format!(
                                "gives a range to the roles {}; give one to each role of `per_role`: {}",
                                listed(&given),
                                listed(&named)
                            ),
                        ));
                    }
                }
                (None, Some(RoleValues::Single(_))) => {}
            }
        }
        for name in envelope.keys() {
            if !work_cell_parameters.contains_key(name) {
                refusals.push(Refusal::new(
                    At::field(format!("envelope.{name}")),
                    "is no parameter of `work_cell_parameters`",
                ));
            }
        }
        Refusals::or_ok(
            refusals,
            Validation {
                thresholds,
                work_cell_parameters,
                envelope,
            },
        )
    }

    /// The refusal of each parameter of a work cell block that does not fit
    /// these parameters, at `field.<parameter>`: a value of a watch
    /// parameter, which the director reads, one value for a parameter with
    /// `per_role`, a value per role for a parameter without it, and a value
    /// for a role that is not in `per_role`.
    pub fn work_cell_refusals(&self, field: &str, work_cell: &WorkCell) -> Vec<Refusal> {
        let mut refusals = Vec::new();
        for (name, values) in work_cell {
            let Some(parameter) = self.work_cell_parameters.get(name) else {
                continue;
            };
            let at = || At::field(format!("{field}.{name}"));
            match (&parameter.kind, values) {
                (ParameterKind::Watch { .. }, _) => refusals.push(Refusal::new(
                    at(),
                    "is a watch parameter, which the director reads through the robots; leave it out",
                )),
                (kind, RoleValues::Single(_)) if kind.per_role().is_some() => {
                    refusals.push(Refusal::new(
                        at(),
                        "has a value per role in the bundle; give a value to each role of its `per_role`",
                    ))
                }
                (kind, RoleValues::PerRole(per_role)) => match kind.per_role() {
                    None => refusals.push(Refusal::new(
                        at(),
                        "has one value in the bundle; give one value, not a value per role",
                    )),
                    Some(roles) => refusals.extend(
                        per_role
                            .keys()
                            .filter(|role| !roles.contains(*role))
                            .map(|role| {
                                Refusal::new(
                                    At::field(format!("{field}.{name}.{role}")),
                                    format!(
                                        "is no role of the `per_role` of the parameter: {}",
                                        listed(&roles.iter().collect())
                                    ),
                                )
                            }),
                    ),
                },
                (_, RoleValues::Single(_)) => {}
            }
        }
        refusals
    }
}

/// `{ a, b }` as "`a`, `b`", or "none".
fn listed(names: &BTreeSet<&Name>) -> String {
    if names.is_empty() {
        return "none".to_owned();
    }
    let quoted: Vec<String> = names.iter().map(|name| format!("`{name}`")).collect();
    quoted.join(", ")
}

/// A value of the work cell that no test covered: the parameter, its role
/// for a parameter with `per_role`, its value when it has one, and why.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Uncovered {
    pub parameter: Name,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<Name>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    pub why: NotCovered,
}

/// Why no test covered a value.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotCovered {
    /// The bundle has the parameter, and the work cell block gives no value.
    Missing,
    /// The value is outside the range that the sweep tested.
    Outside { tested: TestedRange },
    /// No envelope gives the parameter a range: the run has no bundle, its
    /// bundle has no envelope, or the envelope has no such parameter.
    NoEnvelope,
}

impl fmt::Display for Uncovered {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "`{}`", self.parameter)?;
        if let Some(role) = &self.role {
            write!(f, " of the role `{role}`")?;
        }
        match (&self.why, self.value) {
            (NotCovered::Missing, _) => f.write_str(" has no value in the work cell block"),
            (NotCovered::Outside { tested }, Some(value)) => {
                write!(f, " is {value}, outside the tested range {tested}")
            }
            (NotCovered::Outside { tested }, None) => {
                write!(f, " is outside the tested range {tested}")
            }
            (NotCovered::NoEnvelope, Some(value)) => {
                write!(f, " is {value}, and no envelope gives it a range")
            }
            (NotCovered::NoEnvelope, None) => f.write_str(" has no range in an envelope"),
        }
    }
}

/// Each value of `work_cell` that no test covered, against the validation of
/// a bundle when there is one: each control or distance parameter of the
/// bundle with no value (per role of its `per_role`), each value outside its
/// range, and each value of a parameter that the bundle does not have. A
/// value that [`Validation::work_cell_refusals`] refuses is not named.
pub fn uncovered_work_cell(
    validation: Option<&Validation>,
    work_cell: &WorkCell,
) -> Vec<Uncovered> {
    let mut uncovered = Vec::new();
    for (name, values) in work_cell {
        let parameter = validation
            .and_then(|validation| Some((validation, validation.work_cell_parameters.get(name)?)));
        for (role, value) in each_value(values) {
            let why = match parameter {
                None => NotCovered::NoEnvelope,
                Some((validation, parameter)) => {
                    let watched = matches!(parameter.kind, ParameterKind::Watch { .. });
                    // The envelope has a range of the shape of each
                    // parameter, so a value of no range is a misfit that
                    // `work_cell_refusals` refuses.
                    let Some(tested) = tested_range(validation, name, role).filter(|_| !watched)
                    else {
                        continue;
                    };
                    if tested.holds(value) {
                        continue;
                    }
                    NotCovered::Outside { tested }
                }
            };
            uncovered.push(Uncovered {
                parameter: name.clone(),
                role: role.cloned(),
                value: Some(value),
                why,
            });
        }
    }
    let Some(validation) = validation else {
        return uncovered;
    };
    for (name, parameter) in &validation.work_cell_parameters {
        let missing_roles: Vec<Option<&Name>> = match (&parameter.kind, work_cell.get(name)) {
            (ParameterKind::Watch { .. }, _) => continue,
            (kind, None) => match kind.per_role() {
                Some(roles) => roles.iter().map(Some).collect(),
                None => vec![None],
            },
            (kind, Some(RoleValues::PerRole(given))) => kind
                .per_role()
                .into_iter()
                .flatten()
                .filter(|role| !given.contains_key(*role))
                .map(Some)
                .collect(),
            (_, Some(RoleValues::Single(_))) => continue,
        };
        uncovered.extend(missing_roles.into_iter().map(|role| Uncovered {
            parameter: name.clone(),
            role: role.cloned(),
            value: None,
            why: NotCovered::Missing,
        }));
    }
    uncovered
}

/// Each watch value that the director read and that no test covered: each
/// value outside its range, and each that no envelope gives a range.
pub fn uncovered_watch(
    validation: Option<&Validation>,
    values: &BTreeMap<Name, f64>,
) -> Vec<Uncovered> {
    values
        .iter()
        .filter_map(|(name, &value)| {
            let tested = validation.and_then(|validation| tested_range(validation, name, None));
            let why = match tested {
                None => NotCovered::NoEnvelope,
                Some(tested) if tested.holds(value) => return None,
                Some(tested) => NotCovered::Outside { tested },
            };
            Some(Uncovered {
                parameter: name.clone(),
                role: None,
                value: Some(value),
                why,
            })
        })
        .collect()
}

/// Each value of `values`, with its role when it has one.
fn each_value(values: &RoleValues<f64>) -> Vec<(Option<&Name>, f64)> {
    match values {
        RoleValues::Single(value) => vec![(None, *value)],
        RoleValues::PerRole(per_role) => per_role
            .iter()
            .map(|(role, value)| (Some(role), *value))
            .collect(),
    }
}

/// The range of the parameter `name`, of `role` for a value per role, when
/// the envelope gives one of that shape.
fn tested_range(validation: &Validation, name: &Name, role: Option<&Name>) -> Option<TestedRange> {
    match (validation.envelope.get(name)?, role) {
        (RoleValues::Single(tested), None) => Some(*tested),
        (RoleValues::PerRole(ranges), Some(role)) => ranges.get(role).copied(),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sign_off::tests::thresholds;

    fn name(text: &str) -> Name {
        Name::parse(text).unwrap()
    }

    fn read<T: DeserializeOwned>(text: &str) -> Result<T, String> {
        let tree = documents_core::parse(text).map_err(|error| error.to_string())?;
        serde_path_to_error::deserialize(tree).map_err(|error| error.to_string())
    }

    /// The parameters and envelope of the tape line of the plan.
    fn tape_line() -> Validation {
        let parameters = read(
            r#"{
              belt_speed_mps: { unit: "m/s", kind: "control" },
              belt_top_above_base_m: { unit: "m", kind: "distance", per_role: ["so101_a", "so101_b"] },
              loose_items: { unit: "items", kind: "watch", observation: "loose.count" },
            }"#,
        )
        .unwrap();
        let envelope = read(
            r#"{
              belt_speed_mps: [0.05, 0.25],
              belt_top_above_base_m: { so101_a: [0.10, 0.22], so101_b: [0.10, 0.22] },
              loose_items: [0, 12],
            }"#,
        )
        .unwrap();
        Validation::new(thresholds(), parameters, envelope).unwrap()
    }

    #[test]
    fn a_parameter_takes_the_fields_of_its_kind_alone() {
        for (text, refusal) in [
            (
                r#"{ unit: "m", kind: "control", per_role: ["a"] }"#,
                "`per_role` is for a `distance` parameter, and `observation` for a `watch` parameter",
            ),
            (
                r#"{ unit: "m", kind: "distance", observation: "a.b" }"#,
                "`per_role` is for a `distance` parameter, and `observation` for a `watch` parameter",
            ),
            (
                r#"{ unit: "items", kind: "watch" }"#,
                "a `watch` parameter names the `observation` that the director reads",
            ),
            (
                r#"{ unit: " ", kind: "control" }"#,
                "the unit is blank; name the unit of the value, such as `m`",
            ),
            (
                r#"{ unit: "m", kind: "distance", per_role: [] }"#,
                "`per_role` names at least one role; leave it out for one value of the work cell",
            ),
            (
                r#"{ unit: "m", kind: "distance", per_role: ["a", "a"] }"#,
                "`per_role` names the role `a` twice; name each role once",
            ),
        ] {
            let refused = read::<WorkCellParameter>(text).unwrap_err();
            assert!(refused.ends_with(refusal), "{text}: {refused}");
        }
        let kinds = read::<BTreeMap<Name, WorkCellParameter>>(
            r#"{ a: { unit: "m/s", kind: "control" }, b: { unit: "m", kind: "distance" } }"#,
        )
        .unwrap();
        assert_eq!(kinds[&name("a")].kind, ParameterKind::Control);
        assert_eq!(
            kinds[&name("b")].kind,
            ParameterKind::Distance { per_role: None }
        );
    }

    #[test]
    fn a_parameter_writes_back_as_it_reads() {
        let validation = tape_line();
        let text = serde_json::to_string(&validation.work_cell_parameters).unwrap();
        assert_eq!(
            read::<BTreeMap<Name, WorkCellParameter>>(&text).unwrap(),
            validation.work_cell_parameters
        );
        let envelope = serde_json::to_string(&validation.envelope).unwrap();
        assert_eq!(read::<Envelope>(&envelope).unwrap(), validation.envelope);
    }

    #[test]
    fn a_range_ends_at_or_above_its_start() {
        assert!(read::<TestedRange>("[0.3, 0.1]").unwrap_err().ends_with(
            "the range [0.3, 0.1] ends below its start; write [min, max] with min at most max"
        ));
        assert_eq!(
            read::<TestedRange>("[0.1, 0.1]").unwrap(),
            TestedRange { min: 0.1, max: 0.1 }
        );
    }

    #[test]
    fn the_envelope_gives_each_parameter_a_range_of_its_shape_and_no_other() {
        let parameters = tape_line().work_cell_parameters;
        let envelope = |text: &str| read::<Envelope>(text).unwrap();
        let refused = Validation::new(
            thresholds(),
            parameters,
            envelope(
                r#"{ belt_speed_mps: { so101_a: [0, 1] }, belt_top_above_base_m: { so101_a: [0, 1] }, gripper_force: [0, 1] }"#,
            ),
        )
        .unwrap_err();
        let texts: Vec<String> = refused.iter().map(Refusal::to_string).collect();
        assert_eq!(
            texts,
            [
                "envelope.belt_speed_mps: gives a range per role to a parameter without `per_role`; give it one range",
                "envelope.belt_top_above_base_m: gives a range to the roles `so101_a`; give one to each role of `per_role`: `so101_a`, `so101_b`",
                "envelope: gives no range to the parameter `loose_items`",
                "envelope.gripper_force: is no parameter of `work_cell_parameters`",
            ]
        );
    }

    #[test]
    fn a_work_cell_value_takes_the_shape_of_its_parameter() {
        let validation = tape_line();
        let work_cell: WorkCell = read(
            r#"{ belt_speed_mps: { so101_a: 0.2 }, belt_top_above_base_m: 0.18, loose_items: 3, gripper_force: 2 }"#,
        )
        .unwrap();
        let texts: Vec<String> = validation
            .work_cell_refusals("work_cell", &work_cell)
            .iter()
            .map(Refusal::to_string)
            .collect();
        assert_eq!(
            texts,
            [
                "work_cell.belt_speed_mps: has one value in the bundle; give one value, not a value per role",
                "work_cell.belt_top_above_base_m: has a value per role in the bundle; give a value to each role of its `per_role`",
                "work_cell.loose_items: is a watch parameter, which the director reads through the robots; leave it out",
            ]
        );
        let courier: WorkCell =
            read(r#"{ belt_top_above_base_m: { so101_a: 0.18, courier: 0.2 } }"#).unwrap();
        assert_eq!(
            validation.work_cell_refusals("work_cell", &courier)[0].to_string(),
            "work_cell.belt_top_above_base_m.courier: is no role of the `per_role` of the parameter: `so101_a`, `so101_b`"
        );
    }

    #[test]
    fn each_value_no_test_covered_is_named_with_why() {
        let validation = tape_line();
        let inside: WorkCell = read(
            r#"{ belt_speed_mps: 0.20, belt_top_above_base_m: { so101_a: 0.18, so101_b: 0.21 } }"#,
        )
        .unwrap();
        assert_eq!(uncovered_work_cell(Some(&validation), &inside), []);
        let off: WorkCell =
            read(r#"{ belt_top_above_base_m: { so101_a: 0.30 }, gripper_force: 2 }"#).unwrap();
        let named: Vec<String> = uncovered_work_cell(Some(&validation), &off)
            .iter()
            .map(Uncovered::to_string)
            .collect();
        assert_eq!(
            named,
            [
                "`belt_top_above_base_m` of the role `so101_a` is 0.3, outside the tested range [0.1, 0.22]",
                "`gripper_force` is 2, and no envelope gives it a range",
                "`belt_speed_mps` has no value in the work cell block",
                "`belt_top_above_base_m` of the role `so101_b` has no value in the work cell block",
            ]
        );
        let no_envelope: Vec<NotCovered> = uncovered_work_cell(None, &inside)
            .into_iter()
            .map(|uncovered| uncovered.why)
            .collect();
        assert_eq!(no_envelope, vec![NotCovered::NoEnvelope; 3]);
    }

    #[test]
    fn a_watch_value_outside_its_range_or_with_none_is_named() {
        let validation = tape_line();
        let values = BTreeMap::from([(name("loose_items"), 30.0), (name("crumbs"), 1.0)]);
        let named: Vec<String> = uncovered_watch(Some(&validation), &values)
            .iter()
            .map(Uncovered::to_string)
            .collect();
        assert_eq!(
            named,
            [
                "`crumbs` is 1, and no envelope gives it a range",
                "`loose_items` is 30, outside the tested range [0, 12]",
            ]
        );
        let inside = BTreeMap::from([(name("loose_items"), 12.0)]);
        assert_eq!(uncovered_watch(Some(&validation), &inside), []);
    }

    #[test]
    fn an_uncovered_value_reads_back_from_its_json() {
        let uncovered = Uncovered {
            parameter: name("belt_speed_mps"),
            role: None,
            value: Some(0.3),
            why: NotCovered::Outside {
                tested: TestedRange {
                    min: 0.05,
                    max: 0.25,
                },
            },
        };
        let text = serde_json::to_string(&uncovered).unwrap();
        assert_eq!(
            text,
            r#"{"parameter":"belt_speed_mps","value":0.3,"why":{"outside":{"tested":[0.05,0.25]}}}"#
        );
        assert_eq!(read::<Uncovered>(&text).unwrap(), uncovered);
    }
}
