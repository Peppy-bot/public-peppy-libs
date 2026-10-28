//! Judges: checks of ground truth in a live world, which the simulation
//! evaluates and answers with a verdict each.

use serde::{Deserialize, Deserializer, Serialize};

use crate::geometry::Point;
use crate::names::first_named_twice;
use crate::{Error, Name, RobotName, text};

/// One judge: `true`, which always holds, or a check the simulation
/// evaluates in the live world.
#[derive(Clone, Debug, PartialEq)]
pub enum Judge {
    /// Always holds: the judge of a condition a world makes true by
    /// construction.
    True,
    Check(Check),
}

impl Serialize for Judge {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Judge::True => serializer.serialize_bool(true),
            Judge::Check(check) => check.serialize(serializer),
        }
    }
}

/// The keys that name a check, each the key of one variant of [`Check`].
const CHECKS: [&str; 14] = [
    "count",
    "landmark_on",
    "contents_on",
    "belt_line",
    "in_band",
    "spans_band",
    "near_point",
    "broken",
    "crushed",
    "reachable",
    "control",
    "all",
    "any",
    "not",
];

/// The refusal of a value that is not a judge, naming the forms of one.
fn judge_form() -> String {
    let checks = CHECKS.map(|check| format!("`{check}`")).join(", ");
    format!("a judge is `true` or an object with one key naming a check: {checks}")
}

impl<'de> Deserialize<'de> for Judge {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Judge, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;
        if value == serde_json::Value::Bool(true) {
            return Ok(Judge::True);
        }
        let names_one_check = value.as_object().is_some_and(|fields| {
            fields.len() == 1 && fields.keys().all(|key| CHECKS.contains(&key.as_str()))
        });
        if !names_one_check {
            return Err(serde::de::Error::custom(judge_form()));
        }
        serde_path_to_error::deserialize(value)
            .map(Judge::Check)
            .map_err(serde::de::Error::custom)
    }
}

impl From<Check> for Judge {
    fn from(check: Check) -> Judge {
        Judge::Check(check)
    }
}

/// A check of ground truth, by its kind.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Check {
    Count(Count),
    LandmarkOn(LandmarkOn),
    ContentsOn(ContentsOn),
    /// The belt modules, named in any order, form one line whose joints meet.
    BeltLine(BeltLine),
    /// Each object of the item kind is within `within_m` of the band between
    /// two lines of world points.
    InBand(Band),
    /// The belt line of the item kind reaches each end of the band.
    SpansBand(Band),
    NearPoint(NearPoint),
    /// The object made of pieces is broken.
    Broken(Name),
    /// The crushable object is crushed.
    Crushed(Name),
    Reachable(Reachable),
    Control(Control),
    /// Every judge holds; names at least one.
    #[serde(deserialize_with = "all_judges")]
    All(Vec<Judge>),
    /// One judge holds; names at least one.
    #[serde(deserialize_with = "any_judges")]
    Any(Vec<Judge>),
    Not(Box<Judge>),
}

/// The centres of the two landmarks are within `within_m`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a landmark_on, { landmark, on, within_m }"
)]
pub struct LandmarkOn {
    pub landmark: Landmark,
    pub on: Landmark,
    pub within_m: WithinM,
}

/// Each phase of the contents of the object rests on `on`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a contents_on, { object, on }")]
pub struct ContentsOn {
    pub object: Name,
    pub on: Name,
}

/// The object is within `within_m` of a world point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a near_point, { object, point, within_m }"
)]
pub struct NearPoint {
    pub object: Name,
    pub point: Point,
    pub within_m: WithinM,
}

/// The robot reaches the target, by the reach rule of the simulation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a reachable, { robot, target }")]
pub struct Reachable {
    pub robot: RobotName,
    pub target: Target,
}

/// The control value is within the bounds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ControlFields")]
pub struct Control {
    pub object: Name,
    pub id: String,
    #[serde(flatten)]
    pub bounds: Bounds<f64>,
}

/// The fields of a control check as written, its bounds unchecked.
#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a control, { object, id, at_least, at_most }"
)]
struct ControlFields {
    object: Name,
    id: String,
    at_least: Option<f64>,
    at_most: Option<f64>,
}

impl TryFrom<ControlFields> for Control {
    type Error = Error;

    fn try_from(fields: ControlFields) -> Result<Control, Error> {
        Ok(Control {
            object: fields.object,
            id: fields.id,
            bounds: Bounds::new(fields.at_least, fields.at_most)?,
        })
    }
}

fn all_judges<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec<Judge>, D::Error> {
    at_least_one_judge(deserializer, "all")
}

fn any_judges<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec<Judge>, D::Error> {
    at_least_one_judge(deserializer, "any")
}

/// The judges of the combination `named`, at least one.
fn at_least_one_judge<'de, D: Deserializer<'de>>(
    deserializer: D,
    named: &str,
) -> Result<Vec<Judge>, D::Error> {
    let judges = Vec::<Judge>::deserialize(deserializer)?;
    if judges.is_empty() {
        return Err(serde::de::Error::custom(format!(
            "{named} names at least one judge"
        )));
    }
    Ok(judges)
}

/// A count of objects of an item kind in a place.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "CountFields")]
pub struct Count {
    pub item_kind: String,
    #[serde(rename = "in")]
    pub place: Place,
    #[serde(flatten)]
    pub bounds: Bounds<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub each_holding: Option<Holding>,
}

/// The fields of a count as written, its bounds unchecked.
#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a count, { item_kind, in, at_least, at_most, each_holding }"
)]
struct CountFields {
    item_kind: String,
    #[serde(rename = "in")]
    place: Place,
    at_least: Option<u32>,
    at_most: Option<u32>,
    each_holding: Option<Holding>,
}

impl TryFrom<CountFields> for Count {
    type Error = Error;

    fn try_from(fields: CountFields) -> Result<Count, Error> {
        Ok(Count {
            item_kind: fields.item_kind,
            place: fields.place,
            bounds: Bounds::new(fields.at_least, fields.at_most)?,
            each_holding: fields.each_holding,
        })
    }
}

/// What a counted object holds, for it to count.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a holding, { item_kind, at_least }")]
pub struct Holding {
    pub item_kind: String,
    pub at_least: u32,
}

/// A landmark of an object, as the catalogue names it.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a landmark, { object, name }")]
pub struct Landmark {
    pub object: Name,
    pub name: String,
}

/// The belt modules of a line, at least two, each once.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "Vec<Name>", into = "Vec<Name>")]
pub struct BeltLine(Vec<Name>);

impl TryFrom<Vec<Name>> for BeltLine {
    type Error = Error;

    fn try_from(modules: Vec<Name>) -> Result<BeltLine, Error> {
        if modules.len() < 2 {
            return Err(Error::rule(
                "a belt line names at least two modules; name every module of the line",
            ));
        }
        if let Some(twice) = first_named_twice(&modules, PartialEq::eq) {
            return Err(Error::rule(format!(
                "`{twice}` is named twice; name each module once"
            )));
        }
        Ok(BeltLine(modules))
    }
}

impl From<BeltLine> for Vec<Name> {
    fn from(line: BeltLine) -> Vec<Name> {
        line.0
    }
}

impl BeltLine {
    pub fn modules(&self) -> &[Name] {
        &self.0
    }
}

/// Objects of an item kind against the band between two lines of world
/// points, each line of at least two points, drawn in either direction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "BandFields")]
pub struct Band {
    pub item_kind: String,
    pub between: [Vec<Point>; 2],
    pub within_m: WithinM,
}

#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a band, { item_kind, between, within_m }"
)]
struct BandFields {
    item_kind: String,
    between: [Vec<Point>; 2],
    within_m: WithinM,
}

impl TryFrom<BandFields> for Band {
    type Error = Error;

    fn try_from(fields: BandFields) -> Result<Band, Error> {
        if fields.between.iter().any(|line| line.len() < 2) {
            return Err(Error::rule(
                "each line of a band has at least two points; give each line two points or more",
            ));
        }
        Ok(Band {
            item_kind: fields.item_kind,
            between: fields.between,
            within_m: fields.within_m,
        })
    }
}

/// Where a count looks: in an object, a named region or a location.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "PlaceFields", into = "PlaceFields")]
pub enum Place {
    Object(Name),
    Region(Name),
    Location(Name),
}

/// What a robot reaches: an object, a landmark of one, a named region or a
/// location.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "TargetFields", into = "TargetFields")]
pub enum Target {
    Object(Name),
    Landmark(Landmark),
    Region(Name),
    Location(Name),
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a target, { object }, { object, landmark }, { region } or { location }"
)]
struct TargetFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    object: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    landmark: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    region: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    location: Option<Name>,
}

impl TryFrom<TargetFields> for Target {
    type Error = Error;

    fn try_from(fields: TargetFields) -> Result<Target, Error> {
        match fields {
            TargetFields {
                object: Some(object),
                landmark: None,
                region: None,
                location: None,
            } => Ok(Target::Object(object)),
            TargetFields {
                object: Some(object),
                landmark: Some(name),
                region: None,
                location: None,
            } => Ok(Target::Landmark(Landmark { object, name })),
            TargetFields {
                object: None,
                landmark: None,
                region: Some(region),
                location: None,
            } => Ok(Target::Region(region)),
            TargetFields {
                object: None,
                landmark: None,
                region: None,
                location: Some(location),
            } => Ok(Target::Location(location)),
            _ => Err(Error::rule(
                "a target is { object }, { object, landmark }, { region } or { location }",
            )),
        }
    }
}

impl From<Target> for TargetFields {
    fn from(target: Target) -> TargetFields {
        let (object, landmark, region, location) = match target {
            Target::Object(object) => (Some(object), None, None, None),
            Target::Landmark(Landmark { object, name }) => (Some(object), Some(name), None, None),
            Target::Region(region) => (None, None, Some(region), None),
            Target::Location(location) => (None, None, None, Some(location)),
        };
        TargetFields {
            object,
            landmark,
            region,
            location,
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a place, { object }, { region } or { location }"
)]
struct PlaceFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    object: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    region: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    location: Option<Name>,
}

impl TryFrom<PlaceFields> for Place {
    type Error = Error;

    fn try_from(fields: PlaceFields) -> Result<Place, Error> {
        match fields {
            PlaceFields {
                object: Some(object),
                region: None,
                location: None,
            } => Ok(Place::Object(object)),
            PlaceFields {
                object: None,
                region: Some(region),
                location: None,
            } => Ok(Place::Region(region)),
            PlaceFields {
                object: None,
                region: None,
                location: Some(location),
            } => Ok(Place::Location(location)),
            _ => Err(Error::rule(
                "a place is { object }, { region } or { location }",
            )),
        }
    }
}

impl From<Place> for PlaceFields {
    fn from(place: Place) -> PlaceFields {
        let (object, region, location) = match place {
            Place::Object(object) => (Some(object), None, None),
            Place::Region(region) => (None, Some(region), None),
            Place::Location(location) => (None, None, Some(location)),
        };
        PlaceFields {
            object,
            region,
            location,
        }
    }
}

/// A distance a judge compares at, metres, above 0.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "f64", into = "f64")]
pub struct WithinM(f64);

impl TryFrom<f64> for WithinM {
    type Error = Error;

    fn try_from(metres: f64) -> Result<WithinM, Error> {
        let above_zero = metres > 0.0;
        if !above_zero {
            return Err(Error::rule(format!(
                "{metres} is not above 0; write a distance above 0 metres"
            )));
        }
        Ok(WithinM(metres))
    }
}

impl From<WithinM> for f64 {
    fn from(within: WithinM) -> f64 {
        within.0
    }
}

impl WithinM {
    pub fn metres(self) -> f64 {
        self.0
    }
}

/// Bounds on a value: at least one of the two, in order.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(into = "BoundsFields<T>", bound = "T: Bound")]
pub struct Bounds<T> {
    pub at_least: Option<T>,
    pub at_most: Option<T>,
}

/// A value a judge bounds: a count or a control value.
pub trait Bound: Copy + PartialOrd + Serialize + std::fmt::Display {}

impl Bound for u32 {}
impl Bound for f64 {}

impl<T: Bound> Bounds<T> {
    /// The bounds `at_least` and `at_most`. Refuses a bound that is not a
    /// number, no bound at all, and an `at_least` above `at_most`.
    pub(crate) fn new(at_least: Option<T>, at_most: Option<T>) -> Result<Bounds<T>, Error> {
        for (field, bound) in [("at_least", at_least), ("at_most", at_most)] {
            if let Some(value) = bound
                && !is_number(value)
            {
                return Err(Error::rule(format!(
                    "{field} {value} is not a number; write a finite number"
                )));
            }
        }
        match (at_least, at_most) {
            (None, None) => Err(Error::rule("name at_least, at_most or both")),
            (Some(least), Some(most)) if least > most => Err(Error::rule(format!(
                "at_least {least} is above at_most {most}; write an at_least no higher than at_most"
            ))),
            (at_least, at_most) => Ok(Bounds { at_least, at_most }),
        }
    }
}

/// The bounds as a document writes them.
#[derive(Serialize)]
#[serde(bound = "T: Bound")]
struct BoundsFields<T> {
    #[serde(skip_serializing_if = "Option::is_none")]
    at_least: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    at_most: Option<T>,
}

/// Whether `value` compares with itself: every value but NaN.
fn is_number<T: PartialOrd>(value: T) -> bool {
    value.partial_cmp(&value).is_some()
}

impl<T: Bound> From<Bounds<T>> for BoundsFields<T> {
    fn from(bounds: Bounds<T>) -> BoundsFields<T> {
        BoundsFields {
            at_least: bounds.at_least,
            at_most: bounds.at_most,
        }
    }
}

impl<T: Bound> Bounds<T> {
    /// Whether `value` is within the bounds.
    pub fn hold(&self, value: T) -> bool {
        self.at_least.is_none_or(|least| value >= least)
            && self.at_most.is_none_or(|most| value <= most)
    }
}

impl Judge {
    /// Parses the JSON5 text of a list of judges.
    pub fn parse_list(text: &str) -> Result<Vec<Judge>, Error> {
        text::from_json5(text)
    }

    /// A list of judges as JSON5 text. Refuses, by its path, a number a
    /// document cannot hold and a list nested more than 128 levels deep, as
    /// `not`, `all` and `any` can nest it.
    pub fn list_to_json5(judges: &[Judge]) -> Result<String, Error> {
        text::to_json5(&judges)
    }
}

/// The answer of the simulation to one judge: whether it holds, and why in
/// one sentence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a verdict, { holds, why }")]
pub struct Verdict {
    pub holds: bool,
    pub why: String,
}

/// The verdicts of a list of judges, in the order of the judges.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Verdicts(pub Vec<Verdict>);

impl Verdicts {
    pub fn parse(text: &str) -> Result<Verdicts, Error> {
        text::from_json5(text)
    }

    /// The verdicts as JSON5 text.
    pub fn to_json5(&self) -> Result<String, Error> {
        text::to_json5(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::{forms_named, name, refusal_of};

    /// The refusal of a value that is not a judge, at the first judge of a
    /// list.
    const JUDGE_FORM: &str = "[0]: a judge is `true` or an object with one key naming a check: `count`, `landmark_on`, `contents_on`, `belt_line`, `in_band`, `spans_band`, `near_point`, `broken`, `crushed`, `reachable`, `control`, `all`, `any`, `not`";

    const JUDGES: &str = r#"[
  { count: { item_kind: "book", in: { object: "crate" }, at_least: 2 } },
  { count: { item_kind: "carton", in: { region: "box_stock" }, at_least: 2, at_most: 4, each_holding: { item_kind: "tape_roll", at_least: 1 } } },
  { landmark_on: { landmark: { object: "skillet", name: "base" }, on: { object: "cooker", name: "zone_front_left" }, within_m: 0.05 } },
  { contents_on: { object: "pan", on: "cooker" } },
  { belt_line: ["belt_1", "belt_2"] },
  { in_band: { item_kind: "belt_module", between: [[[0, 0, 0], [1, 0, 0]], [[0, 0.5, 0], [1, 0.5, 0]]], within_m: 0.1 } },
  { spans_band: { item_kind: "belt_module", between: [[[0, 0, 0], [1, 0, 0]], [[0, 0.5, 0], [1, 0.5, 0]]], within_m: 0.1 } },
  { near_point: { object: "cup", point: [0.3, 0.2, 0.8], within_m: 0.05 } },
  { broken: "egg" },
  { crushed: "can" },
  { reachable: { robot: "alpha", target: { object: "crate", landmark: "top" } } },
  { control: { object: "cooker", id: "heat", at_least: 2 } },
  { all: [{ broken: "egg" }, { not: { crushed: "can" } }] },
  { any: [true, { broken: "egg" }] },
  true,
]"#;

    #[test]
    fn every_judge_of_the_table_parses_and_writes_back_to_the_same_list() {
        let judges = Judge::parse_list(JUDGES).unwrap();
        assert_eq!(judges.len(), 15);
        assert_eq!(
            judges[0],
            Check::Count(Count {
                item_kind: "book".into(),
                place: Place::Object(name("crate")),
                bounds: Bounds {
                    at_least: Some(2),
                    at_most: None
                },
                each_holding: None
            })
            .into()
        );
        assert_eq!(
            judges[10],
            Check::Reachable(Reachable {
                robot: RobotName::parse("alpha").unwrap(),
                target: Target::Landmark(Landmark {
                    object: name("crate"),
                    name: "top".into()
                })
            })
            .into()
        );
        assert_eq!(
            judges[13],
            Check::Any(vec![Judge::True, Check::Broken(name("egg")).into()]).into()
        );
        assert_eq!(judges[14], Judge::True);
        let written = Judge::list_to_json5(&judges).unwrap();
        assert!(written.ends_with("  true,\n]"), "{written}");
        assert_eq!(Judge::parse_list(&written).unwrap(), judges);
    }

    /// A check with fields of its own names, in the refusal of a value of
    /// another type, the fields it takes and no Rust type.
    #[test]
    fn a_check_of_another_type_is_refused_with_the_fields_it_takes() {
        let refusal = |text: &str| Judge::parse_list(text).unwrap_err().to_string();
        let cases = [
            ("landmark_on", "a landmark_on, { landmark, on, within_m }"),
            ("contents_on", "a contents_on, { object, on }"),
            ("near_point", "a near_point, { object, point, within_m }"),
            ("reachable", "a reachable, { robot, target }"),
            ("control", "a control, { object, id, at_least, at_most }"),
            (
                "count",
                "a count, { item_kind, in, at_least, at_most, each_holding }",
            ),
        ];
        for (check, form) in cases {
            let other_type = refusal(&format!("[{{ {check}: 1 }}]"));
            assert!(
                other_type.ends_with(&format!("expected {form}")),
                "{other_type}"
            );
            let unknown = refusal(&format!("[{{ {check}: {{ zz_unknown: 1 }} }}]"));
            for field in forms_named(form)[0].split(", ") {
                assert!(
                    unknown.contains(&format!("`{field}`")),
                    "{field}: {unknown}"
                );
            }
        }
    }

    /// A target names the forms of its refusal for another type, and a place
    /// names those of them it takes.
    #[test]
    fn the_refusals_of_a_target_and_a_place_name_the_same_forms() {
        let other_type = forms_named(&refusal_of::<Target>("1"));
        assert_eq!(forms_named(&refusal_of::<Target>("{}")), other_type);
        let place = forms_named(&refusal_of::<Place>("{}"));
        assert!(
            place.iter().all(|form| other_type.contains(form)),
            "{place:?}"
        );
    }

    #[test]
    fn refusals_name_the_judge_the_field_and_the_rule() {
        let cases = [
            (
                r#"[{ count: { item_kind: "book", in: { object: "crate" } } }]"#,
                "[0]: count: name at_least, at_most or both",
            ),
            (
                r#"[{ control: { object: "desk", id: "height" } }]"#,
                "[0]: control: name at_least, at_most or both",
            ),
            (
                r#"[{ control: { object: "desk", id: "height", at_least: 0.6, at_most: 0.4 } }]"#,
                "[0]: control: at_least 0.6 is above at_most 0.4; write an at_least no higher than at_most",
            ),
            (
                r#"[{ count: { item_kind: "book", in: { object: "crate" }, at_least: 3, at_most: 2 } }]"#,
                "[0]: count: at_least 3 is above at_most 2; write an at_least no higher than at_most",
            ),
            (
                r#"[{ count: { item_kind: "book", in: { object: "crate", region: "shelf" }, at_least: 1 } }]"#,
                "[0]: count.in: a place is { object }, { region } or { location }",
            ),
            (
                r#"[{ count: { item_kind: "book", in: { object: "crate", landmark: "top" }, at_least: 1 } }]"#,
                "[0]: count.in.landmark: unknown field `landmark`, expected one of `object`, `region`, `location`",
            ),
            (
                r#"[{ near_point: { object: "cup", point: [0, 0, 0], within_m: 0 } }]"#,
                "[0]: near_point.within_m: 0 is not above 0; write a distance above 0 metres",
            ),
            (
                r#"[{ belt_line: ["belt_1"] }]"#,
                "[0]: belt_line: a belt line names at least two modules; name every module of the line",
            ),
            (
                r#"[{ belt_line: ["belt_1", "belt_1"] }]"#,
                "[0]: belt_line: `belt_1` is named twice; name each module once",
            ),
            (
                r#"[{ in_band: { item_kind: "b", between: [[[0, 0, 0]], [[0, 1, 0], [1, 1, 0]]], within_m: 0.1 } }]"#,
                "[0]: in_band: each line of a band has at least two points; give each line two points or more",
            ),
            (
                r#"[{ reachable: { robot: "alpha", target: { landmark: "top" } } }]"#,
                "[0]: reachable.target: a target is { object }, { object, landmark }, { region } or { location }",
            ),
            (r#"[false]"#, JUDGE_FORM),
            (r#"[{ broken: "egg", crushed: "can" }]"#, JUDGE_FORM),
            (r#"[null]"#, JUDGE_FORM),
            (r#"["true"]"#, JUDGE_FORM),
            (r#"[{ xyz: 1 }]"#, JUDGE_FORM),
            (
                r#"[true, { broken: "Egg" }]"#,
                "[1]: broken: the name `Egg` is not lowercase letters, digits and `_`",
            ),
            (r#"[{ all: [] }]"#, "[0]: all: all names at least one judge"),
            (
                r#"[{ not: { any: [] } }]"#,
                "[0]: not: any: any names at least one judge",
            ),
            (
                r#"[{ reachable: { robot: "", target: { object: "crate" } } }]"#,
                "[0]: reachable.robot: the robot name `` is not 1 to 63 letters, digits, `_` and `-`",
            ),
        ];
        for (text, expected) in cases {
            assert_eq!(Judge::parse_list(text).unwrap_err().to_string(), expected);
        }
        assert_eq!(
            Judge::parse_list(r#"[true, { not: [] }]"#)
                .unwrap_err()
                .to_string(),
            format!("[1]: not: {}", &JUDGE_FORM[5..])
        );
    }

    /// A key no judge of the table has is refused under the judge's name, in
    /// each judge that holds fields of its own and in the band of `in_band`
    /// and `spans_band`.
    #[test]
    fn an_unknown_field_of_a_judge_is_refused_by_name() {
        let judges = [
            r#"count: { item_kind: "book", in: { object: "crate" }, at_least: 2, typo: 1 }"#,
            r#"landmark_on: { landmark: { object: "a", name: "b" }, on: { object: "c", name: "d" }, within_m: 0.05, typo: 1 }"#,
            r#"contents_on: { object: "pan", on: "cooker", typo: 1 }"#,
            r#"in_band: { item_kind: "b", between: [[[0, 0, 0], [1, 0, 0]], [[0, 1, 0], [1, 1, 0]]], within_m: 0.1, typo: 1 }"#,
            r#"spans_band: { item_kind: "b", between: [[[0, 0, 0], [1, 0, 0]], [[0, 1, 0], [1, 1, 0]]], within_m: 0.1, typo: 1 }"#,
            r#"near_point: { object: "cup", point: [0, 0, 0], within_m: 0.05, typo: 1 }"#,
            r#"reachable: { robot: "alpha", target: { object: "crate" }, typo: 1 }"#,
            r#"control: { object: "cooker", id: "heat", at_least: 2, typo: 1 }"#,
        ];
        for judge in judges {
            let error = Judge::parse_list(&format!("[{{ {judge} }}]"))
                .unwrap_err()
                .to_string();
            let name = judge.split(':').next().unwrap();
            assert!(
                error.starts_with(&format!("[0]: {name}"))
                    && error.contains("unknown field `typo`"),
                "{judge}: {error}"
            );
        }
    }

    #[test]
    fn verdicts_round_trip() {
        let verdicts = Verdicts(vec![
            Verdict {
                holds: true,
                why: "2 books in crate".into(),
            },
            Verdict {
                holds: false,
                why: "egg has 0 pieces off".into(),
            },
        ]);
        assert_eq!(
            Verdicts::parse(&verdicts.to_json5().unwrap()).unwrap(),
            verdicts
        );
    }

    #[test]
    fn a_count_without_each_holding_writes_none() {
        let count = Judge::from(Check::Count(Count {
            item_kind: "book".into(),
            place: Place::Object(name("crate")),
            bounds: Bounds::new(Some(1), None).unwrap(),
            each_holding: None,
        }));
        let text = Judge::list_to_json5(&[count]).unwrap();
        assert!(!text.contains("each_holding"), "{text}");
    }

    #[test]
    fn a_nan_bound_is_refused() {
        assert_eq!(
            Bounds::new(Some(f64::NAN), None).unwrap_err().to_string(),
            "at_least NaN is not a number; write a finite number"
        );
        assert_eq!(
            Bounds::new(Some(1.0), Some(f64::NAN))
                .unwrap_err()
                .to_string(),
            "at_most NaN is not a number; write a finite number"
        );
    }

    /// The key of each check, by a match over every variant of [`Check`]: a
    /// variant without an arm does not compile.
    fn key_of(check: &Check) -> &'static str {
        match check {
            Check::Count(_) => "count",
            Check::LandmarkOn(_) => "landmark_on",
            Check::ContentsOn(_) => "contents_on",
            Check::BeltLine(_) => "belt_line",
            Check::InBand(_) => "in_band",
            Check::SpansBand(_) => "spans_band",
            Check::NearPoint(_) => "near_point",
            Check::Broken(_) => "broken",
            Check::Crushed(_) => "crushed",
            Check::Reachable(_) => "reachable",
            Check::Control(_) => "control",
            Check::All(_) => "all",
            Check::Any(_) => "any",
            Check::Not(_) => "not",
        }
    }

    /// The keys a judge may name are the keys of the variants of [`Check`], in
    /// the order of the variants, each the key the variant writes.
    #[test]
    fn the_keys_of_the_checks_are_the_variants_of_check() {
        let landmark = Landmark {
            object: name("pan"),
            name: "base".into(),
        };
        let band = Band {
            item_kind: "belt_module".into(),
            between: [
                vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                vec![[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
            ],
            within_m: WithinM::try_from(0.1).unwrap(),
        };
        let examples = [
            Check::Count(Count {
                item_kind: "book".into(),
                place: Place::Object(name("crate")),
                bounds: Bounds {
                    at_least: Some(1),
                    at_most: None,
                },
                each_holding: None,
            }),
            Check::LandmarkOn(LandmarkOn {
                landmark: landmark.clone(),
                on: landmark,
                within_m: WithinM::try_from(0.05).unwrap(),
            }),
            Check::ContentsOn(ContentsOn {
                object: name("pan"),
                on: name("cooker"),
            }),
            Check::BeltLine(vec![name("belt_1"), name("belt_2")].try_into().unwrap()),
            Check::InBand(band.clone()),
            Check::SpansBand(band),
            Check::NearPoint(NearPoint {
                object: name("cup"),
                point: [0.0, 0.0, 0.0],
                within_m: WithinM::try_from(0.05).unwrap(),
            }),
            Check::Broken(name("egg")),
            Check::Crushed(name("can")),
            Check::Reachable(Reachable {
                robot: RobotName::parse("alpha").unwrap(),
                target: Target::Object(name("crate")),
            }),
            Check::Control(Control {
                object: name("cooker"),
                id: "heat".into(),
                bounds: Bounds {
                    at_least: Some(2.0),
                    at_most: None,
                },
            }),
            Check::All(vec![Judge::True]),
            Check::Any(vec![Judge::True]),
            Check::Not(Box::new(Judge::True)),
        ];
        assert_eq!(examples.each_ref().map(key_of), CHECKS);
        for check in examples {
            let written = serde_json::to_value(&check).unwrap();
            let keys: Vec<&String> = written.as_object().unwrap().keys().collect();
            assert_eq!(keys, [key_of(&check)], "{written}");
            let judge = Judge::Check(check);
            let text = Judge::list_to_json5(std::slice::from_ref(&judge)).unwrap();
            assert_eq!(Judge::parse_list(&text).unwrap(), [judge]);
        }
    }

    #[test]
    fn a_distance_is_above_0_and_nan_is_refused() {
        assert_eq!(WithinM::try_from(0.05).unwrap().metres(), 0.05);
        for refused in [0.0, -1.0, f64::NAN] {
            assert_eq!(
                WithinM::try_from(refused).unwrap_err().to_string(),
                format!("{refused} is not above 0; write a distance above 0 metres")
            );
        }
    }

    #[test]
    fn bounds_hold_inclusively() {
        let bounds = Bounds {
            at_least: Some(2),
            at_most: Some(4),
        };
        assert!(bounds.hold(2) && bounds.hold(4) && !bounds.hold(1) && !bounds.hold(5));
    }
}
