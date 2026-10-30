//! Samplers: values of a world pattern that a seed draws.
//!
//! The set of samplers is closed. `{ one_of: [v, ...] }` draws one of its
//! values and `{ range: [min, max] }` a number from `min` to `max`, in place
//! of any value of an entry but the join of a robot. `{ on: <surface>,
//! reachable_by: <robot> }` draws an object pose on a surface, and
//! `{ in: <region> }` an object pose or the position of a stand in a named
//! region; the simulation draws these two, in a model of the drawn world.
//!
//! An entry that holds a sampler keeps its value as written, with the path
//! of each sampler in it ([`Sampled`]). Its rules are those of the concrete
//! entry: the entry with each sampler replaced by each of its candidates
//! reads as a concrete entry, so one parser holds the rules of a value.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::{Map, Value};
use std::fmt;

use crate::{Error, Name, RobotName};

/// One step of the path of a value: a key of an object or an index of an
/// array.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Segment {
    Key(String),
    Index(usize),
}

/// The path of a value in a world, such as `objects.stock_2.pose` or
/// `lighting.key.color[0]`.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SitePath(Vec<Segment>);

impl SitePath {
    pub fn new(segments: Vec<Segment>) -> SitePath {
        SitePath(segments)
    }

    /// The path of the field `key` of the value at this path.
    pub fn key(&self, key: &str) -> SitePath {
        self.joined(&[Segment::Key(key.to_owned())])
    }

    /// The path of the value at `rest` under this path.
    pub fn joined(&self, rest: &[Segment]) -> SitePath {
        SitePath(self.0.iter().chain(rest).cloned().collect())
    }

    pub fn segments(&self) -> &[Segment] {
        &self.0
    }

    /// The keys of the path, an index read as no key.
    fn keys(&self) -> Vec<&str> {
        self.0
            .iter()
            .map(|segment| match segment {
                Segment::Key(key) => key.as_str(),
                Segment::Index(_) => "",
            })
            .collect()
    }
}

impl fmt::Display for SitePath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (index, segment) in self.0.iter().enumerate() {
            match segment {
                Segment::Key(key) if index == 0 => f.write_str(key)?,
                Segment::Key(key) => write!(f, ".{key}")?,
                Segment::Index(at) => write!(f, "[{at}]")?,
            }
        }
        Ok(())
    }
}

/// A surface that `on` names: a surface label, or a landmark of an object.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SurfaceRef {
    Label(String),
    Landmark { object: Name, landmark: String },
}

impl SurfaceRef {
    /// The object whose pose this surface follows, for a landmark.
    pub fn object(&self) -> Option<&Name> {
        match self {
            SurfaceRef::Label(_) => None,
            SurfaceRef::Landmark { object, .. } => Some(object),
        }
    }
}

/// A sampler of the closed set.
#[derive(Clone, Debug, PartialEq)]
pub enum Sampler {
    /// One of the values, with equal chances.
    OneOf(Vec<Value>),
    /// A uniform number from `min` to `max`; an integer, both ends included,
    /// for a count.
    Range { min: f64, max: f64 },
    /// An object pose on one of the surfaces, where `reachable_by` reaches
    /// and sees it when it names a robot.
    On {
        surfaces: Vec<SurfaceRef>,
        reachable_by: Option<RobotName>,
    },
    /// An object pose, or the position of a stand, in a named region.
    In { region: Name },
}

impl Sampler {
    /// Whether the simulation draws this sampler in a model of the drawn
    /// world, rather than the seed alone.
    pub fn is_placement(&self) -> bool {
        matches!(self, Sampler::On { .. } | Sampler::In { .. })
    }
}

/// What a value of an entry takes, by its path in the entry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SiteRule {
    /// `one_of` and `range`.
    Value,
    /// An object pose: `one_of`, `range`, `on` and `in`.
    Pose,
    /// The position of a stand: `one_of`, `range` and `in`.
    StandPosition,
    /// No sampler: the join of a robot.
    Fixed,
}

/// An entry of a world that a sampler can stand in, with the rule of each
/// of its paths.
pub(crate) trait Sampleable: DeserializeOwned {
    fn rule_at(path: &[&str]) -> SiteRule;
}

/// A sampler at its path in an entry.
#[derive(Clone, Debug, PartialEq)]
pub struct Site {
    pub path: SitePath,
    pub sampler: Sampler,
    rule: SiteRule,
}

/// An entry as written: a concrete value, or a value that holds samplers.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Sampled<T> {
    Fixed(T),
    Drawn(DrawnValue),
}

/// A value that holds samplers, as written, with the site of each.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DrawnValue {
    pub(crate) value: Value,
    pub(crate) sites: Vec<Site>,
}

impl<T: Serialize> Serialize for Sampled<T> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Sampled::Fixed(value) => value.serialize(serializer),
            Sampled::Drawn(drawn) => drawn.value.serialize(serializer),
        }
    }
}

impl<'de, T: Sampleable> Deserialize<'de> for Sampled<T> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Sampled<T>, D::Error> {
        let value = Value::deserialize(deserializer)?;
        Sampled::of(value).map_err(serde::de::Error::custom)
    }
}

impl<T: Sampleable> Sampled<T> {
    /// The entry `value` writes: concrete, or a value whose samplers each
    /// give a concrete entry with each of their candidates.
    pub(crate) fn of(value: Value) -> Result<Sampled<T>, Error> {
        let sites = sites_of::<T>(&value)?;
        if sites.is_empty() {
            return read::<T>(value).map(Sampled::Fixed);
        }
        let drawn = DrawnValue { value, sites };
        drawn.check::<T>()?;
        Ok(Sampled::Drawn(drawn))
    }
}

impl DrawnValue {
    /// Refuses a value that does not read as a `T` with each sampler at each
    /// of its candidates, the other samplers at their first.
    fn check<T: Sampleable>(&self) -> Result<(), Error> {
        self.candidate_values::<T>().map(|_| ())
    }

    /// The value read as a `T` with every sampler at its first candidate,
    /// then with each sampler at each of its other candidates, the others at
    /// their first.
    pub(crate) fn candidate_values<T: DeserializeOwned>(&self) -> Result<Vec<T>, Error> {
        let first = |site: &Site| site.candidates().swap_remove(0);
        let mut values = vec![self.concrete::<T>(&|site| Ok(first(site)))?];
        for site in &self.sites {
            for candidate in site.candidates().into_iter().skip(1) {
                let pick = |other: &Site| match other.path == site.path {
                    true => Ok(candidate.clone()),
                    false => Ok(first(other)),
                };
                values.push(self.concrete::<T>(&pick)?);
            }
        }
        Ok(values)
    }

    /// The concrete entry with each sampler replaced by the value `drawn`
    /// gives it.
    pub(crate) fn concrete<T: DeserializeOwned>(
        &self,
        drawn: &dyn Fn(&Site) -> Result<Value, Error>,
    ) -> Result<T, Error> {
        read(self.with_values(drawn)?)
    }

    /// The value with each sampler replaced by the value `drawn` gives it.
    pub(crate) fn with_values(
        &self,
        drawn: &dyn Fn(&Site) -> Result<Value, Error>,
    ) -> Result<Value, Error> {
        let mut value = self.value.clone();
        for site in &self.sites {
            *value_at(&mut value, site.path.segments()) = drawn(site)?;
        }
        Ok(value)
    }
}

impl Site {
    /// The values that the checks of an entry put in place of this sampler:
    /// each value of `one_of`, each end of `range`, and a pose or a position
    /// in place of a sampler that the simulation draws.
    fn candidates(&self) -> Vec<Value> {
        match &self.sampler {
            Sampler::OneOf(values) => values.clone(),
            Sampler::Range { min, max } => vec![Value::from(*min), Value::from(*max)],
            Sampler::On { .. } | Sampler::In { .. } => vec![self.placeholder()],
        }
    }

    /// A concrete value of the form a placement gives at this site.
    fn placeholder(&self) -> Value {
        match self.rule {
            SiteRule::StandPosition => serde_json::json!([0.0, 0.0, 0.0]),
            _ => serde_json::json!({ "position": [0.0, 0.0, 0.0], "yaw": 0.0 }),
        }
    }

    pub(crate) fn rule(&self) -> SiteRule {
        self.rule
    }
}

/// `value` read as a `T`, a refusal naming the path in the value.
fn read<T: DeserializeOwned>(value: Value) -> Result<T, Error> {
    serde_path_to_error::deserialize(value).map_err(|error| Error::rule(error.to_string()))
}

/// The value at `path` in `value`, which holds it.
fn value_at<'a>(value: &'a mut Value, path: &[Segment]) -> &'a mut Value {
    path.iter().fold(value, |value, segment| match segment {
        Segment::Key(key) => &mut value[key.as_str()],
        Segment::Index(at) => &mut value[*at],
    })
}

/// The sites of the samplers of `value`, an entry of the kind `T`, in the
/// order of their paths. Refuses a sampler that breaks the form of its kind,
/// a sampler where the entry takes none, and a sampler inside a value of
/// `one_of`.
pub(crate) fn sites_of<T: Sampleable>(value: &Value) -> Result<Vec<Site>, Error> {
    let mut sites = Vec::new();
    collect_sites(value, &mut Vec::new(), &T::rule_at, &mut sites)?;
    Ok(sites)
}

fn collect_sites(
    value: &Value,
    path: &mut Vec<Segment>,
    rule_at: &dyn Fn(&[&str]) -> SiteRule,
    sites: &mut Vec<Site>,
) -> Result<(), Error> {
    let site_path = SitePath(path.clone());
    let rule = rule_at(&site_path.keys());
    if let Some(sampler) = sampler_of(value, rule, &site_path)? {
        sites.push(Site {
            path: site_path,
            sampler,
            rule,
        });
        return Ok(());
    }
    match value {
        Value::Object(fields) => {
            for (key, field) in fields {
                path.push(Segment::Key(key.clone()));
                collect_sites(field, path, rule_at, sites)?;
                path.pop();
            }
        }
        Value::Array(items) => {
            for (at, item) in items.iter().enumerate() {
                path.push(Segment::Index(at));
                collect_sites(item, path, rule_at, sites)?;
                path.pop();
            }
        }
        _ => {}
    }
    Ok(())
}

/// The sampler `value` writes under `rule`, if it writes one.
fn sampler_of(value: &Value, rule: SiteRule, path: &SitePath) -> Result<Option<Sampler>, Error> {
    let Value::Object(fields) = value else {
        return Ok(None);
    };
    let keys: Vec<&str> = fields.keys().map(String::as_str).collect();
    let at = |reason: String| match path.segments().is_empty() {
        true => Error::rule(reason),
        false => Error::field(path.to_string(), reason),
    };
    let sampler = match (keys.as_slice(), rule) {
        (["one_of"] | ["range"], SiteRule::Fixed) => {
            return Err(at(
                "a join is fixed: a sweep checks the workflow against the join of each robot, so write the join without a sampler"
                    .to_owned(),
            ));
        }
        (["one_of"], _) => Sampler::OneOf(one_of(&fields["one_of"]).map_err(at)?),
        (["range"], _) => {
            let (min, max) = range(&fields["range"]).map_err(at)?;
            Sampler::Range { min, max }
        }
        (["in"], SiteRule::Pose | SiteRule::StandPosition) => Sampler::In {
            region: region(&fields["in"]).map_err(at)?,
        },
        (_, SiteRule::Pose) if fields.contains_key("on") => on(fields).map_err(at)?,
        _ => return Ok(None),
    };
    Ok(Some(sampler))
}

/// The values of `{ one_of }`: at least one, each with no sampler.
fn one_of(values: &Value) -> Result<Vec<Value>, String> {
    let form = "`one_of` takes a list of at least one value";
    let Value::Array(values) = values else {
        return Err(form.to_owned());
    };
    if values.is_empty() {
        return Err(form.to_owned());
    }
    for value in values {
        let mut nested = Vec::new();
        collect_sites(value, &mut Vec::new(), &|_| SiteRule::Value, &mut nested)
            .map_err(|error| error.to_string())?;
        if !nested.is_empty() {
            return Err(
                "a value of `one_of` holds no sampler; write each value in full".to_owned(),
            );
        }
    }
    Ok(values.clone())
}

/// The ends of `{ range: [min, max] }`, `min` at most `max`.
fn range(ends: &Value) -> Result<(f64, f64), String> {
    let form = "`range` takes [min, max], two numbers with min at most max";
    let ends = ends.as_array().filter(|ends| ends.len() == 2);
    let numbers = ends.and_then(|ends| Some((ends[0].as_f64()?, ends[1].as_f64()?)));
    match numbers {
        Some((min, max)) if min <= max => Ok((min, max)),
        _ => Err(form.to_owned()),
    }
}

fn region(name: &Value) -> Result<Name, String> {
    let text = name.as_str().ok_or("`in` names a region of the scene")?;
    Name::parse(text).map_err(|error| error.to_string())
}

/// `{ on, reachable_by }`: `on` a surface label, a landmark `{ object,
/// landmark }`, or `{ one_of }` of them.
fn on(fields: &Map<String, Value>) -> Result<Sampler, String> {
    let form = "`on` draws a pose on a surface: { on: <surface label or { object, landmark }>, reachable_by: <robot> }, and `on` takes `{ one_of }` of surfaces";
    let known = fields
        .keys()
        .all(|key| key == "on" || key == "reachable_by");
    if !known {
        return Err(form.to_owned());
    }
    let surface = |value: &Value| {
        serde_json::from_value::<SurfaceRef>(value.clone()).map_err(|_| form.to_owned())
    };
    let surfaces = match &fields["on"] {
        Value::Object(choice) if choice.contains_key("one_of") && choice.len() == 1 => {
            let Value::Array(values) = &choice["one_of"] else {
                return Err(form.to_owned());
            };
            if values.is_empty() {
                return Err(form.to_owned());
            }
            values.iter().map(surface).collect::<Result<_, _>>()?
        }
        value => vec![surface(value)?],
    };
    let reachable_by = fields
        .get("reachable_by")
        .map(|robot| {
            let text = robot.as_str().ok_or_else(|| form.to_owned())?;
            RobotName::parse(text).map_err(|error| error.to_string())
        })
        .transpose()?;
    Ok(Sampler::On {
        surfaces,
        reachable_by,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// An entry of numbers where each path takes `one_of` and `range`, but
    /// `fixed`, which takes none, and `pose`, an object pose.
    #[derive(Debug, Deserialize, PartialEq)]
    #[serde(deny_unknown_fields)]
    struct Entry {
        #[serde(default)]
        mass: Option<f64>,
        #[serde(default)]
        fixed: Option<String>,
        #[serde(default)]
        pose: Option<crate::Pose>,
    }

    impl Sampleable for Entry {
        fn rule_at(path: &[&str]) -> SiteRule {
            match path {
                ["fixed", ..] => SiteRule::Fixed,
                ["pose"] => SiteRule::Pose,
                _ => SiteRule::Value,
            }
        }
    }

    fn sampled(value: Value) -> Result<Sampled<Entry>, String> {
        Sampled::<Entry>::of(value).map_err(|error| error.to_string())
    }

    fn sites(value: Value) -> Vec<(String, Sampler)> {
        match sampled(value).unwrap() {
            Sampled::Drawn(drawn) => drawn
                .sites
                .into_iter()
                .map(|site| (site.path.to_string(), site.sampler))
                .collect(),
            Sampled::Fixed(_) => Vec::new(),
        }
    }

    #[test]
    fn a_value_with_no_sampler_is_fixed() {
        assert_eq!(
            sampled(json!({ "mass": 1.5 })).unwrap(),
            Sampled::Fixed(Entry {
                mass: Some(1.5),
                fixed: None,
                pose: None
            })
        );
    }

    #[test]
    fn each_sampler_is_found_at_its_path() {
        let found = sites(json!({
            "mass": { "range": [0.5, 2] },
            "pose": { "on": { "object": "conveyor", "landmark": "belt_top" }, "reachable_by": "alpha" },
        }));
        assert_eq!(
            found,
            [
                ("mass".to_owned(), Sampler::Range { min: 0.5, max: 2.0 }),
                (
                    "pose".to_owned(),
                    Sampler::On {
                        surfaces: vec![SurfaceRef::Landmark {
                            object: Name::parse("conveyor").unwrap(),
                            landmark: "belt_top".into()
                        }],
                        reachable_by: Some(RobotName::parse("alpha").unwrap()),
                    }
                ),
            ]
        );
        let nested =
            sites(json!({ "pose": { "position": [{ "one_of": [0, 1] }, 0, 0.4], "yaw": 0 } }));
        assert_eq!(nested[0].0, "pose.position[0]");
        let labels = sites(json!({ "pose": { "on": { "one_of": ["table top", "shelf"] } } }));
        assert_eq!(
            labels[0].1,
            Sampler::On {
                surfaces: vec![
                    SurfaceRef::Label("table top".into()),
                    SurfaceRef::Label("shelf".into())
                ],
                reachable_by: None
            }
        );
    }

    #[test]
    fn each_candidate_of_a_sampler_reads_as_the_concrete_entry() {
        assert!(
            sampled(json!({ "mass": { "one_of": [1, "heavy"] } }))
                .unwrap_err()
                .contains("mass: invalid type: string \"heavy\", expected f64"),
        );
        assert!(
            sampled(json!({ "mass": { "range": [0, 1] }, "typo": 1 }))
                .unwrap_err()
                .contains("unknown field `typo`")
        );
    }

    #[test]
    fn a_malformed_sampler_or_a_sampler_where_none_stands_is_refused() {
        for (value, refusal) in [
            (
                json!({ "mass": { "range": [2, 1] } }),
                "mass: `range` takes [min, max], two numbers with min at most max",
            ),
            (
                json!({ "mass": { "range": [1] } }),
                "mass: `range` takes [min, max], two numbers with min at most max",
            ),
            (
                json!({ "mass": { "one_of": [] } }),
                "mass: `one_of` takes a list of at least one value",
            ),
            (
                json!({ "mass": { "one_of": [{ "range": [0, 1] }] } }),
                "mass: a value of `one_of` holds no sampler; write each value in full",
            ),
            (
                json!({ "fixed": { "one_of": ["a", "b"] } }),
                "fixed: a join is fixed: a sweep checks the workflow against the join of each robot, so write the join without a sampler",
            ),
            (
                json!({ "pose": { "on": "table", "near": 1 } }),
                "pose: `on` draws a pose on a surface",
            ),
            (
                json!({ "pose": { "in": "Box Stock" } }),
                "pose: the name `Box Stock` is not lowercase letters, digits and `_`",
            ),
        ] {
            let refused = sampled(value.clone()).unwrap_err();
            assert!(refused.starts_with(refusal), "{value}: {refused}");
        }
    }

    #[test]
    fn on_and_in_are_samplers_only_where_a_pose_or_a_stand_position_stands() {
        assert!(
            sampled(json!({ "mass": { "in": "box_stock" } }))
                .unwrap_err()
                .contains("mass: invalid type: map, expected f64")
        );
    }

    #[test]
    fn a_drawn_value_gives_the_concrete_entry_of_the_values_drawn() {
        let Sampled::Drawn(drawn) =
            sampled(json!({ "mass": { "range": [0.5, 2] }, "pose": { "in": "box_stock" } }))
                .unwrap()
        else {
            panic!("the value holds samplers");
        };
        let entry: Entry = drawn
            .concrete(&|site| match site.path.to_string().as_str() {
                "mass" => Ok(json!(1.25)),
                _ => Ok(json!({ "position": [1, 2, 0], "yaw": 0.5 })),
            })
            .unwrap();
        assert_eq!(entry.mass, Some(1.25));
        assert_eq!(
            entry.pose,
            Some(crate::Pose::Yawed {
                position: [1.0, 2.0, 0.0],
                yaw: 0.5
            })
        );
    }
}
