//! World patterns: worlds with samplers, which a seed expands into concrete
//! worlds.
//!
//! A world pattern is a world whose entries can hold samplers
//! ([`crate::sampler`]), with three more fields. An entry of `objects` with
//! an `item_kind` adds `<name>_1` up to `<name>_<count>`, each with an asset
//! of that item kind. `work_cell_parameters` says how to measure each
//! quantity of the work cell that an operator can measure in a real one, and
//! `min_spacing_m` is the least distance between sampled objects.
//!
//! An [`Expansion`] of a pattern and a seed draws every sampler that the
//! seed alone draws: `one_of`, `range`, the count and the asset of each
//! object of an item kind. The simulation draws each [`PlacementSite`], a stand
//! or a pose `on` a surface or `in` a region, in a model of the drawn world,
//! in the order of [`Expansion::placements`]: before each, it builds its
//! model from [`Placements::world_so_far`]. [`Expansion::world`] then gives
//! the concrete world.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::{Map, Value};
use std::collections::{BTreeMap, BTreeSet};

use crate::daylight::Daylight;
use crate::geometry::Region;
use crate::sampler::{
    DrawnValue, Sampleable, Sampled, Sampler, Segment, Site, SitePath, SiteRule, SurfaceRef,
    placeholder,
};
use crate::stream::{Seed, Stream};
use crate::world::{
    Entry, Light, Location, Material, Object, Origin, Robot, Scene, SceneObject, WORLD,
    WorldFields, check_name_rules, no_such_region,
};
use crate::{
    Catalogue, Error, Join, Manifest, Name, ParameterName, Physics, RobotName, World,
    WorldDocument, text,
};
use documents_core::Document;

/// The most objects that an entry of an item kind adds.
pub const MAX_COUNT: u32 = 1000;

impl Sampleable for Scene {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

/// A robot takes samplers in its stand alone: a sweep checks the workflow
/// against the join of each robot, so the join is fixed.
impl Sampleable for Robot {
    fn rule_at(path: &[&str]) -> SiteRule {
        match path {
            ["stand", "position"] => SiteRule::StandPosition,
            ["stand", ..] => SiteRule::Value,
            _ => SiteRule::Fixed,
        }
    }
}

impl Sampleable for SceneObject {
    fn rule_at(path: &[&str]) -> SiteRule {
        match path {
            ["pose"] => SiteRule::Pose,
            _ => SiteRule::Value,
        }
    }
}

impl Sampleable for Object {
    fn rule_at(path: &[&str]) -> SiteRule {
        match path {
            ["pose"] => SiteRule::Pose,
            _ => SiteRule::Value,
        }
    }
}

impl Sampleable for Light {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

impl Sampleable for Material {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

impl Sampleable for Daylight {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

impl Sampleable for Location {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

impl Sampleable for Region {
    fn rule_at(_: &[&str]) -> SiteRule {
        SiteRule::Value
    }
}

/// The least distance between sampled objects, metres, at least 0.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "f64", into = "f64")]
pub struct MinSpacing(f64);

impl TryFrom<f64> for MinSpacing {
    type Error = Error;

    fn try_from(metres: f64) -> Result<MinSpacing, Error> {
        let at_least_zero = metres >= 0.0;
        if !at_least_zero {
            return Err(Error::rule(format!(
                "the spacing {metres} is not at least 0; write a distance of 0 m or more"
            )));
        }
        Ok(MinSpacing(metres))
    }
}

impl From<MinSpacing> for f64 {
    fn from(spacing: MinSpacing) -> f64 {
        spacing.0
    }
}

/// How a world pattern measures a quantity of the work cell, with its unit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ParameterFields", into = "ParameterFields")]
pub struct WorkCellParameter {
    pub unit: String,
    pub measure: Measure,
}

/// What a parameter measures.
#[derive(Clone, Debug, PartialEq)]
pub enum Measure {
    /// A control value of an object.
    Control { object: Name, id: String },
    /// The distance from the base of each robot of `per_robot` to a surface
    /// label or a landmark, along an axis of the robot: one value per robot.
    /// Without `per_robot`, from the base of the one robot of the world with
    /// a body.
    Distance {
        surface: SurfaceRef,
        axis: Axis,
        per_robot: Option<BTreeSet<RobotName>>,
    },
}

/// An axis of the frame of a robot base.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    Up,
    Ahead,
    Side,
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a work cell parameter, { unit, control, distance, per_robot }"
)]
struct ParameterFields {
    unit: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    control: Option<ControlFields>,
    #[serde(skip_serializing_if = "Option::is_none")]
    distance: Option<DistanceFields>,
    #[serde(skip_serializing_if = "Option::is_none")]
    per_robot: Option<Vec<RobotName>>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a control, { object, id }")]
struct ControlFields {
    object: Name,
    id: String,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a distance, { surface, axis }")]
struct DistanceFields {
    surface: SurfaceRef,
    axis: Axis,
}

impl TryFrom<ParameterFields> for WorkCellParameter {
    type Error = Error;

    fn try_from(fields: ParameterFields) -> Result<WorkCellParameter, Error> {
        if fields.unit.trim().is_empty() {
            return Err(Error::rule(
                "the unit is blank; name the unit of the value, such as `m`",
            ));
        }
        let measure = match (fields.control, fields.distance, fields.per_robot) {
            (Some(ControlFields { object, id }), None, None) => Measure::Control { object, id },
            (None, Some(DistanceFields { surface, axis }), per_robot) => Measure::Distance {
                surface,
                axis,
                per_robot: per_robot.map(robots_named_once).transpose()?,
            },
            _ => {
                return Err(Error::rule(
                    "a work cell parameter measures a `control` or a `distance`, and `per_robot` is for a distance",
                ));
            }
        };
        Ok(WorkCellParameter {
            unit: fields.unit,
            measure,
        })
    }
}

impl From<WorkCellParameter> for ParameterFields {
    fn from(parameter: WorkCellParameter) -> ParameterFields {
        let (control, distance, per_robot) = match parameter.measure {
            Measure::Control { object, id } => (Some(ControlFields { object, id }), None, None),
            Measure::Distance {
                surface,
                axis,
                per_robot,
            } => (
                None,
                Some(DistanceFields { surface, axis }),
                per_robot.map(|robots| robots.into_iter().collect()),
            ),
        };
        ParameterFields {
            unit: parameter.unit,
            control,
            distance,
            per_robot,
        }
    }
}

/// The work cell parameters of a world pattern, as `measure` of
/// simulation_world takes them: JSON5 text of an object from each name to
/// its parameter.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct WorkCellParameters(pub BTreeMap<ParameterName, WorkCellParameter>);

impl WorkCellParameters {
    pub fn parse(text: &str) -> Result<WorkCellParameters, Error> {
        text::from_json5(text)
    }

    pub fn to_json5(&self) -> Result<String, Error> {
        text::to_json5(self)
    }
}

/// The value that a simulation measured for a work cell parameter in its
/// live world: one value, or one value per robot of its `per_robot`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Measurement {
    Single(f64),
    PerRobot(BTreeMap<RobotName, f64>),
}

/// The value of each work cell parameter, as `measure` of simulation_world
/// gives them: JSON5 text of an object from each name to its measurement.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Measurements(pub BTreeMap<ParameterName, Measurement>);

impl Measurements {
    pub fn parse(text: &str) -> Result<Measurements, Error> {
        text::from_json5(text)
    }

    pub fn to_json5(&self) -> Result<String, Error> {
        text::to_json5(self)
    }

    /// Refuses measurements that are not those of `parameters`: a parameter
    /// with no value, a value of no parameter, and a value of another shape
    /// than its parameter, one per robot of `per_robot` or one without it.
    pub fn check(&self, parameters: &WorkCellParameters) -> Result<(), Error> {
        for name in self.0.keys() {
            if !parameters.0.contains_key(name) {
                return Err(Error::field(name.to_string(), "is no work cell parameter"));
            }
        }
        for (name, parameter) in &parameters.0 {
            let per_robot = match &parameter.measure {
                Measure::Distance { per_robot, .. } => per_robot.as_ref(),
                Measure::Control { .. } => None,
            };
            let fits = match (self.0.get(name), per_robot) {
                (None, _) => false,
                (Some(Measurement::Single(_)), None) => true,
                (Some(Measurement::PerRobot(values)), Some(robots)) => {
                    values.keys().eq(robots.iter())
                }
                _ => false,
            };
            if !fits {
                let shape = match per_robot {
                    Some(robots) => {
                        let names: Vec<String> =
                            robots.iter().map(|robot| format!("`{robot}`")).collect();
                        format!(
                            "one value for each robot of its `per_robot`: {}",
                            names.join(", ")
                        )
                    }
                    None => "one value".to_owned(),
                };
                return Err(Error::field(
                    name.to_string(),
                    format!("has no measurement of its shape; the parameter takes {shape}"),
                ));
            }
        }
        Ok(())
    }
}

/// The robots of `per_robot`, at least one, each named once.
fn robots_named_once(robots: Vec<RobotName>) -> Result<BTreeSet<RobotName>, Error> {
    if robots.is_empty() {
        return Err(Error::rule(
            "`per_robot` names at least one robot; leave it out for the one robot with a body",
        ));
    }
    let mut once = BTreeSet::new();
    for robot in robots {
        if let Some(twice) = once.replace(robot) {
            return Err(Error::rule(format!(
                "`per_robot` names the robot `{twice}` twice; name each robot once"
            )));
        }
    }
    Ok(once)
}

/// An entry of `objects`: one object, or the objects of an item kind.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ObjectEntry {
    Object(Sampled<Object>),
    ItemKind(ItemKindEntry),
}

impl Serialize for ObjectEntry {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            ObjectEntry::Object(object) => object.serialize(serializer),
            ObjectEntry::ItemKind(entry) => entry.written.serialize(serializer),
        }
    }
}

/// An object that holds the key `item_kind` is an entry of an item kind.
impl<'de> Deserialize<'de> for ObjectEntry {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<ObjectEntry, D::Error> {
        let value = Value::deserialize(deserializer)?;
        let entry = match value {
            Value::Object(fields) if fields.contains_key("item_kind") => {
                ItemKindEntry::of(fields).map(ObjectEntry::ItemKind)
            }
            value => Sampled::of(value).map(ObjectEntry::Object),
        };
        entry.map_err(serde::de::Error::custom)
    }
}

/// The objects of an item kind that an entry adds: how many, and what each
/// one is but its asset, which each draws from the assets of the item kind.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ItemKindEntry {
    pub(crate) item_kind: String,
    pub(crate) count: Count,
    /// Each object, with no asset.
    pub(crate) template: Sampled<Object>,
    written: Value,
}

impl ItemKindEntry {
    fn of(fields: Map<String, Value>) -> Result<ItemKindEntry, Error> {
        let written = Value::Object(fields.clone());
        let mut template = fields;
        let item_kind = match template.remove("item_kind") {
            Some(Value::String(kind)) if !kind.trim().is_empty() => kind,
            _ => {
                return Err(Error::field(
                    "item_kind",
                    "names an item kind of the catalogue, such as `carton`",
                ));
            }
        };
        if template.contains_key("asset") {
            return Err(Error::field(
                "asset",
                "an entry of an item kind draws the asset of each object from its item kind; leave `asset` out",
            ));
        }
        let count = template
            .remove("count")
            .ok_or_else(|| Error::field("count", COUNT_FORM))
            .and_then(|count| Count::of(&count))?;
        template.insert("asset".to_owned(), Value::String(String::new()));
        Ok(ItemKindEntry {
            item_kind,
            count,
            template: Sampled::of(Value::Object(template))?,
            written,
        })
    }

    /// The name of the object number `index`, from 1.
    fn object_name(entry: &Name, index: u32) -> Name {
        Name::parse(&format!("{entry}_{index}")).expect("a name, `_` and digits is a name")
    }
}

const COUNT_FORM: &str = "the count is a number of objects from 0 to 1000, `{ range: [min, max] }` or `{ one_of: [...] }` of such numbers";

/// How many objects an entry of an item kind adds.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Count {
    Fixed(u32),
    /// From `min` to `max`, both included.
    Range(u32, u32),
    OneOf(Vec<u32>),
}

impl Count {
    fn of(value: &Value) -> Result<Count, Error> {
        let number = |value: &Value| {
            value
                .as_u64()
                .filter(|count| *count <= u64::from(MAX_COUNT))
                .map(|count| count as u32)
        };
        let sampler = value.as_object().filter(|fields| fields.len() == 1);
        let count = match (value, sampler) {
            (Value::Number(_), _) => number(value).map(Count::Fixed),
            (_, Some(fields)) if fields.contains_key("range") => fields["range"]
                .as_array()
                .filter(|ends| ends.len() == 2)
                .and_then(|ends| Some((number(&ends[0])?, number(&ends[1])?)))
                .filter(|(min, max)| min <= max)
                .map(|(min, max)| Count::Range(min, max)),
            (_, Some(fields)) if fields.contains_key("one_of") => fields["one_of"]
                .as_array()
                .filter(|counts| !counts.is_empty())
                .and_then(|counts| counts.iter().map(number).collect::<Option<Vec<u32>>>())
                .map(Count::OneOf),
            _ => None,
        };
        count.ok_or_else(|| Error::field("count", COUNT_FORM))
    }

    /// The most objects the entry can add.
    pub(crate) fn most(&self) -> u32 {
        match self {
            Count::Fixed(count) => *count,
            Count::Range(_, max) => *max,
            Count::OneOf(counts) => counts.iter().copied().max().unwrap_or(0),
        }
    }

    fn draw(&self, stream: &mut Stream) -> u32 {
        match self {
            Count::Fixed(count) => *count,
            Count::Range(min, max) => stream.integer(u64::from(*min), u64::from(*max)) as u32,
            Count::OneOf(counts) => counts[stream.index(counts.len())],
        }
    }
}

/// A world pattern with its bases folded in.
#[derive(Clone, Debug, PartialEq)]
pub struct WorldPattern {
    pub manifest: Manifest,
    pub catalogue: Catalogue,
    pub work_cell_parameters: WorkCellParameters,
    /// The least distance between sampled objects, metres.
    pub min_spacing_m: f64,
    fields: WorldFields,
    origin: Origin,
}

impl WorldPattern {
    /// Parses JSON5 text as a world pattern that extends no base.
    pub fn parse(text: &str) -> Result<WorldPattern, Error> {
        WorldDocument::parse(text)?.resolve_pattern(&[])
    }

    /// The pattern of fields whose bases are folded in. Refuses a missing
    /// catalogue or scene, a mark with no base entry, an entry of an item
    /// kind that adds the name of another entry or of a scene object, a
    /// `reachable_by` that names no robot with a body, and a distance that
    /// names no robot with a body or, without `per_robot`, a pattern with
    /// other than one robot with a body.
    pub(crate) fn of_fields(fields: WorldFields, origin: Origin) -> Result<WorldPattern, Error> {
        let (catalogue, _) = fields.catalogue_and_scene(origin)?;
        let marked = fields.marked_maps(origin)?;
        for (name, entry) in &marked.objects {
            let ObjectEntry::ItemKind(entry) = entry else {
                continue;
            };
            for index in 1..=entry.count.most() {
                let added = ItemKindEntry::object_name(name, index);
                let taken_by = match () {
                    _ if fields.objects.contains_key(&added) => "another entry of `objects`",
                    _ if fields.scene_objects.contains_key(&added) => "a scene object",
                    _ => continue,
                };
                return Err(Error::field(
                    format!("objects.{name}"),
                    format!(
                        "adds `{added}`, which is the name of {taken_by}; rename one of the two"
                    ),
                ));
            }
        }
        let with_a_body: BTreeSet<&RobotName> = marked
            .robots
            .iter()
            .filter(|(_, robot)| stands(robot))
            .map(|(name, _)| *name)
            .collect();
        let pattern = WorldPattern {
            manifest: fields.manifest.clone(),
            catalogue,
            work_cell_parameters: WorkCellParameters(
                marked
                    .work_cell_parameters
                    .iter()
                    .map(|(name, parameter)| ((*name).clone(), (*parameter).clone()))
                    .collect(),
            ),
            min_spacing_m: fields.min_spacing_m.map_or(0.0, f64::from),
            fields: fields.clone(),
            origin,
        };
        for (name, location) in &marked.locations {
            let candidates = match location {
                Sampled::Fixed(location) => vec![location.clone()],
                Sampled::Drawn(drawn) => drawn.candidate_values::<Location>()?,
            };
            for candidate in candidates {
                if let Location::In(region) = &candidate
                    && !marked.regions.contains_key(region)
                {
                    return Err(no_such_region(format!("locations.{name}"), region));
                }
            }
        }
        for (root, site) in pattern.entry_sites(None) {
            if let Sampler::In { region } = &site.sampler
                && !marked.regions.contains_key(region)
            {
                return Err(no_such_region(
                    root.joined(site.path.segments()).to_string(),
                    region,
                ));
            }
            let Sampler::On {
                reachable_by: Some(robot),
                ..
            } = &site.sampler
            else {
                continue;
            };
            if !with_a_body.contains(robot) {
                return Err(Error::field(
                    root.joined(site.path.segments()).to_string(),
                    format!(
                        "`reachable_by` names `{robot}`, which is no robot of the world with a stand"
                    ),
                ));
            }
        }
        for (name, parameter) in &pattern.work_cell_parameters.0 {
            let Measure::Distance { per_robot, .. } = &parameter.measure else {
                continue;
            };
            let at = format!("work_cell_parameters.{name}");
            match per_robot {
                Some(robots) => {
                    if let Some(robot) = robots.iter().find(|robot| !with_a_body.contains(robot)) {
                        return Err(Error::field(
                            at,
                            format!(
                                "`per_robot` names `{robot}`, which is no robot of the world with a stand"
                            ),
                        ));
                    }
                }
                None if with_a_body.len() != 1 => {
                    return Err(Error::field(
                        at,
                        format!(
                            "measures from the one robot with a stand, and the world has {}; name the robots in `per_robot`",
                            with_a_body.len()
                        ),
                    ));
                }
                None => {}
            }
        }
        Ok(pattern)
    }

    /// The pattern as JSON5 text of a world document that extends no base:
    /// the fields of its bases folded in, a scene object that a base keeps
    /// left out. It reads back as this pattern.
    pub fn to_json5(&self) -> Result<String, Error> {
        let mut fields = self.fields.clone();
        fields
            .scene_objects
            .retain(|_, entry| !matches!(entry, Entry::Kept));
        Ok(Document::of(WORLD, &fields)?.to_json5())
    }

    /// Whether every seed gives the same world: no sampler and no entry of
    /// an item kind.
    pub fn is_concrete(&self) -> bool {
        let no_item_kind = self
            .fields
            .objects
            .values()
            .all(|entry| !matches!(entry, Entry::Value(ObjectEntry::ItemKind(_))));
        no_item_kind && self.entry_sites(None).is_empty()
    }

    /// The robots of the pattern, each with its join.
    pub fn joins(&self) -> BTreeMap<RobotName, Join> {
        let marked = self
            .fields
            .marked_maps(self.origin)
            .expect("the marks of a pattern are resolved");
        marked
            .robots
            .into_iter()
            .map(|(name, robot)| {
                let join = match robot {
                    Sampled::Fixed(robot) => robot.join.clone(),
                    Sampled::Drawn(drawn) => serde_json::from_value(drawn.value["join"].clone())
                        .expect("the join of a robot holds no sampler and reads"),
                };
                (name.clone(), join)
            })
            .collect()
    }

    /// Each physics that the object `name` of the concrete world can take:
    /// `None` in the set for the physics of the catalogue, and no set for a
    /// name that the pattern gives no object.
    pub fn physics_of(&self, name: &Name) -> Option<BTreeSet<Option<Physics>>> {
        let entry = self.fields.objects.iter().find_map(|(key, entry)| {
            let Entry::Value(entry) = entry else {
                return None;
            };
            match entry {
                ObjectEntry::Object(object) if key == name => Some(object),
                ObjectEntry::ItemKind(kind)
                    if (1..=kind.count.most())
                        .any(|index| ItemKindEntry::object_name(key, index) == *name) =>
                {
                    Some(&kind.template)
                }
                _ => None,
            }
        })?;
        let physics = match entry {
            Sampled::Fixed(object) => BTreeSet::from([object.physics]),
            Sampled::Drawn(drawn) => drawn
                .candidate_values::<Object>()
                .expect("the candidates of a pattern read")
                .into_iter()
                .map(|object| object.physics)
                .collect(),
        };
        Some(physics)
    }

    /// An expansion of the pattern with `seed`: each sampler that the seed
    /// alone draws, drawn. `assets_of_kind` gives the assets of the catalogue
    /// of an item kind; an entry of an item kind with no asset is refused.
    pub fn expansion(
        &self,
        seed: Seed,
        assets_of_kind: &dyn Fn(&str) -> Vec<String>,
    ) -> Result<Expansion<'_>, Error> {
        let mut added = BTreeMap::new();
        for (name, entry) in self.item_kind_entries() {
            let count = entry
                .count
                .draw(&mut Stream::new(seed, &objects_path(name).key("count")));
            let mut assets = assets_of_kind(&entry.item_kind);
            assets.sort();
            assets.dedup();
            if assets.is_empty() && count > 0 {
                return Err(no_asset_of_kind(name, &entry.item_kind));
            }
            for index in 1..=count {
                let object = ItemKindEntry::object_name(name, index);
                let mut stream = Stream::new(seed, &objects_path(&object).key("asset"));
                let asset = assets[stream.index(assets.len())].clone();
                added.insert(
                    object,
                    Added {
                        entry: name.clone(),
                        asset,
                    },
                );
            }
        }
        let mut drawn = BTreeMap::new();
        for (root, site) in self.entry_sites(Some(&added)) {
            let path = root.joined(site.path.segments());
            let mut stream = Stream::new(seed, &path);
            let value = match &site.sampler {
                Sampler::OneOf(values) => values[stream.index(values.len())].clone(),
                Sampler::Range { min, max } => Value::from(stream.uniform(*min, *max)),
                Sampler::On { .. } | Sampler::In { .. } => continue,
            };
            drawn.insert(path, value);
        }
        Ok(Expansion {
            pattern: self,
            seed,
            drawn,
            added,
        })
    }

    /// The worlds that check each value that a sampler of the pattern can
    /// draw, but its placements: one world with each sampler at its first
    /// value (the first of `one_of`, the minimum of `range`), then, for each
    /// sampler and each of its other values, the world with that sampler at
    /// that value and every other one at its first. A sampler of the
    /// template of an entry of an item kind takes its value in each object
    /// that the entry adds together. Each entry of an item kind adds its
    /// largest count of objects, each with the first asset of its kind in
    /// the order of their names, and for each other asset of a kind one more
    /// world gives that asset to the objects of its entries. Each placement
    /// stands at the origin, with yaw 0,
    /// where an expansion has the simulation draw it. Each world is named as
    /// the expansion of seed 0 names its world. `assets_of_kind` is that of
    /// [`WorldPattern::expansion`]; an entry of an item kind with no asset is
    /// refused.
    pub fn candidate_worlds(
        &self,
        assets_of_kind: &dyn Fn(&str) -> Vec<String>,
    ) -> Result<Vec<World>, Error> {
        let mut assets_of_entry = BTreeMap::new();
        for (name, entry) in self.item_kind_entries() {
            let mut assets = assets_of_kind(&entry.item_kind);
            assets.sort();
            assets.dedup();
            if assets.is_empty() && entry.count.most() > 0 {
                return Err(no_asset_of_kind(name, &entry.item_kind));
            }
            assets_of_entry.insert(name, (entry.count.most(), assets));
        }
        let added_with = |asset_of: &dyn Fn(&Name, &[String]) -> String| {
            let mut added = BTreeMap::new();
            for (name, (count, assets)) in &assets_of_entry {
                for index in 1..=*count {
                    let added_entry = Added {
                        entry: (*name).clone(),
                        asset: asset_of(name, assets),
                    };
                    added.insert(ItemKindEntry::object_name(name, index), added_entry);
                }
            }
            added
        };
        let first_assets = added_with(&|_, assets| assets[0].clone());
        // The samplers by the value that one choice sets: a sampler of the
        // template of an entry of an item kind under the path of its entry.
        let mut choices: BTreeMap<String, Vec<Value>> = BTreeMap::new();
        let mut sites: Vec<(SitePath, String, Vec<Value>)> = Vec::new();
        for (root, site) in self.entry_sites(Some(&first_assets)) {
            if site.sampler.is_placement() {
                continue;
            }
            let choice = match root.segments() {
                [Segment::Key(field), Segment::Key(object)] if field == "objects" => {
                    let entry = Name::parse(object)
                        .ok()
                        .and_then(|object| first_assets.get(&object))
                        .map_or_else(|| object.clone(), |added| added.entry.to_string());
                    format!("objects.{entry}.{}", site.path)
                }
                _ => root.joined(site.path.segments()).to_string(),
            };
            choices.insert(choice.clone(), site.candidates());
            sites.push((root.joined(site.path.segments()), choice, site.candidates()));
        }
        let drawn_with = |chosen: Option<(&str, usize)>| -> BTreeMap<SitePath, Value> {
            sites
                .iter()
                .map(|(path, choice, candidates)| {
                    let index = match chosen {
                        Some((picked, index)) if picked == choice => index,
                        _ => 0,
                    };
                    (path.clone(), candidates[index].clone())
                })
                .collect()
        };
        let world_of = |drawn, added| {
            let expansion = Expansion {
                pattern: self,
                seed: Seed(0),
                drawn,
                added,
            };
            expansion.placements(&|_| None)?.placeholder_world()
        };
        let mut worlds = vec![world_of(drawn_with(None), first_assets.clone())?];
        for (choice, candidates) in &choices {
            for index in 1..candidates.len() {
                worlds.push(world_of(
                    drawn_with(Some((choice, index))),
                    first_assets.clone(),
                )?);
            }
        }
        let most_assets = assets_of_entry
            .values()
            .map(|(_, assets)| assets.len())
            .max()
            .unwrap_or(0);
        for index in 1..most_assets {
            let added = added_with(&|_, assets| assets[index.min(assets.len() - 1)].clone());
            worlds.push(world_of(drawn_with(None), added)?);
        }
        Ok(worlds)
    }

    /// Each entry of `objects` that adds objects of an item kind, by its
    /// name, with that item kind, in the order of the names.
    pub fn item_kinds(&self) -> impl Iterator<Item = (&Name, &str)> {
        self.item_kind_entries()
            .map(|(name, entry)| (name, entry.item_kind.as_str()))
    }

    fn item_kind_entries(&self) -> impl Iterator<Item = (&Name, &ItemKindEntry)> {
        self.fields
            .objects
            .iter()
            .filter_map(|(name, entry)| match entry {
                Entry::Value(ObjectEntry::ItemKind(entry)) => Some((name, entry)),
                _ => None,
            })
    }

    /// Each entry that holds samplers, by the path of its value: in the
    /// concrete world of an expansion, with the objects that `added` adds in
    /// place of their entry of an item kind, or in the pattern with no
    /// expansion, the entry of an item kind at its own path.
    fn drawn_entries<'a>(
        &'a self,
        added: Option<&BTreeMap<Name, Added>>,
    ) -> Vec<(SitePath, &'a DrawnValue)> {
        let mut entries = Vec::new();
        let root = |key: &str| SitePath::new(vec![Segment::Key(key.to_owned())]);
        if let Some(Sampled::Drawn(scene)) = &self.fields.scene {
            entries.push((root("scene"), scene));
        }
        let fields = &self.fields;
        push_drawn(&mut entries, "robots", &fields.robots);
        push_drawn(&mut entries, "scene_objects", &fields.scene_objects);
        for (name, entry) in &fields.objects {
            match entry {
                Entry::Value(ObjectEntry::Object(Sampled::Drawn(drawn))) => {
                    entries.push((objects_path(name), drawn));
                }
                Entry::Value(ObjectEntry::ItemKind(kind)) => {
                    let Sampled::Drawn(drawn) = &kind.template else {
                        continue;
                    };
                    let Some(added) = added else {
                        entries.push((objects_path(name), drawn));
                        continue;
                    };
                    let objects = added.iter().filter(|(_, object)| object.entry == *name);
                    entries.extend(objects.map(|(object, _)| (objects_path(object), drawn)));
                }
                _ => {}
            }
        }
        push_drawn(&mut entries, "lighting", &fields.lighting);
        push_drawn(&mut entries, "materials", &fields.materials);
        if let Some(Entry::Value(Sampled::Drawn(daylight))) = &fields.daylight {
            entries.push((root("daylight"), daylight));
        }
        push_drawn(&mut entries, "regions", &fields.regions);
        push_drawn(&mut entries, "locations", &fields.locations);
        entries
    }

    /// Each sampler of [`WorldPattern::drawn_entries`], with the path of its
    /// entry.
    fn entry_sites<'a>(
        &'a self,
        added: Option<&BTreeMap<Name, Added>>,
    ) -> Vec<(SitePath, &'a Site)> {
        self.drawn_entries(added)
            .into_iter()
            .flat_map(|(root, drawn)| drawn.sites.iter().map(move |site| (root.clone(), site)))
            .collect()
    }
}

/// Pushes each entry of `entries` that holds samplers, under `field`.
fn push_drawn<'a, K: std::fmt::Display, T>(
    out: &mut Vec<(SitePath, &'a DrawnValue)>,
    field: &str,
    entries: &'a BTreeMap<K, Entry<Sampled<T>>>,
) {
    for (key, entry) in entries {
        if let Entry::Value(Sampled::Drawn(drawn)) = entry {
            let path = SitePath::new(vec![
                Segment::Key(field.to_owned()),
                Segment::Key(key.to_string()),
            ]);
            out.push((path, drawn));
        }
    }
}

fn objects_path(name: &Name) -> SitePath {
    SitePath::new(vec![
        Segment::Key("objects".to_owned()),
        Segment::Key(name.to_string()),
    ])
}

/// Whether a robot entry stands the robot: a robot with a body has a stand.
fn stands(robot: &Sampled<Robot>) -> bool {
    match robot {
        Sampled::Fixed(robot) => robot.stand.is_some(),
        Sampled::Drawn(drawn) => !drawn.value["stand"].is_null(),
    }
}

/// An object that an entry of an item kind adds.
#[derive(Clone, Debug, PartialEq)]
struct Added {
    entry: Name,
    asset: String,
}

/// A world pattern with the samplers that its seed alone draws, drawn.
#[derive(Clone, Debug)]
pub struct Expansion<'p> {
    pattern: &'p WorldPattern,
    seed: Seed,
    /// The value of each sampler that the seed alone draws, by its path in
    /// the concrete world.
    drawn: BTreeMap<SitePath, Value>,
    added: BTreeMap<Name, Added>,
}

/// A stand or a pose that the simulation draws or places in a model of the
/// drawn world.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementSite {
    /// The path of the value that the placement gives in the concrete world:
    /// a pose, the position of a stand, or a stand that draws nothing.
    pub path: SitePath,
    pub kind: PlacementKind,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PlacementKind {
    /// A pose of the object, drawn by its sampler: `on` or `in`.
    ObjectPose { object: Name, sampler: Sampler },
    /// The stand of the robot: its position drawn in a region, or its stand
    /// at the surface of an object placed before it.
    Stand { robot: RobotName, at: StandAt },
}

/// Where a placed stand is.
#[derive(Clone, Debug, PartialEq)]
pub enum StandAt {
    /// A position drawn in the region, with the yaw of the stand.
    Region(Name),
    /// The stand at the surface label, as written.
    Surface(String),
}

impl PlacementKind {
    /// A value of the form that the placement gives, at the origin, for a
    /// world that takes it before the simulation draws it; none for a stand
    /// at a surface, which draws nothing.
    fn placeholder(&self) -> Option<Value> {
        match self {
            PlacementKind::ObjectPose { .. } => Some(placeholder(SiteRule::Pose)),
            PlacementKind::Stand {
                at: StandAt::Region(_),
                ..
            } => Some(placeholder(SiteRule::StandPosition)),
            PlacementKind::Stand {
                at: StandAt::Surface(_),
                ..
            } => None,
        }
    }

    /// The draws without `reachable_by` come first, then the stands, then
    /// the draws with `reachable_by`, when no dependency sets the order.
    fn group(&self) -> u8 {
        match self {
            PlacementKind::ObjectPose {
                sampler:
                    Sampler::On {
                        reachable_by: Some(_),
                        ..
                    },
                ..
            } => 2,
            PlacementKind::ObjectPose { .. } => 0,
            PlacementKind::Stand { .. } => 1,
        }
    }
}

impl<'p> Expansion<'p> {
    pub fn seed(&self) -> Seed {
        self.seed
    }

    pub fn pattern(&self) -> &'p WorldPattern {
        self.pattern
    }

    /// The stream of the sampler at `path` in the concrete world, from which
    /// the simulation draws its placement.
    pub fn stream(&self, path: &SitePath) -> Stream {
        Stream::new(self.seed, path)
    }

    /// The drawn scene.
    pub fn scene(&self) -> Result<Scene, Error> {
        let (_, scene) = self
            .pattern
            .fields
            .catalogue_and_scene(self.pattern.origin)?;
        let root = SitePath::new(vec![Segment::Key("scene".to_owned())]);
        self.concrete(&root, scene, &BTreeMap::new())
    }

    /// The objects that the entries of an item kind add, each with its drawn
    /// asset.
    pub fn added_objects(&self) -> BTreeMap<&Name, &str> {
        self.added
            .iter()
            .map(|(name, added)| (name, added.asset.as_str()))
            .collect()
    }

    /// The placements of the expansion, in the order the simulation draws
    /// them. A placement comes after each placement it depends on: a stand at
    /// a surface after the pose of the object of that surface, a pose `on` a
    /// surface after the pose of its object, and a pose with `reachable_by`
    /// after the stand of its robot. `holder_of_surface` gives the object of
    /// a surface label in the drawn scene. With no dependency, the draws
    /// without `reachable_by` come first, then the stands, then the draws
    /// with `reachable_by`, each group in the order of the paths. Refuses a
    /// cycle of dependencies, naming its placements.
    pub fn placements(
        &self,
        holder_of_surface: &dyn Fn(&str) -> Option<Name>,
    ) -> Result<Placements<'_>, Error> {
        let mut placements = Vec::new();
        for (root, site) in self.pattern.entry_sites(Some(&self.added)) {
            if !site.sampler.is_placement() {
                continue;
            }
            let path = root.joined(site.path.segments());
            let kind = match (root.segments(), site.rule()) {
                ([_, Segment::Key(robot)], SiteRule::StandPosition) => {
                    let Sampler::In { region } = &site.sampler else {
                        unreachable!("a stand position takes `in` alone");
                    };
                    PlacementKind::Stand {
                        robot: RobotName::parse(robot).expect("a robot entry has a robot name"),
                        at: StandAt::Region(region.clone()),
                    }
                }
                ([_, Segment::Key(object)], _) => PlacementKind::ObjectPose {
                    object: Name::parse(object).expect("an object entry has a name"),
                    sampler: site.sampler.clone(),
                },
                _ => unreachable!("a placement stands in an entry of a map"),
            };
            placements.push(PlacementSite { path, kind });
        }
        let placed_objects: BTreeSet<Name> = placements
            .iter()
            .filter_map(|placement| match &placement.kind {
                PlacementKind::ObjectPose { object, .. } => Some(object.clone()),
                PlacementKind::Stand { .. } => None,
            })
            .collect();
        let robots_path = SitePath::new(vec![Segment::Key("robots".to_owned())]);
        let mut holders = BTreeMap::new();
        for (robot, surface) in self.stands_at_surfaces()? {
            let holder =
                holder_of_surface(&surface).filter(|holder| placed_objects.contains(holder));
            if let Some(holder) = holder {
                holders.insert(robot.clone(), holder);
                placements.push(PlacementSite {
                    path: robots_path.key(robot.as_str()).key("stand"),
                    kind: PlacementKind::Stand {
                        robot,
                        at: StandAt::Surface(surface),
                    },
                });
            }
        }
        Ok(Placements {
            expansion: self,
            sites: ordered(placements, holder_of_surface)?,
            holders,
        })
    }

    /// Each robot whose drawn stand is at a surface, with its label.
    fn stands_at_surfaces(&self) -> Result<Vec<(RobotName, String)>, Error> {
        let marked = self.pattern.fields.marked_maps(self.pattern.origin)?;
        let mut stands = Vec::new();
        for (name, robot) in marked.robots {
            let root = SitePath::new(vec![
                Segment::Key("robots".to_owned()),
                Segment::Key(name.to_string()),
            ]);
            let stand = match robot {
                Sampled::Fixed(robot) => {
                    serde_json::to_value(&robot.stand).expect("a stand serializes")
                }
                Sampled::Drawn(drawn) => {
                    let drawn_or_kept = |site: &Site| {
                        let path = root.joined(site.path.segments());
                        Ok(self.drawn.get(&path).cloned().unwrap_or(Value::Null))
                    };
                    drawn.with_values(&drawn_or_kept)?["stand"].clone()
                }
            };
            if let Some(surface) = stand["surface"].as_str() {
                stands.push((name.clone(), surface.to_owned()));
            }
        }
        Ok(stands)
    }

    /// The concrete world of the expansion, with the value of each placement
    /// by its path in `placed`: a pose for a pose, a position `[x, y, z]` for
    /// the position of a stand. Its name is that of the pattern with
    /// `_seed_<seed>`, its revision that of the pattern. Refuses a placement
    /// with no value, and a value that the world refuses, by its path.
    pub fn world(&self, placed: &BTreeMap<SitePath, Value>) -> Result<World, Error> {
        let pattern = self.pattern;
        let mut fields = pattern.fields.clone();
        fields.manifest = Manifest {
            name: format!("{}_seed_{}", pattern.manifest.name, self.seed),
            revision: pattern.manifest.revision,
        };
        fields.work_cell_parameters = BTreeMap::new();
        fields.min_spacing_m = None;
        let root = |key: &str| SitePath::new(vec![Segment::Key(key.to_owned())]);
        if let Some(scene) = &fields.scene {
            fields.scene = Some(Sampled::Fixed(self.concrete(
                &root("scene"),
                scene,
                placed,
            )?));
        }
        self.make_concrete(&root("robots"), &mut fields.robots, placed)?;
        self.make_concrete(&root("scene_objects"), &mut fields.scene_objects, placed)?;
        let mut objects = BTreeMap::new();
        for (name, entry) in std::mem::take(&mut fields.objects) {
            let Entry::Value(object) = entry else {
                objects.insert(name, entry);
                continue;
            };
            match object {
                ObjectEntry::Object(object) => {
                    let concrete = self.concrete(&objects_path(&name), &object, placed)?;
                    let entry = ObjectEntry::Object(Sampled::Fixed(concrete));
                    objects.insert(name, Entry::Value(entry));
                }
                ObjectEntry::ItemKind(kind) => {
                    for (added, Added { asset, .. }) in
                        self.added.iter().filter(|(_, added)| added.entry == name)
                    {
                        let mut concrete =
                            self.concrete(&objects_path(added), &kind.template, placed)?;
                        concrete.asset = asset.clone();
                        let entry = ObjectEntry::Object(Sampled::Fixed(concrete));
                        objects.insert(added.clone(), Entry::Value(entry));
                    }
                }
            }
        }
        fields.objects = objects;
        self.make_concrete(&root("lighting"), &mut fields.lighting, placed)?;
        self.make_concrete(&root("materials"), &mut fields.materials, placed)?;
        if let Some(Entry::Value(daylight)) = &fields.daylight {
            let concrete = self.concrete(&root("daylight"), daylight, placed)?;
            fields.daylight = Some(Entry::Value(Sampled::Fixed(concrete)));
        }
        self.make_concrete(&root("regions"), &mut fields.regions, placed)?;
        self.make_concrete(&root("locations"), &mut fields.locations, placed)?;
        check_name_rules(&fields)?;
        WorldDocument { fields }.into_world(pattern.origin)
    }

    /// Each entry of `entries` under `field`, made concrete.
    fn make_concrete<K: Ord + std::fmt::Display, T: DeserializeOwned + Clone>(
        &self,
        field: &SitePath,
        entries: &mut BTreeMap<K, Entry<Sampled<T>>>,
        placed: &BTreeMap<SitePath, Value>,
    ) -> Result<(), Error> {
        for (key, entry) in entries.iter_mut() {
            if let Entry::Value(sampled) = entry {
                let concrete = self.concrete(&field.key(&key.to_string()), sampled, placed)?;
                *sampled = Sampled::Fixed(concrete);
            }
        }
        Ok(())
    }

    /// The concrete value of the entry at `root`, each sampler replaced by
    /// its drawn value or its value in `placed`.
    fn concrete<T: DeserializeOwned + Clone>(
        &self,
        root: &SitePath,
        sampled: &Sampled<T>,
        placed: &BTreeMap<SitePath, Value>,
    ) -> Result<T, Error> {
        let drawn = match sampled {
            Sampled::Fixed(value) => return Ok(value.clone()),
            Sampled::Drawn(drawn) => drawn,
        };
        let value_of = |site: &Site| {
            let path = root.joined(site.path.segments());
            self.drawn
                .get(&path)
                .or_else(|| placed.get(&path))
                .cloned()
                .ok_or_else(|| {
                    Error::field(site.path.to_string(), "has no placed value; place it first")
                })
        };
        drawn
            .concrete(&value_of)
            .map_err(|refusal| Error::field(root.to_string(), refusal.to_string()))
    }
}

/// The refusal of the entry `name` of the item kind `kind`, which no asset
/// of the catalogue has.
fn no_asset_of_kind(name: &Name, kind: &str) -> Error {
    Error::field(
        format!("objects.{name}"),
        format!("no asset of the catalogue has the item kind `{kind}`"),
    )
}

/// The placements of an expansion in the order the simulation draws them
/// ([`Expansion::placements`]).
#[derive(Clone, Debug)]
pub struct Placements<'e> {
    expansion: &'e Expansion<'e>,
    sites: Vec<PlacementSite>,
    /// The object of the surface of each robot whose stand is a placement
    /// at a surface.
    holders: BTreeMap<RobotName, Name>,
}

impl<'e> Placements<'e> {
    /// The placements, in the order the simulation draws them.
    pub fn sites(&self) -> &[PlacementSite] {
        &self.sites
    }

    /// The world of the expansion with each placement at a placeholder: a
    /// pose at the origin with yaw 0, the position of a stand at the origin;
    /// a stand at a surface keeps its value.
    pub fn placeholder_world(&self) -> Result<World, Error> {
        let placed = self
            .sites
            .iter()
            .filter_map(|placement| Some((placement.path.clone(), placement.kind.placeholder()?)))
            .collect();
        self.expansion.world(&placed)
    }

    /// The world of the placements that `placed` holds so far, as
    /// [`Expansion::world`] gives it with each other placement left out: an
    /// object whose pose has no value, a scene object whose pose has no
    /// value, removed, the stand of a robot whose position has no value, and
    /// the stand of a robot at a surface of an object left out. The
    /// simulation builds its model of the drawn world from it before it
    /// draws the next placement.
    pub fn world_so_far(&self, placed: &BTreeMap<SitePath, Value>) -> Result<World, Error> {
        let mut filled = placed.clone();
        let mut left_out = BTreeSet::new();
        let mut removed = BTreeSet::new();
        let mut standless = BTreeSet::new();
        for placement in self
            .sites
            .iter()
            .filter(|placement| !placed.contains_key(&placement.path))
        {
            let Some(placeholder) = placement.kind.placeholder() else {
                continue;
            };
            match &placement.kind {
                PlacementKind::ObjectPose { object, .. } if placement.is_of_a_scene_object() => {
                    removed.insert(object.clone())
                }
                PlacementKind::ObjectPose { object, .. } => left_out.insert(object.clone()),
                PlacementKind::Stand { robot, .. } => standless.insert(robot.clone()),
            };
            filled.insert(placement.path.clone(), placeholder);
        }
        for (robot, holder) in &self.holders {
            if left_out.contains(holder) || removed.contains(holder) {
                standless.insert(robot.clone());
            }
        }
        let mut world = self.expansion.world(&filled)?;
        world.objects.retain(|name, _| !left_out.contains(name));
        for name in removed {
            world.scene_objects.insert(name, SceneObject::Removed);
        }
        for robot in &standless {
            if let Some(robot) = world.robots.get_mut(robot) {
                robot.stand = None;
            }
        }
        Ok(world)
    }
}

impl PlacementSite {
    /// Whether the placement draws the pose of a scene object.
    fn is_of_a_scene_object(&self) -> bool {
        matches!(
            self.path.segments().first(),
            Some(Segment::Key(field)) if field == "scene_objects"
        )
    }
}

/// `placements` in the order of their dependencies, as
/// [`Expansion::placements`] says.
fn ordered(
    placements: Vec<PlacementSite>,
    holder_of_surface: &dyn Fn(&str) -> Option<Name>,
) -> Result<Vec<PlacementSite>, Error> {
    let object_at: BTreeMap<&Name, usize> = placements
        .iter()
        .enumerate()
        .filter_map(|(index, placement)| match &placement.kind {
            PlacementKind::ObjectPose { object, .. } => Some((object, index)),
            PlacementKind::Stand { .. } => None,
        })
        .collect();
    let robot_at: BTreeMap<&RobotName, usize> = placements
        .iter()
        .enumerate()
        .filter_map(|(index, placement)| match &placement.kind {
            PlacementKind::Stand { robot, .. } => Some((robot, index)),
            PlacementKind::ObjectPose { .. } => None,
        })
        .collect();
    let holder_index =
        |label: &str| holder_of_surface(label).and_then(|holder| object_at.get(&holder).copied());
    let after: Vec<BTreeSet<usize>> = placements
        .iter()
        .map(|placement| match &placement.kind {
            PlacementKind::ObjectPose {
                sampler:
                    Sampler::On {
                        surfaces,
                        reachable_by,
                    },
                ..
            } => {
                let on = surfaces.iter().filter_map(|surface| match surface {
                    SurfaceRef::Label(label) => holder_index(label),
                    SurfaceRef::Landmark { object, .. } => object_at.get(object).copied(),
                });
                let robot = reachable_by
                    .iter()
                    .filter_map(|robot| robot_at.get(robot).copied());
                on.chain(robot).collect()
            }
            PlacementKind::Stand {
                at: StandAt::Surface(label),
                ..
            } => holder_index(label).into_iter().collect(),
            PlacementKind::ObjectPose { .. }
            | PlacementKind::Stand {
                at: StandAt::Region(_),
                ..
            } => BTreeSet::new(),
        })
        .collect();
    let mut order = Vec::with_capacity(placements.len());
    let mut done = vec![false; placements.len()];
    while order.len() < placements.len() {
        let next = (0..placements.len())
            .filter(|&index| !done[index] && after[index].iter().all(|&before| done[before]))
            .min_by(|&a, &b| {
                let key = |index: usize| (placements[index].kind.group(), &placements[index].path);
                key(a).cmp(&key(b))
            });
        let Some(next) = next else {
            let cycle: Vec<String> = (0..placements.len())
                .filter(|&index| !done[index])
                .map(|index| format!("`{}`", placements[index].path))
                .collect();
            return Err(Error::rule(format!(
                "the stands and poses {} depend on each other in a cycle; place one of them without the others",
                cycle.join(", ")
            )));
        };
        done[next] = true;
        order.push(next);
    }
    let mut by_index: Vec<Option<PlacementSite>> = placements.into_iter().map(Some).collect();
    Ok(order
        .into_iter()
        .map(|index| by_index[index].take().expect("each placement once"))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::{RELEASE, name, robot};
    use crate::{Feature, Offset, Pose, Stand};
    use serde_json::json;

    /// The tape line of the plan: two arms at their stations with a drawn
    /// offset, a courier in a region, a conveyor with drawn controls, and the
    /// cartons and tape rolls of their item kinds.
    fn tape_line_text() -> String {
        format!(
            r#"{{
  simulation_schema: "world/v1",
  manifest: {{ name: "tape_line", revision: 1 }},
  catalogue: "{RELEASE}",
  scene: {{ asset: "scene/tape_line" }},
  robots: {{
    alpha: {{ join: {{ option: "so101_sim", features: ["aspire", "packing_skills"] }}, stand: {{ surface: "station_a", offset: {{ ahead_m: {{ range: [-0.05, 0.05] }}, side_m: {{ range: [-0.03, 0.03] }} }} }} }},
    bravo: {{ join: {{ option: "so101_sim", features: ["aspire", "packing_skills"] }}, stand: {{ surface: "station_b" }} }},
    charlie: {{ join: {{ option: "sourccey_sim" }}, stand: {{ position: {{ in: "aisle" }}, yaw: {{ range: [-3, 3] }} }} }},
  }},
  objects: {{
    conveyor: {{ asset: "object/conveyor_straight_3m", pose: {{ position: [0, 0, 0], yaw: 0 }}, physics: "static", controls: {{ height: {{ range: [0.38, 0.50] }}, belt_speed: {{ range: [0.05, 0.25] }} }} }},
    stock: {{ item_kind: "carton", count: {{ range: [3, 6] }}, pose: {{ in: "box_stock" }} }},
    tapes: {{ item_kind: "tape_roll", count: {{ range: [12, 20] }}, pose: {{ on: {{ object: "conveyor", landmark: "belt_top" }} }} }},
  }},
  regions: {{
    aisle: {{ centre: [0, -1.2, 0.5], size: [3, 0.8, 1] }},
    box_stock: {{ centre: [2, 1, 0.5], size: [0.8, 0.6, 1], yaw: 0.3 }},
  }},
  work_cell_parameters: {{
    belt_speed_mps: {{ unit: "m/s", control: {{ object: "conveyor", id: "belt_speed" }} }},
    belt_top_above_base_m: {{ unit: "m", per_robot: ["alpha", "bravo"], distance: {{ surface: {{ object: "conveyor", landmark: "belt_top" }}, axis: "up" }} }},
  }},
  min_spacing_m: 0.02,
}}"#
        )
    }

    fn tape_line() -> WorldPattern {
        WorldPattern::parse(&tape_line_text()).unwrap()
    }

    fn assets_of_kind(kind: &str) -> Vec<String> {
        match kind {
            "carton" => vec!["object/carton_b".into(), "object/carton_a".into()],
            "tape_roll" => vec!["object/tape_roll_blue".into()],
            _ => Vec::new(),
        }
    }

    fn no_holder(_: &str) -> Option<Name> {
        None
    }

    /// A value for each placement: a pose for a pose, a position for a stand
    /// in a region.
    fn placed(placements: &[PlacementSite]) -> BTreeMap<SitePath, Value> {
        placements
            .iter()
            .enumerate()
            .filter_map(|(index, placement)| {
                let value = match &placement.kind {
                    PlacementKind::ObjectPose { .. } => {
                        json!({ "position": [index as f64, 0.0, 0.8], "yaw": 0.0 })
                    }
                    PlacementKind::Stand {
                        at: StandAt::Region(_),
                        ..
                    } => json!([index as f64, -1.0, 0.0]),
                    PlacementKind::Stand {
                        at: StandAt::Surface(_),
                        ..
                    } => return None,
                };
                Some((placement.path.clone(), value))
            })
            .collect()
    }

    fn expanded(pattern: &WorldPattern, seed: u32) -> World {
        let expansion = pattern.expansion(Seed(seed), &assets_of_kind).unwrap();
        let placements = expansion.placements(&no_holder).unwrap();
        expansion.world(&placed(placements.sites())).unwrap()
    }

    fn refusal(text: &str) -> String {
        WorldPattern::parse(text).unwrap_err().to_string()
    }

    #[test]
    fn the_tape_line_parses_as_a_pattern_with_its_fields() {
        let pattern = tape_line();
        assert!(!pattern.is_concrete());
        assert_eq!(pattern.min_spacing_m, 0.02);
        let top = &pattern.work_cell_parameters.0
            [&ParameterName::parse("belt_top_above_base_m").unwrap()];
        assert_eq!(
            top.measure,
            Measure::Distance {
                surface: SurfaceRef::Landmark {
                    object: name("conveyor"),
                    landmark: "belt_top".into()
                },
                axis: Axis::Up,
                per_robot: Some(BTreeSet::from([robot("alpha"), robot("bravo")])),
            }
        );
        assert_eq!(
            pattern.joins()[&robot("alpha")].features,
            vec![
                Feature::Named("aspire".into()),
                Feature::Named("packing_skills".into())
            ]
        );
    }

    #[test]
    fn a_concrete_world_refuses_a_sampler_and_the_fields_of_a_pattern() {
        assert_eq!(
            World::parse(&tape_line_text()).unwrap_err().to_string(),
            "work_cell_parameters: is a field of a world pattern; resolve the world as a pattern, or leave the field out"
        );
        let without_fields = tape_line_text()
            .replace("min_spacing_m: 0.02,", "")
            .replace("work_cell_parameters: {", "zz: {");
        let without_fields = without_fields.split("zz: {").next().unwrap().to_owned() + "}";
        assert_eq!(
            World::parse(&without_fields).unwrap_err().to_string(),
            "robots.alpha: holds a sampler, so the world is a world pattern; expand it with a seed, or write each value"
        );
    }

    #[test]
    fn a_seed_expands_the_pattern_into_a_concrete_world_within_its_samplers() {
        let pattern = tape_line();
        let world = expanded(&pattern, 7);
        assert_eq!(world.manifest.name, "tape_line_seed_7");
        assert_eq!(world.manifest.revision, pattern.manifest.revision);
        let conveyor = &world.objects[&name("conveyor")];
        assert!((0.38..=0.50).contains(&conveyor.controls["height"]));
        assert!((0.05..=0.25).contains(&conveyor.controls["belt_speed"]));
        let Some(Stand::Surface { offset, .. }) = &world.robots[&robot("alpha")].stand else {
            panic!("alpha stands at its station");
        };
        assert!(
            (-0.05..=0.05).contains(&offset.ahead_m) && (-0.03..=0.03).contains(&offset.side_m)
        );
        let cartons: Vec<&Name> = world
            .objects
            .keys()
            .filter(|object| object.as_str().starts_with("stock_"))
            .collect();
        assert!((3..=6).contains(&cartons.len()), "{cartons:?}");
        for carton in cartons {
            let asset = &world.objects[carton].asset;
            assert!(
                asset == "object/carton_a" || asset == "object/carton_b",
                "{asset}"
            );
        }
        assert!(
            world
                .objects
                .keys()
                .any(|object| object.as_str() == "tapes_12")
        );
        let Some(Stand::Placed(stand)) = &world.robots[&robot("charlie")].stand else {
            panic!("charlie stands where its region draw placed it");
        };
        assert!((-3.0..=3.0).contains(&stand.yaw));
    }

    #[test]
    fn the_candidate_worlds_give_each_value_of_each_sampler() {
        let pattern = tape_line();
        let worlds = pattern.candidate_worlds(&assets_of_kind).unwrap();
        let heights: BTreeSet<String> = worlds
            .iter()
            .map(|world| world.objects[&name("conveyor")].controls["height"].to_string())
            .collect();
        assert_eq!(
            heights,
            BTreeSet::from(["0.38".to_owned(), "0.5".to_owned()])
        );
        let first = &worlds[0];
        let cartons: Vec<&Name> = first
            .objects
            .keys()
            .filter(|object| object.as_str().starts_with("stock_"))
            .collect();
        assert_eq!(cartons.len(), 6, "the largest count of the entry");
        let carton_assets: BTreeSet<&str> = worlds
            .iter()
            .map(|world| world.objects[&name("stock_1")].asset.as_str())
            .collect();
        assert_eq!(
            carton_assets,
            BTreeSet::from(["object/carton_a", "object/carton_b"])
        );
        let tape_offsets: BTreeSet<String> = worlds
            .iter()
            .filter_map(|world| match &world.robots[&robot("alpha")].stand {
                Some(Stand::Surface { offset, .. }) => Some(offset.ahead_m.to_string()),
                _ => None,
            })
            .collect();
        assert_eq!(
            tape_offsets,
            BTreeSet::from(["-0.05".to_owned(), "0.05".to_owned()])
        );
        // The first world, and one more for each other value of each sampler
        // (five ranges, each with two ends) and each other asset of a kind.
        assert_eq!(worlds.len(), 1 + 5 + 1);
        let without_assets = pattern.candidate_worlds(&|_| Vec::new()).unwrap_err();
        assert_eq!(
            without_assets.to_string(),
            "objects.stock: no asset of the catalogue has the item kind `carton`"
        );
    }

    #[test]
    fn the_item_kinds_of_a_pattern_are_its_entries_of_an_item_kind() {
        let pattern = tape_line();
        let kinds: Vec<(&str, &str)> = pattern
            .item_kinds()
            .map(|(name, kind)| (name.as_str(), kind))
            .collect();
        assert_eq!(kinds, [("stock", "carton"), ("tapes", "tape_roll")]);
    }

    #[test]
    fn the_world_so_far_leaves_out_each_placement_without_a_value() {
        let text = tape_line_text()
            .replace(
                "    stock:",
                "    table: { asset: \"object/table\", pose: { in: \"box_stock\" } },\n    crate: { asset: \"object/crate\", pose: { on: \"table top\", reachable_by: \"delta\" } },\n    stock:",
            )
            .replace(
                "    charlie:",
                "    delta: { join: { option: \"openarm_sim\" }, stand: { surface: \"table top\" } },\n    charlie:",
            );
        let pattern = WorldPattern::parse(&text).unwrap();
        let expansion = pattern.expansion(Seed(2), &assets_of_kind).unwrap();
        let holder = |label: &str| (label == "table top").then(|| name("table"));
        let placements = expansion.placements(&holder).unwrap();
        let nothing = placements.world_so_far(&BTreeMap::new()).unwrap();
        let objects: Vec<&str> = nothing.objects.keys().map(Name::as_str).collect();
        assert_eq!(objects, ["conveyor"]);
        let standing = |world: &World| -> Vec<String> {
            world
                .robots
                .iter()
                .filter(|(_, robot)| robot.stand.is_some())
                .map(|(name, _)| name.to_string())
                .collect()
        };
        assert_eq!(standing(&nothing), ["alpha", "bravo"]);

        let table_path = SitePath::new(vec![
            Segment::Key("objects".into()),
            Segment::Key("table".into()),
            Segment::Key("pose".into()),
        ]);
        let table_placed = BTreeMap::from([(
            table_path,
            json!({ "position": [2.0, 1.0, 0.0], "yaw": 0.0 }),
        )]);
        let with_the_table = placements.world_so_far(&table_placed).unwrap();
        assert!(with_the_table.objects.contains_key(&name("table")));
        assert!(!with_the_table.objects.contains_key(&name("crate")));
        assert_eq!(standing(&with_the_table), ["alpha", "bravo", "delta"]);

        let everything = placements
            .world_so_far(&placed(placements.sites()))
            .unwrap();
        assert_eq!(
            everything,
            expansion.world(&placed(placements.sites())).unwrap()
        );
    }

    /// A scene object whose pose a placement draws stands in the world so
    /// far only once its pose has a value: until then the world removes it,
    /// and a robot that stands at its surface has no stand.
    #[test]
    fn the_world_so_far_removes_each_scene_object_whose_pose_has_no_value() {
        let text = tape_line_text()
            .replace(
                "  robots: {",
                "  scene_objects: { shelf: { pose: { in: \"box_stock\" } } },\n  robots: {",
            )
            .replace(
                "    charlie:",
                "    delta: { join: { option: \"openarm_sim\" }, stand: { surface: \"shelf top\" } },\n    charlie:",
            );
        let pattern = WorldPattern::parse(&text).unwrap();
        let expansion = pattern.expansion(Seed(2), &assets_of_kind).unwrap();
        let holder = |label: &str| (label == "shelf top").then(|| name("shelf"));
        let placements = expansion.placements(&holder).unwrap();
        let nothing = placements.world_so_far(&BTreeMap::new()).unwrap();
        assert_eq!(
            nothing.scene_objects.get(&name("shelf")),
            Some(&SceneObject::Removed)
        );
        assert!(nothing.robots[&robot("delta")].stand.is_none());

        let shelf_path = SitePath::new(vec![
            Segment::Key("scene_objects".into()),
            Segment::Key("shelf".into()),
            Segment::Key("pose".into()),
        ]);
        let shelf_placed = BTreeMap::from([(
            shelf_path,
            json!({ "position": [2.0, 1.0, 0.0], "yaw": 0.0 }),
        )]);
        let with_the_shelf = placements.world_so_far(&shelf_placed).unwrap();
        assert!(matches!(
            with_the_shelf.scene_objects.get(&name("shelf")),
            Some(SceneObject::Placed { .. })
        ));
        assert!(with_the_shelf.robots[&robot("delta")].stand.is_some());
    }

    /// The placeholder world gives each placement its placeholder: an object
    /// at the origin with yaw 0, a stand in a region at the origin, and a
    /// stand at a surface as written.
    #[test]
    fn the_placeholder_world_puts_each_placement_at_the_origin() {
        let pattern = tape_line();
        let expansion = pattern.expansion(Seed(4), &assets_of_kind).unwrap();
        let world = expansion
            .placements(&no_holder)
            .unwrap()
            .placeholder_world()
            .unwrap();
        let origin = Pose::Yawed {
            position: [0.0, 0.0, 0.0],
            yaw: 0.0,
        };
        assert_eq!(world.objects[&name("stock_1")].pose, origin);
        let Some(Stand::Placed(stand)) = &world.robots[&robot("charlie")].stand else {
            panic!("charlie stands in a region");
        };
        assert_eq!(stand.position, [0.0, 0.0, 0.0]);
        assert!(matches!(
            world.robots[&robot("bravo")].stand,
            Some(Stand::Surface { .. })
        ));
    }

    #[test]
    fn the_same_seed_gives_the_same_world_and_another_seed_another() {
        let pattern = tape_line();
        assert_eq!(expanded(&pattern, 7), expanded(&pattern, 7));
        let worlds: BTreeSet<String> = (1..=5)
            .map(|seed| {
                let mut world = expanded(&pattern, seed);
                world.manifest.name.clear();
                world.to_json5().unwrap()
            })
            .collect();
        assert_eq!(worlds.len(), 5);
    }

    #[test]
    fn an_added_entry_or_count_changes_no_other_value() {
        let fewer =
            WorldPattern::parse(&tape_line_text().replace("count: { range: [3, 6] }", "count: 3"))
                .unwrap();
        let more = WorldPattern::parse(
            &tape_line_text()
                .replace("count: { range: [3, 6] }", "count: 5")
                .replace(
                    "    stock:",
                    "    lamp: { asset: \"object/lamp\", pose: { position: [2, 2, 0], yaw: { range: [0, 1] } } },\n    stock:",
                ),
        )
        .unwrap();
        let (fewer, more) = (expanded(&fewer, 11), expanded(&more, 11));
        for index in 1..=3 {
            let carton = name(&format!("stock_{index}"));
            assert_eq!(fewer.objects[&carton].asset, more.objects[&carton].asset);
        }
        assert_eq!(
            fewer.objects[&name("conveyor")],
            more.objects[&name("conveyor")]
        );
        assert_eq!(fewer.robots[&robot("alpha")], more.robots[&robot("alpha")]);
    }

    #[test]
    fn an_item_kind_with_no_asset_or_a_placement_with_no_value_is_refused() {
        let pattern = tape_line();
        let no_tapes = |kind: &str| match kind {
            "tape_roll" => Vec::new(),
            other => assets_of_kind(other),
        };
        assert_eq!(
            pattern
                .expansion(Seed(1), &no_tapes)
                .unwrap_err()
                .to_string(),
            "objects.tapes: no asset of the catalogue has the item kind `tape_roll`"
        );
        let expansion = pattern.expansion(Seed(1), &assets_of_kind).unwrap();
        assert_eq!(
            expansion.world(&BTreeMap::new()).unwrap_err().to_string(),
            "robots.charlie: stand.position: has no placed value; place it first"
        );
    }

    #[test]
    fn placements_follow_their_dependencies_then_their_groups_and_paths() {
        let text = tape_line_text()
            .replace(
                "    stock:",
                "    table: { asset: \"object/table\", pose: { in: \"box_stock\" } },\n    crate: { asset: \"object/crate\", pose: { on: \"table top\", reachable_by: \"delta\" } },\n    stock:",
            )
            .replace(
                "    charlie:",
                "    delta: { join: { option: \"openarm_sim\" }, stand: { surface: \"table top\" } },\n    charlie:",
            );
        let pattern = WorldPattern::parse(&text).unwrap();
        let expansion = pattern.expansion(Seed(2), &assets_of_kind).unwrap();
        let holder = |label: &str| (label == "table top").then(|| name("table"));
        let order: Vec<String> = expansion
            .placements(&holder)
            .unwrap()
            .sites()
            .iter()
            .map(|placement| placement.path.to_string())
            .filter(|path| !path.starts_with("objects.tapes") && !path.starts_with("objects.stock"))
            .collect();
        assert_eq!(
            order,
            [
                "objects.table.pose",
                "robots.charlie.stand.position",
                "robots.delta.stand",
                "objects.crate.pose",
            ]
        );
    }

    #[test]
    fn a_cycle_of_stands_and_poses_is_refused_naming_its_placements() {
        let text = tape_line_text()
            .replace(
                "    stock:",
                "    table: { asset: \"object/table\", pose: { on: \"floor\", reachable_by: \"delta\" } },\n    stock:",
            )
            .replace(
                "    charlie:",
                "    delta: { join: { option: \"openarm_sim\" }, stand: { surface: \"table top\" } },\n    charlie:",
            );
        let pattern = WorldPattern::parse(&text).unwrap();
        let expansion = pattern.expansion(Seed(2), &assets_of_kind).unwrap();
        let holder = |label: &str| (label == "table top").then(|| name("table"));
        assert_eq!(
            expansion.placements(&holder).unwrap_err().to_string(),
            "the stands and poses `objects.table.pose`, `robots.delta.stand` depend on each other in a cycle; place one of them without the others"
        );
    }

    #[test]
    fn the_rules_of_a_pattern_refuse_names_robots_and_counts_by_their_place() {
        for (from, to, expected) in [
            (
                "    stock:",
                "    stock_2: { asset: \"object/box\", pose: { position: [0, 0, 0], yaw: 0 } },\n    stock:",
                "objects.stock: adds `stock_2`, which is the name of another entry of `objects`; rename one of the two",
            ),
            (
                "  robots: {",
                "  scene_objects: { tapes_20: { removed: true } },\n  robots: {",
                "objects.tapes: adds `tapes_20`, which is the name of a scene object; rename one of the two",
            ),
            (
                "landmark: \"belt_top\" } } },",
                "landmark: \"belt_top\" }, reachable_by: \"echo\" } },",
                "objects.tapes.pose: `reachable_by` names `echo`, which is no robot of the world with a stand",
            ),
            (
                "pose: { in: \"box_stock\" }",
                "pose: { in: \"box_shelf\" }",
                "objects.stock.pose: names the region `box_shelf`, which the world does not declare in `regions`",
            ),
            (
                "  min_spacing_m:",
                "  locations: { drop: { one_of: [{ surface: \"belt\" }, { in: \"output_area\" }] } },\n  min_spacing_m:",
                "locations.drop: names the region `output_area`, which the world does not declare in `regions`",
            ),
            (
                "per_robot: [\"alpha\", \"bravo\"]",
                "per_robot: [\"alpha\", \"echo\"]",
                "work_cell_parameters.belt_top_above_base_m: `per_robot` names `echo`, which is no robot of the world with a stand",
            ),
            (
                "per_robot: [\"alpha\", \"bravo\"], ",
                "",
                "work_cell_parameters.belt_top_above_base_m: measures from the one robot with a stand, and the world has 3; name the robots in `per_robot`",
            ),
            (
                "count: { range: [3, 6] }",
                "count: { range: [6, 3] }",
                "objects.stock: count: the count is a number of objects from 0 to 1000",
            ),
            (
                "count: { range: [3, 6] }",
                "count: 1001",
                "objects.stock: count: the count is a number of objects from 0 to 1000",
            ),
            (
                "item_kind: \"carton\",",
                "item_kind: \"carton\", asset: \"object/box\",",
                "objects.stock: asset: an entry of an item kind draws the asset of each object from its item kind; leave `asset` out",
            ),
            (
                "join: { option: \"sourccey_sim\" }",
                "join: { option: { one_of: [\"sourccey_sim\", \"lekiwi_sim\"] } }",
                "robots.charlie: join.option: a join is fixed: a sweep checks the workflow against the join of each robot, so write the join without a sampler",
            ),
            (
                "min_spacing_m: 0.02",
                "min_spacing_m: -1",
                "min_spacing_m: the spacing -1 is not at least 0; write a distance of 0 m or more",
            ),
        ] {
            let text = tape_line_text().replace(from, to);
            assert_ne!(text, tape_line_text(), "{from}");
            let refused = refusal(&text);
            assert!(refused.starts_with(expected), "{expected}\n{refused}");
        }
    }

    #[test]
    fn a_pattern_extends_a_concrete_world_and_another_pattern() {
        let chat_world = WorldDocument::parse(&format!(
            r#"{{ simulation_schema: "world/v1", manifest: {{ name: "world_3f9a1c27", revision: 3 }}, catalogue: "{RELEASE}", scene: {{ asset: "scene/living_room" }},
              robots: {{ alpha: {{ join: {{ option: "openarm_sim" }}, stand: {{ surface: "coffee table at the origin" }} }} }},
              objects: {{ crate: {{ asset: "object/crate", pose: {{ position: [0.55, 0, 0.39], yaw: 0 }}, physics: "dynamic" }} }} }}"#
        ))
        .unwrap();
        let books = WorldDocument::parse(
            r#"{ simulation_schema: "world/v1", manifest: { name: "books", revision: 1 }, extends: { name: "world_3f9a1c27", revision: 3 },
              objects: { books: { item_kind: "book", count: { range: [1, 4] }, pose: { on: "coffee table at the origin", reachable_by: "alpha" } } },
              work_cell_parameters: { table_ahead_m: { unit: "m", distance: { surface: "coffee table at the origin", axis: "ahead" } } } }"#,
        )
        .unwrap();
        let pattern = books
            .clone()
            .resolve_pattern(std::slice::from_ref(&chat_world))
            .unwrap();
        assert_eq!(
            pattern.physics_of(&name("crate")),
            Some(BTreeSet::from([Some(Physics::Dynamic)]))
        );
        assert_eq!(
            pattern.physics_of(&name("books_4")),
            Some(BTreeSet::from([None]))
        );
        assert_eq!(pattern.physics_of(&name("books_5")), None);
        let fewer = WorldDocument::parse(
            r#"{ simulation_schema: "world/v1", manifest: { name: "few_books", revision: 1 }, extends: { name: "books", revision: 1 },
              objects: { books: { item_kind: "book", count: { one_of: [0, 1] }, pose: { on: "coffee table at the origin" } } } }"#,
        )
        .unwrap();
        let few = fewer.resolve_pattern(&[books, chat_world.clone()]).unwrap();
        assert_eq!(few.work_cell_parameters.0.len(), 1);
        let written = WorldPattern::parse(&few.to_json5().unwrap()).unwrap();
        assert_eq!(written.work_cell_parameters, few.work_cell_parameters);
        let expand = |pattern: &WorldPattern| {
            let expansion = pattern
                .expansion(Seed(3), &|_| vec!["object/book".into()])
                .unwrap();
            let placements = expansion.placements(&no_holder).unwrap();
            let mut world = expansion.world(&placed(placements.sites())).unwrap();
            world.manifest.name.clear();
            world
        };
        assert_eq!(expand(&written), expand(&few));
        let world = {
            let expansion = few
                .expansion(Seed(3), &|_| vec!["object/book".into()])
                .unwrap();
            let placements = expansion.placements(&no_holder).unwrap();
            expansion.world(&placed(placements.sites())).unwrap()
        };
        assert!(world.objects.len() <= 2, "{:?}", world.objects.keys());
        let concrete = chat_world.resolve_pattern(&[]).unwrap();
        assert!(concrete.is_concrete());
        let same = concrete
            .expansion(Seed(9), &|_| Vec::new())
            .unwrap()
            .world(&BTreeMap::new())
            .unwrap();
        assert_eq!(
            same.objects[&name("crate")].pose,
            Pose::Yawed {
                position: [0.55, 0.0, 0.39],
                yaw: 0.0
            }
        );
        assert_eq!(
            same.robots[&robot("alpha")].stand,
            Some(Stand::Surface {
                surface: "coffee table at the origin".into(),
                offset: Offset::default()
            })
        );
    }

    #[test]
    fn a_work_cell_parameter_measures_a_control_or_a_distance() {
        let read = |text: &str| {
            crate::text::from_json5::<WorkCellParameter>(text).map_err(|error| error.to_string())
        };
        assert_eq!(
            read(r#"{ unit: "m/s", control: { object: "conveyor", id: "belt_speed" } }"#).unwrap(),
            WorkCellParameter {
                unit: "m/s".into(),
                measure: Measure::Control {
                    object: name("conveyor"),
                    id: "belt_speed".into()
                }
            }
        );
        for (text, expected) in [
            (
                r#"{ unit: "m/s", control: { object: "conveyor", id: "belt_speed" }, per_robot: ["alpha"] }"#,
                "a work cell parameter measures a `control` or a `distance`, and `per_robot` is for a distance",
            ),
            (
                r#"{ unit: "m" }"#,
                "a work cell parameter measures a `control` or a `distance`, and `per_robot` is for a distance",
            ),
            (
                r#"{ unit: "", control: { object: "conveyor", id: "belt_speed" } }"#,
                "the unit is blank; name the unit of the value, such as `m`",
            ),
            (
                r#"{ unit: "m", distance: { surface: "belt", axis: "up" }, per_robot: ["alpha", "alpha"] }"#,
                "`per_robot` names the robot `alpha` twice; name each robot once",
            ),
        ] {
            assert_eq!(read(text).unwrap_err(), expected, "{text}");
        }
        let parameter = read(
            r#"{ unit: "m", distance: { surface: "belt", axis: "side" }, per_robot: ["alpha"] }"#,
        )
        .unwrap();
        let text = crate::text::to_json5(&parameter).unwrap();
        assert_eq!(read(&text).unwrap(), parameter);
    }

    #[test]
    fn measurements_take_the_shape_of_their_parameters() {
        let parameters = tape_line().work_cell_parameters;
        let text = parameters.to_json5().unwrap();
        assert_eq!(WorkCellParameters::parse(&text).unwrap(), parameters);
        let measured = Measurements::parse(
            "{ belt_speed_mps: 0.2, belt_top_above_base_m: { alpha: 0.18, bravo: 0.21 } }",
        )
        .unwrap();
        measured.check(&parameters).unwrap();
        assert_eq!(
            Measurements::parse(&measured.to_json5().unwrap()).unwrap(),
            measured
        );
        for (text, refusal) in [
            (
                "{ belt_speed_mps: 0.2 }",
                "belt_top_above_base_m: has no measurement of its shape; the parameter takes one value for each robot of its `per_robot`: `alpha`, `bravo`",
            ),
            (
                "{ belt_speed_mps: { alpha: 0.2 }, belt_top_above_base_m: { alpha: 0.18, bravo: 0.21 } }",
                "belt_speed_mps: has no measurement of its shape; the parameter takes one value",
            ),
            (
                "{ belt_speed_mps: 0.2, belt_top_above_base_m: { alpha: 0.18 } }",
                "belt_top_above_base_m: has no measurement of its shape; the parameter takes one value for each robot of its `per_robot`: `alpha`, `bravo`",
            ),
            (
                "{ belt_speed_mps: 0.2, belt_top_above_base_m: { alpha: 0.18, bravo: 0.21 }, gripper_force: 2 }",
                "gripper_force: is no work cell parameter",
            ),
        ] {
            let refused = Measurements::parse(text)
                .unwrap()
                .check(&parameters)
                .unwrap_err();
            assert_eq!(refused.to_string(), refusal, "{text}");
        }
    }

    #[test]
    fn a_pattern_writes_as_a_document_that_extends_no_base_and_reads_back() {
        let pattern = tape_line();
        let written = WorldPattern::parse(&pattern.to_json5().unwrap()).unwrap();
        assert_eq!(written, pattern);
    }
}
