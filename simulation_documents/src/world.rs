//! The world/v1 document: its fields, their rules, and the text form.

use std::collections::BTreeMap;

use documents_core::{Digest, Document, Kind, Manifest};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::geometry::{Offset, Placement, Point, Pose};
use crate::names::{
    RESERVED_PEPPY_NAME, RESERVED_REASON, first_named_twice, is_peppy_name, peppy_name_grammar,
};
use crate::{Error, Name, RobotName};

/// The kind of a world document.
pub(crate) const WORLD: Kind = Kind::new("simulation_schema", "world", 1);

/// A concrete world: every value fixed, every base resolved.
#[derive(Clone, Debug, PartialEq)]
pub struct World {
    pub manifest: Manifest,
    pub catalogue: Catalogue,
    pub scene: Scene,
    pub robots: BTreeMap<RobotName, Robot>,
    pub scene_objects: BTreeMap<Name, SceneObject>,
    pub objects: BTreeMap<Name, Object>,
    pub lighting: BTreeMap<String, Light>,
    pub materials: BTreeMap<String, Material>,
    pub locations: BTreeMap<Name, Location>,
}

/// The catalogue release a world is built from: the digest of the release,
/// or `local` for a simulation with a local catalogue directory.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub enum Catalogue {
    Release(Digest),
    Local,
}

impl TryFrom<String> for Catalogue {
    type Error = Error;

    fn try_from(text: String) -> Result<Catalogue, Error> {
        if text == "local" {
            return Ok(Catalogue::Local);
        }
        Digest::parse(&text)
            .map(Catalogue::Release)
            .ok_or_else(|| {
                Error::rule(format!(
                    "`{text}` is not the digest of a catalogue release (64 lowercase hex digits) or `local`"
                ))
            })
    }
}

impl From<Catalogue> for String {
    fn from(catalogue: Catalogue) -> String {
        match catalogue {
            Catalogue::Release(digest) => digest.to_string(),
            Catalogue::Local => "local".to_owned(),
        }
    }
}

/// The scene asset of the world and the scale it is loaded at.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "SceneFields", into = "SceneFields")]
pub struct Scene {
    pub asset: String,
    pub scale: f64,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a scene, { asset, scale }")]
struct SceneFields {
    asset: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    scale: Option<f64>,
}

impl TryFrom<SceneFields> for Scene {
    type Error = Error;

    fn try_from(fields: SceneFields) -> Result<Scene, Error> {
        Ok(Scene {
            asset: fields.asset,
            scale: positive_scale(fields.scale)?,
        })
    }
}

impl From<Scene> for SceneFields {
    fn from(scene: Scene) -> SceneFields {
        SceneFields {
            asset: scene.asset,
            scale: written_scale(scene.scale),
        }
    }
}

/// An omitted scale is 1; a written one is positive.
fn positive_scale(scale: Option<f64>) -> Result<f64, Error> {
    let scale = scale.unwrap_or(1.0);
    let above_zero = scale > 0.0;
    if !above_zero {
        return Err(Error::rule(format!(
            "the scale {scale} is not above 0; write a scale above 0, or leave it out for 1"
        )));
    }
    Ok(scale)
}

fn written_scale(scale: f64) -> Option<f64> {
    (scale != 1.0).then_some(scale)
}

/// A simulated robot of the world: how it joins the stack and where it stands.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a robot, { join, stand }")]
pub struct Robot {
    pub join: Join,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stand: Option<Stand>,
}

/// The join of a robot: an option of the `stack_copies` scope, its features,
/// each named once, and the core node it runs on when that is another machine
/// than the daemon.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "JoinFields", into = "JoinFields")]
pub struct Join {
    pub option: String,
    pub features: Vec<Feature>,
    pub core_node: Option<String>,
}

/// Two joins are equal when they name the same option, the same core node and
/// the same features, in any order: the stack lists a copy's features in the
/// order of its option.
impl PartialEq for Join {
    fn eq(&self, other: &Join) -> bool {
        let holds_each_of = |join: &Join, of: &Join| {
            of.features
                .iter()
                .all(|feature| join.features.contains(feature))
        };
        self.option == other.option
            && self.core_node == other.core_node
            && holds_each_of(self, other)
            && holds_each_of(other, self)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a join, { option, features, core_node }"
)]
struct JoinFields {
    option: String,
    #[serde(
        default,
        skip_serializing_if = "Vec::is_empty",
        deserialize_with = "features_named_once"
    )]
    features: Vec<Feature>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "core_node_of_join"
    )]
    core_node: Option<String>,
}

impl TryFrom<JoinFields> for Join {
    type Error = Error;

    fn try_from(fields: JoinFields) -> Result<Join, Error> {
        Ok(Join {
            option: named(fields.option, "option", "an option of the stack")?,
            features: fields.features,
            core_node: fields.core_node,
        })
    }
}

/// The features of a join, each named once, as peppy joins them.
fn features_named_once<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<Vec<Feature>, D::Error> {
    let features = Vec::<Feature>::deserialize(deserializer)?;
    let twice = first_named_twice(&features, |a, b| a.name() == b.name()).map(Feature::name);
    if let Some(name) = twice {
        return Err(serde::de::Error::custom(format!(
            "the feature `{name}` is named twice; name each feature once"
        )));
    }
    Ok(features)
}

fn core_node_of_join<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<String>, D::Error> {
    Option::<String>::deserialize(deserializer)?
        .map(core_node_named)
        .transpose()
        .map_err(serde::de::Error::custom)
}

/// `text` when it names a `field`; a blank one is refused with `what` to name.
fn named(text: String, field: &str, what: &str) -> Result<String, Error> {
    if text.trim().is_empty() {
        return Err(Error::rule(format!("the {field} is blank; name {what}")));
    }
    Ok(text)
}

/// `text` when it names a core node as peppy does ([`peppy_name_grammar`]),
/// and not [`RESERVED_PEPPY_NAME`].
fn core_node_named(text: String) -> Result<String, Error> {
    if text == RESERVED_PEPPY_NAME {
        return Err(Error::rule(format!(
            "the core_node `{RESERVED_PEPPY_NAME}` {RESERVED_REASON}; write the core node's own name"
        )));
    }
    if !is_peppy_name(&text) {
        return Err(Error::rule(format!(
            "the core_node `{text}` is not {}; write the name of a core node of the stack",
            peppy_name_grammar()
        )));
    }
    Ok(text)
}

/// A join writes its features in name order, so two equal joins write one
/// text and give one digest.
impl From<Join> for JoinFields {
    fn from(join: Join) -> JoinFields {
        let mut features = join.features;
        features.sort_by(|a, b| a.name().cmp(b.name()));
        JoinFields {
            option: join.option,
            features,
            core_node: join.core_node,
        }
    }
}

/// A feature of a join: by name, or placed on a core node.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Feature {
    Named(String),
    Placed { feature: String, core_node: String },
}

const FEATURE_FORM: &str = "a feature is a name, or { feature, core_node }";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PlacedFeatureFields {
    feature: String,
    core_node: String,
}

impl<'de> Deserialize<'de> for Feature {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Feature, D::Error> {
        let value = Value::deserialize(deserializer)?;
        feature_of(value).map_err(serde::de::Error::custom)
    }
}

/// The feature `value` writes: a name, or `{ feature, core_node }`.
fn feature_of(value: Value) -> Result<Feature, Error> {
    let feature_named = |text| named(text, "feature", "a feature of the option");
    match value {
        Value::String(name) => Ok(Feature::Named(feature_named(name)?)),
        Value::Object(fields) => {
            let PlacedFeatureFields { feature, core_node } =
                serde_json::from_value(Value::Object(fields))
                    .map_err(|_| Error::rule(FEATURE_FORM))?;
            Ok(Feature::Placed {
                feature: feature_named(feature)?,
                core_node: core_node_named(core_node)?,
            })
        }
        _ => Err(Error::rule(FEATURE_FORM)),
    }
}

impl Feature {
    pub fn name(&self) -> &str {
        match self {
            Feature::Named(name) | Feature::Placed { feature: name, .. } => name,
        }
    }
}

/// Where a robot with a body stands: at a work surface or a named pose of the
/// scene, offset on its own axes, or at a position with a yaw.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "StandFields", into = "StandFields")]
pub enum Stand {
    Surface { surface: String, offset: Offset },
    At { at: Name, offset: Offset },
    Placed(Placement),
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a stand, { surface, offset }, { at, offset } or { position, yaw }"
)]
struct StandFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    surface: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    at: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position: Option<Point>,
    #[serde(skip_serializing_if = "Option::is_none")]
    yaw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    offset: Option<Offset>,
}

impl TryFrom<StandFields> for Stand {
    type Error = Error;

    fn try_from(fields: StandFields) -> Result<Stand, Error> {
        match fields {
            StandFields {
                surface: Some(surface),
                at: None,
                position: None,
                yaw: None,
                offset,
            } => Ok(Stand::Surface {
                surface,
                offset: offset.unwrap_or_default(),
            }),
            StandFields {
                surface: None,
                at: Some(at),
                position: None,
                yaw: None,
                offset,
            } => Ok(Stand::At {
                at,
                offset: offset.unwrap_or_default(),
            }),
            StandFields {
                surface: None,
                at: None,
                position: Some(position),
                yaw: Some(yaw),
                offset: None,
            } => Ok(Stand::Placed(Placement { position, yaw })),
            _ => Err(Error::rule(
                "a stand is { surface, offset }, { at, offset } (a named pose of the scene) or { position, yaw }; offset is optional",
            )),
        }
    }
}

impl From<Stand> for StandFields {
    fn from(stand: Stand) -> StandFields {
        let written_offset = |offset: Offset| (offset != Offset::default()).then_some(offset);
        match stand {
            Stand::Surface { surface, offset } => StandFields {
                surface: Some(surface),
                at: None,
                position: None,
                yaw: None,
                offset: written_offset(offset),
            },
            Stand::At { at, offset } => StandFields {
                surface: None,
                at: Some(at),
                position: None,
                yaw: None,
                offset: written_offset(offset),
            },
            Stand::Placed(Placement { position, yaw }) => StandFields {
                surface: None,
                at: None,
                position: Some(position),
                yaw: Some(yaw),
                offset: None,
            },
        }
    }
}

/// An object the scene places, as the world has it: removed, or at a pose
/// with its control values. Written as `{ removed: true }` or
/// `{ pose, controls }`, read by the parser of the marks of a map entry.
#[derive(Clone, Debug, PartialEq)]
pub enum SceneObject {
    Removed,
    Placed {
        pose: Pose,
        controls: BTreeMap<String, f64>,
    },
}

/// The fields of a placed scene object.
#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a scene object, { removed: true } or { pose, controls }"
)]
struct PlacedFields {
    pose: Pose,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    controls: BTreeMap<String, f64>,
}

impl Serialize for SceneObject {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let entry = match self {
            SceneObject::Removed => Entry::Removed,
            SceneObject::Placed { pose, controls } => Entry::Value(PlacedFields {
                pose: pose.clone(),
                controls: controls.clone(),
            }),
        };
        entry.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for SceneObject {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<SceneObject, D::Error> {
        match Entry::<PlacedFields>::deserialize(deserializer)? {
            Entry::Removed => Ok(SceneObject::Removed),
            Entry::Kept => Err(serde::de::Error::custom(
                "`kept` drops an entry of a base, in a world document that extends one",
            )),
            Entry::Value(PlacedFields { pose, controls }) => {
                Ok(SceneObject::Placed { pose, controls })
            }
        }
    }
}

/// An object spawned into the world. An omitted `physics`, `mass` or control
/// takes the value of the asset in the catalogue.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ObjectFields", into = "ObjectFields")]
pub struct Object {
    pub asset: String,
    pub pose: Pose,
    pub scale: f64,
    pub physics: Option<Physics>,
    pub mass: Option<f64>,
    pub controls: BTreeMap<String, f64>,
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "an object, { asset, pose, scale, physics, mass, controls }"
)]
struct ObjectFields {
    asset: String,
    pose: Pose,
    #[serde(skip_serializing_if = "Option::is_none")]
    scale: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    physics: Option<Physics>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mass: Option<f64>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    controls: BTreeMap<String, f64>,
}

impl TryFrom<ObjectFields> for Object {
    type Error = Error;

    fn try_from(fields: ObjectFields) -> Result<Object, Error> {
        if let Some(mass) = fields.mass {
            let at_least_zero = mass >= 0.0;
            if !at_least_zero {
                return Err(Error::rule(format!(
                    "the mass {mass} is not at least 0; write a mass of 0 kg or more, or leave it out for the mass of the catalogue"
                )));
            }
        }
        Ok(Object {
            asset: fields.asset,
            pose: fields.pose,
            scale: positive_scale(fields.scale)?,
            physics: fields.physics,
            mass: fields.mass,
            controls: fields.controls,
        })
    }
}

impl From<Object> for ObjectFields {
    fn from(object: Object) -> ObjectFields {
        ObjectFields {
            asset: object.asset,
            pose: object.pose,
            scale: written_scale(object.scale),
            physics: object.physics,
            mass: object.mass,
            controls: object.controls,
        }
    }
}

/// How an object takes part in physics, as `scene.spawn_object` takes it:
/// written `dynamic`, `static` or `none`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub enum Physics {
    Dynamic,
    Static,
    None,
}

impl TryFrom<String> for Physics {
    type Error = Error;

    fn try_from(word: String) -> Result<Physics, Error> {
        match word.as_str() {
            "dynamic" => Ok(Physics::Dynamic),
            "static" => Ok(Physics::Static),
            "none" => Ok(Physics::None),
            other => Err(Error::rule(format!(
                "the physics `{other}` is none of dynamic, static and none; write one of them"
            ))),
        }
    }
}

impl From<Physics> for String {
    fn from(physics: Physics) -> String {
        match physics {
            Physics::Dynamic => "dynamic",
            Physics::Static => "static",
            Physics::None => "none",
        }
        .to_owned()
    }
}

/// The settings of one light of the scene, by its id: each the last value a
/// `lighting.set_*` call gave it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "LightFields")]
pub struct Light {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intensity: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Point>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cone: Option<Cone>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orientation: Option<[f64; 4]>,
}

#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a light, { intensity, color, position, direction, cone, orientation }"
)]
struct LightFields {
    intensity: Option<f64>,
    color: Option<[f64; 3]>,
    position: Option<Point>,
    direction: Option<[f64; 3]>,
    cone: Option<Cone>,
    orientation: Option<[f64; 4]>,
}

impl TryFrom<LightFields> for Light {
    type Error = Error;

    fn try_from(fields: LightFields) -> Result<Light, Error> {
        let light = Light {
            intensity: fields.intensity,
            color: fields.color,
            position: fields.position,
            direction: fields.direction,
            cone: fields.cone,
            orientation: fields
                .orientation
                .map(crate::geometry::unit_quaternion)
                .transpose()?,
        };
        if light == Light::EMPTY {
            return Err(Error::rule(
                "a light entry names at least one setting: intensity, color, position, direction, cone or orientation",
            ));
        }
        Ok(light)
    }
}

impl Light {
    pub const EMPTY: Light = Light {
        intensity: None,
        color: None,
        position: None,
        direction: None,
        cone: None,
        orientation: None,
    };
}

/// The cone of a spot light, radians.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a cone, { inner_angle, outer_angle }"
)]
pub struct Cone {
    pub inner_angle: f64,
    pub outer_angle: f64,
}

/// The settings of one material, by its id: each the last value a
/// `materials.set_*` call gave it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "MaterialFields")]
pub struct Material {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub finish: Option<Finish>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a material, { color, finish }")]
struct MaterialFields {
    color: Option<[f64; 3]>,
    finish: Option<Finish>,
}

impl TryFrom<MaterialFields> for Material {
    type Error = Error;

    fn try_from(fields: MaterialFields) -> Result<Material, Error> {
        let material = Material {
            color: fields.color,
            finish: fields.finish,
        };
        if material == Material::EMPTY {
            return Err(Error::rule(
                "a material entry names at least one setting: color or finish",
            ));
        }
        Ok(material)
    }
}

impl Material {
    pub const EMPTY: Material = Material {
        color: None,
        finish: None,
    };
}

/// The finish of a material, as `materials.set_material_finish` takes it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a finish, { metallic, roughness }")]
pub struct Finish {
    pub metallic: f64,
    pub roughness: f64,
}

/// A named location of the task: a landmark of an object, a work surface, a
/// named pose or a named region of the scene.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "LocationFields", into = "LocationFields")]
pub enum Location {
    Landmark { object: Name, landmark: String },
    Surface(String),
    At(Name),
    In(Name),
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a location, { object, landmark }, { surface }, { at } or { in }"
)]
struct LocationFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    object: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    landmark: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    surface: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    at: Option<Name>,
    #[serde(rename = "in", skip_serializing_if = "Option::is_none")]
    region: Option<Name>,
}

impl TryFrom<LocationFields> for Location {
    type Error = Error;

    fn try_from(fields: LocationFields) -> Result<Location, Error> {
        match fields {
            LocationFields {
                object: Some(object),
                landmark: Some(landmark),
                surface: None,
                at: None,
                region: None,
            } => Ok(Location::Landmark { object, landmark }),
            LocationFields {
                object: None,
                landmark: None,
                surface: Some(surface),
                at: None,
                region: None,
            } => Ok(Location::Surface(surface)),
            LocationFields {
                object: None,
                landmark: None,
                surface: None,
                at: Some(at),
                region: None,
            } => Ok(Location::At(at)),
            LocationFields {
                object: None,
                landmark: None,
                surface: None,
                at: None,
                region: Some(region),
            } => Ok(Location::In(region)),
            _ => Err(Error::rule(
                "a location is { object, landmark }, { surface }, { at } (a named pose) or { in } (a named region)",
            )),
        }
    }
}

impl From<Location> for LocationFields {
    fn from(location: Location) -> LocationFields {
        let (object, landmark, surface, at, region) = match location {
            Location::Landmark { object, landmark } => {
                (Some(object), Some(landmark), None, None, None)
            }
            Location::Surface(surface) => (None, None, Some(surface), None, None),
            Location::At(at) => (None, None, None, Some(at), None),
            Location::In(region) => (None, None, None, None, Some(region)),
        };
        LocationFields {
            object,
            landmark,
            surface,
            at,
            region,
        }
    }
}

/// An entry of a map of a world document: a value, or a mark that drops the
/// base's entry of the key (`{ removed: true }`), or, in `scene_objects`,
/// a mark that drops it so the scene object stays as the scene places it
/// (`{ kept: true }`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Entry<T> {
    Removed,
    Kept,
    Value(T),
}

impl<T: Serialize> Serialize for Entry<T> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mark = |key: &str| Value::Object(Map::from_iter([(key.to_owned(), Value::Bool(true))]));
        match self {
            Entry::Removed => mark("removed").serialize(serializer),
            Entry::Kept => mark("kept").serialize(serializer),
            Entry::Value(value) => value.serialize(serializer),
        }
    }
}

/// The parser of the marks: an object that holds a mark key is that mark,
/// written `{ <mark>: true }` alone; any other value is the entry's value.
impl<'de, T: DeserializeOwned> Deserialize<'de> for Entry<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Entry<T>, D::Error> {
        let value = Value::deserialize(deserializer)?;
        let marked = value.as_object().and_then(|fields| {
            [("removed", Entry::Removed), ("kept", Entry::Kept)]
                .into_iter()
                .find(|(key, _)| fields.contains_key(*key))
                .map(|(key, entry)| (key, entry, fields))
        });
        let Some((key, entry, fields)) = marked else {
            return serde_path_to_error::deserialize(value)
                .map(Entry::Value)
                .map_err(serde::de::Error::custom);
        };
        if fields.len() != 1 || fields[key] != Value::Bool(true) {
            return Err(serde::de::Error::custom(format!(
                "`{key}` is `true` for a {key} entry; write `{key}: true` or the entry's fields"
            )));
        }
        Ok(entry)
    }
}

/// The fields of a world document as written, before its base is resolved.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct WorldFields {
    pub manifest: Manifest,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub catalogue: Option<Catalogue>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extends: Option<Manifest>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scene: Option<Scene>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub robots: BTreeMap<RobotName, Entry<Robot>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub scene_objects: BTreeMap<Name, Entry<SceneObject>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub objects: BTreeMap<Name, Entry<Object>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub lighting: BTreeMap<String, Entry<Light>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub materials: BTreeMap<String, Entry<Material>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub locations: BTreeMap<Name, Entry<Location>>,
}

/// A world document as written: it may extend a base.
/// [`WorldDocument::resolve`] gives the concrete [`World`].
#[derive(Clone, Debug, PartialEq)]
pub struct WorldDocument {
    pub(crate) fields: WorldFields,
}

impl WorldDocument {
    /// Parses JSON5 text as a world document. Refuses a text whose root key or
    /// kind is not `simulation_schema: "world/v1"` before it reads another
    /// field, then each field by its rule.
    pub fn parse(text: &str) -> Result<WorldDocument, Error> {
        let document = Document::parse(text, WORLD)?;
        if !document.holds("manifest") {
            return Err(Error::rule(
                "the world has no manifest; add `manifest: { name: \"<its name>\", revision: 1 }`",
            ));
        }
        let fields: WorldFields = document.read()?;
        check_name_rules(&fields)?;
        Ok(WorldDocument { fields })
    }

    pub(crate) fn manifest(&self) -> &Manifest {
        &self.fields.manifest
    }

    /// The concrete world of fields that extend no base. `origin` says where
    /// they come from: the refusal of a missing catalogue or scene says where
    /// it was looked for, and a scene object a base keeps has no entry.
    pub(crate) fn into_world(self, origin: Origin) -> Result<World, Error> {
        let fields = self.fields;
        if fields.extends.is_some() {
            return Err(Error::Unresolved(fields.manifest.name));
        }
        let unnamed = |field: &str, what: &str, form: &str| {
            let reason = match origin {
                Origin::Alone => format!("a world that extends no base names its {what}"),
                Origin::WithBases => format!("neither the world nor its bases name a {what}"),
            };
            Error::field(field, format!("{reason}; add {form}"))
        };
        let catalogue = fields.catalogue.clone().ok_or_else(|| {
            unnamed(
                "catalogue",
                "catalogue release",
                "`catalogue: \"<digest of the release>\"`, or `catalogue: \"local\"` for a local catalogue",
            )
        })?;
        let scene = fields
            .scene
            .clone()
            .ok_or_else(|| unnamed("scene", "scene", "`scene: { asset: \"scene/<id>\" }`"))?;
        let maps = fields.concrete_maps(origin)?;
        Ok(World {
            manifest: fields.manifest,
            catalogue,
            scene,
            robots: maps.robots,
            scene_objects: maps.scene_objects,
            objects: maps.objects,
            lighting: maps.lighting,
            materials: maps.materials,
            locations: maps.locations,
        })
    }
}

/// Where the fields of a concrete world come from.
#[derive(Clone, Copy)]
pub(crate) enum Origin {
    /// One document that extends no base.
    Alone,
    /// A document merged with its bases.
    WithBases,
}

/// The maps of a world, every entry a value.
pub(crate) struct ConcreteMaps {
    robots: BTreeMap<RobotName, Robot>,
    scene_objects: BTreeMap<Name, SceneObject>,
    objects: BTreeMap<Name, Object>,
    lighting: BTreeMap<String, Light>,
    materials: BTreeMap<String, Material>,
    locations: BTreeMap<Name, Location>,
}

impl WorldFields {
    /// The maps of a document, every entry a value. A document that extends
    /// no base is refused `kept`, and `removed` outside `scene_objects`: each
    /// drops an entry of a base. In a document merged with its bases, a kept
    /// scene object stands as the scene has it, under no entry.
    pub(crate) fn concrete_maps(&self, origin: Origin) -> Result<ConcreteMaps, Error> {
        let scene_objects = self.scene_objects.iter().filter_map(|(name, entry)| {
            let object = match (entry, origin) {
                (Entry::Value(object), _) => object.clone(),
                (Entry::Removed, _) => SceneObject::Removed,
                (Entry::Kept, Origin::WithBases) => return None,
                (Entry::Kept, Origin::Alone) => {
                    return Some(Err(Error::field(
                        format!("scene_objects.{name}"),
                        "`kept` drops an entry of a base; this world extends no base",
                    )));
                }
            };
            Some(Ok((name.clone(), object)))
        });
        Ok(ConcreteMaps {
            robots: concrete_entries("robots", &self.robots)?,
            scene_objects: scene_objects.collect::<Result<_, _>>()?,
            objects: concrete_entries("objects", &self.objects)?,
            lighting: concrete_entries("lighting", &self.lighting)?,
            materials: concrete_entries("materials", &self.materials)?,
            locations: concrete_entries("locations", &self.locations)?,
        })
    }
}

/// The values of a map whose entries drop nothing, named by `field`.
fn concrete_entries<K: Ord + Clone + std::fmt::Display, T: Clone>(
    field: &str,
    entries: &BTreeMap<K, Entry<T>>,
) -> Result<BTreeMap<K, T>, Error> {
    entries
        .iter()
        .map(|(key, entry)| match entry {
            Entry::Value(value) => Ok((key.clone(), value.clone())),
            Entry::Removed => Err(Error::field(
                format!("{field}.{key}"),
                "`removed` drops an entry of a base; this world extends no base",
            )),
            Entry::Kept => Err(kept_outside_scene_objects(field, key)),
        })
        .collect()
}

/// The refusal of `{ kept: true }` at `key` of `field`, a map other than
/// `scene_objects`.
pub(crate) fn kept_outside_scene_objects(field: &str, key: &dyn std::fmt::Display) -> Error {
    Error::field(
        format!("{field}.{key}"),
        "`kept` drops a `scene_objects` entry of a base; write `removed: true` to drop this entry, or the entry's fields",
    )
}

/// The rules on names that hold in each document and after each base is
/// resolved: no name of a robot the world stands followed by `_` begins
/// another, and no object shares a name with a scene object. A removed or a
/// kept entry stands nothing, but a removed or a kept scene object keeps its
/// name in the scene.
pub(crate) fn check_name_rules(fields: &WorldFields) -> Result<(), Error> {
    robot_names_free_of_prefixes(
        fields
            .robots
            .iter()
            .filter(|(_, entry)| matches!(entry, Entry::Value(_)))
            .map(|(name, _)| name),
    )?;
    first_shared_name(
        fields
            .objects
            .iter()
            .filter(|(_, entry)| matches!(entry, Entry::Value(_)))
            .map(|(name, _)| name),
        |name| {
            matches!(
                fields.scene_objects.get(name),
                Some(Entry::Value(_) | Entry::Removed | Entry::Kept)
            )
        },
    )
}

/// Object names and scene object names share one namespace.
pub(crate) fn check_namespaces(world: &World) -> Result<(), Error> {
    first_shared_name(world.objects.keys(), |name| {
        world.scene_objects.contains_key(name)
    })
}

/// The refusal of the first of `objects` that `in_the_scene` holds as the
/// name of a scene object.
fn first_shared_name<'a>(
    mut objects: impl Iterator<Item = &'a Name>,
    in_the_scene: impl Fn(&Name) -> bool,
) -> Result<(), Error> {
    match objects.find(|name| in_the_scene(name)) {
        Some(name) => Err(Error::NameInBothNamespaces(name.to_string())),
        None => Ok(()),
    }
}

pub(crate) fn robot_names_free_of_prefixes<'a>(
    names: impl Iterator<Item = &'a RobotName> + Clone,
) -> Result<(), Error> {
    for shorter in names.clone() {
        if let Some(longer) = names.clone().find(|longer| shorter.prefixes(longer)) {
            return Err(Error::RobotNamePrefix {
                shorter: shorter.to_string(),
                longer: longer.to_string(),
            });
        }
    }
    Ok(())
}

impl World {
    /// Parses JSON5 text as a concrete world: a world document that extends no
    /// base.
    pub fn parse(text: &str) -> Result<World, Error> {
        WorldDocument::parse(text)?.into_world(Origin::Alone)
    }

    /// The document of this world. Refuses a number of the world that a
    /// document cannot hold, by its path: a float that is not finite, or a
    /// number beyond 2^53.
    pub fn document(&self) -> Result<Document, Error> {
        Ok(Document::of(WORLD, &self.fields())?)
    }

    /// The world as JSON5 text, with the refusals of [`World::document`].
    pub fn to_json5(&self) -> Result<String, Error> {
        Ok(self.document()?.to_json5())
    }

    /// Whether `other` sets up the same world, whatever the two manifests say.
    pub fn same_setup_as(&self, other: &World) -> bool {
        let mut mine = self.clone();
        mine.manifest = other.manifest.clone();
        mine == *other
    }

    pub(crate) fn fields(&self) -> WorldFields {
        WorldFields {
            manifest: self.manifest.clone(),
            catalogue: Some(self.catalogue.clone()),
            extends: None,
            scene: Some(self.scene.clone()),
            robots: value_entries(&self.robots),
            scene_objects: value_entries(&self.scene_objects),
            objects: value_entries(&self.objects),
            lighting: value_entries(&self.lighting),
            materials: value_entries(&self.materials),
            locations: value_entries(&self.locations),
        }
    }
}

/// Each entry of `map` as a value entry.
fn value_entries<K: Clone + Ord, T: Clone>(map: &BTreeMap<K, T>) -> BTreeMap<K, Entry<T>> {
    map.iter()
        .map(|(key, value)| (key.clone(), Entry::Value(value.clone())))
        .collect()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use documents_core::Revision;
    use std::collections::BTreeSet;

    pub(crate) const RELEASE: &str =
        "504661c4504661c4504661c4504661c4504661c4504661c4504661c4504661c4";

    /// The world of the ticket's worked example, revision 3.
    pub(crate) fn example_text() -> String {
        format!(
            r#"{{
  simulation_schema: "world/v1",
  manifest: {{ name: "world_3f9a1c27", revision: 3 }},
  catalogue: "{RELEASE}",
  scene: {{ asset: "scene/living_room", scale: 1 }},
  robots: {{ alpha: {{ join: {{ option: "openarm_sim", features: ["ai_brain"] }}, stand: {{ surface: "coffee table at the origin" }} }} }},
  scene_objects: {{ mantel_clock_01: {{ removed: true }} }},
  objects: {{ crate: {{ asset: "object/polyhaven_plastic_crate_01", pose: {{ position: [0.55, 0.0, 0.39], yaw: 0.0 }}, physics: "dynamic", mass: 1.6 }} }},
}}"#
        )
    }

    pub(crate) fn example() -> World {
        World::parse(&example_text()).unwrap()
    }

    pub(crate) fn name(text: &str) -> Name {
        Name::parse(text).unwrap()
    }

    pub(crate) fn robot(text: &str) -> RobotName {
        RobotName::parse(text).unwrap()
    }

    #[test]
    fn the_worked_example_parses_into_its_fields() {
        let world = example();
        assert_eq!(
            world.manifest,
            Manifest {
                name: "world_3f9a1c27".into(),
                revision: Revision::new(3).unwrap()
            }
        );
        assert_eq!(
            world.catalogue,
            Catalogue::Release(Digest::parse(RELEASE).unwrap())
        );
        assert_eq!(
            world.scene,
            Scene {
                asset: "scene/living_room".into(),
                scale: 1.0
            }
        );
        let alpha = &world.robots[&robot("alpha")];
        assert_eq!(
            alpha.join,
            Join {
                option: "openarm_sim".into(),
                features: vec![Feature::Named("ai_brain".into())],
                core_node: None
            }
        );
        assert_eq!(
            alpha.stand,
            Some(Stand::Surface {
                surface: "coffee table at the origin".into(),
                offset: Offset::default()
            })
        );
        assert_eq!(
            world.scene_objects[&name("mantel_clock_01")],
            SceneObject::Removed
        );
        let crate_ = &world.objects[&name("crate")];
        assert_eq!(crate_.asset, "object/polyhaven_plastic_crate_01");
        assert_eq!(
            crate_.pose,
            Pose::Yawed {
                position: [0.55, 0.0, 0.39],
                yaw: 0.0
            }
        );
        assert_eq!(
            (crate_.scale, crate_.physics, crate_.mass),
            (1.0, Some(Physics::Dynamic), Some(1.6))
        );
        assert!(
            crate_.controls.is_empty()
                && world.lighting.is_empty()
                && world.materials.is_empty()
                && world.locations.is_empty()
        );
    }

    #[test]
    fn a_world_writes_as_json5_that_parses_back_to_it() {
        let world = example();
        let text = world.to_json5().unwrap();
        assert!(
            text.starts_with("{\n  simulation_schema: \"world/v1\",\n  manifest: {"),
            "{text}"
        );
        assert!(
            !text.contains("scale"),
            "an omitted scale of 1 stays omitted: {text}"
        );
        assert_eq!(World::parse(&text).unwrap(), world);
        assert_eq!(
            World::parse(&text).unwrap().document().unwrap().digest(),
            world.document().unwrap().digest()
        );
    }

    #[test]
    fn a_world_with_a_number_no_document_holds_is_refused_at_its_path() {
        let mut world = example();
        world.objects.get_mut(&name("crate")).unwrap().pose = Pose::Yawed {
            position: [0.0, 0.0, 0.0],
            yaw: f64::NAN,
        };
        assert_eq!(
            world.to_json5().unwrap_err().to_string(),
            "objects.crate.pose.yaw: the number NaN is not finite; a document holds finite numbers only"
        );
        let mut world = example();
        world.scene.scale = 1e16;
        assert_eq!(
            world.document().unwrap_err().to_string(),
            "scene.scale: the number 1e16 is beyond 2^53, the largest magnitude of a number in a document"
        );
    }

    #[test]
    fn another_kind_is_refused_before_the_fields() {
        let other_kind =
            r#"the root key `simulation_schema` holds "world/v2"; this parser reads "world/v1""#;
        let text = example_text()
            .replace("world/v1", "world/v2")
            .replace("manifest", "manifesto");
        assert_eq!(World::parse(&text).unwrap_err().to_string(), other_kind);
        let with_nan = example_text()
            .replace("world/v1", "world/v2")
            .replace("mass: 1.6", "mass: NaN");
        assert_eq!(World::parse(&with_nan).unwrap_err().to_string(), other_kind);
        let other_side = example_text()
            .replace("simulation_schema", "peppy_schema")
            .replace("\n  objects: {", "\n  objects: {}, objects: {");
        assert_eq!(
            World::parse(&other_side).unwrap_err().to_string(),
            r#"the document has no root key `simulation_schema`; add `simulation_schema: "world/v1"` as the first key"#
        );
    }

    #[test]
    fn a_nan_scale_or_mass_is_refused() {
        assert_eq!(
            json5::from_str::<Scene>("{ asset: 'scene/default', scale: NaN }")
                .unwrap_err()
                .to_string(),
            "the scale NaN is not above 0; write a scale above 0, or leave it out for 1"
        );
        assert_eq!(
            json5::from_str::<Object>(
                "{ asset: 'object/box', pose: { at: 'corner_a' }, mass: NaN }"
            )
            .unwrap_err()
            .to_string(),
            "the mass NaN is not at least 0; write a mass of 0 kg or more, or leave it out for the mass of the catalogue"
        );
    }

    /// The `{ ... }` forms a refusal names, in order.
    pub(crate) fn forms_named(refusal: &str) -> Vec<String> {
        refusal
            .split('{')
            .skip(1)
            .filter_map(|rest| rest.split_once('}'))
            .map(|(form, _)| form.trim().to_owned())
            .collect()
    }

    /// The refusal of `text` read as a `T`, as the crate reads a document's
    /// values.
    pub(crate) fn refusal_of<T: DeserializeOwned + std::fmt::Debug>(text: &str) -> String {
        crate::text::from_json5::<T>(text).unwrap_err().to_string()
    }

    /// The field names the forms of a refusal name, marks left out.
    fn fields_named(refusal: &str) -> BTreeSet<String> {
        forms_named(refusal)
            .iter()
            .filter(|form| !form.contains(':'))
            .flat_map(|form| form.split(',').map(|field| field.trim().to_owned()))
            .collect()
    }

    /// The field names serde lists in its refusal of an unknown field.
    fn fields_serde_takes(refusal: &str) -> BTreeSet<String> {
        let (_, listed) = refusal
            .split_once(", expected ")
            .unwrap_or_else(|| panic!("no field list in {refusal}"));
        listed
            .split('`')
            .skip(1)
            .step_by(2)
            .map(str::to_owned)
            .collect()
    }

    /// The refusal of a value of another type and of an unknown field, for
    /// `T` read as the crate reads a document's values.
    fn refusals_of<T: DeserializeOwned + std::fmt::Debug>() -> (String, String) {
        (refusal_of::<T>("1"), refusal_of::<T>("{ zz_unknown: 1 }"))
    }

    /// Every type a document reads from an object names, in the refusal of a
    /// value of another type, the forms to write and no Rust type, and the
    /// fields those forms name are the fields it takes.
    #[test]
    fn every_value_of_another_type_is_refused_with_the_fields_it_takes() {
        use crate::{
            Band, ContentsOn, Control, Count, Holding, Landmark, LandmarkOn, NearPoint, Offset,
            Place, Placement, Reachable, Target, Verdict,
        };
        let cases = [
            ("scene", refusals_of::<Scene>()),
            ("join", refusals_of::<Join>()),
            ("robot", refusals_of::<Robot>()),
            ("stand", refusals_of::<Stand>()),
            ("scene object", refusals_of::<SceneObject>()),
            ("object", refusals_of::<Object>()),
            ("light", refusals_of::<Light>()),
            ("cone", refusals_of::<Cone>()),
            ("material", refusals_of::<Material>()),
            ("finish", refusals_of::<Finish>()),
            ("location", refusals_of::<Location>()),
            ("pose", refusals_of::<Pose>()),
            ("offset", refusals_of::<Offset>()),
            ("placement", refusals_of::<Placement>()),
            ("holding", refusals_of::<Holding>()),
            ("landmark", refusals_of::<Landmark>()),
            ("band", refusals_of::<Band>()),
            ("target", refusals_of::<Target>()),
            ("place", refusals_of::<Place>()),
            ("landmark_on", refusals_of::<LandmarkOn>()),
            ("contents_on", refusals_of::<ContentsOn>()),
            ("near_point", refusals_of::<NearPoint>()),
            ("reachable", refusals_of::<Reachable>()),
            ("control", refusals_of::<Control>()),
            ("verdict", refusals_of::<Verdict>()),
            ("setup record", refusals_of::<crate::SetupRecord>()),
            ("count", refusals_of::<Count>()),
        ];
        for (named, (other_type, unknown)) in cases {
            assert!(other_type.contains("expected "), "{named}: {other_type}");
            for rust in ["struct", "Fields", "variant", "enum"] {
                assert!(!other_type.contains(rust), "{named}: {other_type}");
            }
            assert_eq!(
                fields_named(&other_type),
                fields_serde_takes(&unknown),
                "{named}: {other_type} | {unknown}"
            );
        }
    }

    /// A stand and a location name the same forms whether a value has another
    /// type or its fields form none.
    #[test]
    fn the_two_refusals_of_a_stand_or_a_location_name_the_same_forms() {
        let pairs = [
            (refusal_of::<Stand>("1"), refusal_of::<Stand>("{}")),
            (refusal_of::<Location>("1"), refusal_of::<Location>("{}")),
        ];
        for (other_type, no_form) in pairs {
            assert_eq!(
                forms_named(&other_type),
                forms_named(&no_form),
                "{other_type} | {no_form}"
            );
        }
    }

    /// A placement read on its own, as the first stands hold one, is refused
    /// as a stand: the one noun of where a robot stands.
    #[test]
    fn a_placement_of_another_type_is_refused_as_a_stand() {
        let refusal = refusal_of::<Placement>("1");
        assert!(
            refusal.ends_with("expected a stand, { position, yaw }"),
            "{refusal}"
        );
    }

    #[test]
    fn a_physics_is_one_of_three_words() {
        assert_eq!(
            crate::text::from_json5::<Physics>("\"static\"").unwrap(),
            Physics::Static
        );
        for other in ["springy", "Static", " static", "off"] {
            assert_eq!(
                refusal_of::<Physics>(&format!("{other:?}")),
                format!(
                    "the physics `{other}` is none of dynamic, static and none; write one of them"
                )
            );
        }
        for other in ["1", "{ static: null }"] {
            assert!(
                refusal_of::<Physics>(other).ends_with("expected a string"),
                "{other}: {}",
                refusal_of::<Physics>(other)
            );
        }
    }

    #[test]
    fn a_feature_written_as_an_object_takes_its_two_fields_alone() {
        assert_eq!(
            refusal_of::<Feature>(r#"{ feature: "ai_brain_vla", core_node: "gpu_box", zz: 1 }"#),
            FEATURE_FORM
        );
    }

    #[test]
    fn a_mass_of_0_is_taken() {
        let object =
            json5::from_str::<Object>("{ asset: 'object/box', pose: { at: 'corner_a' }, mass: 0 }")
                .unwrap();
        assert_eq!(object.mass, Some(0.0));
    }

    #[test]
    fn the_required_fields_are_the_root_key_the_manifest_the_catalogue_and_the_scene() {
        let fields = [
            ("manifest", r#"manifest: { name: "w", revision: 1 }"#),
            ("catalogue", r#"catalogue: "local""#),
            ("scene", r#"scene: { asset: "scene/default" }"#),
        ];
        let minimal = |omitted: &str| {
            let kept = fields
                .iter()
                .filter(|(field, _)| *field != omitted)
                .map(|(_, text)| *text)
                .collect::<Vec<_>>()
                .join(", ");
            format!(r#"{{ simulation_schema: "world/v1", {kept} }}"#)
        };
        let world = World::parse(&minimal("")).unwrap();
        assert_eq!(
            (world.catalogue, world.scene.scale),
            (Catalogue::Local, 1.0)
        );
        for (omitted, expected) in [
            (
                "manifest",
                "the world has no manifest; add `manifest: { name: \"<its name>\", revision: 1 }`",
            ),
            (
                "catalogue",
                "catalogue: a world that extends no base names its catalogue release; add `catalogue: \"<digest of the release>\"`, or `catalogue: \"local\"` for a local catalogue",
            ),
            (
                "scene",
                "scene: a world that extends no base names its scene; add `scene: { asset: \"scene/<id>\" }`",
            ),
        ] {
            assert_eq!(
                World::parse(&minimal(omitted)).unwrap_err().to_string(),
                expected
            );
        }
        assert_eq!(
            World::parse(&minimal("").replace("simulation_schema", "peppy_schema"))
                .unwrap_err()
                .to_string(),
            r#"the document has no root key `simulation_schema`; add `simulation_schema: "world/v1"` as the first key"#
        );
    }

    #[test]
    fn refusals_name_the_field_and_the_rule() {
        let cases = [
            (
                example_text().replace(RELEASE, "504661c4"),
                "catalogue: `504661c4` is not the digest of a catalogue release (64 lowercase hex digits) or `local`",
            ),
            (
                example_text().replace(RELEASE, &RELEASE.to_uppercase()),
                "catalogue: `504661C4504661C4504661C4504661C4504661C4504661C4504661C4504661C4` is not the digest of a catalogue release (64 lowercase hex digits) or `local`",
            ),
            (
                example_text().replace(RELEASE, "Local"),
                "catalogue: `Local` is not the digest of a catalogue release (64 lowercase hex digits) or `local`",
            ),
            (
                example_text().replace("catalogue:", "workflow: 1, catalogue:"),
                "workflow: unknown field `workflow`, expected one of `manifest`, `catalogue`, `extends`, `scene`, `robots`, `scene_objects`, `objects`, `lighting`, `materials`, `locations`",
            ),
            (
                example_text().replace("scale: 1", "scale: 0"),
                "scene: the scale 0 is not above 0; write a scale above 0, or leave it out for 1",
            ),
            (
                example_text().replace("mass: 1.6", "mass: -1"),
                "objects.crate: the mass -1 is not at least 0; write a mass of 0 kg or more, or leave it out for the mass of the catalogue",
            ),
            (
                example_text().replace("crate:", "mantel_clock_01:"),
                "`mantel_clock_01` is an object name and a scene object name; the two share one namespace, so give the object another name",
            ),
            (
                example_text().replace("alpha:", "alpha_2:").replace(
                    "robots: {",
                    "robots: { alpha: { join: { option: \"so101_sim\" } },",
                ),
                "the robot names `alpha` and `alpha_2` cannot stand together: a robot name followed by `_` begins no other, so rename one of the two robots",
            ),
            (
                example_text().replace("crate:", "Crate:"),
                "objects.Crate: the name `Crate` is not lowercase letters, digits and `_`",
            ),
            (
                example_text().replace("position: [0.55, 0.0, 0.39]", "position: [0.55, 0.0]"),
                "objects.crate: pose.position: invalid length 2, expected an array of length 3",
            ),
            (
                example_text().replace(r#"option: "openarm_sim""#, r#"option: " ""#),
                "robots.alpha: join: the option is blank; name an option of the stack",
            ),
            (
                example_text().replace(r#"features: ["ai_brain"]"#, r#"core_node: """#),
                "robots.alpha: join.core_node: the core_node `` is not 1 to 63 letters, digits, `_` and `-`; write the name of a core node of the stack",
            ),
            (
                example_text().replace(r#"features: ["ai_brain"]"#, r#"core_node: "gpu box""#),
                "robots.alpha: join.core_node: the core_node `gpu box` is not 1 to 63 letters, digits, `_` and `-`; write the name of a core node of the stack",
            ),
            (
                example_text().replace(r#"features: ["ai_brain"]"#, r#"core_node: "self""#),
                "robots.alpha: join.core_node: the core_node `self` is reserved: it names the daemon a launch or join targets; write the core node's own name",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: ["ai_brain", "camera", "ai_brain"]"#,
                ),
                "robots.alpha: join.features: the feature `ai_brain` is named twice; name each feature once",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: ["ai_brain", { feature: "ai_brain", core_node: "gpu_box" }]"#,
                ),
                "robots.alpha: join.features: the feature `ai_brain` is named twice; name each feature once",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: [{ feature: " ", core_node: "gpu_box" }]"#,
                ),
                "robots.alpha: join.features[0]: the feature is blank; name a feature of the option",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: [{ feature: "ai_brain", core_node: "a/b" }]"#,
                ),
                "robots.alpha: join.features[0]: the core_node `a/b` is not 1 to 63 letters, digits, `_` and `-`; write the name of a core node of the stack",
            ),
            (
                example_text().replace(r#"features: ["ai_brain"]"#, r#"features: [" "]"#),
                "robots.alpha: join.features[0]: the feature is blank; name a feature of the option",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: [{ feature: "ai_brain", core_node: "" }]"#,
                ),
                "robots.alpha: join.features[0]: the core_node `` is not 1 to 63 letters, digits, `_` and `-`; write the name of a core node of the stack",
            ),
            (
                example_text().replace(r#"features: ["ai_brain"]"#, r#"features: [7]"#),
                "robots.alpha: join.features[0]: a feature is a name, or { feature, core_node }",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: [{ feature: "ai_brain" }]"#,
                ),
                "robots.alpha: join.features[0]: a feature is a name, or { feature, core_node }",
            ),
            (
                example_text().replace(
                    r#"features: ["ai_brain"]"#,
                    r#"features: [{ feature: "ai_brain", core_node: "gpu_box", at: "x" }]"#,
                ),
                "robots.alpha: join.features[0]: a feature is a name, or { feature, core_node }",
            ),
            (
                example_text().replace("alpha:", "\"alpha 2\":"),
                "robots.alpha 2: the robot name `alpha 2` is not 1 to 63 letters, digits, `_` and `-`",
            ),
            (
                example_text().replace("{ removed: true }", "{ kept: true }"),
                "scene_objects.mantel_clock_01: `kept` drops an entry of a base; this world extends no base",
            ),
            (
                example_text().replace(
                    "\n  objects: { crate:",
                    "\n  objects: { box: { removed: true }, crate:",
                ),
                "objects.box: `removed` drops an entry of a base; this world extends no base",
            ),
            (
                example_text().replace("{ removed: true }", "{ removed: false }"),
                "scene_objects.mantel_clock_01: `removed` is `true` for a removed entry; write `removed: true` or the entry's fields",
            ),
            (
                example_text().replace(
                    "{ removed: true }",
                    "{ removed: true, pose: { at: \"corner_a\" } }",
                ),
                "scene_objects.mantel_clock_01: `removed` is `true` for a removed entry; write `removed: true` or the entry's fields",
            ),
            (
                example_text().replace("{ removed: true }", "{ removed: true, controls: {} }"),
                "scene_objects.mantel_clock_01: `removed` is `true` for a removed entry; write `removed: true` or the entry's fields",
            ),
            (
                example_text().replace("{ removed: true }", "{ kept: 1 }"),
                "scene_objects.mantel_clock_01: `kept` is `true` for a kept entry; write `kept: true` or the entry's fields",
            ),
            (
                example_text().replace(
                    "{ removed: true }",
                    "{ pose: { at: \"corner_a\" }, typo: 1 }",
                ),
                "scene_objects.mantel_clock_01: typo: unknown field `typo`, expected `pose` or `controls`",
            ),
            (
                example_text().replace(
                    "\n  objects: {",
                    "\n  lighting: { key: { intensity: 1, typo: 1 } }, objects: {",
                ),
                "lighting.key: typo: unknown field `typo`, expected one of `intensity`, `color`, `position`, `direction`, `cone`, `orientation`",
            ),
            (
                example_text().replace(
                    "\n  objects: {",
                    "\n  materials: { alpha_arm_metal: { color: [1, 1, 1], typo: 1 } }, objects: {",
                ),
                "materials.alpha_arm_metal: typo: unknown field `typo`, expected `color` or `finish`",
            ),
            (
                example_text().replace("\n  objects: {", "\n  lighting: { key: {} }, objects: {"),
                "lighting.key: a light entry names at least one setting: intensity, color, position, direction, cone or orientation",
            ),
            (
                example_text().replace(
                    "\n  objects: {",
                    "\n  lighting: { environment: { orientation: [0, 0, 0, 2] } }, objects: {",
                ),
                "lighting.environment: the orientation [x, y, z, w] has norm 2; a unit quaternion has norm 1 within 0.001",
            ),
            (
                example_text().replace(
                    "\n  objects: {",
                    "\n  materials: { alpha_arm_metal: {} }, objects: {",
                ),
                "materials.alpha_arm_metal: a material entry names at least one setting: color or finish",
            ),
            (
                example_text().replace(
                    "stand: { surface: \"coffee table at the origin\" }",
                    "stand: { surface: \"x\", yaw: 1 }",
                ),
                "robots.alpha: stand: a stand is { surface, offset }, { at, offset } (a named pose of the scene) or { position, yaw }; offset is optional",
            ),
            (
                example_text().replace(
                    "stand: { surface: \"coffee table at the origin\" }",
                    "stand: { position: [0, 0, 0], yaw: 1, offset: { ahead_m: 0.2 } }",
                ),
                "robots.alpha: stand: a stand is { surface, offset }, { at, offset } (a named pose of the scene) or { position, yaw }; offset is optional",
            ),
            (
                example_text().replace(
                    "\n  objects: {",
                    "\n  locations: { drop: { object: \"crate\" } }, objects: {",
                ),
                "locations.drop: a location is { object, landmark }, { surface }, { at } (a named pose) or { in } (a named region)",
            ),
        ];
        for (text, expected) in cases {
            assert_eq!(World::parse(&text).unwrap_err().to_string(), expected);
        }
    }

    #[test]
    fn stands_locations_and_features_take_each_of_their_forms() {
        let text = example_text()
            .replace("stand: { surface: \"coffee table at the origin\" }", "stand: { at: \"corner_a\", offset: { ahead_m: 0.2 } }")
            .replace("features: [\"ai_brain\"]", "features: [{ feature: \"ai_brain\", core_node: \"gpu_box\" }], core_node: \"arm_box\"")
            .replace("\n  objects: {", "\n  locations: { drop: { in: \"output_area\" }, lid: { object: \"crate\", landmark: \"top\" }, bench: { surface: \"workbench\" }, spot: { at: \"corner_b\" } }, objects: {");
        let world = World::parse(&text).unwrap();
        let alpha = &world.robots[&robot("alpha")];
        assert_eq!(
            alpha.stand,
            Some(Stand::At {
                at: name("corner_a"),
                offset: Offset {
                    ahead_m: 0.2,
                    side_m: 0.0
                }
            })
        );
        assert_eq!(
            alpha.join.features,
            vec![Feature::Placed {
                feature: "ai_brain".into(),
                core_node: "gpu_box".into()
            }]
        );
        assert_eq!(alpha.join.core_node.as_deref(), Some("arm_box"));
        assert_eq!(
            world.locations[&name("drop")],
            Location::In(name("output_area"))
        );
        assert_eq!(
            world.locations[&name("lid")],
            Location::Landmark {
                object: name("crate"),
                landmark: "top".into()
            }
        );
        assert_eq!(
            world.locations[&name("bench")],
            Location::Surface("workbench".into())
        );
        assert_eq!(
            world.locations[&name("spot")],
            Location::At(name("corner_b"))
        );
        assert_eq!(World::parse(&world.to_json5().unwrap()).unwrap(), world);
    }

    #[test]
    fn a_stand_writes_its_offset_unless_the_offset_is_zero() {
        let stand = |offset: &str| {
            World::parse(&example_text().replace(
                "stand: { surface: \"coffee table at the origin\" }",
                &format!("stand: {{ surface: \"coffee table at the origin\"{offset} }}"),
            ))
            .unwrap()
        };
        let zero = stand(", offset: { ahead_m: 0, side_m: 0 }");
        assert_eq!(zero, example());
        assert!(!zero.to_json5().unwrap().contains("offset"));
        let ahead = stand(", offset: { ahead_m: 0.2 }").to_json5().unwrap();
        assert!(ahead.contains("ahead_m: 0.2"), "{ahead}");
    }

    #[test]
    fn a_join_is_the_same_whatever_the_order_of_its_features() {
        let world = |features: &str| {
            World::parse(&example_text().replace(r#"features: ["ai_brain"]"#, features)).unwrap()
        };
        let join = |features: &str| world(features).robots[&robot("alpha")].join.clone();
        let ordered = r#"features: ["ai_brain", { feature: "camera", core_node: "gpu_box" }]"#;
        let reordered = r#"features: [{ feature: "camera", core_node: "gpu_box" }, "ai_brain"]"#;
        assert_eq!(join(ordered), join(reordered));
        assert_ne!(join(ordered), join(r#"features: ["ai_brain"]"#));
        assert_ne!(
            join(r#"features: ["ai_brain"]"#),
            join(ordered),
            "a join is not equal to one that holds a feature more"
        );
        let same_features = join(r#"features: ["ai_brain"]"#);
        assert_ne!(
            Join {
                option: "so101_sim".into(),
                ..same_features.clone()
            },
            same_features,
            "another option is another join"
        );
        assert_ne!(
            Join {
                core_node: Some("gpu_box".into()),
                ..same_features.clone()
            },
            same_features,
            "another core node is another join"
        );
        assert_ne!(
            join(ordered),
            join(r#"features: ["ai_brain", { feature: "camera", core_node: "arm_box" }]"#)
        );
        let mut renamed = world(reordered);
        renamed.manifest.name = "world_other".into();
        assert!(world(ordered).same_setup_as(&renamed));
        assert_eq!(
            world(ordered).to_json5().unwrap(),
            world(reordered).to_json5().unwrap(),
            "equal joins write one text"
        );
        assert_eq!(
            world(ordered).document().unwrap().digest(),
            world(reordered).document().unwrap().digest()
        );
    }

    #[test]
    fn same_setup_ignores_the_manifest_alone() {
        let world = example();
        let mut renamed = world.clone();
        renamed.manifest = Manifest {
            name: "world_other".into(),
            revision: Revision::FIRST,
        };
        assert!(world.same_setup_as(&renamed));
        let mut moved = world.clone();
        moved.scene.scale = 2.0;
        assert!(!world.same_setup_as(&moved));
    }
}
