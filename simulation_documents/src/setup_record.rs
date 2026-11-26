//! The setup record a simulation keeps: the last value of each setup call.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::daylight::Daylight;
use crate::geometry::Region;
use crate::world::{
    Light, Location, Material, Object, RecordedObject, Riser, Scene, SceneObject, Stand,
    check_asset_kinds, objects_and_risers, recorded_objects,
};
use crate::{Error, Name, RobotName, World, text};

/// The last value of each setup call the simulation took, without the joins
/// of the robots, which the stack listing gives.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(from = "RecordFields")]
pub struct SetupRecord {
    pub scene: Scene,
    pub scene_objects: BTreeMap<Name, SceneObject>,
    pub objects: BTreeMap<Name, Object>,
    /// The risers that their stand or their raise left, which the record
    /// writes among its `objects`.
    pub risers: BTreeMap<Name, Riser>,
    pub lighting: BTreeMap<String, Light>,
    pub materials: BTreeMap<String, Material>,
    pub daylight: Option<Daylight>,
    pub regions: BTreeMap<Name, Region>,
    pub locations: BTreeMap<Name, Location>,
    pub stands: BTreeMap<RobotName, Stand>,
}

/// A setup record as its text writes it: its `objects` hold the objects and
/// the risers.
#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a setup record, { scene, scene_objects, objects, lighting, materials, daylight, regions, locations, stands }"
)]
struct RecordFields {
    scene: Scene,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    scene_objects: BTreeMap<Name, SceneObject>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    objects: BTreeMap<Name, RecordedObject>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    lighting: BTreeMap<String, Light>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    materials: BTreeMap<String, Material>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    daylight: Option<Daylight>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    regions: BTreeMap<Name, Region>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    locations: BTreeMap<Name, Location>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    stands: BTreeMap<RobotName, Stand>,
}

impl From<RecordFields> for SetupRecord {
    fn from(fields: RecordFields) -> SetupRecord {
        let (objects, risers) = objects_and_risers(fields.objects);
        SetupRecord {
            scene: fields.scene,
            scene_objects: fields.scene_objects,
            objects,
            risers,
            lighting: fields.lighting,
            materials: fields.materials,
            daylight: fields.daylight,
            regions: fields.regions,
            locations: fields.locations,
            stands: fields.stands,
        }
    }
}

/// A record writes the fields of its text, with the refusal of a name that
/// an object and a riser share.
impl Serialize for SetupRecord {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        RecordFields::try_from(self.clone())
            .map_err(serde::ser::Error::custom)?
            .serialize(serializer)
    }
}

impl TryFrom<SetupRecord> for RecordFields {
    type Error = Error;

    fn try_from(record: SetupRecord) -> Result<RecordFields, Error> {
        check_asset_kinds(&record.scene, &record.objects)?;
        Ok(RecordFields {
            scene: record.scene,
            scene_objects: record.scene_objects,
            objects: recorded_objects(record.objects, record.risers)?,
            lighting: record.lighting,
            materials: record.materials,
            daylight: record.daylight,
            regions: record.regions,
            locations: record.locations,
            stands: record.stands,
        })
    }
}

impl SetupRecord {
    /// The record of a simulation that started on `scene` with nothing set up.
    pub fn at_start(scene: Scene) -> SetupRecord {
        SetupRecord {
            scene,
            scene_objects: BTreeMap::new(),
            objects: BTreeMap::new(),
            risers: BTreeMap::new(),
            lighting: BTreeMap::new(),
            materials: BTreeMap::new(),
            daylight: None,
            regions: BTreeMap::new(),
            locations: BTreeMap::new(),
            stands: BTreeMap::new(),
        }
    }

    /// The record of a world that `build` built.
    pub fn built(world: &World) -> SetupRecord {
        SetupRecord {
            scene: world.scene.clone(),
            scene_objects: world.scene_objects.clone(),
            objects: world.objects.clone(),
            risers: world.risers.clone(),
            lighting: world.lighting.clone(),
            materials: world.materials.clone(),
            daylight: world.daylight.clone(),
            regions: world.regions.clone(),
            locations: world.locations.clone(),
            stands: world
                .robots
                .iter()
                .filter_map(|(name, robot)| Some((name.clone(), robot.stand.clone()?)))
                .collect(),
        }
    }

    /// Parses the JSON5 text of a record.
    pub fn parse(text: &str) -> Result<SetupRecord, Error> {
        text::from_json5(text)
    }

    /// The record as JSON5 text, with the refusals of [`World::to_json5`].
    pub fn to_json5(&self) -> Result<String, Error> {
        text::to_json5(&RecordFields::try_from(self.clone())?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::{asset, example};

    #[test]
    fn a_record_at_the_start_holds_the_scene_alone() {
        let scene = Scene {
            asset: asset("scene/default"),
            scale: 1.0,
        };
        let record = SetupRecord::at_start(scene.clone());
        assert_eq!(record.scene, scene);
        assert!(
            record.scene_objects.is_empty()
                && record.objects.is_empty()
                && record.lighting.is_empty()
                && record.materials.is_empty()
                && record.daylight.is_none()
                && record.regions.is_empty()
                && record.locations.is_empty()
                && record.stands.is_empty()
        );
    }

    #[test]
    fn the_built_record_of_the_worked_example_survives_its_text() {
        let record = SetupRecord::built(&example());
        assert_eq!(
            SetupRecord::parse(&record.to_json5().unwrap()).unwrap(),
            record
        );
        assert_eq!(
            record
                .stands
                .keys()
                .map(RobotName::as_str)
                .collect::<Vec<_>>(),
            ["alpha"]
        );
        assert!(record.objects.contains_key(&Name::parse("crate").unwrap()));
        assert_eq!(
            record.scene_objects[&Name::parse("mantel_clock_01").unwrap()],
            SceneObject::Removed
        );
    }

    /// A scene object of the record reads through the parser of the marks of
    /// a world document's entry, with its refusals.
    #[test]
    fn a_scene_object_of_the_record_is_removed_or_placed() {
        let text = |scene_object| {
            format!(
                r#"{{ scene: {{ asset: "scene/default" }}, scene_objects: {{ clock: {scene_object} }} }}"#
            )
        };
        assert_eq!(
            SetupRecord::parse(&text("{ removed: true }"))
                .unwrap()
                .scene_objects[&Name::parse("clock").unwrap()],
            SceneObject::Removed
        );
        assert!(SetupRecord::parse(&text("{ pose: { at: \"corner_a\" } }")).is_ok());
        let removed_form = "scene_objects.clock: `removed` is `true` for a removed entry; write `removed: true` or the entry's fields";
        for (bad, refusal) in [
            ("{ removed: false }", removed_form),
            ("{ removed: true, controls: {} }", removed_form),
            (
                "{ removed: true, pose: { at: \"corner_a\" } }",
                removed_form,
            ),
            (
                "{ kept: true }",
                "scene_objects.clock: `kept` drops an entry of a base, in a world document that extends one",
            ),
            (
                "{ pose: { at: \"corner_a\" }, typo: 1 }",
                "scene_objects.clock: typo: unknown field `typo`, expected one of `pose`, `scale`, `controls`, `raise_m`",
            ),
        ] {
            assert_eq!(
                SetupRecord::parse(&text(bad)).unwrap_err().to_string(),
                refusal,
                "{bad}"
            );
        }
    }

    /// A placed scene object writes the scale it has of its own, and none
    /// for the scale that its scene places it at: a record without a scale
    /// reads as it did, and writes no scale back.
    #[test]
    fn a_placed_scene_object_writes_its_scale_only_when_it_has_one() {
        let text = |scale: &str| {
            format!(
                r#"{{ scene: {{ asset: "scene/default" }}, scene_objects: {{ clock: {{ pose: {{ position: [1, 2, 0.5], yaw: 0 }}{scale} }} }} }}"#
            )
        };
        let clock = Name::parse("clock").unwrap();
        let scale_of = |record: &SetupRecord| match &record.scene_objects[&clock] {
            SceneObject::Placed { scale, .. } => *scale,
            SceneObject::Removed => panic!("the clock is placed"),
        };

        let as_placed = SetupRecord::parse(&text("")).unwrap();
        assert_eq!(scale_of(&as_placed), None);
        let written = as_placed.to_json5().unwrap();
        assert!(!written.contains("scale"), "{written}");
        assert_eq!(SetupRecord::parse(&written).unwrap(), as_placed);

        let resized = SetupRecord::parse(&text(", scale: 1.5")).unwrap();
        assert_eq!(scale_of(&resized), Some(1.5));
        let written = resized.to_json5().unwrap();
        assert!(written.contains("scale: 1.5"), "{written}");
        assert_eq!(SetupRecord::parse(&written).unwrap(), resized);

        // A scale of 1 is a scale of its own too: the scene can place the
        // object at another one.
        let authored = SetupRecord::parse(&text(", scale: 1")).unwrap();
        assert_eq!(scale_of(&authored), Some(1.0));
        assert!(authored.to_json5().unwrap().contains("scale: 1"));
    }

    /// A scale of a placed scene object that is not above 0 is refused at
    /// its entry. A document holds finite numbers only, so a scale that is
    /// not finite is refused as each such number of a record is.
    #[test]
    fn a_scale_of_a_placed_scene_object_is_a_finite_number_above_zero() {
        const NOT_A_SCALE: &str = " is not above 0; write a scale above 0, or leave it out for the scale that the scene places the object at";
        let record = |scale: &str| {
            format!(
                r#"{{ scene: {{ asset: "scene/default" }}, scene_objects: {{ clock: {{ pose: {{ at: "corner_a" }}, scale: {scale} }} }} }}"#
            )
        };
        for scale in ["0", "-1"] {
            assert_eq!(
                SetupRecord::parse(&record(scale)).unwrap_err().to_string(),
                format!("scene_objects.clock: the scale {scale}{NOT_A_SCALE}"),
            );
        }
        for (scale, not_finite) in [
            ("NaN", "the number NaN is not finite"),
            (
                "Infinity",
                "the number is an infinity or beyond the range of a double",
            ),
        ] {
            assert_eq!(
                SetupRecord::parse(&record(scale)).unwrap_err().to_string(),
                format!(
                    "scene_objects.clock.scale: {not_finite}; a document holds finite numbers only"
                ),
            );
        }
    }

    #[test]
    fn a_record_with_a_nan_is_refused_at_its_path() {
        let mut record = SetupRecord::built(&example());
        record.scene.scale = f64::NAN;
        assert_eq!(
            record.to_json5().unwrap_err().to_string(),
            "scene.scale: the number NaN is not finite; a document holds finite numbers only"
        );
        let mut record = SetupRecord::built(&example());
        record
            .objects
            .get_mut(&Name::parse("crate").unwrap())
            .unwrap()
            .mass = Some(f64::NAN);
        assert_eq!(
            record.to_json5().unwrap_err().to_string(),
            "objects.crate.mass: the number NaN is not finite; a document holds finite numbers only"
        );
    }
}
