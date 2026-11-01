//! The setup record a simulation keeps: the last value of each setup call.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::daylight::Daylight;
use crate::geometry::Region;
use crate::world::{Light, Location, Material, Object, Scene, SceneObject, Stand};
use crate::{Error, Name, RobotName, World, text};

/// The last value of each setup call the simulation took, without the joins
/// of the robots, which the stack listing gives.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a setup record, { scene, scene_objects, objects, lighting, materials, daylight, regions, locations, stands }"
)]
pub struct SetupRecord {
    pub scene: Scene,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub scene_objects: BTreeMap<Name, SceneObject>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub objects: BTreeMap<Name, Object>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub lighting: BTreeMap<String, Light>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub materials: BTreeMap<String, Material>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub daylight: Option<Daylight>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub regions: BTreeMap<Name, Region>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub locations: BTreeMap<Name, Location>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub stands: BTreeMap<RobotName, Stand>,
}

impl SetupRecord {
    /// The record of a simulation that started on `scene` with nothing set up.
    pub fn at_start(scene: Scene) -> SetupRecord {
        SetupRecord {
            scene,
            scene_objects: BTreeMap::new(),
            objects: BTreeMap::new(),
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
        text::to_json5(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::example;

    #[test]
    fn a_record_at_the_start_holds_the_scene_alone() {
        let scene = Scene {
            asset: "scene/default".into(),
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
                "scene_objects.clock: typo: unknown field `typo`, expected `pose` or `controls`",
            ),
        ] {
            assert_eq!(
                SetupRecord::parse(&text(bad)).unwrap_err().to_string(),
                refusal,
                "{bad}"
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
