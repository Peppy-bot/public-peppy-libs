//! The locations a skill knows through locations:v1: each as `teach` and
//! `list` give it.

use frame_tree::FrameName;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::names::Name;

/// A location as `teach` and `list` of locations:v1 give it: its frame,
/// `world` in a simulation or a frame of its robot's tree as transforms names
/// frames; its position in meters, its orientation as a unit quaternion
/// `[x, y, z, w]`, and its extent in meters along its axes. A location holds
/// at every time, so its record holds no timestamp. The z of the location is
/// the normal of a landmark or a surface (up), its x runs along `size[0]`,
/// the axis of a landmark or the first edge of a surface, and its y completes
/// the right hand.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KnownLocation {
    pub name: Name,
    pub kind: LocationKind,
    #[serde(with = "location_frame")]
    pub frame: FrameName,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub size: [f64; 3],
}

/// The frame of a location as its text: refused when it is empty, saying
/// what to name.
mod location_frame {
    use super::*;

    pub fn serialize<S: Serializer>(frame: &FrameName, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(frame.as_str())
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<FrameName, D::Error> {
        let text = String::deserialize(deserializer)?;
        FrameName::parse(&text).map_err(|_| {
            serde::de::Error::custom(
                "the frame of a location is empty: name world in a simulation, or a frame of \
                 its robot's tree, such as map",
            )
        })
    }
}

/// What a location is, as the `kind` of locations:v1 names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LocationKind {
    /// A pose, with no extent.
    Pose,
    /// The landmark of an object.
    Landmark,
    /// A level surface.
    Surface,
    /// A region of space.
    Region,
}

impl LocationKind {
    pub const ALL: [LocationKind; 4] = [
        LocationKind::Pose,
        LocationKind::Landmark,
        LocationKind::Surface,
        LocationKind::Region,
    ];

    /// The kind that `name` names in the `kind` field, if any.
    pub fn parse(name: &str) -> Option<LocationKind> {
        LocationKind::ALL
            .into_iter()
            .find(|kind| kind.name() == name)
    }

    /// The name of the kind in the `kind` field.
    pub fn name(self) -> &'static str {
        match self {
            LocationKind::Pose => "pose",
            LocationKind::Landmark => "landmark",
            LocationKind::Surface => "surface",
            LocationKind::Region => "region",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn each_kind_parses_from_its_name_and_serializes_as_it() {
        for kind in LocationKind::ALL {
            assert_eq!(LocationKind::parse(kind.name()), Some(kind));
            assert_eq!(serde_json::to_value(kind).unwrap(), json!(kind.name()));
        }
        for other in ["", "Pose", "point", "surface "] {
            assert_eq!(LocationKind::parse(other), None, "{other:?}");
        }
    }

    fn shelf() -> serde_json::Value {
        json!({
            "name": "shelf",
            "kind": "surface",
            "frame": "map",
            "position": [0.4, -0.1, 0.75],
            "orientation": [0.0, 0.0, 0.6, 0.8],
            "size": [0.6, 0.3, 0.0],
        })
    }

    /// The location `shelf` with `field` set to `value`, read back.
    fn read_with(field: &str, value: serde_json::Value) -> Result<KnownLocation, String> {
        let mut written = shelf();
        written[field] = value;
        serde_json::from_value::<KnownLocation>(written).map_err(|error| error.to_string())
    }

    #[test]
    fn a_known_location_reads_back_as_it_was_written() {
        let shelf_location = KnownLocation {
            name: Name::parse("shelf").unwrap(),
            kind: LocationKind::Surface,
            frame: FrameName::parse("map").unwrap(),
            position: [0.4, -0.1, 0.75],
            orientation: [0.0, 0.0, 0.6, 0.8],
            size: [0.6, 0.3, 0.0],
        };
        assert_eq!(serde_json::to_value(&shelf_location).unwrap(), shelf());
        let read: KnownLocation = serde_json::from_value(shelf()).unwrap();
        assert_eq!(read, shelf_location);
        assert!(read_with("name", json!("2nd_shelf")).is_err());
        let mut unframed = shelf();
        unframed.as_object_mut().unwrap().remove("frame");
        assert!(serde_json::from_value::<KnownLocation>(unframed).is_err());
    }

    #[test]
    fn a_location_in_an_empty_frame_is_refused_saying_what_to_name() {
        let refusal = read_with("frame", json!("")).unwrap_err();
        assert!(
            refusal.starts_with(
                "the frame of a location is empty: name world in a simulation, or a frame of its \
                 robot's tree, such as map"
            ),
            "{refusal}"
        );
        assert_eq!(
            read_with("frame", json!("world")).unwrap().frame.as_str(),
            "world"
        );
    }
}
