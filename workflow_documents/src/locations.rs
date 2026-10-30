//! The locations a skill knows through locations:v1: each as `teach` takes
//! it and `list` gives it.

use serde::{Deserialize, Serialize};

use crate::names::Name;

/// A location as locations:v1 carries it, in the world frame of the
/// simulation or in the frame the work cell measures its places in: its
/// position in meters, its orientation as a unit quaternion `[x, y, z, w]`,
/// and its extent in meters along its axes. The z of the location is the
/// normal of a landmark or a surface (up), its x runs along `size[0]`, the
/// axis of a landmark or the first edge of a surface, and its y completes the
/// right hand.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KnownLocation {
    pub name: Name,
    pub kind: LocationKind,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub size: [f64; 3],
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

    #[test]
    fn a_known_location_reads_back_as_it_was_written() {
        let shelf = KnownLocation {
            name: Name::parse("shelf").unwrap(),
            kind: LocationKind::Surface,
            position: [0.4, -0.1, 0.75],
            orientation: [0.0, 0.0, 0.6, 0.8],
            size: [0.6, 0.3, 0.0],
        };
        let written = serde_json::to_value(&shelf).unwrap();
        assert_eq!(
            written,
            json!({
                "name": "shelf",
                "kind": "surface",
                "position": [0.4, -0.1, 0.75],
                "orientation": [0.0, 0.0, 0.6, 0.8],
                "size": [0.6, 0.3, 0.0],
            })
        );
        let read: KnownLocation = serde_json::from_value(written).unwrap();
        assert_eq!(read, shelf);
        let unnamed = json!({ "name": "2nd_shelf", "kind": "surface", "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0], "size": [0.0, 0.0, 0.0] });
        assert!(serde_json::from_value::<KnownLocation>(unnamed).is_err());
    }
}
