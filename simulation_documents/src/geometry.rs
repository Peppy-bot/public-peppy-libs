//! Poses, points and offsets of a world, in the world frame, metres and
//! radians.

use serde::{Deserialize, Serialize};

use crate::{Error, Name};

/// How far a quaternion's norm may be from 1 and still count as a unit
/// quaternion: the bound `scene_manipulation` applies to a written
/// orientation.
const UNIT_NORM_TOLERANCE: f64 = 1e-3;

/// A point in the world frame, metres.
pub type Point = [f64; 3];

/// A pose in the world frame: a position with a full orientation
/// (`[x, y, z, w]`), with a yaw about +z, or a named pose of the scene.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "PoseFields", into = "PoseFields")]
pub enum Pose {
    Oriented {
        position: Point,
        orientation: [f64; 4],
    },
    Yawed {
        position: Point,
        yaw: f64,
    },
    At(Name),
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a pose, { position, orientation }, { position, yaw } or { at }"
)]
struct PoseFields {
    #[serde(skip_serializing_if = "Option::is_none")]
    position: Option<Point>,
    #[serde(skip_serializing_if = "Option::is_none")]
    orientation: Option<[f64; 4]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    yaw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    at: Option<Name>,
}

impl TryFrom<PoseFields> for Pose {
    type Error = Error;

    fn try_from(fields: PoseFields) -> Result<Pose, Error> {
        match fields {
            PoseFields {
                position: Some(position),
                orientation: Some(orientation),
                yaw: None,
                at: None,
            } => {
                unit_quaternion(orientation)?;
                Ok(Pose::Oriented {
                    position,
                    orientation,
                })
            }
            PoseFields {
                position: Some(position),
                orientation: None,
                yaw: Some(yaw),
                at: None,
            } => Ok(Pose::Yawed { position, yaw }),
            PoseFields {
                position: None,
                orientation: None,
                yaw: None,
                at: Some(at),
            } => Ok(Pose::At(at)),
            _ => Err(Error::rule(
                "a pose is { position, orientation }, { position, yaw } or { at } (a named pose of the scene)",
            )),
        }
    }
}

impl From<Pose> for PoseFields {
    fn from(pose: Pose) -> PoseFields {
        let (position, orientation, yaw, at) = match pose {
            Pose::Oriented {
                position,
                orientation,
            } => (Some(position), Some(orientation), None, None),
            Pose::Yawed { position, yaw } => (Some(position), None, Some(yaw), None),
            Pose::At(name) => (None, None, None, Some(name)),
        };
        PoseFields {
            position,
            orientation,
            yaw,
            at,
        }
    }
}

/// `orientation` when it is a unit quaternion within [`UNIT_NORM_TOLERANCE`].
pub(crate) fn unit_quaternion(orientation: [f64; 4]) -> Result<[f64; 4], Error> {
    let norm = orientation.iter().map(|c| c * c).sum::<f64>().sqrt();
    let unit = (norm - 1.0).abs() <= UNIT_NORM_TOLERANCE;
    if !unit {
        return Err(Error::rule(format!(
            "the orientation [x, y, z, w] has norm {norm}; a unit quaternion has norm 1 within {UNIT_NORM_TOLERANCE}"
        )));
    }
    Ok(orientation)
}

/// Where a robot with a body stands: a position in the world frame and a yaw
/// about +z, as `scene.move_robot` takes them.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a stand, { position, yaw }")]
pub struct Placement {
    pub position: Point,
    pub yaw: f64,
}

/// Where a robot stands relative to a surface's joining spot or a named pose,
/// on the robot's own axes: ahead of it and to its side, metres. The default
/// offset is zero on both axes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "OffsetFields")]
pub struct Offset {
    #[serde(skip_serializing_if = "is_zero")]
    pub ahead_m: f64,
    #[serde(skip_serializing_if = "is_zero")]
    pub side_m: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "an offset, { ahead_m, side_m }")]
struct OffsetFields {
    #[serde(default)]
    ahead_m: f64,
    #[serde(default)]
    side_m: f64,
}

impl TryFrom<OffsetFields> for Offset {
    type Error = Error;

    fn try_from(fields: OffsetFields) -> Result<Offset, Error> {
        for (axis, metres) in [("ahead_m", fields.ahead_m), ("side_m", fields.side_m)] {
            if metres.is_nan() {
                return Err(Error::rule(format!(
                    "{axis} {metres} is not a number; write a finite number"
                )));
            }
        }
        Ok(Offset {
            ahead_m: fields.ahead_m,
            side_m: fields.side_m,
        })
    }
}

fn is_zero(value: &f64) -> bool {
    *value == 0.0
}

/// A named region of a world: a box in the world frame, with its centre, its
/// size along its own axes and a yaw about +z. A point is in the region when
/// it is inside the box or on its faces.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "RegionFields", into = "RegionFields")]
pub struct Region {
    pub centre: Point,
    pub size: [f64; 3],
    pub yaw: f64,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, expecting = "a region, { centre, size, yaw }")]
struct RegionFields {
    centre: Point,
    size: [f64; 3],
    #[serde(default, skip_serializing_if = "is_zero")]
    yaw: f64,
}

impl TryFrom<RegionFields> for Region {
    type Error = Error;

    fn try_from(fields: RegionFields) -> Result<Region, Error> {
        let finite =
            fields.centre.iter().all(|metres| metres.is_finite()) && fields.yaw.is_finite();
        if !finite {
            return Err(Error::rule(
                "the centre and the yaw of a region are finite numbers",
            ));
        }
        let has_a_volume = fields
            .size
            .iter()
            .all(|metres| *metres > 0.0 && metres.is_finite());
        if !has_a_volume {
            return Err(Error::rule(format!(
                "the size {:?} of a region has a side that is not above 0; write [x, y, z], each above 0 m",
                fields.size
            )));
        }
        Ok(Region {
            centre: fields.centre,
            size: fields.size,
            yaw: fields.yaw,
        })
    }
}

impl From<Region> for RegionFields {
    fn from(region: Region) -> RegionFields {
        RegionFields {
            centre: region.centre,
            size: region.size,
            yaw: region.yaw,
        }
    }
}

impl Region {
    /// Whether `point`, in the world frame, is inside the region or on its
    /// faces.
    pub fn contains(&self, point: Point) -> bool {
        let local = self.local(point);
        (0..3).all(|axis| local[axis].abs() <= self.size[axis] / 2.0)
    }

    /// The point of the region at `fractions` of its size along each of its
    /// axes, from its lowest corner (0) to its highest (1). A uniform draw of
    /// the three fractions gives a uniform point of the region.
    pub fn point_at(&self, fractions: [f64; 3]) -> Point {
        let along = |axis: usize| (fractions[axis] - 0.5) * self.size[axis];
        let (x, y, z) = (along(0), along(1), along(2));
        let (sin, cos) = self.yaw.sin_cos();
        [
            self.centre[0] + cos * x - sin * y,
            self.centre[1] + sin * x + cos * y,
            self.centre[2] + z,
        ]
    }

    /// `point` on the axes of the region, from its centre.
    fn local(&self, point: Point) -> Point {
        let (dx, dy, dz) = (
            point[0] - self.centre[0],
            point[1] - self.centre[1],
            point[2] - self.centre[2],
        );
        let (sin, cos) = self.yaw.sin_cos();
        [cos * dx + sin * dy, -sin * dx + cos * dy, dz]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region(text: &str) -> Result<Region, String> {
        serde_json::from_str::<Region>(text).map_err(|error| error.to_string())
    }

    #[test]
    fn a_region_is_a_box_with_a_volume_and_an_omitted_yaw_is_zero() {
        let stock = region(r#"{ "centre": [1, 2, 0.5], "size": [0.6, 0.4, 1] }"#).unwrap();
        assert_eq!(
            stock,
            Region {
                centre: [1.0, 2.0, 0.5],
                size: [0.6, 0.4, 1.0],
                yaw: 0.0,
            }
        );
        assert_eq!(
            serde_json::to_string(&stock).unwrap(),
            r#"{"centre":[1.0,2.0,0.5],"size":[0.6,0.4,1.0]}"#
        );
        let flat = region(r#"{ "centre": [0, 0, 0], "size": [1, 0, 1] }"#).unwrap_err();
        assert!(
            flat.starts_with("the size [1.0, 0.0, 1.0] of a region has a side that is not above 0"),
            "{flat}"
        );
        let other_field =
            region(r#"{ "centre": [0, 0, 0], "size": [1, 1, 1], "pose": 1 }"#).unwrap_err();
        assert!(
            other_field
                .starts_with("unknown field `pose`, expected one of `centre`, `size`, `yaw`"),
            "{other_field}"
        );
    }

    #[test]
    fn a_yawed_region_holds_the_points_of_its_own_axes() {
        let quarter_turn = Region {
            centre: [1.0, 1.0, 0.0],
            size: [2.0, 0.2, 1.0],
            yaw: std::f64::consts::FRAC_PI_2,
        };
        assert!(quarter_turn.contains([1.0, 1.9, 0.0]));
        assert!(!quarter_turn.contains([1.9, 1.0, 0.0]));
        assert!(!quarter_turn.contains([1.0, 1.0, 0.6]));
        let corner = quarter_turn.point_at([1.0, 1.0, 1.0]);
        let expected = [0.9, 2.0, 0.5];
        assert!(
            (0..3).all(|axis| (corner[axis] - expected[axis]).abs() < 1e-12),
            "{corner:?}"
        );
        assert!(quarter_turn.contains(quarter_turn.point_at([0.3, 0.7, 0.1])));
    }

    /// A pose names the same forms whether a value has another type or its
    /// fields form none, and an offset of another type names its fields.
    #[test]
    fn a_pose_or_an_offset_of_another_type_names_the_forms_to_write() {
        use crate::world::tests::{forms_named, refusal_of};
        let other_type = refusal_of::<Pose>("1");
        assert!(
            other_type.ends_with(
                "expected a pose, { position, orientation }, { position, yaw } or { at }"
            ),
            "{other_type}"
        );
        assert_eq!(
            forms_named(&other_type),
            forms_named(&refusal_of::<Pose>("{}"))
        );
        let offset = refusal_of::<Offset>("1");
        assert!(
            offset.ends_with("expected an offset, { ahead_m, side_m }"),
            "{offset}"
        );
    }

    fn pose(text: &str) -> Result<Pose, String> {
        serde_json::from_str::<Pose>(text).map_err(|error| error.to_string())
    }

    #[test]
    fn the_three_forms_of_a_pose_parse_and_other_mixes_are_refused() {
        assert_eq!(
            pose(r#"{"position": [1, 2, 3], "orientation": [0, 0, 0, 1]}"#).unwrap(),
            Pose::Oriented {
                position: [1.0, 2.0, 3.0],
                orientation: [0.0, 0.0, 0.0, 1.0]
            }
        );
        assert_eq!(
            pose(r#"{"position": [1, 2, 3], "yaw": 0.5}"#).unwrap(),
            Pose::Yawed {
                position: [1.0, 2.0, 3.0],
                yaw: 0.5
            }
        );
        assert_eq!(
            pose(r#"{"at": "corner_a"}"#).unwrap(),
            Pose::At(Name::parse("corner_a").unwrap())
        );
        let mixed = pose(r#"{"position": [1, 2, 3], "yaw": 0.5, "at": "corner_a"}"#).unwrap_err();
        assert!(
            mixed.contains("a pose is { position, orientation }, { position, yaw } or { at }"),
            "{mixed}"
        );
        assert!(pose(r#"{"position": [1, 2, 3]}"#).is_err());
        let oriented_and_yawed =
            pose(r#"{"position": [1, 2, 3], "orientation": [0, 0, 0, 1], "yaw": 0.5}"#)
                .unwrap_err();
        assert!(
            oriented_and_yawed
                .starts_with("a pose is { position, orientation }, { position, yaw } or { at }"),
            "{oriented_and_yawed}"
        );
        assert!(
            pose(r#"{"position": [1, 2, 3], "yaw": 0.5, "scale": 2}"#)
                .unwrap_err()
                .contains("unknown field `scale`")
        );
    }

    #[test]
    fn an_orientation_is_a_unit_quaternion_within_a_thousandth() {
        let error = pose(r#"{"position": [0, 0, 0], "orientation": [0, 0, 0, 2]}"#).unwrap_err();
        assert!(
            error.contains("has norm 2; a unit quaternion has norm 1 within 0.001"),
            "{error}"
        );
        assert!(pose(r#"{"position": [0, 0, 0], "orientation": [0.7071, 0, 0, 0.7071]}"#).is_ok());
        assert!(pose(r#"{"position": [0, 0, 0], "orientation": [0, 0, 0, 1.0011]}"#).is_err());
    }

    #[test]
    fn a_nan_orientation_or_offset_is_refused() {
        let orientation =
            json5::from_str::<Pose>("{ position: [0, 0, 0], orientation: [NaN, 0, 0, 1] }")
                .unwrap_err()
                .to_string();
        assert!(
            orientation.starts_with(
                "the orientation [x, y, z, w] has norm NaN; a unit quaternion has norm 1 within 0.001"
            ),
            "{orientation}"
        );
        for (text, refusal) in [
            (
                "{ ahead_m: NaN }",
                "ahead_m NaN is not a number; write a finite number",
            ),
            (
                "{ ahead_m: 0.2, side_m: NaN }",
                "side_m NaN is not a number; write a finite number",
            ),
        ] {
            assert_eq!(
                json5::from_str::<Offset>(text).unwrap_err().to_string(),
                refusal,
                "{text}"
            );
        }
    }

    #[test]
    fn a_pose_writes_back_in_its_own_form() {
        let yawed = Pose::Yawed {
            position: [0.55, 0.0, 0.39],
            yaw: 0.0,
        };
        assert_eq!(
            serde_json::to_string(&yawed).unwrap(),
            r#"{"position":[0.55,0.0,0.39],"yaw":0.0}"#
        );
        assert_eq!(
            serde_json::to_string(&Pose::At(Name::parse("corner_a").unwrap())).unwrap(),
            r#"{"at":"corner_a"}"#
        );
    }

    #[test]
    fn an_omitted_offset_is_zero_and_zero_is_not_written() {
        assert_eq!(
            serde_json::from_str::<Offset>("{}").unwrap(),
            Offset::default()
        );
        assert!(
            serde_json::from_str::<Offset>(r#"{"ahead_m": 0.2, "up_m": 1}"#)
                .unwrap_err()
                .to_string()
                .contains("unknown field `up_m`")
        );
        assert_eq!(
            serde_json::to_string(&Offset {
                ahead_m: 0.2,
                side_m: 0.0
            })
            .unwrap(),
            r#"{"ahead_m":0.2}"#
        );
    }
}
