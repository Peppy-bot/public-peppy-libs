//! tf2's `Transform`: where one frame sits in another, as a rotation matrix and an origin, a
//! position in meters and a unit quaternion [x, y, z, w] as camera_geometry:v1 carries one. A
//! transform carries a point into the frame it is placed in, composes with another and inverts,
//! each in tf2's arithmetic ([`crate::arithmetic`]), operation for operation, so a point lands on
//! the bits tf2 gives it. occupancy_map's parity with MoveIt's point cloud updater, which places
//! each point by tf2, compares those bits. A transform also gives its orientation back as a
//! quaternion and runs a share of the way to another, for a sensor that moves while it measures.

use std::fmt;

use crate::arithmetic::{self, Rotation, Vector};

/// How far from 1 the length of a transform's quaternion may be. A quaternion that is not of
/// unit length scales every point it turns.
const UNIT_QUATERNION_TOLERANCE: f64 = 1e-6;

/// Where one frame, the placed one, sits in another: the rotation of the
/// placed frame as tf2 builds the matrix of a quaternion, and its origin. A
/// point `p` of the placed frame is `R * p + origin` in the other.
#[derive(Debug, Clone, Copy)]
pub struct Transform {
    rows: Rotation,
    origin: Vector,
}

/// A position and an orientation that make no transform: a coordinate that is not
/// a number, or a quaternion that is not of unit length.
#[derive(Debug)]
pub struct NotATransform {
    position: [f64; 3],
    orientation: [f64; 4],
}

impl fmt::Display for NotATransform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "position {:?} and orientation {:?} make no transform: give a position of numbers \
             and a unit quaternion [x, y, z, w]",
            self.position, self.orientation
        )
    }
}

impl std::error::Error for NotATransform {}

impl Transform {
    /// The pose whose origin is `position` and whose orientation is the unit
    /// quaternion `orientation`, [x, y, z, w], with its matrix built as tf2's
    /// `Matrix3x3::setRotation` builds it.
    pub fn new(position: [f64; 3], orientation: [f64; 4]) -> Result<Self, NotATransform> {
        if !is_valid(&position, &orientation) {
            return Err(NotATransform {
                position,
                orientation,
            });
        }
        Ok(Self {
            rows: arithmetic::rotation_of(orientation),
            origin: position,
        })
    }

    /// The placed frame's origin, in the frame it is placed in.
    pub fn origin(&self) -> [f64; 3] {
        self.origin
    }

    /// A point of the placed frame in the frame it is placed in: each row of
    /// the rotation dotted with the point, plus the origin.
    pub fn apply(&self, point: [f64; 3]) -> [f64; 3] {
        let turned = arithmetic::rotated(&self.rows, point);
        std::array::from_fn(|i| turned[i] + self.origin[i])
    }

    /// The pose of the frame `inner` places in this pose's placed frame, in
    /// the frame this pose is placed in: a point goes through `inner`, then
    /// through this pose. tf2's product of two transforms.
    pub fn compose(&self, inner: &Transform) -> Transform {
        Transform {
            rows: arithmetic::rotation_product(&self.rows, &inner.rows),
            origin: self.apply(inner.origin),
        }
    }

    /// The placed frame's orientation, a unit quaternion [x, y, z, w], from the
    /// rotation as tf2's `Matrix3x3::getRotation` takes it: from the trace when
    /// it is above 0, else from the largest diagonal entry.
    pub fn orientation(&self) -> [f64; 4] {
        arithmetic::quaternion_of(&self.rows)
    }

    /// The pose `ratio` of the way from this pose to `to`, 0 this pose and 1
    /// `to`: the origins interpolated as tf2's `Vector3::setInterpolate3`, and
    /// the orientations by tf2's `Quaternion::slerp`, the shorter way round. A
    /// ratio that is not finite, or one that carries the origin past the
    /// largest double, makes no pose: the error gives the position and the
    /// orientation.
    pub fn interpolated(&self, to: &Transform, ratio: f64) -> Result<Transform, NotATransform> {
        let origin = arithmetic::interpolated(self.origin, to.origin, ratio);
        let orientation = arithmetic::slerp(self.orientation(), to.orientation(), ratio);
        Transform::new(origin, orientation)
    }

    /// The pose of the frame this pose is placed in, in the placed frame:
    /// tf2's inverse, the rotation transposed and the origin carried back
    /// through it and negated.
    pub fn inverse(&self) -> Transform {
        let rows = arithmetic::transposed(&self.rows);
        let back = self.origin.map(|c| -c);
        Transform {
            rows,
            origin: arithmetic::rotated(&rows, back),
        }
    }
}

/// A transform a consumer can apply: a position made of numbers and a quaternion of unit
/// length.
fn is_valid(position: &[f64; 3], orientation: &[f64; 4]) -> bool {
    let length = orientation.iter().map(|c| c * c).sum::<f64>().sqrt();
    position.iter().all(|c| c.is_finite()) && (length - 1.0).abs() <= UNIT_QUATERNION_TOLERANCE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arithmetic::{quaternion_of_yaw, yaw_of_quaternion};

    /// Whether every coordinate of `a` lies within `tolerance` of `b`'s.
    fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < tolerance)
    }

    /// A pose with a turn about each axis and an offset on each.
    fn tilted() -> Transform {
        let norm = (0.1_f64 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9).sqrt();
        Transform::new([0.4, -0.5, 0.6], [0.1, 0.2, 0.3, 0.9].map(|c| c / norm))
            .expect("a unit quaternion")
    }

    #[test]
    fn a_pose_turns_a_point_and_moves_it_by_its_origin() {
        // A quarter turn about +Z takes +X to +Y, then the origin is added.
        let half = std::f64::consts::FRAC_PI_4;
        let pose = Transform::new([0.015, 0.0, -0.001], [0.0, 0.0, half.sin(), half.cos()])
            .expect("a unit quaternion");
        assert!(close(
            pose.apply([1.0, 0.0, 2.0]),
            [0.015, 1.0, 1.999],
            1e-12
        ));
        assert_eq!(pose.origin(), [0.015, 0.0, -0.001]);
        let identity = Transform::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("the identity");
        assert_eq!(identity.apply([0.3, -0.2, 1.5]), [0.3, -0.2, 1.5]);
    }

    // A composed pose carries a point as its two poses do one after the
    // other, inner first, and the inverse carries it back.
    #[test]
    fn composing_runs_the_inner_pose_first_and_the_inverse_undoes_a_pose() {
        let outer = tilted();
        let half = 0.35_f64;
        let inner = Transform::new([-0.06, 0.01, 0.002], [0.0, half.sin(), 0.0, half.cos()])
            .expect("a unit quaternion");
        let point = [0.7, -0.3, 1.1];
        let composed = outer.compose(&inner);
        assert!(close(
            composed.apply(point),
            outer.apply(inner.apply(point)),
            1e-12
        ));
        assert!(!close(
            composed.apply(point),
            inner.apply(outer.apply(point)),
            1e-12
        ));
        assert!(close(
            outer.inverse().apply(outer.apply(point)),
            point,
            1e-12
        ));
        assert!(close(
            outer.apply(outer.inverse().apply(point)),
            point,
            1e-12
        ));
    }

    /// The pose turned by `angle_rad` about the unit axis `axis`.
    fn turned_about(axis: [f64; 3], angle_rad: f64) -> Transform {
        let (sin, cos) = (angle_rad / 2.0).sin_cos();
        Transform::new(
            [0.4, -0.5, 0.6],
            [axis[0] * sin, axis[1] * sin, axis[2] * sin, cos],
        )
        .expect("a unit quaternion")
    }

    // The quaternion a pose gives back makes the same pose, by either branch of
    // tf2's extraction: a turn whose trace lies above 0, and a half turn, whose
    // trace lies below.
    #[test]
    fn a_pose_gives_back_its_orientation() {
        let half_turn =
            Transform::new([0.0; 3], [1.0, 0.0, 0.0, 0.0]).expect("a half turn about +X");
        for pose in [tilted(), half_turn] {
            let again =
                Transform::new(pose.origin(), pose.orientation()).expect("a unit quaternion");
            let point = [0.3, -1.2, 0.8];
            assert!(close(again.apply(point), pose.apply(point), 1e-12));
        }
        assert_eq!(half_turn.orientation(), [1.0, 0.0, 0.0, 0.0]);
    }

    // A turn of 120 degrees or more has a trace of at most 0, so tf2 reads its
    // quaternion from its largest diagonal entry: the entry of +Z for a heading
    // past 120 degrees either way, of +X and +Y for turns about them, and of
    // +Z, reached past the other two, for a turn about a general axis. Each
    // quaternion makes the same pose again.
    #[test]
    fn a_turn_past_120_degrees_gives_back_its_orientation() {
        let general = {
            let norm = (0.2_f64 * 0.2 + 0.5 * 0.5 + 0.84 * 0.84).sqrt();
            [0.2 / norm, 0.5 / norm, 0.84 / norm]
        };
        let turns = [
            ([0.0, 0.0, 1.0], 2.2),
            ([0.0, 0.0, 1.0], 2.6),
            ([0.0, 0.0, 1.0], 3.0),
            ([0.0, 0.0, 1.0], -2.5),
            ([1.0, 0.0, 0.0], 2.6),
            ([0.0, 1.0, 0.0], 2.6),
            (general, 2.6),
        ];
        let point = [0.3, -1.2, 0.8];
        for (axis, angle_rad) in turns {
            let pose = turned_about(axis, angle_rad);
            let again =
                Transform::new(pose.origin(), pose.orientation()).expect("a unit quaternion");
            assert!(
                close(again.apply(point), pose.apply(point), 1e-12),
                "{angle_rad} rad about {axis:?}"
            );
        }
    }

    // An interpolated pose runs from the first pose at 0 to the second at 1,
    // its origin along the line between them and its heading turning by the
    // share of the turn between them, the shorter way round whichever sign the
    // second quaternion carries.
    #[test]
    fn an_interpolated_pose_runs_from_one_pose_to_the_other() {
        let start = Transform::new([1.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let quarter = std::f64::consts::FRAC_PI_4;
        let turned = [0.0, 0.0, quarter.sin(), quarter.cos()];
        for orientation in [turned, turned.map(|c| -c)] {
            let end =
                Transform::new([3.0, -2.0, 0.5], orientation).expect("a quarter turn about +Z");
            let point = [1.0, 0.0, 0.0];
            assert!(close(
                start.interpolated(&end, 0.0).unwrap().apply(point),
                start.apply(point),
                1e-12
            ));
            assert!(close(
                start.interpolated(&end, 1.0).unwrap().apply(point),
                end.apply(point),
                1e-12
            ));
            let middle = start.interpolated(&end, 0.5).unwrap();
            let half_way = std::f64::consts::FRAC_PI_4;
            assert!(close(
                middle.apply(point),
                [2.0 + half_way.cos(), -1.0 + half_way.sin(), 0.5],
                1e-12
            ));
        }
    }

    // A ratio that is not finite, and one that carries the origin past the
    // largest double, interpolate to no pose.
    #[test]
    fn an_interpolation_to_no_pose_is_refused() {
        let start = Transform::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let far = Transform::new([f64::MAX / 1.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        for ratio in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(start.interpolated(&far, ratio).is_err(), "{ratio}");
        }
        assert!(start.interpolated(&far, 2.0).is_err());
        assert!(start.interpolated(&far, 1.0).is_ok());
    }

    // A heading comes back from its quaternion to rounding on the whole
    // circle; a heading of -0 comes back as +0, as the x and y that tf2's
    // roll and pitch of 0 make +0 add +0 to the -0 of w z; and a quaternion
    // of a pitch of a quarter turn, either way, reads its heading from its x
    // and y.
    #[test]
    fn a_heading_reads_back_from_its_quaternion_as_tf2_reads_it() {
        use std::f64::consts::{FRAC_PI_4, PI};
        for step in -100..=100 {
            let yaw = f64::from(step) * PI / 100.0;
            let back = yaw_of_quaternion(quaternion_of_yaw(yaw));
            assert!((back - yaw).abs() < 1e-14, "{yaw}: {back}");
        }
        let minus_zero = quaternion_of_yaw(-0.0);
        assert_eq!(minus_zero[2].to_bits(), (-0.0_f64).to_bits());
        assert_eq!(yaw_of_quaternion(minus_zero).to_bits(), 0.0_f64.to_bits());
        let (sin_pitch, cos_pitch) = FRAC_PI_4.sin_cos();
        let (sin_yaw, cos_yaw) = 0.2_f64.sin_cos();
        let pitched = |sign: f64| {
            [
                -sign * sin_pitch * sin_yaw,
                sign * sin_pitch * cos_yaw,
                cos_pitch * sin_yaw,
                cos_pitch * cos_yaw,
            ]
        };
        let (up, down) = (pitched(1.0), pitched(-1.0));
        assert_eq!(yaw_of_quaternion(up), 2.0 * up[1].atan2(up[0]));
        assert_eq!(yaw_of_quaternion(down), -2.0 * down[1].atan2(down[0]));
    }

    #[test]
    fn a_pose_wants_numbers_and_a_unit_quaternion() {
        for (position, orientation) in [
            ([0.0; 3], [0.0; 4]),
            ([0.0; 3], [0.0, 0.0, 0.0, 2.0]),
            ([0.0; 3], [f64::NAN, 0.0, 0.0, 1.0]),
            ([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ([f64::INFINITY, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ] {
            assert!(
                Transform::new(position, orientation).is_err(),
                "{position:?} {orientation:?}"
            );
        }
        assert_eq!(
            Transform::new([0.0; 3], [0.0; 4]).unwrap_err().to_string(),
            "position [0.0, 0.0, 0.0] and orientation [0.0, 0.0, 0.0, 0.0] make no transform: \
             give a position of numbers and a unit quaternion [x, y, z, w]"
        );
    }
}
