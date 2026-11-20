//! tf2's `Transform`: where one frame sits in another, as a rotation matrix and a position, made
//! from a position in meters and a [`UnitQuaternion`]. A transform carries a point into the frame
//! it is placed in, composes with another and inverts, each in tf2's arithmetic
//! ([`crate::arithmetic`]), operation for operation, so a point lands on the bits tf2 gives it.
//! occupancy_map's parity with MoveIt's point cloud updater, which places each point by tf2,
//! compares those bits. A transform also gives its orientation back and runs a share of the way
//! to another, for a sensor that moves while it measures.

use crate::arithmetic::{self, Rotation, Vector};
use crate::not_finite::NotFinite;
#[cfg(feature = "nalgebra")]
use crate::unit_quaternion::OrientationError;
use crate::unit_quaternion::UnitQuaternion;

/// Where one frame, the placed one, sits in another: the rotation of the
/// placed frame as tf2 builds the matrix of a quaternion, and its position. A
/// point `p` of the placed frame is `R * p + position` in the other.
#[derive(Debug, Clone, Copy)]
pub struct Transform {
    rows: Rotation,
    position: Vector,
}

/// `position`, in meters, when every coordinate of it is finite; else its refusal, each
/// coordinate named.
pub fn finite_position(position: [f64; 3]) -> Result<[f64; 3], NotFinite<3>> {
    let [x, y, z] = position;
    NotFinite::check(
        "a position",
        [("x", x), ("y", y), ("z", z)],
        "a finite number of meters on each axis",
    )?;
    Ok(position)
}

/// An isometry that makes no transform: a coordinate of its position or of its rotation that
/// is not finite.
#[cfg(feature = "nalgebra")]
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum NotATransform {
    #[error(transparent)]
    Position(Box<NotFinite<3>>),
    #[error(transparent)]
    Orientation(#[from] OrientationError),
}

impl Transform {
    /// The frame in itself, tf2's `Transform::getIdentity`.
    pub const IDENTITY: Transform = Transform {
        rows: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        position: [0.0; 3],
    };

    /// The pose at `position` turned by `orientation`, with its matrix built as tf2's
    /// `Matrix3x3::setRotation` builds it; refused for a position that is not finite.
    pub fn new(position: [f64; 3], orientation: UnitQuaternion) -> Result<Self, NotFinite<3>> {
        Ok(Self::from_computed(finite_position(position)?, orientation))
    }

    /// The transform at `position` turned by `orientation`, its matrix built as
    /// `Matrix3x3::setRotation` builds it, as tf2 makes the transform of a lookup's answer
    /// (buffer_core.cpp:794-805): a position the crate's arithmetic computed from finite ones,
    /// taken as it stands.
    pub(crate) fn from_computed(position: [f64; 3], orientation: UnitQuaternion) -> Self {
        Self {
            rows: arithmetic::rotation_of(orientation.to_xyzw()),
            position,
        }
    }

    /// The placed frame's position, in the frame it is placed in.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// A point of the placed frame in the frame it is placed in: each row of
    /// the rotation dotted with the point, plus the position.
    pub fn apply(&self, point: [f64; 3]) -> [f64; 3] {
        let turned = arithmetic::rotated(&self.rows, point);
        std::array::from_fn(|i| turned[i] + self.position[i])
    }

    /// The pose of the frame `inner` places in this pose's placed frame, in
    /// the frame this pose is placed in: a point goes through `inner`, then
    /// through this pose. tf2's product of two transforms.
    pub fn compose(&self, inner: &Transform) -> Transform {
        Transform {
            rows: arithmetic::rotation_product(&self.rows, &inner.rows),
            position: self.apply(inner.position),
        }
    }

    /// The placed frame's orientation, from the rotation as tf2's `Matrix3x3::getRotation`
    /// takes it: from the trace when it is above 0, else from the largest diagonal entry.
    pub fn orientation(&self) -> UnitQuaternion {
        UnitQuaternion::from_computed(arithmetic::quaternion_of(&self.rows))
    }

    /// The pose `ratio` of the way from this pose to `to`, 0 this pose and 1
    /// `to`: the positions interpolated as tf2's `Vector3::setInterpolate3`, and
    /// the orientations by tf2's `Quaternion::slerp`, the shorter way round. A
    /// ratio that is not finite, or one that carries the position past the
    /// largest double, makes no pose: the error gives the position.
    pub fn interpolated(&self, to: &Transform, ratio: f64) -> Result<Transform, NotFinite<3>> {
        // A ratio that is not finite makes every coordinate of the position not finite, so the
        // ratio the slerp takes is finite.
        let position =
            finite_position(arithmetic::interpolated(self.position, to.position, ratio))?;
        let orientation = self.orientation().slerp(to.orientation(), ratio);
        Ok(Transform::from_computed(position, orientation))
    }

    /// The pose of the frame this pose is placed in, in the placed frame:
    /// tf2's inverse, the rotation transposed and the position carried back
    /// through it and negated.
    pub fn inverse(&self) -> Transform {
        let rows = arithmetic::transposed(&self.rows);
        let back = self.position.map(|c| -c);
        Transform {
            rows,
            position: arithmetic::rotated(&rows, back),
        }
    }
}

/// The transform of nalgebra's isometry: its translation as the position, and its rotation;
/// refused where a coordinate of either is not finite.
#[cfg(feature = "nalgebra")]
impl TryFrom<&nalgebra::Isometry3<f64>> for Transform {
    type Error = NotATransform;

    fn try_from(isometry: &nalgebra::Isometry3<f64>) -> Result<Self, NotATransform> {
        let orientation = UnitQuaternion::try_from(isometry.rotation)?;
        Transform::new(isometry.translation.vector.into(), orientation)
            .map_err(|refusal| NotATransform::Position(Box::new(refusal)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arithmetic::{quaternion_of_yaw, yaw_of_quaternion};

    /// The unit quaternion `orientation`, [x, y, z, w], as the contracts' parse takes it.
    fn unit(orientation: [f64; 4]) -> UnitQuaternion {
        UnitQuaternion::parse(orientation).expect("a unit quaternion")
    }

    /// Whether every coordinate of `a` lies within `tolerance` of `b`'s.
    fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < tolerance)
    }

    /// A pose with a turn about each axis and an offset on each.
    fn tilted() -> Transform {
        let norm = (0.1_f64 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9).sqrt();
        Transform::new(
            [0.4, -0.5, 0.6],
            unit([0.1, 0.2, 0.3, 0.9].map(|c| c / norm)),
        )
        .expect("a finite position")
    }

    #[test]
    fn a_pose_turns_a_point_and_moves_it_by_its_position() {
        // A quarter turn about +Z takes +X to +Y, then the position is added.
        let half = std::f64::consts::FRAC_PI_4;
        let pose = Transform::new(
            [0.015, 0.0, -0.001],
            unit([0.0, 0.0, half.sin(), half.cos()]),
        )
        .expect("a finite position");
        assert!(close(
            pose.apply([1.0, 0.0, 2.0]),
            [0.015, 1.0, 1.999],
            1e-12
        ));
        assert_eq!(pose.position(), [0.015, 0.0, -0.001]);
        let identity = Transform::IDENTITY;
        assert_eq!(identity.apply([0.3, -0.2, 1.5]), [0.3, -0.2, 1.5]);
    }

    // A composed pose carries a point as its two poses do one after the
    // other, inner first, and the inverse carries it back.
    #[test]
    fn composing_runs_the_inner_pose_first_and_the_inverse_undoes_a_pose() {
        let outer = tilted();
        let half = 0.35_f64;
        let inner = Transform::new(
            [-0.06, 0.01, 0.002],
            unit([0.0, half.sin(), 0.0, half.cos()]),
        )
        .expect("a finite position");
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
            unit([axis[0] * sin, axis[1] * sin, axis[2] * sin, cos]),
        )
        .expect("a finite position")
    }

    // The quaternion a pose gives back makes the same pose, by either branch of
    // tf2's extraction: a turn whose trace lies above 0, and a half turn, whose
    // trace lies below.
    #[test]
    fn a_pose_gives_back_its_orientation() {
        let half_turn =
            Transform::new([0.0; 3], unit([1.0, 0.0, 0.0, 0.0])).expect("a half turn about +X");
        for pose in [tilted(), half_turn] {
            let again =
                Transform::new(pose.position(), pose.orientation()).expect("a finite position");
            let point = [0.3, -1.2, 0.8];
            assert!(close(again.apply(point), pose.apply(point), 1e-12));
        }
        assert_eq!(half_turn.orientation().to_xyzw(), [1.0, 0.0, 0.0, 0.0]);
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
                Transform::new(pose.position(), pose.orientation()).expect("a finite position");
            assert!(
                close(again.apply(point), pose.apply(point), 1e-12),
                "{angle_rad} rad about {axis:?}"
            );
        }
    }

    // An interpolated pose runs from the first pose at 0 to the second at 1,
    // its position along the line between them and its heading turning by the
    // share of the turn between them, the shorter way round whichever sign the
    // second quaternion carries.
    #[test]
    fn an_interpolated_pose_runs_from_one_pose_to_the_other() {
        let start = Transform::new([1.0, 0.0, 0.5], UnitQuaternion::IDENTITY).expect("a pose");
        let quarter = std::f64::consts::FRAC_PI_4;
        let turned = [0.0, 0.0, quarter.sin(), quarter.cos()];
        for orientation in [turned, turned.map(|c| -c)] {
            let end = Transform::new([3.0, -2.0, 0.5], unit(orientation))
                .expect("a quarter turn about +Z");
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

    // A ratio that is not finite, and one that carries the position past the
    // largest double, interpolate to no pose.
    #[test]
    fn an_interpolation_to_no_pose_is_refused() {
        let start = Transform::IDENTITY;
        let far =
            Transform::new([f64::MAX / 1.5, 0.0, 0.0], UnitQuaternion::IDENTITY).expect("a pose");
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

    // The identity is the transform tf2 builds of no turn at the origin.
    #[test]
    fn the_identity_is_no_turn_at_the_origin() {
        let built = Transform::new([0.0; 3], UnitQuaternion::IDENTITY).expect("the origin");
        assert_eq!(Transform::IDENTITY.rows, built.rows);
        assert_eq!(Transform::IDENTITY.position(), built.position());
        assert_eq!(
            Transform::IDENTITY.apply([0.3, -0.2, 1.1]),
            [0.3, -0.2, 1.1]
        );
    }

    // A position that is not finite makes no transform, the refusal saying what to give.
    #[test]
    fn a_pose_wants_a_position_of_numbers() {
        for position in [[f64::NAN, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0]] {
            assert!(
                Transform::new(position, UnitQuaternion::IDENTITY).is_err(),
                "{position:?}"
            );
        }
        assert_eq!(
            Transform::new([f64::NAN, 0.0, 0.0], UnitQuaternion::IDENTITY)
                .unwrap_err()
                .to_string(),
            "a position must be finite: got x NaN, y 0, z 0; give a finite number of meters on \
             each axis"
        );
    }
}
