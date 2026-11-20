//! The orientation of a transform and of the contracts' geometry types: a unit quaternion
//! [x, y, z, w]. A value a caller gives is parsed as tf2's buffer takes a quaternion, refused
//! when a number is not finite or its squared norm lies [`QUATERNION_NORMALIZATION_TOLERANCE`]
//! or more from 1, and normalized. A value that is unit by construction comes from a typed
//! constructor: the identity, a turn about +Z, the product of two orientations, or, behind the
//! `nalgebra` feature, nalgebra's unit quaternion, refused when a coordinate is not finite. An
//! orientation gives its heading as tf2's `getYaw` reads it, its tilt, and turns a vector as
//! tf2's `quatRotate` turns it ([`crate::arithmetic`]), and gives its coordinates back as
//! [x, y, z, w] ([`UnitQuaternion::to_xyzw`]).

use std::fmt;

use crate::arithmetic::{self, Quaternion, Vector};
use crate::not_finite::NotFinite;

/// tf2's tolerance on a quaternion's squared norm: an orientation whose squared norm
/// differs from 1 by this much or more is refused (buffer_core.cpp:66, 250-255).
pub const QUATERNION_NORMALIZATION_TOLERANCE: f64 = 10e-3;

/// An orientation, a unit quaternion [x, y, z, w].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnitQuaternion(Quaternion);

impl UnitQuaternion {
    /// The orientation that turns nothing, [0, 0, 0, 1].
    pub const IDENTITY: UnitQuaternion = UnitQuaternion(arithmetic::IDENTITY);

    /// `orientation` normalized, as tf2's buffer normalizes the quaternion of a transform it
    /// keeps; refused when a number is not finite or tf2's check of a quaternion refuses it.
    pub fn parse(orientation: [f64; 4]) -> Result<Self, OrientationError> {
        let [x, y, z, w] = finite(orientation)?;
        // tf2 sums the squared norm w first (buffer_core.cpp:250-255).
        let squared_norm = w * w + x * x + y * y + z * z;
        if (squared_norm - 1.0).abs() >= QUATERNION_NORMALIZATION_TOLERANCE {
            return Err(OrientationError::NotUnit {
                orientation,
                squared_norm,
            });
        }
        Ok(Self(arithmetic::normalized(orientation)))
    }

    /// The turn by `yaw` (rad) about +Z, as tf2's `Quaternion::setRPY(0, 0, yaw)` makes it;
    /// refused for a yaw that is not finite.
    pub fn from_yaw(yaw: f64) -> Result<Self, NotFinite<1>> {
        NotFinite::check("a yaw", [("yaw", yaw)], "a finite angle in radians")?;
        Ok(Self(arithmetic::quaternion_of_yaw(yaw)))
    }

    /// The orientation `inner` turned by this one: `inner` first, then this, normalized as a
    /// product of two unit quaternions is.
    pub fn compose(self, inner: UnitQuaternion) -> UnitQuaternion {
        Self(arithmetic::normalized(arithmetic::product(self.0, inner.0)))
    }

    /// The heading of this orientation, its turn about +Z, as tf2's `getYaw` reads it
    /// ([`arithmetic::yaw_of_quaternion`]).
    pub fn yaw(self) -> f64 {
        arithmetic::yaw_of_quaternion(self.0)
    }

    /// The angle (rad) between +Z and +Z turned by this orientation, its turn about a horizontal
    /// axis: `2 asin(sqrt(x^2 + y^2))`.
    pub fn tilt(self) -> f64 {
        let [x, y, _, _] = self.0;
        2.0 * (x * x + y * y).sqrt().min(1.0).asin()
    }

    /// `vector` turned by this orientation, as tf2's `quatRotate` turns it.
    pub fn rotate(self, vector: Vector) -> Vector {
        arithmetic::rotate(self.0, vector)
    }

    /// The orientation's coordinates, [x, y, z, w].
    pub fn to_xyzw(self) -> [f64; 4] {
        self.0
    }

    /// The orientation `ratio` of the way from this one to `to`, by tf2's `Quaternion::slerp`,
    /// the shorter way round; `ratio` is finite, as each of the crate's interpolations gives it.
    pub(crate) fn slerp(self, to: UnitQuaternion, ratio: f64) -> UnitQuaternion {
        assert!(ratio.is_finite(), "the ratio {ratio} of a slerp is finite");
        Self(arithmetic::slerp(self.0, to.0, ratio))
    }

    /// A quaternion that the crate's arithmetic computed from unit values, taken as it stands.
    pub(crate) const fn from_computed(quaternion: Quaternion) -> Self {
        Self(quaternion)
    }
}

/// The coordinates as a refusal prints them: `[x, y, z, w]`, each number as `Display` prints it.
impl fmt::Display for UnitQuaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&listed(&self.0))
    }
}

/// An orientation that no contract takes, the refusal saying what to send.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OrientationError {
    #[error(transparent)]
    NotFinite(Box<NotFinite<4>>),
    #[error(
        "the orientation {} has a squared norm of {squared_norm}, {QUATERNION_NORMALIZATION_TOLERANCE} \
         or more from 1: send a unit quaternion [x, y, z, w], such as [0, 0, 0, 1]",
        listed(.orientation)
    )]
    NotUnit {
        orientation: [f64; 4],
        squared_norm: f64,
    },
}

/// `orientation` when every number of it is finite; else its refusal, each coordinate named.
fn finite(orientation: [f64; 4]) -> Result<[f64; 4], OrientationError> {
    let [x, y, z, w] = orientation;
    NotFinite::check(
        "an orientation",
        [("x", x), ("y", y), ("z", z), ("w", w)],
        "a unit quaternion [x, y, z, w] of finite numbers, such as [0, 0, 0, 1]",
    )
    .map_err(|refusal| OrientationError::NotFinite(Box::new(refusal)))?;
    Ok(orientation)
}

/// `values` as a refusal prints them: `[a, b, ...]`, each number as `Display` prints it.
fn listed(values: &[f64]) -> String {
    let printed: Vec<String> = values.iter().map(f64::to_string).collect();
    format!("[{}]", printed.join(", "))
}

/// nalgebra holds a quaternion's coordinates in the contracts' order, [x, y, z, w], and the
/// orientation is unit, so nalgebra takes it as it stands.
#[cfg(feature = "nalgebra")]
impl From<UnitQuaternion> for nalgebra::UnitQuaternion<f64> {
    fn from(orientation: UnitQuaternion) -> Self {
        Self::new_unchecked(nalgebra::Quaternion::from_vector(nalgebra::Vector4::from(
            orientation.0,
        )))
    }
}

/// nalgebra's unit quaternion in the contracts' order, [x, y, z, w]; refused when a coordinate
/// is not finite, as nalgebra's rotation of an angle that is not finite holds.
#[cfg(feature = "nalgebra")]
impl TryFrom<nalgebra::UnitQuaternion<f64>> for UnitQuaternion {
    type Error = OrientationError;

    fn try_from(orientation: nalgebra::UnitQuaternion<f64>) -> Result<Self, OrientationError> {
        finite(orientation.quaternion().coords.into()).map(Self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A quaternion within tf2's tolerance is taken and normalized; one at the tolerance or past
    // it, or with a number that is not finite, is refused with the text the contracts give.
    #[test]
    #[expect(
        clippy::approx_constant,
        reason = "a quarter turn typed to four places, as a caller types one"
    )]
    fn a_quaternion_is_taken_within_tf2s_tolerance_and_normalized() {
        let taken = UnitQuaternion::parse([0.0, 0.0, 0.0, 1.009_f64.sqrt()]).expect("within");
        assert_eq!(taken.to_xyzw(), [0.0, 0.0, 0.0, 1.0]);
        let typed = UnitQuaternion::parse([0.0, 0.0, 0.7071, 0.7071]).expect("within");
        let norm = typed.to_xyzw().iter().map(|c| c * c).sum::<f64>();
        assert!((norm - 1.0).abs() < 1e-15, "{norm}");
        assert!(matches!(
            UnitQuaternion::parse([0.0, 0.0, 0.0, 1.0101_f64.sqrt()]),
            Err(OrientationError::NotUnit { .. })
        ));
        let refused = UnitQuaternion::parse([f64::NAN, 0.0, 0.0, 1.0]).unwrap_err();
        assert_eq!(
            refused.to_string(),
            "an orientation must be finite: got x NaN, y 0, z 0, w 1; give a unit quaternion \
             [x, y, z, w] of finite numbers, such as [0, 0, 0, 1]"
        );
    }

    // A turn about +Z is tf2's setRPY(0, 0, yaw); a yaw that is not finite is refused.
    #[test]
    fn a_yaw_makes_tf2s_turn_about_z() {
        let yaw = std::f64::consts::FRAC_PI_3;
        assert_eq!(
            UnitQuaternion::from_yaw(yaw).expect("finite").to_xyzw(),
            arithmetic::quaternion_of_yaw(yaw)
        );
        assert!(UnitQuaternion::from_yaw(f64::INFINITY).is_err());
    }

    // nalgebra's rotation comes over coordinate for coordinate and goes back unchanged; one of
    // an angle that is not finite is refused, naming its coordinates.
    #[cfg(feature = "nalgebra")]
    #[test]
    fn nalgebras_rotation_is_taken_when_finite() {
        let turn = nalgebra::UnitQuaternion::from_euler_angles(0.3, -0.2, 1.1);
        let taken = UnitQuaternion::try_from(turn).expect("a finite rotation");
        assert_eq!(taken.to_xyzw(), <[f64; 4]>::from(turn.quaternion().coords));
        assert_eq!(nalgebra::UnitQuaternion::from(taken), turn);
        let not_finite = nalgebra::UnitQuaternion::from_euler_angles(f64::NAN, 0.0, 0.0);
        assert!(matches!(
            UnitQuaternion::try_from(not_finite),
            Err(OrientationError::NotFinite(_))
        ));
    }

    // Composing turns `inner` first: a quarter turn about +Z composed with itself is a half turn.
    #[test]
    fn composing_two_quarter_turns_makes_a_half_turn() {
        let quarter = UnitQuaternion::from_yaw(std::f64::consts::FRAC_PI_2).expect("finite");
        let half = UnitQuaternion::from_yaw(std::f64::consts::PI).expect("finite");
        let composed = quarter.compose(quarter).to_xyzw();
        let expected = half.to_xyzw();
        assert!(
            composed
                .iter()
                .zip(expected)
                .all(|(got, want)| (got - want).abs() < 1e-12),
            "{composed:?} against {expected:?}"
        );
    }

    // An orientation's heading and its turn of a vector are tf2's getYaw and quatRotate.
    #[test]
    fn the_heading_and_the_turn_of_a_vector_are_tf2s() {
        let quarter = UnitQuaternion::from_yaw(std::f64::consts::FRAC_PI_2).expect("finite");
        assert_eq!(
            quarter.yaw(),
            arithmetic::yaw_of_quaternion(quarter.to_xyzw())
        );
        assert_eq!(
            quarter.rotate([1.0, 0.0, 0.5]),
            arithmetic::rotate(quarter.to_xyzw(), [1.0, 0.0, 0.5])
        );
        let [x, y, z] = quarter.rotate([1.0, 0.0, 0.5]);
        assert!(
            x.abs() < 1e-15 && (y - 1.0).abs() < 1e-15 && (z - 0.5).abs() < 1e-15,
            "{x} {y} {z}"
        );
    }

    // A turn about +Z alone does not tilt; a roll tilts by its angle.
    #[test]
    fn a_turn_about_z_does_not_tilt_and_a_roll_tilts_by_its_angle() {
        assert_eq!(UnitQuaternion::from_yaw(2.0).expect("finite").tilt(), 0.0);
        let roll = UnitQuaternion::parse([0.2_f64.sin(), 0.0, 0.0, 0.2_f64.cos()]).expect("unit");
        assert!((roll.tilt() - 0.4).abs() < 1e-15, "{}", roll.tilt());
    }

    // An orientation prints its coordinates as a refusal names them.
    #[test]
    fn an_orientation_prints_its_coordinates() {
        assert_eq!(UnitQuaternion::IDENTITY.to_string(), "[0, 0, 0, 1]");
    }
}
