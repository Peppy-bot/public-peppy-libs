//! The straight line between two poses: the interpolation a Cartesian move
//! walks.

use nalgebra::Isometry3;

/// Below this sine of the half-angle between two orientations, a turn of
/// 2e-6 rad, they are one orientation, and the arc between them is whatever
/// the last few bits of the quaternion happened to be. Slerp divides by that
/// sine, so the goal's orientation is taken instead: four orders below the
/// orientation tolerance a chain is servoed to.
pub const COINCIDENT_ARC_EPS: f64 = 1e-6;

/// Pose on the straight line between two poses: position lerped, orientation
/// slerped along the shorter arc, the goal's where the two orientations are
/// one (see [`COINCIDENT_ARC_EPS`]), so a held orientation is held exactly
/// along the whole line.
pub fn interpolate(a: &Isometry3<f64>, b: &Isometry3<f64>, s: f64) -> Isometry3<f64> {
    let s = s.clamp(0.0, 1.0);
    Isometry3::from_parts(
        (a.translation.vector + s * (b.translation.vector - a.translation.vector)).into(),
        a.rotation
            .try_slerp(&b.rotation, s, COINCIDENT_ARC_EPS)
            .unwrap_or(b.rotation),
    )
}
