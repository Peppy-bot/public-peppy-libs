//! A pose as the contracts write it: a position in meters and a unit quaternion
//! `[x, y, z, w]`, read and written, for the planning scene's objects and a goal's frame.

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

/// How far a quaternion's norm may lie from 1 and still name an orientation, which is
/// then normalized.
const UNIT_NORM_SLACK: f64 = 1e-3;

/// The orientation of the quaternion `[x, y, z, w]`, normalized: its norm lies within
/// 0.001 of 1.
pub fn orientation_from_parts(orientation: [f64; 4]) -> Result<UnitQuaternion<f64>, String> {
    let [x, y, z, w] = orientation;
    let quaternion = Quaternion::new(w, x, y, z);
    let norm = quaternion.norm();
    if !(1.0 - UNIT_NORM_SLACK..=1.0 + UNIT_NORM_SLACK).contains(&norm) {
        return Err(format!(
            "an orientation is a unit quaternion [x, y, z, w]; {orientation:?} has norm {norm}"
        ));
    }
    Ok(UnitQuaternion::from_quaternion(quaternion))
}

/// The pose of `position` (m) and the orientation `[x, y, z, w]`.
pub fn pose_from_parts(
    position: [f64; 3],
    orientation: [f64; 4],
) -> Result<Isometry3<f64>, String> {
    if !position.iter().all(|v| v.is_finite()) {
        return Err(format!("a position is finite; {position:?} is not"));
    }
    Ok(Isometry3::from_parts(
        Translation3::from(position),
        orientation_from_parts(orientation)?,
    ))
}

/// The position and the quaternion `[x, y, z, w]` of `pose`.
pub fn pose_parts(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let q = pose.rotation.quaternion();
    (pose.translation.vector.into(), [q.i, q.j, q.k, q.w])
}
