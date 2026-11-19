//! A pose as the contracts write it: a position in meters and a unit quaternion
//! `[x, y, z, w]`, taken or refused by control_core's one parse of the contracts' geometry.
//! The planning scene reads the pose of each shape of an object with it, and control_core's
//! `pose_arrays` writes one back.

use control_core::geometry::Pose;
use nalgebra::Isometry3;

/// The pose of `position` (m) and the orientation `[x, y, z, w]` where control_core's parse
/// takes them: each number finite, and the orientation's squared norm within
/// `QUATERNION_NORMALIZATION_TOLERANCE` of 1, normalized as tf2's buffer normalizes a
/// transform's. The refusal is the parse's, and says what to send.
pub fn pose_from_parts(
    position: [f64; 3],
    orientation: [f64; 4],
) -> Result<Isometry3<f64>, String> {
    Pose::parse(position, orientation)
        .map(Isometry3::from)
        .map_err(|e| e.to_string())
}
