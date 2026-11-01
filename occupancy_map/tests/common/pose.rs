//! The pose of a frame that nalgebra places, for the suites that build scenes with it.

use camera_core::pose::Pose;
use nalgebra::Isometry3;

/// The pose of the frame `isometry` places: its translation, and its quaternion as
/// tf2 builds the matrix of one.
pub fn pose_of(isometry: &Isometry3<f64>) -> Pose {
    let q = isometry.rotation;
    Pose::new(isometry.translation.vector.into(), [q.i, q.j, q.k, q.w])
        .expect("an isometry is a pose")
}
