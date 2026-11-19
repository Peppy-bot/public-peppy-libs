//! The pose of a frame that nalgebra places, for the suites that build scenes with it.

use frame_tree::Transform;
use nalgebra::Isometry3;

/// The pose of the frame `isometry` places: its translation, and its quaternion as
/// tf2 builds the matrix of one.
pub fn pose_of(isometry: &Isometry3<f64>) -> Transform {
    let q = isometry.rotation;
    Transform::new(isometry.translation.vector.into(), [q.i, q.j, q.k, q.w])
        .expect("an isometry is a pose")
}
