//! Hegel generators for property tests over chains: configurations inside a
//! chain's limits, points, directions, rotations and poses in space, and the
//! tolerance the properties share.

use chain_kinematics::Limit;
use chain_kinematics::nalgebra::{Isometry3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use hegel::TestCase;
use hegel::generators as gs;
use std::f64::consts::PI;

/// Largest miss a property allows, in meters or radians. A defect these
/// properties look for misses by far more.
pub const TOLERANCE: f64 = 1e-6;

/// An angle in the principal range, both ends included.
pub fn angles() -> gs::FloatGenerator<f64> {
    gs::floats().min_value(-PI).max_value(PI)
}

/// A unit vector, uniform on the sphere.
#[hegel::composite]
pub fn unit_vectors(tc: &TestCase) -> Vector3<f64> {
    let azimuth = tc.draw(angles());
    let z: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let ring = (1.0 - z * z).sqrt();
    Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z)
}

/// Any rotation: any angle about any axis.
#[hegel::composite]
pub fn rotations(tc: &TestCase) -> Rotation3<f64> {
    let axis = Unit::new_normalize(tc.draw(unit_vectors()));
    Rotation3::from_axis_angle(&axis, tc.draw(angles()))
}

/// A configuration inside `limits`, bounds included.
#[hegel::composite]
pub fn configs<const N: usize>(tc: &TestCase, limits: [Limit; N]) -> [f64; N] {
    limits.map(|limit| tc.draw(gs::floats().min_value(limit.lo).max_value(limit.hi)))
}

/// A point within a meter of the origin on each axis, an arm's workspace scale.
#[hegel::composite]
pub fn points(tc: &TestCase) -> Vector3<f64> {
    Vector3::from(tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0))))
}

/// A pose: a point with any rotation.
#[hegel::composite]
pub fn poses(tc: &TestCase) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(tc.draw(points())),
        UnitQuaternion::from_rotation_matrix(&tc.draw(rotations())),
    )
}
