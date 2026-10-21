//! The tolerance and the Hegel generators the property tests share: the unit
//! tests (the crate includes this file as `properties` under `cfg(test)`) and
//! the integration tests (`common::properties`).
//!
//! The file is compiled into the unit tests and into each integration test
//! crate, so items unused by one of them are expected; the allow keeps clippy's
//! -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use chain_kinematics::Limit;
use chain_kinematics::nalgebra::{Rotation3, Unit, Vector3};
use hegel::generators as gs;
use std::f64::consts::PI;

/// Largest miss a property allows, and the margin a drawn value keeps from a
/// boundary it must fall on one side of: the solver's pose tolerance, in
/// metres or radians. A defect these properties look for misses by far more.
pub const TOLERANCE: f64 = 1e-6;

/// An angle in the principal range, both ends included.
pub fn angles() -> gs::FloatGenerator<f64> {
    gs::floats().min_value(-PI).max_value(PI)
}

/// A unit vector, uniform on the sphere.
#[hegel::composite]
pub fn unit_vectors(tc: &hegel::TestCase) -> Vector3<f64> {
    let azimuth = tc.draw(angles());
    let z: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let ring = (1.0 - z * z).sqrt();
    Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z)
}

/// Any rotation: any angle about any axis. Its columns are the orthonormal
/// axes the solver decomposes the shoulder and the wrist about.
#[hegel::composite]
pub fn rotations(tc: &hegel::TestCase) -> Rotation3<f64> {
    let axis = Unit::new_normalize(tc.draw(unit_vectors()));
    Rotation3::from_axis_angle(&axis, tc.draw(angles()))
}

/// Either arm of the fixture, named by the side of its base link.
pub fn sides() -> gs::SampledFromGenerator<'static, &'static str> {
    gs::sampled_from(["left", "right"].as_slice())
}

/// Index of the elbow flex (joint 4) in a configuration.
pub const ELBOW_FLEX: usize = 3;

/// Elbow flex (rad) below which the solver may refuse a pose: it refuses a wrist
/// center within 1e-9 m of full reach, which on the fixture is an elbow flex
/// under 1.4e-4 rad.
pub const STRAIGHT_ELBOW_BAND: f64 = 1e-3;

/// A configuration inside `limits`, bounds included.
#[hegel::composite]
pub fn configs<const N: usize>(tc: &hegel::TestCase, limits: [Limit; N]) -> [f64; N] {
    limits.map(|limit| tc.draw(gs::floats().min_value(limit.lo).max_value(limit.hi)))
}

/// `limits` with the elbow held straight: elbow flex 0.
pub fn straight_elbow<const N: usize>(limits: [Limit; N]) -> [Limit; N] {
    std::array::from_fn(|joint| match joint {
        ELBOW_FLEX => Limit { lo: 0.0, hi: 0.0 },
        _ => limits[joint],
    })
}
