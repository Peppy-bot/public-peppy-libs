//! The arm sides and the straight-arm band the property tests share: the unit
//! tests (the crate includes this file as `properties` under `cfg(test)`) and
//! the integration tests (`common::properties`). Configurations, directions
//! and rotations come from `kinematics_properties`.
//!
//! The file is compiled into the unit tests and into each integration test
//! crate, so items unused by one of them are expected; the allow keeps clippy's
//! -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use hegel::generators as gs;

/// Either arm of the fixture, named by the side of its base link.
pub fn sides() -> gs::SampledFromGenerator<'static, &'static str> {
    gs::sampled_from(["left", "right"].as_slice())
}

/// Elbow flex (rad) below which the solver may refuse a pose: it refuses a wrist
/// center within 1e-9 m of full reach, which on the fixture is an elbow flex
/// under 1.4e-4 rad.
pub const STRAIGHT_ELBOW_BAND: f64 = 1e-3;
