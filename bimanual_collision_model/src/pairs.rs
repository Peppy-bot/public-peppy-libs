//! Which body pairs the runtime checks, by name, for explicit pair lists
//! (tests and special-purpose tools).
//!
//! Pairs are data, not logic: the model checks whatever list it is given.
//! Derived models get their pair set and per-pair readings from the URDF
//! and the policy at construction instead.

/// One checked pair of bodies, by URDF link name. `margin` is subtracted
/// from the raw capsule distance for this pair; zero reports the raw
/// clearance. Derived models do not use this field: their per-pair
/// readings come from the reference baselines at construction.
#[derive(Debug, Clone, PartialEq)]
pub struct PairSpec {
    pub a: String,
    pub b: String,
    pub margin: f64,
}

impl PairSpec {
    pub fn new(a: impl Into<String>, b: impl Into<String>) -> Self {
        Self { a: a.into(), b: b.into(), margin: 0.0 }
    }
}
