//! Which body pairs the runtime checks, by name, with a per-pair margin
//! offset subtracted from the raw capsule distance.
//!
//! Pairs are data, not logic: the model checks whatever list it is given.
//! The caller supplies the candidate set (a checked-in JSON next to its
//! config); the classifier attaches the sampled per-pair margins.

/// One checked pair of bodies, by config/URDF name. `margin` is subtracted
/// from the raw capsule distance for this pair. The classifier sets a
/// negative margin for permanently snug pairs, moving their zero point to
/// the pair's reference baseline minus the headroom: such a pair reads the
/// headroom at rest and reaches zero only when it gets that much closer
/// than its rest baseline.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PairSpec {
    pub a: String,
    pub b: String,
    #[serde(default)]
    pub margin: f64,
}

impl PairSpec {
    pub fn new(a: impl Into<String>, b: impl Into<String>) -> Self {
        Self { a: a.into(), b: b.into(), margin: 0.0 }
    }
}
