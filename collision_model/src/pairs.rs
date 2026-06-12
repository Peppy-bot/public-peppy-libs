//! Which body pairs the runtime checks, by name, with a per-pair margin
//! offset subtracted from the raw capsule distance.
//!
//! Pairs are data, not logic: the model checks whatever list it is given.
//! [`openarm_structural_pairs`] is the OpenArm V1.0 starting set derived from
//! the kinematic structure (what can geometrically approach), pending the
//! sampled per-pair margins.

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

/// The OpenArm V1.0 structural pair set: every pair that can geometrically
/// approach, none that is adjacent or permanently co-located.
///
/// The SRS arm clusters joints at the shoulder (links 1..3), elbow (link 4),
/// and wrist (links 5..7); links within and between adjacent clusters of the
/// same arm stay near by construction and are excluded, as are the two mount
/// spheres (permanently 8 mm apart) and the torso against the shoulder
/// cluster that is bolted to it.
pub fn openarm_structural_pairs() -> Vec<PairSpec> {
    let link = |side: &str, i: usize| format!("openarm_{side}_link{i}");
    let mut pairs = Vec::new();

    // Cross arm: everything that moves against everything that moves.
    for i in 1..=7 {
        for j in 1..=7 {
            pairs.push(PairSpec::new(link("left", i), link("right", j)));
        }
    }
    // Cross arm: each arm against the opposite mount sphere.
    for i in 1..=7 {
        pairs.push(PairSpec::new(link("left", i), "openarm_right_link0"));
        pairs.push(PairSpec::new(link("right", i), "openarm_left_link0"));
    }
    for side in ["left", "right"] {
        // Torso against the arm from the elbow out; the shoulder cluster is
        // mounted on the torso and permanently near it.
        for i in 3..=7 {
            pairs.push(PairSpec::new("openarm_body_link0", link(side, i)));
        }
        // Intra arm: shoulder cluster against wrist cluster (the forearm
        // folding back onto the upper arm). Adjacent clusters stay near by
        // construction.
        for i in 1..=3 {
            for j in 5..=7 {
                pairs.push(PairSpec::new(link(side, i), link(side, j)));
            }
        }
        // Intra arm: the elbow-out links against the arm's own mount sphere
        // (the wrist can fold back onto its own shoulder).
        for i in 4..=7 {
            pairs.push(PairSpec::new(format!("openarm_{side}_link0"), link(side, i)));
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn structural_set_has_expected_shape() {
        let pairs = openarm_structural_pairs();
        // 49 cross moving + 14 cross mount + 2*5 torso + 2*9 intra cluster
        // + 2*4 own mount.
        assert_eq!(pairs.len(), 49 + 14 + 10 + 18 + 8);

        let key = |p: &PairSpec| (p.a.clone(), p.b.clone());
        let set: HashSet<_> = pairs.iter().map(key).collect();
        assert_eq!(set.len(), pairs.len(), "no duplicate pairs");

        for p in &pairs {
            assert_ne!(p.a, p.b, "no self pairs");
            assert_eq!(p.margin, 0.0, "structural set starts with zero margins");
        }
        // Spot checks: the permanently snug pairs are absent.
        assert!(!set.contains(&("openarm_left_link0".into(), "openarm_right_link0".into())));
        assert!(!set.contains(&("openarm_body_link0".into(), "openarm_left_link1".into())));
        assert!(!set.contains(&("openarm_left_link1".into(), "openarm_left_link2".into())));
    }
}
