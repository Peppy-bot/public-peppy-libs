//! The runtime model: both arms' capsules placed by forward kinematics and
//! the minimum distance over the checked pairs.
//!
//! Built once from the URDF plus the generated capsule config; queried every
//! tick with the two joint configurations, reusing per-body buffers so a
//! query costs FK plus a few hundred capsule distances.
//!
//! The checked pairs are derived at construction from the URDF: every body
//! pair except those that cannot inform (two fixed bodies never change
//! distance) or that touch by construction (URDF-adjacent bodies). Pairs
//! closer than the band's `d_safe` at a reference pose get rebased to read
//! exactly `d_safe` there, so structural closeness reads as the edge of
//! full speed, not alarm.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::assemble::fit_bodies;
use crate::geometry::Capsule;
use crate::pairs::PairSpec;
use crate::urdf_collision::UrdfCollisions;
/// The caller's safety assertions for margin derivation, applied per pair
/// at construction.
///
/// `references` are joint configurations (applied to both arms, clamped
/// into each arm's own limits) that the caller declares legitimate and
/// expects to read as clear. Pairs that sit closer than the band's `d_safe`
/// at any reference get their reading adjusted so that worst reference
/// reads exactly `d_safe`: a reference pose sits at the edge of full speed
/// and approaching past it throttles from the first millimeter. Clear
/// pairs are stretched (capsule contact still reads zero, so they can
/// never be commanded into interpenetration); overlapping pairs are
/// shifted (the alarm is relative to their rest overlap). The model cannot
/// know which poses are legitimate, so a reference that is actually a
/// collision weakens protection for exactly the pairs it touches. There is
/// deliberately no default: declaring these poses is the caller's
/// statement about the robot, not a library guess.
///
/// `band` is the same [`GovernorBand`](crate::GovernorBand) the consumer
/// gates motion with; the model reads its `d_safe` as the rebase floor so
/// the two cannot disagree. The band also carries the buffer role: the
/// fitted capsules are tight around their meshes with no added padding, so
/// size `d_stop` to cover tracking drift plus watchdog reaction distance.
#[derive(Debug, Clone)]
pub struct MarginPolicy {
    pub band: crate::GovernorBand,
    pub references: Vec<JointVec>,
}

impl MarginPolicy {
    fn validate(&self) -> Result<(), String> {
        if self.references.is_empty() {
            return Err("margin policy needs at least one reference pose".into());
        }
        if self.references.iter().flatten().any(|x| !x.is_finite()) {
            return Err("margin policy reference poses must be finite".into());
        }
        Ok(())
    }
}

/// How a body's capsules reach the world frame.
enum Placement {
    /// Already in world frame (torso, mounts); placed once at construction.
    Fixed,
    /// Link `segment` of the left or right arm; placed by FK every query.
    Left(usize),
    Right(usize),
}

struct Body {
    name: String,
    /// Link-local capsules (world for `Fixed`).
    local: Vec<Capsule>,
    placement: Placement,
}

/// One checked pair, resolved to body indices. The reported distance is
/// [`apply_reading`]`(gain, offset, raw)`: identity for pairs clear by at
/// least `d_safe` at every reference, a stretch (`gain = d_safe /
/// baseline`, clearances only, so capsule contact still reads zero and
/// penetration depths stay physical) for pairs clear but snugger than
/// `d_safe`, and a shift (`offset = d_safe - baseline`) for pairs whose
/// capsules already overlap at a reference, where no zero-preserving
/// monotone map exists. `baseline` is kept for diagnostics on derived
/// models; explicit [`PairSpec`] lists have none.
struct Pair {
    a: usize,
    b: usize,
    gain: f64,
    offset: f64,
    baseline: Option<f64>,
}

/// Best candidate while scanning pairs in [`DualArmCollisionModel::min_distance`].
struct Closest {
    distance: f64,
    a: usize,
    b: usize,
    on_a: Point3<f64>,
    on_b: Point3<f64>,
}

/// The closest approach over all checked pairs at one configuration.
/// `distance` is the adjusted surface distance of the winning pair: raw
/// meters for pairs clear by at least `d_safe` at reference, stretched
/// (zero still means capsule contact) for snugger clear pairs, shifted
/// (baseline-relative) for pairs overlapping at reference. The witness
/// points are raw geometry: their gap coincides with `|distance|` only for
/// unadjusted pairs, and when the capsule axes themselves intersect they
/// degenerate to the axis points (no outward direction exists).
#[derive(Debug, Clone)]
pub struct Proximity<'a> {
    pub distance: f64,
    pub link_a: &'a str,
    pub link_b: &'a str,
    /// Witness points on the two capsule surfaces, world frame.
    pub on_a: Point3<f64>,
    pub on_b: Point3<f64>,
}

pub struct DualArmCollisionModel {
    left: Arm,
    right: Arm,
    bodies: Vec<Body>,
    pairs: Vec<Pair>,
    /// Per-body world capsules, reused across queries. Fixed bodies are
    /// filled at construction and never rewritten.
    world: Vec<Vec<Capsule>>,
}

impl DualArmCollisionModel {
    /// Build from the URDF (both chains) and its collision meshes, fitting
    /// the capsules and deriving the checked pairs and their readings at
    /// construction; there is no intermediate artifact to go stale. Mesh
    /// files are resolved as `<meshes_dir>/<basename>` from the URDF's
    /// collision entries. Fitting the fixture robot takes ~0.25 s in
    /// release.
    pub fn new(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        policy: &MarginPolicy,
    ) -> Result<Self, String> {
        policy.validate()?;
        // Candidate pairs: everything that can inform. Excluded structurally:
        // two fixed bodies (their distance never changes), and pairs within
        // two moving joints of each other, same-side or torso against a
        // chain's first links. Those are joint-yoked: shoulder or wrist
        // cluster members orbit each other through their whole range, so
        // their capsule distance swings with every legitimate motion and
        // would smear the global minimum with noise while real contact
        // between them is blocked by the link in between. Cross-arm pairs
        // are always checked; the arms are independently driven.
        let mut probe = Self::with_pairs(urdf, meshes_dir, left_base, right_base, &[])?;
        let lineage: Vec<(String, Lineage)> = probe
            .bodies
            .iter()
            .map(|b| {
                let lineage = match b.placement {
                    Placement::Left(i) => Lineage::Side(0, i + 1),
                    Placement::Right(i) => Lineage::Side(1, i + 1),
                    Placement::Fixed if b.name == left_base => Lineage::Side(0, 0),
                    Placement::Fixed if b.name == right_base => Lineage::Side(1, 0),
                    Placement::Fixed => Lineage::Torso,
                };
                (b.name.clone(), lineage)
            })
            .collect();

        let mut specs = Vec::new();
        for (i, (a, la)) in lineage.iter().enumerate() {
            for (b, lb) in &lineage[i + 1..] {
                let keep = match (la, lb) {
                    // Two world-fixed bodies never change distance.
                    (Lineage::Torso, Lineage::Torso) => false,
                    (Lineage::Side(_, 0), Lineage::Torso) | (Lineage::Torso, Lineage::Side(_, 0)) => false,
                    (Lineage::Side(sa, 0), Lineage::Side(sb, 0)) if sa != sb => false,
                    // Same side: keep only beyond the joint-yoked horizon.
                    (Lineage::Side(sa, da), Lineage::Side(sb, db)) if sa == sb => da.abs_diff(*db) > 2,
                    (Lineage::Torso, Lineage::Side(_, d)) | (Lineage::Side(_, d), Lineage::Torso) => *d > 2,
                    // Different sides: always checked.
                    (Lineage::Side(..), Lineage::Side(..)) => true,
                };
                if keep {
                    specs.push(PairSpec::new(a.clone(), b.clone()));
                }
            }
        }

        // Margins: the worst reference baseline, clamped into each arm's
        // own limits (the arms' ranges can be mirrored).
        let limits_l = probe.left.limits();
        let limits_r = probe.right.limits();
        let mut baselines: HashMap<(String, String), f64> = HashMap::new();
        for q in &policy.references {
            let ql: JointVec = std::array::from_fn(|i| q[i].clamp(limits_l[i].lo, limits_l[i].hi));
            let qr: JointVec = std::array::from_fn(|i| q[i].clamp(limits_r[i].lo, limits_r[i].hi));
            probe.place(&ql, &qr);
            for spec in &specs {
                let d = probe.raw_pair_distance(&spec.a, &spec.b);
                baselines
                    .entry((spec.a.clone(), spec.b.clone()))
                    .and_modify(|m| *m = m.min(d))
                    .or_insert(d);
            }
        }
        probe.set_pairs(&specs)?;
        let floor = policy.band.d_safe();
        for (spec, pair) in specs.iter().zip(probe.pairs.iter_mut()) {
            let baseline = baselines[&(spec.a.clone(), spec.b.clone())];
            (pair.gain, pair.offset) = pair_adjustment(baseline, floor);
            if baseline < floor {
                pair.baseline = Some(baseline);
            }
        }
        Ok(probe)
    }

    /// Like [`new`](Self::new) but checking an explicit pair list with the
    /// margins given (tests and special-purpose tools). An empty list
    /// builds the bodies with no checked pairs; call
    /// [`set_pairs`](Self::set_pairs) before querying.
    pub fn with_pairs(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        pair_specs: &[PairSpec],
    ) -> Result<Self, String> {
        if left_base == right_base {
            return Err(format!("left and right base links are both '{left_base}'; a bimanual model needs two chains"));
        }
        let mut left = Arm::from_urdf(urdf, left_base)?;
        let mut right = Arm::from_urdf(urdf, right_base)?;

        let home = [0.0; ARM_DOF];
        let chain_names = |arm: &mut Arm| -> Vec<String> {
            let posed = arm.at(&home);
            (0..ARM_DOF).map(|i| posed.link_name(i)).collect()
        };
        let left_names = chain_names(&mut left);
        let right_names = chain_names(&mut right);

        let parsed = UrdfCollisions::from_urdf(urdf)?;
        let fitted = fit_bodies(&parsed, &[left_names.clone(), right_names.clone()], meshes_dir)?;

        let mut bodies: Vec<Body> = Vec::new();
        let mut world = Vec::new();
        let push_body = |bodies: &mut Vec<Body>, body: Body| -> Result<(), String> {
            if bodies.iter().any(|b| b.name == body.name) {
                return Err(format!("duplicate body name '{}'", body.name));
            }
            bodies.push(body);
            Ok(())
        };
        for (name, capsules) in fitted.fixed {
            world.push(capsules.clone());
            push_body(&mut bodies, Body { name, local: capsules, placement: Placement::Fixed })?;
        }
        for (names, side_left) in [(&left_names, true), (&right_names, false)] {
            for (i, name) in names.iter().enumerate() {
                let capsules = fitted.links.get(name).expect("fit_bodies covers every chain link").clone();
                let placement = if side_left { Placement::Left(i) } else { Placement::Right(i) };
                world.push(capsules.clone());
                push_body(&mut bodies, Body { name: name.clone(), local: capsules, placement })?;
            }
        }

        let mut model = Self { left, right, bodies, pairs: Vec::new(), world };
        model.set_pairs(pair_specs)?;
        Ok(model)
    }

    /// Replace the checked pair list (names resolved against the bodies).
    fn set_pairs(&mut self, pair_specs: &[PairSpec]) -> Result<(), String> {
        let index: HashMap<&str, usize> =
            self.bodies.iter().enumerate().map(|(i, b)| (b.name.as_str(), i)).collect();
        self.pairs = pair_specs
            .iter()
            .map(|p| {
                let a = *index.get(p.a.as_str()).ok_or_else(|| format!("pair references unknown body '{}'", p.a))?;
                let b = *index.get(p.b.as_str()).ok_or_else(|| format!("pair references unknown body '{}'", p.b))?;
                if a == b {
                    return Err(format!("pair '{}' against itself", p.a));
                }
                if !p.margin.is_finite() {
                    return Err(format!("pair {}/{} has non-finite margin", p.a, p.b));
                }
                Ok(Pair { a, b, gain: 1.0, offset: -p.margin, baseline: None })
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok(())
    }

    /// Like [`new`](Self::new) but reading the URDF from a file.
    pub fn from_urdf_file(
        path: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        policy: &MarginPolicy,
    ) -> Result<Self, String> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::new(&urdf, meshes_dir, left_base, right_base, policy)
    }

    /// Link-local capsules of a body (fixed bodies are in the root frame),
    /// for diagnostics and the containment tests.
    pub fn local_capsules(&self, name: &str) -> Option<Vec<Capsule>> {
        self.bodies.iter().find(|b| b.name == name).map(|b| b.local.clone())
    }

    /// All checked pairs by name, for diagnostics and tests.
    pub fn checked_pairs(&self) -> Vec<(&str, &str)> {
        self.pairs
            .iter()
            .map(|p| (self.bodies[p.a].name.as_str(), self.bodies[p.b].name.as_str()))
            .collect()
    }

    /// The pairs whose reading is adjusted (reference baseline closer than
    /// the band's `d_safe`), with that baseline: positive means clear but
    /// snug (stretched reading, contact still reads zero), zero or negative
    /// means overlapping at reference (shifted reading, baseline-relative
    /// alarm). Empty on models built from an explicit pair list.
    pub fn adjusted_pairs(&self) -> Vec<(&str, &str, f64)> {
        self.pairs
            .iter()
            .filter_map(|p| {
                let baseline = p.baseline?;
                Some((self.bodies[p.a].name.as_str(), self.bodies[p.b].name.as_str(), baseline))
            })
            .collect()
    }

    /// Raw (unadjusted) minimum distance between two placed bodies; callers
    /// must have called [`place`](Self::place) first.
    fn raw_pair_distance(&self, a: &str, b: &str) -> f64 {
        let idx = |n: &str| self.bodies.iter().position(|x| x.name == n).expect("derived names resolve");
        let (ia, ib) = (idx(a), idx(b));
        self.world[ia]
            .iter()
            .flat_map(|ca| self.world[ib].iter().map(move |cb| ca.distance_to(cb).distance))
            .fold(f64::INFINITY, f64::min)
    }

    /// Minimum adjusted distance over all checked pairs at the given
    /// configurations. Non-finite joint values are rejected so the caller
    /// fails safe rather than comparing against NaN.
    pub fn min_distance(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<Proximity<'_>, String> {
        ensure_finite(q_left, q_right)?;
        self.place(q_left, q_right);

        let mut best: Option<Closest> = None;
        for pair in &self.pairs {
            for ca in &self.world[pair.a] {
                for cb in &self.world[pair.b] {
                    let d = ca.distance_to(cb);
                    let adjusted = apply_reading(pair.gain, pair.offset, d.distance);
                    if best.as_ref().is_none_or(|c| adjusted < c.distance) {
                        best = Some(Closest { distance: adjusted, a: pair.a, b: pair.b, on_a: d.on_a, on_b: d.on_b });
                    }
                }
            }
        }
        let Some(c) = best else {
            return Err("no pairs to check".into());
        };
        Ok(Proximity {
            distance: c.distance,
            link_a: &self.bodies[c.a].name,
            link_b: &self.bodies[c.b].name,
            on_a: c.on_a,
            on_b: c.on_b,
        })
    }

    /// True if any checked pair is at or below `threshold` adjusted
    /// distance.
    pub fn in_collision(&mut self, q_left: &JointVec, q_right: &JointVec, threshold: f64) -> Result<bool, String> {
        Ok(self.min_distance(q_left, q_right)?.distance <= threshold)
    }

    /// Names of all bodies, in checking order (for diagnostics and tools).
    pub fn body_names(&self) -> Vec<&str> {
        self.bodies.iter().map(|b| b.name.as_str()).collect()
    }

    /// World capsules of every body at the given configuration, paired with
    /// the body name (for visualization tools; runtime queries use
    /// [`min_distance`](Self::min_distance)).
    pub fn world_capsules(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<Vec<(&str, Vec<Capsule>)>, String> {
        ensure_finite(q_left, q_right)?;
        self.place(q_left, q_right);
        Ok(self
            .bodies
            .iter()
            .zip(&self.world)
            .map(|(b, w)| (b.name.as_str(), w.clone()))
            .collect())
    }

    /// Refresh the world-frame capsules of the moving bodies from FK.
    fn place(&mut self, q_left: &JointVec, q_right: &JointVec) {
        let poses_l = link_poses(&mut self.left, q_left);
        let poses_r = link_poses(&mut self.right, q_right);
        for (body, world) in self.bodies.iter().zip(self.world.iter_mut()) {
            let pose = match body.placement {
                Placement::Fixed => continue,
                Placement::Left(i) => &poses_l[i],
                Placement::Right(i) => &poses_r[i],
            };
            for (w, l) in world.iter_mut().zip(&body.local) {
                *w = l.transformed(pose);
            }
        }
    }
}

/// Where a body sits in the kinematic tree, for the structural pair rules:
/// the torso, or chain side plus moving-joint depth (mount = 0, link k = k).
enum Lineage {
    Torso,
    Side(u8, usize),
}

fn link_poses(arm: &mut Arm, q: &JointVec) -> [Isometry3<f64>; ARM_DOF] {
    let posed = arm.at(q);
    std::array::from_fn(|i| posed.link_pose_world(i))
}

/// Reject NaN/inf joint values so queries fail safe instead of comparing
/// against NaN downstream.
fn ensure_finite(q_left: &JointVec, q_right: &JointVec) -> Result<(), String> {
    if q_left.iter().chain(q_right).any(|x| !x.is_finite()) {
        return Err("non-finite joint configuration".into());
    }
    Ok(())
}

/// The `(gain, offset)` of a derived pair's reading, `reported =
/// gain * raw + offset`, from its worst reference baseline and the band's
/// `d_safe`. Three regimes, all mapping the baseline to at least `d_safe`
/// so a reference pose is never throttled:
///
/// - clear by at least `d_safe`: identity, absolute clearances;
/// - clear but snugger: stretched through zero, so capsule contact still
///   reads zero and the hard stop lands at `baseline * d_stop / d_safe`,
///   never past contact (and positive whenever `d_stop` is): such a pair
///   can never be commanded into interpenetration;
/// - overlapping at reference: no zero-preserving monotone map exists, so
///   the reading is shifted and alarms relative to the rest overlap.
fn pair_adjustment(baseline: f64, d_safe: f64) -> (f64, f64) {
    if baseline >= d_safe {
        (1.0, 0.0)
    } else if baseline > 0.0 {
        (d_safe / baseline, 0.0)
    } else {
        (1.0, d_safe - baseline)
    }
}

/// A pair's reported distance from its raw capsule distance. The gain
/// stretches clearances only: below contact the raw depth passes through,
/// so penetration depths stay physical meters and a stretched pair cannot
/// out-deep the genuinely deepest pair in the global minimum.
fn apply_reading(gain: f64, offset: f64, raw: f64) -> f64 {
    gain * raw.max(0.0) + raw.min(0.0) + offset
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pairs::PairSpec;

    const URDF: &str = include_str!("../tests/fixtures/openarm_v10.urdf");
    const MESHES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes");

    fn policy() -> MarginPolicy {
        MarginPolicy {
            band: crate::GovernorBand::new(0.01, 0.03).expect("valid band"),
            references: vec![[0.0; ARM_DOF], [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]],
        }
    }

    fn build(pairs: &[PairSpec]) -> Result<DualArmCollisionModel, String> {
        DualArmCollisionModel::with_pairs(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", pairs)
    }

    #[test]
    fn rejects_unknown_pairs_and_querying_with_no_pairs() {
        let err = |r: Result<DualArmCollisionModel, String>| r.err().expect("expected an error");
        let bad = [PairSpec::new("openarm_left_link1", "no_such_body")];
        assert!(err(build(&bad)).contains("unknown body"));

        let mut empty = build(&[]).expect("bodies build without pairs");
        assert!(empty.min_distance(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).is_err());
    }

    #[test]
    fn explicit_pair_margins_offset_the_raw_reading() {
        let pair = |margin| PairSpec { a: "openarm_left_link7".into(), b: "openarm_right_link7".into(), margin };
        let mut raw_model = build(&[pair(0.0)]).expect("model");
        let mut offset_model = build(&[pair(0.01)]).expect("model");
        let q = [0.0; ARM_DOF];
        let raw = raw_model.min_distance(&q, &q).expect("query").distance;
        let adjusted = offset_model.min_distance(&q, &q).expect("query").distance;
        assert!((adjusted - (raw - 0.01)).abs() < 1e-12);
        assert!(offset_model.adjusted_pairs().is_empty(), "explicit lists carry no baselines");
    }

    #[test]
    fn rejects_self_pairs_and_non_finite_margins() {
        let pair = |a: &str, b: &str, margin| PairSpec { a: a.into(), b: b.into(), margin };
        assert!(build(&[pair("openarm_left_link7", "openarm_left_link7", 0.0)]).is_err());
        assert!(build(&[pair("openarm_left_link7", "openarm_right_link7", f64::NAN)]).is_err());
    }

    #[test]
    fn adjusted_winner_reports_band_floor_and_raw_witnesses() {
        let mut m = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &policy())
            .expect("model");
        let q = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let adjusted: Vec<(String, String, f64)> =
            m.adjusted_pairs().iter().map(|(a, b, v)| (a.to_string(), b.to_string(), *v)).collect();
        let p = m.min_distance(&q, &q).expect("query");
        let baseline = adjusted
            .iter()
            .find(|(a, b, _)| (a == p.link_a && b == p.link_b) || (a == p.link_b && b == p.link_a))
            .expect("rest winner should be an adjusted pair")
            .2;
        // Witnesses are raw geometry: undo the pair's reading to compare.
        let d_safe = policy().band.d_safe();
        let (gain, offset) = pair_adjustment(baseline, d_safe);
        let x = p.distance - offset;
        let raw = if x >= 0.0 { x / gain } else { x };
        let gap = (p.on_a - p.on_b).norm();
        assert!(
            (gap - raw.abs()).abs() < 1e-9,
            "gap {gap:.4} vs adjusted {:+.4} baseline {baseline:+.4}",
            p.distance,
        );
    }

    #[test]
    fn pair_adjustment_maps_each_regime_onto_the_band() {
        let (d_stop, d_safe) = (0.01, 0.03);
        let reported = |baseline: f64, raw: f64| {
            let (gain, offset) = pair_adjustment(baseline, d_safe);
            apply_reading(gain, offset, raw)
        };
        // Clear by at least d_safe: identity, absolute clearances.
        assert_eq!(pair_adjustment(0.05, d_safe), (1.0, 0.0));
        assert_eq!(pair_adjustment(d_safe, d_safe), (1.0, 0.0));
        // Adjusted regimes read exactly d_safe at their baseline and the
        // identity regime reads its larger baseline as-is, so a reference
        // pose is never throttled.
        for baseline in [0.05, 0.03, 0.02, 0.007, 0.0, -0.047] {
            assert!((reported(baseline, baseline) - baseline.max(d_safe)).abs() < 1e-12);
        }
        // Clear but snug: zero is preserved (contact reads contact) and the
        // hard stop lands at positive raw clearance, so a pair clear at
        // reference is never commanded into interpenetration.
        for baseline in [0.029, 0.02, 0.007, 1e-4] {
            assert_eq!(reported(baseline, 0.0), 0.0);
            let raw_at_stop = baseline * d_stop / d_safe;
            assert!((reported(baseline, raw_at_stop) - d_stop).abs() < 1e-12);
            assert!(raw_at_stop > 0.0 && raw_at_stop < d_stop);
            // Below contact the depth passes through unstretched, so a
            // snug pair cannot out-deep the genuinely deepest pair.
            assert_eq!(reported(baseline, -0.05), -0.05);
        }
        // Overlapping: shifted, so the alarm is baseline-relative; zero
        // reported means "closed d_safe past the rest overlap".
        let (gain, offset) = pair_adjustment(-0.047, d_safe);
        assert_eq!(gain, 1.0);
        assert!((reported(-0.047, -0.047 - d_safe) - 0.0).abs() < 1e-12);
        assert!(offset > d_safe);
    }

    #[test]
    fn multi_capsule_bodies_take_part_in_the_minimum() {
        // Wrists wrapped toward each other: the winning bodies carry several
        // capsules (wrist bands + fingers), exercising the inner loops.
        let mut m = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &policy())
            .expect("model");
        let ql = [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0];
        let qr = [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0];
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(p.link_a.contains("link7") && p.link_b.contains("link7"), "{} vs {}", p.link_a, p.link_b);
        assert!(p.distance < 0.0);
    }

    #[test]
    fn derived_pairs_skip_fixed_pairs_and_adjacency_and_adjust_snug_bodies() {
        let mut m = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &policy())
            .expect("model");
        let checked: Vec<(String, String)> =
            m.checked_pairs().iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
        let has = |a: &str, b: &str| {
            checked.iter().any(|(x, y)| (x == a && y == b) || (x == b && y == a))
        };
        // Two fixed bodies never change distance.
        assert!(!has("openarm_left_link0", "openarm_right_link0"));
        assert!(!has("openarm_body_link0", "openarm_left_link0"));
        // Same-side pairs within two moving joints are joint-yoked noise.
        assert!(!has("openarm_left_link0", "openarm_left_link1"));
        assert!(!has("openarm_left_link3", "openarm_left_link4"));
        assert!(!has("openarm_left_link1", "openarm_left_link3"));
        assert!(!has("openarm_left_link0", "openarm_left_link2"));
        assert!(!has("openarm_body_link0", "openarm_left_link2"));
        // Beyond the horizon they are checked (the elbow fold, own mount).
        assert!(has("openarm_left_link1", "openarm_left_link7"));
        assert!(has("openarm_left_link0", "openarm_left_link4"));
        assert!(has("openarm_body_link0", "openarm_left_link3"));
        // Cross-arm pairs are checked; structurally snug ones are adjusted,
        // and both regimes (clear-snug and overlapping) occur on the fixture.
        assert!(has("openarm_left_link7", "openarm_right_link7"));
        let baselines: Vec<f64> = m.adjusted_pairs().iter().map(|(_, _, b)| *b).collect();
        assert!(baselines.iter().any(|b| *b > 0.0), "some snug pairs are clear at reference");
        assert!(baselines.iter().any(|b| *b < 0.0), "some pairs overlap at reference");
        // The rest pose reads exactly the band's d_safe: the edge of full
        // speed, so the governor never hard-stops a reference pose.
        let home = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let p = m.min_distance(&home, &home).expect("query");
        assert!((p.distance - 0.03).abs() < 1e-3, "home floor {:+.4}", p.distance);
    }

    #[test]
    fn rest_floor_follows_the_band_d_safe() {
        let wider = MarginPolicy {
            band: crate::GovernorBand::new(0.005, 0.02).expect("valid band"),
            references: policy().references,
        };
        let mut m = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &wider)
            .expect("model");
        let home = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let p = m.min_distance(&home, &home).expect("query");
        assert!((p.distance - 0.02).abs() < 1e-3, "home floor {:+.4}", p.distance);
    }

    #[test]
    fn model_is_send_for_task_ownership() {
        fn assert_send<T: Send>() {}
        assert_send::<DualArmCollisionModel>();
    }

    #[test]
    fn rejects_identical_base_links() {
        let e = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_left_link0", &policy())
            .err()
            .expect("identical bases must fail");
        assert!(e.contains("two chains"), "{e}");
    }

    #[test]
    fn rejects_bad_margin_policies() {
        // Bad bands are unconstructible (GovernorBand::new validates), so
        // only the references can be wrong here.
        let build = |policy: &MarginPolicy| {
            DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", policy)
        };
        assert!(build(&MarginPolicy { band: policy().band, references: vec![] }).is_err());
        let mut bad = [0.0; ARM_DOF];
        bad[2] = f64::INFINITY;
        assert!(build(&MarginPolicy { band: policy().band, references: vec![bad] }).is_err());
    }

    #[test]
    fn world_capsules_rejects_non_finite_configurations() {
        let mut m = DualArmCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &policy())
            .expect("model");
        let mut bad = [0.0; ARM_DOF];
        bad[0] = f64::NAN;
        assert!(m.world_capsules(&bad, &[0.0; ARM_DOF]).is_err());
        assert!(m.world_capsules(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).is_ok());
    }
}
