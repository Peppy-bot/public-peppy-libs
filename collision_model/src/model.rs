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
//! closer than the policy headroom at a reference pose get that baseline as
//! their zero point, so structural closeness reads as headroom, not alarm.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::config::LoadedConfig;
use crate::geometry::Capsule;
use crate::pairs::PairSpec;
/// How margins are derived at construction. Margined pairs read exactly
/// `headroom` at their worst reference pose, so a watchdog threshold or
/// governor `d_safe` must stay below `headroom` or rest poses throttle.
/// References are clamped into each arm's own joint limits before use.
#[derive(Debug, Clone)]
pub struct MarginPolicy {
    pub headroom: f64,
    pub references: Vec<JointVec>,
}

impl MarginPolicy {
    fn validate(&self) -> Result<(), String> {
        if !(self.headroom.is_finite() && self.headroom > 0.0) {
            return Err(format!("margin policy headroom must be finite and positive, got {}", self.headroom));
        }
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

/// One checked pair, resolved to body indices.
struct Pair {
    a: usize,
    b: usize,
    margin: f64,
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
/// `distance` is the margin-adjusted surface distance of the winning pair;
/// zero or negative means that pair violates its margin (or interpenetrates).
/// The witness points are raw geometry: their gap equals `distance` plus the
/// winning pair's margin, so they coincide with `|distance|` only for
/// unmargined pairs, and when the capsule axes themselves intersect they
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
    /// Build from the URDF (both chains) and the generated capsule config,
    /// deriving the checked pairs and their margins from the URDF and the
    /// margin policy. Every moving link must have capsules: failing loudly
    /// at construction beats silently not checking a body at runtime.
    pub fn new(
        urdf: &str,
        left_base: &str,
        right_base: &str,
        config: &LoadedConfig,
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
        let mut probe = Self::with_pairs(urdf, left_base, right_base, config, &derive_probe_pairs(config))?;
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
        for spec in &mut specs {
            let baseline = baselines[&(spec.a.clone(), spec.b.clone())];
            if baseline < policy.headroom {
                spec.margin = baseline - policy.headroom;
            }
        }

        Self::with_pairs(urdf, left_base, right_base, config, &specs)
    }

    /// Like [`new`](Self::new) but checking an explicit pair list with the
    /// margins given (tests and special-purpose tools).
    pub fn with_pairs(
        urdf: &str,
        left_base: &str,
        right_base: &str,
        config: &LoadedConfig,
        pair_specs: &[PairSpec],
    ) -> Result<Self, String> {
        let mut left = Arm::from_urdf(urdf, left_base)?;
        let mut right = Arm::from_urdf(urdf, right_base)?;

        let mut bodies: Vec<Body> = Vec::new();
        let mut world = Vec::new();
        let push_body = |bodies: &mut Vec<Body>, body: Body| -> Result<(), String> {
            if bodies.iter().any(|b| b.name == body.name) {
                return Err(format!("duplicate body name '{}' in config", body.name));
            }
            bodies.push(body);
            Ok(())
        };
        for b in &config.fixed {
            push_body(&mut bodies, Body { name: b.name.clone(), local: b.capsules.clone(), placement: Placement::Fixed })?;
            world.push(b.capsules.clone());
        }
        let home = [0.0; ARM_DOF];
        for (arm, side) in [(&mut left, Placement::Left(0)), (&mut right, Placement::Right(0))] {
            let names: Vec<String> = {
                let posed = arm.at(&home);
                (0..ARM_DOF).map(|i| posed.link_name(i)).collect()
            };
            for (i, name) in names.into_iter().enumerate() {
                let capsules = config
                    .links
                    .get(&name)
                    .ok_or_else(|| format!("capsule config has no link '{name}'"))?
                    .clone();
                let placement = match side {
                    Placement::Left(_) => Placement::Left(i),
                    _ => Placement::Right(i),
                };
                world.push(capsules.clone());
                push_body(&mut bodies, Body { name, local: capsules, placement })?;
            }
        }

        let index: HashMap<&str, usize> =
            bodies.iter().enumerate().map(|(i, b)| (b.name.as_str(), i)).collect();
        let pairs = pair_specs
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
                Ok(Pair { a, b, margin: p.margin })
            })
            .collect::<Result<Vec<_>, String>>()?;
        if pairs.is_empty() {
            return Err("no pairs to check".into());
        }

        Ok(Self { left, right, bodies, pairs, world })
    }

    /// Like [`new`](Self::new) but reading the URDF from a file.
    pub fn from_urdf_file(
        path: &str,
        left_base: &str,
        right_base: &str,
        config: &LoadedConfig,
        policy: &MarginPolicy,
    ) -> Result<Self, String> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::new(&urdf, left_base, right_base, config, policy)
    }

    /// The checked pairs and their margins, for diagnostics and tests.
    pub fn pair_margins(&self) -> Vec<(&str, &str, f64)> {
        self.pairs
            .iter()
            .map(|p| (self.bodies[p.a].name.as_str(), self.bodies[p.b].name.as_str(), p.margin))
            .collect()
    }

    /// Raw (margin-free) minimum distance between two placed bodies; callers
    /// must have called [`place`](Self::place) first.
    fn raw_pair_distance(&self, a: &str, b: &str) -> f64 {
        let idx = |n: &str| self.bodies.iter().position(|x| x.name == n).expect("derived names resolve");
        let (ia, ib) = (idx(a), idx(b));
        self.world[ia]
            .iter()
            .flat_map(|ca| self.world[ib].iter().map(move |cb| ca.distance_to(cb).distance))
            .fold(f64::INFINITY, f64::min)
    }

    /// Minimum margin-adjusted distance over all checked pairs at the given
    /// configurations. Non-finite joint values are rejected so the caller
    /// fails safe rather than comparing against NaN.
    pub fn min_distance(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<Proximity<'_>, String> {
        if q_left.iter().chain(q_right).any(|x| !x.is_finite()) {
            return Err("non-finite joint configuration".into());
        }
        self.place(q_left, q_right);

        let mut best: Option<Closest> = None;
        for pair in &self.pairs {
            for ca in &self.world[pair.a] {
                for cb in &self.world[pair.b] {
                    let d = ca.distance_to(cb);
                    let adjusted = d.distance - pair.margin;
                    if best.as_ref().is_none_or(|c| adjusted < c.distance) {
                        best = Some(Closest { distance: adjusted, a: pair.a, b: pair.b, on_a: d.on_a, on_b: d.on_b });
                    }
                }
            }
        }
        let c = best.expect("constructor guarantees at least one pair");
        Ok(Proximity {
            distance: c.distance,
            link_a: &self.bodies[c.a].name,
            link_b: &self.bodies[c.b].name,
            on_a: c.on_a,
            on_b: c.on_b,
        })
    }

    /// True if any checked pair is at or below `threshold` margin-adjusted
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
        if q_left.iter().chain(q_right).any(|x| !x.is_finite()) {
            return Err("non-finite joint configuration".into());
        }
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

/// A minimal pair list (one arbitrary pair) used to construct the probe
/// model that the real pair derivation then measures with.
fn derive_probe_pairs(config: &LoadedConfig) -> Vec<PairSpec> {
    let mut names = config.fixed.iter().map(|b| b.name.clone()).chain(config.links.keys().cloned());
    let a = names.next().expect("config has bodies");
    let b = names.next().expect("config has at least two bodies");
    vec![PairSpec::new(a, b)]
}

fn link_poses(arm: &mut Arm, q: &JointVec) -> [Isometry3<f64>; ARM_DOF] {
    let posed = arm.at(q);
    std::array::from_fn(|i| posed.link_pose_world(i))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::CollisionConfig;
    use crate::pairs::PairSpec;

    const URDF: &str = include_str!("../tests/fixtures/openarm_v10.urdf");

    fn policy() -> MarginPolicy {
        MarginPolicy {
            headroom: 0.04,
            references: vec![[0.0; ARM_DOF], [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]],
        }
    }

    fn loaded() -> LoadedConfig {
        CollisionConfig::from_json(include_str!("../tests/fixtures/openarm_v10_capsules.json"))
            .expect("embedded config")
            .parse()
            .expect("valid config")
    }

    fn build(config: &LoadedConfig, pairs: &[PairSpec]) -> Result<DualArmCollisionModel, String> {
        DualArmCollisionModel::with_pairs(URDF, "openarm_left_link0", "openarm_right_link0", config, pairs)
    }

    #[test]
    fn rejects_duplicate_body_names_unknown_pairs_and_empty_pairs() {
        let err = |r: Result<DualArmCollisionModel, String>| r.err().expect("expected an error");
        let mut config = loaded();
        config.fixed.push(config.fixed[0].clone());
        let pairs = [PairSpec::new("openarm_left_link1", "openarm_right_link1")];
        assert!(err(build(&config, &pairs)).contains("duplicate body"));

        let config = loaded();
        let bad = [PairSpec::new("openarm_left_link1", "no_such_body")];
        assert!(err(build(&config, &bad)).contains("unknown body"));

        assert!(err(build(&config, &[])).contains("no pairs"));
    }

    #[test]
    fn margined_winner_reports_adjusted_distance_and_raw_witnesses() {
        let config = loaded();
        let mut m = DualArmCollisionModel::new(URDF, "openarm_left_link0", "openarm_right_link0", &config, &policy())
            .expect("model");
        let q = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let margins: Vec<(String, String, f64)> =
            m.pair_margins().iter().map(|(a, b, v)| (a.to_string(), b.to_string(), *v)).collect();
        let p = m.min_distance(&q, &q).expect("query");
        let margin = margins
            .iter()
            .find(|(a, b, _)| (a == p.link_a && b == p.link_b) || (a == p.link_b && b == p.link_a))
            .expect("winning pair is checked")
            .2;
        assert!(margin < 0.0, "rest winner should be a margined pair, got margin {margin}");
        let gap = (p.on_a - p.on_b).norm();
        // Witnesses are raw geometry: gap equals |raw| = |distance + margin|.
        assert!(
            (gap - (p.distance + margin).abs()) < 1e-9,
            "gap {gap:.4} vs adjusted {:+.4} margin {margin:+.4}",
            p.distance,
        );
    }

    #[test]
    fn multi_capsule_bodies_take_part_in_the_minimum() {
        // Wrists wrapped toward each other: the winning bodies carry several
        // capsules (wrist bands + fingers), exercising the inner loops.
        let mut m =
            DualArmCollisionModel::new(URDF, "openarm_left_link0", "openarm_right_link0", &loaded(), &policy())
                .expect("model");
        let ql = [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0];
        let qr = [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0];
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(p.link_a.contains("link7") && p.link_b.contains("link7"), "{} vs {}", p.link_a, p.link_b);
        assert!(p.distance < 0.0);
    }

    #[test]
    fn derived_pairs_skip_fixed_pairs_and_adjacency_and_margin_snug_bodies() {
        let mut m =
            DualArmCollisionModel::new(URDF, "openarm_left_link0", "openarm_right_link0", &loaded(), &policy())
                .expect("model");
        let margins: Vec<(String, String, f64)> =
            m.pair_margins().iter().map(|(a, b, v)| (a.to_string(), b.to_string(), *v)).collect();
        let has = |a: &str, b: &str| {
            margins.iter().any(|(x, y, _)| (x == a && y == b) || (x == b && y == a))
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
        // Cross-arm pairs are checked; structurally snug ones carry margins.
        assert!(has("openarm_left_link7", "openarm_right_link7"));
        assert!(margins.iter().any(|(_, _, m)| *m < 0.0), "some pairs are margined");
        // The rest pose reads exactly the headroom.
        let home = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let p = m.min_distance(&home, &home).expect("query");
        assert!((p.distance - 0.04).abs() < 1e-3, "home floor {:+.4}", p.distance);
    }

    #[test]
    fn rejects_bad_margin_policies() {
        let config = loaded();
        let build = |policy: &MarginPolicy| {
            DualArmCollisionModel::new(URDF, "openarm_left_link0", "openarm_right_link0", &config, policy)
        };
        assert!(build(&MarginPolicy { headroom: 0.0, references: vec![[0.0; ARM_DOF]] }).is_err());
        assert!(build(&MarginPolicy { headroom: f64::NAN, references: vec![[0.0; ARM_DOF]] }).is_err());
        assert!(build(&MarginPolicy { headroom: 0.04, references: vec![] }).is_err());
        let mut bad = [0.0; ARM_DOF];
        bad[2] = f64::INFINITY;
        assert!(build(&MarginPolicy { headroom: 0.04, references: vec![bad] }).is_err());
    }

    #[test]
    fn world_capsules_rejects_non_finite_configurations() {
        let mut m =
            DualArmCollisionModel::new(URDF, "openarm_left_link0", "openarm_right_link0", &loaded(), &policy())
                .expect("model");
        let mut bad = [0.0; ARM_DOF];
        bad[0] = f64::NAN;
        assert!(m.world_capsules(&bad, &[0.0; ARM_DOF]).is_err());
        assert!(m.world_capsules(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).is_ok());
    }
}
