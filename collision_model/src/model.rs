//! The runtime model: both arms' capsules placed by forward kinematics and
//! the minimum distance over the checked pairs.
//!
//! Built once from the URDF plus the generated capsule config; queried every
//! tick with the two joint configurations, reusing per-body buffers so a
//! query costs FK plus a few hundred capsule distances.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::config::LoadedConfig;
use crate::geometry::Capsule;
use crate::pairs::PairSpec;

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
    /// checking the config's classified pairs. Every pair name must resolve
    /// to a config body and every moving link must have capsules: failing
    /// loudly at construction beats silently not checking a body at runtime.
    pub fn new(urdf: &str, left_base: &str, right_base: &str, config: &LoadedConfig) -> Result<Self, String> {
        Self::with_pairs(urdf, left_base, right_base, config, &config.pairs)
    }

    /// Like [`new`](Self::new) but checking an explicit pair list instead of
    /// the config's (the pair classifier itself uses this to evaluate the
    /// structural candidate set).
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
        for (name, capsules) in &config.fixed {
            push_body(&mut bodies, Body { name: name.clone(), local: capsules.clone(), placement: Placement::Fixed })?;
            world.push(capsules.clone());
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
    pub fn from_urdf_file(path: &str, left_base: &str, right_base: &str, config: &LoadedConfig) -> Result<Self, String> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::new(&urdf, left_base, right_base, config)
    }

    /// The OpenArm V1.0 model from the assets embedded in this crate: the
    /// vendored URDF plus the checked-in capsule config and its classified
    /// pairs. Runtime nodes need no file plumbing; sim and real load
    /// identical geometry.
    pub fn openarm_v10() -> Result<Self, String> {
        let config = crate::config::CollisionConfig::from_json(include_str!("../assets/openarm_v10_capsules.json"))?
            .parse()?;
        if config.pairs.is_empty() {
            return Err("embedded config has no classified pairs; run classify_pairs".into());
        }
        Self::new(include_str!("../assets/openarm_v10.urdf"), "openarm_left_link0", "openarm_right_link0", &config)
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

    /// Raw (margin-ignoring) per-pair minimum distances at one configuration,
    /// for the offline pair classifier and diagnostics. Runtime callers want
    /// [`min_distance`](Self::min_distance).
    pub fn pair_distances_raw(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<Vec<(String, String, f64)>, String> {
        if q_left.iter().chain(q_right).any(|x| !x.is_finite()) {
            return Err("non-finite joint configuration".into());
        }
        self.place(q_left, q_right);
        Ok(self
            .pairs
            .iter()
            .map(|pair| {
                let d = self.world[pair.a]
                    .iter()
                    .flat_map(|ca| self.world[pair.b].iter().map(move |cb| ca.distance_to(cb).distance))
                    .fold(f64::INFINITY, f64::min);
                (self.bodies[pair.a].name.clone(), self.bodies[pair.b].name.clone(), d)
            })
            .collect())
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

fn link_poses(arm: &mut Arm, q: &JointVec) -> [Isometry3<f64>; ARM_DOF] {
    let posed = arm.at(q);
    std::array::from_fn(|i| posed.link_pose_world(i))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::CollisionConfig;
    use crate::pairs::PairSpec;

    const URDF: &str = include_str!("../assets/openarm_v10.urdf");

    fn loaded() -> LoadedConfig {
        CollisionConfig::from_json(include_str!("../assets/openarm_v10_capsules.json"))
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
        let mut m = DualArmCollisionModel::openarm_v10().expect("model");
        let config = loaded();
        let q = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
        let p = m.min_distance(&q, &q).expect("query");
        let margin = config
            .pairs
            .iter()
            .find(|s| (s.a == p.link_a && s.b == p.link_b) || (s.a == p.link_b && s.b == p.link_a))
            .expect("winning pair is classified")
            .margin;
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
        let mut m = DualArmCollisionModel::openarm_v10().expect("model");
        let ql = [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0];
        let qr = [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0];
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(p.link_a.contains("link7") && p.link_b.contains("link7"), "{} vs {}", p.link_a, p.link_b);
        assert!(p.distance < 0.0);
    }

    #[test]
    fn world_capsules_rejects_non_finite_configurations() {
        let mut m = DualArmCollisionModel::openarm_v10().expect("model");
        let mut bad = [0.0; ARM_DOF];
        bad[0] = f64::NAN;
        assert!(m.world_capsules(&bad, &[0.0; ARM_DOF]).is_err());
        assert!(m.world_capsules(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).is_ok());
    }
}
