//! The runtime model: both arms' convex-hull pieces placed by forward
//! kinematics and the minimum signed distance over the checked pairs.
//!
//! Built once from the URDF plus its collision meshes, queried every tick with
//! the two joint configurations. A query is FK plus GJK over the checked
//! piece-pairs; where pieces overlap, EPA recovers the penetration depth, so
//! the distance is signed and continuous through contact.
//!
//! The checked pairs are derived at construction from the URDF: every body
//! pair except those that cannot inform (two fixed bodies never change
//! distance) or that touch by construction (URDF-adjacent, joint-yoked bodies).
//! The hulls are tight, so no rebasing is needed: instead every checked pair is
//! required to read at least the band's `d_safe` at each declared reference
//! pose, or construction fails loudly. A reference that is actually a near-miss
//! is the caller's error to fix, not a margin to paper over.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3, Vector3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::assemble::fit_bodies;
use crate::gjk::{self, Hull, Placed};
use crate::pairs::PairSpec;
use crate::urdf_collision::UrdfCollisions;

/// How a body's hulls reach the world frame.
#[derive(Clone, Copy)]
enum Placement {
    /// Already in world frame (torso, mounts); identity for the whole run.
    Fixed,
    /// Link `i` of the left or right arm; placed by FK every query.
    Left(usize),
    Right(usize),
}

/// A body's bounding sphere in its own local frame: centre over the hull
/// vertices, radius covering the rounded pieces. The radius is rotation
/// invariant, so placing the centre gives a cheap distance lower bound for the
/// broadphase.
struct BoundingSphere {
    center: Point3<f64>,
    radius: f64,
}

impl BoundingSphere {
    fn of(hulls: &[Hull]) -> BoundingSphere {
        let verts: Vec<&Point3<f64>> = hulls.iter().flat_map(|h| h.vertices()).collect();
        let center = Point3::from(verts.iter().fold(Vector3::zeros(), |a, p| a + p.coords) / verts.len() as f64);
        let mut radius = 0.0_f64;
        for h in hulls {
            for v in h.vertices() {
                radius = radius.max((v - center).norm() + h.inflation());
            }
        }
        BoundingSphere { center, radius }
    }
}

struct Body {
    name: String,
    /// Convex-hull pieces, in the body's local frame (world for `Fixed`).
    local: Vec<Hull>,
    placement: Placement,
    bound: BoundingSphere,
}

/// One checked pair, resolved to body indices.
struct Pair {
    a: usize,
    b: usize,
}

/// Best candidate while scanning pairs in [`BimanualCollisionModel::min_distance`].
struct Closest {
    distance: f64,
    a: usize,
    b: usize,
    on_a: Point3<f64>,
    on_b: Point3<f64>,
}

/// The closest approach over all checked pairs at one configuration. `distance`
/// is the signed surface distance of the winning pair (negative is penetration,
/// from EPA). The witnesses are the closest points on the two hull surfaces in
/// world frame.
#[derive(Debug, Clone)]
pub struct Proximity<'a> {
    pub distance: f64,
    pub link_a: &'a str,
    pub link_b: &'a str,
    pub on_a: Point3<f64>,
    pub on_b: Point3<f64>,
}

/// One hull piece placed in the world: the vertices, the face triangles
/// indexing them, and the inflation `radius` swept around the core to recover
/// the mesh. Enough to draw the true rounded collision surface (offset faces
/// plus edge and vertex fillets); runtime queries never materialize this.
pub struct PlacedPiece {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[usize; 3]>,
    pub radius: f64,
}

/// Per-body world-frame hull pieces: the shape
/// [`BimanualCollisionModel::world_pieces`] returns.
pub type BodyPieces<'a> = Vec<(&'a str, Vec<PlacedPiece>)>;

pub struct BimanualCollisionModel {
    left: Arm,
    right: Arm,
    bodies: Vec<Body>,
    pairs: Vec<Pair>,
    /// Pairs dropped by the `exclude` argument to [`new`](Self::new), kept for
    /// the caller to report.
    excluded: Vec<(String, String)>,
    /// Per-body world pose, refreshed by [`place`](Self::place). Fixed bodies
    /// keep the identity (their hulls are already in world frame).
    world_iso: Vec<Isometry3<f64>>,
}

impl BimanualCollisionModel {
    /// Build from the URDF (both chains) and its collision meshes, fitting the
    /// hulls and deriving the checked pairs at construction; there is no
    /// intermediate artifact to go stale. The model is a pure distance oracle:
    /// it reports clearances, and the caller decides what to do with them (see
    /// [`GovernorBand`](crate::GovernorBand) for the proximity law).
    ///
    /// `exclude` names pairs the caller asserts can never collide (a base link
    /// and a link a few joints down, say); they are dropped from the checked set.
    /// The assertion is trusted, not re-derived, so it is the caller's
    /// responsibility to get right: excluding a pair that can in fact collide
    /// silently removes that protection. The names must resolve to real bodies.
    /// The dropped pairs are reported by [`excluded_pairs`](Self::excluded_pairs).
    pub fn new(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        exclude: &[PairSpec],
    ) -> Result<Self, String> {
        // Candidate pairs: everything that can inform. Excluded structurally:
        // two world-fixed bodies (their distance never changes), and pairs
        // within two moving joints of each other, same-side or torso against a
        // chain's first links. Those are joint-yoked: shoulder or wrist cluster
        // members orbit each other through their whole range, so their distance
        // swings with every legitimate motion while real contact between them
        // is blocked by the link in between. Cross-arm pairs are always checked.
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
                    (Lineage::Torso, Lineage::Torso) => false,
                    (Lineage::Side(_, 0), Lineage::Torso) | (Lineage::Torso, Lineage::Side(_, 0)) => false,
                    (Lineage::Side(sa, 0), Lineage::Side(sb, 0)) if sa != sb => false,
                    (Lineage::Side(sa, da), Lineage::Side(sb, db)) if sa == sb => da.abs_diff(*db) > 2,
                    (Lineage::Torso, Lineage::Side(_, d)) | (Lineage::Side(_, d), Lineage::Torso) => *d > 2,
                    (Lineage::Side(..), Lineage::Side(..)) => true,
                };
                if keep {
                    specs.push(PairSpec::new(a.clone(), b.clone()));
                }
            }
        }
        probe.set_pairs(&specs)?;
        probe.exclude_named(exclude)?;
        Ok(probe)
    }

    /// Like [`new`](Self::new) but checking an explicit pair list and skipping
    /// the structural derivation (tests and special-purpose tools). An empty list
    /// builds the bodies with no checked pairs.
    pub fn with_pairs(urdf: &str, meshes_dir: &str, left_base: &str, right_base: &str, pair_specs: &[PairSpec]) -> Result<Self, String> {
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
        let push_body = |bodies: &mut Vec<Body>, body: Body| -> Result<(), String> {
            if bodies.iter().any(|b| b.name == body.name) {
                return Err(format!("duplicate body name '{}'", body.name));
            }
            bodies.push(body);
            Ok(())
        };
        let mut links = fitted.links;
        for (name, hulls) in fitted.fixed {
            let bound = BoundingSphere::of(&hulls);
            push_body(&mut bodies, Body { name, local: hulls, placement: Placement::Fixed, bound })?;
        }
        for (names, side_left) in [(&left_names, true), (&right_names, false)] {
            for (i, name) in names.iter().enumerate() {
                let hulls = links.remove(name).ok_or_else(|| format!("link '{name}' is shared between the two chains"))?;
                let placement = if side_left { Placement::Left(i) } else { Placement::Right(i) };
                let bound = BoundingSphere::of(&hulls);
                push_body(&mut bodies, Body { name: name.clone(), local: hulls, placement, bound })?;
            }
        }

        let world_iso = vec![Isometry3::identity(); bodies.len()];
        let mut model = Self { left, right, bodies, pairs: Vec::new(), excluded: Vec::new(), world_iso };
        model.set_pairs(pair_specs)?;
        Ok(model)
    }

    /// Drop the caller's named exclusions (see [`new`](Self::new)). The names
    /// must resolve to real bodies, but the assertion that the pair cannot
    /// collide is trusted, not re-derived: a pair that is not currently checked
    /// is a harmless no-op.
    fn exclude_named(&mut self, exclude: &[PairSpec]) -> Result<(), String> {
        for spec in exclude {
            let a = self.body_index(&spec.a)?;
            let b = self.body_index(&spec.b)?;
            let is_pair = |p: &Pair| (p.a == a && p.b == b) || (p.a == b && p.b == a);
            let before = self.pairs.len();
            self.pairs.retain(|p| !is_pair(p));
            if self.pairs.len() < before {
                self.excluded.push((self.bodies[a].name.clone(), self.bodies[b].name.clone()));
            }
        }
        Ok(())
    }

    /// The pairs dropped by the `exclude` argument to [`new`](Self::new), for the
    /// caller to report.
    pub fn excluded_pairs(&self) -> &[(String, String)] {
        &self.excluded
    }

    fn body_index(&self, name: &str) -> Result<usize, String> {
        self.bodies.iter().position(|b| b.name == name).ok_or_else(|| format!("unknown body '{name}'"))
    }

    /// Replace the checked pair list (names resolved against the bodies).
    fn set_pairs(&mut self, pair_specs: &[PairSpec]) -> Result<(), String> {
        let index: HashMap<&str, usize> = self.bodies.iter().enumerate().map(|(i, b)| (b.name.as_str(), i)).collect();
        self.pairs = pair_specs
            .iter()
            .map(|p| {
                let a = *index.get(p.a.as_str()).ok_or_else(|| format!("pair references unknown body '{}'", p.a))?;
                let b = *index.get(p.b.as_str()).ok_or_else(|| format!("pair references unknown body '{}'", p.b))?;
                if a == b {
                    return Err(format!("pair '{}' against itself", p.a));
                }
                Ok(Pair { a, b })
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
        exclude: &[PairSpec],
    ) -> Result<Self, String> {
        let urdf = std::fs::read_to_string(path).map_err(|e| format!("read urdf '{path}': {e}"))?;
        Self::new(&urdf, meshes_dir, left_base, right_base, exclude)
    }

    /// Link-local hull pieces of a body (fixed bodies are in the root frame),
    /// for diagnostics and tests.
    pub fn local_hulls(&self, name: &str) -> Option<&[Hull]> {
        self.bodies.iter().find(|b| b.name == name).map(|b| b.local.as_slice())
    }

    /// All checked pairs by name, for diagnostics and tests.
    pub fn checked_pairs(&self) -> Vec<(&str, &str)> {
        self.pairs.iter().map(|p| (self.bodies[p.a].name.as_str(), self.bodies[p.b].name.as_str())).collect()
    }

    /// Minimum signed distance over all checked pairs at the given
    /// configurations, with the witness points. Non-finite joint values are
    /// rejected so the caller fails safe rather than comparing against NaN.
    pub fn min_distance(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<Proximity<'_>, String> {
        ensure_finite(q_left, q_right)?;
        self.place(q_left, q_right);

        // Broadphase: a pair's bounding-sphere gap is a lower bound on its true
        // distance. Scanned in ascending order of that bound, once it exceeds
        // the best distance found no remaining pair can win, so the scan stops.
        let centers: Vec<Point3<f64>> = self.bodies.iter().zip(&self.world_iso).map(|(b, iso)| iso * b.bound.center).collect();
        let mut order: Vec<(f64, usize)> = self
            .pairs
            .iter()
            .enumerate()
            .map(|(i, p)| ((centers[p.a] - centers[p.b]).norm() - self.bodies[p.a].bound.radius - self.bodies[p.b].bound.radius, i))
            .collect();
        order.sort_by(|x, y| x.0.total_cmp(&y.0));

        let mut best: Option<Closest> = None;
        for (lower_bound, i) in order {
            if best.as_ref().is_some_and(|c| lower_bound > c.distance) {
                break;
            }
            let pair = &self.pairs[i];
            for ha in &self.bodies[pair.a].local {
                for hb in &self.bodies[pair.b].local {
                    let r = gjk::distance(&Placed::new(ha, self.world_iso[pair.a]), &Placed::new(hb, self.world_iso[pair.b]));
                    if best.as_ref().is_none_or(|c| r.distance < c.distance) {
                        best = Some(Closest { distance: r.distance, a: pair.a, b: pair.b, on_a: r.on_a, on_b: r.on_b });
                    }
                }
            }
        }
        let Some(c) = best else {
            return Err("no pairs to check".into());
        };
        Ok(Proximity { distance: c.distance, link_a: &self.bodies[c.a].name, link_b: &self.bodies[c.b].name, on_a: c.on_a, on_b: c.on_b })
    }

    /// True if any checked pair is at or below `threshold`.
    pub fn in_collision(&mut self, q_left: &JointVec, q_right: &JointVec, threshold: f64) -> Result<bool, String> {
        if !threshold.is_finite() {
            return Err(format!("collision threshold must be finite, got {threshold}"));
        }
        Ok(self.min_distance(q_left, q_right)?.distance <= threshold)
    }

    /// World-frame hull pieces of every body at the given configuration, paired
    /// with the body name (for visualization; runtime queries use
    /// [`min_distance`](Self::min_distance)). Each piece carries its placed
    /// vertices, the face triangles, and the inflation radius, so a caller can
    /// draw the true rounded collision surface, not just the bare core.
    pub fn world_pieces(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<BodyPieces<'_>, String> {
        ensure_finite(q_left, q_right)?;
        self.place(q_left, q_right);
        Ok(self
            .bodies
            .iter()
            .zip(&self.world_iso)
            .map(|(b, iso)| {
                let pieces = b
                    .local
                    .iter()
                    .map(|h| PlacedPiece {
                        vertices: h.vertices().iter().map(|v| iso * v).collect(),
                        faces: h.faces().to_vec(),
                        radius: h.inflation(),
                    })
                    .collect();
                (b.name.as_str(), pieces)
            })
            .collect())
    }

    /// Refresh the world pose of the moving bodies from FK.
    fn place(&mut self, q_left: &JointVec, q_right: &JointVec) {
        let poses_l = link_poses(&mut self.left, q_left);
        let poses_r = link_poses(&mut self.right, q_right);
        for (body, iso) in self.bodies.iter().zip(self.world_iso.iter_mut()) {
            *iso = match body.placement {
                Placement::Fixed => continue,
                Placement::Left(i) => poses_l[i],
                Placement::Right(i) => poses_r[i],
            };
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pairs::PairSpec;
    use crate::GovernorBand;

    const URDF: &str = include_str!("../tests/fixtures/openarm_v10.urdf");
    const MESHES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes");

    // The band the governor-law test gates with; d_safe 20 mm clears the rest
    // pose comfortably (closest pair ~33 mm).
    fn band() -> GovernorBand {
        GovernorBand::new(0.005, 0.02).expect("valid band")
    }

    fn model() -> BimanualCollisionModel {
        BimanualCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", &[]).expect("model")
    }

    fn build(pairs: &[PairSpec]) -> Result<BimanualCollisionModel, String> {
        BimanualCollisionModel::with_pairs(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", pairs)
    }

    #[test]
    fn rejects_unknown_pairs_and_querying_with_no_pairs() {
        assert!(build(&[PairSpec::new("openarm_left_link1", "no_such_body")]).err().expect("error").contains("unknown body"));
        let mut empty = build(&[]).expect("bodies build without pairs");
        assert!(empty.min_distance(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).is_err());
    }

    #[test]
    fn rejects_self_pairs_and_identical_bases() {
        assert!(build(&[PairSpec::new("openarm_left_link7", "openarm_left_link7")]).is_err());
        let e = BimanualCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_left_link0", &[])
            .err()
            .expect("identical bases must fail");
        assert!(e.contains("two chains"), "{e}");
    }

    #[test]
    fn derived_pairs_skip_fixed_pairs_and_adjacency() {
        let m = model();
        let checked: Vec<(String, String)> = m.checked_pairs().iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
        let has = |a: &str, b: &str| checked.iter().any(|(x, y)| (x == a && y == b) || (x == b && y == a));
        // Two fixed bodies never change distance; same-side within two joints
        // is joint-yoked noise.
        assert!(!has("openarm_left_link0", "openarm_right_link0"));
        assert!(!has("openarm_body_link0", "openarm_left_link0"));
        assert!(!has("openarm_left_link0", "openarm_left_link1"));
        assert!(!has("openarm_left_link3", "openarm_left_link4"));
        assert!(!has("openarm_body_link0", "openarm_left_link2"));
        // Beyond the horizon, and cross-arm, are checked.
        assert!(has("openarm_left_link1", "openarm_left_link7"));
        assert!(has("openarm_left_link0", "openarm_left_link4"));
        assert!(has("openarm_body_link0", "openarm_left_link3"));
        assert!(has("openarm_left_link7", "openarm_right_link7"));
    }

    #[test]
    fn torso_decomposes_into_several_pieces() {
        let m = model();
        assert!(m.local_hulls("openarm_body_link0").expect("torso").len() > 1, "the torso should decompose");
        assert_eq!(m.local_hulls("openarm_left_link7").expect("gripper").len(), 1, "the gripper stays one hull");
    }

    #[test]
    fn rest_pose_clears_d_safe() {
        let mut m = model();
        let p = m.min_distance(&[0.0; ARM_DOF], &[0.0; ARM_DOF]).expect("query");
        assert!(p.distance >= 0.02 - 1e-9, "rest min {:+.4} should clear d_safe", p.distance);
    }

    #[test]
    fn broadphase_min_matches_a_brute_force_scan() {
        // The broadphase sorts pairs and stops early once the lower bound exceeds
        // the running minimum. Pin that this never changes the answer: it must
        // equal a full scan of every checked pair at every pose.
        let mut m = model();
        let pairs: Vec<(String, String)> = m.checked_pairs().iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
        let poses = [
            ([0.0; ARM_DOF], [0.0; ARM_DOF]),
            ([0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0], [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0]),
            ([-0.4, -0.1, 0.0, 0.5, 0.0, -0.3, 0.0], [0.4, 0.1, 0.0, 0.7, 0.0, -0.2, 0.0]),
        ];
        for (ql, qr) in poses {
            let fast = m.min_distance(&ql, &qr).expect("query").distance;
            let placed: HashMap<String, Vec<gjk::Hull>> = m
                .world_pieces(&ql, &qr)
                .expect("pieces")
                .into_iter()
                .map(|(name, ps)| {
                    let hulls = ps
                        .into_iter()
                        .map(|p| gjk::Hull::new(&crate::hull::ConvexHull { vertices: p.vertices, faces: p.faces }, p.radius).expect("hull"))
                        .collect();
                    (name.to_string(), hulls)
                })
                .collect();
            let slow = pairs.iter().fold(f64::INFINITY, |best, (a, b)| {
                placed[a].iter().flat_map(|ha| placed[b].iter().map(move |hb| gjk::distance(ha, hb).distance)).fold(best, f64::min)
            });
            assert!((fast - slow).abs() < 1e-9, "broadphase {fast:+.6} != brute force {slow:+.6}");
        }
    }

    fn excluding(pairs: &[PairSpec]) -> Result<BimanualCollisionModel, String> {
        BimanualCollisionModel::new(URDF, MESHES, "openarm_left_link0", "openarm_right_link0", pairs)
    }

    #[test]
    fn excludes_a_named_pair_and_reports_it() {
        let same = |a: &str, b: &str, x: &str, y: &str| (a == x && b == y) || (a == y && b == x);
        let m = excluding(&[PairSpec::new("openarm_left_link0", "openarm_left_link3")]).expect("model");
        assert!(!m.checked_pairs().iter().any(|(a, b)| same(a, b, "openarm_left_link0", "openarm_left_link3")), "should be dropped");
        assert!(m.excluded_pairs().iter().any(|(a, b)| same(a, b, "openarm_left_link0", "openarm_left_link3")), "should be reported");
    }

    #[test]
    fn rejects_excluding_an_unknown_body() {
        let e = excluding(&[PairSpec::new("openarm_left_link0", "no_such_link")]).err().expect("unknown body must fail");
        assert!(e.contains("unknown body"), "{e}");
    }

    #[test]
    fn overlapping_bodies_report_negative_distance() {
        // Grippers wrapped toward each other across the torso: the winner
        // overlaps, and EPA reports a negative depth.
        let mut m = model();
        let p = m.min_distance(&[0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0], &[0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0]).expect("query");
        assert!(p.distance < 0.0, "wrapped pose should overlap, got {:+.4}", p.distance);
    }

    #[test]
    fn separating_motion_always_passes_even_from_overlap() {
        // The criterion: from a colliding pose, moving apart is full speed;
        // moving deeper is throttled. EPA's continuous signed distance is what
        // lets the band tell the two apart inside an overlap.
        let mut m = model();
        let deep = m.min_distance(&[0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0], &[0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0]).expect("q").distance;
        let shallow = m.min_distance(&[0.0, 0.0, 1.0, 0.4, 0.0, 0.0, 0.0], &[0.0, 0.0, -1.0, 0.4, 0.0, 0.0, 0.0]).expect("q").distance;
        assert!(deep < 0.0 && shallow > deep, "deep {deep:+.4} shallow {shallow:+.4}");
        let band = band();
        assert_eq!(band.scale(deep, shallow), 1.0, "separating from overlap must pass at full speed");
        assert!(band.scale(shallow, deep) < 1.0, "approaching into overlap must throttle");
    }

    #[test]
    fn rejects_non_finite_queries() {
        let mut m = model();
        let mut bad = [0.0; ARM_DOF];
        bad[0] = f64::NAN;
        assert!(m.min_distance(&bad, &[0.0; ARM_DOF]).is_err());
        assert!(m.world_pieces(&bad, &[0.0; ARM_DOF]).is_err());
    }

    #[test]
    fn model_is_send_for_task_ownership() {
        fn assert_send<T: Send>() {}
        assert_send::<BimanualCollisionModel>();
    }
}
