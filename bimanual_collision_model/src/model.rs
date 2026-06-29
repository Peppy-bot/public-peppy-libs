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
//! The hulls are tight: the reported distance is the true surface clearance, with
//! no safety margin baked into the geometry. Keeping the arms apart is the
//! caller's job (a deployment band over the reported distance), not a margin the
//! model papers over.

use std::collections::HashMap;

use srs_model::nalgebra::{Isometry3, Point3, Vector3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::assemble::fit_bodies;
use crate::{BuildError, CollisionError};
use crate::gjk::{self, Hull, Placed};
use crate::hull::ConvexPiece;
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
        let center = Point3::from(
            verts.iter().fold(Vector3::zeros(), |a, p| a + p.coords) / verts.len() as f64,
        );
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

/// The nearest-pair [`Proximity`] at one configuration plus the gradient of its
/// surface distance with respect to each arm's joints. `grad_left[j]` is
/// `d(distance)/d(q_left[j])`; separating motion has a positive gradient. Computed
/// analytically from the nearest pair's witness points (the gradient of the active
/// pair, by the envelope theorem), so it costs one distance query plus two point
/// Jacobians.
#[derive(Debug, Clone)]
pub struct DistanceGradient<'a> {
    pub proximity: Proximity<'a>,
    pub grad_left: JointVec,
    pub grad_right: JointVec,
}

/// Witness separation below which the surface normal is ill-defined (deep
/// penetration where the two witnesses coincide); the gradient query fails there.
const WITNESS_MIN_SEPARATION: f64 = 1e-9;

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
    /// Pairs dropped by [`Builder::exclude`], kept for the caller to report.
    excluded: Vec<(String, String)>,
    /// Per-body world pose, refreshed by [`place`](Self::place). Fixed bodies
    /// keep the identity (their hulls are already in world frame).
    world_iso: Vec<Isometry3<f64>>,
}

/// Configures and builds a [`BimanualCollisionModel`]; start from
/// [`BimanualCollisionModel::builder`].
pub struct Builder {
    urdf: String,
    meshes_dir: String,
    left_base: String,
    right_base: String,
    exclude: Vec<PairSpec>,
    supplied: HashMap<String, Vec<ConvexPiece>>,
}

impl Builder {
    /// Drop these pairs from checking. The caller asserts they can never collide;
    /// the assertion is trusted, not re-derived, so excluding a pair that can in
    /// fact collide silently removes that protection. The names must resolve to
    /// real bodies. Dropped pairs are reported by
    /// [`excluded_pairs`](BimanualCollisionModel::excluded_pairs).
    pub fn exclude(mut self, pairs: &[PairSpec]) -> Self {
        self.exclude.extend_from_slice(pairs);
        self
    }

    /// Supply convex pieces for a body, replacing its auto-fit hull. The pieces
    /// must together contain the body's mesh, checked at build. This is how to
    /// give a concave body (a torso) a tight proxy that a single convex hull
    /// cannot. Naming a body that does not exist errors at build.
    pub fn hulls(mut self, body: &str, pieces: Vec<ConvexPiece>) -> Self {
        self.supplied.insert(body.to_string(), pieces);
        self
    }

    /// Fit the bodies (supplied pieces override the auto-fit), derive the checked
    /// pairs from the structural rules, and apply the exclusions.
    pub fn build(self) -> Result<BimanualCollisionModel, CollisionError> {
        let mut model = BimanualCollisionModel::assemble(
            &self.urdf,
            &self.meshes_dir,
            &self.left_base,
            &self.right_base,
            &self.supplied,
        )?;
        // Candidate pairs: everything that can inform. Excluded structurally:
        // two world-fixed bodies (their distance never changes), and pairs within
        // two moving joints of each other, same-side or torso against a chain's
        // first links. Those are joint-yoked: shoulder or wrist cluster members
        // orbit each other through their whole range, so their distance swings
        // with every legitimate motion while real contact between them is blocked
        // by the link in between. Cross-arm pairs are always checked.
        let lineage: Vec<(String, Lineage)> = model
            .bodies
            .iter()
            .map(|b| {
                let lineage = match b.placement {
                    Placement::Left(i) => Lineage::Side(0, i + 1),
                    Placement::Right(i) => Lineage::Side(1, i + 1),
                    Placement::Fixed if b.name == self.left_base => Lineage::Side(0, 0),
                    Placement::Fixed if b.name == self.right_base => Lineage::Side(1, 0),
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
                    (Lineage::Side(_, 0), Lineage::Torso)
                    | (Lineage::Torso, Lineage::Side(_, 0)) => false,
                    (Lineage::Side(sa, 0), Lineage::Side(sb, 0)) if sa != sb => false,
                    (Lineage::Side(sa, da), Lineage::Side(sb, db)) if sa == sb => {
                        da.abs_diff(*db) > 2
                    }
                    (Lineage::Torso, Lineage::Side(_, d))
                    | (Lineage::Side(_, d), Lineage::Torso) => *d > 2,
                    (Lineage::Side(..), Lineage::Side(..)) => true,
                };
                if keep {
                    specs.push(PairSpec::new(a.clone(), b.clone()));
                }
            }
        }
        model.set_pairs(&specs)?;
        model.exclude_named(&self.exclude)?;
        Ok(model)
    }
}

impl BimanualCollisionModel {
    /// Start building a model from a URDF string and its collision mesh
    /// directory, naming the two chain base links. See [`Builder`]. The model is
    /// a pure distance oracle: it reports clearances (and, via
    /// [`distance_gradient`](Self::distance_gradient), their gradient), and the
    /// caller decides how to throttle on them.
    pub fn builder(urdf: &str, meshes_dir: &str, left_base: &str, right_base: &str) -> Builder {
        Builder {
            urdf: urdf.to_string(),
            meshes_dir: meshes_dir.to_string(),
            left_base: left_base.to_string(),
            right_base: right_base.to_string(),
            exclude: Vec::new(),
            supplied: HashMap::new(),
        }
    }

    /// Like [`builder`](Self::builder) but reading the URDF from a file.
    pub fn builder_from_file(
        path: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
    ) -> Result<Builder, CollisionError> {
        let urdf = std::fs::read_to_string(path)
            .map_err(|e| BuildError::Geometry(format!("read urdf '{path}': {e}")))?;
        Ok(Self::builder(&urdf, meshes_dir, left_base, right_base))
    }

    /// Build the bodies with an explicit checked-pair list and no structural
    /// derivation, bypassing the safety-relevant pair rules. Test-only: the
    /// public path is [`builder`](Self::builder). An empty list builds the
    /// bodies with no checked pairs.
    #[cfg(test)]
    fn with_pairs(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        pair_specs: &[PairSpec],
    ) -> Result<Self, BuildError> {
        let mut model = Self::assemble(urdf, meshes_dir, left_base, right_base, &HashMap::new())?;
        model.set_pairs(pair_specs)?;
        Ok(model)
    }

    /// Fit every collision body (supplied pieces override the auto-fit) and place
    /// them, with no checked pairs set yet.
    fn assemble(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        supplied: &HashMap<String, Vec<ConvexPiece>>,
    ) -> Result<Self, BuildError> {
        if left_base == right_base {
            return Err(BuildError::IdenticalBases { base: left_base.to_string() });
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
        let fitted = fit_bodies(
            &parsed,
            &[left_names.clone(), right_names.clone()],
            meshes_dir,
            supplied,
        )?;

        let mut bodies: Vec<Body> = Vec::new();
        let push_body = |bodies: &mut Vec<Body>, body: Body| -> Result<(), BuildError> {
            if bodies.iter().any(|b| b.name == body.name) {
                return Err(BuildError::DuplicateBody { name: body.name.clone() });
            }
            bodies.push(body);
            Ok(())
        };
        let mut links = fitted.links;
        for (name, hulls) in fitted.fixed {
            let bound = BoundingSphere::of(&hulls);
            push_body(
                &mut bodies,
                Body {
                    name,
                    local: hulls,
                    placement: Placement::Fixed,
                    bound,
                },
            )?;
        }
        for (names, side_left) in [(&left_names, true), (&right_names, false)] {
            for (i, name) in names.iter().enumerate() {
                let hulls = links
                    .remove(name)
                    .ok_or_else(|| BuildError::SharedLink { name: name.clone() })?;
                let placement = if side_left {
                    Placement::Left(i)
                } else {
                    Placement::Right(i)
                };
                let bound = BoundingSphere::of(&hulls);
                push_body(
                    &mut bodies,
                    Body {
                        name: name.clone(),
                        local: hulls,
                        placement,
                        bound,
                    },
                )?;
            }
        }

        let world_iso = vec![Isometry3::identity(); bodies.len()];
        Ok(Self {
            left,
            right,
            bodies,
            pairs: Vec::new(),
            excluded: Vec::new(),
            world_iso,
        })
    }

    /// Drop the caller's named exclusions (see [`Builder::exclude`]). The names
    /// must resolve to real bodies, but the assertion that the pair cannot
    /// collide is trusted, not re-derived: a pair that is not currently checked
    /// is a harmless no-op.
    fn exclude_named(&mut self, exclude: &[PairSpec]) -> Result<(), BuildError> {
        for spec in exclude {
            let a = self.body_index(&spec.a)?;
            let b = self.body_index(&spec.b)?;
            let is_pair = |p: &Pair| (p.a == a && p.b == b) || (p.a == b && p.b == a);
            let before = self.pairs.len();
            self.pairs.retain(|p| !is_pair(p));
            if self.pairs.len() < before {
                self.excluded
                    .push((self.bodies[a].name.clone(), self.bodies[b].name.clone()));
            }
        }
        Ok(())
    }

    /// The pairs dropped by [`Builder::exclude`], for the caller to report.
    pub fn excluded_pairs(&self) -> &[(String, String)] {
        &self.excluded
    }

    fn body_index(&self, name: &str) -> Result<usize, BuildError> {
        self.bodies
            .iter()
            .position(|b| b.name == name)
            .ok_or_else(|| BuildError::UnknownBody { name: name.to_string() })
    }

    /// Replace the checked pair list (names resolved against the bodies).
    fn set_pairs(&mut self, pair_specs: &[PairSpec]) -> Result<(), BuildError> {
        let index: HashMap<&str, usize> = self
            .bodies
            .iter()
            .enumerate()
            .map(|(i, b)| (b.name.as_str(), i))
            .collect();
        self.pairs = pair_specs
            .iter()
            .map(|p| {
                let a = *index
                    .get(p.a.as_str())
                    .ok_or_else(|| BuildError::UnknownPairBody { name: p.a.clone() })?;
                let b = *index
                    .get(p.b.as_str())
                    .ok_or_else(|| BuildError::UnknownPairBody { name: p.b.clone() })?;
                if a == b {
                    return Err(BuildError::SelfPair { name: p.a.clone() });
                }
                Ok(Pair { a, b })
            })
            .collect::<Result<Vec<_>, BuildError>>()?;
        Ok(())
    }

    /// Link-local hull pieces of a body (fixed bodies are in the root frame).
    /// Exposes the internal [`Hull`], so it is test-only.
    #[cfg(test)]
    fn local_hulls(&self, name: &str) -> Option<&[Hull]> {
        self.bodies
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.local.as_slice())
    }

    /// All checked pairs by name, for diagnostics and tests.
    pub fn checked_pairs(&self) -> Vec<(&str, &str)> {
        self.pairs
            .iter()
            .map(|p| {
                (
                    self.bodies[p.a].name.as_str(),
                    self.bodies[p.b].name.as_str(),
                )
            })
            .collect()
    }

    /// The nearest checked pair at the given configurations: places the hulls by
    /// FK, then scans the pairs (broadphase-ordered) for the minimum signed
    /// distance. The shared core of [`min_distance`](Self::min_distance) and
    /// [`distance_gradient`](Self::distance_gradient).
    fn closest(&mut self, q_left: &JointVec, q_right: &JointVec) -> Result<Closest, CollisionError> {
        self.place(q_left, q_right);

        // Broadphase: a pair's bounding-sphere gap is a lower bound on its true
        // distance. Scanned in ascending order of that bound, once it exceeds
        // the best distance found no remaining pair can win, so the scan stops.
        let centers: Vec<Point3<f64>> = self
            .bodies
            .iter()
            .zip(&self.world_iso)
            .map(|(b, iso)| iso * b.bound.center)
            .collect();
        let mut order: Vec<(f64, usize)> = self
            .pairs
            .iter()
            .enumerate()
            .map(|(i, p)| {
                (
                    (centers[p.a] - centers[p.b]).norm()
                        - self.bodies[p.a].bound.radius
                        - self.bodies[p.b].bound.radius,
                    i,
                )
            })
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
                    let r = gjk::distance(
                        &Placed::new(ha, self.world_iso[pair.a]),
                        &Placed::new(hb, self.world_iso[pair.b]),
                    );
                    if best.as_ref().is_none_or(|c| r.distance < c.distance) {
                        best = Some(Closest {
                            distance: r.distance,
                            a: pair.a,
                            b: pair.b,
                            on_a: r.on_a,
                            on_b: r.on_b,
                        });
                    }
                }
            }
        }
        best.ok_or(CollisionError::NoPairs)
    }

    /// Minimum signed distance over all checked pairs at the given
    /// configurations, with the witness points. Non-finite joint values are
    /// rejected so the caller fails safe rather than comparing against NaN.
    pub fn min_distance(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<Proximity<'_>, CollisionError> {
        ensure_finite(q_left, q_right)?;
        let c = self.closest(q_left, q_right)?;
        Ok(Proximity {
            distance: c.distance,
            link_a: &self.bodies[c.a].name,
            link_b: &self.bodies[c.b].name,
            on_a: c.on_a,
            on_b: c.on_b,
        })
    }

    /// The nearest-pair [`Proximity`] and the analytic gradient of its distance
    /// with respect to each arm's joints (see [`DistanceGradient`]). The gradient
    /// is the nearest pair's witness normal projected through each witness point's
    /// velocity Jacobian, so it reflects the same min-over-pairs distance
    /// `min_distance` returns at one distance query's cost. Fails on a non-finite
    /// configuration, or when the witnesses coincide (deep penetration) and the
    /// normal is undefined; a velocity-barrier caller holds there.
    pub fn distance_gradient(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<DistanceGradient<'_>, CollisionError> {
        ensure_finite(q_left, q_right)?;
        let c = self.closest(q_left, q_right)?;
        let separation = c.on_b - c.on_a;
        let norm = separation.norm();
        if norm < WITNESS_MIN_SEPARATION {
            return Err(CollisionError::WitnessesCoincide { distance: c.distance });
        }
        // Unit normal along the witness separation (points a -> b). Each witness
        // moves only its own arm; a world-fixed witness (torso) contributes nothing.
        // The per-body signs below (+1 on a, -1 on b) are the convention that makes
        // the projected witness velocities sum to d(distance)/dq; the result is
        // checked against central differences in
        // `distance_gradient_matches_finite_difference`.
        let normal = separation / norm;
        let (place_a, place_b) = (self.bodies[c.a].placement, self.bodies[c.b].placement);
        let (left_a, right_a) = self.gradient_contribution(place_a, &c.on_a, &normal, 1.0, q_left, q_right);
        let (left_b, right_b) = self.gradient_contribution(place_b, &c.on_b, &normal, -1.0, q_left, q_right);
        Ok(DistanceGradient {
            proximity: Proximity {
                distance: c.distance,
                link_a: &self.bodies[c.a].name,
                link_b: &self.bodies[c.b].name,
                on_a: c.on_a,
                on_b: c.on_b,
            },
            grad_left: std::array::from_fn(|j| left_a[j] + left_b[j]),
            grad_right: std::array::from_fn(|j| right_a[j] + right_b[j]),
        })
    }

    /// One body's contribution to the per-arm distance gradient: the witness
    /// `normal` projected through the witness `point`'s velocity Jacobian, on the
    /// arm the body belongs to. `sign` is +1 for body a's witness and -1 for body
    /// b's (the convention that yields d(distance)/dq; see `distance_gradient` and
    /// its finite-difference test). A world-fixed body (torso) contributes nothing.
    fn gradient_contribution(
        &mut self,
        placement: Placement,
        point: &Point3<f64>,
        normal: &Vector3<f64>,
        sign: f64,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> (JointVec, JointVec) {
        let zero: JointVec = [0.0; ARM_DOF];
        let project = |arm: &mut Arm, q: &JointVec, segment: usize| -> JointVec {
            let cols = arm.at(q).point_world_jacobian(point, segment);
            std::array::from_fn(|j| sign * normal.dot(&cols[j]))
        };
        match placement {
            Placement::Fixed => (zero, zero),
            Placement::Left(s) => (project(&mut self.left, q_left, s), zero),
            Placement::Right(s) => (zero, project(&mut self.right, q_right, s)),
        }
    }

    /// True if any checked pair is at or below `threshold`.
    pub fn in_collision(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
        threshold: f64,
    ) -> Result<bool, CollisionError> {
        if !threshold.is_finite() {
            return Err(CollisionError::NonFinite);
        }
        Ok(self.min_distance(q_left, q_right)?.distance <= threshold)
    }

    /// World-frame hull pieces of every body at the given configuration, paired
    /// with the body name (for visualization; runtime queries use
    /// [`min_distance`](Self::min_distance)). Each piece carries its placed
    /// vertices, the face triangles, and the inflation radius, so a caller can
    /// draw the true rounded collision surface, not just the bare core.
    pub fn world_pieces(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<BodyPieces<'_>, CollisionError> {
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
fn ensure_finite(q_left: &JointVec, q_right: &JointVec) -> Result<(), CollisionError> {
    if q_left.iter().chain(q_right).any(|x| !x.is_finite()) {
        return Err(CollisionError::NonFinite);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pairs::PairSpec;

    const URDF: &str = include_str!("../tests/fixtures/openarm_v10.urdf");
    const MESHES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes");

    fn model() -> BimanualCollisionModel {
        BimanualCollisionModel::builder(URDF, MESHES, "openarm_left_link0", "openarm_right_link0")
            .build()
            .expect("model")
    }

    // Two boxes that jointly span the padded torso bounding box (split in z), so
    // they trivially contain the mesh. Just enough to exercise the multi-piece
    // replacement path; the tuned deployment geometry lives in the shared
    // tests/fixtures/openarm.rs, exercised by the integration test.
    fn containing_boxes() -> Vec<ConvexPiece> {
        vec![
            ConvexPiece::aabb(
                Point3::new(-0.157, -0.097, -0.002),
                Point3::new(0.097, 0.097, 0.404),
            ),
            ConvexPiece::aabb(
                Point3::new(-0.157, -0.097, 0.396),
                Point3::new(0.097, 0.097, 0.775),
            ),
        ]
    }

    fn build(pairs: &[PairSpec]) -> Result<BimanualCollisionModel, BuildError> {
        BimanualCollisionModel::with_pairs(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
            pairs,
        )
    }

    #[test]
    fn distance_gradient_matches_finite_difference() {
        let mut m = model();
        let h = 1e-5;
        // Wrists folded inward but ASYMMETRICALLY, so one moving cross-arm pair is
        // unambiguously nearest. A left/right-symmetric pose sits on a pair-switch
        // tie where the single-pair analytic gradient and the straddling central
        // difference legitimately disagree, so it would not be a valid check.
        let configs: [(JointVec, JointVec); 3] = [
            ([0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0], [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0]),
            ([0.0, 0.3, 0.95, 0.45, 0.1, 0.0, 0.0], [0.0, -0.1, -0.55, 0.4, 0.0, 0.1, 0.0]),
            ([0.25, -0.1, 0.6, 0.65, 0.0, 0.2, 0.1], [-0.1, 0.05, -0.7, 0.3, 0.0, -0.2, 0.0]),
        ];
        for (ql, qr) in configs {
            let grad = m.distance_gradient(&ql, &qr).expect("gradient defined");
            let (analytic_left, analytic_right) = (grad.grad_left, grad.grad_right);
            for j in 0..ARM_DOF {
                let mut lp = ql;
                let mut lm = ql;
                lp[j] += h;
                lm[j] -= h;
                let fd_left = (m.min_distance(&lp, &qr).unwrap().distance
                    - m.min_distance(&lm, &qr).unwrap().distance)
                    / (2.0 * h);
                let mut rp = qr;
                let mut rm = qr;
                rp[j] += h;
                rm[j] -= h;
                let fd_right = (m.min_distance(&ql, &rp).unwrap().distance
                    - m.min_distance(&ql, &rm).unwrap().distance)
                    / (2.0 * h);
                assert!((analytic_left[j] - fd_left).abs() < 3e-3, "left j{j}: analytic {} fd {fd_left}", analytic_left[j]);
                assert!((analytic_right[j] - fd_right).abs() < 3e-3, "right j{j}: analytic {} fd {fd_right}", analytic_right[j]);
            }
        }
    }

    #[test]
    fn rejects_unknown_pairs_and_querying_with_no_pairs() {
        assert!(matches!(
            build(&[PairSpec::new("openarm_left_link1", "no_such_body")]).err(),
            Some(BuildError::UnknownPairBody { .. })
        ));
        let mut empty = build(&[]).expect("bodies build without pairs");
        assert!(
            empty
                .min_distance(&[0.0; ARM_DOF], &[0.0; ARM_DOF])
                .is_err()
        );
    }

    #[test]
    fn rejects_self_pairs_and_identical_bases() {
        assert!(build(&[PairSpec::new("openarm_left_link7", "openarm_left_link7")]).is_err());
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_left_link0",
        )
        .build()
        .err()
        .expect("identical bases must fail");
        assert!(matches!(&e, CollisionError::Build(BuildError::IdenticalBases { .. })), "{e}");
    }

    #[test]
    fn derived_pairs_skip_fixed_pairs_and_adjacency() {
        let m = model();
        let checked: Vec<(String, String)> = m
            .checked_pairs()
            .iter()
            .map(|(a, b)| (a.to_string(), b.to_string()))
            .collect();
        let has = |a: &str, b: &str| {
            checked
                .iter()
                .any(|(x, y)| (x == a && y == b) || (x == b && y == a))
        };
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
    fn auto_fit_is_one_hull_per_body() {
        let m = model();
        assert_eq!(
            m.local_hulls("openarm_body_link0").expect("torso").len(),
            1,
            "auto-fit is a single hull"
        );
        assert_eq!(
            m.local_hulls("openarm_left_link7").expect("gripper").len(),
            1
        );
    }

    #[test]
    fn supplied_hulls_replace_the_auto_fit() {
        let m = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .hulls("openarm_body_link0", containing_boxes())
        .build()
        .expect("supplied torso boxes contain the mesh");
        assert_eq!(
            m.local_hulls("openarm_body_link0").expect("torso").len(),
            2,
            "torso uses the two supplied boxes"
        );
    }

    #[test]
    fn rejects_supplied_hulls_that_miss_the_mesh() {
        let tiny = vec![ConvexPiece::aabb(
            Point3::new(-0.01, -0.01, 0.0),
            Point3::new(0.01, 0.01, 0.02),
        )];
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .hulls("openarm_body_link0", tiny)
        .build()
        .err()
        .expect("a too-small hull must be rejected");
        assert!(matches!(&e, CollisionError::Build(BuildError::HullMissesMesh { .. })), "{e}");
    }

    #[test]
    fn rejects_supplied_hulls_for_unknown_body() {
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .hulls("no_such_body", containing_boxes())
        .build()
        .err()
        .expect("unknown body must fail");
        assert!(matches!(&e, CollisionError::Build(BuildError::UnknownSuppliedBody { .. })), "{e}");
    }

    #[test]
    fn rejects_empty_supplied_hulls() {
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .hulls("openarm_body_link0", Vec::new())
        .build()
        .err()
        .expect("empty pieces must fail");
        assert!(matches!(&e, CollisionError::Build(BuildError::EmptyHulls { .. })), "{e}");
    }

    #[test]
    fn broadphase_min_matches_a_brute_force_scan() {
        // The broadphase sorts pairs and stops early once the lower bound exceeds
        // the running minimum. Pin that this never changes the answer: it must
        // equal a full scan of every checked pair at every pose.
        let mut m = model();
        let pairs: Vec<(String, String)> = m
            .checked_pairs()
            .iter()
            .map(|(a, b)| (a.to_string(), b.to_string()))
            .collect();
        let poses = [
            ([0.0; ARM_DOF], [0.0; ARM_DOF]),
            (
                [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
            ),
            (
                [-0.4, -0.1, 0.0, 0.5, 0.0, -0.3, 0.0],
                [0.4, 0.1, 0.0, 0.7, 0.0, -0.2, 0.0],
            ),
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
                        .map(|p| {
                            gjk::Hull::new(
                                &crate::hull::ConvexHull {
                                    vertices: p.vertices,
                                    faces: p.faces,
                                },
                                p.radius,
                            )
                            .expect("hull")
                        })
                        .collect();
                    (name.to_string(), hulls)
                })
                .collect();
            let slow = pairs.iter().fold(f64::INFINITY, |best, (a, b)| {
                placed[a]
                    .iter()
                    .flat_map(|ha| {
                        placed[b]
                            .iter()
                            .map(move |hb| gjk::distance(ha, hb).distance)
                    })
                    .fold(best, f64::min)
            });
            assert!(
                (fast - slow).abs() < 1e-9,
                "broadphase {fast:+.6} != brute force {slow:+.6}"
            );
        }
    }

    fn excluding(pairs: &[PairSpec]) -> Result<BimanualCollisionModel, CollisionError> {
        BimanualCollisionModel::builder(URDF, MESHES, "openarm_left_link0", "openarm_right_link0")
            .exclude(pairs)
            .build()
    }

    #[test]
    fn excludes_a_named_pair_and_reports_it() {
        let same = |a: &str, b: &str, x: &str, y: &str| (a == x && b == y) || (a == y && b == x);
        let m =
            excluding(&[PairSpec::new("openarm_left_link0", "openarm_left_link3")]).expect("model");
        assert!(
            !m.checked_pairs().iter().any(|(a, b)| same(
                a,
                b,
                "openarm_left_link0",
                "openarm_left_link3"
            )),
            "should be dropped"
        );
        assert!(
            m.excluded_pairs().iter().any(|(a, b)| same(
                a,
                b,
                "openarm_left_link0",
                "openarm_left_link3"
            )),
            "should be reported"
        );
    }

    #[test]
    fn rejects_excluding_an_unknown_body() {
        let e = excluding(&[PairSpec::new("openarm_left_link0", "no_such_link")])
            .err()
            .expect("unknown body must fail");
        assert!(matches!(&e, CollisionError::Build(BuildError::UnknownBody { .. })), "{e}");
    }

    #[test]
    fn overlapping_bodies_report_negative_distance() {
        // Grippers wrapped toward each other across the torso: the winner
        // overlaps, and EPA reports a negative depth.
        let mut m = model();
        let p = m
            .min_distance(
                &[0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
                &[0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
            )
            .expect("query");
        assert!(
            p.distance < 0.0,
            "wrapped pose should overlap, got {:+.4}",
            p.distance
        );
    }

    #[test]
    fn epa_gives_continuous_signed_distance_through_overlap() {
        // EPA recovers penetration depth as a continuous signed distance: a more
        // deeply wrapped pose reads more negative than a shallower one, so a caller
        // can tell approaching from separating even from inside an overlap.
        let mut m = model();
        let deep = m
            .min_distance(
                &[0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
                &[0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
            )
            .expect("q")
            .distance;
        let shallow = m
            .min_distance(
                &[0.0, 0.0, 1.0, 0.4, 0.0, 0.0, 0.0],
                &[0.0, 0.0, -1.0, 0.4, 0.0, 0.0, 0.0],
            )
            .expect("q")
            .distance;
        assert!(
            deep < 0.0 && shallow > deep,
            "deep {deep:+.4} shallow {shallow:+.4}"
        );
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
