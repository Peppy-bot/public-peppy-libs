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

use srs_model::nalgebra::{Isometry3, Point3, Unit, Vector3};
use srs_model::{ARM_DOF, Arm, JointVec};

use crate::assemble::fit_bodies;
use crate::clip::ClipRegion;
use crate::gjk::{self, Hull, Placed};
use crate::pairs::PairSpec;
use crate::urdf_collision::{JointKind, ParentJoint, UrdfCollisions, ZERO_AXIS_EPS, place_1dof};
use crate::{BuildError, CollisionError};

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
        let (center, radius) =
            gjk::enclosing_sphere(hulls.iter().map(|h| (h.vertices(), h.inflation())));
        BoundingSphere { center, radius }
    }
}

struct Body {
    name: String,
    /// Convex-hull pieces, in the body's local frame (world for `Fixed`).
    local: Vec<Hull>,
    placement: Placement,
    bound: BoundingSphere,
    /// For a gripper finger: how its hull hangs off its host chain link, so it
    /// can be placed at the live opening. `None` for every ordinary body (whose
    /// hulls sit directly in the link frame).
    finger: Option<Finger>,
}

impl Body {
    /// World pose of this body given its host chain link's world pose and its
    /// side's gripper opening: the link pose directly for an ordinary body, or
    /// composed with the finger offset at `opening` for a gripper finger.
    fn place_on(&self, link_world: Isometry3<f64>, opening: f64) -> Isometry3<f64> {
        match &self.finger {
            Some(f) => link_world * f.offset(opening),
            None => link_world,
        }
    }
}

/// A gripper finger's parent joint, parsed at build into an infallible placer:
/// the joint origin, its unit axis, whether it rotates (revolute) or slides
/// (prismatic), and the finger-joint travel oriented closed-to-open by the
/// build's mesh-separation check (see `assemble::orient_finger_pair`).
/// Validated once here (bounded 1-DOF, non-zero axis) so placing it at any
/// opening cannot fail in the per-tick hot path.
#[derive(Clone, Copy)]
struct Finger {
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
    revolute: bool,
    /// Finger-joint position (URDF joint units) at fully closed and fully open.
    closed: f64,
    open: f64,
}

impl Finger {
    /// Parse a finger's parent joint into a validated placer, the single gate
    /// for finger-joint kinds: only a prismatic or revolute finger can be
    /// placed, and its axis must normalize. `closed`/`open` are the
    /// mesh-oriented travel extremes from the fit.
    fn from_joint(
        name: &str,
        joint: &ParentJoint,
        closed: f64,
        open: f64,
    ) -> Result<Self, BuildError> {
        let revolute = match joint.kind {
            JointKind::Revolute => true,
            JointKind::Prismatic => false,
            _ => {
                return Err(BuildError::Geometry(format!(
                    "finger '{name}' joint kind {:?} is not a placeable 1-DOF finger",
                    joint.kind
                )));
            }
        };
        if joint.axis.norm() < ZERO_AXIS_EPS {
            return Err(BuildError::Geometry(format!(
                "finger '{name}' parent joint has a zero axis"
            )));
        }
        Ok(Finger {
            origin: joint.origin,
            axis: Unit::new_normalize(joint.axis),
            revolute,
            closed,
            open,
        })
    }

    /// Placement of the finger hull in its host link's frame at opening `fraction`
    /// in `[0, 1]` (0 = fully closed, 1 = fully open), linearly interpolating the
    /// finger-joint travel. The whole opening pipeline is fraction-native, so the
    /// fraction maps straight onto the joint's own travel for the prismatic (v1)
    /// and revolute (v2) grippers alike.
    fn offset(&self, fraction: f64) -> Isometry3<f64> {
        let q = self.closed + fraction.clamp(0.0, 1.0) * (self.open - self.closed);
        place_1dof(&self.origin, &self.axis, self.revolute, q)
    }

    /// World velocity of a point riding this finger per unit opening fraction,
    /// given the host link's world pose. The joint frame's world rotation and
    /// origin are invariant under the finger's own motion (a joint moves about
    /// its own axis), so the field needs no current fraction: a slide moves every
    /// point along the world axis; a rotation swings the point about the joint
    /// origin. Both scale by the joint travel per unit fraction.
    fn point_velocity_per_fraction(
        &self,
        link_world: &Isometry3<f64>,
        point: &Point3<f64>,
    ) -> Vector3<f64> {
        let joint_world = link_world * self.origin;
        let axis_world = joint_world.rotation * self.axis.into_inner();
        let travel = self.open - self.closed;
        if self.revolute {
            (axis_world * travel).cross(&(point.coords - joint_world.translation.vector))
        } else {
            axis_world * travel
        }
    }
}

/// The parts of a [`Body`] the gradient needs, copied out so the arm's FK can
/// be borrowed mutably while they are in hand: which chain the body rides and,
/// for a gripper finger, its live placer.
#[derive(Clone, Copy)]
struct BodyKinematics {
    placement: Placement,
    finger: Option<Finger>,
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
    /// Separating direction for body `a`; `None` on degenerate core contact.
    normal: Option<Unit<Vector3<f64>>>,
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
/// surface distance with respect to each arm's joints and each gripper's opening
/// fraction. `grad_left[j]` is `d(distance)/d(q_left[j])`; `grad_openings[s]` is
/// `d(distance)/d(opening_s)` for side `s` (0 = left, 1 = right), nonzero only
/// when a finger body carries a witness; separating motion has a positive
/// gradient. Computed analytically from the nearest pair's witness points (the
/// gradient of the active pair, by the envelope theorem), so it costs one
/// distance query plus two point Jacobians.
#[derive(Debug, Clone)]
pub struct DistanceGradient<'a> {
    pub proximity: Proximity<'a>,
    pub grad_left: JointVec,
    pub grad_right: JointVec,
    pub grad_openings: [f64; 2],
}

/// One hull piece placed in the world: the vertices, the face triangles
/// indexing them, and any `radius` swept around that core. Enough to draw the
/// exact collision surface: the offset faces, plus edge and vertex fillets when
/// the radius is nonzero. A circumscribing fit needs no sweep to contain its
/// mesh, so its pieces report zero. Runtime queries never materialize this.
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
    /// Gripper opening per side (0 = left, 1 = right) as a fraction in `[0, 1]`
    /// (0 = fully closed, 1 = fully open), set by
    /// [`set_gripper_openings`](Self::set_gripper_openings). Finger bodies are
    /// placed at this opening every query. Defaults to fully open, the widest
    /// outboard envelope; it is not a full substitute for the real opening
    /// (closed fingers occupy between-jaws space the open placement vacates),
    /// so a caller governing near closed jaws must feed the measured opening
    /// before trusting the clearance.
    openings: [f64; 2],
    /// Per-side, per-joint Lipschitz levers of the min surface distance (m/rad):
    /// the max over checked pairs of the sum of both bodies' per-joint reach
    /// bounds on that side (a same-side pair moves both witnesses with one
    /// arm's joints; see [`body_reaches`]). Rebuilt whenever the pair list
    /// changes; feeds [`clearance_step_bound`](Self::clearance_step_bound).
    levers: [JointVec; 2],
    /// Per-side Lipschitz levers of the min surface distance per unit opening
    /// fraction (m): the max over checked pairs of the paired finger bodies'
    /// surface speed bounds under their own joint travel. Rebuilt with `levers`;
    /// feeds [`clearance_step_bound`](Self::clearance_step_bound).
    opening_levers: [f64; 2],
    /// How many of `bodies` came from the URDF. Runtime obstacles occupy
    /// `bodies[urdf_body_count..]`: [`add_obstacle`](BimanualCollisionModel::add_obstacle)
    /// only appends and
    /// [`remove_obstacle`](BimanualCollisionModel::remove_obstacle) only removes
    /// from that tail, so "is an obstacle" is the index test
    /// `i >= urdf_body_count` and needs no per-body flag.
    urdf_body_count: usize,
    /// How many of `pairs` were derived at build, exclusions already applied.
    /// Obstacle pairs are appended after them and are rebuilt wholesale by
    /// [`rederive_obstacle_pairs`](BimanualCollisionModel::rederive_obstacle_pairs),
    /// which is what keeps a build-time exclusion from being re-derived by an
    /// insertion.
    urdf_pair_count: usize,
}

/// A world-frame convex obstacle, fitted and ready to insert into a model with
/// [`BimanualCollisionModel::add_obstacle`]. Opaque and only constructible
/// through [`Obstacle::fit`], so a cloud that bounds no solid cannot reach a
/// model. Fitting is the expensive half of an insertion, and this type is what
/// lets a control-loop caller pay it off the loop and insert on it.
#[derive(Debug)]
pub struct Obstacle {
    name: String,
    hulls: Vec<Hull>,
}

/// Tightest fit an obstacle may ask for (m). Below the link standard there is
/// nothing to gain: the hull of a flat-faced body is exact at any budget, and a
/// curved one runs the plane count up long before this matters.
pub const MIN_OBSTACLE_TOLERANCE_M: f64 = 1e-4;

/// Loosest fit an obstacle may ask for (m). A looser fit is conservative, since
/// the hull still contains the cloud, so this is not a safety bound: it is the
/// point past which the hull stops describing the object and starts eating
/// workspace the arms need. Ten centimetres carries a curved body of any size a
/// workspace holds, by the hundredth-of-the-radius rule the fit follows.
pub const MAX_OBSTACLE_TOLERANCE_M: f64 = 0.1;

/// Thinnest an obstacle may be (m), measured on its axis-aligned bounding box.
/// Under the fit's own deviation budget a body is thinner than the error it
/// would be fitted with. Nothing downstream refuses such a cloud: the fit
/// accepts a nanometre-thick slab and returns a hull no thicker, so this bound
/// is the only thing standing between one and a contact query with no
/// separating direction.
const MIN_OBSTACLE_EXTENT_M: f64 = crate::assemble::MAX_DEVIATION_M;

/// Widest an obstacle may be (m), measured the same way. The hull kernel's
/// degeneracy tests are absolute, so a cloud kilometres across stops reading as
/// a solid; refusing it here says so, where letting it through reports a 20 km
/// box as "coplanar cloud has no volume".
const MAX_OBSTACLE_EXTENT_M: f64 = 1_000.0;

impl Obstacle {
    /// Fit a convex obstacle to a world-frame point cloud, under the same
    /// deviation budget as the URDF bodies.
    ///
    /// `tolerance_m` is how far the fitted hull may stand off the cloud, and it
    /// is the caller's cost lever. The fit adds supporting planes until it holds,
    /// every plane is a per-tick cost once the obstacle is live, and a curved
    /// body needs roughly a hundredth of its radius: a 20 cm ball wants 2 mm, a
    /// 1 m ball 10 mm. A flat-faced body (a wall, a table, a box) hulls to its
    /// corners at any tolerance and pays nothing for a tight one. Asking for
    /// tighter than the shape allows errors rather than silently costing the
    /// control loop, and looser is always the conservative direction, since the
    /// hull contains the cloud either way.
    ///
    /// Errors on an empty name, a tolerance outside
    /// [`MIN_OBSTACLE_TOLERANCE_M`] to [`MAX_OBSTACLE_TOLERANCE_M`], a
    /// non-finite coordinate, a cloud that bounds no solid (empty, collinear,
    /// coplanar), or one whose bounding box is outside
    /// [`MIN_OBSTACLE_EXTENT_M`] to [`MAX_OBSTACLE_EXTENT_M`] on any axis.
    ///
    /// That last reads the bounding box, so it bounds an axis-aligned body
    /// exactly and a tilted one loosely: a tilted sliver has a fat bounding box
    /// and passes, and the fit will take it, so a caller who cannot rule those
    /// out should measure the hull it gets back. Within that limit it is doing
    /// real work, not just phrasing: an over-large cloud would otherwise be
    /// refused as "coplanar", and a paper-thin one would not be refused at
    /// all.
    pub fn fit(
        name: &str,
        points: &[Point3<f64>],
        tolerance_m: f64,
    ) -> Result<Self, CollisionError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(BuildError::UnnamedObstacle.into());
        }
        if !(tolerance_m.is_finite()
            && (MIN_OBSTACLE_TOLERANCE_M..=MAX_OBSTACLE_TOLERANCE_M).contains(&tolerance_m))
        {
            return Err(BuildError::ToleranceOutOfRange {
                body: name.to_string(),
                tolerance_m,
                min: MIN_OBSTACLE_TOLERANCE_M,
                max: MAX_OBSTACLE_TOLERANCE_M,
            }
            .into());
        }
        if points
            .iter()
            .flat_map(|p| p.coords.iter())
            .any(|c| !c.is_finite())
        {
            return Err(CollisionError::NonFinite);
        }
        check_extents(name, points)?;
        Ok(Obstacle {
            name: name.to_string(),
            hulls: vec![crate::assemble::fit_one_hull(name, points, tolerance_m)?],
        })
    }

    /// The name it will answer to once inserted.
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Configures and builds a [`BimanualCollisionModel`]; start from
/// [`BimanualCollisionModel::builder`].
pub struct Builder {
    urdf: String,
    meshes_dir: String,
    left_base: String,
    right_base: String,
    exclude: Vec<PairSpec>,
    supplied: HashMap<String, Vec<ClipRegion>>,
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

    /// Decompose a body into clip regions, replacing its auto-fit single hull.
    /// Each region's slice of the mesh gets the same rounded simplified-hull fit
    /// a link gets, so a concave body (a torso) is bound as tightly as the links
    /// are, piece by piece. The regions must jointly cover the body's mesh,
    /// checked at build; see [`ClipRegion`] for the overlap and bound-placement
    /// rules. Naming a body that does not exist errors at build.
    pub fn regions(mut self, body: &str, regions: Vec<ClipRegion>) -> Self {
        self.supplied.insert(body.to_string(), regions);
        self
    }

    /// Fit the bodies (supplied regions override the auto-fit), derive the checked
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

    /// Fit every collision body (supplied regions override the auto-fit) and place
    /// them, with no checked pairs set yet.
    fn assemble(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        supplied: &HashMap<String, Vec<ClipRegion>>,
    ) -> Result<Self, BuildError> {
        if left_base == right_base {
            return Err(BuildError::IdenticalBases {
                base: left_base.to_string(),
            });
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
                    finger: None,
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
                        finger: None,
                    },
                )?;
            }
        }
        // Finger bodies hang off their host chain link's FK segment (so they share
        // its lineage: not checked against their own hand or sibling finger, but
        // checked cross-arm and against the torso) and carry a `Finger` placer so
        // each query positions them at the live opening.
        for finger in fitted.fingers {
            let placement = chain_segment(&left_names, &right_names, &finger.parent_link)
                .expect("fit_bodies only emits fingers hosted on a chain link");
            let placer =
                Finger::from_joint(&finger.name, &finger.joint, finger.closed, finger.open)?;
            let bound = BoundingSphere::of(&finger.hulls);
            push_body(
                &mut bodies,
                Body {
                    name: finger.name,
                    local: finger.hulls,
                    placement,
                    bound,
                    finger: Some(placer),
                },
            )?;
        }

        let world_iso = vec![Isometry3::identity(); bodies.len()];
        Ok(Self {
            urdf_body_count: bodies.len(),
            urdf_pair_count: 0,
            left,
            right,
            bodies,
            pairs: Vec::new(),
            excluded: Vec::new(),
            world_iso,
            openings: [1.0, 1.0],
            levers: [[0.0; ARM_DOF]; 2],
            opening_levers: [0.0; 2],
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
        self.record_urdf_pairs();
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
            .ok_or_else(|| BuildError::UnknownBody {
                name: name.to_string(),
            })
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
        self.record_urdf_pairs();
        Ok(())
    }

    /// Rebuild the per-side Lipschitz levers from the checked pairs: for each
    /// side and joint, the max over pairs of the sum of both bodies' reaches on
    /// that side. Summing per pair is what keeps the bound sound for a same-side
    /// pair, whose two witnesses both move with that arm's joints. Called
    /// whenever the pair list changes (at build, and on every obstacle change;
    /// recomputing the
    /// per-body reaches costs two forward-kinematics passes, which an obstacle
    /// insertion pays once, never per tick).
    fn recompute_levers(&mut self) {
        let reaches = body_reaches(&mut self.left, &mut self.right, &self.bodies);
        let side_reach = |body: usize, side: usize, j: usize| -> f64 {
            let on_side = match self.bodies[body].placement {
                Placement::Left(_) => side == 0,
                Placement::Right(_) => side == 1,
                Placement::Fixed => false,
            };
            if on_side { reaches[body][j] } else { 0.0 }
        };
        self.levers = std::array::from_fn(|side| {
            std::array::from_fn(|j| {
                self.pairs
                    .iter()
                    .map(|p| side_reach(p.a, side, j) + side_reach(p.b, side, j))
                    .fold(0.0, f64::max)
            })
        });
        // A finger body's surface moves at most `opening_reach` metres per unit
        // opening fraction: the full joint travel, times (revolute only) the
        // farthest hull point's distance from the joint axis (the axis passes
        // through the finger frame origin along `axis`, so the lever arm of a
        // local point is its component perpendicular to the axis) plus the
        // inflation radius. Non-finger bodies do not move with an opening.
        let opening_reach = |body: &Body| -> f64 {
            let Some(f) = &body.finger else { return 0.0 };
            let travel = (f.open - f.closed).abs();
            if !f.revolute {
                return travel;
            }
            let r_max = body
                .local
                .iter()
                .flat_map(|h| {
                    let radius = h.inflation();
                    h.vertices().iter().map(move |p| {
                        let along = p.coords.dot(&f.axis);
                        (p.coords - f.axis.into_inner() * along).norm() + radius
                    })
                })
                .fold(0.0, f64::max);
            travel * r_max
        };
        // Precomputed per body, as the joint reaches above are: the scan over a
        // body's hull vertices is the expensive part, and the pair fold below
        // would otherwise repeat it once per pair the body appears in.
        let opening_reaches: Vec<f64> = self.bodies.iter().map(opening_reach).collect();
        let side_opening_reach = |body: usize, side: usize| -> f64 {
            let on_side = match self.bodies[body].placement {
                Placement::Left(_) => side == 0,
                Placement::Right(_) => side == 1,
                Placement::Fixed => false,
            };
            if on_side { opening_reaches[body] } else { 0.0 }
        };
        self.opening_levers = std::array::from_fn(|side| {
            self.pairs
                .iter()
                .map(|p| side_opening_reach(p.a, side) + side_opening_reach(p.b, side))
                .fold(0.0, f64::max)
        });
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

    /// Insert a fitted world-frame obstacle, checked against every moving body
    /// (both chains' links and their gripper fingers) and against nothing else:
    /// the torso, the chain mounts, and other obstacles are all world-fixed, so
    /// their distance to it never changes and could not inform a caller. Errors
    /// if the name is already a body's.
    ///
    /// The pairs derived at build are left untouched, so an exclusion the
    /// builder was given survives every insertion and removal.
    pub fn add_obstacle(&mut self, obstacle: Obstacle) -> Result<(), CollisionError> {
        let bound = BoundingSphere::of(&obstacle.hulls);
        push_body(
            &mut self.bodies,
            Body {
                name: obstacle.name,
                local: obstacle.hulls,
                placement: Placement::Fixed,
                bound,
                finger: None,
            },
        )?;
        self.world_iso.push(Isometry3::identity());
        self.rederive_obstacle_pairs();
        Ok(())
    }

    /// Remove an obstacle added by [`add_obstacle`](Self::add_obstacle), with
    /// its checked pairs. Errors on a name that is not a live obstacle's; a URDF
    /// body is not removable.
    pub fn remove_obstacle(&mut self, name: &str) -> Result<(), CollisionError> {
        let index = self.obstacle_index(name)?;
        self.bodies.remove(index);
        self.world_iso.remove(index);
        self.rederive_obstacle_pairs();
        Ok(())
    }

    /// Remove every obstacle, returning how many there were.
    pub fn clear_obstacles(&mut self) -> usize {
        let removed = self.obstacle_count();
        self.bodies.truncate(self.urdf_body_count);
        self.world_iso.truncate(self.urdf_body_count);
        self.rederive_obstacle_pairs();
        removed
    }

    /// Record the pair list as the URDF's own and refresh the levers. Called
    /// by the two build-time mutators, whose result is only the URDF pair set
    /// while no obstacle has been added yet.
    fn record_urdf_pairs(&mut self) {
        debug_assert_eq!(
            self.bodies.len(),
            self.urdf_body_count,
            "the build-time pair set is being recorded with obstacles present"
        );
        self.urdf_pair_count = self.pairs.len();
        self.recompute_levers();
    }

    /// Rebuild every obstacle pair from the bodies, after any change to the
    /// obstacle set. The obstacle pairs are wholly derived (each obstacle
    /// against each moving body, with no structural rules and no exclusions to
    /// respect), so deriving them again is both simpler and safer than editing
    /// the pair list around a change: the URDF pairs below `urdf_pair_count`,
    /// exclusions included, are never touched, and no body index can outlive
    /// the body it pointed at.
    fn rederive_obstacle_pairs(&mut self) {
        self.pairs.truncate(self.urdf_pair_count);
        let moving: Vec<usize> = self.moving_bodies().collect();
        for obstacle in self.urdf_body_count..self.bodies.len() {
            self.pairs
                .extend(moving.iter().map(|&i| Pair { a: i, b: obstacle }));
        }
        self.recompute_levers();
    }

    /// Every body the arms carry, by index: the bodies an obstacle is checked
    /// against, and the only ones whose pose a query has to place.
    fn moving_bodies(&self) -> impl Iterator<Item = usize> + '_ {
        (0..self.bodies.len()).filter(|&i| !matches!(self.bodies[i].placement, Placement::Fixed))
    }

    /// How many obstacles are in force, without naming them.
    pub fn obstacle_count(&self) -> usize {
        self.bodies.len() - self.urdf_body_count
    }

    /// The live obstacles' names, in insertion order.
    pub fn obstacle_names(&self) -> Vec<&str> {
        self.bodies[self.urdf_body_count..]
            .iter()
            .map(|b| b.name.as_str())
            .collect()
    }

    /// Signed surface distance from one obstacle to its nearest moving body at
    /// the given configurations, ignoring every pair the obstacle is not in.
    /// This is what weighs an obstacle against the *robot's* clearance to it
    /// rather than against a self-collision pair that happens to be closer.
    /// Errors on an unknown obstacle or a non-finite configuration.
    pub fn obstacle_clearance(
        &mut self,
        name: &str,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<f64, CollisionError> {
        ensure_finite(q_left, q_right)?;
        let index = self.obstacle_index(name)?;
        Ok(self
            .closest_over(q_left, q_right, |p| p.a == index || p.b == index)?
            .distance)
    }

    /// Resolve an obstacle's name to its body index. A URDF body's name does not
    /// resolve: only obstacles are removable or separately measurable.
    fn obstacle_index(&self, name: &str) -> Result<usize, CollisionError> {
        self.bodies[self.urdf_body_count..]
            .iter()
            .position(|b| b.name == name)
            .map(|i| i + self.urdf_body_count)
            .ok_or_else(|| CollisionError::UnknownObstacle {
                name: name.to_string(),
            })
    }

    /// The nearest checked pair at the given configurations: places the hulls by
    /// FK, then scans the pairs (broadphase-ordered) for the minimum signed
    /// distance. The shared core of [`min_distance`](Self::min_distance) and
    /// [`distance_gradient`](Self::distance_gradient).
    fn closest(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<Closest, CollisionError> {
        self.closest_over(q_left, q_right, |_| true)
    }

    /// [`closest`](Self::closest) over the subset of checked pairs `keep`
    /// admits, so a caller can ask about one body's clearance under the same
    /// broadphase and prefilter as the full scan.
    fn closest_over(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
        keep: impl Fn(&Pair) -> bool,
    ) -> Result<Closest, CollisionError> {
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
            .filter(|(_, p)| keep(p))
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
            let (iso_a, iso_b) = (self.world_iso[pair.a], self.world_iso[pair.b]);
            // The transformed piece centres of body b are constant across body
            // a's pieces; place them once per pair, not once per (ha, hb).
            let centers_b: Vec<Point3<f64>> = self.bodies[pair.b]
                .local
                .iter()
                .map(|hb| iso_b * hb.bound_center())
                .collect();
            for ha in &self.bodies[pair.a].local {
                let center_a = iso_a * ha.bound_center();
                for (hb, center_b) in self.bodies[pair.b].local.iter().zip(&centers_b) {
                    // Piece-level prefilter, same sphere bound as the pair
                    // broadphase: a piece pair that cannot beat the best
                    // distance skips its GJK. A multi-piece body (the torso's
                    // region decomposition) otherwise pays one GJK per piece
                    // for pieces nowhere near the query.
                    let gap = (center_a - center_b).norm() - ha.bound_radius() - hb.bound_radius();
                    if best.as_ref().is_some_and(|c| gap > c.distance) {
                        continue;
                    }
                    let r = gjk::distance(&Placed::new(ha, iso_a), &Placed::new(hb, iso_b));
                    if best.as_ref().is_none_or(|c| r.distance < c.distance) {
                        best = Some(Closest {
                            distance: r.distance,
                            a: pair.a,
                            b: pair.b,
                            on_a: r.on_a,
                            on_b: r.on_b,
                            normal: r.normal,
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
    /// is the nearest pair's separating direction projected through each witness
    /// point's velocity Jacobian, so it reflects the same min-over-pairs distance
    /// `min_distance` returns at one distance query's cost. Fails on a non-finite
    /// configuration, or on degenerate core contact (the rounded surfaces then
    /// overlap by the summed radii) where no separating direction is defined; a
    /// velocity-barrier caller holds there.
    pub fn distance_gradient(
        &mut self,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> Result<DistanceGradient<'_>, CollisionError> {
        ensure_finite(q_left, q_right)?;
        let c = self.closest(q_left, q_right)?;
        // GJK carries the separating direction explicitly (the witness difference
        // reverses sense with the sign of the distance); the +1/-1 signs below
        // sum the projected witness velocities to d(distance)/dq.
        let Some(normal) = c.normal.map(Unit::into_inner) else {
            return Err(CollisionError::WitnessesCoincide {
                distance: c.distance,
            });
        };
        let kinematics = |body: &Body| BodyKinematics {
            placement: body.placement,
            finger: body.finger,
        };
        let (kin_a, kin_b) = (kinematics(&self.bodies[c.a]), kinematics(&self.bodies[c.b]));
        let (left_a, right_a, open_a) =
            self.gradient_contribution(kin_a, &c.on_a, &normal, 1.0, q_left, q_right);
        let (left_b, right_b, open_b) =
            self.gradient_contribution(kin_b, &c.on_b, &normal, -1.0, q_left, q_right);
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
            grad_openings: [open_a[0] + open_b[0], open_a[1] + open_b[1]],
        })
    }

    /// One body's contribution to the distance gradient: the pair's separating
    /// direction `normal` projected through the witness `point`'s velocity
    /// Jacobian, plus, for a finger body, through the point's velocity per unit
    /// opening fraction. `sign` is +1 for body a and -1 for body b (`normal`
    /// increases the distance for a, decreases it for b). A world-fixed body
    /// (torso) contributes nothing.
    fn gradient_contribution(
        &mut self,
        kinematics: BodyKinematics,
        point: &Point3<f64>,
        normal: &Vector3<f64>,
        sign: f64,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> (JointVec, JointVec, [f64; 2]) {
        let BodyKinematics { placement, finger } = kinematics;
        let zero: JointVec = [0.0; ARM_DOF];
        let contribution = |arm: &mut Arm, q: &JointVec, segment: usize| -> (JointVec, f64) {
            let posed = arm.at(q);
            let cols = posed.point_world_jacobian(point, segment);
            let joints = std::array::from_fn(|j| sign * normal.dot(&cols[j]));
            let opening = finger.map_or(0.0, |f| {
                let v = f.point_velocity_per_fraction(&posed.link_pose_world(segment), point);
                sign * normal.dot(&v)
            });
            (joints, opening)
        };
        match placement {
            Placement::Fixed => (zero, zero, [0.0; 2]),
            Placement::Left(s) => {
                let (joints, opening) = contribution(&mut self.left, q_left, s);
                (joints, zero, [opening, 0.0])
            }
            Placement::Right(s) => {
                let (joints, opening) = contribution(&mut self.right, q_right, s);
                (zero, joints, [0.0, opening])
            }
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

    /// Set the gripper opening per side as a fraction in `[0, 1]` (0 = fully
    /// closed, 1 = fully open); values are clamped. Finger bodies are placed at
    /// this opening on every subsequent query, so the reported clearance follows
    /// the fingers' true positions instead of their full swept envelope. A
    /// non-finite value is ignored for that side (the last good opening stands),
    /// so a bad reading never poisons the placement.
    pub fn set_gripper_openings(&mut self, left: f64, right: f64) {
        if left.is_finite() {
            self.openings[0] = left.clamp(0.0, 1.0);
        }
        if right.is_finite() {
            self.openings[1] = right.clamp(0.0, 1.0);
        }
    }

    /// Upper bound (m) on how much the minimum surface distance can change over a
    /// step of `dq_left` / `dq_right` on the arm joints and `dopenings` on the
    /// gripper opening fractions, valid along the whole straight segment in that
    /// combined space: `sum_j levers[side][j] * |dq[j]|` over both arms plus
    /// `opening_levers[side] * |dopenings[side]|` over both grippers. Each lever
    /// bounds, over all poses and openings, the worst per-unit closing rate of
    /// any checked pair (summing both witnesses' travel when a pair's bodies
    /// share a side), and a minimum of Lipschitz functions is Lipschitz, so a
    /// segment whose start clearance exceeds a floor by more than this bound
    /// cannot cross that floor anywhere along the step. Deliberately loose
    /// (chain-length bounds), so it is sound for a caller skipping an exact scan,
    /// never tight.
    pub fn clearance_step_bound(
        &self,
        dq_left: &JointVec,
        dq_right: &JointVec,
        dopenings: &[f64; 2],
    ) -> f64 {
        let dot_abs = |lever: &JointVec, dq: &JointVec| -> f64 {
            lever.iter().zip(dq).map(|(l, d)| l * d.abs()).sum::<f64>()
        };
        let bound = dot_abs(&self.levers[0], dq_left)
            + dot_abs(&self.levers[1], dq_right)
            + self.opening_levers[0] * dopenings[0].abs()
            + self.opening_levers[1] * dopenings[1].abs();
        // A non-finite delta gives no finite bound: return infinity so a caller's
        // skip predicate (`margin > bound`) can never pass on bad data, rather
        // than a NaN whose comparison direction the caller must not rely on.
        if bound.is_finite() {
            bound
        } else {
            f64::INFINITY
        }
    }

    /// Refresh the world pose of the moving bodies from FK. Finger bodies are
    /// additionally offset by their host link pose at the side's current opening.
    fn place(&mut self, q_left: &JointVec, q_right: &JointVec) {
        // The `zip` below would drop a tail of bodies rather than place them,
        // and the broadphase would then index a `world_iso` that no longer has
        // a row for them.
        debug_assert_eq!(self.bodies.len(), self.world_iso.len());
        let poses_l = link_poses(&mut self.left, q_left);
        let poses_r = link_poses(&mut self.right, q_right);
        let openings = self.openings;
        for (body, iso) in self.bodies.iter().zip(self.world_iso.iter_mut()) {
            *iso = match body.placement {
                Placement::Fixed => continue,
                Placement::Left(i) => body.place_on(poses_l[i], openings[0]),
                Placement::Right(i) => body.place_on(poses_r[i], openings[1]),
            };
        }
    }
}

/// The [`Placement`] of a body hanging off chain link `parent`: `Left`/`Right`
/// with the link's FK segment index, or `None` if `parent` is not a chain link.
fn chain_segment(left: &[String], right: &[String], parent: &str) -> Option<Placement> {
    if let Some(i) = left.iter().position(|n| n == parent) {
        return Some(Placement::Left(i));
    }
    right.iter().position(|n| n == parent).map(Placement::Right)
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

/// Per-body, per-joint surface-speed bounds (m/rad). A point rigidly attached
/// distal of revolute joint `j` moves at most `r * |dq_j|`, with `r` its distance
/// from the joint axis; that distance is bounded, over all poses, by the chain
/// hops from joint `j`'s origin out to the body's link origin (each hop's norm is
/// pose invariant: a rigid link separates consecutive joint origins) plus the
/// body's own reach: bounding-sphere centre offset + radius, and for a finger the
/// worst translation its joint offset can add across the travel (a prismatic
/// slide adds the full travel; a revolute offset only rotates about its origin).
/// Rows are zero for fixed bodies and for joints distal of the body.
fn body_reaches(left: &mut Arm, right: &mut Arm, bodies: &[Body]) -> Vec<JointVec> {
    let home = [0.0; ARM_DOF];
    let hops = |arm: &mut Arm| -> [f64; ARM_DOF - 1] {
        let poses = link_poses(arm, &home);
        std::array::from_fn(|k| {
            (poses[k + 1].translation.vector - poses[k].translation.vector).norm()
        })
    };
    let hops = [hops(left), hops(right)];

    bodies
        .iter()
        .map(|body| {
            let (side, seg) = match body.placement {
                Placement::Fixed => return [0.0; ARM_DOF], // never moved by a joint
                Placement::Left(s) => (0, s),
                Placement::Right(s) => (1, s),
            };
            let finger_reach = body.finger.as_ref().map_or(0.0, |f| {
                let travel = if f.revolute {
                    0.0
                } else {
                    f.closed.abs().max(f.open.abs())
                };
                f.origin.translation.vector.norm() + travel
            });
            let reach = finger_reach + body.bound.center.coords.norm() + body.bound.radius;
            std::array::from_fn(|j| {
                if j <= seg {
                    hops[side][j..seg].iter().sum::<f64>() + reach
                } else {
                    0.0 // a joint distal of the body does not move it
                }
            })
        })
        .collect()
}

/// Reject a cloud whose bounding box is the wrong scale to be a workspace
/// obstacle, naming the span measured, so a caller is told what it sent rather
/// than what the hull kernel made of it. An empty cloud is the fit's own
/// refusal to make.
fn check_extents(name: &str, points: &[Point3<f64>]) -> Result<(), BuildError> {
    let Some(first) = points.first() else {
        return Ok(());
    };
    let (min, max) = points.iter().fold((*first, *first), |(lo, hi), p| {
        (
            Point3::from(lo.coords.inf(&p.coords)),
            Point3::from(hi.coords.sup(&p.coords)),
        )
    });
    let spans = max - min;
    let degenerate = |reason: String| BuildError::DegenerateBody {
        body: name.to_string(),
        reason,
    };
    if spans.min() < MIN_OBSTACLE_EXTENT_M {
        return Err(degenerate(format!(
            "it is {:.2e} m thick, under the {MIN_OBSTACLE_EXTENT_M} m minimum",
            spans.min()
        )));
    }
    if spans.max() > MAX_OBSTACLE_EXTENT_M {
        return Err(degenerate(format!(
            "it spans {:.2e} m, over the {MAX_OBSTACLE_EXTENT_M} m maximum (check the units)",
            spans.max()
        )));
    }
    Ok(())
}

/// Append a body, refusing a name another body already answers to. The one
/// gate for that, so a name is unique across the URDF bodies and the runtime
/// obstacles alike.
fn push_body(bodies: &mut Vec<Body>, body: Body) -> Result<(), BuildError> {
    if bodies.iter().any(|b| b.name == body.name) {
        return Err(BuildError::DuplicateBody { name: body.name });
    }
    bodies.push(body);
    Ok(())
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
    use crate::assemble::MAX_DEVIATION_M;
    use crate::pairs::PairSpec;

    const URDF: &str = include_str!("../tests/fixtures/openarm_v10.urdf");
    const MESHES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes");

    fn model() -> BimanualCollisionModel {
        BimanualCollisionModel::builder(URDF, MESHES, "openarm_left_link0", "openarm_right_link0")
            .build()
            .expect("model")
    }

    const INF: f64 = f64::INFINITY;

    fn region(min: [f64; 3], max: [f64; 3]) -> ClipRegion {
        ClipRegion::new(
            Point3::new(min[0], min[1], min[2]),
            Point3::new(max[0], max[1], max[2]),
        )
        .expect("test region")
    }

    // Two overlapping z-slabs that jointly cover the torso mesh. Just enough to
    // exercise the multi-piece decomposition path; the tuned deployment regions
    // live in the shared tests/fixtures/openarm.rs, exercised by the
    // integration test.
    fn covering_regions() -> Vec<ClipRegion> {
        vec![
            region([-INF, -INF, -INF], [INF, INF, 0.404]),
            region([-INF, -INF, 0.396], [INF, INF, INF]),
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

    /// Check every analytic gradient column (both arms' joints and both opening
    /// fractions) against central differences at one configuration, with the
    /// grippers at mid-travel 0.6 so the finger bodies participate with a
    /// nontrivial live offset (the opening is a constant across each FD
    /// perturbation, exactly as it is across one governor tick).
    fn assert_gradient_matches_finite_difference(
        m: &mut BimanualCollisionModel,
        ql: &JointVec,
        qr: &JointVec,
    ) {
        let h = 1e-5;
        m.set_gripper_openings(0.6, 0.6);
        let grad = m.distance_gradient(ql, qr).expect("gradient defined");
        let (analytic_left, analytic_right) = (grad.grad_left, grad.grad_right);
        let analytic_openings = grad.grad_openings;
        // Opening columns against central differences on the fractions, the
        // same envelope-theorem check as the joints below.
        for s in 0..2 {
            let openings_at = |frac: f64| -> [f64; 2] {
                let mut o = [0.6, 0.6];
                o[s] = frac;
                o
            };
            let probe = |m: &mut BimanualCollisionModel, frac: f64| -> f64 {
                let o = openings_at(frac);
                m.set_gripper_openings(o[0], o[1]);
                let d = m.min_distance(ql, qr).unwrap().distance;
                m.set_gripper_openings(0.6, 0.6);
                d
            };
            let fd = (probe(m, 0.6 + h) - probe(m, 0.6 - h)) / (2.0 * h);
            assert!(
                (analytic_openings[s] - fd).abs() < 3e-3,
                "opening {s}: analytic {} fd {fd}",
                analytic_openings[s]
            );
        }
        for j in 0..ARM_DOF {
            let mut lp = *ql;
            let mut lm = *ql;
            lp[j] += h;
            lm[j] -= h;
            let fd_left = (m.min_distance(&lp, qr).unwrap().distance
                - m.min_distance(&lm, qr).unwrap().distance)
                / (2.0 * h);
            let mut rp = *qr;
            let mut rm = *qr;
            rp[j] += h;
            rm[j] -= h;
            let fd_right = (m.min_distance(ql, &rp).unwrap().distance
                - m.min_distance(ql, &rm).unwrap().distance)
                / (2.0 * h);
            assert!(
                (analytic_left[j] - fd_left).abs() < 3e-3,
                "left j{j}: analytic {} fd {fd_left}",
                analytic_left[j]
            );
            assert!(
                (analytic_right[j] - fd_right).abs() < 3e-3,
                "right j{j}: analytic {} fd {fd_right}",
                analytic_right[j]
            );
        }
    }

    #[test]
    fn distance_gradient_matches_finite_difference_in_penetration() {
        let mut m = model();
        m.set_gripper_openings(0.6, 0.6);
        // Wrists folded inward but ASYMMETRICALLY, so one moving cross-arm pair is
        // unambiguously nearest (a symmetric pose sits on a pair-switch tie where
        // the analytic gradient and a straddling central difference legitimately
        // disagree). Every config penetrates this model's auto-fit torso hull
        // (asserted below), covering deep EPA; the companion test below walks
        // through contact. The last config has a finger body nearest.
        let configs: [(JointVec, JointVec); 4] = [
            (
                [0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0],
                [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0],
            ),
            (
                [0.0, 0.3, 0.95, 0.45, 0.1, 0.0, 0.0],
                [0.0, -0.1, -0.55, 0.4, 0.0, 0.1, 0.0],
            ),
            (
                [0.25, -0.1, 0.6, 0.65, 0.0, 0.2, 0.1],
                [-0.1, 0.05, -0.7, 0.3, 0.0, -0.2, 0.0],
            ),
            (
                [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
                [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
            ),
        ];
        {
            let (ql, qr) = &configs[3];
            let p = m.min_distance(ql, qr).expect("query");
            assert!(
                p.link_a.contains("finger") || p.link_b.contains("finger"),
                "setup: the finger config's nearest pair should involve a finger, got {} vs {}",
                p.link_a,
                p.link_b
            );
        }
        for (ql, qr) in configs {
            let d = m.min_distance(&ql, &qr).expect("query").distance;
            assert!(d < 0.0, "setup: expected a penetrating config, got d={d}");
            assert_gradient_matches_finite_difference(&mut m, &ql, &qr);
        }
    }

    #[test]
    fn distance_gradient_matches_finite_difference_across_contact() {
        // The default model's auto-fit torso hull swallows every reachable pose,
        // so the test above only ever sees deep penetration. Restrict the pairs
        // to cross-arm wrist and finger bodies and walk an asymmetric
        // wrists-inward family from clearance through contact into shallow
        // overlap, the regimes where the witness separation reverses sense.
        let mut cross_pairs = Vec::new();
        for a in ["link6", "link7", "left_finger", "right_finger"] {
            for b in ["link6", "link7", "left_finger", "right_finger"] {
                cross_pairs.push(PairSpec::new(
                    format!("openarm_left_{a}"),
                    format!("openarm_right_{b}"),
                ));
            }
        }
        let mut m = BimanualCollisionModel::with_pairs(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
            &cross_pairs,
        )
        .expect("cross-arm model");
        m.set_gripper_openings(0.6, 0.6);
        let pose_at = |t: f64| -> (JointVec, JointVec) {
            (
                [0.1, 0.05, t, 0.45, -0.1, 0.05, 0.0],
                [-0.05, -0.1, -t - 0.08, 0.4, 0.1, -0.05, 0.0],
            )
        };
        let (mut separated, mut penetrating) = (0, 0);
        let mut finger_pair_covered = false;
        for i in 0..=60 {
            let (ql, qr) = pose_at(i as f64 * 0.02);
            let p = m.min_distance(&ql, &qr).expect("query");
            // Stop before the nearest pair switches bodies (a pair-switch tie
            // would make the central difference straddle two gradients).
            if p.distance <= -0.03 {
                break;
            }
            if p.distance > 0.0 {
                separated += 1;
            } else {
                penetrating += 1;
            }
            finger_pair_covered |= p.link_a.contains("finger") || p.link_b.contains("finger");
            assert_gradient_matches_finite_difference(&mut m, &ql, &qr);
        }
        assert!(
            separated >= 3 && penetrating >= 3 && finger_pair_covered,
            "setup: expected configs on both sides of contact with a finger pair nearest, \
             got {separated} separated / {penetrating} penetrating (finger: {finger_pair_covered})"
        );
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
        assert!(
            matches!(&e, CollisionError::Build(BuildError::IdenticalBases { .. })),
            "{e}"
        );
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
    fn fingers_are_their_own_bodies_not_baked_into_the_wrist() {
        // Each gripper finger is fit as its own single-hull body (in its finger
        // frame); the wrist hull covers only the wrist mesh and fixed children.
        let m = model();
        for finger in [
            "openarm_left_left_finger",
            "openarm_left_right_finger",
            "openarm_right_left_finger",
            "openarm_right_right_finger",
        ] {
            assert_eq!(
                m.local_hulls(finger)
                    .unwrap_or_else(|| panic!("finger body {finger} missing"))
                    .len(),
                1,
                "{finger} should be one auto-fit hull"
            );
        }
    }

    #[test]
    fn finger_pairs_check_across_arms_but_not_own_hand_or_sibling() {
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
        // The whole point: a left finger is checked against the right gripper's
        // fingers and hand, and against the torso, so the arms cannot drive their
        // grippers into each other undetected.
        assert!(has(
            "openarm_left_right_finger",
            "openarm_right_right_finger"
        ));
        assert!(has("openarm_left_right_finger", "openarm_right_link7"));
        assert!(has("openarm_left_right_finger", "openarm_body_link0"));
        // A finger shares its wrist link's lineage, so it is not checked against
        // its own hand or its sibling finger (they touch by construction as the
        // jaws close on an object).
        assert!(!has("openarm_left_right_finger", "openarm_left_link7"));
        assert!(!has(
            "openarm_left_right_finger",
            "openarm_left_left_finger"
        ));
    }

    #[test]
    fn supplied_regions_replace_the_auto_fit() {
        let m = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .regions("openarm_body_link0", covering_regions())
        .build()
        .expect("covering torso regions contain the mesh");
        assert_eq!(
            m.local_hulls("openarm_body_link0").expect("torso").len(),
            2,
            "torso uses the two supplied region pieces"
        );
    }

    #[test]
    fn rejects_regions_that_leave_mesh_uncovered() {
        // Only the lower torso is clipped in; the head vertices escape.
        let lower_only = vec![region([-INF, -INF, -INF], [INF, INF, 0.3])];
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .regions("openarm_body_link0", lower_only)
        .build()
        .err()
        .expect("under-covering regions must be rejected");
        assert!(
            matches!(&e, CollisionError::Build(BuildError::HullMissesMesh { .. })),
            "{e}"
        );
    }

    #[test]
    fn rejects_a_region_that_clips_nothing() {
        // The second region floats above the whole robot: its slice of the mesh
        // is empty, which cannot bound a solid piece.
        let with_empty = vec![
            region([-INF, -INF, -INF], [INF, INF, INF]),
            region([-INF, -INF, 5.0], [INF, INF, 6.0]),
        ];
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .regions("openarm_body_link0", with_empty)
        .build()
        .err()
        .expect("an empty slice must be rejected");
        assert!(
            matches!(
                &e,
                CollisionError::Build(BuildError::DegenerateRegion { index: 1, .. })
            ),
            "{e}"
        );
    }

    #[test]
    fn rejects_supplied_regions_for_unknown_body() {
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .regions("no_such_body", covering_regions())
        .build()
        .err()
        .expect("unknown body must fail");
        assert!(
            matches!(
                &e,
                CollisionError::Build(BuildError::UnknownSuppliedBody { .. })
            ),
            "{e}"
        );
    }

    #[test]
    fn rejects_empty_supplied_regions() {
        let e = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .regions("openarm_body_link0", Vec::new())
        .build()
        .err()
        .expect("an empty region list must fail");
        assert!(
            matches!(&e, CollisionError::Build(BuildError::EmptyRegions { .. })),
            "{e}"
        );
    }

    #[test]
    fn clearance_step_bound_dominates_the_real_change() {
        // The scan-skip soundness contract: over a joint step, the real change in
        // min surface distance never exceeds clearance_step_bound. Sampled across
        // poses (clear, in-band, near-contact), directions, magnitudes, and
        // openings; each segment is also probed at interior points, since the
        // bound must hold along the whole segment, not just at its ends.
        //
        // Run twice: the bound has to dominate obstacle pairs as well as the
        // URDF's own. The slab cuts through a link, so it wins the scan over
        // some of the sampled poses instead of merely being present.
        clearance_step_bound_dominates_on(&mut model(), None);
        let mut with_obstacles = model();
        with_obstacles
            .add_obstacle(overlapping_slab())
            .expect("add slab");
        with_obstacles
            .add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add wall");
        let obstacle_wins = clearance_step_bound_dominates_on(&mut with_obstacles, Some("slab"));
        assert!(
            obstacle_wins > 0,
            "no sampled step had an obstacle as the nearest pair, so the second \
             run checked nothing the first did not"
        );
    }

    /// Walk the sweep, returning how many samples had a body whose name starts
    /// with `watch` as the nearest pair, so a caller can assert the run covered
    /// what it was set up to cover.
    fn clearance_step_bound_dominates_on(
        m: &mut BimanualCollisionModel,
        watch: Option<&str>,
    ) -> usize {
        let mut watched = 0_usize;
        let poses: [(JointVec, JointVec); 3] = [
            (
                [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
            ),
            (
                [0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0],
                [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0],
            ),
            (
                [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
                [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
            ),
        ];
        // A deterministic spread of step directions: single joints, all joints,
        // and mixed-sign combinations, at a small and a large magnitude.
        let dirs: Vec<(JointVec, JointVec)> = {
            let mut d: Vec<(JointVec, JointVec)> = Vec::new();
            for j in 0..ARM_DOF {
                let mut l = [0.0; ARM_DOF];
                l[j] = 1.0;
                d.push((l, [0.0; ARM_DOF]));
                d.push(([0.0; ARM_DOF], l));
            }
            d.push(([1.0; ARM_DOF], [-1.0; ARM_DOF]));
            d.push((
                std::array::from_fn(|i| if i % 2 == 0 { 1.0 } else { -1.0 }),
                std::array::from_fn(|i| if i % 3 == 0 { -1.0 } else { 1.0 }),
            ));
            d
        };
        // Opening deltas ride the same segment: the bound must dominate a step
        // that slides/swings the fingers too, not only the arm joints.
        for (open_l, open_r, dopen) in [
            (1.0, 1.0, [0.0, 0.0]),
            (0.3, 0.8, [0.7, -0.8]),
            (0.0, 0.0, [1.0, 1.0]),
        ] {
            for (ql, qr) in &poses {
                m.set_gripper_openings(open_l, open_r);
                let d0 = m.min_distance(ql, qr).expect("query").distance;
                for (dl, dr) in &dirs {
                    for mag in [0.02, 0.2] {
                        let sl: JointVec = std::array::from_fn(|i| dl[i] * mag);
                        let sr: JointVec = std::array::from_fn(|i| dr[i] * mag);
                        let bound = m.clearance_step_bound(&sl, &sr, &dopen);
                        for t in [0.25, 0.5, 1.0] {
                            let qlt: JointVec = std::array::from_fn(|i| ql[i] + t * sl[i]);
                            let qrt: JointVec = std::array::from_fn(|i| qr[i] + t * sr[i]);
                            m.set_gripper_openings(open_l + t * dopen[0], open_r + t * dopen[1]);
                            let p = m.min_distance(&qlt, &qrt).expect("query");
                            let dt = p.distance;
                            if watch
                                .is_some_and(|w| p.link_a.starts_with(w) || p.link_b.starts_with(w))
                            {
                                watched += 1;
                            }
                            assert!(
                                (dt - d0).abs() <= bound + 1e-9,
                                "step bound violated: |{dt:+.5} - {d0:+.5}| > {bound:.5} \
                                 (mag {mag}, t {t}, dopen {dopen:?})"
                            );
                        }
                        m.set_gripper_openings(open_l, open_r);
                    }
                }
            }
        }
        watched
    }

    #[test]
    fn step_bound_is_infinite_on_non_finite_deltas() {
        // The scan-skip predicate compares `margin > bound`; a NaN bound would
        // make that false by comparison semantics alone, which a caller must not
        // have to rely on. Bad deltas must yield an explicitly infinite bound.
        let m = model();
        let mut dq = [0.0; ARM_DOF];
        dq[2] = f64::NAN;
        assert_eq!(
            m.clearance_step_bound(&dq, &[0.0; ARM_DOF], &[0.0, 0.0]),
            f64::INFINITY
        );
        assert_eq!(
            m.clearance_step_bound(&[0.0; ARM_DOF], &[0.0; ARM_DOF], &[f64::NAN, 0.0]),
            f64::INFINITY
        );
        assert_eq!(
            m.clearance_step_bound(&[0.0; ARM_DOF], &[0.0; ARM_DOF], &[0.0, f64::INFINITY]),
            f64::INFINITY
        );
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
        assert!(
            matches!(&e, CollisionError::Build(BuildError::UnknownBody { .. })),
            "{e}"
        );
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
        // A fitted obstacle is meant to be fitted on one thread and inserted
        // from another; that is the whole reason the two are separate calls.
        assert_send::<Obstacle>();
    }

    /// The eight corners of an axis-aligned box, the cloud an operator sends for
    /// a wall. At the fixture's home pose both arms hang inside |y| < 0.226 and
    /// |x| < 0.05, so a box beyond that clears them by a known margin.
    fn box_points(min: [f64; 3], max: [f64; 3]) -> Vec<Point3<f64>> {
        let mut points = Vec::with_capacity(8);
        for x in [min[0], max[0]] {
            for y in [min[1], max[1]] {
                for z in [min[2], max[2]] {
                    points.push(Point3::new(x, y, z));
                }
            }
        }
        points
    }

    fn obstacle(name: &str, min: [f64; 3], max: [f64; 3]) -> Obstacle {
        Obstacle::fit(name, &box_points(min, max), MAX_DEVIATION_M).expect("box bounds a solid")
    }

    /// The plane the test wall presents to the arms, so the clearance to it is
    /// a distance a test can derive rather than pin.
    const WALL_FACE_Y: f64 = 0.3;
    /// A wall standing off the left arm's outboard side, its inner face on
    /// [`WALL_FACE_Y`] and clear of both arms at home.
    const WALL_MIN: [f64; 3] = [-0.5, WALL_FACE_Y, 0.0];
    const WALL_MAX: [f64; 3] = [0.5, 0.6, 1.0];

    /// The gap from the wall's inner face to the outermost point of any moving
    /// body, read off the placed hulls the query itself scans. The wall is a
    /// slab normal to `y` and well clear of the arms, so the nearest approach
    /// is exactly this span: an independent value to check the pair scan
    /// against, where a pinned float would only restate one fit's output.
    fn wall_gap_from_placed_hulls(m: &mut BimanualCollisionModel) -> f64 {
        let moving: Vec<String> = m
            .moving_bodies()
            .map(|i| m.bodies[i].name.clone())
            .collect();
        let pieces = m.world_pieces(&home(), &home()).expect("pieces");
        // Every fitted piece circumscribes its cloud and carries no sweep, so
        // its vertices are its surface.
        let outermost = pieces
            .iter()
            .filter(|(name, _)| moving.iter().any(|m| m == name))
            .flat_map(|(_, ps)| ps.iter())
            .flat_map(|p| p.vertices.iter().map(|v| v.y))
            .fold(f64::NEG_INFINITY, f64::max);
        WALL_FACE_Y - outermost
    }

    /// A slab through the left arm's link3, so it wins the scan at the poses
    /// these tests use. Clear of the torso (|y| <= 0.095).
    fn overlapping_slab() -> Obstacle {
        obstacle("slab", [-0.5, 0.15, 0.5], [0.5, 0.6, 0.55])
    }

    fn home() -> JointVec {
        [0.0; ARM_DOF]
    }

    #[test]
    fn an_obstacle_measures_its_own_clearance_to_the_arms() {
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        let expected = wall_gap_from_placed_hulls(&mut m);
        let d = m
            .obstacle_clearance("wall", &home(), &home())
            .expect("clearance");
        assert!(
            (d - expected).abs() < 1e-6,
            "wall clearance {d:+.5}, but the outermost moving hull point sits {expected:+.5} from its face"
        );
    }

    #[test]
    fn an_obstacles_clearance_follows_the_gripper_opening() {
        // Finger bodies are placed at the live opening, and the outermost body
        // at home is a finger, so closing the grippers must open the gap.
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        m.set_gripper_openings(1.0, 1.0);
        let open = m
            .obstacle_clearance("wall", &home(), &home())
            .expect("clearance");
        m.set_gripper_openings(0.0, 0.0);
        let closed = m
            .obstacle_clearance("wall", &home(), &home())
            .expect("clearance");
        assert!(
            closed > open + 1e-3,
            "closing the grippers should recover clearance: open {open:+.4}, closed {closed:+.4}"
        );
    }

    #[test]
    fn an_obstacles_clearance_ignores_the_pairs_it_is_not_in() {
        // At home with the grippers open the fixture's own nearest pair is in
        // shallow penetration, so a global minimum would report that instead of
        // the wall the caller asked about.
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        let overall = m.min_distance(&home(), &home()).expect("query").distance;
        let wall = m
            .obstacle_clearance("wall", &home(), &home())
            .expect("clearance");
        assert!(
            overall < 0.0 && wall > 0.0,
            "setup: expected a self-pair nearer than the wall, got overall {overall:+.4} wall {wall:+.4}"
        );
    }

    #[test]
    fn an_obstacle_is_paired_with_every_moving_body_and_nothing_fixed() {
        let mut m = model();
        let moving: Vec<String> = m
            .bodies
            .iter()
            .filter(|b| !matches!(b.placement, Placement::Fixed))
            .map(|b| b.name.clone())
            .collect();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        let mut partners: Vec<&str> = m
            .checked_pairs()
            .into_iter()
            .filter_map(|(a, b)| match (a, b) {
                ("wall", other) | (other, "wall") => Some(other),
                _ => None,
            })
            .collect();
        partners.sort_unstable();
        let mut expected: Vec<&str> = moving.iter().map(String::as_str).collect();
        expected.sort_unstable();
        assert_eq!(partners, expected);
    }

    #[test]
    fn two_obstacles_are_never_paired_with_each_other() {
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add wall");
        m.add_obstacle(obstacle("floor", [-1.0, -1.0, -0.2], [1.0, 1.0, -0.1]))
            .expect("add floor");
        assert!(
            !m.checked_pairs()
                .iter()
                .any(|&(a, b)| matches!((a, b), ("wall", "floor") | ("floor", "wall"))),
            "two world-fixed obstacles cannot inform a caller, so they must not be checked"
        );
    }

    #[test]
    fn an_obstacle_in_the_way_becomes_the_nearest_pair() {
        let mut m = model();
        m.add_obstacle(overlapping_slab()).expect("add");
        let p = m.min_distance(&home(), &home()).expect("query");
        assert!(
            p.link_a == "slab" || p.link_b == "slab",
            "expected the slab to win, got {} vs {} at {:+.4}",
            p.link_a,
            p.link_b,
            p.distance
        );
        assert!(
            p.distance < 0.0,
            "the slab overlaps a link: {:+.4}",
            p.distance
        );
    }

    #[test]
    fn the_gradient_at_an_obstacle_points_away_from_it() {
        // What a velocity barrier needs from an obstacle pair: the obstacle is
        // world-fixed and contributes nothing, so the whole gradient rides the
        // moving witness, and a step along it must open the gap.
        // Built with no URDF pairs, so the nearest pair is unambiguously the
        // obstacle's; the pair derivation is covered on its own above.
        let mut m = build(&[]).expect("model");
        // A box in front of the left wrist, which swings in x under the shoulder.
        m.add_obstacle(obstacle("wall", [0.12, 0.10, 0.15], [0.5, 0.30, 0.30]))
            .expect("add");
        let (ql, qr) = (home(), home());
        let g = m.distance_gradient(&ql, &qr).expect("gradient defined");
        assert!(
            g.proximity.link_a == "wall" || g.proximity.link_b == "wall",
            "setup: expected the wall to be the nearest pair, got {} vs {}",
            g.proximity.link_a,
            g.proximity.link_b
        );
        assert!(
            g.grad_left.iter().any(|c| c.abs() > 1e-6),
            "setup: the left arm must be able to move relative to this wall"
        );
        let before = g.proximity.distance;
        let grad_left = g.grad_left;
        let stepped: JointVec = std::array::from_fn(|j| ql[j] + 1e-3 * grad_left[j]);
        let after = m.min_distance(&stepped, &qr).expect("query").distance;
        assert!(
            after > before,
            "a step along the gradient must open the gap: {before:+.5} -> {after:+.5}"
        );
    }

    /// The twelve triangles of an axis-aligned box, as an STL would carry it.
    /// A partial surface would hull to a strictly smaller solid, which is the
    /// thing the comparison below has to be able to catch.
    fn box_stl_bytes(min: [f64; 3], max: [f64; 3]) -> Vec<u8> {
        let corner = |i: usize| {
            let pick = |axis: usize| {
                if i >> axis & 1 == 0 {
                    min[axis]
                } else {
                    max[axis]
                }
            };
            [pick(0) as f32, pick(1) as f32, pick(2) as f32]
        };
        // Each face as two triangles, corners indexed by their x/y/z bits.
        let faces = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let triangles: Vec<[[f32; 3]; 3]> = faces
            .iter()
            .flat_map(|f| {
                [
                    [corner(f[0]), corner(f[1]), corner(f[2])],
                    [corner(f[0]), corner(f[2]), corner(f[3])],
                ]
            })
            .collect();
        crate::stl::stl_bytes(&triangles)
    }

    #[test]
    fn an_obstacle_fitted_from_stl_bytes_matches_one_fitted_from_its_points() {
        let bytes = box_stl_bytes(WALL_MIN, WALL_MAX);
        let points = crate::parse_binary_stl(&bytes).expect("parse");
        let mut from_stl = model();
        from_stl
            .add_obstacle(Obstacle::fit("wall", &points, MAX_DEVIATION_M).expect("fit"))
            .expect("add");
        let mut from_points = model();
        from_points
            .add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        // Compared away from home too: at home the witness lands on one face,
        // where a smaller solid would still measure the same.
        for (ql, qr) in [
            (home(), home()),
            ([0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0], home()),
            ([0.0, 0.4, 0.0, 0.9, 0.0, 0.0, 0.0], home()),
        ] {
            let stl = from_stl
                .obstacle_clearance("wall", &ql, &qr)
                .expect("clearance");
            let hull = from_points
                .obstacle_clearance("wall", &ql, &qr)
                .expect("clearance");
            assert!(
                (stl - hull).abs() < 1e-6,
                "the same box through two readers must fit the same solid: \
                 stl {stl:+.5}, points {hull:+.5}"
            );
        }
    }

    /// Points spread over a sphere of radius `r`, deterministically.
    fn sphere_cloud(r: f64, n: usize) -> Vec<Point3<f64>> {
        let mut state = 0x5eed_u64;
        let mut unit = || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((state >> 11) as f64) / ((1u64 << 53) as f64)
        };
        (0..n)
            .map(|_| {
                let z = 2.0 * unit() - 1.0;
                let theta = 2.0 * std::f64::consts::PI * unit();
                let band = (1.0 - z * z).sqrt();
                Point3::new(r * band * theta.cos(), r * band * theta.sin(), r * z)
            })
            .collect()
    }

    /// The rim points of a `sides`-gon prism: a pillar, curved one way only.
    fn cylinder_cloud(r: f64, height: f64, sides: usize) -> Vec<Point3<f64>> {
        (0..sides)
            .flat_map(|i| {
                let a = 2.0 * std::f64::consts::PI * i as f64 / sides as f64;
                let (x, y) = (r * a.cos(), r * a.sin());
                [Point3::new(x, y, 0.0), Point3::new(x, y, height)]
            })
            .collect()
    }

    #[test]
    fn a_tolerance_outside_the_band_is_refused() {
        let wall = box_points(WALL_MIN, WALL_MAX);
        for bad in [
            0.0,
            -0.001,
            f64::NAN,
            f64::INFINITY,
            MIN_OBSTACLE_TOLERANCE_M / 2.0,
            MAX_OBSTACLE_TOLERANCE_M * 2.0,
        ] {
            let err = Obstacle::fit("wall", &wall, bad)
                .expect_err("a tolerance outside the band is refused");
            assert!(
                matches!(
                    err,
                    CollisionError::Build(BuildError::ToleranceOutOfRange { .. })
                ),
                "tolerance {bad} refused for the wrong reason: {err}"
            );
        }
        for good in [MIN_OBSTACLE_TOLERANCE_M, 0.001, MAX_OBSTACLE_TOLERANCE_M] {
            Obstacle::fit("wall", &wall, good).expect("a tolerance in the band fits");
        }
    }

    #[test]
    fn a_looser_tolerance_buys_a_cheaper_hull() {
        // The whole point of the parameter: the caller trades precision for
        // per-tick cost, and cost is the hull's vertex count.
        let ball = sphere_cloud(0.1, 1000);
        let verts = |tolerance: f64| -> usize {
            let mut m = build(&[]).expect("model");
            m.add_obstacle(Obstacle::fit("ball", &ball, tolerance).expect("fit"))
                .expect("add");
            m.world_pieces(&home(), &home())
                .expect("pieces")
                .iter()
                .filter(|(name, _)| *name == "ball")
                .flat_map(|(_, ps)| ps.iter())
                .map(|p| p.vertices.len())
                .sum()
        };
        let tight = verts(0.001);
        let loose = verts(0.01);
        assert!(
            loose < tight,
            "a looser fit must cost fewer vertices: {loose} vs {tight}"
        );
    }

    #[test]
    fn a_doubly_curved_body_is_refused_where_a_singly_curved_one_fits() {
        // The fit's plane budget was sized for robot links, whose worst mesh
        // converges in 75 planes. It bounds what an obstacle may be, and the
        // bound is about curvature, not size: a pillar needs planes only around
        // its circumference, while a ball needs them over a whole solid angle
        // and runs the budget out at a radius an operator could plausibly send.
        let ball = Obstacle::fit("ball", &sphere_cloud(0.2, 1000), MAX_DEVIATION_M)
            .expect_err("a 20 cm ball exceeds the plane budget at a 1 mm fit");
        assert!(
            matches!(
                ball,
                CollisionError::Build(BuildError::ToleranceTooTight { .. })
            ),
            "the refusal must name the tolerance, not read as a bad cloud: {ball}"
        );
        // The lever: the same ball at a hundredth of its radius.
        Obstacle::fit("ball", &sphere_cloud(0.2, 1000), 0.002).expect("a 2 mm fit holds it");
        Obstacle::fit("pillar", &cylinder_cloud(0.2, 1.0, 64), MAX_DEVIATION_M)
            .expect("a pillar fits");
        // Sparse sampling of the same ball fits, because the cloud's own hull is
        // then a coarse polyhedron with genuinely flat faces. Refining the mesh
        // an operator sends can therefore turn an accepted obstacle into a
        // refused one, which is the surprising half of this boundary.
        Obstacle::fit("coarse_ball", &sphere_cloud(0.2, 200), MAX_DEVIATION_M)
            .expect("a coarse ball fits");
    }

    #[test]
    fn a_cloud_that_bounds_no_solid_is_refused() {
        let flat = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let collinear: Vec<Point3<f64>> = (0..4).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
        for (case, points) in [
            ("empty", [].as_slice()),
            ("coplanar", &flat),
            ("collinear", &collinear),
        ] {
            assert!(
                Obstacle::fit("bad", points, MAX_DEVIATION_M).is_err(),
                "a {case} cloud bounds no solid and must be refused"
            );
        }
    }

    #[test]
    fn a_cloud_outside_the_fittable_extents_is_refused_by_its_size() {
        // Neither is refused for what it is without this check: the kernel
        // calls the over-large one "coplanar", describing its own arithmetic
        // rather than the caller's cloud, and it accepts the paper-thin one
        // outright.
        let paper_thin = Obstacle::fit(
            "sliver",
            &box_points([-1.0, -1.0, 0.0], [1.0, 1.0, 1e-9]),
            MAX_DEVIATION_M,
        )
        .expect_err("thinner than the fit budget");
        assert!(paper_thin.to_string().contains("thick"), "{paper_thin}");

        let kilometres = Obstacle::fit("room", &box_points([-1e4; 3], [1e4; 3]), MAX_DEVIATION_M)
            .expect_err("wider than the fittable range");
        assert!(kilometres.to_string().contains("units"), "{kilometres}");

        // The workspace-sized case in between is exactly what must still fit.
        Obstacle::fit("wall", &box_points(WALL_MIN, WALL_MAX), MAX_DEVIATION_M)
            .expect("a wall is fittable");
    }

    #[test]
    fn an_obstacle_needs_a_name() {
        for empty in ["", "   "] {
            assert!(
                Obstacle::fit(empty, &box_points(WALL_MIN, WALL_MAX), MAX_DEVIATION_M).is_err(),
                "an unnamed obstacle could never be removed"
            );
        }
    }

    #[test]
    fn a_non_finite_point_is_refused() {
        for bad in [f64::NAN, f64::INFINITY] {
            let mut points = box_points(WALL_MIN, WALL_MAX);
            points[0].y = bad;
            assert!(matches!(
                Obstacle::fit("wall", &points, MAX_DEVIATION_M),
                Err(CollisionError::NonFinite)
            ));
        }
    }

    #[test]
    fn a_name_already_taken_is_refused() {
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        for taken in ["wall", "openarm_left_link3"] {
            assert!(
                m.add_obstacle(obstacle(taken, WALL_MIN, WALL_MAX)).is_err(),
                "'{taken}' is already a body's name"
            );
        }
        assert_eq!(m.obstacle_names(), ["wall"]);
    }

    #[test]
    fn only_an_obstacle_can_be_removed_or_measured() {
        let mut m = model();
        for name in ["openarm_left_link3", "never_added"] {
            assert!(
                m.remove_obstacle(name).is_err(),
                "'{name}' is not an obstacle"
            );
            assert!(
                m.obstacle_clearance(name, &home(), &home()).is_err(),
                "'{name}' is not an obstacle"
            );
        }
    }

    #[test]
    fn removing_one_obstacle_leaves_the_others_measurable() {
        // The removed body shifts every later body's index down one, so a
        // surviving obstacle's pairs have to follow it. Measuring the third
        // after removing the second is what catches a stale index.
        let mut m = model();
        for (name, min, max) in [
            ("first", [-0.5, 0.30, 0.0], [0.5, 0.35, 1.0]),
            ("second", [-0.5, 0.40, 0.0], [0.5, 0.45, 1.0]),
            ("third", [-0.5, 0.50, 0.0], [0.5, 0.55, 1.0]),
        ] {
            m.add_obstacle(obstacle(name, min, max)).expect("add");
        }
        let before = m
            .obstacle_clearance("third", &home(), &home())
            .expect("clearance");
        m.remove_obstacle("second").expect("remove");
        assert_eq!(m.obstacle_names(), ["first", "third"]);
        let after = m
            .obstacle_clearance("third", &home(), &home())
            .expect("clearance");
        assert_eq!(before, after, "the third obstacle moved with the removal");
    }

    #[test]
    fn removing_every_obstacle_restores_the_original_model() {
        let mut m = model();
        let pairs_before = sorted_pairs(&m);
        let bound_before = m.clearance_step_bound(&[0.1; ARM_DOF], &[0.1; ARM_DOF], &[0.1; 2]);
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add wall");
        m.add_obstacle(overlapping_slab()).expect("add slab");
        assert_eq!(m.clear_obstacles(), 2);
        assert!(m.obstacle_names().is_empty());
        assert_eq!(
            sorted_pairs(&m),
            pairs_before,
            "the URDF pairs must come back exactly, not merely in the same number"
        );
        assert_eq!(
            m.clearance_step_bound(&[0.1; ARM_DOF], &[0.1; ARM_DOF], &[0.1; 2]),
            bound_before,
            "the levers must return to the URDF-only model's"
        );
    }

    /// Every checked pair by name, ordered, so two pair sets can be compared
    /// for identity rather than for size.
    fn sorted_pairs(m: &BimanualCollisionModel) -> Vec<(String, String)> {
        let mut pairs: Vec<(String, String)> = m
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| {
                if a <= b {
                    (a.to_string(), b.to_string())
                } else {
                    (b.to_string(), a.to_string())
                }
            })
            .collect();
        pairs.sort();
        pairs
    }

    #[test]
    fn an_obstacle_pair_raises_the_step_bound_it_is_scanned_under() {
        // The scan-skip bound is only sound if the levers follow the pair set.
        // With the URDF pairs in place every cross-arm pair already dominates,
        // so an obstacle can never raise the max and an omission would hide:
        // built with no pairs the levers start at zero, and the obstacle's
        // contribution is then the whole bound.
        let mut m = build(&[]).expect("model");
        let step = [0.1; ARM_DOF];
        assert_eq!(
            m.clearance_step_bound(&step, &step, &[0.0; 2]),
            0.0,
            "setup: no pairs means nothing can close"
        );
        m.add_obstacle(overlapping_slab()).expect("add");
        let bound = m.clearance_step_bound(&step, &step, &[0.0; 2]);
        assert!(
            bound > 0.0,
            "the obstacle's pairs never reached the step bound"
        );
        let moved: JointVec = std::array::from_fn(|j| home()[j] + step[j]);
        let d0 = m.min_distance(&home(), &home()).expect("query").distance;
        let d1 = m.min_distance(&moved, &home()).expect("query").distance;
        assert!(
            (d1 - d0).abs() <= bound + 1e-9,
            "bound {bound:.5} did not dominate |{d1:+.5} - {d0:+.5}|"
        );
        m.remove_obstacle("slab").expect("remove");
        assert_eq!(
            m.clearance_step_bound(&step, &step, &[0.0; 2]),
            0.0,
            "the levers kept a removed obstacle's pairs"
        );
    }

    #[test]
    fn an_obstacle_can_be_removed_after_being_the_nearest_pair() {
        // A removed body must leave the scan, not merely stop being reported.
        let mut m = model();
        let before = m.min_distance(&home(), &home()).expect("query").distance;
        m.add_obstacle(overlapping_slab()).expect("add");
        let p = m.min_distance(&home(), &home()).expect("query");
        assert!(
            p.link_a == "slab" || p.link_b == "slab",
            "setup: the slab must win before it is removed"
        );
        m.remove_obstacle("slab").expect("remove");
        let after = m.min_distance(&home(), &home()).expect("query");
        assert!(
            after.link_a != "slab" && after.link_b != "slab",
            "a removed obstacle is still being scanned"
        );
        assert_eq!(after.distance, before, "the model did not return to itself");
    }

    #[test]
    fn removing_the_first_obstacle_leaves_the_rest_measurable() {
        // The boundary of the tail: the removed body is the first obstacle, so
        // every surviving obstacle moves.
        let mut m = model();
        for (name, min, max) in [
            ("first", [-0.5, 0.30, 0.0], [0.5, 0.35, 1.0]),
            ("second", [-0.5, 0.40, 0.0], [0.5, 0.45, 1.0]),
        ] {
            m.add_obstacle(obstacle(name, min, max)).expect("add");
        }
        let before = m
            .obstacle_clearance("second", &home(), &home())
            .expect("clearance");
        m.remove_obstacle("first").expect("remove");
        assert_eq!(m.obstacle_names(), ["second"]);
        assert_eq!(
            m.obstacle_clearance("second", &home(), &home())
                .expect("clearance"),
            before
        );
    }

    #[test]
    fn an_obstacle_can_be_added_again_after_a_clear() {
        // Clearing frees the names and the body slots the next insertion reuses.
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        let before = m
            .obstacle_clearance("wall", &home(), &home())
            .expect("clearance");
        assert_eq!(m.clear_obstacles(), 1);
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("the name is free again");
        assert_eq!(
            m.obstacle_clearance("wall", &home(), &home())
                .expect("clearance"),
            before
        );
    }

    #[test]
    fn an_obstacles_clearance_matches_a_brute_force_scan_of_its_own_pairs() {
        // The filtered scan keeps the broadphase ordering and its early break;
        // this is the check that the filter cannot drop a pair that would win.
        let mut m = model();
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add wall");
        m.add_obstacle(overlapping_slab()).expect("add slab");
        for (ql, qr) in [
            (home(), home()),
            ([0.0, 0.5, 0.0, 0.4, 0.0, 0.0, 0.0], home()),
            ([0.2, 0.0, 0.3, 0.9, 0.0, 0.0, 0.0], [0.0; ARM_DOF]),
        ] {
            for name in ["wall", "slab"] {
                let scanned = m.obstacle_clearance(name, &ql, &qr).expect("clearance");
                let brute = brute_force_obstacle_clearance(&mut m, name, &ql, &qr);
                assert!(
                    (scanned - brute).abs() < 1e-9,
                    "{name}: filtered scan {scanned:+.6}, brute force {brute:+.6}"
                );
            }
        }
    }

    /// The minimum over every checked pair the named obstacle is in, with no
    /// broadphase and no early break: the filtered scan's independent answer.
    fn brute_force_obstacle_clearance(
        m: &mut BimanualCollisionModel,
        name: &str,
        q_left: &JointVec,
        q_right: &JointVec,
    ) -> f64 {
        let index = m.obstacle_index(name).expect("a live obstacle");
        let partners: Vec<usize> = m
            .pairs
            .iter()
            .filter(|p| p.a == index || p.b == index)
            .map(|p| if p.a == index { p.b } else { p.a })
            .collect();
        m.place(q_left, q_right);
        let mut best = f64::INFINITY;
        for other in partners {
            let (iso_a, iso_b) = (m.world_iso[index], m.world_iso[other]);
            for ha in &m.bodies[index].local {
                for hb in &m.bodies[other].local {
                    let r = gjk::distance(&Placed::new(ha, iso_a), &Placed::new(hb, iso_b));
                    best = best.min(r.distance);
                }
            }
        }
        best
    }

    #[test]
    fn an_exclusion_survives_obstacle_churn() {
        let excluded = PairSpec::new("openarm_left_link7", "openarm_right_link7");
        let mut m = BimanualCollisionModel::builder(
            URDF,
            MESHES,
            "openarm_left_link0",
            "openarm_right_link0",
        )
        .exclude(std::slice::from_ref(&excluded))
        .build()
        .expect("model");
        let has_excluded_pair = |m: &BimanualCollisionModel| {
            m.checked_pairs().iter().any(|&(a, b)| {
                (a == excluded.a && b == excluded.b) || (a == excluded.b && b == excluded.a)
            })
        };
        assert!(!has_excluded_pair(&m));
        m.add_obstacle(obstacle("wall", WALL_MIN, WALL_MAX))
            .expect("add");
        m.remove_obstacle("wall").expect("remove");
        assert!(
            !has_excluded_pair(&m),
            "an insertion must not re-derive the pairs the builder was told to drop"
        );
    }
}
