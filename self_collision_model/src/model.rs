//! The runtime model: every chain's convex-hull pieces placed by forward
//! kinematics and the minimum signed distance over the checked pairs.
//!
//! Built once from the URDF plus its collision meshes, queried with the joint
//! configuration of every chain. A query is FK plus GJK over the checked
//! piece-pairs; where pieces overlap, EPA recovers the penetration depth, so
//! the distance is signed and continuous through contact.
//!
//! The checked pairs are every pair of bodies but those whose collisions the robot's
//! description disables. The hulls are tight: the reported distance is the true
//! surface clearance, with no safety margin baked into the geometry. Keeping the
//! bodies apart is the caller's job (a band over the reported distance).

use std::collections::HashSet;

use crate::nalgebra::{Isometry3, Point3, Unit, Vector3};
use chain_kinematics::{Chain, ChainSpec, JointSelection};
use robot_description::{Finger, RobotDescription};

use crate::assemble::fit_bodies;
use collision_geometry::chain::{CarriedPart, Rider};
use collision_geometry::gjk::{self, Hull, Placed};
use collision_geometry::urdf_collision::{JointKind, ParentJoint, UrdfCollisions, place_1dof};

use crate::{BuildError, CollisionError, FingerError};

/// The joint positions of every chain of a model, in the order of its description's
/// arms.
pub type Joints<const N: usize, const K: usize> = [[f64; N]; K];

/// How a body's hulls reach the world frame.
#[derive(Clone, Copy)]
enum Placement {
    /// Already in world frame (a torso, a base, a mount); identity for the whole run.
    Fixed,
    /// Link `segment` of chain `chain`; placed by FK every query.
    Chain { chain: usize, segment: usize },
}

impl Placement {
    /// The chain the body rides, none for a fixed body.
    fn chain(self) -> Option<usize> {
        match self {
            Placement::Fixed => None,
            Placement::Chain { chain, .. } => Some(chain),
        }
    }
}

/// A body's bounding sphere in its own local frame: center over the hull
/// vertices, radius covering the rounded pieces. The radius is rotation
/// invariant, so placing the center gives a cheap distance lower bound for the
/// broadphase.
#[derive(Clone)]
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

#[derive(Clone)]
struct Body {
    name: String,
    /// Convex-hull pieces, in the body's local frame (world for `Fixed`).
    local: Vec<Hull>,
    placement: Placement,
    bound: BoundingSphere,
    /// For a finger: how its hull hangs off its host chain link, so it can be
    /// placed at the live opening. `None` for every other body (whose hulls sit
    /// directly in the link frame).
    finger: Option<FingerJoint>,
}

impl Body {
    /// World pose of this body given its host chain link's world pose and its
    /// chain's opening: the link pose directly for an ordinary body, or composed
    /// with the finger offset at `opening` for a finger.
    fn place_on(&self, link_world: Isometry3<f64>, opening: f64) -> Isometry3<f64> {
        match &self.finger {
            Some(f) => link_world * f.offset(opening),
            None => link_world,
        }
    }
}

/// A finger's parent joint, parsed at build into an infallible placer: the joint
/// origin, its unit axis, whether it rotates (revolute) or slides (prismatic), and
/// the joint positions at which the finger is shut and fully open. Validated once
/// here (bounded 1-DOF, non-zero axis) so placing it at any opening cannot fail in
/// the per-tick hot path.
#[derive(Clone, Copy)]
struct FingerJoint {
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
    revolute: bool,
    /// Joint position (URDF joint units) at shut and at fully open.
    closed: f64,
    open: f64,
}

impl FingerJoint {
    /// Parse a finger's parent joint into a validated placer, the single gate
    /// for finger-joint kinds: only a prismatic or revolute finger can be
    /// placed, and its axis must normalize.
    fn from_joint(
        name: &str,
        joint: &ParentJoint,
        closed: f64,
        open: f64,
    ) -> Result<Self, BuildError> {
        if joint.kind == JointKind::Continuous {
            return Err(BuildError::Geometry(format!(
                "finger '{name}' joint is continuous, which bounds no travel"
            )));
        }
        let (axis, revolute) = joint
            .one_dof()
            .map_err(|e| BuildError::Geometry(format!("finger '{name}': {e}")))?;
        Ok(FingerJoint {
            origin: joint.origin,
            axis,
            revolute,
            closed,
            open,
        })
    }

    /// Placement of the finger hull in its host link's frame at opening `fraction`
    /// in `[0, 1]` (0 = shut, 1 = fully open), linearly interpolating the
    /// finger-joint travel, for a prismatic and a revolute finger alike.
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

/// The parts of a [`Body`] the gradient needs, copied out so the chains' FK can
/// be read while they are in hand: which chain the body rides and, for a finger,
/// its live placer.
#[derive(Clone, Copy)]
struct BodyKinematics {
    placement: Placement,
    finger: Option<FingerJoint>,
}

/// One checked pair, resolved to body indices.
#[derive(Clone)]
struct Pair {
    a: usize,
    b: usize,
}

/// The checked pairs a distance query reads.
enum Read {
    /// Every checked pair.
    All,
    /// Each pair one of whose bodies rides the chain.
    OfChain(usize),
    /// Each pair one of whose bodies rides the chain and neither of whose bodies
    /// rides another chain.
    OfChainAlone(usize),
}

/// What a scan of the checked pairs at one configuration finds
/// ([`SelfCollisionModel::scan`]).
struct Scan {
    /// The nearest of the pairs the scan measured; none where it measured no pair.
    nearest: Option<Closest>,
    /// The least of the lower bounds at which the scan set pairs and pieces aside at
    /// `within` or past it; infinite where it set none aside there.
    set_aside: f64,
}

/// Best candidate while scanning pairs in [`SelfCollisionModel::scan`].
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
/// surface distance with respect to each chain's joints and each chain's opening
/// fraction. `joints[k][j]` is `d(distance)/d(q[k][j])`; `openings[k]` is
/// `d(distance)/d(opening_k)`, nonzero only when a finger body carries a witness;
/// separating motion has a positive gradient. Computed analytically from the
/// nearest pair's witness points (the gradient of the active pair, by the envelope
/// theorem), so it costs one distance query plus two point Jacobians.
#[derive(Debug, Clone)]
pub struct DistanceGradient<'a, const N: usize, const K: usize> {
    pub proximity: Proximity<'a>,
    pub joints: Joints<N, K>,
    pub openings: [f64; K],
}

/// The self-collision model of a robot of `K` serial chains of `N` joints each.
#[derive(Clone)]
pub struct SelfCollisionModel<const N: usize, const K: usize> {
    chains: [Chain<N>; K],
    bodies: Vec<Body>,
    pairs: Vec<Pair>,
    /// Per-body world pose, refreshed by [`place`](Self::place). Fixed bodies
    /// keep the identity (their hulls are already in world frame).
    world_iso: Vec<Isometry3<f64>>,
    /// Opening per chain as a fraction in `[0, 1]` (0 = shut, 1 = fully open), set
    /// by [`set_gripper_openings`](Self::set_gripper_openings). Finger bodies are
    /// placed at this opening every query. Defaults to fully open, the widest
    /// outboard envelope; it is not a full substitute for the real opening (shut
    /// fingers occupy between-jaws space the open placement vacates), so a caller
    /// governing near shut fingers must feed the measured opening before trusting
    /// the clearance.
    openings: [f64; K],
    /// Per-chain, per-joint Lipschitz levers of the min surface distance (m/rad):
    /// the max over checked pairs of the sum of both bodies' per-joint reach
    /// bounds on that chain (a same-chain pair moves both witnesses with one
    /// chain's joints; see [`body_reaches`]). Rebuilt whenever the pair list
    /// changes; feeds [`clearance_step_bound`](Self::clearance_step_bound).
    levers: Joints<N, K>,
    /// Per-chain Lipschitz levers of the min surface distance per unit opening
    /// fraction (m): the max over checked pairs of the paired finger bodies'
    /// surface speed bounds under their own joint travel. Rebuilt with `levers`;
    /// feeds [`clearance_step_bound`](Self::clearance_step_bound).
    opening_levers: [f64; K],
}

impl<const N: usize, const K: usize> SelfCollisionModel<N, K> {
    /// The self model of the robot `description` describes, its collision meshes read
    /// from `meshes_dir`: a chain per arm, in the description's order, from the arm's base
    /// link to its grasp link over the arm's joints; the fingers of each arm's gripper,
    /// each placed between where the gripper's `closed` and `open` states put it; and
    /// every pair of bodies checked but those whose collisions the description disables.
    /// A description of other than `K` arms, or with an arm of other than `N` joints, is
    /// refused. The model is a pure distance oracle: it reports clearances (and, via
    /// [`distance_gradient`](Self::distance_gradient), their gradient), and the caller
    /// decides how to throttle on them.
    pub fn from_description(
        description: &RobotDescription,
        meshes_dir: &str,
    ) -> Result<Self, CollisionError> {
        let mut model = Self::assembled(description, meshes_dir)?;
        let disabled: HashSet<(&str, &str)> = description
            .disabled_pairs()
            .iter()
            .flat_map(|pair| {
                [
                    (pair.link1.as_str(), pair.link2.as_str()),
                    (pair.link2.as_str(), pair.link1.as_str()),
                ]
            })
            .collect();
        let bodies = &model.bodies;
        let checked: Vec<Pair> = (0..bodies.len())
            .flat_map(|a| (a + 1..bodies.len()).map(move |b| Pair { a, b }))
            .filter(|pair| {
                !disabled.contains(&(bodies[pair.a].name.as_str(), bodies[pair.b].name.as_str()))
            })
            .collect();
        model.set_pairs(checked);
        Ok(model)
    }

    /// The bodies of the robot `description` describes, with no checked pairs set yet:
    /// a chain per arm and the fingers of each arm's gripper, as
    /// [`from_description`](Self::from_description) builds them.
    fn assembled(description: &RobotDescription, meshes_dir: &str) -> Result<Self, BuildError> {
        let fingers: Vec<&Finger> = description
            .arms()
            .iter()
            .flat_map(|arm| &arm.gripper.fingers)
            .collect();
        Self::assembled_with_fingers(description, meshes_dir, &fingers)
    }

    /// The bodies of the robot `description` describes, as [`Self::assembled`] gives
    /// them, with `fingers` for its grippers' fingers.
    fn assembled_with_fingers(
        description: &RobotDescription,
        meshes_dir: &str,
        fingers: &[&Finger],
    ) -> Result<Self, BuildError> {
        const {
            assert!(
                N > 0 && K > 0,
                "a model holds at least one chain of one joint"
            )
        };
        let arms = description.arms();
        if arms.len() != K {
            return Err(BuildError::Arms {
                expected: K,
                found: arms.len(),
            });
        }
        if let Some(arm) = arms.iter().find(|arm| arm.joints.len() != N) {
            return Err(BuildError::ArmJoints {
                arm: arm.name.clone(),
                expected: N,
                found: arm.joints.len(),
            });
        }
        if let Some((first, second)) = arms.iter().enumerate().find_map(|(i, arm)| {
            arms[i + 1..]
                .iter()
                .find(|other| other.base_link == arm.base_link)
                .map(|other| (arm, other))
        }) {
            return Err(BuildError::SharedBase {
                first: first.name.clone(),
                second: second.name.clone(),
                base: first.base_link.clone(),
            });
        }
        let joints: Vec<Vec<&str>> = arms
            .iter()
            .map(|arm| arm.joints.iter().map(String::as_str).collect())
            .collect();
        let specs: [ChainSpec<'_>; K] = std::array::from_fn(|index| ChainSpec {
            base_link: Some(arms[index].base_link.as_str()),
            tip_link: arms[index].grasp_link.as_str(),
            joints: JointSelection::Named(&joints[index]),
        });
        Self::assemble(description, meshes_dir, &specs, fingers)
    }

    /// Fit every collision body of `description` and place them, a chain per spec of
    /// `specs`, with no checked pairs set yet.
    fn assemble(
        description: &RobotDescription,
        meshes_dir: &str,
        specs: &[ChainSpec<'_>; K],
        fingers: &[&Finger],
    ) -> Result<Self, BuildError> {
        let built: Vec<Chain<N>> = specs
            .iter()
            .map(|spec| Chain::<N>::from_urdf(description.robot(), spec))
            .collect::<Result<_, _>>()?;
        let chains: [Chain<N>; K] = built
            .try_into()
            .unwrap_or_else(|_| unreachable!("one chain per spec"));

        let names: Vec<Vec<String>> = chains
            .iter()
            .map(|chain| {
                let posed = chain.at(&[0.0; N]);
                (0..N).map(|i| posed.link_name(i).to_string()).collect()
            })
            .collect();

        let parsed = UrdfCollisions::from_urdf(description.urdf())?;
        let fitted = fit_bodies(&parsed, &names, fingers, meshes_dir)?;

        let mut bodies: Vec<Body> = Vec::new();
        let push_body = |bodies: &mut Vec<Body>, body: Body| -> Result<(), BuildError> {
            if bodies.iter().any(|b| b.name == body.name) {
                return Err(BuildError::DuplicateBody {
                    name: body.name.clone(),
                });
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
                    finger: None,
                },
            )?;
        }
        for (chain, chain_names) in names.iter().enumerate() {
            for (segment, name) in chain_names.iter().enumerate() {
                let hulls = links
                    .remove(name)
                    .ok_or_else(|| BuildError::SharedLink { name: name.clone() })?;
                let bound = BoundingSphere::of(&hulls);
                push_body(
                    &mut bodies,
                    Body {
                        name: name.clone(),
                        local: hulls,
                        placement: Placement::Chain { chain, segment },
                        bound,
                        finger: None,
                    },
                )?;
            }
        }
        // Finger bodies hang off their host chain link's FK segment (so they share
        // its lineage: not checked against their own link or a sibling finger, but
        // checked against another chain and a fixed body) and carry a placer so
        // each query positions them at the live opening.
        for finger in fitted.fingers {
            let placement = chain_segment(&names, &finger.parent_link)
                .expect("fit_bodies only emits fingers hosted on a chain link");
            let placer =
                FingerJoint::from_joint(&finger.name, &finger.joint, finger.closed, finger.open)?;
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
            chains,
            bodies,
            pairs: Vec::new(),
            world_iso,
            openings: [1.0; K],
            levers: [[0.0; N]; K],
            opening_levers: [0.0; K],
        })
    }

    /// Replace the checked pairs with `pairs`, each of two bodies of the model.
    fn set_pairs(&mut self, pairs: Vec<Pair>) {
        self.pairs = pairs;
        self.recompute_levers();
    }

    /// Rebuild the per-chain Lipschitz levers from the checked pairs: for each
    /// chain and joint, the max over pairs of the sum of both bodies' reaches on
    /// that chain. Summing per pair is what keeps the bound sound for a same-chain
    /// pair, whose two witnesses both move with that chain's joints. Called
    /// whenever the pair list changes (build-time only, so recomputing the
    /// per-body reaches here is free).
    fn recompute_levers(&mut self) {
        let reaches = body_reaches(&self.chains, &self.bodies);
        let rides = |body: usize, chain: usize| self.bodies[body].placement.chain() == Some(chain);
        let chain_reach = |body: usize, chain: usize, j: usize| -> f64 {
            if rides(body, chain) {
                reaches[body][j]
            } else {
                0.0
            }
        };
        self.levers = std::array::from_fn(|chain| {
            std::array::from_fn(|j| {
                self.pairs
                    .iter()
                    .map(|p| chain_reach(p.a, chain, j) + chain_reach(p.b, chain, j))
                    .fold(0.0, f64::max)
            })
        });
        // A finger body's surface moves at most `opening_reach` meters per unit
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
        let chain_opening_reach = |body: usize, chain: usize| -> f64 {
            if rides(body, chain) {
                opening_reach(&self.bodies[body])
            } else {
                0.0
            }
        };
        self.opening_levers = std::array::from_fn(|chain| {
            self.pairs
                .iter()
                .map(|p| chain_opening_reach(p.a, chain) + chain_opening_reach(p.b, chain))
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

    /// The nearest checked pair at configuration `q`, of the pairs `read` names: the
    /// minimum signed distance over them. The shared core of
    /// [`min_distance`](Self::min_distance),
    /// [`min_distance_of_chain`](Self::min_distance_of_chain),
    /// [`min_distance_of_chain_alone`](Self::min_distance_of_chain_alone) and
    /// [`distance_gradient`](Self::distance_gradient).
    fn closest(&mut self, q: &Joints<N, K>, read: Read) -> Result<Closest, CollisionError> {
        self.scan(q, read, f64::INFINITY)?
            .nearest
            .ok_or(CollisionError::NoPairs)
    }

    /// The minimum signed distance over the pairs `read` names as a check at `within`
    /// meters reads it: exact where it lies under `within`, and elsewhere a lower bound
    /// at or above `within`, the least of the distances the scan measured and of the
    /// bounds of the pairs it set aside. The shared core of
    /// [`min_distance_within`](Self::min_distance_within),
    /// [`min_distance_of_chain_within`](Self::min_distance_of_chain_within) and
    /// [`min_distance_of_chain_alone_within`](Self::min_distance_of_chain_alone_within).
    fn distance_within(
        &mut self,
        q: &Joints<N, K>,
        read: Read,
        within: f64,
    ) -> Result<f64, CollisionError> {
        ensure_finite(q)?;
        if within.is_nan() {
            return Err(CollisionError::NonFinite);
        }
        let scan = self.scan(q, read, within)?;
        Ok(scan
            .nearest
            .map_or(scan.set_aside, |c| c.distance.min(scan.set_aside)))
    }

    /// Places the hulls by FK, then scans the pairs `read` names (broadphase-ordered)
    /// for the nearest, measuring each piece pair that a lower bound on its distance,
    /// its bounding spheres' gap and then its separation along the line of its centers,
    /// leaves under `within` meters and no farther than the best distance found, and
    /// setting aside with its bound each one a bound puts at `within` or past it. Fails
    /// where `read` names no pair.
    fn scan(&mut self, q: &Joints<N, K>, read: Read, within: f64) -> Result<Scan, CollisionError> {
        self.place(q);
        let on = |body: usize| self.bodies[body].placement.chain();
        let reads = |pair: &Pair| match read {
            Read::All => true,
            Read::OfChain(chain) => on(pair.a) == Some(chain) || on(pair.b) == Some(chain),
            Read::OfChainAlone(chain) => {
                let elsewhere = |body: usize| on(body).is_some_and(|other| other != chain);
                (on(pair.a) == Some(chain) || on(pair.b) == Some(chain))
                    && !elsewhere(pair.a)
                    && !elsewhere(pair.b)
            }
        };

        // Broadphase: a pair's bounding-sphere gap is a lower bound on its true
        // distance. Scanned in ascending order of that bound, once it exceeds
        // the best distance found, or reaches `within`, no remaining pair can
        // win, so the scan stops.
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
            .filter(|(_, p)| reads(p))
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

        if order.is_empty() {
            return Err(CollisionError::NoPairs);
        }

        let mut best: Option<Closest> = None;
        let mut set_aside = f64::INFINITY;
        // Whether `bound`, a lower bound on the distance of a pair or of a piece pair,
        // sets it aside: at or above `within`, where the bound is its reading, or past
        // the best distance found.
        let mut sets_aside = |bound: f64, best: &Option<Closest>| {
            if bound >= within {
                set_aside = set_aside.min(bound);
                return true;
            }
            best.as_ref().is_some_and(|c| bound > c.distance)
        };
        for (lower_bound, i) in order {
            if sets_aside(lower_bound, &best) {
                break;
            }
            let pair = &self.pairs[i];
            let (iso_a, iso_b) = (self.world_iso[pair.a], self.world_iso[pair.b]);
            // The transformed piece centers of body b are constant across body
            // a's pieces; place them once per pair, not once per (ha, hb).
            let centers_b: Vec<Point3<f64>> = self.bodies[pair.b]
                .local
                .iter()
                .map(|hb| iso_b * hb.center())
                .collect();
            for ha in &self.bodies[pair.a].local {
                let center_a = iso_a * ha.center();
                for (hb, center_b) in self.bodies[pair.b].local.iter().zip(&centers_b) {
                    // Piece-level prefilter, the pair broadphase's sphere bound and
                    // then the pieces' separation along the line of their centers,
                    // a bound that follows their shapes: two long links side by
                    // side lie apart along it where their spheres overlap. A piece
                    // pair either bound sets aside skips its GJK, so a multi-piece
                    // body (a torso's region decomposition) pays GJK for its
                    // pieces near the query alone. Pieces whose centers coincide
                    // take the x axis: every direction bounds their distance.
                    let offset = center_b - center_a;
                    let gap = offset.norm() - ha.bound_radius() - hb.bound_radius();
                    if sets_aside(gap, &best) {
                        continue;
                    }
                    let (placed_a, placed_b) = (Placed::new(ha, iso_a), Placed::new(hb, iso_b));
                    let toward_b = Unit::try_new(offset, 0.0).unwrap_or(Vector3::x_axis());
                    if sets_aside(
                        gjk::separation_along(&placed_a, &placed_b, &toward_b),
                        &best,
                    ) {
                        continue;
                    }
                    let r = gjk::distance(&placed_a, &placed_b);
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
        Ok(Scan {
            nearest: best,
            set_aside,
        })
    }

    /// Minimum signed distance over all checked pairs at configuration `q`, with the
    /// witness points. Non-finite joint values are rejected, so the caller fails safe.
    pub fn min_distance(&mut self, q: &Joints<N, K>) -> Result<Proximity<'_>, CollisionError> {
        self.proximity(q, Read::All)
    }

    /// [`min_distance`](Self::min_distance) over the checked pairs one of whose
    /// bodies rides chain `chain`, its fingers included: the distance a motion of
    /// that chain changes while every other chain holds where it stands. Panics on a
    /// chain past the model's `K`.
    pub fn min_distance_of_chain(
        &mut self,
        chain: usize,
        q: &Joints<N, K>,
    ) -> Result<Proximity<'_>, CollisionError> {
        assert!(chain < K, "chain {chain} of a model of {K}");
        self.proximity(q, Read::OfChain(chain))
    }

    /// [`min_distance_of_chain`](Self::min_distance_of_chain) over the pairs neither
    /// of whose bodies rides another chain: the chain against the robot's fixed
    /// bodies and against itself, with every other chain's bodies set aside. Panics
    /// on a chain past the model's `K`.
    pub fn min_distance_of_chain_alone(
        &mut self,
        chain: usize,
        q: &Joints<N, K>,
    ) -> Result<Proximity<'_>, CollisionError> {
        assert!(chain < K, "chain {chain} of a model of {K}");
        self.proximity(q, Read::OfChainAlone(chain))
    }

    /// The distance of [`min_distance`](Self::min_distance) as a check at `within`
    /// meters reads it: exact where it lies under `within`, and elsewhere a lower bound
    /// at or above `within`. The scan measures a pair only where the bounds on its
    /// distance leave it under `within`, so a check of a clearance pays for the pairs
    /// near that clearance. Fails on a `within` that is not a number.
    pub fn min_distance_within(
        &mut self,
        q: &Joints<N, K>,
        within: f64,
    ) -> Result<f64, CollisionError> {
        self.distance_within(q, Read::All, within)
    }

    /// The distance of [`min_distance_of_chain`](Self::min_distance_of_chain), read as
    /// [`min_distance_within`](Self::min_distance_within) reads its own. Panics on a
    /// chain past the model's `K`.
    pub fn min_distance_of_chain_within(
        &mut self,
        chain: usize,
        q: &Joints<N, K>,
        within: f64,
    ) -> Result<f64, CollisionError> {
        assert!(chain < K, "chain {chain} of a model of {K}");
        self.distance_within(q, Read::OfChain(chain), within)
    }

    /// The distance of
    /// [`min_distance_of_chain_alone`](Self::min_distance_of_chain_alone), read as
    /// [`min_distance_within`](Self::min_distance_within) reads its own. Panics on a
    /// chain past the model's `K`.
    pub fn min_distance_of_chain_alone_within(
        &mut self,
        chain: usize,
        q: &Joints<N, K>,
        within: f64,
    ) -> Result<f64, CollisionError> {
        assert!(chain < K, "chain {chain} of a model of {K}");
        self.distance_within(q, Read::OfChainAlone(chain), within)
    }

    /// The nearest pair of the pairs [`closest`](Self::closest) scans, with its
    /// witness points, at a finite configuration.
    fn proximity(&mut self, q: &Joints<N, K>, read: Read) -> Result<Proximity<'_>, CollisionError> {
        ensure_finite(q)?;
        let c = self.closest(q, read)?;
        Ok(Proximity {
            distance: c.distance,
            link_a: &self.bodies[c.a].name,
            link_b: &self.bodies[c.b].name,
            on_a: c.on_a,
            on_b: c.on_b,
        })
    }

    /// The nearest-pair [`Proximity`] and the analytic gradient of its distance
    /// with respect to each chain's joints (see [`DistanceGradient`]). The gradient
    /// is the nearest pair's separating direction projected through each witness
    /// point's velocity Jacobian, so it reflects the same min-over-pairs distance
    /// `min_distance` returns at one distance query's cost. Fails on a non-finite
    /// configuration, or on degenerate core contact (the rounded surfaces then
    /// overlap by the summed radii) where no separating direction is defined; a
    /// velocity-barrier caller holds there.
    pub fn distance_gradient(
        &mut self,
        q: &Joints<N, K>,
    ) -> Result<DistanceGradient<'_, N, K>, CollisionError> {
        ensure_finite(q)?;
        let c = self.closest(q, Read::All)?;
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
        let (joints_a, open_a) = self.gradient_contribution(kin_a, &c.on_a, &normal, 1.0, q);
        let (joints_b, open_b) = self.gradient_contribution(kin_b, &c.on_b, &normal, -1.0, q);
        Ok(DistanceGradient {
            proximity: Proximity {
                distance: c.distance,
                link_a: &self.bodies[c.a].name,
                link_b: &self.bodies[c.b].name,
                on_a: c.on_a,
                on_b: c.on_b,
            },
            joints: std::array::from_fn(|k| {
                std::array::from_fn(|j| joints_a[k][j] + joints_b[k][j])
            }),
            openings: std::array::from_fn(|k| open_a[k] + open_b[k]),
        })
    }

    /// One body's contribution to the distance gradient: the pair's separating
    /// direction `normal` projected through the witness `point`'s velocity
    /// Jacobian, plus, for a finger body, through the point's velocity per unit
    /// opening fraction. `sign` is +1 for body a and -1 for body b (`normal`
    /// increases the distance for a, decreases it for b). A world-fixed body
    /// contributes nothing.
    fn gradient_contribution(
        &self,
        kinematics: BodyKinematics,
        point: &Point3<f64>,
        normal: &Vector3<f64>,
        sign: f64,
        q: &Joints<N, K>,
    ) -> (Joints<N, K>, [f64; K]) {
        let BodyKinematics { placement, finger } = kinematics;
        let mut joints = [[0.0; N]; K];
        let mut openings = [0.0; K];
        if let Placement::Chain { chain, segment } = placement {
            let posed = self.chains[chain].at(&q[chain]);
            let cols = posed.point_world_jacobian(point, segment);
            joints[chain] = std::array::from_fn(|j| sign * normal.dot(&cols[j]));
            openings[chain] = finger.map_or(0.0, |f| {
                let v = f.point_velocity_per_fraction(&posed.link_pose_world(segment), point);
                sign * normal.dot(&v)
            });
        }
        (joints, openings)
    }

    /// World-frame hull pieces of every body at configuration `q`, paired with the
    /// body name: each piece gives its placed vertices, and its hull the face
    /// triangles and the rounding radius, the exact collision surface. A planner
    /// measures the objects a chain carries against the bodies the plan does not move
    /// by these pieces; [`min_distance`](Self::min_distance) measures the checked
    /// pairs between the bodies.
    pub fn world_pieces(
        &mut self,
        q: &Joints<N, K>,
    ) -> Result<Vec<(&str, Vec<Placed<'_>>)>, CollisionError> {
        ensure_finite(q)?;
        self.place(q);
        Ok(self
            .bodies
            .iter()
            .zip(&self.world_iso)
            .map(|(b, iso)| {
                let pieces = b.local.iter().map(|h| Placed::new(h, *iso)).collect();
                (b.name.as_str(), pieces)
            })
            .collect())
    }

    /// The fingers of chain `chain` where they hang at its current opening: the parts
    /// `carried` that the chain carries (`ChainGeometry::carried`), each as its rider
    /// at the offset in its link's frame at which the opening places it. A part the
    /// model does not hold as a finger of that chain, on the same link, is refused
    /// ([`FingerError`]).
    pub fn fingers<'g>(
        &self,
        chain: usize,
        carried: &'g [CarriedPart],
    ) -> Result<Vec<Rider<'g>>, FingerError> {
        carried
            .iter()
            .map(|part| {
                let rider = part.rider();
                Ok(Rider {
                    offset: self.finger_offset(chain, rider.name, rider.segment)?,
                    ..rider
                })
            })
            .collect()
    }

    /// The offset in the frame of link `segment` of chain `chain` at which that chain's
    /// opening places the finger `link`.
    fn finger_offset(
        &self,
        chain: usize,
        link: &str,
        segment: usize,
    ) -> Result<Isometry3<f64>, FingerError> {
        let named = || link.to_string();
        let body = self
            .bodies
            .iter()
            .find(|body| body.name == link)
            .ok_or_else(|| FingerError::NoBody { link: named() })?;
        let finger = body
            .finger
            .as_ref()
            .ok_or_else(|| FingerError::NotAFinger { link: named() })?;
        let host = match body.placement {
            Placement::Chain {
                chain: on,
                segment: host,
            } if on == chain => host,
            _ => {
                return Err(FingerError::OtherChain {
                    link: named(),
                    chain,
                });
            }
        };
        if host != segment {
            return Err(FingerError::OtherLink {
                link: named(),
                model: host,
                chain: segment,
            });
        }
        Ok(finger.offset(self.openings[chain]))
    }

    /// Set each chain's opening as a fraction in `[0, 1]` (0 = shut, 1 = fully open);
    /// values are clamped. Finger bodies are placed at this opening on every
    /// subsequent query, so the reported clearance follows the fingers' true
    /// positions. A non-finite value is ignored for that chain (the last good opening
    /// stands), so a bad reading never poisons the placement.
    pub fn set_gripper_openings(&mut self, openings: &[f64; K]) {
        for (held, opening) in self.openings.iter_mut().zip(openings) {
            if opening.is_finite() {
                *held = opening.clamp(0.0, 1.0);
            }
        }
    }

    /// Upper bound (m) on how much the minimum surface distance can change over a
    /// step of `dq` on the chains' joints and `dopenings` on their opening fractions,
    /// valid along the whole straight segment in that combined space:
    /// `sum_j levers[k][j] * |dq[k][j]|` over every chain plus
    /// `opening_levers[k] * |dopenings[k]|` over every chain. Each lever bounds, over
    /// all poses and openings, the worst per-unit closing rate of any checked pair
    /// (summing both witnesses' travel when a pair's bodies share a chain), and a
    /// minimum of Lipschitz functions is Lipschitz, so a segment whose start clearance
    /// exceeds a floor by more than this bound cannot cross that floor anywhere along
    /// the step. Deliberately loose (chain-length bounds), so it is sound for a caller
    /// skipping an exact scan, never tight.
    pub fn clearance_step_bound(&self, dq: &Joints<N, K>, dopenings: &[f64; K]) -> f64 {
        let dot_abs = |lever: &[f64; N], dq: &[f64; N]| -> f64 {
            lever.iter().zip(dq).map(|(l, d)| l * d.abs()).sum::<f64>()
        };
        let joints: f64 = self
            .levers
            .iter()
            .zip(dq)
            .map(|(lever, dq)| dot_abs(lever, dq))
            .sum();
        let bound = self
            .opening_levers
            .iter()
            .zip(dopenings)
            .fold(joints, |bound, (lever, d)| bound + lever * d.abs());
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
    /// additionally offset by their host link pose at the chain's current opening.
    fn place(&mut self, q: &Joints<N, K>) {
        let poses: [[Isometry3<f64>; N]; K] =
            std::array::from_fn(|chain| link_poses(&self.chains[chain], &q[chain]));
        let openings = self.openings;
        for (body, iso) in self.bodies.iter().zip(self.world_iso.iter_mut()) {
            if let Placement::Chain { chain, segment } = body.placement {
                *iso = body.place_on(poses[chain][segment], openings[chain]);
            }
        }
    }
}

/// The [`Placement`] of a body hanging off chain link `parent`: the chain and the
/// link's FK segment index, or `None` if `parent` is not a chain link.
fn chain_segment(names: &[Vec<String>], parent: &str) -> Option<Placement> {
    names.iter().enumerate().find_map(|(chain, links)| {
        links
            .iter()
            .position(|n| n == parent)
            .map(|segment| Placement::Chain { chain, segment })
    })
}

fn link_poses<const N: usize>(chain: &Chain<N>, q: &[f64; N]) -> [Isometry3<f64>; N] {
    let posed = chain.at(q);
    std::array::from_fn(|i| posed.link_pose_world(i))
}

/// Per-body, per-joint surface-speed bounds (m/rad). A point rigidly attached
/// distal of revolute joint `j` moves at most `r * |dq_j|`, with `r` its distance
/// from the joint axis; that distance is bounded, over all poses, by the chain
/// hops from joint `j`'s origin out to the body's link origin (each hop's norm is
/// pose invariant: a rigid link separates consecutive joint origins) plus the
/// body's own reach: bounding-sphere center offset + radius, and for a finger the
/// worst translation its joint offset can add across the travel (a prismatic
/// slide adds the full travel; a revolute offset only rotates about its origin).
/// Rows are zero for fixed bodies and for joints distal of the body.
fn body_reaches<const N: usize>(chains: &[Chain<N>], bodies: &[Body]) -> Vec<[f64; N]> {
    let hops: Vec<Vec<f64>> = chains
        .iter()
        .map(|chain| {
            let poses = link_poses(chain, &[0.0; N]);
            (0..N - 1)
                .map(|k| (poses[k + 1].translation.vector - poses[k].translation.vector).norm())
                .collect()
        })
        .collect();

    bodies
        .iter()
        .map(|body| {
            let Placement::Chain { chain, segment } = body.placement else {
                return [0.0; N]; // never moved by a joint
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
                if j <= segment {
                    hops[chain][j..segment].iter().sum::<f64>() + reach
                } else {
                    0.0 // a joint distal of the body does not move it
                }
            })
        })
        .collect()
}

/// Reject NaN/inf joint values so queries fail safe instead of comparing
/// against NaN downstream.
fn ensure_finite<const N: usize, const K: usize>(q: &Joints<N, K>) -> Result<(), CollisionError> {
    if q.iter().flatten().any(|x| !x.is_finite()) {
        return Err(CollisionError::NonFinite);
    }
    Ok(())
}

#[cfg(test)]
impl<const N: usize, const K: usize> SelfCollisionModel<N, K> {
    /// Keep the checked pairs whose two bodies' names `keep` takes, in either order.
    fn retain_pairs(&mut self, keep: impl Fn(&str, &str) -> bool) {
        let kept: Vec<Pair> = self
            .pairs
            .iter()
            .filter(|pair| {
                let (a, b) = (&self.bodies[pair.a].name, &self.bodies[pair.b].name);
                keep(a, b) || keep(b, a)
            })
            .cloned()
            .collect();
        self.set_pairs(kept);
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;
    use collision_geometry::GeometryError;

    use crate::fixture::openarm::{ARM_DOF, MODEL, Model, model};
    use crate::fixture::{description, edited, meshes};

    type Q = [f64; ARM_DOF];

    /// The fixture's model built from `description`, or why it is refused.
    fn built(description: &RobotDescription) -> Result<Model, CollisionError> {
        SelfCollisionModel::from_description(description, &meshes(MODEL))
    }

    /// Check every analytic gradient column (both arms' joints and both opening
    /// fractions) against central differences at one configuration, with the
    /// grippers at mid-travel 0.6 so the finger bodies participate with a
    /// nontrivial live offset (the opening is a constant across each FD
    /// perturbation, exactly as it is across one governor tick).
    fn assert_gradient_matches_finite_difference(m: &mut Model, q: &Joints<ARM_DOF, 2>) {
        let h = 1e-5;
        m.set_gripper_openings(&[0.6, 0.6]);
        let grad = m.distance_gradient(q).expect("gradient defined");
        let (analytic_joints, analytic_openings) = (grad.joints, grad.openings);
        // Opening columns against central differences on the fractions, the
        // same envelope-theorem check as the joints below.
        for s in 0..2 {
            let probe = |m: &mut Model, frac: f64| -> f64 {
                let mut o = [0.6, 0.6];
                o[s] = frac;
                m.set_gripper_openings(&o);
                let d = m.min_distance(q).unwrap().distance;
                m.set_gripper_openings(&[0.6, 0.6]);
                d
            };
            let fd = (probe(m, 0.6 + h) - probe(m, 0.6 - h)) / (2.0 * h);
            assert!(
                (analytic_openings[s] - fd).abs() < 3e-3,
                "opening {s}: analytic {} fd {fd}",
                analytic_openings[s]
            );
        }
        for chain in 0..2 {
            for j in 0..ARM_DOF {
                let (mut plus, mut minus) = (*q, *q);
                plus[chain][j] += h;
                minus[chain][j] -= h;
                let fd = (m.min_distance(&plus).unwrap().distance
                    - m.min_distance(&minus).unwrap().distance)
                    / (2.0 * h);
                assert!(
                    (analytic_joints[chain][j] - fd).abs() < 3e-3,
                    "chain {chain} j{j}: analytic {} fd {fd}",
                    analytic_joints[chain][j]
                );
            }
        }
    }

    // A clone is a model of its own: it answers a query as the model it was
    // cloned from, and the gripper openings set on it leave the original's.
    // The configuration has a finger nearest, so its answer moves with the
    // openings.
    #[test]
    fn a_clone_answers_as_its_original_and_keeps_its_own_openings() {
        let q = [
            [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
            [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
        ];
        let nearest = |m: &mut Model| {
            let p = m.min_distance(&q).expect("query");
            (p.distance, p.link_a.to_string(), p.link_b.to_string())
        };
        let mut original = model();
        original.set_gripper_openings(&[0.2, 0.2]);
        let mut clone = original.clone();
        let before = nearest(&mut original);
        assert_eq!(nearest(&mut clone), before);
        clone.set_gripper_openings(&[1.0, 1.0]);
        assert_ne!(nearest(&mut clone), before, "the openings move the answer");
        assert_eq!(nearest(&mut original), before);
    }

    #[test]
    fn distance_gradient_matches_finite_difference_in_penetration() {
        let mut m = model();
        m.set_gripper_openings(&[0.6, 0.6]);
        // Wrists folded inward but ASYMMETRICALLY, so one moving cross-arm pair is
        // unambiguously nearest (a symmetric pose sits on a pair-switch tie where
        // the analytic gradient and a straddling central difference legitimately
        // disagree). Every config penetrates the model (asserted below), covering
        // deep EPA; the companion test below walks through contact. The last config
        // has a finger body nearest.
        let configs: [Joints<ARM_DOF, 2>; 4] = [
            [
                [0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0],
                [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0],
            ],
            [
                [0.0, 0.3, 0.95, 0.45, 0.1, 0.0, 0.0],
                [0.0, -0.1, -0.55, 0.4, 0.0, 0.1, 0.0],
            ],
            [
                [0.25, -0.1, 1.0, 0.65, 0.0, 0.2, 0.1],
                [-0.1, 0.05, -1.1, 0.3, 0.0, -0.2, 0.0],
            ],
            [
                [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
                [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
            ],
        ];
        {
            let p = m.min_distance(&configs[3]).expect("query");
            assert!(
                p.link_a.contains("finger") || p.link_b.contains("finger"),
                "setup: the finger config's nearest pair should involve a finger, got {} vs {}",
                p.link_a,
                p.link_b
            );
        }
        for q in configs {
            let d = m.min_distance(&q).expect("query").distance;
            assert!(d < 0.0, "setup: expected a penetrating config, got d={d}");
            assert_gradient_matches_finite_difference(&mut m, &q);
        }
    }

    #[test]
    fn distance_gradient_matches_finite_difference_across_contact() {
        // The test above sees deep penetration alone. Restrict the pairs to
        // cross-arm wrist and finger bodies and walk an asymmetric
        // wrists-inward family from clearance through contact into shallow
        // overlap, the regimes where the witness separation reverses sense.
        let wrist = ["link6", "link7", "left_finger", "right_finger"];
        let of_the_wrist = |body: &str, arm: &str| {
            wrist
                .iter()
                .any(|link| body == format!("openarm_{arm}_{link}"))
        };
        let mut m = model();
        m.retain_pairs(|a, b| of_the_wrist(a, "left") && of_the_wrist(b, "right"));
        assert_eq!(
            m.checked_pairs().len(),
            wrist.len() * wrist.len(),
            "setup: every cross-arm pair of the wrists is checked"
        );
        m.set_gripper_openings(&[0.6, 0.6]);
        let pose_at = |t: f64| -> Joints<ARM_DOF, 2> {
            [
                [0.1, 0.05, t, 0.45, -0.1, 0.05, 0.0],
                [-0.05, -0.1, -t - 0.08, 0.4, 0.1, -0.05, 0.0],
            ]
        };
        let (mut separated, mut penetrating) = (0, 0);
        let mut finger_pair_covered = false;
        for i in 0..=60 {
            let q = pose_at(i as f64 * 0.02);
            let p = m.min_distance(&q).expect("query");
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
            assert_gradient_matches_finite_difference(&mut m, &q);
        }
        assert!(
            separated >= 3 && penetrating >= 3 && finger_pair_covered,
            "setup: expected configs on both sides of contact with a finger pair nearest, \
             got {separated} separated / {penetrating} penetrating (finger: {finger_pair_covered})"
        );
    }

    #[test]
    fn rejects_querying_with_no_pairs() {
        let mut empty = model();
        empty.retain_pairs(|_, _| false);
        assert!(empty.min_distance(&[[0.0; ARM_DOF]; 2]).is_err());
    }

    #[test]
    fn rejects_chains_that_share_a_base() {
        let from_the_torso = edited(MODEL, str::to_string, |srdf| {
            srdf.replace(
                "openarm_left_link0\" tip_link",
                "openarm_body_link0\" tip_link",
            )
            .replace(
                "openarm_right_link0\" tip_link",
                "openarm_body_link0\" tip_link",
            )
        })
        .expect("arms from the torso describe the robot");
        let e = built(&from_the_torso)
            .err()
            .expect("a shared base must fail");
        assert!(
            matches!(
                &e,
                CollisionError::Build(BuildError::SharedBase { first, second, base })
                    if (first.as_str(), second.as_str(), base.as_str())
                        == ("left_arm", "right_arm", "openarm_body_link0")
            ),
            "{e}"
        );
    }

    // A description of other than the model's arms, or with an arm of other than its
    // joints, is refused, naming the counts.
    #[test]
    fn rejects_a_description_of_another_shape() {
        let one_arm =
            SelfCollisionModel::<ARM_DOF, 1>::from_description(&description(MODEL), &meshes(MODEL))
                .err()
                .expect("two arms are not one");
        assert!(
            matches!(
                &one_arm,
                CollisionError::Build(BuildError::Arms {
                    expected: 1,
                    found: 2
                })
            ),
            "{one_arm}"
        );
        let six = SelfCollisionModel::<6, 2>::from_description(&description(MODEL), &meshes(MODEL))
            .err()
            .expect("seven joints are not six");
        assert!(
            matches!(&six, CollisionError::Build(BuildError::ArmJoints { arm, expected: 6, found: 7 }) if arm == "left_arm"),
            "{six}"
        );
    }

    // Every link on a moving joint off a chain link that has collision geometry is a
    // finger of its arm's gripper, and the grippers name no other.
    #[test]
    fn the_fingers_are_the_grippers_fingers() {
        let description = description(MODEL);
        let all: Vec<Finger> = description
            .arms()
            .iter()
            .flat_map(|arm| arm.gripper.fingers.clone())
            .collect();
        let refused = |fingers: &[Finger]| {
            let fingers: Vec<&Finger> = fingers.iter().collect();
            Model::assembled_with_fingers(&description, &meshes(MODEL), &fingers)
                .err()
                .expect("the fingers are refused")
        };
        let e = refused(&all[..3]);
        assert!(
            matches!(&e, BuildError::UnnamedFinger { link } if *link == all[3].link),
            "{e}"
        );
        let mut extra = all.clone();
        extra.push(Finger {
            joint: "openarm_left_joint7".to_string(),
            link: "openarm_left_link7".to_string(),
            closed: 0.0,
            open: 1.0,
            mimic: None,
        });
        let e = refused(&extra);
        assert!(
            matches!(&e, BuildError::NoSuchFinger { link } if link == "openarm_left_link7"),
            "{e}"
        );
    }

    #[test]
    fn each_collision_element_is_one_piece() {
        let m = model();
        assert_eq!(
            m.local_hulls("openarm_body_link0").expect("torso").len(),
            8,
            "the torso's eight elements"
        );
        assert_eq!(
            m.local_hulls("openarm_left_link7").expect("gripper").len(),
            2,
            "link7's element and its hand's"
        );
    }

    #[test]
    fn fingers_are_their_own_bodies_not_baked_into_the_wrist() {
        // Each gripper finger is fit as its own single-hull body (in its finger
        // frame); the wrist hull covers only the wrist mesh and fixed children.
        let m = model();
        let description = description(MODEL);
        for finger in description
            .arms()
            .iter()
            .flat_map(|arm| &arm.gripper.fingers)
        {
            assert_eq!(
                m.local_hulls(&finger.link)
                    .unwrap_or_else(|| panic!("finger body {} missing", finger.link))
                    .len(),
                1,
                "{} should be one auto-fit hull",
                finger.link
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
    fn rejects_a_collision_link_on_a_movable_joint_off_the_body() {
        let flap = r#"<link name="flap">
              <collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision>
            </link>
            <joint name="flap_joint" type="revolute">
              <parent link="openarm_body_link0"/><child link="flap"/><axis xyz="0 0 1"/>
              <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
          </robot>"#;
        let with_a_flap = edited(
            MODEL,
            |urdf| urdf.replacen("</robot>", flap, 1),
            str::to_string,
        )
        .expect("the flap describes the robot");
        let e = built(&with_a_flap)
            .err()
            .expect("a link no chain carries on a movable joint must fail");
        assert!(
            matches!(
                &e,
                CollisionError::Build(BuildError::Geometry(reason))
                    if reason.starts_with(
                        "collision link 'flap' is neither a chain link, an attached child, nor world-fixed: "
                    )
            ),
            "{e}"
        );
    }

    #[test]
    fn a_world_fixed_link_whose_fit_fails_answers_the_fit() {
        let plate = r#"<link name="plate">
              <collision><geometry><mesh filename="package://none/no_such_mesh.stl"/></geometry></collision>
            </link>
            <joint name="plate_joint" type="fixed">
              <parent link="openarm_body_link0"/><child link="plate"/>
            </joint>
          </robot>"#;
        let with_a_plate = edited(
            MODEL,
            |urdf| urdf.replacen("</robot>", plate, 1),
            str::to_string,
        )
        .expect("the plate describes the robot");
        let e = built(&with_a_plate)
            .err()
            .expect("a world-fixed link with no mesh to read must fail");
        assert!(
            matches!(
                &e,
                CollisionError::Build(BuildError::Fit(GeometryError::Geometry(_)))
            ),
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
        let mut m = model();
        let poses: [Joints<ARM_DOF, 2>; 3] = [
            [
                [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
            ],
            [
                [0.15, 0.1, 0.85, 0.5, -0.2, 0.1, 0.0],
                [-0.05, -0.25, -0.45, 0.35, 0.1, -0.1, 0.0],
            ],
            [
                [0.0, 0.0, 0.95, 0.4, 0.1, 0.0, 0.2],
                [0.0, 0.0, -1.05, 0.4, -0.1, 0.1, 0.0],
            ],
        ];
        // A deterministic spread of step directions: single joints, all joints,
        // and mixed-sign combinations, at a small and a large magnitude.
        let dirs: Vec<Joints<ARM_DOF, 2>> = {
            let mut d: Vec<Joints<ARM_DOF, 2>> = Vec::new();
            for j in 0..ARM_DOF {
                let mut l = [0.0; ARM_DOF];
                l[j] = 1.0;
                d.push([l, [0.0; ARM_DOF]]);
                d.push([[0.0; ARM_DOF], l]);
            }
            d.push([[1.0; ARM_DOF], [-1.0; ARM_DOF]]);
            d.push([
                std::array::from_fn(|i| if i % 2 == 0 { 1.0 } else { -1.0 }),
                std::array::from_fn(|i| if i % 3 == 0 { -1.0 } else { 1.0 }),
            ]);
            d
        };
        // Opening deltas ride the same segment: the bound must dominate a step
        // that slides/swings the fingers too, not only the arm joints.
        for (opening, dopen) in [
            ([1.0, 1.0], [0.0, 0.0]),
            ([0.3, 0.8], [0.7, -0.8]),
            ([0.0, 0.0], [1.0, 1.0]),
        ] {
            for q in &poses {
                m.set_gripper_openings(&opening);
                let d0 = m.min_distance(q).expect("query").distance;
                for dir in &dirs {
                    for mag in [0.02, 0.2] {
                        let step: Joints<ARM_DOF, 2> =
                            std::array::from_fn(|k| std::array::from_fn(|i| dir[k][i] * mag));
                        let bound = m.clearance_step_bound(&step, &dopen);
                        for t in [0.25, 0.5, 1.0] {
                            let qt: Joints<ARM_DOF, 2> = std::array::from_fn(|k| {
                                std::array::from_fn(|i| q[k][i] + t * step[k][i])
                            });
                            m.set_gripper_openings(&std::array::from_fn(|k| {
                                opening[k] + t * dopen[k]
                            }));
                            let dt = m.min_distance(&qt).expect("query").distance;
                            assert!(
                                (dt - d0).abs() <= bound + 1e-9,
                                "step bound violated: |{dt:+.5} - {d0:+.5}| > {bound:.5} \
                                 (mag {mag}, t {t}, dopen {dopen:?})"
                            );
                        }
                        m.set_gripper_openings(&opening);
                    }
                }
            }
        }
    }

    #[test]
    fn step_bound_is_infinite_on_non_finite_deltas() {
        // The scan-skip predicate compares `margin > bound`; a NaN bound would
        // make that false by comparison semantics alone, which a caller must not
        // have to rely on. Bad deltas must yield an explicitly infinite bound.
        let m = model();
        let mut dq = [[0.0; ARM_DOF]; 2];
        dq[0][2] = f64::NAN;
        assert_eq!(m.clearance_step_bound(&dq, &[0.0, 0.0]), f64::INFINITY);
        let zero = [[0.0; ARM_DOF]; 2];
        assert_eq!(
            m.clearance_step_bound(&zero, &[f64::NAN, 0.0]),
            f64::INFINITY
        );
        assert_eq!(
            m.clearance_step_bound(&zero, &[0.0, f64::INFINITY]),
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
        let poses: [Joints<ARM_DOF, 2>; 3] = [
            [[0.0; ARM_DOF], [0.0; ARM_DOF]],
            [
                [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
            ],
            [
                [-0.4, -0.1, 0.0, 0.5, 0.0, -0.3, 0.0],
                [0.4, 0.1, 0.0, 0.7, 0.0, -0.2, 0.0],
            ],
        ];
        for q in poses {
            let fast = m.min_distance(&q).expect("query").distance;
            let placed: HashMap<&str, Vec<Placed>> =
                m.world_pieces(&q).expect("pieces").into_iter().collect();
            let slow = pairs.iter().fold(f64::INFINITY, |best, (a, b)| {
                placed[a.as_str()]
                    .iter()
                    .flat_map(|ha| {
                        placed[b.as_str()]
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

    #[test]
    fn overlapping_bodies_report_negative_distance() {
        // Grippers wrapped toward each other across the torso: the winner
        // overlaps, and EPA reports a negative depth.
        let mut m = model();
        let p = m
            .min_distance(&[
                [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
            ])
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
        let wrapped = |m: &mut Model, elbow: f64| {
            let (left, right): (Q, Q) = (
                [0.0, 0.0, elbow, 0.4, 0.0, 0.0, 0.0],
                [0.0, 0.0, -elbow, 0.4, 0.0, 0.0, 0.0],
            );
            m.min_distance(&[left, right]).expect("q").distance
        };
        let deep = wrapped(&mut m, 1.2);
        let shallow = wrapped(&mut m, 1.0);
        assert!(
            deep < 0.0 && shallow > deep,
            "deep {deep:+.4} shallow {shallow:+.4}"
        );
    }

    #[test]
    fn rejects_non_finite_queries() {
        let mut m = model();
        let mut bad = [[0.0; ARM_DOF]; 2];
        bad[0][0] = f64::NAN;
        assert!(m.min_distance(&bad).is_err());
        assert!(m.world_pieces(&bad).is_err());
    }

    #[test]
    fn model_is_send_for_task_ownership() {
        fn assert_send<T: Send>() {}
        assert_send::<Model>();
    }
}
