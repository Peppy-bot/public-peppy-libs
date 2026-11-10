//! An arm a plan moves: its name, its chain, the collision pieces of its links and the
//! bodies its links carry rigidly through a plan, the parts of the robot such as its
//! fingers and the objects attached to its grasp link, and how far any point of them moves
//! per unit of each joint; and the robot's self model, which checks the arms of a plan
//! ([`Arms`](crate::Arms)).

use chain_kinematics::{Chain, JointKind, Posed};
use collision_geometry::chain::{ChainGeometry, Rider, RiderBody};
use collision_geometry::gjk::{self, Placed, Support};
use collision_geometry::shapes::PlacedShape;
use nalgebra::{Isometry3, Point3};

/// A robot's check of its own body: the smallest distance between the pairs of its
/// self model with the arms of a plan at a state, and the rest of the robot where it
/// stands. The walk of a motion bounds how far a pair comes nearer by how far the arms'
/// links and riders move: the bound holds for a pair each of whose links that the arms'
/// joints move is a link or a rider of an arm of the plan.
pub trait SelfModel<const N: usize>: Sync {
    /// The least distance at `q` between the pairs the model checks that `moving`
    /// judges, in meters.
    fn clearance(&self, q: &[f64; N], moving: Moving) -> f64;
    /// The names of the two links of the nearest of those pairs at `q`.
    fn nearest_pair(&self, q: &[f64; N], moving: Moving) -> (String, String);
}

/// The pairs a check of a state of a plan's group judges: every pair, or those of the
/// arm at a place in the group after the arms before it: each pair that holds a body of
/// that arm and none of an arm after it, against the rest of the robot, the world and
/// the arms before it.
#[derive(Clone, Copy, Debug)]
pub enum Moving {
    All,
    Arm(usize),
}

impl Moving {
    /// Whether a check judges a pair whose bodies ride the arms at `a` and `b` in the
    /// group; a pair of one arm's body and a body of no arm of the group rides `a` twice.
    pub fn judges(self, a: usize, b: usize) -> bool {
        match self {
            Self::All => true,
            Self::Arm(k) => a.max(b) == k,
        }
    }
}

/// A rider that the child link of no joint of the arm carries.
#[derive(Debug, PartialEq, thiserror::Error)]
#[error(
    "{name} rides the child link of joint {segment}, and the arm's joints run from 0 to {last}"
)]
pub struct RiderOffTheArm {
    pub name: String,
    pub segment: usize,
    pub last: usize,
}

/// A body of the arm placed at a configuration: a piece of a link or of a part a link
/// carries, or a shape of an object a link carries.
#[derive(Debug)]
pub enum Body<'a> {
    Piece(Placed<'a>),
    Shape(PlacedShape<'a>),
}

impl Body<'_> {
    /// The center and the radius of a sphere that holds the body.
    pub fn bounding_sphere(&self) -> (Point3<f64>, f64) {
        match self {
            Self::Piece(piece) => piece.bounding_sphere(),
            Self::Shape(shape) => shape.bounding_sphere(),
        }
    }

    /// The signed distance from the body to the convex `other`, which `sphere` holds:
    /// negative where they overlap, and where `other` lies wholly inside a mesh.
    pub fn distance_to_convex(&self, other: &impl Support, sphere: (Point3<f64>, f64)) -> f64 {
        match self {
            Self::Piece(piece) => gjk::distance(piece, other).distance,
            Self::Shape(shape) => shape.distance_to(other, sphere),
        }
    }

    /// The signed distance from the body to `other`, a shape of an object: negative
    /// where they overlap, and where one lies wholly inside the other's mesh.
    pub fn distance_to_shape(&self, other: &PlacedShape) -> f64 {
        match self {
            Self::Piece(piece) => other.distance_to(piece, piece.bounding_sphere()),
            Self::Shape(shape) => shape.distance_to_shape(other),
        }
    }

    /// The shape, for a shape of an object a link carries.
    pub fn shape(&self) -> Option<&PlacedShape<'_>> {
        match self {
            Self::Piece(_) => None,
            Self::Shape(shape) => Some(shape),
        }
    }
}

/// An arm of a plan.
#[derive(Clone)]
pub struct Arm<'a, const N: usize> {
    name: &'a str,
    geometry: &'a ChainGeometry<N>,
    riders: &'a [Rider<'a>],
    reach: [f64; N],
}

impl<'a, const N: usize> Arm<'a, N> {
    /// The arm `name`, as a message names it, whose links `geometry` holds.
    pub fn new(name: &'a str, geometry: &'a ChainGeometry<N>) -> Self {
        Self {
            name,
            geometry,
            riders: &[],
            reach: reach_of(geometry, &[]),
        }
    }

    /// The arm with its links carrying `riders`, which every clearance check reads and
    /// its reach covers; a rider on no joint of the arm is refused.
    pub fn riding(self, riders: &'a [Rider<'a>]) -> Result<Self, RiderOffTheArm> {
        if let Some(rider) = riders.iter().find(|rider| rider.segment >= N) {
            return Err(RiderOffTheArm {
                name: rider.name.to_string(),
                segment: rider.segment,
                last: N.saturating_sub(1),
            });
        }
        Ok(Self {
            riders,
            reach: reach_of(self.geometry, riders),
            ..self
        })
    }

    /// Every body of the arm, each piece of its links and each piece or shape of its
    /// riders, placed with the arm at `q` in the frame of the URDF root, with the name of
    /// its link or of its rider.
    pub fn bodies(&self, q: &[f64; N]) -> Vec<(&'a str, Body<'a>)> {
        let posed = self.geometry.chain().at(q);
        self.geometry
            .place_at(&posed)
            .pieces()
            .map(|(link, piece)| (link, Body::Piece(piece)))
            .chain(self.riders.iter().flat_map(|rider| {
                let pose = rider_pose(&posed, rider);
                rider_bodies(rider, pose).map(move |body| (rider.name, body))
            }))
            .collect()
    }

    /// Whether a link of the arm carries an object: a rider of a shape.
    pub fn carries_an_object(&self) -> bool {
        self.riders
            .iter()
            .any(|rider| matches!(rider.body, RiderBody::Shape(_)))
    }

    pub fn name(&self) -> &'a str {
        self.name
    }

    pub fn geometry(&self) -> &ChainGeometry<N> {
        self.geometry
    }

    /// The bodies the arm's links carry.
    pub(crate) fn riders(&self) -> &'a [Rider<'a>] {
        self.riders
    }

    /// For each joint, the farthest any point of a link or of a rider moves per unit the
    /// joint moves: per radian for a revolute joint, per meter for a prismatic one.
    pub fn reach(&self) -> &[f64; N] {
        &self.reach
    }

    /// A bound on how far any point of a link or of a rider moves on the straight joint
    /// motion from `from` to `to`.
    pub fn displacement(&self, from: &[f64; N], to: &[f64; N]) -> f64 {
        (0..N)
            .map(|i| (to[i] - from[i]).abs() * self.reach[i])
            .sum()
    }

    /// The farthest any point of a link or of a rider moves per unit of the joints' L2
    /// distance, along any joint motion: the L2 norm of the joints' reach.
    pub fn lever(&self) -> f64 {
        self.reach.iter().map(|r| r * r).sum::<f64>().sqrt()
    }

    /// The farthest any point of a link or of a rider moves per unit of a joint path's
    /// length along a stretch on which each joint's share of the path's unit tangent is
    /// at most `tangent_share`: the sum over the joints of the share times the joint's
    /// reach, and at most [`Arm::lever`].
    pub fn lever_along(&self, tangent_share: &[f64; N]) -> f64 {
        let along: f64 = (0..N).map(|i| tangent_share[i] * self.reach[i]).sum();
        along.min(self.lever())
    }
}

/// The grasp point of `chain`, its end effector, as a frame on the child link of its last
/// joint: where a goal of the grasp point and an object attached to the grasp link ride.
pub(crate) fn grasp_frame<const N: usize>(chain: &Chain<N>) -> Isometry3<f64> {
    let zero = [0.0; N];
    chain.at(&zero).link_pose_world(N - 1).inverse() * chain.world_ee_pose(&zero)
}

/// Where `rider` stands in the frame of the URDF root with its chain posed at `posed`.
pub(crate) fn rider_pose<const N: usize>(posed: &Posed<N>, rider: &Rider) -> Isometry3<f64> {
    posed.link_pose_world(rider.segment) * rider.offset
}

/// The bodies of `rider` standing at `pose` in the frame of the URDF root: each of a
/// part's pieces, or the object's shape.
fn rider_bodies<'a>(rider: &Rider<'a>, pose: Isometry3<f64>) -> impl Iterator<Item = Body<'a>> {
    let (pieces, shape) = match rider.body {
        RiderBody::Part(part) => (part.pieces(), None),
        RiderBody::Shape(shape) => (&[][..], Some(shape)),
    };
    pieces
        .iter()
        .map(move |piece| Body::Piece(Placed::new(piece, pose)))
        .chain(shape.map(|shape| Body::Shape(PlacedShape::new(shape, pose))))
}

/// For each joint of `geometry`'s chain, the farthest any point of a link it moves, or of
/// one of `riders`, lies from it, over every configuration: a revolute joint turning by an
/// angle moves a point at most that angle times the point's distance from the joint's
/// origin. That distance is at most the lengths between the joint origins from the joint
/// to the link's own joint, plus the farthest point of the link's pieces, or of the
/// rider's pieces or shape, from its joint's origin, as each one's bounding sphere bounds
/// it. The length from one joint's origin to the next one's is the
/// one at 0, and longer by up to the next joint's farthest limit where that joint is
/// prismatic, which moves its own origin along its axis. A prismatic joint moves every
/// point it carries by its own travel.
fn reach_of<const N: usize>(geometry: &ChainGeometry<N>, riders: &[Rider]) -> [f64; N] {
    let chain = geometry.chain();
    let posed = chain.at(&[0.0; N]);
    let origin = |j: usize| posed.origin_world(j);
    let limits = chain.limits();
    let prismatic = |j: usize| matches!(posed.kind(j), JointKind::Prismatic { .. });
    // The longest the distance from joint j's origin to joint j + 1's can be.
    let span: Vec<f64> = (0..N.saturating_sub(1))
        .map(|j| {
            let at_zero = (origin(j + 1) - origin(j)).norm();
            if prismatic(j + 1) {
                at_zero + limits[j + 1].lo.abs().max(limits[j + 1].hi.abs())
            } else {
                at_zero
            }
        })
        .collect();
    // The farthest point of `body`, placed with the chain at 0, from the origin of joint
    // `segment`, whose link carries it.
    let extent = |segment: usize, body: Body| {
        let (center, radius) = body.bounding_sphere();
        (center.coords - origin(segment)).norm() + radius
    };
    let placed = geometry.place(&[0.0; N]);
    // Each body's joint, and the farthest point of the body from that joint's origin.
    let links = placed.links().flat_map(|(link, pose)| {
        let segment = link.segment();
        link.pieces().iter().map(move |piece| {
            (
                segment,
                extent(segment, Body::Piece(Placed::new(piece.hull(), *pose))),
            )
        })
    });
    let carried = riders.iter().flat_map(|rider| {
        rider_bodies(rider, rider_pose(&posed, rider))
            .map(|body| (rider.segment, extent(rider.segment, body)))
    });
    let extents: Vec<(usize, f64)> = links.chain(carried).collect();
    std::array::from_fn(|i| {
        if prismatic(i) {
            return 1.0;
        }
        extents
            .iter()
            .filter(|(segment, _)| *segment >= i)
            .map(|&(segment, farthest)| span[i..segment].iter().sum::<f64>() + farthest)
            .fold(0.0, f64::max)
    })
}
