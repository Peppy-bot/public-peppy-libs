//! The arm a plan moves: its chain, the collision pieces of its links and of the parts
//! its links carry rigidly through a plan, how far any point of them moves per unit of
//! each joint, and the robot's self model where the robot has one.

use chain_kinematics::{JointKind, Posed};
use collision_geometry::chain::{ChainGeometry, Rider};
use collision_geometry::gjk::{Hull, Placed};
use nalgebra::Isometry3;

/// A robot's check of its own body: the smallest distance between the pairs of its
/// self model with the arm at a configuration, and the rest of the robot where it
/// stands. The walk of a motion bounds how far a pair comes nearer by how far the arm's
/// links and riders move: the bound holds for a pair each of whose links that the arm's
/// joints move is a link or a rider of the arm.
pub trait SelfModel<const N: usize>: Sync {
    /// The least distance between the pairs the model checks at `q`, in meters.
    fn clearance(&self, q: &[f64; N]) -> f64;
    /// The names of the two links of the pair nearest each other at `q`.
    fn nearest_pair(&self, q: &[f64; N]) -> (String, String);
}

/// A rider that the child link of no joint of the arm carries.
#[derive(Debug, PartialEq, thiserror::Error)]
#[error(
    "{link} rides the child link of joint {segment}, and the arm's joints run from 0 to {last}"
)]
pub struct RiderOffTheArm {
    pub link: String,
    pub segment: usize,
    pub last: usize,
}

/// The arm of a plan.
pub struct Arm<'a, const N: usize> {
    geometry: &'a ChainGeometry<N>,
    riders: &'a [Rider<'a>],
    reach: [f64; N],
    self_model: Option<&'a dyn SelfModel<N>>,
}

impl<'a, const N: usize> Arm<'a, N> {
    /// The arm whose links `geometry` holds, checked against `self_model` when the
    /// robot has one.
    pub fn new(geometry: &'a ChainGeometry<N>, self_model: Option<&'a dyn SelfModel<N>>) -> Self {
        Self {
            geometry,
            riders: &[],
            reach: reach_of(geometry, &[]),
            self_model,
        }
    }

    /// The arm with its links carrying `riders`, which every clearance check reads and
    /// its reach covers; a rider on no joint of the arm is refused.
    pub fn riding(self, riders: &'a [Rider<'a>]) -> Result<Self, RiderOffTheArm> {
        if let Some(rider) = riders.iter().find(|rider| rider.segment >= N) {
            return Err(RiderOffTheArm {
                link: rider.link.to_string(),
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

    /// Every piece of the arm's links and of its riders, placed with the arm at `q` in
    /// the frame of the URDF root, with the name of its link.
    pub fn pieces(&self, q: &[f64; N]) -> Vec<(&'a str, Placed<'a>)> {
        let posed = self.geometry.chain().at(q);
        self.geometry
            .place_at(&posed)
            .pieces()
            .chain(self.riders.iter().flat_map(|rider| {
                let pose = rider_pose(&posed, rider);
                rider
                    .pieces
                    .iter()
                    .map(move |piece| (rider.link, Placed::new(piece, pose)))
            }))
            .collect()
    }

    pub fn geometry(&self) -> &ChainGeometry<N> {
        self.geometry
    }

    pub fn self_model(&self) -> Option<&dyn SelfModel<N>> {
        self.self_model
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

/// Where the frame `offset` in the frame of the child link of joint `segment` stands in
/// the frame of the URDF root with the chain posed at `posed`: a rider's frame, and the
/// frame of a goal on a link or on a part a link carries.
pub(crate) fn frame_on_segment<const N: usize>(
    posed: &Posed<N>,
    segment: usize,
    offset: &Isometry3<f64>,
) -> Isometry3<f64> {
    posed.link_pose_world(segment) * offset
}

/// Where `rider` stands in the frame of the URDF root with its chain posed at `posed`.
fn rider_pose<const N: usize>(posed: &Posed<N>, rider: &Rider) -> Isometry3<f64> {
    frame_on_segment(posed, rider.segment, &rider.offset)
}

/// For each joint of `geometry`'s chain, the farthest any point of a link it moves, or of
/// one of `riders`, lies from it, over every configuration: a revolute joint turning by an
/// angle moves a point at most that angle times the point's distance from the joint's
/// origin. That distance is at most the lengths between the joint origins from the joint
/// to the link's own joint, plus the farthest point of the link's pieces, or the rider's,
/// from its joint's origin. The length from one joint's origin to the next one's is the
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
    // The farthest point of `hull`, placed at `pose` with the chain at 0, from the origin
    // of joint `segment`, whose link carries it.
    let extent = |segment: usize, pose: &Isometry3<f64>, hull: &Hull| {
        ((pose * hull.center()).coords - origin(segment)).norm() + hull.bound_radius()
    };
    let placed = geometry.place(&[0.0; N]);
    // Each piece's joint, and the farthest point of the piece from that joint's origin.
    let links = placed.links().flat_map(|(link, pose)| {
        let segment = link.segment();
        link.pieces()
            .iter()
            .map(move |piece| (segment, extent(segment, pose, piece.hull())))
    });
    let carried = riders.iter().flat_map(|rider| {
        let pose = rider_pose(&posed, rider);
        rider
            .pieces
            .iter()
            .map(move |hull| (rider.segment, extent(rider.segment, &pose, hull)))
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
