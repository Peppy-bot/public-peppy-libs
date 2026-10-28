//! The Hegel generators this crate's property tests share beyond
//! `kinematics_properties`: the chains of a drawn robot with the joints in any
//! order, and poses past a chain's reach.

use chain_kinematics::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use chain_kinematics::{ACCEPT_POSITION_M, Chain, ChainSpec, JointSelection, Limit};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TIP, TOLERANCE, joint_name, rotations, unit_vectors};
use urdf_rs::Robot;

/// The chain from a drawn robot's root to its tip, joints in path order.
pub fn chain_of<const N: usize>(robot: &Robot) -> Chain<N> {
    chain_ending_at::<N>(robot, TIP)
}

/// The chain from a drawn robot's root to `tip_link`, a link at or past the one
/// its last joint moves, joints in path order.
pub fn chain_ending_at<const N: usize>(robot: &Robot, tip_link: &str) -> Chain<N> {
    Chain::<N>::from_urdf(
        robot,
        &ChainSpec {
            base_link: None,
            tip_link,
            joints: JointSelection::PathOrder,
        },
    )
    .expect("a drawn robot is a valid chain")
}

/// An order of `len` items: a permutation of `0..len`, shrinking toward the
/// identity.
#[hegel::composite]
pub fn permutations(tc: &TestCase, len: usize) -> Vec<usize> {
    (1..len)
        .rev()
        .fold((0..len).collect(), |mut order: Vec<usize>, last| {
            let back: usize = tc.draw(gs::integers().min_value(0).max_value(last));
            order.swap(last, last - back);
            order
        })
}

/// The chain from a drawn robot's root to its tip, its joints named in `order`:
/// entry `i` of `q` drives the joint `order[i]` places along the path, counted
/// from 0 at the root.
pub fn named_chain_of<const N: usize>(robot: &Robot, order: &[usize]) -> Chain<N> {
    let names: Vec<String> = order.iter().map(|&joint| joint_name(joint + 1)).collect();
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    Chain::<N>::from_urdf(
        robot,
        &ChainSpec {
            base_link: None,
            tip_link: TIP,
            joints: JointSelection::Named(&names),
        },
    )
    .expect("every drawn joint is on the path")
}

/// How far the end effector can get from the base origin, at most, in the base
/// frame the chain reports poses in. A revolute joint turns everything past it
/// about a point on its axis, so the spacing between that point and the next is
/// fixed; a prismatic joint stretches it by at most its travel. The spacing at
/// the zero configuration plus every prismatic travel bounds the reach.
pub fn reach_bound<const N: usize>(chain: &Chain<N>) -> f64 {
    let posed = chain.at(&[0.0; N]);
    let points: Vec<Vector3<f64>> = std::iter::once(Vector3::zeros())
        .chain((0..N).map(|joint| posed.origin_base(joint)))
        .chain(std::iter::once(posed.ee_pose().translation.vector))
        .collect();
    let spacing: f64 = points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum();
    let travel: f64 = (0..N)
        .filter(|&joint| {
            matches!(
                posed.kind(joint),
                chain_kinematics::JointKind::Prismatic { .. }
            )
        })
        .map(|joint| {
            let Limit { lo, hi } = chain.limits()[joint];
            lo.abs().max(hi.abs())
        })
        .sum();
    spacing + travel
}

/// A pose no configuration reaches within the acceptance bar, in the world frame
/// goals are given in: any orientation, at a point past the chain's reach bound
/// by the bar and [`TOLERANCE`], out to a reach beyond that, from the base
/// origin.
#[hegel::composite]
pub fn unreachable_poses<const N: usize>(tc: &TestCase, chain: &Chain<N>) -> Isometry3<f64> {
    let reach = reach_bound(chain);
    let nearest = reach + ACCEPT_POSITION_M + TOLERANCE;
    let distance: f64 = tc.draw(gs::floats().min_value(nearest).max_value(nearest + reach));
    let position = tc.draw(unit_vectors()) * distance;
    let rotation = UnitQuaternion::from_rotation_matrix(&tc.draw(rotations()));
    chain.world_pose(&Isometry3::from_parts(
        Translation3::from(position),
        rotation,
    ))
}
