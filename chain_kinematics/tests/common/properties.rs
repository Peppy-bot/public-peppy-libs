//! The Hegel generators this crate's property tests share beyond
//! `kinematics_properties`: robots of any serial shape and mass, their chains
//! with the joints in any order, and poses past a chain's reach.

use chain_kinematics::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use chain_kinematics::{ACCEPT_POSITION_M, Chain, ChainSpec, JointSelection, Limit};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, angles, rotations, unit_vectors};
use std::f64::consts::PI;
use urdf_rs::{Axis, Inertia, Inertial, JointLimit, JointType, LinkName, Mass, Pose, Robot, Vec3};

/// The link every drawn robot ends in.
pub const TIP: &str = "tip";

/// The name of a drawn robot's actuated joint `i`, counted from 1 at the root.
pub fn joint_name(i: usize) -> String {
    format!("joint_{i}")
}

/// The name of a drawn robot's link `i`: the root is link 0, and joint `i`
/// moves link `i`.
pub fn link_name(i: usize) -> String {
    format!("link_{i}")
}

/// A frame on a link: any offset within 0.2 m along each axis, any
/// orientation.
#[hegel::composite]
fn placements(tc: &TestCase) -> Pose {
    Pose {
        xyz: Vec3(tc.draw(gs::arrays(gs::floats().min_value(-0.2).max_value(0.2)))),
        rpy: Vec3(tc.draw(gs::arrays(angles()))),
    }
}

/// How a link carries its mass: up to 1 kg, centred on any frame of the link
/// [`placements`] draws, with principal axes along that frame. Each principal
/// moment is the sum of two of the mass's second moments about the centre,
/// each up to 0.1 kg m², so the moments are positive and meet the triangle
/// inequality, as a rigid body's do.
#[hegel::composite]
fn inertials(tc: &TestCase) -> Inertial {
    let [x, y, z]: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(1e-4).max_value(0.1)));
    Inertial {
        origin: tc.draw(placements()),
        mass: Mass {
            value: tc.draw(gs::floats().min_value(1e-3).max_value(1.0)),
        },
        inertia: Inertia {
            ixx: y + z,
            iyy: x + z,
            izz: x + y,
            ixy: 0.0,
            ixz: 0.0,
            iyz: 0.0,
        },
    }
}

/// A robot whose links form one serial chain of `joints` actuated joints, then
/// a fixed [`TIP`]. Each joint is revolute or prismatic, placed anywhere on its
/// parent, about any axis, with any range: within a turn for a revolute joint,
/// within 0.2 m for a prismatic one. Each link carries its own [`inertials`].
#[hegel::composite]
pub fn robots(tc: &TestCase, joints: usize) -> Robot {
    let link = |i: usize| LinkName { link: link_name(i) };
    let actuated: Vec<urdf_rs::Joint> = (1..=joints)
        .map(|i| {
            let (joint_type, span) = if tc.draw(gs::booleans()) {
                (JointType::Revolute, PI)
            } else {
                (JointType::Prismatic, 0.2)
            };
            let ends = (
                tc.draw(gs::floats().min_value(-span).max_value(span)),
                tc.draw(gs::floats().min_value(-span).max_value(span)),
            );
            tc.assume(ends.0 != ends.1);
            urdf_rs::Joint {
                name: joint_name(i),
                joint_type,
                origin: tc.draw(placements()),
                parent: link(i - 1),
                child: link(i),
                axis: Axis {
                    xyz: Vec3(tc.draw(unit_vectors()).into()),
                },
                limit: JointLimit {
                    lower: ends.0.min(ends.1),
                    upper: ends.0.max(ends.1),
                    effort: 1.0,
                    velocity: 1.0,
                    ..JointLimit::default()
                },
                calibration: None,
                dynamics: None,
                mimic: None,
                safety_controller: None,
            }
        })
        .collect();
    let tip = urdf_rs::Joint {
        name: "tip_joint".to_string(),
        joint_type: JointType::Fixed,
        origin: tc.draw(placements()),
        parent: link(joints),
        child: LinkName {
            link: TIP.to_string(),
        },
        axis: Axis::default(),
        limit: JointLimit::default(),
        calibration: None,
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };
    Robot {
        name: "drawn".to_string(),
        version: None,
        links: (0..=joints)
            .map(link_name)
            .chain(std::iter::once(TIP.to_string()))
            .map(|name| urdf_rs::Link {
                name,
                inertial: tc.draw(inertials()),
                visual: Vec::new(),
                collision: Vec::new(),
            })
            .collect(),
        joints: actuated.into_iter().chain(std::iter::once(tip)).collect(),
        materials: Vec::new(),
    }
}

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

/// A property of every chain, whatever its joint count.
pub trait ChainProperty {
    fn check<const N: usize>(tc: &TestCase, robot: &Robot);
}

/// Checks `P` on a robot drawn with 1 to 7 actuated joints. The joint count is
/// a type parameter of a chain, so each count is its own instantiation.
pub fn on_a_drawn_robot<P: ChainProperty>(tc: &TestCase) {
    let joints = tc.draw(gs::integers::<usize>().min_value(1).max_value(7));
    let robot = tc.draw(robots(joints));
    match joints {
        1 => P::check::<1>(tc, &robot),
        2 => P::check::<2>(tc, &robot),
        3 => P::check::<3>(tc, &robot),
        4 => P::check::<4>(tc, &robot),
        5 => P::check::<5>(tc, &robot),
        6 => P::check::<6>(tc, &robot),
        7 => P::check::<7>(tc, &robot),
        _ => unreachable!("joint counts are drawn from 1 to 7"),
    }
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
