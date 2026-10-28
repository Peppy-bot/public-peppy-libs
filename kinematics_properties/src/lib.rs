//! Hegel generators for property tests over chains: robots of any serial shape and
//! mass, configurations inside a chain's limits, points, directions, rotations and
//! poses in space, and the tolerance the properties share.

use chain_kinematics::Limit;
use chain_kinematics::nalgebra::{
    Isometry3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3,
};
use hegel::TestCase;
use hegel::generators as gs;
use std::f64::consts::PI;
use urdf_rs::{Axis, Inertia, Inertial, JointLimit, JointType, LinkName, Mass, Pose, Robot, Vec3};

/// Largest miss a property allows, in meters or radians. A defect these
/// properties look for misses by far more.
pub const TOLERANCE: f64 = 1e-6;

/// An angle in the principal range, both ends included.
pub fn angles() -> gs::FloatGenerator<f64> {
    gs::floats().min_value(-PI).max_value(PI)
}

/// A unit vector, uniform on the sphere.
#[hegel::composite]
pub fn unit_vectors(tc: &TestCase) -> Vector3<f64> {
    let azimuth = tc.draw(angles());
    let z: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let ring = (1.0 - z * z).sqrt();
    Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z)
}

/// Any rotation: any angle about any axis.
#[hegel::composite]
pub fn rotations(tc: &TestCase) -> Rotation3<f64> {
    let axis = Unit::new_normalize(tc.draw(unit_vectors()));
    Rotation3::from_axis_angle(&axis, tc.draw(angles()))
}

/// A configuration inside `limits`, bounds included.
#[hegel::composite]
pub fn configs<const N: usize>(tc: &TestCase, limits: [Limit; N]) -> [f64; N] {
    limits.map(|limit| tc.draw(gs::floats().min_value(limit.lo).max_value(limit.hi)))
}

/// A point within a meter of the origin on each axis, an arm's workspace scale.
#[hegel::composite]
pub fn points(tc: &TestCase) -> Vector3<f64> {
    Vector3::from(tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0))))
}

/// A pose: a point with any rotation.
#[hegel::composite]
pub fn poses(tc: &TestCase) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(tc.draw(points())),
        UnitQuaternion::from_rotation_matrix(&tc.draw(rotations())),
    )
}

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

/// How a link carries its mass: up to 1 kg, centered on any frame of the link
/// [`placements`] draws, with principal axes along that frame. Each principal
/// moment is the sum of two of the mass's second moments about the center,
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
fn robots(tc: &TestCase, joints: usize) -> Robot {
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
