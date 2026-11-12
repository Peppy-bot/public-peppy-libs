//! The fixture the tests share (included via `#[path]`): each robot model
//! robot_description bundles, by its model id, with its description, the collision meshes
//! its URDF names, its self model, its postures and its arms' chains; the measure of a
//! pair of bodies over the joints between them; the run of a check on every bundled
//! model, at the shape of its self model; and OpenArm, whose v1 is the model of the unit
//! tests and the two-arm scenarios.

#![allow(dead_code)]

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use collision_geometry::gjk;
use robot_description::{
    Arm, DescriptionError, OnShape, RobotDescription, on_shape, states, urdf_rs,
};
use self_collision_model::{Joints, SelfCollisionModel};

/// The description of the model `id`.
pub fn description(id: &str) -> RobotDescription {
    bundled(id).description()
}

/// The description of the model `id`, its URDF and its SRDF edited by `urdf` and `srdf`.
pub fn edited(
    id: &str,
    urdf: impl FnOnce(&str) -> String,
    srdf: impl FnOnce(&str) -> String,
) -> Result<RobotDescription, DescriptionError> {
    let bundled = bundled(id);
    RobotDescription::parse(&urdf(bundled.urdf()), &srdf(bundled.srdf()))
}

fn bundled(id: &str) -> &'static robot_description::Model {
    robot_description::model(id).unwrap_or_else(|error| panic!("{error}"))
}

/// The directory of the collision meshes the URDF of the model `id` names.
pub fn meshes(id: &str) -> String {
    bundled(id)
        .directory()
        .join("meshes")
        .to_str()
        .expect("the model's directory is UTF-8")
        .to_owned()
}

/// The self model of the model `id`, of `K` arms of `N` joints each.
pub fn model<const N: usize, const K: usize>(id: &str) -> SelfCollisionModel<N, K> {
    SelfCollisionModel::from_description(&description(id), &meshes(id))
        .unwrap_or_else(|error| panic!("the self model of {id}: {error}"))
}

/// Each arm's posture `name`, in the order of the description's arms.
pub fn posture<const N: usize, const K: usize>(
    description: &RobotDescription,
    name: &str,
) -> Joints<N, K> {
    std::array::from_fn(|index| {
        let group = &description.arms()[index].name;
        description
            .state(group, name)
            .unwrap_or_else(|| panic!("{group} has the posture {name}"))
            .try_into()
            .unwrap_or_else(|_| panic!("the posture {name} of {group} holds {N} joints"))
    })
}

/// The chain of `arm`, from its base link to its grasp point.
pub fn chain_spec(arm: &Arm) -> ChainSpec<'_> {
    ChainSpec {
        base_link: Some(&arm.base_link),
        tip_link: &arm.grasp_link,
        joints: JointSelection::PathOrder,
    }
}

/// The kinematic chain of `arm`.
pub fn arm_chain<const N: usize>(description: &RobotDescription, arm: &Arm) -> Chain<N> {
    Chain::from_urdf(description.robot(), &chain_spec(arm))
        .unwrap_or_else(|error| panic!("the chain of {}: {error}", arm.name))
}

/// The pieces of the chain of `arm` and of the parts it carries.
pub fn arm_geometry<const N: usize>(
    description: &RobotDescription,
    meshes: &str,
    arm: &Arm,
) -> ChainGeometry<N> {
    ChainGeometry::from_urdf(description.urdf(), meshes, &chain_spec(arm))
        .unwrap_or_else(|error| panic!("the pieces of {}: {error}", arm.name))
}

/// The links of the chain of `arm`, each the child of the joint of its index.
pub fn chain_links<const N: usize>(description: &RobotDescription, arm: &Arm) -> [String; N] {
    let chain = arm_chain::<N>(description, arm);
    let posed = chain.at(&[0.0; N]);
    std::array::from_fn(|joint| posed.link_name(joint).to_string())
}

/// The URDF's joint `name`.
pub fn joint<'d>(description: &'d RobotDescription, name: &str) -> &'d urdf_rs::Joint {
    description
        .robot()
        .joints
        .iter()
        .find(|joint| joint.name == name)
        .unwrap_or_else(|| panic!("the joint {name}"))
}

/// The link the joint `name` hangs its child off.
pub fn parent_link(description: &RobotDescription, name: &str) -> String {
    joint(description, name).parent.link.clone()
}

/// The fingers of every gripper, by their links.
pub fn fingers(description: &RobotDescription) -> Vec<String> {
    description
        .arms()
        .iter()
        .flat_map(|arm| &arm.gripper.fingers)
        .map(|finger| finger.link.clone())
        .collect()
}

/// The torso: the link the first arm's base link is fixed to.
pub fn torso(description: &RobotDescription) -> String {
    let base = &description.arms()[0].base_link;
    description
        .robot()
        .joints
        .iter()
        .find(|joint| joint.child.link == *base)
        .unwrap_or_else(|| panic!("{base} hangs on a joint"))
        .parent
        .link
        .clone()
}

/// The pair of `a` and `b`, in the order of their names.
pub fn unordered(a: &str, b: &str) -> (String, String) {
    let (first, second) = if a <= b { (a, b) } else { (b, a) };
    (first.to_string(), second.to_string())
}

/// The grid's step over each joint, in radians.
pub const GRID_STEP_RAD: f64 = 0.05;

/// The gripper openings on the grid, for a pair that holds a finger.
pub const OPENINGS: [f64; 3] = [0.0, 0.5, 1.0];

/// The most joints between two bodies the grid walks: its configurations grow as a
/// power of them.
pub const MOST_GRID_JOINTS: usize = 3;

/// The joints that turn the body `link` in the world, each by its arm and its index, and
/// whether the body is a finger, which its gripper's opening moves too. A fixed body turns
/// on none. `links` holds the links of each arm's chain ([`chain_links`]).
pub fn joints_moving<const N: usize>(
    description: &RobotDescription,
    links: &[[String; N]],
    link: &str,
) -> (Vec<(usize, usize)>, bool) {
    let finger = description
        .arms()
        .iter()
        .flat_map(|arm| &arm.gripper.fingers)
        .find(|finger| finger.link == link);
    let host = finger.map_or(link.to_string(), |finger| {
        parent_link(description, &finger.joint)
    });
    let joints = links
        .iter()
        .enumerate()
        .find_map(|(arm, names)| {
            let segment = names.iter().position(|name| *name == host)?;
            Some((0..=segment).map(|joint| (arm, joint)).collect())
        })
        .unwrap_or_default();
    (joints, finger.is_some())
}

/// The distance between the pieces of the bodies `a` and `b` at `q`.
pub fn distance_between<const N: usize, const K: usize>(
    model: &mut SelfCollisionModel<N, K>,
    q: &Joints<N, K>,
    a: &str,
    b: &str,
) -> f64 {
    let bodies = model.world_pieces(q).expect("a finite configuration");
    let pieces = |name: &str| {
        &bodies
            .iter()
            .find(|(body, _)| *body == name)
            .unwrap_or_else(|| panic!("the model holds {name}"))
            .1
    };
    pieces(a)
        .iter()
        .flat_map(|on_a| {
            pieces(b)
                .iter()
                .map(move |on_b| gjk::distance(on_a, on_b).distance)
        })
        .fold(f64::INFINITY, f64::min)
}

/// The points from `lo` to `hi`, both included, every [`GRID_STEP_RAD`] or finer: `lo`
/// alone when the two are one.
fn grid(lo: f64, hi: f64) -> impl Iterator<Item = f64> {
    let steps = ((hi - lo) / GRID_STEP_RAD).ceil() as u32;
    (0..=steps).map(move |step| lo + (hi - lo) * f64::from(step) / f64::from(steps.max(1)))
}

/// The least distance between the pieces of the bodies `a` and `b` of `model`, the self
/// model of `description`, over a grid of the joints between the two within their
/// limits, every [`GRID_STEP_RAD`] or finer, every other joint at home, at each of
/// [`OPENINGS`] where a finger is one of them. Two bodies with more than
/// [`MOST_GRID_JOINTS`] joints between them are refused.
pub fn least_distance_within_the_limits<const N: usize, const K: usize>(
    model: &mut SelfCollisionModel<N, K>,
    description: &RobotDescription,
    a: &str,
    b: &str,
) -> f64 {
    let arms = description.arms();
    let links: Vec<[String; N]> = arms
        .iter()
        .map(|arm| chain_links(description, arm))
        .collect();
    let limits: Vec<_> = arms
        .iter()
        .map(|arm| arm_chain::<N>(description, arm).limits())
        .collect();
    let (on_a, a_is_finger) = joints_moving(description, &links, a);
    let (on_b, b_is_finger) = joints_moving(description, &links, b);
    let between: Vec<(usize, usize)> = on_a
        .iter()
        .chain(&on_b)
        .filter(|joint| on_a.contains(joint) != on_b.contains(joint))
        .copied()
        .collect();
    assert!(
        between.len() <= MOST_GRID_JOINTS,
        "{a} and {b} turn against each other on {} joints; the grid walks at most \
         {MOST_GRID_JOINTS}",
        between.len()
    );
    let configurations = between.iter().fold(
        vec![posture::<N, K>(description, states::HOME)],
        |configurations, &(arm, joint)| {
            let limit = limits[arm][joint];
            configurations
                .iter()
                .flat_map(|q| {
                    grid(limit.lo, limit.hi).map(move |at| {
                        let mut q = *q;
                        q[arm][joint] = at;
                        q
                    })
                })
                .collect()
        },
    );
    // A pair of no finger stands alike at every opening, so one opening covers it.
    let openings = if a_is_finger || b_is_finger {
        &OPENINGS[..]
    } else {
        &OPENINGS[..1]
    };
    openings
        .iter()
        .flat_map(|&opening| configurations.iter().map(move |q| (opening, q)))
        .map(|(opening, q)| {
            model.set_gripper_openings(&[opening; K]);
            distance_between(model, q, a, b)
        })
        .fold(f64::INFINITY, f64::min)
}

/// Runs the check `check` makes of each model id on every model robot_description
/// bundles, at the shape of its self model ([`on_shape`]): a model of a shape no code is
/// compiled for fails, naming it.
pub fn check_every_model<C: OnShape<Output = ()>>(check: impl Fn(&'static str) -> C) {
    for bundled in robot_description::models() {
        let id = bundled.id();
        on_shape(&bundled.description(), check(id)).unwrap_or_else(|error| panic!("{id}: {error}"));
    }
}

/// OpenArm: the shape of the self model of each of its generations, and the generation
/// the unit tests and the two-arm scenarios build.
pub mod openarm {
    use self_collision_model::SelfCollisionModel;

    /// The joints of each arm.
    pub const ARM_DOF: usize = 7;

    /// The self model: both arms, left then right.
    pub type Model = SelfCollisionModel<ARM_DOF, 2>;

    /// The model id of the generation the unit tests and the two-arm scenarios build:
    /// OpenArm v1.
    pub const MODEL: &str = "openarm_v1";

    /// The self model of [`MODEL`].
    pub fn model() -> Model {
        super::model(MODEL)
    }
}
