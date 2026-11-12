//! Self distance against MoveIt's FCL: the pieces of each body of `SelfCollisionModel`
//! hold the collision meshes of its link, so the signed distance between the pieces of
//! two bodies is no larger than the distance between their meshes. The evidence: at every
//! configuration of every self distance scenario of the ROS reference harness, built from
//! the scenario's description, for each pair that the robot's model checks, every pair
//! of its bodies but those whose collisions the SRDF disables, the fingers' pairs
//! included, the distance between the pieces is no larger than FCL's distance between the
//! meshes, where FCL finds them apart. Where FCL finds the meshes in contact, its distance
//! is the depth of the deepest touching pair of triangles, which bounds neither solid's
//! depth; there the meshes overlap, so the pieces that hold them overlap or touch, and the
//! distance between them is no larger than zero. The model places a gripper's fingers at
//! an opening fraction, which the scenario's position of the gripper's finger joint gives:
//! from where the gripper's `closed` state puts the finger toward where its `open` state
//! puts it.

use collision_geometry::gjk;
use robot_description::{Arm, OnShape, RobotDescription, on_shape};
use ros_reference::self_distance::{Cases, Configuration};
use self_collision_model::{Joints, SelfCollisionModel};

/// The rounding of two distance computations, in meters.
const TOLERANCE: f64 = 1e-9;

/// The robots the scenarios pose, by their arms and the joints of each: OpenArm v2's two
/// arms of seven joints, and SO-101's one of five.
const OPENARM_V2: (usize, usize) = (2, 7);
const SO101: (usize, usize) = (1, 5);

/// The positions of the joints of `arm` that `configuration` sets, in the arm's order.
fn arm_joints<const N: usize>(configuration: &Configuration, arm: &Arm) -> [f64; N] {
    let positions: Vec<f64> = arm
        .joints
        .iter()
        .map(|joint| configuration.positions[joint])
        .collect();
    positions
        .try_into()
        .expect("a position for each joint of the arm")
}

/// The opening fraction at which `configuration` poses the gripper of `arm`: the position
/// it sets of a finger joint of the gripper, from where the gripper's `closed` state puts
/// that finger toward where its `open` state puts it.
fn opening(configuration: &Configuration, arm: &Arm) -> f64 {
    let (finger, position) = arm
        .gripper
        .fingers
        .iter()
        .find_map(|finger| Some((finger, *configuration.positions.get(&finger.joint)?)))
        .unwrap_or_else(|| panic!("the configuration poses a finger of {}", arm.gripper.name));
    (position - finger.closed) / (finger.open - finger.closed)
}

#[test]
fn each_checked_distance_is_no_larger_than_fcls() {
    let scenarios = Cases::load_all();
    let robots: Vec<(usize, usize)> = scenarios
        .iter()
        .map(|cases| robot_of(&cases.robot.description))
        .collect();
    assert!(
        robots.contains(&OPENARM_V2) && robots.contains(&SO101),
        "the harness holds a self distance scenario of OpenArm v2 and of SO-101"
    );
    for cases in &scenarios {
        on_shape(
            &cases.robot.description,
            EachDistanceIsNoLargerThanFcls(cases),
        )
        .unwrap_or_else(|error| panic!("{}: {error}", cases.name));
    }
}

/// The arms of `description` and the joints of its first.
fn robot_of(description: &RobotDescription) -> (usize, usize) {
    let arms = description.arms();
    (arms.len(), arms.first().map_or(0, |arm| arm.joints.len()))
}

/// Each checked pair of the self model of the scenario, a robot of `K` arms of `N` joints
/// each, at each of its configurations.
struct EachDistanceIsNoLargerThanFcls<'c>(&'c Cases);

impl OnShape for EachDistanceIsNoLargerThanFcls<'_> {
    type Output = ();

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) {
        let cases = self.0;
        let name = &cases.name;
        let description = &cases.robot.description;
        let meshes = cases
            .robot
            .meshes
            .to_str()
            .expect("the mesh directory is UTF-8");
        let mut model = SelfCollisionModel::<N, K>::from_description(description, meshes)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let arms: &[Arm; K] = description.arms().try_into().expect("a model of each arm");
        let pairs: Vec<(String, String)> = model
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| (a.to_owned(), b.to_owned()))
            .collect();
        let (mut apart, mut in_contact) = (0, 0);
        for (index, configuration) in cases.configurations.iter().enumerate() {
            model.set_gripper_openings(&arms.each_ref().map(|arm| opening(configuration, arm)));
            let joints: Joints<N, K> = arms.each_ref().map(|arm| arm_joints(configuration, arm));
            let bodies = model.world_pieces(&joints).expect("a finite configuration");
            let pieces_of = |body_name: &str| {
                &bodies
                    .iter()
                    .find(|(body, _)| *body == body_name)
                    .unwrap_or_else(|| panic!("the model holds {body_name}"))
                    .1
            };
            for (a, b) in &pairs {
                let ours = pieces_of(a)
                    .iter()
                    .flat_map(|piece_a| {
                        pieces_of(b)
                            .iter()
                            .map(move |piece_b| gjk::distance(piece_a, piece_b).distance)
                    })
                    .fold(f64::INFINITY, f64::min);
                let fcl = configuration
                    .distance(a, b)
                    .unwrap_or_else(|| panic!("{name}: FCL measures {a} and {b}"));
                if fcl > 0.0 {
                    apart += 1;
                } else {
                    in_contact += 1;
                }
                assert!(
                    ours <= fcl.max(0.0) + TOLERANCE,
                    "{name} configuration {index}, {a} and {b}: the model reads {ours} m, FCL \
                     {fcl} m"
                );
            }
        }
        assert!(
            apart > 0 && in_contact > 0,
            "{name} holds pairs apart and in contact: {apart} apart, {in_contact} in contact"
        );
    }
}
