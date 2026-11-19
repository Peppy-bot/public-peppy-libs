//! The model's transforms on the bundled models: one per link but the root, and, down
//! each arm, the forward kinematics of chain_kinematics.

use std::collections::{HashMap, HashSet};

use chain_kinematics::nalgebra::Isometry3;
use chain_kinematics::{Chain, ChainSpec, JointSelection};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::configs;
use robot_description::states::{CLOSED, OPEN};
use robot_description::{Gripper, LinkTransform, RobotDescription, model};
use urdf_rs::JointType;

/// How far the transforms composed down an arm may stand from the arm's forward
/// kinematics, in meters and radians: rounding of the same compositions taken in
/// another grouping.
const TOLERANCE: f64 = 1e-12;

fn description(id: &str) -> RobotDescription {
    model(id).expect("a bundled model").description()
}

/// Every transform of `description` with each joint that moves of `positions`.
fn every_transform<'a>(
    description: &'a RobotDescription,
    positions: &HashMap<&str, f64>,
) -> Vec<LinkTransform<'a>> {
    let mut transforms = description.fixed_transforms();
    transforms.extend(description.transforms_at(positions));
    transforms
}

/// `link`'s pose in the frame of the root, composed up through `transforms`.
fn composed(transforms: &[LinkTransform], link: &str) -> Isometry3<f64> {
    let by_child: HashMap<&str, &LinkTransform> = transforms.iter().map(|t| (t.child, t)).collect();
    let mut pose = Isometry3::identity();
    let mut at = link;
    while let Some(transform) = by_child.get(at) {
        pose = transform.pose * pose;
        at = transform.parent;
    }
    pose
}

/// Down each arm of `id`, the transforms at drawn positions of the arm's joints compose
/// to the pose chain_kinematics gives its grasp link.
fn arms_compose_as_chains<const N: usize>(tc: &TestCase, id: &str) {
    let description = description(id);
    for arm in description.arms() {
        let joints: Vec<&str> = arm.joints.iter().map(String::as_str).collect();
        let chain = Chain::<N>::from_urdf(
            description.robot(),
            &ChainSpec {
                base_link: None,
                tip_link: &arm.grasp_link,
                joints: JointSelection::Named(&joints),
            },
        )
        .expect("an arm is a chain");
        let q = tc.draw(configs(chain.limits()));
        let positions: HashMap<&str, f64> = joints.iter().copied().zip(q).collect();
        let transforms = every_transform(&description, &positions);
        let miss = composed(&transforms, &arm.grasp_link).inverse() * chain.root_ee_pose(&q);
        assert!(
            miss.translation.vector.norm() <= TOLERANCE && miss.rotation.angle() <= TOLERANCE,
            "{id} {}: the transforms miss the chain by {miss}",
            arm.name
        );
    }
}

#[hegel::test]
fn the_openarm_v2_transforms_compose_as_its_arms_chains(tc: TestCase) {
    arms_compose_as_chains::<7>(&tc, "openarm_v2");
}

#[hegel::test]
fn the_so101_transforms_compose_as_its_arms_chain(tc: TestCase) {
    arms_compose_as_chains::<5>(&tc, "so101");
}

/// With every joint that moves at a drawn position, a bundled model gives one transform
/// for each link but the root, each from the link's parent.
#[hegel::test]
fn a_bundled_model_gives_one_transform_per_link_but_the_root(tc: TestCase) {
    for id in ["openarm_v2", "so101"] {
        let description = description(id);
        let robot = description.robot();
        let positions: HashMap<&str, f64> = robot
            .joints
            .iter()
            .filter(|joint| joint.joint_type != JointType::Fixed)
            .map(|joint| {
                let position = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
                (joint.name.as_str(), position)
            })
            .collect();
        let transforms = every_transform(&description, &positions);
        let children: HashSet<&str> = transforms.iter().map(|t| t.child).collect();
        assert_eq!(
            children.len(),
            transforms.len(),
            "{id}: a link placed twice"
        );
        let links: HashSet<&str> = robot.links.iter().map(|link| link.name.as_str()).collect();
        let roots: Vec<&&str> = links.difference(&children).collect();
        assert_eq!(
            roots.len(),
            1,
            "{id}: the links with no transform are {roots:?}"
        );
        for transform in &transforms {
            let joint = robot
                .joints
                .iter()
                .find(|joint| joint.child.link == transform.child)
                .expect("a placed link hangs on a joint");
            assert_eq!(
                transform.parent, joint.parent.link,
                "{id}: {}",
                transform.child
            );
        }
    }
}

/// With every joint that moves at a drawn position, each transform of a bundled model is
/// one the transforms contract takes, between the same links and at the same pose.
#[hegel::test]
fn a_bundled_model_s_transforms_are_the_contract_s(tc: TestCase) {
    for id in ["openarm_v2", "so101"] {
        let description = description(id);
        let positions: HashMap<&str, f64> = description
            .robot()
            .joints
            .iter()
            .filter(|joint| joint.joint_type != JointType::Fixed)
            .map(|joint| {
                let position = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
                (joint.name.as_str(), position)
            })
            .collect();
        for transform in every_transform(&description, &positions) {
            let carried = transform
                .frame_transform()
                .unwrap_or_else(|error| panic!("{id}: {error}"));
            assert_eq!(carried.parent().as_str(), transform.parent, "{id}");
            assert_eq!(carried.child().as_str(), transform.child, "{id}");
            let placement = carried.placement();
            let origin = transform.pose.translation.vector;
            let rotation = transform.pose.rotation;
            let expected = [
                origin.x, origin.y, origin.z, rotation.i, rotation.j, rotation.k, rotation.w,
            ];
            let actual: Vec<f64> = placement
                .position()
                .into_iter()
                .chain(placement.orientation())
                .collect();
            assert!(
                actual
                    .iter()
                    .zip(expected)
                    .all(|(a, e)| (a - e).abs() <= TOLERANCE),
                "{id}: {} in {}: {actual:?} against {expected:?}",
                transform.child,
                transform.parent
            );
        }
    }
}

#[test]
fn a_transform_the_contract_refuses_names_its_joint_s_links() {
    let description = description("openarm_v2");
    let fixed = description.fixed_transforms()[0];
    let mut pose = fixed.pose;
    pose.translation.vector.x = f64::INFINITY;
    let refused = LinkTransform { pose, ..fixed }
        .frame_transform()
        .expect_err("an origin that is not finite");
    assert_eq!(
        (refused.parent.as_str(), refused.child.as_str()),
        (fixed.parent, fixed.child)
    );
    assert!(refused.to_string().contains(fixed.child), "{refused}");
}

/// Where the SRDF's state `state` of `gripper` puts each finger joint, by joint name:
/// the state gives a position to each finger that moves of its own, in the gripper's
/// order, and a finger that mimics another stands where its mimic puts it from the
/// finger it follows.
fn fingers_in_state<'a>(
    description: &RobotDescription,
    gripper: &'a Gripper,
    state: &str,
) -> HashMap<&'a str, f64> {
    let values = description
        .state(&gripper.name, state)
        .expect("a gripper with a finger gives its closed and open states");
    let own: Vec<&str> = gripper
        .fingers
        .iter()
        .filter(|finger| finger.mimic.is_none())
        .map(|finger| finger.joint.as_str())
        .collect();
    assert_eq!(own.len(), values.len(), "{} {state}", gripper.name);
    let own: HashMap<&str, f64> = own.into_iter().zip(values.iter().copied()).collect();
    gripper
        .fingers
        .iter()
        .map(|finger| {
            let position = finger.mimic.as_ref().map_or_else(
                || own[finger.joint.as_str()],
                |mimic| mimic.position_from(own[mimic.joint()]),
            );
            (finger.joint.as_str(), position)
        })
        .collect()
}

/// A gripper puts every finger where the SRDF's `closed` state puts it at opening 0,
/// where its `open` state puts it at 1, and on the line through the two at a drawn
/// opening; a finger that mimics another stands where its mimic puts it from the finger
/// it follows.
#[hegel::test]
fn a_gripper_places_every_finger_between_its_closed_and_open_states(tc: TestCase) {
    let drawn: f64 = tc.draw(gs::floats().min_value(-0.5).max_value(1.5));
    for id in ["openarm_v1", "openarm_v2", "so101"] {
        let description = description(id);
        let grippers: Vec<&Gripper> = description
            .arms()
            .iter()
            .map(|arm| &arm.gripper)
            .filter(|gripper| !gripper.fingers.is_empty())
            .collect();
        assert!(!grippers.is_empty(), "{id}: a gripper with a finger");
        for gripper in grippers {
            let closed = fingers_in_state(&description, gripper, CLOSED);
            let open = fingers_in_state(&description, gripper, OPEN);
            for opening in [0.0, 1.0, drawn] {
                let positions: HashMap<&str, f64> = gripper.finger_positions_at(opening).collect();
                assert_eq!(positions.len(), gripper.fingers.len(), "{id}");
                for finger in &gripper.fingers {
                    let joint = finger.joint.as_str();
                    let position = positions[joint];
                    let on_the_line = (1.0 - opening) * closed[joint] + opening * open[joint];
                    assert!(
                        (position - on_the_line).abs() <= TOLERANCE,
                        "{id} {joint} at {opening}: {position}, the states' line {on_the_line}"
                    );
                    if let Some(mimic) = &finger.mimic {
                        let from_leader = mimic.position_from(positions[mimic.joint()]);
                        assert!(
                            (position - from_leader).abs() <= TOLERANCE,
                            "{id} {joint} at {opening}: {position} and its mimic's {from_leader}"
                        );
                    }
                }
            }
        }
    }
}
