//! The links a description names on the bundled models: the root link, each camera's
//! optical frame, and the arm that carries each link.

use std::collections::{HashMap, HashSet};

use robot_description::{RobotDescription, UnknownLink, model};
use urdf_rs::JointType;

const MODELS: [&str; 3] = ["openarm_v1", "openarm_v2", "so101"];

fn description(id: &str) -> RobotDescription {
    model(id).expect("a bundled model").description()
}

/// `link` and every link below it in `description`'s URDF.
fn at_or_below<'a>(description: &'a RobotDescription, link: &'a str) -> HashSet<&'a str> {
    let joints = &description.robot().joints;
    let mut found = HashSet::from([link]);
    let mut frontier = vec![link];
    while let Some(parent) = frontier.pop() {
        for joint in joints.iter().filter(|joint| joint.parent.link == parent) {
            found.insert(joint.child.link.as_str());
            frontier.push(joint.child.link.as_str());
        }
    }
    found
}

#[test]
fn the_root_link_is_the_one_link_that_hangs_on_no_joint() {
    for id in MODELS {
        let description = description(id);
        let robot = description.robot();
        let children: HashSet<&str> = robot
            .joints
            .iter()
            .map(|joint| joint.child.link.as_str())
            .collect();
        let roots: Vec<&str> = robot
            .links
            .iter()
            .map(|link| link.name.as_str())
            .filter(|link| !children.contains(link))
            .collect();
        assert_eq!(roots, [description.root_link()], "{id}");
    }
}

#[test]
fn a_cameras_frame_is_the_link_its_fixed_joint_holds_on_its_parent_link() {
    let mut cameras = 0;
    for id in MODELS {
        let description = description(id);
        for camera in description.cameras() {
            let frame = camera.frame();
            let joint = description
                .robot()
                .joints
                .iter()
                .find(|joint| joint.child.link == frame)
                .unwrap_or_else(|| panic!("{id}: {frame} hangs on a joint"));
            assert_eq!(joint.joint_type, JointType::Fixed, "{id}: {frame}");
            assert_eq!(joint.parent.link, camera.parent_link, "{id}: {frame}");
            assert_eq!(
                (joint.origin.xyz.0, joint.origin.rpy.0),
                (camera.xyz, camera.rpy),
                "{id}: {frame}"
            );
            cameras += 1;
        }
    }
    assert_eq!(cameras, 4, "openarm_v2's three cameras and so101's one");
}

/// On a bundled model, whose arms hold no joint in common, the arm that carries a link is
/// the one arm a joint of which has the link at or below its child link, and none
/// carries a link that no arm's joint has below it.
#[test]
fn the_arm_that_carries_a_link_is_the_one_whose_joint_it_hangs_below() {
    for id in MODELS {
        let description = description(id);
        let joints: HashMap<&str, &str> = description
            .robot()
            .joints
            .iter()
            .map(|joint| (joint.name.as_str(), joint.child.link.as_str()))
            .collect();
        let held: Vec<(&str, HashSet<&str>)> = description
            .arms()
            .iter()
            .map(|arm| {
                let below = arm
                    .joints
                    .iter()
                    .flat_map(|joint| at_or_below(&description, joints[joint.as_str()]))
                    .collect();
                (arm.name.as_str(), below)
            })
            .collect();
        let mut carried = 0;
        for link in &description.robot().links {
            let link = link.name.as_str();
            let mut holders = held
                .iter()
                .filter(|(_, below)| below.contains(link))
                .map(|(arm, _)| *arm);
            let holder = holders.next();
            assert_eq!(holders.next(), None, "{id}: two arms hold {link}");
            let carrier = description
                .arm_carrying(link)
                .expect("a link of the URDF")
                .map(|arm| arm.name.as_str());
            assert_eq!(carrier, holder, "{id}: {link}");
            carried += usize::from(carrier.is_some());
        }
        assert!(carried > 0, "{id}: an arm carries a link");
        assert_eq!(description.arm_carrying(description.root_link()), Ok(None));
    }
}

#[test]
fn an_arm_carries_its_wrist_camera_and_none_carries_the_chest_camera() {
    for (id, camera, arm) in [
        ("openarm_v2", "wrist_left", Some("left_arm")),
        ("openarm_v2", "wrist_right", Some("right_arm")),
        ("openarm_v2", "chest", None),
        ("so101", "wrist", Some("arm")),
    ] {
        let description = description(id);
        let camera = description
            .cameras()
            .iter()
            .find(|candidate| candidate.name == camera)
            .unwrap_or_else(|| panic!("{id} carries the camera {camera}"));
        for link in [camera.frame(), camera.parent_link.clone()] {
            let carrier = description
                .arm_carrying(&link)
                .expect("a link of the URDF")
                .map(|arm| arm.name.as_str());
            assert_eq!(carrier, arm, "{id}: {link}");
        }
    }
}

#[test]
fn a_link_the_urdf_lacks_is_refused() {
    let description = description("so101");
    assert_eq!(
        description.arm_carrying("no_such_link"),
        Err(UnknownLink {
            link: "no_such_link".to_string()
        })
    );
}
