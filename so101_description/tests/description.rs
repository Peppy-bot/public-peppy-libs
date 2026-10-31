//! The description against the files it embeds: the meshes it writes are the ones the
//! URDF names, byte for byte the Python package's; the arm's chain runs from the base
//! link to the tool point through its joints; the limb names are the Python package's;
//! and the jaw's position runs between its joint's limits.

use std::path::Path;

use so101_description::{
    ARM_DOF, ARM_LIMB, BASE_LINK, GRIPPER_LIMB, JAW_LINK, TCP_LINK, URDF, jaw_position_rad,
    write_meshes_to,
};
use urdf_rs::{Geometry, JointType, Robot};

/// The Python package, beside this crate's sources.
const PACKAGE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/src/so101_description");

fn robot() -> Robot {
    urdf_rs::read_from_string(URDF).expect("the URDF parses")
}

// The meshes written are the ones the URDF names, each the package's file.
#[test]
fn the_meshes_written_are_the_ones_the_urdf_names_as_the_package_holds_them() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("written_meshes");
    if dir.exists() {
        std::fs::remove_dir_all(&dir).expect("the last run's meshes go");
    }
    write_meshes_to(&dir).expect("the meshes write");
    let mut named: Vec<String> = robot()
        .links
        .iter()
        .flat_map(|link| &link.collision)
        .filter_map(|collision| match &collision.geometry {
            Geometry::Mesh { filename, .. } => Some(filename.clone()),
            _ => None,
        })
        .collect();
    named.sort();
    named.dedup();
    let mut written: Vec<String> = std::fs::read_dir(&dir)
        .expect("the mesh directory lists")
        .map(|entry| {
            let name = entry.expect("a mesh's entry reads").file_name();
            format!(
                "collision/{}",
                name.to_str().expect("a mesh's name is UTF-8")
            )
        })
        .collect();
    written.sort();
    assert_eq!(written, named);
    for mesh in &named {
        let file = mesh
            .strip_prefix("collision/")
            .expect("a mesh under collision/");
        let package = Path::new(PACKAGE).join("urdf").join(mesh);
        assert!(
            std::fs::read(dir.join(file)).expect("a written mesh reads")
                == std::fs::read(&package).expect("the package's mesh reads"),
            "{mesh} is not the package's"
        );
    }
}

// From the tool point toward the root, the URDF's joints reach the base link through
// the arm's revolute joints.
#[test]
fn the_arm_runs_from_the_base_link_to_the_tool_point_through_its_joints() {
    let robot = robot();
    let mut link = TCP_LINK;
    let mut revolute = 0;
    while let Some(joint) = robot.joints.iter().find(|joint| joint.child.link == link) {
        match joint.joint_type {
            JointType::Fixed => {}
            JointType::Revolute => revolute += 1,
            ref other => panic!("{} is a {other:?} joint", joint.name),
        }
        link = &joint.parent.link;
    }
    assert_eq!(link, BASE_LINK);
    assert_eq!(revolute, ARM_DOF);
}

#[test]
fn the_limb_names_are_the_python_packages() {
    let limbs = std::fs::read_to_string(Path::new(PACKAGE).join("limbs.py"))
        .expect("the package's limbs.py reads");
    for (name, value) in [("ARM_LIMB", ARM_LIMB), ("GRIPPER_LIMB", GRIPPER_LIMB)] {
        let line = format!("{name} = {value:?}");
        assert!(
            limbs.lines().any(|held| held == line),
            "limbs.py holds no {line}"
        );
    }
}

// Shut, the jaw stands at its joint's lower limit and fully open at its upper one; an
// opening past either end stands at that end, and a non-finite one has no position.
#[test]
fn the_jaws_position_runs_from_its_joints_lower_limit_shut_to_its_upper_open() {
    const ROUNDING_RAD: f64 = 1e-12;
    let robot = robot();
    let joint = robot
        .joints
        .iter()
        .find(|joint| joint.child.link == JAW_LINK)
        .expect("a joint carries the jaw");
    let (lower, upper) = (joint.limit.lower, joint.limit.upper);
    for (opening, position) in [
        (0.0, lower),
        (0.5, (lower + upper) / 2.0),
        (1.0, upper),
        (-0.5, lower),
        (1.5, upper),
    ] {
        let at = jaw_position_rad(opening).expect("a finite opening has a position");
        assert!(
            (at - position).abs() < ROUNDING_RAD,
            "opening {opening}: {at} rad"
        );
    }
    for opening in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(jaw_position_rad(opening), None);
    }
}
