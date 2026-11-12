//! OpenArm v2 as robot_description bundles it, and its arms, fitted once, for the suites
//! that place them.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use std::path::Path;
use std::sync::LazyLock;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use robot_description::{ArmId, PerArm, RobotDescription};

/// The joints of each arm of OpenArm v2.
pub const JOINTS: usize = 7;
/// OpenArm v2's arms.
pub const ARMS: usize = 2;
/// OpenArm v2's left arm and its right, the first and the second of its description.
pub const LEFT: ArmId = ArmId::new(0);
pub const RIGHT: ArmId = ArmId::new(1);

/// OpenArm v2's model id.
const MODEL: &str = "openarm_v2";

/// OpenArm v2's description: its arms `left_arm` then `right_arm`, each of `JOINTS`
/// joints.
pub static OPENARM_V2: LazyLock<RobotDescription> = LazyLock::new(|| {
    let description = robot_description::model(MODEL)
        .expect("robot_description bundles OpenArm v2")
        .description();
    let names: Vec<&str> = description
        .arms()
        .iter()
        .map(|arm| arm.name.as_str())
        .collect();
    assert_eq!(names, ["left_arm", "right_arm"], "OpenArm v2's arms");
    for arm in description.arms() {
        assert_eq!(arm.joints.len(), JOINTS, "the joints of {}", arm.name);
    }
    description
});

/// OpenArm v2's two arms, each the chain from the URDF root to its grasp
/// point over its arm's joints, fitted to the collision meshes the description carries.
pub static OPENARM_V2_ARMS: LazyLock<PerArm<ChainGeometry<JOINTS>, ARMS>> = LazyLock::new(|| {
    let meshes = Path::new(env!("CARGO_TARGET_TMPDIR")).join("openarm_v2_meshes");
    robot_description::model(MODEL)
        .expect("robot_description bundles OpenArm v2")
        .write_meshes_to(&meshes)
        .expect("write the meshes");
    let meshes = meshes.to_str().expect("the mesh directory is UTF-8");
    PerArm::from_fn(|id| {
        let arm = &OPENARM_V2.arms()[id.place()];
        let joints: Vec<&str> = arm.joints.iter().map(String::as_str).collect();
        ChainGeometry::from_urdf(
            OPENARM_V2.urdf(),
            meshes,
            &ChainSpec {
                base_link: None,
                tip_link: &arm.grasp_link,
                joints: JointSelection::Named(&joints),
            },
        )
        .expect("the arm fits")
    })
});

/// Each arm's posture `name`, as OpenArm v2's description gives it.
pub fn posture(name: &str) -> PerArm<[f64; JOINTS], ARMS> {
    PerArm::from_fn(|id| {
        let arm = &OPENARM_V2.arms()[id.place()].name;
        OPENARM_V2
            .state(arm, name)
            .unwrap_or_else(|| panic!("{arm} has the posture {name}"))
            .try_into()
            .expect("a posture of each of the arm's joints")
    })
}
