//! The OpenArm fixture shared by the tests (included via `#[path]`): the fixture URDF,
//! the generation of `openarm_description` that it describes, whose fingers, yoked joints,
//! torso body and regions a caller reads from it, the collision meshes of that generation,
//! which the fixture URDF names, and the builder of the fixture's model.

#![allow(dead_code)]

use chain_kinematics::{ChainSpec, JointSelection};
use openarm_description::{ARM_DOF, HardwareVersion, Side};
use self_collision_model::{Builder, Finger, SelfCollisionModel, openarm_fingers};

/// The generation that the fixture URDF describes.
pub const VERSION: HardwareVersion = HardwareVersion::V1;

/// The fixture URDF: OpenArm v1.0's arms on its torso.
pub const URDF: &str = include_str!("openarm_v10.urdf");

/// The collision meshes of OpenArm v1.0, which the fixture URDF names.
pub const MESHES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../openarm_description/assets/meshes"
);

/// The fixture's arms, each from its base link to its last link, left then right.
pub fn chains() -> [ChainSpec<'static>; 2] {
    Side::BOTH.map(|side| ChainSpec {
        base_link: Some(VERSION.base_link(side)),
        tip_link: match side {
            Side::Left => "openarm_left_link7",
            Side::Right => "openarm_right_link7",
        },
        joints: JointSelection::PathOrder,
    })
}

/// The fixture's fingers, each shut where the generation shuts it.
pub fn fingers() -> Vec<Finger<'static>> {
    openarm_fingers(VERSION)
}

/// The builder of the fixture's model from `urdf`: its arms, its fingers and the
/// generation's yoked joints, the torso fitted whole.
pub fn builder(urdf: &str) -> Builder<'_, ARM_DOF, 2> {
    SelfCollisionModel::builder(urdf, MESHES, chains(), VERSION.yoked_joints()).fingers(&fingers())
}
