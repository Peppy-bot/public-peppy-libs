//! SO-101's description for the Rust nodes: the kinematics URDF and the collision
//! meshes of the Python package `so101_description` beside this crate, the arm's chain
//! from [`BASE_LINK`], the robot frame, to [`TCP_LINK`], the limb names limb_state and
//! limb_motion carry, and where the jaw stands at a gripper opening.

use std::path::Path;
use std::sync::LazyLock;

/// The arm's joints, base to wrist.
pub const ARM_DOF: usize = 5;

/// The first link of the arm's chain: the robot frame.
pub const BASE_LINK: &str = "base_link";

/// The point of the gripper every pose of the robot names.
pub const TCP_LINK: &str = "gripper_frame_link";

/// The link of the gripper's one moving jaw, on a joint of its own off the gripper link.
pub const JAW_LINK: &str = "moving_jaw_so101_v1_link";

/// The arm's limb name, as limb_state and limb_motion carry it.
pub const ARM_LIMB: &str = "arm";

/// The gripper's limb name, as limb_state and limb_motion carry it.
pub const GRIPPER_LIMB: &str = "gripper";

/// The kinematics URDF, each collision mesh named by its file under `collision/`.
pub const URDF: &str = include_str!("so101_description/urdf/so101_kinematics.urdf");

/// Each collision mesh, by the name of its file.
const COLLISION_MESHES: &[(&str, &[u8])] = &[
    (
        "base_motor_holder_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/base_motor_holder_so101_v1.stl"),
    ),
    (
        "base_so101_v2.stl",
        include_bytes!("so101_description/urdf/collision/base_so101_v2.stl"),
    ),
    (
        "motor_holder_so101_base_v1.stl",
        include_bytes!("so101_description/urdf/collision/motor_holder_so101_base_v1.stl"),
    ),
    (
        "motor_holder_so101_wrist_v1.stl",
        include_bytes!("so101_description/urdf/collision/motor_holder_so101_wrist_v1.stl"),
    ),
    (
        "moving_jaw_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/moving_jaw_so101_v1.stl"),
    ),
    (
        "rotation_pitch_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/rotation_pitch_so101_v1.stl"),
    ),
    (
        "sts3215_03a_no_horn_v1.stl",
        include_bytes!("so101_description/urdf/collision/sts3215_03a_no_horn_v1.stl"),
    ),
    (
        "sts3215_03a_v1.stl",
        include_bytes!("so101_description/urdf/collision/sts3215_03a_v1.stl"),
    ),
    (
        "under_arm_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/under_arm_so101_v1.stl"),
    ),
    (
        "upper_arm_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/upper_arm_so101_v1.stl"),
    ),
    (
        "waveshare_mounting_plate_so101_v2.stl",
        include_bytes!("so101_description/urdf/collision/waveshare_mounting_plate_so101_v2.stl"),
    ),
    (
        "wrist_camera_module.stl",
        include_bytes!("so101_description/urdf/collision/wrist_camera_module.stl"),
    ),
    (
        "wrist_roll_follower_so101_v1.stl",
        include_bytes!("so101_description/urdf/collision/wrist_roll_follower_so101_v1.stl"),
    ),
    (
        "wrist_roll_pitch_so101_v2.stl",
        include_bytes!("so101_description/urdf/collision/wrist_roll_pitch_so101_v2.stl"),
    ),
];

/// Writes the collision meshes into `dir`, created if absent, under the names of their
/// files, where a builder of the URDF's collision geometry reads them.
pub fn write_meshes_to(dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    for (name, bytes) in COLLISION_MESHES {
        std::fs::write(dir.join(name), bytes)?;
    }
    Ok(())
}

/// The lower and the upper limit, in radians, of the joint that carries [`JAW_LINK`].
static JAW_LIMITS_RAD: LazyLock<[f64; 2]> = LazyLock::new(|| {
    let robot = urdf_rs::read_from_string(URDF).expect("the embedded URDF parses");
    let joint = robot
        .joints
        .iter()
        .find(|joint| joint.child.link == JAW_LINK)
        .expect("the embedded URDF carries the jaw on a joint");
    [joint.limit.lower, joint.limit.upper]
});

/// The position, in radians, of the jaw's joint at a gripper `opening` as limb_state
/// reports it, 0 closed and 1 fully open: the jaw closes at its joint's lower limit and
/// opens fully at its upper one. An opening outside 0..1 counts as the nearer end; a
/// non-finite one gives `None`.
pub fn jaw_position_rad(opening: f64) -> Option<f64> {
    if !opening.is_finite() {
        return None;
    }
    let [closed, open] = *JAW_LIMITS_RAD;
    Some(closed + opening.clamp(0.0, 1.0) * (open - closed))
}
