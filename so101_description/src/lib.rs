//! SO-101's description for the Rust nodes: the kinematics URDF and the collision
//! meshes of the Python package `so101_description` beside this crate, the arm's chain
//! from [`BASE_LINK`], the robot frame, to [`TCP_LINK`], the limb names limb_state and
//! limb_motion carry, where the jaw stands at a gripper opening and where it shuts, and
//! the data of its self-collision model: its [`YOKED_JOINTS`] and its
//! [`NEVER_TOUCHING_PAIRS`].

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

/// The position, in radians, of the jaw's joint at which the jaw is shut: its lower
/// limit.
pub fn jaw_shut_rad() -> f64 {
    JAW_LIMITS_RAD[0]
}

/// The position, in radians, of the jaw's joint at a gripper `opening` as limb_state
/// reports it, 0 shut and 1 fully open: the jaw opens from [`jaw_shut_rad`] to its
/// joint's upper limit. An opening outside 0..1 counts as the nearer end; a non-finite
/// one gives `None`.
pub fn jaw_position_rad(opening: f64) -> Option<f64> {
    if !opening.is_finite() {
        return None;
    }
    let (closed, open) = (jaw_shut_rad(), JAW_LIMITS_RAD[1]);
    Some(closed + opening.clamp(0.0, 1.0) * (open - closed))
}

/// The gripper opening at which the jaw's joint stands at `position_rad`: the inverse of
/// [`jaw_position_rad`], 0 at [`jaw_shut_rad`] and 1 at the joint's upper limit, and past
/// those ends for a position past the joint's limits.
pub fn jaw_opening(position_rad: f64) -> f64 {
    let (closed, open) = (jaw_shut_rad(), JAW_LIMITS_RAD[1]);
    (position_rad - closed) / (open - closed)
}

/// The most moving joints two bodies of the arm, or the base and a link, lie apart and
/// a collision model still leaves unchecked: the links of adjacent joints alone, for the
/// arm folds two links apart onto each other, the lower arm onto the shoulder.
pub const YOKED_JOINTS: usize = 1;

/// The pairs of bodies the joint limits keep apart, which a collision model leaves out,
/// as MoveIt's setup assistant disables a pair it never finds in contact: the base and
/// the upper arm, the gripper and the lower arm, and the lower arm and the jaw. Over the
/// joints between each pair's bodies within the limits, every 0.05 rad, the jaw shut, half
/// open and fully open, the base and the upper arm stay 17 mm or farther apart, the
/// gripper and the lower arm 38 mm, and the lower arm and the jaw 46 mm.
pub const NEVER_TOUCHING_PAIRS: [[&str; 2]; 3] = [
    [BASE_LINK, "upper_arm_link"],
    ["gripper_link", "lower_arm_link"],
    ["lower_arm_link", JAW_LINK],
];
