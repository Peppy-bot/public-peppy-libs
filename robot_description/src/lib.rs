//! A robot's description as two files every node reads: its URDF, which holds its links,
//! joints, limits, inertias and collision geometry, and its SRDF, which holds what the
//! robot is made of for a planner: its groups, its end effectors, its named states and
//! the pairs of links whose collisions are disabled.
//!
//! [`RobotDescription::parse`] reads the two together and holds each name of the SRDF
//! to the URDF, so a consumer reads facts and never names a link of one robot:
//!
//! - [`RobotDescription::arms`]: each group of one chain whose tip an end effector
//!   stands on, with its base link, its grasp point and its joints, and its
//!   [`Gripper`], each [`Finger`] with where the gripper's `closed` and `open` states
//!   put it and, for a finger that follows another, its [`Mimic`]; a gripper gives each
//!   finger joint's position at an opening ([`Gripper::finger_positions_at`]).
//! - [`RobotDescription::group_arms`]: the arms of a group, such as `both_arms`'s two.
//! - [`RobotDescription::state`]: a named state of a group, such as an arm's `ready`.
//! - [`RobotDescription::disabled_pairs`]: the pairs a collision model leaves out, each
//!   with its [`DisabledReason`].
//! - [`RobotDescription::cameras`]: the optical frame of each camera the robot carries,
//!   a link named `<camera>_optical_frame` ([`CameraFrame::frame`]), and the
//!   [`DepthRange`] it measures where the URDF gives it a depth sensor in the form of
//!   Gazebo's URDF extension: a `<gazebo>` element that references the optical frame and
//!   holds a `<sensor type="depth">` whose `<camera><clip>` gives the `<near>` and
//!   `<far>` depths.
//! - [`RobotDescription::lidars`]: each 2-D lidar the robot carries ([`LidarFrame`]), as
//!   Gazebo's URDF extension gives one: a `<gazebo>` element that references the lidar's
//!   frame, a link on a fixed joint, and holds a `<sensor type="gpu_lidar">` named for the
//!   lidar whose `<lidar><range>` gives the `<min>` and `<max>` ranges ([`LidarRange`]).
//! - [`RobotDescription::root_link`]: the URDF's root link, the robot frame,
//!   [`RobotDescription::parent_joint`]: the joint a link hangs on, and
//!   [`RobotDescription::arm_carrying`]: the arm whose chain carries a link, such as a
//!   camera's, or none for a link no arm's joint moves.
//! - [`RobotDescription::fixed_transforms`] and [`RobotDescription::transforms_at`]: where
//!   each link stands in its parent link, a [`LinkTransform`] per joint, the fixed joints'
//!   for the life of the robot and the moving joints' at their positions; and
//!   [`LinkTransform::frame_transform`]: the transform as the transforms contract carries it.
//! - [`RobotDescription::link_frames_at_zero`]: where each link stands in the root link's
//!   frame with every joint at zero.
//! - [`RobotDescription::differential_base`]: the two wheels of a differential base, the
//!   continuous joints that no group of the SRDF holds, on a link that fixed joints alone
//!   join to the root link, each with its radius and the width of its tire, and their
//!   separation.
//! - [`RobotDescription::outline`]: the rectangle on the floor that holds every collision
//!   element of the robot, each a box, a cylinder or a sphere, with every joint at zero, of
//!   their bounds along the root link's axes ([`chain_kinematics::primitive_bounds`]).
//!
//! The reader is strict: an element or attribute of the SRDF or of a `<gazebo>` element of
//! the URDF that it does not read, or a name the URDF lacks, is a refusal
//! ([`DescriptionError`]) that names it. A URDF's links and joints are held to one tree
//! by the rules of chain_kinematics, which poses every chain of it
//! ([`chain_kinematics::TreeError`]). [`RobotDescription::position_limits`] gives a
//! joint's position limits, and [`mesh_name`] the file name a description holds a
//! collision mesh by.
//!
//! [`Mimic`]: chain_kinematics::Mimic
//!
//! [`model`] gives the bundled description of a robot model of this repository by its
//! model id, the id robot_identity carries: its files, the [`Model::directory`] the build
//! bundled them from, the [`Model::digest`] of them all and, behind the `meshes` feature,
//! the collision meshes the URDF names, which [`write_meshes`] writes where a builder of
//! the URDF's collision geometry reads them. [`Model::hold_digest`] refuses a description
//! served under another digest ([`OtherDescription`]). [`models_with_arms`] gives the
//! bundled models that have an arm.
//!
//! [`on_shape`] runs code compiled for the shapes of a robot's arms, its arm count and
//! its arms' joint count, at the shape of a description's arms. [`NodeArms`] holds a
//! bundled model to the arms a node is compiled for, by the names the node's pairing
//! slots carry, and gives the model it drives as a [`Driven`]; [`DrivenModels`] lists the
//! models a node drives by model id. A model a node does not drive is an
//! [`UndrivenModel`].
#![deny(unsafe_code)]

mod arms;
mod base;
mod bundled;
mod description;
#[cfg(all(test, feature = "meshes"))]
mod digest;
mod driven;
mod error;
mod gazebo;
mod shape;
mod srdf;
mod transforms;
mod xml;

pub use arms::{ArmId, PerArm};
pub use base::{DifferentialBase, NoDifferentialBase, NoOutline, Outline, Wheel};
pub use bundled::{
    Model, OtherDescription, UnknownModel, model, models, models_with_arms, write_meshes,
};
pub use description::{
    Arm, CameraFrame, DepthRange, DisabledPair, DisabledReason, Finger, FingerSpan, Gripper,
    LidarFrame, LidarRange, RobotDescription, mesh_name,
};
pub use driven::{Driven, DrivenModels, UndrivenModel};
pub use error::{
    DescriptionError, FingerSpanError, UnknownGroup, UnknownJoint, UnknownLink, UnknownState,
};
pub use shape::{NodeArms, OnShape, UnsupportedShape, on_shape};
pub use transforms::{LinkTransform, LinkTransformError};

/// The URDF crate of [`RobotDescription::robot`], so a caller uses the same version.
pub use urdf_rs;

/// The kinematics crate of [`Finger::mimic`] and of the description's tree, so a caller
/// uses the same version.
pub use chain_kinematics;

/// The names of the group states that the readers of a description look up with
/// [`RobotDescription::state`].
pub mod states {
    /// An arm's ready posture, the one it works from.
    pub const READY: &str = "ready";
    /// An arm's home posture, the one it rests in.
    pub const HOME: &str = "home";
    /// An arm's start posture, the one Waldo stands it in when its robot joins.
    pub const START: &str = "start";
    /// An arm's park posture, which robot_guide answers.
    pub const PARK: &str = "park";
    /// Where a gripper's fingers stand shut. Every gripper with a finger gives it.
    pub const CLOSED: &str = "closed";
    /// Where a gripper's fingers stand open. Every gripper with a finger gives it.
    pub const OPEN: &str = "open";
}
