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
//!   put it and, for a finger that follows another, its [`Mimic`].
//! - [`RobotDescription::group_arms`]: the arms of a group, such as `both_arms`'s two.
//! - [`RobotDescription::state`]: a named state of a group, such as an arm's `ready`.
//! - [`RobotDescription::disabled_pairs`]: the pairs a collision model leaves out, each
//!   with its [`DisabledReason`].
//! - [`RobotDescription::cameras`]: the optical frame of each camera the robot carries,
//!   a link named `<camera>_optical_frame`, and the [`DepthRange`] it measures where the
//!   URDF gives it a depth sensor in the form of Gazebo's URDF extension: a `<gazebo>`
//!   element that references the optical frame and holds a `<sensor type="depth">` whose
//!   `<camera><clip>` gives the `<near>` and `<far>` depths.
//!
//! The reader is strict: an element or attribute of the SRDF or of a `<gazebo>` element of
//! the URDF that it does not read, or a name the URDF lacks, is a refusal
//! ([`DescriptionError`]) that names it. A URDF's links and joints are
//! held to one tree by the rules of chain_kinematics, which poses every chain of it
//! ([`chain_kinematics::TreeError`]). [`position_limits`] gives a joint's position limits,
//! and [`mesh_name`] the file name a description holds a collision mesh by.
//!
//! [`model`] gives the bundled description of a robot model of this repository by its
//! model id, the id robot_identity carries: its files, the [`Model::directory`] the build
//! bundled them from, the [`Model::digest`] of them all and, behind the `meshes` feature,
//! the collision meshes the URDF names, which [`write_meshes`] writes where a builder of
//! the URDF's collision geometry reads them.
//!
//! [`on_shape`] runs code compiled for the shapes of a robot's arms, its arm count and
//! its arms' joint count, at the shape of a description's arms. [`NodeArms`] holds a
//! bundled model to the arms a node is compiled for, by the names the node's pairing
//! slots carry, and gives the model it drives as a [`Driven`]; [`DrivenModels`] lists the
//! models a node drives by model id. A model a node does not drive is an
//! [`UndrivenModel`].
#![deny(unsafe_code)]

mod arms;
mod bundled;
mod description;
#[cfg(all(test, feature = "meshes"))]
mod digest;
mod driven;
mod error;
mod gazebo;
mod shape;
mod srdf;
mod xml;

pub use arms::{ArmId, PerArm};
pub use bundled::{Model, UnknownModel, model, models, write_meshes};
pub use description::{
    Arm, CameraFrame, DepthRange, DisabledPair, DisabledReason, Finger, Gripper, Mimic,
    RobotDescription, mesh_name, position_limits,
};
pub use driven::{Driven, DrivenModels, UndrivenModel};
pub use error::DescriptionError;
pub use shape::{NodeArms, OnShape, UnsupportedShape, on_shape};

/// The URDF crate of [`RobotDescription::robot`], so a caller uses the same version.
pub use urdf_rs;

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
