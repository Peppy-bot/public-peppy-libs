//! Kinematics + dynamics for a 7-DOF SRS (spherical-revolute-spherical) arm.
//!
//! - [`fk`]: forward kinematics from a URDF (via the `k` crate).
//! - [`model`]: the SRS geometry (shoulder/elbow/wrist centers, link lengths,
//!   joint limits) derived once from the FK chain.
//! - [`ik`]: closed-form arm-angle (Shimizu) inverse kinematics.
//! - [`dynamics`]: gravity / Coriolis / friction feedforward torques, for the
//!   arm control loop (gravity/Coriolis validated against KDL).
//!
//! Frames: [`fk`] and [`ik`] report poses in the **arm base frame** (the arm's
//! own mounting link). [`dynamics`] instead computes in the **world frame**,
//! because gravity is a world quantity (acts along world -Z). The fixed
//! world <-> base mount transform relating the two is captured here and exposed
//! via [`model::ArmModel::base_pose`] / [`world_pose`](model::ArmModel::world_pose),
//! so a caller converting between world and base frames does not redo it.
//!
//! Robot-agnostic: geometry, joint limits and inertials are all derived from
//! whatever URDF the caller passes, and a non-SRS chain is rejected with `Err`.
//! The robot-to-(URDF, link-names, friction) mapping is the one openarm-specific
//! part, isolated in the [`description`] module. The crate currently serves the
//! OpenArm; if the agnostic core is ever reused on another SRS arm, that module
//! moves out and the rest lifts to a shared hub unchanged.
//!
//! Hardware-free: it defines its own [`ARM_DOF`] rather than depending on
//! `openarm_can`, so the full IK<->FK round-trip runs under a plain `cargo test`
//! on any host.

pub mod description;
pub mod dynamics;
pub mod fk;
pub mod ik;
pub mod model;

/// Degrees of freedom of the arm. The URDF chain is validated against this at
/// load (the closed-form arm-angle redundancy resolution is specific to 7 DOF).
pub const ARM_DOF: usize = 7;

/// One joint-space configuration, j1..j7 in radians.
pub type JointVec = [f64; ARM_DOF];

/// Smallest sine of an angle between two unit axes (or two link directions) we
/// treat as non-degenerate (~1e-6 rad). Below it the perpendicular / cross
/// direction is ill-conditioned to normalize, so the caller bails out (parallel
/// axes, straight arm). Sites that test a `sin²` quantity compare its square.
pub(crate) const PARALLEL_SIN_EPS: f64 = 1e-6;

/// Re-export the linear-algebra types so downstream crates use the same
/// `nalgebra` version `k` was built against.
pub use k::nalgebra;

/// Test fixtures: load a concrete SRS arm (the OpenArm V1.0) through the
/// [`description`] module, giving the agnostic tests a real arm to check against.
/// `side` is `"left"`/`"right"`.
#[cfg(test)]
pub(crate) mod test_support {
    use crate::description::{ArmSide, Description, Version};
    use crate::fk::ForwardKinematics;
    use crate::model::ArmModel;

    fn v1(side: &str) -> Description {
        Description::new(Version::V1, ArmSide::from_param(side).expect("left/right"))
    }

    pub(crate) fn v1_fk(side: &str) -> ForwardKinematics {
        v1(side).forward_kinematics().expect("load v1 fk")
    }

    pub(crate) fn v1_model(side: &str) -> ArmModel {
        v1(side).model().expect("load v1 model")
    }
}
