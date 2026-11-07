//! Kinematics for a serial chain read from a URDF: forward kinematics, the
//! geometric Jacobian, damped resolved-rate steps, and MoveIt's KDL inverse
//! kinematics.
//!
//! Nothing here assumes a topology. A [`Chain`] is any path through any URDF
//! with `N` actuated joints - a seven-axis arm, a five-axis arm, a leg, a
//! pan-tilt head - and every type is generic over `N`, fixed at compile time so
//! a control tick allocates nothing.
//!
//! Build one from a parsed URDF and a [`ChainSpec`] naming where the chain
//! starts, where it ends, and which joints are yours to move:
//!
//! ```no_run
//! # use chain_kinematics::{Chain, ChainSpec, JointSelection};
//! let robot = urdf_rs::read_from_string(MY_URDF)?;
//! let chain = Chain::<5>::from_urdf(&robot, &ChainSpec {
//!     base_link: None,
//!     tip_link: "gripper_frame_link",
//!     joints: JointSelection::Named(&[
//!         "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll",
//!     ]),
//! })?;
//! let ee = chain.at(&[0.0; 5]).ee_pose();
//! # const MY_URDF: &str = "";
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! The tip and the joint order are **named, not discovered**. Discovering a tip
//! by counting joints only works for the arm it was written for, and inferring
//! joint order from the URDF's own ordering is luck rather than a contract: the
//! order is the order of `q`, and it is usually the order the robot's wire
//! protocol uses, which no URDF knows about.
//!
//! Frames: forward kinematics reports poses in the **base frame** named by the
//! spec, and [`Chain::base_from_world`] relates that to the URDF root for a
//! caller that needs the world frame (gravity, or composing two chains of one
//! robot).
//!
//! Evidence for the forward kinematics: at every configuration of the forward
//! kinematics scenarios of the ROS reference harness (`ros_reference`), the
//! OpenArm v2 arms and SO-101, each link pose agrees with the global link
//! transform of MoveIt's robot state within 16 units of
//! `f64::EPSILON * max(1, |x|)` (`tests/ros_reference_forward_kinematics.rs`).
//!
//! Evidence for the inverse kinematics: from the seeds of seven descents of MoveIt's
//! KDL plugin, converged, stuck and at the step cap, on OpenArm v2's left arm, each
//! descent here ends within 1e-6 rad of KDL's after as many passes of its loop
//! (`src/ik.rs`, `tests/fixtures/kdl_descents.json`).
#![forbid(unsafe_code)]

mod chain;
mod draws;
mod dynamics;
mod error;
mod ik;
mod jacobian;
mod line;
mod payload;
mod rate;
mod tree;

pub use chain::{Chain, ChainSpec, JointSelection, Posed};
pub use draws::Draws;
pub use error::ChainError;
pub use ik::{Aims, ChainFrame, Ends, IkSolver, RandomSource, SEARCH_ITERATIONS};
pub use jacobian::{Jacobian, JacobianPinv, damped_pseudo_inverse};
pub use line::{
    CARTESIAN_STEP_M, CARTESIAN_STEP_RAD, COINCIDENT_ARC_EPS, LinePrecision, LineSolver, LineState,
    MAX_RESOLUTION, PRECISION, PRECISION_M, PRECISION_RAD, PrecisionError, WalkEnd, interpolate,
    line_blends, line_extents, line_precision, walk_line,
};
pub use payload::{Payload, segments_carrying};
pub use rate::{
    DEFAULT_DLS_LAMBDA, EeCaps, PoseMiss, StepLimits, pose_miss, rate_step, rate_step_toward,
};
pub use tree::{JointKind, Tree};

/// Re-exported so downstream crates speak the same `nalgebra` version this crate
/// was built against, rather than each picking their own and finding the
/// `Isometry3`s do not match.
pub use nalgebra;

/// How far outside a joint limit a target may lie and still be taken, at that
/// limit (rad, or m for a prismatic joint). A measured position sent back as
/// a target lies that close: a joint that a posture holds at a limit settles
/// a hair on either side of it.
pub const TARGET_SLACK: f64 = 0.001;

/// Inclusive joint position limit, radians (or metres for a prismatic joint).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limit {
    pub lo: f64,
    pub hi: f64,
}

impl Limit {
    /// True if `x` lies within `[lo, hi]`. Non-finite `x` (NaN/inf) compares
    /// false on both sides, so it is rejected.
    pub fn contains(&self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }

    /// `x` as a target of the joint: `x` inside the limit, the limit for an
    /// `x` at most [`TARGET_SLACK`] outside it, and none for an `x` farther
    /// out or not finite.
    pub fn take_target(&self, x: f64) -> Option<f64> {
        let with_slack = Limit {
            lo: self.lo - TARGET_SLACK,
            hi: self.hi + TARGET_SLACK,
        };
        with_slack.contains(x).then(|| self.clamp(x))
    }

    /// `x` brought inside the limit.
    pub fn clamp(&self, x: f64) -> f64 {
        x.clamp(self.lo, self.hi)
    }

    /// The centre of the range. The seed of last resort for a caller with no
    /// better one: an all-zeros configuration sits exactly on a limit for any
    /// joint whose range does not straddle zero.
    pub fn midpoint(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }
}

/// A joint of a target farther outside its limit than [`TARGET_SLACK`]: the
/// joint counted from 0, its position and its limit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutsideLimits {
    pub joint: usize,
    pub position: f64,
    pub limit: Limit,
}

impl std::fmt::Display for OutsideLimits {
    /// The joint counted from 1, with every digit of its position and its
    /// limits, so that a position a hair outside a limit reads apart from it.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "joint {} at {} rad, outside its limits [{}, {}]",
            self.joint + 1,
            self.position,
            self.limit.lo,
            self.limit.hi
        )
    }
}

/// `q` as a target ([`Limit::take_target`]): each joint inside its limit as
/// it is, each at most [`TARGET_SLACK`] outside it at the limit, or the
/// first joint that lies farther out.
pub fn take_targets<const N: usize>(
    limits: &[Limit; N],
    q: &[f64; N],
) -> Result<[f64; N], OutsideLimits> {
    let mut taken = *q;
    for (joint, (position, limit)) in taken.iter_mut().zip(limits).enumerate() {
        *position = limit.take_target(*position).ok_or(OutsideLimits {
            joint,
            position: *position,
            limit: *limit,
        })?;
    }
    Ok(taken)
}

/// `q` with every joint brought inside its limit.
pub fn clamp_into<const N: usize>(limits: &[Limit; N], q: &[f64; N]) -> [f64; N] {
    std::array::from_fn(|i| limits[i].clamp(q[i]))
}

/// True if every joint of `q` lies inside its limit.
pub fn all_within<const N: usize>(limits: &[Limit; N], q: &[f64; N]) -> bool {
    limits.iter().zip(q).all(|(limit, v)| limit.contains(*v))
}
