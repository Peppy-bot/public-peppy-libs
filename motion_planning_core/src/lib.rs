//! Plans the paths of one serial chain of joints around a voxel map of what the
//! robot's cameras see. It knows no robot: a robot's node gives it the chain, the parts
//! its links carry, the robot's self model where the robot has one, and a goal sampler.
//!
//! - [`Arm`]: the chain, the collision pieces of its links and of the
//!   [`Rider`](collision_geometry::chain::Rider)s its links carry, and how far a joint
//!   moves any point of them; [`SelfModel`] is the robot's check of its own body.
//! - [`Surroundings`]: whether the arm at a configuration keeps its clearance from the
//!   map and between the pairs of its self model.
//! - [`Goal`]: the constraints a path's last waypoint meets; a [`GoalSampler`] finds
//!   configurations meeting them, [`IkGoalSampler`] by inverse kinematics for any chain.
//! - [`plan_arm_path`]: plans a [`PlanRequest`] on a [`Scene`], and checks the rounded
//!   path the robot runs; [`check_arm_path`] checks a caller's path, a [`CheckRequest`],
//!   with the same checks.
#![forbid(unsafe_code)]

mod arm;
mod clearance;
mod draws;
mod goal;
mod plan;
mod stop;

pub use arm::{Arm, RiderOffTheArm, SelfModel};
pub use clearance::{Contact, Surroundings};
pub use goal::{
    Axis, Frame, Goal, GoalError, GoalSampler, IkGoalSampler, JointTarget, OrientationTarget,
    PositionTarget,
};
pub use plan::{
    CheckFailure, CheckRequest, MAX_STEP_M, PathCheck, PathFault, Place, PlanFailure, PlanRequest,
    Scene, StartFault, check_arm_path, plan_arm_path, start_inside_limits,
};
