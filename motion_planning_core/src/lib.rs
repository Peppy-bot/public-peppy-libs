//! Plans the paths of one serial chain of joints around a voxel map of what the
//! robot's cameras see and the objects of its planning scene. It knows no robot: a
//! robot's node gives it the chain, the parts its links carry, the robot's self model
//! where the robot has one, its links, and a goal sampler.
//!
//! - [`Arm`]: the chain, the collision pieces of its links and the
//!   [`Rider`](collision_geometry::chain::Rider)s its links carry, parts of the robot and
//!   the shapes of objects attached to its grasp link, each placed as a [`Body`], and how
//!   far a joint moves any point of them; [`SelfModel`] is the robot's check of its own
//!   body.
//! - [`PlanningScene`]: the collision objects, the attached objects and the pairs
//!   allowed to touch, as MoveIt's planning scene holds them, with every rule of
//!   motion_planning's scene members; a plan of one arm meets them as [`ArmObjects`].
//! - [`Surroundings`]: whether the arm at a configuration keeps its clearance from the
//!   map and the [`Objects`] that stand still, within its own body, and between an
//!   attached object and the arm that carries it, each term the clearance
//!   [`term_clearances`] asks of it, and the distance of each pair it checks.
//! - [`Goal`]: the constraints a path's last waypoint meets; [`sample_goals`] samples
//!   configurations meeting them as MoveIt's goal sampler does, one goal sample at a time
//!   by a [`GoalSampler`] from the plan's [`Draws`], [`IkGoalSampler`] by a port of
//!   MoveIt's KDL inverse kinematics for any chain.
//! - [`plan_arm_path`]: plans a [`PlanRequest`] on a [`Scene`], and checks the rounded
//!   path the robot runs; [`check_arm_path`] checks a caller's path, a [`CheckRequest`],
//!   with the same checks.
//! - The map's intake of a depth frame: a [`DepthCamera`]'s points of the frame, the
//!   robot's joint positions at the frame's stamp from a [`JointHistory`], and
//!   [`insert_cut_out`], which inserts the points with the robot there, a [`RobotMask`]
//!   of its [`PosedChain`]s, its body and the objects, cut out.
#![forbid(unsafe_code)]

mod arm;
mod clearance;
mod draws;
mod goal;
mod goal_ik;
mod intake;
mod plan;
mod pose;
mod scene;
mod stop;

pub use arm::{Arm, Body, RiderOffTheArm, SelfModel};
pub use clearance::{Contact, Reading, Surroundings, TERMS, term_clearances};
pub use draws::Draws;
pub use goal::{
    Axis, Frame, Goal, GoalError, GoalSampler, IkGoalSampler, JointTarget, OrientationTarget,
    PositionTarget,
};
pub use intake::{DepthCamera, JointHistory, PosedChain, RobotMask, insert_cut_out};
pub use plan::{
    CheckFailure, CheckRequest, MAX_STEP_M, PathCheck, PathFault, Place, PlanFailure, PlanRequest,
    Scene, StartFault, check_arm_path, plan_arm_path, sample_goals, start_inside_limits,
};
pub use pose::{orientation_from_parts, pose_from_parts, pose_parts};
pub use scene::{
    ArmObjects, AttachRequest, AttachedObjectFields, OCCUPANCY_MAP, ObjectFields, ObjectRequest,
    Objects, Obstacle, PlanningScene, RobotLinks, RobotPart, SceneError, SceneSnapshot,
    ShapeFields,
};
