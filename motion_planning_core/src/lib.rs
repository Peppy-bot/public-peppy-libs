//! Plans the paths of one or more serial chains of joints, moved together, around a voxel
//! map of what the robot's cameras see and the objects of its planning scene. It knows no
//! robot: a robot's node gives it the chains, the parts their links carry, the robot's
//! self model where the robot has one, its links, and a goal sampler for each chain.
//!
//! - [`Arm`]: a chain, the collision pieces of its links and the
//!   [`Rider`](collision_geometry::chain::Rider)s its links carry, parts of the robot and
//!   the shapes of objects attached to its grasp link, each placed as a [`Body`], and how
//!   far a joint moves any point of them. [`Arms`]: the arms a plan moves together, a
//!   state holding their joints one arm after another, and how far a motion moves any
//!   point of them; [`SelfModel`] is the robot's check of its own body.
//! - [`PlanningScene`]: the collision objects, the attached objects and the pairs
//!   allowed to touch, as MoveIt's planning scene holds them, with every rule of
//!   motion_planning's scene members; a plan meets them as [`PlanObjects`].
//! - [`Surroundings`]: whether the arms at a state keep their clearance from the map and
//!   the [`Objects`] that stand still, within the robot's own body and from one another,
//!   and between an attached object and the arm that carries it, each term the clearance
//!   [`term_clearances`] asks of it, and the distance of each pair it checks.
//! - [`Goal`]: the constraints an arm's last waypoint meets; [`sample_goals`] samples
//!   configurations meeting a plan's [`ArmGoal`]s as MoveIt's goal sampler does, one goal
//!   sample at a time by each arm's [`GoalSampler`] from the plan's [`Draws`],
//!   [`IkGoalSampler`] by a port of MoveIt's KDL inverse kinematics for any chain.
//! - [`plan_path`]: plans a [`PlanRequest`] on a [`Scene`], each arm to its [`ArmGoal`],
//!   and checks the rounded path the robot runs; [`check_path`] checks a caller's path, a
//!   [`CheckRequest`], with the same checks.
//! - [`cartesian_path`]: a frame of one arm along straight lines through the poses of a
//!   [`CartesianRequest`], as MoveIt's `computeCartesianPath` follows them, as much of them
//!   as is valid, and the [`CartesianPath`]'s fraction covered.
//! - The map's intake of a depth frame: a [`DepthCamera`]'s points of the frame, the
//!   robot's joint positions at the frame's stamp from a [`JointHistory`], and
//!   [`insert_cut_out`], which inserts the points with the robot there, a [`RobotMask`]
//!   of its [`PosedChain`]s, its body and the objects, cut out.
#![forbid(unsafe_code)]

mod arm;
mod arms;
mod cartesian;
mod clearance;
mod draws;
mod goal;
mod goal_ik;
mod intake;
mod plan;
mod pose;
mod scene;
mod stop;

pub use arm::{Arm, Body, Moving, RiderOffTheArm, SelfModel};
pub use arms::{Arms, GroupBody};
pub use cartesian::{
    CARTESIAN_STEP_M, CARTESIAN_STEP_RAD, CartesianFailure, CartesianPath, CartesianRequest,
    Collisions, FollowedFrame, PRECISION_M, PRECISION_RAD, cartesian_path, poses_in_range,
};
pub use clearance::{Contact, Reading, Surroundings, TERMS, term_clearances};
pub use draws::Draws;
pub use goal::{
    Axis, Frame, Goal, GoalError, GoalSampler, IkGoalSampler, JointTarget, OrientationTarget,
    PositionTarget,
};
pub use intake::{DepthCamera, JointHistory, PosedChain, RobotMask, insert_cut_out};
pub use plan::{
    ArmGoal, CheckFailure, CheckRequest, MAX_STEP_M, PathCheck, PathFault, Place, PlanFailure,
    PlanRequest, Scene, StartFault, check_path, plan_path, sample_goals, start_inside_limits,
};
pub use pose::{orientation_from_parts, pose_from_parts, pose_parts};
pub use scene::{
    AttachRequest, AttachedObjectFields, OCCUPANCY_MAP, ObjectFields, ObjectRequest, Objects,
    Obstacle, PlanObjects, PlanningScene, RobotLinks, RobotPart, SceneError, SceneSnapshot,
    ShapeFields,
};
