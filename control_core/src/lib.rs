//! Shared control-loop and runtime primitives: pacing, filtering, rate
//! limiting, the timing of a motion, the health of a motor, a mobile base's
//! footprint, its body velocity, its limiter and its wheel odometry, where a
//! frame sits on the floor plane, the geometry types of the contracts, the
//! setpoint stream, the single-flight move slot of a goal, the service,
//! receive and ask loops, the fault that ends a node, the model a component is
//! configured for, latched failure logging, a node's wall or simulation clock,
//! a robot's tree of frames with the lookups that wait on that clock, waiting
//! and locking from synchronous code, the reason a robot stop gives each move
//! it ends, drive_link's window and follow_state, and the arms a request
//! names, for robot nodes and the kinematics crates that plan their moves.
//!
//! - [`pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//! - [`chase`]: the rate-limited chase that absorbs a setpoint's target jumps
//!   ([`chase_step`](chase::chase_step)), and the chase along the line to a
//!   target ([`chase_along`](chase::chase_along)).
//! - [`filters`]: scalar signal filters ([`LowPassFilter`](filters::LowPassFilter),
//!   [`ButterworthFilter`](filters::ButterworthFilter),
//!   [`HampelFilter`](filters::HampelFilter)).
//! - [`time`]: a configured rate or seconds value as a `Duration` to wait on,
//!   a parameter of seconds refused by its name,
//!   the ticks a window spans, waits a cancel ends, async and blocking, and
//!   times on a ROS clock as integer nanoseconds.
//! - [`minimum_jerk`]: the quintic blend between two values and its peak rate.
//! - [`path_timing`]: a path through waypoints rounded at each corner
//!   within a deviation, its time-optimal timing under velocity and
//!   acceleration limits, and the trajectory that plays a joint path.
//! - [`timing`]: the clock a motion plays on and the duration a caller may ask
//!   of it.
//! - [`node_clock`]: the clock a node reads, the wall clock or a simulation
//!   clock fed the ticks the node receives, the time now, a wait until a
//!   deadline on it, whether it went back between two looks, and a time of it
//!   on the wire.
//! - [`robot_tree`]: a robot's tree of frames as a node holds it, its followers
//!   that fill it from the messages of transforms, the lookups in it that wait
//!   on the node's clock until the tree answers them
//!   ([`lookup_within`](robot_tree::RobotTree::lookup_within),
//!   [`lookup_within_through`](robot_tree::RobotTree::lookup_within_through)),
//!   and a value of the geometry types placed in a frame
//!   ([`place`](robot_tree::RobotTree::place)) or in the root of its frame's
//!   chain ([`place_in_root`](robot_tree::RobotTree::place_in_root)), refused
//!   with a [`PlaceError`](robot_tree::PlaceError), the failure of a command
//!   the tree cannot place in a node's own failure kinds
//!   ([`TargetFailure`](robot_tree::TargetFailure)), and the
//!   [`Placer`](robot_tree::Placer) of a node's commands.
//! - [`motor_health`]: judging one motor's load, temperature and condition
//!   against its ratings.
//! - [`move_slot`]: the single-flight move slot a goal claims on its admission and
//!   releases on its end, which a stop marks for a goal its loop has not taken.
//! - [`planar_pose`]: where one frame sits in another on the floor plane, with its heading read
//!   as Nav2's controller reads it, in frame_tree's arithmetic.
//! - [`twist`]: a body velocity on the floor plane in the robot frame, finite by construction.
//! - [`geometry`]: the geometry types of the contracts, each parsed from a generated record's
//!   fields, their planar forms, and behind the `nalgebra` feature their nalgebra forms.
//! - [`limiter`]: the limiter of a base's body velocity, as Nav2 1.5.2's
//!   velocity smoother limits a stream of setpoints in open loop: each axis
//!   clamped to its velocity range, ramped at its acceleration and
//!   deceleration limits, the axes scaled together, and brought to rest once
//!   the setpoints stop.
//! - [`odometry`]: the wheel odometry of a differential base, the robot frame
//!   in the odometry frame integrated from the wheel positions as
//!   ros2_controllers 6.9.0's diff drive controller integrates it; the wheel
//!   rates of a body velocity, as that controller mixes them; and the wheel
//!   setpoints that drive the base, each wheel's angle anchored by
//!   [`odometry::WheelSetpoint::next`].
//! - [`non_negative_finite`]: a finite float of at least zero, carried as proof it was
//!   checked.
//! - [`move_seconds`]: the seconds a move goal asks for, within the bound
//!   limb_motion and base_motion state
//!   ([`MAX_MOVE_DURATION_S`](move_seconds::MAX_MOVE_DURATION_S)), parsed off the wire.
//! - [`footprint`]: a mobile base's footprint, the outline it covers about its
//!   center.
//! - [`positive_finite`]: a positive finite float, carried as proof it was checked,
//!   and a field of a request parsed into one
//!   ([`PositiveFinite::parse_field`](positive_finite::PositiveFinite::parse_field)).
//! - [`receiving`]: the loop that receives one subscription for the life of
//!   a node, with the pause after a receive error.
//! - [`serving`]: the loop that serves one service for the life of a node,
//!   with the pause after a runtime error.
//! - [`asking`]: the loop that asks another node until it answers, with the
//!   pause after a failed ask, the wait until another node is ready
//!   ([`ask_until_ready`](asking::ask_until_ready)), and the ask of a loop of the
//!   node's own within a timeout ([`ask_loop`](asking::ask_loop)).
//! - [`fault`]: the fault that ends a node after its setup has returned, the
//!   error the node exits with for it, and the task whose panic raises it.
//! - [`component_model`]: the model of the robot a component is a part of, as
//!   the robot's initializer names it on each readiness call.
//! - [`setpoint_stream`]: the latest setpoint, published to a sink on a fixed
//!   interval until cancelled, with one warning per failure burst.
//! - [`throttle`]: admitting a repeating event at most once per window, and
//!   the window of a node's throttled failure log.
//! - [`report_once`]: marking a run of repeated failures, so only its first is
//!   reported.
//! - [`latched_log`]: one log line per run of repeated failures, re-armed by
//!   a success.
//! - [`sync`]: waiting on a future and taking a lock from synchronous code
//!   inside the runtime.
//! - [`stop_reason`]: the reason a robot stop gives each move it ends, the
//!   stop and the reason of the stop request.
//! - [`stream_window`]: whether a leader's stream of commands is live, its
//!   newest command within the window its link's contract states.
//! - [`drive_link`]: drive_link's window of the leader's stream and the
//!   follow_state of each drive_states, shared by its leaders and its follower.
//! - [`requested_arms`]: the arms a request names, their places among the robot's arms in
//!   the request's order, the place of an arm or a gripper it names by name, the limb a
//!   node's parameter names by its place, and the waypoints every arm of a joint path
//!   passes through.
//!
//! Each fallible operation names its own failure, so a signature says exactly
//! what can go wrong with it.

pub mod asking;
pub mod chase;
pub mod component_model;
pub mod drive_link;
pub mod fault;
pub mod filters;
pub mod footprint;
pub mod geometry;
pub mod latched_log;
pub mod limiter;
pub mod minimum_jerk;
pub mod motor_health;
pub mod move_seconds;
pub mod move_slot;
pub mod node_clock;
pub mod non_negative_finite;
pub mod odometry;
pub mod pacer;
pub mod path_timing;
pub mod planar_pose;
pub mod positive_finite;
pub mod receiving;
pub mod report_once;
pub mod requested_arms;
pub mod robot_tree;
pub mod serving;
pub mod setpoint_stream;
pub mod stop_reason;
pub mod stream_window;
pub mod sync;
#[cfg(test)]
mod test_support;
pub mod throttle;
pub mod time;
pub mod timing;
pub mod twist;
