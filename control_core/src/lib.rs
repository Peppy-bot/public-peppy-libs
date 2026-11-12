//! Shared control-loop and runtime primitives: pacing, filtering, rate
//! limiting, the timing of a motion, the health of a motor, the setpoint
//! stream, the service, receive and ask loops, the fault that ends a node, the
//! model a component is configured for, latched failure logging, waiting and
//! locking from synchronous code, and the arms a request names, for robot
//! nodes and the kinematics crates that plan their moves.
//!
//! - [`pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//! - [`chase`]: the rate-limited chase that absorbs a setpoint's target jumps
//!   ([`chase_step`](chase::chase_step)), and the chase along the line to a
//!   target ([`chase_along`](chase::chase_along)).
//! - [`filters`]: scalar signal filters ([`LowPassFilter`](filters::LowPassFilter),
//!   [`ButterworthFilter`](filters::ButterworthFilter),
//!   [`HampelFilter`](filters::HampelFilter)).
//! - [`time`]: a configured rate or seconds value as a `Duration` to wait on,
//!   the ticks a window spans, and waits a cancel ends, async and blocking.
//! - [`minimum_jerk`]: the quintic blend between two values and its peak rate.
//! - [`path_timing`]: a path through waypoints rounded at each corner
//!   within a deviation, its time-optimal timing under velocity and
//!   acceleration limits, and the trajectory that plays a joint path.
//! - [`timing`]: the clock a motion plays on and the duration a caller may ask
//!   of it.
//! - [`motor_health`]: judging one motor's load, temperature and condition
//!   against its ratings.
//! - [`positive_finite`]: a positive finite float, carried as proof it was checked.
//! - [`receiving`]: the loop that receives one subscription for the life of
//!   a node, with the pause after a receive error.
//! - [`serving`]: the loop that serves one service for the life of a node,
//!   with the pause after a runtime error.
//! - [`asking`]: the loop that asks another node until it answers, with the
//!   pause after a failed ask.
//! - [`fault`]: the fault that ends a node after its setup has returned, the
//!   error the node exits with for it, and the task whose panic raises it.
//! - [`component_model`]: the model of the robot a component is a part of, as
//!   the robot's initializer names it on each readiness call.
//! - [`setpoint_stream`]: the latest setpoint, published to a sink on a fixed
//!   interval until cancelled, with one warning per failure burst.
//! - [`throttle`]: admitting a repeating event at most once per window.
//! - [`report_once`]: marking a run of repeated failures, so only its first is
//!   reported.
//! - [`latched_log`]: one log line per run of repeated failures, re-armed by
//!   a success.
//! - [`sync`]: waiting on a future and taking a lock from synchronous code
//!   inside the runtime.
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
pub mod fault;
pub mod filters;
pub mod latched_log;
pub mod minimum_jerk;
pub mod motor_health;
pub mod pacer;
pub mod path_timing;
pub mod positive_finite;
pub mod receiving;
pub mod report_once;
pub mod requested_arms;
pub mod serving;
pub mod setpoint_stream;
pub mod sync;
#[cfg(test)]
mod test_support;
pub mod throttle;
pub mod time;
pub mod timing;
