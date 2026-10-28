//! Shared control-loop and runtime primitives: pacing, filtering, rate
//! limiting, the timing of a motion, the health of a motor, the setpoint
//! stream, the service and receive loops, latched failure logging, and
//! waiting and locking from synchronous code, for robot nodes and the
//! kinematics crates that plan their moves.
//!
//! - [`pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//! - [`chase`]: the rate-limited chase that absorbs a setpoint's target jumps.
//! - [`filters`]: scalar signal filters ([`LowPassFilter`](filters::LowPassFilter),
//!   [`ButterworthFilter`](filters::ButterworthFilter),
//!   [`HampelFilter`](filters::HampelFilter)).
//! - [`time`]: a configured rate or seconds value as a `Duration` to wait on,
//!   the ticks a window spans, and waits a cancel ends, async and blocking.
//! - [`minimum_jerk`]: the quintic blend between two values and its peak rate.
//! - [`path_timing`]: the time-optimal timing of a path through waypoints
//!   under velocity and acceleration limits, and the trajectories that play
//!   it: a timed path over a duration, and the straight joint move.
//! - [`timing`]: the clock a motion plays on, the duration a caller may ask
//!   of it, its control ticks, and the check that what they command moves and
//!   accelerates within the limits.
//! - [`motor_health`]: judging one motor's load, temperature and condition
//!   against its ratings.
//! - [`positive_finite`]: a positive finite float, carried as proof it was checked.
//! - [`receiving`]: the loop that receives one subscription for the life of
//!   a node, with the pause after a receive error.
//! - [`serving`]: the loop that serves one service for the life of a node,
//!   with the pause after a runtime error.
//! - [`servo`]: the tolerances that decide when a Cartesian goal is reached.
//! - [`setpoint_stream`]: the latest setpoint, published to a sink on a fixed
//!   interval until cancelled, with one warning per failure burst.
//! - [`throttle`]: admitting a repeating event at most once per window.
//! - [`report_once`]: marking a run of repeated failures, so only its first is
//!   reported.
//! - [`latched_log`]: one log line per run of repeated failures, re-armed by
//!   a success.
//! - [`sync`]: waiting on a future and taking a lock from synchronous code
//!   inside the runtime.
//!
//! Each fallible operation names its own failure, so a signature says exactly
//! what can go wrong with it.

pub mod chase;
pub mod filters;
pub mod latched_log;
pub mod minimum_jerk;
pub mod motor_health;
pub mod pacer;
pub mod path_timing;
pub mod positive_finite;
pub mod receiving;
pub mod report_once;
pub mod serving;
pub mod servo;
pub mod setpoint_stream;
pub mod sync;
#[cfg(test)]
mod test_support;
pub mod throttle;
pub mod time;
pub mod timing;
