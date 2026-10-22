//! Shared control-loop primitives: pacing, filtering, rate limiting, the
//! timing of a motion and the health of a motor, for robot nodes and the
//! kinematics crates that plan their moves.
//!
//! - [`pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//! - [`chase`]: the rate-limited chase that absorbs a setpoint's target jumps.
//! - [`filters`]: scalar signal filters ([`LowPassFilter`](filters::LowPassFilter),
//!   [`ButterworthFilter`](filters::ButterworthFilter),
//!   [`HampelFilter`](filters::HampelFilter)).
//! - [`time`]: a configured rate or seconds value as a `Duration` to wait on.
//! - [`minimum_jerk`]: the quintic blend between two values and its peak rate.
//! - [`path_timing`]: the time-optimal timing of a path through waypoints
//!   under velocity and acceleration limits, and the trajectories that play
//!   it: a timed path over a duration, and the straight joint move.
//! - [`timing`]: the clock a motion plays on, the duration a caller may ask
//!   of it, its control ticks, and the check that what they command moves and
//!   accelerates within the limits.
//! - [`move_budget`]: how long a move is allowed to take, and how that moves
//!   with the speed the operator retunes.
//! - [`motor_health`]: judging one motor's load, temperature and condition
//!   against its ratings.
//! - [`positive_finite`]: a positive finite float, carried as proof it was checked.
//! - [`servo`]: the tolerances that decide when a Cartesian goal is reached.
//! - [`throttle`]: admitting a repeating event at most once per window.
//!
//! Each fallible operation names its own failure, so a signature says exactly
//! what can go wrong with it.

pub mod chase;
pub mod filters;
pub mod minimum_jerk;
pub mod motor_health;
pub mod move_budget;
pub mod pacer;
pub mod path_timing;
pub mod positive_finite;
pub mod servo;
pub mod throttle;
pub mod time;
pub mod timing;
