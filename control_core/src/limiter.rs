//! The limiter of a base's body velocity: a stream of setpoints limited as
//! Nav2 1.5.2's velocity smoother limits it in open loop with
//! `scale_velocities` on (nav2_velocity_smoother src/velocity_smoother.cpp,
//! `VelocitySmoother::smootherTimer`, `findEtaConstraint` and
//! `applyConstraints`, its three-axis branch), and as base_state:v1's limiter
//! bounds each step of period dt, by an axis's limit times dt, the limiter
//! drive_link's follower commands a setpoint through.
//!
//! Each step of [`Limiter::tick`] is one cycle of Nav2's smoother: the latest
//! setpoint is clamped to each axis's velocity range, which holds 0, and the
//! velocity of the last step moves toward it by at most
//! `max_acceleration / smoothing_frequency_hz` while the clamped value is at
//! least as large in magnitude as the velocity and not of the opposite sign,
//! and by at most `max_deceleration / smoothing_frequency_hz` otherwise, each
//! bound computed as Nav2 computes it (velocity_smoother.cpp:270-276,
//! :301-307). The axes scale together: every axis moves by the share of its
//! change that keeps the most limited axis within its step, so each step's
//! change points toward the clamped setpoint. A step computes the velocity
//! plus its clamped change, `current + clamp(eta * change)`, in double
//! precision as Nav2 does (:309), so a step that reaches an end of the range
//! lands on it within the roundings of the change and of that sum, Nav2's
//! bits. Once the newest setpoint is older than `velocity_timeout_s`, the
//! setpoint becomes zero, so the velocity ramps to rest at the deceleration
//! limits; once at rest past the timeout, a tick gives nothing until the next
//! setpoint. Times are integer nanoseconds, as ROS compares them. The deadband
//! zeroes an axis of the output whose magnitude lies below it, and the next
//! step starts from the velocity before the deadband, as the smoother does
//! (:468-481).
//!
//! The steps fall on the grid of the smoother's timer. Nav2 makes its timer of
//! a period of whole milliseconds, 1000 / `smoothing_frequency_hz` truncated
//! (velocity_smoother.cpp:169-171), and rcl 10.4.5 runs it (rcl
//! src/rcl/timer.c). The limiter takes only a frequency whose period 1000 /
//! `smoothing_frequency_hz` is a whole number of milliseconds, so the period
//! is 1 / `smoothing_frequency_hz` and a step's bound, a limit /
//! `smoothing_frequency_hz`, is that limit times the period. The first tick
//! steps and starts the grid; rcl's timer first calls one period after it is
//! made (timer.c:146-147), and the limiter holds no time before its first
//! tick. A tick at or after the next slot steps, and the next slot becomes
//! the first slot of the grid after that tick, as `rcl_timer_call` moves its
//! next call (timer.c:274-291), so a late tick keeps the grid. A tick before
//! the next slot gives the last output and does not step. A tick before the
//! last call, the last tick that stepped or started the grid again (rcl's
//! `last_call_time`), comes from a clock that went back: it does not step,
//! and starts the grid again one period after itself, as the timer's jump
//! callback does for a jump back (timer.c:93-98). rcl calls that callback
//! only when a ROS time override is enabled, disabled or set (rcl
//! src/rcl/time.c:311-313, :333-335, :380-382); on the system clock, which a
//! ROS clock reads while no override is active (time.c:68-69), the timer
//! keeps its next call and calls nothing until the clock reaches it again
//! (timer.c:316). The limiter starts the grid again on any clock that goes
//! back, so it keeps stepping after a system clock steps back, where Nav2's
//! smoother publishes nothing until the clock catches up, and the drive holds
//! the last command the smoother published only until the drive's command
//! timeout: drive_link's follower ends a leader's stream 0.5 s after its last
//! setpoint and brings the base to rest. So at most one step falls in each
//! period of the grid, and each moves an axis by at most its limit times the
//! period, the bound base_state:v1's limiter sets on a step.
//!
//! A setpoint whose time lies after a tick's time arrived before a clock that
//! went back, and each tick gives it that tick's time, so a setpoint from
//! before a step back holds for `velocity_timeout_s` from the first tick after
//! the step, and the velocity then ramps to rest, as drive_link's follower
//! ends a leader's stream 0.5 s after its last setpoint. Here the limiter
//! departs from Nav2's smoother, whose timer runs again after a jump back of
//! a ROS time override: it ages its last command as `now() -
//! last_command_time_`, below 0 after the step back, and drives that command
//! until the clock passes the command's time plus `velocity_timeout`
//! (velocity_smoother.cpp:320, :330).
//!
//! An axis's limits take one of two shapes, as describe_base of base_state:v1
//! gives them: an axis the base does not move along, whose range is 0 to 0 and
//! whose `max_acceleration`, `max_deceleration` and `deadband` are 0, as
//! Nav2's smoother takes such an axis; and an axis it moves along, whose range
//! holds 0 and is not 0 to 0, whose `max_acceleration` and `max_deceleration`
//! lie above 0, so the axis moves toward its setpoint and comes to rest, and
//! whose deadband lies from 0 to below the larger magnitude of its range. The
//! width of a range, `max_velocity - min_velocity`, is finite, so the change
//! between two velocities of the range is finite; Nav2 refuses only a range
//! that does not hold 0 (velocity_smoother.cpp:110-127).

use thiserror::Error;

use crate::non_negative_finite::NonNegativeFinite;
use crate::positive_finite::{NotPositiveFinite, PositiveFinite};
use crate::time::{elapsed_ns, nanoseconds};
use crate::twist::Twist;

/// The frequencies the limiter steps at, in Hz: each whole number of hertz whose period,
/// 1000 / the frequency, is a whole number of milliseconds, from 20 Hz, a step of at most
/// 0.05 s as base_state:v1 bounds a step of its limiter and the period of Nav2's smoother
/// at its default 20 Hz, to 1000 Hz, the fastest whose period in whole milliseconds, as
/// Nav2 makes its smoother's timer, is not 0.
const SMOOTHING_FREQUENCIES_HZ: [f64; 10] = [
    20.0, 25.0, 40.0, 50.0, 100.0, 125.0, 200.0, 250.0, 500.0, 1000.0,
];

const MILLISECONDS_PER_SECOND: f64 = 1000.0;

const NANOSECONDS_PER_MILLISECOND: i64 = 1_000_000;

/// A limit that cannot bound a velocity, or a frequency the limiter cannot step at.
#[derive(Debug, Clone, Error)]
pub enum LimiterError {
    #[error("{name} {reason}: give {name} above 0 for an axis the base moves along")]
    NotPositive {
        name: &'static str,
        reason: NotPositiveFinite,
    },
    #[error(
        "the velocity range of an axis must hold 0, min_velocity at most 0 and max_velocity \
         at least 0: got {min} and {max}"
    )]
    Range { min: f64, max: f64 },
    #[error(
        "the velocity range of an axis from {min} to {max} has no finite width: give a \
         finite min_velocity and max_velocity whose difference is finite"
    )]
    RangeWidth { min: f64, max: f64 },
    #[error(
        "an axis whose range is 0 to 0 has 0 max_acceleration, max_deceleration and deadband, \
         as in Nav2's velocity smoother: got {max_acceleration}, {max_deceleration} and \
         {deadband}"
    )]
    StillAxis {
        max_acceleration: f64,
        max_deceleration: f64,
        deadband: f64,
    },
    #[error(
        "the deadband {deadband} of an axis is no smaller than its fastest velocity \
         {fastest}: give a deadband below the larger magnitude of min_velocity and \
         max_velocity"
    )]
    Deadband { deadband: f64, fastest: f64 },
    #[error(
        "smoothing_frequency_hz {0} is not a frequency the limiter steps at: give one of \
         {frequencies} Hz, whose period is a whole number of milliseconds from 1 to 50 ms",
        frequencies = listed(&SMOOTHING_FREQUENCIES_HZ)
    )]
    Frequency(f64),
}

/// The limits of one axis of a body velocity, by the names of an axis of
/// describe_base's `linear_limits` (its `yaw_limits` names the range
/// `min_rate` and `max_rate`): an axis the base does not move along, of range
/// 0 to 0 and 0 limits, or one it moves along.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimitsParams {
    /// The lowest velocity, at most 0.
    pub min_velocity: f64,
    /// The highest velocity, at least 0.
    pub max_velocity: f64,
    /// How fast the axis speeds up, a magnitude per second.
    pub max_acceleration: NonNegativeFinite,
    /// How fast the axis slows down, a magnitude per second.
    pub max_deceleration: NonNegativeFinite,
    /// The magnitude below which a command executes as 0.
    pub deadband: NonNegativeFinite,
}

/// The limits of one axis, of either shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimits {
    min_velocity: f64,
    max_velocity: f64,
    max_acceleration: f64,
    max_deceleration: f64,
    deadband: f64,
}

impl AxisLimits {
    /// The limits of `params`: a range that holds 0 and has a finite width;
    /// an axis of range 0 to 0 takes 0 for each limit; any other axis takes a
    /// `max_acceleration` and a `max_deceleration` above 0 and a deadband
    /// below its fastest velocity.
    pub fn parse(params: AxisLimitsParams) -> Result<Self, LimiterError> {
        let AxisLimitsParams {
            min_velocity,
            max_velocity,
            max_acceleration,
            max_deceleration,
            deadband,
        } = params;
        let (max_acceleration, max_deceleration, deadband) = (
            max_acceleration.get(),
            max_deceleration.get(),
            deadband.get(),
        );
        if !(min_velocity <= 0.0 && max_velocity >= 0.0) {
            return Err(LimiterError::Range {
                min: min_velocity,
                max: max_velocity,
            });
        }
        if !(max_velocity - min_velocity).is_finite() {
            return Err(LimiterError::RangeWidth {
                min: min_velocity,
                max: max_velocity,
            });
        }
        if min_velocity == 0.0 && max_velocity == 0.0 {
            if [max_acceleration, max_deceleration, deadband] != [0.0; 3] {
                return Err(LimiterError::StillAxis {
                    max_acceleration,
                    max_deceleration,
                    deadband,
                });
            }
            return Ok(Self {
                min_velocity: 0.0,
                max_velocity: 0.0,
                max_acceleration: 0.0,
                max_deceleration: 0.0,
                deadband: 0.0,
            });
        }
        let positive = |name, value| {
            PositiveFinite::parse(value)
                .map_err(|reason| LimiterError::NotPositive { name, reason })
        };
        positive("max_acceleration", max_acceleration)?;
        positive("max_deceleration", max_deceleration)?;
        let fastest = min_velocity.abs().max(max_velocity);
        if deadband >= fastest {
            return Err(LimiterError::Deadband { deadband, fastest });
        }
        Ok(Self {
            min_velocity,
            max_velocity,
            max_acceleration,
            max_deceleration,
            deadband,
        })
    }

    /// The lowest and the highest velocity of the axis.
    pub fn velocity_range(&self) -> (f64, f64) {
        (self.min_velocity, self.max_velocity)
    }

    /// The deadband, a magnitude: a command of smaller magnitude executes as
    /// 0; 0 on an axis of range 0 to 0.
    pub fn deadband(&self) -> f64 {
        self.deadband
    }

    /// Nav2's `max_decel`, the deceleration limit as a number of at most 0.
    fn signed_deceleration(&self) -> f64 {
        -self.max_deceleration
    }
}

/// How the limiter runs, by the names of Nav2's smoother: the limits of each
/// axis of a body velocity, by the names describe_base gives them, the
/// frequency of its steps, and how long a setpoint holds.
#[derive(Debug, Clone, PartialEq)]
pub struct LimiterParams {
    /// The limits of the forward velocity.
    pub linear_x: AxisLimits,
    /// The limits of the leftward velocity.
    pub linear_y: AxisLimits,
    /// The limits of the turn rate, counterclockwise.
    pub angular_z: AxisLimits,
    /// The rate the limiter steps at, in Hz (`smoothing_frequency`): a whole
    /// number of hertz from 20 to 1000 whose period, 1000 / the frequency, is
    /// a whole number of milliseconds.
    pub smoothing_frequency_hz: PositiveFinite,
    /// How long a setpoint holds, in seconds (`velocity_timeout`).
    pub velocity_timeout_s: NonNegativeFinite,
}

impl LimiterParams {
    /// The limits of the axis at `axis` in the order of [`Twist::axes`]:
    /// linear_x, linear_y, angular_z.
    fn axis(&self, axis: usize) -> &AxisLimits {
        [&self.linear_x, &self.linear_y, &self.angular_z][axis]
    }

    /// The step bounds of one axis this step: accelerating when the setpoint
    /// is at least as fast as the current velocity and on the same side of 0,
    /// decelerating otherwise. Nav2 writes them as `findEtaConstraint` and
    /// `applyConstraints` both do.
    fn step_bounds(&self, axis: usize, current: f64, setpoint: f64) -> (f64, f64) {
        let limits = self.axis(axis);
        let frequency = self.smoothing_frequency_hz.get();
        if setpoint.abs() >= current.abs() && current * setpoint >= 0.0 {
            let acceleration = limits.max_acceleration;
            (-acceleration / frequency, acceleration / frequency)
        } else {
            let deceleration = limits.signed_deceleration();
            (deceleration / frequency, -deceleration / frequency)
        }
    }

    /// `findEtaConstraint`: the share of the change that keeps the axis
    /// within its step bound, or -1 when the whole change fits.
    fn eta_constraint(&self, axis: usize, current: f64, setpoint: f64) -> f64 {
        let change = setpoint - current;
        let (step_min, step_max) = self.step_bounds(axis, current, setpoint);
        if change > step_max {
            return step_max / change;
        }
        if change < step_min {
            return step_min / change;
        }
        -1.0
    }

    /// `applyConstraints`: the axis moved by `eta` of its change, the step
    /// clamped to its bounds.
    fn constrained(&self, axis: usize, current: f64, setpoint: f64, eta: f64) -> f64 {
        let change = setpoint - current;
        let (step_min, step_max) = self.step_bounds(axis, current, setpoint);
        current + (eta * change).clamp(step_min, step_max)
    }
}

/// The timer's grid after its last call: when that call was, the last tick
/// that stepped or started the grid again (rcl's `last_call_time`), when the
/// next step falls, and what the last step gave.
#[derive(Debug, Clone, Copy)]
struct LastStep {
    called_ns: i64,
    next_slot_ns: i64,
    output: Option<Twist>,
}

/// The limiter's state: the latest setpoint and when it came, the velocity of
/// the last step, and the timer's grid.
#[derive(Debug)]
pub struct Limiter {
    params: LimiterParams,
    velocity_timeout_ns: i64,
    period_ns: i64,
    setpoint: Twist,
    setpoint_time_ns: i64,
    received_a_setpoint: bool,
    last_velocity: Twist,
    last_step: Option<LastStep>,
}

impl Limiter {
    /// The limiter of `params`, which steps at a whole number of hertz from 20
    /// to 1000 whose period is a whole number of milliseconds; a setpoint
    /// holds for `velocity_timeout_s` in nanoseconds as
    /// `rclcpp::Duration::from_seconds` makes it (velocity_smoother.cpp:75),
    /// and the timer's period is whole milliseconds as Nav2 makes it.
    pub fn parse(params: LimiterParams) -> Result<Self, LimiterError> {
        let frequency_hz = params.smoothing_frequency_hz.get();
        if !SMOOTHING_FREQUENCIES_HZ.contains(&frequency_hz) {
            return Err(LimiterError::Frequency(frequency_hz));
        }
        Ok(Self {
            velocity_timeout_ns: nanoseconds(params.velocity_timeout_s.get()),
            period_ns: timer_period_ns(frequency_hz),
            params,
            setpoint: Twist::ZERO,
            setpoint_time_ns: 0,
            received_a_setpoint: false,
            last_velocity: Twist::ZERO,
            last_step: None,
        })
    }

    /// Take a setpoint that arrived at `received_ns` on the caller's clock.
    pub fn take_setpoint(&mut self, setpoint: Twist, received_ns: i64) {
        self.setpoint = setpoint;
        self.setpoint_time_ns = received_ns;
        self.received_a_setpoint = true;
    }

    /// The velocity of the last step, before the deadband: where the next
    /// step starts from.
    pub fn last_velocity(&self) -> Twist {
        self.last_velocity
    }

    /// One tick at `now_ns`: the velocity to execute, or nothing before the
    /// first setpoint and once the base rests after the setpoints stopped. A
    /// tick steps on the timer's grid; any other tick gives the last step's
    /// output. A setpoint whose time lies after `now_ns`, from before a clock
    /// that went back, takes `now_ns` as its time.
    pub fn tick(&mut self, now_ns: i64) -> Option<Twist> {
        self.setpoint_time_ns = self.setpoint_time_ns.min(now_ns);
        if let Some(last) = self.last_step
            && now_ns < last.called_ns
        {
            self.last_step = Some(LastStep {
                called_ns: now_ns,
                next_slot_ns: now_ns.saturating_add(self.period_ns),
                ..last
            });
            return last.output;
        }
        if let Some(last) = self.last_step
            && now_ns < last.next_slot_ns
        {
            return last.output;
        }
        let output = self.step(now_ns);
        let slot_ns = self.last_step.map_or(now_ns, |last| last.next_slot_ns);
        self.last_step = Some(LastStep {
            called_ns: now_ns,
            next_slot_ns: slot_after(slot_ns, now_ns, self.period_ns),
            output,
        });
        output
    }

    /// One cycle of Nav2's smoother at `now_ns`. Past the timeout, a velocity of
    /// zero gives nothing, as Nav2's test of `last_cmd_` and `stopped_` does in
    /// open loop, where `stopped_` holds only while `last_cmd_` is zero
    /// (velocity_smoother.cpp:329-337).
    fn step(&mut self, now_ns: i64) -> Option<Twist> {
        if !self.received_a_setpoint {
            return None;
        }
        if elapsed_ns(self.setpoint_time_ns, now_ns) > self.velocity_timeout_ns {
            if self.last_velocity == Twist::ZERO {
                return None;
            }
            self.setpoint = Twist::ZERO;
        }

        let current = self.last_velocity;
        let params = &self.params;
        let setpoint = self
            .setpoint
            .map_axes(|axis, value| {
                let limits = params.axis(axis);
                value.clamp(limits.min_velocity, limits.max_velocity)
            })
            .expect("a finite setpoint clamped to a finite range is finite");
        self.setpoint = setpoint;

        let current_axes = current.axes();
        let setpoint_axes = setpoint.axes();
        let eta = (0..3).fold(1.0, |eta: f64, axis| {
            let axis_eta = params.eta_constraint(axis, current_axes[axis], setpoint_axes[axis]);
            if axis_eta > 0.0 && (1.0 - axis_eta).abs() > (1.0 - eta).abs() {
                axis_eta
            } else {
                eta
            }
        });
        let velocity = current
            .map_axes(|axis, value| params.constrained(axis, value, setpoint_axes[axis], eta))
            .expect("a step from a velocity of the range toward a setpoint of it is finite");
        self.last_velocity = velocity;

        Some(
            velocity
                .map_axes(|axis, value| {
                    if value.abs() < params.axis(axis).deadband {
                        0.0
                    } else {
                        value
                    }
                })
                .expect("a finite velocity or 0 is finite"),
        )
    }
}

/// The period of Nav2's smoother's timer at `frequency_hz`, in nanoseconds:
/// `std::chrono::milliseconds(static_cast<int>(1000.0 / smoothing_frequency_))`
/// (velocity_smoother.cpp:169-171), whole milliseconds truncated; at each of
/// `SMOOTHING_FREQUENCIES_HZ` the quotient is whole.
fn timer_period_ns(frequency_hz: f64) -> i64 {
    (MILLISECONDS_PER_SECOND / frequency_hz) as i64 * NANOSECONDS_PER_MILLISECOND
}

/// The first slot of the grid of `period_ns` through `slot_ns` that lies after
/// `now_ns`, a tick at or after `slot_ns`: as `rcl_timer_call` moves its next
/// call (rcl src/rcl/timer.c:274-291), a period past `slot_ns`, and on by
/// whole periods past a tick a period or more late.
fn slot_after(slot_ns: i64, now_ns: i64, period_ns: i64) -> i64 {
    let next_ns = slot_ns.saturating_add(period_ns);
    if next_ns > now_ns {
        return next_ns;
    }
    let periods_ahead = 1 + elapsed_ns(next_ns, now_ns) / period_ns;
    next_ns.saturating_add(periods_ahead.saturating_mul(period_ns))
}

/// `frequencies` as the list an error names: "20, 25, 40".
fn listed(frequencies: &[f64]) -> String {
    let names: Vec<String> = frequencies.iter().map(f64::to_string).collect();
    names.join(", ")
}
