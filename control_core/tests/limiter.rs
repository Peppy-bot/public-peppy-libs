//! Properties of the limiter on drawn limits, axes the base does not move
//! along among them, and setpoint streams: the limits of an axis give back the
//! range, deadband, acceleration and deceleration they were parsed from, every
//! velocity lies within its axis's range, or past an end by at most the
//! rounding of a step, and moves by no more than its acceleration or
//! deceleration step per step, the axes keep the setpoint's direction as they
//! scale together, a setpoint stream that stops comes to rest by 0.5 s after
//! its last setpoint plus its deceleration time in whole steps and then gives
//! nothing, an axis of range 0 to 0 stays at rest, and the deadband zeroes a
//! smaller output and keeps one equal to it. The limiter takes only a
//! frequency whose period is a whole number of milliseconds, and its steps
//! fall on the grid of the timer Nav2 makes for its smoother: one step in each
//! period of the grid at the first tick within it, each by at most a limit
//! times the period, the grid kept after a late tick, and started again a
//! period after a tick of a clock that went back.

mod common;

use common::finite::{non_negative, positive};
use control_core::limiter::{AxisLimits, AxisLimitsParams, Limiter, LimiterError, LimiterParams};
use control_core::twist::PlanarTwist;
use hegel::TestCase;
use hegel::generators as gs;

/// The slack of a comparison of two computed velocities.
const TOLERANCE: f64 = 1e-12;
/// The frequency of the limiters these suites draw, in Hz.
const FREQUENCY_HZ: f64 = 20.0;
/// One step at 20 Hz, in nanoseconds: the period of Nav2's timer at 20 Hz, a
/// whole 50 ms.
const CYCLE_NS: i64 = 50_000_000;
/// One millisecond, in nanoseconds.
const MILLISECOND_NS: i64 = 1_000_000;

/// The parameters of an axis of the range `min_velocity` to `max_velocity`,
/// with `max_acceleration`, `max_deceleration` and `deadband`.
fn axis_params(
    [min_velocity, max_velocity]: [f64; 2],
    [max_acceleration, max_deceleration]: [f64; 2],
    deadband: f64,
) -> AxisLimitsParams {
    AxisLimitsParams {
        min_velocity,
        max_velocity,
        max_acceleration: non_negative(max_acceleration),
        max_deceleration: non_negative(max_deceleration),
        deadband: non_negative(deadband),
    }
}

/// The limits of an axis of the range `min_velocity` to `max_velocity`, with
/// `max_acceleration`, `max_deceleration` and `deadband`.
fn axis_limits(
    range: [f64; 2],
    rates: [f64; 2],
    deadband: f64,
) -> Result<AxisLimits, LimiterError> {
    AxisLimits::parse(axis_params(range, rates, deadband))
}

/// An axis the base does not move along, one time in four, or one it moves along: a
/// range whose ends may differ in magnitude, acceleration and deceleration limits above
/// 0, and a deadband below half its fastest velocity.
fn axis(tc: &TestCase) -> AxisLimitsParams {
    if tc.draw(gs::integers::<u8>().max_value(3)) == 0 {
        return axis_params([0.0, 0.0], [0.0, 0.0], 0.0);
    }
    let max: f64 = tc.draw(gs::floats().min_value(0.1).max_value(3.0));
    let min = -tc.draw(gs::floats().min_value(0.0).max_value(3.0));
    let acceleration = tc.draw(gs::floats().min_value(0.1).max_value(5.0));
    let deceleration = tc.draw(gs::floats().min_value(0.1).max_value(5.0));
    let deadband = tc.draw(gs::floats().min_value(0.0).max_value(max.max(-min) / 2.0));
    axis_params([min, max], [acceleration, deceleration], deadband)
}

/// The limiter of `axes`, in the order of `PlanarTwist::axes`, at `frequency_hz`, a setpoint
/// holding for 0.5 s.
fn limiter_at(axes: [AxisLimits; 3], frequency_hz: f64) -> Limiter {
    let [linear_x, linear_y, angular_z] = axes;
    Limiter::parse(LimiterParams {
        linear_x,
        linear_y,
        angular_z,
        smoothing_frequency_hz: positive(frequency_hz),
        velocity_timeout_s: non_negative(0.5),
    })
    .expect("a frequency whose period is a whole number of milliseconds")
}

/// The limiter of `axes` at 20 Hz, a setpoint holding for 0.5 s.
fn limiter(axes: [AxisLimits; 3]) -> Limiter {
    limiter_at(axes, FREQUENCY_HZ)
}

fn limits(tc: &TestCase) -> ([AxisLimitsParams; 3], Limiter) {
    let params = [axis(tc), axis(tc), axis(tc)];
    let axes = params.map(|params| AxisLimits::parse(params).expect("valid limits"));
    (params, limiter(axes))
}

fn setpoint(tc: &TestCase) -> PlanarTwist {
    let value = || tc.draw(gs::floats().min_value(-4.0).max_value(4.0));
    PlanarTwist::new(value(), value(), value()).expect("finite")
}

/// The largest change of an axis in one step at 20 Hz.
fn largest_step(params: &AxisLimitsParams) -> f64 {
    params
        .max_acceleration
        .get()
        .max(params.max_deceleration.get())
        / FREQUENCY_HZ
}

/// How far a step can land past `end`, an end of its axis's range: the
/// rounding of the velocity plus its step, `current + clamp(eta * change)` in
/// double precision, which Nav2's step gives the same bits of
/// (velocity_smoother.cpp:309). A step that reaches `end` moves by at most the
/// axis's largest step, so it rounds by at most `f64::EPSILON` times the
/// magnitudes of `end` and of that step.
fn one_rounding_past(end: f64, params: &AxisLimitsParams) -> f64 {
    f64::EPSILON * (end.abs() + largest_step(params))
}

#[hegel::test]
fn a_limited_velocity_leaves_its_range_by_no_more_than_one_rounding_of_a_step(tc: TestCase) {
    let (params, mut limiter) = limits(&tc);
    let steps = tc.draw(gs::integers::<i64>().min_value(1).max_value(60));
    for step in 0..steps {
        let now = step * CYCLE_NS;
        if tc.draw(gs::booleans()) {
            limiter.take_setpoint(setpoint(&tc), now);
        }
        let Some(velocity) = limiter.tick(now) else {
            continue;
        };
        for (value, params) in velocity.axes().iter().zip(&params) {
            let (min, max) = (params.min_velocity, params.max_velocity);
            assert!(
                *value >= min - one_rounding_past(min, params)
                    && *value <= max + one_rounding_past(max, params),
                "{value} outside [{min}, {max}] by more than a rounding"
            );
        }
    }
}

// x of range -1 to 0.3, accelerating at 1 and decelerating at 8 m/s^2: from -0.1 m/s, the
// setpoint 1 clamped to 0.3 lies a change of 0.4 away, the whole deceleration step at
// 20 Hz, so the step lands on -0.1 + 0.4 in double precision, one rounding above 0.3, as
// Nav2's `current + clamp(eta * change)` gives it.
#[test]
fn a_step_to_the_end_of_the_range_lands_where_nav2s_arithmetic_puts_it() {
    let x = axis_limits([-1.0, 0.3], [1.0, 8.0], 0.0).expect("valid limits");
    let mut limiter = limiter([x, AxisLimits::STILL, AxisLimits::STILL]);
    limiter.take_setpoint(PlanarTwist::new(-0.1, 0.0, 0.0).expect("finite"), 0);
    limiter.tick(0);
    limiter.tick(CYCLE_NS);
    assert_eq!(limiter.last_velocity().linear_x(), -0.1);
    limiter.take_setpoint(
        PlanarTwist::new(1.0, 0.0, 0.0).expect("finite"),
        2 * CYCLE_NS,
    );
    let reached = limiter.tick(2 * CYCLE_NS).expect("moving").linear_x();
    assert_eq!(reached.to_bits(), (-0.1_f64 + 0.4).to_bits(), "{reached}");
    assert!(reached > 0.3, "{reached}");
}

#[hegel::test]
fn a_limited_velocity_ramps_within_the_acceleration_and_deceleration_limits(tc: TestCase) {
    let (params, mut limiter) = limits(&tc);
    let steps = tc.draw(gs::integers::<i64>().min_value(1).max_value(60));
    let mut previous = PlanarTwist::ZERO;
    for step in 0..steps {
        let now = step * CYCLE_NS;
        if tc.draw(gs::booleans()) {
            limiter.take_setpoint(setpoint(&tc), now);
        }
        if limiter.tick(now).is_none() {
            continue;
        }
        let velocity = limiter.last_velocity();
        for ((now, before), params) in velocity.axes().iter().zip(previous.axes()).zip(&params) {
            // A step away from 0 that keeps the axis's sign speeds it up; every other
            // step slows it or turns it back through 0. Nav2 tells the two apart by the
            // sign of the velocity times the setpoint, a product that rounds to 0 when
            // the velocity is subnormal, so a step from a subnormal velocity through 0
            // takes either limit.
            let (acceleration, deceleration) =
                (params.max_acceleration.get(), params.max_deceleration.get());
            let crosses_zero = (before < 0.0 && *now > 0.0) || (before > 0.0 && *now < 0.0);
            let limit = if crosses_zero && before.is_subnormal() {
                acceleration.max(deceleration)
            } else if !crosses_zero && now.abs() > before.abs() {
                acceleration
            } else {
                deceleration
            };
            let step = limit / FREQUENCY_HZ;
            assert!(
                (now - before).abs() <= step + TOLERANCE,
                "a step of {} from {before} to {now} past the limit {step}",
                (now - before).abs()
            );
        }
        previous = velocity;
    }
}

/// How near a whole number of steps an axis's deceleration time may lie for the rounding
/// of its steps to decide whether the last step falls on it or one later.
const WHOLE_STEP_SLACK: f64 = 1e-9;

// A setpoint at 0 s, then none: at 0.5 s it still holds, and from the next step the
// setpoint is zero. Each axis decelerates from its velocity at 0.5 s at its deceleration
// limit: the velocity before the deadband reaches zero by 0.5 s plus the largest
// velocity / max_deceleration over the axes, rounded up to whole steps of 0.05 s, a step
// later where it lies within WHOLE_STEP_SLACK of a whole number, and the stream gives
// nothing after.
#[hegel::test]
fn a_stream_that_stops_rests_by_its_timeout_and_deceleration_time(tc: TestCase) {
    let (params, mut limiter) = limits(&tc);
    limiter.take_setpoint(setpoint(&tc), 0);
    let held = 10;
    for step in 0..=held {
        limiter.tick(step * CYCLE_NS);
    }
    let at_timeout = limiter.last_velocity();
    let steps_to_rest = params
        .iter()
        .zip(at_timeout.axes())
        .filter(|(params, _)| params.max_deceleration.get() > 0.0)
        .map(|(params, current)| {
            let deceleration = params.max_deceleration.get();
            ((current.abs() * FREQUENCY_HZ / deceleration) + WHOLE_STEP_SLACK).ceil() as i64
        })
        .max()
        .unwrap_or(0);
    let rest = held + steps_to_rest;
    let rested = (held + 1..=rest).any(|step| {
        limiter.tick(step * CYCLE_NS);
        limiter.last_velocity() == PlanarTwist::ZERO
    });
    assert!(
        rested || rest == held,
        "still moving {} steps after 0.5 s, {at_timeout:?} at the timeout",
        steps_to_rest
    );
    assert_eq!(
        limiter.tick((rest + 2) * CYCLE_NS),
        None,
        "at rest, nothing more"
    );
}

// An axis of range 0 to 0, with 0 acceleration, deceleration and deadband as Nav2's
// smoother takes it, stays at 0 whatever its setpoint, and the axes beside it move.
#[test]
fn a_still_axis_takes_zero_limits_and_stays_at_rest() {
    let moving = axis_limits([-0.4, 0.6], [1.0, 2.0], 0.0).expect("valid limits");
    let still = AxisLimits::STILL;
    assert_eq!(still.velocity_range(), (0.0, 0.0));
    let mut limiter = limiter([moving, still, moving]);
    limiter.take_setpoint(PlanarTwist::new(0.6, 0.5, -0.3).expect("finite"), 0);
    for step in 0..10 {
        let velocity = limiter.tick(step * CYCLE_NS).expect("moving");
        assert_eq!(
            velocity.linear_y().to_bits(),
            0.0_f64.to_bits(),
            "{velocity:?}"
        );
        assert!(
            velocity.linear_x() > 0.0 && velocity.angular_z() < 0.0,
            "{velocity:?}"
        );
    }
}

#[hegel::test]
fn scaled_axes_keep_the_direction_of_the_change(tc: TestCase) {
    let (params, mut limiter) = limits(&tc);
    let target = setpoint(&tc);
    limiter.take_setpoint(target, 0);
    limiter.tick(0).expect("a setpoint was taken");
    let velocity = limiter.last_velocity();
    // From rest, every axis's velocity before the deadband moves by one share of
    // its change to the clamped setpoint: the share of the axis its step bound
    // limits most.
    let shares: Vec<f64> = velocity
        .axes()
        .iter()
        .zip(target.axes())
        .zip(&params)
        .map(|((got, wanted), params)| {
            (got, wanted.clamp(params.min_velocity, params.max_velocity))
        })
        .filter(|(_, clamped)| clamped.abs() > 1e-6)
        .map(|(got, clamped)| got / clamped)
        .collect();
    for share in &shares {
        assert!(
            (share - shares[0]).abs() < 1e-9 && *share > 0.0 && *share <= 1.0 + TOLERANCE,
            "shares {shares:?}"
        );
    }
}

#[test]
fn half_a_second_after_the_last_setpoint_the_velocity_ramps_to_rest_and_then_gives_nothing() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 2.0], 0.0).expect("valid limits");
    let mut limiter = limiter([axis; 3]);
    limiter.take_setpoint(PlanarTwist::new(0.5, 0.0, 0.0).expect("finite"), 0);
    // Accelerate at 1 m/s^2: 0.05 m/s per step, at 0.5 m/s after ten steps.
    for step in 0..=10 {
        limiter.tick(step * CYCLE_NS);
    }
    assert!((limiter.last_velocity().linear_x() - 0.5).abs() < TOLERANCE);
    // At 0.5 s the setpoint still holds; past it, the velocity falls at
    // 2 m/s^2, 0.1 m/s per step, to rest in five steps.
    let holding = limiter.tick(10 * CYCLE_NS).expect("in time");
    assert!((holding.linear_x() - 0.5).abs() < TOLERANCE);
    let falling: Vec<f64> = (11..=15)
        .map(|step| {
            limiter
                .tick(step * CYCLE_NS)
                .expect("still moving")
                .linear_x()
        })
        .collect();
    for (got, wanted) in falling.iter().zip([0.4, 0.3, 0.2, 0.1, 0.0]) {
        assert!((got - wanted).abs() < 1e-9, "{falling:?}");
    }
    // Once a step gives zero, every later step gives nothing until the next
    // setpoint. The last step down can leave a remainder of rounding, which
    // one more step takes to zero.
    let (rest, _) = (16..20)
        .map(|step| (step, limiter.tick(step * CYCLE_NS)))
        .find(|(_, velocity)| velocity.is_none_or(|velocity| velocity == PlanarTwist::ZERO))
        .expect("the velocity comes to rest");
    for later in rest + 1..rest + 5 {
        assert_eq!(
            limiter.tick(later * CYCLE_NS),
            None,
            "at rest, nothing more"
        );
    }
}

#[test]
fn nothing_comes_before_the_first_setpoint() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.0).expect("valid limits");
    assert_eq!(limiter([axis; 3]).tick(0), None);
}

#[test]
fn the_deadband_zeroes_a_small_output_and_the_next_step_starts_from_before_it() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.08).expect("valid limits");
    let mut limiter = limiter([axis; 3]);
    limiter.take_setpoint(PlanarTwist::new(1.0, 0.0, 0.0).expect("finite"), 0);
    let first = limiter.tick(0).expect("a setpoint was taken");
    assert_eq!(first.linear_x(), 0.0, "0.05 lies inside the deadband");
    let second = limiter.tick(CYCLE_NS).expect("moving");
    assert!((second.linear_x() - 0.1).abs() < TOLERANCE, "{second:?}");
}

// The deadband zeroes an output smaller in magnitude than it, as Nav2 compares them
// (velocity_smoother.cpp:474): the first step from rest at 1 m/s^2 and 20 Hz is 0.05 m/s,
// equal to the deadband, and comes out as it is.
#[test]
fn the_deadband_keeps_an_output_equal_to_it() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.05).expect("valid limits");
    let mut limiter = limiter([axis; 3]);
    limiter.take_setpoint(PlanarTwist::new(1.0, 0.0, 0.0).expect("finite"), 0);
    let first = limiter.tick(0).expect("a setpoint was taken");
    assert_eq!(first.linear_x(), 0.05, "{first:?}");
}

#[test]
fn limits_that_cannot_bound_a_velocity_are_refused_naming_the_limit() {
    let refused = |result: Result<AxisLimits, LimiterError>| result.unwrap_err().to_string();
    for (min, max) in [(1.0, -1.0), (0.2, 0.5), (-0.5, -0.2), (f64::NAN, 1.0)] {
        let error = refused(axis_limits([min, max], [1.0, 1.0], 0.0));
        assert!(error.contains("must hold 0"), "{min}, {max}: {error}");
    }
    // Ends that hold 0 and are finite, or not, whose width is not finite.
    for (min, max) in [(-1.0, f64::INFINITY), (-f64::MAX, f64::MAX)] {
        let error = refused(axis_limits([min, max], [1.0, 1.0], 0.0));
        assert!(
            error.contains("has no finite width")
                && error.contains("give a finite min_velocity and max_velocity"),
            "{min}, {max}: {error}"
        );
    }
    for (limits, name) in [
        ([0.0, 1.0], "max_acceleration"),
        ([1.0, 0.0], "max_deceleration"),
    ] {
        let error = refused(axis_limits([-1.0, 1.0], limits, 0.0));
        assert!(
            error.starts_with(name) && error.contains(&format!("give {name} above 0")),
            "{error}"
        );
    }
    let error = refused(axis_limits([-0.5, 0.8], [1.0, 1.0], 0.8));
    assert!(error.contains("below the larger magnitude"), "{error}");
    for (limits, deadband) in [([1.0, 0.0], 0.0), ([0.0, 0.5], 0.0), ([0.0, 0.0], 0.1)] {
        let error = refused(axis_limits([0.0, 0.0], limits, deadband));
        assert!(
            error.starts_with("an axis whose range is 0 to 0 has 0 max_acceleration"),
            "{error}"
        );
    }
}

/// The frequencies the limiter takes, in Hz, as its refusal names them.
const FREQUENCIES_NAMED: &str = "20, 25, 40, 50, 100, 125, 200, 250, 500, 1000 Hz";

/// The refusal of a limiter at `frequency_hz`, or `None` where it takes it.
fn frequency_refusal(frequency_hz: f64) -> Option<String> {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.0).expect("valid limits");
    Limiter::parse(LimiterParams {
        linear_x: axis,
        linear_y: axis,
        angular_z: axis,
        smoothing_frequency_hz: positive(frequency_hz),
        velocity_timeout_s: non_negative(0.5),
    })
    .err()
    .map(|error| error.to_string())
}

// The limiter takes a whole number of hertz from 20 to 1000, a step of at most 0.05 s as
// base_state:v1 bounds a step of its limiter, exactly when its period, 1000 / the
// frequency, is a whole number of milliseconds, so a step's bound, a limit / the frequency,
// is the limit times the period: every whole frequency from 20 to 1000 Hz is taken or
// refused by that rule, and a refusal names the frequencies to give.
#[test]
fn the_limiter_takes_exactly_the_frequencies_of_a_period_of_whole_milliseconds() {
    for frequency in 20..=1000_u32 {
        let refusal = frequency_refusal(f64::from(frequency));
        assert_eq!(
            refusal.is_none(),
            1000 % frequency == 0,
            "{frequency} Hz: {refusal:?}"
        );
    }
    for refused in [
        10.0,
        19.9,
        30.0,
        1000.0 / 3.0,
        40.0_f64.next_up(),
        1000.5,
        2000.0,
    ] {
        let error = frequency_refusal(refused).expect("refused");
        assert!(
            error.starts_with(&format!("smoothing_frequency_hz {refused} "))
                && error.contains(&format!("give one of {FREQUENCIES_NAMED}")),
            "{refused}: {error}"
        );
    }
}

// An axis of range 0 to 0 and no limit is the still axis.
#[test]
fn an_axis_of_no_range_and_no_limit_is_the_still_axis() {
    assert_eq!(
        axis_limits([0.0, 0.0], [0.0, 0.0], 0.0).expect("an axis that does not move"),
        AxisLimits::STILL
    );
}

// The limits of a drawn axis, the still axis among them, give back the range, the
// deadband, the acceleration and the deceleration they were parsed from.
#[hegel::test]
fn the_limits_of_an_axis_give_back_their_range_deadband_acceleration_and_deceleration(
    tc: TestCase,
) {
    let params = axis(&tc);
    let limits = AxisLimits::parse(params).expect("a drawn axis is valid");
    assert_eq!(
        limits.velocity_range(),
        (params.min_velocity, params.max_velocity)
    );
    assert_eq!(limits.deadband(), params.deadband.get());
    assert_eq!(limits.max_acceleration(), params.max_acceleration.get());
    assert_eq!(limits.max_deceleration(), params.max_deceleration.get());
}

// An axis that speeds up at 1.0 per second gives 1.0, and the still axis gives 0.
#[test]
fn the_acceleration_of_an_axis_is_its_max_acceleration() {
    let axis = axis_limits([-0.4, 0.6], [1.0, 2.5], 0.0).expect("valid limits");
    assert_eq!(axis.max_acceleration(), 1.0);
    assert_eq!(AxisLimits::STILL.max_acceleration(), 0.0);
}

// An axis that slows at 2.5 per second gives 2.5, and the still axis gives 0.
#[test]
fn the_deceleration_of_an_axis_is_its_max_deceleration() {
    let axis = axis_limits([-0.4, 0.6], [1.0, 2.5], 0.0).expect("valid limits");
    assert_eq!(axis.max_deceleration(), 2.5);
    assert_eq!(AxisLimits::STILL.max_deceleration(), 0.0);
}

// A limiter steps once a period of whole milliseconds, 1 / its frequency.
#[test]
fn a_limiter_steps_once_a_period_of_its_frequency() {
    for (frequency_hz, period_ns) in [(20.0, 50_000_000), (50.0, 20_000_000), (1000.0, 1_000_000)] {
        assert_eq!(
            ramping_at(frequency_hz).period_ns(),
            period_ns,
            "{frequency_hz} Hz"
        );
    }
}

/// The limiter of an axis along x of range -100 to 100 m/s, accelerating at 0.1 m/s^2,
/// and two still axes, at `frequency_hz`: from rest toward a setpoint of 100 m/s, every
/// step of the first hundreds raises the velocity, so a step shows in the velocity.
fn ramping_at(frequency_hz: f64) -> Limiter {
    let x = axis_limits([-100.0, 100.0], [0.1, 0.1], 0.0).expect("valid limits");
    limiter_at([x, AxisLimits::STILL, AxisLimits::STILL], frequency_hz)
}

/// The setpoint `ramping_at`'s limiter ramps toward.
fn far_ahead() -> PlanarTwist {
    PlanarTwist::new(100.0, 0.0, 0.0).expect("finite")
}

/// Takes the far setpoint at `now_ns` and ticks there, and gives whether the tick stepped.
fn stepped_at(limiter: &mut Limiter, now_ns: i64) -> bool {
    let before = limiter.last_velocity();
    limiter.take_setpoint(far_ahead(), now_ns);
    limiter.tick(now_ns);
    limiter.last_velocity() != before
}

// At 125 Hz Nav2's timer has a period of 8 ms, 1000 / 125: the first tick steps, a tick
// 1 ns before 8 ms does not, the ticks at 8 and 16 ms do, and each of the three steps
// moves the velocity by the acceleration / 125, the acceleration times the period.
#[test]
fn at_125_hz_the_steps_fall_every_8_ms_each_by_the_acceleration_times_the_period() {
    let period_s = 0.008;
    let mut limiter = ramping_at(125.0);
    let mut velocities = vec![0.0];
    for (at_ns, steps) in [
        (0, true),
        (8 * MILLISECOND_NS - 1, false),
        (8 * MILLISECOND_NS, true),
        (16 * MILLISECOND_NS - 1, false),
        (16 * MILLISECOND_NS, true),
    ] {
        assert_eq!(stepped_at(&mut limiter, at_ns), steps, "at {at_ns} ns");
        if steps {
            velocities.push(limiter.last_velocity().linear_x());
        }
    }
    for pair in velocities.windows(2) {
        assert!(
            (pair[1] - pair[0] - 0.1 * period_s).abs() < TOLERANCE,
            "{velocities:?}"
        );
    }
}

// A tick 120 ms after the first at 20 Hz, late past the slots at 50 and 100 ms, steps once,
// and the grid stays: the next step falls at 150 ms, as rcl's timer moves its next call by
// whole periods (rcl src/rcl/timer.c:274-291).
#[test]
fn a_late_tick_steps_once_and_the_grid_stays() {
    let mut limiter = ramping_at(FREQUENCY_HZ);
    assert!(stepped_at(&mut limiter, 0));
    let first = limiter.last_velocity().linear_x();
    assert!(stepped_at(&mut limiter, 120 * MILLISECOND_NS));
    let second = limiter.last_velocity().linear_x();
    assert!(
        (second - 2.0 * first).abs() < TOLERANCE,
        "one step: {second}"
    );
    assert!(!stepped_at(&mut limiter, 150 * MILLISECOND_NS - 1));
    assert!(stepped_at(&mut limiter, 150 * MILLISECOND_NS));
}

// A tick on the slot a period after a missed one, at 100 ms after a step at 0 at 20 Hz,
// steps once, and the next slot lies a period after it, at 150 ms, as rcl's
// `next_call_time <= now` moves it past the tick (rcl src/rcl/timer.c:279-289).
#[test]
fn a_tick_on_a_slot_a_period_late_steps_and_the_next_slot_follows_it() {
    let mut limiter = ramping_at(FREQUENCY_HZ);
    assert!(stepped_at(&mut limiter, 0));
    assert!(stepped_at(&mut limiter, 2 * CYCLE_NS));
    assert!(!stepped_at(&mut limiter, 2 * CYCLE_NS + MILLISECOND_NS));
    assert!(stepped_at(&mut limiter, 3 * CYCLE_NS));
}

// A second tick at the instant of a late step, 120 ms at 20 Hz, comes from no clock that
// went back: it gives the last output, and the grid stays, the next step at 150 ms, as rcl
// starts its grid again only for a time before its last call (rcl src/rcl/timer.c:93).
#[test]
fn a_second_tick_at_the_instant_of_a_late_step_keeps_the_grid() {
    let mut limiter = ramping_at(FREQUENCY_HZ);
    assert!(stepped_at(&mut limiter, 0));
    assert!(stepped_at(&mut limiter, 120 * MILLISECOND_NS));
    assert!(!stepped_at(&mut limiter, 120 * MILLISECOND_NS));
    assert!(stepped_at(&mut limiter, 150 * MILLISECOND_NS));
}

// A setpoint of 0.5 m/s from 100 s, ticked every 50 ms to 100.45 s, reaches 0.5 m/s. The
// clock then goes back to 0: the first tick after it, at 50 ms, does not step, and starts
// the grid again a period later, as rcl's timer does after a jump back past its last call
// (rcl src/rcl/timer.c:93-98); at 100 ms the limiter steps toward the new setpoint of 0, by
// the deceleration of 1 m/s^2, a step of 0.05 m/s.
#[test]
fn a_clock_that_goes_back_starts_the_grid_again_a_period_later() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.0).expect("valid limits");
    let mut limiter = limiter([axis; 3]);
    let start_ns = 100_000_000_000;
    limiter.take_setpoint(PlanarTwist::new(0.5, 0.0, 0.0).expect("finite"), start_ns);
    for step in 0..10 {
        limiter.tick(start_ns + step * CYCLE_NS);
    }
    let forward = |velocity: Option<PlanarTwist>| velocity.expect("a setpoint holds").linear_x();
    assert!((limiter.last_velocity().linear_x() - 0.5).abs() < TOLERANCE);
    limiter.take_setpoint(PlanarTwist::ZERO, 0);
    let at_the_jump = forward(limiter.tick(CYCLE_NS));
    assert!((at_the_jump - 0.5).abs() < TOLERANCE, "{at_the_jump}");
    let within = forward(limiter.tick(2 * CYCLE_NS - 1));
    assert!((within - 0.5).abs() < TOLERANCE, "{within}");
    let after = forward(limiter.tick(2 * CYCLE_NS));
    assert!((after - 0.45).abs() < TOLERANCE, "{after}");
}

// A setpoint of 0.5 m/s from 100 s, and no setpoint after it, on a clock that goes back
// 59.5 s: a limiter that ticked every 50 ms to 100.45 s, and one whose first tick comes
// after the step, each give the setpoint the time of their first tick after the step,
// 40.5 s, hold it for the 0.5 s timeout to 41 s, ramp to rest from 41.05 s at 1 m/s^2,
// and then give nothing.
#[test]
fn a_setpoint_from_before_a_clock_that_goes_back_times_out_after_the_step() {
    let axis = axis_limits([-1.0, 1.0], [1.0, 1.0], 0.0).expect("valid limits");
    let setpoint_ns = 100_000_000_000;
    let back_ns = 40_500_000_000;
    let taking_the_setpoint = || {
        let mut limiter = limiter([axis; 3]);
        limiter.take_setpoint(
            PlanarTwist::new(0.5, 0.0, 0.0).expect("finite"),
            setpoint_ns,
        );
        limiter
    };
    let mut ticked_before = taking_the_setpoint();
    for step in 0..10 {
        ticked_before.tick(setpoint_ns + step * CYCLE_NS);
    }
    for (what, mut limiter) in [
        ("ticked before the step", ticked_before),
        ("first ticked after it", taking_the_setpoint()),
    ] {
        let outputs: Vec<Option<PlanarTwist>> = (0..30)
            .map(|step| limiter.tick(back_ns + step * CYCLE_NS))
            .collect();
        let forward_at = |step: usize| outputs[step].map(PlanarTwist::linear_x);
        let holding = forward_at(10).expect("the setpoint holds to 41 s");
        assert!((holding - 0.5).abs() < TOLERANCE, "{what}: {holding}");
        let ramping = forward_at(11).expect("a ramp to rest from 41.05 s");
        assert!((ramping - 0.45).abs() < TOLERANCE, "{what}: {ramping}");
        assert_eq!(outputs[29], None, "{what}: at rest and silent at 41.95 s");
    }
}

/// The frequencies the grid property draws, each with the period of Nav2's timer at it.
const FREQUENCIES_AND_PERIODS: [(f64, i64); 2] = [(20.0, CYCLE_NS), (125.0, 8 * MILLISECOND_NS)];

// A caller that ticks after drawn gaps, shorter and longer than the timer's period, from
// a drawn start: a tick steps exactly when it is the first tick in its period of the grid
// that starts at the first tick, so each period of the grid holds at most one step, and a
// tick a period or more late keeps the grid.
#[hegel::test]
fn a_tick_steps_when_it_is_the_first_in_its_period_of_the_grid(tc: TestCase) {
    let index = tc.draw(gs::integers::<usize>().max_value(FREQUENCIES_AND_PERIODS.len() - 1));
    let (frequency_hz, period_ns) = FREQUENCIES_AND_PERIODS[index];
    let start_ns = tc.draw(
        gs::integers::<i64>()
            .min_value(0)
            .max_value(1_000_000_000_000),
    );
    let ticks = tc.draw(gs::integers::<usize>().min_value(1).max_value(100));
    let mut limiter = ramping_at(frequency_hz);
    let period_of = |at_ns: i64| (at_ns - start_ns) / period_ns;
    let mut previous_ns: Option<i64> = None;
    for tick in 0..ticks {
        let at_ns = previous_ns.map_or(start_ns, |previous_ns| {
            previous_ns
                + tc.draw(
                    gs::integers::<i64>()
                        .min_value(MILLISECOND_NS)
                        .max_value(period_ns * 5 / 2),
                )
        });
        let first_in_its_period =
            previous_ns.is_none_or(|previous_ns| period_of(at_ns) != period_of(previous_ns));
        assert_eq!(
            stepped_at(&mut limiter, at_ns),
            first_in_its_period,
            "tick {tick} at {at_ns} ns from {start_ns} ns at {frequency_hz} Hz"
        );
        previous_ns = Some(at_ns);
    }
}
