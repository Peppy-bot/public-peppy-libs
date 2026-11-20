//! The limiter against Nav2's velocity smoother (nav2_velocity_smoother 1.5.2): `Limiter`
//! ports `VelocitySmoother::smootherTimer` in open loop with the axes scaled together,
//! `findEtaConstraint` and `applyConstraints` in their operation order, and the timeout in
//! integer nanoseconds, so for the same setpoints, limits and clock each cycle gives the
//! velocity Nav2's publishes, bit for bit, and nothing where Nav2's publishes nothing, an
//! axis of range 0 to 0 with 0 limits among them. The scenarios tick at each slot of the
//! timer's grid, where every tick steps. The evidence: every velocity smoother scenario of
//! the ROS reference harness, whose committed outputs hold the bits of Nav2's amd64 build.
#![cfg(target_arch = "x86_64")]

mod common;

use common::finite::{non_negative, positive};
use control_core::limiter::{AxisLimits, AxisLimitsParams, Limiter, LimiterParams};
use control_core::twist::PlanarTwist;
use ros_reference::velocity_smoother::{Cases, Event, SmootherParameters};

/// The port of a scenario's parameters: each axis's range, its acceleration, its
/// deceleration as a magnitude, Nav2's `max_decel` negated, and its deadband, the axes
/// at Nav2's indexes 0, 1 and 2, x, y and theta.
fn params(smoother: &SmootherParameters) -> LimiterParams {
    let axis = |i: usize| {
        AxisLimits::parse(AxisLimitsParams {
            min_velocity: smoother.min_velocity[i],
            max_velocity: smoother.max_velocity[i],
            max_acceleration: non_negative(smoother.max_accel[i]),
            max_deceleration: non_negative(-smoother.max_decel[i]),
            deadband: non_negative(smoother.deadband_velocity[i]),
        })
        .expect("the scenario's limits")
    };
    LimiterParams {
        linear_x: axis(0),
        linear_y: axis(1),
        angular_z: axis(2),
        smoothing_frequency_hz: positive(smoother.smoothing_frequency),
        velocity_timeout_s: non_negative(smoother.velocity_timeout),
    }
}

fn bits(velocity: Option<[f64; 3]>) -> Option<[u64; 3]> {
    velocity.map(|axes| axes.map(f64::to_bits))
}

// Each cycle publishes Nav2's velocity, bit for bit, or nothing where Nav2's publishes
// nothing.
#[test]
fn each_limited_velocity_is_nav2s() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a velocity smoother scenario"
    );
    for cases in &scenarios {
        let smoother = &cases.smoother;
        assert!(
            smoother.feedback == "OPEN_LOOP"
                && smoother.scale_velocities
                && smoother.velocity_timeout == 0.5,
            "{}: the scenario runs the smoother as base_state:v1's limiter limits a command: \
             open loop, the axes scaled together, a velocity timeout of 0.5 s",
            cases.name
        );
        let mut limiter = Limiter::parse(params(smoother))
            .expect("a frequency whose period is a whole number of milliseconds");
        let mut cycles = cases.cycles.iter();
        for event in &cases.events {
            match *event {
                Event::Setpoint { at_ns, twist } => {
                    let [x, y, z] = twist;
                    limiter.take_setpoint(PlanarTwist::new(x, y, z).expect("finite"), at_ns);
                }
                Event::Tick { at_ns } => {
                    let expected = cycles.next().expect("a cycle a tick");
                    let ours = limiter.tick(at_ns).map(PlanarTwist::axes);
                    assert_eq!(
                        bits(ours),
                        bits(expected.published),
                        "{} at {at_ns} ns: ours {ours:?}, Nav2's {:?}",
                        cases.name,
                        expected.published
                    );
                }
            }
        }
    }
}
