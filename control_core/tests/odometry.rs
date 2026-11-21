//! Properties of the wheel odometry on drawn wheel geometries and wheel
//! motions: equal wheel turns drive straight by the arc they roll, opposite
//! turns spin in place by the turn the wheels' difference makes, a constant
//! pair of rates draws one circle whatever the update rate, an update over less
//! than 1e-6 s changes nothing, and wheel angles to measure from that are not
//! finite are refused. The wheel rates of a velocity measure that velocity, no
//! wheel within the limits turns faster than the fastest wheel rate, and each
//! wheel setpoint anchors by its rule.

mod common;

use std::num::NonZeroUsize;

use common::finite::positive;
use control_core::odometry::{Odometry, WheelGeometry, WheelSetpoint};
use control_core::twist::PlanarTwist;
use hegel::TestCase;
use hegel::generators as gs;

/// The slack of a comparison of two computed poses.
const TOLERANCE: f64 = 1e-9;

/// The geometry of wheels `separation_m` apart, each of `radius_m`.
fn wheels(separation_m: f64, radius_m: f64) -> WheelGeometry {
    WheelGeometry {
        wheel_separation_m: positive(separation_m),
        left_wheel_radius_m: positive(radius_m),
        right_wheel_radius_m: positive(radius_m),
    }
}

/// The odometry of `geometry` over a rolling window of `window` velocities.
fn odometry_of(geometry: WheelGeometry, window: usize) -> Odometry {
    Odometry::new(geometry, NonZeroUsize::new(window).expect("a window"))
}

fn geometry(tc: &TestCase) -> (f64, f64, WheelGeometry) {
    let separation = tc.draw(gs::floats().min_value(0.1).max_value(1.0));
    let radius = tc.draw(gs::floats().min_value(0.02).max_value(0.3));
    (separation, radius, wheels(separation, radius))
}

#[hegel::test]
fn equal_wheel_turns_drive_straight_by_the_arc_they_roll(tc: TestCase) {
    let (_, radius, geometry) = geometry(&tc);
    let mut odometry = odometry_of(geometry, 4);
    let turn = tc.draw(gs::floats().min_value(-20.0).max_value(20.0));
    let steps = tc.draw(gs::integers::<u32>().min_value(1).max_value(50));
    for step in 1..=steps {
        let angle = turn * f64::from(step) / f64::from(steps);
        assert!(odometry.update_from_positions(angle, angle, 0.01));
    }
    assert!(
        (odometry.x() - turn * radius).abs() < TOLERANCE,
        "x {}",
        odometry.x()
    );
    assert!(odometry.y().abs() < TOLERANCE, "y {}", odometry.y());
    assert!(
        odometry.heading().abs() < TOLERANCE,
        "heading {}",
        odometry.heading()
    );
}

#[hegel::test]
fn opposite_wheel_turns_spin_in_place(tc: TestCase) {
    let (separation, radius, geometry) = geometry(&tc);
    let mut odometry = odometry_of(geometry, 4);
    let turn = tc.draw(gs::floats().min_value(-20.0).max_value(20.0));
    let steps = tc.draw(gs::integers::<u32>().min_value(1).max_value(50));
    for step in 1..=steps {
        let angle = turn * f64::from(step) / f64::from(steps);
        assert!(odometry.update_from_positions(-angle, angle, 0.01));
    }
    let heading = 2.0 * turn * radius / separation;
    assert!(
        (odometry.heading() - heading).abs() < TOLERANCE,
        "heading {}",
        odometry.heading()
    );
    assert!(odometry.x().abs() < TOLERANCE && odometry.y().abs() < TOLERANCE);
}

#[hegel::test]
fn constant_rates_draw_one_circle_whatever_the_update_rate(tc: TestCase) {
    let (separation, radius, geometry) = geometry(&tc);
    let left_rate = tc.draw(gs::floats().min_value(0.5).max_value(5.0));
    let right_rate = tc.draw(gs::floats().min_value(5.5).max_value(10.0));
    let duration = tc.draw(gs::floats().min_value(0.5).max_value(3.0));
    let pose_after = |steps: u32| {
        let mut odometry = odometry_of(geometry, 1);
        let dt = duration / f64::from(steps);
        for step in 1..=steps {
            let elapsed = dt * f64::from(step);
            assert!(odometry.update_from_positions(left_rate * elapsed, right_rate * elapsed, dt));
        }
        [odometry.x(), odometry.y(), odometry.heading()]
    };
    let linear = (left_rate + right_rate) * radius * 0.5;
    let angular = (right_rate - left_rate) * radius / separation;
    let heading = angular * duration;
    let exact = [
        linear / angular * heading.sin(),
        linear / angular * (1.0 - heading.cos()),
        heading,
    ];
    for steps in [1, 7, 100] {
        let pose = pose_after(steps);
        for (got, wanted) in pose.iter().zip(exact) {
            assert!(
                (got - wanted).abs() < 1e-6,
                "{steps} steps: {pose:?} against {exact:?}"
            );
        }
    }
}

// An update integrates over a time of 1e-6 s or more in magnitude, as odometry.cpp's
// `update_from_pos` compares `fabs(dt) < 1e-6`, and changes nothing over a shorter one: the
// wheels turned by 1 rad over 1e-6 s carry the base 0.08 m forward, and turned back to 0
// over -1e-6 s, a rate of 1e6 rad/s times -1e-6 s, they carry it back to 0.
#[test]
fn an_update_over_less_than_a_microsecond_changes_nothing() {
    let geometry = wheels(0.36, 0.08);
    let mut odometry = odometry_of(geometry, 4);
    assert!(!odometry.update_from_positions(1.0, 1.0, 9.99e-7));
    assert!(!odometry.update_from_positions(1.0, 1.0, -9.99e-7));
    assert_eq!(
        (odometry.x(), odometry.y(), odometry.heading()),
        (0.0, 0.0, 0.0)
    );
    assert!(odometry.update_from_positions(1.0, 1.0, 1e-6));
    assert!((odometry.x() - 0.08).abs() < TOLERANCE, "{}", odometry.x());
    assert!(odometry.update_from_positions(0.0, 0.0, -1e-6));
    assert!(odometry.x().abs() < TOLERANCE, "{}", odometry.x());
}

// Wheel rates of 1, 3 and 5 rad/s over 0.1 s each, wheels of 0.1 m: velocities of 0.1, 0.3
// and 0.5 m/s.
#[test]
fn the_velocity_is_the_rolling_mean_of_the_window() {
    let geometry = wheels(0.36, 0.1);
    let mut odometry = odometry_of(geometry, 2);
    assert!(odometry.update_from_positions(0.1, 0.1, 0.1));
    assert!(
        (odometry.linear() - 0.1).abs() < TOLERANCE,
        "one value: its own mean"
    );
    assert!(odometry.update_from_positions(0.4, 0.4, 0.1));
    assert!((odometry.linear() - 0.2).abs() < TOLERANCE, "two values");
    assert!(odometry.update_from_positions(0.9, 0.9, 0.1));
    assert!(
        (odometry.linear() - 0.4).abs() < TOLERANCE,
        "the window drops the oldest"
    );
}

#[test]
fn an_update_that_is_not_finite_changes_nothing() {
    let geometry = wheels(0.36, 0.08);
    let mut odometry = odometry_of(geometry, 4);
    assert!(odometry.update_from_positions(1.0, 1.0, 0.1));
    let before = (
        odometry.x(),
        odometry.y(),
        odometry.heading(),
        odometry.linear(),
    );
    assert!(!odometry.update_from_positions(f64::NAN, 1.0, 0.1));
    assert!(!odometry.update_from_positions(1.0, f64::INFINITY, 0.1));
    assert!(!odometry.update_from_positions(1.0, 1.0, f64::NAN));
    assert!(!odometry.update_from_positions(f64::MAX, f64::MAX, 1e-3));
    let after = (
        odometry.x(),
        odometry.y(),
        odometry.heading(),
        odometry.linear(),
    );
    assert_eq!(after, before);
    // The refused angles moved no anchor: the next move is measured from 1.
    assert!(odometry.update_from_positions(2.0, 2.0, 0.1));
    assert!((odometry.x() - 0.16).abs() < TOLERANCE, "{}", odometry.x());
}

#[test]
fn anchored_wheels_measure_the_next_move_from_their_angles_and_keep_the_pose() {
    let geometry = wheels(0.36, 0.08);
    let mut odometry = odometry_of(geometry, 4);
    assert!(odometry.update_from_positions(1.0, 1.0, 0.1));
    let before = (odometry.x(), odometry.y(), odometry.heading());
    odometry.anchor_wheels(10.0, -3.0).expect("finite angles");
    assert_eq!((odometry.x(), odometry.y(), odometry.heading()), before);
    assert!(odometry.update_from_positions(11.0, -2.0, 0.1));
    assert!(
        (odometry.x() - before.0 - 0.08).abs() < TOLERANCE,
        "one more radian of both wheels from the anchor: {}",
        odometry.x()
    );
}

// Wheel angles to measure from that are not finite are refused, naming them, and the next
// update measures from the angles before.
#[test]
fn wheel_angles_to_measure_from_that_are_not_finite_are_refused() {
    let geometry = wheels(0.36, 0.08);
    let mut odometry = odometry_of(geometry, 4);
    assert!(odometry.update_from_positions(1.0, 1.0, 0.1));
    for (left, right) in [
        (f64::NAN, 0.0),
        (0.0, f64::INFINITY),
        (f64::NEG_INFINITY, 1.0),
    ] {
        let refused = odometry.anchor_wheels(left, right).unwrap_err();
        assert_eq!(
            refused.values.map(|(_, angle)| angle.to_bits()),
            [left.to_bits(), right.to_bits()]
        );
        assert!(refused.to_string().contains("must be finite"), "{refused}");
    }
    assert!(odometry.update_from_positions(2.0, 2.0, 0.1));
    assert!((odometry.x() - 0.16).abs() < TOLERANCE, "{}", odometry.x());
}

/// A geometry of a drawn separation and a drawn radius for each wheel.
fn uneven_geometry(tc: &TestCase) -> WheelGeometry {
    let separation = tc.draw(gs::floats().min_value(0.1).max_value(1.0));
    let left = tc.draw(gs::floats().min_value(0.02).max_value(0.3));
    let right = tc.draw(gs::floats().min_value(0.02).max_value(0.3));
    WheelGeometry {
        wheel_separation_m: positive(separation),
        left_wheel_radius_m: positive(left),
        right_wheel_radius_m: positive(right),
    }
}

#[hegel::test]
fn the_wheel_rates_of_a_velocity_measure_that_velocity(tc: TestCase) {
    let geometry = uneven_geometry(&tc);
    let linear = tc.draw(gs::floats().min_value(-2.0).max_value(2.0));
    let angular = tc.draw(gs::floats().min_value(-3.0).max_value(3.0));
    let velocity = PlanarTwist::new(linear, 0.0, angular).expect("finite");
    let [left_rate, right_rate] = geometry.wheel_rates(velocity);
    let mut odometry = odometry_of(geometry, 1);
    assert!(odometry.update_from_positions(left_rate * STEP_S, right_rate * STEP_S, STEP_S));
    assert!(
        (odometry.linear() - linear).abs() < TOLERANCE
            && (odometry.angular() - angular).abs() < TOLERANCE,
        "{linear}, {angular} measured as {}, {}",
        odometry.linear(),
        odometry.angular()
    );
}

#[test]
fn the_mix_turns_the_right_wheel_faster_in_a_left_turn() {
    let geometry = wheels(0.4, 0.1);
    let [left, right] = geometry.wheel_rates(PlanarTwist::new(0.5, 0.0, 1.0).expect("finite"));
    assert!(
        (left - 3.0).abs() < 1e-12 && (right - 7.0).abs() < 1e-12,
        "{left}, {right}"
    );
}

// Within forward speeds up to the linear limit and turn rates up to the
// angular one, no wheel turns faster than the fastest wheel rate, and at a
// corner of those limits one wheel turns at it.
#[hegel::test]
fn no_wheel_within_the_limits_turns_faster_than_the_fastest_wheel_rate(tc: TestCase) {
    let geometry = uneven_geometry(&tc);
    let linear_limit = tc.draw(gs::floats().min_value(0.01).max_value(2.0));
    let angular_limit = tc.draw(gs::floats().min_value(0.01).max_value(3.0));
    let fastest = geometry.fastest_wheel_rate(linear_limit, angular_limit);
    let fastest_of = |linear: f64, angular: f64| {
        let velocity = PlanarTwist::new(linear, 0.0, angular).expect("finite");
        let [left, right] = geometry.wheel_rates(velocity);
        left.abs().max(right.abs())
    };
    let linear = tc.draw(
        gs::floats()
            .min_value(-linear_limit)
            .max_value(linear_limit),
    );
    let angular = tc.draw(
        gs::floats()
            .min_value(-angular_limit)
            .max_value(angular_limit),
    );
    assert!(
        fastest_of(linear, angular) <= fastest * (1.0 + 1e-12),
        "{linear}, {angular} turns a wheel faster than {fastest}"
    );
    let at_corners = [-1.0, 1.0]
        .map(|turn| fastest_of(linear_limit, turn * angular_limit))
        .into_iter()
        .fold(0.0, f64::max);
    assert!(
        (at_corners - fastest).abs() <= fastest * 1e-12,
        "{at_corners} at the corners against {fastest}"
    );
}

/// The step the wheel setpoint cases advance over (s).
const STEP_S: f64 = 0.5;

/// What [`WheelSetpoint::next`] sends both wheels of a base whose wheels
/// were last `sent` (angle, rate), measure `measured` and turn at `rate`
/// next, within `bound`: its angle, after checking that it carries `rate`.
fn sends(sent: Option<(f64, f64)>, measured: f64, rate: f64, bound: f64) -> f64 {
    let sent = sent.map(|(position, rate)| WheelSetpoint {
        positions: [position; 2],
        rates: [rate; 2],
    });
    let next = WheelSetpoint::next(sent, [measured; 2], [rate; 2], bound, STEP_S);
    assert_eq!(next.rates, [rate; 2], "the rate is the feedforward");
    assert_eq!(next.positions[0], next.positions[1]);
    next.positions[0]
}

// A wheel's first setpoint starts from its measured angle.
#[test]
fn a_first_setpoint_anchors_on_the_measured_angle() {
    assert_eq!(sends(None, 1.5, 2.0, 0.5), 1.5 + 2.0 * STEP_S);
    assert_eq!(sends(None, 1.5, 0.0, 0.5), 1.5);
}

// A wheel that keeps or gains speed runs on from its last setpoint while its
// measured angle lies within the bound of it, and from its measured angle
// past the bound, so a wheel held back winds up no more than the bound and
// one step of its rate.
#[test]
fn a_wheel_keeping_speed_anchors_on_its_setpoint_within_the_bound() {
    assert_eq!(sends(Some((2.0, 3.0)), 1.6, 3.0, 0.5), 2.0 + 3.0 * STEP_S);
    assert_eq!(sends(Some((2.0, 3.0)), 1.5, 4.0, 0.5), 2.0 + 4.0 * STEP_S);
    assert_eq!(sends(Some((2.0, 3.0)), 1.4, 3.0, 0.5), 1.4 + 3.0 * STEP_S);
    assert_eq!(
        sends(Some((-2.0, -3.0)), -1.6, -3.0, 0.5),
        -2.0 - 3.0 * STEP_S
    );
    // A wheel that starts from rest anchors on where it stands.
    assert_eq!(sends(Some((2.0, 0.0)), 1.9, 1.0, 0.5), 1.9 + 1.0 * STEP_S);
}

// A wheel that reverses anchors on its measured angle, whatever its speed:
// its last setpoint lies ahead of it the other way.
#[test]
fn a_reversing_wheel_anchors_on_its_measured_angle() {
    assert_eq!(sends(Some((2.0, 3.0)), 1.9, -3.0, 0.5), 1.9 - 3.0 * STEP_S);
    assert_eq!(sends(Some((2.0, 3.0)), 1.9, -4.0, 0.5), 1.9 - 4.0 * STEP_S);
    assert_eq!(
        sends(Some((-2.0, -3.0)), -1.9, 3.0, 0.5),
        -1.9 + 3.0 * STEP_S
    );
}

// A slowing wheel anchors on its measured angle, and a wheel whose rate falls
// to zero holds that angle; at zero after that it holds its last setpoint.
#[test]
fn a_slowing_or_stopping_wheel_anchors_on_its_measured_angle() {
    assert_eq!(sends(Some((2.0, 3.0)), 1.9, 2.0, 0.5), 1.9 + 2.0 * STEP_S);
    assert_eq!(sends(Some((2.0, 3.0)), 1.9, 0.0, 0.5), 1.9);
    assert_eq!(sends(Some((1.9, 0.0)), 2.1, 0.0, 0.5), 1.9);
}

// Each wheel anchors by its own last setpoint, measured angle and rate.
#[test]
fn each_wheel_anchors_on_its_own() {
    let sent = WheelSetpoint {
        positions: [2.0, 5.0],
        rates: [3.0, 3.0],
    };
    let next = WheelSetpoint::next(Some(sent), [1.8, 4.9], [3.0, 2.0], 0.5, STEP_S);
    assert_eq!(next.positions, [2.0 + 3.0 * STEP_S, 4.9 + 2.0 * STEP_S]);
    assert_eq!(next.rates, [3.0, 2.0]);
}
