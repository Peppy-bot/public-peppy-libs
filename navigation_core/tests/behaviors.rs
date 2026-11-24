//! Spin's turn rate and DriveOnHeading's speed on Nav2's default settings, and their properties
//! on drawn settings: a spin turns no faster as less of it is left, and a drive stepped on its
//! own commands speeds up by at most one cycle of its acceleration limit and ends within the
//! cycles its minimum speed takes over its length.

mod common;

use common::finite::{non_negative, positive};
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::behaviors::{DriveOnHeadingParams, SpinParams};

/// Spin's defaults (spin.cpp:53-55).
fn nav2_spin() -> SpinParams {
    SpinParams {
        rotational_acc_lim_rad_s2: positive(3.2),
        min_rotational_vel_rad_s: non_negative(0.4),
        max_rotational_vel_rad_s: non_negative(1.0),
    }
}

/// DriveOnHeading's defaults (drive_on_heading.hpp:251-256) at the behavior server's default
/// cycle_frequency (timed_behavior.hpp:76): one cycle of either limit is 0.25 m/s.
fn nav2_drive() -> DriveOnHeadingParams {
    DriveOnHeadingParams {
        acceleration_limit_m_s2: positive(2.5),
        deceleration_limit_m_s2: positive(2.5),
        minimum_speed_m_s: non_negative(0.1),
        cycle_frequency_hz: positive(10.0),
    }
}

fn assert_near(got: Option<f64>, expected: f64) {
    let got = got.expect("a command");
    assert!((got - expected).abs() < 1e-12, "{got} against {expected}");
}

// sqrt(2 x 3.2 x 1.57) = 3.17 rad/s, held to 1 rad/s; 0.1 rad left gives 0.8 rad/s, clockwise for
// a clockwise spin; 0.01 rad left gives 0.25 rad/s, raised to 0.4 rad/s; the spin ends below
// 1e-6 rad left, and past its angle.
#[test]
fn a_spin_slows_into_its_angle_and_ends_within_1e_6_rad_of_it() {
    let spin = nav2_spin();
    assert_eq!(spin.rate(1.57, 1.57), Some(1.0));
    assert_near(spin.rate(-1.57, 0.1), -0.8);
    assert_eq!(spin.rate(1.57, 0.01), Some(0.4));
    assert_eq!(spin.rate(1.57, 1e-6), Some(0.4));
    assert_eq!(spin.rate(1.57, 0.99e-6), None);
    assert_eq!(spin.rate(1.57, -0.01), None);
}

// From rest the drive speeds up by one cycle of the acceleration limit, then holds its speed;
// 4 mm short of its end it slows to sqrt(2 x 2.5 x 0.004) = 0.141 m/s, and 1 mm short to
// 0.071 m/s, raised to the minimum speed; a speed asked below the last falls by one cycle of the
// deceleration limit; a back-up mirrors each; and the drive ends once no distance is left.
#[test]
fn a_drive_ramps_to_its_speed_slows_into_its_end_and_ends_there() {
    let drive = nav2_drive();
    assert_eq!(drive.speed(0.3, 0.0, 1.0), Some(0.25));
    assert_eq!(drive.speed(0.3, 0.25, 1.0), Some(0.3));
    assert_near(drive.speed(0.3, 0.3, 0.004), 0.02_f64.sqrt());
    assert_eq!(drive.speed(0.3, 0.3, 0.001), Some(0.1));
    assert_eq!(drive.speed(0.1, 0.5, 1.0), Some(0.25));
    assert_eq!(drive.speed(-0.3, 0.0, 1.0), Some(-0.25));
    assert_near(drive.speed(-0.3, -0.3, 0.004), -(0.02_f64.sqrt()));
    assert_eq!(drive.speed(-0.3, -0.3, 0.001), Some(-0.1));
    assert_eq!(drive.speed(0.3, 0.3, 0.0), None);
    assert_eq!(drive.speed(-0.3, -0.3, -0.01), None);
}

// Whatever the settings, a spin with less left turns no faster, within its slowest and fastest
// rates, the way it goes.
#[hegel::test]
fn a_spin_turns_no_faster_as_less_of_it_is_left(tc: TestCase) {
    let slowest = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    let spin = SpinParams {
        rotational_acc_lim_rad_s2: positive(tc.draw(gs::floats().min_value(0.1).max_value(10.0))),
        min_rotational_vel_rad_s: non_negative(slowest),
        max_rotational_vel_rad_s: non_negative(
            slowest + tc.draw(gs::floats().min_value(0.0).max_value(3.0)),
        ),
    };
    let cmd_yaw = tc.draw(gs::floats().min_value(-6.0).max_value(6.0));
    let less = tc.draw(gs::floats().min_value(1e-6).max_value(6.0));
    let more = less + tc.draw(gs::floats().min_value(0.0).max_value(6.0));
    let slower = spin.rate(cmd_yaw, less).expect("1e-6 rad or more left");
    let faster = spin.rate(cmd_yaw, more).expect("1e-6 rad or more left");
    assert!(
        slower.abs() <= faster.abs(),
        "{slower} with {less} left, {faster} with {more}"
    );
    for rate in [slower, faster] {
        let magnitude = rate.abs();
        assert!(
            spin.min_rotational_vel_rad_s.get() <= magnitude
                && magnitude <= spin.max_rotational_vel_rad_s.get(),
            "{rate} within {spin:?}"
        );
        assert_eq!(
            rate.is_sign_negative(),
            cmd_yaw.is_sign_negative(),
            "{rate} for {cmd_yaw}"
        );
    }
}

// A drive whose base moves at each command for one cycle, from rest, speeds up by at most one
// cycle of its acceleration limit at each command, unless the minimum speed raises it, goes the
// way of its commanded speed, and ends within the cycles its minimum speed takes over its length.
#[hegel::test]
fn a_drive_stepped_on_its_commands_keeps_its_acceleration_and_ends(tc: TestCase) {
    let drive = DriveOnHeadingParams {
        acceleration_limit_m_s2: positive(tc.draw(gs::floats().min_value(0.1).max_value(5.0))),
        deceleration_limit_m_s2: positive(tc.draw(gs::floats().min_value(0.1).max_value(5.0))),
        minimum_speed_m_s: non_negative(tc.draw(gs::floats().min_value(0.01).max_value(0.2))),
        cycle_frequency_hz: positive(tc.draw(gs::floats().min_value(5.0).max_value(50.0))),
    };
    let length = tc.draw(gs::floats().min_value(0.01).max_value(3.0));
    let command_speed = tc.draw(gs::floats().min_value(0.05).max_value(1.0))
        * if tc.draw(gs::booleans()) { 1.0 } else { -1.0 };
    let cycle_s = 1.0 / drive.cycle_frequency_hz.get();
    let minimum = drive.minimum_speed_m_s.get();
    let most_cycles = (length / (minimum * cycle_s)).ceil() as usize + 1;
    let (mut current, mut driven) = (0.0_f64, 0.0);
    for _ in 0..=most_cycles {
        let Some(speed) = drive.speed(command_speed, current, length - driven) else {
            return;
        };
        let most = current.abs() + drive.acceleration_limit_m_s2.get() * cycle_s;
        assert!(
            speed.abs() <= most * (1.0 + 1e-12) || speed.abs() == minimum,
            "from {current} to {speed} m/s"
        );
        assert_eq!(speed.is_sign_negative(), command_speed.is_sign_negative());
        current = speed;
        driven += speed.abs() * cycle_s;
    }
    panic!("{length} m at {command_speed} m/s is not over after {most_cycles} cycles");
}
