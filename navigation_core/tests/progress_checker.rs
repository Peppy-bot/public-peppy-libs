//! Properties of the progress checker on drawn paths and clocks: a base
//! that stays within the required movement of where it last made progress
//! makes none once the allowance has passed, and a base that moves farther
//! than it between checks always makes progress; a move of exactly the
//! required movement is none, as Nav2 compares it.

mod common;

use std::f64::consts::PI;

use common::finite::non_negative;
use common::poses::planar;
use control_core::planar_pose::PlanarPose;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::progress_checker::{ProgressChecker, ProgressCheckerParams};

/// The share of the required movement within which a drawn position stays,
/// and past which a drawn move goes: the margin keeps the draws off the
/// distance the checker compares at.
const INSIDE: f64 = 0.99;
const OUTSIDE: f64 = 1.01;
/// One second in nanoseconds.
const SECOND_NS: i64 = 1_000_000_000;

/// A checker of a drawn required movement and a whole number of seconds of
/// allowance, which nanoseconds hold exactly.
fn checker(tc: &TestCase) -> (f64, i64, ProgressChecker) {
    let required = tc.draw(gs::floats().min_value(0.05).max_value(1.0));
    let allowance_s = tc.draw(gs::integers::<i64>().min_value(1).max_value(20));
    let params = ProgressCheckerParams {
        required_movement_m: non_negative(required),
        movement_time_allowance_s: non_negative(allowance_s as f64),
    };
    (
        required,
        allowance_s * SECOND_NS,
        ProgressChecker::new(params),
    )
}

/// The position `distance` from `from` in a drawn direction.
fn moved(tc: &TestCase, from: PlanarPose, distance: f64) -> PlanarPose {
    let direction = tc.draw(gs::floats().min_value(-PI).max_value(PI));
    planar(
        from.x() + distance * direction.cos(),
        from.y() + distance * direction.sin(),
        from.yaw(),
    )
}

// A base that stays within the required movement of where it first stood
// makes progress until the allowance has passed since then, and none after.
#[hegel::test]
fn too_little_movement_is_no_progress(tc: TestCase) {
    let (required, allowance_ns, mut checker) = checker(&tc);
    let start = planar(
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        0.0,
    );
    let start_ns = tc.draw(
        gs::integers::<i64>()
            .min_value(0)
            .max_value(100 * SECOND_NS),
    );
    assert_eq!(checker.check(start, start_ns), Ok(()));
    let mut now_ns = start_ns;
    for _ in 0..tc.draw(gs::integers::<usize>().min_value(1).max_value(20)) {
        now_ns += tc.draw(gs::integers::<i64>().min_value(0).max_value(3 * SECOND_NS));
        let distance = tc.draw(gs::floats().min_value(0.0).max_value(INSIDE * required));
        let pose = moved(&tc, start, distance);
        let result = checker.check(pose, now_ns);
        assert_eq!(
            result.is_err(),
            now_ns - start_ns > allowance_ns,
            "{} ns after the start, {distance} m from it: {result:?}",
            now_ns - start_ns
        );
    }
}

// A base that moves farther than the required movement between checks
// makes progress, however long each move takes.
#[hegel::test]
fn moving_the_required_distance_is_progress(tc: TestCase) {
    let (required, _, mut checker) = checker(&tc);
    let mut pose = planar(0.0, 0.0, 0.0);
    let mut now_ns = 0;
    assert_eq!(checker.check(pose, now_ns), Ok(()));
    for _ in 0..tc.draw(gs::integers::<usize>().min_value(1).max_value(20)) {
        now_ns += tc.draw(gs::integers::<i64>().min_value(0).max_value(60 * SECOND_NS));
        let distance = tc.draw(gs::floats().min_value(OUTSIDE * required).max_value(3.0));
        pose = moved(&tc, pose, distance);
        assert_eq!(
            checker.check(pose, now_ns),
            Ok(()),
            "a move of {distance} m"
        );
    }
}

// A reset takes the next check as the new start, so a base that made no
// progress has its allowance again.
#[test]
fn a_reset_starts_the_allowance_again() {
    let params = ProgressCheckerParams {
        required_movement_m: non_negative(0.5),
        movement_time_allowance_s: non_negative(10.0),
    };
    let mut checker = ProgressChecker::new(params);
    let still = planar(1.0, 2.0, 0.0);
    assert_eq!(checker.check(still, 0), Ok(()));
    assert!(checker.check(still, 11 * SECOND_NS).is_err());
    checker.reset();
    assert_eq!(checker.check(still, 12 * SECOND_NS), Ok(()));
    assert_eq!(checker.check(still, 22 * SECOND_NS), Ok(()));
    assert!(checker.check(still, 22 * SECOND_NS + 1).is_err());
}

// A move of exactly the required movement keeps the baseline, as Nav2 takes a
// new one only past it: once the allowance has passed since the first check,
// the base has made no progress, a move of no more than the required movement.
// The distance is exact in double precision.
#[test]
fn a_move_of_exactly_the_required_movement_is_no_progress() {
    let params = ProgressCheckerParams {
        required_movement_m: non_negative(0.5),
        movement_time_allowance_s: non_negative(10.0),
    };
    let mut checker = ProgressChecker::new(params);
    let start = planar(0.0, 0.0, 0.0);
    let moved = planar(0.5, start.y(), start.yaw());
    assert_eq!(checker.check(start, 0), Ok(()));
    assert_eq!(checker.check(moved, SECOND_NS), Ok(()));
    let refused = checker.check(moved, 10 * SECOND_NS + 1).unwrap_err();
    assert!(
        refused
            .to_string()
            .contains("moved 0.5 m in 10.000000001 s, no more than the required movement of 0.5 m"),
        "{refused}"
    );
}
