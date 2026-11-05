//! The straight line between two poses; and the joint limits every
//! configuration a chain commands is kept in.

use std::f64::consts::PI;

use chain_kinematics::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use chain_kinematics::{
    COINCIDENT_ARC_EPS, Limit, all_within, clamp_into, interpolate, limit_margin,
};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, points, poses, unit_vectors};

// The line between any two poses puts, at every blend `s`, the position at the
// fraction `s` of the way and the orientation at the fraction `s` of one
// geodesic turn, or, for two orientations slerp cannot separate, at the goal's
// from the first blend on.
#[hegel::test]
fn a_line_blends_the_position_and_the_orientation_along_the_geodesic(tc: TestCase) {
    let start = tc.draw(poses());
    let axis = Unit::new_normalize(tc.draw(unit_vectors()));
    let turn: f64 = tc.draw(gs::floats().min_value(0.0).max_value(PI));
    let end = Isometry3::from_parts(
        Translation3::from(tc.draw(points())),
        UnitQuaternion::from_axis_angle(&axis, turn) * start.rotation,
    );
    let turn = start.rotation.angle_to(&end.rotation);
    let separable = start
        .rotation
        .try_slerp(&end.rotation, 0.5, COINCIDENT_ARC_EPS)
        .is_some();
    for k in 0..=20 {
        let s = f64::from(k) / 20.0;
        let got = interpolate(&start, &end, s);
        let along = start.translation.vector.lerp(&end.translation.vector, s);
        assert!(
            (got.translation.vector - along).norm() <= TOLERANCE,
            "at blend {s}"
        );
        let on_the_turn = if separable {
            (got.rotation.angle_to(&start.rotation) - s * turn).abs() <= TOLERANCE
                && (got.rotation.angle_to(&end.rotation) - (1.0 - s) * turn).abs() <= TOLERANCE
        } else {
            got.rotation == end.rotation
        };
        assert!(
            on_the_turn,
            "at blend {s} the orientation is off the geodesic of a {turn} rad turn"
        );
    }
}

/// Seven joint limits and a configuration, each anywhere within 10 rad of
/// zero, so the configuration lands inside, on and outside its limits.
fn limits_and_configuration(tc: &TestCase) -> ([Limit; 7], [f64; 7]) {
    let span = || gs::floats().min_value(-10.0).max_value(10.0);
    let q: [f64; 7] = tc.draw(gs::arrays(span()));
    let ends: [f64; 14] = tc.draw(gs::arrays(span()));
    let limits = std::array::from_fn(|i| Limit {
        lo: ends[2 * i].min(ends[2 * i + 1]),
        hi: ends[2 * i].max(ends[2 * i + 1]),
    });
    (limits, q)
}

// A clamped configuration lies inside every joint's limits, a joint already
// inside them is left where it was, and a configuration is within its limits
// exactly when clamping leaves it as it was.
#[hegel::test]
fn a_clamped_configuration_lies_inside_the_limits(tc: TestCase) {
    let (limits, q) = limits_and_configuration(&tc);
    let clamped = clamp_into(&limits, &q);
    assert!(all_within(&limits, &clamped));
    for i in 0..7 {
        if limits[i].contains(q[i]) {
            assert_eq!(clamped[i], q[i], "joint {i} inside its limits moved");
        }
    }
    assert_eq!(all_within(&limits, &q), clamped == q);
}

// A configuration's limit margin is non-negative exactly when it lies inside
// its limits, and for one outside them it is minus the distance clamping
// moves the joint it moves furthest.
#[hegel::test]
fn the_limit_margin_is_signed_by_whether_the_limits_hold(tc: TestCase) {
    let (limits, q) = limits_and_configuration(&tc);
    let margin = limit_margin(&limits, &q);
    assert_eq!(margin >= 0.0, all_within(&limits, &q));
    if margin < 0.0 {
        let clamped = clamp_into(&limits, &q);
        let furthest = (0..7)
            .map(|i| (clamped[i] - q[i]).abs())
            .fold(0.0, f64::max);
        assert_eq!(-margin, furthest);
    }
}
