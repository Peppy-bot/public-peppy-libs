//! Orientation constraints against MoveIt: `OrientationTarget::error` measures a link's
//! orientation against a target as the contract states `tolerance_rad` and `free_axis`, the
//! angle of the rotation between them, or the angle between the free axis and its direction
//! in the target. MoveIt bounds the components of a rotation vector about the target's axes.
//! So MoveIt's box inside our region accepts only what ours meets, and its box around our
//! region rejects only what ours breaks: with no free axis, at the tolerance over the square
//! root of 3 and at the tolerance about each axis; with a free axis, at pi about it, and
//! about each other axis at the tolerance over the square root of 3 and at pi times the sine
//! of half the tolerance.

mod common;
mod openarm_v2;

use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{Axis, Frame, OrientationTarget, orientation_from_parts};
use ros_reference::orientation_constraints::{Case, Cases};

use common::LEFT_ARM;
use openarm_v2::{DESCRIPTION, JOINTS, LEFT};

/// How far a case may lie past a bound and still count for either side: the difference of
/// our forward kinematics and MoveIt's, at most some 4e-15 per coordinate, within this.
const ROUNDING_RAD: f64 = 1e-9;

/// The angle our target measures for the grasp link at `case`, in radians.
fn our_error(case: &Case) -> f64 {
    let positions: [f64; JOINTS] = case
        .positions
        .as_slice()
        .try_into()
        .expect("one position per joint of the arm");
    OrientationTarget {
        frame: Frame::GraspPoint,
        orientation: orientation_from_parts(case.orientation).expect("a unit quaternion"),
        tolerance_rad: PositiveFinite::parse(case.tolerance_rad).expect("a tolerance above 0"),
        free_axis: Axis::free_named(&case.free_axis).expect("a free axis's name"),
    }
    .error_at(LEFT_ARM.chain(), &positions)
}

// On every case of the scenario, a state MoveIt accepts within the box inside our region
// meets our target, and one MoveIt rejects within the box around it breaks our target. The
// cases hold every verdict for each value of free_axis, so neither implication is vacuous.
#[test]
fn moveits_boxes_bound_our_orientation_verdicts() {
    let all = Cases::load_all();
    assert!(!all.is_empty(), "the harness holds a scenario of this kind");
    for cases in &all {
        assert_eq!(
            cases.robot.description.urdf(),
            DESCRIPTION.urdf(),
            "{}: our chain poses the scenario's robot",
            cases.name
        );
        assert_eq!(
            cases.arm,
            *openarm_v2::arm(LEFT),
            "{}: our chain poses the scenario's arm",
            cases.name
        );
        for case in &cases.cases {
            let error = our_error(case);
            let at = format!(
                "{}: {} rad from a target of {} free about {}",
                cases.name, error, case.tolerance_rad, case.free_axis
            );
            if case.accepted_within_inner {
                assert!(
                    error <= case.tolerance_rad + ROUNDING_RAD,
                    "{at}: MoveIt accepts it inside our region"
                );
            }
            if !case.accepted_within_outer {
                assert!(
                    error >= case.tolerance_rad - ROUNDING_RAD,
                    "{at}: MoveIt rejects it around our region"
                );
            }
        }
        for free_axis in ["none", "x", "y", "z"] {
            let of_the_axis = || cases.cases.iter().filter(|c| c.free_axis == free_axis);
            assert!(
                of_the_axis().any(|c| c.accepted_within_inner),
                "{}: no case of free_axis {free_axis} that the inner box accepts",
                cases.name
            );
            assert!(
                of_the_axis().any(|c| !c.accepted_within_outer),
                "{}: no case of free_axis {free_axis} that the outer box rejects",
                cases.name
            );
            assert!(
                of_the_axis().any(|c| !c.accepted_within_inner && c.accepted_within_outer),
                "{}: no case of free_axis {free_axis} between the boxes",
                cases.name
            );
        }
    }
}
