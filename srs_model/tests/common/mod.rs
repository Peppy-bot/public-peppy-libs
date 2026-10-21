//! Shared fixture loading for the integration tests. Builds an [`Arm`] from the
//! bundled fixture URDF and the `base` link, the same robot-agnostic entry point
//! (`from_urdf`) production uses. `side` is `"left"`/`"right"`; left vs right is
//! just a different chain in the same URDF, selected by the base link.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use properties::{ELBOW_FLEX, STRAIGHT_ELBOW_BAND, TOLERANCE};
use srs_model::{Arm, ArmAnglePolicy, JointVec};

pub mod properties;

const FIXTURE_URDF: &str = include_str!("../fixtures/openarm_v10.urdf");

fn base(side: &str) -> String {
    format!("openarm_{side}_link0")
}

pub fn arm(side: &str) -> Arm {
    Arm::from_urdf(FIXTURE_URDF, &base(side)).expect("load fixture arm")
}

/// Asserts `solution` is inside the arm's limits and reaches the pose FK
/// reports at `q`.
pub fn assert_reaches(arm: &Arm, q: &JointVec, solution: &JointVec) {
    for (joint, (&value, limit)) in solution.iter().zip(&arm.limits()).enumerate() {
        assert!(
            limit.contains(value),
            "joint {joint} = {value} outside [{}, {}]",
            limit.lo,
            limit.hi
        );
    }
    let (target, reached) = (arm.at(q).ee_pose(), arm.at(solution).ee_pose());
    let position_miss = (reached.translation.vector - target.translation.vector).norm();
    let orientation_miss = reached.rotation.angle_to(&target.rotation);
    assert!(
        position_miss < TOLERANCE && orientation_miss < TOLERANCE,
        "{solution:?} misses the pose of {q:?} by {position_miss:.2e} m and \
         {orientation_miss:.2e} rad"
    );
}

/// The solution IK returns, seeded with `seed`, for the pose FK reports at `q`.
/// Asserts it reaches that pose within limits, and that it exists whenever
/// `q`'s elbow is bent past [`STRAIGHT_ELBOW_BAND`].
pub fn round_trip(arm: &Arm, q: &JointVec, seed: &JointVec) -> Option<JointVec> {
    let target = arm.at(q).ee_pose();
    let Some(solution) = arm.solve_ik(&target, ArmAnglePolicy::FromSeed, seed) else {
        assert!(
            q[ELBOW_FLEX] < STRAIGHT_ELBOW_BAND,
            "no IK solution from {seed:?} for the pose of {q:?}"
        );
        return None;
    };
    assert_reaches(arm, q, &solution.q);
    Some(solution.q)
}
