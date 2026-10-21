//! The IK<->FK round trip through the public `Arm` API, on either arm: every
//! in-limit configuration is mapped to an EE pose by FK and handed back to the
//! closed-form IK, seeded with itself. Any solution the IK returns is in limits
//! and reaches the pose, it returns one for every elbow bent past the
//! straight-arm band, and it refuses every pose of the straight arm.

mod common;

use common::properties::{configs, sides, straight_elbow};
use srs_model::ArmAnglePolicy;

/// The explicit cases hold joint 7 on a limit at a posture where it touches
/// that limit at a single arm angle, the only one reaching the pose.
#[hegel::test]
#[hegel::explicit_test_case(side = "left", q = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.5707959999999999])]
#[hegel::explicit_test_case(side = "right", q = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.5707959999999999])]
fn ik_solves_every_bent_pose_fk_reports(tc: hegel::TestCase) {
    let side = tc.draw(sides());
    let arm = common::arm(side);
    let q = tc.draw(configs(arm.limits()));
    common::round_trip(&arm, &q, &q);
}

#[hegel::test]
fn ik_refuses_every_straight_arm_pose(tc: hegel::TestCase) {
    let side = tc.draw(sides());
    let arm = common::arm(side);
    let q = tc.draw(configs(straight_elbow(arm.limits())));
    let target = arm.at(&q).ee_pose();
    assert!(
        arm.solve_ik(&target, ArmAnglePolicy::FromSeed, &q)
            .is_none(),
        "the {side} arm solved the straight-arm pose of {q:?}"
    );
}
