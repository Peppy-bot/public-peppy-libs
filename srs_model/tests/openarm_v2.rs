//! Verify srs_model builds a valid 7-DOF SRS chain from the vendored OpenArm v2.0 URDF,
//! whose revolute pinch-gripper fingers branch off the wrist tip (`ee_base_link`). This
//! guards the load-bearing assumption that the chain walk stops at the 7th revolute joint
//! and does not trip on the finger branch, and that the reoriented v2 frames still satisfy
//! the SRS concurrency checks so gravity/Coriolis feedforward evaluates.

use srs_model::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use srs_model::{ARM_DOF, Arm, ArmAnglePolicy};

const V2_URDF: &str = "../openarm_description/assets/openarm_v20.urdf";
const V1_URDF: &str = "../openarm_description/assets/openarm_v10.urdf";

#[test]
fn builds_v2_srs_chain_for_both_arms() {
    for base in ["openarm_left_base_link", "openarm_right_base_link"] {
        let arm = Arm::from_urdf_file(V2_URDF, base).unwrap_or_else(|e| panic!("v2 {base}: {e}"));

        let limits = arm.limits();
        assert_eq!(limits.len(), ARM_DOF);
        assert_eq!(
            limits[3].lo, 0.0,
            "{base}: elbow (j4) mechanical lower is 0.0"
        );

        // The feedforward path the real arm runs each control tick: gravity + Coriolis
        // from the posed chain must evaluate to finite torques for the v2 model.
        let posed = arm.at(&[0.0; ARM_DOF]);
        let gravity = posed.gravity_torques();
        let coriolis = posed.coriolis_torques(&[0.1; ARM_DOF]);
        assert!(
            gravity.iter().chain(coriolis.iter()).all(|t| t.is_finite()),
            "{base}: gravity/Coriolis torques must be finite"
        );
    }
}

#[test]
fn v1_still_builds() {
    let arm = Arm::from_urdf_file(V1_URDF, "openarm_left_link0").expect("v1 builds");
    assert_eq!(arm.limits().len(), ARM_DOF);
}

#[test]
fn a_pose_the_arm_takes_falls_short_by_nothing_and_a_far_one_by_its_distance_past_the_reach() {
    let arm = Arm::from_urdf_file(V2_URDF, "openarm_left_base_link")
        .and_then(|arm| arm.with_tool_link("openarm_left_tcp"))
        .expect("v2 left arm with its tool");
    for q in [
        [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
        [0.15, 0.4, -0.48, 0.95, 0.0, 0.0, 0.0],
        [-0.3, 0.6, 0.2, 1.6, 0.4, -0.3, 0.5],
    ] {
        let pose = arm.at(&q).ee_pose();
        assert_eq!(arm.reach_shortfall(&pose), 0.0, "{q:?}");
        assert!(
            arm.solve_ik(&pose, ArmAnglePolicy::FromSeed, &q).is_some(),
            "{q:?}: a pose the arm takes is one it reaches"
        );
    }
    // Far along one direction, a metre further is a metre shorter, within the
    // shoulder's offset from the base origin seen from that far.
    let direction = Vector3::new(0.6, -0.3, 0.74).normalize();
    let at = |distance: f64| {
        Isometry3::from_parts(
            Translation3::from(direction * distance),
            UnitQuaternion::identity(),
        )
    };
    let (near, far) = (
        arm.reach_shortfall(&at(20.0)),
        arm.reach_shortfall(&at(21.0)),
    );
    assert!(near > 18.0, "{near}");
    assert!((far - near - 1.0).abs() < 0.01, "{near} then {far}");
    assert!(
        arm.solve_ik(&at(20.0), ArmAnglePolicy::FromSeed, &[0.0; ARM_DOF])
            .is_none()
    );
}
