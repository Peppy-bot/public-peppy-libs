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

#[test]
fn a_point_falls_short_by_the_least_shortfall_of_a_pose_there_in_any_orientation() {
    let arm = Arm::from_urdf_file(V2_URDF, "openarm_left_base_link")
        .and_then(|arm| arm.with_tool_link("openarm_left_tcp"))
        .expect("v2 left arm with its tool");
    // Directions spread evenly over the sphere (a Fibonacci lattice): each
    // turns the tool so the wrist center stands that way from the target.
    let wrist_offset = arm.tool().inverse().translation.vector;
    let directions = (0..4000).map(|i| {
        let z = 1.0 - (2.0 * f64::from(i) + 1.0) / 4000.0;
        let azimuth = f64::from(i) * std::f64::consts::PI * (3.0 - 5f64.sqrt());
        let ring = (1.0 - z * z).sqrt();
        Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z)
    });
    let orientations: Vec<UnitQuaternion<f64>> = directions
        .map(|direction| {
            UnitQuaternion::rotation_between(&wrist_offset, &direction).unwrap_or_else(|| {
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
            })
        })
        .collect();
    let least = |target: Vector3<f64>| {
        orientations
            .iter()
            .map(|&rotation| {
                arm.reach_shortfall(&Isometry3::from_parts(Translation3::from(target), rotation))
            })
            .fold(f64::INFINITY, f64::min)
    };
    for target in [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.3, -0.1, 0.2),
        Vector3::new(0.9, 0.2, -0.3),
        Vector3::new(-0.4, 1.1, 0.6),
        Vector3::new(3.0, -2.0, 1.0),
    ] {
        let shortfall = arm.position_shortfall(&target);
        let sampled = least(target);
        assert!(
            shortfall <= sampled + 1e-9 && sampled - shortfall < 1e-3,
            "{target:?}: {shortfall} against the least sampled {sampled}"
        );
    }
    assert_eq!(arm.position_shortfall(&Vector3::new(0.3, -0.1, 0.2)), 0.0);
    assert!(arm.position_shortfall(&Vector3::new(3.0, -2.0, 1.0)) > 2.0);
}

#[test]
fn a_pose_just_beyond_the_reach_is_solved_on_the_reach_within_the_tolerance() {
    // The elbow held 0.05 rad off the straight arm, as the OpenArm backbone
    // holds it: the arm cannot reach the shell's outer edge itself.
    let arm = Arm::from_urdf_file(V2_URDF, "openarm_left_base_link")
        .map(|arm| arm.with_lower_floor(3, 0.05))
        .and_then(|arm| arm.with_tool_link("openarm_left_tcp"))
        .expect("v2 left arm with its tool");
    // A pose of the nearly straight arm, pushed out along the arm's reach
    // until its wrist center stands `beyond` outside the shell.
    let seed = [0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0];
    let taken = arm.at(&seed).ee_pose();
    let outward = taken.translation.vector.normalize();
    let pushed = |by: f64| Translation3::from(outward * by) * taken;
    let beyond = |shortfall: f64| {
        let (mut low, mut high) = (0.0, 0.5);
        for _ in 0..60 {
            let middle = (low + high) / 2.0;
            if arm.reach_shortfall(&pushed(middle)) < shortfall {
                low = middle;
            } else {
                high = middle;
            }
        }
        pushed(high)
    };
    let tolerance = 0.01;
    let near = beyond(0.005);
    assert!((arm.reach_shortfall(&near) - 0.005).abs() < 1e-9);
    assert!(
        arm.solve_ik(&near, ArmAnglePolicy::FromSeed, &seed)
            .is_none()
    );
    let solution = arm
        .solve_ik_within(&near, tolerance, ArmAnglePolicy::FromSeed, &seed)
        .expect("5 mm beyond the reach is within 1 cm");
    let reached = arm.at(&solution.q).ee_pose();
    let missed = (reached.translation.vector - near.translation.vector).norm();
    assert!(missed < tolerance, "{missed}");
    assert!(reached.rotation.angle_to(&near.rotation) < 1e-6);
    // Just inside the shell, past the elbow's floor: solved within the
    // tolerance too.
    let inside = Translation3::from(outward * -0.0001) * beyond(1e-9);
    assert_eq!(arm.reach_shortfall(&inside), 0.0);
    assert!(
        arm.solve_ik(&inside, ArmAnglePolicy::FromSeed, &seed)
            .is_none()
    );
    assert!(
        arm.solve_ik_within(&inside, tolerance, ArmAnglePolicy::FromSeed, &seed)
            .is_some()
    );
    let far = beyond(0.02);
    assert!(
        arm.solve_ik_within(&far, tolerance, ArmAnglePolicy::FromSeed, &seed)
            .is_none()
    );
    // A pose the arm takes is solved as it is.
    assert_eq!(
        arm.solve_ik_within(&taken, tolerance, ArmAnglePolicy::FromSeed, &seed)
            .map(|solution| solution.q),
        arm.solve_ik(&taken, ArmAnglePolicy::FromSeed, &seed)
            .map(|solution| solution.q)
    );
}
