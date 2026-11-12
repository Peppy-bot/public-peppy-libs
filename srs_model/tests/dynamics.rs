//! Feedforward dynamics on the fixture arm, validated two independent ways: KDL's
//! recursive Newton-Euler inverse dynamics (`ChainIdSolver_RNE`, ROS's KDL), over the
//! arm's chain from its base link to link7 with every link below link7 folded into it,
//! the gripper fingers at home, so the folded distal payload is included; and the
//! potential-energy gradient, which is what gravity torque is. The computation itself
//! lives in `chain_kinematics`; this suite is the evidence it holds on a 7-DOF arm,
//! mirror sides included.
//!
//! Tolerance 1e-3 Nm; sub-threshold reference values written 0.0. `tools/kdl_reference.py`
//! prints the KDL tables, as the README says.

mod common;

use common::arm;
use srs_model::{ARM_DOF, JointVec};
use std::f64::consts::FRAC_PI_2;

const GRAVITY_MAGNITUDE: f64 = 9.81;

const GRAVITY_POSTURES: [JointVec; 4] = [
    [0.0; ARM_DOF],
    [FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, FRAC_PI_2, 0.0, 0.0, 0.0],
    [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
];

fn assert_gravity_matches_kdl(side: &str, expected: [JointVec; 4]) {
    let arm = arm(side);
    for (q, exp) in GRAVITY_POSTURES.iter().zip(&expected) {
        let tau = arm.at(q).gravity_torques();
        for i in 0..ARM_DOF {
            assert!(
                (tau[i] - exp[i]).abs() < 1e-3,
                "{side} arm, q={q:?}, joint {i}: actual={} expected={}",
                tau[i],
                exp[i],
            );
        }
    }
}

#[test]
fn gravity_left_matches_kdl() {
    assert_gravity_matches_kdl(
        "left",
        [
            [0.0983, -0.0504, 0.0, -0.0299, 0.0, 0.0584, -0.0049],
            [11.0882, 0.0, -0.0504, -4.1891, -0.0637, 0.0, 0.5417],
            [-4.1207, -0.0504, 0.0, 4.1891, 0.0637, 0.0, -0.5417],
            [2.9372, -2.6350, 0.3153, -2.4532, 0.0573, -0.1985, 0.4403],
        ],
    );
}

#[test]
fn gravity_right_matches_kdl() {
    assert_gravity_matches_kdl(
        "right",
        [
            [-0.0983, 0.0791, 0.0, -0.0299, 0.0, -0.0605, 0.0049],
            [11.0882, 0.0, -0.0791, 4.1891, -0.0658, 0.0, 0.5417],
            [4.1207, 0.0791, 0.0, 4.1891, -0.0658, 0.0, 0.5417],
            [-0.4976, -2.1101, 0.2972, -1.2396, 0.1085, -0.2493, 0.1030],
        ],
    );
}

#[test]
fn gravity_matches_potential_energy_gradient_both_sides() {
    for side in ["left", "right"] {
        let arm = arm(side);
        for q in [
            [0.0; ARM_DOF],
            [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
            [-0.5, 0.3, -0.2, 0.8, 0.1, 0.4, -0.3],
        ] {
            let potential = |q: &JointVec| -> f64 {
                let posed = arm.at(q);
                (0..ARM_DOF)
                    .map(|i| posed.mass(i) * GRAVITY_MAGNITUDE * posed.com_world(i).z)
                    .sum()
            };
            let tau = arm.at(&q).gravity_torques();
            let h = 1e-6;
            for i in 0..ARM_DOF {
                let (mut qp, mut qm) = (q, q);
                qp[i] += h;
                qm[i] -= h;
                let grad = (potential(&qp) - potential(&qm)) / (2.0 * h);
                assert!(
                    (tau[i] - grad).abs() < 1e-3,
                    "{side} j{i}: tau={} grad={grad}",
                    tau[i],
                );
            }
        }
    }
}

const CORIOLIS_CASES: [(JointVec, JointVec); 4] = [
    ([0.0; ARM_DOF], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ([0.0; ARM_DOF], [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]),
    (
        [0.0, 0.0, 0.0, FRAC_PI_2, 0.0, 0.0, 0.0],
        [3.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
    ),
    (
        [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
        [1.0, -1.5, 2.0, -2.5, 3.0, -3.5, 4.0],
    ),
];

fn assert_coriolis_matches_kdl(side: &str, expected: [JointVec; 4]) {
    let arm = arm(side);
    for ((q, qdot), exp) in CORIOLIS_CASES.iter().zip(&expected) {
        let tau = arm.at(q).coriolis_torques(qdot);
        for i in 0..ARM_DOF {
            assert!(
                (tau[i] - exp[i]).abs() < 1e-3,
                "{side} arm, q={q:?} qd={qdot:?}, joint {i}: actual={} expected={}",
                tau[i],
                exp[i],
            );
        }
    }
}

#[test]
fn coriolis_zero_velocity_gives_zero_torque() {
    let tau = arm("left")
        .at(&[0.0; ARM_DOF])
        .coriolis_torques(&[0.0; ARM_DOF]);
    assert_eq!(tau, [0.0; ARM_DOF]);
}

#[test]
fn coriolis_left_matches_kdl() {
    assert_coriolis_matches_kdl(
        "left",
        [
            [0.0, -0.0699, 0.0, -0.0168, 0.0, 0.0725, -0.0054],
            [-0.0168, -0.0416, 0.0, 0.0, 0.0, 0.0398, -0.0027],
            [-0.8455, -0.0102, 0.0, 0.8455, 0.0129, 0.0, -0.1093],
            [-3.0458, -1.1971, 0.6039, 0.7945, 0.1688, 0.0769, 0.2765],
        ],
    );
}

#[test]
fn coriolis_right_matches_kdl() {
    assert_coriolis_matches_kdl(
        "right",
        [
            [0.0, 0.0882, 0.0, -0.0168, 0.0, -0.0755, 0.0054],
            [0.0168, 0.0439, 0.0, 0.0, 0.0, -0.0416, 0.0027],
            [-2.5365, 0.0159, 0.0632, 0.8455, -0.0132, -0.0599, 0.1132],
            [0.0978, -1.0228, 0.1573, -0.2619, -0.0304, -0.0176, 0.0157],
        ],
    );
}
