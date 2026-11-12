//! The two real robots the integration suites drive: the 5-DOF SO-101 and the
//! 7-DOF arms of the OpenArm v1, as robot_description bundles their models,
//! loaded through the public API exactly as a consumer would.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use std::sync::LazyLock;

use chain_kinematics::nalgebra::DMatrix;
use chain_kinematics::{Chain, ChainSpec, Jacobian, JointSelection};
use robot_description::RobotDescription;

pub mod properties;

pub static SO101: LazyLock<RobotDescription> = LazyLock::new(|| bundled("so101"));
pub static OPENARM: LazyLock<RobotDescription> = LazyLock::new(|| bundled("openarm_v1"));

fn bundled(model: &str) -> RobotDescription {
    robot_description::model(model)
        .unwrap_or_else(|unknown| panic!("{unknown}"))
        .description()
}

pub const SO101_JOINTS: [&str; 5] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
];

pub fn so101() -> Chain<5> {
    Chain::<5>::from_urdf(
        SO101.robot(),
        &ChainSpec {
            base_link: None,
            tip_link: "gripper_frame_link",
            joints: JointSelection::Named(&SO101_JOINTS),
        },
    )
    .expect("SO-101 is a five-joint chain")
}

pub fn openarm() -> Chain<7> {
    Chain::<7>::from_urdf(
        OPENARM.robot(),
        &ChainSpec {
            base_link: Some("openarm_left_link0"),
            tip_link: "openarm_left_link7",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("OpenArm is a seven-joint chain")
}

/// The singular values of `j`, from the eigenvalues of whichever Gram matrix is
/// the smaller: `Jᵀ J` below six joints, one value per joint, and `J Jᵀ` from
/// six up, six values. nalgebra 0.35's SVD misreports some rank-deficient
/// Jacobians, and its symmetric eigen-decomposition reads them right once the
/// Gram matrix is divided by its largest entry: its rotations underflow to
/// infinities on entries near 1e-170, which a drawn chain of coaxial joints
/// gives.
pub fn singular_values<const N: usize>(j: &Jacobian<N>) -> Vec<f64> {
    let j = DMatrix::from_iterator(6, N, j.iter().copied());
    let gram = if N < 6 {
        j.transpose() * &j
    } else {
        &j * j.transpose()
    };
    let scale = gram.amax();
    if scale == 0.0 {
        return vec![0.0; gram.nrows()];
    }
    (gram / scale)
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .map(|&value| (value.max(0.0) * scale).sqrt())
        .collect()
}
