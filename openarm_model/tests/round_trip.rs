//! End-to-end IK<->FK round trip through the [`Description`] layer: a handful of
//! in-limit configurations are mapped to an EE pose by FK and recovered by the
//! closed-form IK, checking that the URDF -> FK -> model wiring is correct.
//! The exhaustive seeded-random sweep over the configuration space lives in
//! `ik.rs` (which drives the same solver through `ArmModel::from_fk` directly);
//! this test exists only to prove the `Description`/URDF path reaches it.
//! Parameterized by [`Version`] so a future revision validates the same way
//! (add `round_trip(Version::V2)` when wired).

use openarm_model::description::{ArmSide, Description, Version};
use openarm_model::ik::{self, ArmAnglePolicy};

fn round_trip(version: Version) {
    let desc = Description::new(version, ArmSide::Left);
    let mut fk = desc.forward_kinematics().expect("fk");
    let m = desc.model().expect("model");

    // A small deterministic spread of in-limit, non-singular configurations.
    let samples: [[f64; 7]; 4] = [
        [0.3, 0.4, -0.2, 0.6, 0.1, -0.3, 0.2],
        [-0.5, 0.2, 0.4, 1.0, -0.4, 0.5, -0.1],
        [0.1, -0.3, 0.5, 0.8, 0.3, -0.2, 0.4],
        [-0.2, 0.5, -0.3, 1.2, -0.1, 0.4, -0.5],
    ];

    for q in samples {
        let target = fk.at(&q).ee_pose();
        let r_d = target.rotation.to_rotation_matrix();
        let p_d = target.translation.vector;

        let sol = ik::solve(&m, &r_d, &p_d, ArmAnglePolicy::FromSeed, &q)
            .unwrap_or_else(|| panic!("{version:?}: no IK solution for {q:?}"));
        for (i, (&v, l)) in sol.q.iter().zip(&m.limits).enumerate() {
            assert!(l.contains(v), "{version:?} joint {i} = {v} outside [{}, {}]", l.lo, l.hi);
        }
        let got = fk.at(&sol.q).ee_pose();
        let pos_err = (got.translation.vector - p_d).norm();
        let rot_err = got.rotation.angle_to(&target.rotation);
        assert!(
            pos_err < 1e-6 && rot_err < 1e-6,
            "{version:?}: round-trip pose error pos={pos_err:.2e} rot={rot_err:.2e} for {q:?}"
        );
    }
}

#[test]
fn v1_round_trip() {
    round_trip(Version::V1);
}
