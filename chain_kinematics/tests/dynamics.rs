//! Feedforward dynamics against closed forms: fixtures small enough that the
//! right answer is a textbook expression, not another implementation.

mod common;

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use common::{SplitMix64, openarm};

const PLANAR: &str = r#"<?xml version="1.0"?><robot name="polar">
  <link name="base"/>
  <link name="boom"><inertial><mass value="0.0"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
  <link name="slider"><inertial><origin xyz="0 0 0"/><mass value="0.7"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
  <joint name="swing" type="revolute">
    <parent link="base"/><child link="boom"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="1" velocity="1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="boom"/><child link="slider"/><axis xyz="1 0 0"/>
    <limit lower="0.1" upper="1.5" effort="1" velocity="1"/>
  </joint>
</robot>"#;

/// A point mass sliding along a rotating boom: the polar-coordinates classic.
fn polar() -> Chain<2> {
    let robot = urdf_rs::read_from_string(PLANAR).expect("parse polar fixture");
    Chain::<2>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "slider",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("polar fixture is a two-joint chain")
}

const VERTICAL_SLIDER: &str = r#"<?xml version="1.0"?><robot name="lift">
  <link name="base"/>
  <link name="carriage"><inertial><mass value="1.3"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
  <joint name="lift" type="prismatic">
    <parent link="base"/><child link="carriage"/><axis xyz="0 0 1"/>
    <limit lower="0.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
</robot>"#;

#[test]
fn a_vertical_slider_holds_its_own_weight() {
    let robot = urdf_rs::read_from_string(VERTICAL_SLIDER).expect("parse lift fixture");
    let chain = Chain::<1>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "carriage",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("lift fixture is a one-joint chain");
    let tau = chain.at(&[0.4]).gravity_torques();
    // Holding still against gravity takes exactly the weight, at any height.
    assert!((tau[0] - 1.3 * 9.81).abs() < 1e-12, "tau = {}", tau[0]);
}

#[test]
fn the_polar_slider_matches_the_textbook_coriolis() {
    // Point mass m on a horizontal rotating boom (swing rate w) sliding at
    // rate rdot, mass at radius r. With qddot = 0 the velocity coupling is
    //   swing torque: 2 m r rdot w      (Coriolis)
    //   slide force: -m r w^2           (centripetal)
    let chain = polar();
    let (m, r, w, rdot) = (0.7, 0.9, 1.7, 0.35);
    let tau = chain.at(&[0.3, r]).coriolis_torques(&[w, rdot]);
    let expected_swing = 2.0 * m * r * rdot * w;
    let expected_slide = -m * r * w * w;
    assert!(
        (tau[0] - expected_swing).abs() < 1e-12,
        "swing: got {} expected {expected_swing}",
        tau[0]
    );
    assert!(
        (tau[1] - expected_slide).abs() < 1e-12,
        "slide: got {} expected {expected_slide}",
        tau[1]
    );
    // And the boom's gravity torque is zero: the mass moves in a horizontal
    // plane, while the slide axis is horizontal too.
    let grav = chain.at(&[0.3, r]).gravity_torques();
    assert!(grav[0].abs() < 1e-12 && grav[1].abs() < 1e-12, "{grav:?}");
}

#[test]
fn gravity_is_the_potential_energy_gradient_on_a_real_arm() {
    // The property that defines gravity torque, checked on the 7-DOF fixture
    // by central difference of U = sum(m_i * g * z_com_i).
    let chain = openarm();
    let mut rng = SplitMix64(21);
    for _ in 0..5 {
        let q = rng.config(&chain);
        let tau = chain.at(&q).gravity_torques();
        let h = 1e-6;
        for i in 0..7 {
            let (mut qp, mut qm) = (q, q);
            qp[i] += h;
            qm[i] -= h;
            let u = |q: &[f64; 7]| -> f64 {
                let posed = chain.at(q);
                (0..7)
                    .map(|s| posed.mass(s) * 9.81 * posed.com_world(s).z)
                    .sum()
            };
            let grad = (u(&qp) - u(&qm)) / (2.0 * h);
            assert!(
                (tau[i] - grad).abs() < 1e-3,
                "joint {i}: tau {} vs gradient {grad}",
                tau[i]
            );
        }
    }
}

#[test]
fn zero_velocity_couples_no_torque() {
    let chain = openarm();
    let tau = chain
        .at(&[0.2, -0.4, 0.3, 0.9, -0.2, 0.5, 0.1])
        .coriolis_torques(&[0.0; 7]);
    assert_eq!(tau, [0.0; 7]);
}
