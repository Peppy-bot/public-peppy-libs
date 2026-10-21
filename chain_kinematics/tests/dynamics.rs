//! Feedforward dynamics against closed forms: fixtures small enough that the
//! right answer is a textbook expression, not another implementation.

mod common;

use chain_kinematics::nalgebra::{Point3, Vector3};
use chain_kinematics::{Chain, ChainSpec, JointKind, JointSelection};
use common::properties::{ChainProperty, chain_of, named_chain_of, on_a_drawn_robot, permutations};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs};
use urdf_rs::Robot;

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

/// Gravitational acceleration, m/s².
const GRAVITY: f64 = 9.81;

/// Step of the central differences the properties read derivatives with.
const STEP: f64 = 1e-5;

/// Joint rates, each within 1 rad/s or 1 m/s.
fn rates<const N: usize>(tc: &TestCase) -> [f64; N] {
    tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)))
}

#[hegel::test]
fn a_vertical_slider_holds_its_own_weight(tc: TestCase) {
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
    let tau = chain
        .at(&tc.draw(configs(chain.limits())))
        .gravity_torques();
    // Holding still against gravity takes exactly the weight, at any height.
    assert!(
        (tau[0] - 1.3 * GRAVITY).abs() < TOLERANCE,
        "tau = {}",
        tau[0]
    );
}

#[hegel::test]
fn the_polar_slider_matches_the_textbook_coriolis(tc: TestCase) {
    // Point mass m on a horizontal rotating boom (swing rate w) sliding at
    // rate rdot, mass at radius r. With qddot = 0 the velocity coupling is
    //   swing torque: 2 m r rdot w      (Coriolis)
    //   slide force: -m r w^2           (centripetal)
    let chain = polar();
    let m = 0.7;
    let [swing, r] = tc.draw(configs(chain.limits()));
    let [w, rdot] = rates::<2>(&tc);
    let tau = chain.at(&[swing, r]).coriolis_torques(&[w, rdot]);
    let expected_swing = 2.0 * m * r * rdot * w;
    let expected_slide = -m * r * w * w;
    assert!(
        (tau[0] - expected_swing).abs() < TOLERANCE,
        "swing: got {} expected {expected_swing}",
        tau[0]
    );
    assert!(
        (tau[1] - expected_slide).abs() < TOLERANCE,
        "slide: got {} expected {expected_slide}",
        tau[1]
    );
    // And the boom's gravity torque is zero: the mass moves in a horizontal
    // plane, while the slide axis is horizontal too.
    let grav = chain.at(&[swing, r]).gravity_torques();
    assert!(
        grav[0].abs() < TOLERANCE && grav[1].abs() < TOLERANCE,
        "{grav:?}"
    );
}

#[hegel::test]
fn gravity_is_the_potential_energy_gradient(tc: TestCase) {
    // The property that defines gravity torque: each joint's is the derivative
    // along it of U = sum(m_i * g * z_com_i), read here by central difference.
    struct PotentialGradient;
    impl ChainProperty for PotentialGradient {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let potential = |q: &[f64; N]| -> f64 {
                let posed = chain.at(q);
                (0..N)
                    .map(|segment| posed.mass(segment) * GRAVITY * posed.com_world(segment).z)
                    .sum()
            };
            let q = tc.draw(configs(chain.limits()));
            let tau = chain.at(&q).gravity_torques();
            for joint in 0..N {
                let (mut ahead, mut behind) = (q, q);
                ahead[joint] += STEP;
                behind[joint] -= STEP;
                let gradient = (potential(&ahead) - potential(&behind)) / (2.0 * STEP);
                assert!(
                    (tau[joint] - gradient).abs() < TOLERANCE,
                    "joint {joint}: tau {} vs gradient {gradient}",
                    tau[joint]
                );
            }
        }
    }
    on_a_drawn_robot::<PotentialGradient>(&tc);
}

#[hegel::test]
fn the_coriolis_torques_are_lagrange_equations_of_the_kinetic_energy(tc: TestCase) {
    // With no joint acceleration and no gravity, each joint's torque is
    // d/dt(∂T/∂q̇) - ∂T/∂q of the kinetic energy T = ½ Σ (m v² + ωᵀ I ω),
    // built here from each segment's mass, centre of mass and inertia. T is
    // quadratic in q̇, so a central difference of one unit in q̇ reads ∂T/∂q̇
    // exactly; the derivatives in q are central differences along the motion
    // and along each joint.
    struct Lagrange;
    impl ChainProperty for Lagrange {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let qdot = rates::<N>(tc);
            let kinetic = |q: &[f64; N], qdot: &[f64; N]| -> f64 {
                let posed = chain.at(q);
                (0..N)
                    .map(|segment| {
                        let com = Point3::from(posed.com_world(segment));
                        let velocity: Vector3<f64> = posed
                            .point_world_jacobian(&com, segment)
                            .iter()
                            .zip(qdot)
                            .map(|(column, rate)| column * *rate)
                            .sum();
                        let spin: Vector3<f64> = (0..=segment)
                            .filter(|&joint| {
                                matches!(posed.kind(joint), JointKind::Revolute { .. })
                            })
                            .map(|joint| posed.axis_world(joint) * qdot[joint])
                            .sum();
                        0.5 * (posed.mass(segment) * velocity.norm_squared()
                            + spin.dot(&(posed.inertia_world(segment) * spin)))
                    })
                    .sum()
            };
            let shifted = |x: &[f64; N], along: &[f64; N], by: f64| -> [f64; N] {
                std::array::from_fn(|i| x[i] + by * along[i])
            };
            let unit = |joint: usize| -> [f64; N] {
                std::array::from_fn(|i| if i == joint { 1.0 } else { 0.0 })
            };
            let momentum = |q: &[f64; N], joint: usize| {
                (kinetic(q, &shifted(&qdot, &unit(joint), 1.0))
                    - kinetic(q, &shifted(&qdot, &unit(joint), -1.0)))
                    / 2.0
            };
            let tau = chain.at(&q).coriolis_torques(&qdot);
            let scale = kinetic(&q, &qdot).max(1.0);
            for (joint, &torque) in tau.iter().enumerate() {
                let momentum_rate = (momentum(&shifted(&q, &qdot, STEP), joint)
                    - momentum(&shifted(&q, &qdot, -STEP), joint))
                    / (2.0 * STEP);
                let force = (kinetic(&shifted(&q, &unit(joint), STEP), &qdot)
                    - kinetic(&shifted(&q, &unit(joint), -STEP), &qdot))
                    / (2.0 * STEP);
                let want = momentum_rate - force;
                assert!(
                    (torque - want).abs() <= TOLERANCE * scale,
                    "joint {joint}: the coupling is {torque}, Lagrange's equation {want}"
                );
            }
        }
    }
    on_a_drawn_robot::<Lagrange>(&tc);
}

#[hegel::test]
fn zero_velocity_couples_no_torque(tc: TestCase) {
    struct Still;
    impl ChainProperty for Still {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            assert_eq!(chain.at(&q).coriolis_torques(&[0.0; N]), [0.0; N]);
        }
    }
    on_a_drawn_robot::<Still>(&tc);
}

#[hegel::test]
fn the_torques_follow_the_chain_and_not_the_order_of_q(tc: TestCase) {
    // Both recursions walk proximal to distal, which is the order of the linkage
    // and not of `q`. Naming the same joints in another order must permute the
    // torques and change nothing else: it is the same arm at the same pose.
    struct FollowTheChain;
    impl ChainProperty for FollowTheChain {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let order = tc.draw(permutations(N));
            let (path_order, named) = (chain_of::<N>(robot), named_chain_of::<N>(robot, &order));
            let q = tc.draw(configs(path_order.limits()));
            let qdot = rates::<N>(tc);
            let permuted: [f64; N] = std::array::from_fn(|i| q[order[i]]);
            let permuted_dot: [f64; N] = std::array::from_fn(|i| qdot[order[i]]);

            let (want, got) = (path_order.at(&q), named.at(&permuted));
            assert!(
                (want.ee_pose().translation.vector - got.ee_pose().translation.vector).norm()
                    < TOLERANCE,
                "the two orders do not pose the same arm"
            );
            let (want_g, got_g) = (want.gravity_torques(), got.gravity_torques());
            let (want_c, got_c) = (
                want.coriolis_torques(&qdot),
                got.coriolis_torques(&permuted_dot),
            );
            for (i, &joint) in order.iter().enumerate() {
                assert!(
                    (want_g[joint] - got_g[i]).abs() < TOLERANCE,
                    "gravity on joint {joint} is {} in path order and {} named in {order:?}",
                    want_g[joint],
                    got_g[i]
                );
                assert!(
                    (want_c[joint] - got_c[i]).abs() < TOLERANCE,
                    "Coriolis on joint {joint} is {} in path order and {} named in {order:?}",
                    want_c[joint],
                    got_c[i]
                );
            }
        }
    }
    on_a_drawn_robot::<FollowTheChain>(&tc);
}

#[hegel::test]
fn the_joints_that_move_a_point_are_the_ones_above_it_on_the_chain(tc: TestCase) {
    // A witness point on a segment is moved by the joints proximal to it and by
    // no others. "Proximal" is a fact about the linkage, so a chain naming its
    // joints in another order must report the same per-joint contributions, at
    // whatever entries it puts them, and zero the same joints.
    struct AboveIt;
    impl ChainProperty for AboveIt {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let order = tc.draw(permutations(N));
            let (path_order, named) = (chain_of::<N>(robot), named_chain_of::<N>(robot, &order));
            let q = tc.draw(configs(path_order.limits()));
            let permuted: [f64; N] = std::array::from_fn(|i| q[order[i]]);
            let (posed, posed_named) = (path_order.at(&q), named.at(&permuted));

            for segment in 0..N {
                let point = Point3::from(posed.link_pose_world(segment).translation.vector);
                // The same segment, at the entry of `q` the named chain gives it.
                let as_named = order
                    .iter()
                    .position(|&joint| joint == segment)
                    .expect("every path-order joint is named");
                let want = posed.point_world_jacobian(&point, segment);
                let got = posed_named.point_world_jacobian(&point, as_named);
                for (column, &joint) in got.iter().zip(&order) {
                    assert!(
                        (want[joint] - column).norm() < TOLERANCE,
                        "segment {segment}: joint {joint} contributes {:?} in path order and \
                         {column:?} named in {order:?}",
                        want[joint]
                    );
                    assert!(
                        joint <= segment || column.norm() == 0.0,
                        "segment {segment}: joint {joint} is below the point and still moves it: \
                         {column:?}"
                    );
                }
            }
        }
    }
    on_a_drawn_robot::<AboveIt>(&tc);
}
