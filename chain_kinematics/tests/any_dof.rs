//! The point of this crate: chains that are not one particular arm.
//!
//! Two real robots with different joint counts, different topologies and
//! different vendors, loaded and driven through the same API. The SO-101 is
//! five actuated joints with a gripper branching off the last link; the OpenArm
//! is seven with a two-finger gripper past the wrist. Neither is special-cased.

use chain_kinematics::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use chain_kinematics::{
    Chain, ChainSpec, EeCaps, JointSelection, NoSmoothing, ServoLimits, ServoState,
    ServoTolerances, damped_pseudo_inverse, manipulability,
};

const SO101: &str = include_str!("fixtures/so101_5dof.urdf");
const OPENARM: &str = include_str!("fixtures/openarm_7dof.urdf");

const SO101_JOINTS: [&str; 5] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
];

fn so101() -> Chain<5> {
    let robot = urdf_rs::read_from_string(SO101).expect("parse SO-101");
    Chain::<5>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "gripper_frame_link",
            joints: JointSelection::Named(&SO101_JOINTS),
        },
    )
    .expect("SO-101 is a five-joint chain")
}

fn openarm() -> Chain<7> {
    let robot = urdf_rs::read_from_string(OPENARM).expect("parse OpenArm");
    Chain::<7>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: Some("openarm_left_link0"),
            tip_link: "openarm_left_link7",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("OpenArm is a seven-joint chain")
}

/// A configuration spread across each joint's range, deterministically.
fn sample<const N: usize>(chain: &Chain<N>, k: usize) -> [f64; N] {
    let limits = chain.limits();
    std::array::from_fn(|i| {
        let t = ((k + 1) as f64 * 0.618_033_988_749_894_9 * (i + 1) as f64).fract();
        limits[i].lo + t * (limits[i].hi - limits[i].lo)
    })
}

#[test]
fn loads_arms_of_different_joint_counts() {
    let five = so101();
    let seven = openarm();
    assert_eq!(five.limits().len(), 5);
    assert_eq!(seven.limits().len(), 7);
    for (i, l) in five.limits().iter().enumerate() {
        assert!(l.lo < l.hi, "SO-101 joint {i} has an empty range");
    }
    for (i, l) in seven.limits().iter().enumerate() {
        assert!(l.lo < l.hi, "OpenArm joint {i} has an empty range");
    }
}

/// The strongest check that forward kinematics and the Jacobian agree: every
/// column against a central finite difference of the pose, on both robots.
#[test]
fn jacobian_matches_finite_difference() {
    fn check<const N: usize>(chain: &Chain<N>, label: &str) {
        let h = 1e-7;
        for k in 0..24 {
            let q = sample(chain, k);
            let j = chain.at(&q).jacobian();
            for i in 0..N {
                let (mut qp, mut qm) = (q, q);
                qp[i] += h;
                qm[i] -= h;
                let (a, b) = (chain.at(&qp).ee_pose(), chain.at(&qm).ee_pose());
                let lin = (a.translation.vector - b.translation.vector) / (2.0 * h);
                let ang = (a.rotation * b.rotation.inverse()).scaled_axis() / (2.0 * h);
                for (row, want) in [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
                    .into_iter()
                    .enumerate()
                {
                    let got = j[(row, i)];
                    assert!(
                        (got - want).abs() < 1e-5,
                        "{label} column {i} row {row}: {got} vs finite difference {want}"
                    );
                }
            }
        }
    }
    check(&so101(), "SO-101");
    check(&openarm(), "OpenArm");
}

/// Naming the joints fixes the order of `q`, which is the whole reason the order
/// is an input: it is usually the robot's wire order, which no URDF knows.
#[test]
fn named_selection_fixes_the_joint_order() {
    let robot = urdf_rs::read_from_string(SO101).expect("parse SO-101");
    let spec = |names: &'static [&'static str]| ChainSpec {
        base_link: None,
        tip_link: "gripper_frame_link",
        joints: JointSelection::Named(names),
    };
    let forward = Chain::<5>::from_urdf(&robot, &spec(&SO101_JOINTS)).unwrap();
    const REVERSED: [&str; 5] = [
        "wrist_roll",
        "wrist_flex",
        "elbow_flex",
        "shoulder_lift",
        "shoulder_pan",
    ];
    let reversed = Chain::<5>::from_urdf(&robot, &spec(&REVERSED)).unwrap();

    let q = [0.1, -0.2, 0.3, -0.4, 0.5];
    let mut flipped = q;
    flipped.reverse();
    let a = forward.at(&q).ee_pose();
    let b = reversed.at(&flipped).ee_pose();
    assert!(
        (a.translation.vector - b.translation.vector).norm() < 1e-12
            && a.rotation.angle_to(&b.rotation) < 1e-12,
        "reversing the names and the values must describe the same pose"
    );
    // And path order agrees with the names given in path order.
    let path_order = Chain::<5>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "gripper_frame_link",
            joints: JointSelection::PathOrder,
        },
    )
    .unwrap();
    let c = path_order.at(&q).ee_pose();
    assert!((a.translation.vector - c.translation.vector).norm() < 1e-12);
}

#[test]
fn a_wrong_joint_count_is_refused_at_load() {
    let robot = urdf_rs::read_from_string(SO101).expect("parse SO-101");
    let spec = ChainSpec {
        base_link: None,
        tip_link: "gripper_frame_link",
        joints: JointSelection::PathOrder,
    };
    // The SO-101 arm has five actuated joints; asking for seven must fail here,
    // at load, rather than producing a chain that silently ignores `q[5..]`.
    let err = match Chain::<7>::from_urdf(&robot, &spec) {
        Ok(_) => panic!("a five-joint arm must not load as a seven-joint chain"),
        Err(e) => e,
    };
    assert!(
        format!("{err}").contains("expected 7"),
        "error should name the mismatch: {err}"
    );
}

#[test]
fn the_damped_inverse_serves_an_under_actuated_chain() {
    // Five joints cannot meet an arbitrary six-dimensional twist. The damped
    // inverse must still return a bounded least-squares answer rather than
    // failing, which is what lets one control law drive both robots.
    let chain = so101();
    for k in 0..16 {
        let q = sample(&chain, k);
        let j = chain.at(&q).jacobian();
        let pinv = damped_pseudo_inverse(&j, 0.05);
        assert!(
            pinv.iter().all(|x| x.is_finite()),
            "damped inverse must stay finite on a 6x5 Jacobian"
        );
        assert!(manipulability(&j).is_finite());
    }
}

/// The servo law is the crate's reason to exist, so it has to drive a robot it
/// was not written for.
#[test]
fn the_servo_law_converges_on_both_robots() {
    fn run<const N: usize>(chain: &Chain<N>, seed: [f64; N], label: &str) {
        let limits = ServoLimits {
            max_joint_velocity: [2.0; N],
            ee: EeCaps {
                linear_m_s: 0.15,
                angular_rad_s: 1.0,
            },
            tolerances: ServoTolerances {
                position_m: 1e-3,
                orientation_rad: 1e-2,
            },
            dt_s: 0.01,
        };

        // A goal the chain provably reaches: the pose of another configuration.
        let goal_q: [f64; N] = std::array::from_fn(|i| seed[i] * 0.4);
        let start = chain.world_pose(&chain.at(&seed).ee_pose());
        let goal = chain.world_pose(&chain.at(&goal_q).ee_pose());

        let mut state = ServoState::new(start, goal, 0.05, NoSmoothing);
        let took = chain_kinematics::rollout(chain, &mut state, seed, &limits, 30.0);
        let took = took.unwrap_or_else(|| panic!("{label}: servo did not converge"));
        assert!(took > 0.0 && took < 30.0, "{label}: converged in {took}s");
    }
    run(&so101(), [0.3, -0.4, 0.5, -0.3, 0.2], "SO-101");
    run(&openarm(), [0.3, -0.4, 0.2, 0.8, -0.5, 0.3, 0.6], "OpenArm");
}

#[test]
fn a_held_orientation_is_held_exactly_along_the_whole_line() {
    // A pure-translation move interpolates between two orientations that are the
    // same one, where slerp divides one vanishing sine by another. It stays inside
    // any tolerance, but it is not the identity, so over a move's length the held
    // orientation walks. Taking the endpoint there is what pins it.
    let hold = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.3);
    let at = |x: f64| Isometry3::from_parts(Translation3::new(x, 0.0, 0.0), hold);
    let (from, to) = (at(0.0), at(0.25));
    for k in 0..=20 {
        let s = k as f64 / 20.0;
        let held = chain_kinematics::interpolate(&from, &to, s).rotation;
        assert_eq!(
            held,
            hold,
            "orientation drifted at s = {s}: {:e} rad",
            held.angle_to(&hold)
        );
    }
}

#[test]
fn a_tool_frame_moves_the_end_effector_and_not_the_tip() {
    let bare = openarm();
    let tooled = openarm()
        .with_tool_link("openarm_left_tcp")
        .expect("the OpenArm URDF carries a tcp frame");
    let q = [0.3, -0.4, 0.2, 0.8, -0.5, 0.3, 0.6];
    assert_eq!(bare.at(&q).tip_pose(), tooled.at(&q).tip_pose());
    let tool = tooled.tool();
    assert!(
        tool.translation.vector.norm() > 0.1,
        "tcp should sit off the tip"
    );
    let expected = bare.at(&q).tip_pose() * tool;
    let ee = tooled.at(&q).ee_pose();
    assert!((ee.translation.vector - expected.translation.vector).norm() < 1e-12);
}

#[test]
fn a_tool_reached_through_a_moving_joint_is_refused() {
    // The moving jaw's offset tracks the gripper opening, so it is not a frame
    // anything can be commanded at.
    let err = so101()
        .with_tool_link("moving_jaw_so101_v1_link")
        .expect_err("a link below a revolute joint is not a fixed tool frame");
    assert!(format!("{err}").contains("fixed below the tip"), "{err}");
}

#[test]
fn poses_are_reported_in_the_named_base_frame() {
    // With the base at the URDF root the transform is the identity; naming a link
    // partway down moves the reported frame with it.
    let root_based = openarm();
    assert_ne!(root_based.base_from_world(), Isometry3::identity());
    let q = [0.2; 7];
    let world = root_based.world_pose(&root_based.at(&q).ee_pose());
    let back = root_based.base_pose(&world);
    let ee = root_based.at(&q).ee_pose();
    assert!((back.translation.vector - ee.translation.vector).norm() < 1e-12);
    assert!(world.translation.vector != Vector3::zeros());
}
