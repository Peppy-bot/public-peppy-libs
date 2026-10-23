//! The point of this crate: chains that are not one particular arm.
//!
//! Two real robots with different joint counts, different topologies and
//! different vendors, loaded and driven through the same API. The SO-101 is
//! five actuated joints with a gripper branching off the last link; the OpenArm
//! is seven with a two-finger gripper past the wrist. Neither is special-cased.

use chain_kinematics::nalgebra::{
    Isometry3, SVector, Translation3, Unit, UnitQuaternion, Vector3, Vector6,
};
use chain_kinematics::{
    Chain, ChainSpec, DEFAULT_DLS_LAMBDA, EeCaps, JointSelection, NoSmoothing, ServoLimits,
    ServoState, ServoStep, ServoTolerances, all_within, damped_pseudo_inverse, rate_step,
    rate_step_toward,
};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, angles, configs, poses, unit_vectors};
use urdf_rs::Robot;

mod common;
use common::properties::{
    ChainProperty, TIP, chain_ending_at, chain_of, link_name, named_chain_of, on_a_drawn_robot,
    permutations,
};
use common::{SO101, openarm, so101};

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

/// The servo limits these tests drive at: 2 rad/s a joint, 0.15 m/s and 1 rad/s
/// at the end effector, a 100 Hz tick, and arrival within `position_m` and
/// 1e-2 rad.
fn servo_limits<const N: usize>(position_m: f64) -> ServoLimits<N> {
    ServoLimits {
        max_joint_velocity: [2.0; N],
        max_joint_acceleration: [5.0; N],
        ee: EeCaps {
            linear_m_s: 0.15,
            angular_rad_s: 1.0,
        },
        tolerances: ServoTolerances::new(position_m, 1e-2).expect("a reachable tolerance"),
        dt_s: 0.01,
    }
}

/// Asserts every column of `chain`'s Jacobian at `q` against a central finite
/// difference of the end-effector pose.
fn assert_jacobian_matches_finite_difference<const N: usize>(chain: &Chain<N>, q: [f64; N]) {
    const STEP: f64 = 1e-7;
    let j = chain.at(&q).jacobian();
    for i in 0..N {
        let (mut qp, mut qm) = (q, q);
        qp[i] += STEP;
        qm[i] -= STEP;
        let (a, b) = (chain.at(&qp).ee_pose(), chain.at(&qm).ee_pose());
        let lin = (a.translation.vector - b.translation.vector) / (2.0 * STEP);
        let ang = (a.rotation * b.rotation.inverse()).scaled_axis() / (2.0 * STEP);
        for (row, want) in [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
            .into_iter()
            .enumerate()
        {
            let got = j[(row, i)];
            assert!(
                (got - want).abs() < TOLERANCE,
                "column {i} row {row}: {got} vs finite difference {want}"
            );
        }
    }
}

/// The strongest check that forward kinematics and the Jacobian agree: every
/// column against a central finite difference of the pose, on chains of every
/// shape.
#[hegel::test]
fn jacobian_matches_finite_difference(tc: TestCase) {
    struct MatchesFiniteDifference;
    impl ChainProperty for MatchesFiniteDifference {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            assert_jacobian_matches_finite_difference(&chain, q);
        }
    }
    on_a_drawn_robot::<MatchesFiniteDifference>(&tc);
}

/// A mounted tool moves the point the Jacobian describes: its columns are the
/// rates of the tool frame, not of the link the tool is mounted on.
#[hegel::test]
fn the_jacobian_follows_a_mounted_tool(tc: TestCase) {
    struct FollowsTheTool;
    impl ChainProperty for FollowsTheTool {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let tooled = chain_ending_at::<N>(robot, &link_name(N))
                .with_tool_link(TIP)
                .expect("the drawn tip is fixed below the last link");
            let q = tc.draw(configs(tooled.limits()));
            assert_jacobian_matches_finite_difference(&tooled, q);
        }
    }
    on_a_drawn_robot::<FollowsTheTool>(&tc);
}

/// Naming the joints fixes the order of `q`, which is the whole reason the order
/// is an input: it is usually the robot's wire order, which no URDF knows.
/// Named in any order, a chain is the same arm with its values and limits
/// permuted alike.
#[hegel::test]
fn named_selection_fixes_the_joint_order(tc: TestCase) {
    struct NamesFixTheOrder;
    impl ChainProperty for NamesFixTheOrder {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let order = tc.draw(permutations(N));
            let (path_order, named) = (chain_of::<N>(robot), named_chain_of::<N>(robot, &order));
            let q = tc.draw(configs(path_order.limits()));
            let permuted: [f64; N] = std::array::from_fn(|i| q[order[i]]);
            assert_eq!(
                named.limits(),
                std::array::from_fn(|i| path_order.limits()[order[i]]),
                "the limits follow the names"
            );
            let (a, b) = (path_order.at(&q).ee_pose(), named.at(&permuted).ee_pose());
            assert!(
                (a.translation.vector - b.translation.vector).norm() < TOLERANCE
                    && a.rotation.angle_to(&b.rotation) < TOLERANCE,
                "naming the joints in {order:?} and permuting the values must describe the same pose"
            );
        }
    }
    on_a_drawn_robot::<NamesFixTheOrder>(&tc);
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

/// The servo law is the crate's reason to exist, so it has to drive a robot it
/// was not written for.
#[test]
fn the_servo_law_converges_on_both_robots() {
    fn run<const N: usize>(chain: &Chain<N>, seed: [f64; N], label: &str) {
        let limits = servo_limits::<N>(1e-3);

        // A goal the chain provably reaches: the pose of another configuration.
        let goal_q: [f64; N] = std::array::from_fn(|i| seed[i] * 0.4);
        let start = chain.world_ee_pose(&seed);
        let goal = chain.world_ee_pose(&goal_q);

        let state = ServoState::new(start, goal, 0.05, NoSmoothing);
        let configs = chain_kinematics::rollout(chain, state, seed, &limits, 30.0)
            .unwrap_or_else(|| panic!("{label}: servo did not converge"));
        let took = (configs.len() - 1) as f64 * limits.dt_s;
        assert!(took > 0.0 && took < 30.0, "{label}: converged in {took}s");
        assert_eq!(
            configs[0], seed,
            "{label}: the path does not start at the seed"
        );
        // A step is clamped to the velocity limit before it is added to the
        // configuration, so the difference read back rounds by up to an ulp of it.
        for step in configs.windows(2) {
            for i in 0..N {
                assert!(
                    (step[1][i] - step[0][i]).abs()
                        <= limits.max_joint_velocity[i] * limits.dt_s + 1e-12,
                    "{label}: joint {i} stepped past its velocity limit in {step:?}"
                );
            }
        }
        let short_m =
            chain_kinematics::pose_miss(&chain.world_ee_pose(&configs[configs.len() - 1]), &goal)
                .position_m;
        assert!(
            short_m < limits.tolerances.position_m(),
            "{label}: the path ends {short_m} m short of the goal"
        );
    }
    run(&so101(), [0.3, -0.4, 0.5, -0.3, 0.2], "SO-101");
    run(&openarm(), [0.3, -0.4, 0.2, 0.8, -0.5, 0.3, 0.6], "OpenArm");
}

#[hegel::test]
fn a_held_orientation_is_held_exactly_along_the_whole_line(tc: TestCase) {
    // A pure-translation move interpolates between two orientations that are the
    // same one, where slerp divides one vanishing sine by another. It stays inside
    // any tolerance, but it is not the identity, so over a move's length the held
    // orientation walks. Taking the endpoint there is what pins it.
    let hold = UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(tc.draw(unit_vectors())),
        tc.draw(angles()),
    );
    let end = || {
        Isometry3::from_parts(
            Translation3::from(
                tc.draw(unit_vectors()) * tc.draw(gs::floats().min_value(0.0).max_value(1.0)),
            ),
            hold,
        )
    };
    let (from, to) = (end(), end());
    for k in 0..=20 {
        let s = f64::from(k) / 20.0;
        let held = chain_kinematics::interpolate(&from, &to, s).rotation;
        assert_eq!(
            held,
            hold,
            "orientation drifted at s = {s}: {:e} rad",
            held.angle_to(&hold)
        );
    }
}

#[hegel::test]
fn a_tool_frame_moves_the_end_effector_and_not_the_tip(tc: TestCase) {
    // Mounting the drawn tip as a tool on the last link: the tip pose stays the
    // link's, and the end effector is the tool frame on it, the frame a chain
    // ending at the tool link reports.
    struct ToolFrame;
    impl ChainProperty for ToolFrame {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let bare = chain_ending_at::<N>(robot, &link_name(N));
            let tooled = chain_ending_at::<N>(robot, &link_name(N))
                .with_tool_link(TIP)
                .expect("the drawn tip is fixed below the last link");
            let q = tc.draw(configs(bare.limits()));
            assert_eq!(bare.at(&q).tip_pose(), tooled.at(&q).tip_pose());
            let ee = tooled.at(&q).ee_pose();
            for (label, want) in [
                (
                    "the tip with the tool",
                    bare.at(&q).tip_pose() * tooled.tool(),
                ),
                (
                    "a chain ending at the tool",
                    chain_of::<N>(robot).at(&q).ee_pose(),
                ),
            ] {
                assert!(
                    (ee.translation.vector - want.translation.vector).norm() < TOLERANCE
                        && ee.rotation.angle_to(&want.rotation) < TOLERANCE,
                    "the end effector {ee:?} is off {label} at {want:?}"
                );
            }
        }
    }
    on_a_drawn_robot::<ToolFrame>(&tc);
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
    // The OpenArm chain names a base partway down the URDF, so the world -> base
    // transform is not the identity, and a pose taken to world and back is the
    // one it started as.
    let root_based = openarm();
    assert_ne!(root_based.base_from_world(), Isometry3::identity());
    let q = [0.2; 7];
    let world = root_based.world_pose(&root_based.at(&q).ee_pose());
    let back = root_based.base_pose(&world);
    let ee = root_based.at(&q).ee_pose();
    assert!((back.translation.vector - ee.translation.vector).norm() < 1e-12);
    assert!(world.translation.vector != Vector3::zeros());
}

#[hegel::test]
fn an_arrival_tolerance_is_a_finite_positive_slack_on_each_axis(tc: TestCase) {
    let slack: f64 = tc.draw(gs::floats());
    assert_eq!(
        ServoTolerances::new(slack, 1e-2).is_ok(),
        slack.is_finite() && slack > 0.0,
        "position slack {slack} m"
    );
    assert_eq!(
        ServoTolerances::new(1e-2, slack).is_ok(),
        slack.is_finite() && slack > 0.0,
        "orientation slack {slack} rad"
    );
}

// A step toward any target leaves every joint inside its limits, and a joint
// that moved toward a limit can still stop before it from the speed it moved
// at, braking at its acceleration limit from the next tick on: the room left
// covers its braking distance.
#[hegel::test]
fn a_step_heads_into_a_limit_no_faster_than_it_can_stop_there(tc: TestCase) {
    struct StopsInTime;
    impl ChainProperty for StopsInTime {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let target = tc.draw(poses());
            let limits = ServoLimits {
                max_joint_acceleration: tc
                    .draw(gs::arrays(gs::floats().min_value(0.1).max_value(50.0))),
                ..servo_limits::<N>(1e-3)
            };
            let next = rate_step_toward(&chain, &q, &chain.world_ee_pose(&q), &target, &limits);
            assert!(all_within(&chain.limits(), &next));
            for (i, limit) in chain.limits().iter().enumerate() {
                let (moved, room_left) = if next[i] < q[i] {
                    (q[i] - next[i], next[i] - limit.lo)
                } else {
                    (next[i] - q[i], limit.hi - next[i])
                };
                let speed = moved / limits.dt_s;
                let braking = speed * speed / (2.0 * limits.max_joint_acceleration[i]);
                assert!(
                    room_left + TOLERANCE >= braking,
                    "joint {i} moved at {speed} rad/s with {room_left} left, under the {braking} it \
                     brakes over"
                );
            }
        }
    }
    on_a_drawn_robot::<StopsInTime>(&tc);
}

// A task increment of up to 0.1 m and 0.1 rad on each axis, from anywhere in
// the limits. A step that carries no joint past its limit is the free damped
// step; one that would carry exactly one joint past holds that joint on its
// limit and gives the rest the damped least-squares step for the task it
// leaves them: the gradient of that cost over the joints left free vanishes.
#[hegel::test]
fn a_joint_held_on_its_limit_leaves_the_task_to_the_rest(tc: TestCase) {
    struct TheRestTakeTheTask;
    impl ChainProperty for TheRestTakeTheTask {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let limits = chain.limits();
            let q = tc.draw(configs(limits));
            let twist: [f64; 6] = tc.draw(gs::arrays(gs::floats().min_value(-0.1).max_value(0.1)));
            let twist_world = Vector6::from(twist);
            let to_base = chain.base_from_world().rotation;
            let dp = to_base * twist_world.fixed_rows::<3>(0).into_owned();
            let dw = to_base * twist_world.fixed_rows::<3>(3).into_owned();
            let twist = Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z);
            let jacobian = chain.at(&q).jacobian();
            let free_step = damped_pseudo_inverse(&jacobian, DEFAULT_DLS_LAMBDA) * twist;
            let free: [f64; N] = std::array::from_fn(|i| q[i] + free_step[i]);
            let step = rate_step(
                &chain,
                &q,
                &twist_world,
                &limits,
                &[f64::INFINITY; N],
                0.01,
                DEFAULT_DLS_LAMBDA,
            );
            let past: Vec<usize> = (0..N).filter(|&i| !limits[i].contains(free[i])).collect();
            if past.is_empty() {
                assert!((0..N).all(|i| (step[i] - free[i]).abs() <= TOLERANCE));
                return;
            }
            let [held] = past[..] else {
                return;
            };
            // A joint the solve on the rest carries past its limit is held too;
            // the claim below is for exactly one held joint.
            let on_a_limit = |i: usize| {
                (step[i] - limits[i].lo)
                    .abs()
                    .min((limits[i].hi - step[i]).abs())
                    <= TOLERANCE
            };
            if (0..N).any(|i| i != held && on_a_limit(i)) {
                return;
            }
            let dq = SVector::<f64, N>::from_fn(|i, _| step[i] - q[i]);
            let residual = jacobian * dq - twist;
            for i in (0..N).filter(|&i| i != held) {
                let gradient =
                    jacobian.column(i).dot(&residual) + DEFAULT_DLS_LAMBDA.powi(2) * dq[i];
                assert!(
                    gradient.abs() <= TOLERANCE,
                    "with joint {held} held, the cost still falls along joint {i} ({gradient})"
                );
            }
        }
    }
    on_a_drawn_robot::<TheRestTakeTheTask>(&tc);
}

/// One revolute joint about z at the origin, reaching a quarter metre out along
/// x to its tip, which the joint therefore sweeps through y. Every coordinate is
/// a dyadic rational, so the tip pose at `q = 0` is exact and an offset from it
/// in y is exactly the number written down: the only way to put a pose error
/// *on* the tracking floor rather than a rounding step off it.
const DYADIC_ARM: &str = r#"<?xml version="1.0"?><robot name="dyadic">
  <link name="base"/>
  <link name="arm"/>
  <link name="tip"/>
  <joint name="spin" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <origin xyz="0 0 0"/>
    <limit lower="-2.0" upper="2.0" effort="1" velocity="1"/>
  </joint>
  <joint name="reach" type="fixed">
    <parent link="arm"/><child link="tip"/><origin xyz="0.25 0 0"/>
  </joint>
</robot>"#;

#[test]
fn a_tiny_position_error_is_corrected_and_the_law_arrives_within_a_tight_slack() {
    // Every position error draws a step, so an arrival slack is any positive
    // distance: a pose a micron away is reached to a tenth of that.
    const TINY_M: f64 = 1e-6;
    let robot = urdf_rs::read_from_string(DYADIC_ARM).expect("parse the dyadic arm");
    let chain = Chain::<1>::from_urdf(
        &robot,
        &ChainSpec {
            base_link: None,
            tip_link: "tip",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("one revolute joint");

    let q = [0.0];
    let ee = chain.world_ee_pose(&q);
    // The tip sits 0.25 m out, so a micron along the circle is this much joint.
    let target = chain.world_ee_pose(&[TINY_M / 0.25]);
    let limits = servo_limits::<1>(TINY_M / 10.0);
    assert_ne!(
        rate_step_toward(&chain, &q, &ee, &target, &limits),
        q,
        "an error of a micron must draw a step"
    );

    let mut state = ServoState::new(ee, target, 0.05, NoSmoothing);
    let mut walked = q;
    let converged = (0..2000).any(|_| match state.step(&chain, &walked, &limits) {
        ServoStep::Stepped(next) => {
            walked = next;
            false
        }
        ServoStep::Converged(_) => true,
    });
    assert!(
        converged,
        "a move of a micron must arrive within a tenth of it"
    );
}
