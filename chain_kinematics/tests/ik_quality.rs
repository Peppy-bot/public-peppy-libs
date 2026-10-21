//! The qualification suite for the point-to-point layer: the same measured bar
//! `chain_kinematics_py` published, ported as tests, so "the engine passes the
//! oracle" is a cargo test verdict rather than a claim.
//!
//! Poses are reachable by construction (forward kinematics of in-limit
//! configurations), seeds are unrelated to the answers, and every draw comes
//! from a fixed generator, so a verdict is a function of the code alone.

mod common;

use chain_kinematics::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use chain_kinematics::properties::configs;
use chain_kinematics::{ACCEPT_POSITION_M, Goal, Kinematics};
use common::properties::{ChainProperty, chain_of, on_a_drawn_robot, unreachable_poses};
use common::{SplitMix64, openarm, so101};
use hegel::TestCase;
use hegel::generators as gs;
use urdf_rs::Robot;

fn kin5() -> Kinematics<5> {
    Kinematics::new(so101())
}

fn kin7() -> Kinematics<7> {
    Kinematics::new(openarm())
}

#[test]
fn reachable_poses_are_never_refused() {
    fn run<const N: usize>(kinematics: &Kinematics<N>, cases: usize, rng: u64, label: &str) {
        let mut rng = SplitMix64(rng);
        let unrelated = kinematics.limits().map(|l| l.midpoint());
        let one_degree = std::f64::consts::PI / 180.0;
        let (mut refused, mut worst_m, mut abandoned, mut worst_angle) =
            (0usize, 0.0f64, 0usize, 0.0f64);
        for _ in 0..cases {
            let target = kinematics.forward_kinematics(&rng.config(kinematics.chain()));
            match kinematics.inverse_kinematics(&unrelated, &Goal::at(target)) {
                None => refused += 1,
                Some(solved) => {
                    let reached = kinematics.forward_kinematics(&solved);
                    worst_m = worst_m
                        .max((reached.translation.vector - target.translation.vector).norm());
                    let angle = reached.rotation.angle_to(&target.rotation);
                    if angle > one_degree {
                        abandoned += 1;
                        worst_angle = worst_angle.max(angle);
                    }
                }
            }
        }
        eprintln!(
            "{label}: refused {refused}/{cases}, worst position {worst_m:.2e} m, \
             orientation abandoned {abandoned}/{cases} (worst {:.1} deg)",
            worst_angle.to_degrees()
        );
        assert_eq!(
            refused, 0,
            "{label}: refused {refused} of {cases} reachable poses"
        );
        assert!(
            worst_m <= 1.0e-3,
            "{label}: an accepted solution missed by {worst_m:.2e} m"
        );
        // The measured bar: 1/1000 abandoned on the 5-DOF arm, 0/300 on the
        // 7-DOF, worst miss 2.1 degrees. The gate carries slack for the debug
        // subset, where one abandoned draw is a larger fraction.
        let allowed = (cases as f64 * 0.003 + 1.0) as usize;
        assert!(
            abandoned <= allowed,
            "{label}: abandoned {abandoned}/{cases} attainable orientations (allowed {allowed})"
        );
        assert!(
            worst_angle <= 15.0_f64.to_radians(),
            "{label}: worst abandoned orientation {:.1} deg",
            worst_angle.to_degrees()
        );
    }
    // Full scale is the qualification run (`cargo test --release`); the debug
    // build keeps a fast regression subset of the same draws.
    let (so101_cases, openarm_cases) = if cfg!(debug_assertions) {
        (150, 60)
    } else {
        (1000, 300)
    };
    run(&kin5(), so101_cases, 11, "SO-101");
    run(&kin7(), openarm_cases, 12, "OpenArm");
}

#[test]
fn track_comes_to_rest_against_an_unreachable_target() {
    // The frozen clause, ported shape and all: seven unreachable directions on
    // both chains, and exactly zero motion over the last 50 of 200 ticks.
    fn run<const N: usize>(kinematics: &Kinematics<N>, label: &str) {
        let directions: [[f64; 3]; 7] = [
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.577, 0.577, 0.577],
            [-0.577, 0.577, -0.577],
        ];
        for direction in directions {
            let target = Isometry3::from_parts(
                Translation3::new(
                    direction[0] * 10.0,
                    direction[1] * 10.0,
                    direction[2] * 10.0,
                ),
                UnitQuaternion::identity(),
            );
            let mut q = kinematics.limits().map(|l| l.midpoint());
            let mut trail = Vec::with_capacity(200);
            for _ in 0..200 {
                q = kinematics.track(&q, &target);
                trail.push(q);
            }
            let settled = trail[150];
            for (tick, sample) in trail.iter().enumerate().skip(150) {
                assert_eq!(
                    *sample, settled,
                    "{label}: direction {direction:?} still moving at tick {tick}"
                );
            }
        }
    }
    run(&kin5(), "SO-101");
    run(&kin7(), "OpenArm");
}

#[hegel::test]
fn track_never_moves_away_from_its_target(tc: TestCase) {
    // The descent keeps the best configuration it reaches, with the seed among
    // its candidates, so a tick never ends farther from the target than it began.
    struct Approaches;
    impl ChainProperty for Approaches {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let kinematics = Kinematics::new(chain_of::<N>(robot));
            let target = tc.draw(unreachable_poses(kinematics.chain()));
            let miss = |q: &[f64; N]| {
                (kinematics.forward_kinematics(q).translation.vector - target.translation.vector)
                    .norm()
            };
            let start = tc.draw(configs(kinematics.limits()));
            std::iter::successors(Some(start), |q| Some(kinematics.track(q, &target)))
                .take(200)
                .collect::<Vec<_>>()
                .windows(2)
                .enumerate()
                .for_each(|(tick, pair)| {
                    assert!(
                        miss(&pair[1]) <= miss(&pair[0]),
                        "tick {tick}: the miss grew from {} to {}",
                        miss(&pair[0]),
                        miss(&pair[1])
                    );
                });
        }
    }
    on_a_drawn_robot::<Approaches>(&tc);
}

#[hegel::test]
fn identical_solves_answer_identically(tc: TestCase) {
    // The determinism contract, with an unrelated solve between the two calls
    // so any hidden state would show.
    struct Deterministic;
    impl ChainProperty for Deterministic {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let kinematics = Kinematics::new(chain_of::<N>(robot));
            let limits = kinematics.limits();
            let goal = Goal::at(kinematics.forward_kinematics(&tc.draw(configs(limits))));
            let seed = tc.draw(configs(limits));
            let first = kinematics.inverse_kinematics(&seed, &goal);
            let unrelated = Goal::at(kinematics.forward_kinematics(&tc.draw(configs(limits))));
            let _ = kinematics.inverse_kinematics(&tc.draw(configs(limits)), &unrelated);
            assert_eq!(first, kinematics.inverse_kinematics(&seed, &goal));
        }
    }
    on_a_drawn_robot::<Deterministic>(&tc);
}

#[hegel::test]
fn an_unreachable_target_is_refused(tc: TestCase) {
    struct Refused;
    impl ChainProperty for Refused {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let kinematics = Kinematics::new(chain_of::<N>(robot));
            let goal = Goal::at(tc.draw(unreachable_poses(kinematics.chain())));
            let seed = tc.draw(configs(kinematics.limits()));
            assert!(kinematics.inverse_kinematics(&seed, &goal).is_none());
            assert!(kinematics.continue_to(&seed, &goal).is_none());
        }
    }
    on_a_drawn_robot::<Refused>(&tc);
}

#[test]
fn a_caller_named_position_bar_loosens_acceptance_per_goal() {
    // The same target flips refusal to acceptance on the caller's own bar, with
    // no second solver instance.
    let kinematics = kin5();
    let seed = kinematics.limits().map(|l| l.midpoint());
    let far = Isometry3::from_parts(Translation3::new(0.0, 0.0, 5.0), UnitQuaternion::identity());
    assert!(
        kinematics
            .inverse_kinematics(&seed, &Goal::at(far))
            .is_none()
    );
    let loose = Goal::at(far)
        .position_within(10.0)
        .expect("a positive bar parses");
    let accepted = kinematics
        .inverse_kinematics(&seed, &loose)
        .expect("a 10 m bar accepts the boundary");
    let reached = kinematics.forward_kinematics(&accepted);
    assert!((reached.translation.vector - far.translation.vector).norm() <= 10.0);
}

#[hegel::test]
fn tolerances_are_parsed_at_the_boundary(tc: TestCase) {
    // A bar parses exactly when it is a positive, finite number.
    let goal = Goal::at(Isometry3::identity());
    let bar: f64 = tc.draw(gs::floats());
    let valid = bar.is_finite() && bar > 0.0;
    assert_eq!(
        goal.position_within(bar).is_ok(),
        valid,
        "position bar {bar}"
    );
    assert_eq!(
        goal.orientation_within(bar).is_ok(),
        valid,
        "orientation bar {bar}"
    );
}

#[test]
#[should_panic(expected = "finite")]
fn a_non_finite_seed_is_rejected() {
    let kinematics = kin5();
    let mut seed = kinematics.limits().map(|l| l.midpoint());
    seed[1] = f64::NAN;
    let _ = kinematics.track(&seed, &Isometry3::identity());
}

#[hegel::test]
fn every_answer_is_in_limits_and_reaches_the_goal(tc: TestCase) {
    // From any finite seed, in limits or far outside them: a seed is clamped
    // before the search, so whatever the search answers is a configuration the
    // chain can hold, and it reaches the goal within the acceptance bar.
    struct Sound;
    impl ChainProperty for Sound {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let kinematics = Kinematics::new(chain_of::<N>(robot));
            let limits = kinematics.limits();
            let target = kinematics.forward_kinematics(&tc.draw(configs(limits)));
            let seed: [f64; N] =
                tc.draw(gs::arrays(gs::floats().min_value(-1.0e3).max_value(1.0e3)));
            let Some(solved) = kinematics.inverse_kinematics(&seed, &Goal::at(target)) else {
                return;
            };
            assert!(
                limits.iter().zip(solved).all(|(l, v)| l.contains(v)),
                "the answer {solved:?} must be in limits"
            );
            let chain = kinematics.chain();
            let reached = chain.at(&solved).ee_pose().translation.vector;
            let wanted = chain.base_pose(&target).translation.vector;
            assert!(
                (reached - wanted).norm() <= ACCEPT_POSITION_M,
                "the answer {solved:?} misses by {:.2e} m",
                (reached - wanted).norm()
            );
        }
    }
    on_a_drawn_robot::<Sound>(&tc);
}

#[test]
fn continue_to_follows_a_step_while_a_joint_rests_on_a_limit() {
    // Every joint starts on a limit, and the step's least-squares answer pushes
    // joint 2, which slides along the same line as joint 1, below its lower
    // limit of 0: joint 1 alone carries the step.
    let urdf = r#"<robot name="t">
  <link name="link_0"/><link name="link_1"/><link name="link_2"/><link name="link_3"/>
  <link name="link_4"/><link name="tip"/>
  <joint name="joint_1" type="prismatic"><parent link="link_0"/><child link="link_1"/>
    <axis xyz="1 0 0"/><limit lower="-0.125" upper="0" effort="1" velocity="1"/></joint>
  <joint name="joint_2" type="prismatic"><parent link="link_1"/><child link="link_2"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="0.125" effort="1" velocity="1"/></joint>
  <joint name="joint_3" type="prismatic"><parent link="link_2"/><child link="link_3"/>
    <origin rpy="0 0.125 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.125" upper="0" effort="1" velocity="1"/></joint>
  <joint name="joint_4" type="prismatic"><parent link="link_3"/><child link="link_4"/>
    <origin rpy="0 1 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.125" upper="0" effort="1" velocity="1"/></joint>
  <joint name="tip_joint" type="fixed"><parent link="link_4"/><child link="tip"/></joint>
</robot>"#;
    let robot = urdf_rs::read_from_string(urdf).expect("parse fixture");
    let kinematics = Kinematics::new(chain_of::<4>(&robot));
    let goal = Goal::at(kinematics.forward_kinematics(&[-0.03125, 0.0, 0.0, 0.0]));
    assert!(kinematics.continue_to(&[0.0; 4], &goal).is_some());
}

#[hegel::test]
fn continue_to_stays_in_the_seed_neighbourhood(tc: TestCase) {
    // A path's next sample: solvable from the previous one, and answered with
    // the continuation rather than a distant reconfiguration.
    struct Continues;
    impl ChainProperty for Continues {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let kinematics = Kinematics::new(chain_of::<N>(robot));
            let limits = kinematics.limits();
            let q0 = tc.draw(configs(limits));
            let step: [f64; N] = tc.draw(gs::arrays(gs::floats().min_value(-0.05).max_value(0.05)));
            let q1: [f64; N] = std::array::from_fn(|i| limits[i].clamp(q0[i] + step[i]));
            let stepped = kinematics
                .continue_to(&q0, &Goal::at(kinematics.forward_kinematics(&q1)))
                .unwrap_or_else(|| panic!("the step from {q0:?} to {q1:?} is unsolvable"));
            let drift = stepped
                .iter()
                .zip(q0)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f64, f64::max);
            assert!(
                drift < 0.5,
                "continue_to jumped {drift:.2} away from its seed {q0:?}"
            );
        }
    }
    on_a_drawn_robot::<Continues>(&tc);
}
