//! The search of the inverse kinematics on drawn robots: an answer stands at its target
//! inside the limits, a descent aims at what its solver names and moves a frame by the
//! joints that move it, and a search offers its caller the ends its solver names, going on
//! from drawn configurations until the caller accepts one or its budget ends. The descents
//! recorded from MoveIt's KDL plugin are checked beside the solver (`src/ik.rs`).

mod common;

use std::cell::Cell;

use chain_kinematics::nalgebra::Isometry3;
use chain_kinematics::{Aims, Chain, ChainFrame, Draws, Ends, IkSolver, Limit, SEARCH_ITERATIONS};
use common::openarm;
use common::properties::{chain_of, named_chain_of, permutations, unreachable_poses};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{ChainProperty, TOLERANCE, configs, on_a_drawn_robot, poses};
use urdf_rs::Robot;

/// The error at which a descent converges, in meters and radians.
const CONVERGED: f64 = 1e-5;
/// The budget of a search on a drawn robot: two descents at the cap of 500 steps.
const ITERATIONS: usize = 1000;
/// The budget of a search of one descent: its first pass spends it.
const ONE_DESCENT: usize = 1;
/// The state of the draws of a search.
const DRAWS: u64 = 20261105;

/// Whether every joint of `q` stands inside `limits`, to the rounding of a clipped step.
fn inside<const N: usize>(limits: &[Limit; N], q: &[f64; N]) -> bool {
    limits
        .iter()
        .zip(q)
        .all(|(limit, v)| *v >= limit.lo - TOLERANCE && *v <= limit.hi + TOLERANCE)
}

/// The inverse kinematics of the end effector of `chain`, as KDL solves it.
fn end_effector_solver<const N: usize>(chain: &Chain<N>) -> IkSolver<'_, N> {
    IkSolver::new(chain, ChainFrame::EndEffector, Aims::Both, Ends::Converged)
}

/// A segment of a chain of `N` joints.
fn segment_of<const N: usize>(tc: &TestCase) -> usize {
    tc.draw(gs::integers::<usize>().min_value(0).max_value(N - 1))
}

/// Asserts that `chain` at `q` holds its end effector within the convergence of `target`.
fn assert_stands_at<const N: usize>(chain: &Chain<N>, q: &[f64; N], target: &Isometry3<f64>) {
    let reached = chain.world_ee_pose(q);
    let moved = (reached.translation.vector - target.translation.vector).norm();
    let turned = reached.rotation.angle_to(&target.rotation);
    assert!(moved <= CONVERGED + TOLERANCE, "{q:?} misses by {moved} m");
    assert!(
        turned <= CONVERGED + TOLERANCE,
        "{q:?} misses by {turned} rad"
    );
}

// An answer stands within the convergence of its target, inside the limits.
#[hegel::test]
fn an_answer_stands_at_its_target_inside_the_limits(tc: TestCase) {
    struct Sound;
    impl ChainProperty for Sound {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let limits = chain.limits();
            let target = chain.world_ee_pose(&tc.draw(configs(limits)));
            let seed = tc.draw(configs(limits));
            let found = end_effector_solver(&chain).search(
                seed,
                &target,
                &mut Draws::new(DRAWS),
                &|_| true,
                ITERATIONS,
            );
            let Some(answer) = found else {
                return;
            };
            assert_stands_at(&chain, &answer, &target);
            assert!(inside(&limits, &answer), "{answer:?}");
        }
    }
    on_a_drawn_robot::<Sound>(&tc);
}

// A seed already at the target is the answer, offered once, with no draw taken.
#[hegel::test]
fn a_seed_at_the_target_is_the_answer(tc: TestCase) {
    struct AtTarget;
    impl ChainProperty for AtTarget {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let offers = Cell::new(0_usize);
            let mut draws = Draws::new(DRAWS);
            let answer = end_effector_solver(&chain).search(
                q,
                &chain.world_ee_pose(&q),
                &mut draws,
                &|_| {
                    offers.set(offers.get() + 1);
                    true
                },
                ITERATIONS,
            );
            assert_eq!(answer, Some(q));
            assert_eq!(offers.get(), 1);
            assert_eq!(draws.next_bits(), Draws::new(DRAWS).next_bits());
        }
    }
    on_a_drawn_robot::<AtTarget>(&tc);
}

// A refused end does not answer, and a budget one descent spends starts no other.
#[hegel::test]
fn a_budget_one_descent_spends_starts_no_other(tc: TestCase) {
    struct OneDescent;
    impl ChainProperty for OneDescent {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let offers = Cell::new(0_usize);
            let answer = end_effector_solver(&chain).search(
                q,
                &chain.world_ee_pose(&q),
                &mut Draws::new(DRAWS),
                &|_| {
                    offers.set(offers.get() + 1);
                    false
                },
                ONE_DESCENT,
            );
            assert_eq!(answer, None);
            assert_eq!(offers.get(), 1);
        }
    }
    on_a_drawn_robot::<OneDescent>(&tc);
}

// The same search answers the same, and leaves its draws in the same state, with another
// search between the two.
#[hegel::test]
fn identical_searches_answer_identically(tc: TestCase) {
    struct Deterministic;
    impl ChainProperty for Deterministic {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let limits = chain.limits();
            let solver = end_effector_solver(&chain);
            let target = chain.world_ee_pose(&tc.draw(configs(limits)));
            let seed = tc.draw(configs(limits));
            let search = || {
                let mut draws = Draws::new(DRAWS);
                let answer = solver.search(seed, &target, &mut draws, &|_| true, ITERATIONS);
                (answer, draws.next_bits())
            };
            let first = search();
            let unrelated = chain.world_ee_pose(&tc.draw(configs(limits)));
            let _ = solver.search(
                tc.draw(configs(limits)),
                &unrelated,
                &mut Draws::new(DRAWS + 1),
                &|_| true,
                ITERATIONS,
            );
            assert_eq!(first, search());
        }
    }
    on_a_drawn_robot::<Deterministic>(&tc);
}

// A target past the chain's reach has no converged end to offer, and a search that offers
// approximate ends offers where its descent ended, inside the limits.
#[hegel::test]
fn a_target_past_the_reach_gets_an_approximate_end_alone(tc: TestCase) {
    struct PastReach;
    impl ChainProperty for PastReach {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let limits = chain.limits();
            let target = tc.draw(unreachable_poses(&chain));
            let seed = tc.draw(configs(limits));
            let search = |ends: Ends| {
                IkSolver::new(&chain, ChainFrame::EndEffector, Aims::Both, ends).search(
                    seed,
                    &target,
                    &mut Draws::new(DRAWS),
                    &|_| true,
                    ONE_DESCENT,
                )
            };
            assert_eq!(search(Ends::Converged), None);
            let end = search(Ends::Approximate).expect("the end of the search's one descent");
            assert!(inside(&limits, &end), "{end:?}");
        }
    }
    on_a_drawn_robot::<PastReach>(&tc);
}

// A descent aims at what its solver names and leaves the rest free: a seed whose end
// effector stands at the target's position, with any orientation, ends a descent aiming at
// the position, and one at the target's orientation, anywhere, ends one aiming at the
// orientation.
#[hegel::test]
fn a_descent_leaves_free_what_it_does_not_aim_at(tc: TestCase) {
    struct Aiming;
    impl ChainProperty for Aiming {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let (here, elsewhere) = (chain.world_ee_pose(&q), tc.draw(poses()));
            let search = |aims: Aims, target: Isometry3<f64>| {
                IkSolver::new(&chain, ChainFrame::EndEffector, aims, Ends::Converged).search(
                    q,
                    &target,
                    &mut Draws::new(DRAWS),
                    &|_| true,
                    ONE_DESCENT,
                )
            };
            let at_position = Isometry3::from_parts(here.translation, elsewhere.rotation);
            let at_orientation = Isometry3::from_parts(elsewhere.translation, here.rotation);
            assert_eq!(search(Aims::Position, at_position), Some(q));
            assert_eq!(search(Aims::Orientation, at_orientation), Some(q));
        }
    }
    on_a_drawn_robot::<Aiming>(&tc);
}

// A descent moves a frame on a segment by the joints on the segment's path: until it steps
// off a singularity, the joints past the segment stay at the seed.
#[hegel::test]
fn a_descent_leaves_the_joints_past_a_segment_at_the_seed(tc: TestCase) {
    struct PastTheSegment;
    impl ChainProperty for PastTheSegment {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let limits = chain.limits();
            let segment = segment_of::<N>(tc);
            let frame = ChainFrame::OnSegment {
                segment,
                offset: tc.draw(poses()),
            };
            let seed = tc.draw(configs(limits));
            let target = frame.pose_at(&chain.at(&tc.draw(configs(limits))));
            let mut draws = Draws::new(DRAWS);
            let end = IkSolver::new(&chain, frame, Aims::Both, Ends::Approximate)
                .search(seed, &target, &mut draws, &|_| true, ONE_DESCENT)
                .expect("the end of the search's one descent");
            if draws.next_bits() == Draws::new(DRAWS).next_bits() {
                assert_eq!(end[segment + 1..], seed[segment + 1..]);
            }
        }
    }
    on_a_drawn_robot::<PastTheSegment>(&tc);
}

// A frame's pose is the end effector's, or its offset on its segment's link.
#[hegel::test]
fn a_frame_stands_where_the_chain_puts_it(tc: TestCase) {
    struct Posing;
    impl ChainProperty for Posing {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let q = tc.draw(configs(chain.limits()));
            let posed = chain.at(&q);
            assert_eq!(
                ChainFrame::EndEffector.pose_at(&posed),
                chain.world_ee_pose(&q)
            );
            let (segment, offset) = (segment_of::<N>(tc), tc.draw(poses()));
            assert_eq!(
                ChainFrame::OnSegment { segment, offset }.pose_at(&posed),
                posed.link_pose_world(segment) * offset
            );
        }
    }
    on_a_drawn_robot::<Posing>(&tc);
}

// Every joint moves the end effector and the last segment along the chain, whatever the
// order of `q`, and no other segment.
#[hegel::test]
fn every_joint_moves_the_end_effector_and_the_last_segment_alone(tc: TestCase) {
    struct Moved;
    impl ChainProperty for Moved {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let order = tc.draw(permutations(N));
            let chain = named_chain_of::<N>(robot, &order);
            assert!(ChainFrame::EndEffector.moved_by_every_joint(&chain));
            for (segment, place) in order.iter().enumerate() {
                let frame = ChainFrame::OnSegment {
                    segment,
                    offset: Isometry3::identity(),
                };
                assert_eq!(frame.moved_by_every_joint(&chain), *place == N - 1);
            }
        }
    }
    on_a_drawn_robot::<Moved>(&tc);
}

// Refused ends send a search on from drawn configurations until its caller accepts one, on
// a seven-joint arm whose every pose has many configurations.
#[test]
fn a_search_goes_on_from_drawn_configurations_until_its_caller_accepts() {
    const REFUSED: usize = 3;
    let chain = openarm();
    let q = Draws::new(DRAWS).within(&chain.limits());
    let target = chain.world_ee_pose(&q);
    let offers = Cell::new(0_usize);
    let answer = end_effector_solver(&chain)
        .search(
            q,
            &target,
            &mut Draws::new(DRAWS + 1),
            &|_| {
                offers.set(offers.get() + 1);
                offers.get() > REFUSED
            },
            SEARCH_ITERATIONS,
        )
        .expect("the descents from drawn configurations converge");
    assert_eq!(offers.get(), REFUSED + 1);
    assert_ne!(answer, q);
    assert_stands_at(&chain, &answer, &target);
}
