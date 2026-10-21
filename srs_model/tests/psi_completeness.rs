//! Completeness of the IK `FromSeed` redundancy resolution.
//!
//! `Arm::solve_ik` resolves the arm angle analytically: it computes the exact
//! set of feasible ψ intervals and returns the one nearest the seed. So whenever
//! a reachable target admits *any* in-limit configuration, it must return one,
//! regardless of the seed.
//!
//! Each target is the FK of an in-limit configuration, so it is reachable and at
//! least one ψ (the configuration's own arm angle) is feasible.

mod common;

use chain_kinematics::properties::configs;
use common::properties::sides;

/// The seed is a second, unrelated configuration, whose arm angle is not the
/// target's, so the solver searches the feasible ψ set. The explicit case is a
/// target whose only feasible arm angle is the one at which joint 7 touches its
/// limit, which no feasible interval holds.
#[hegel::test]
#[hegel::explicit_test_case(
    side = "left",
    q = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.5707959999999999],
    seed = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)]
fn from_seed_solves_every_reachable_target_from_any_seed(tc: hegel::TestCase) {
    let side = tc.draw(sides());
    let arm = common::arm(side);
    let q = tc.draw(configs(arm.limits()));
    let seed = tc.draw(configs(arm.limits()));
    common::round_trip(&arm, &q, &seed);
}
