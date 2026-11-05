//! The damped inverse, held to the identities that define it on robots of any
//! serial shape.

use chain_kinematics::{Jacobian, damped_pseudo_inverse};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{ChainProperty, TOLERANCE, configs, on_a_drawn_robot};
use urdf_rs::Robot;

mod common;
use common::properties::chain_of;
use common::singular_values;

/// The Jacobian of a drawn robot at a drawn configuration inside its limits.
fn drawn_jacobian<const N: usize>(tc: &TestCase, robot: &Robot) -> Jacobian<N> {
    let chain = chain_of::<N>(robot);
    chain.at(&tc.draw(configs(chain.limits()))).jacobian()
}

#[hegel::test]
fn the_damped_inverse_is_bounded_by_its_damping(tc: TestCase) {
    // Each singular value s of the Jacobian becomes s / (s² + λ²) in the damped
    // inverse, which is at most 1 / (2λ) whatever s is: the answer stays
    // bounded on every chain and at every posture, singular or under-actuated.
    // Damping from the square root of TOLERANCE up, where the damped normal
    // equations resolve the inverse well inside TOLERANCE.
    struct Bounded;
    impl ChainProperty for Bounded {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let j = drawn_jacobian::<N>(tc, robot);
            let lambda: f64 = tc.draw(gs::floats().min_value(TOLERANCE.sqrt()).max_value(1.0));
            let damped = damped_pseudo_inverse(&j, lambda);
            let largest = singular_values(&damped.transpose())
                .into_iter()
                .fold(0.0, f64::max);
            assert!(
                largest <= (1.0 + TOLERANCE) / (2.0 * lambda),
                "damped at {lambda}, the inverse reaches {largest}, past {}",
                1.0 / (2.0 * lambda)
            );
        }
    }
    on_a_drawn_robot::<Bounded>(&tc);
}

#[hegel::test]
fn the_damped_inverse_approaches_the_exact_one_as_its_damping_vanishes(tc: TestCase) {
    // Wherever the Jacobian has full row rank, the damped inverse differs from
    // its Moore-Penrose inverse J⁺ by the factor λ² / (s² + λ²) along each
    // singular direction, so by at most that factor at the smallest s, relative
    // to J⁺. The damped inverse solves its
    // normal equations through J Jᵀ, whose rounding grows with the cube of the
    // Jacobian's condition number; the comparison holds where that rounding
    // stays inside TOLERANCE. Damping from TOLERANCE up: below it the inverse
    // holds an internal floor on λ².
    struct Converges;
    impl ChainProperty for Converges {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            if N < 6 {
                return;
            }
            let j = drawn_jacobian::<N>(tc, robot);
            let singular = singular_values(&j);
            let (smallest, largest) = (
                singular.iter().copied().fold(f64::INFINITY, f64::min),
                singular.iter().copied().fold(0.0, f64::max),
            );
            if (largest / smallest).powi(3) * f64::EPSILON > TOLERANCE {
                return;
            }
            // The Moore-Penrose inverse of a Jacobian of full row rank.
            let exact = j.transpose()
                * (j * j.transpose())
                    .cholesky()
                    .expect("a Jacobian of full row rank has a positive-definite J Jᵀ")
                    .inverse();
            let lambda: f64 = tc.draw(gs::floats().min_value(TOLERANCE).max_value(1.0));
            let vanishing = lambda * lambda / (smallest * smallest + lambda * lambda);
            let distance = (damped_pseudo_inverse(&j, lambda) - exact).norm();
            assert!(
                distance <= (vanishing + TOLERANCE) * exact.norm(),
                "damped at {lambda}, the inverse is {distance} from J+, past {vanishing} of {}",
                exact.norm()
            );
        }
    }
    on_a_drawn_robot::<Converges>(&tc);
}

#[hegel::test]
fn the_damped_inverse_answers_for_any_damping(tc: TestCase) {
    // Only λ² enters, so the sign of λ changes nothing, and a zero or
    // non-finite λ still returns a finite inverse, on any posture.
    struct AnyDamping;
    impl ChainProperty for AnyDamping {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let j = drawn_jacobian::<N>(tc, robot);
            let lambda: f64 = tc.draw(gs::floats());
            let damped = damped_pseudo_inverse(&j, lambda);
            assert!(
                damped.iter().all(|x| x.is_finite()),
                "damped at {lambda}, the inverse is {damped:?}"
            );
            assert_eq!(damped, damped_pseudo_inverse(&j, -lambda), "λ = {lambda}");
        }
    }
    on_a_drawn_robot::<AnyDamping>(&tc);
}

#[hegel::test]
fn the_posture_answers_as_the_free_functions_do(tc: TestCase) {
    // The methods on a posture are the free functions applied to that
    // posture's Jacobian, value for value.
    struct SameAnswers;
    impl ChainProperty for SameAnswers {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let chain = chain_of::<N>(robot);
            let posed = chain.at(&tc.draw(configs(chain.limits())));
            let j = posed.jacobian();
            let lambda: f64 = tc.draw(gs::floats());
            assert_eq!(
                posed.damped_pseudo_inverse(lambda),
                damped_pseudo_inverse(&j, lambda)
            );
        }
    }
    on_a_drawn_robot::<SameAnswers>(&tc);
}
