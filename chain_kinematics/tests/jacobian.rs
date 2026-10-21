//! The damped inverse and manipulability, held to the identities that define
//! them on robots of any serial shape.

use chain_kinematics::{Jacobian, damped_pseudo_inverse, manipulability};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs};
use urdf_rs::Robot;

mod common;
use common::properties::{
    ChainProperty, TIP, chain_ending_at, chain_of, link_name, on_a_drawn_robot,
};
use common::singular_values;

/// The Jacobian of a drawn robot at a drawn configuration inside its limits.
fn drawn_jacobian<const N: usize>(tc: &TestCase, robot: &Robot) -> Jacobian<N> {
    let chain = chain_of::<N>(robot);
    chain.at(&tc.draw(configs(chain.limits()))).jacobian()
}

/// The scale the determinant behind [`manipulability`] rounds on: the largest
/// singular value of `j` to the power of their count.
fn determinant_scale<const N: usize>(j: &Jacobian<N>) -> f64 {
    let singular = singular_values(j);
    let largest = singular.iter().copied().fold(0.0, f64::max);
    largest.powi(i32::try_from(singular.len()).expect("at most six singular values"))
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
            assert_eq!(posed.manipulability(), manipulability(&j));
        }
    }
    on_a_drawn_robot::<SameAnswers>(&tc);
}

#[hegel::test]
fn manipulability_is_the_product_of_the_singular_values_at_any_joint_count(tc: TestCase) {
    // The index is a distance to a singularity, and a chain of fewer than six
    // joints is not standing in one just for being short. Read off `J Jᵀ` alone
    // it would be: that product is 6x6 of rank at most N, so its determinant is
    // zero at every posture a five-joint arm can hold. The determinant the index
    // takes rounds on its own scale, which bounds the miss.
    struct Product;
    impl ChainProperty for Product {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            let j = drawn_jacobian::<N>(tc, robot);
            let singular = singular_values(&j);
            let want: f64 = singular.iter().product();
            let got = manipulability(&j);
            assert!(
                (got - want).abs() <= TOLERANCE * determinant_scale(&j),
                "manipulability is {got}, the singular values {singular:?} multiply to {want}"
            );
        }
    }
    on_a_drawn_robot::<Product>(&tc);
}

#[hegel::test]
fn manipulability_is_invariant_under_a_mounted_tool(tc: TestCase) {
    // A tool moves the point the linear rows read, which adds the angular rows
    // crossed with the offset to them: a shear of determinant one. On a chain of
    // six joints or more the index reads det(J Jᵀ), which a shear of determinant
    // one leaves alone; below six it reads det(Jᵀ J), which a shear changes.
    struct ToolInvariant;
    impl ChainProperty for ToolInvariant {
        fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
            if N < 6 {
                return;
            }
            let bare = chain_ending_at::<N>(robot, &link_name(N));
            let tooled = chain_ending_at::<N>(robot, &link_name(N))
                .with_tool_link(TIP)
                .expect("the drawn tip is fixed below the last link");
            let q = tc.draw(configs(bare.limits()));
            let (bare_j, tooled_j) = (bare.at(&q).jacobian(), tooled.at(&q).jacobian());
            let (without, with) = (manipulability(&bare_j), manipulability(&tooled_j));
            let scale = determinant_scale(&bare_j).max(determinant_scale(&tooled_j));
            assert!(
                (without - with).abs() <= TOLERANCE * scale,
                "the tool moved the manipulability from {without} to {with}"
            );
        }
    }
    on_a_drawn_robot::<ToolInvariant>(&tc);
}
