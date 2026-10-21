//! Differential kinematics: the geometric Jacobian's damped inverse and its
//! manipulability, generic over the number of joints.
//!
//! A chain has `N` joints but an end-effector twist is 6-dimensional, so the
//! Jacobian is **6xN**. When `N > 6` the surplus degrees of freedom are a null
//! space; when `N < 6` not every twist is achievable. [`damped_pseudo_inverse`]
//! is the damped-least-squares inverse, defined at every posture and trading
//! tracking accuracy for conditioning near singularities: the inverse a
//! resolved-rate loop runs every tick. [`manipulability`] reports proximity to a
//! singularity.
//!
//! All quantities are in the frame the Jacobian was taken in.

use nalgebra::{DMatrix, Matrix6, SMatrix};

/// Floor on `λ²` in [`damped_pseudo_inverse`], keeping `J Jᵀ + λ²I` strictly
/// positive-definite (hence invertible) even if a caller passes `lambda = 0` or a
/// non-finite value. Negligible against any real damping (`lambda ~ 1e-2`), so it
/// only guards the degenerate input, never alters intended behavior.
const MIN_DAMPING_SQ: f64 = 1e-12;

/// Geometric Jacobian of the end-effector: maps joint rates (rad/s) to the EE
/// spatial twist, both in the arm base frame. Rows 0..3 are linear velocity
/// (m/s), rows 3..6 are angular velocity (rad/s).
pub type Jacobian<const N: usize> = SMatrix<f64, 6, N>;

/// A (pseudo-)inverse of the [`Jacobian`]: maps an EE twist to joint rates.
pub type JacobianPinv<const N: usize> = SMatrix<f64, N, 6>;

/// Damped-least-squares inverse `J⁺ = Jᵀ (J Jᵀ + λ²I)⁻¹`, defined everywhere,
/// including at singularities: the damping bounds the joint rates at the cost
/// of some tracking error. A
/// resolved-rate controller runs this every tick; pick `lambda` (~1e-2) to trade
/// tracking accuracy against rate magnitude near singularities.
///
/// Only `λ²` enters, so the sign of `lambda` is irrelevant. A zero or non-finite
/// `lambda` is clamped to a negligible internal floor so the result stays defined
/// rather than inverting a singular matrix; that floor is not a meaningful damping,
/// so pass a genuine value. A `lambda` whose square overflows damps at the largest
/// finite square. This inverse never fails.
pub fn damped_pseudo_inverse<const N: usize>(j: &Jacobian<N>, lambda: f64) -> JacobianPinv<N> {
    // λ² must be strictly positive and finite for J Jᵀ + λ²I to be SPD; clamp a
    // zero or non-finite lambda to the floor and an overflowing square to the
    // largest finite one, so Cholesky always succeeds and this stays infallible.
    let lambda2 = if lambda.is_finite() {
        (lambda * lambda).clamp(MIN_DAMPING_SQ, f64::MAX)
    } else {
        MIN_DAMPING_SQ
    };
    let jt = j.transpose();
    let damped: Matrix6<f64> = j * jt + Matrix6::identity() * lambda2;
    let inv = damped
        .cholesky()
        .expect("J Jᵀ + λ²I is SPD for λ² > 0")
        .inverse();
    jt * inv
}

/// Yoshikawa manipulability index, the product of the Jacobian's singular
/// values: a scalar measure of how far the posture is from a singularity (0
/// exactly at one, larger is better-conditioned). Useful for monitoring a
/// control loop or steering the redundancy away from singular regions.
///
/// Read off whichever Gram matrix is the smaller: `√det(J Jᵀ)` once the chain
/// has six joints or more, and `√det(Jᵀ J)` below that, where `J Jᵀ` is 6x6 of
/// rank at most `N` and so has determinant zero at every posture, singular or
/// not. The two agree on a square Jacobian, and both are the product of the
/// singular values, so the number means the same thing at any joint count.
///
/// The `N < 6` branch goes through a dynamically-sized matrix, which nalgebra
/// needs to take an `NxN` determinant, and so allocates; the six-and-above
/// branch is fixed-size throughout.
pub fn manipulability<const N: usize>(j: &Jacobian<N>) -> f64 {
    // The determinant is non-negative in exact arithmetic; clamp away rounding
    // noise that can make a near-singular value slightly negative before the sqrt.
    let gram = if N < 6 {
        let dynamic = DMatrix::from_iterator(6, N, j.iter().copied());
        (dynamic.transpose() * dynamic).determinant()
    } else {
        (j * j.transpose()).determinant()
    };
    gram.max(0.0).sqrt()
}

use crate::Posed;

impl<const N: usize> Posed<'_, N> {
    /// Damped-least-squares inverse of this posture's Jacobian, defined
    /// everywhere including singularities.
    pub fn damped_pseudo_inverse(&self, lambda: f64) -> JacobianPinv<N> {
        damped_pseudo_inverse(&self.jacobian(), lambda)
    }

    /// Yoshikawa manipulability of this posture.
    pub fn manipulability(&self) -> f64 {
        manipulability(&self.jacobian())
    }
}
