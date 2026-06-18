//! Differential kinematics for the 7-DOF SRS arm: the geometric Jacobian and the
//! redundancy-aware inverses a velocity-level controller needs.
//!
//! The arm has 7 joints but the end-effector twist is 6-dimensional, so the
//! Jacobian is **6x7** and has no square inverse. The one extra DOF is the same
//! redundancy the IK layer parameterizes as the arm angle. Two inverses cover the
//! usual control schemes:
//!
//! - [`try_pseudo_inverse`] is the Moore-Penrose (minimum-norm) right inverse,
//!   returning `None` at a singularity where the Jacobian drops rank.
//! - [`damped_pseudo_inverse`] is the damped-least-squares inverse, always defined,
//!   trading tracking accuracy for conditioning near singularities. This is the one
//!   a resolved-rate loop runs every tick.
//!
//! [`manipulability`] reports proximity to a singularity, and
//! [`null_space_projector`] maps a secondary joint-rate objective (posture,
//! joint-limit avoidance, elbow control) into the Jacobian's null space so it does
//! not disturb the end-effector.
//!
//! All quantities are in the **arm base frame**, matching [`Posed::ee_pose`].

use k::nalgebra::{Matrix6, SMatrix, Vector6};

use crate::fk::Posed;
use crate::ARM_DOF;

/// Floor on `λ²` in [`damped_pseudo_inverse`], keeping `J Jᵀ + λ²I` strictly
/// positive-definite (hence invertible) even if a caller passes `lambda = 0` or a
/// non-finite value. Negligible against any real damping (`lambda ~ 1e-2`), so it
/// only guards the degenerate input, never alters intended behavior.
const MIN_DAMPING_SQ: f64 = 1e-12;

/// Geometric Jacobian of the end-effector: maps joint rates (rad/s) to the EE
/// spatial twist, both in the arm base frame. Rows 0..3 are linear velocity
/// (m/s), rows 3..6 are angular velocity (rad/s).
pub type Jacobian = SMatrix<f64, 6, { ARM_DOF }>;

/// A (pseudo-)inverse of the [`Jacobian`]: maps an EE twist to joint rates.
pub type JacobianPinv = SMatrix<f64, { ARM_DOF }, 6>;

impl Posed<'_> {
    /// Geometric Jacobian of the end-effector in the arm base frame (see
    /// [`Jacobian`]). Column `i` is revolute joint `i`'s contribution: the linear
    /// part is `zᵢ × (p_ee − pᵢ)` and the angular part is the joint axis `zᵢ`,
    /// where `pᵢ` is a point on the axis and `p_ee` the end-effector origin.
    pub fn jacobian(&self) -> Jacobian {
        let p_ee = self.ee_pose().translation.vector;
        let cols: [Vector6<f64>; ARM_DOF] = std::array::from_fn(|i| {
            let z = self.axis_base(i);
            let linear = z.cross(&(p_ee - self.origin_base(i)));
            Vector6::new(linear.x, linear.y, linear.z, z.x, z.y, z.z)
        });
        Jacobian::from_columns(&cols)
    }

    /// Minimum-norm pseudo-inverse of this posture's [`Jacobian`]; `None` at a
    /// singularity. Convenience for the one-shot case. When you also need the
    /// Jacobian itself, call [`jacobian`](Self::jacobian) once and pass it to the
    /// free [`try_pseudo_inverse`] rather than recomputing it here.
    pub fn try_pseudo_inverse(&self, eps: f64) -> Option<JacobianPinv> {
        try_pseudo_inverse(&self.jacobian(), eps)
    }

    /// Damped-least-squares inverse of this posture's [`Jacobian`] (infallible for
    /// `lambda > 0`). Convenience for a resolved-rate tick that needs only the
    /// inverse; see [`damped_pseudo_inverse`].
    pub fn damped_pseudo_inverse(&self, lambda: f64) -> JacobianPinv {
        damped_pseudo_inverse(&self.jacobian(), lambda)
    }

    /// Manipulability of this posture; see [`manipulability`].
    pub fn manipulability(&self) -> f64 {
        manipulability(&self.jacobian())
    }
}

/// Moore-Penrose (minimum-norm) right inverse `J⁺ = Jᵀ (J Jᵀ)⁻¹`, the joint rates
/// of least norm that realize a commanded EE twist. Returns `None` when the
/// Jacobian's smallest singular value is `<= eps`, i.e. at (or near) a singularity
/// where the rate solution is ill-conditioned; use [`damped_pseudo_inverse`] there
/// instead.
pub fn try_pseudo_inverse(j: &Jacobian, eps: f64) -> Option<JacobianPinv> {
    let svd = j.svd(true, true);
    // A full-row-rank 6x7 Jacobian has 6 singular values; the smallest gauges how
    // close it is to losing rank. Guarding on it (rather than letting the SVD
    // silently zero small values) is what makes this the fallible variant.
    let s_min = svd
        .singular_values
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    if s_min <= eps {
        return None;
    }
    svd.pseudo_inverse(eps).ok()
}

/// Damped-least-squares inverse `J⁺ = Jᵀ (J Jᵀ + λ²I)⁻¹`. Unlike
/// [`try_pseudo_inverse`] it is defined everywhere, including at singularities: the
/// damping bounds the joint rates at the cost of some tracking error. A
/// resolved-rate controller runs this every tick; pick `lambda` (~1e-2) to trade
/// tracking accuracy against rate magnitude near singularities.
///
/// Only `λ²` enters, so the sign of `lambda` is irrelevant. A zero or non-finite
/// `lambda` is clamped to a negligible internal floor so the result stays defined
/// rather than inverting a singular matrix; that floor is not a meaningful damping,
/// so pass a genuine value. This inverse never fails.
pub fn damped_pseudo_inverse(j: &Jacobian, lambda: f64) -> JacobianPinv {
    // λ² must be strictly positive for J Jᵀ + λ²I to be SPD; clamp a zero or
    // non-finite lambda to the floor so Cholesky always succeeds and this stays
    // infallible.
    let lambda2 = if lambda.is_finite() {
        (lambda * lambda).max(MIN_DAMPING_SQ)
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

/// Yoshikawa manipulability index `√det(J Jᵀ)`: a scalar measure of how far the
/// posture is from a singularity (0 exactly at one, larger is better-conditioned).
/// Useful for monitoring a control loop or steering the redundancy away from
/// singular regions.
pub fn manipulability(j: &Jacobian) -> f64 {
    // det(J Jᵀ) is non-negative in exact arithmetic; clamp away rounding noise
    // that can make a near-singular value slightly negative before the sqrt.
    (j * j.transpose()).determinant().max(0.0).sqrt()
}

/// Exact null-space projector `N = I − J⁺J` for the redundant DOF, where `J⁺` is
/// the Moore-Penrose inverse of `j`. Joint rates `N q̇` produce no end-effector
/// motion, so a secondary objective (posture, joint-limit avoidance, elbow
/// placement) can be added as `task + N q̇₀` without disturbing the commanded
/// twist. `N` is a true orthogonal projector (symmetric and idempotent).
///
/// It is built from the Moore-Penrose inverse on purpose: `I − J_dls⁺J` from a
/// damped inverse is only an *approximate* projector and leaks secondary motion
/// into the twist, so even a damped-least-squares controller should track its task
/// with the damped inverse but project the secondary term with this. `eps` is the
/// singular-value rank tolerance; the projector stays exact at singularities, where
/// the null space simply grows.
pub fn null_space_projector(j: &Jacobian, eps: f64) -> SMatrix<f64, { ARM_DOF }, { ARM_DOF }> {
    let pinv = j
        .svd(true, true)
        .pseudo_inverse(eps)
        .expect("SVD pseudo-inverse only fails for eps < 0");
    SMatrix::<f64, { ARM_DOF }, { ARM_DOF }>::identity() - pinv * j
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::ForwardKinematics;
    use crate::test_support::v1_fk;
    use crate::JointVec;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// Uniform in-limit joint sample (mirrors the IK tests' sampler).
    fn sample_q(rng: &mut StdRng, fk: &ForwardKinematics) -> JointVec {
        let limits = fk.limits();
        std::array::from_fn(|i| rng.gen_range(limits[i].lo..limits[i].hi))
    }

    /// EE twist between two configurations by central finite difference: the linear
    /// part from the origin delta, the angular part from the rotation delta read as
    /// a rotation vector (axis·angle).
    fn fd_twist(fk: &mut ForwardKinematics, q: &JointVec, i: usize, h: f64) -> Vector6<f64> {
        let mut q_plus = *q;
        let mut q_minus = *q;
        q_plus[i] += h;
        q_minus[i] -= h;
        let p_plus = fk.at(&q_plus).ee_pose();
        let p_minus = fk.at(&q_minus).ee_pose();
        let lin = (p_plus.translation.vector - p_minus.translation.vector) / (2.0 * h);
        let drot = p_plus.rotation * p_minus.rotation.inverse();
        let ang = drot.scaled_axis() / (2.0 * h);
        Vector6::new(lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
    }

    #[test]
    fn jacobian_matches_finite_difference() {
        for side in ["left", "right"] {
            let mut fk = v1_fk(side);
            let mut rng = StdRng::seed_from_u64(0x5A5);
            for _ in 0..200 {
                let q = sample_q(&mut rng, &fk);
                let j = fk.at(&q).jacobian();
                for i in 0..ARM_DOF {
                    let fd = fd_twist(&mut fk, &q, i, 1e-6);
                    let err = (j.column(i) - fd).norm();
                    assert!(err < 1e-5, "{side} joint {i}: column off by {err}");
                }
            }
        }
    }

    #[test]
    fn pseudo_inverse_is_a_right_inverse() {
        let mut fk = v1_fk("left");
        let mut rng = StdRng::seed_from_u64(11);
        let mut checked = 0;
        for _ in 0..200 {
            let q = sample_q(&mut rng, &fk);
            // Stay off the straight-arm singularity where rank drops.
            if q[3] < 0.2 {
                continue;
            }
            let j = fk.at(&q).jacobian();
            let pinv = try_pseudo_inverse(&j, 1e-6).expect("non-singular config has a pseudo-inverse");
            // Right inverse: J J⁺ = I₆ (so any commanded twist is realized exactly).
            let resid = (j * pinv - Matrix6::identity()).norm();
            assert!(resid < 1e-9, "J J⁺ - I = {resid}");
            checked += 1;
        }
        assert!(checked > 100, "too few non-singular samples: {checked}");
    }

    #[test]
    fn damped_approaches_pseudo_inverse_for_small_lambda() {
        let mut fk = v1_fk("left");
        let q = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let j = fk.at(&q).jacobian();
        let pinv = try_pseudo_inverse(&j, 1e-9).expect("non-singular");
        let damped = damped_pseudo_inverse(&j, 1e-6);
        assert!((pinv - damped).norm() < 1e-4, "DLS should approach J⁺ as lambda -> 0");
    }

    #[test]
    fn damped_inverse_stays_finite_at_singularity() {
        // Straight arm (elbow at its 0 limit) is a kinematic singularity: the
        // Jacobian drops rank, so the plain pseudo-inverse is unavailable but DLS
        // stays bounded.
        let mut fk = v1_fk("left");
        let q = [0.0; ARM_DOF];
        let j = fk.at(&q).jacobian();
        assert!(manipulability(&j) < 1e-6, "straight arm should be (near) singular");
        assert!(try_pseudo_inverse(&j, 1e-6).is_none(), "singular: no pseudo-inverse");
        let damped = damped_pseudo_inverse(&j, 0.05);
        assert!(damped.norm().is_finite() && damped.norm() < 1e3, "DLS blew up: {}", damped.norm());
    }

    #[test]
    fn damped_inverse_never_panics_on_degenerate_lambda() {
        // Even at a singularity, a zero / negative / non-finite lambda must yield a
        // finite inverse rather than panicking: the damping floor keeps J Jᵀ + λ²I
        // invertible.
        let mut fk = v1_fk("left");
        let j = fk.at(&[0.0; ARM_DOF]).jacobian();
        for lambda in [0.0, -0.05, f64::NAN, f64::INFINITY] {
            let d = damped_pseudo_inverse(&j, lambda);
            assert!(d.iter().all(|x| x.is_finite()), "lambda={lambda} gave non-finite inverse");
        }
    }

    #[test]
    fn manipulability_positive_off_singularity() {
        let mut fk = v1_fk("left");
        let q = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let w = manipulability(&fk.at(&q).jacobian());
        assert!(w > 1e-4, "generic posture should be well-conditioned, got {w}");
    }

    #[test]
    fn posed_methods_match_free_functions() {
        let mut fk = v1_fk("left");
        let q = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let j = fk.at(&q).jacobian();
        let want_pinv = try_pseudo_inverse(&j, 1e-9).expect("non-singular");
        let want_dls = damped_pseudo_inverse(&j, 0.05);
        let want_w = manipulability(&j);

        let posed = fk.at(&q);
        assert_eq!(posed.try_pseudo_inverse(1e-9), Some(want_pinv));
        assert_eq!(posed.damped_pseudo_inverse(0.05), want_dls);
        assert_eq!(posed.manipulability(), want_w);
    }

    #[test]
    fn null_space_projector_produces_no_ee_motion() {
        let mut fk = v1_fk("left");
        let q = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let j = fk.at(&q).jacobian();
        let n = null_space_projector(&j, 1e-9);
        // No EE twist from any null-space rate, N is an idempotent projector, and
        // (being built from the Moore-Penrose inverse) it is also symmetric.
        assert!((j * n).norm() < 1e-9, "J N should annihilate to zero");
        assert!((n * n - n).norm() < 1e-9, "N should be idempotent");
        assert!((n - n.transpose()).norm() < 1e-9, "N should be symmetric");
        // A concrete secondary joint rate maps to genuine joint motion that the EE
        // does not see.
        let qdot0 = SMatrix::<f64, { ARM_DOF }, 1>::from_fn(|i, _| (i as f64) - 3.0);
        let projected = n * qdot0;
        assert!(projected.norm() > 1e-6, "null-space motion should be non-trivial");
        assert!((j * projected).norm() < 1e-9, "projected motion must not move the EE");
    }
}
