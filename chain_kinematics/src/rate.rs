//! One damped resolved-rate step: the inner loop of every Cartesian controller
//! here, and the whole of a streaming jog.

use nalgebra::{Isometry3, SVector, Vector3, Vector6};

use crate::jacobian::damped_pseudo_inverse;
use crate::{Chain, Limit};

/// Damping for [`rate_step`] when the caller has no reason to pick another:
/// heavy enough to stay bounded through singular postures, light enough not to
/// visibly lag a jog step. Control paths that share a step should share this
/// too, so they cannot drift apart on damping alone.
pub const DEFAULT_DLS_LAMBDA: f64 = 0.05;

/// One damped resolved-rate joint step at `q` toward a world-frame task
/// increment: `dp_world` metres of end-effector translation and `dw_world`
/// axis-angle radians of rotation, either of which may be zero to softly hold
/// that component.
///
/// The caller caps the increments to its speed budgets; this rotates them into
/// the chain's base frame, solves `dq = J⁺(λ) ξ` with the damped pseudo-inverse
/// (bounded through singularities), scales `dq` so every joint respects its
/// velocity budget over `dt_s` while preserving direction, and clamps the result
/// into the position limits.
pub fn rate_step<const N: usize>(
    chain: &Chain<N>,
    q: &[f64; N],
    dp_world: Vector3<f64>,
    dw_world: Vector3<f64>,
    max_joint_velocity: &[f64; N],
    dt_s: f64,
    lambda: f64,
) -> [f64; N] {
    let to_base = chain.base_from_world().rotation;
    let dp = to_base * dp_world;
    let dw = to_base * dw_world;
    let twist = Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z);
    let jacobian = chain.at(q).jacobian();
    let mut dq: SVector<f64, N> = damped_pseudo_inverse(&jacobian, lambda) * twist;
    let scale = (0..N)
        .map(|i| {
            let cap = max_joint_velocity[i] * dt_s;
            if dq[i].abs() > cap {
                cap / dq[i].abs()
            } else {
                1.0
            }
        })
        .fold(1.0_f64, f64::min);
    dq *= scale;
    let limits: [Limit; N] = chain.limits();
    std::array::from_fn(|i| limits[i].clamp(q[i] + dq[i]))
}

/// A pose difference as a world-frame twist: the translation, and the rotation
/// as an axis-angle vector. Shared by the callers that build a task error. A
/// rotation whose quaternion vector part is within rounding of zero is none, so
/// equal orientations draw exactly no correction; any other is read off the
/// quaternion, whose half-angle `atan2` stays finite down to the smallest angle.
pub(crate) fn pose_error(
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let dp = to.translation.vector - from.translation.vector;
    let dr = to.rotation * from.rotation.inverse();
    let dw = if dr.imag().norm() <= f64::EPSILON {
        Vector3::zeros()
    } else {
        dr.scaled_axis()
    };
    (dp, dw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::generators as gs;
    use kinematics_properties::{rotations, unit_vectors};
    use nalgebra::{Unit, UnitQuaternion};

    #[hegel::test]
    fn the_smallest_rotations_give_a_finite_twist(tc: hegel::TestCase) {
        // A rotation of 1e-16 to 1e-8 rad, about any axis, after any base
        // rotation. The twist matches the angle to within the rounding of
        // composing the rotations, a few units in the last place of 1.
        let base = UnitQuaternion::from_rotation_matrix(&tc.draw(rotations()));
        let exponent: f64 = tc.draw(gs::floats().min_value(-16.0).max_value(-8.0));
        let angle = 10f64.powf(exponent);
        let axis = Unit::new_normalize(tc.draw(unit_vectors()));
        let from = Isometry3::from_parts(Vector3::zeros().into(), base);
        let to = Isometry3::from_parts(
            Vector3::zeros().into(),
            UnitQuaternion::from_axis_angle(&axis, angle) * base,
        );
        let (_, dw) = pose_error(&from, &to);
        assert!(
            dw.iter().all(|v| v.is_finite()) && (dw.norm() - angle).abs() < 1e-15,
            "a {angle:e} rad rotation gave the twist {dw:?}"
        );
    }
}
