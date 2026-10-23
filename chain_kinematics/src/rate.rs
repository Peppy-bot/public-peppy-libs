//! One damped resolved-rate step: the inner loop of every Cartesian controller
//! here, and the whole of a streaming jog.

use nalgebra::{Isometry3, SVector, Vector3, Vector6};

use crate::jacobian::damped_pseudo_inverse;
use crate::{Chain, Jacobian, Limit, clamp_into};

/// Damping for [`rate_step`] when the caller has no reason to pick another:
/// heavy enough to stay bounded through singular postures, light enough not to
/// visibly lag a jog step. Control paths that share a step should share this
/// too, so they cannot drift apart on damping alone.
pub const DEFAULT_DLS_LAMBDA: f64 = 0.05;

/// One damped resolved-rate joint step at `q` toward a world-frame task
/// increment `twist_world`: meters of end-effector translation in its first
/// three rows and axis-angle radians of rotation in its last three, the
/// Jacobian's own layout, either of which may be zero to softly hold that
/// component.
///
/// The caller caps the increments to its speed budgets; this rotates them into
/// the chain's base frame, solves `dq = J⁺(λ) ξ` with the damped pseudo-inverse
/// (bounded through singularities), scales `dq` so every joint respects its
/// velocity budget over `dt_s` while preserving direction, and keeps the result
/// within `limits`. A joint the step would carry past its limit is held on it
/// and the task solved again on the joints left free, so they take up its
/// share of the motion (the joint clamping of Baerlocher and Boulic): the
/// joint furthest past is held first, and a held joint stays held for the
/// rest of the step.
pub fn rate_step<const N: usize>(
    chain: &Chain<N>,
    q: &[f64; N],
    twist_world: &Vector6<f64>,
    limits: &[Limit; N],
    max_joint_velocity: &[f64; N],
    dt_s: f64,
    lambda: f64,
) -> [f64; N] {
    let to_base = chain.base_from_world().rotation;
    let dp = to_base * twist_world.fixed_rows::<3>(0).into_owned();
    let dw = to_base * twist_world.fixed_rows::<3>(3).into_owned();
    let twist = Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z);
    let jacobian = chain.at(q).jacobian();
    let mut held: [Option<f64>; N] = [None; N];
    loop {
        let dq = held_step(&jacobian, &twist, &held, max_joint_velocity, dt_s, lambda);
        let next: [f64; N] = std::array::from_fn(|i| q[i] + dq[i]);
        let furthest_past = (0..N)
            .filter(|&i| held[i].is_none())
            .map(|i| (i, (limits[i].lo - next[i]).max(next[i] - limits[i].hi)))
            .filter(|&(_, past)| past > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        let Some((i, _)) = furthest_past else {
            return clamp_into(limits, &next);
        };
        held[i] = Some(limits[i].clamp(next[i]) - q[i]);
    }
}

/// The damped solve of `twist` with each `held` joint moved by its fixed
/// displacement and the rest free, the free joints scaled together so each
/// respects its velocity budget over `dt_s`.
fn held_step<const N: usize>(
    jacobian: &Jacobian<N>,
    twist: &Vector6<f64>,
    held: &[Option<f64>; N],
    max_joint_velocity: &[f64; N],
    dt_s: f64,
    lambda: f64,
) -> SVector<f64, N> {
    let fixed = SVector::<f64, N>::from_fn(|i, _| held[i].unwrap_or(0.0));
    let free = Jacobian::<N>::from_fn(|row, col| {
        if held[col].is_some() {
            0.0
        } else {
            jacobian[(row, col)]
        }
    });
    let dq = fixed + damped_pseudo_inverse(&free, lambda) * (twist - jacobian * fixed);
    // A held joint is placed on its limit; the free joints' step is scaled to
    // their velocity budgets together, keeping its direction.
    let scale = (0..N)
        .filter(|&i| held[i].is_none())
        .map(|i| {
            let cap = max_joint_velocity[i] * dt_s;
            if dq[i].abs() > cap {
                cap / dq[i].abs()
            } else {
                1.0
            }
        })
        .fold(1.0_f64, f64::min);
    SVector::from_fn(|i, _| {
        if held[i].is_some() {
            dq[i]
        } else {
            dq[i] * scale
        }
    })
}

/// How far apart two poses are: the distance between their positions and the
/// angle between their orientations.
#[derive(Debug, Clone, Copy)]
pub struct PoseMiss {
    pub position_m: f64,
    pub orientation_rad: f64,
}

/// How far `a` is from `b`: the size of each half of the twist from one to
/// the other, so equal poses are exactly no distance apart.
pub fn pose_miss(a: &Isometry3<f64>, b: &Isometry3<f64>) -> PoseMiss {
    let (dp, dw) = pose_error(a, b);
    PoseMiss {
        position_m: dp.norm(),
        orientation_rad: dw.norm(),
    }
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
