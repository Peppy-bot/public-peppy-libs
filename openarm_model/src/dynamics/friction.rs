//! Per-joint friction torque the actuator must apply to overcome friction.
//! Tanh model with per-joint Fo/Fv/Fc/k constants:
//!
//!   τ_fric[i] = Fo[i] + Fv[i]·ω[i] + Fc[i]·tanh(k[i]·ω[i])
//!
//! The constants are revision-specific, so this crate defines only the
//! [`Params`] type and the [`torques`] math; the caller (the `openarm_description`
//! layer) supplies the right `Params` per robot revision.

use crate::{ARM_DOF, JointVec};

/// Per-joint friction-model constants for the tanh model above.
#[derive(Debug, Clone, Copy)]
pub struct Params {
    pub fc: [f64; ARM_DOF],
    pub fv: [f64; ARM_DOF],
    pub fo: [f64; ARM_DOF],
    pub k: [f64; ARM_DOF],
}

/// Friction torque at velocity `qdot` for the given constants.
pub fn torques(p: &Params, qdot: &JointVec) -> JointVec {
    std::array::from_fn(|i| p.fo[i] + p.fv[i] * qdot[i] + p.fc[i] * f64::tanh(p.k[i] * qdot[i]))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Self-contained constants for the agnostic `torques` math (distinct,
    /// non-zero per joint). The real revision values live in
    /// `openarm_description::friction`.
    const FIXTURE: Params = Params {
        fc: [0.09, 0.10, 0.12, 0.05, 0.015, 0.025, 0.05],
        fv: [0.02, 0.03, 0.18, 0.24, 0.009, 0.02, 0.025],
        fo: [0.026, 0.027, 0.002, -0.017, 0.0015, 0.0027, -0.018],
        k: [2.84, 2.90, 2.95, 13.0, 15.0, 24.0, 0.79],
    };

    #[test]
    fn at_zero_velocity_equals_offset() {
        let p = &FIXTURE;
        let tau = torques(p, &[0.0; ARM_DOF]);
        // ω=0 → tanh(0)=0, Fv·ω=0, so τ = Fo.
        for (i, &t) in tau.iter().enumerate() {
            assert!(
                (t - p.fo[i]).abs() < 1e-12,
                "joint {i}: tau={t} Fo={}",
                p.fo[i]
            );
        }
    }

    #[test]
    fn at_high_velocity_saturates() {
        let (p, omega) = (&FIXTURE, 100.0);
        let tau = torques(p, &[omega; ARM_DOF]);
        // ω large positive → tanh→+1, so τ ≈ Fo + Fv·ω + Fc.
        for (i, &t) in tau.iter().enumerate() {
            let expected = p.fo[i] + p.fv[i] * omega + p.fc[i];
            assert!(
                (t - expected).abs() < 1e-6,
                "joint {i}: tau={t} expected={expected}"
            );
        }
    }

    #[test]
    fn antisymmetric_about_zero_modulo_offset() {
        // Coulomb + viscous components are odd in ω; only Fo breaks antisymmetry.
        let (p, omega) = (&FIXTURE, 0.5);
        let pos = torques(p, &[omega; ARM_DOF]);
        let neg = torques(p, &[-omega; ARM_DOF]);
        for (i, (&pp, &nn)) in pos.iter().zip(&neg).enumerate() {
            assert!((pp + nn - 2.0 * p.fo[i]).abs() < 1e-9, "joint {i}");
        }
    }
}
