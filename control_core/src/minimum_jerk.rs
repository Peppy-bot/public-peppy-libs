//! The quintic minimum-jerk blend between two values, and its peak rate.

/// Peak normalised velocity of the quintic blend `s(tau)`: `ds/dtau` at
/// `tau = 0.5`. Over a blend of duration `T`, a quantity changing by `delta`
/// peaks at `QUINTIC_PEAK_VELOCITY * delta / T`. 15/8 exactly.
pub const QUINTIC_PEAK_VELOCITY: f64 = 1.875;

/// The quintic minimum-jerk blend `s(tau) = 10 tau³ - 15 tau⁴ + 6 tau⁵` for
/// `tau = t/T`, held at its endpoints: 0 at or before `tau = 0`, 1 at or after
/// `tau = 1`. In between, `s` rises from 0 to 1 with zero rate and zero
/// acceleration at both ends, so a path blended by it starts and stops at
/// rest: the smoothest profile that meets fixed boundary conditions.
pub fn blend(tau: f64) -> f64 {
    let tau = tau.clamp(0.0, 1.0);
    ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    /// `ds/dtau` of the blend inside `[0, 1]`, read off the blend itself by
    /// central differences over `H`: its error is the blend's third derivative
    /// times `H²`, a part in a million.
    const H: f64 = 1e-4;

    fn rate(tau: f64) -> f64 {
        (blend(tau + H) - blend(tau - H)) / (2.0 * H)
    }

    // The blend rests at 0 and 1, rises monotonically to its rounding, mirrors
    // about its midpoint, holds outside `[0, 1]`, and its rate, read off the
    // blend by finite differences, never passes the documented peak, 15/8 at
    // the midpoint, and reaches it there.
    #[hegel::test]
    fn the_blend_rises_from_rest_to_rest_within_its_peak_rate(tc: TestCase) {
        assert_eq!(QUINTIC_PEAK_VELOCITY, 15.0 / 8.0);
        assert!((rate(0.5) - QUINTIC_PEAK_VELOCITY).abs() < 1e-6);
        assert_eq!((blend(0.0), blend(1.0)), (0.0, 1.0));
        // Inside the blend by `H`, where the differences read the polynomial
        // and not its clamp.
        let tau: f64 = tc.draw(gs::floats().min_value(H).max_value(1.0 - H));
        let later: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let outside: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        assert!(
            rate(tau) >= -1e-6 && rate(tau) <= QUINTIC_PEAK_VELOCITY + 1e-6,
            "rate at {tau}"
        );
        assert!(
            (blend(tau) + blend(1.0 - tau) - 1.0).abs() < 1e-12,
            "asymmetric at {tau}"
        );
        if later >= tau {
            // Near the end the polynomial cancels, `10 - 9`, so its value
            // there carries a few ulps of 10.
            assert!(
                blend(later) >= blend(tau) - 1e-14,
                "fell from {tau} to {later}"
            );
        }
        assert_eq!((blend(-outside), blend(1.0 + outside)), (0.0, 1.0));
    }
}
