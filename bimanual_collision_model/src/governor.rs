//! The proximity governor law: how much of a commanded step to allow given
//! the current and would-be next clearance.
//!
//! Direction-aware: separating motion is never throttled, including from
//! inside the stop band, so a violation state (e.g. arms tangled at power-on)
//! is recoverable by an ordinary separating move. Only approaching motion is
//! scaled, smoothly from 1 at `d_safe` down to 0 at `d_stop`.
//!
//! Pure math over distances; the caller decides what "a step" is (a streamed
//! setpoint increment, a trajectory clock advance).
//!
//! Two scoping facts callers must know. The law sees only the endpoint
//! distances of a step, so steps must be small against the band and the body
//! sizes (true for per-tick control steps; not for arbitrary jumps). And it
//! is deliberately discontinuous at `d_next == d_now` inside the band:
//! barely-separating motion passes at 1 while hovering motion is scaled, so a
//! consumer tracking a tangential path may see the scale chatter between the
//! two; rate-limit or filter the commanded step if that matters downstream.

/// The proximity band: full speed at or above `d_safe`, full stop at or below
/// `d_stop`, linear in between. Parse, don't validate: constructing the band
/// proves `0 <= d_stop < d_safe` and finiteness once, so `scale` cannot be
/// called with a nonsense band.
#[derive(Debug, Clone, Copy)]
pub struct GovernorBand {
    d_stop: f64,
    d_safe: f64,
}

impl GovernorBand {
    pub fn new(d_stop: f64, d_safe: f64) -> Result<Self, String> {
        if !(d_stop.is_finite() && d_safe.is_finite()) {
            return Err(format!("governor band must be finite, got stop={d_stop} safe={d_safe}"));
        }
        if !(0.0 <= d_stop && d_stop < d_safe) {
            return Err(format!("governor band needs 0 <= d_stop < d_safe, got stop={d_stop} safe={d_safe}"));
        }
        Ok(Self { d_stop, d_safe })
    }

    pub fn d_stop(&self) -> f64 {
        self.d_stop
    }

    pub fn d_safe(&self) -> f64 {
        self.d_safe
    }

    /// Fraction of the commanded step to allow, in `[0, 1]`, given the
    /// clearance now and the clearance the step would produce.
    ///
    /// Separating steps (`d_next > d_now`) pass at full scale regardless of
    /// the band. Approaching steps are scaled by where they would land:
    /// 1 at or above `d_safe`, 0 at or below `d_stop`, linear between. A
    /// non-finite distance fails safe to 0.
    pub fn scale(&self, d_now: f64, d_next: f64) -> f64 {
        if !(d_now.is_finite() && d_next.is_finite()) {
            return 0.0;
        }
        if d_next > d_now {
            return 1.0;
        }
        ((d_next - self.d_stop) / (self.d_safe - self.d_stop)).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn band() -> GovernorBand {
        GovernorBand::new(0.02, 0.08).expect("valid band")
    }

    #[test]
    fn rejects_inverted_negative_or_nonfinite_bands() {
        assert!(GovernorBand::new(0.08, 0.02).is_err());
        assert!(GovernorBand::new(-0.01, 0.08).is_err());
        assert!(GovernorBand::new(0.02, 0.02).is_err());
        assert!(GovernorBand::new(f64::NAN, 0.08).is_err());
        assert!(GovernorBand::new(0.02, f64::INFINITY).is_err());
    }

    #[test]
    fn full_scale_at_and_above_safe() {
        let b = band();
        assert_eq!(b.scale(0.10, 0.08), 1.0);
        assert_eq!(b.scale(0.20, 0.15), 1.0);
    }

    #[test]
    fn zero_scale_at_and_below_stop() {
        let b = band();
        assert_eq!(b.scale(0.03, 0.02), 0.0);
        assert_eq!(b.scale(0.02, 0.01), 0.0);
        assert_eq!(b.scale(0.01, -0.01), 0.0);
    }

    #[test]
    fn linear_ramp_between_band_edges() {
        let b = band();
        let mid = (0.02 + 0.08) / 2.0;
        assert!((b.scale(0.09, mid) - 0.5).abs() < 1e-12);
        assert!((b.scale(0.09, 0.065) - 0.75).abs() < 1e-12);
    }

    #[test]
    fn approaching_scale_is_monotonic_in_landing_distance() {
        let b = band();
        let mut prev = -1.0;
        for i in 0..=100 {
            let d_next = -0.02 + 0.14 * (i as f64) / 100.0;
            let s = b.scale(d_next + 0.01, d_next);
            assert!(s >= prev, "scale not monotonic at d_next={d_next}");
            prev = s;
        }
    }

    #[test]
    fn separating_motion_is_never_throttled() {
        let b = band();
        // Above, inside, and below the band, including from penetration.
        assert_eq!(b.scale(0.10, 0.11), 1.0);
        assert_eq!(b.scale(0.05, 0.06), 1.0);
        assert_eq!(b.scale(0.005, 0.006), 1.0);
        assert_eq!(b.scale(-0.03, -0.02), 1.0);
    }

    #[test]
    fn hovering_inside_the_band_is_scaled_not_passed() {
        let b = band();
        // d_next == d_now is not separating; it lands where it stands.
        assert_eq!(b.scale(0.01, 0.01), 0.0);
        assert!((b.scale(0.05, 0.05) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn nonfinite_distances_fail_safe_to_zero() {
        let b = band();
        assert_eq!(b.scale(f64::NAN, 0.05), 0.0);
        assert_eq!(b.scale(0.05, f64::NAN), 0.0);
        assert_eq!(b.scale(f64::INFINITY, 0.05), 0.0);
    }
}
