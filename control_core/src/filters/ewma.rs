//! Exponentially weighted moving average over measured wall time.

use crate::filters::FilterError;

/// An exponentially weighted moving average of a scalar signal, integrating
/// wall time: each step folds one sample in with `alpha = 1 - e^(-dt/tau)`
/// for the measured interval `dt`, exact for any spacing (two half-steps
/// compose to one full step), so a loop that jitters or chronically
/// overruns still averages over true seconds where a fixed coefficient
/// would silently stretch the time constant with the real loop period.
///
/// The state seeds at zero and earns its value over `tau`, so a threshold
/// judged on the average cannot trip on the first samples. That makes this
/// an integrator for judgment windows ("sustained above X for seconds"),
/// where the smoothers in this module condition a signal: they seed on the
/// first sample and track it.
///
/// `max_step_s` bounds the elapsed time one sample may claim. The exact
/// alpha weights a sample by its interval, so a sample arriving after a
/// long stall would otherwise count as though its value had held for the
/// whole gap, letting one reading overwrite the entire history in either
/// direction. Beyond the bound the interval is capped and the average ages
/// toward the new sample instead of snapping to it.
#[derive(Debug, Clone, Copy)]
pub struct Ewma {
    tau_s: f64,
    max_step_s: f64,
    value: f64,
}

impl Ewma {
    /// A zero-seeded average with time constant `tau_s`, no single step
    /// counting more than `max_step_s` of elapsed time. Both must be finite
    /// and positive, or this is [`FilterError::InvalidEwma`]: a degenerate
    /// time constant has no averaging meaning, and an unbounded step is the
    /// one-sample-rewrites-history hazard the cap exists for.
    pub fn new(tau_s: f64, max_step_s: f64) -> Result<Self, FilterError> {
        let valid = tau_s.is_finite() && tau_s > 0.0 && max_step_s.is_finite() && max_step_s > 0.0;
        if !valid {
            return Err(FilterError::InvalidEwma);
        }
        Ok(Self {
            tau_s,
            max_step_s,
            value: 0.0,
        })
    }

    /// Folds one sample in over the measured interval `dt_s`, clamped to
    /// `max_step_s`, and returns the updated average. Asserts both finite
    /// (and the interval non-negative) because a NaN would silently poison
    /// the average, and every threshold judged on it would disengage
    /// without a trace.
    pub fn step(&mut self, x: f64, dt_s: f64) -> f64 {
        assert!(x.is_finite(), "sample must be finite, got {x}");
        assert!(
            dt_s.is_finite() && dt_s >= 0.0,
            "dt_s must be finite and non-negative, got {dt_s}"
        );
        let alpha = 1.0 - (-dt_s.min(self.max_step_s) / self.tau_s).exp();
        self.value += alpha * (x - self.value);
        self.value
    }

    /// The current average: zero until the first step, thereafter what the
    /// last [`step`](Self::step) returned.
    pub fn value(&self) -> f64 {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TAU: f64 = 5.0;
    const MAX_STEP: f64 = 1.0;

    fn ewma() -> Ewma {
        Ewma::new(TAU, MAX_STEP).expect("valid parameters")
    }

    #[test]
    fn new_rejects_non_positive_or_non_finite_parameters() {
        for (tau, max_step) in [
            (0.0, MAX_STEP),
            (-1.0, MAX_STEP),
            (f64::NAN, MAX_STEP),
            (f64::INFINITY, MAX_STEP),
            (TAU, 0.0),
            (TAU, -1.0),
            (TAU, f64::NAN),
            (TAU, f64::INFINITY),
        ] {
            assert_eq!(
                Ewma::new(tau, max_step).map(|_| ()),
                Err(FilterError::InvalidEwma),
                "tau={tau} max_step={max_step}"
            );
        }
    }

    #[test]
    fn a_step_input_reaches_63_percent_in_one_time_constant() {
        let mut e = ewma();
        let ticks = (TAU / 0.01) as usize;
        let last = (0..ticks).map(|_| e.step(1.0, 0.01)).last().unwrap();
        assert!((last - (1.0 - (-1.0f64).exp())).abs() < 1e-3);
    }

    #[test]
    fn uneven_sample_spacing_integrates_like_even_spacing() {
        let mut even = ewma();
        let mut uneven = ewma();
        for _ in 0..200 {
            even.step(1.0, 0.01);
            uneven.step(1.0, 0.004);
            uneven.step(1.0, 0.006);
        }
        let (a, b) = (even.value(), uneven.value());
        assert!((a - b).abs() < 1e-9, "{a} vs {b}");
    }

    #[test]
    fn a_long_gap_is_capped_not_rejected() {
        let mut capped = ewma();
        let mut exact = ewma();
        capped.step(1.0, MAX_STEP * 100.0);
        exact.step(1.0, MAX_STEP);
        assert_eq!(capped.value(), exact.value());
    }

    #[test]
    fn a_zero_length_step_is_a_no_op_rather_than_a_panic() {
        let mut e = ewma();
        e.step(1.0, 0.01);
        let before = e.value();
        assert_eq!(e.step(0.0, 0.0), before);
    }

    #[test]
    fn the_state_seeds_at_zero_not_on_the_first_sample() {
        let mut e = ewma();
        assert_eq!(e.value(), 0.0);
        let first = e.step(1.0, 0.01);
        assert!(first < 0.01, "one tick must not reach the sample: {first}");
    }

    #[test]
    #[should_panic(expected = "sample must be finite")]
    fn a_non_finite_sample_is_rejected() {
        ewma().step(f64::NAN, 0.01);
    }

    #[test]
    #[should_panic(expected = "dt_s must be finite")]
    fn a_non_finite_interval_is_rejected() {
        ewma().step(0.0, f64::NAN);
    }

    #[test]
    #[should_panic(expected = "dt_s must be finite and non-negative")]
    fn a_negative_interval_is_rejected() {
        ewma().step(0.0, -0.01);
    }
}
