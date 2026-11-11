//! A floating-point magnitude that may be zero and has already proven itself
//! usable.

use thiserror::Error;

/// A number that cannot measure anything.
#[derive(Debug, Clone, Copy, Error)]
#[error("must be a finite number of at least 0, got {0}")]
pub struct NotNonNegativeFinite(pub f64);

/// A finite `f64` of at least zero: holding one is the proof. Negative zero
/// reads as zero.
///
/// NaN, the infinities and negative numbers cannot be inside it. The name
/// states the invariant and nothing more; whether it is a tolerance, a weight
/// or a gain is the caller's meaning to give.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct NonNegativeFinite(f64);

impl NonNegativeFinite {
    /// The only way to obtain one.
    pub fn parse(value: f64) -> Result<Self, NotNonNegativeFinite> {
        (value.is_finite() && value >= 0.0)
            .then_some(Self(value + 0.0))
            .ok_or(NotNonNegativeFinite(value))
    }

    pub fn get(self) -> f64 {
        self.0
    }
}

impl std::fmt::Display for NonNegativeFinite {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_value_that_is_negative_or_not_finite_is_refused() {
        for value in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -1.0,
            -f64::MIN_POSITIVE,
        ] {
            assert!(
                NonNegativeFinite::parse(value).is_err(),
                "{value} must be refused"
            );
        }
    }

    #[test]
    fn a_parsed_value_is_held_and_negative_zero_reads_as_zero() {
        assert_eq!(NonNegativeFinite::parse(0.25).unwrap().get(), 0.25);
        assert_eq!(NonNegativeFinite::parse(0.0).unwrap().get(), 0.0);
        let zero = NonNegativeFinite::parse(-0.0).unwrap().get();
        assert_eq!(zero.to_bits(), 0.0_f64.to_bits());
    }
}
