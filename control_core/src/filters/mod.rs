//! Scalar signal filters for control loops.
//!
//! Each filter processes one sample at a time and is signal-agnostic (velocity, an opening
//! fraction, a joint command, ...); compose an array to filter a vector. All seed their
//! state on the first sample, so there is no startup transient from an assumed-zero history.
//!
//! - [`LowPassFilter`]: first-order (one-pole), the cheapest smoother.
//! - [`ButterworthFilter`]: second-order (two-pole), maximally flat with a steeper rolloff
//!   for stripping high-frequency content while barely touching the passband.
//! - [`HampelFilter`]: median-based outlier rejector for impulsive spikes; passes clean
//!   samples through unchanged where the smoothers would smear the spike into the output.

mod butterworth;
mod hampel;
mod lowpass;

use thiserror::Error;

/// Parameters a filter cannot be built from.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum FilterError {
    #[error("low-pass cutoff and sample period must be finite and positive")]
    InvalidLowPass,
    #[error("Butterworth cutoff and sample period must be finite, positive, and below Nyquist")]
    InvalidButterworth,
    #[error("Hampel window must be within 3..=1024 and thresholds finite and positive")]
    InvalidHampel,
}

pub use butterworth::ButterworthFilter;
pub use hampel::HampelFilter;
pub use lowpass::LowPassFilter;
