//! Scalar signal smoothers for control loops.
//!
//! Each filter processes one sample at a time and is signal-agnostic (velocity, an opening
//! fraction, a joint command, ...); compose an array to filter a vector. All seed their
//! state on the first sample, so there is no startup transient from an assumed-zero history.
//!
//! - [`LowPassFilter`]: first-order (one-pole), the cheapest smoother.
//! - [`ButterworthFilter`]: second-order (two-pole), maximally flat with a steeper rolloff
//!   for stripping high-frequency content while barely touching the passband.

mod butterworth;
mod lowpass;

pub use butterworth::ButterworthFilter;
pub use lowpass::LowPassFilter;
