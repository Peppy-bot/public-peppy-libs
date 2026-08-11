//! Shared control-loop primitives for the openarm control nodes.
//!
//! - [`Pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//! - [`filters`]: scalar signal filters ([`LowPassFilter`](filters::LowPassFilter),
//!   [`ButterworthFilter`](filters::ButterworthFilter),
//!   [`HampelFilter`](filters::HampelFilter)).
//! - [`minimum_jerk`]: the quintic profile and the duration a velocity budget
//!   implies for it.
//! - [`servo`]: the tolerances that decide when a Cartesian goal is reached.
//!
//! The bimanual backbone (openarm_backbone) and the real arm
//! (openarm_arm) both pace their real-time control loops with [`Pacer`]; this is
//! their one tested implementation. A home for further control primitives as they
//! are factored out of the nodes.

pub mod filters;
pub mod minimum_jerk;
mod pacer;
pub mod servo;

pub use pacer::Pacer;

use thiserror::Error;

/// Errors from constructing or driving a control_core primitive.
#[derive(Debug, Error)]
pub enum Error {
    #[error("pacer period must be non-zero")]
    ZeroPacerPeriod,
    #[error("low-pass cutoff and sample period must be finite and positive")]
    InvalidLowPass,
    #[error("Butterworth cutoff and sample period must be finite, positive, and below Nyquist")]
    InvalidButterworth,
    #[error("velocity ratio and requested duration must be finite and non-negative")]
    InvalidVelocityBudget,
    #[error("Hampel window must be within 3..=1024 and thresholds finite and positive")]
    InvalidHampel,
}
