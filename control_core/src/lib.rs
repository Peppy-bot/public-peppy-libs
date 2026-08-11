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
//! Each fallible operation names its own failure, so a signature says exactly
//! what can go wrong with it rather than what can go wrong anywhere in the
//! crate.
//!
//! The bimanual backbone (openarm_backbone) and the real arm
//! (openarm_arm) both pace their real-time control loops with [`Pacer`]; this is
//! their one tested implementation. A home for further control primitives as they
//! are factored out of the nodes.

pub mod filters;
pub mod minimum_jerk;
mod pacer;
pub mod servo;

pub use pacer::{Pacer, ZeroPacerPeriod};
