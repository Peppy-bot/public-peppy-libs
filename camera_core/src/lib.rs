//! What the camera nodes share, free of peppy types: each node maps these
//! onto its own generated contract members.
//!
//! - [`controls`]: why a control write gave no value, by the failure kind of
//!   peppy, the value a control answers when it has none, and the auto/manual
//!   mode of a control with a setpoint, with the `control_failure!` macro that
//!   maps a failure onto a generated service's own kinds.
//! - [`encoding`]: the color encodings a camera publishes and the depth one.
//! - [`v4l2`]: the V4L2 control ids and values the UVC cameras drive.
//! - [`geometry`]: the checks a pinhole model, a depth range, a depth unit
//!   and a pose pass before a consumer computes with them, the `z` depth
//!   model, and the refusal a color camera gives for its depth members.
//! - [`relay`]: the pieces of a simulated camera's relay: the camera a relay
//!   is in its copy, the timestamp check of a relayed frame, why a forwarded
//!   call has no answer, and the bounds and messages of the relay.

pub mod controls;
pub mod encoding;
pub mod geometry;
pub mod relay;
pub mod v4l2;
