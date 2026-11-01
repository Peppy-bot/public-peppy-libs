//! What the camera nodes and the crates that read them share, free of peppy
//! types: workspace_core sees a perception camera through their pinhole
//! model. Each node maps these onto its own generated contract members.
//!
//! - [`controls`]: why a control write gave no value, by the failure kind of
//!   peppy, the value a control answers when it has none, and the auto/manual
//!   mode of a control with a setpoint, with the `control_failure!` macro that
//!   maps a failure onto a generated service's own kinds.
//! - [`encoding`]: the color encodings a camera publishes and the depth one,
//!   and a z16 depth frame read pixel by pixel.
//! - [`v4l2`]: the V4L2 control ids and values the UVC cameras drive.
//! - [`geometry`]: the checks a pinhole model, a depth range, a depth unit
//!   and a pose pass before a consumer computes with them, the `z` depth
//!   model, and the refusal a color camera gives for its depth members.
//! - [`pinhole`]: a stream's pinhole model with its lens, parsed from its
//!   intrinsics or a rendered camera's from its field of view and giving
//!   back the intrinsics a camera node answers, what its depth samples
//!   measure, and the arithmetic between its pixels and the points of its
//!   optical frame: the ray of a pixel, the pixel that shows a point, the
//!   point of a depth reading and the reading of a point.
//! - [`pose`]: where one frame sits in another, as camera_geometry:v1 carries
//!   the depth-to-color transform, applied, composed and inverted as tf2 does.
//! - [`relay`]: the pieces of a simulated camera's relay: the camera a relay
//!   is in its copy, the timestamp check of a relayed frame, why a forwarded
//!   call has no answer, and the bounds and messages of the relay.

pub mod controls;
pub mod encoding;
pub mod geometry;
pub mod pinhole;
pub mod pose;
pub mod relay;
pub mod v4l2;

#[cfg(test)]
mod test_support {
    /// Whether every coordinate of `a` lies within `tolerance` of `b`'s.
    pub fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < tolerance)
    }
}
