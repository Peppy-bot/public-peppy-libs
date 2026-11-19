//! What the camera nodes and the crates that read them share, free of peppy
//! types: occupancy_map reads their depth frames, workspace_core sees a
//! perception camera through their pinhole model, and navigation_core reads
//! the scans of a 2-D lidar. Each node maps these onto its own generated
//! contract members.
//!
//! - [`controls`]: why a control write gave no value, by the failure kind of
//!   peppy, the value a control answers when it has none, and the auto/manual
//!   mode of a control with a setpoint, with the `control_failure!` macro that
//!   maps a failure onto a generated service's own kinds.
//! - [`encoding`]: the color encodings a camera publishes and the depth one,
//!   and a z16 depth frame read pixel by pixel.
//! - [`v4l2`]: the V4L2 control ids and values the UVC cameras drive.
//! - [`geometry`]: the checks a pinhole model, a depth range and a depth unit
//!   pass before a consumer computes with them, the `z` depth model, and the
//!   refusal a color camera gives for its depth members.
//! - [`pinhole`]: a stream's pinhole model with its lens, parsed from its
//!   intrinsics or a rendered camera's from its field of view and giving
//!   back the intrinsics a camera node answers, what its depth samples
//!   measure, and the arithmetic between its pixels and the points of its
//!   optical frame: the ray of a pixel, the pixel that shows a point, the
//!   point of a depth reading and the reading of a point.
//! - [`laser_scan`]: the scans of a 2-D lidar as lidar_2d:v1 carries them,
//!   each ray's hit placed where the sensor stood at the ray's time.
//! - [`relay`]: the pieces of a simulated camera's relay: why a forwarded
//!   call has no answer, and the bounds and messages of the relay.
//! - [`sensor_mounts`]: the sensor a node runs as, of a class that holds the
//!   kinds it takes, the camera class among them, read once at its start: the
//!   name it runs under in its copy and the frame it names in each
//!   measurement, the one sensor_mounts:v1's `describe_sensors` gives that
//!   name through the node's poll of its bound link, or a frame of its own name
//!   with the link vacant, and the check that holds a value a relay forwards to
//!   that frame.

pub mod controls;
pub mod encoding;
pub mod geometry;
pub mod laser_scan;
pub mod pinhole;
pub mod relay;
pub mod sensor_mounts;
pub mod v4l2;

#[cfg(test)]
mod test_support {
    /// Whether every coordinate of `a` lies within `tolerance` of `b`'s.
    pub fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < tolerance)
    }
}
