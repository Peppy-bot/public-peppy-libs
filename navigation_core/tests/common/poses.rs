//! The poses the suites place bases and sensors by.

use camera_core::pose::{PlanarPose, Pose};

/// The planar pose at `x` and `y`, heading `yaw`, each finite.
pub fn planar(x: f64, y: f64, yaw: f64) -> PlanarPose {
    PlanarPose::new(x, y, yaw).expect("a finite pose")
}

/// The pose of a sensor at `sensor` in the map's frame, in the frame of a base at `base` on
/// the map's floor: where the sensor sits on that base.
pub fn on_base(base: PlanarPose, sensor: &Pose) -> Pose {
    base.to_pose().inverse().compose(sensor)
}
