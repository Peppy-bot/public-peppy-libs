//! The poses the suites place bases and sensors by.

use control_core::planar_pose::PlanarPose;
use frame_tree::Transform;

/// The planar pose at `x` and `y`, heading `yaw`, each finite.
pub fn planar(x: f64, y: f64, yaw: f64) -> PlanarPose {
    PlanarPose::new(x, y, yaw).expect("a finite pose")
}

/// The pose of a sensor at `sensor` in the map's frame, in the frame of a base at `base` on
/// the map's floor: where the sensor sits on that base.
pub fn on_base(base: PlanarPose, sensor: &Transform) -> Transform {
    base.to_transform().inverse().compose(sensor)
}

/// `pose` moved for `dt_s` at `velocity`, linear x, linear y and angular z in the base's
/// frame, as Nav2's collision monitor's `projectState` moves a pose.
pub fn moved(pose: PlanarPose, velocity: [f64; 3], dt_s: f64) -> PlanarPose {
    let [vx, vy, wz] = velocity;
    let (sin, cos) = pose.yaw().sin_cos();
    planar(
        pose.x() + (vx * cos - vy * sin) * dt_s,
        pose.y() + (vx * sin + vy * cos) * dt_s,
        pose.yaw() + wz * dt_s,
    )
}
