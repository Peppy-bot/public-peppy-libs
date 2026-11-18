//! The wheel odometry against the diff drive odometry of ros2_controllers 6.9.0
//! (diff_drive_controller src/odometry.cpp): `Odometry::update_from_positions` ports
//! `update_from_pos`, `update_from_vel` and `integrate`, and the rolling means rcpputils'
//! accumulator, so for the same wheel positions, geometry and period, each update's pose
//! and velocity lie within 1e-12 of the diff drive controller's: the two sides compute the
//! same operations, and their sines and cosines come from the libm of each side's build.
//! The evidence: every diff drive odometry scenario of the ROS reference harness.

mod common;

use std::num::NonZeroUsize;

use common::finite::positive;
use control_core::odometry::{Odometry, WheelGeometry};
use ros_reference::diff_drive_odometry::Cases;

/// How far a pose coordinate or a velocity may lie from the diff drive controller's.
const BOUND: f64 = 1e-12;

// Each update's pose and velocity lie within the bound of the diff drive controller's.
#[test]
fn the_odometry_is_the_diff_drive_controllers() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a diff drive odometry scenario"
    );
    for cases in &scenarios {
        let wheels = cases.wheels;
        let geometry = WheelGeometry {
            wheel_separation_m: positive(wheels.wheel_separation),
            left_wheel_radius_m: positive(wheels.left_wheel_radius),
            right_wheel_radius_m: positive(wheels.right_wheel_radius),
        };
        let window =
            NonZeroUsize::new(wheels.velocity_rolling_window_size).expect("the scenario's window");
        let mut odometry = Odometry::new(geometry, window);
        for (index, expected) in cases.updates.iter().enumerate() {
            let [left, right] = expected.positions;
            let integrated = odometry.update_from_positions(left, right, cases.period);
            let ours = [
                odometry.x(),
                odometry.y(),
                odometry.heading(),
                odometry.linear(),
                odometry.angular(),
            ];
            let theirs = [
                expected.pose[0],
                expected.pose[1],
                expected.pose[2],
                expected.velocity[0],
                expected.velocity[1],
            ];
            let worst = ours
                .iter()
                .zip(&theirs)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            assert!(
                integrated == expected.integrated && worst <= BOUND,
                "{} update {index}: ours {ours:?} ({integrated}), the controller's {theirs:?} \
                 ({}), {worst:e} apart",
                cases.name,
                expected.integrated
            );
        }
    }
}
