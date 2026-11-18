//! Wheel odometry of a differential base, as ros2_controllers 6.9.0's diff
//! drive controller integrates it from wheel positions
//! (diff_drive_controller/src/odometry.cpp: `Odometry::update_from_pos`,
//! `update_from_vel` and `integrate`, with the rolling mean of rcpputils'
//! `RollingMeanAccumulator`).
//!
//! Each update takes the two wheel angles (rad) and the time since the last
//! update (s): the wheel rates are the angle changes over that time, the base
//! velocity their mix by the wheel radii and the wheel separation, and the
//! pose moves along the arc that velocity draws over the time, or along its
//! heading when the turn is below 1e-6 rad. The heading
//! accumulates without wrapping. The reported velocity is the rolling mean of
//! the last `velocity_rolling_window_size` velocities.
//!
//! An update whose wheel angles, rates or time are not finite, or whose pose
//! would not be, changes nothing, where ros2_controllers integrates it into
//! the pose for good; wheel angles to measure from that are not finite are
//! refused.

use std::num::NonZeroUsize;

use crate::not_finite::NotFinite;
use crate::positive_finite::PositiveFinite;

/// The shortest time (s) an update integrates over, as odometry.cpp's
/// updates compare it.
const MIN_UPDATE_DT_S: f64 = 1e-6;

/// The smallest turn (rad) of one update integrated along an arc; a smaller
/// one moves the pose along its heading, as odometry.cpp's `integrate` does.
const MIN_ARC_TURN_RAD: f64 = 1e-6;

/// The mean of the last `window` values, summed as rcpputils sums them: each
/// value added to the running sum and the value it replaces subtracted.
#[derive(Debug, Clone)]
struct RollingMean {
    buffer: Vec<f64>,
    next_insert: usize,
    sum: f64,
    buffer_filled: bool,
}

impl RollingMean {
    fn new(window: NonZeroUsize) -> Self {
        Self {
            buffer: vec![0.0; window.get()],
            next_insert: 0,
            sum: 0.0,
            buffer_filled: false,
        }
    }

    /// Takes `value`, and gives the mean of the values taken, at most the
    /// window, of which `value` is one.
    fn accumulate(&mut self, value: f64) -> f64 {
        self.sum -= self.buffer[self.next_insert];
        self.sum += value;
        self.buffer[self.next_insert] = value;
        self.next_insert += 1;
        self.buffer_filled |= self.next_insert >= self.buffer.len();
        self.next_insert %= self.buffer.len();
        let count = if self.buffer_filled {
            self.buffer.len()
        } else {
            self.next_insert
        };
        self.sum / count as f64
    }
}

/// The geometry of a differential base's two driven wheels, by
/// ros2_controllers' names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelGeometry {
    /// The distance between the wheels' contact points, in meters
    /// (`wheel_separation`).
    pub wheel_separation_m: PositiveFinite,
    /// The left wheel's radius, in meters (`left_wheel_radius`).
    pub left_wheel_radius_m: PositiveFinite,
    /// The right wheel's radius, in meters (`right_wheel_radius`).
    pub right_wheel_radius_m: PositiveFinite,
}

/// The robot frame in the odometry frame, from the wheels.
#[derive(Debug, Clone)]
pub struct Odometry {
    geometry: WheelGeometry,
    x: f64,
    y: f64,
    heading: f64,
    linear: f64,
    angular: f64,
    left_wheel_old_position: f64,
    right_wheel_old_position: f64,
    linear_mean: RollingMean,
    angular_mean: RollingMean,
}

impl Odometry {
    /// At the origin, both wheels at angle 0, with the rolling mean over
    /// `velocity_rolling_window_size` velocities.
    pub fn new(geometry: WheelGeometry, velocity_rolling_window_size: NonZeroUsize) -> Self {
        Self {
            geometry,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            linear: 0.0,
            angular: 0.0,
            left_wheel_old_position: 0.0,
            right_wheel_old_position: 0.0,
            linear_mean: RollingMean::new(velocity_rolling_window_size),
            angular_mean: RollingMean::new(velocity_rolling_window_size),
        }
    }

    /// The wheel angles (rad) where they stand now, taken as the angles the
    /// next update measures from. The pose and the velocity stay. Angles that
    /// are not finite are refused and change nothing.
    pub fn anchor_wheels(
        &mut self,
        left_position: f64,
        right_position: f64,
    ) -> Result<(), NotFinite<2>> {
        NotFinite::check(
            "the wheel angles to measure from",
            [("left", left_position), ("right", right_position)],
            "the angles in radians each wheel's encoder reads",
        )?;
        self.left_wheel_old_position = left_position;
        self.right_wheel_old_position = right_position;
        Ok(())
    }

    /// `update_from_pos`: integrate the wheels' move to these angles (rad)
    /// over `dt` (s), and measure the next move from them. Returns false,
    /// changing nothing, when |dt| is below 1e-6 s or the update is not
    /// finite.
    pub fn update_from_positions(
        &mut self,
        left_position: f64,
        right_position: f64,
        dt: f64,
    ) -> bool {
        let left_rate = (left_position - self.left_wheel_old_position) / dt;
        let right_rate = (right_position - self.right_wheel_old_position) / dt;
        if !self.update_from_rates(left_rate, right_rate, dt) {
            return false;
        }
        self.left_wheel_old_position = left_position;
        self.right_wheel_old_position = right_position;
        true
    }

    /// `update_from_vel`: integrate the wheel rates (rad/s) over `dt` (s).
    /// Returns false, changing nothing, when |dt| is below 1e-6 s or the
    /// update is not finite.
    fn update_from_rates(&mut self, left_rate: f64, right_rate: f64, dt: f64) -> bool {
        if !(dt.is_finite() && dt.abs() >= MIN_UPDATE_DT_S) {
            return false;
        }
        let left_radius = self.geometry.left_wheel_radius_m.get();
        let right_radius = self.geometry.right_wheel_radius_m.get();
        let linear = (left_rate * left_radius + right_rate * right_radius) * 0.5;
        let angular = (right_rate * right_radius - left_rate * left_radius)
            / self.geometry.wheel_separation_m.get();
        let (x, y, heading) = self.integrated(linear, angular, dt);
        if ![linear, angular, x, y, heading]
            .iter()
            .all(|value| value.is_finite())
        {
            return false;
        }
        (self.x, self.y, self.heading) = (x, y, heading);
        self.linear = self.linear_mean.accumulate(linear);
        self.angular = self.angular_mean.accumulate(angular);
        true
    }

    /// `integrate`: the pose moved along the arc of this velocity over `dt`.
    fn integrated(&self, linear: f64, angular: f64, dt: f64) -> (f64, f64, f64) {
        let distance = linear * dt;
        let turn = angular * dt;
        if turn.abs() < MIN_ARC_TURN_RAD {
            return (
                self.x + distance * self.heading.cos(),
                self.y + distance * self.heading.sin(),
                self.heading + turn,
            );
        }
        let heading = self.heading + turn;
        (
            self.x + (distance / turn) * (heading.sin() - self.heading.sin()),
            self.y + -(distance / turn) * (heading.cos() - self.heading.cos()),
            heading,
        )
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    /// The heading (rad), accumulated without wrapping.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// The rolling mean of the forward velocity (m/s).
    pub fn linear(&self) -> f64 {
        self.linear
    }

    /// The rolling mean of the turn rate (rad/s).
    pub fn angular(&self) -> f64 {
        self.angular
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Finite rates over a finite time whose arc runs past the largest number change
    // nothing.
    #[test]
    fn an_update_whose_pose_would_not_be_finite_changes_nothing() {
        let positive = |value| PositiveFinite::parse(value).expect("above 0");
        let geometry = WheelGeometry {
            wheel_separation_m: positive(0.36),
            left_wheel_radius_m: positive(0.08),
            right_wheel_radius_m: positive(0.08),
        };
        let mut odometry = Odometry::new(geometry, NonZeroUsize::MIN);
        assert!(odometry.update_from_rates(1.0, 1.0, 0.1));
        let before = (
            odometry.x(),
            odometry.y(),
            odometry.heading(),
            odometry.linear(),
        );
        assert!(!odometry.update_from_rates(f64::MAX, f64::MAX, 1e300));
        let after = (
            odometry.x(),
            odometry.y(),
            odometry.heading(),
            odometry.linear(),
        );
        assert_eq!(after, before);
    }
}
