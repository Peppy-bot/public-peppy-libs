//! The command of one cycle of two of Nav2 1.5.2's behaviors (nav2_behaviors), each as its
//! `onCycleUpdate` computes it: Spin's turn rate (plugins/spin.cpp) and DriveOnHeading's speed
//! along x (include/nav2_behaviors/plugins/drive_on_heading.hpp), which BackUp drives backward
//! (plugins/back_up.cpp). Each gives none once its goal is reached.
//!
//! Spin turns at sqrt(2 x `rotational_acc_lim` x the turn left), raised to `min_rotational_vel`
//! and then held to `max_rotational_vel`, the way its goal turns, and ends once less than 1e-6 rad
//! of the turn is left. DriveOnHeading clamps its commanded speed to its last command plus or
//! minus one cycle of `acceleration_limit` and `deceleration_limit`, slows it to the speed from
//! which `deceleration_limit` brings the base to rest at the end of the drive when that is
//! slower, and raises it to `minimum_speed` when it is slower than that, the way its goal drives;
//! it ends once no distance is left.

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;

/// The turn left, in radians, below which a spin ends (spin.cpp:119).
const SPIN_DONE_RAD: f64 = 1e-6;

/// Spin's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpinParams {
    /// The deceleration that brings a turn to rest at its end, in rad/s² (`rotational_acc_lim`).
    pub rotational_acc_lim_rad_s2: PositiveFinite,
    /// The slowest turn rate, in rad/s (`min_rotational_vel`).
    pub min_rotational_vel_rad_s: NonNegativeFinite,
    /// The fastest turn rate, in rad/s (`max_rotational_vel`).
    pub max_rotational_vel_rad_s: NonNegativeFinite,
}

impl SpinParams {
    /// The turn rate, in rad/s, of a spin by `cmd_yaw` (rad, counterclockwise when positive) with
    /// `remaining_yaw` (rad) of it left, `|cmd_yaw|` less the magnitude of the turn made so far
    /// (spin.cpp:118-130).
    pub fn rate(&self, cmd_yaw: f64, remaining_yaw: f64) -> Option<f64> {
        if remaining_yaw < SPIN_DONE_RAD {
            return None;
        }
        let vel = (2.0 * self.rotational_acc_lim_rad_s2.get() * remaining_yaw).sqrt();
        let vel = vel
            .max(self.min_rotational_vel_rad_s.get())
            .min(self.max_rotational_vel_rad_s.get());
        Some(vel.copysign(cmd_yaw))
    }
}

/// DriveOnHeading's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveOnHeadingParams {
    /// How fast the speed may grow, in m/s² (`acceleration_limit`).
    pub acceleration_limit_m_s2: PositiveFinite,
    /// How fast the speed may fall, and the deceleration that brings the base to rest at the end
    /// of the drive, in m/s², a magnitude (`deceleration_limit`, which Nav2 gives below 0).
    pub deceleration_limit_m_s2: PositiveFinite,
    /// The slowest speed, in m/s (`minimum_speed`).
    pub minimum_speed_m_s: NonNegativeFinite,
    /// The rate the behavior runs at, in Hz (the behavior server's `cycle_frequency`).
    pub cycle_frequency_hz: PositiveFinite,
}

impl DriveOnHeadingParams {
    /// The speed along x, in m/s, of a drive at `command_speed` (m/s, backward when not above 0)
    /// with `remaining_distance` (m) of it left, its length less how far the base stands from
    /// where it started, one cycle after it commanded `current_speed` (m/s, 0 before its first
    /// command) (drive_on_heading.hpp:131-134, 142-164).
    pub fn speed(
        &self,
        command_speed: f64,
        current_speed: f64,
        remaining_distance: f64,
    ) -> Option<f64> {
        if remaining_distance <= 0.0 {
            return None;
        }
        let acceleration_limit = self.acceleration_limit_m_s2.get();
        let deceleration_limit = -self.deceleration_limit_m_s2.get();
        let cycle_frequency = self.cycle_frequency_hz.get();
        let forward = command_speed > 0.0;
        let along = |speed: f64| if forward { speed } else { -speed };
        let (min_feasible_speed, max_feasible_speed) = if forward {
            (
                current_speed + deceleration_limit / cycle_frequency,
                current_speed + acceleration_limit / cycle_frequency,
            )
        } else {
            (
                current_speed - acceleration_limit / cycle_frequency,
                current_speed - deceleration_limit / cycle_frequency,
            )
        };
        let feasible = command_speed.clamp(min_feasible_speed, max_feasible_speed);
        let max_vel_to_stop = (-2.0 * deceleration_limit * remaining_distance).sqrt();
        let slowed = if max_vel_to_stop < feasible.abs() {
            along(max_vel_to_stop)
        } else {
            feasible
        };
        let minimum_speed = self.minimum_speed_m_s.get();
        if slowed.abs() < minimum_speed {
            return Some(along(minimum_speed));
        }
        Some(slowed)
    }
}
