//! Whether a base makes progress, as Nav2 1.5.2's simple progress checker
//! judges it (nav2_controller plugins/simple_progress_checker.cpp: `check`,
//! `reset`, `resetBaselinePose` and `isRobotMovedEnough`).
//!
//! The first check takes the base's position and the time as its baseline.
//! A check that finds the base more than `required_movement_m` from the
//! baseline takes the new position and time as the baseline; one that finds
//! it within that distance for longer than `movement_time_allowance_s` since
//! the baseline finds no progress. Times are integer nanoseconds, as ROS
//! compares them.

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::planar_pose::PlanarPose;
use control_core::time::{elapsed_ns, nanoseconds, seconds};

use crate::pose::distance;

/// The progress checker's settings: Nav2's `required_movement_radius` and
/// `movement_time_allowance`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProgressCheckerParams {
    /// How far, in meters, the base must move to make progress.
    pub required_movement_m: NonNegativeFinite,
    /// How long, in seconds, the base may take to move that far.
    pub movement_time_allowance_s: NonNegativeFinite,
}

/// A base that moved too little for too long.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "the base moved {moved_m} m in {elapsed_s} s, no more than the required movement of \
     {required_m} m within the allowance of {allowance_s} s"
)]
pub struct NoProgress {
    pub moved_m: f64,
    pub elapsed_s: f64,
    pub required_m: f64,
    pub allowance_s: f64,
}

/// Where and when the base last made progress.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Baseline {
    pose: PlanarPose,
    time_ns: i64,
}

/// The progress checker and its baseline.
#[derive(Debug)]
pub struct ProgressChecker {
    params: ProgressCheckerParams,
    allowance_ns: i64,
    baseline: Option<Baseline>,
}

impl ProgressChecker {
    pub fn new(params: ProgressCheckerParams) -> Self {
        Self {
            params,
            allowance_ns: nanoseconds(params.movement_time_allowance_s.get()),
            baseline: None,
        }
    }

    /// `reset`: take the next check's position and time as the baseline.
    pub fn reset(&mut self) {
        self.baseline = None;
    }

    /// `check` of the base at `pose` at `now_ns` on the caller's clock.
    pub fn check(&mut self, pose: PlanarPose, now_ns: i64) -> Result<(), NoProgress> {
        let here = Baseline {
            pose,
            time_ns: now_ns,
        };
        let Some(baseline) = self.baseline else {
            self.baseline = Some(here);
            return Ok(());
        };
        let moved_m = distance(pose, baseline.pose);
        if moved_m > self.params.required_movement_m.get() {
            self.baseline = Some(here);
            return Ok(());
        }
        let elapsed = elapsed_ns(baseline.time_ns, now_ns);
        if elapsed > self.allowance_ns {
            return Err(NoProgress {
                moved_m,
                elapsed_s: seconds(elapsed),
                required_m: self.params.required_movement_m.get(),
                allowance_s: self.params.movement_time_allowance_s.get(),
            });
        }
        Ok(())
    }
}
