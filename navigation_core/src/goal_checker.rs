//! Whether a base has reached its goal, as Nav2 1.5.2's simple goal checker
//! judges it with `stateful` true and `symmetric_yaw_tolerance` false
//! (nav2_controller plugins/simple_goal_checker.cpp: `isGoalReached`,
//! `isGoalXYReached` and `reset`). The goal is the pruned plan's goal, the
//! whole plan's last pose, as the controller server hands its goal checker
//! the plan's end pose (nav2_controller src/controller_server.cpp:699,
//! 962-973).
//!
//! Each judgment first measures the plan ahead of the base: a plan longer than
//! `path_length_tolerance_m` leaves the position unreached. Nav2's default of
//! 1 m (plugins/simple_goal_checker.cpp:92-93) is the value to keep: a base
//! within `xy_goal_tolerance_m` of a goal its plan still leads away from, as
//! on a path that passes near its goal before its end, has not reached it,
//! and a plan ahead no longer than the path handler's `prune_distance_m` ends
//! the pruned plan at the goal, whose heading the controller turns the base
//! to once the position is reached. Within that length, the position counts
//! as reached once the base stands within `xy_goal_tolerance_m` of the goal,
//! and from then on stays reached without that distance check for as long as
//! the plan holds, or, with a buffer above 0, until the base strays more than
//! `xy_goal_tolerance_m + xy_goal_tolerance_buffer_m` from the goal. The goal
//! is reached once its position is and the heading lies within
//! `yaw_goal_tolerance_rad` of the goal's.
//!
//! Each plan holds its own goal checker, which judges the position afresh:
//! [`GlobalPlan::new`](crate::path_handler::GlobalPlan::new) makes it with the
//! plan, as Nav2's controller server resets its goal checker as it takes a new
//! plan (`ControllerServer::setPlannerPath`, src/controller_server.cpp:697-701),
//! and [`GlobalPlan::goal_checker`](crate::path_handler::GlobalPlan::goal_checker)
//! gives it.

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::planar_pose::PlanarPose;

use crate::path_handler::PrunedPlan;
use crate::pose::{path_length, shortest_angular_distance};

/// The goal checker's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GoalCheckerParams {
    /// How near the goal, in meters, the base must stand
    /// (`xy_goal_tolerance`).
    pub xy_goal_tolerance_m: NonNegativeFinite,
    /// How much farther than the tolerance, in meters, a base that reached
    /// the position may stray before it must reach it again
    /// (`xy_goal_tolerance_buffer`); with 0, the distance check stays off from
    /// the position's first reach until a reset.
    pub xy_goal_tolerance_buffer_m: NonNegativeFinite,
    /// How near the goal's heading, in radians, the base's must lie
    /// (`yaw_goal_tolerance`).
    pub yaw_goal_tolerance_rad: NonNegativeFinite,
    /// The longest plan ahead of the base, in meters, at which the position
    /// can count as reached (`path_length_tolerance`).
    pub path_length_tolerance_m: NonNegativeFinite,
}

/// The goal checker of a plan and whether it still judges the position.
#[derive(Debug, PartialEq)]
pub struct GoalChecker {
    params: GoalCheckerParams,
    tolerance_sq: f64,
    reset_tolerance_sq: f64,
    check_xy: bool,
}

impl GoalChecker {
    /// The goal checker of a new plan: it judges the position.
    pub(crate) fn new(params: GoalCheckerParams) -> Self {
        let tolerance = params.xy_goal_tolerance_m.get();
        let reset_tolerance = tolerance + params.xy_goal_tolerance_buffer_m.get();
        Self {
            params,
            tolerance_sq: tolerance * tolerance,
            reset_tolerance_sq: reset_tolerance * reset_tolerance,
            check_xy: true,
        }
    }

    /// `isGoalXYReached`: whether the base at `pose`, with `plan` ahead of
    /// it, has reached the position of the plan's goal.
    pub fn is_goal_xy_reached(&mut self, pose: PlanarPose, plan: &PrunedPlan) -> bool {
        if path_length(plan.poses()) > self.params.path_length_tolerance_m.get() {
            return false;
        }
        let goal = plan.goal();
        let dx = pose.x() - goal.x();
        let dy = pose.y() - goal.y();
        let distance_sq = dx * dx + dy * dy;
        if self.check_xy {
            if distance_sq > self.tolerance_sq {
                return false;
            }
            self.check_xy = false;
            return true;
        }
        if self.params.xy_goal_tolerance_buffer_m.get() > 0.0
            && distance_sq > self.reset_tolerance_sq
        {
            self.check_xy = true;
            return false;
        }
        true
    }

    /// `isGoalReached`: whether the base at `pose`, with `plan` ahead of it,
    /// has reached the plan's goal, its position and its heading.
    pub fn is_goal_reached(&mut self, pose: PlanarPose, plan: &PrunedPlan) -> bool {
        if !self.is_goal_xy_reached(pose, plan) {
            return false;
        }
        shortest_angular_distance(pose.yaw(), plan.goal().yaw()).abs()
            <= self.params.yaw_goal_tolerance_rad.get()
    }
}
