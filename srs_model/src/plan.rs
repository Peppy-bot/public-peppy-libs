//! The Cartesian plan of an arm move: the limits a plan is sized against, the
//! choice between the line tier (`crate::line`) and the servo tier
//! ([`chain_kinematics::plan_servo`]), and why no plan reached the goal.

use std::time::Duration;

use chain_kinematics::{
    CORRIDOR_M, EeCaps, JointSmoothing, ServoLimits, ServoPlan, ServoTolerances, ToleranceError,
    joints, plan_servo,
};
use control_core::filters::ButterworthFilter;
use nalgebra::Isometry3;

use crate::line::{LinePlan, plan_line};
use crate::{ARM_DOF, Arm, JointVec};

/// The motion limits a Cartesian plan sizes and validates against: the joint
/// velocity and acceleration limits, the end-effector speed caps, the control
/// period a plan is checked tick by tick at and the servo rollout steps at, and
/// the servo's per-joint command smoother, built for that period.
pub struct PlanLimits {
    pub max_joint_velocity_rad_s: JointVec,
    pub max_joint_acceleration_rad_s2: JointVec,
    pub ee: EeCaps,
    pub control_period: Duration,
    pub smoothing: ButterworthFilter,
}

impl PlanLimits {
    /// The plan's budgets as one step of the servo law at the control period,
    /// so the rollout that plans a move and the step that follows a streamed
    /// pose run under the same numbers. `tolerance` is the caller's arrival
    /// slack, which decides convergence alone.
    pub fn servo_at(&self, tolerance: ServoTolerances) -> ServoLimits<ARM_DOF> {
        ServoLimits {
            max_joint_velocity: self.max_joint_velocity_rad_s,
            max_joint_acceleration: self.max_joint_acceleration_rad_s2,
            ee: self.ee,
            tolerances: tolerance,
            dt_s: self.control_period.as_secs_f64(),
        }
    }
}

/// Why no plan reached the goal pose: named where the decision is taken, so
/// the operator is told which limit refused the move.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanRejection {
    /// No line was usable (none tracks the pose continuously, or the ones that
    /// do overrun the duration this request allows) and the guarded servo's
    /// rollout did not converge within its ceiling either.
    Unreachable,
    /// No line was usable, and the guarded servo that would take over cannot
    /// arrive within the slack asked for.
    ServoCannotArrive(ToleranceError),
    /// No line was usable, and the guarded servo's path stands farther off the
    /// straight line than [`CORRIDOR_M`] allows, at its widest by this much.
    ServoLeavesCorridor { excursion_m: f64 },
}

impl std::fmt::Display for PlanRejection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unreachable => write!(
                f,
                "no line plan fits the limits and the servo rollout stalls"
            ),
            Self::ServoCannotArrive(refusal) => write!(
                f,
                "no line plan fits the limits, and the guarded servo that would take \
                 over cannot arrive that tightly ({refusal})"
            ),
            Self::ServoLeavesCorridor { excursion_m } => write!(
                f,
                "no line plan fits the limits, and the guarded servo's path would stand \
                 {:.0} mm off the line, past the {:.0} mm corridor",
                excursion_m * 1000.0,
                CORRIDOR_M * 1000.0
            ),
        }
    }
}

/// How an accepted Cartesian move executes, decided by [`plan_cartesian`].
#[derive(Debug, Clone, PartialEq)]
pub enum CartesianPlan {
    /// The straight line is continuously trackable: track it over its
    /// duration, swiveling the elbow the way the plan walked.
    Line(LinePlan),
    /// No line exists: every continuous tracking demands a branch jump, is
    /// untrackably slow, or leaves reach mid-path. Play the guarded servo law's
    /// path over its duration instead: its offline rollout, which arrives
    /// within the caller's slack.
    Servo(ServoPlan),
}

impl CartesianPlan {
    /// How long the move takes.
    pub fn duration(&self) -> Duration {
        match self {
            Self::Line(line) => line.duration,
            Self::Servo(servo) => servo.duration,
        }
    }

    /// `count` joint configurations spaced evenly along the path from its
    /// start to its goal, both included, for a check that walks the path
    /// before it plays. One sample is the start.
    pub fn joint_samples(&self, count: usize) -> Vec<JointVec> {
        let path = match self {
            Self::Line(line) => &line.path,
            Self::Servo(servo) => &servo.path,
        };
        let steps = count.saturating_sub(1).max(1) as f64;
        (0..count)
            .map(|i| joints::<ARM_DOF>(&path.at_fraction(i as f64 / steps)))
            .collect()
    }
}

/// Plan a Cartesian move from the end effector's pose at `seed` to `end`,
/// preferring the quietest execution that works: the straight line, its elbow
/// held or swiveled (`plan_line`), and only when no line is usable the guarded
/// servo ([`plan_servo`]), whose damped law crosses the singular surfaces a
/// discrete IK walk cannot. `tolerance` is the slack the servo would arrive
/// within, or why none can, which refuses the move once no line fits. A
/// refusal names the limit that refused the move.
pub fn plan_cartesian(
    arm: &Arm,
    end: &Isometry3<f64>,
    seed: JointVec,
    limits: &PlanLimits,
    requested: Duration,
    tolerance: Result<ServoTolerances, ToleranceError>,
) -> Result<CartesianPlan, PlanRejection> {
    if let Some(line) = plan_line(arm, end, seed, limits, requested) {
        return Ok(CartesianPlan::Line(line));
    }
    let tolerance = tolerance.map_err(PlanRejection::ServoCannotArrive)?;
    let servo = plan_servo(
        arm.chain(),
        end,
        seed,
        &limits.servo_at(tolerance),
        JointSmoothing::new(limits.smoothing),
        requested,
    )
    .ok_or(PlanRejection::Unreachable)?;
    within_corridor(servo).map(CartesianPlan::Servo)
}

/// Admit a servo plan whose path stays within [`CORRIDOR_M`] of its line.
fn within_corridor(servo: ServoPlan) -> Result<ServoPlan, PlanRejection> {
    if servo.excursion_m > CORRIDOR_M {
        return Err(PlanRejection::ServoLeavesCorridor {
            excursion_m: servo.excursion_m,
        });
    }
    Ok(servo)
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_core::path_timing::TimedPath;

    /// A servo plan of a nominal two-dimensional path, standing `excursion_m`
    /// off its line at the widest.
    fn servo_standing_off_by(excursion_m: f64) -> ServoPlan {
        ServoPlan {
            duration: Duration::from_secs(1),
            path: TimedPath::new(&[vec![0.0, 0.0], vec![0.5, 1.0]], &[1.0, 1.0], &[1.0, 1.0]),
            excursion_m,
        }
    }

    #[test]
    fn joint_samples_walk_the_path_from_its_start_to_its_goal() {
        let path = TimedPath::new(
            &[vec![0.0; ARM_DOF + 1], {
                let mut end = vec![1.0; ARM_DOF];
                end.push(1.0);
                end
            }],
            &[1.0; ARM_DOF + 1],
            &[1.0; ARM_DOF + 1],
        );
        let plan = CartesianPlan::Servo(ServoPlan {
            duration: Duration::from_secs(1),
            path,
            excursion_m: 0.0,
        });
        let samples = plan.joint_samples(5);
        assert_eq!(samples.len(), 5);
        assert_eq!(samples[0], [0.0; ARM_DOF]);
        assert_eq!(samples[4], [1.0; ARM_DOF]);
        assert!(
            samples.windows(2).all(|w| w[0][0] < w[1][0]),
            "monotone along the path"
        );
        assert_eq!(plan.joint_samples(1), vec![[0.0; ARM_DOF]]);
    }

    #[test]
    fn a_servo_plan_inside_the_corridor_is_admitted_and_one_past_it_refused_by_its_excursion() {
        let inside = servo_standing_off_by(CORRIDOR_M);
        assert_eq!(within_corridor(inside.clone()), Ok(inside));
        let excursion_m = CORRIDOR_M + 0.011;
        let refused = within_corridor(servo_standing_off_by(excursion_m));
        assert_eq!(
            refused,
            Err(PlanRejection::ServoLeavesCorridor { excursion_m })
        );
        assert_eq!(
            refused.unwrap_err().to_string(),
            "no line plan fits the limits, and the guarded servo's path would stand 61 mm off \
             the line, past the 50 mm corridor"
        );
    }
}
