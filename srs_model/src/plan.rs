//! The Cartesian plan of an arm move: the limits a plan is sized against, the
//! choice between the line tier (`crate::line`) and the servo tier
//! ([`chain_kinematics::plan_servo`]), and why no plan reached the goal.

use std::time::Duration;

use chain_kinematics::{
    EeCaps, JointSmoothing, ServoLimits, ServoPlan, ServoTolerances, ToleranceError, joints,
    plan_servo,
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

    /// The joint configurations the path passes through every `step` of its
    /// length, from its start to its goal, each with the fraction of the
    /// plan's duration it is reached at, for a check that walks the path
    /// before it plays (see [`TimedPath::probes_every`]).
    pub fn probes_every(&self, step: f64) -> Vec<(f64, JointVec)> {
        let path = match self {
            Self::Line(line) => &line.path,
            Self::Servo(servo) => &servo.path,
        };
        path.probes_every(step)
            .into_iter()
            .map(|(fraction, q)| (fraction, joints::<ARM_DOF>(&q)))
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
    plan_servo(
        arm.chain(),
        end,
        seed,
        &limits.servo_at(tolerance),
        JointSmoothing::new(limits.smoothing),
        requested,
    )
    .map(CartesianPlan::Servo)
    .ok_or(PlanRejection::Unreachable)
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_core::path_timing::TimedPath;
    use hegel::TestCase;
    use hegel::generators as gs;

    // A plan's probes are its path's, with the end effector's progress
    // dropped: the first on the start, the last on the goal, no joint
    // changing by more than the step between neighbours, and each where
    // the path stands at the fraction it names.
    #[hegel::test]
    fn probes_walk_a_plan_from_its_start_to_its_goal(tc: TestCase) {
        let [start, goal]: [JointVec; 2] = tc.draw(gs::arrays(gs::arrays(
            gs::floats().min_value(-3.0).max_value(3.0),
        )));
        let step: f64 = tc.draw(gs::floats().min_value(0.01).max_value(1.0));
        let waypoint = |q: &JointVec| [q.as_slice(), &[0.0]].concat();
        let path = TimedPath::new(
            &[waypoint(&start), waypoint(&goal)],
            &[1.0; ARM_DOF + 1],
            &[1.0; ARM_DOF + 1],
        );
        let plan = CartesianPlan::Servo(ServoPlan {
            duration: Duration::from_secs(1),
            path: path.clone(),
        });
        let probes = plan.probes_every(step);
        assert_eq!(probes[0], (0.0, start));
        assert_eq!(probes[probes.len() - 1].1, goal);
        for pair in probes.windows(2) {
            let ((f0, q0), (f1, q1)) = (pair[0], pair[1]);
            assert!(f1 >= f0);
            for j in 0..ARM_DOF {
                assert!((q1[j] - q0[j]).abs() <= step * (1.0 + 1e-6));
            }
        }
        for (fraction, q) in &probes {
            let at = joints::<ARM_DOF>(&path.at_fraction(*fraction));
            for j in 0..ARM_DOF {
                assert!(
                    (q[j] - at[j]).abs() <= 1e-6,
                    "{q:?} probed where the path stands at {at:?}"
                );
            }
        }
    }
}
