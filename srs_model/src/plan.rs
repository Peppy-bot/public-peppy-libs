//! The Cartesian plan of an arm move: the limits a plan is sized against, the
//! choice between the line tier (`crate::line`), the servo tier
//! ([`chain_kinematics::plan_servo`]) and the joint tier (a joint-space path to
//! the exact solver's solution), and why no plan reached the goal.

use std::time::Duration;

use chain_kinematics::{
    EeCaps, JointSmoothing, ServoLimits, ServoPlan, ServoTolerances, ToleranceError, plan_servo,
    timed_with_ee,
};
use control_core::filters::ButterworthFilter;
use control_core::timing::duration_at_least;
use nalgebra::Isometry3;

use crate::line::{LinePlan, plan_line};
use crate::{ARM_DOF, Arm, ArmAnglePolicy, JointVec};

/// The largest step of any joint between two configurations of a joint-tier
/// path (rad): the end effector's travel is timed chord by chord, so its
/// chords stay close to its arc.
const JOINT_PATH_STEP_RAD: f64 = 0.01;

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
    /// do overrun the duration this request allows), the guarded servo's
    /// rollout did not converge within its ceiling, and the exact solver has
    /// no in-limit solution within the position slack either.
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
                "no line plan fits the limits, the servo rollout stalls, and no joint \
                 configuration within the limits reaches the pose"
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
    /// Neither a line nor the servo reaches the pose, and the exact solver
    /// does: a straight joint-space path from the seed to its solution, timed
    /// and played as the servo's path is. The end effector does not travel
    /// on a straight line, and it stops within the caller's position slack of
    /// the pose, in its orientation.
    Joint(ServoPlan),
}

impl CartesianPlan {
    /// How long the move takes.
    pub fn duration(&self) -> Duration {
        match self {
            Self::Line(line) => line.duration,
            Self::Servo(servo) | Self::Joint(servo) => servo.duration,
        }
    }
}

/// Plan a Cartesian move from the end effector's pose at `seed` to `end`,
/// preferring the quietest execution that works: the straight line, its elbow
/// held or swiveled (`plan_line`), and only when no line is usable the guarded
/// servo ([`plan_servo`]), whose damped law crosses the singular surfaces a
/// discrete IK walk cannot, and only when the servo stalls too the joint tier
/// ([`plan_joint`]), which reaches every pose the exact solver reaches.
/// `tolerance` is the slack the servo and the joint tier arrive within, or why
/// none can, which refuses the move once no line fits. A refusal names the
/// limit that refused the move.
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
    if let Some(servo) = plan_servo(
        arm.chain(),
        end,
        seed,
        &limits.servo_at(tolerance),
        JointSmoothing::new(limits.smoothing),
        requested,
    ) {
        return Ok(CartesianPlan::Servo(servo));
    }
    plan_joint(arm, end, seed, limits, requested, tolerance)
        .map(CartesianPlan::Joint)
        .ok_or(PlanRejection::Unreachable)
}

/// The joint-tier plan of a move from `seed` to `end`: the exact solver's
/// in-limit solution within the position slack of `tolerance`
/// ([`Arm::solve_ik_within`], at the seed's arm angle or the nearest feasible
/// one), reached on a straight joint-space path whose configurations stand at
/// most [`JOINT_PATH_STEP_RAD`] apart in every joint, timed with the end
/// effector under `limits` and played over the longer of `requested` and the
/// path's own time from rest to rest; `None` when no solution exists. A path
/// between two configurations inside the joint limits stays inside them.
fn plan_joint(
    arm: &Arm,
    end: &Isometry3<f64>,
    seed: JointVec,
    limits: &PlanLimits,
    requested: Duration,
    tolerance: ServoTolerances,
) -> Option<ServoPlan> {
    let goal = arm
        .solve_ik_within(
            &arm.base_pose(end),
            tolerance.position_m(),
            ArmAnglePolicy::FromSeed,
            &seed,
        )?
        .q;
    let configs = joint_path(&seed, &goal);
    let path = timed_with_ee(
        arm.chain(),
        &configs,
        &limits.max_joint_velocity_rad_s,
        &limits.max_joint_acceleration_rad_s2,
        &limits.ee,
    );
    let duration = duration_at_least(path.duration_s(), requested);
    Some(ServoPlan { duration, path })
}

/// The configurations of the straight joint-space path from `from` to `to`:
/// both ends, and between them the fewest configurations equally spaced that
/// stand at most [`JOINT_PATH_STEP_RAD`] apart in every joint.
fn joint_path(from: &JointVec, to: &JointVec) -> Vec<JointVec> {
    let widest = from
        .iter()
        .zip(to)
        .map(|(start, end)| (end - start).abs())
        .fold(0.0, f64::max);
    let steps = (widest / JOINT_PATH_STEP_RAD).ceil().max(1.0) as usize;
    (0..=steps)
        .map(|k| {
            let fraction = k as f64 / steps as f64;
            std::array::from_fn(|j| from[j] + fraction * (to[j] - from[j]))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A joint path runs from its start to its end in steps of at most
    /// [`JOINT_PATH_STEP_RAD`] in every joint, as few as that allows, and a
    /// path that does not move is its two ends.
    #[test]
    fn a_joint_path_steps_at_most_its_step_in_every_joint() {
        let from = [0.0, -0.5, 0.2, 1.0, 0.0, 0.1, -0.3];
        let to = [0.105, -0.5, 0.15, 1.0, -0.02, 0.1, -0.3];
        let path = joint_path(&from, &to);
        assert_eq!(path.len(), 12, "0.105 rad in steps of 0.01 takes 11 steps");
        assert_eq!(path[0], from);
        assert_eq!(path[path.len() - 1], to);
        for pair in path.windows(2) {
            for (start, end) in pair[0].iter().zip(&pair[1]) {
                assert!((end - start).abs() <= JOINT_PATH_STEP_RAD + 1e-12);
            }
        }
        assert_eq!(joint_path(&from, &from), [from, from]);
    }
}
