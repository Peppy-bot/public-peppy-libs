//! A guarded Cartesian move: the damped resolved-rate law run against a
//! reference that walks toward the goal, leashed whenever the chain falls
//! behind.
//!
//! A discrete inverse-kinematics walk cannot cross the singular surface between
//! two solution branches. The damped law can: the damping bounds the joint rates
//! while the task error carries the chain across, deviating from the straight
//! line only where the geometry forces it and re-converging beyond. Leashing the
//! reference is what keeps that honest - the goal never runs away from an arm
//! that is grinding through a wall.
//!
//! The same law rolls out offline ([`rollout`]) before a move is accepted, which
//! is the reachability check: a goal that does not converge within the caller's
//! budget is refused, and one that does comes back as the configurations the
//! law commanded, which [`timed_with_ee`] times and the caller plays, each
//! corner rounded on an arc within the two steps it joins. The law is
//! deterministic, so the configurations that were validated are the ones that
//! run.
//!
//! Tolerances are the caller's. The output smoother is whatever bounds the
//! command's jerk: the [`Smoother`] a caller passes to [`plan_servo`], such as
//! [`JointSmoothing`], one `control_core` Butterworth per joint at MoveIt
//! Servo's default cutoff for the control period.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use control_core::chase::chase_step;
use control_core::filters::{ButterworthFilter, FilterError};
use control_core::path_timing::{TimedPath, TimedTrajectory};
use control_core::timing::{MotionClock, duration_at_least};
use nalgebra::{Isometry3, Vector3, Vector6};

use crate::line::interpolate;
use crate::rate::{DEFAULT_DLS_LAMBDA, pose_error, rate_step};
use crate::{Chain, Limit, clamp_into, pose_miss};

/// The end-effector speed budget a Cartesian step runs under.
#[derive(Debug, Clone, Copy)]
pub struct EeCaps {
    pub linear_m_s: f64,
    pub angular_rad_s: f64,
}

impl EeCaps {
    /// The time the hand takes from `a` to `b` at these caps: the longer of
    /// its travel at the linear cap and its turn at the angular cap.
    pub fn time_between(&self, a: &Isometry3<f64>, b: &Isometry3<f64>) -> f64 {
        let miss = pose_miss(a, b);
        (miss.position_m / self.linear_m_s).max(miss.orientation_rad / self.angular_rad_s)
    }
}

/// How close counts as arrived: a finite positive distance and angle. Arrival
/// is all it decides: a caller's slack changes when a move is done, never how
/// it is tracked.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoTolerances {
    position_m: f64,
    orientation_rad: f64,
}

impl ServoTolerances {
    /// The arrival slack a move is judged against, or a refusal of a value
    /// that is not a slack.
    pub fn new(position_m: f64, orientation_rad: f64) -> Result<Self, ToleranceError> {
        if !(position_m.is_finite() && position_m > 0.0) {
            return Err(ToleranceError::Position(position_m));
        }
        if !(orientation_rad.is_finite() && orientation_rad > 0.0) {
            return Err(ToleranceError::Orientation(orientation_rad));
        }
        Ok(Self {
            position_m,
            orientation_rad,
        })
    }

    pub fn position_m(&self) -> f64 {
        self.position_m
    }

    pub fn orientation_rad(&self) -> f64 {
        self.orientation_rad
    }
}

/// A value that is not an arrival slack. Plain data, so a caller can carry the
/// refusal in a type of its own and stay `Copy`.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum ToleranceError {
    #[error("position tolerance {0} m is not a finite positive distance")]
    Position(f64),
    #[error("orientation tolerance {0} rad is not a finite positive angle")]
    Orientation(f64),
}

/// Everything one servo tick runs under: the per-joint velocity and
/// acceleration budgets, the end-effector speed caps, the arrival tolerances,
/// and the control period.
///
/// Grouped because they are one decision - "how this move is allowed to go" -
/// taken once by the caller and then threaded unchanged through every step, and
/// because a step is easier to get right when its budgets cannot be passed in
/// the wrong order.
#[derive(Debug, Clone, Copy)]
pub struct ServoLimits<const N: usize> {
    pub max_joint_velocity: [f64; N],
    pub max_joint_acceleration: [f64; N],
    pub ee: EeCaps,
    pub tolerances: ServoTolerances,
    pub dt_s: f64,
}

/// Per-joint output smoothing, bounding the jerk of the commanded step: a
/// caller passes whatever its controller already uses, so the servo's output
/// is smoothed exactly as the rest of that controller's commands are.
pub trait Smoother<const N: usize> {
    fn smooth(&mut self, q: &[f64; N]) -> [f64; N];
}

/// A smoother that does nothing, for a caller that bounds jerk elsewhere or not
/// at all.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoSmoothing;

impl<const N: usize> Smoother<N> for NoSmoothing {
    fn smooth(&mut self, q: &[f64; N]) -> [f64; N] {
        *q
    }
}

/// One damped resolved-rate step toward a pose target, given the end-effector
/// pose already computed at `q`.
///
/// The task error is capped before it is resolved: position at the speed budget,
/// orientation at the slew budget. A target metres away therefore produces the
/// same bounded step as one millimetres away, which is what lets one law serve
/// both a planned move and a live stream.
///
/// A joint heading toward a limit moves no faster than lets it come to rest on
/// that limit decelerating within its acceleration budget, so it arrives there
/// braking, never stopped by the limit itself.
///
/// It always returns a configuration. An unreachable target moves toward the
/// closest pose this step can find and saturates there: a teleoperator pushing
/// past the edge feels a wall.
pub fn rate_step_toward<const N: usize>(
    chain: &Chain<N>,
    q: &[f64; N],
    ee: &Isometry3<f64>,
    target: &Isometry3<f64>,
    limits: &ServoLimits<N>,
) -> [f64; N] {
    let (caps, dt_s) = (limits.ee, limits.dt_s);
    let (dp, dw) = pose_error(ee, target);
    let dp = if dp.norm() > 0.0 {
        dp * (caps.linear_m_s * dt_s / dp.norm()).min(1.0)
    } else {
        Vector3::zeros()
    };
    let dw = if dw.norm() > 0.0 {
        dw * (caps.angular_rad_s * dt_s / dw.norm()).min(1.0)
    } else {
        Vector3::zeros()
    };
    rate_step(
        chain,
        q,
        &Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z),
        &stopping_limits(&chain.limits(), q, &limits.max_joint_acceleration, dt_s),
        &limits.max_joint_velocity,
        dt_s,
        DEFAULT_DLS_LAMBDA,
    )
}

/// The positions each joint of `q` may reach within `dt_s` and still stop on
/// the limit it heads toward at its acceleration budget: at `room` from a
/// limit, a joint moves toward it this tick no faster than the speed `v` whose
/// travel this tick and braking distance after it, `v dt + v²/2a`, fit the
/// room, so it comes to rest exactly on the limit decelerating at most at
/// `a`. A joint outside its limits counts from the nearer one, and an infinite
/// budget brakes nothing.
fn stopping_limits<const N: usize>(
    limits: &[Limit; N],
    q: &[f64; N],
    max_joint_acceleration: &[f64; N],
    dt_s: f64,
) -> [Limit; N] {
    std::array::from_fn(|i| {
        let Limit { lo, hi } = limits[i];
        let at = limits[i].clamp(q[i]);
        let a = max_joint_acceleration[i];
        let reach = |room: f64| {
            if !a.is_finite() {
                return f64::INFINITY;
            }
            if room <= 0.0 {
                return 0.0;
            }
            a * (dt_s * dt_s + 2.0 * room / a).sqrt().mul_add(1.0, -dt_s) * dt_s
        };
        Limit {
            lo: (at - reach(at - lo)).max(lo),
            hi: (at + reach(hi - at)).min(hi),
        }
    })
}

/// One servo move's state: where the reference has reached along the line, and
/// the output smoother. The joint state stays with the caller, which each tick
/// advances.
pub struct ServoState<const N: usize, S: Smoother<N>> {
    start: Isometry3<f64>,
    end: Isometry3<f64>,
    /// Reference progress along the line, 0..=1.
    reference_s: f64,
    /// The reference stops while the chain is farther than this from it, so a
    /// wall is ground through with the reference waiting at its edge.
    leash_m: f64,
    smoother: S,
}

/// One tick's outcome.
#[derive(Debug, Clone, Copy)]
pub enum ServoStep<const N: usize> {
    /// Advanced: the new joint setpoint to command.
    Stepped([f64; N]),
    /// Reached the goal pose within tolerance.
    Converged([f64; N]),
}

impl<const N: usize, S: Smoother<N>> ServoState<N, S> {
    pub fn new(start: Isometry3<f64>, end: Isometry3<f64>, leash_m: f64, smoother: S) -> Self {
        Self {
            start,
            end,
            reference_s: 0.0,
            leash_m,
            smoother,
        }
    }

    /// Advance one tick of `dt_s`: walk the reference (leashed to the chain),
    /// take one damped resolved-rate step toward it, and report whether the goal
    /// pose is reached.
    pub fn step(
        &mut self,
        chain: &Chain<N>,
        q: &[f64; N],
        limits: &ServoLimits<N>,
    ) -> ServoStep<N> {
        let (caps, tol, dt_s) = (limits.ee, limits.tolerances, limits.dt_s);
        let ee = chain.world_ee_pose(q);

        let goal = pose_miss(&ee, &self.end);
        if self.reference_s >= 1.0
            && goal.position_m < tol.position_m()
            && goal.orientation_rad < tol.orientation_rad()
        {
            return ServoStep::Converged(*q);
        }

        // Walk the reference at the speed cap while the chain holds the leash;
        // a line of no length is walked in the first tick.
        let line_len = (self.end.translation.vector - self.start.translation.vector).norm();
        let reference = interpolate(&self.start, &self.end, self.reference_s);
        let ref_pos_err = (reference.translation.vector - ee.translation.vector).norm();
        if ref_pos_err < self.leash_m {
            self.reference_s = (self.reference_s + caps.linear_m_s * dt_s / line_len).min(1.0);
        }
        let reference = interpolate(&self.start, &self.end, self.reference_s);

        let next = rate_step_toward(chain, q, &ee, &reference, limits);
        // Smooth, then re-clamp to the velocity limit: a smoother can overshoot
        // its input, so without this the smoothed command could exceed the limit
        // the step enforced. The clamp is the final stage, so the commanded
        // velocity holds regardless of the filter transient.
        let smoothed = self.smoother.smooth(&next);
        let chased = chase_step(q, &smoothed, &limits.max_joint_velocity, dt_s);
        ServoStep::Stepped(clamp_into(&chain.limits(), &chased))
    }
}

/// Roll the servo law out offline at the control period per step: the plan-time
/// proof that it reaches the pose, and the path it takes there, every
/// configuration the law commands from `seed` to the converged one, one control
/// period apart; or `None` when it has not converged within `budget_s`.
///
/// Takes the state by value: a rollout spends it, walking the reference to the
/// goal, and a spent one restarted mid-line would answer for a move nobody asked
/// for. The runtime builds its own.
pub fn rollout<const N: usize, S: Smoother<N>>(
    chain: &Chain<N>,
    mut state: ServoState<N, S>,
    seed: [f64; N],
    limits: &ServoLimits<N>,
    budget_s: f64,
) -> Option<Vec<[f64; N]>> {
    let mut q = seed;
    let mut configs = vec![seed];
    let steps = (budget_s / limits.dt_s).ceil() as usize;
    for _ in 0..steps {
        match state.step(chain, &q, limits) {
            ServoStep::Stepped(next) => {
                q = next;
                configs.push(next);
            }
            ServoStep::Converged(_) => return Some(configs),
        }
    }
    None
}

/// The timed path through `configs`, the configurations a Cartesian move
/// commands, under the joints' velocity and acceleration limits and the end
/// effector's `ee` caps. Alongside the joints runs the end effector's progress:
/// the time its travel from one configuration to the next takes at the caps,
/// summed along the path, which runs no faster than real time, so each step
/// between two configurations takes at least the time its chord needs at the
/// caps. The path starts at rest on the first configuration and ends at rest
/// on the last.
pub fn timed_with_ee<const N: usize>(
    chain: &Chain<N>,
    configs: &[[f64; N]],
    max_joint_velocity: &[f64; N],
    max_joint_acceleration: &[f64; N],
    ee: &EeCaps,
) -> TimedPath {
    assert!(
        !configs.is_empty(),
        "a path has at least its first configuration"
    );
    let poses: Vec<Isometry3<f64>> = configs.iter().map(|q| chain.world_ee_pose(q)).collect();
    let ee_time = std::iter::once(0.0).chain(poses.windows(2).scan(0.0, |t, w| {
        *t += ee.time_between(&w[0], &w[1]);
        Some(*t)
    }));
    let with_ee = |joints: &[f64; N], progress: f64| -> Vec<f64> {
        joints
            .iter()
            .copied()
            .chain(std::iter::once(progress))
            .collect()
    };
    let waypoints: Vec<Vec<f64>> = configs
        .iter()
        .zip(ee_time)
        .map(|(q, t)| with_ee(q, t))
        .collect();
    TimedPath::new(
        &waypoints,
        &with_ee(max_joint_velocity, 1.0),
        &with_ee(max_joint_acceleration, f64::INFINITY),
    )
}

/// The configuration in a sample of a path from [`timed_with_ee`]: its first
/// `N` components.
pub fn joints<const N: usize>(sample: &[f64]) -> [f64; N] {
    sample[..N]
        .try_into()
        .expect("a sample of a path timed with the end effector carries a configuration")
}

/// The end effector's progress in a sample of a path from [`timed_with_ee`]:
/// its last component, the seconds of its travel at the caps.
pub fn ee_progress(sample: &[f64]) -> f64 {
    sample[sample.len() - 1]
}

/// A path from [`timed_with_ee`] played over a duration its planner sized, no
/// shorter than the path's own: at time `now`, the configuration at the elapsed
/// fraction of that duration along it.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoTrajectory<const N: usize> {
    timed: TimedTrajectory,
}

impl<const N: usize> ServoTrajectory<N> {
    /// `path` over `duration`, starting at `now`.
    pub fn new(path: TimedPath, duration: Duration, now: Instant) -> Self {
        assert_eq!(
            path.dimension(),
            N + 1,
            "a servo path is a configuration and the end effector's progress"
        );
        Self {
            timed: TimedTrajectory::new(path, duration, now),
        }
    }

    pub fn clock(&self) -> &MotionClock {
        self.timed.clock()
    }

    pub fn sample(&self, now: Instant) -> [f64; N] {
        joints(&self.timed.sample(now))
    }

    /// The configurations the trajectory plays every `step` of its path's
    /// length, from its start to its end, each with the instant it plays
    /// them (see [`TimedPath::probes_every`]).
    pub fn probes_every(&self, step: f64) -> Vec<(Instant, [f64; N])> {
        self.timed
            .probes_every(step)
            .into_iter()
            .map(|(at, sample)| (at, joints(&sample)))
            .collect()
    }
}

/// MoveIt Servo's default first-order low-pass coefficient
/// (`low_pass_filter_coeff`), whose -3 dB cutoff at a control period `Ts` is
/// `atan(1 / coeff) / (pi Ts)`.
const MOVEIT_LOW_PASS_COEFF: f64 = 1.5;

/// The per-joint command smoother for one control `period`: a second-order
/// Butterworth at MoveIt Servo's default cutoff for that period, so it smooths
/// at least as much as MoveIt's first-order filter; or a refusal when the
/// period cannot carry the cutoff (Nyquist at `0.5 / period` must sit above
/// it). Built once at setup, where a bad period can still be refused.
pub fn command_smoothing(period: Duration) -> Result<ButterworthFilter, FilterError> {
    let period_s = period.as_secs_f64();
    let cutoff_hz = (1.0 / MOVEIT_LOW_PASS_COEFF).atan() / (PI * period_s);
    ButterworthFilter::from_cutoff(cutoff_hz, period_s)
}

/// One Butterworth filter per joint, bounding the jerk of the servo's command.
/// Run in the plan-time rollout, so the path a servo move plays is already
/// smoothed.
#[derive(Clone, Copy)]
pub struct JointSmoothing<const N: usize>([ButterworthFilter; N]);

impl<const N: usize> JointSmoothing<N> {
    /// `filter` on every joint.
    pub fn new(filter: ButterworthFilter) -> Self {
        Self([filter; N])
    }

    /// Forget every filter's state, so the next step seeds them on itself.
    pub fn reset(&mut self) {
        for filter in &mut self.0 {
            filter.reset();
        }
    }
}

impl<const N: usize> Smoother<N> for JointSmoothing<N> {
    fn smooth(&mut self, q: &[f64; N]) -> [f64; N] {
        std::array::from_fn(|i| self.0[i].filter(q[i]))
    }
}

/// The reference stops walking while the chain is farther than this from it,
/// so a wall crossing is ground through with the reference waiting at its
/// edge. Bespoke to the leashed-reference law; MoveIt Servo has no reference
/// to leash.
pub const LEASH_M: f64 = 0.05;

/// The longest the law may take to converge at full speed. The plan-time
/// rollout runs at most this long; a goal that has not converged by then is
/// taken as unreachable.
pub const MAX_SERVO_S: f64 = 30.0;

/// A servo move the law reaches: its path, timed with the end effector (see
/// [`timed_with_ee`]), and the duration it plays over.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoPlan {
    pub duration: Duration,
    pub path: TimedPath,
}

/// The servo plan for a move from the end effector's pose at `seed` to `end`,
/// under `limits` and behind `smoother`: the law's rollout, which must converge
/// within [`MAX_SERVO_S`] with the reference leashed by [`LEASH_M`], timed with
/// the end effector and played over the longer of `requested` and the path's
/// own time from rest to rest; `None` when the law does not converge.
pub fn plan_servo<const N: usize, S: Smoother<N>>(
    chain: &Chain<N>,
    end: &Isometry3<f64>,
    seed: [f64; N],
    limits: &ServoLimits<N>,
    smoother: S,
    requested: Duration,
) -> Option<ServoPlan> {
    let state = ServoState::new(chain.world_ee_pose(&seed), *end, LEASH_M, smoother);
    let configs = rollout(chain, state, seed, limits, MAX_SERVO_S)?;
    let path = timed_with_ee(
        chain,
        &configs,
        &limits.max_joint_velocity,
        &limits.max_joint_acceleration,
        &limits.ee,
    );
    let duration = duration_at_least(path.duration_s(), requested);
    Some(ServoPlan { duration, path })
}
