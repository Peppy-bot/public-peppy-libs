//! The line tier of a Cartesian move: the end effector's straight line from
//! its pose to the goal, walked by closed-form IK with the elbow held or
//! swiveled at a constant rate along the line, timed with the end effector
//! within the joints' velocity and acceleration limits and the end-effector
//! caps, and checked tick by tick as the runtime commands it: the runtime's
//! ticks are fresh IK solutions of the exact line, which the hundred-sample
//! walk the timing runs on only approximates, so the line alone is checked on
//! the ticks themselves, where a joint move and a servo path play their timed
//! path, whose bounds hold at every point. [`plan_line`] takes the line when a
//! swivel tracks it continuously at a sane duration, a [`LineCursor`] walks it,
//! at plan time and at run time alike, and a [`LineTrajectory`] plays it.
//! Both walks are `chain_kinematics`' [`walk_line`] with the elbow swiveling.

use std::time::{Duration, Instant};

use chain_kinematics::{
    LineSolver, LineState, WalkEnd, ee_progress, interpolate, limit_margin, timed_with_ee,
    walk_line,
};
use control_core::path_timing::{TimedPath, TimedTrajectory};
use control_core::timing::{MotionClock, duration_at_least, tick_checked_duration, tick_fractions};
use nalgebra::Isometry3;

use crate::plan::PlanLimits;
use crate::{ARM_DOF, Arm, ArmAnglePolicy, JointVec};

/// Path resolution of a line's plan-time walk: the line is IK-solved at this
/// many samples to time the walk and to check that it tracks. Closed-form IK
/// makes a walk a few tens of microseconds.
const LINE_PLAN_SAMPLES: usize = 100;

/// The plan's validation grid spacing in blend parameter: what one plan sample
/// spans. The runtime never advances its IK walk coarser than this (see
/// [`subdivided_blends`]), or a short move could step past geometry the plan
/// validated cell by cell.
const LINE_PLAN_DS: f64 = 1.0 / LINE_PLAN_SAMPLES as f64;

/// The blend samples one runtime tick IK-solves, walking from `prev`
/// (exclusive) to `next` (inclusive) in equal steps no wider than
/// [`LINE_PLAN_DS`]: the single sample `next` for a tick that progressed no
/// more than one plan cell (a zero-progress hold tick included), intermediate
/// samples for a short move whose progress outpaces the plan grid, so the walk
/// executed runs at least as fine as the walk that validated the line.
fn subdivided_blends(prev: f64, next: f64) -> impl Iterator<Item = f64> {
    let span = (next - prev).max(0.0);
    let steps = ((span / LINE_PLAN_DS).ceil() as usize).max(1);
    (1..=steps).map(move |k| prev + span * (k as f64 / steps as f64))
}

/// The most a line swivels its elbow end to end, in radians of arm angle:
/// room to swing clear of a joint limit or a singular posture the held elbow
/// runs into, a third of the way round the redundancy circle, short of the
/// mirror posture on its far side. A swivel spreads evenly over the line, so
/// one plan sample carries `MAX_ELBOW_SWIVEL_RAD * LINE_PLAN_DS` of it at
/// most, and the swivels tried step through the budget in those shares.
const MAX_ELBOW_SWIVEL_RAD: f64 = 2.0;

/// Largest joint step (rad) one plan sample of the line may demand and still be
/// tracked as a straight line. Above it the exact IK solution has jumped to
/// another branch (a genuine discontinuity: sampling finer does not shrink it),
/// which no continuous tracking can execute.
const MAX_LINE_STEP_RAD: f64 = 0.35;

/// A trackable straight line whose duration overruns the request by more than
/// this factor is too slow to keep straight, and the servo, which reconfigures
/// around the near-singular posture forcing the slowdown, takes the move. The
/// cap scales with the move: a long move gets proportional slack and a short
/// one gets no multi-second budget it never needed. Near a singularity the
/// duration diverges, so the factor only arbitrates the mild-graze regime: 2x
/// keeps a line that is merely slowed and sends one that has to crawl to the
/// servo.
const LINE_OVERRUN_FACTOR: f64 = 2.0;

/// Floor on the line-acceptance budget: a line whose duration is within this
/// many seconds is kept straight whatever the request, so a request under half
/// of it, "as fast as possible" among them, still admits a few seconds of
/// near-singular graze as honest motion.
const MIN_LINE_BUDGET_S: f64 = 3.0;

/// The longest duration a straight line may take and still be kept over the
/// servo: [`LINE_OVERRUN_FACTOR`] times the request, floored at
/// [`MIN_LINE_BUDGET_S`].
fn line_duration_cap(requested: Duration) -> f64 {
    (requested.as_secs_f64() * LINE_OVERRUN_FACTOR).max(MIN_LINE_BUDGET_S)
}

/// Why a line's elbow moves as it does, in the order the line tier tries them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Elbow {
    /// Held at the start's arm angle, the quiet default: an ordinary move never
    /// swings joints it does not need.
    Held,
    /// Swiveled toward the widest joint-limit margin, when the held elbow
    /// would carry a joint past its limit mid-move.
    LimitMargin,
    /// Swiveled toward higher manipulability, when no line above tracks at a
    /// sane duration (a singular graze inflating it, or a limit wall the
    /// swivel can dodge).
    Manipulability,
}

/// How far a line swivels its elbow, end to end: the arm angle moves on from
/// the start's by `by_rad` in step with the end effector along the line, so
/// the joints turn no corner, and a held elbow swivels by zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Swivel {
    by_rad: f64,
}

impl Swivel {
    /// The elbow held where it starts.
    pub(crate) const HELD: Self = Self { by_rad: 0.0 };
}

/// The arm's IK along a line with its elbow swiveling by `swivel` over the
/// line: each solve moves the arm angle on from the state before by the share
/// of the swivel its step of the line carries.
struct Swiveling<'a> {
    arm: &'a Arm,
    swivel: Swivel,
}

impl LineSolver<ARM_DOF> for Swiveling<'_> {
    fn solve(&self, pose: &Isometry3<f64>, from: &LineState<ARM_DOF>, to: f64) -> Option<JointVec> {
        let policy = ArmAnglePolicy::Swiveled {
            by_rad: self.swivel.by_rad * (to - from.blend),
        };
        self.arm
            .solve_ik(&self.arm.base_pose(pose), policy, &from.q)
            .map(|solution| solution.q)
    }

    fn pose(&self, q: &JointVec) -> Isometry3<f64> {
        self.arm.world_ee_pose(q)
    }
}

/// Where a walk along a line has got to: the configuration it last solved and
/// the blend it solved at. The plan walks the line with one and the runtime
/// follows it with one, each solve seeded from the last, so execution follows
/// the branch the plan validated.
#[derive(Debug, Clone, Copy)]
pub struct LineCursor {
    at: LineState<ARM_DOF>,
    swivel: Swivel,
}

impl LineCursor {
    /// A walk from `start_q`, the line's start, swiveling the elbow by
    /// `swivel` over the line.
    pub fn new(start_q: JointVec, swivel: Swivel) -> Self {
        Self {
            at: LineState {
                blend: 0.0,
                q: start_q,
            },
            swivel,
        }
    }

    /// The configuration the walk last solved.
    pub fn q(&self) -> JointVec {
        self.at.q
    }

    /// Walk on to `blend` in steps no wider than one plan cell, the resolution
    /// the plan walked the line at; the configuration at `blend`, or `None`
    /// when a solve fails, the cursor then at the last configuration solved.
    pub fn advance(
        &mut self,
        arm: &Arm,
        at_blend: impl Fn(f64) -> Isometry3<f64>,
        blend: f64,
    ) -> Option<JointVec> {
        let solver = Swiveling {
            arm,
            swivel: self.swivel,
        };
        let blends = subdivided_blends(self.at.blend, blend);
        let end = walk_line(&solver, at_blend, self.at, blends, None, |_, step| {
            self.at = step[step.len() - 1];
            true
        });
        (end == WalkEnd::Whole).then_some(self.at.q)
    }
}

/// Why a line is not continuously trackable under a swivel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Untracked {
    /// A pose on the line has no in-limit solution at the swivel's arm angle,
    /// though another arm angle has one: the elbow would carry a joint past
    /// its limit.
    PastALimit,
    /// The solution jumps branch between samples, or a pose on the line has
    /// no in-limit solution at any arm angle.
    Discontinuous,
}

/// The configurations at the plan's samples of the line `at_blend` traces,
/// walked from `start_q` swiveling by `swivel`, each solve seeded from the
/// last; or why the line is not continuously trackable under it.
fn sampled_walk(
    arm: &Arm,
    at_blend: &impl Fn(f64) -> Isometry3<f64>,
    start_q: JointVec,
    swivel: Swivel,
) -> Result<Vec<JointVec>, Untracked> {
    let mut walk = vec![start_q];
    let start = LineState {
        blend: 0.0,
        q: start_q,
    };
    let samples = (1..=LINE_PLAN_SAMPLES).map(|k| k as f64 * LINE_PLAN_DS);
    let end = walk_line(
        &Swiveling { arm, swivel },
        at_blend,
        start,
        samples,
        None,
        |before, step| {
            let q = step[step.len() - 1].q;
            let jumped = (0..ARM_DOF).any(|i| (q[i] - before.q[i]).abs() > MAX_LINE_STEP_RAD);
            if !jumped {
                walk.push(q);
            }
            !jumped
        },
    );
    match end {
        WalkEnd::Whole => Ok(walk),
        WalkEnd::NoSolution { blend } => {
            let reachable = arm
                .solve_ik(
                    &arm.base_pose(&at_blend(blend)),
                    ArmAnglePolicy::FromSeed,
                    &walk[walk.len() - 1],
                )
                .is_some();
            Err(if reachable {
                Untracked::PastALimit
            } else {
                Untracked::Discontinuous
            })
        }
        WalkEnd::Refused { .. } | WalkEnd::Imprecise { .. } => Err(Untracked::Discontinuous),
    }
}

/// Every swivel within [`MAX_ELBOW_SWIVEL_RAD`] that tracks the line, with its
/// walk, in the order tried: held first, then each share outward, the
/// positive swivel before the negative one.
fn swivel_walks(
    arm: &Arm,
    at_blend: &impl Fn(f64) -> Isometry3<f64>,
    start_q: JointVec,
) -> Vec<(Swivel, Vec<JointVec>)> {
    let share = MAX_ELBOW_SWIVEL_RAD * LINE_PLAN_DS;
    std::iter::once(0.0)
        .chain((1..=LINE_PLAN_SAMPLES).flat_map(|k| [k as f64, -(k as f64)]))
        .map(|k| Swivel { by_rad: k * share })
        .filter_map(|swivel| Some((swivel, sampled_walk(arm, at_blend, start_q, swivel).ok()?)))
        .collect()
}

/// The swivels of `walks` from the one whose walk keeps the lowest `score`
/// highest down; the start, which every walk shares, does not count, and the
/// swivel tried first wins a tie, so the elbow swings no further than the
/// score asks.
fn ranked(
    walks: &[(Swivel, Vec<JointVec>)],
    score: impl Fn(&JointVec) -> f64,
) -> impl Iterator<Item = (Swivel, &[JointVec])> {
    let mut scored: Vec<(f64, usize)> = walks
        .iter()
        .enumerate()
        .map(|(i, (_, walk))| {
            (
                walk[1..].iter().map(&score).fold(f64::INFINITY, f64::min),
                i,
            )
        })
        .collect();
    scored.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    scored
        .into_iter()
        .map(move |(_, i)| (walks[i].0, walks[i].1.as_slice()))
}

/// The blend of the line the end effector has reached at `progress` seconds of
/// its time at the caps, out of the `ee_s` the whole line takes: 1 from the
/// whole on, so a line the end effector does not move along is at its end at
/// once.
fn blend_of(progress: f64, ee_s: f64) -> f64 {
    if progress >= ee_s {
        1.0
    } else {
        progress / ee_s
    }
}

/// The end effector's time at the caps along a timed walk: its progress at the
/// end.
fn ee_s(path: &TimedPath) -> f64 {
    ee_progress(&path.at_fraction(1.0))
}

/// A straight line the arm tracks: its duration, how its elbow moves, the
/// configuration the runtime seeds its walk from, and the timed walk the
/// runtime reads the line's blend off.
#[derive(Debug, Clone, PartialEq)]
pub struct LinePlan {
    pub duration: Duration,
    pub elbow: Elbow,
    pub swivel: Swivel,
    /// The seed's own solution at the start pose, which the runtime walks the
    /// line from: the seed and its re-solve can differ at a redundancy or
    /// joint-limit boundary.
    pub start_q: JointVec,
    /// The plan's walk and the end effector's progress along it, timed with
    /// the end effector (see [`timed_with_ee`]).
    pub path: TimedPath,
}

/// A planned line played over a duration its planner sized, no shorter than
/// its path's own: at time `now`, the blend of the line the end effector has
/// reached, and the pose there.
#[derive(Debug, Clone, PartialEq)]
pub struct LineTrajectory {
    start: Isometry3<f64>,
    end: Isometry3<f64>,
    ee_s: f64,
    timed: TimedTrajectory,
}

impl LineTrajectory {
    /// The line from `start` to `end` along the planned `path`, over
    /// `duration`, starting at `now`.
    pub fn new(
        start: Isometry3<f64>,
        end: Isometry3<f64>,
        path: TimedPath,
        duration: Duration,
        now: Instant,
    ) -> Self {
        Self {
            start,
            end,
            ee_s: ee_s(&path),
            timed: TimedTrajectory::new(path, duration, now),
        }
    }

    pub fn clock(&self) -> &MotionClock {
        self.timed.clock()
    }

    /// The pose at blend `s` in `[0, 1]` on the line.
    pub fn pose_at_blend(&self, s: f64) -> Isometry3<f64> {
        interpolate(&self.start, &self.end, s)
    }

    /// The blend of the line the end effector has reached at `now`.
    pub fn blend(&self, now: Instant) -> f64 {
        blend_of(ee_progress(&self.timed.sample(now)), self.ee_s)
    }
}

/// The line plan for a move from the end effector's pose at `seed` to `end`,
/// from the quietest elbow that tracks the line (see [`Elbow`]): held, unless
/// the held elbow would carry a joint past its limit, when the swivels are
/// tried from the widest joint-limit margin down; then from the highest
/// manipulability down.
///
/// A swivel is taken when the line tracks continuously under it and its
/// duration stays within twice the request, or three seconds for a near-zero
/// request; `None` when none is. The duration is the longer of the request and
/// the walk's time with the end effector ([`timed_with_ee`]), which keeps every
/// joint within its velocity and acceleration limits along the walk and the
/// end effector within both caps, stretched, to within a control period, until
/// every joint moves and accelerates within its limits as the runtime commands
/// the line tick by tick. Poses are in the world frame; IK runs in the arm base
/// frame, so each sample is converted with [`Arm::base_pose`].
pub(crate) fn plan_line(
    arm: &Arm,
    end: &Isometry3<f64>,
    seed: JointVec,
    limits: &PlanLimits,
    requested: Duration,
) -> Option<LinePlan> {
    let start = arm.world_ee_pose(&seed);
    let start_q = arm
        .solve_ik(&arm.base_pose(&start), ArmAnglePolicy::FromSeed, &seed)?
        .q;
    let at_blend = |s: f64| interpolate(&start, end, s);
    let duration_cap = line_duration_cap(requested);
    let period = limits.control_period.as_secs_f64();
    let sized = |swivel: Swivel, walk: &[JointVec]| -> Option<(Duration, TimedPath)> {
        let path = timed_with_ee(
            arm.chain(),
            walk,
            &limits.max_joint_velocity_rad_s,
            &limits.max_joint_acceleration_rad_s2,
            &limits.ee,
        );
        let floor = duration_at_least(path.duration_s(), requested).as_secs_f64();
        if floor > duration_cap {
            return None;
        }
        let ee_s = ee_s(&path);
        // The configurations the runtime commands, one per tick, over a line
        // lasting `secs`; none when a tick's solve fails, where the runtime
        // would fail the move.
        let ticks_at = |secs: f64| -> Option<Vec<JointVec>> {
            let fractions: Vec<f64> = tick_fractions(secs, period).collect();
            let mut cursor = LineCursor::new(start_q, swivel);
            let ticks: Vec<JointVec> = fractions
                .iter()
                .map_while(|&f| {
                    cursor.advance(
                        arm,
                        at_blend,
                        blend_of(ee_progress(&path.at_fraction(f)), ee_s),
                    )
                })
                .collect();
            (ticks.len() == fractions.len()).then_some(ticks)
        };
        let secs = tick_checked_duration(
            floor,
            duration_cap,
            period,
            &limits.max_joint_velocity_rad_s,
            &limits.max_joint_acceleration_rad_s2,
            ticks_at,
        )?;
        Some((Duration::from_secs_f64(secs), path))
    };
    let line = |elbow, swivel, (duration, path)| LinePlan {
        duration,
        elbow,
        swivel,
        start_q,
        path,
    };
    let held_walk = sampled_walk(arm, &at_blend, start_q, Swivel::HELD);
    if let Ok(walk) = &held_walk
        && let Some(sized) = sized(Swivel::HELD, walk)
    {
        return Some(line(Elbow::Held, Swivel::HELD, sized));
    }
    let walks = swivel_walks(arm, &at_blend, start_q);
    if held_walk == Err(Untracked::PastALimit) {
        let joint_limits = arm.limits();
        for (swivel, walk) in ranked(&walks, |q| limit_margin(&joint_limits, q)) {
            if let Some(sized) = sized(swivel, walk) {
                return Some(line(Elbow::LimitMargin, swivel, sized));
            }
        }
    }
    for (swivel, walk) in ranked(&walks, |q| arm.at(q).manipulability()) {
        if let Some(sized) = sized(swivel, walk) {
            return Some(line(Elbow::Manipulability, swivel, sized));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::properties::sides;
    use crate::test_support::v1_arm;
    use hegel::TestCase;
    use hegel::generators as gs;
    use kinematics_properties::{configs, unit_vectors};
    use nalgebra::{Translation3, Unit, UnitQuaternion};

    const EPS: f64 = 1e-9;

    // A tick's IK samples walk from `prev` (exclusive) to `next` (inclusive)
    // in equal steps no wider than one plan cell, the last on `next` itself,
    // and as few as fit: one fewer would leave a step wider than a cell, and a
    // tick that progressed no more than one cell gets the single sample
    // `next`.
    #[hegel::test]
    fn subdivided_blends_cover_the_progress_at_the_plan_resolution(tc: TestCase) {
        let prev: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let progress: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0 - prev));
        let next = prev + progress;
        let samples: Vec<f64> = subdivided_blends(prev, next).collect();
        assert!((samples[samples.len() - 1] - next).abs() <= EPS);
        let mut last = prev;
        for &s in &samples {
            assert!(s - last <= LINE_PLAN_DS + EPS && s >= last, "{last} -> {s}");
            last = s;
        }
        if samples.len() > 1 {
            assert!(
                progress / (samples.len() - 1) as f64 > LINE_PLAN_DS - EPS,
                "{} samples where fewer would do for {prev} -> {next}",
                samples.len()
            );
        }
    }

    // The blend of a line is the end effector's progress over its whole time,
    // 1 from the whole on, and 1 at once for a line the end effector does not
    // move along.
    #[hegel::test]
    fn the_blend_is_the_end_effectors_share_of_its_time_saturating_at_the_end(tc: TestCase) {
        let ee_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let progress: f64 = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
        let blend = blend_of(progress, ee_s);
        if progress >= ee_s {
            assert_eq!(blend, 1.0);
        } else {
            assert_eq!(blend, progress / ee_s);
        }
    }

    // One plan sample's share of the swivel budget, `MAX_ELBOW_SWIVEL_RAD *
    // LINE_PLAN_DS`, stays well inside the branch-jump guard `MAX_LINE_STEP_RAD`
    // from a well-conditioned pose: if one sample's swivel could induce a joint
    // move near the guard, a swivel would trip the very discontinuity check
    // meant to catch real branch jumps. This pins the worst case, a full share
    // either way, comfortably under the guard.
    #[test]
    fn a_swivels_share_of_one_sample_stays_under_the_branch_guard() {
        let arm = v1_arm("left");
        let q: JointVec = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let target = arm.at(&q).ee_pose();
        let share = MAX_ELBOW_SWIVEL_RAD * LINE_PLAN_DS;
        for by_rad in [-share, share] {
            let swiveled = arm
                .solve_ik(&target, ArmAnglePolicy::Swiveled { by_rad }, &q)
                .expect("a full share from a well-conditioned pose is feasible");
            let induced = (0..ARM_DOF)
                .map(|i| (swiveled.q[i] - q[i]).abs())
                .fold(0.0_f64, f64::max);
            assert!(
                induced < 0.5 * MAX_LINE_STEP_RAD,
                "one share of swivel induced {induced:.4} rad, not comfortably under the \
                 {MAX_LINE_STEP_RAD} branch guard"
            );
        }
    }

    // The swivels of any walks come out from the one whose lowest score after
    // the shared start is highest, down; two with the same lowest score keep
    // the order they were tried in.
    #[hegel::test]
    fn walks_rank_by_their_lowest_score_after_the_start_and_then_by_order(tc: TestCase) {
        let walks: Vec<(Swivel, Vec<JointVec>)> = tc
            .draw(
                gs::vecs(
                    gs::vecs(gs::arrays(gs::floats().min_value(-3.0).max_value(3.0)))
                        .min_size(2)
                        .max_size(5),
                )
                .min_size(1)
                .max_size(8),
            )
            .into_iter()
            .enumerate()
            .map(|(k, walk): (usize, Vec<JointVec>)| (Swivel { by_rad: k as f64 }, walk))
            .collect();
        let score = |q: &JointVec| q[0];
        let lowest = |walk: &[JointVec]| walk[1..].iter().map(score).fold(f64::INFINITY, f64::min);
        let order: Vec<(Swivel, f64)> = ranked(&walks, score)
            .map(|(swivel, walk)| (swivel, lowest(walk)))
            .collect();
        assert_eq!(order.len(), walks.len());
        for w in order.windows(2) {
            assert!(w[0].1 >= w[1].1, "{:?} ranked above {:?}", w[0], w[1]);
            if w[0].1.total_cmp(&w[1].1).is_eq() {
                assert!(w[0].0.by_rad < w[1].0.by_rad, "a tie left the order tried");
            }
        }
    }

    // The line-acceptance budget scales with the request and floors at
    // MIN_LINE_BUDGET_S for a request under its share of the floor.
    #[hegel::test]
    fn the_line_duration_cap_scales_with_the_request_above_its_floor(tc: TestCase) {
        let requested =
            Duration::from_secs_f64(tc.draw(gs::floats().min_value(0.0).max_value(100.0)));
        let cap = line_duration_cap(requested);
        let requested = requested.as_secs_f64();
        assert!(cap >= requested);
        assert_eq!(
            cap,
            (requested * LINE_OVERRUN_FACTOR).max(MIN_LINE_BUDGET_S)
        );
    }

    /// A line of either fixture arm from any configuration: the end effector
    /// moves up to 0.5 m and turns up to 1.2 rad, the elbow swivels by up to
    /// the whole budget either way, so walks along it track, pass a limit and
    /// jump branch.
    struct DrawnLine {
        arm: &'static Arm,
        start_q: JointVec,
        start: Isometry3<f64>,
        end: Isometry3<f64>,
        swivel: Swivel,
    }

    impl DrawnLine {
        fn draw(tc: &TestCase) -> Self {
            let arm = v1_arm(tc.draw(sides()));
            let start_q = tc.draw(configs(arm.limits()));
            let start = arm.world_ee_pose(&start_q);
            let travel: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.5));
            let turn: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.2));
            let end = Isometry3::from_parts(
                Translation3::from(start.translation.vector + tc.draw(unit_vectors()) * travel),
                UnitQuaternion::from_axis_angle(
                    &Unit::new_normalize(tc.draw(unit_vectors())),
                    turn,
                ) * start.rotation,
            );
            let by_rad: f64 = tc.draw(
                gs::floats()
                    .min_value(-MAX_ELBOW_SWIVEL_RAD)
                    .max_value(MAX_ELBOW_SWIVEL_RAD),
            );
            Self {
                arm,
                start_q,
                start,
                end,
                swivel: Swivel { by_rad },
            }
        }

        fn at_blend(&self, s: f64) -> Isometry3<f64> {
            interpolate(&self.start, &self.end, s)
        }

        /// The IK solution at blend `to`, seeded from `q` solved at blend
        /// `from`, the elbow swiveled on by the share of the swivel between
        /// them.
        fn solved_from(&self, from: f64, q: &JointVec, to: f64) -> Option<JointVec> {
            let policy = ArmAnglePolicy::Swiveled {
                by_rad: self.swivel.by_rad * (to - from),
            };
            self.arm
                .solve_ik(&self.arm.base_pose(&self.at_blend(to)), policy, q)
                .map(|solution| solution.q)
        }
    }

    fn bits(q: &JointVec) -> [u64; ARM_DOF] {
        q.map(f64::to_bits)
    }

    // A cursor walks each tick by one IK solve per subdivided blend, each
    // seeded from the solution before with the elbow swiveled on by the
    // blend's share since it, to the bit; a tick whose solve fails leaves the
    // cursor at the last solution, and the ticks after walk on from there.
    #[hegel::test]
    fn a_cursor_solves_each_blend_from_the_solution_before(tc: TestCase) {
        let line = DrawnLine::draw(&tc);
        let ticks: Vec<f64> = tc
            .draw(gs::vecs(gs::floats().min_value(0.0).max_value(0.3)).max_size(12))
            .into_iter()
            .scan(0.0, |blend: &mut f64, step: f64| {
                *blend = (*blend + step).min(1.0);
                Some(*blend)
            })
            .collect();
        let mut cursor = LineCursor::new(line.start_q, line.swivel);
        let mut solved = (0.0, line.start_q);
        for tick in ticks {
            let walked = subdivided_blends(solved.0, tick).try_fold(solved, |(from, q), s| {
                line.solved_from(from, &q, s)
                    .map(|next| (s, next))
                    .ok_or((from, q))
            });
            let expected = walked.ok().map(|(_, q)| bits(&q));
            solved = walked.unwrap_or_else(|stopped| stopped);
            let got = cursor.advance(line.arm, |s| line.at_blend(s), tick);
            assert_eq!(got.map(|q| bits(&q)), expected, "the tick to {tick}");
            assert_eq!(
                bits(&cursor.q()),
                bits(&solved.1),
                "the cursor after {tick}"
            );
        }
    }

    // The plan's walk of a line is one IK solve per plan sample, each seeded
    // from the solution before, to the bit; it stops at the first sample whose
    // solution moves a joint further than the branch guard, a discontinuity,
    // or that has no solution, past a limit when the seed's own arm angle
    // solves that sample's pose and a discontinuity when it does not.
    #[hegel::test]
    fn the_plans_walk_solves_each_sample_from_the_solution_before(tc: TestCase) {
        let line = DrawnLine::draw(&tc);
        let mut expected = vec![line.start_q];
        let mut blend = 0.0;
        let mut verdict = Ok(());
        for k in 1..=LINE_PLAN_SAMPLES {
            let s = k as f64 * LINE_PLAN_DS;
            let q = expected[expected.len() - 1];
            let Some(next) = line.solved_from(blend, &q, s) else {
                let reachable = line
                    .arm
                    .solve_ik(
                        &line.arm.base_pose(&line.at_blend(s)),
                        ArmAnglePolicy::FromSeed,
                        &q,
                    )
                    .is_some();
                verdict = Err(if reachable {
                    Untracked::PastALimit
                } else {
                    Untracked::Discontinuous
                });
                break;
            };
            if (0..ARM_DOF).any(|i| (next[i] - q[i]).abs() > MAX_LINE_STEP_RAD) {
                verdict = Err(Untracked::Discontinuous);
                break;
            }
            expected.push(next);
            blend = s;
        }
        tc.event(format!("{verdict:?}"));
        let expected = verdict.map(|()| expected.iter().map(bits).collect::<Vec<_>>());
        let walked = sampled_walk(line.arm, &|s| line.at_blend(s), line.start_q, line.swivel)
            .map(|walk| walk.iter().map(bits).collect::<Vec<_>>());
        assert_eq!(walked, expected);
    }
}
