//! Planning an arm path: the start, the goal configurations, OMPL's plan and its
//! simplification, and the check of the path the robot runs.
//!
//! A motion is walked, and each arc of a rounded path, by conservative advancement. A
//! joint turning by an angle moves a point of a link, or of a part a link carries, at
//! most that angle times the joint's reach (`Arm::reach`). A term of the clearances, the
//! map's or the robot's own body's, has `k` moving bodies: 1 for the map, which stands
//! still, and 2 for a pair of the self model, each of whose links moves. At each checked
//! state the walk measures each term's distance, and the term's spare is its distance
//! above `min_clearance_m` over `k`: a step that moves no point of a link farther than
//! the least spare of the terms keeps every state up to the next checked one at
//! `min_clearance_m` or more. A step moves a point at most `MAX_STEP_M`. So:
//!
//! - A state keeps the plan's clearance of each term: `min_clearance_m` plus `k` times
//!   `MIN_SPARE_M`, 1 mm from the map and 2 mm between the robot's own links, so that each
//!   step from it moves a point at least `MIN_SPARE_M`; a walk ends at a state that falls
//!   short of it. The voxel's size needs no inflation: a state is measured against each
//!   occupied voxel's whole box, which holds every point the voxel stands for.
//! - The robot runs a path from its measured joints, the start. A term the start falls
//!   short of the plan's clearance of departs along the straight line from the start by
//!   the start rule until it keeps it. At each checked state the term lies no nearer than
//!   the farthest checked state before it, or than its plan clearance where that is
//!   nearer, to the rounding of the arithmetic, `ROUNDING_SLACK_M`. The start sets the
//!   term's floor: 0 from a start below `min_clearance_m`, and from one at or above it
//!   `min_clearance_m`, or the start's distance less `2k` times `MIN_DEPARTURE_STEP_M`
//!   and `ROUNDING_SLACK_M` where that is lower, never below 0. While the term falls
//!   short, a step moves no point farther than the term's room, the farthest checked
//!   distance above the floor, over `2k`, so between two checked states the term comes
//!   nearer by at most half its room: from a start below `min_clearance_m` it stays above
//!   half its farthest distance, so above 0; from one at least `2k` times
//!   `MIN_DEPARTURE_STEP_M` above it, above `min_clearance_m`; and from one nearer above
//!   it, at most `k` times `MIN_DEPARTURE_STEP_M` below it, 0.1 mm for the map and 0.2 mm
//!   for the robot's own body. A motion ends where its step falls under
//!   `MIN_DEPARTURE_STEP_M`, so a start within `2k` times that above a floor of 0 takes
//!   no step. A term the start keeps the plan's clearance of, or that has risen to it,
//!   steps by its spare and keeps it, as on any run. A run on the straight line from the
//!   start, past a waypoint on it such as one a corner's repair adds to the first run, is
//!   walked by the rule from the start. Any other run is walked by its spare from its
//!   first state, so one that leaves a state short of the plan's clearance fails there.
//! - The planner keeps every state of its paths but the start at the plan's clearances:
//!   its check of a motion fails one whose first state, the start excepted, falls short
//!   of them, and one from the start that ends before each term keeps them. OMPL's
//!   simplifier keeps states it checks only as points of a checked motion, such as one
//!   between two checked states, so a plan answers the simplified path when the path the
//!   robot runs through it passes its check, and the planned path when only that one
//!   passes. MoveIt checks the simplified path alone and answers INVALID_MOTION_PLAN
//!   when it fails.
//! - Run from the plan's start, the path keeps `min_clearance_m` as these say. Run from
//!   up to a tolerance from the start in each joint, as a robot's controller allows, it
//!   comes nearer anything near its start by at most the sum over the joints of the
//!   tolerance times the joint's reach, as MoveIt's execution starts within its allowed
//!   start tolerance and checks nothing more.
//! - Each arc of a rounded path is walked from where it leaves a run to where it joins the
//!   next, along the path: a step of the path's length moves a point at most the step
//!   times the arc's lever, the sum over the joints of the largest share of the path's
//!   unit tangent the joint has on the arc times its reach, at most the joints' reach in
//!   their L2 norm (`Arm::lever_along`). A state on a run is not checked: the plan walks
//!   every run of its answer, a run it joins past a waypoint it drops, one within
//!   `MIN_JOINT_CHANGE_RAD` of the one before, among them. Each planned waypoint after the
//!   start keeps the plan's clearances, so the start rule holds at no corner, and each arc
//!   keeps the plan's clearances. A corner's repair adds waypoints at states its runs'
//!   walks check, the start rule's among them, so each run they split walks only states
//!   the plan walked: walked from a state it checks, a run steps as the walk stepped.
//!
//! The request's time limit bounds the plan as MoveIt's allowed planning time bounds a
//! request: the goal sampler draws up to `goal_samples` configurations, ending early once
//! the limit passes, OMPL makes the request's attempts in what remains of it, each under
//! all of it, and its simplifier takes what remains after the attempts, at most
//! `SIMPLIFICATION_ITERATIONS` iterations, starting only before the limit. A planner that
//! plans until its time ends, such as RRT*, leaves the simplifier none, and the plan
//! answers the path it planned. A corner's repair runs only before the limit: a corner
//! whose rounding fails once it has passed ends the plan with no path. The check of the
//! rounded path runs to its end past the limit.

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::AtomicBool;
use std::time::{Duration, Instant};

use control_core::path_timing::{MIN_JOINT_CHANGE_RAD, RoundedPath, distinct_joint_waypoints};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::{CheckError, Checks, Motion, PlanError, Planner, Request, Space, plan};

use crate::arm::Arm;
use crate::clearance::{Contact, Surroundings};
use crate::draws::mixed_seed;
use crate::goal::{Goal, GoalSampler};
use crate::stop::is_set;

/// The iterations the simplification of a planned path takes at most.
const SIMPLIFICATION_ITERATIONS: u32 = 1000;

/// The least spare a checked state keeps of each term, as the module says: a walk ends at
/// a state from which a step would move a point less than this. 1 mm: near a surface at
/// `min_clearance_m` the spare, and with it the step, shrinks toward 0, and the floor
/// bounds a motion's checks by its length over 1 mm.
const MIN_SPARE_M: f64 = 1e-3;

/// The farthest any point of a link moves between two checked states. 1.5 cm: a corner's
/// repair adds waypoints at states its runs' walks check, so each run of a path holds a
/// checked state within this of any point of it, and a state's distance from the map is
/// measured exactly up to `min_clearance_m` plus this, past which a piece's bounding
/// sphere bounds it.
pub const MAX_STEP_M: f64 = 0.015;

/// The shortest step of a motion from a start that falls short of its clearances.
const MIN_DEPARTURE_STEP_M: f64 = 1e-4;

/// How many of a term's bodies a motion moves: the map stands still, and both links of
/// a pair of the self model may move. The terms are the map's and the robot's own
/// body's, in that order.
const MOVED: [f64; 2] = [1.0, 2.0];

/// The share of each of its runs at which the first waypoints added at a corner that
/// fails its check lie from it; each failed try halves it.
const FIRST_CORNER_FRACTION: f64 = 0.25;

/// How far from a segment, in the joints' L2 norm, a point of a straight run of the
/// rounded path lies: the rounding of its interpolation.
const ON_SEGMENT_RAD: f64 = 1e-9;

/// How much nearer than the farthest checked state before it a checked state of the
/// start rule may lie: the rounding of the arithmetic, which moves a pair of links that
/// one rigid motion carries by some 1e-17 m.
const ROUNDING_SLACK_M: f64 = 1e-9;

/// The halvings that find where an arc of a rounded path starts and ends: a path some
/// meters long to well under a nanometer.
const BISECTIONS: usize = 48;

/// The map a plan keeps clear of, as it stood at `stamp`.
pub struct Scene<'a> {
    pub map: &'a VoxelMap,
    pub field: &'a DistanceField,
    /// The stamp of the last frame the map took in, in nanoseconds. It joins the
    /// request's seed, so the same request on the same map gives the same path.
    pub stamp: u64,
}

/// One plan of an arm's path.
pub struct PlanRequest<'a, const N: usize> {
    /// Where the arm stands.
    pub start: [f64; N],
    /// How far, per joint, the start may lie outside a joint limit and still be moved
    /// onto it.
    pub start_tolerance: [f64; N],
    pub goal: &'a Goal,
    pub sampler: &'a dyn GoalSampler<N>,
    /// The configurations the sampler draws for the goal.
    pub goal_samples: NonZeroUsize,
    pub planner: Planner,
    /// The time the goal sampling and the planner's plan take at most, as the module
    /// says.
    pub time_limit: Duration,
    pub max_iterations: NonZeroU64,
    /// The attempts OMPL makes, as MoveIt's `num_planning_attempts`: each from its own
    /// seed, under the whole time limit, their paths joined by OMPL's path hybridization.
    pub attempts: NonZeroU32,
    pub seed: u32,
    pub min_clearance_m: f64,
    /// A flag the caller sets to end the plan, at any part of it.
    pub stop: Option<&'a AtomicBool>,
}

/// Why a plan gives no path.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum PlanFailure {
    #[error("{0}")]
    Request(String),
    #[error(
        "joint {joint} starts at {position}, outside its limits {low} to {high} by more than its start tolerance {tolerance}"
    )]
    StartOutsideLimits {
        joint: usize,
        position: f64,
        low: f64,
        high: f64,
        tolerance: f64,
    },
    #[error("the start is in contact: {0}")]
    StartInCollision(Contact),
    #[error("{0}")]
    Unreachable(String),
    #[error("{0}")]
    NoPath(String),
    #[error("the plan was stopped")]
    Stopped,
    #[error(transparent)]
    Planner(#[from] PlanError),
}

/// Plans `request` for `arm` around `scene`: the waypoints of a path from the start to
/// a configuration that meets the goal, whose rounded path, as execute_joint_path
/// rounds it, keeps its clearance.
pub fn plan_arm_path<const N: usize>(
    arm: &Arm<N>,
    scene: &Scene,
    request: &PlanRequest<N>,
) -> Result<Vec<[f64; N]>, PlanFailure> {
    let distance = |what: &str, value: f64| {
        (value.is_finite() && value >= 0.0)
            .then_some(())
            .ok_or_else(|| {
                PlanFailure::Request(format!(
                    "{what} is {value}; it is a finite distance of 0 or more"
                ))
            })
    };
    distance("min_clearance_m", request.min_clearance_m)?;
    request
        .start_tolerance
        .iter()
        .try_for_each(|&t| distance("a start tolerance", t))?;
    let deadline = Instant::now()
        .checked_add(request.time_limit)
        .ok_or_else(|| {
            PlanFailure::Request(format!(
                "the time limit {:?} runs past the clock's range; it is at most a few years",
                request.time_limit
            ))
        })?;
    let start = start_inside_limits(arm, &request.start, &request.start_tolerance)?;
    if let Some(contact) = Surroundings::new(scene.map, scene.field, 0.0, 0.0).contact(arm, &start)
    {
        return Err(PlanFailure::StartInCollision(contact));
    }
    let checks = ArmChecks::new(arm, scene, start, request.min_clearance_m);
    let goals = goal_configurations(arm, &checks, request, deadline)?;
    let remaining = deadline.saturating_duration_since(Instant::now());
    if remaining.is_zero() {
        return Err(PlanFailure::NoPath(format!(
            "the time limit {:?} ended before {} planned",
            request.time_limit, request.planner
        )));
    }
    let limits = arm.geometry().chain().limits();
    let bounds: Vec<(f64, f64)> = limits.iter().map(|l| (l.lo, l.hi)).collect();
    let space = Space::new(&bounds).map_err(|e| PlanFailure::NoPath(e.to_string()))?;
    let goal_values = vectors(&goals);
    let answer = plan(
        &Request {
            planner: request.planner,
            space: &space,
            start: &start,
            goals: &goal_values,
            time_limit: remaining,
            iterations: request.max_iterations,
            seed: mixed_seed(request.seed, scene.stamp),
            attempts: request.attempts,
            simplification: NonZeroU32::new(SIMPLIFICATION_ITERATIONS),
            simplification_time: Duration::ZERO,
            projection: None,
            stop: request.stop,
        },
        &checks,
    )?;
    if is_set(request.stop) {
        return Err(PlanFailure::Stopped);
    }
    let (Some(planned), Some(simplification)) = (answer.path, answer.simplified) else {
        return Err(PlanFailure::NoPath(format!(
            "{} found no path within {} iterations or {:?}, keeping {}",
            request.planner,
            request.max_iterations,
            request.time_limit,
            checks.plan_clearance()
        )));
    };
    let simplified = simplification.path;
    answered(&checks, &simplified, deadline, request.stop).or_else(|failure| match failure {
        PlanFailure::NoPath(_) if planned != simplified => {
            answered(&checks, &planned, deadline, request.stop)
        }
        other => Err(other),
    })
}

/// The answer of a plan whose planner found `path`: its waypoints, the waypoints added
/// near its corners among them, when the path the robot runs through them passes its
/// check, or why it does not. A set `stop` ends the check of its corners.
fn answered<const N: usize>(
    checks: &ArmChecks<N>,
    path: &[Vec<f64>],
    deadline: Instant,
    stop: Option<&AtomicBool>,
) -> Result<Vec<[f64; N]>, PlanFailure> {
    let waypoints: Vec<[f64; N]> = path
        .iter()
        .map(|state| {
            state
                .as_slice()
                .try_into()
                .expect("a state of the arm's space")
        })
        .collect();
    let answer = rounded_clear(
        checks,
        &distinct_joint_waypoints(&waypoints),
        deadline,
        stop,
    )?;
    match first_failing_run(checks, &answer) {
        None => Ok(answer),
        Some(k) => Err(PlanFailure::NoPath(format!(
            "the run from waypoint {k} of the path to the next falls short of {}",
            checks.plan_clearance()
        ))),
    }
}

/// The floor a start `distance` meters from the bodies of a term with `moved` moving
/// bodies sets, as the module says.
fn floor_of(distance: f64, min_clearance_m: f64, moved: f64) -> f64 {
    if distance < min_clearance_m {
        return 0.0;
    }
    min_clearance_m
        .min(distance - 2.0 * moved * (MIN_DEPARTURE_STEP_M + ROUNDING_SLACK_M))
        .max(0.0)
}

/// `start`, each joint outside its limits by no more than its tolerance moved onto
/// the limit.
fn start_inside_limits<const N: usize>(
    arm: &Arm<N>,
    start: &[f64; N],
    tolerance: &[f64; N],
) -> Result<[f64; N], PlanFailure> {
    let limits = arm.geometry().chain().limits();
    (0..N)
        .map(|i| {
            let (low, high) = (limits[i].lo, limits[i].hi);
            let position = start[i];
            if !(low - tolerance[i]..=high + tolerance[i]).contains(&position) {
                return Err(PlanFailure::StartOutsideLimits {
                    joint: i,
                    position,
                    low,
                    high,
                    tolerance: tolerance[i],
                });
            }
            Ok(position.clamp(low, high))
        })
        .collect::<Result<Vec<f64>, _>>()
        .map(|values| values.try_into().expect("one value per joint"))
}

/// The configurations that meet the goal and keep their clearance, drawn before
/// `deadline`, or why there are none; a set stop flag ends the plan.
fn goal_configurations<const N: usize>(
    arm: &Arm<N>,
    checks: &ArmChecks<N>,
    request: &PlanRequest<N>,
    deadline: Instant,
) -> Result<Vec<[f64; N]>, PlanFailure> {
    let sampled = request.sampler.sample(
        request.goal,
        request.goal_samples.get(),
        u64::from(request.seed),
        deadline,
        request.stop,
    );
    if is_set(request.stop) {
        return Err(PlanFailure::Stopped);
    }
    if sampled.is_empty() {
        return Err(PlanFailure::Unreachable(format!(
            "inverse kinematics found no configuration that meets the goal in {} samples or {:?}",
            request.goal_samples, request.time_limit
        )));
    }
    let clear: Vec<[f64; N]> = sampled
        .iter()
        .filter(|q| checks.surroundings.clear(arm, q))
        .copied()
        .collect();
    if clear.is_empty() {
        return Err(PlanFailure::Unreachable(format!(
            "each of the {} configurations that meet the goal is in collision, nearer than {}",
            sampled.len(),
            checks.plan_clearance()
        )));
    }
    Ok(clear)
}

/// The checks of the plan's states and motions.
struct ArmChecks<'a, const N: usize> {
    arm: &'a Arm<'a, N>,
    /// The plan's clearances.
    surroundings: Surroundings<'a>,
    /// The surroundings at which a state's distances are measured, exactly up to the
    /// distance a step of `MAX_STEP_M` needs.
    measuring: Surroundings<'a>,
    min_clearance: f64,
    start: [f64; N],
    /// Whether the start falls short of the plan's clearance of each term, the map's and
    /// the robot's own body's.
    start_short: [bool; 2],
    /// The plan's clearance of each term.
    clearance: [f64; 2],
    /// The floor the start sets of each term, as the module says.
    floor: [f64; 2],
}

impl<'a, const N: usize> ArmChecks<'a, N> {
    /// The checks of `arm`'s states and motions around `scene`, keeping `min_clearance_m`
    /// along every path from `start`, as the module says.
    fn new(arm: &'a Arm<'a, N>, scene: &Scene<'a>, start: [f64; N], min_clearance_m: f64) -> Self {
        let clearance = MOVED.map(|moved| min_clearance_m + moved * MIN_SPARE_M);
        let measured = MOVED.map(|moved| min_clearance_m + moved * MAX_STEP_M);
        let measuring = Surroundings::new(scene.map, scene.field, measured[0], measured[1]);
        let at_start = measuring.distances(arm, &start);
        Self {
            arm,
            surroundings: Surroundings::new(scene.map, scene.field, clearance[0], clearance[1]),
            measuring,
            min_clearance: min_clearance_m,
            start,
            start_short: std::array::from_fn(|term| at_start[term] < clearance[term]),
            clearance,
            floor: std::array::from_fn(|term| {
                floor_of(at_start[term], min_clearance_m, MOVED[term])
            }),
        }
    }

    /// The planner's check of the motion from `from` to `to`, which keeps every state
    /// the planner keeps but the start at the plan's clearances: `from` keeps them unless
    /// it is the start, and the motion keeps them as `walk_run` walks it, a motion by the
    /// start rule ending where each term keeps them. A motion by the start rule that fails
    /// keeps no state.
    fn walk(&self, from: &[f64; N], to: &[f64; N]) -> Motion {
        if *from != self.start && !self.surroundings.clear(self.arm, from) {
            return Motion::Invalid { last_valid: 0.0 };
        }
        let by_rule = self.departs_by_rule(from, to);
        match self.walk_run(from, to) {
            Ok(false) => Motion::Valid,
            Err(last_valid) if !by_rule => Motion::Invalid { last_valid },
            Ok(true) | Err(_) => Motion::Invalid { last_valid: 0.0 },
        }
    }

    /// The plan's clearances, as a message names them.
    fn plan_clearance(&self) -> String {
        let [map, own] = self.clearance;
        match self.arm.self_model() {
            None => format!("the plan's clearance, {map:.4} m from the map"),
            Some(_) => format!(
                "the plan's clearances, {map:.4} m from the map and {own:.4} m between the robot's own links"
            ),
        }
    }

    /// Whether `walk_run` walks the run from `from` to `to` by the start rule from the
    /// start: a run that leaves the start, or lies on the straight line from it, while
    /// the start falls short of a term's plan clearance.
    fn departs_by_rule(&self, from: &[f64; N], to: &[f64; N]) -> bool {
        self.start_short.contains(&true)
            && (*from == self.start || on_segment(from, &self.start, to))
    }

    /// Walks the run from `from` to `to` of a path from the start, as the module says: a
    /// run on the straight line from the start by the start rule from the start, and any
    /// other run by its spare, as `advance_by_spare` walks it. Whether a term the start
    /// falls short of still falls short of its plan clearance at `to`; where the run
    /// breaks, the share of it before the state that breaks it.
    fn walk_run(&self, from: &[f64; N], to: &[f64; N]) -> Result<bool, f64> {
        self.walk_run_checking(from, to, &mut |_| {})
    }

    /// The shares of the run from `from` to `to` at which `walk_run` checks a state that
    /// passes.
    fn walked_shares(&self, from: &[f64; N], to: &[f64; N]) -> Vec<f64> {
        let mut shares = Vec::new();
        // A run that breaks leaves the shares of the states before the break.
        let _ = self.walk_run_checking(from, to, &mut |share| shares.push(share));
        shares
    }

    /// `walk_run`, handing `checked` the share of the run at each state it checks that
    /// passes.
    fn walk_run_checking(
        &self,
        from: &[f64; N],
        to: &[f64; N],
        checked: &mut dyn FnMut(f64),
    ) -> Result<bool, f64> {
        if self.departs_by_rule(from, to) {
            let skipped = self.arm.displacement(&self.start, from);
            let total = self.arm.displacement(&self.start, to);
            let mut past_from = |t: f64| {
                if t * total > skipped {
                    checked(share_past(t, skipped, total));
                }
            };
            return self
                .depart_from_start(to, &mut past_from)
                .map_err(|t| share_past(t, skipped, total));
        }
        let total = self.arm.displacement(from, to);
        self.advance_by_spare(0.0, 1.0, total, |t| toward(from, to, t), checked)
            .map(|()| false)
    }

    /// Walks a motion by conservative advancement, as the module says, from the position
    /// `from` of the motion to `to`, its state at a position `at`, a step of a unit of
    /// position moving any point of a link at most `lever` meters, handing `checked` each
    /// position past `from` at which it checks a state that keeps the plan's clearances.
    /// The motion's state at `from` sets the first step, and a state from which a step
    /// would move a point less than `MIN_SPARE_M` ends it. The position of the last state
    /// it checks, `from` among them, where the motion breaks.
    fn advance_by_spare(
        &self,
        from: f64,
        to: f64,
        lever: f64,
        at: impl Fn(f64) -> [f64; N],
        checked: &mut dyn FnMut(f64),
    ) -> Result<(), f64> {
        let mut position = from;
        let mut here = self.distances(&at(from));
        while position < to {
            let step = self.spare_step(&here).ok_or(position)?;
            let next = if lever > 0.0 {
                (position + step / lever).min(to)
            } else {
                to
            };
            here = self.distances(&at(next));
            if !self.keeps_clearances(&here) {
                return Err(position);
            }
            checked(next);
            position = next;
        }
        Ok(())
    }

    /// The step, in meters any point of a link moves, that keeps every state up to the next
    /// checked one at `min_clearance_m` from a state `distances` meters from the bodies of
    /// each term: the least spare of the terms, at most `MAX_STEP_M`. None under
    /// `MIN_SPARE_M`.
    fn spare_step(&self, distances: &[f64; 2]) -> Option<f64> {
        let step = (0..2)
            .map(|term| self.spare(term, distances[term]))
            .fold(MAX_STEP_M, f64::min);
        (step >= MIN_SPARE_M).then_some(step)
    }

    /// The spare of a term `distance` meters from its bodies: its distance above
    /// `min_clearance_m` over its moving bodies.
    fn spare(&self, term: usize, distance: f64) -> f64 {
        (distance - self.min_clearance) / MOVED[term]
    }

    /// Whether a state `distances` meters from the bodies of each term keeps each of the
    /// plan's clearances.
    fn keeps_clearances(&self, distances: &[f64; 2]) -> bool {
        (0..2).all(|term| distances[term] >= self.clearance[term])
    }

    /// Walks the motion from the start to `to` by the start rule, as the module says,
    /// handing `checked` the share of the motion at each state it checks that passes.
    /// Whether a term the start falls short of still falls short of its plan clearance
    /// at `to`; where the rule breaks, the share of the motion before the state that
    /// breaks it.
    fn depart_from_start(&self, to: &[f64; N], checked: &mut dyn FnMut(f64)) -> Result<bool, f64> {
        let total = self.arm.displacement(&self.start, to);
        let mut here = self.distances(&self.start);
        let mut before = here;
        let mut t = 0.0;
        while t < 1.0 {
            let step = self.step_from(&here, &before).ok_or(t)?;
            let next = advance(t, step, total);
            here = self.distances(&toward(&self.start, to, next));
            if !self.keeps(&here, &before) {
                return Err(t);
            }
            checked(next);
            before = std::array::from_fn(|term| before[term].max(here[term]));
            t = next;
        }
        Ok((0..2).any(|term| self.start_short[term] && before[term] < self.clearance[term]))
    }

    /// The distance of the arm at `q` from the bodies of each term: exact up to the
    /// distance a step of `MAX_STEP_M` needs, and a lower bound past it.
    fn distances(&self, q: &[f64; N]) -> [f64; 2] {
        self.measuring.distances(self.arm, q)
    }

    /// The longest step by the start rule from a state `here` meters from the bodies of
    /// each term, after states at most `before` meters from them: a term that still falls
    /// short of the plan's clearance steps by its room, at least `MIN_DEPARTURE_STEP_M`,
    /// and any other term by its spare, as `spare_step` says. None where a term's step
    /// falls under its floor.
    fn step_from(&self, here: &[f64; 2], before: &[f64; 2]) -> Option<f64> {
        (0..2).try_fold(MAX_STEP_M, |step, term| {
            let (term_step, least) =
                if self.start_short[term] && before[term] < self.clearance[term] {
                    (
                        departing_step(before[term], self.floor[term], MOVED[term]),
                        MIN_DEPARTURE_STEP_M,
                    )
                } else {
                    (self.spare(term, here[term]), MIN_SPARE_M)
                };
            (term_step >= least).then_some(step.min(term_step))
        })
    }

    /// Whether a state `here` meters from the bodies of each term keeps the start rule
    /// after states at most `before` meters from them: a term the start falls short of
    /// no nearer than before or than its plan clearance, to `ROUNDING_SLACK_M`, and any
    /// other term at its plan clearance.
    fn keeps(&self, here: &[f64; 2], before: &[f64; 2]) -> bool {
        (0..2).all(|term| {
            let clearance = self.clearance[term];
            if self.start_short[term] {
                here[term] >= before[term].min(clearance) - ROUNDING_SLACK_M
            } else {
                here[term] >= clearance
            }
        })
    }
}

/// The longest step by the start rule from a state `distance` meters from the bodies of a
/// term with `moved` moving bodies that falls short of its plan clearance, whose floor
/// the start set at `floor`: the term's room above its floor over twice its moving bodies.
fn departing_step(distance: f64, floor: f64, moved: f64) -> f64 {
    (distance - floor) / (2.0 * moved)
}

/// The share, of the motion from a point `skipped` meters along a straight motion of
/// `total` meters to its end, at which the share `t` of the whole motion lies; 0 at or
/// before that point.
fn share_past(t: f64, skipped: f64, total: f64) -> f64 {
    let rest = total - skipped;
    if rest > 0.0 {
        ((t * total - skipped) / rest).max(0.0)
    } else {
        0.0
    }
}

/// The share of a motion `step` meters on from the share `t`, where the motion moves a
/// point at most `total` meters in all.
fn advance(t: f64, step: f64, total: f64) -> f64 {
    if total > 0.0 {
        (t + step / total).min(1.0)
    } else {
        1.0
    }
}

impl<const N: usize> Checks for ArmChecks<'_, N> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        let q: [f64; N] = state.try_into()?;
        Ok(q == self.start || self.surroundings.clear(self.arm, &q))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.walk(from.try_into()?, to.try_into()?))
    }
}

/// The planned waypoints, with waypoints added near each corner whose rounding, as
/// execute_joint_path rounds it, does not keep its clearance, until each does; or why
/// some corner cannot before `deadline`, or which planned waypoint after the start falls
/// short of the plan's clearances. A set `stop` ends it before each walk of the path.
fn rounded_clear<const N: usize>(
    checks: &ArmChecks<N>,
    planned: &[[f64; N]],
    deadline: Instant,
    stop: Option<&AtomicBool>,
) -> Result<Vec<[f64; N]>, PlanFailure> {
    // Each planned waypoint after the start keeps the plan's clearances, so the start rule
    // holds at no corner, and each arc keeps the plan's clearances.
    if let Some(k) =
        (1..planned.len()).find(|&k| !checks.surroundings.clear(checks.arm, &planned[k]))
    {
        return Err(PlanFailure::NoPath(format!(
            "waypoint {k} of the planned path falls short of {}",
            checks.plan_clearance()
        )));
    }
    // For each planned waypoint, the share of its runs within which the waypoints added at
    // its corner lie, and those waypoints.
    let mut added: Vec<Option<(f64, [[f64; N]; 2])>> = vec![None; planned.len()];
    loop {
        let (waypoints, corner_of): (Vec<[f64; N]>, Vec<usize>) = planned
            .iter()
            .enumerate()
            .flat_map(|(k, &at)| match added[k] {
                None => vec![(at, k)],
                Some((_, [into, out])) => vec![(into, k), (at, k), (out, k)],
            })
            .unzip();
        if is_set(stop) {
            return Err(PlanFailure::Stopped);
        }
        let path = RoundedPath::joint_path(&vectors(&waypoints));
        let Some(corner) = first_failing_probe(checks, &path, &corner_of, planned) else {
            return Ok(waypoints);
        };
        if Instant::now() >= deadline {
            return Err(PlanFailure::NoPath(
                "the plan's time limit ended before its rounded path kept its clearance"
                    .to_string(),
            ));
        }
        let fraction = added[corner].map_or(FIRST_CORNER_FRACTION, |(f, _)| f / 2.0);
        let apart = |q: &[f64; N]| {
            (0..N).any(|i| (q[i] - planned[corner][i]).abs() > 2.0 * MIN_JOINT_CHANGE_RAD)
        };
        let repair = (corner > 0 && corner + 1 < planned.len())
            .then(|| repair_waypoints(checks, planned, corner, fraction))
            .flatten()
            .filter(|added| added.iter().all(apart));
        let Some(repair) = repair else {
            return Err(PlanFailure::NoPath(format!(
                "the rounded path falls short of {} at the corner of waypoint {corner}, however near it the waypoints added lie",
                checks.plan_clearance()
            )));
        };
        added[corner] = Some((fraction, repair));
    }
}

/// The waypoints a repair of the corner of waypoint `corner` of `planned` adds, one on
/// each of its runs within the share `fraction` of the run from the corner: the state the
/// run's walk checks there farthest from the corner, so that each run they split walks
/// only states the plan walked. `None` when the walk of a run checks no state there.
fn repair_waypoints<const N: usize>(
    checks: &ArmChecks<N>,
    planned: &[[f64; N]],
    corner: usize,
    fraction: f64,
) -> Option<[[f64; N]; 2]> {
    let (before, at, after) = (&planned[corner - 1], &planned[corner], &planned[corner + 1]);
    let into = checks
        .walked_shares(before, at)
        .into_iter()
        .find(|&s| s >= 1.0 - fraction && s < 1.0)?;
    let out = checks
        .walked_shares(at, after)
        .into_iter()
        .take_while(|&s| s <= fraction)
        .last()
        .filter(|&s| s < 1.0)?;
    Some([toward(before, at, into), toward(at, after, out)])
}

/// Each configuration of `waypoints` as a vector of its joint values.
fn vectors<const N: usize>(waypoints: &[[f64; N]]) -> Vec<Vec<f64>> {
    waypoints.iter().map(|q| q.to_vec()).collect()
}

/// The point `fraction` of the way from `at` to `other`.
fn toward<const N: usize>(at: &[f64; N], other: &[f64; N], fraction: f64) -> [f64; N] {
    std::array::from_fn(|i| at[i] + (other[i] - at[i]) * fraction)
}

/// The first run of `path`, which leaves from the start, by the waypoint it leaves from,
/// that does not keep its clearances as `ArmChecks::walk_run` walks it.
fn first_failing_run<const N: usize>(checks: &ArmChecks<N>, path: &[[f64; N]]) -> Option<usize> {
    path.windows(2)
        .position(|run| checks.walk_run(&run[0], &run[1]).is_err())
}

/// The waypoint of `planned` whose corner holds the first arc of `path` that does not keep
/// its clearances as `ArmChecks::advance_by_spare` walks it along the path, at the arc's
/// lever (`Arm::lever_along`). `corner_of` names, for each waypoint of `path`, the
/// waypoint of `planned` whose corner it rounds. A state on a segment of `planned` is not
/// checked: the plan walks each run of its answer.
fn first_failing_probe<const N: usize>(
    checks: &ArmChecks<N>,
    path: &RoundedPath,
    corner_of: &[usize],
    planned: &[[f64; N]],
) -> Option<usize> {
    let length = path.length();
    let at = |s: f64| -> [f64; N] { path.at(s).as_slice().try_into().expect("a configuration") };
    let walked = |s: f64| {
        let q = at(s);
        planned.windows(2).any(|w| on_segment(&q, &w[0], &w[1]))
    };
    (1..corner_of.len().saturating_sub(1)).find_map(|corner| {
        // The arc at `corner` runs from where the path reaches the corner's waypoint to
        // where it lies on a planned segment again, before the next waypoint.
        let from = first_position(0.0, length, |s| path.waypoint_at(s) >= corner);
        let next = first_position(from, length, |s| path.waypoint_at(s) > corner);
        let to = first_position(from, next, walked);
        let share: [f64; N] = path
            .max_abs_tangent(from, to)
            .try_into()
            .expect("a share of the tangent per joint");
        let lever = checks.arm.lever_along(&share);
        checks
            .advance_by_spare(from, to, lever, at, &mut |_| {})
            .err()
            .map(|s| corner_of[path.waypoint_at(s)])
    })
}

/// The first position from `low` to `high` at which `holds` holds, to the rounding of
/// bisection, for a `holds` that, once it holds, holds on to `high`; `high` when it holds
/// nowhere before it.
fn first_position(low: f64, high: f64, holds: impl Fn(f64) -> bool) -> f64 {
    let (mut low, mut high) = (low, high);
    for _ in 0..BISECTIONS {
        let middle = (low + high) / 2.0;
        if holds(middle) {
            high = middle;
        } else {
            low = middle;
        }
    }
    high
}

/// Whether `q` lies on the segment from `a` to `b`, to the rounding of the path's
/// arithmetic.
fn on_segment<const N: usize>(q: &[f64; N], a: &[f64; N], b: &[f64; N]) -> bool {
    let run: [f64; N] = std::array::from_fn(|i| b[i] - a[i]);
    let squared: f64 = run.iter().map(|r| r * r).sum();
    let t = if squared > 0.0 {
        ((0..N).map(|i| (q[i] - a[i]) * run[i]).sum::<f64>() / squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (0..N)
        .map(|i| (q[i] - (a[i] + run[i] * t)).powi(2))
        .sum::<f64>()
        .sqrt()
        <= ON_SEGMENT_RAD
}

#[cfg(test)]
mod tests {
    use camera_core::pose::Pose;
    use chain_kinematics::{ChainSpec, JointSelection};
    use collision_geometry::chain::ChainGeometry;
    use control_core::positive_finite::PositiveFinite;
    use hegel::TestCase;
    use hegel::generators as gs;
    use occupancy_map::{Bounds, Frame, Scan};

    use super::*;
    use crate::arm::SelfModel;

    const MIN_CLEARANCE_M: f64 = 0.02;

    /// The voxels of the maps of these tests.
    const VOXEL_M: f64 = 0.01;

    /// How far a link point moves at most between two states of a dense check.
    const DENSE_M: f64 = 5e-4;

    /// An arm of one prismatic joint along x whose link is a box of `size`, its center
    /// `offset_y` meters along y from the joint.
    fn sliding_box(size: [f64; 3], offset_y: f64) -> ChainGeometry<1> {
        let [x, y, z] = size;
        let urdf = format!(
            r#"<robot name="sliding_box">
                <link name="base"/>
                <link name="link_1">
                  <collision><geometry><box size="{x} {y} {z}"/></geometry></collision>
                </link>
                <joint name="joint_1" type="prismatic">
                  <origin xyz="0 {offset_y} 0"/>
                  <parent link="base"/><child link="link_1"/><axis xyz="1 0 0"/>
                  <limit lower="-0.5" upper="0.5" effort="1" velocity="1"/>
                </joint>
              </robot>"#
        );
        ChainGeometry::from_urdf(&urdf, "", &spec("link_1")).expect("a sliding box")
    }

    /// An arm of two prismatic joints, along x and then along y, whose link is a 2 cm
    /// cube: its joint space is the cube's place.
    fn gantry() -> ChainGeometry<2> {
        const URDF: &str = r#"<robot name="gantry">
            <link name="base"/>
            <link name="link_1"/>
            <link name="link_2">
              <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
            </link>
            <joint name="joint_1" type="prismatic">
              <parent link="base"/><child link="link_1"/><axis xyz="1 0 0"/>
              <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="joint_2" type="prismatic">
              <parent link="link_1"/><child link="link_2"/><axis xyz="0 1 0"/>
              <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
          </robot>"#;
        ChainGeometry::from_urdf(URDF, "", &spec("link_2")).expect("a gantry")
    }

    /// An arm of a revolute joint about z and then a prismatic joint along x, from 0 to
    /// 0.6 m, whose link is a 4 cm cube: the cube turns about the origin at the radius
    /// the second joint sets.
    fn turn_and_slide() -> ChainGeometry<2> {
        const URDF: &str = r#"<robot name="turn_and_slide">
            <link name="base"/>
            <link name="link_1"/>
            <link name="link_2">
              <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
            </link>
            <joint name="joint_1" type="revolute">
              <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="joint_2" type="prismatic">
              <parent link="link_1"/><child link="link_2"/><axis xyz="1 0 0"/>
              <limit lower="0" upper="0.6" effort="1" velocity="1"/>
            </joint>
          </robot>"#;
        ChainGeometry::from_urdf(URDF, "", &spec("link_2")).expect("a turning slide")
    }

    /// A planar arm of two revolute joints about z, links of 30 cm and 25 cm.
    fn two_links() -> ChainGeometry<2> {
        const URDF: &str = r#"<robot name="two_links">
            <link name="base"/>
            <link name="link_1">
              <collision><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry></collision>
            </link>
            <link name="link_2">
              <collision><origin xyz="0.125 0 0"/><geometry><box size="0.25 0.04 0.04"/></geometry></collision>
            </link>
            <joint name="joint_1" type="revolute">
              <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="joint_2" type="revolute">
              <origin xyz="0.3 0 0"/>
              <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
          </robot>"#;
        ChainGeometry::from_urdf(URDF, "", &spec("link_2")).expect("a two-link arm")
    }

    fn spec(tip_link: &str) -> ChainSpec<'_> {
        ChainSpec {
            base_link: None,
            tip_link,
            joints: JointSelection::PathOrder,
        }
    }

    /// A map of `voxel_m` voxels over the box from `min` to `max`, each voxel of `points`
    /// occupied, and its distance field.
    fn world(
        voxel_m: f64,
        min: [f64; 3],
        max: [f64; 3],
        points: &[[f64; 3]],
    ) -> (VoxelMap, DistanceField) {
        let frame = Frame::parse("world").expect("a frame name");
        let mut map = VoxelMap::new(
            frame.clone(),
            PositiveFinite::parse(voxel_m).expect("positive"),
            Bounds { min, max },
        )
        .expect("a valid map");
        let relative: Vec<[f32; 3]> = points
            .iter()
            .map(|p| std::array::from_fn(|i| p[i] as f32))
            .collect();
        map.insert(
            &Scan {
                frame: &frame,
                sensor: Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose"),
                points: &relative,
                max_range_m: PositiveFinite::parse(10.0).expect("positive"),
            },
            &[],
        )
        .expect("the scan stands in the map's frame");
        let field = DistanceField::of(&map);
        (map, field)
    }

    /// A self model whose pair stands `apart` meters apart at every configuration.
    struct StandsApart(f64);

    impl<const N: usize> SelfModel<N> for StandsApart {
        fn clearance(&self, _: &[f64; N]) -> f64 {
            self.0
        }

        fn nearest_pair(&self, _: &[f64; N]) -> (String, String) {
            ("a link".to_string(), "its neighbor".to_string())
        }
    }

    // A set stop flag ends the check of a rounded path before its walk, and the same
    // path with the flag clear passes.
    #[test]
    fn a_stop_ends_the_check_of_the_rounded_path() {
        let geometry = two_links();
        let arm = Arm::new(&geometry, None);
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &[]);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
        };
        let checks = ArmChecks::new(&arm, &scene, [0.0, 0.0], MIN_CLEARANCE_M);
        let path = [[0.0, 0.0], [0.5, 0.5], [1.0, 0.0]];
        let far = Instant::now() + Duration::from_secs(3600);
        let (clear, set) = (AtomicBool::new(false), AtomicBool::new(true));
        assert!(rounded_clear(&checks, &path, far, Some(&clear)).is_ok());
        assert_eq!(
            rounded_clear(&checks, &path, far, Some(&set)),
            Err(PlanFailure::Stopped)
        );
    }

    /// Whether the plain walk, a run from a state that is not the start, of the sliding
    /// box from x = -0.3 to 0.3 passes, at `gap` meters from a wall of voxels beside it or,
    /// with no wall, between a pair of its own body `apart` meters apart.
    fn sliding_walk_passes(gap: Option<f64>, apart: Option<f64>) -> bool {
        // The wall's face stands at y = 0.1, so the box's face, 5 cm from its center, at
        // y = 0.1 less the gap.
        let geometry = sliding_box([0.1; 3], 0.05 - gap.unwrap_or(0.0));
        let wall: Vec<[f64; 3]> = gap
            .map(|_| {
                (-50..=50)
                    .flat_map(|x| {
                        (-7..=7).map(move |z| [x as f64 * VOXEL_M, 0.105, z as f64 * VOXEL_M])
                    })
                    .collect()
            })
            .unwrap_or_default();
        let (map, field) = world(VOXEL_M, [-0.6, -0.2, -0.1], [0.6, 0.2, 0.1], &wall);
        let model = apart.map(StandsApart);
        let arm = Arm::new(&geometry, model.as_ref().map(|m| m as &dyn SelfModel<1>));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
        };
        let checks = ArmChecks::new(&arm, &scene, [0.45], MIN_CLEARANCE_M);
        checks.walk_run(&[-0.3], &[0.3]).is_ok()
    }

    // A motion that holds 2 mm more than `MIN_CLEARANCE_M` from the map, a spare of 2 mm
    // over its one moving body, passes; one that holds 0.5 mm more, a spare under
    // `MIN_SPARE_M`, fails at its first state.
    #[test]
    fn a_motion_two_millimeters_past_its_clearance_from_the_map_passes() {
        assert!(sliding_walk_passes(Some(MIN_CLEARANCE_M + 2e-3), None));
        assert!(!sliding_walk_passes(Some(MIN_CLEARANCE_M + 5e-4), None));
    }

    // A motion whose own pair holds 3 mm more than `MIN_CLEARANCE_M`, a spare of 1.5 mm
    // over its two moving bodies, passes; one whose pair holds 0.5 mm more, a spare of
    // 0.25 mm, fails at its first state.
    #[test]
    fn a_motion_three_millimeters_past_its_clearance_between_its_own_links_passes() {
        assert!(sliding_walk_passes(None, Some(MIN_CLEARANCE_M + 3e-3)));
        assert!(!sliding_walk_passes(None, Some(MIN_CLEARANCE_M + 5e-4)));
    }

    /// The distance from the tip of `two_links` at `q` to a ball of radius 5 cm at
    /// (-0.1, 0, 0): a pair of its own body.
    struct TipAndBall;

    impl SelfModel<2> for TipAndBall {
        fn clearance(&self, q: &[f64; 2]) -> f64 {
            let tip = [
                0.3 * q[0].cos() + 0.25 * (q[0] + q[1]).cos(),
                0.3 * q[0].sin() + 0.25 * (q[0] + q[1]).sin(),
            ];
            ((tip[0] + 0.1).powi(2) + tip[1].powi(2)).sqrt() - 0.05
        }

        fn nearest_pair(&self, _: &[f64; 2]) -> (String, String) {
            ("link_2".to_string(), "the ball".to_string())
        }
    }

    /// Where a corner of the outer end of `two_links`'s second link stands at `q`, on the
    /// side `side`, 1 or -1, of the link.
    fn outer_corner(q: &[f64; 2], side: f64) -> [f64; 3] {
        let (elbow, along) = (q[0], q[0] + q[1]);
        let (x, y) = (0.25, 0.02 * side);
        [
            0.3 * elbow.cos() + x * along.cos() - y * along.sin(),
            0.3 * elbow.sin() + x * along.sin() + y * along.cos(),
            0.0,
        ]
    }

    // On drawn worlds about a planar arm, at a drawn `min_clearance_m`, every straight
    // motion and every rounded path through three drawn configurations that the walk
    // passes, a path at the lever of its tangent (`Arm::lever_along`), keeps
    // `min_clearance_m` from the map and between its own pair at every state of a dense
    // check, each link point moving at most `DENSE_M` between two. Each world
    // holds a voxel within 1.5 cm of where an outer corner of the second link passes
    // partway along the first motion, where a corner grazing a voxel's edge comes nearer
    // between two states than at either, and up to two drawn blocks besides.
    #[hegel::test(test_cases = 200)]
    fn conservative_advancement_misses_no_contact_a_dense_check_finds(tc: TestCase) {
        let joint = || -> f64 { tc.draw(gs::floats().min_value(-2.5).max_value(2.5)) };
        let turn = || -> f64 { tc.draw(gs::floats().min_value(-0.6).max_value(0.6)) };
        let from = [joint(), joint()];
        let via = [from[0] + turn(), from[1] + turn()];
        let to = [via[0] + turn(), via[1] + turn()];
        let partway = toward(
            &from,
            &via,
            tc.draw(gs::floats().min_value(0.2).max_value(0.8)),
        );
        let side = if tc.draw(gs::booleans()) { 1.0 } else { -1.0 };
        let corner = outer_corner(&partway, side);
        let near = || -> f64 { tc.draw(gs::floats().min_value(-0.015).max_value(0.015)) };
        let grazed = [corner[0] + near(), corner[1] + near(), 0.0];
        let blocks = tc.draw(gs::integers::<usize>().min_value(0).max_value(2));
        let points: Vec<[f64; 3]> = std::iter::once(grazed)
            .chain((0..blocks).flat_map(|_| {
                let angle: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
                let radius: f64 = tc.draw(gs::floats().min_value(0.15).max_value(0.55));
                let side = tc.draw(gs::integers::<i32>().min_value(1).max_value(4));
                let (cx, cy) = (radius * angle.cos(), radius * angle.sin());
                (0..side).flat_map(move |i| {
                    (0..side).map(move |j| {
                        [
                            cx + f64::from(i) * VOXEL_M,
                            cy + f64::from(j) * VOXEL_M,
                            0.0,
                        ]
                    })
                })
            }))
            .collect();
        let (map, field) = world(VOXEL_M, [-0.7, -0.7, -0.1], [0.7, 0.7, 0.1], &points);
        let geometry = two_links();
        let arm = Arm::new(&geometry, Some(&TipAndBall));
        let min_clearance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.01));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
        };
        let checks = ArmChecks::new(&arm, &scene, [2.9, 2.9], min_clearance);
        tc.assume(checks.surroundings.clear(&arm, &from));
        let dense = Surroundings::new(&map, &field, min_clearance, min_clearance);
        let keeps = |q: &[f64; 2]| dense.clear(&arm, q);
        if checks.walk_run(&from, &via).is_ok() {
            let count = (arm.displacement(&from, &via) / DENSE_M).ceil() as usize;
            for k in 0..=count {
                let q = toward(&from, &via, k as f64 / count.max(1) as f64);
                assert!(
                    keeps(&q),
                    "the motion from {from:?} to {via:?} misses {q:?}"
                );
            }
        }
        let path = RoundedPath::joint_path(&vectors(&[from, via, to]));
        let share: [f64; 2] = path
            .max_abs_tangent(0.0, path.length())
            .try_into()
            .expect("a share of the tangent per joint");
        let lever = arm.lever_along(&share);
        let at =
            |s: f64| -> [f64; 2] { path.at(s).as_slice().try_into().expect("a configuration") };
        if checks
            .advance_by_spare(0.0, path.length(), lever, at, &mut |_| {})
            .is_ok()
        {
            let count = (path.length() * arm.lever() / DENSE_M).ceil() as usize;
            for k in 0..=count {
                let q = at(path.length() * k as f64 / count.max(1) as f64);
                assert!(
                    keeps(&q),
                    "the path through {from:?}, {via:?} and {to:?} misses {q:?}"
                );
            }
        }
    }

    // A departure from a start below `min_clearance_m` keeps the floor 0 its start set as
    // its distance rises through `min_clearance_m`: just above it, a step of either term
    // is its distance over twice its moving bodies. From a start at or above
    // `min_clearance_m`, a step is the room above it over twice the moving bodies.
    #[test]
    fn a_departure_steps_by_the_room_above_the_floor_its_start_set() {
        let just_above = MIN_CLEARANCE_M + 1e-5;
        for moved in MOVED {
            let rising = departing_step(just_above, 0.0, moved);
            assert_eq!(rising, just_above / (2.0 * moved));
            assert!(rising >= MIN_DEPARTURE_STEP_M);
            assert_eq!(
                departing_step(MIN_CLEARANCE_M + 0.004, MIN_CLEARANCE_M, moved),
                (MIN_CLEARANCE_M + 0.004 - MIN_CLEARANCE_M) / (2.0 * moved)
            );
        }
    }

    // A start below `min_clearance_m` sets the floor 0, and one far enough above it sets
    // `min_clearance_m`. From one 0.1 mm above it, under twice the moving bodies times
    // the shortest step above it, a step of either term is the shortest step. The floor
    // is never below 0.
    #[test]
    fn a_start_just_above_the_minimum_clearance_steps_the_shortest_step() {
        for moved in MOVED {
            assert_eq!(
                floor_of(MIN_CLEARANCE_M - 1e-4, MIN_CLEARANCE_M, moved),
                0.0
            );
            assert_eq!(
                floor_of(MIN_CLEARANCE_M + 0.01, MIN_CLEARANCE_M, moved),
                MIN_CLEARANCE_M
            );
            let start = MIN_CLEARANCE_M + 1e-4;
            let step = departing_step(start, floor_of(start, MIN_CLEARANCE_M, moved), moved);
            assert!(
                (MIN_DEPARTURE_STEP_M..MIN_DEPARTURE_STEP_M + 1e-8).contains(&step),
                "{step}"
            );
            assert_eq!(floor_of(1e-4, 0.0, moved), 0.0);
        }
    }

    // A plate 1 mm thick slides along x through a voxel of 2 mm, from 5 mm before it to
    // 3 cm past it, at a clearance of 0: a step of the plate moves each of its points by
    // the step itself, so the walk steps the 5 mm of its spare onto the voxel's face,
    // which falls short, and refuses the motion. A step of twice the spare, of four times
    // it, or of `MAX_STEP_M`, lands past the voxel at 2 mm or more and passes over the
    // states in contact.
    #[test]
    fn a_plate_sliding_through_a_voxel_is_refused_however_far_it_steps() {
        const VOXEL: f64 = 0.002;
        const PLATE: [f64; 3] = [0.001, 0.04, 0.04];
        let (map, field) = world(VOXEL, [-0.1; 3], [0.1; 3], &[[0.011, 0.0, 0.0]]);
        let face =
            map.center(map.key([0.011, 0.0, 0.0]).expect("a voxel of the map"))[0] - VOXEL / 2.0;
        let geometry = sliding_box(PLATE, 0.0);
        let arm = Arm::new(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
        };
        let from = [face - 0.005 - PLATE[0] / 2.0];
        let to = [face + 0.03];
        let checks = ArmChecks::new(&arm, &scene, [-0.4], 0.0);
        let touching = Surroundings::new(&map, &field, 0.0, 0.0);
        assert!(touching.contact(&arm, &[face + VOXEL / 2.0]).is_some());
        let gap = checks.distances(&from)[0];
        assert!(
            (gap - 0.005).abs() < 1e-12,
            "the plate starts {gap} m from the voxel"
        );
        assert!(checks.walk_run(&from, &to).is_err());
    }

    /// A goal sampler that answers one configuration.
    struct At<const N: usize>([f64; N]);

    impl<const N: usize> GoalSampler<N> for At<N> {
        fn sample(
            &self,
            _: &Goal,
            _: usize,
            _: u64,
            _: Instant,
            _: Option<&AtomicBool>,
        ) -> Vec<[f64; N]> {
            vec![self.0]
        }
    }

    // From -0.6 rad to 0.6 rad with the slide out at 0.5 m, a point of the cube of
    // `turn_and_slide` moves over half a meter, within the reach bound: the reach of the
    // turn covers the slide's travel past it.
    #[test]
    fn the_reach_of_a_turn_covers_the_slide_past_it() {
        let geometry = turn_and_slide();
        let arm = Arm::new(&geometry, None);
        let (from, to) = ([-0.6, 0.5], [0.6, 0.5]);
        let moved = arm
            .pieces(&from)
            .iter()
            .zip(&arm.pieces(&to))
            .flat_map(|((_, a), (_, b))| {
                a.vertices()
                    .zip(b.vertices())
                    .map(|(p, q)| (q - p).norm())
                    .collect::<Vec<_>>()
            })
            .fold(0.0, f64::max);
        assert!(moved > 0.5, "a point of the cube moves {moved} m");
        assert!(
            moved <= arm.displacement(&from, &to),
            "a point of the cube moves {moved} m, past the bound {} m",
            arm.displacement(&from, &to)
        );
    }

    // A wall of voxels along x from the origin to 0.75 m stands across every turn of the
    // cube of `turn_and_slide` from -0.6 rad to 0.6 rad, at any radius the slide sets, and
    // the turn's limits reach no way around it: the plan finds no path. The reach of the
    // turn covers the slide's travel, so no walk passes the cube through the wall.
    #[test]
    fn a_world_with_no_path_gives_no_path() {
        let wall: Vec<[f64; 3]> = (0..=75).map(|i| [f64::from(i) * 0.01, 0.0, 0.0]).collect();
        let (map, field) = world(0.02, [-0.8, -0.8, -0.2], [0.8, 0.8, 0.2], &wall);
        let geometry = turn_and_slide();
        let arm = Arm::new(&geometry, None);
        let (start, end) = ([-0.6, 0.5], [0.6, 0.5]);
        let goal = Goal::new(
            2,
            (0..2)
                .map(|joint| crate::goal::JointTarget {
                    joint,
                    position: end[joint],
                    above: 0.01,
                    below: 0.01,
                })
                .collect(),
            Vec::new(),
            Vec::new(),
        )
        .expect("a valid goal");
        let answer = plan_arm_path(
            &arm,
            &Scene {
                map: &map,
                field: &field,
                stamp: 1,
            },
            &PlanRequest {
                start,
                start_tolerance: [0.0; 2],
                goal: &goal,
                sampler: &At(end),
                goal_samples: NonZeroUsize::new(1).expect("nonzero"),
                planner: Planner::RrtConnect,
                time_limit: Duration::from_secs(600),
                max_iterations: NonZeroU64::new(20_000).expect("nonzero"),
                attempts: NonZeroU32::MIN,
                seed: 7,
                min_clearance_m: MIN_CLEARANCE_M,
                stop: None,
            },
        );
        assert_eq!(
            answer,
            Err(PlanFailure::NoPath(
                "RRTConnect found no path within 20000 iterations or 600s, keeping the plan's \
                 clearance, 0.0210 m from the map"
                    .to_string()
            ))
        );
    }

    // The gantry's cube runs 50 cm along x to the origin and then 50 cm along y, past a
    // block of voxels inside the corner, 6 cm from each run. Rounded within 0.1 m, the
    // corner sweeps the cube through the block. The plan adds a waypoint on each run near
    // the corner, and the rounded path through its answer keeps `MIN_CLEARANCE_M` at every
    // state of a dense check.
    #[test]
    fn a_corner_rounded_into_a_block_gets_waypoints_until_its_rounding_clears() {
        let block: Vec<[f64; 3]> = [-0.085, -0.075]
            .into_iter()
            .flat_map(|x| [0.075, 0.085].map(|y| [x, y, 0.0]))
            .collect();
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &block);
        let geometry = gantry();
        let arm = Arm::new(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
        };
        let planned = [[-0.5, 0.0], [0.0, 0.0], [0.0, 0.5]];
        let checks = ArmChecks::new(&arm, &scene, planned[0], MIN_CLEARANCE_M);
        let rounded_first = RoundedPath::joint_path(&vectors(&planned));
        let dense = Surroundings::new(&map, &field, MIN_CLEARANCE_M, MIN_CLEARANCE_M);
        let at = |path: &RoundedPath, s: f64| -> [f64; 2] {
            path.at(s).as_slice().try_into().expect("a configuration")
        };
        let probes = |path: &RoundedPath| -> Vec<[f64; 2]> {
            let count = (path.length() * arm.lever() / DENSE_M).ceil() as usize;
            (0..=count)
                .map(|k| at(path, path.length() * k as f64 / count as f64))
                .collect()
        };
        assert!(
            probes(&rounded_first).iter().any(|q| !dense.clear(&arm, q)),
            "the corner's rounding clears the block: nothing to repair"
        );
        let deadline = Instant::now() + Duration::from_secs(3600);
        let answer = match answered(&checks, &vectors(&planned), deadline, None) {
            Ok(answer) => answer,
            Err(failure) => panic!("the corner is not repaired: {failure}"),
        };
        for q in probes(&RoundedPath::joint_path(&vectors(&answer))) {
            assert!(
                dense.clear(&arm, &q),
                "the rounded path through {answer:?} touches the block at {q:?}"
            );
        }
        assert_eq!(answer.len(), planned.len() + 2, "{answer:?}");
    }
}
