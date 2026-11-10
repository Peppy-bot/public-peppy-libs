//! Planning the path of one or more arms that a plan moves together (`Arms`): the start,
//! the goal configurations, OMPL's plan and its simplification, and the check of the path
//! the robot runs. A state of the plan holds the joints of every arm, one arm after
//! another.
//!
//! A motion is walked, and each arc of a rounded path, by conservative advancement. A
//! joint turning by an angle moves a point of a link, or of a body a link carries, a part
//! of the robot or an object, at most that angle times the joint's reach (`Arm::reach`);
//! a point rides one arm, so a motion moves it at most the largest of the arms' bounds
//! (`Arms::displacement`). Each term of the clearances, as `Surroundings` holds them, is
//! asked a clearance (`term_clearances`): `min_clearance_m` from the map and the objects
//! and within the robot's own body, and 0 between an attached object and the links of the
//! arm that carries it, outside its touch links, which keep out of contact. A term has
//! `k` moving bodies: 1 for the map and the objects, which stand still, and 2 for the
//! robot's own body and for an attached object against the arm that carries it, both
//! bodies of whose pairs move. At each checked state the walk measures each term's
//! distance, and the term's spare is its distance above the clearance asked of it over
//! `k`: a step that moves no point of a link farther than the least spare of the terms
//! keeps every state up to the next checked one at the clearance asked of each term or
//! more. A step moves a point at most `MAX_STEP_M`. So:
//!
//! - A state keeps the plan's clearance of each term: the clearance asked of it plus `k`
//!   times `MIN_SPARE_M`, 1 mm from the map and the objects and 2 mm within the robot's
//!   own body and between an attached object and the arm that carries it, so that each
//!   step from it moves a point at least `MIN_SPARE_M`; a walk ends at a state that falls
//!   short of it. The voxel's size needs no inflation: a state is measured against each
//!   occupied voxel's whole box, which holds every point the voxel stands for.
//! - The robot runs a path from its measured joints, the start. A start nearer than
//!   `min_clearance_m` to the map or to an object no arm holds is refused, as MoveIt's
//!   collision check, padded by the clearance, refuses a start in collision
//!   (START_STATE_IN_COLLISION), and so is one in contact; MoveIt pads no pair of the
//!   robot's own body. A term the start otherwise falls short of the plan's clearance of,
//!   the robot's own body, an attached object against the arm that carries it, or the map
//!   and the objects within the spare above `min_clearance_m`, departs along the straight
//!   line from the start by the start rule until it keeps it. At each checked state the term lies no nearer than
//!   the farthest checked state before it, or than its plan clearance where that is
//!   nearer, to the rounding of the arithmetic, `ROUNDING_SLACK_M`. The start sets the
//!   term's floor: 0 from a start below the clearance asked of it, and from one at or
//!   above it that clearance, or the start's distance less `2k` times
//!   `MIN_DEPARTURE_STEP_M` and `ROUNDING_SLACK_M` where that is lower, never below 0.
//!   While the term falls short, a step moves no point farther than the term's room, the
//!   farthest checked distance above the floor, over `2k`, so between two checked states
//!   the term comes nearer by at most half its room: from a start below the clearance
//!   asked of it, it stays above half its farthest distance, so above 0; from one at
//!   least `2k` times `MIN_DEPARTURE_STEP_M` above it, above that clearance; and from one
//!   nearer above it, at most `k` times `MIN_DEPARTURE_STEP_M` below it, 0.1 mm for the
//!   map and the objects and 0.2 mm for the robot's own body. An attached object, asked
//!   to keep 0 from the arm that carries it, has a floor of 0 there and never comes
//!   nearer than 0. A motion ends where its step falls under `MIN_DEPARTURE_STEP_M`, so a
//!   start within `2k` times that above a floor of 0 takes no step. A term the start
//!   keeps the plan's clearance of, or that has risen to it, steps by its spare and keeps
//!   it, as on any run. A run whose path from the start lies on the straight line from
//!   the start to the run's end, past waypoints on it such as one a corner's repair adds
//!   to the first run, is walked by the rule from the start, and so is a planner's motion
//!   whose first state lies on the straight line from the start to its end. Any other run
//!   is walked by its spare from its first state, so one that leaves a state short of the
//!   plan's clearance fails there.
//! - The planner keeps every state of its paths but the start at the plan's clearances:
//!   its check of a motion fails one whose first state, the start excepted, falls short
//!   of them, and one from the start that ends before each term keeps them. OMPL's
//!   simplifier keeps states it checks only as points of a checked motion, such as one
//!   between two checked states, so a plan answers the simplified path when the path the
//!   robot runs through it passes its check, and the planned path when only that one
//!   passes. MoveIt checks the simplified path alone and answers INVALID_MOTION_PLAN
//!   when it fails.
//! - Run from the plan's start, the path keeps the clearance asked of each term as these
//!   say. Run from up to a tolerance from the start in each joint, as a robot's
//!   controller allows, it comes nearer anything near its start by at most the sum over
//!   the joints of the tolerance times the joint's reach, over the joints of both arms for
//!   a pair of two arms, as MoveIt's execution starts within its allowed start tolerance
//!   and checks nothing more.
//! - Each arc of a rounded path is walked from where it leaves a run to where it joins
//!   the next, along the path: a step of the path's length moves a point at most the step
//!   times the arc's lever, the sum over the joints of the largest share of the path's
//!   unit tangent the joint has on the arc times its reach, at most the joints' reach in
//!   their L2 norm, the largest of the arms' (`Arms::lever_along`). A state on a run is
//!   not checked: the plan walks every run of its answer, a run it joins past a waypoint
//!   it drops, one within `MIN_JOINT_CHANGE_RAD` of the one before, among them. Each
//!   planned waypoint after the start keeps the plan's clearances, so the start rule
//!   holds at no corner, and each arc keeps the plan's clearances. A corner's repair of its
//!   clearance adds waypoints at states its runs' walks check that keep the plan's
//!   clearances, the start rule's among them, so each run they split walks only states the
//!   plan walked: walked from a state it checks, a run steps as the walk stepped.
//! - A plan that holds path orientations (`crate::path_orientation`) corrects its states
//!   onto them with OMPL's projected state space, walks each motion along them, and checks
//!   each straight stretch of the walk as any motion, the walk's states inside the joints'
//!   limits. Each walk of a run or an arc walks each orientation too, so every state of the
//!   rounded path holds it. A corner whose arc breaks one gains waypoints at shares of its
//!   runs that keep the plan's clearances, and the answer's walk checks the runs they split
//!   from those states. A start or a goal that breaks one fails with no path.
//!
//! The request's deadline bounds the plan as MoveIt's allowed planning time bounds a
//! request: OMPL makes the request's attempts before it, each under all of it, each
//! sampling up to `goal_samples` goals as its planner asks for them (`Sampling`,
//! `ompl_bridge::Goals::Sampled`), ending before a sample once the deadline passes, as a
//! planner of MoveIt plans while its goal sampler samples, and its simplifier takes what
//! remains after the attempts, at most `SIMPLIFICATION_ITERATIONS` iterations, as MoveIt
//! simplifies in the allowed planning time less the plan's (moveit2 2.15.2
//! `ModelBasedPlanningContext::solve`), whatever is left of it. OMPL's simplifier runs
//! its first pass however little time it has (`PathSimplifier::simplify`, `atLeastOnce`),
//! so a planner that plans until its time ends, such as RRT*, has its path simplified
//! once, past the deadline. A corner's repair runs only before the deadline: when a
//! corner of the simplified path rounds short of the clearances once it has passed, the
//! plan answers its planned path if that passes its own check, and no path otherwise. The
//! check of the rounded path runs to its end past the deadline.
//!
//! `check_path` checks a caller's path with the same checks: each waypoint after the
//! start keeps the plan's clearances, each run keeps them as the plan's runs keep them,
//! the run from the start by the start rule, and each arc as the plan's arcs; its walks
//! end at the check's deadline.

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use chain_kinematics::{Chain, Draws, Limit};
use control_core::path_timing::{MIN_JOINT_CHANGE_RAD, RoundedPath, distinct_joint_waypoints};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::{
    CheckError, Checks, Ending, GoalSample, GoalSampling, Goals, Motion, Plan, PlanError, Planner,
    Request, SampledGoals, Space, plan,
};

use crate::arm::Moving;
use crate::arms::Arms;
use crate::clearance::{Contact, Reading, Surroundings, TERMS, term_clearances};
use crate::goal::{Goal, GoalSampler};
use crate::path_orientation::{PathOrientation, PathOrientations, Turned};
use crate::scene::Objects;
use crate::stop::is_set;

/// The iterations the simplification of a planned path takes at most.
const SIMPLIFICATION_ITERATIONS: u32 = 1000;

/// The least spare a checked state keeps of each term, as the module says: a walk ends at
/// a state from which a step would move a point less than this. 1 mm: near a surface at
/// the clearance asked of a term the spare, and with it the step, shrinks toward 0, and
/// the floor bounds a motion's checks by its length over 1 mm.
const MIN_SPARE_M: f64 = 1e-3;

/// The farthest any point of a link moves between two checked states. 1.5 cm: a corner's
/// repair adds waypoints at states its runs' walks check, so each run of a path holds a
/// checked state within this of any point of it, and a state's distance from the map and
/// the objects is measured exactly up to `min_clearance_m` plus this, past which a body's
/// bounding sphere bounds it.
pub const MAX_STEP_M: f64 = 0.015;

/// The shortest step of a motion from a start that falls short of its clearances.
const MIN_DEPARTURE_STEP_M: f64 = 1e-4;

/// How many of a term's bodies a motion moves, for each of the terms `clearance` names:
/// the map and the objects stand still, and both bodies of a pair of the robot's own body,
/// or of an attached object and the arm that carries it, may move.
const MOVED: [f64; TERMS] = [1.0, 2.0, 2.0];

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

/// What a plan keeps clear of: the map as it stood at `stamp`, and the objects of the
/// planning scene.
pub struct Scene<'a> {
    pub map: &'a VoxelMap,
    pub field: &'a DistanceField,
    /// The stamp of the last frame the map took in, in nanoseconds. It joins the
    /// request's seed, so the same request on the same map gives the same path.
    pub stamp: u64,
    pub objects: Objects<'a>,
}

impl<'a> Scene<'a> {
    /// The surroundings of a state in the scene at the clearance of each term of
    /// `clearances`.
    fn surroundings(&self, clearances: [f64; TERMS]) -> Surroundings<'a> {
        Surroundings::new(self.map, self.field, clearances).with_objects(self.objects)
    }
}

/// The goal of one arm of a plan: the constraints its last waypoint meets, and what finds
/// configurations of the arm meeting them.
pub struct ArmGoal<'a, const N: usize> {
    pub goal: &'a Goal,
    pub sampler: &'a dyn GoalSampler<N>,
}

/// One plan of the path of the arms of a group, `N` joints each and `D` in all.
pub struct PlanRequest<'a, const N: usize, const D: usize> {
    /// Where the arms stand.
    pub start: [f64; D],
    /// How far, per joint, the start may lie outside a joint limit and still be moved
    /// onto it.
    pub start_tolerance: [f64; D],
    /// The goal of each arm, in the group's order.
    pub goals: Vec<ArmGoal<'a, N>>,
    /// The orientations every state of the path holds, fewer than `D`, in no order.
    pub path_orientations: &'a [PathOrientation<'a>],
    /// The most goals each attempt of the plan samples, as MoveIt's `max_goal_samples`
    /// bounds the goal states its goal sampler adds (`Sampling`).
    pub goal_samples: NonZeroUsize,
    pub planner: Planner,
    /// The instant the goal sampling, the planner's plan and its simplification end by,
    /// as the module says.
    pub deadline: Instant,
    pub max_iterations: NonZeroU64,
    /// The attempts OMPL makes, as MoveIt's `num_planning_attempts`: each from its own
    /// seed, under all the time before the deadline, their paths joined by OMPL's path
    /// hybridization.
    pub attempts: NonZeroU32,
    pub seed: u32,
    /// The clearance each state keeps, as `term_clearances` asks it of each term.
    pub min_clearance_m: f64,
    /// A flag the caller sets to end the plan, at any part of it.
    pub stop: Option<&'a AtomicBool>,
}

/// A path of the arms of a group to check, as limb_motion's execute_joint_path runs it.
pub struct CheckRequest<'a, const D: usize> {
    /// Where the arms stand.
    pub start: [f64; D],
    /// How far, per joint, the start may lie outside a joint limit and still be moved
    /// onto it, as a plan's.
    pub start_tolerance: [f64; D],
    /// The waypoints after the start, at least one, each holding every arm's joints.
    pub waypoints: &'a [[f64; D]],
    /// The clearance each state keeps, as `term_clearances` asks it of each term.
    pub min_clearance_m: f64,
    /// The instant the check ends by.
    pub deadline: Instant,
}

/// Why a check of a path gives no answer.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum CheckFailure {
    #[error("{0}")]
    Request(String),
    #[error("the check of the path ran past its deadline")]
    TimeEnded,
}

/// Why a path cannot start where it starts.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum StartFault {
    /// The joint, by its URDF name, that starts outside its limits.
    #[error(
        "{joint} starts at {position}, outside its limits {low} to {high} by more than its start tolerance {tolerance}"
    )]
    OutsideLimits {
        joint: String,
        position: f64,
        low: f64,
        high: f64,
        tolerance: f64,
    },
    #[error("the start is in contact: {0}")]
    InCollision(Contact),
    /// The nearest pair of a body and the map or an object, out of contact, nearer than
    /// `min_clearance_m` at the start, and that clearance (m).
    #[error("the start keeps {reading}, inside the clearance {clearance:.4} m")]
    InsideClearance { reading: Reading, clearance: f64 },
}

/// What a check of a path finds.
#[derive(Debug, PartialEq)]
pub struct PathCheck {
    /// The most waypoints, from the first, whose path, run on its own, passes: all of
    /// them when the whole path passes, and 0 when the start or the path to the first
    /// waypoint fails.
    pub clear_waypoints: usize,
    /// The first fault along the path; `None` when the whole path passes.
    pub fault: Option<PathFault>,
}

/// A fault of a path.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum PathFault {
    #[error(transparent)]
    Start(StartFault),
    /// Waypoint `waypoint`, counted from 1 after the start, holds `joint`, by its URDF
    /// name, outside its limits.
    #[error("waypoint {waypoint} holds {joint} at {position}, outside its limits {low} to {high}")]
    OutsideLimits {
        waypoint: usize,
        joint: String,
        position: f64,
        low: f64,
        high: f64,
    },
    /// A state at `place` of a path of `arms` arms falls short of a plan clearance: it
    /// lies `distances` meters from the bodies of each term, and the plan keeps
    /// `clearances`.
    #[error("{place}, {}", nearness(*arms, distances, clearances))]
    Clearance {
        place: Place,
        arms: usize,
        distances: [f64; TERMS],
        clearances: [f64; TERMS],
    },
}

/// Where on a path a state falls short of its clearances, by its waypoints counted from 1
/// after the start, the start 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Place {
    /// At waypoint `k`.
    Waypoint(usize),
    /// On the run from waypoint `k` to the next.
    Run(usize),
    /// On the arc that rounds the corner at waypoint `k`.
    Corner(usize),
}

impl std::fmt::Display for Place {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Waypoint(k) => write!(f, "at waypoint {k}"),
            Self::Run(0) => write!(f, "on the run from the start"),
            Self::Run(k) => write!(f, "on the run from waypoint {k}"),
            Self::Corner(k) => write!(f, "on the corner rounded at waypoint {k}"),
        }
    }
}

/// How near a state of a path of `arms` arms comes that lies `distances` meters from the
/// bodies of each term, of each term it keeps less than its plan clearance of
/// `clearances`.
fn nearness(arms: usize, distances: &[f64; TERMS], clearances: &[f64; TERMS]) -> String {
    let [world, own, held] = *distances;
    let [world_clearance, own_clearance, held_clearance] = *clearances;
    let moving = if arms == 1 {
        "the arm comes"
    } else {
        "the arms come"
    };
    let terms = [
        (world < world_clearance).then(|| {
            format!(
                "{moving} {world:.4} m from the map and the objects, nearer than the plan's \
                 clearance, {world_clearance:.4} m"
            )
        }),
        (own < own_clearance).then(|| {
            format!(
                "the robot's own links and what they carry come {own:.4} m apart, nearer \
                 than the plan's clearance, {own_clearance:.4} m"
            )
        }),
        (held < held_clearance).then(|| {
            format!(
                "an attached object comes {held:.4} m from a link of the arm that carries it, \
                 nearer than the plan's clearance, {held_clearance:.4} m"
            )
        }),
    ];
    terms
        .into_iter()
        .flatten()
        .collect::<Vec<_>>()
        .join(", and ")
}

/// Why a plan gives no path.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum PlanFailure {
    #[error("{0}")]
    Request(String),
    #[error(transparent)]
    Start(#[from] StartFault),
    #[error("{0}")]
    Unreachable(String),
    #[error("{0}")]
    NoPath(String),
    #[error("the plan was stopped")]
    Stopped,
    #[error(transparent)]
    Planner(#[from] PlanError),
}

/// Plans `request` for `arms` around `scene`: the waypoints of a path from the start to
/// a state at which each arm meets its goal, whose rounded path, as execute_joint_path
/// rounds it, keeps its clearance.
///
/// # Panics
///
/// When `request` holds a goal for other than each arm of `arms`, or a path orientation of
/// an arm outside them.
pub fn plan_path<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    scene: &Scene,
    request: &PlanRequest<N, D>,
) -> Result<Vec<[f64; D]>, PlanFailure> {
    assert_eq!(
        request.goals.len(),
        arms.arms().len(),
        "a plan holds one goal for each arm"
    );
    distances_in_range(request.min_clearance_m, &request.start_tolerance)
        .map_err(PlanFailure::Request)?;
    let start = start_clear(
        arms,
        scene,
        &request.start,
        &request.start_tolerance,
        request.min_clearance_m,
    )?;
    let orientations = PathOrientations::new(arms, request.path_orientations, &start)?;
    if let Some(apart) = (request.goals.iter().enumerate())
        .find_map(|(k, goal)| orientations.goal_apart(k, goal.goal))
    {
        return Err(PlanFailure::NoPath(apart));
    }
    let checks = PathChecks::new(arms, scene, start, request.min_clearance_m).holding(orientations);
    let deadline = request.deadline;
    let remaining = deadline.saturating_duration_since(Instant::now());
    if remaining.is_zero() {
        return Err(PlanFailure::NoPath(format!(
            "the deadline passed before {} planned",
            request.planner
        )));
    }
    let answer = plan_to_sampled_goals(arms, scene, &checks, request, &start, remaining)?;
    let (Some(planned), Some(simplification)) = (answer.path, answer.simplified) else {
        return Err(PlanFailure::NoPath(format!(
            "{} found no path within {} iterations or by its deadline, keeping {}{}",
            request.planner,
            request.max_iterations,
            checks.plan_clearance(),
            checks.orientations.kept()
        )));
    };
    answer_of(
        &checks,
        &planned,
        &simplification.path,
        deadline,
        request.stop,
    )
}

/// The answer of a plan whose planner found `planned` and simplified it to `simplified`,
/// as the module says: the simplified path's when the path the robot runs through it
/// passes its check, and the planned path's when only that one passes.
fn answer_of<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    planned: &[Vec<f64>],
    simplified: &[Vec<f64>],
    deadline: Instant,
    stop: Option<&AtomicBool>,
) -> Result<Vec<[f64; D]>, PlanFailure> {
    answered(checks, simplified, deadline, stop).or_else(|failure| match failure {
        PlanFailure::NoPath(_) if planned != simplified => {
            answered(checks, planned, deadline, stop)
        }
        other => Err(other),
    })
}

/// The answer of a plan whose planner found `path`: its waypoints, the waypoints added
/// near its corners among them, when the path the robot runs through them passes its
/// check, or why it does not. A set `stop` ends the check before its next run or corner.
fn answered<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &[Vec<f64>],
    deadline: Instant,
    stop: Option<&AtomicBool>,
) -> Result<Vec<[f64; D]>, PlanFailure> {
    let waypoints: Vec<[f64; D]> = path
        .iter()
        .map(|state| {
            state
                .as_slice()
                .try_into()
                .expect("a state of the arms' space")
        })
        .collect();
    let distinct = distinct_joint_waypoints(&waypoints);
    let kept = if checks.orientations.is_empty() {
        distinct
    } else {
        without_straight_waypoints(&distinct)
    };
    let answer = rounded_clear(checks, &kept, deadline, stop)?;
    let turning = first_turning_run(checks, &answer, &|| is_set(stop))
        .map_err(|Ended| PlanFailure::Stopped)?;
    if let Some((k, turned)) = turning {
        return Err(PlanFailure::NoPath(format!(
            "the planned path falls short {}; send a wider tolerance_rad",
            checks.turned(Place::Run(k), &turned)
        )));
    }
    match first_failing_run(checks, &answer, &|| is_set(stop)) {
        Err(Ended) => Err(PlanFailure::Stopped),
        Ok(None) => Ok(answer),
        Ok(Some((k, broken))) => Err(PlanFailure::NoPath(format!(
            "the planned path falls short {}",
            checks.fault(Place::Run(k), broken.distances)
        ))),
    }
}

/// `waypoints` without each that lies on the straight segment from the waypoint kept before
/// it to the one after it: a walk along a path orientation spaces waypoints along each
/// stretch the correction leaves straight, which the robot runs as one.
fn without_straight_waypoints<const D: usize>(waypoints: &[[f64; D]]) -> Vec<[f64; D]> {
    let Some((first, rest)) = waypoints.split_first() else {
        return Vec::new();
    };
    let mut kept = vec![*first];
    for (k, at) in rest.iter().enumerate() {
        let before = kept.last().expect("the first waypoint is kept");
        if !rest
            .get(k + 1)
            .is_some_and(|next| on_segment(at, before, next))
        {
            kept.push(*at);
        }
    }
    kept
}

/// Checks the path of `request` for `arms` around `scene` with the checks of a plan's
/// answer, as the module says: the start inside its limits by its tolerance and out of
/// contact, each waypoint inside the joint limits and, after the start, at the plan's
/// clearances, each run as the plan walks the runs of its answer, the first from the
/// start by the start rule, and each arc of the rounded path as the plan walks its arcs.
/// The walks follow the run into the first waypoint outside a limit up to where it leaves
/// the limits, and go no farther. The fault nearest the start along the path leaves the
/// waypoints before it passing:
/// one on the run from a waypoint, or on the corner rounded at it, the waypoints up to
/// that one, the start counted as waypoint 0; one at a waypoint, those before it. A walk
/// of a run or a corner that starts past the deadline ends the check.
pub fn check_path<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    scene: &Scene,
    request: &CheckRequest<D>,
) -> Result<PathCheck, CheckFailure> {
    check_path_until(arms, scene, request, &|| Instant::now() >= request.deadline)
}

/// `check_path`, its walks of a run or a corner ended once `ended` holds before them, in
/// place of `request`'s deadline.
pub(crate) fn check_path_until<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    scene: &Scene,
    request: &CheckRequest<D>,
    ended: &dyn Fn() -> bool,
) -> Result<PathCheck, CheckFailure> {
    distances_in_range(request.min_clearance_m, &request.start_tolerance)
        .map_err(CheckFailure::Request)?;
    if request.waypoints.is_empty() {
        return Err(CheckFailure::Request(
            "the path holds no waypoint after its start: give one or more".to_string(),
        ));
    }
    let start = match start_clear(
        arms,
        scene,
        &request.start,
        &request.start_tolerance,
        request.min_clearance_m,
    ) {
        Ok(start) => start,
        Err(fault) => {
            return Ok(PathCheck {
                clear_waypoints: 0,
                fault: Some(PathFault::Start(fault)),
            });
        }
    };
    let checks = PathChecks::new(arms, scene, start, request.min_clearance_m);
    let limits = arms.limits();
    let path: Vec<[f64; D]> = std::iter::once(start)
        .chain(
            request
                .waypoints
                .iter()
                .map(|waypoint| as_targets(&limits, waypoint)),
        )
        .collect();
    // Each fault with the waypoints its path leaves passing, and its rank among the
    // faults that leave as many: a corner's arc comes before the rest of the run from its
    // waypoint, and that run before the waypoint it ends at. The walks follow the run
    // into the first waypoint outside a limit up to where it leaves the limits, so a
    // fault the arms meet on their way there comes first.
    let outside = first_outside_limits(arms, &path);
    let (inside, walked) = match &outside {
        None => (path.as_slice(), path.clone()),
        Some((k, _)) => {
            let leaving = where_it_leaves_the_limits(arms, &path[k - 1], &path[*k]);
            (&path[..*k], [&path[..*k], &[leaving]].concat())
        }
    };
    let walked = walked.as_slice();
    let outside = outside.map(|(k, fault)| (k - 1, 2, fault));
    let short = first_short_waypoint(&checks, inside)
        .map(|(k, distances)| (k - 1, 2, checks.fault(Place::Waypoint(k), distances)));
    let on_run = first_failing_run(&checks, walked, ended)
        .map_err(|Ended| CheckFailure::TimeEnded)?
        .map(|(k, broken)| (k, 1, checks.fault(Place::Run(k), broken.distances)));
    let each_its_own: Vec<usize> = (0..walked.len()).collect();
    let on_corner = first_failing_probe(
        &checks,
        &RoundedPath::joint_path(&vectors(walked)),
        &each_its_own,
        walked,
        ended,
    )
    .map_err(|Ended| CheckFailure::TimeEnded)?
    .map(|(k, broken)| (k, 0, checks.fault(Place::Corner(k), broken.distances)));
    let first = [outside, short, on_run, on_corner]
        .into_iter()
        .flatten()
        .min_by_key(|(passing, rank, _)| (*passing, *rank));
    Ok(match first {
        None => PathCheck {
            clear_waypoints: request.waypoints.len(),
            fault: None,
        },
        Some((passing, _, fault)) => PathCheck {
            clear_waypoints: passing,
            fault: Some(fault),
        },
    })
}

/// Refuses `min_clearance_m` or a start tolerance that is no distance, saying why.
pub(crate) fn distances_in_range<const N: usize>(
    min_clearance_m: f64,
    start_tolerance: &[f64; N],
) -> Result<(), String> {
    let distance = |what: &str, value: f64| {
        (value.is_finite() && value >= 0.0)
            .then_some(())
            .ok_or_else(|| format!("{what} is {value}; it is a finite distance of 0 or more"))
    };
    distance("min_clearance_m", min_clearance_m)?;
    start_tolerance
        .iter()
        .try_for_each(|&t| distance("a start tolerance", t))
}

/// The start a plan or a check of `arms` takes from `start`: each arm's joints inside its
/// joint limits by `tolerance`, the arms in the group's order, out of contact with the map,
/// the objects and the robot's own body, and `min_clearance_m` or farther from the map and
/// the objects, as MoveIt's collision check, padded by the clearance, refuses a start in
/// collision.
pub(crate) fn start_clear<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    scene: &Scene,
    start: &[f64; D],
    tolerance: &[f64; D],
    min_clearance_m: f64,
) -> Result<[f64; D], StartFault> {
    let inside = arms
        .arms()
        .iter()
        .enumerate()
        .map(|(k, arm)| {
            start_inside_limits(
                arm.geometry().chain(),
                &Arms::<N, D>::slice(start, k),
                &Arms::<N, D>::slice(tolerance, k),
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let start = Arms::<N, D>::join(&inside);
    if let Some(contact) = scene.surroundings([0.0; TERMS]).contact(arms, &start) {
        return Err(StartFault::InCollision(contact));
    }
    let nearest = scene
        .surroundings(term_clearances(min_clearance_m))
        .nearest_in_the_world(arms, &start);
    match nearest.filter(|reading| reading.distance < min_clearance_m) {
        Some(reading) => Err(StartFault::InsideClearance {
            reading,
            clearance: min_clearance_m,
        }),
        None => Ok(start),
    }
}

/// `waypoint` as execute_joint_path runs it: each joint at most
/// [`TARGET_SLACK`](chain_kinematics::TARGET_SLACK) outside
/// its limit at the limit ([`Limit::take_target`]), and each other joint where it is.
fn as_targets<const D: usize>(limits: &[Limit; D], waypoint: &[f64; D]) -> [f64; D] {
    std::array::from_fn(|i| limits[i].take_target(waypoint[i]).unwrap_or(waypoint[i]))
}

/// The first waypoint of `path`, the start first, that holds a joint outside the limits of
/// `arms`' chains, by its index, and the fault.
fn first_outside_limits<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    path: &[[f64; D]],
) -> Option<(usize, PathFault)> {
    let limits = arms.limits();
    path.iter().enumerate().skip(1).find_map(|(k, q)| {
        let joint = (0..D).find(|&i| !(limits[i].lo..=limits[i].hi).contains(&q[i]))?;
        Some((
            k,
            PathFault::OutsideLimits {
                waypoint: k,
                joint: arms.joint_name(joint).to_string(),
                position: q[joint],
                low: limits[joint].lo,
                high: limits[joint].hi,
            },
        ))
    })
}

/// The state where the straight run from `from`, inside the joint limits of `arms`'
/// chains, to `to`, outside them, leaves them: the first limit it crosses, held on it. A
/// `to` with a value that is not finite leaves them at `from`.
fn where_it_leaves_the_limits<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    from: &[f64; D],
    to: &[f64; D],
) -> [f64; D] {
    let limits = arms.limits();
    let share = (0..D)
        .filter(|&i| !(limits[i].lo..=limits[i].hi).contains(&to[i]))
        .map(|i| {
            let bound = if to[i] > limits[i].hi {
                limits[i].hi
            } else {
                limits[i].lo
            };
            ((bound - from[i]) / (to[i] - from[i])).max(0.0)
        })
        .fold(1.0, f64::min);
    if share == 0.0 {
        return *from;
    }
    let leaving = toward(from, to, share);
    std::array::from_fn(|i| leaving[i].clamp(limits[i].lo, limits[i].hi))
}

/// The floor a start `distance` meters from the bodies of a term asked to keep `asked`,
/// with `moved` moving bodies, sets, as the module says.
fn floor_of(distance: f64, asked: f64, moved: f64) -> f64 {
    if distance < asked {
        return 0.0;
    }
    asked
        .min(distance - 2.0 * moved * (MIN_DEPARTURE_STEP_M + ROUNDING_SLACK_M))
        .max(0.0)
}

/// One seed from `seed` and `stamp`: the bits of both mixed, never zero.
fn mixed_seed(seed: u32, stamp: u64) -> NonZeroU32 {
    let mut draws = Draws::new(u64::from(seed) ^ stamp.rotate_left(32));
    let mixed = (draws.next_bits() >> 32) as u32;
    NonZeroU32::new(mixed).unwrap_or(NonZeroU32::MIN)
}

/// `start`, each joint outside the limits of `chain` by no more than its tolerance moved
/// onto the limit; a start farther out is refused. A plan and a check start where this
/// puts them, and a caller asks it before it takes a request.
pub fn start_inside_limits<const N: usize>(
    chain: &Chain<N>,
    start: &[f64; N],
    tolerance: &[f64; N],
) -> Result<[f64; N], StartFault> {
    let limits = chain.limits();
    (0..N)
        .map(|i| {
            let (low, high) = (limits[i].lo, limits[i].hi);
            let position = start[i];
            if !(low - tolerance[i]..=high + tolerance[i]).contains(&position) {
                return Err(StartFault::OutsideLimits {
                    joint: chain.joint_name(i).to_string(),
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

/// OMPL's plan of `request` from `start` around `scene` under `checks`, `remaining` before the
/// request's deadline, whose attempts each sample the states at which each arm meets its goal and
/// that keep their clearance as their planners ask (`PlanGoals`), the plan's clearance check
/// deciding inside each sample; or why it has none: no sample found a goal, or the stop flag
/// was set. Each turn of a sample judges the whole state from a start that keeps its
/// clearances, as MoveIt's validity callback does, and from a start nearer than the plan's
/// clearance, which the start rule plans from, the pairs `Moving::Arm` names of the turn's
/// arm. Once a sample of any attempt has found a goal, an attempt that found none answers no
/// path, whichever attempt it is. An attempt of a plan that holds orientations samples each goal
/// alone after each sample that misses (`GoalsAlone`), so that a plan with no goal says which of
/// the two the caller may change: a state of the goals alone is no path, and none is
/// unreachable, as a plan that holds nothing answers.
fn plan_to_sampled_goals<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    scene: &Scene,
    checks: &PathChecks<N, D>,
    request: &PlanRequest<N, D>,
    start: &[f64; D],
    remaining: Duration,
) -> Result<Plan, PlanFailure> {
    let deadline = request.deadline;
    let ended = || Instant::now() >= deadline || is_set(request.stop);
    // The rejections of states that fall short of the clearances in the pairs a turn
    // judges: how many, and the state that falls short least, with those pairs.
    let short = Mutex::new((0_usize, None::<([f64; D], Moving, f64)>));
    // The rejections of states clear of collisions that lie past a path orientation's
    // allowed angle: how many, and the one nearest it.
    let turned = Mutex::new((0_usize, None::<Turned>));
    let start_keeps = !checks.start_short.contains(&true);
    let moving_of = |turn: Moving| if start_keeps { Moving::All } else { turn };
    let clear_of_bodies = |q: &[f64; D], turn: Moving| {
        let moving = moving_of(turn);
        if checks.surroundings.clear_moving(arms, q, moving) {
            return true;
        }
        let shortfall = checks.shortfall(q, moving);
        let mut short = short.lock().unwrap_or_else(PoisonError::into_inner);
        short.0 += 1;
        if short.1.is_none_or(|(_, _, least)| shortfall < least) {
            short.1 = Some((*q, moving, shortfall));
        }
        false
    };
    let clear = |q: &[f64; D], turn: Moving| {
        if !clear_of_bodies(q, turn) {
            return false;
        }
        let Some(past) = checks.orientations.past_allowed(q, turn) else {
            return true;
        };
        let mut turned = turned.lock().unwrap_or_else(PoisonError::into_inner);
        turned.0 += 1;
        if turned.1.is_none_or(|nearest| past.error < nearest.error) {
            turned.1 = Some(past);
        }
        false
    };
    let recording: Vec<Recording<N>> = request
        .goals
        .iter()
        .map(|goal| Recording {
            sampler: goal.sampler,
            found: AtomicBool::new(false),
        })
        .collect();
    let held_goals: Vec<Goal> = request
        .goals
        .iter()
        .enumerate()
        .map(|(k, goal)| goal.goal.holding(&checks.orientations.targets_of(k)))
        .collect();
    let goals: Vec<ArmGoal<N>> = held_goals
        .iter()
        .zip(&recording)
        .map(|(goal, sampler)| ArmGoal { goal, sampler })
        .collect();
    let goals_alone: Vec<ArmGoal<N>> = (request.goals.iter())
        .zip(&recording)
        .map(|(goal, sampler)| ArmGoal {
            goal: goal.goal,
            sampler,
        })
        .collect();
    let found_a_goal = AtomicBool::new(false);
    let goals_alone_found = AtomicBool::new(false);
    let sampled = PlanGoals {
        goals: &goals,
        start: *start,
        goal_samples: request.goal_samples,
        seed: u64::from(request.seed),
        ended: &ended,
        valid: &clear,
        found_a_goal: &found_a_goal,
        goals_alone: (!checks.orientations.is_empty()).then_some(GoalsAlone {
            goals: &goals_alone,
            valid: &clear_of_bodies,
            found: &goals_alone_found,
        }),
    };
    let bounds: Vec<(f64, f64)> = arms.limits().iter().map(|l| (l.lo, l.hi)).collect();
    let space = Space::new(&bounds).map_err(|e| PlanFailure::NoPath(e.to_string()))?;
    let answer = match plan(
        &Request {
            planner: request.planner,
            space: &space,
            start,
            goals: Goals::Sampled(&sampled),
            time_limit: remaining,
            iterations: request.max_iterations,
            seed: mixed_seed(request.seed, scene.stamp),
            attempts: request.attempts,
            simplification: NonZeroU32::new(SIMPLIFICATION_ITERATIONS),
            simplification_time: Duration::ZERO,
            projection: checks.orientations.projection(),
            stop: request.stop,
        },
        checks,
    ) {
        Err(PlanError::InvalidGoal) => Plan {
            path: None,
            simplified: None,
            ended_by: Ending::Returned,
        },
        Err(failure) => return Err(failure.into()),
        Ok(answer) => answer,
    };
    if is_set(request.stop) {
        return Err(PlanFailure::Stopped);
    }
    if found_a_goal.load(Ordering::Relaxed) {
        return Ok(answer);
    }
    let several = arms.arms().len() > 1;
    let more_time = if Instant::now() >= deadline {
        "; the deadline ended the sampling, and more time may find a goal"
    } else {
        ""
    };
    let (past_allowed, nearest) = turned.into_inner().unwrap_or_else(PoisonError::into_inner);
    if let Some(nearest) = nearest {
        let (link, tolerance) = checks.orientations.named(nearest.orientation);
        return Err(PlanFailure::NoPath(format!(
            "the goal sampling found {past_allowed} configurations that meet the goal clear of \
             collisions, each breaking a path orientation; the nearest holds {link} {:.4} rad \
             from it, and a goal stands within {:.4} rad of its tolerance_rad {tolerance:.4}; \
             send a goal nearer it or a wider tolerance_rad{more_time}",
            nearest.error,
            checks.orientations.allowed(nearest.orientation)
        )));
    }
    if goals_alone_found.load(Ordering::Relaxed) {
        return Err(PlanFailure::NoPath(format!(
            "the goal can be met, and the goal sampling found no configuration of it that keeps \
             the path orientations of {}; send a goal nearer them or a wider tolerance_rad{more_time}",
            checks.orientations.links()
        )));
    }
    let (rejections, least_short) = short.into_inner().unwrap_or_else(PoisonError::into_inner);
    let Some((least_short, moving, _)) = least_short else {
        let of = arms
            .arms()
            .iter()
            .zip(&recording)
            .find(|(_, sampler)| !sampler.found.load(Ordering::Relaxed))
            .filter(|_| several)
            .map_or(String::new(), |(arm, _)| format!(" of {}", arm.name()));
        return Err(PlanFailure::Unreachable(format!(
            "inverse kinematics found no configuration{of} that meets the goal in {MAX_GOAL_SAMPLING_ATTEMPTS} goal samples or by the deadline{more_time}"
        )));
    };
    let tried = if several {
        "states of the arms"
    } else {
        "configurations that meet the goal"
    };
    Err(PlanFailure::Unreachable(format!(
        "the goal sampling rejected {rejections} {tried}, each nearer than {}; the one that falls short least has {}{more_time}",
        checks.plan_clearance(),
        checks.short_of(&least_short, moving)
    )))
}

/// Each goal of a plan that holds orientations without them. An attempt samples them after each
/// sample of its own that misses, until a sample of either finds a state, so a plan whose goal
/// its first samples find samples nothing more.
struct GoalsAlone<'p, const N: usize, const D: usize> {
    goals: &'p [ArmGoal<'p, N>],
    valid: &'p (dyn Fn(&[f64; D], Moving) -> bool + Sync),
    /// Whether a sample of an attempt has found a state of them.
    found: &'p AtomicBool,
}

/// The goals of a plan's attempts, each attempt sampling its own (`Sampling`): attempt 0 from
/// `seed`, as `sample_goals` samples from it, and each other from the seeds after those of
/// the attempts before it, one for each arm.
struct PlanGoals<'p, const N: usize, const D: usize> {
    goals: &'p [ArmGoal<'p, N>],
    start: [f64; D],
    goal_samples: NonZeroUsize,
    seed: u64,
    ended: &'p (dyn Fn() -> bool + Sync),
    valid: &'p (dyn Fn(&[f64; D], Moving) -> bool + Sync),
    /// Whether a sample of an attempt has found a goal.
    found_a_goal: &'p AtomicBool,
    goals_alone: Option<GoalsAlone<'p, N, D>>,
}

impl<const N: usize, const D: usize> SampledGoals for PlanGoals<'_, N, D> {
    fn sampling(&self, attempt: u32) -> Box<dyn GoalSampling + '_> {
        let seed = self.seed + u64::from(attempt) * self.goals.len() as u64;
        Box::new(AttemptSampling {
            sampling: Sampling::new(
                self.goals,
                &self.start,
                self.goal_samples,
                seed,
                self.ended,
                self.valid,
            ),
            found_a_goal: self.found_a_goal,
            alone: self.goals_alone.as_ref().map(|alone| {
                let sampling = Sampling::new(
                    alone.goals,
                    &self.start,
                    NonZeroUsize::MIN,
                    seed,
                    self.ended,
                    alone.valid,
                );
                (sampling, alone.found)
            }),
        })
    }
}

/// The goal sampling of one attempt of a plan.
struct AttemptSampling<'p, const N: usize, const D: usize> {
    sampling: Sampling<'p, N, D>,
    found_a_goal: &'p AtomicBool,
    /// The sampling of the goals alone, and whether a sample of an attempt has found a state.
    alone: Option<(Sampling<'p, N, D>, &'p AtomicBool)>,
}

impl<const N: usize, const D: usize> AttemptSampling<'_, N, D> {
    /// One sample of the goals alone, while no sample of either has found a state.
    fn sample_goals_alone(&mut self) {
        let Some((alone, found)) = self.alone.as_mut() else {
            return;
        };
        if self.found_a_goal.load(Ordering::Relaxed) || found.load(Ordering::Relaxed) {
            return;
        }
        if matches!(alone.sample(), Sampled::Goal(_)) {
            found.store(true, Ordering::Relaxed);
        }
    }
}

impl<const N: usize, const D: usize> GoalSampling for AttemptSampling<'_, N, D> {
    fn sample(&mut self, goal: &mut [f64]) -> Result<GoalSample, CheckError> {
        Ok(match self.sampling.sample() {
            Sampled::Goal(found) => {
                self.found_a_goal.store(true, Ordering::Relaxed);
                goal.copy_from_slice(&found);
                GoalSample::Found
            }
            Sampled::Missed => {
                self.sample_goals_alone();
                GoalSample::Missed
            }
            Sampled::Ended => GoalSample::Ended,
        })
    }
}

/// An arm's goal sampler that records whether it found a configuration.
struct Recording<'a, const N: usize> {
    sampler: &'a dyn GoalSampler<N>,
    found: AtomicBool,
}

impl<const N: usize> GoalSampler<N> for Recording<'_, N> {
    fn sample_from(
        &self,
        goal: &Goal,
        working: &[f64; N],
        attempts: usize,
        draws: &mut Draws,
        valid: &dyn Fn(&[f64; N]) -> bool,
    ) -> Option<[f64; N]> {
        let found = self
            .sampler
            .sample_from(goal, working, attempts, draws, valid);
        if found.is_some() {
            self.found.store(true, Ordering::Relaxed);
        }
        found
    }
}

/// The attempts of each goal sample: MoveIt's `max_state_sampling_attempts` default
/// (moveit2 2.15.2 planning_context_manager.cpp:259), the attempts its
/// `ConstrainedGoalSampler` gives its constraint sampler
/// (constrained_goal_sampler.cpp:137).
const MAX_STATE_SAMPLING_ATTEMPTS: usize = 4;

/// MoveIt's `max_goal_sampling_attempts` default (planning_context_manager.cpp:260), which
/// bounds the goal samples in a row that find no goal, as `Sampling` says.
const MAX_GOAL_SAMPLING_ATTEMPTS: usize = 1000;

/// What one goal sample of a `Sampling` found.
enum Sampled<const D: usize> {
    /// A goal, farther than the double's epsilon from each goal before it.
    Goal([f64; D]),
    /// No new goal; a later sample may find one.
    Missed,
    /// No sample ran: the sampling has ended.
    Ended,
}

/// The goal sampling of each arm of `goals`, a group of `D` joints in all, one sample at a
/// time, from `start`, as MoveIt's `ConstrainedGoalSampler` finds the goals of a group
/// (moveit2 2.15.2 constrained_goal_sampler.cpp). A working state starts at `start` (:66).
/// Each goal sample runs from it (`sample_group`), each arm drawing from the draws of
/// `seed` plus its place in the group, `valid` deciding inside it as MoveIt's validity
/// callback does (:84-94, :126-137), asked of the pairs `sample_group` names. The state a
/// sample finds becomes the working state (:137), and joins the goals when it lies farther
/// than the double's epsilon from each of them in the sum of its joints' differences, as
/// OMPL adds a goal state (`GoalLazySamples::addStateIfDifferent` at its default `minDist`,
/// GoalLazySamples.cpp:205-215, :211, GoalLazySamples.h:101) in MoveIt's distance of a
/// group's states (`JointModelGroup::distance`, joint_model_group.cpp:466-476). Sampling
/// ends at `goal_samples` goals (:105-107), before a sample once `ended` holds, and once
/// the samples since the last goal found reach `MAX_GOAL_SAMPLING_ATTEMPTS` less the goals
/// found so far: OMPL counts each goal its sampling function returns
/// (GoalLazySamples.cpp:114-116), and MoveIt's function samples from that count up to
/// `MAX_GOAL_SAMPLING_ATTEMPTS` until a sample finds a goal, and ends the sampling when
/// none does (constrained_goal_sampler.cpp:98-114, :169).
struct Sampling<'s, const N: usize, const D: usize> {
    goals: &'s [ArmGoal<'s, N>],
    goal_samples: NonZeroUsize,
    ended: &'s (dyn Fn() -> bool + Sync),
    valid: &'s (dyn Fn(&[f64; D], Moving) -> bool + Sync),
    draws: Vec<Draws>,
    working: [f64; D],
    kept: Vec<[f64; D]>,
    /// The goals the samples have found, OMPL's `samplingAttempts_`.
    found_so_far: usize,
    /// The samples since the last goal found.
    failing: usize,
}

impl<'s, const N: usize, const D: usize> Sampling<'s, N, D> {
    /// # Panics
    ///
    /// When `goals` holds other than one goal for each arm of `N` joints of the `D`.
    fn new(
        goals: &'s [ArmGoal<'s, N>],
        start: &[f64; D],
        goal_samples: NonZeroUsize,
        seed: u64,
        ended: &'s (dyn Fn() -> bool + Sync),
        valid: &'s (dyn Fn(&[f64; D], Moving) -> bool + Sync),
    ) -> Self {
        assert_eq!(
            goals.len() * N,
            D,
            "a goal for each arm of {N} joints of the {D}"
        );
        Self {
            goals,
            goal_samples,
            ended,
            valid,
            draws: (0..goals.len())
                .map(|k| Draws::new(seed + k as u64))
                .collect(),
            working: *start,
            kept: Vec::new(),
            found_so_far: 0,
            failing: 0,
        }
    }

    fn sample(&mut self) -> Sampled<D> {
        if self.kept.len() >= self.goal_samples.get()
            || self.failing >= MAX_GOAL_SAMPLING_ATTEMPTS.saturating_sub(self.found_so_far)
            || (self.ended)()
        {
            return Sampled::Ended;
        }
        let Some(found) = sample_group(self.goals, &self.working, &mut self.draws, self.valid)
        else {
            self.failing += 1;
            return Sampled::Missed;
        };
        (self.found_so_far, self.failing) = (self.found_so_far + 1, 0);
        self.working = found;
        let distance = |goal: &[f64; D]| (0..D).map(|i| (goal[i] - found[i]).abs()).sum::<f64>();
        if self.kept.iter().any(|goal| distance(goal) <= f64::EPSILON) {
            return Sampled::Missed;
        }
        self.kept.push(found);
        Sampled::Goal(found)
    }
}

/// The states at which each arm of `goals`, a group of `D` joints in all, meets its goal,
/// found from `start` by a `Sampling` of them sampled until it ends.
///
/// # Panics
///
/// When `goals` holds other than one goal for each arm of `N` joints of the `D`.
pub fn sample_goals<const N: usize, const D: usize>(
    goals: &[ArmGoal<'_, N>],
    start: &[f64; D],
    goal_samples: NonZeroUsize,
    seed: u64,
    ended: &(dyn Fn() -> bool + Sync),
    valid: &(dyn Fn(&[f64; D], Moving) -> bool + Sync),
) -> Vec<[f64; D]> {
    let mut sampling = Sampling::new(goals, start, goal_samples, seed, ended, valid);
    while !matches!(sampling.sample(), Sampled::Ended) {}
    sampling.kept
}

/// One goal sample of the arms of `goals` from the working state `working`, as MoveIt's
/// `UnionConstraintSampler::sample` samples a group from its arms' samplers (moveit2 2.15.2
/// union_constraint_sampler.cpp): each arm in turn (`GoalSampler::sample_from`), from its
/// joints of the state the sample has reached, drawing from its `draws`, `valid` deciding on
/// that state with the arm at the configuration tried, the arms before it at their sampled
/// joints and the arms after it at `working`'s, asked of the pairs of the arm's turn
/// (`Moving::Arm`): each that holds a body of the arm and none of an arm after it. A pair
/// with an arm after it is asked on that arm's turn, where both stand as the state the
/// sample finds holds them, so that state keeps each pair `valid` judges of it. A caller
/// that judges the whole state on every turn samples as MoveIt's validity callback does.
/// None when an arm finds none.
fn sample_group<const N: usize, const D: usize>(
    goals: &[ArmGoal<'_, N>],
    working: &[f64; D],
    draws: &mut [Draws],
    valid: &dyn Fn(&[f64; D], Moving) -> bool,
) -> Option<[f64; D]> {
    let mut state = *working;
    for (k, (goal, draws)) in goals.iter().zip(draws.iter_mut()).enumerate() {
        let placed = |q: &[f64; N]| {
            let mut placed = state;
            placed[k * N..(k + 1) * N].copy_from_slice(q);
            placed
        };
        let found = goal.sampler.sample_from(
            goal.goal,
            &Arms::<N, D>::slice(&state, k),
            MAX_STATE_SAMPLING_ATTEMPTS,
            draws,
            &|q| valid(&placed(q), Moving::Arm(k)),
        )?;
        state = placed(&found);
    }
    Some(state)
}

/// Where a walk breaks: the position of the last state it checks that keeps its
/// clearances, the walk's first among them, and the distances of the state that breaks it
/// from the bodies of each term.
struct Break {
    last_valid: f64,
    distances: [f64; TERMS],
}

/// A walk of a path that its caller ended before it reached the path's end: the plan's
/// stop flag, or the check's deadline.
struct Ended;

/// The checks of the plan's states and motions.
pub(crate) struct PathChecks<'a, const N: usize, const D: usize> {
    arms: &'a Arms<'a, N, D>,
    /// The plan's clearances.
    surroundings: Surroundings<'a>,
    /// The surroundings at which a state's distances are measured, exactly up to the
    /// distance a step of `MAX_STEP_M` needs.
    measuring: Surroundings<'a>,
    /// The clearance the caller asks of each term (`term_clearances`).
    asked: [f64; TERMS],
    start: [f64; D],
    /// Whether the start falls short of the plan's clearance of each term.
    start_short: [bool; TERMS],
    /// The plan's clearance of each term.
    clearance: [f64; TERMS],
    /// The floor the start sets of each term, as the module says.
    floor: [f64; TERMS],
    /// The orientations the path holds.
    orientations: PathOrientations<'a, N, D>,
}

impl<'a, const N: usize, const D: usize> PathChecks<'a, N, D> {
    /// The checks of `arms`' states and motions around `scene`, keeping the clearance
    /// `min_clearance_m` asks of each term along every path from `start`, as the module
    /// says.
    pub(crate) fn new(
        arms: &'a Arms<'a, N, D>,
        scene: &Scene<'a>,
        start: [f64; D],
        min_clearance_m: f64,
    ) -> Self {
        let asked = term_clearances(min_clearance_m);
        let clearance = std::array::from_fn(|term| asked[term] + MOVED[term] * MIN_SPARE_M);
        let measured = std::array::from_fn(|term| asked[term] + MOVED[term] * MAX_STEP_M);
        let measuring = scene.surroundings(measured);
        let at_start = measuring.distances(arms, &start, Moving::All);
        Self {
            arms,
            surroundings: scene.surroundings(clearance),
            measuring,
            asked,
            start,
            start_short: std::array::from_fn(|term| at_start[term] < clearance[term]),
            clearance,
            floor: std::array::from_fn(|term| floor_of(at_start[term], asked[term], MOVED[term])),
            orientations: PathOrientations::none(),
        }
    }

    /// The checks that hold `orientations` as well.
    fn holding(self, orientations: PathOrientations<'a, N, D>) -> Self {
        Self {
            orientations,
            ..self
        }
    }

    /// The planner's check of the motion from `from` to `to`, which keeps every state
    /// the planner keeps but the start at the plan's clearances: `from` keeps them unless
    /// it is the start, and the motion keeps them as `walk_run` walks it, a motion by the
    /// start rule ending where each term keeps them. A motion by the start rule that fails
    /// keeps no state.
    fn walk(&self, from: &[f64; D], to: &[f64; D]) -> Motion {
        if *from != self.start && !self.surroundings.clear(self.arms, from) {
            return Motion::Invalid { last_valid: 0.0 };
        }
        if self.walk_orientations(from, to).is_err() {
            return Motion::Invalid { last_valid: 0.0 };
        }
        // The planner's path to `from` is unknown here: a motion whose first state lies on
        // the straight line from the start to its end departs from the start.
        let along = [self.start, *from];
        let reached: &[[f64; D]] = if on_segment(from, &self.start, to) {
            &along
        } else {
            std::slice::from_ref(from)
        };
        let by_rule = self.departs_by_rule(reached, to);
        match self.walk_run(reached, to) {
            Ok(false) => Motion::Valid,
            Err(Break { last_valid, .. }) if !by_rule => Motion::Invalid { last_valid },
            Ok(true) | Err(_) => Motion::Invalid { last_valid: 0.0 },
        }
    }

    /// The fault of a state at `place` that lies `distances` meters from the bodies of
    /// each term, as a check of a path answers it.
    fn fault(&self, place: Place, distances: [f64; TERMS]) -> PathFault {
        PathFault::Clearance {
            place,
            arms: self.arms.arms().len(),
            distances,
            clearances: self.clearance,
        }
    }

    /// What a message says of a state at `place` too near a path orientation's tolerance.
    fn turned(&self, place: Place, turned: &Turned) -> String {
        let (link, tolerance) = self.orientations.named(turned.orientation);
        format!(
            "{place}, {link} stands {:.4} rad from its path orientation, too near its \
             tolerance_rad {tolerance:.4}",
            turned.error
        )
    }

    /// Walks each path orientation along the straight run from `from` to `to`, and answers
    /// where along the run one breaks.
    fn walk_orientations(&self, from: &[f64; D], to: &[f64; D]) -> Result<(), (f64, Turned)> {
        let motion: [f64; D] = std::array::from_fn(|i| (to[i] - from[i]).abs());
        self.orientations
            .walk(0.0, 1.0, &motion, |t| toward(from, to, t))
    }

    /// How far `q` falls short of the plan's clearance of the term it falls shortest of, in
    /// the pairs that a body of a `moving` arm is one of, in meters; at or below 0 where it
    /// keeps each.
    pub(crate) fn shortfall(&self, q: &[f64; D], moving: Moving) -> f64 {
        let distances = self.surroundings.distances(self.arms, q, moving);
        (0..TERMS)
            .map(|term| self.clearance[term] - distances[term])
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// What `q` falls short of the plan's clearances against in the pairs that a body of a
    /// `moving` arm is one of, as a message names it: the nearest pair of each term it falls
    /// short of, the robot's own body first, then an attached object against the arm that
    /// carries it, then the map and the objects, and how far apart they stand.
    fn short_of(&self, q: &[f64; D], moving: Moving) -> String {
        let [world, own, held] = self.surroundings.nearest_pairs(self.arms, q, moving);
        let [world_clearance, own_clearance, held_clearance] = self.clearance;
        [
            (own, own_clearance),
            (held, held_clearance),
            (world, world_clearance),
        ]
        .into_iter()
        .filter_map(|(nearest, clearance)| nearest.filter(|r| r.distance < clearance))
        .map(|reading| reading.to_string())
        .collect::<Vec<_>>()
        .join(" and ")
    }

    /// The plan's clearances, as a message names them.
    fn plan_clearance(&self) -> String {
        let [world, own, held] = self.clearance;
        let terms = [
            Some(format!("{world:.4} m from the map and the objects")),
            (self.arms.self_model().is_some()
                || (self.arms.arms().len() > 1 && self.arms.carries_an_object()))
            .then(|| format!("{own:.4} m between the robot's own links and what they carry")),
            self.arms.carries_an_object().then(|| {
                format!("{held:.4} m between an attached object and the arm that carries it")
            }),
        ];
        let named: Vec<String> = terms.into_iter().flatten().collect();
        match named.as_slice() {
            [world] => format!("the plan's clearance, {world}"),
            _ => format!("the plan's clearances, {}", named.join(" and ")),
        }
    }

    /// Whether `walk_run` walks the run from the last of `reached`, the path's waypoints
    /// up to the run's first, to `to` by the start rule from the start: `reached` leaves
    /// the start and each of its waypoints after the start lies on the straight line from
    /// the start to `to`, while the start falls short of a term's plan clearance.
    fn departs_by_rule(&self, reached: &[[f64; D]], to: &[f64; D]) -> bool {
        let (first, after) = reached.split_first().expect("a run leaves a waypoint");
        self.start_short.contains(&true)
            && *first == self.start
            && after.iter().all(|q| on_segment(q, &self.start, to))
    }

    /// Walks the run from the last of `reached`, the path's waypoints from the start up to
    /// the run's first, to `to`, as the module says: a run on the straight line from the
    /// start by the start rule from the start, and any other run by its spare, as
    /// `advance_by_spare` walks it. Whether a term the start falls short of still falls
    /// short of its plan clearance at `to`; where the run breaks, the share of it before
    /// the state that breaks it, and that state's distances.
    fn walk_run(&self, reached: &[[f64; D]], to: &[f64; D]) -> Result<bool, Break> {
        self.walk_run_checking(reached, to, &mut |_| {})
    }

    /// The shares of the run from the last of `reached` to `to` at which `walk_run` checks
    /// a state that passes.
    fn walked_shares(&self, reached: &[[f64; D]], to: &[f64; D]) -> Vec<f64> {
        let mut shares = Vec::new();
        // A run that breaks leaves the shares of the states before the break.
        let _ = self.walk_run_checking(reached, to, &mut |share| shares.push(share));
        shares
    }

    /// `walk_run`, handing `checked` the share of the run at each state it checks that
    /// passes.
    fn walk_run_checking(
        &self,
        reached: &[[f64; D]],
        to: &[f64; D],
        checked: &mut dyn FnMut(f64),
    ) -> Result<bool, Break> {
        let from = reached.last().expect("a run leaves a waypoint");
        if self.departs_by_rule(reached, to) {
            let skipped = self.arms.displacement(&self.start, from);
            let total = self.arms.displacement(&self.start, to);
            let mut past_from = |t: f64| {
                if t * total > skipped {
                    checked(share_past(t, skipped, total));
                }
            };
            return self
                .depart_from_start(to, &mut past_from)
                .map_err(|broken| Break {
                    last_valid: share_past(broken.last_valid, skipped, total),
                    ..broken
                });
        }
        let total = self.arms.displacement(from, to);
        self.advance_by_spare(0.0, 1.0, total, |t| toward(from, to, t), checked)
            .map(|()| false)
    }

    /// Walks a motion by conservative advancement, as the module says, from the position
    /// `from` of the motion to `to`, its state at a position `at`, a step of a unit of
    /// position moving any point of a link at most `lever` meters, handing `checked` each
    /// position past `from` at which it checks a state that keeps the plan's clearances.
    /// The motion's state at `from` sets the first step, and a state from which a step
    /// would move a point less than `MIN_SPARE_M` ends it. Where the motion breaks, the
    /// position of the last state it checks, `from` among them, and the distances of the
    /// state that breaks it.
    fn advance_by_spare(
        &self,
        from: f64,
        to: f64,
        lever: f64,
        at: impl Fn(f64) -> [f64; D],
        checked: &mut dyn FnMut(f64),
    ) -> Result<(), Break> {
        let mut position = from;
        let mut here = self.distances(&at(from));
        while position < to {
            let broken = |distances| Break {
                last_valid: position,
                distances,
            };
            let step = self.spare_step(&here).ok_or(broken(here))?;
            let next = if lever > 0.0 {
                (position + step / lever).min(to)
            } else {
                to
            };
            here = self.distances(&at(next));
            if !self.keeps_clearances(&here) {
                return Err(broken(here));
            }
            checked(next);
            position = next;
        }
        Ok(())
    }

    /// The step, in meters any point of a link moves, that keeps every state up to the next
    /// checked one at the clearance asked of each term from a state `distances` meters
    /// from the bodies of each term: the least spare of the terms, at most `MAX_STEP_M`.
    /// None under `MIN_SPARE_M`.
    fn spare_step(&self, distances: &[f64; TERMS]) -> Option<f64> {
        let step = (0..TERMS)
            .map(|term| self.spare(term, distances[term]))
            .fold(MAX_STEP_M, f64::min);
        (step >= MIN_SPARE_M).then_some(step)
    }

    /// The spare of a term `distance` meters from its bodies: its distance above the
    /// clearance asked of it over its moving bodies.
    fn spare(&self, term: usize, distance: f64) -> f64 {
        (distance - self.asked[term]) / MOVED[term]
    }

    /// Whether a state `distances` meters from the bodies of each term keeps each of the
    /// plan's clearances.
    fn keeps_clearances(&self, distances: &[f64; TERMS]) -> bool {
        (0..TERMS).all(|term| distances[term] >= self.clearance[term])
    }

    /// Walks the motion from the start to `to` by the start rule, as the module says,
    /// handing `checked` the share of the motion at each state it checks that passes.
    /// Whether a term the start falls short of still falls short of its plan clearance
    /// at `to`; where the rule breaks, the share of the motion before the state that
    /// breaks it, and that state's distances.
    fn depart_from_start(
        &self,
        to: &[f64; D],
        checked: &mut dyn FnMut(f64),
    ) -> Result<bool, Break> {
        let total = self.arms.displacement(&self.start, to);
        let mut here = self.distances(&self.start);
        let mut before = here;
        let mut t = 0.0;
        while t < 1.0 {
            let broken = |distances| Break {
                last_valid: t,
                distances,
            };
            let step = self.step_from(&here, &before).ok_or(broken(here))?;
            let next = advance(t, step, total);
            here = self.distances(&toward(&self.start, to, next));
            if !self.keeps(&here, &before) {
                return Err(broken(here));
            }
            checked(next);
            before = std::array::from_fn(|term| before[term].max(here[term]));
            t = next;
        }
        Ok((0..TERMS).any(|term| self.start_short[term] && before[term] < self.clearance[term]))
    }

    /// The distance of the arms at `q` from the bodies of each term: exact up to the
    /// distance a step of `MAX_STEP_M` needs, and a lower bound past it.
    fn distances(&self, q: &[f64; D]) -> [f64; TERMS] {
        self.measuring.distances(self.arms, q, Moving::All)
    }

    /// The longest step by the start rule from a state `here` meters from the bodies of
    /// each term, after states at most `before` meters from them: a term that still falls
    /// short of the plan's clearance steps by its room, at least `MIN_DEPARTURE_STEP_M`,
    /// and any other term by its spare, as `spare_step` says. None where a term's step
    /// falls under its floor.
    fn step_from(&self, here: &[f64; TERMS], before: &[f64; TERMS]) -> Option<f64> {
        (0..TERMS).try_fold(MAX_STEP_M, |step, term| {
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
    fn keeps(&self, here: &[f64; TERMS], before: &[f64; TERMS]) -> bool {
        (0..TERMS).all(|term| {
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

impl<const N: usize, const D: usize> Checks for PathChecks<'_, N, D> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        let q: [f64; D] = state.try_into()?;
        Ok(q == self.start || self.surroundings.clear(self.arms, &q))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.walk(from.try_into()?, to.try_into()?))
    }
}

/// The planned waypoints, with waypoints added near each corner whose rounding, as
/// execute_joint_path rounds it, does not keep its clearance, until each does; or why
/// some corner cannot before `deadline`, or which planned waypoint after the start falls
/// short of the plan's clearances. A set `stop` ends it before each walk of the path.
fn rounded_clear<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    planned: &[[f64; D]],
    deadline: Instant,
    stop: Option<&AtomicBool>,
) -> Result<Vec<[f64; D]>, PlanFailure> {
    // Each planned waypoint after the start keeps the plan's clearances, so the start rule
    // holds at no corner, and each arc keeps the plan's clearances.
    if let Some((k, distances)) = first_short_waypoint(checks, planned) {
        return Err(PlanFailure::NoPath(format!(
            "the planned path falls short {}",
            checks.fault(Place::Waypoint(k), distances)
        )));
    }
    // For each planned waypoint, the share of its runs within which the waypoints added at
    // its corner lie, and those waypoints.
    let mut added: Vec<Option<(f64, [[f64; D]; 2])>> = vec![None; planned.len()];
    loop {
        let (waypoints, corner_of): (Vec<[f64; D]>, Vec<usize>) = planned
            .iter()
            .enumerate()
            .flat_map(|(k, &at)| match added[k] {
                None => vec![(at, k)],
                Some((_, [into, out])) => vec![(into, k), (at, k), (out, k)],
            })
            .unzip();
        let path = RoundedPath::joint_path(&vectors(&waypoints));
        let failing = first_failing_probe(checks, &path, &corner_of, planned, &|| is_set(stop))
            .map_err(|Ended| PlanFailure::Stopped)?
            .map(|(corner, _)| (corner, None));
        let turning = if failing.is_some() || checks.orientations.is_empty() {
            None
        } else {
            first_turning_arc(checks, &path, &corner_of, planned, &|| is_set(stop))
                .map_err(|Ended| PlanFailure::Stopped)?
                .map(|(corner, turned)| (corner, Some(turned)))
        };
        let Some((corner, turned)) = failing.or(turning) else {
            return Ok(waypoints);
        };
        if Instant::now() >= deadline {
            return Err(PlanFailure::NoPath(format!(
                "the plan's deadline passed before its rounded path kept its clearance{}",
                checks.orientations.kept()
            )));
        }
        let fraction = added[corner].map_or(FIRST_CORNER_FRACTION, |(f, _)| f / 2.0);
        let repair = (corner > 0 && corner + 1 < planned.len())
            .then(|| match turned {
                None => repair_waypoints(checks, planned, corner, fraction),
                Some(_) => turning_repair_waypoints(checks, planned, corner, fraction),
            })
            .flatten()
            .filter(|added| {
                added
                    .iter()
                    .all(|q| stands_apart_from_corner(q, &planned[corner]))
            });
        let Some(repair) = repair else {
            return Err(PlanFailure::NoPath(match turned {
                None => format!(
                    "the rounded path falls short of {} at the corner of waypoint {corner}, however near it the waypoints added lie",
                    checks.plan_clearance()
                ),
                Some(turned) => format!(
                    "the rounded path falls short {}, however near it the waypoints added lie; \
                     send a wider tolerance_rad",
                    checks.turned(Place::Corner(corner), &turned)
                ),
            }));
        };
        added[corner] = Some((fraction, repair));
    }
}

/// The waypoints a repair of the corner of waypoint `corner` of `planned` adds, one on
/// each of its runs within the share `fraction` of the run from the corner: the state the
/// run's walk checks there farthest from the corner that keeps the plan's clearances, so
/// that each run they split walks only states the plan walked and each waypoint after the
/// start keeps them. `None` when the walk of a run checks no such state there.
fn repair_waypoints<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    planned: &[[f64; D]],
    corner: usize,
    fraction: f64,
) -> Option<[[f64; D]; 2]> {
    let (before, at, after) = (&planned[corner - 1], &planned[corner], &planned[corner + 1]);
    let keeps = |from: &[f64; D], to: &[f64; D], s: f64| {
        checks.surroundings.clear(checks.arms, &toward(from, to, s))
    };
    let into = checks
        .walked_shares(&planned[..corner], at)
        .into_iter()
        .find(|&s| s >= 1.0 - fraction && s < 1.0 && keeps(before, at, s))?;
    let out = checks
        .walked_shares(&planned[..=corner], after)
        .into_iter()
        .take_while(|&s| s <= fraction)
        .filter(|&s| s < 1.0 && keeps(at, after, s))
        .last()?;
    Some([toward(before, at, into), toward(at, after, out)])
}

/// Whether a waypoint added at a corner stands far enough from it for the path to round both.
fn stands_apart_from_corner<const D: usize>(q: &[f64; D], corner: &[f64; D]) -> bool {
    (0..D).any(|i| (q[i] - corner[i]).abs() > 2.0 * MIN_JOINT_CHANGE_RAD)
}

/// The waypoints a repair adds at the corner of waypoint `corner` of `planned` whose rounding
/// breaks a path orientation: on each run, the state at the largest of `fraction` and its
/// halves that keeps the plan's clearances. Each lies on a run that holds the orientations,
/// and the plan walks the runs they split, so the shares shrink an arc to its corner however
/// short the runs are. `None` when every such state of a run that stands apart from the corner
/// falls short of them.
fn turning_repair_waypoints<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    planned: &[[f64; D]],
    corner: usize,
    fraction: f64,
) -> Option<[[f64; D]; 2]> {
    let at = &planned[corner];
    let nearest_clear = |other: &[f64; D]| {
        std::iter::successors(Some(fraction), |share| Some(share / 2.0))
            .map(|share| toward(at, other, share))
            .take_while(|q| stands_apart_from_corner(q, at))
            .find(|q| checks.surroundings.clear(checks.arms, q))
    };
    Some([
        nearest_clear(&planned[corner - 1])?,
        nearest_clear(&planned[corner + 1])?,
    ])
}

/// Each configuration of `waypoints` as a vector of its joint values.
pub(crate) fn vectors<const N: usize>(waypoints: &[[f64; N]]) -> Vec<Vec<f64>> {
    waypoints.iter().map(|q| q.to_vec()).collect()
}

/// The point `fraction` of the way from `at` to `other`.
fn toward<const N: usize>(at: &[f64; N], other: &[f64; N], fraction: f64) -> [f64; N] {
    std::array::from_fn(|i| at[i] + (other[i] - at[i]) * fraction)
}

/// The first waypoint of `path` after the start that falls short of the plan's
/// clearances, by its index, and its distances from the bodies of each term.
fn first_short_waypoint<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &[[f64; D]],
) -> Option<(usize, [f64; TERMS])> {
    (1..path.len())
        .find(|&k| !checks.surroundings.clear(checks.arms, &path[k]))
        .map(|k| (k, checks.distances(&path[k])))
}

/// The first run of `path`, which leaves from the start, by the waypoint it leaves from,
/// that does not keep its clearances as `PathChecks::walk_run` walks it, and where it
/// breaks. `ended` ends the walk before a run.
fn first_failing_run<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &[[f64; D]],
    ended: &dyn Fn() -> bool,
) -> Result<Option<(usize, Break)>, Ended> {
    for k in 0..path.len().saturating_sub(1) {
        if ended() {
            return Err(Ended);
        }
        if let Err(broken) = checks.walk_run(&path[..=k], &path[k + 1]) {
            return Ok(Some((k, broken)));
        }
    }
    Ok(None)
}

/// The first run of `path` along which a path orientation breaks, by the waypoint it leaves
/// from, and which. `ended` ends the walk before a run.
fn first_turning_run<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &[[f64; D]],
    ended: &dyn Fn() -> bool,
) -> Result<Option<(usize, Turned)>, Ended> {
    for (k, run) in path.windows(2).enumerate() {
        if ended() {
            return Err(Ended);
        }
        if let Err((_, turned)) = checks.walk_orientations(&run[0], &run[1]) {
            return Ok(Some((k, turned)));
        }
    }
    Ok(None)
}

/// The waypoint of `planned` whose corner holds the first arc of `path` that does not keep
/// its clearances as `PathChecks::advance_by_spare` walks it along the path, at the arc's
/// lever (`Arms::lever_along`), and where the arc breaks. `ended` ends the walk before an
/// arc.
fn first_failing_probe<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &RoundedPath,
    corner_of: &[usize],
    planned: &[[f64; D]],
    ended: &dyn Fn() -> bool,
) -> Result<Option<(usize, Break)>, Ended> {
    first_failing_arc(path, corner_of, planned, ended, &|from, to, share, at| {
        let lever = checks.arms.lever_along(share);
        checks
            .advance_by_spare(from, to, lever, at, &mut |_| {})
            .map_err(|broken| (broken.last_valid, broken))
    })
}

/// The waypoint of `planned` whose corner holds the first arc of `path` along which a path
/// orientation breaks, and where. `ended` ends the walk before an arc.
fn first_turning_arc<const N: usize, const D: usize>(
    checks: &PathChecks<N, D>,
    path: &RoundedPath,
    corner_of: &[usize],
    planned: &[[f64; D]],
    ended: &dyn Fn() -> bool,
) -> Result<Option<(usize, Turned)>, Ended> {
    first_failing_arc(path, corner_of, planned, ended, &|from, to, share, at| {
        checks.orientations.walk(from, to, share, at)
    })
}

/// A walk of an arc, as `first_failing_arc` says.
type ArcWalk<'a, const D: usize, T> =
    dyn Fn(f64, f64, &[f64; D], &dyn Fn(f64) -> [f64; D]) -> Result<(), (f64, T)> + 'a;

/// The waypoint of `planned` whose corner holds the first arc of `path` that `walk` fails,
/// and what it finds. `walk` takes the positions the arc runs between, the largest share of
/// the path's tangent each joint has on it, and the path's state at a position, and answers
/// the position where the arc fails. `corner_of` names, for each waypoint of `path`, the
/// waypoint of `planned` whose corner it rounds. A state on a segment of `planned` is not
/// checked: the plan walks each run of its answer. `ended` ends the walk before an arc.
fn first_failing_arc<const D: usize, T>(
    path: &RoundedPath,
    corner_of: &[usize],
    planned: &[[f64; D]],
    ended: &dyn Fn() -> bool,
    walk: &ArcWalk<'_, D, T>,
) -> Result<Option<(usize, T)>, Ended> {
    let length = path.length();
    let at = |s: f64| -> [f64; D] { path.at(s).as_slice().try_into().expect("a configuration") };
    let walked = |s: f64| {
        let q = at(s);
        planned.windows(2).any(|w| on_segment(&q, &w[0], &w[1]))
    };
    for corner in 1..corner_of.len().saturating_sub(1) {
        if ended() {
            return Err(Ended);
        }
        // The arc at `corner` runs from where the path reaches the corner's waypoint to
        // where it lies on a planned segment again, before the next waypoint.
        let from = first_position(0.0, length, |s| path.waypoint_at(s) >= corner);
        let next = first_position(from, length, |s| path.waypoint_at(s) > corner);
        let to = first_position(from, next, walked);
        let share: [f64; D] = path
            .max_abs_tangent(from, to)
            .try_into()
            .expect("a share of the tangent per joint");
        if let Err((position, found)) = walk(from, to, &share, &at) {
            return Ok(Some((corner_of[path.waypoint_at(position)], found)));
        }
    }
    Ok(None)
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
pub(crate) mod tests {
    use camera_core::pose::Pose;
    use chain_kinematics::{ChainSpec, JointSelection};
    use collision_geometry::chain::ChainGeometry;
    use collision_geometry::gjk::Placed;
    use control_core::positive_finite::PositiveFinite;
    use hegel::TestCase;
    use hegel::generators as gs;
    use occupancy_map::{Bounds, Frame, Scan};

    use super::*;
    use crate::arm::{Arm, Body, SelfModel};
    use crate::goal::OrientationTarget;
    use crate::scene::{ObjectFields, ObjectRequest, PlanningScene, RobotLinks, ShapeFields};

    // Waypoints inside a straight stretch go, the first and the last and each corner stay, and a
    // turn back on the same line stays: the path rests there.
    #[test]
    fn the_waypoints_inside_a_straight_stretch_go() {
        let path = [
            [0.0, 0.0],
            [0.1, 0.0],
            [0.2, 0.0],
            [0.3, 0.1],
            [0.4, 0.2],
            [0.1, -0.1],
            [0.0, -0.2],
        ];
        assert_eq!(
            without_straight_waypoints(&path),
            [[0.0, 0.0], [0.2, 0.0], [0.4, 0.2], [0.0, -0.2]]
        );
        assert_eq!(without_straight_waypoints::<2>(&[]), Vec::<[f64; 2]>::new());
        assert_eq!(without_straight_waypoints(&[[1.0, 2.0]]), [[1.0, 2.0]]);
    }

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
    pub(crate) fn world(
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

    /// The group of the one arm `geometry` holds, checked against `self_model`.
    fn one_arm<'a, const N: usize>(
        geometry: &'a ChainGeometry<N>,
        self_model: Option<&'a dyn SelfModel<N>>,
    ) -> Arms<'a, N, N> {
        Arms::new(vec![Arm::new("arm", geometry)], self_model)
    }

    /// A self model whose pair stands `apart` meters apart at every configuration.
    struct StandsApart(f64);

    impl<const N: usize> SelfModel<N> for StandsApart {
        fn clearance(&self, _: &[f64; N], _: Moving) -> f64 {
            self.0
        }

        fn nearest_pair(&self, _: &[f64; N], _: Moving) -> (String, String) {
            ("a link".to_string(), "its neighbor".to_string())
        }
    }

    // A set stop flag ends the check of a rounded path before the walk of its corner, and
    // the check of a plan's answer before the walk of its run; the same paths with the
    // flag clear pass.
    #[test]
    fn a_stop_ends_the_check_of_the_rounded_path() {
        let geometry = two_links();
        let arms = one_arm(&geometry, None);
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &[]);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, [0.0, 0.0], MIN_CLEARANCE_M);
        let path = [[0.0, 0.0], [0.5, 0.5], [1.0, 0.0]];
        let far = Instant::now() + Duration::from_secs(3600);
        let (clear, set) = (AtomicBool::new(false), AtomicBool::new(true));
        assert!(rounded_clear(&checks, &path, far, Some(&clear)).is_ok());
        assert_eq!(
            rounded_clear(&checks, &path, far, Some(&set)),
            Err(PlanFailure::Stopped)
        );
        let run = [vec![0.0, 0.0], vec![0.5, 0.5]];
        assert!(answered(&checks, &run, far, Some(&clear)).is_ok());
        assert_eq!(
            answered(&checks, &run, far, Some(&set)),
            Err(PlanFailure::Stopped)
        );
    }

    /// A wrist of three revolute joints about x, y and z at one point, its links with no body.
    fn wrist() -> ChainGeometry<3> {
        const URDF: &str = r#"<robot name="wrist">
            <link name="base"/>
            <link name="link_1"/>
            <link name="link_2"/>
            <link name="link_3"/>
            <joint name="joint_1" type="revolute">
              <parent link="base"/><child link="link_1"/><axis xyz="1 0 0"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="joint_2" type="revolute">
              <parent link="link_1"/><child link="link_2"/><axis xyz="0 1 0"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="joint_3" type="revolute">
              <parent link="link_2"/><child link="link_3"/><axis xyz="0 0 1"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
          </robot>"#;
        ChainGeometry::from_urdf(URDF, "", &spec("link_3")).expect("a wrist")
    }

    /// The target that the wrist's last link holds within `tolerance`, its z axis free to turn
    /// about: that axis opposite to where `q` holds it, so the error is a half turn at `q` and
    /// falls as the wrist leaves `q`, the way a state moves off the far side of a sphere.
    fn half_a_turn_from(
        geometry: &ChainGeometry<3>,
        q: &[f64; 3],
        tolerance: f64,
    ) -> OrientationTarget {
        let frame = crate::goal::Frame::Link {
            segment: 2,
            offset: nalgebra::Isometry3::identity(),
        };
        OrientationTarget {
            frame,
            orientation: geometry.chain().at(q).link_pose_world(2).rotation
                * nalgebra::UnitQuaternion::from_axis_angle(
                    &nalgebra::Vector3::x_axis(),
                    std::f64::consts::PI,
                ),
            tolerance_rad: PositiveFinite::parse(tolerance).expect("a positive tolerance"),
            free_axis: Some(crate::goal::Axis::Z),
        }
    }

    /// The largest error of `target` on the wrist along the rounded path through `waypoints`.
    fn largest_on_the_wrist(
        geometry: &ChainGeometry<3>,
        target: &OrientationTarget,
        waypoints: &[[f64; 3]],
    ) -> f64 {
        const STEP_RAD: f64 = 1e-3;
        let path = RoundedPath::joint_path(&vectors(waypoints));
        let count = (path.length() / STEP_RAD).ceil() as usize;
        (0..=count)
            .map(|k| {
                let q: [f64; 3] = path
                    .at(path.length() * k as f64 / count as f64)
                    .as_slice()
                    .try_into()
                    .expect("a state of the wrist");
                target.error_at(geometry.chain(), &q)
            })
            .fold(0.0, f64::max)
    }

    /// Runs `body` on the wrist's checks from `start`, holding `target`.
    fn with_the_wrist_holding<T>(
        geometry: &ChainGeometry<3>,
        target: OrientationTarget,
        start: [f64; 3],
        body: impl Fn(&PathChecks<3, 3>) -> T,
    ) -> T {
        let arms = one_arm(geometry, None);
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &[]);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let orientations = [PathOrientation {
            arm: 0,
            link_name: "link_3",
            target,
        }];
        let held = PathOrientations::new(&arms, &orientations, &start)
            .unwrap_or_else(|failure| panic!("{failure}"));
        body(&PathChecks::new(&arms, &scene, start, MIN_CLEARANCE_M).holding(held))
    }

    // A corner's rounding can leave an orientation both of its runs hold: the held axis stands
    // opposite to its target at the arc's middle. The wrist has no body, so a clearance's walk
    // holds no state near the corner, and the repair adds waypoints that pull the arc in, which
    // then holds the orientation.
    #[test]
    fn a_corner_that_breaks_an_orientation_gains_waypoints_that_hold_it() {
        /// The share of a tolerance a state may hold: `FLOOR_SHARE` from the tolerance.
        const LARGEST_SHARE: f64 = 0.99;
        let geometry = wrist();
        let planned = [[0.4, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.4, 0.0]];
        let rounded = RoundedPath::joint_path(&vectors(&planned));
        let middle: [f64; 3] = rounded
            .at(rounded.length() / 2.0)
            .as_slice()
            .try_into()
            .expect("a state of the wrist");
        let measure = half_a_turn_from(&geometry, &middle, 1.0);
        let on_the_runs = largest_on_the_wrist(&geometry, &measure, &planned[..2])
            .max(largest_on_the_wrist(&geometry, &measure, &planned[1..]));
        let on_the_arc = largest_on_the_wrist(&geometry, &measure, &planned);
        assert!(
            on_the_arc > on_the_runs,
            "the arc holds {on_the_arc}, the runs {on_the_runs}: nothing to repair"
        );
        let tolerance = (on_the_arc + on_the_runs) / 2.0 / LARGEST_SHARE;
        let target = half_a_turn_from(&geometry, &middle, tolerance);
        let answer = with_the_wrist_holding(&geometry, target, planned[0], |checks| {
            let far = Instant::now() + Duration::from_secs(3600);
            assert!(
                first_turning_arc(checks, &rounded, &[0, 1, 2], &planned, &|| false)
                    .is_ok_and(|turning| turning.is_some()),
                "the arc holds the orientation: nothing to repair"
            );
            rounded_clear(checks, &planned, far, None).unwrap_or_else(|failure| panic!("{failure}"))
        });
        assert!(answer.len() > planned.len(), "{answer:?}");
        let largest = largest_on_the_wrist(&geometry, &target, &answer);
        assert!(largest <= tolerance, "{largest} rad, past {tolerance}");
    }

    // A repair of a turning corner takes, on each run, the largest of its share and its halves
    // that keeps the plan's clearances, and none where each stands too near. The arm's tip
    // sweeps past a block on the run into the corner.
    #[test]
    fn a_turning_repair_takes_the_largest_share_clear_of_the_bodies() {
        let geometry = two_links();
        let arms = one_arm(&geometry, None);
        let planned = [[1.0, 0.0], [0.0, 0.0], [0.0, 0.4]];
        let repair_with_a_block_at = |block: [f64; 3]| {
            let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &[block]);
            let scene = Scene {
                map: &map,
                field: &field,
                stamp: 0,
                objects: Objects::none(),
            };
            let checks = PathChecks::new(&arms, &scene, planned[0], MIN_CLEARANCE_M);
            turning_repair_waypoints(&checks, &planned, 1, FIRST_CORNER_FRACTION)
        };
        let [into, out] = repair_with_a_block_at([0.533, 0.136, 0.0]).expect("a share clear of it");
        assert!(into[0] > 0.0 && into[0] < FIRST_CORNER_FRACTION, "{into:?}");
        assert_eq!(out, toward(&planned[1], &planned[2], FIRST_CORNER_FRACTION));
        assert_eq!(repair_with_a_block_at([0.56, 0.0, 0.0]), None);
    }

    // A corner that stands past an orientation's tolerance by more than a corner's deviation has
    // no repair: however near it the added waypoints pull the arc, the arc breaks the orientation,
    // and the plan says so.
    #[test]
    fn a_corner_past_its_tolerance_has_no_repair() {
        let geometry = wrist();
        let planned = [[0.4, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.4, 0.0]];
        let target = half_a_turn_from(&geometry, &planned[1], 3.0);
        let failure = with_the_wrist_holding(&geometry, target, planned[0], |checks| {
            let far = Instant::now() + Duration::from_secs(3600);
            rounded_clear(checks, &planned, far, None).expect_err("no repair holds the corner")
        });
        let PlanFailure::NoPath(message) = failure else {
            panic!("{failure:?}");
        };
        assert!(
            message.contains("from its path orientation, too near its tolerance_rad 3.0000")
                && message.contains("however near it the waypoints added lie"),
            "{message}"
        );
    }

    // A straight run whose middle breaks an orientation its ends hold is no motion of the
    // planner's, and it fails the answer's walk, naming the link; a set stop flag ends that walk
    // first.
    #[test]
    fn a_run_that_breaks_an_orientation_fails_the_answer() {
        let geometry = wrist();
        let run = [[0.4, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let target = half_a_turn_from(&geometry, &[0.2, 0.0, 0.0], 3.1);
        let (motion, failure, stopped) =
            with_the_wrist_holding(&geometry, target, run[0], |checks| {
                let far = Instant::now() + Duration::from_secs(3600);
                let set = AtomicBool::new(true);
                let path = vectors(&run);
                (
                    checks.motion(&run[0], &run[1]),
                    answered(checks, &path, far, None),
                    answered(checks, &path, far, Some(&set)),
                )
            });
        assert!(matches!(motion, Ok(Motion::Invalid { .. })), "{motion:?}");
        let Err(PlanFailure::NoPath(message)) = failure else {
            panic!("{failure:?}");
        };
        assert!(
            message.starts_with("the planned path falls short")
                && message.contains("link_3 stands")
                && message.contains("send a wider tolerance_rad"),
            "{message}"
        );
        assert_eq!(stopped, Err(PlanFailure::Stopped));
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
        let arms = one_arm(&geometry, model.as_ref().map(|m| m as &dyn SelfModel<1>));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, [0.45], MIN_CLEARANCE_M);
        checks.walk_run(&[[-0.3]], &[0.3]).is_ok()
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
        fn clearance(&self, q: &[f64; 2], _: Moving) -> f64 {
            let tip = [
                0.3 * q[0].cos() + 0.25 * (q[0] + q[1]).cos(),
                0.3 * q[0].sin() + 0.25 * (q[0] + q[1]).sin(),
            ];
            ((tip[0] + 0.1).powi(2) + tip[1].powi(2)).sqrt() - 0.05
        }

        fn nearest_pair(&self, _: &[f64; 2], _: Moving) -> (String, String) {
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
    // passes, a path at the lever of its tangent (`Arms::lever_along`), keeps
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
        let arms = one_arm(&geometry, Some(&TipAndBall));
        let min_clearance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.01));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, [2.9, 2.9], min_clearance);
        tc.assume(checks.surroundings.clear(&arms, &from));
        let dense = Surroundings::new(&map, &field, term_clearances(min_clearance));
        let keeps = |q: &[f64; 2]| dense.clear(&arms, q);
        if checks.walk_run(&[from], &via).is_ok() {
            let count = (arms.displacement(&from, &via) / DENSE_M).ceil() as usize;
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
        let lever = arms.lever_along(&share);
        let at =
            |s: f64| -> [f64; 2] { path.at(s).as_slice().try_into().expect("a configuration") };
        if checks
            .advance_by_spare(0.0, path.length(), lever, at, &mut |_| {})
            .is_ok()
        {
            let count = (path.length() * arms.lever() / DENSE_M).ceil() as usize;
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
        let arms = one_arm(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let from = [face - 0.005 - PLATE[0] / 2.0];
        let to = [face + 0.03];
        let checks = PathChecks::new(&arms, &scene, [-0.4], 0.0);
        let touching = Surroundings::new(&map, &field, [0.0; TERMS]);
        assert!(touching.contact(&arms, &[face + VOXEL / 2.0]).is_some());
        let gap = checks.distances(&from)[0];
        assert!(
            (gap - 0.005).abs() < 1e-12,
            "the plate starts {gap} m from the voxel"
        );
        assert!(checks.walk_run(&[from], &to).is_err());
    }

    /// A goal sampler that answers one configuration where it is valid.
    struct At<const N: usize>([f64; N]);

    impl<const N: usize> GoalSampler<N> for At<N> {
        fn sample_from(
            &self,
            _: &Goal,
            _: &[f64; N],
            _: usize,
            _: &mut Draws,
            valid: &dyn Fn(&[f64; N]) -> bool,
        ) -> Option<[f64; N]> {
            valid(&self.0).then_some(self.0)
        }
    }

    // From -0.6 rad to 0.6 rad with the slide out at 0.5 m, a point of the cube of
    // `turn_and_slide` moves over half a meter, within the reach bound: the reach of the
    // turn covers the slide's travel past it.
    #[test]
    fn the_reach_of_a_turn_covers_the_slide_past_it() {
        let geometry = turn_and_slide();
        let arm = Arm::new("arm", &geometry);
        let (from, to) = ([-0.6, 0.5], [0.6, 0.5]);
        let pieces = |q: &[f64; 2]| -> Vec<Placed> {
            arm.bodies(q)
                .into_iter()
                .map(|(_, body)| match body {
                    Body::Piece(piece) => piece,
                    Body::Shape(_) => panic!("the arm carries no object"),
                })
                .collect()
        };
        let moved = pieces(&from)
            .iter()
            .zip(&pieces(&to))
            .flat_map(|(a, b)| {
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
        let arms = one_arm(&geometry, None);
        let (start, end) = ([-0.6, 0.5], [0.6, 0.5]);
        let tolerance = PositiveFinite::parse(0.01).expect("a positive tolerance");
        let goal = Goal::new(
            2,
            (0..2)
                .map(|joint| crate::goal::JointTarget {
                    joint,
                    position: end[joint],
                    above: tolerance,
                    below: tolerance,
                })
                .collect(),
            Vec::new(),
            Vec::new(),
        )
        .expect("a valid goal");
        let answer = plan_path(
            &arms,
            &Scene {
                map: &map,
                field: &field,
                stamp: 1,
                objects: Objects::none(),
            },
            &PlanRequest {
                start,
                start_tolerance: [0.0; 2],
                goals: vec![ArmGoal {
                    goal: &goal,
                    sampler: &At(end),
                }],
                path_orientations: &[],
                goal_samples: NonZeroUsize::new(1).expect("nonzero"),
                planner: Planner::RrtConnect,
                deadline: Instant::now() + Duration::from_secs(600),
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
                "RRTConnect found no path within 20000 iterations or by its deadline, keeping \
                 the plan's clearance, 0.0210 m from the map and the objects"
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
        let arms = one_arm(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let planned = [[-0.5, 0.0], [0.0, 0.0], [0.0, 0.5]];
        let checks = PathChecks::new(&arms, &scene, planned[0], MIN_CLEARANCE_M);
        let rounded_first = RoundedPath::joint_path(&vectors(&planned));
        let dense = Surroundings::new(&map, &field, term_clearances(MIN_CLEARANCE_M));
        let at = |path: &RoundedPath, s: f64| -> [f64; 2] {
            path.at(s).as_slice().try_into().expect("a configuration")
        };
        let probes = |path: &RoundedPath| -> Vec<[f64; 2]> {
            let count = (path.length() * arms.lever() / DENSE_M).ceil() as usize;
            (0..=count)
                .map(|k| at(path, path.length() * k as f64 / count as f64))
                .collect()
        };
        assert!(
            probes(&rounded_first)
                .iter()
                .any(|q| !dense.clear(&arms, q)),
            "the corner's rounding clears the block: nothing to repair"
        );
        let deadline = Instant::now() + Duration::from_secs(3600);
        let answer = match answered(&checks, &vectors(&planned), deadline, None) {
            Ok(answer) => answer,
            Err(failure) => panic!("the corner is not repaired: {failure}"),
        };
        for q in probes(&RoundedPath::joint_path(&vectors(&answer))) {
            assert!(
                dense.clear(&arms, &q),
                "the rounded path through {answer:?} touches the block at {q:?}"
            );
        }
        assert_eq!(answer.len(), planned.len() + 2, "{answer:?}");
        // A check of a caller's path finds the corner the plan repairs, and passes the
        // repaired path.
        let check = |waypoints: &[[f64; 2]]| {
            check_path(
                &arms,
                &scene,
                &CheckRequest {
                    start: planned[0],
                    start_tolerance: [0.0; 2],
                    waypoints,
                    min_clearance_m: MIN_CLEARANCE_M,
                    deadline,
                },
            )
            .unwrap_or_else(|failure| panic!("{failure}"))
        };
        let unrepaired = check(&planned[1..]);
        assert_eq!(unrepaired.clear_waypoints, 1, "{unrepaired:?}");
        assert!(
            matches!(
                unrepaired.fault,
                Some(PathFault::Clearance {
                    place: Place::Corner(1),
                    ..
                })
            ),
            "{unrepaired:?}"
        );
        let repaired = check(&answer[1..]);
        assert_eq!(
            (repaired.clear_waypoints, repaired.fault),
            (answer.len() - 1, None)
        );
    }

    /// The check of `waypoints` from the start `start` of the gantry around `block`'s
    /// voxels, its walks ended once `ended` holds.
    fn gantry_check(
        start: [f64; 2],
        waypoints: &[[f64; 2]],
        block: &[[f64; 3]],
        ended: &dyn Fn() -> bool,
    ) -> Result<PathCheck, CheckFailure> {
        let (map, field) = world(VOXEL_M, [-1.2, -0.6, -0.1], [1.2, 0.6, 0.1], block);
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let request = CheckRequest {
            start,
            start_tolerance: [0.0; 2],
            waypoints,
            min_clearance_m: MIN_CLEARANCE_M,
            deadline: Instant::now(),
        };
        check_path_until(&arms, &scene, &request, ended)
    }

    /// A self model of the gantry whose pair stands 1 cm apart at the origin and farther by
    /// how far the cube stands from it.
    struct RisingFromTheOrigin;

    impl SelfModel<2> for RisingFromTheOrigin {
        fn clearance(&self, q: &[f64; 2], _: Moving) -> f64 {
            0.01 + q[0].hypot(q[1])
        }

        fn nearest_pair(&self, _: &[f64; 2], _: Moving) -> (String, String) {
            ("link_2".to_string(), "its base".to_string())
        }
    }

    // A start short of the plan's clearance of the robot's own body, a path from it around a
    // block by a waypoint off the straight line to its end, then on through a waypoint on
    // that line: the run from that waypoint is walked by its spare, and the path passes. The
    // straight path to that waypoint runs into the block.
    #[test]
    fn a_run_on_the_starts_line_after_a_waypoint_off_it_walks_by_its_spare() {
        let block: Vec<[f64; 3]> = [0.115, 0.125]
            .into_iter()
            .flat_map(|x| [0.115, 0.125].map(|y| [x, y, 0.0]))
            .collect();
        let (map, field) = world(VOXEL_M, [-1.2, -0.6, -0.1], [1.2, 0.6, 0.1], &block);
        let geometry = gantry();
        let arms = one_arm(&geometry, Some(&RisingFromTheOrigin));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checked = |waypoints: &[[f64; 2]]| {
            check_path(
                &arms,
                &scene,
                &CheckRequest {
                    start: [0.0, 0.0],
                    start_tolerance: [0.0; 2],
                    waypoints,
                    min_clearance_m: MIN_CLEARANCE_M,
                    deadline: Instant::now() + Duration::from_secs(3600),
                },
            )
            .unwrap_or_else(|failure| panic!("{failure}"))
        };
        let around = [[0.0, 0.25], [0.25, 0.25], [0.45, 0.45]];
        let check = checked(&around);
        assert_eq!((check.clear_waypoints, check.fault), (3, None));
        let straight = checked(&around[1..]);
        assert_eq!(check_place(&straight), Some(Place::Run(0)), "{straight:?}");
    }

    // A simplified path that runs through a block fails its check, and the plan answers
    // the planned path around it, which passes; with no block the simplified path passes
    // and is the answer.
    #[test]
    fn a_plan_whose_simplified_path_fails_its_check_answers_the_planned_path() {
        let planned: Vec<Vec<f64>> = [[0.0, 0.0], [0.0, 0.25], [0.4, 0.25], [0.4, 0.0]]
            .map(Vec::from)
            .to_vec();
        let simplified: Vec<Vec<f64>> = [[0.0, 0.0], [0.4, 0.0]].map(Vec::from).to_vec();
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let deadline = Instant::now() + Duration::from_secs(3600);
        let answer = |block: &[[f64; 3]]| {
            let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], block);
            let scene = Scene {
                map: &map,
                field: &field,
                stamp: 0,
                objects: Objects::none(),
            };
            let checks = PathChecks::new(&arms, &scene, [0.0, 0.0], MIN_CLEARANCE_M);
            answer_of(&checks, &planned, &simplified, deadline, None)
                .unwrap_or_else(|failure| panic!("{failure}"))
        };
        let around = answer(&[[0.2, 0.0, 0.0]]);
        assert_eq!(around.first(), Some(&[0.0, 0.0]));
        assert_eq!(around.last(), Some(&[0.4, 0.0]));
        assert!(
            around.iter().any(|q| q[1] >= 0.2),
            "the answer does not go around the block: {around:?}"
        );
        assert_eq!(answer(&[]), [[0.0, 0.0], [0.4, 0.0]]);
    }

    /// The fault of the check of the gantry's path from the origin to (0.3, 0) among `block`
    /// and `objects`, with `self_model`.
    fn fault_from_the_origin(
        block: &[[f64; 3]],
        objects: &[ObjectRequest],
        self_model: Option<&dyn SelfModel<2>>,
    ) -> Option<PathFault> {
        let (mut map, _) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], block);
        let mut scene = PlanningScene::new(RobotLinks::new(
            ["link_1", "link_2"].map(String::from),
            ["link_2".to_string()],
        ));
        scene
            .apply_collision_objects(objects, &mut map)
            .expect("the objects apply");
        let field = DistanceField::of(&map);
        let objects = scene.for_arms(&["link_2"], |link| panic!("{link} is no other arm's"));
        let geometry = gantry();
        let arms = one_arm(&geometry, self_model);
        let met = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: objects.objects(&[]),
        };
        check_path(
            &arms,
            &met,
            &CheckRequest {
                start: [0.0, 0.0],
                start_tolerance: [0.0; 2],
                waypoints: &[[0.3, 0.0]],
                min_clearance_m: MIN_CLEARANCE_M,
                deadline: Instant::now() + Duration::from_secs(3600),
            },
        )
        .unwrap_or_else(|failure| panic!("{failure}"))
        .fault
    }

    /// A box of side `side` at `position`, as an object request adds it.
    fn box_at(id: &str, position: [f64; 3], side: f64) -> ObjectRequest {
        ObjectRequest {
            operation: "add".to_string(),
            object: ObjectFields {
                id: id.to_string(),
                position,
                orientation: [0.0, 0.0, 0.0, 1.0],
                shapes: vec![ShapeFields {
                    kind: "box".to_string(),
                    dimensions: vec![side; 3],
                    mesh_vertices: Vec::new(),
                    mesh_triangles: Vec::new(),
                    position: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                }],
            },
        }
    }

    // A start 1 cm from a voxel or from an object, out of contact and nearer than
    // `min_clearance_m`, is refused, naming the pair, its distance and the clearance; the
    // same starts 3 cm away pass. A start 1 cm from the robot's own body, which MoveIt does
    // not pad, and one in the spare above the clearance from an object, leave by the start
    // rule.
    #[test]
    fn a_start_inside_the_clearance_of_the_map_or_an_object_is_refused() {
        let cube = Contact::Map {
            body: "link_2".to_string(),
        };
        let shelf = Contact::Object {
            body: "link_2".to_string(),
            object: "shelf".to_string(),
        };
        let refused = |fault: &Option<PathFault>, pair: &Contact| {
            matches!(
                fault,
                Some(PathFault::Start(StartFault::InsideClearance { reading, clearance }))
                    if reading.pair == *pair
                        && (reading.distance - 0.01).abs() < 1e-9
                        && *clearance == MIN_CLEARANCE_M
            )
        };
        let near = fault_from_the_origin(&[[0.0, -0.025, 0.0]], &[], None);
        assert!(refused(&near, &cube), "{near:?}");
        assert_eq!(
            fault_from_the_origin(&[[0.0, -0.045, 0.0]], &[], None),
            None
        );
        let near = fault_from_the_origin(&[], &[box_at("shelf", [0.0, 0.04, 0.0], 0.04)], None);
        assert!(refused(&near, &shelf), "{near:?}");
        assert_eq!(
            fault_from_the_origin(&[], &[box_at("shelf", [0.0, 0.06, 0.0], 0.04)], None),
            None
        );
        let in_the_spare = MIN_CLEARANCE_M + MIN_SPARE_M / 2.0;
        let spare_shelf = box_at("shelf", [0.0, 0.03 + in_the_spare, 0.0], 0.04);
        assert_eq!(fault_from_the_origin(&[], &[spare_shelf], None), None);
        assert_eq!(
            fault_from_the_origin(&[], &[], Some(&RisingFromTheOrigin)),
            None
        );
    }

    /// Where the first fault of `check` lies, when it is a clearance fault.
    fn check_place(check: &PathCheck) -> Option<Place> {
        match check.fault {
            Some(PathFault::Clearance { place, .. }) => Some(place),
            _ => None,
        }
    }

    // A path whose first waypoint lies past joint 1's limit at 1 m, its run into a block
    // before it leaves the limits: the check answers the block on the run, the first
    // fault along the path. With the block past the limit, where the arm cannot go, and
    // with no block, the check answers the waypoint outside the limit.
    #[test]
    fn a_fault_on_the_run_to_a_waypoint_outside_a_limit_comes_first() {
        let check = gantry_check([0.0, 0.0], &[[1.2, 0.0]], &[[0.3, 0.0, 0.0]], &|| false)
            .unwrap_or_else(|failure| panic!("{failure}"));
        assert_eq!(check.clear_waypoints, 0, "{check:?}");
        assert_eq!(check_place(&check), Some(Place::Run(0)), "{check:?}");
        for block in [&[[1.1, 0.0, 0.0]][..], &[]] {
            let past = gantry_check([0.0, 0.0], &[[1.2, 0.0]], block, &|| false)
                .unwrap_or_else(|failure| panic!("{failure}"));
            assert!(
                matches!(
                    past.fault,
                    Some(PathFault::OutsideLimits { waypoint: 1, .. })
                ),
                "{block:?}: {past:?}"
            );
        }
    }

    // A waypoint a hair past joint 1's limit at 1 m runs at the limit, as
    // execute_joint_path runs it, and the path passes; one 1.1 mm past it lies outside.
    #[test]
    fn a_waypoint_a_hair_past_a_limit_runs_at_the_limit_and_one_farther_out_lies_outside() {
        let hair = gantry_check([0.0, 0.0], &[[1.0005, 0.0]], &[], &|| false)
            .unwrap_or_else(|failure| panic!("{failure}"));
        assert_eq!((hair.clear_waypoints, hair.fault), (1, None));
        let past = gantry_check([0.0, 0.0], &[[1.0011, 0.0]], &[], &|| false)
            .unwrap_or_else(|failure| panic!("{failure}"));
        assert!(
            matches!(
                past.fault,
                Some(PathFault::OutsideLimits { waypoint: 1, position, .. }) if position == 1.0011
            ),
            "{past:?}"
        );
    }

    // A waypoint with a value that is not finite answers that it lies outside the limits,
    // first on the path and after a waypoint that passes.
    #[test]
    fn a_waypoint_that_is_not_finite_lies_outside_the_limits() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for (waypoints, waypoint) in [
                (vec![[value, 0.0]], 1),
                (vec![[0.0, value]], 1),
                (vec![[0.3, 0.0], [value, 0.0]], 2),
            ] {
                let check = gantry_check([0.0, 0.0], &waypoints, &[], &|| false)
                    .unwrap_or_else(|failure| panic!("{failure}"));
                assert_eq!(check.clear_waypoints, waypoint - 1, "{check:?}");
                assert!(
                    matches!(
                        check.fault,
                        Some(PathFault::OutsideLimits { waypoint: k, .. }) if k == waypoint
                    ),
                    "{value} in {waypoints:?}: {check:?}"
                );
            }
        }
    }

    /// A self model whose pair stands a meter apart at every configuration, which counts
    /// the states it is asked about and refuses, by a panic, to be asked about more than
    /// `MOST_STATES`.
    struct CountsItsStates(std::sync::atomic::AtomicUsize);

    /// The states a check of one run along 1 m of a joint asks `CountsItsStates` about at
    /// most: its walk measures some 67, at steps of `MAX_STEP_M`.
    const MOST_STATES: usize = 1000;

    impl<const N: usize> SelfModel<N> for CountsItsStates {
        fn clearance(&self, _: &[f64; N], _: Moving) -> f64 {
            let asked = self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            assert!(
                asked < MOST_STATES,
                "asked about more than {MOST_STATES} states"
            );
            1.0
        }

        fn nearest_pair(&self, _: &[f64; N], _: Moving) -> (String, String) {
            ("a link".to_string(), "its neighbor".to_string())
        }
    }

    // A waypoint 1e12 m along joint 1 answers that it lies outside the limit, and the walk
    // of the run to it stops where the run leaves the limit, 1 m from the start: steps of
    // `MAX_STEP_M` there, nothing near, take some 67 states, within `MOST_STATES`.
    #[test]
    fn a_waypoint_far_outside_a_limit_ends_the_walk_at_the_limit() {
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &[]);
        let geometry = gantry();
        let model = CountsItsStates(std::sync::atomic::AtomicUsize::new(0));
        let arms = one_arm(&geometry, Some(&model));
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let request = CheckRequest {
            start: [0.0, 0.0],
            start_tolerance: [0.0; 2],
            waypoints: &[[1e12, 0.0]],
            min_clearance_m: MIN_CLEARANCE_M,
            deadline: Instant::now(),
        };
        let check = check_path_until(&arms, &scene, &request, &|| false)
            .unwrap_or_else(|failure| panic!("{failure}"));
        assert!(
            matches!(
                check.fault,
                Some(PathFault::OutsideLimits { waypoint: 1, position, .. }) if position == 1e12
            ),
            "{check:?}"
        );
    }

    // A check whose deadline passes once it has walked its runs ends before it walks its
    // corner; the same check with no deadline passes.
    #[test]
    fn a_check_ends_at_its_deadline_before_the_walk_of_a_corner() {
        let path = [[0.3, 0.0], [0.3, 0.3]];
        let asked = std::cell::Cell::new(0);
        let after_the_runs = || {
            asked.set(asked.get() + 1);
            asked.get() > path.len()
        };
        assert_eq!(
            gantry_check([0.0, 0.0], &path, &[], &after_the_runs),
            Err(CheckFailure::TimeEnded)
        );
        assert_eq!(
            asked.get(),
            path.len() + 1,
            "the check asked past its corner"
        );
        let check = gantry_check([0.0, 0.0], &path, &[], &|| false)
            .unwrap_or_else(|failure| panic!("{failure}"));
        assert_eq!((check.clear_waypoints, check.fault), (2, None));
    }

    // From a start 1 cm from a voxel behind it, short of the plan's clearance, the
    // planner's motion from a state further along the x axis, clear of everything, departs
    // by the start rule from the start, and a block between the start and that state ends
    // it; the same motion beside the start's line walks by its spare and passes.
    #[test]
    fn a_planner_motion_from_the_starts_line_departs_by_the_rule() {
        let points = [[-0.025, 0.0, 0.0], [0.15, 0.0, 0.0]];
        let (map, field) = world(VOXEL_M, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], &points);
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, [0.0, 0.0], MIN_CLEARANCE_M);
        assert!(
            checks.start_short[0],
            "the start keeps the plan's clearance"
        );
        assert_eq!(
            checks.walk(&[0.3, 0.0], &[0.45, 0.0]),
            Motion::Invalid { last_valid: 0.0 }
        );
        assert_eq!(checks.walk(&[0.3, 0.05], &[0.45, 0.05]), Motion::Valid);
    }
}
