//! OMPL's geometric planners from Rust: plan a path through a real vector state
//! space with bounds, by a planner chosen by name, under the caller's state and
//! motion checks, optionally on the zero set of the caller's constraint, and
//! simplify it with OMPL's path simplifier.
//!
//! - [`plan`] runs one [`Request`] and answers a [`Plan`].
//! - [`Planner`] names the planners; [`Space`] holds the bounds.
//! - [`Goals`] are states given with the request, or goals the caller samples while each
//!   attempt plans ([`SampledGoals`]), as MoveIt samples a request's goals.
//! - [`Checks`] and [`Constraint`] are the caller's functions. An error in one ends the
//!   plan with [`PlanError::Check`] and never reaches C++, and so does a panic in a
//!   binary that unwinds on panic, Rust's default; under `panic = "abort"`, a panic in
//!   one aborts the process.
//!
//! A plan makes the request's attempts, as MoveIt makes the attempts of a request: one
//! on the thread that calls [`plan`], or several on threads of their own,
//! [`PLANNING_THREADS`] at once, whose paths join into their hybrid by OMPL's path
//! hybridization. A plan seeds every random number generator OMPL draws from in it, the
//! planners', the samplers' and the simplifier's, from the request's seed: attempt 0 from
//! the seed itself, each other attempt from a seed derived from it, and the simplifier of
//! a plan of several attempts from another. Each attempt's planner ends at whichever
//! comes first: the planner's return, the caller's stop, the request's iteration bound or
//! its time limit, and the plan's simplifier the same way, under the simplification bound
//! and the simplification's time; [`Plan::ended_by`] and [`Simplification::ended_by`] say
//! which. A plan that neither a time limit nor the caller's stop ends, in its planners or
//! in its simplifier, repeats, path for path, in the same process and in another, for
//! every planner but PRM, with any number of goals and of attempts, given or sampled by a
//! sampling that repeats. PRM and PRM* take
//! every goal into their roadmap before they search it, grow and expand the roadmap in
//! rounds of questions whether to stop, and search it on a second thread, whose questions
//! do not count toward the bound. PRM ends at the first path that search finds, and that
//! path depends on when the search runs. Otherwise the plan searches the finished roadmap
//! once the planner has ended, so PRM*'s path repeats, and whether PRM finds a path.
//! Plans run at once in a process.
//!
//! The crate builds OMPL 2.0.2 from the source archive of its release tag, which its
//! build reads from the copy that `OMPL_SOURCE_TARBALL` names, or from the download
//! cache that `PEPPY_DOWNLOAD_CACHE` names, or downloads, and checks against the
//! archive's SHA-256 digest. One patch changes the archive's sources: it gives a thread
//! a seed generator of its own.

mod callbacks;
mod ffi;

/// The README, whose quick start runs as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;

use std::fmt;
use std::num::{NonZeroU32, NonZeroU64};
use std::str::FromStr;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use callbacks::{Callbacks, Sampled};
use ffi::bridge;

/// What a caller's check or constraint answers when it cannot answer.
pub type CheckError = Box<dyn std::error::Error + Send + Sync>;

/// The [`Projection::spacing`] OMPL's projected state space has by default: its delta,
/// 0.05, times its lambda, 2 (OMPL 2.0.2 `ConstrainedStateSpace.h:53-54`).
pub const CONSTRAINED_SPACING: f64 = 0.1;

/// The attempts of a plan that plan at once, each on a thread of its own: MoveIt's
/// default `max_planning_threads`. A plan of more attempts makes them in batches of this
/// many, each batch once the one before it has ended.
pub const PLANNING_THREADS: NonZeroU32 = NonZeroU32::new(4).expect("4 is above 0");

/// A geometric planner of OMPL.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Planner {
    RrtConnect,
    Rrt,
    RrtStar,
    BiTrrt,
    Kpiece1,
    Bkpiece1,
    Lbkpiece1,
    Est,
    Sbl,
    Prm,
    PrmStar,
}

impl Planner {
    pub const ALL: [Self; 11] = [
        Self::RrtConnect,
        Self::Rrt,
        Self::RrtStar,
        Self::BiTrrt,
        Self::Kpiece1,
        Self::Bkpiece1,
        Self::Lbkpiece1,
        Self::Est,
        Self::Sbl,
        Self::Prm,
        Self::PrmStar,
    ];

    /// The name of the planner's class in OMPL.
    pub const fn name(self) -> &'static str {
        match self {
            Self::RrtConnect => "RRTConnect",
            Self::Rrt => "RRT",
            Self::RrtStar => "RRTstar",
            Self::BiTrrt => "BiTRRT",
            Self::Kpiece1 => "KPIECE1",
            Self::Bkpiece1 => "BKPIECE1",
            Self::Lbkpiece1 => "LBKPIECE1",
            Self::Est => "EST",
            Self::Sbl => "SBL",
            Self::Prm => "PRM",
            Self::PrmStar => "PRMstar",
        }
    }

    /// Whether a plan of the planner that neither the time limit nor the caller's stop
    /// ends, in its planner or in its simplifier, repeats under one seed, with any number
    /// of goals. PRM's path is the first that its search finds, and depends on when that
    /// is; whether it finds one repeats.
    pub const fn repeats(self) -> bool {
        !matches!(self, Self::Prm)
    }

    const fn bridged(self) -> bridge::Planner {
        match self {
            Self::RrtConnect => bridge::Planner::RrtConnect,
            Self::Rrt => bridge::Planner::Rrt,
            Self::RrtStar => bridge::Planner::RrtStar,
            Self::BiTrrt => bridge::Planner::BiTrrt,
            Self::Kpiece1 => bridge::Planner::Kpiece1,
            Self::Bkpiece1 => bridge::Planner::Bkpiece1,
            Self::Lbkpiece1 => bridge::Planner::Lbkpiece1,
            Self::Est => bridge::Planner::Est,
            Self::Sbl => bridge::Planner::Sbl,
            Self::Prm => bridge::Planner::Prm,
            Self::PrmStar => bridge::Planner::PrmStar,
        }
    }
}

impl fmt::Display for Planner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A name that names no planner.
#[derive(Debug, PartialEq, thiserror::Error)]
#[error("no planner is named {0:?}; the planners are {names}", names = planner_names())]
pub struct UnknownPlanner(String);

fn planner_names() -> String {
    Planner::ALL.map(Planner::name).join(", ")
}

impl FromStr for Planner {
    type Err = UnknownPlanner;

    /// The planner `name` names, spelled as OMPL spells its class.
    fn from_str(name: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|planner| planner.name() == name)
            .ok_or_else(|| UnknownPlanner(name.to_string()))
    }
}

/// A real vector state space: one interval per dimension.
pub struct Space {
    low: Vec<f64>,
    high: Vec<f64>,
}

/// Bounds that hold no space.
#[derive(Debug, thiserror::Error)]
pub enum SpaceError {
    #[error("a space has at least one dimension")]
    Empty,
    #[error(
        "dimension {dimension} runs from {low} to {high}; a dimension runs from a finite low to a finite high above it"
    )]
    Interval {
        dimension: usize,
        low: f64,
        high: f64,
    },
}

impl Space {
    /// The space of the intervals `bounds`, each from its low to its high.
    pub fn new(bounds: &[(f64, f64)]) -> Result<Self, SpaceError> {
        if bounds.is_empty() {
            return Err(SpaceError::Empty);
        }
        if let Some((dimension, &(low, high))) = bounds
            .iter()
            .enumerate()
            .find(|(_, (low, high))| !(low.is_finite() && high.is_finite() && low < high))
        {
            return Err(SpaceError::Interval {
                dimension,
                low,
                high,
            });
        }
        Ok(Self {
            low: bounds.iter().map(|b| b.0).collect(),
            high: bounds.iter().map(|b| b.1).collect(),
        })
    }

    fn dimension(&self) -> usize {
        self.low.len()
    }

    /// Why `state`, which `what` names, lies outside the space, when it does: the
    /// first of its values outside its dimension's interval.
    fn outside(&self, what: &str, state: &[f64]) -> Option<String> {
        state
            .iter()
            .zip(self.low.iter().zip(&self.high))
            .enumerate()
            .find(|(_, (value, (low, high)))| !(**low..=**high).contains(*value))
            .map(|(dimension, (value, (low, high)))| {
                format!(
                    "{what} holds {value} in dimension {dimension}, which runs from {low} to {high}"
                )
            })
    }
}

/// What a motion check answers.
#[derive(Debug, PartialEq)]
pub enum Motion {
    Valid,
    /// The motion is invalid; its last valid state lies `last_valid` of the way
    /// along it, from 0 to 1.
    Invalid {
        last_valid: f64,
    },
}

/// The caller's checks. A plan calls them on the thread that calls [`plan`], and, when it
/// makes several attempts, on the thread of each, and shares them with the threads on
/// which PRM and PRM* search their roadmap.
pub trait Checks: Sync {
    /// Whether `state` is valid.
    fn state(&self, state: &[f64]) -> Result<bool, CheckError>;

    /// Whether the straight motion from `from` to `to` is valid, `to` included: OMPL
    /// takes `from` as valid and judges `to` by this answer alone, as a planner that
    /// steps toward a sampled state does. A constrained plan walks each motion along the
    /// constraint and asks this of each straight stretch of the walk, the steps in one
    /// line joined in one motion.
    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError>;
}

/// The caller's constraint: the states whose function is zero. A plan calls it on the
/// thread that calls [`plan`], and, when it makes several attempts, on the thread of
/// each, and shares it with the threads on which PRM and PRM* search their roadmap.
pub trait Constraint: Sync {
    /// The number of the function's components.
    fn codimension(&self) -> usize;

    /// Writes the function at `state` into `out`, one component per value. `out`
    /// holds NaN when the call starts, and a value the call leaves unwritten stays
    /// NaN, which OMPL takes as off the constraint.
    fn function(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError>;

    /// Writes the function's Jacobian at `state` into `out`, row by row: one row
    /// per component, one column per dimension of the space. `out` holds NaN when
    /// the call starts.
    fn jacobian(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError>;
}

/// A constraint a plan projects its states onto, as OMPL's projected state space
/// does: within `tolerance` of the zero set, in at most `iterations` Newton steps
/// per projection.
#[derive(Clone, Copy)]
pub struct Projection<'a> {
    pub constraint: &'a dyn Constraint,
    pub tolerance: f64,
    pub iterations: NonZeroU32,
    /// The farthest apart two consecutive states of the plan's path lie, in the path and
    /// in its simplification. The plan walks each motion along the constraint in steps of
    /// half this length, as OMPL's projected state space walks one, and OMPL takes a
    /// motion whose step stretches beyond this length as invalid.
    pub spacing: f64,
}

/// The goals of a plan.
pub enum Goals<'a> {
    /// The states that end the plan, each within the space's bounds. The plan reaches
    /// one of those that pass the state check and lie on the constraint.
    States(&'a [Vec<f64>]),
    /// Goals the caller samples while each attempt plans, as MoveIt's
    /// `ConstrainedGoalSampler` samples a request's goals beside its planner (moveit2
    /// 2.15.2 constrained_goal_sampler.cpp), here on the attempt's thread when its planner
    /// asks for a goal: once the planner has been handed every goal found, each question
    /// samples once, and before the first goal the question samples until one is found or
    /// the sampling ends, as a planner waits for its first goal. RRTConnect asks for its
    /// first goal before it plans, and for another whenever it has been handed fewer goals
    /// than half its goal tree's states; PRM and PRM* take goals until the sampling ends,
    /// before their roadmap grows. A goal found before the time limit joins the attempt's
    /// goals when it passes the state check and lies on the constraint. An attempt's
    /// sampling ends at the
    /// caller's [`GoalSample::Ended`], and no sample starts once the plan has been stopped
    /// or a check has failed, or past the time limit; a sampling that neither finds a goal
    /// nor ends keeps a planner that waits for its first goal waiting until then, and the
    /// planner then answers no path, as one ended later does.
    Sampled(&'a dyn SampledGoals),
}

/// The caller's goal sampling: a sampling of each attempt's goals.
pub trait SampledGoals: Sync {
    /// The sampling of attempt `attempt`'s goals, attempt 0 the first, which the plan
    /// starts at the attempt's first goal sample, on the attempt's thread.
    fn sampling(&self, attempt: u32) -> Box<dyn GoalSampling + '_>;
}

/// The goal sampling of one attempt of a plan. An error or a panic in it, in the error's
/// message or in its drop, ends the plan with [`PlanError::Check`].
pub trait GoalSampling: Send {
    /// One goal sample: what it found, and the goal, when it found one, written into
    /// `goal`, one value per dimension of the space.
    fn sample(&mut self, goal: &mut [f64]) -> Result<GoalSample, CheckError>;
}

/// What one goal sample found.
#[derive(Debug, PartialEq)]
pub enum GoalSample {
    /// A goal, written into the sample's `goal`. A goal outside the space's bounds fails
    /// the plan with [`PlanError::Check`].
    Found,
    /// No goal; a later sample may find one.
    Missed,
    /// No goal, and the sampling has ended: its attempt asks for no other.
    Ended,
}

/// One plan.
pub struct Request<'a> {
    pub planner: Planner,
    pub space: &'a Space,
    /// A state within the space's bounds.
    pub start: &'a [f64],
    pub goals: Goals<'a>,
    /// The time limit of the plan's attempts, from the call of `plan`. The planners end
    /// at their first question whether to stop past the limit, and no batch of attempts
    /// starts past it. What runs between two questions runs to its end, a call of the
    /// caller's functions among it, and so do the checks of the start and the goals and
    /// the hybrid of the attempts' paths, with the motion checks that join the paths and
    /// those of the hybrid. So do two steps of a plan of PRM or PRM*: taking every goal
    /// into the roadmap, with the motion checks of each, before the first question, and
    /// searching the finished roadmap once the planner has ended, with one A* search for
    /// each goal that the roadmap joins to the start, unless PRM's search found a path
    /// that ended the planner. PRM* planning the 7-D wall of the crate's tests until the
    /// limit answered 14 ms past a limit of 1 s with one goal and 55 ms past a limit of
    /// 3 s with 32 goals, as measured on an x86-64 host.
    pub time_limit: Duration,
    /// The times each attempt's planner may ask whether to stop, on the attempt's thread,
    /// before it stops. A planner's wait for its first sampled goal asks no such question.
    pub iterations: NonZeroU64,
    pub seed: NonZeroU32,
    /// The attempts the plan makes, as MoveIt's `num_planning_attempts`: attempt 0 from
    /// `seed`, and each other from a seed derived from it. One attempt plans on the
    /// thread that calls `plan`. Several plan on threads of their own, in batches of
    /// [`PLANNING_THREADS`], each batch once the one before it has ended, and the plan
    /// keeps the shortest of their paths and of the hybrid that OMPL's path hybridization
    /// joins of two or more of them, the hybrid when it passes the checks in its order.
    pub attempts: NonZeroU32,
    /// The times the simplifier may ask whether to stop before it stops, once its
    /// first pass, which runs to its end, has ended; none skips the simplification.
    pub simplification: Option<NonZeroU32>,
    /// The time limit of the simplification, counted from the end of `time_limit`. The
    /// simplification starts past it too, as MoveIt's `simplifySolution` starts with
    /// whatever time its plan left, and its first pass runs to its end, which OMPL
    /// bounds by counts of attempts, most of them the path's number of states; the
    /// simplifier ends at its first question whether to stop past it after that pass.
    pub simplification_time: Duration,
    pub projection: Option<Projection<'a>>,
    /// A flag the caller sets to end the plan: the planners and the simplifier end at
    /// their next question whether to stop, and no attempt, hybrid or simplification
    /// starts once it is set.
    pub stop: Option<&'a AtomicBool>,
}

/// What ended a plan's planner, or its simplifier.
#[derive(Debug, PartialEq)]
pub enum Ending {
    /// The planner, or the simplifier, returned of its own accord.
    Returned,
    /// The request's time limit passed while a planner planned or before an attempt
    /// started, or the simplification's time passed by a question of its simplifier
    /// after its first pass.
    TimeLimit,
    /// The planner asked whether to stop once more after the request's iteration
    /// bound, or the simplifier after its simplification bound.
    IterationBound,
    /// The caller's stop flag ended a planner or the simplifier, or kept an attempt or
    /// the simplifier from starting.
    Stopped,
}

/// The answer to a request.
#[derive(Debug, PartialEq)]
pub struct Plan {
    /// The states of the path from the start to a goal, when the planner found an
    /// exact one. On a constrained plan, consecutive states lie at most the projection's
    /// spacing apart, and each state the plan adds between two states of the planner lies
    /// within the projection's tolerance of the constraint.
    pub path: Option<Vec<Vec<f64>>>,
    /// The simplification of the path, when the request asked for it and the planner
    /// found an exact path.
    pub simplified: Option<Simplification>,
    /// What ended the planner. Of several attempts, what ended any of them: the caller's
    /// stop first, then the time limit, then the iteration bound; and their return when
    /// each returned.
    pub ended_by: Ending,
}

/// A path after simplification.
#[derive(Debug, PartialEq)]
pub struct Simplification {
    /// OMPL's simplification of the planned path when OMPL finds it valid and it is at
    /// most as long, and the planned path otherwise. Its states lie as those of
    /// [`Plan::path`] do.
    pub path: Vec<Vec<f64>>,
    /// What ended the simplifier.
    pub ended_by: Ending,
}

/// Why a request gives no plan.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum PlanError {
    #[error("{0}")]
    Request(String),
    #[error("the start fails the state check or lies off the constraint")]
    InvalidStart,
    #[error("every goal fails the state check or lies off the constraint")]
    InvalidGoal,
    #[error("a check, the constraint or the goal sampling failed: {0}")]
    Check(String),
    #[error("OMPL: {0}")]
    Ompl(String),
}

/// Plans `request` under `checks`.
pub fn plan(request: &Request, checks: &dyn Checks) -> Result<Plan, PlanError> {
    if callbacks::in_callback() {
        return Err(PlanError::Request(
            "plan was called from a check, the constraint or the goal sampling of a plan; a plan seeds the generator of its thread, which the other plan draws from, so call plan outside the checks of another"
                .to_string(),
        ));
    }
    let ffi_request = ffi_request(request)?;
    let sampled = match request.goals {
        Goals::States(_) => None,
        Goals::Sampled(goals) => Some(Sampled::new(goals, request.space)),
    };
    let callbacks = Callbacks::new(
        checks,
        request.projection.map(|p| p.constraint),
        sampled,
        request.stop,
    );
    let answer = bridge::plan(&ffi_request, &callbacks);
    callbacks.end_samplings();
    if let Some(failure) = callbacks.failure() {
        return Err(PlanError::Check(failure));
    }
    let answer = answer.map_err(|exception| PlanError::Ompl(exception.what().to_string()))?;
    let dimension = request.space.dimension();
    let ended_by = ending(answer.ended_by);
    let unflattened = |values: &[f64]| -> Vec<Vec<f64>> {
        values
            .chunks_exact(dimension)
            .map(<[f64]>::to_vec)
            .collect()
    };
    match answer.status {
        bridge::Status::Exact => Ok(Plan {
            path: Some(unflattened(&answer.path)),
            simplified: request.simplification.map(|_| Simplification {
                path: unflattened(&answer.simplified),
                ended_by: ending(answer.simplification_ended_by),
            }),
            ended_by,
        }),
        bridge::Status::Approximate | bridge::Status::NotSolved => Ok(Plan {
            path: None,
            simplified: None,
            ended_by,
        }),
        bridge::Status::InvalidStart => Err(PlanError::InvalidStart),
        bridge::Status::InvalidGoal => Err(PlanError::InvalidGoal),
        _ => Err(PlanError::Ompl(answer.status_name)),
    }
}

/// What the bridge says ended a planner or a simplifier of a plan whose checks did not
/// fail.
fn ending(ended_by: bridge::Ending) -> Ending {
    match ended_by {
        bridge::Ending::Returned => Ending::Returned,
        bridge::Ending::TimeLimit => Ending::TimeLimit,
        bridge::Ending::IterationBound => Ending::IterationBound,
        bridge::Ending::Stopped => Ending::Stopped,
        other => unreachable!(
            "the bridge ends a plan by a failure only when a check failed, not by {}",
            other.repr
        ),
    }
}

/// `count` and `noun`, in the plural unless `count` is 1.
fn counted(count: usize, noun: &str) -> String {
    if count == 1 {
        format!("1 {noun}")
    } else {
        format!("{count} {noun}s")
    }
}

/// `request` as the bridge takes it, once its states fit its space.
fn ffi_request(request: &Request) -> Result<bridge::PlanRequest, PlanError> {
    let space = request.space;
    let dimension = space.dimension();
    let refuse = |message: String| Err(PlanError::Request(message));
    if request.start.len() != dimension {
        return refuse(format!(
            "the start has {}; the space has {}",
            counted(request.start.len(), "value"),
            counted(dimension, "dimension")
        ));
    }
    let (given, sampled_goals) = match request.goals {
        Goals::States([]) => return refuse("a plan has at least one goal".to_string()),
        Goals::States(goals) => (goals, false),
        Goals::Sampled(_) => (&[][..], true),
    };
    if let Some(goal) = given.iter().find(|goal| goal.len() != dimension) {
        return refuse(format!(
            "a goal has {}; the space has {}",
            counted(goal.len(), "value"),
            counted(dimension, "dimension")
        ));
    }
    let outside = space.outside("the start", request.start).or_else(|| {
        given
            .iter()
            .enumerate()
            .find_map(|(index, goal)| space.outside(&format!("goal {index}"), goal))
    });
    if let Some(outside) = outside {
        return refuse(format!("{outside}; a state lies within the space's bounds"));
    }
    if request.time_limit.is_zero() {
        return refuse("the time limit is 0; a plan needs time to plan".to_string());
    }
    let (codimension, tolerance, projection_iterations, constrained_spacing) = match request
        .projection
    {
        None => (0, 0.0, 0, CONSTRAINED_SPACING),
        Some(Projection {
            constraint,
            tolerance,
            iterations,
            spacing,
        }) => {
            let codimension =
                callbacks::caught(|| Ok(constraint.codimension())).map_err(PlanError::Check)?;
            if codimension == 0 || codimension >= dimension {
                return refuse(format!(
                    "a constraint of {} in {}; a constraint has at least 1 component, and fewer components than the space has dimensions",
                    counted(codimension, "component"),
                    counted(dimension, "dimension")
                ));
            }
            if !(tolerance.is_finite() && tolerance > 0.0) {
                return refuse(format!(
                    "the projection tolerance is {tolerance}; it is a positive number"
                ));
            }
            if !(spacing.is_finite() && spacing > 0.0) {
                return refuse(format!(
                    "the projection spacing is {spacing}; it is a positive number"
                ));
            }
            (
                u32::try_from(codimension).expect("fewer components than dimensions"),
                tolerance,
                iterations.get(),
                spacing,
            )
        }
    };
    Ok(bridge::PlanRequest {
        planner: request.planner.bridged(),
        low: space.low.clone(),
        high: space.high.clone(),
        start: request.start.to_vec(),
        goals: given.iter().flatten().copied().collect(),
        sampled_goals,
        time_limit_s: request.time_limit.as_secs_f64(),
        iterations: request.iterations.get(),
        seed: request.seed.get(),
        attempts: request.attempts.get(),
        planning_threads: PLANNING_THREADS.get(),
        simplification_iterations: request.simplification.map_or(0, NonZeroU32::get),
        simplification_time_s: request.simplification_time.as_secs_f64(),
        codimension,
        tolerance,
        projection_iterations,
        constrained_spacing,
    })
}
