//! Plans in worlds of box obstacles, with every planner: their paths pass the
//! checks, repeat under one seed in this process and in another, with several goals and
//! on a slow host, simplify to paths at most as long, and walk a constraint; a request
//! that does not fit its space, or whose start or goals fail the checks, is refused; a
//! failure in a check or the constraint ends a plan without ending the process; a plan
//! says what ended its planner and its simplifier; and the time limit bounds a plan from
//! its call.

use std::fmt;
use std::num::{NonZeroU32, NonZeroU64};
use std::ops::RangeInclusive;
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock, mpsc};
use std::thread;
use std::time::Duration;

mod world;

use ompl_bridge::{
    CONSTRAINED_SPACING, CheckError, Checks, Constraint, Ending, Motion, Plan, PlanError, Planner,
    Projection, Request, Space, plan,
};
use world::{Obstacle, World, distance, length};

/// The step the worlds here walk a motion in.
const MOTION_STEP: f64 = 0.01;
/// The iterations every plan here may take: enough for each planner to solve the worlds
/// here, the bound that ends the anytime planners, and more than a round of growth and a
/// round of expansion of the roadmap of PRM and PRM*.
const ITERATIONS: u64 = 2_000;
/// A time limit that never passes: a plan under it has no deadline.
const NO_TIME_LIMIT: Duration = Duration::MAX;
/// The time limit of a plan here that its time limit ends.
const SHORT_TIME_LIMIT: Duration = Duration::from_millis(10);
/// How long a test here waits for a plan that its time limit ends before it fails, so a
/// plan that never answers fails the test.
const ANSWER_GUARD: Duration = Duration::from_secs(60);
/// The environment variable under which a test here runs alone in its process, which
/// `run_alone` starts.
const ALONE: &str = "OMPL_BRIDGE_TEST_ALONE";
/// The seeds a test plans PRM under, whose path depends on when its search finds one.
const ROADMAP_SEEDS: RangeInclusive<u32> = 1..=5;
/// The numbers of goals of the plans of PRM and PRM* with several goals.
const GOAL_COUNTS: [usize; 2] = [2, 8];
/// The plans of one request in the tests that a plan of PRM or PRM* repeats.
const REPEATS: usize = 4;
/// How long the slow host's first check of a state of the roadmap takes: longer than
/// OMPL's first round of roadmap growth, 0.4 s.
const SLOW_CHECK: Duration = Duration::from_millis(450);
/// The state of the roadmap that the planning thread samples after its last question
/// before the bound, in the test of the search of the finished roadmap.
const FINAL_ROADMAP_STATE: usize = 10;
/// The iteration bound of the test of the search of the finished roadmap: the questions
/// the planning thread asks up to the one before it samples its `FINAL_ROADMAP_STATE`th
/// state of the roadmap, one before its first round of growth and two for each state.
const FINAL_SEARCH_BOUND: u64 = 2 * FINAL_ROADMAP_STATE as u64 + 1;
/// The tolerance of the constrained plans here.
const TOLERANCE: f64 = 1e-4;
/// The simplification iterations of the plans here that simplify.
const SIMPLIFICATION: u32 = 100;
/// What `plan` says when a check calls it.
const REENTERED: &str = "plan was called from a check or the constraint of a plan; a process runs one plan at a time, so call plan outside the checks of another";

/// A wall across the first dimension at its middle, open at the top of the second
/// dimension, and a lower block before it, each whole across every other dimension.
fn wall(dimension: usize) -> World {
    let slab = |first: (f64, f64), second: (f64, f64)| {
        let mut min = vec![-1.0; dimension];
        let mut max = vec![2.0; dimension];
        (min[0], max[0]) = first;
        (min[1], max[1]) = second;
        Obstacle { min, max }
    };
    World {
        obstacles: vec![
            slab((0.45, 0.55), (-1.0, 0.8)),
            slab((0.2, 0.3), (-1.0, 0.6)),
        ],
        motion_step: MOTION_STEP,
    }
}

fn unit_cube(dimension: usize) -> Space {
    Space::new(&vec![(0.0, 1.0); dimension]).expect("a valid space")
}

fn request<'a>(
    planner: Planner,
    space: &'a Space,
    start: &'a [f64],
    goals: &'a [Vec<f64>],
    seed: u32,
) -> Request<'a> {
    Request {
        planner,
        space,
        start,
        goals,
        time_limit: NO_TIME_LIMIT,
        iterations: NonZeroU64::new(ITERATIONS).expect("nonzero"),
        seed: NonZeroU32::new(seed).expect("nonzero"),
        simplification: None,
        projection: None,
    }
}

/// The unit cube of a wall's dimensions, with a start and a goal on either side of the
/// wall, low in the second dimension.
struct WallEnds {
    space: Space,
    start: Vec<f64>,
    goals: [Vec<f64>; 1],
}

impl WallEnds {
    fn new(dimension: usize) -> Self {
        let mut start = vec![0.5; dimension];
        let mut goal = vec![0.5; dimension];
        (start[0], start[1]) = (0.1, 0.1);
        (goal[0], goal[1]) = (0.9, 0.1);
        Self {
            space: unit_cube(dimension),
            start,
            goals: [goal],
        }
    }

    fn goal(&self) -> &[f64] {
        &self.goals[0]
    }

    /// The request of `planner` from the start to the goal under `seed`.
    fn request(&self, planner: Planner, seed: u32) -> Request<'_> {
        request(planner, &self.space, &self.start, &self.goals, seed)
    }
}

/// The plan of `planner` through the wall of `dimension` dimensions under `seed`,
/// simplified within `simplification` iterations when it is not 0.
fn wall_plan_simplified(
    planner: Planner,
    dimension: usize,
    seed: u32,
    simplification: u32,
) -> Plan {
    let ends = WallEnds::new(dimension);
    let request = Request {
        simplification: NonZeroU32::new(simplification),
        ..ends.request(planner, seed)
    };
    plan(&request, &wall(dimension)).expect("a plan")
}

/// The plan of `planner` through the wall of `dimension` dimensions under `seed`.
fn wall_plan(planner: Planner, dimension: usize, seed: u32) -> Plan {
    wall_plan_simplified(planner, dimension, seed, 0)
}

/// The seeds a test plans `planner` under: seed 1 for a planner whose plans repeat,
/// and several for PRM.
fn seeds(planner: Planner) -> RangeInclusive<u32> {
    if planner.repeats() {
        1..=1
    } else {
        ROADMAP_SEEDS
    }
}

/// The plans of `planner` under its seeds that found a path, at least one.
fn found(planner: Planner, what: &str, plan_under: impl Fn(u32) -> Plan) -> Vec<Plan> {
    let found: Vec<Plan> = seeds(planner)
        .map(plan_under)
        .filter(|plan| plan.path.is_some())
        .collect();
    assert!(!found.is_empty(), "{what} found no path");
    found
}

/// Asserts that `path` runs from `start` to `goal` through states and motions that
/// pass `world`'s checks.
fn assert_passes(world: &World, path: &[Vec<f64>], ends: &WallEnds, what: &str) {
    assert_eq!(
        path.first().map(Vec::as_slice),
        Some(ends.start.as_slice()),
        "{what} starts elsewhere"
    );
    assert_eq!(
        path.last().map(Vec::as_slice),
        Some(ends.goal()),
        "{what} ends elsewhere"
    );
    for state in path {
        assert!(world.free(state), "{what} passes through {state:?}");
    }
    for w in path.windows(2) {
        assert_eq!(
            world.walk(&w[0], &w[1]),
            Motion::Valid,
            "{what} moves through the wall from {:?} to {:?}",
            w[0],
            w[1]
        );
    }
}

// In a 2-D and a 7-D world of two boxes, every planner returns a path whose every
// state and motion pass the checks: a repeating planner under seed 1, and PRM under
// each of several seeds, finding a path under one of them at least.
#[test]
fn every_planner_finds_a_path_that_passes_the_checks() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        for planner in Planner::ALL {
            let what = format!("{planner} in {dimension} dimensions");
            for answer in found(planner, &what, |seed| wall_plan(planner, dimension, seed)) {
                let path = answer.path.expect("a found path");
                assert_passes(&world, &path, &ends, &what);
            }
        }
    }
}

// Plans with the seeds 1, 2 and 1 give equal first and third paths, simplifications and
// endings, and a different second path, for every planner whose plans repeat.
#[test]
fn a_plan_repeats_under_its_seed() {
    for dimension in [2, 7] {
        for planner in Planner::ALL.into_iter().filter(|p| p.repeats()) {
            let plan_under = |seed| wall_plan_simplified(planner, dimension, seed, SIMPLIFICATION);
            let (first, second, third) = (plan_under(1), plan_under(2), plan_under(1));
            assert_eq!(first, third, "{planner} in {dimension} dimensions");
            assert_ne!(first.path, second.path, "{planner} ignores its seed");
        }
    }
}

/// The path of a 7-D plan of every repeatable planner with seed 1, as the bits of its
/// values, one planner per line.
fn path_bits() -> String {
    Planner::ALL
        .into_iter()
        .filter(|p| p.repeats())
        .map(|planner| {
            let path = wall_plan(planner, 7, 1).path.expect("a path");
            let bits: Vec<String> = path
                .iter()
                .flatten()
                .map(|v| format!("{:016x}", v.to_bits()))
                .collect();
            format!("{planner} {}", bits.join(" "))
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Whether this process runs one test alone, as `run_alone` starts it.
fn alone() -> bool {
    std::env::var_os(ALONE).is_some()
}

/// Runs the test `name` alone in another process of this test binary, and answers what
/// it printed, once it passed.
fn run_alone(name: &str) -> String {
    let output = Command::new(std::env::current_exe().expect("the test binary"))
        .args(["--exact", name, "--nocapture"])
        .env(ALONE, "1")
        .output()
        .expect("the other process runs");
    assert!(
        output.status.success(),
        "{name} failed in a process of its own: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).expect("UTF-8")
}

// A plan with seed 1 in another process gives the paths that it gives in this one.
#[test]
fn a_plan_repeats_in_another_process() {
    if alone() {
        println!("BEGIN\n{}\nEND", path_bits());
        return;
    }
    let stdout = run_alone("a_plan_repeats_in_another_process");
    let theirs = stdout
        .split_once("BEGIN\n")
        .and_then(|(_, rest)| rest.split_once("\nEND"))
        .map(|(paths, _)| paths)
        .expect("the other process prints its paths");
    assert_eq!(theirs, path_bits());
}

// Simplification gives a path at most as long as the planned one, whose states and
// motions pass the checks, for every planner.
#[test]
fn a_simplified_path_is_at_most_as_long_and_passes_the_checks() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        for planner in Planner::ALL {
            let what = format!("{planner}'s simplified path in {dimension} dimensions");
            let plan_under = |seed| wall_plan_simplified(planner, dimension, seed, SIMPLIFICATION);
            for answer in found(planner, &what, plan_under) {
                let path = answer.path.expect("a found path");
                let simplified = answer.simplified.expect("a simplified path").path;
                assert!(length(&simplified) <= length(&path), "{what} is longer");
                assert_passes(&world, &simplified, &ends, &what);
            }
        }
    }
}

// RRT's path through the 2-D wall under seed 1 turns where it need not, and its
// simplification is strictly shorter.
#[test]
fn simplification_shortens_a_path_that_turns_where_it_need_not() {
    let plan = wall_plan_simplified(Planner::Rrt, 2, 1, SIMPLIFICATION);
    let path = plan.path.expect("a path");
    let simplified = plan.simplified.expect("a simplified path").path;
    assert!(
        length(&simplified) < length(&path),
        "the simplified path is {} long, the path {}",
        length(&simplified),
        length(&path)
    );
}

/// The unit sphere about the origin: the states whose norm is 1.
struct UnitSphere;

impl Constraint for UnitSphere {
    fn codimension(&self) -> usize {
        1
    }

    fn function(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        out[0] = norm(state) - 1.0;
        Ok(())
    }

    fn jacobian(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        let n = norm(state);
        for (o, x) in out.iter_mut().zip(state) {
            *o = x / n;
        }
        Ok(())
    }
}

fn norm(state: &[f64]) -> f64 {
    state.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// A band around the sphere's equator, open at the far side of the second axis.
struct Equator;

impl Checks for Equator {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        Ok(!(state[2].abs() < 0.1 && state[1] < 0.5))
    }

    fn motion(&self, _: &[f64], _: &[f64]) -> Result<Motion, CheckError> {
        Err("a constrained plan checks its motions by its states".into())
    }
}

/// The space, the start and the goal of the plans on the sphere: from one pole to the
/// other.
fn sphere_ends() -> (Space, Vec<f64>, Vec<f64>) {
    let space = Space::new(&[(-2.0, 2.0); 3]).expect("a valid space");
    (space, vec![0.0, 0.0, 1.0], vec![0.0, 0.0, -1.0])
}

fn on_the_sphere<'a>(
    planner: Planner,
    space: &'a Space,
    start: &'a [f64],
    goals: &'a [Vec<f64>],
    constraint: &'a dyn Constraint,
) -> Request<'a> {
    Request {
        projection: Some(Projection {
            constraint,
            tolerance: TOLERANCE,
            iterations: NonZeroU32::new(50).expect("nonzero"),
        }),
        ..request(planner, space, start, goals, 1)
    }
}

/// Asserts that `path` runs from `start` to `goal` through states on the sphere and
/// clear of the band, each at most `CONSTRAINED_SPACING` from the one before it.
fn assert_walks_the_sphere(path: &[Vec<f64>], start: &[f64], goal: &[f64], what: &str) {
    assert_eq!(path.first().map(Vec::as_slice), Some(start), "{what}");
    assert_eq!(path.last().map(Vec::as_slice), Some(goal), "{what}");
    for state in path {
        assert!(
            (norm(state) - 1.0).abs() <= TOLERANCE,
            "{what} leaves the sphere at {state:?}"
        );
        assert!(
            Equator.state(state).expect("a check"),
            "{what} crosses the band"
        );
    }
    for w in path.windows(2) {
        assert!(
            distance(&w[0], &w[1]) <= CONSTRAINED_SPACING,
            "{what} steps {} from {:?} to {:?}",
            distance(&w[0], &w[1]),
            w[0],
            w[1]
        );
    }
}

// A constrained plan from one pole of the unit sphere to the other gives a path and a
// simplification whose every state lies within the constraint's tolerance, each at
// most the constrained spacing from the one before it.
#[test]
fn a_constrained_plan_walks_the_constraint() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal.clone()];
    for planner in [Planner::RrtConnect, Planner::Rrt, Planner::Est] {
        let request = Request {
            simplification: NonZeroU32::new(SIMPLIFICATION),
            ..on_the_sphere(planner, &space, &start, &goals, &UnitSphere)
        };
        let answer = plan(&request, &Equator).expect("a plan");
        let path = answer
            .path
            .unwrap_or_else(|| panic!("{planner} found no path on the sphere"));
        assert_walks_the_sphere(&path, &start, &goal, &format!("{planner}'s path"));
        let simplified = answer.simplified.expect("a simplified path").path;
        let what = format!("{planner}'s simplified path");
        assert_walks_the_sphere(&simplified, &start, &goal, &what);
    }
}

/// The unit sphere, whose function and Jacobian err from their `fail_at`th call on,
/// counting the calls of both, and which records whether every call found its `out`
/// filled with NaN.
struct FailingSphere {
    calls: AtomicUsize,
    fail_at: usize,
    out_was_nan: AtomicBool,
}

impl FailingSphere {
    fn new(fail_at: usize) -> Self {
        Self {
            calls: AtomicUsize::new(0),
            fail_at,
            out_was_nan: AtomicBool::new(true),
        }
    }

    fn call(&self, out: &[f64]) -> Result<(), CheckError> {
        if !out.iter().all(|v| v.is_nan()) {
            self.out_was_nan.store(false, Ordering::SeqCst);
        }
        if self.calls.fetch_add(1, Ordering::SeqCst) + 1 >= self.fail_at {
            return Err("the constraint could not judge".into());
        }
        Ok(())
    }
}

impl Constraint for FailingSphere {
    fn codimension(&self) -> usize {
        1
    }

    fn function(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        self.call(out)?;
        UnitSphere.function(state, out)
    }

    fn jacobian(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        self.call(out)?;
        UnitSphere.jacobian(state, out)
    }
}

// A constraint that fails, on the start's test or while the planner projects, ends
// its plan with an error, and every call of the constraint finds its output NaN.
#[test]
fn a_failing_constraint_ends_its_plan_and_finds_its_output_nan() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal];
    for fail_at in [1, 50] {
        let sphere = FailingSphere::new(fail_at);
        let request = on_the_sphere(Planner::RrtConnect, &space, &start, &goals, &sphere);
        assert_eq!(
            plan(&request, &Equator),
            Err(PlanError::Check(
                "the constraint could not judge".to_string()
            )),
            "failing at call {fail_at}"
        );
        assert!(
            sphere.out_was_nan.load(Ordering::SeqCst),
            "a call found its output other than NaN"
        );
    }
}

/// The unit sphere, whose codimension panics.
struct PanickingCodimension;

impl Constraint for PanickingCodimension {
    fn codimension(&self) -> usize {
        panic!("the codimension broke")
    }

    fn function(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        UnitSphere.function(state, out)
    }

    fn jacobian(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        UnitSphere.jacobian(state, out)
    }
}

// A constraint whose codimension panics ends its plan with an error that says so, and
// the next plan in the process succeeds.
#[test]
fn a_panicking_codimension_ends_its_plan() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal];
    let request = on_the_sphere(
        Planner::RrtConnect,
        &space,
        &start,
        &goals,
        &PanickingCodimension,
    );
    assert_eq!(
        plan(&request, &Equator),
        Err(PlanError::Check(
            "panicked: the codimension broke".to_string()
        ))
    );
    assert!(
        wall_plan(Planner::RrtConnect, 2, 1).path.is_some(),
        "the plan after the panic found no path"
    );
}

/// The check of a `Failing` world that fails.
#[derive(Clone, Copy, Debug, PartialEq)]
enum FailingCheck {
    State,
    Motion,
}

/// How a `Failing` world's check fails.
#[derive(Clone, Copy, Debug)]
enum Failure {
    Panics,
    Errs,
    /// Errs with an error whose message panics.
    ErrsUnprintably,
    /// Errs with an error whose drop panics.
    ErrsWithAPanickingDrop,
    /// Panics with a payload whose drop panics.
    PanicsWithAPanickingPayload,
}

impl Failure {
    const ALL: [Self; 5] = [
        Self::Panics,
        Self::Errs,
        Self::ErrsUnprintably,
        Self::ErrsWithAPanickingDrop,
        Self::PanicsWithAPanickingPayload,
    ];

    /// What `plan` says of the failure.
    fn says(self) -> &'static str {
        match self {
            Self::Panics => "panicked: the check broke",
            Self::Errs => "the check could not judge",
            Self::ErrsUnprintably => "panicked: the error's message broke",
            Self::ErrsWithAPanickingDrop => "panicked: the error's drop broke",
            Self::PanicsWithAPanickingPayload => "panicked: a panic with no message",
        }
    }

    fn fail<T>(self) -> Result<T, CheckError> {
        match self {
            Self::Panics => panic!("the check broke"),
            Self::Errs => Err("the check could not judge".into()),
            Self::ErrsUnprintably => Err(Box::new(Unprintable)),
            Self::ErrsWithAPanickingDrop => Err(Box::new(PanicsWhenDropped)),
            Self::PanicsWithAPanickingPayload => std::panic::panic_any(PanicsWhenDropped),
        }
    }
}

/// An error whose message panics.
#[derive(Debug)]
struct Unprintable;

impl fmt::Display for Unprintable {
    fn fmt(&self, _: &mut fmt::Formatter<'_>) -> fmt::Result {
        panic!("the error's message broke")
    }
}

impl std::error::Error for Unprintable {}

/// An error, and a panic's payload, whose drop panics.
#[derive(Debug)]
struct PanicsWhenDropped;

impl fmt::Display for PanicsWhenDropped {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("an error that panics when dropped")
    }
}

impl std::error::Error for PanicsWhenDropped {}

impl Drop for PanicsWhenDropped {
    fn drop(&mut self) {
        panic!("the error's drop broke")
    }
}

/// The wall, one of whose checks fails by `failure` from its `fail_at`th call on.
struct Failing {
    world: World,
    check: FailingCheck,
    calls: AtomicUsize,
    fail_at: usize,
    failure: Failure,
}

impl Failing {
    fn new(check: FailingCheck, fail_at: usize, failure: Failure) -> Self {
        Self {
            world: wall(2),
            check,
            calls: AtomicUsize::new(0),
            fail_at,
            failure,
        }
    }

    /// `answer`, or the failure once the failing check has been called `fail_at` times.
    fn answer<T>(&self, check: FailingCheck, answer: T) -> Result<T, CheckError> {
        if check == self.check && self.calls.fetch_add(1, Ordering::SeqCst) + 1 >= self.fail_at {
            return self.failure.fail();
        }
        Ok(answer)
    }
}

impl Checks for Failing {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.answer(FailingCheck::State, self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.answer(FailingCheck::Motion, self.world.walk(from, to))
    }
}

// A state check or a motion check that panics or errs, an error whose message or drop
// panics, and a panic whose payload's drop panics, each end a plan of RRTConnect, PRM
// or PRM* with an error that says what failed, and the next plan in the process
// succeeds.
#[test]
fn a_failing_check_ends_its_plan_and_the_next_plan_succeeds() {
    let ends = WallEnds::new(2);
    for planner in [Planner::RrtConnect, Planner::Prm, Planner::PrmStar] {
        for check in [FailingCheck::State, FailingCheck::Motion] {
            for failure in Failure::ALL {
                let failing = Failing::new(check, 5, failure);
                assert_eq!(
                    plan(&ends.request(planner, 1), &failing),
                    Err(PlanError::Check(failure.says().to_string())),
                    "{planner} {check:?} {failure:?}"
                );
                assert!(
                    wall_plan(planner, 2, 1).path.is_some(),
                    "the plan of {planner} after a failure found no path"
                );
            }
        }
    }
}

// A motion check that fails on its first call past the planner's last ends the plan
// in its simplification, with an error.
#[test]
fn a_failure_in_the_simplification_ends_the_plan() {
    let ends = WallEnds::new(2);
    let planner_only = Failing::new(FailingCheck::Motion, usize::MAX, Failure::Errs);
    plan(&ends.request(Planner::Rrt, 1), &planner_only)
        .expect("a plan")
        .path
        .expect("a path");
    let planner_calls = planner_only.calls.load(Ordering::SeqCst);
    let failing = Failing::new(FailingCheck::Motion, planner_calls + 1, Failure::Errs);
    let request = Request {
        simplification: NonZeroU32::new(SIMPLIFICATION),
        ..ends.request(Planner::Rrt, 1)
    };
    assert_eq!(
        plan(&request, &failing),
        Err(PlanError::Check(Failure::Errs.says().to_string()))
    );
}

/// `count` goals beyond the wall of `dimension` dimensions, spread along its second
/// dimension, each in the middle of every other dimension but the first.
fn goals_beyond_the_wall(dimension: usize, count: usize) -> Vec<Vec<f64>> {
    (0..count)
        .map(|k| {
            let mut goal = vec![0.5; dimension];
            (goal[0], goal[1]) = (0.9, 0.1 + 0.8 * k as f64 / count as f64);
            goal
        })
        .collect()
}

/// What repeats of a plan of a planner under one seed: the plan, or whether a plan of PRM
/// found a path.
#[derive(Debug, PartialEq)]
enum Repeating {
    Plan(Plan),
    FoundAPath(bool),
}

fn repeating(planner: Planner, plan: Plan) -> Repeating {
    if planner.repeats() {
        Repeating::Plan(plan)
    } else {
        Repeating::FoundAPath(plan.path.is_some())
    }
}

// Plans of PRM and PRM* with 2 and with 8 goals beyond the 2-D and the 7-D wall repeat
// under one seed: PRM*'s plan, and whether PRM finds a path. Each takes every goal into
// its roadmap before its search thread starts, and its first path reaches a goal.
#[test]
fn a_roadmap_plan_with_several_goals_repeats() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        for count in GOAL_COUNTS {
            let goals = goals_beyond_the_wall(dimension, count);
            for planner in [Planner::Prm, Planner::PrmStar] {
                let what = format!("{planner} with {count} goals in {dimension} dimensions");
                let request = Request {
                    goals: &goals,
                    ..ends.request(planner, 1)
                };
                let first = plan(&request, &world).expect("a plan");
                let reached = first.path.as_ref().and_then(|path| path.last());
                assert!(
                    reached.is_some_and(|state| goals.contains(state)),
                    "{what} reaches no goal"
                );
                let first = repeating(planner, first);
                for _ in 1..REPEATS {
                    let again = repeating(planner, plan(&request, &world).expect("a plan"));
                    assert_eq!(again, first, "{what}");
                }
            }
        }
    }
}

/// The wall on a slow host: the first check of a state other than the start and the
/// goal, a state of the roadmap, sleeps for `SLOW_CHECK`.
struct SlowHost {
    world: World,
    ends: [Vec<f64>; 2],
    slept: AtomicBool,
}

impl Checks for SlowHost {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        if !self.ends.iter().any(|end| end == state) && !self.slept.swap(true, Ordering::SeqCst) {
            thread::sleep(SLOW_CHECK);
        }
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.world.walk(from, to))
    }
}

// On a host so slow that the first state of a roadmap takes longer than OMPL's first round
// of roadmap growth, a plan of PRM or PRM* through the 2-D and the 7-D wall gives what it
// gives at this host's speed: PRM*'s plan, and whether PRM finds a path. Its rounds of
// growth end by questions whether to stop. The search thread asks whether to stop before
// each of its searches, which it runs 1 ms apart, while the planning thread sleeps, and
// those questions add nothing to the iteration bound's count.
#[test]
fn a_roadmap_plan_repeats_on_a_slow_host() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        for planner in [Planner::Prm, Planner::PrmStar] {
            let request = ends.request(planner, 1);
            let slow_host = SlowHost {
                world: wall(dimension),
                ends: [ends.start.clone(), ends.goal().to_vec()],
                slept: AtomicBool::new(false),
            };
            let slow = plan(&request, &slow_host).expect("a plan");
            assert!(slow_host.slept.load(Ordering::SeqCst), "no check slept");
            let at_speed = plan(&request, &world).expect("a plan");
            assert_eq!(
                repeating(planner, slow),
                repeating(planner, at_speed),
                "{planner} in {dimension} dimensions"
            );
        }
    }
}

/// The wall, whose checks err on any thread but `planning`.
struct OneThread {
    world: World,
    planning: thread::ThreadId,
}

impl OneThread {
    fn on_the_planning_thread(&self, what: &str) -> Result<(), CheckError> {
        if thread::current().id() == self.planning {
            return Ok(());
        }
        Err(format!("a {what} was checked off the planning thread").into())
    }
}

impl Checks for OneThread {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.on_the_planning_thread("state")?;
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.on_the_planning_thread("motion")?;
        Ok(self.world.walk(from, to))
    }
}

// A plan of PRM or PRM* with several goals checks every state and every motion on the
// thread that calls plan: the thread that searches the roadmap checks none.
#[test]
fn a_roadmap_plan_checks_on_the_planning_thread_alone() {
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, GOAL_COUNTS[1]);
    let checks = OneThread {
        world: wall(2),
        planning: thread::current().id(),
    };
    for planner in [Planner::Prm, Planner::PrmStar] {
        let request = Request {
            goals: &goals,
            ..ends.request(planner, 1)
        };
        let answer = plan(&request, &checks).unwrap_or_else(|error| panic!("{planner}: {error}"));
        assert!(answer.path.is_some(), "{planner} found no path");
    }
}

/// A world divided at the middle of its first dimension. Every state is valid, and a
/// motion that crosses the middle is valid only from or to the `FINAL_ROADMAP_STATE`th
/// state of the roadmap, the joining state: a state other than the start and the goal
/// that the planning thread checks. The world counts those states.
struct JoinedLast {
    ends: [Vec<f64>; 2],
    roadmap_states: AtomicUsize,
    joining: OnceLock<Vec<f64>>,
}

impl Checks for JoinedLast {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        if !self.ends.iter().any(|end| end == state) {
            let checked = self.roadmap_states.fetch_add(1, Ordering::SeqCst) + 1;
            if checked == FINAL_ROADMAP_STATE {
                self.joining
                    .set(state.to_vec())
                    .expect("one state of the roadmap joins");
            }
        }
        Ok(true)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        let joins = self
            .joining
            .get()
            .is_some_and(|joining| joining == from || joining == to);
        Ok(if joins || (from[0] < 0.5) == (to[0] < 0.5) {
            Motion::Valid
        } else {
            Motion::Invalid { last_valid: 0.0 }
        })
    }
}

// Once the iteration bound has ended the planner of PRM or PRM*, the plan searches the
// finished roadmap. The planning thread asks whether to stop once before its first round of
// growth and twice for each state of the roadmap, before it samples the state: under
// FINAL_SEARCH_BOUND, the FINAL_ROADMAP_STATE-th state is the last, and the only one whose
// motions cross the middle, so the path runs through it.
#[test]
fn a_roadmap_plan_searches_the_finished_roadmap() {
    let ends = WallEnds::new(2);
    for planner in [Planner::Prm, Planner::PrmStar] {
        let checks = JoinedLast {
            ends: [ends.start.clone(), ends.goal().to_vec()],
            roadmap_states: AtomicUsize::new(0),
            joining: OnceLock::new(),
        };
        let request = Request {
            iterations: NonZeroU64::new(FINAL_SEARCH_BOUND).expect("nonzero"),
            ..ends.request(planner, 1)
        };
        let answer = plan(&request, &checks).expect("a plan");
        assert_eq!(answer.ended_by, Ending::IterationBound, "{planner}");
        assert_eq!(
            checks.roadmap_states.load(Ordering::SeqCst),
            FINAL_ROADMAP_STATE,
            "{planner}: the states of the roadmap"
        );
        let joining = checks.joining.get().expect("the joining state");
        let path = answer.path.expect("a path");
        assert!(
            path.contains(joining),
            "{planner}: the path {path:?} misses the joining state {joining:?}"
        );
    }
}

/// A world whose motion check answers a last valid fraction beyond the motion.
struct BeyondTheMotion;

impl Checks for BeyondTheMotion {
    fn state(&self, _: &[f64]) -> Result<bool, CheckError> {
        Ok(true)
    }

    fn motion(&self, _: &[f64], _: &[f64]) -> Result<Motion, CheckError> {
        Ok(Motion::Invalid { last_valid: 1.5 })
    }
}

// A motion check whose last valid state lies beyond its motion ends the plan with an
// error naming the fraction.
#[test]
fn a_last_valid_fraction_beyond_the_motion_ends_the_plan() {
    let ends = WallEnds::new(2);
    assert_eq!(
        plan(&ends.request(Planner::Rrt, 1), &BeyondTheMotion),
        Err(PlanError::Check(
            "a motion's last valid fraction lies in [0, 1], not 1.5".to_string()
        ))
    );
}

/// The plan of RRTConnect through the 2-D wall under `checks`, within `time_limit`
/// and with no bound on its iterations.
fn plan_within(time_limit: Duration, checks: &dyn Checks) -> Result<Plan, PlanError> {
    let ends = WallEnds::new(2);
    let request = Request {
        time_limit,
        iterations: NonZeroU64::MAX,
        ..ends.request(Planner::RrtConnect, 1)
    };
    plan(&request, checks)
}

/// The wall, whose first state check plans a path of its own and keeps the answer.
struct Replanning {
    world: World,
    inner: Mutex<Option<Result<Plan, PlanError>>>,
}

impl Checks for Replanning {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        let first = self.inner.lock().expect("unpoisoned").is_none();
        if first {
            let answer = plan_within(SHORT_TIME_LIMIT, &self.world);
            *self.inner.lock().expect("unpoisoned") = Some(answer);
        }
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.world.walk(from, to))
    }
}

// A check that calls plan gets a refusal, and the plan it checks goes on to a path.
#[test]
fn a_plan_called_from_a_check_is_refused() {
    let replanning = Replanning {
        world: wall(2),
        inner: Mutex::new(None),
    };
    let outer = plan_within(NO_TIME_LIMIT, &replanning).expect("a plan");
    assert!(outer.path.is_some(), "the checked plan found no path");
    assert_eq!(
        replanning.inner.into_inner().expect("unpoisoned"),
        Some(Err(PlanError::Request(REENTERED.to_string())))
    );
}

// A plan that its iteration bound ends before the planner finds a path says so and
// gives no path, and a plan whose planner returns says so.
#[test]
fn a_plan_says_what_ended_it() {
    let ends = WallEnds::new(2);
    let short = Request {
        iterations: NonZeroU64::new(1).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(
        plan(&short, &wall(2)),
        Ok(Plan {
            path: None,
            simplified: None,
            ended_by: Ending::IterationBound
        })
    );
    let solved = wall_plan(Planner::RrtConnect, 2, 1);
    assert_eq!(solved.ended_by, Ending::Returned);
}

// A simplifier under a bound that it does not reach says that it returned, and one under
// a bound of one question, which its first pass runs past to its end, says that the
// bound ended it.
#[test]
fn a_simplification_says_what_ended_it() {
    for (bound, ended_by) in [(u32::MAX, Ending::Returned), (1, Ending::IterationBound)] {
        let plan = wall_plan_simplified(Planner::Rrt, 2, 1, bound);
        assert_eq!(plan.ended_by, Ending::Returned, "the planner");
        let simplification = plan.simplified.expect("a simplification");
        assert_eq!(simplification.ended_by, ended_by, "a bound of {bound}");
    }
}

/// What a plan that its time limit ends before it finds a path answers.
const TIMED_OUT: Plan = Plan {
    path: None,
    simplified: None,
    ended_by: Ending::TimeLimit,
};

/// A world whose first state check, the check of the start, sleeps past the short time
/// limit.
struct Slow {
    world: World,
    slept: AtomicBool,
}

impl Slow {
    fn new(world: World) -> Self {
        Self {
            world,
            slept: AtomicBool::new(false),
        }
    }
}

impl Checks for Slow {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        if !self.slept.swap(true, Ordering::SeqCst) {
            thread::sleep(2 * SHORT_TIME_LIMIT);
        }
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.world.walk(from, to))
    }
}

// A plan whose time limit passes before its planner asks whether to stop gives no
// path and says that its time limit ended it.
#[test]
fn a_time_limit_that_passes_ends_the_plan() {
    assert_eq!(
        plan_within(SHORT_TIME_LIMIT, &Slow::new(wall(2))),
        Ok(TIMED_OUT)
    );
}

// PRM joins the start to the goal in a world with no obstacle as it takes them into its
// roadmap, before it asks whether to stop. A plan whose time limit passes before that
// question says that the time limit ended its planner, gives the path of the roadmap,
// and says that the time limit ended its simplification before it started, whose path
// is then the planned path. The test runs alone in its process, where no other plan
// keeps it waiting toward its time limit.
#[test]
fn a_time_limit_that_passes_before_the_simplification_ends_it() {
    if !alone() {
        run_alone("a_time_limit_that_passes_before_the_simplification_ends_it");
        return;
    }
    let ends = WallEnds::new(2);
    let checks = Slow::new(World {
        obstacles: Vec::new(),
        motion_step: MOTION_STEP,
    });
    let request = Request {
        time_limit: SHORT_TIME_LIMIT,
        simplification: NonZeroU32::new(SIMPLIFICATION),
        ..ends.request(Planner::Prm, 1)
    };
    let answer = plan(&request, &checks).expect("a plan");
    assert_eq!(answer.ended_by, Ending::TimeLimit, "the planner");
    let joined = [ends.start.clone(), ends.goal().to_vec()];
    assert_eq!(answer.path.as_deref(), Some(joined.as_slice()));
    let simplification = answer.simplified.expect("a simplification");
    assert_eq!(simplification.ended_by, Ending::TimeLimit, "the simplifier");
    assert_eq!(Some(simplification.path), answer.path);
}

/// The wall, whose first state check tells the test that its plan holds the process
/// and waits for the test to release it.
struct Holding {
    world: World,
    channels: Mutex<Option<(mpsc::Sender<()>, mpsc::Receiver<()>)>>,
}

impl Checks for Holding {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        let channels = self.channels.lock().expect("unpoisoned").take();
        if let Some((holding, release)) = channels {
            holding.send(()).expect("the test listens");
            release.recv().expect("the test releases the plan");
        }
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(self.world.walk(from, to))
    }
}

// A plan that waits for another plan of the process past its time limit gives no path
// and says that its time limit ended it; the other plan then goes on to a path.
#[test]
fn a_plan_waits_for_another_plan_until_its_time_limit() {
    let (holding, held) = mpsc::channel();
    let (release, released) = mpsc::channel();
    let first_checks = Holding {
        world: wall(2),
        channels: Mutex::new(Some((holding, released))),
    };
    let world = wall(2);
    thread::scope(|scope| {
        let first = scope.spawn(|| plan_within(NO_TIME_LIMIT, &first_checks));
        held.recv().expect("the first plan holds the process");
        let (answer, answered) = mpsc::channel();
        let world = &world;
        scope.spawn(move || answer.send(plan_within(SHORT_TIME_LIMIT, world)));
        let waited = answered.recv_timeout(ANSWER_GUARD);
        release.send(()).expect("the first plan waits");
        assert_eq!(
            waited.expect("the second plan answers at its time limit"),
            Ok(TIMED_OUT)
        );
        let first = first.join().expect("the first plan returns");
        assert!(
            first.expect("a plan").path.is_some(),
            "the first plan found no path"
        );
    });
}

// A start or every goal that fails the state check or lies off the constraint is
// refused before the planner starts; a plan with one goal that fails and one that
// passes reaches the one that passes.
#[test]
fn a_start_or_goals_that_fail_the_checks_are_refused() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let inside = vec![0.5, 0.5];
    let inside_start = Request {
        start: &inside,
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(plan(&inside_start, &world), Err(PlanError::InvalidStart));
    let inside_goals = [inside.clone(), vec![0.5, 0.4]];
    let inside_goals = Request {
        goals: &inside_goals,
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(plan(&inside_goals, &world), Err(PlanError::InvalidGoal));
    let one_passes = [inside, ends.goal().to_vec()];
    let one_passes = Request {
        goals: &one_passes,
        ..ends.request(Planner::RrtConnect, 1)
    };
    let path = plan(&one_passes, &world)
        .expect("a plan")
        .path
        .expect("a path");
    assert_eq!(path.last().map(Vec::as_slice), Some(ends.goal()));

    let (space, start, goal) = sphere_ends();
    let off = vec![0.0, 0.0, 0.5];
    let goals = [goal];
    let off_start = on_the_sphere(Planner::RrtConnect, &space, &off, &goals, &UnitSphere);
    assert_eq!(plan(&off_start, &Equator), Err(PlanError::InvalidStart));
    let off_goals = [off];
    let off_goal = on_the_sphere(Planner::RrtConnect, &space, &start, &off_goals, &UnitSphere);
    assert_eq!(plan(&off_goal, &Equator), Err(PlanError::InvalidGoal));
}

/// A constraint of a number of components, each zero.
struct Components(usize);

impl Constraint for Components {
    fn codimension(&self) -> usize {
        self.0
    }

    fn function(&self, _: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        out.fill(0.0);
        Ok(())
    }

    fn jacobian(&self, _: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        out.fill(0.0);
        Ok(())
    }
}

/// Asserts that `plan` refuses `request`, saying `says`.
fn assert_refused(request: &Request, says: &str) {
    assert_eq!(
        plan(request, &wall(2)),
        Err(PlanError::Request(says.to_string()))
    );
}

/// What `plan` says of a constraint of `components` in `dimensions`.
fn too_many_components(components: &str, dimensions: &str) -> String {
    format!(
        "a constraint of {components} in {dimensions}; a constraint has at least 1 component, and fewer components than the space has dimensions"
    )
}

// A request whose states do not fit its space, with nothing to reach, with no time,
// or with a constraint that the space cannot hold or a tolerance that is not positive,
// is refused before OMPL sees it; names parse to their planners and a wrong one lists
// them; bounds that hold no space are refused.
#[test]
fn a_request_that_does_not_fit_its_space_is_refused() {
    let ends = WallEnds::new(2);
    let short_goal = [vec![0.9]];
    let beyond = [1.5, 0.1];
    let unknown = [f64::NAN, 0.1];
    let goal_beyond = [ends.goal().to_vec(), vec![0.9, -0.5]];
    assert_refused(
        &Request {
            start: &ends.start[..1],
            ..ends.request(Planner::Rrt, 1)
        },
        "the start has 1 value; the space has 2 dimensions",
    );
    assert_refused(
        &Request {
            goals: &[],
            ..ends.request(Planner::Rrt, 1)
        },
        "a plan has at least one goal",
    );
    assert_refused(
        &Request {
            goals: &short_goal,
            ..ends.request(Planner::Rrt, 1)
        },
        "a goal has 1 value; the space has 2 dimensions",
    );
    assert_refused(
        &Request {
            start: &beyond,
            ..ends.request(Planner::Rrt, 1)
        },
        "the start holds 1.5 in dimension 0, which runs from 0 to 1; a state lies within the space's bounds",
    );
    assert_refused(
        &Request {
            start: &unknown,
            ..ends.request(Planner::Rrt, 1)
        },
        "the start holds NaN in dimension 0, which runs from 0 to 1; a state lies within the space's bounds",
    );
    assert_refused(
        &Request {
            goals: &goal_beyond,
            ..ends.request(Planner::Rrt, 1)
        },
        "goal 1 holds -0.5 in dimension 1, which runs from 0 to 1; a state lies within the space's bounds",
    );
    assert_refused(
        &Request {
            time_limit: Duration::ZERO,
            ..ends.request(Planner::Rrt, 1)
        },
        "the time limit is 0; a plan needs time to plan",
    );
    for (dimension, components, tolerance, says) in [
        (
            2,
            0,
            TOLERANCE,
            too_many_components("0 components", "2 dimensions"),
        ),
        (
            2,
            2,
            TOLERANCE,
            too_many_components("2 components", "2 dimensions"),
        ),
        (
            1,
            1,
            TOLERANCE,
            too_many_components("1 component", "1 dimension"),
        ),
        (
            2,
            1,
            0.0,
            "the projection tolerance is 0; it is a positive number".to_string(),
        ),
        (
            2,
            1,
            f64::NAN,
            "the projection tolerance is NaN; it is a positive number".to_string(),
        ),
        (
            2,
            1,
            f64::INFINITY,
            "the projection tolerance is inf; it is a positive number".to_string(),
        ),
    ] {
        let space = unit_cube(dimension);
        let middle = vec![0.5; dimension];
        let goals = [middle.clone()];
        assert_refused(
            &Request {
                projection: Some(Projection {
                    constraint: &Components(components),
                    tolerance,
                    iterations: NonZeroU32::new(50).expect("nonzero"),
                }),
                ..request(Planner::Rrt, &space, &middle, &goals, 1)
            },
            &says,
        );
    }

    for planner in Planner::ALL {
        assert_eq!(planner.name().parse(), Ok(planner));
    }
    assert_eq!(
        "RRT*".parse::<Planner>().unwrap_err().to_string(),
        "no planner is named \"RRT*\"; the planners are RRTConnect, RRT, RRTstar, BiTRRT, \
         KPIECE1, BKPIECE1, LBKPIECE1, EST, SBL, PRM, PRMstar"
    );
    let refusal = |bounds: &[(f64, f64)]| Space::new(bounds).err().map(|e| e.to_string());
    assert_eq!(
        refusal(&[]).as_deref(),
        Some("a space has at least one dimension")
    );
    assert_eq!(
        refusal(&[(1.0, 1.0)]).as_deref(),
        Some(
            "dimension 0 runs from 1 to 1; a dimension runs from a finite low to a finite high above it"
        )
    );
    assert_eq!(
        refusal(&[(0.0, f64::INFINITY)]).as_deref(),
        Some(
            "dimension 0 runs from 0 to inf; a dimension runs from a finite low to a finite high above it"
        )
    );
}
