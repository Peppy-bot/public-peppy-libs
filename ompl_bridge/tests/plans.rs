//! Plans in worlds of box obstacles, with every planner: their paths pass the
//! checks, repeat under one seed in this process and in another, with several goals and
//! on a slow host, simplify to paths at most as long, and walk a constraint; a request
//! that does not fit its space, or whose start or goals fail the checks, is refused; a
//! failure in a check or the constraint ends a plan without ending the process; a plan
//! says what ended its planner and its simplifier; the time limit bounds a plan from its
//! call, and the simplification's time the simplification after it, whose first pass runs
//! past it; plans run at once;
//! the caller's stop ends a plan; several attempts, four at once, join into a path at
//! most as long as one attempt's; and goals sampled while a plan runs serve each planner.

use std::fmt;
use std::num::{NonZeroU32, NonZeroU64};
use std::ops::RangeInclusive;
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Condvar, Mutex, OnceLock, mpsc};
use std::thread;
use std::time::Duration;

mod world;

use ompl_bridge::{
    CONSTRAINED_SPACING, CheckError, Checks, Constraint, Ending, GoalSample, GoalSampling, Goals,
    Motion, PLANNING_THREADS, Plan, PlanError, Planner, Projection, Request, SampledGoals, Space,
    plan,
};
use ros_reference::planning::Obstacle;
use world::{World, distance, length};

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
/// How long a test here waits for a plan, or for attempts to meet, before it fails, so a
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
/// A time limit within which a plan here poses its problem and asks its planner's first
/// question whether to stop, on any host: OMPL's setup of a 2-dimensional wall takes
/// well under a millisecond.
const SETUP_TIME_LIMIT: Duration = Duration::from_secs(1);
/// How long the slow first motion check takes: past `SETUP_TIME_LIMIT`.
const SLOW_MOTION: Duration = Duration::from_secs(2);
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
/// The attempts that plan at once, as a count of threads.
const AT_ONCE: usize = PLANNING_THREADS.get() as usize;
/// The seeds the test of a simplifier that moves a path's end plans under.
const SIMPLIFIER_SEEDS: u32 = 40;
/// The attempts of the plans here that make several: one batch.
const ATTEMPTS: u32 = PLANNING_THREADS.get();
/// What `plan` says when a check calls it.
const REENTERED: &str = "plan was called from a check, the constraint or the goal sampling of a plan; a plan seeds the generator of its thread, which the other plan draws from, so call plan outside the checks of another";

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
        goals: Goals::States(goals),
        time_limit: NO_TIME_LIMIT,
        iterations: NonZeroU64::new(ITERATIONS).expect("nonzero"),
        seed: NonZeroU32::new(seed).expect("nonzero"),
        attempts: NonZeroU32::MIN,
        simplification: None,
        simplification_time: NO_TIME_LIMIT,
        projection: None,
        stop: None,
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

/// The plan of `attempts` attempts of `planner` through the wall of `dimension`
/// dimensions under `seed`.
fn wall_attempts(planner: Planner, dimension: usize, seed: u32, attempts: u32) -> Plan {
    let ends = WallEnds::new(dimension);
    let request = Request {
        attempts: NonZeroU32::new(attempts).expect("nonzero"),
        ..ends.request(planner, seed)
    };
    plan(&request, &wall(dimension)).expect("a plan")
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

/// The bits of the values of `plan`'s path, state after state.
fn bits(plan: Plan) -> String {
    let path = plan.path.expect("a path");
    let bits: Vec<String> = path
        .iter()
        .flatten()
        .map(|v| format!("{:016x}", v.to_bits()))
        .collect();
    bits.join(" ")
}

/// The path of a 7-D plan of every repeatable planner with seed 1, one planner per line,
/// and of `ATTEMPTS` attempts of RRTConnect, as the bits of its values.
fn path_bits() -> String {
    let attempts = format!(
        "{ATTEMPTS} attempts of RRTConnect {}",
        bits(wall_attempts(Planner::RrtConnect, 7, 1, ATTEMPTS))
    );
    Planner::ALL
        .into_iter()
        .filter(|p| p.repeats())
        .map(|planner| format!("{planner} {}", bits(wall_plan(planner, 7, 1))))
        .chain([attempts])
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

// A plan with seed 1 in another process gives the paths that it gives in this one, of one
// attempt and of several.
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

/// The wall's checks, the simplifier's check refusing every motion.
struct SimplifierRefuses<'a>(&'a World);

impl Checks for SimplifierRefuses<'_> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.0.state(state)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.0.motion(from, to)
    }

    fn simplifier_motion(&self, _: &[f64], _: &[f64]) -> Result<Motion, CheckError> {
        Ok(Motion::Invalid { last_valid: 0.0 })
    }
}

// The simplifier judges each motion it checks by the simplifier's check: RRT's path through
// the 2-D wall under seed 1, which the simplifier shortens under the planner's check, stays
// the planned path when the simplifier's check refuses every motion.
#[test]
fn the_simplifier_judges_its_motions_by_the_simplifiers_check() {
    let ends = WallEnds::new(2);
    let world = wall(2);
    let request = Request {
        simplification: NonZeroU32::new(SIMPLIFICATION),
        ..ends.request(Planner::Rrt, 1)
    };
    let refused = plan(&request, &SimplifierRefuses(&world)).expect("a plan");
    let path = refused.path.expect("a path");
    assert_eq!(refused.simplified.expect("a simplified path").path, path);
    let shortened = plan(&request, &world)
        .expect("a plan")
        .simplified
        .expect("a simplified path")
        .path;
    assert!(length(&shortened) < length(&path));
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

    fn motion(&self, _: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(if self.state(to)? {
            Motion::Valid
        } else {
            Motion::Invalid { last_valid: 0.0 }
        })
    }
}

/// The band of `Equator` that only a motion check sees: every state passes, and a motion
/// that ends in the band fails.
struct EquatorOfMotions;

impl Checks for EquatorOfMotions {
    fn state(&self, _: &[f64]) -> Result<bool, CheckError> {
        Ok(true)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Equator.motion(from, to)
    }
}

/// Every state, as a constraint: its function is 0 everywhere.
struct Anywhere;

impl Constraint for Anywhere {
    fn codimension(&self) -> usize {
        1
    }

    fn function(&self, _: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        out[0] = 0.0;
        Ok(())
    }

    fn jacobian(&self, _: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        out.fill(0.0);
        Ok(())
    }
}

/// The longest motion asked of the checks, by the Euclidean distance and by the sum of each
/// dimension's absolute change.
#[derive(Default, Clone, Copy)]
struct Longest {
    euclidean: f64,
    summed: f64,
}

/// Checks that pass every state and motion and record the longest motion asked of them.
#[derive(Default)]
struct LongestMotion(Mutex<Longest>);

impl LongestMotion {
    fn longest(&self) -> Longest {
        *self.0.lock().expect("no check panics")
    }
}

impl Checks for LongestMotion {
    fn state(&self, _: &[f64]) -> Result<bool, CheckError> {
        Ok(true)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        let summed = from.iter().zip(to).map(|(a, b)| (a - b).abs()).sum();
        let mut longest = self.0.lock().expect("no check panics");
        *longest = Longest {
            euclidean: longest.euclidean.max(distance(from, to)),
            summed: longest.summed.max(summed),
        };
        Ok(Motion::Valid)
    }
}

// A planner's range is a fifth of the space's extent, by the space's distance: in the unit
// square, whose extent is the sum of its sides, 2, RRT steps at most 0.4 in the sum of the
// dimensions' changes, and a step off the diagonal runs past a fifth of the square's
// diagonal, 0.2 * sqrt(2), the range of a space measured by the Euclidean distance.
#[test]
fn a_planners_range_is_a_fifth_of_the_extent_by_the_spaces_distance() {
    let space = Space::new(&[(0.0, 1.0); 2]).expect("the unit square");
    let checks = LongestMotion::default();
    let goals = [vec![1.0, 1.0]];
    plan(
        &request(Planner::Rrt, &space, &[0.0, 0.0], &goals, 1),
        &checks,
    )
    .expect("a plan");
    let longest = checks.longest();
    assert!(longest.summed <= 0.4 + 1e-12, "{}", longest.summed);
    assert!(
        longest.euclidean > 0.2 * 2.0_f64.sqrt(),
        "{}",
        longest.euclidean
    );
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
            spacing: CONSTRAINED_SPACING,
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

// A constrained plan judges each stretch of a motion's walk by the motion check: a band that
// only the motion check sees stands in the way of every planner's path.
#[test]
fn a_constrained_plan_judges_the_stretches_of_its_walks_by_the_motion_check() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal.clone()];
    for planner in [Planner::RrtConnect, Planner::Rrt, Planner::Est] {
        let answer = plan(
            &on_the_sphere(planner, &space, &start, &goals, &UnitSphere),
            &EquatorOfMotions,
        )
        .expect("a plan");
        let path = answer
            .path
            .unwrap_or_else(|| panic!("{planner} found no path on the sphere"));
        assert_walks_the_sphere(&path, &start, &goal, &format!("{planner}'s path"));
    }
}

// The steps of a walk that the projection leaves in one line join one motion, so a check of
// a motion walks it whole: with a constraint every state meets, the motion check sees a
// motion longer than the spacing, and the plan's path keeps to the spacing all the same.
/// The checks of the sphere's equator, the simplifier's check refusing every motion and
/// counting the motions it is asked.
struct EquatorSimplifierRefuses(AtomicUsize);

impl Checks for EquatorSimplifierRefuses {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        EquatorOfMotions.state(state)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        EquatorOfMotions.motion(from, to)
    }

    fn simplifier_motion(&self, _: &[f64], _: &[f64]) -> Result<Motion, CheckError> {
        self.0.fetch_add(1, Ordering::SeqCst);
        Ok(Motion::Invalid { last_valid: 0.0 })
    }
}

// A constrained plan's simplifier judges each stretch of its walks by the simplifier's
// check: with that check refusing every motion, RRT's path on the sphere, which the
// simplifier has motions to try on, stays the planned path, and the simplifier asked the
// check.
#[test]
fn a_constrained_simplifier_judges_its_stretches_by_the_simplifiers_check() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal];
    let checks = EquatorSimplifierRefuses(AtomicUsize::new(0));
    let request = Request {
        simplification: NonZeroU32::new(SIMPLIFICATION),
        ..on_the_sphere(Planner::Rrt, &space, &start, &goals, &UnitSphere)
    };
    let answer = plan(&request, &checks).expect("a plan");
    let path = answer.path.expect("a path on the sphere");
    assert_eq!(answer.simplified.expect("a simplified path").path, path);
    assert!(
        checks.0.load(Ordering::SeqCst) > 0,
        "the simplifier asked the simplifier's check of no motion"
    );
}

#[test]
fn the_steps_of_a_walk_in_one_line_join_one_motion() {
    let space = unit_cube(2);
    let (start, goal) = (vec![0.05, 0.05], vec![0.95, 0.95]);
    let goals = [goal.clone()];
    let checks = LongestMotion::default();
    let answer = plan(
        &on_the_sphere(Planner::RrtConnect, &space, &start, &goals, &Anywhere),
        &checks,
    )
    .expect("a plan");
    let path = answer.path.expect("a path through an empty space");
    let longest = checks.longest().euclidean;
    assert!(
        longest > CONSTRAINED_SPACING,
        "the longest motion asked is {longest}"
    );
    assert!(
        path.windows(2)
            .all(|w| distance(&w[0], &w[1]) <= CONSTRAINED_SPACING)
    );
}

// A plan walks at its projection's spacing: consecutive states of its path and of its
// simplification lie at most that far apart.
#[test]
fn a_constrained_plan_keeps_its_states_at_its_spacing() {
    let (space, start, goal) = sphere_ends();
    let goals = [goal];
    let spacing = CONSTRAINED_SPACING / 4.0;
    let on_the_sphere = on_the_sphere(Planner::RrtConnect, &space, &start, &goals, &UnitSphere);
    let answer = plan(
        &Request {
            simplification: NonZeroU32::new(SIMPLIFICATION),
            projection: on_the_sphere.projection.map(|projection| Projection {
                spacing,
                ..projection
            }),
            ..on_the_sphere
        },
        &Equator,
    )
    .expect("a plan");
    let path = answer.path.expect("a path on the sphere");
    let simplified = answer.simplified.expect("a simplified path").path;
    for states in [&path, &simplified] {
        assert!(states.windows(2).all(|w| distance(&w[0], &w[1]) <= spacing));
    }
}

// A walk that leaves the space's bounds is invalid, though its ends lie inside them and every
// state passes the caller's checks. The space holds the sphere's lower cap outside, and the
// shortest way between the two ends crosses it, so the plan goes around.
#[test]
fn a_constrained_plan_keeps_its_states_inside_the_bounds() {
    const LOWEST_Z: f64 = -0.9;
    let space = Space::new(&[(-1.5, 1.5), (-1.5, 1.5), (LOWEST_Z, 1.5)]).expect("a space");
    let at_the_side = |x: f64| [x, 0.0, -(1.0 - x * x).sqrt()];
    let (start, goal) = (at_the_side(-0.5), at_the_side(0.5));
    assert!(
        start[2] > LOWEST_Z
            && (0..=20).any(|k| at_the_side(0.05 * f64::from(k) - 0.5)[2] < LOWEST_Z),
        "the shortest way between the ends stays inside the bounds: nothing to go around"
    );
    let goals = [goal.to_vec()];
    let answer = plan(
        &Request {
            simplification: NonZeroU32::new(SIMPLIFICATION),
            ..on_the_sphere(Planner::RrtConnect, &space, &start, &goals, &UnitSphere)
        },
        &LongestMotion::default(),
    )
    .expect("a plan");
    let path = answer.path.expect("a path around the cap");
    let simplified = answer.simplified.expect("a simplified path").path;
    for states in [&path, &simplified] {
        assert!(
            states.iter().all(|state| state[2] >= LOWEST_Z),
            "a state of the path stands below {LOWEST_Z}"
        );
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
                    goals: Goals::States(&goals),
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
            goals: Goals::States(&goals),
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
// gives no path, and a plan whose planner returns says so, of one attempt and of
// several.
#[test]
fn a_plan_says_what_ended_it() {
    let ends = WallEnds::new(2);
    for attempts in [1, ATTEMPTS] {
        let short = Request {
            iterations: NonZeroU64::new(1).expect("nonzero"),
            attempts: NonZeroU32::new(attempts).expect("nonzero"),
            ..ends.request(Planner::RrtConnect, 1)
        };
        assert_eq!(
            plan(&short, &wall(2)),
            Ok(Plan {
                path: None,
                simplified: None,
                ended_by: Ending::IterationBound
            }),
            "{attempts} attempts"
        );
        let solved = wall_attempts(Planner::RrtConnect, 2, 1, attempts);
        assert_eq!(solved.ended_by, Ending::Returned, "{attempts} attempts");
    }
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

/// Checks whose first state check, the check of the start, sleeps past the short time
/// limit.
struct Slow<C> {
    checks: C,
    slept: AtomicBool,
}

impl<C: Checks> Slow<C> {
    fn new(checks: C) -> Self {
        Self {
            checks,
            slept: AtomicBool::new(false),
        }
    }
}

impl<C: Checks> Checks for Slow<C> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        if !self.slept.swap(true, Ordering::SeqCst) {
            thread::sleep(2 * SHORT_TIME_LIMIT);
        }
        self.checks.state(state)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.checks.motion(from, to)
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

/// Checks whose first motion check sleeps `SLOW_MOTION`, past the setup's time limit.
struct SlowMotion<C> {
    checks: C,
    slept: AtomicBool,
}

impl<C: Checks> Checks for SlowMotion<C> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.checks.state(state)
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        if !self.slept.swap(true, Ordering::SeqCst) {
            thread::sleep(SLOW_MOTION);
        }
        self.checks.motion(from, to)
    }
}

// RRTConnect joins the start to the goal in its first iteration in a world with no
// obstacle, through the states each tree grew, and returns before it asks whether to
// stop again. A plan whose time limit passes during that iteration, with no time of its
// own for the simplification, still simplifies the path, as MoveIt does in the time its
// plan left: the simplifier's first pass joins the start straight to the goal, and the
// simplifier says that the time limit ended it.
#[test]
fn the_simplifier_makes_its_first_pass_past_the_time_limit() {
    let ends = WallEnds::new(2);
    let checks = SlowMotion {
        checks: World {
            obstacles: Vec::new(),
            motion_step: MOTION_STEP,
        },
        slept: AtomicBool::new(false),
    };
    let request = Request {
        time_limit: SETUP_TIME_LIMIT,
        simplification: NonZeroU32::new(SIMPLIFICATION),
        simplification_time: Duration::ZERO,
        ..ends.request(Planner::RrtConnect, 1)
    };
    let answer = plan(&request, &checks).expect("a plan");
    assert_eq!(answer.ended_by, Ending::Returned, "the planner");
    let planned = answer.path.expect("a path");
    assert!(
        planned.len() > 2,
        "the planned path holds {} states",
        planned.len()
    );
    let simplification = answer.simplified.expect("a simplification");
    assert_eq!(simplification.ended_by, Ending::TimeLimit, "the simplifier");
    assert_eq!(
        simplification.path,
        [ends.start.clone(), ends.goal().to_vec()]
    );
}

/// The wall, whose first state check tells the test that its plan has started and waits
/// for the test to release it.
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

/// The simplified plan of RRTConnect through the 2-D wall under `seed`, with no time
/// limit and no iteration bound, under `checks`.
fn simplified_wall_plan(seed: u32, checks: &dyn Checks) -> Result<Plan, PlanError> {
    let ends = WallEnds::new(2);
    let request = Request {
        iterations: NonZeroU64::MAX,
        simplification: NonZeroU32::new(SIMPLIFICATION),
        ..ends.request(Planner::RrtConnect, seed)
    };
    plan(&request, checks)
}

// Plans run at once, each drawing from the seeds of its own thread. The first plan waits
// in its first state check, before it builds its planner and its simplifier and their
// generators; a plan under another seed runs to its end meanwhile; then the first goes
// on. Each answers the simplified plan it gives alone.
#[test]
fn a_plan_runs_while_another_waits_in_its_check() {
    let (holding, held) = mpsc::channel();
    let (release, released) = mpsc::channel();
    let first_checks = Holding {
        world: wall(2),
        channels: Mutex::new(Some((holding, released))),
    };
    let world = wall(2);
    let (first_alone, second_alone) = (
        simplified_wall_plan(1, &world),
        simplified_wall_plan(2, &world),
    );
    thread::scope(|scope| {
        let first = scope.spawn(|| simplified_wall_plan(1, &first_checks));
        held.recv().expect("the first plan has started");
        let (answer, answered) = mpsc::channel();
        let world = &world;
        scope.spawn(move || answer.send(simplified_wall_plan(2, world)));
        let second = answered.recv_timeout(ANSWER_GUARD);
        release.send(()).expect("the first plan waits");
        assert_eq!(
            second.expect("the second plan answers while the first waits"),
            second_alone
        );
        assert_eq!(first.join().expect("the first plan returns"), first_alone);
    });
}

// A start or every goal that fails the state check or lies off the constraint is
// refused before the planner starts, in a plan of one attempt and of several; a plan
// with one goal that fails and one that passes reaches the one that passes.
#[test]
fn a_start_or_goals_that_fail_the_checks_are_refused() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let inside = vec![0.5, 0.5];
    let inside_goals = [inside.clone(), vec![0.5, 0.4]];
    for attempts in [1, ATTEMPTS] {
        let attempts = NonZeroU32::new(attempts).expect("nonzero");
        let inside_start = Request {
            start: &inside,
            attempts,
            ..ends.request(Planner::RrtConnect, 1)
        };
        assert_eq!(plan(&inside_start, &world), Err(PlanError::InvalidStart));
        let inside_goals = Request {
            goals: Goals::States(&inside_goals),
            attempts,
            ..ends.request(Planner::RrtConnect, 1)
        };
        assert_eq!(plan(&inside_goals, &world), Err(PlanError::InvalidGoal));
    }
    let one_passes = [inside, ends.goal().to_vec()];
    let one_passes = Request {
        goals: Goals::States(&one_passes),
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
            goals: Goals::States(&[]),
            ..ends.request(Planner::Rrt, 1)
        },
        "a plan has at least one goal",
    );
    assert_refused(
        &Request {
            goals: Goals::States(&short_goal),
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
            goals: Goals::States(&goal_beyond),
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
                    spacing: CONSTRAINED_SPACING,
                }),
                ..request(Planner::Rrt, &space, &middle, &goals, 1)
            },
            &says,
        );
    }
    for spacing in [0.0, f64::NAN, f64::INFINITY] {
        let space = unit_cube(2);
        let middle = vec![0.5; 2];
        let goals = [middle.clone()];
        assert_refused(
            &Request {
                projection: Some(Projection {
                    constraint: &Components(1),
                    tolerance: TOLERANCE,
                    iterations: NonZeroU32::new(50).expect("nonzero"),
                    spacing,
                }),
                ..request(Planner::Rrt, &space, &middle, &goals, 1)
            },
            &format!("the projection spacing is {spacing}; it is a positive number"),
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

// A plan whose time limit passes during its planner, with time of its own left for the
// simplification, simplifies its path: the simplification's time runs from the end of
// the time limit.
#[test]
fn a_simplification_has_its_own_time_after_the_time_limit() {
    let ends = WallEnds::new(2);
    let checks = Slow::new(World {
        obstacles: Vec::new(),
        motion_step: MOTION_STEP,
    });
    let request = Request {
        time_limit: SHORT_TIME_LIMIT,
        simplification: Some(NonZeroU32::MAX),
        ..ends.request(Planner::Prm, 1)
    };
    let answer = plan(&request, &checks).expect("a plan");
    assert_eq!(answer.ended_by, Ending::TimeLimit, "the planner");
    let simplification = answer.simplified.expect("a simplification");
    assert_eq!(simplification.ended_by, Ending::Returned, "the simplifier");
}

/// The wall, which counts the checks it answers off the thread that built it, on the
/// threads of a plan's attempts, and sets `stop`, when it holds one, at each of them.
struct OffThread<'a> {
    world: World,
    planning: thread::ThreadId,
    calls: AtomicUsize,
    stop: Option<&'a AtomicBool>,
}

impl<'a> OffThread<'a> {
    fn new(stop: Option<&'a AtomicBool>) -> Self {
        Self {
            world: wall(2),
            planning: thread::current().id(),
            calls: AtomicUsize::new(0),
            stop,
        }
    }

    fn count(&self) {
        if thread::current().id() == self.planning {
            return;
        }
        self.calls.fetch_add(1, Ordering::SeqCst);
        if let Some(stop) = self.stop {
            stop.store(true, Ordering::SeqCst);
        }
    }
}

impl Checks for OffThread<'_> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.count();
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.count();
        Ok(self.world.walk(from, to))
    }
}

/// What a plan that its stop ends before it finds a path answers.
const STOPPED: Plan = Plan {
    path: None,
    simplified: None,
    ended_by: Ending::Stopped,
};

// A plan whose stop flag is set before it starts ends at its planner's first question
// whether to stop, says so, and gives no path; a plan of several attempts starts none.
#[test]
fn a_stopped_plan_ends_without_a_path() {
    let ends = WallEnds::new(2);
    let stop = AtomicBool::new(true);
    for attempts in [1, ATTEMPTS] {
        let checks = OffThread::new(None);
        let stopped = Request {
            stop: Some(&stop),
            attempts: NonZeroU32::new(attempts).expect("nonzero"),
            ..ends.request(Planner::RrtStar, 1)
        };
        assert_eq!(plan(&stopped, &checks), Ok(STOPPED), "{attempts} attempts");
        assert_eq!(
            checks.calls.load(Ordering::SeqCst),
            0,
            "the checks of {attempts} attempts"
        );
    }
}

// The first check of each attempt sets the stop flag, which ends every attempt at its
// planner's first question whether to stop: the plan says so and gives no path.
#[test]
fn a_stop_ends_every_running_attempt() {
    let ends = WallEnds::new(2);
    let stop = AtomicBool::new(false);
    let checks = OffThread::new(Some(&stop));
    let stopped = Request {
        stop: Some(&stop),
        attempts: NonZeroU32::new(ATTEMPTS).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(plan(&stopped, &checks), Ok(STOPPED));
    assert!(
        checks.calls.load(Ordering::SeqCst) > 0,
        "no attempt started"
    );
}

// A plan of several attempts whose time limit passes in the check of its start, on the
// thread that calls plan, starts no attempt, and says that its time limit ended it.
#[test]
fn a_time_limit_that_passes_before_the_attempts_start_none() {
    let ends = WallEnds::new(2);
    let checks = Slow::new(OffThread::new(None));
    let request = Request {
        time_limit: SHORT_TIME_LIMIT,
        attempts: NonZeroU32::new(ATTEMPTS).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(plan(&request, &checks), Ok(TIMED_OUT));
    assert_eq!(
        checks.checks.calls.load(Ordering::SeqCst),
        0,
        "an attempt checked"
    );
}

/// The wall, whose motion checks set `stop` at the `stop_at`th of them, and count them.
struct StopAt<'a> {
    world: World,
    motions: AtomicUsize,
    stop_at: usize,
    stop: &'a AtomicBool,
}

impl Checks for StopAt<'_> {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        if self.motions.fetch_add(1, Ordering::SeqCst) + 1 == self.stop_at {
            self.stop.store(true, Ordering::SeqCst);
        }
        Ok(self.world.walk(from, to))
    }
}

// RRT* plans until its iteration bound, and has found a path by its last motion check. A
// stop at that check ends the planner at its next question with that path, and keeps
// the simplification from starting: the simplification says that the stop ended it,
// and its path is the planned path.
#[test]
fn a_plan_stopped_after_its_path_answers_it_unsimplified() {
    let ends = WallEnds::new(2);
    let unstopped = AtomicBool::new(false);
    let counting = StopAt {
        world: wall(2),
        motions: AtomicUsize::new(0),
        stop_at: usize::MAX,
        stop: &unstopped,
    };
    let full = plan(&ends.request(Planner::RrtStar, 1), &counting).expect("a plan");
    assert_eq!(
        full.ended_by,
        Ending::IterationBound,
        "the unstopped planner"
    );
    let stop = AtomicBool::new(false);
    let stopping = StopAt {
        world: wall(2),
        motions: AtomicUsize::new(0),
        stop_at: counting.motions.load(Ordering::SeqCst),
        stop: &stop,
    };
    let request = Request {
        simplification: NonZeroU32::new(SIMPLIFICATION),
        stop: Some(&stop),
        ..ends.request(Planner::RrtStar, 1)
    };
    let answer = plan(&request, &stopping).expect("a plan");
    assert_eq!(answer.ended_by, Ending::Stopped, "the planner");
    let path = answer.path.expect("a path");
    let simplification = answer.simplified.expect("a simplification");
    assert_eq!(simplification.ended_by, Ending::Stopped, "the simplifier");
    assert_eq!(simplification.path, path);
}

// A check that fails on the thread of an attempt ends a plan of several attempts with an
// error that says what failed, and the next plan in the process succeeds. The fifth
// state check comes after the checks of the start and the goal on the thread that calls
// plan, on an attempt's thread.
#[test]
fn a_failing_check_on_an_attempts_thread_ends_its_plan() {
    let ends = WallEnds::new(2);
    let failing = Failing::new(FailingCheck::State, 5, Failure::Panics);
    let request = Request {
        attempts: NonZeroU32::new(ATTEMPTS).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(
        plan(&request, &failing),
        Err(PlanError::Check(Failure::Panics.says().to_string()))
    );
    assert!(
        wall_attempts(Planner::RrtConnect, 2, 1, ATTEMPTS)
            .path
            .is_some(),
        "the plan after the failure found no path"
    );
}

// Several attempts plan at once, each from its own seed, and join their paths: the plan
// repeats path for path, passes the checks, and is at most as long as the plan of its
// attempt 0 alone, which plans from the same seed, for RRTConnect and for PRM*, whose
// search thread runs beside each attempt's.
#[test]
fn several_attempts_repeat_and_keep_the_shortest_path() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        for planner in [Planner::RrtConnect, Planner::PrmStar] {
            let what = format!("{ATTEMPTS} attempts of {planner} in {dimension} dimensions");
            let one = wall_plan(planner, dimension, 3)
                .path
                .expect("one attempt finds a path");
            let joined = wall_attempts(planner, dimension, 3, ATTEMPTS);
            assert_eq!(
                wall_attempts(planner, dimension, 3, ATTEMPTS),
                joined,
                "{what} repeat"
            );
            let path = joined.path.expect("several attempts find a path");
            assert_passes(&world, &path, &ends, &what);
            assert!(length(&path) <= length(&one), "{what} give a longer path");
        }
    }
}

// Four attempts of RRTConnect from seed 1 join into their hybrid: the joined path is
// shorter than the path of attempt 0, which plans from the same seed, and holds a state
// of that path between its ends, so it joins part of it with another attempt's.
#[test]
fn several_attempts_join_into_their_hybrid() {
    let first = wall_plan(Planner::RrtConnect, 2, 1)
        .path
        .expect("one attempt finds a path");
    let joined = wall_attempts(Planner::RrtConnect, 2, 1, ATTEMPTS)
        .path
        .expect("several attempts find a path");
    assert!(
        length(&joined) < length(&first),
        "the joined path is {} long, attempt 0's {}",
        length(&joined),
        length(&first)
    );
    let between = &first[1..first.len() - 1];
    assert!(
        joined.iter().any(|state| between.contains(state)),
        "the joined path holds no state of attempt 0's"
    );
}

/// The wall, a motion that descends in the second dimension before the wall's far face
/// refused: valid one way and refused the other. It records each motion it checks on the
/// thread that built it, the plan's own, with its verdict.
struct OneWay {
    world: World,
    thread: thread::ThreadId,
    checked: Mutex<Vec<Judged>>,
}

/// A motion a check judged, and whether it passed.
struct Judged {
    from: Vec<f64>,
    to: Vec<f64>,
    passed: bool,
}

impl OneWay {
    fn walk(&self, from: &[f64], to: &[f64]) -> Motion {
        if to[1] < from[1] && from[0].min(to[0]) < 0.55 {
            return Motion::Invalid { last_valid: 0.0 };
        }
        self.world.walk(from, to)
    }
}

impl Checks for OneWay {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        let motion = self.walk(from, to);
        if thread::current().id() == self.thread {
            self.checked.lock().expect("the record").push(Judged {
                from: from.to_vec(),
                to: to.to_vec(),
                passed: motion == Motion::Valid,
            });
        }
        Ok(motion)
    }
}

// With a check that passes a motion one way and refuses it the other, eight attempts of
// RRTConnect from seed 4 plan on threads of their own, and the plan's thread joins their
// paths: the hybrid holds a motion whose other way passed when it joined two paths, and
// its check in the path's order refuses it. The plan keeps the shortest path whose every
// motion passes in its order.
#[test]
fn a_hybrid_that_fails_the_checks_in_its_order_is_not_kept() {
    let world = OneWay {
        world: wall(2),
        thread: thread::current().id(),
        checked: Mutex::default(),
    };
    let ends = WallEnds::new(2);
    let eight = Request {
        attempts: NonZeroU32::new(8).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 4)
    };
    let path = plan(&eight, &world)
        .expect("a plan")
        .path
        .expect("eight attempts find a path");
    for w in path.windows(2) {
        assert_eq!(
            world.walk(&w[0], &w[1]),
            Motion::Valid,
            "the path moves from {:?} to {:?}",
            w[0],
            w[1]
        );
    }
    let checked = world.checked.lock().expect("the record");
    let backward = checked.iter().enumerate().any(|(k, refused)| {
        !refused.passed
            && checked[..k]
                .iter()
                .any(|other| other.passed && other.from == refused.to && other.to == refused.from)
    });
    assert!(backward, "the hybrid holds no motion checked the other way");
}

/// When each check on an attempt's thread came, in the order of all of them: its
/// thread's first and last, in the order the threads made their first.
#[derive(Default)]
struct Spans {
    checks: usize,
    threads: Vec<(thread::ThreadId, usize, usize)>,
}

/// The wall, whose checks on the threads of a plan's attempts record their `Spans`, and
/// hold a thread's first check until the threads of its batch, `AT_ONCE` in the
/// order of their first checks, have all made theirs.
struct Batches {
    world: World,
    planning: thread::ThreadId,
    spans: Mutex<Spans>,
    met: Condvar,
}

impl Batches {
    fn record(&self) {
        if thread::current().id() == self.planning {
            return;
        }
        let mut spans = self.spans.lock().expect("the spans");
        spans.checks += 1;
        let at = spans.checks;
        let id = thread::current().id();
        if let Some(span) = spans.threads.iter_mut().find(|span| span.0 == id) {
            span.2 = at;
            return;
        }
        spans.threads.push((id, at, at));
        let batch_end = spans.threads.len().div_ceil(AT_ONCE) * AT_ONCE;
        self.met.notify_all();
        let (_spans, waited) = self
            .met
            .wait_timeout_while(spans, ANSWER_GUARD, |spans| spans.threads.len() < batch_end)
            .expect("the spans");
        assert!(
            !waited.timed_out(),
            "{AT_ONCE} attempts never planned at once"
        );
    }
}

impl Checks for Batches {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        self.record();
        Ok(self.world.free(state))
    }

    fn motion(&self, from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        self.record();
        Ok(self.world.walk(from, to))
    }
}

// Eight attempts plan four at once: each of the first four checks while the other three
// do, and the second four check only once the first four have ended.
#[test]
fn four_attempts_plan_at_once_and_the_next_four_after_them() {
    assert_eq!(AT_ONCE, 4, "MoveIt's default max_planning_threads");
    let checks = Batches {
        world: wall(2),
        planning: thread::current().id(),
        spans: Mutex::default(),
        met: Condvar::new(),
    };
    let ends = WallEnds::new(2);
    let eight = Request {
        attempts: NonZeroU32::new(2 * ATTEMPTS).expect("nonzero"),
        ..ends.request(Planner::RrtConnect, 1)
    };
    let answer = plan(&eight, &checks).unwrap_or_else(|error| panic!("{error}"));
    assert!(answer.path.is_some(), "eight attempts found no path");
    let spans = checks.spans.into_inner().expect("the spans");
    assert_eq!(spans.threads.len(), 2 * AT_ONCE, "the attempts' threads");
    let (first, second) = spans.threads.split_at(AT_ONCE);
    let first_ended = first.iter().map(|span| span.2).max();
    let second_started = second.iter().map(|span| span.1).min();
    assert!(
        first_ended < second_started,
        "the first four checked until check {first_ended:?}, and the second four from check \
         {second_started:?}"
    );
}

/// What a sample of `ScriptedGoals` answers, given the attempt, the sample's count in its
/// attempt from 0 and the goal to write.
type Script<'a> = Box<dyn Fn(u32, usize, &mut [f64]) -> Result<GoalSample, CheckError> + Sync + 'a>;

/// Goal samplings whose samples answer as a script says, each sample and each attempt
/// counted.
struct ScriptedGoals<'a> {
    script: Script<'a>,
    samples: AtomicUsize,
    attempts: Mutex<Vec<u32>>,
}

impl<'a> ScriptedGoals<'a> {
    fn new(
        script: impl Fn(u32, usize, &mut [f64]) -> Result<GoalSample, CheckError> + Sync + 'a,
    ) -> Self {
        Self {
            script: Box::new(script),
            samples: AtomicUsize::new(0),
            attempts: Mutex::new(Vec::new()),
        }
    }

    fn samples(&self) -> usize {
        self.samples.load(Ordering::SeqCst)
    }
}

impl SampledGoals for ScriptedGoals<'_> {
    fn sampling(&self, attempt: u32) -> Box<dyn GoalSampling + '_> {
        self.attempts.lock().expect("no panic").push(attempt);
        Box::new(Scripted {
            goals: self,
            attempt,
            taken: 0,
        })
    }
}

/// The sampling of one attempt of `ScriptedGoals`.
struct Scripted<'s, 'a> {
    goals: &'s ScriptedGoals<'a>,
    attempt: u32,
    taken: usize,
}

impl GoalSampling for Scripted<'_, '_> {
    fn sample(&mut self, goal: &mut [f64]) -> Result<GoalSample, CheckError> {
        self.goals.samples.fetch_add(1, Ordering::SeqCst);
        let answer = (self.goals.script)(self.attempt, self.taken, goal);
        self.taken += 1;
        answer
    }
}

/// Samplings of `goals` in turn, each after `misses` samples that find none, then ended.
fn listed(goals: &[Vec<f64>], misses: usize) -> ScriptedGoals<'_> {
    ScriptedGoals::new(move |_, taken, goal| {
        let Some(found) = goals.get(taken / (misses + 1)) else {
            return Ok(GoalSample::Ended);
        };
        if taken % (misses + 1) < misses {
            return Ok(GoalSample::Missed);
        }
        goal.copy_from_slice(found);
        Ok(GoalSample::Found)
    })
}

// Every planner plans to goals sampled while it plans, some samples finding none: it reaches
// one of them, and its plan repeats under one seed, or whether PRM finds a path.
#[test]
fn a_plan_of_sampled_goals_reaches_one_and_repeats() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, GOAL_COUNTS[0]);
    for planner in Planner::ALL {
        let sampled = listed(&goals, 2);
        let request = Request {
            goals: Goals::Sampled(&sampled),
            ..ends.request(planner, 1)
        };
        let first = plan(&request, &world).expect("a plan");
        match &first.path {
            Some(path) => {
                let reached = path.last().expect("a path has states");
                assert!(goals.contains(reached), "{planner} reaches no sampled goal");
            }
            None => assert!(!planner.repeats(), "{planner} found no path"),
        }
        let first = repeating(planner, first);
        let again = repeating(planner, plan(&request, &world).expect("a plan"));
        assert_eq!(again, first, "{planner}");
    }
}

// RRTConnect takes a sampled goal when it asks for one, where it takes a given goal: one
// goal found after any misses, and several found each at the sample that asks for it, give
// the plan of the same goals given.
#[test]
fn rrt_connect_plans_to_sampled_goals_as_to_given_ones() {
    for dimension in [2, 7] {
        let world = wall(dimension);
        let ends = WallEnds::new(dimension);
        let several = goals_beyond_the_wall(dimension, GOAL_COUNTS[1]);
        let cases: [(&[Vec<f64>], usize); 3] = [(&ends.goals, 0), (&ends.goals, 3), (&several, 0)];
        for (goals, misses) in cases {
            let given = Request {
                goals: Goals::States(goals),
                ..ends.request(Planner::RrtConnect, 1)
            };
            let sampled = listed(goals, misses);
            let on_demand = Request {
                goals: Goals::Sampled(&sampled),
                ..ends.request(Planner::RrtConnect, 1)
            };
            let answer = plan(&on_demand, &world).expect("a plan");
            assert!(answer.path.is_some(), "no path in {dimension} dimensions");
            assert_eq!(
                answer,
                plan(&given, &world).expect("a plan"),
                "{} goals in {dimension} dimensions, {misses} misses each",
                goals.len()
            );
        }
    }
}

// PRM* takes every sampled goal before its roadmap grows, whatever samples miss among them:
// its plan is the plan of the same goals given.
#[test]
fn prm_star_plans_to_sampled_goals_as_to_given_ones() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, GOAL_COUNTS[1]);
    let given = Request {
        goals: Goals::States(&goals),
        ..ends.request(Planner::PrmStar, 1)
    };
    let sampled = listed(&goals, 2);
    let on_demand = Request {
        goals: Goals::Sampled(&sampled),
        ..ends.request(Planner::PrmStar, 1)
    };
    let answer = plan(&on_demand, &world).expect("a plan");
    assert!(answer.path.is_some(), "no path");
    assert_eq!(answer, plan(&given, &world).expect("a plan"));
}

// A sampling that has not ended holds no planner back: RRTConnect plans to the goal of the
// first sample, though no later sample finds one and the sampling never ends.
#[test]
fn a_sampling_that_has_not_ended_holds_no_planner_back() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let sampled = ScriptedGoals::new(|_, taken, goal| {
        if taken > 0 {
            return Ok(GoalSample::Missed);
        }
        goal.copy_from_slice(ends.goal());
        Ok(GoalSample::Found)
    });
    let request = Request {
        goals: Goals::Sampled(&sampled),
        time_limit: ANSWER_GUARD,
        ..ends.request(Planner::RrtConnect, 1)
    };
    let answer = plan(&request, &world).expect("a plan");
    assert!(
        answer.path.is_some(),
        "no path, ended by {:?}",
        answer.ended_by
    );
}

// A sampling that ends with no goal takes no sample after it, and every planner answers what
// has no goal: those that hold a tree or a roadmap of goals that its goals are invalid, those
// that grow from the start alone, which plan to their bound, that they found no path.
#[test]
fn a_sampling_that_ends_with_no_goal_leaves_no_path() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    for planner in Planner::ALL {
        let sampled = ScriptedGoals::new(|_, taken, _| {
            Ok(if taken < 2 {
                GoalSample::Missed
            } else {
                GoalSample::Ended
            })
        });
        let request = Request {
            goals: Goals::Sampled(&sampled),
            ..ends.request(planner, 1)
        };
        let answer = plan(&request, &world);
        match planner {
            Planner::Rrt | Planner::RrtStar | Planner::Kpiece1 | Planner::Est => {
                assert_eq!(answer.map(|plan| plan.path), Ok(None), "{planner}")
            }
            _ => assert_eq!(answer, Err(PlanError::InvalidGoal), "{planner}"),
        }
        assert_eq!(sampled.samples(), 3, "{planner}");
    }
}

// An error or a panic in a sampling, and a goal it finds outside the space, end the plan
// with what they say, and the next plan in the process succeeds.
#[test]
fn a_failing_sampling_ends_its_plan() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let plan_of = |sampled: &dyn SampledGoals| {
        let request = Request {
            goals: Goals::Sampled(sampled),
            ..ends.request(Planner::RrtConnect, 1)
        };
        plan(&request, &world)
    };
    assert_eq!(
        plan_of(&ScriptedGoals::new(|_, _, _| Err(
            "the sampling broke".into()
        ))),
        Err(PlanError::Check("the sampling broke".to_string()))
    );
    assert_eq!(
        plan_of(&ScriptedGoals::new(|_, _, _| panic!(
            "the sampling panicked"
        ))),
        Err(PlanError::Check(
            "panicked: the sampling panicked".to_string()
        ))
    );
    assert_eq!(
        plan_of(&ScriptedGoals::new(|_, _, goal| {
            goal.fill(f64::NAN);
            Ok(GoalSample::Found)
        })),
        Err(PlanError::Check(
            "a sampled goal holds NaN in dimension 0, which runs from 0 to 1; a goal lies within the space's bounds"
                .to_string()
        ))
    );
    assert!(
        wall_plan(Planner::RrtConnect, 2, 1).path.is_some(),
        "the plan after the failures found no path"
    );
}

// The caller's stop ends a planner waiting for its first sampled goal, as it ends a planner
// later on: the plan answers no path and says the stop ended it.
#[test]
fn a_stop_ends_the_wait_for_the_first_goal() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    for planner in [Planner::RrtConnect, Planner::Prm, Planner::PrmStar] {
        let stop = AtomicBool::new(false);
        let sampled = ScriptedGoals::new(|_, taken, _| {
            stop.store(taken >= 2, Ordering::SeqCst);
            Ok(GoalSample::Missed)
        });
        let request = Request {
            goals: Goals::Sampled(&sampled),
            stop: Some(&stop),
            ..ends.request(planner, 1)
        };
        assert_eq!(
            plan(&request, &world).expect("a plan"),
            Plan {
                path: None,
                simplified: None,
                ended_by: Ending::Stopped,
            },
            "{planner}"
        );
        assert_eq!(sampled.samples(), 3, "{planner}");
    }
}

// The time limit ends a planner waiting on a sampling that neither finds a goal nor ends:
// the plan answers no path and says the time limit ended it.
#[test]
fn the_time_limit_ends_the_wait_for_the_first_goal() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    for planner in [Planner::RrtConnect, Planner::Prm, Planner::PrmStar] {
        let sampled = ScriptedGoals::new(|_, _, _| Ok(GoalSample::Missed));
        let request = Request {
            goals: Goals::Sampled(&sampled),
            time_limit: SHORT_TIME_LIMIT,
            ..ends.request(planner, 1)
        };
        assert_eq!(
            plan(&request, &world).expect("a plan"),
            Plan {
                path: None,
                simplified: None,
                ended_by: Ending::TimeLimit,
            },
            "{planner}"
        );
    }
}

// A goal found past the time limit joins no goals: in a world with nothing in the way, where
// RRTConnect's first iteration reaches a goal it holds, the plan answers no path, and says
// the time limit ended it.
#[test]
fn a_goal_found_past_the_time_limit_joins_no_goals() {
    let world = World {
        obstacles: Vec::new(),
        motion_step: MOTION_STEP,
    };
    let ends = WallEnds::new(2);
    let sampled = ScriptedGoals::new(|_, _, goal| {
        thread::sleep(2 * SHORT_TIME_LIMIT);
        goal.copy_from_slice(ends.goal());
        Ok(GoalSample::Found)
    });
    let request = Request {
        goals: Goals::Sampled(&sampled),
        time_limit: SHORT_TIME_LIMIT,
        ..ends.request(Planner::RrtConnect, 1)
    };
    assert_eq!(
        plan(&request, &world).expect("a plan"),
        Plan {
            path: None,
            simplified: None,
            ended_by: Ending::TimeLimit,
        }
    );
}

// Each attempt of a plan of several starts a sampling of its own, on its first sample, in
// each batch of attempts, and the plan repeats, path for path.
#[test]
fn each_attempt_samples_its_own_goals() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, GOAL_COUNTS[0]);
    let plan_once = || {
        let sampled = listed(&goals, 1);
        let request = Request {
            goals: Goals::Sampled(&sampled),
            attempts: NonZeroU32::new(2 * ATTEMPTS).expect("nonzero"),
            simplification: NonZeroU32::new(SIMPLIFICATION),
            ..ends.request(Planner::RrtConnect, 1)
        };
        let answer = plan(&request, &world).expect("a plan");
        let mut attempts = sampled.attempts.lock().expect("no panic").clone();
        attempts.sort_unstable();
        assert_eq!(attempts, (0..2 * ATTEMPTS).collect::<Vec<u32>>());
        answer
    };
    let first = plan_once();
    assert!(first.path.is_some(), "no path");
    assert_eq!(plan_once(), first);
}

// A sampling ends with its planner: the simplifier, which reads the plan's goals, samples no
// more of them, and the same plan with no simplification takes as many samples.
#[test]
fn a_simplifier_samples_no_goal() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, GOAL_COUNTS[1]);
    let samples_of = |simplification| {
        let sampled = listed(&goals, 1);
        let request = Request {
            goals: Goals::Sampled(&sampled),
            simplification,
            ..ends.request(Planner::RrtConnect, 1)
        };
        let answer = plan(&request, &world).expect("a plan");
        assert!(answer.path.is_some(), "no path");
        sampled.samples()
    };
    assert_eq!(
        samples_of(NonZeroU32::new(SIMPLIFICATION)),
        samples_of(None)
    );
}

// The simplifier of a plan of several attempts reads the goals of all of them: each attempt
// samples a goal of its own, attempt 1 the nearest to the wall's opening, and under some seed
// the simplifier moves the end of the path of another attempt to it.
#[test]
fn a_simplifier_reads_the_goals_of_every_attempt() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    let goals = goals_beyond_the_wall(2, ATTEMPTS as usize);
    let nearest = goals.last().expect("goals beyond the wall");
    let goal_of = |attempt: u32| match attempt {
        1 => nearest,
        k if k + 1 == ATTEMPTS => &goals[1],
        k => &goals[k as usize],
    };
    let moved_to_the_nearest = |seed| {
        let sampled = ScriptedGoals::new(|attempt, taken, goal| {
            if taken > 0 {
                return Ok(GoalSample::Ended);
            }
            goal.copy_from_slice(goal_of(attempt));
            Ok(GoalSample::Found)
        });
        let request = Request {
            goals: Goals::Sampled(&sampled),
            attempts: NonZeroU32::new(ATTEMPTS).expect("nonzero"),
            simplification: NonZeroU32::new(SIMPLIFICATION),
            ..ends.request(Planner::RrtConnect, seed)
        };
        let answer = plan(&request, &world).expect("a plan");
        let planned = answer.path.expect("a path");
        let simplified = answer.simplified.expect("a simplified path").path;
        planned.last() != Some(nearest) && simplified.last() == Some(nearest)
    };
    assert!((1..=SIMPLIFIER_SEEDS).any(moved_to_the_nearest));
}

/// A sampling that panics as it drops.
struct SamplingPanicsWhenDropped;

impl GoalSampling for SamplingPanicsWhenDropped {
    fn sample(&mut self, _: &mut [f64]) -> Result<GoalSample, CheckError> {
        Ok(GoalSample::Ended)
    }
}

impl Drop for SamplingPanicsWhenDropped {
    fn drop(&mut self) {
        panic!("the sampling's drop broke");
    }
}

struct PanickingDrops;

impl SampledGoals for PanickingDrops {
    fn sampling(&self, _: u32) -> Box<dyn GoalSampling + '_> {
        Box::new(SamplingPanicsWhenDropped)
    }
}

// A panic in the drop of a sampling ends its plan with what it said, at the plan's end and
// where a later batch's attempt replaces it, and the next plan in the process succeeds.
#[test]
fn a_panic_in_the_drop_of_a_sampling_ends_its_plan() {
    let world = wall(2);
    let ends = WallEnds::new(2);
    for attempts in [1, 2 * ATTEMPTS] {
        let request = Request {
            goals: Goals::Sampled(&PanickingDrops),
            attempts: NonZeroU32::new(attempts).expect("nonzero"),
            ..ends.request(Planner::RrtConnect, 1)
        };
        assert_eq!(
            plan(&request, &world),
            Err(PlanError::Check(
                "panicked: the sampling's drop broke".to_string()
            )),
            "{attempts} attempts"
        );
    }
    assert!(
        wall_plan(Planner::RrtConnect, 2, 1).path.is_some(),
        "the plan after the panic found no path"
    );
}

// A sampled goal off the constraint joins no goals: a constrained plan reaches the goal on
// the constraint that its sampling finds after it.
#[test]
fn a_sampled_goal_off_the_constraint_is_left_out() {
    let (space, start, goal) = sphere_ends();
    let off = vec![0.1; goal.len()];
    let goals = [off, goal.clone()];
    let given = [goal.clone()];
    let sampled = listed(&goals, 0);
    let request = Request {
        goals: Goals::Sampled(&sampled),
        ..on_the_sphere(Planner::RrtConnect, &space, &start, &given, &UnitSphere)
    };
    let path = plan(&request, &Equator)
        .expect("a plan")
        .path
        .expect("a path on the sphere");
    assert_walks_the_sphere(&path, &start, &goal, "the path to the sampled goal");
}
