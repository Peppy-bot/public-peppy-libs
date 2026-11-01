//! Parity with OMPL's planners on every planning scenario of the ROS reference harness.
//! A plan that its iteration bound ends gives the same path under one seed, for every
//! planner but PRM, whose path is the first that its search finds; and the evidence that
//! the plan is OMPL's: each plan of the scenario gives the status, the ending, the path,
//! its length and its simplification that the C++ program gives with the image's OMPL,
//! under the same space, checks, parameters, seed and stop rule, bit for bit with the
//! committed outputs on x86-64; its ending, its path and its length within a bound of the
//! outputs of every image; and its simplification within that bound of the committed
//! outputs on every architecture. A plan of PRM finds a path exactly where the C++
//! program's does. The C++ program plans PRM and PRM* through the bridge's subclass of
//! each, code for code.

mod world;

use std::num::{NonZeroU32, NonZeroU64};
use std::time::Duration;

use ompl_bridge::{Planner, Request, Space, plan};
use ros_reference::planning::{Case, Ending, PlanCase, PlanStatus, Planned};
use world::{World, length};

/// The time limit of the plans here, which never passes: a plan under it has no deadline.
const TIME_LIMIT: Duration = Duration::MAX;

/// The largest difference of a value of a planned path, or of its length, from the
/// outputs of an image, and of a value of a simplified path from the committed outputs.
/// The arm64 image's OMPL fuses multiply-adds, and its paths of `planning_boxes` differ
/// from the amd64 ones by at most 3.3e-16 in a value and 1.8e-15 in a length.
const ACROSS_IMAGES: f64 = 1e-12;

/// The bridge's C++ source, which holds the subclass that PRM and PRM* plan through.
const BRIDGE_SOURCE: &str = include_str!("../src/bridge.cpp");

/// The C++ program's planning reference, which holds its copy of that subclass.
const REFERENCE_SOURCE: &str = include_str!("../../ros_reference/reference_programs/planning.cpp");

/// The start of the subclass's code in each source: the first of its constants.
const ROADMAP_CODE_START: &str = "constexpr unsigned int expansion_round";

/// The declaration of the subclass.
const ROADMAP_CLASS: &str = "class RepeatableRoadmap";

/// The first line `};` after the declaration, which ends the class.
const CLASS_END: &str = "\n};\n";

/// The number of words of each source that a difference of the subclass's code shows.
const DIFFERENCE_WORDS: usize = 8;

/// The worlds of `case`, each with its checks.
fn worlds(case: &Case) -> Vec<World> {
    case.worlds
        .iter()
        .map(|world| World {
            obstacles: world.obstacles.clone(),
            motion_step: case.motion_step,
        })
        .collect()
}

/// A plan of world `index` of `case` under `planner`, `seed`, `iterations` and
/// `simplification`, of which 0 skips the simplification.
fn plan_world(
    case: &Case,
    worlds: &[World],
    index: usize,
    planner: Planner,
    seed: u32,
    iterations: u64,
    simplification: u32,
) -> ompl_bridge::Plan {
    let world = &case.worlds[index];
    let space = Space::new(&vec![(0.0, 1.0); world.start.len()]).expect("the unit cube");
    let goals = [world.goal.clone()];
    let request = Request {
        planner,
        space: &space,
        start: &world.start,
        goals: &goals,
        time_limit: TIME_LIMIT,
        iterations: NonZeroU64::new(iterations).expect("nonzero"),
        seed: NonZeroU32::new(seed).expect("nonzero"),
        simplification: NonZeroU32::new(simplification),
        projection: None,
    };
    plan(&request, &worlds[index]).expect("a plan")
}

/// What ended a plan's planner, as the reference output names it.
fn ending(ending: &ompl_bridge::Ending) -> Ending {
    match ending {
        ompl_bridge::Ending::Returned => Ending::PlannerReturned,
        ompl_bridge::Ending::IterationBound => Ending::IterationBound,
        ompl_bridge::Ending::TimeLimit => Ending::TimeLimit,
    }
}

/// The bits of each value of each state of `path`.
#[cfg(target_arch = "x86_64")]
fn bits(path: Option<&Vec<Vec<f64>>>) -> Option<Vec<Vec<u64>>> {
    path.map(|states| {
        states
            .iter()
            .map(|state| state.iter().map(|v| v.to_bits()).collect())
            .collect()
    })
}

/// Whether `ours` holds as many states as `theirs`, each value within ACROSS_IMAGES of
/// theirs.
fn within_across_images(ours: &[Vec<f64>], theirs: &[Vec<f64>]) -> bool {
    ours.len() == theirs.len()
        && ours.iter().zip(theirs).all(|(our, their)| {
            our.len() == their.len()
                && our
                    .iter()
                    .zip(their)
                    .all(|(a, b)| (a - b).abs() <= ACROSS_IMAGES)
        })
}

/// The plan of `p` of `case` through its world.
fn plan_case(case: &Case, worlds: &[World], p: &PlanCase) -> ompl_bridge::Plan {
    let planner: Planner = p.planner.parse().expect("a planner of OMPL");
    plan_world(
        case,
        worlds,
        p.world,
        planner,
        p.seed,
        p.iterations,
        p.simplification,
    )
}

/// Asserts that `answer` gives the C++ program's ending in `planned`, and a path of as
/// many states as its path, each value and the length within ACROSS_IMAGES.
fn assert_planned(what: &str, answer: &ompl_bridge::Plan, planned: &Planned) {
    assert_eq!(
        ending(&answer.ended_by),
        planned.ended_by,
        "{what}: the ending"
    );
    let (Some(ours), Some(theirs)) = (answer.path.as_deref(), planned.path.as_deref()) else {
        assert_eq!(
            answer.path.is_some(),
            planned.path.is_some(),
            "{what}: a path where the C++ program has one"
        );
        return;
    };
    assert!(
        within_across_images(ours, theirs),
        "{what}: the path of {} states, {ours:?}, against the C++ program's {} states, \
         {theirs:?}",
        ours.len(),
        theirs.len()
    );
    let difference = (length(ours) - planned.length.expect("its length")).abs();
    assert!(
        difference <= ACROSS_IMAGES,
        "{what}: the length differs from the C++ program's by {difference}"
    );
}

/// Asserts that `answer` gives a simplification exactly where `committed`, the committed
/// output, holds one, of as many states as it, each value within ACROSS_IMAGES.
fn assert_simplified(what: &str, answer: &ompl_bridge::Plan, committed: &Planned) {
    let ours = answer.simplified.as_ref().map(|s| s.path.as_slice());
    let (Some(ours), Some(theirs)) = (ours, committed.simplified.as_deref()) else {
        assert_eq!(
            ours.is_some(),
            committed.simplified.is_some(),
            "{what}: a simplification where the committed output has one"
        );
        return;
    };
    assert!(
        within_across_images(ours, theirs),
        "{what}: the simplification of {} states, {ours:?}, against the committed {} states, \
         {theirs:?}",
        ours.len(),
        theirs.len()
    );
}

/// Asserts that `answer` gives the C++ program's path, length and simplification in
/// `planned` bit for bit.
#[cfg(target_arch = "x86_64")]
fn assert_planned_bits(what: &str, answer: &ompl_bridge::Plan, planned: &Planned) {
    assert_eq!(
        bits(answer.path.as_ref()),
        bits(planned.path.as_ref()),
        "{what}: the path"
    );
    assert_eq!(
        answer.path.as_deref().map(|path| length(path).to_bits()),
        planned.length.map(f64::to_bits),
        "{what}: the length, as OMPL measures it"
    );
    assert_eq!(
        bits(answer.simplified.as_ref().map(|s| &s.path)),
        bits(planned.simplified.as_ref()),
        "{what}: the simplified path"
    );
}

// Each plan finds an exact path exactly where the C++ program's does. Each plan of a
// planner other than PRM gives the C++ program's ending, and a path of as many states,
// each value and the length within ACROSS_IMAGES, against the outputs it reads: the
// committed outputs, which the amd64 image wrote, or the outputs of the image that
// `ROS_REFERENCE_OUTPUTS` names, as run.sh names the arm64 image's. Its simplification
// lies within ACROSS_IMAGES of the committed one on every architecture: the simplifier's
// random shortcuts branch on the last bits of the path, and 6 of the 21 simplifications
// of the arm64 image differ from the amd64 ones beyond ACROSS_IMAGES, while this crate
// built for aarch64 gave every committed simplification bit for bit on a Jetson. On
// x86-64, each plan gives the path, the length and the simplification of the committed
// outputs bit for bit: OMPL's own sources, as the crate compiles them, and the test's
// checks round each product and each sum on its own, as the amd64 image does. A plan of
// PRM is compared by whether it found a path alone: its path is the first that its search
// finds. Each scenario holds a plan of each planner that finds a path and one that finds
// none.
#[test]
fn each_plan_gives_ompls_answer() {
    let cases = Case::load_all();
    let committed = Case::load_committed();
    assert!(!cases.is_empty(), "the harness holds a planning scenario");
    for (case, committed) in cases.iter().zip(&committed) {
        assert_eq!(case.name, committed.name, "the scenarios in one order");
        let worlds = worlds(case);
        for planner in Planner::ALL.map(|p| p.to_string()) {
            for exact in [true, false] {
                assert!(
                    case.plans
                        .iter()
                        .any(|p| p.planner == planner && (p.status == PlanStatus::Exact) == exact),
                    "{}: a plan of {planner} that {} a path",
                    case.name,
                    if exact { "finds" } else { "finds no" }
                );
            }
        }
        for (p, committed) in case.plans.iter().zip(&committed.plans) {
            let what = format!(
                "{}: {} with seed {} in world {}",
                case.name, p.planner, p.seed, p.world
            );
            let answer = plan_case(case, &worlds, p);
            assert_eq!(
                answer.path.is_some(),
                p.status == PlanStatus::Exact,
                "{what}: an exact path where the C++ program found one"
            );
            if let Some(planned) = &p.planned {
                assert_planned(&what, &answer, planned);
                #[cfg(target_arch = "x86_64")]
                assert_planned_bits(&what, &answer, planned);
            }
            if let Some(planned) = &committed.planned {
                assert_simplified(&what, &answer, planned);
            }
        }
    }
}

/// The code of the subclass of PRM and PRM* in `source`, with its constants: the words
/// from `ROADMAP_CODE_START` to the end of the class, without line comments.
fn roadmap_code(source: &str) -> Vec<&str> {
    let start = source
        .find(ROADMAP_CODE_START)
        .expect("the source holds the subclass's constants");
    let class = start
        + source[start..]
            .find(ROADMAP_CLASS)
            .expect("the source holds the subclass after its constants");
    let end = class
        + source[class..]
            .find(CLASS_END)
            .expect("the source holds the end of the subclass")
        + CLASS_END.len();
    let code: Vec<&str> = source[start..end]
        .lines()
        .map(|line| line.split_once("//").map_or(line, |(code, _comment)| code))
        .collect();
    assert!(
        code.iter()
            .all(|line| !line.contains("/*") && !line.contains('"')),
        "the subclass holds a block comment or a string literal, and roadmap_code strips \
         line comments alone"
    );
    code.into_iter().flat_map(str::split_whitespace).collect()
}

// The C++ program plans PRM and PRM* through the bridge's subclass of each: its copy and
// the bridge's, with their constants, are the same code once their comments and their
// whitespace are stripped.
#[test]
fn the_reference_program_plans_prm_through_the_bridges_subclass() {
    let bridge = roadmap_code(BRIDGE_SOURCE);
    let reference = roadmap_code(REFERENCE_SOURCE);
    let same = bridge
        .iter()
        .zip(&reference)
        .take_while(|(ours, theirs)| ours == theirs)
        .count();
    let shown = |code: &[&str]| code[same..code.len().min(same + DIFFERENCE_WORDS)].join(" ");
    assert!(
        bridge == reference,
        "the subclass of PRM and PRM* in src/bridge.cpp and its copy in \
         ros_reference/reference_programs/planning.cpp differ from word {same} on: `{}` in \
         the bridge, `{}` in the copy",
        shown(&bridge),
        shown(&reference)
    );
}
