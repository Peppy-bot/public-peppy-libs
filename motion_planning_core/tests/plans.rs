//! Plans of OpenArm v2's left arm around boxes of voxels: the rounded path of every
//! returned path keeps its clearance under a dense check and passes the check of a
//! caller's path, the same request on the same map gives the same path, of one attempt
//! or several, a stop ends a plan, and a goal or a world that admits no path says why.
//! Checks of a caller's path: where each fault lies and the waypoints before it. Plans
//! and checks among the objects of a planning scene: an object is avoided until it is
//! removed, an object the hand holds is carried around an obstacle, the fingers and the
//! table it rests on touch it once pairs allow them, an object the other arm holds stands
//! where that arm holds it, a link passes the open corner of an L-shaped mesh and collides
//! inside a closed one, and a start near an object or holding one near the arm departs.

mod common;

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread::ThreadId;
use std::time::{Duration, Instant};

use collision_geometry::chain::Rider;
use collision_geometry::gjk::{Hull, Placed};
use collision_geometry::hull::exact_hull;
use control_core::path_timing::RoundedPath;
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, ArmObjects, AttachRequest, CheckFailure, CheckRequest, Contact, Goal, GoalSampler,
    IkGoalSampler, JointTarget, MAX_STEP_M, ObjectFields, ObjectRequest, Objects, PathCheck,
    PathFault, Place, PlanFailure, PlanRequest, PlanningScene, RobotLinks, RobotPart, Scene,
    ShapeFields, StartFault, Surroundings, check_arm_path, plan_arm_path, pose_parts,
    term_clearances,
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::Planner;

use common::{
    JOINTS, LEFT_ARM, an_hour_from_now, distance_to_map, held_from_its_arm, left_fingers, map,
    occupied_at, positive, voxel_in_a_finger,
};

const VOXEL_M: f64 = 0.04;
const MIN_CLEARANCE_M: f64 = 0.02;

/// A configuration with the arm reaching forward and down, and one with it raised.
const LOW: [f64; JOINTS] = [0.4, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0];
const HIGH: [f64; JOINTS] = [-0.6, -0.5, 0.3, 1.6, 0.2, 0.3, 0.0];

/// The points of a solid box of voxels from `min` to `max`, one per voxel.
fn solid(min: Vector3<f64>, max: Vector3<f64>) -> Vec<[f64; 3]> {
    let steps = ((max - min) / VOXEL_M).map(|s| s.ceil() as usize);
    (0..=steps.x)
        .flat_map(|i| (0..=steps.y).flat_map(move |j| (0..=steps.z).map(move |k| (i, j, k))))
        .map(|(i, j, k)| {
            let p = min + Vector3::new(i as f64, j as f64, k as f64) * VOXEL_M;
            [p.x.min(max.x), p.y.min(max.y), p.z.min(max.z)]
        })
        .collect()
}

/// The grasp point of the left arm at `q`.
fn hand(q: &[f64; JOINTS]) -> Vector3<f64> {
    LEFT_ARM.chain().world_ee_pose(q).translation.vector
}

/// A map holding `points`, seen from above the robot.
fn world(points: &[[f64; 3]]) -> (VoxelMap, DistanceField) {
    let mut map = map(VOXEL_M, 1.6);
    occupied_at(&mut map, [0.0, 0.0, 3.0], points);
    let field = DistanceField::of(&map);
    (map, field)
}

/// The goal of every joint within 0.01 rad of `q`.
fn at(q: &[f64; JOINTS]) -> Goal {
    let joints = (0..JOINTS)
        .map(|joint| JointTarget {
            joint,
            position: q[joint],
            above: positive(0.01),
            below: positive(0.01),
        })
        .collect();
    Goal::new(JOINTS, joints, Vec::new(), Vec::new()).expect("a valid goal")
}

fn request<'a>(
    planner: Planner,
    start: [f64; JOINTS],
    goal: &'a Goal,
    sampler: &'a IkGoalSampler<JOINTS>,
) -> PlanRequest<'a, JOINTS> {
    PlanRequest {
        start,
        start_tolerance: [0.0; JOINTS],
        goal,
        sampler,
        goal_samples: NonZeroUsize::new(8).expect("nonzero"),
        planner,
        deadline: an_hour_from_now(),
        max_iterations: NonZeroU64::new(20_000).expect("nonzero"),
        attempts: NonZeroU32::MIN,
        seed: 7,
        min_clearance_m: MIN_CLEARANCE_M,
        stop: None,
    }
}

/// Each configuration of `waypoints` as a vector of its joint values.
fn vectors(waypoints: &[[f64; JOINTS]]) -> Vec<Vec<f64>> {
    waypoints.iter().map(|q| q.to_vec()).collect()
}

/// The points of `path` at the positions [`RoundedPath::arcs_every`] gives.
fn probes(path: &RoundedPath, spacing: PositiveFinite) -> Vec<Vec<f64>> {
    path.arcs_every(spacing)
        .into_iter()
        .map(|s| path.at(s))
        .collect()
}

/// Asserts that the rounded path through `waypoints` keeps `MIN_CLEARANCE_M` from the map
/// at every state of a dense check.
fn assert_dense_clear(
    arm: &Arm<JOINTS>,
    map: &VoxelMap,
    field: &DistanceField,
    waypoints: &[[f64; JOINTS]],
) {
    let surroundings = Surroundings::new(map, field, term_clearances(MIN_CLEARANCE_M));
    assert_dense_clear_of(arm, &surroundings, waypoints);
}

/// The states of the rounded path through `waypoints`, so dense that no point of `arm`
/// moves a tenth of a voxel between two.
fn dense_states(arm: &Arm<JOINTS>, waypoints: &[[f64; JOINTS]]) -> Vec<[f64; JOINTS]> {
    let path = RoundedPath::joint_path(&vectors(waypoints));
    let spacing = PositiveFinite::parse(VOXEL_M / 10.0 / arm.lever()).expect("positive");
    probes(&path, spacing)
        .iter()
        .map(|probe| probe.as_slice().try_into().expect("a configuration"))
        .collect()
}

/// Asserts that the rounded path through `waypoints` keeps the clearances of
/// `surroundings` at every state of a dense check.
fn assert_dense_clear_of(
    arm: &Arm<JOINTS>,
    surroundings: &Surroundings,
    waypoints: &[[f64; JOINTS]],
) {
    for q in dense_states(arm, waypoints) {
        assert!(
            surroundings.clear(arm, &q),
            "the rounded path comes within its clearance at {q:?}"
        );
    }
}

/// A box in front of the arm, under the grasp point's straight joint motion from `LOW`
/// to `HIGH` and clear of the arm at both.
fn box_between() -> Vec<[f64; 3]> {
    solid(
        Vector3::new(0.30, 0.10, 0.30),
        Vector3::new(0.45, 0.30, 0.42),
    )
}

// With a box across the way, each planner's path reaches the goal, and its rounded path
// keeps the clearance under a dense check.
#[test]
fn a_plan_around_a_box_keeps_its_clearance_on_its_rounded_path() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let straight = Surroundings::new(&map, &field, term_clearances(MIN_CLEARANCE_M));
    assert!(
        (0..=100).any(|k| {
            let t = f64::from(k) / 100.0;
            let q: [f64; JOINTS] = std::array::from_fn(|i| LOW[i] + (HIGH[i] - LOW[i]) * t);
            !straight.clear(&arm, &q)
        }),
        "the straight motion clears the box: the plans need not go around it"
    );
    for planner in [
        Planner::RrtConnect,
        Planner::Rrt,
        Planner::Kpiece1,
        Planner::Est,
    ] {
        let waypoints = plan_arm_path(&arm, &scene, &request(planner, LOW, &goal, &sampler))
            .unwrap_or_else(|e| panic!("{planner}: {e}"));
        assert_eq!(waypoints.first(), Some(&LOW), "{planner} starts elsewhere");
        let last = waypoints.last().expect("a waypoint");
        assert!(
            goal.met(LEFT_ARM.chain(), last),
            "{planner} ends off the goal"
        );
        assert_dense_clear(&arm, &map, &field, &waypoints);
    }
}

// The same request on the same map gives the same path, and another stamp of the map
// another.
#[test]
fn the_same_request_on_the_same_map_gives_the_same_path() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let plan_at = |stamp: u64| {
        let scene = Scene {
            map: &map,
            field: &field,
            stamp,
            objects: Objects::none(),
        };
        plan_arm_path(
            &arm,
            &scene,
            &request(Planner::RrtConnect, LOW, &goal, &sampler),
        )
        .expect("a path")
    };
    assert_eq!(plan_at(1), plan_at(1));
    assert_ne!(plan_at(1), plan_at(2));
}

// A goal inside a box is unreachable, and the message says that each configuration
// meeting it is in collision, nearer than the plan's clearance it names, and names the
// link of the one that falls short least nearest the map.
#[test]
fn a_goal_inside_an_obstacle_is_unreachable() {
    let goal_hand = hand(&HIGH);
    let (map, field) = world(&solid(
        goal_hand - Vector3::repeat(0.1),
        goal_hand + Vector3::repeat(0.1),
    ));
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let failure = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    )
    .expect_err("no path into a box");
    let PlanFailure::Unreachable(message) = failure else {
        panic!("not unreachable: {failure}");
    };
    let (clearance, short) = message
        .split_once("; the one that falls short least has ")
        .expect("the message names what the nearest configuration falls short against");
    assert!(
        clearance.ends_with(
            "that meet the goal is in collision, nearer than the plan's clearance, 0.0210 m from \
             the map and the objects"
        ),
        "{message}"
    );
    assert!(short.ends_with(" m from the map"), "{message}");
}

// A goal whose grasp point a point of the robot's own body stands on is unreachable: each
// configuration within the goal's 0.01 rad of it stands within the plan's own clearance,
// and the message names the pair of the one that falls short least and how far apart
// they stand.
#[test]
fn a_goal_within_the_clearance_of_its_own_body_names_the_pair() {
    let (map, field) = world(&[]);
    let model = HandAndBall {
        center: hand(&HIGH),
        radius: 0.0,
    };
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let failure = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    )
    .expect_err("no goal clear of the ball");
    let PlanFailure::Unreachable(message) = failure else {
        panic!("not unreachable: {failure}");
    };
    let distance: f64 = message
        .split_once("; the one that falls short least has grasp point ")
        .and_then(|(_, short)| short.strip_suffix(" m from ball"))
        .expect("the message names the pair")
        .parse()
        .expect("a distance");
    // The plan's own clearance: the clearance and 1 mm for each of the pair's two links.
    assert!(
        (0.0..MIN_CLEARANCE_M + 0.002).contains(&distance),
        "{message}"
    );
}

// A plan whose iteration bound ends it before it finds a path gives no path, and says
// what ended it and the plan's clearance it keeps.
#[test]
fn a_plan_its_iterations_end_gives_no_path() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let short = PlanRequest {
        max_iterations: NonZeroU64::new(1).expect("nonzero"),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    let failure = plan_arm_path(&arm, &scene, &short).expect_err("one iteration finds no path");
    assert_eq!(
        failure,
        PlanFailure::NoPath(
            "RRTConnect found no path within 1 iterations or by its deadline, keeping the \
             plan's clearance, 0.0210 m from the map and the objects"
                .to_string()
        )
    );
}

/// A goal sampler that waits out the plan's time limit, then answers `0`.
struct WaitingOut([f64; JOINTS]);

impl GoalSampler<JOINTS> for WaitingOut {
    fn sample(
        &self,
        _: &Goal,
        _: usize,
        _: u64,
        deadline: Instant,
        _: Option<&AtomicBool>,
    ) -> Vec<[f64; JOINTS]> {
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
        vec![self.0]
    }
}

// The time limit bounds the goal sampling and the planner's plan: a plan whose goal
// sampling takes all of it gives no path, and says the time ended before the planner
// planned.
#[test]
fn a_plan_whose_sampling_takes_its_time_gives_no_path() {
    let (map, field) = world(&[]);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let waiting = WaitingOut(HIGH);
    let limited = PlanRequest {
        sampler: &waiting,
        deadline: Instant::now() + Duration::from_millis(50),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    assert_eq!(
        plan_arm_path(&arm, &scene, &limited),
        Err(PlanFailure::NoPath(
            "the deadline passed before RRTConnect planned".to_string()
        ))
    );
}

// From a start 1 cm above a slab, with a goal raised away from it, the clearance of the
// rounded path rises from the start until it keeps the clearance, falling below the
// start's by no more than a step of the departure, at most half the start's.
#[test]
fn from_a_start_near_an_obstacle_the_clearance_rises() {
    assert_departs_from(0.01);
}

// From a start half a millimeter from the slab, where a step of the departure is a
// quarter of a millimeter, the path departs as it does from a centimeter.
#[test]
fn from_a_start_half_a_millimeter_away_the_clearance_rises() {
    assert_departs_from(0.0005);
}

/// How far a link point moves at most between two probes of a departure.
const DEPARTURE_PROBE_M: f64 = 1e-3;

/// The least a distance between a link point and a still body falls to along a
/// departure from a start `start` meters away: a step moves a point at most `MAX_STEP_M`,
/// and at most half the distance's room, above `MIN_CLEARANCE_M` at or above it
/// and above 0 below it, at the state it leaves, which is no nearer than the start.
fn departed(start: f64) -> f64 {
    let room = if start >= MIN_CLEARANCE_M {
        start - MIN_CLEARANCE_M
    } else {
        start
    };
    start - MAX_STEP_M.min(room / 2.0)
}

/// A slab under the arm, a start `start_distance_m` from it, and the turn of one joint
/// from `LOW` past the start that brings the arm nearer.
struct NearTheSlab {
    map: VoxelMap,
    field: DistanceField,
    start: [f64; JOINTS],
    toward: [f64; JOINTS],
}

/// A slab of voxels under the arm.
fn slab() -> Vec<[f64; 3]> {
    solid(Vector3::new(0.0, 0.1, 0.10), Vector3::new(0.3, 0.35, 0.16))
}

fn near_the_slab(start_distance_m: f64) -> NearTheSlab {
    let (map, field) = world(&slab());
    let arm = Arm::new(&LEFT_ARM, None);
    // A turn of one joint that brings the arm within the start distance of the slab:
    // bisect along it to where the arm comes within it.
    let limits = LEFT_ARM.chain().limits();
    let distance = |q: &[f64; JOINTS]| distance_to_map(&arm, &map, q);
    assert!(
        distance(&LOW) > start_distance_m,
        "the arm starts within {start_distance_m} m of the slab"
    );
    let toward = (0..JOINTS)
        .flat_map(|j| (1..=16).flat_map(move |k| [0.05, -0.05].map(|d| (j, d * f64::from(k)))))
        .map(|(j, delta)| {
            let mut q = LOW;
            q[j] = (q[j] + delta).clamp(limits[j].lo, limits[j].hi);
            q
        })
        .find(|q| distance(q) < start_distance_m)
        .expect("a turn of one joint brings the arm to the slab");
    let along =
        |t: f64| -> [f64; JOINTS] { std::array::from_fn(|i| LOW[i] + (toward[i] - LOW[i]) * t) };
    let (mut far, mut near) = (0.0, 1.0);
    while near - far > 1e-9 {
        let mid = (far + near) / 2.0;
        if distance(&along(mid)) > start_distance_m {
            far = mid;
        } else {
            near = mid;
        }
    }
    let start = along(far);
    let start_clearance = distance(&start);
    assert!(
        (start_clearance - start_distance_m).abs() < start_distance_m / 10.0,
        "the start lies {start_clearance} from the slab"
    );
    NearTheSlab {
        map,
        field,
        start,
        toward,
    }
}

/// Asserts that from a start `start_distance_m` from a slab, with a goal raised away
/// from it, the clearance of the rounded path rises from the start until it keeps the
/// clearance.
fn assert_departs_from(start_distance_m: f64) {
    let NearTheSlab {
        map, field, start, ..
    } = near_the_slab(start_distance_m);
    let arm = Arm::new(&LEFT_ARM, None);
    let start_clearance = distance_to_map(&arm, &map, &start);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, start, &goal, &sampler),
    )
    .expect("a path away from the slab");
    let path = RoundedPath::joint_path(&vectors(&waypoints));
    let spacing = PositiveFinite::parse(DEPARTURE_PROBE_M / arm.lever()).expect("positive");
    let mut best = start_clearance;
    for probe in probes(&path, spacing) {
        let q: [f64; JOINTS] = probe.as_slice().try_into().expect("a configuration");
        let here = distance_to_map(&arm, &map, &q);
        if best >= MIN_CLEARANCE_M {
            break;
        }
        assert!(
            here >= departed(start_clearance),
            "the clearance falls from {start_clearance} to {here} at {q:?}"
        );
        best = best.max(here);
    }
    assert!(
        best >= MIN_CLEARANCE_M,
        "the path never keeps its clearance"
    );
    assert_dense_clear_after_departure(&arm, &map, &field, &waypoints);
}

/// Asserts that the rounded path through `waypoints` keeps `MIN_CLEARANCE_M` at every
/// state of a dense check from the first that keeps it on.
fn assert_dense_clear_after_departure(
    arm: &Arm<JOINTS>,
    map: &VoxelMap,
    field: &DistanceField,
    waypoints: &[[f64; JOINTS]],
) {
    let path = RoundedPath::joint_path(&vectors(waypoints));
    let spacing = PositiveFinite::parse(VOXEL_M / 10.0 / arm.lever()).expect("positive");
    let surroundings = Surroundings::new(map, field, term_clearances(MIN_CLEARANCE_M));
    let probes = probes(&path, spacing);
    let kept = probes
        .iter()
        .position(|p| surroundings.clear(arm, &p.as_slice().try_into().expect("a configuration")))
        .expect("the path keeps its clearance somewhere");
    for probe in &probes[kept..] {
        let q: [f64; JOINTS] = probe.as_slice().try_into().expect("a configuration");
        assert!(
            surroundings.clear(arm, &q),
            "the rounded path touches the map at {q:?}"
        );
    }
}

/// Whether the middle one of three waypoints lies on the segment between the other two:
/// a waypoint the planner added near a corner.
fn lies_between(w: &[[f64; JOINTS]]) -> bool {
    let (a, q, b) = (w[0], w[1], w[2]);
    let t = (0..JOINTS)
        .map(|i| (q[i] - a[i]) * (b[i] - a[i]))
        .sum::<f64>()
        / (0..JOINTS).map(|i| (b[i] - a[i]).powi(2)).sum::<f64>();
    (0..JOINTS)
        .map(|i| (q[i] - (a[i] + (b[i] - a[i]) * t)).powi(2))
        .sum::<f64>()
        .sqrt()
        < 1e-9
}

// From a start 1 cm from the slab, with the box across the way, the plan of seed 10
// rounds its first corner into the box and adds a waypoint on the first run, which the
// start rule walks: the answer's rounded path keeps the clearance once it first keeps
// it, and the answer passes the check of a caller's path.
#[test]
fn a_first_corner_from_a_start_near_the_slab_gets_waypoints_until_it_clears() {
    let NearTheSlab { start, .. } = near_the_slab(0.01);
    let (map, field) = world(&[slab(), box_between()].concat());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &PlanRequest {
            seed: 10,
            ..request(Planner::RrtConnect, start, &goal, &sampler)
        },
    )
    .expect("a path around the box");
    assert!(
        lies_between(&waypoints[0..3]),
        "no waypoint added on the first run: {waypoints:?}"
    );
    assert_dense_clear_after_departure(&arm, &map, &field, &waypoints);
    assert_eq!(
        check(&arm, &scene, start, &waypoints[1..]),
        passing(&waypoints[1..])
    );
}

// On drawn worlds of boxes in front of the arm, clear of it at the start and at the
// goal, every path a planner returns has a rounded path that keeps the clearance under
// a dense check and passes the check of a caller's path, and every plan that returns
// none says the goal is unreachable or that it found no path. A world whose plan returns
// none is discarded, so Hegel's FilterTooMuch health check fails a run in which few plans
// return a path.
#[hegel::test(test_cases = 40)]
fn on_drawn_worlds_every_returned_path_clears_its_rounding(tc: hegel::TestCase) {
    use hegel::generators as gs;
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(3));
    let points: Vec<[f64; 3]> = (0..count)
        .flat_map(|_| {
            let corner = Vector3::new(
                tc.draw(gs::floats().min_value(0.15).max_value(0.5)),
                tc.draw(gs::floats().min_value(-0.1).max_value(0.4)),
                tc.draw(gs::floats().min_value(0.2).max_value(0.7)),
            );
            let size =
                Vector3::from_fn(|_, _| tc.draw(gs::floats().min_value(0.04).max_value(0.2)));
            solid(corner, corner + size)
        })
        .collect();
    let (map, field) = world(&points);
    let arm = Arm::new(&LEFT_ARM, None);
    let surroundings = Surroundings::new(&map, &field, term_clearances(MIN_CLEARANCE_M));
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let planner = tc.draw(gs::sampled_from(&[
        Planner::RrtConnect,
        Planner::Rrt,
        Planner::BiTrrt,
    ]));
    // A world whose boxes stand on the arm at the start or at the goal tests nothing
    // here.
    tc.assume(surroundings.clear(&arm, &LOW) && surroundings.clear(&arm, &HIGH));
    match plan_arm_path(&arm, &scene, &request(planner, LOW, &goal, &sampler)) {
        Ok(waypoints) => {
            assert_dense_clear(&arm, &map, &field, &waypoints);
            assert_eq!(
                check(&arm, &scene, LOW, &waypoints[1..]),
                passing(&waypoints[1..])
            );
        }
        Err(PlanFailure::Unreachable(_) | PlanFailure::NoPath(_)) => tc.assume(false),
        Err(other) => panic!("{planner}: {other}"),
    }
}

/// A self model of one pair: the grasp point and a ball of the robot's own body.
struct HandAndBall {
    center: Vector3<f64>,
    radius: f64,
}

impl motion_planning_core::SelfModel<JOINTS> for HandAndBall {
    fn clearance(&self, q: &[f64; JOINTS]) -> f64 {
        (hand(q) - self.center).norm() - self.radius
    }

    fn nearest_pair(&self, _: &[f64; JOINTS]) -> (String, String) {
        ("grasp point".to_string(), "ball".to_string())
    }
}

// With a self model whose ball stands on the grasp point's straight way, the path keeps
// the clearance from it at every state of a dense check.
#[test]
fn a_path_keeps_its_clearance_from_the_self_model() {
    let mid: [f64; JOINTS] = std::array::from_fn(|i| (LOW[i] + HIGH[i]) / 2.0);
    let model = HandAndBall {
        center: hand(&mid),
        radius: 0.05,
    };
    let (map, field) = world(&[]);
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    use motion_planning_core::SelfModel;
    assert!(
        model.clearance(&mid) < MIN_CLEARANCE_M,
        "the ball stands off the straight way"
    );
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    )
    .expect("a path around the ball");
    let path = RoundedPath::joint_path(&vectors(&waypoints));
    let spacing = PositiveFinite::parse(VOXEL_M / 10.0 / arm.lever()).expect("positive");
    for probe in probes(&path, spacing) {
        let q: [f64; JOINTS] = probe.as_slice().try_into().expect("a configuration");
        assert!(
            model.clearance(&q) >= MIN_CLEARANCE_M,
            "the hand comes within {} of the ball",
            model.clearance(&q)
        );
    }
}

// A start outside a joint limit by no more than its tolerance moves onto the limit; one
// farther out, or at no number, is refused, naming the joint.
#[test]
fn a_start_moves_onto_a_limit_within_its_tolerance() {
    let (map, field) = world(&[]);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let limit = LEFT_ARM.chain().limits()[3].lo;
    let mut near = LOW;
    near[3] = limit - 0.005;
    let within = PlanRequest {
        start_tolerance: [0.01; JOINTS],
        ..request(Planner::RrtConnect, near, &goal, &sampler)
    };
    let waypoints = plan_arm_path(&arm, &scene, &within).expect("a start within its tolerance");
    assert_eq!(waypoints[0][3], limit);
    let mut nan = LOW;
    nan[3] = f64::NAN;
    for start in [near, nan] {
        let beyond = PlanRequest {
            start_tolerance: [0.001; JOINTS],
            ..request(Planner::RrtConnect, start, &goal, &sampler)
        };
        let refused = plan_arm_path(&arm, &scene, &beyond);
        assert!(
            matches!(
                &refused,
                Err(PlanFailure::Start(StartFault::OutsideLimits { joint, .. }))
                    if joint == "openarm_left_joint4"
            ),
            "{refused:?}"
        );
    }
}

// A start whose grasp point sits inside a box is refused, the message naming the link
// that touches the map: no motion may start from it.
#[test]
fn a_start_inside_an_obstacle_is_refused() {
    let start_hand = hand(&LOW);
    let (map, field) = world(&solid(
        start_hand - Vector3::repeat(0.05),
        start_hand + Vector3::repeat(0.05),
    ));
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let refused = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    );
    let Err(PlanFailure::Start(StartFault::InCollision(Contact::Map { body }))) = refused else {
        panic!("a start inside the box gives {refused:?}");
    };
    assert!(
        LEFT_ARM.links().iter().any(|l| l.name() == body),
        "{body} is not a link of the arm"
    );
}

// A request whose clearance or start tolerance is no distance is refused, by name.
#[test]
fn a_request_without_distances_is_refused() {
    let (map, field) = world(&[]);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let base = request(Planner::RrtConnect, LOW, &goal, &sampler);
    let mut tolerance = [0.0; JOINTS];
    tolerance[2] = -0.1;
    for (bad, says) in [
        (
            PlanRequest {
                min_clearance_m: f64::NAN,
                ..request(Planner::RrtConnect, LOW, &goal, &sampler)
            },
            "min_clearance_m is NaN; it is a finite distance of 0 or more",
        ),
        (
            PlanRequest {
                start_tolerance: tolerance,
                ..request(Planner::RrtConnect, LOW, &goal, &sampler)
            },
            "a start tolerance is -0.1; it is a finite distance of 0 or more",
        ),
    ] {
        assert_eq!(
            plan_arm_path(&arm, &scene, &bad),
            Err(PlanFailure::Request(says.to_string()))
        );
    }
    assert!(plan_arm_path(&arm, &scene, &base).is_ok());
}

// From a start a centimeter from the slab, the hand a centimeter from a ball of its own
// body, the planned path departs from both: until it keeps both clearances, neither
// its distance from the map nor the hand's from the ball comes nearer than at the start
// by more than a step of the departure.
#[test]
fn from_a_start_near_the_slab_and_its_own_body_both_clearances_rise() {
    let NearTheSlab {
        map, field, start, ..
    } = near_the_slab(0.01);
    let aside = (hand(&HIGH) - hand(&start))
        .cross(&Vector3::z())
        .try_normalize(1e-6)
        .expect("the way to HIGH is not vertical");
    let model = HandAndBall {
        center: hand(&start) + aside * 0.06,
        radius: 0.05,
    };
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    use motion_planning_core::SelfModel;
    let bare = Arm::new(&LEFT_ARM, None);
    let from_map = |q: &[f64; JOINTS]| distance_to_map(&bare, &map, q);
    let (map_start, own_start) = (from_map(&start), model.clearance(&start));
    assert!(
        own_start < MIN_CLEARANCE_M,
        "the hand starts {own_start} from the ball"
    );
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, start, &goal, &sampler),
    )
    .expect("a path away from the slab and the ball");
    let path = RoundedPath::joint_path(&vectors(&waypoints));
    let spacing = PositiveFinite::parse(DEPARTURE_PROBE_M / arm.lever()).expect("positive");
    let kept = probes(&path, spacing).into_iter().any(|probe| {
        let q: [f64; JOINTS] = probe.as_slice().try_into().expect("a configuration");
        let (on_map, own) = (from_map(&q), model.clearance(&q));
        assert!(
            on_map >= departed(map_start) && own >= departed(own_start),
            "at {q:?} the map lies {on_map} away and the ball {own}, from {map_start} and {own_start}"
        );
        on_map >= MIN_CLEARANCE_M && own >= MIN_CLEARANCE_M
    });
    assert!(kept, "the path never keeps both clearances");
}

/// How far the turn of joint 0 from `LOW` moves a point of the arm before the pair of
/// `ApartNearTheStart` stands clear.
const CLEAR_FROM_M: f64 = 0.12;

/// How a pair of the robot's own body stands along the turn of joint 0 from `LOW`, by how
/// far the turn has moved a point of the arm: `apart` meters apart up to 110.5 mm, `dip`
/// meters apart up to 111.5 mm, `past` meters apart up to `CLEAR_FROM_M`, and a meter
/// apart beyond.
#[derive(Clone, Copy)]
struct Stands {
    apart: f64,
    dip: f64,
    past: f64,
}

impl Stands {
    /// A pair that holds `apart` meters apart up to `CLEAR_FROM_M`.
    fn held(apart: f64) -> Self {
        Self {
            apart,
            dip: apart,
            past: apart,
        }
    }
}

/// A self model whose pair stands as `stands` says, the turn of joint 0 moving a point
/// of the arm `per_rad` meters a radian.
struct ApartNearTheStart {
    stands: Stands,
    per_rad: f64,
}

impl motion_planning_core::SelfModel<JOINTS> for ApartNearTheStart {
    fn clearance(&self, q: &[f64; JOINTS]) -> f64 {
        let moved = (q[0] - LOW[0]).abs() * self.per_rad;
        let Stands { apart, dip, past } = self.stands;
        if moved < 0.1105 {
            apart
        } else if moved <= 0.1115 {
            dip
        } else if moved < CLEAR_FROM_M {
            past
        } else {
            1.0
        }
    }

    fn nearest_pair(&self, _: &[f64; JOINTS]) -> (String, String) {
        ("a dipping link".to_string(), "its neighbor".to_string())
    }
}

/// The plan from `LOW` to `HIGH` around no map, of an arm whose own body stands as
/// `stands` says, in at most 200 iterations. Every state but the start that the turn of
/// joint 0 has moved less than `CLEAR_FROM_M` falls short of the plan's clearance, so the
/// path's first run crosses each stretch.
fn plan_from_a_pair(stands: Stands) -> Result<Vec<[f64; JOINTS]>, PlanFailure> {
    let (map, field) = world(&[]);
    let model = ApartNearTheStart {
        stands,
        per_rad: Arm::new(&LEFT_ARM, None).reach()[0],
    };
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    plan_arm_path(
        &arm,
        &scene,
        &PlanRequest {
            max_iterations: NonZeroU64::new(200).expect("nonzero"),
            ..request(Planner::RrtConnect, LOW, &goal, &sampler)
        },
    )
}

// A start whose own body falls short of the plan's clearance from it, `MIN_CLEARANCE_M`
// plus 2 mm, by a millimeter departs along a run that holds that distance: a shortfall
// that does not grow.
#[test]
fn a_start_short_of_its_own_clearance_departs_holding_its_distance() {
    assert!(plan_from_a_pair(Stands::held(MIN_CLEARANCE_M + 1e-3)).is_ok());
}

// A pair of its own body 0.3 mm apart, below `MIN_CLEARANCE_M`, lets a departure step a
// quarter of that, under the 0.1 mm floor of a step: every run from the start ends at
// it, and the plan finds no path. At 0.5 mm apart the step is 0.125 mm, and the path
// departs.
#[test]
fn a_departure_ends_where_its_step_falls_under_its_floor() {
    assert!(matches!(
        plan_from_a_pair(Stands::held(3e-4)),
        Err(PlanFailure::NoPath(_))
    ));
    assert!(plan_from_a_pair(Stands::held(5e-4)).is_ok());
}

/// Asserts that a departure from a pair `apart` meters apart finds the pair `dip` meters
/// apart in the 1 mm stretch from 110.5 mm to 111.5 mm along the turn of joint 0, so the
/// plan finds no path, and that it finds one where the pair holds its distance there.
fn assert_departs_into_the_dip(apart: f64, dip: f64) {
    assert!(matches!(
        plan_from_a_pair(Stands {
            apart,
            dip,
            past: apart
        }),
        Err(PlanFailure::NoPath(_))
    ));
    assert!(plan_from_a_pair(Stands::held(apart)).is_ok());
}

// A departure from a pair 4 mm apart, below `MIN_CLEARANCE_M`, steps a quarter of that,
// 1 mm, and checks a state inside the stretch from 110.5 mm to 111.5 mm where the pair
// comes nearer. Steps of 2 mm along a turn of joint 0 alone check 110 mm and 112 mm,
// and steps of `MAX_STEP_M` 105 mm and 120 mm: both pass over it.
#[test]
fn a_departure_below_the_clearance_steps_a_quarter_of_its_distance() {
    assert_departs_into_the_dip(0.004, 0.002);
}

// A departure from a pair 1.6 mm above `MIN_CLEARANCE_M`, short of the plan's clearance
// of its own body by 0.4 mm, steps a quarter of that room, 0.4 mm, and checks a state
// inside the stretch where the pair comes nearer. Steps of a quarter of its whole
// distance, 5.4 mm, along a turn of joint 0 alone check 108 mm and 113.4 mm, and pass
// over it.
#[test]
fn a_departure_above_the_clearance_steps_a_quarter_of_its_room() {
    assert_departs_into_the_dip(MIN_CLEARANCE_M + 0.0016, MIN_CLEARANCE_M + 0.0012);
}

// A pair 1 cm apart, below `MIN_CLEARANCE_M`, that stands 0.1 mm above `MIN_CLEARANCE_M`
// past 111.5 mm: the departure rises through `MIN_CLEARANCE_M` and steps on by a quarter
// of its distance above the floor 0 its start set, some 5 mm. A step of a quarter of its
// room above `MIN_CLEARANCE_M`, 0.025 mm, falls under the 0.1 mm floor of a step.
#[test]
fn a_departure_rising_through_the_minimum_clearance_steps_on() {
    assert!(
        plan_from_a_pair(Stands {
            apart: 0.01,
            dip: 0.01,
            past: MIN_CLEARANCE_M + 1e-4,
        })
        .is_ok()
    );
}

/// The plan from a start `start_distance_m` from the slab to `HIGH`, in at most 200
/// iterations.
fn plan_from_near_the_slab(start_distance_m: f64) -> Result<Vec<[f64; JOINTS]>, PlanFailure> {
    let NearTheSlab {
        map, field, start, ..
    } = near_the_slab(start_distance_m);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    plan_arm_path(
        &arm,
        &scene,
        &PlanRequest {
            max_iterations: NonZeroU64::new(200).expect("nonzero"),
            ..request(Planner::RrtConnect, start, &goal, &sampler)
        },
    )
}

// A start 0.1 mm above `MIN_CLEARANCE_M`, from the slab or between a pair of its own
// body, departs: the floor it sets lies below it by twice the term's moving bodies times
// the 0.1 mm shortest step, so a step of either term is that step.
#[test]
fn a_start_just_above_the_minimum_clearance_departs() {
    assert!(plan_from_near_the_slab(MIN_CLEARANCE_M + 1e-4).is_ok());
    assert!(plan_from_a_pair(Stands::held(MIN_CLEARANCE_M + 1e-4)).is_ok());
}

/// Asserts that the plan of `planner` from seed `seed`, from a start `distance` meters
/// from the slab to `HIGH`, answers a path whose rounded path keeps the clearance once it
/// first keeps it.
fn assert_answers_from_near_the_slab(distance: f64, planner: Planner, seed: u32) {
    let NearTheSlab {
        map, field, start, ..
    } = near_the_slab(distance);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let answer = plan_arm_path(
        &arm,
        &scene,
        &PlanRequest {
            seed,
            ..request(planner, start, &goal, &sampler)
        },
    );
    let Ok(waypoints) = answer else {
        panic!("{planner} from {distance} m, seed {seed}: {answer:?}");
    };
    assert_dense_clear_after_departure(&arm, &map, &field, &waypoints);
}

// From starts 1 cm and 0.5 mm from the slab, the plans of RRTConnect and BiTRRT from
// each of eight seeds answer a path: every state a planner keeps but the start keeps the
// plan's clearance, so no planned waypoint falls short of it.
#[test]
fn from_a_start_near_the_slab_every_seed_answers_a_path() {
    for distance in [0.01, 0.0005] {
        for planner in [Planner::RrtConnect, Planner::BiTrrt] {
            for seed in 1..=8 {
                assert_answers_from_near_the_slab(distance, planner, seed);
            }
        }
    }
}

// From a start half a millimeter from the slab, BiTRRT's plan from seed 23 simplifies its
// path to one whose check fails: the plan answers the planned path, which passes.
#[test]
fn a_plan_whose_simplified_path_fails_its_check_answers_the_planned_path() {
    assert_answers_from_near_the_slab(0.0005, Planner::BiTrrt, 23);
}

// A start 0.15 mm from the slab steps half that, 0.075 mm, under the 0.1 mm floor of a
// step: every run from it ends at it, and the plan finds no path. From 0.3 mm the step
// is 0.15 mm, and the path departs. The map stands still, so a step of the map's term is
// half its room, twice a step of the robot's own body's.
#[test]
fn a_departure_from_the_map_steps_half_its_distance() {
    assert!(matches!(
        plan_from_near_the_slab(0.00015),
        Err(PlanFailure::NoPath(_))
    ));
    assert!(plan_from_near_the_slab(0.0003).is_ok());
}

// A start whose finger holds an occupied voxel, the voxel 2 cm or more from every link of
// the bare arm, is refused, the message naming the finger.
#[test]
fn a_start_whose_finger_touches_the_map_is_refused_naming_it() {
    let mut map = map(0.02, 1.0);
    let (voxel, away) = voxel_in_a_finger(&map, &LOW);
    assert!(
        away >= MIN_CLEARANCE_M,
        "the bare arm lies {away} m from the voxel"
    );
    occupied_at(&mut map, [0.0, 0.0, 3.0], &[voxel]);
    let field = DistanceField::of(&map);
    let riders = left_fingers();
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the fingers ride the hand");
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let refused = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    );
    let Err(PlanFailure::Start(StartFault::InCollision(Contact::Map { body }))) = refused else {
        panic!("a start whose finger holds a voxel gives {refused:?}");
    };
    assert!(
        riders.iter().any(|rider| rider.name == body),
        "{body} is not a finger"
    );
}

/// A goal sampler that sets `stop` once it has drawn, as a caller's stop that comes while
/// the plan runs.
struct StoppingOnceDrawn<'a> {
    sampler: IkGoalSampler<JOINTS>,
    stop: &'a AtomicBool,
}

impl GoalSampler<JOINTS> for StoppingOnceDrawn<'_> {
    fn sample(
        &self,
        goal: &Goal,
        count: usize,
        seed: u64,
        deadline: Instant,
        stop: Option<&AtomicBool>,
    ) -> Vec<[f64; JOINTS]> {
        let drawn = self.sampler.sample(goal, count, seed, deadline, stop);
        self.stop.store(true, Ordering::Relaxed);
        drawn
    }
}

// A stop set before the plan, or once its goal sampling has drawn, ends it: the plan
// says it was stopped and gives no path. The same plan with its flag clear finds a path.
#[test]
fn a_stopped_plan_says_so() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let clear = AtomicBool::new(false);
    let unstopped = PlanRequest {
        stop: Some(&clear),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    assert!(plan_arm_path(&arm, &scene, &unstopped).is_ok());
    let set = AtomicBool::new(true);
    let stopped = PlanRequest {
        stop: Some(&set),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    assert_eq!(
        plan_arm_path(&arm, &scene, &stopped),
        Err(PlanFailure::Stopped)
    );
    let stop = AtomicBool::new(false);
    let stopping = StoppingOnceDrawn {
        sampler: IkGoalSampler::new(LEFT_ARM.chain().clone()),
        stop: &stop,
    };
    let stopped_while_sampling = PlanRequest {
        sampler: &stopping,
        stop: Some(&stop),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    assert_eq!(
        plan_arm_path(&arm, &scene, &stopped_while_sampling),
        Err(PlanFailure::Stopped)
    );
}

// A plan of three attempts around the box repeats path for path, reaches the goal, and
// keeps the clearance on its rounded path.
#[test]
fn a_plan_of_several_attempts_repeats_and_keeps_its_clearance() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let attempts = PlanRequest {
        attempts: NonZeroU32::new(3).expect("nonzero"),
        ..request(Planner::RrtConnect, LOW, &goal, &sampler)
    };
    let waypoints = plan_arm_path(&arm, &scene, &attempts).expect("a path");
    assert_eq!(
        plan_arm_path(&arm, &scene, &attempts).expect("a path"),
        waypoints,
        "three attempts repeat"
    );
    assert_eq!(waypoints.first(), Some(&LOW));
    assert!(goal.met(LEFT_ARM.chain(), waypoints.last().expect("a waypoint")));
    assert_dense_clear(&arm, &map, &field, &waypoints);
}

/// A self model that keeps its clearance only within 0.05 rad of `LOW` and of `HIGH` in
/// each joint, so no motion between them passes. Asked on a thread other than the one
/// that made it, an attempt's, it sets `stop`, and it counts the questions asked once
/// `stop` is set.
struct StopsTheAttempts<'a> {
    made_on: ThreadId,
    stop: &'a AtomicBool,
    asked_once_stopped: AtomicUsize,
}

impl motion_planning_core::SelfModel<JOINTS> for StopsTheAttempts<'_> {
    fn clearance(&self, q: &[f64; JOINTS]) -> f64 {
        if std::thread::current().id() != self.made_on {
            self.stop.store(true, Ordering::Relaxed);
        }
        if self.stop.load(Ordering::Relaxed) {
            self.asked_once_stopped.fetch_add(1, Ordering::Relaxed);
        }
        let near = |at: &[f64; JOINTS]| (0..JOINTS).all(|i| (q[i] - at[i]).abs() <= 0.05);
        if near(&LOW) || near(&HIGH) { 1.0 } else { -1.0 }
    }

    fn nearest_pair(&self, _: &[f64; JOINTS]) -> (String, String) {
        ("a link".to_string(), "a wall of its own".to_string())
    }
}

/// The plan from `LOW` to `HIGH` in `attempts` attempts of at most 2000 iterations each,
/// of the arm checked against a [`StopsTheAttempts`] that sets `stop`; the plan's answer,
/// and how many times the attempts asked the self model once the stop was set.
fn plan_with_a_stopping_self_model(
    attempts: u32,
) -> (Result<Vec<[f64; JOINTS]>, PlanFailure>, usize) {
    let (map, field) = world(&[]);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let stop = AtomicBool::new(false);
    let model = StopsTheAttempts {
        made_on: std::thread::current().id(),
        stop: &stop,
        asked_once_stopped: AtomicUsize::new(0),
    };
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    let answer = plan_arm_path(
        &arm,
        &scene,
        &PlanRequest {
            max_iterations: NonZeroU64::new(2000).expect("nonzero"),
            attempts: NonZeroU32::new(attempts).expect("nonzero"),
            stop: Some(&stop),
            ..request(Planner::RrtConnect, LOW, &goal, &sampler)
        },
    );
    (answer, model.asked_once_stopped.load(Ordering::Relaxed))
}

/// The questions the attempts of a plan may ask the self model once the stop is set,
/// each before its next question whether to stop: far fewer than one attempt of 2000
/// iterations asks.
const ASKED_ONCE_STOPPED: usize = 200;

// The caller's stop, set as an attempt of two first checks a state on its own thread,
// ends a plan that finds no path at the attempts' next question whether to stop: the
// plan says it was stopped, its attempts having asked the self model fewer than
// `ASKED_ONCE_STOPPED` times since. A plan of one attempt plans on the caller's thread,
// sets no stop, and its iteration bound ends it.
#[test]
fn a_stop_ends_the_attempts_at_their_next_question() {
    let (answer, asked) = plan_with_a_stopping_self_model(2);
    assert_eq!(answer, Err(PlanFailure::Stopped));
    assert!(
        asked < ASKED_ONCE_STOPPED,
        "the attempts asked the self model {asked} times once stopped"
    );
    let (answer, _) = plan_with_a_stopping_self_model(1);
    assert!(
        matches!(answer, Err(PlanFailure::NoPath(_))),
        "one attempt: {answer:?}"
    );
}

/// The check of the path from `start` through `waypoints` for `arm` around `scene`, at
/// the plan's clearance, with no start tolerance and a deadline an hour away.
fn check(
    arm: &Arm<JOINTS>,
    scene: &Scene,
    start: [f64; JOINTS],
    waypoints: &[[f64; JOINTS]],
) -> PathCheck {
    check_arm_path(
        arm,
        scene,
        &CheckRequest {
            start,
            start_tolerance: [0.0; JOINTS],
            waypoints,
            min_clearance_m: MIN_CLEARANCE_M,
            deadline: an_hour_from_now(),
        },
    )
    .expect("a check of a valid request answers")
}

/// The check of a path through `waypoints` that passes.
fn passing(waypoints: &[[f64; JOINTS]]) -> PathCheck {
    PathCheck {
        clear_waypoints: waypoints.len(),
        fault: None,
    }
}

/// The path around the box from `LOW` to `HIGH` that RRTConnect plans under seed 7.
fn around_the_box(arm: &Arm<JOINTS>, scene: &Scene) -> Vec<[f64; JOINTS]> {
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    plan_arm_path(
        arm,
        scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    )
    .expect("a path around the box")
}

/// Asserts that `check` finds a state short of the plan's clearance from the map and the
/// objects at `place`, leaving `passing` waypoints before it.
fn assert_falls_short(check: &PathCheck, place: Place, passing: usize) {
    let Some(PathFault::Clearance {
        place: found,
        distances,
        clearances,
    }) = check.fault
    else {
        panic!("{check:?} finds no state short of its clearance");
    };
    assert_eq!(
        (found, check.clear_waypoints),
        (place, passing),
        "{check:?}"
    );
    assert!(distances[0] < clearances[0], "{check:?}");
    let reason = check.fault.as_ref().expect("a fault").to_string();
    assert!(
        reason.starts_with(&format!("{place}, the arm comes "))
            && reason.ends_with(
                "m from the map and the objects, nearer than the plan's clearance, 0.0210 m"
            ),
        "{reason}"
    );
}

/// How much farther than `MIN_CLEARANCE_M` from the map a plan keeps its states, as its
/// messages name it: the 0.0210 m of the plan's clearance.
const PLAN_SPARE_M: f64 = 0.001;

// The path the plan answers around the box passes its check; the same path and then
// straight back to the start, through the box, passes up to its last waypoint and
// breaks on the run from it; and, around no box, with a waypoint past joint 4's lower
// limit added after it, passes up to its last waypoint and names the joint.
#[test]
fn a_check_answers_the_waypoints_before_the_first_fault() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let planned = around_the_box(&arm, &scene);
    let waypoints = &planned[1..];
    let count = waypoints.len();
    assert_eq!(check(&arm, &scene, LOW, waypoints), passing(waypoints));
    let back: Vec<[f64; JOINTS]> = waypoints.iter().copied().chain([LOW]).collect();
    assert_falls_short(&check(&arm, &scene, LOW, &back), Place::Run(count), count);
    let limit = LEFT_ARM.chain().limits()[3].lo;
    let mut outside = waypoints[count - 1];
    outside[3] = limit - 0.01;
    let past: Vec<[f64; JOINTS]> = waypoints.iter().copied().chain([outside]).collect();
    let (open, open_field) = world(&[]);
    let open_scene = Scene {
        map: &open,
        field: &open_field,
        stamp: 1,
        objects: Objects::none(),
    };
    let checked = check(&arm, &open_scene, LOW, &past);
    assert_eq!(checked.clear_waypoints, count);
    assert_eq!(
        checked.fault,
        Some(PathFault::OutsideLimits {
            waypoint: count + 1,
            joint: "openarm_left_joint4".to_string(),
            position: limit - 0.01,
            low: limit,
            high: LEFT_ARM.chain().limits()[3].hi,
        })
    );
}

/// A self model whose pair stands a meter apart at every configuration, which refuses, by
/// a panic, to be asked about more than 10 000 states.
struct AskedAtMostTenThousand(AtomicUsize);

impl motion_planning_core::SelfModel<JOINTS> for AskedAtMostTenThousand {
    fn clearance(&self, _: &[f64; JOINTS]) -> f64 {
        let asked = self.0.fetch_add(1, Ordering::Relaxed);
        assert!(asked < 10_000, "asked about more than 10 000 states");
        1.0
    }

    fn nearest_pair(&self, _: &[f64; JOINTS]) -> (String, String) {
        ("a link".to_string(), "its neighbor".to_string())
    }
}

// A waypoint at 1e12 rad on joint 7 answers that it lies outside the joint's limits:
// the walk of the run to it ends where the run leaves them, within 10 000 states of the
// arm.
#[test]
fn a_waypoint_far_outside_a_limit_answers_promptly() {
    let (map, field) = world(&[]);
    let model = AskedAtMostTenThousand(AtomicUsize::new(0));
    let arm = Arm::new(&LEFT_ARM, Some(&model));
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let mut far = LOW;
    far[6] = 1e12;
    let checked = check(&arm, &scene, LOW, &[far]);
    assert_eq!(checked.clear_waypoints, 0);
    assert!(
        matches!(
            &checked.fault,
            Some(PathFault::OutsideLimits { waypoint: 1, joint, position, .. })
                if joint == "openarm_left_joint7" && *position == 1e12
        ),
        "{checked:?}"
    );
}

// A start inside the box fails with its contact, a start past a joint's limit by more
// than its tolerance with the joint, each passing no waypoint, and a start within its
// tolerance of the limit moves onto it and passes.
#[test]
fn a_check_answers_the_faults_of_its_start() {
    let start_hand = hand(&LOW);
    let (map, field) = world(&solid(
        start_hand - Vector3::repeat(0.05),
        start_hand + Vector3::repeat(0.05),
    ));
    let arm = Arm::new(&LEFT_ARM, None);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let checked = check(&arm, &scene, LOW, &[HIGH]);
    assert_eq!(checked.clear_waypoints, 0);
    assert!(
        matches!(
            checked.fault,
            Some(PathFault::Start(StartFault::InCollision(
                Contact::Map { .. }
            )))
        ),
        "{checked:?}"
    );
    let (empty, empty_field) = world(&[]);
    let clear = Scene {
        map: &empty,
        field: &empty_field,
        stamp: 1,
        objects: Objects::none(),
    };
    let limit = LEFT_ARM.chain().limits()[3].lo;
    let mut near = HIGH;
    near[3] = limit - 0.005;
    let at_tolerance = |tolerance: f64| {
        check_arm_path(
            &arm,
            &clear,
            &CheckRequest {
                start: near,
                start_tolerance: [tolerance; JOINTS],
                waypoints: &[HIGH],
                min_clearance_m: MIN_CLEARANCE_M,
                deadline: an_hour_from_now(),
            },
        )
        .expect("a valid request")
    };
    assert_eq!(at_tolerance(0.01), passing(&[HIGH]));
    let beyond = at_tolerance(0.001);
    assert_eq!(beyond.clear_waypoints, 0);
    assert!(
        matches!(
            &beyond.fault,
            Some(PathFault::Start(StartFault::OutsideLimits { joint, .. }))
                if joint == "openarm_left_joint4"
        ),
        "{beyond:?}"
    );
}

// From a start a centimeter from the slab, the path the plan answers passes its check,
// and so does a path whose run from the start rises from the slab; a path whose first
// waypoint lies nearer the slab than the start breaks the start rule on the run from
// the start, and one whose first run rises yet ends short of the plan's clearance
// passes the run and fails at its waypoint; each passes no waypoint.
#[test]
fn a_check_walks_the_run_from_its_start_by_the_start_rule() {
    let NearTheSlab {
        map, field, start, ..
    } = near_the_slab(0.01);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let planned = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, start, &goal, &sampler),
    )
    .expect("a path away from the slab");
    assert_eq!(
        check(&arm, &scene, start, &planned[1..]),
        passing(&planned[1..])
    );
    // Along the line from `LOW` through the start, the slab lies past the start.
    let along =
        |t: f64| -> [f64; JOINTS] { std::array::from_fn(|i| LOW[i] + (start[i] - LOW[i]) * t) };
    let distance = |q: &[f64; JOINTS]| distance_to_map(&arm, &map, q);
    let toward_the_slab = along(1.02);
    assert!(distance(&toward_the_slab) < distance(&start));
    assert_falls_short(
        &check(&arm, &scene, start, &[toward_the_slab]),
        Place::Run(0),
        0,
    );
    let plan_clearance = MIN_CLEARANCE_M + PLAN_SPARE_M;
    let rising_short = (1..100)
        .map(|k| along(1.0 - f64::from(k) * 0.01))
        .find(|q| {
            let d = distance(q);
            d > distance(&start) + 0.002 && d < plan_clearance
        })
        .expect("a state on the line, rising from the start, short of the plan's clearance");
    assert_falls_short(
        &check(&arm, &scene, start, &[rising_short]),
        Place::Waypoint(1),
        0,
    );
}

// A check refuses a request that holds no waypoint, a clearance that is not a number, or
// a negative start tolerance, and one whose deadline has passed ends before its walks.
#[test]
fn a_check_refuses_a_request_out_of_range_and_ends_by_its_deadline() {
    let (map, field) = world(&box_between());
    let arm = Arm::new(&LEFT_ARM, None);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
        objects: Objects::none(),
    };
    let base = CheckRequest {
        start: LOW,
        start_tolerance: [0.0; JOINTS],
        waypoints: &[HIGH],
        min_clearance_m: MIN_CLEARANCE_M,
        deadline: an_hour_from_now(),
    };
    for refused in [
        CheckRequest {
            waypoints: &[],
            ..base
        },
        CheckRequest {
            min_clearance_m: f64::NAN,
            ..base
        },
        CheckRequest {
            start_tolerance: [-0.01; JOINTS],
            ..base
        },
    ] {
        assert!(
            matches!(
                check_arm_path(&arm, &scene, &refused),
                Err(CheckFailure::Request(_))
            ),
            "a request out of range"
        );
    }
    let late = CheckRequest {
        deadline: Instant::now(),
        ..base
    };
    assert_eq!(
        check_arm_path(&arm, &scene, &late),
        Err(CheckFailure::TimeEnded)
    );
}

/// The link of the left arm's grasp point, which carries what the arm holds.
const GRASP_LINK: &str = "openarm_left_tcp";
/// The gripper's links, which a held item may touch: the hand and its two fingers.
const GRIPPER: [&str; 3] = [
    "openarm_left_ee_base_link",
    "openarm_left_ee_link1",
    "openarm_left_ee_link2",
];
/// The side of the item the tests' gripper holds: 2 cm, which the closed fingers overlap.
const ITEM_M: f64 = 0.02;
/// The right arm's grasp link, which the scene of both arms names besides the left's.
const RIGHT_GRASP_LINK: &str = "openarm_right_tcp";

/// The scene of a plan on `map`, whose distances `field` holds, among `objects`.
fn among<'a>(map: &'a VoxelMap, field: &'a DistanceField, objects: Objects<'a>) -> Scene<'a> {
    Scene {
        map,
        field,
        stamp: 1,
        objects,
    }
}

/// The planning scene of the left arm's links and fingers and of the grasp links
/// `grasp_links`, the left arm's among them.
fn scene_of_grasp_links(grasp_links: &[&str]) -> PlanningScene {
    let links = LEFT_ARM
        .links()
        .iter()
        .map(|link| link.name())
        .chain(grasp_links.iter().copied())
        .chain(GRIPPER)
        .map(String::from);
    PlanningScene::new(RobotLinks::new(
        links,
        grasp_links.iter().map(|link| link.to_string()),
    ))
}

/// The grasp point of the left arm at `q`.
fn grasp_pose(q: &[f64; JOINTS]) -> Isometry3<f64> {
    LEFT_ARM.chain().world_ee_pose(q)
}

/// A shape of `kind` and `dimensions` centered `position` from its object's origin,
/// turned as its object.
fn primitive(kind: &str, dimensions: &[f64], position: [f64; 3]) -> ShapeFields {
    ShapeFields {
        kind: kind.to_string(),
        dimensions: dimensions.to_vec(),
        mesh_vertices: Vec::new(),
        mesh_triangles: Vec::new(),
        position,
        orientation: [0.0, 0.0, 0.0, 1.0],
    }
}

/// An object of `shapes`, at `pose`, as apply_collision_objects adds it.
fn add_object(id: &str, pose: &Isometry3<f64>, shapes: Vec<ShapeFields>) -> ObjectRequest {
    let (position, orientation) = pose_parts(pose);
    ObjectRequest {
        operation: "add".to_string(),
        object: ObjectFields {
            id: id.to_string(),
            position,
            orientation,
            shapes,
        },
    }
}

/// An object of one shape at `pose`.
fn add_shape(id: &str, pose: &Isometry3<f64>, kind: &str, dimensions: &[f64]) -> ObjectRequest {
    add_object(id, pose, vec![primitive(kind, dimensions, [0.0; 3])])
}

/// The removal of the object `id`.
fn remove_object(id: &str) -> ObjectRequest {
    ObjectRequest {
        operation: "remove".to_string(),
        ..add_object(id, &Isometry3::identity(), Vec::new())
    }
}

/// The left arm's scene after `requests`, applied to `map`.
fn scene_with(map: &mut VoxelMap, requests: &[ObjectRequest]) -> PlanningScene {
    let mut scene = scene_of_grasp_links(&[GRASP_LINK]);
    scene
        .apply_collision_objects(requests, map)
        .expect("the objects apply");
    scene
}

/// `scene` with the object `id` attached to the grasp link `link` standing at
/// `link_pose`, touching `touch_links`.
fn attached(
    scene: &PlanningScene,
    id: &str,
    link: &str,
    link_pose: Isometry3<f64>,
    touch_links: &[&str],
) -> PlanningScene {
    let mut held = scene.clone();
    held.apply_attached_collision_object(
        &AttachRequest {
            object_id: id.to_string(),
            link_name: link.to_string(),
            touch_links: touch_links.iter().map(|link| link.to_string()).collect(),
            operation: "attach".to_string(),
        },
        |_| Some(link_pose),
        &mut map(VOXEL_M, 1.6),
    )
    .expect("attached");
    held
}

/// `scene` with the object "item" attached to the left grasp point where the arm stands
/// at `q`, touching `touch_links`.
fn holding(scene: &PlanningScene, q: &[f64; JOINTS], touch_links: &[&str]) -> PlanningScene {
    attached(scene, "item", GRASP_LINK, grasp_pose(q), touch_links)
}

/// The scene as a plan of the left arm meets it, the left arm its only arm.
fn left_objects(scene: &PlanningScene) -> ArmObjects<'_> {
    scene.for_arm(GRASP_LINK, |link| {
        panic!("the left arm is the only arm, and {link} another's grasp link")
    })
}

/// The riders of the left arm: `fingers`, and the shapes of the objects of `objects` it
/// carries.
fn riders<'a>(objects: &ArmObjects<'a>, fingers: &[Rider<'a>]) -> Vec<Rider<'a>> {
    fingers
        .iter()
        .copied()
        .chain(objects.riders(&LEFT_ARM))
        .collect()
}

/// The box `box_between` fills with voxels, as an object.
fn box_object() -> ObjectRequest {
    add_shape(
        "box",
        &Isometry3::translation(0.375, 0.20, 0.36),
        "box",
        &[0.15, 0.20, 0.12],
    )
}

/// `LOW` with the shoulder turned by 0.05 rad the way that raises the grasp point.
fn raised() -> [f64; JOINTS] {
    [-0.05, 0.05]
        .map(|turn| {
            let mut q = LOW;
            q[1] += turn;
            q
        })
        .into_iter()
        .max_by(|a, b| {
            let height = |q: &[f64; JOINTS]| grasp_pose(q).translation.vector.z;
            height(a).total_cmp(&height(b))
        })
        .expect("two turns")
}

/// Whether `check` breaks on the run from the start, a state of it short of the plan's
/// clearance of the term `term`, 0 for the map and the objects, 1 for the robot's own body
/// and `HELD` for an attached object against the arm that carries it, passing no waypoint.
fn breaks_first_run_on(check: &PathCheck, term: usize) -> bool {
    check.clear_waypoints == 0
        && matches!(
            check.fault,
            Some(PathFault::Clearance { place: Place::Run(0), distances, clearances })
                if distances[term] < clearances[term]
        )
}

// A box added between the start and the goal is avoided, as the box of voxels is: the
// straight path into it fails on its run, and the path around it keeps the clearance under
// a dense check. Once it is removed, the straight path through its place passes its check,
// and the plan goes through its place.
#[test]
fn a_box_is_avoided_until_it_is_removed() {
    let mut voxels = map(VOXEL_M, 1.6);
    let with_box = scene_with(&mut voxels, &[box_object()]);
    let field = DistanceField::of(&voxels);
    let arm = Arm::new(&LEFT_ARM, None);
    let boxed = left_objects(&with_box);
    let near_the_box = Surroundings::new(&voxels, &field, term_clearances(MIN_CLEARANCE_M))
        .with_objects(boxed.objects(&[]));
    let straight = check(
        &arm,
        &among(&voxels, &field, boxed.objects(&[])),
        LOW,
        &[HIGH],
    );
    assert!(breaks_first_run_on(&straight, 0), "{straight:?}");
    let around = around_the_box(&arm, &among(&voxels, &field, boxed.objects(&[])));
    assert_dense_clear_of(&arm, &near_the_box, &around);
    let mut emptied = with_box.clone();
    emptied
        .apply_collision_objects(&[remove_object("box")], &mut voxels.clone())
        .expect("removed");
    let empty = left_objects(&emptied);
    assert_eq!(
        check(
            &arm,
            &among(&voxels, &field, empty.objects(&[])),
            LOW,
            &[HIGH]
        ),
        passing(&[HIGH])
    );
    let through = around_the_box(&arm, &among(&voxels, &field, empty.objects(&[])));
    assert!(
        dense_states(&arm, &through)
            .iter()
            .any(|q| !near_the_box.clear(&arm, q)),
        "the plan keeps clear of where the box stood"
    );
}

// A goal inside an object is unreachable, and the message names, of the configuration
// that falls short least, the body of the arm nearest the object and the object.
#[test]
fn a_goal_inside_an_object_names_the_object() {
    let goal_hand = hand(&HIGH);
    let mut voxels = map(VOXEL_M, 1.6);
    let around_the_goal = Isometry3::translation(goal_hand.x, goal_hand.y, goal_hand.z);
    let with_box = scene_with(
        &mut voxels,
        &[add_shape("box", &around_the_goal, "box", &[0.2; 3])],
    );
    let field = DistanceField::of(&voxels);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let boxed = left_objects(&with_box);
    let failure = plan_arm_path(
        &arm,
        &among(&voxels, &field, boxed.objects(&[])),
        &request(Planner::RrtConnect, LOW, &at(&HIGH), &sampler),
    )
    .expect_err("no path into the box");
    let PlanFailure::Unreachable(message) = failure else {
        panic!("not unreachable: {failure}");
    };
    let (_, short) = message
        .split_once("; the one that falls short least has ")
        .expect("the message names what the nearest configuration falls short against");
    assert!(short.ends_with(" m from box"), "{message}");
}

// An item attached to the grasp point is carried around the box of voxels: with the
// gripper as its touch links, the fingers closed on it are no collision, and the path
// keeps the item's clearance from the map under a dense check. Without touch links, the
// start is in contact, the item and a link of the gripper.
#[test]
fn an_attached_item_is_carried_around_the_box() {
    let mut voxels = map(VOXEL_M, 1.6);
    occupied_at(&mut voxels, [0.0, 0.0, 3.0], &box_between());
    let item = add_shape("item", &grasp_pose(&LOW), "box", &[ITEM_M; 3]);
    let scene = scene_with(&mut voxels, &[item]);
    let field = DistanceField::of(&voxels);
    let fingers = left_fingers();
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let plan = |held: &PlanningScene| {
        let objects = left_objects(held);
        let riders = riders(&objects, &fingers);
        let arm = Arm::new(&LEFT_ARM, None)
            .riding(&riders)
            .expect("the fingers and the item ride the hand");
        let scene = among(&voxels, &field, objects.objects(&[]));
        let planned = plan_arm_path(
            &arm,
            &scene,
            &request(Planner::RrtConnect, LOW, &goal, &sampler),
        );
        if let Ok(waypoints) = &planned {
            let surroundings = Surroundings::new(&voxels, &field, term_clearances(MIN_CLEARANCE_M))
                .with_objects(objects.objects(&[]));
            assert_dense_clear_of(&arm, &surroundings, waypoints);
        }
        planned
    };
    let refused = plan(&holding(&scene, &LOW, &[]));
    assert!(
        matches!(
            &refused,
            Err(PlanFailure::Start(StartFault::InCollision(Contact::Own { a, b })))
                if a == "item" && GRIPPER.contains(&b.as_str())
        ),
        "an item touching the gripper with no touch links gives {refused:?}"
    );
    let carried = plan(&holding(&scene, &LOW, &GRIPPER)).expect("the item carried around the box");
    assert!(goal.met(LEFT_ARM.chain(), carried.last().expect("a waypoint")));
}

/// The grasp point's frame at `q` moved `ahead_m` along the way from the hand's origin to
/// the grasp point: past every link.
fn ahead_of_the_grasp_point(q: &[f64; JOINTS], ahead_m: f64) -> Isometry3<f64> {
    let hand = LEFT_ARM.chain().at(q).link_pose_world(JOINTS - 1);
    let grasp = grasp_pose(q);
    let out = (grasp.translation.vector - hand.translation.vector).normalize();
    Isometry3::from_parts(
        Translation3::from(grasp.translation.vector + out * ahead_m),
        grasp.rotation,
    )
}

// A 4 cm item held 15 cm past the grasp point, a 5 mm box object 1.75 cm past its face,
// while the bare arm keeps the plan's clearance from the object: the start falls short of
// the plan's clearance by the item alone, departs by the start rule, and the planned path
// passes its check whole.
#[test]
fn a_start_whose_held_item_comes_near_an_object_departs() {
    let ahead = ahead_of_the_grasp_point(&LOW, 0.15);
    let mut voxels = map(VOXEL_M, 1.6);
    let planning = scene_with(
        &mut voxels,
        &[
            add_shape("item", &ahead, "box", &[0.04; 3]),
            add_shape(
                "pebble",
                &(ahead * Isometry3::translation(0.04, 0.0, 0.0)),
                "box",
                &[0.005; 3],
            ),
        ],
    );
    let field = DistanceField::of(&voxels);
    let held = holding(&planning, &LOW, &[]);
    let objects = left_objects(&held);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the item rides the hand");
    let plan_clearance =
        Surroundings::new(&voxels, &field, [MIN_CLEARANCE_M + PLAN_SPARE_M, 0.0, 0.0])
            .with_objects(objects.objects(&[]));
    assert!(
        !plan_clearance.clear(&arm, &LOW),
        "the item keeps the plan's clearance from the pebble"
    );
    assert!(
        plan_clearance.clear(&Arm::new(&LEFT_ARM, None), &LOW),
        "the bare arm comes within the plan's clearance of the pebble"
    );
    let scene = among(&voxels, &field, objects.objects(&[]));
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    )
    .expect("a path away from the pebble");
    assert_eq!(
        check(&arm, &scene, LOW, &waypoints[1..]),
        passing(&waypoints[1..])
    );
}

/// The plan's clearance between an attached object and the arm that carries it: the 0
/// `term_clearances` asks of it and the 2 mm of spare its two moving bodies keep.
const HELD_PLAN_CLEARANCE_M: f64 = 0.002;
/// The place of that term in an array of the terms.
const HELD: usize = 2;

/// `LOW` with the last joint turned to `angle`.
fn last_joint_at(angle: f64) -> [f64; JOINTS] {
    let mut q = LOW;
    q[JOINTS - 1] = angle;
    q
}

/// The scene of an empty map holding a 2 cm item, the gripper its touch links, at the
/// arm's configuration `q`, `apart` meters from the nearest link of the arm it may not
/// touch: the item stands on the way from the grasp point to the last joint, at the share
/// of the way that bisection finds.
fn held_apart_from_its_arm(empty: &VoxelMap, q: &[f64; JOINTS], apart: f64) -> PlanningScene {
    let tcp = grasp_pose(q);
    let joint = LEFT_ARM.chain().at(q).origin_world(JOINTS - 1);
    let held_at = |t: f64| {
        let at = Isometry3::from_parts(
            Translation3::from(tcp.translation.vector + (joint - tcp.translation.vector) * t),
            tcp.rotation,
        );
        let scene = scene_with(
            &mut empty.clone(),
            &[add_shape("item", &at, "box", &[ITEM_M; 3])],
        );
        attached(&scene, "item", GRASP_LINK, tcp, &GRIPPER)
    };
    let (mut far, mut near) = (0.0, 1.0);
    assert!(from_its_arm(&held_at(far), q) > apart);
    assert!(from_its_arm(&held_at(near), q) < apart);
    while near - far > 1e-9 {
        let mid = (far + near) / 2.0;
        if from_its_arm(&held_at(mid), q) > apart {
            far = mid;
        } else {
            near = mid;
        }
    }
    held_at(far)
}

/// The least distance at `q` between the item `held` attaches to the left arm and the
/// pieces of the arm's links it may not touch.
fn from_its_arm(held: &PlanningScene, q: &[f64; JOINTS]) -> f64 {
    let objects = left_objects(held);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the item rides the hand");
    held_from_its_arm(&arm, q, |_, link| GRIPPER.contains(&link))
}

/// What `with` gives of the left arm carrying what `held` attaches to it, among the
/// objects of `held` on `empty`, whose distances `field` holds.
fn holding_in<R>(
    held: &PlanningScene,
    empty: &VoxelMap,
    field: &DistanceField,
    with: impl FnOnce(&Arm<JOINTS>, &Scene) -> R,
) -> R {
    let objects = left_objects(held);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the item rides the hand");
    with(&arm, &among(empty, field, objects.objects(&[])))
}

// An item held between the grasp point and the last joint, the gripper touching it, a
// centimeter from the nearest link of the arm it may not touch, nearer than
// `MIN_CLEARANCE_M`: an attached object is asked only to keep out of contact with the arm
// that carries it, so the start keeps every clearance and the turn of the last joint
// to 0 passes its check. Held 2 mm into that link, the start is in contact, the item and
// the link.
#[test]
fn a_deep_grasp_near_its_own_arm_keeps_out_of_contact_alone() {
    let (start, away) = (last_joint_at(0.1), last_joint_at(0.0));
    let empty = map(VOXEL_M, 1.6);
    let field = DistanceField::of(&empty);
    let deep = held_apart_from_its_arm(&empty, &start, 0.01);
    holding_in(&deep, &empty, &field, |arm, scene| {
        let surroundings = Surroundings::new(&empty, &field, term_clearances(MIN_CLEARANCE_M))
            .with_objects(scene.objects);
        assert!(surroundings.clear(arm, &start));
        assert_eq!(check(arm, scene, start, &[away]), passing(&[away]));
    });
    let into = held_apart_from_its_arm(&empty, &start, -0.002);
    holding_in(&into, &empty, &field, |arm, scene| {
        let checked = check(arm, scene, start, &[away]);
        assert!(
            matches!(
                &checked.fault,
                Some(PathFault::Start(StartFault::InCollision(Contact::Own { a, b })))
                    if a == "item" && !GRIPPER.contains(&b.as_str())
            ),
            "{checked:?}"
        );
    });
}

// The item held 0.5 mm from the nearest link of the arm it may not touch, the last joint
// turned 0.1 rad: the start falls short of the plan's clearance between an attached
// object and the arm that carries it, 2 mm. Turning the last joint back to 0, which carries
// the item away from the link, departs by the start rule on its run and stops at a
// waypoint still short of the clearance; turning it on to 0.2 rad, which carries the item
// nearer, breaks the rule on the run, on that term. An attached object is a term of the
// start rule as the self model is.
#[test]
fn a_start_whose_held_item_comes_near_its_own_arm_departs() {
    let (start, away, nearer) = (last_joint_at(0.1), last_joint_at(0.0), last_joint_at(0.2));
    let empty = map(VOXEL_M, 1.6);
    let field = DistanceField::of(&empty);
    let held = held_apart_from_its_arm(&empty, &start, 0.0005);
    let at_start = from_its_arm(&held, &start);
    assert!(
        from_its_arm(&held, &away) > at_start && from_its_arm(&held, &nearer) < at_start,
        "the turns do not carry the item away from and nearer its arm, from {at_start} m"
    );
    assert!(
        from_its_arm(&held, &away) < HELD_PLAN_CLEARANCE_M,
        "the turn away carries the item {} m from its arm, past the plan's clearance",
        from_its_arm(&held, &away)
    );
    holding_in(&held, &empty, &field, |arm, scene| {
        let departing = check(arm, scene, start, &[away]);
        assert!(
            departing.clear_waypoints == 0
                && matches!(
                    departing.fault,
                    Some(PathFault::Clearance { place: Place::Waypoint(1), distances, clearances })
                        if distances[HELD] > at_start && clearances[HELD] == HELD_PLAN_CLEARANCE_M
                ),
            "{departing:?}"
        );
        let toward = check(arm, scene, start, &[nearer]);
        assert!(breaks_first_run_on(&toward, HELD), "{toward:?}");
    });
}

/// The hull of a cube of side `side`.
fn cube_hull(side: f64) -> Hull {
    let h = side / 2.0;
    let corners: Vec<Point3<f64>> = (0..8)
        .map(|i| {
            let at = |bit: usize| if i & bit == 0 { -h } else { h };
            Point3::new(at(1), at(2), at(4))
        })
        .collect();
    Hull::new(&exact_hull(&corners).expect("a cube's hull"), 0.0).expect("a hull")
}

// A 4 cm item held 15 cm past the grasp point, and a 5 mm piece 1.75 cm past its face,
// of the other arm or of the torso, which stand still through the plan: the item keeps
// the plan's full clearance from each, as from the map and the objects, so the state falls
// short of it by the item alone, while the bare arm keeps it.
#[test]
fn a_held_item_keeps_the_full_clearance_from_the_other_arm_and_the_torso() {
    let ahead = ahead_of_the_grasp_point(&LOW, 0.15);
    let mut voxels = map(VOXEL_M, 1.6);
    let planning = scene_with(&mut voxels, &[add_shape("item", &ahead, "box", &[0.04; 3])]);
    let field = DistanceField::of(&voxels);
    let held = holding(&planning, &LOW, &[]);
    let objects = left_objects(&held);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the item rides the hand");
    let hull = cube_hull(0.005);
    let beside = ahead * Isometry3::translation(0.04, 0.0, 0.0);
    for link in ["openarm_right_link5", "openarm_body_link0"] {
        let parts = [RobotPart {
            link,
            piece: Placed::new(&hull, beside),
        }];
        let plan_clearance = Surroundings::new(
            &voxels,
            &field,
            term_clearances(MIN_CLEARANCE_M + PLAN_SPARE_M),
        )
        .with_objects(objects.objects(&parts));
        assert!(
            !plan_clearance.clear(&arm, &LOW),
            "the item keeps the plan's clearance from {link}"
        );
        assert!(
            plan_clearance.clear(&Arm::new(&LEFT_ARM, None), &LOW),
            "the bare arm comes within the plan's clearance of {link}"
        );
    }
}

// An item resting on the table touches it: the start is in contact until a pair lets them
// touch, and then a path from it passes its check. An item between the closed fingers
// touches them until a pair of the item and the gripper lets them close on it.
#[test]
fn pairs_let_the_item_rest_on_the_table_and_the_fingers_close_on_it() {
    let mut voxels = map(VOXEL_M, 1.6);
    let at = grasp_pose(&LOW);
    // A table top 4 cm square under the item, its top 5 mm above the item's lowest face, so
    // the item reaches into it.
    let top = at.translation.vector.z - ITEM_M / 2.0 + 0.005;
    let table = add_shape(
        "table",
        &Isometry3::translation(at.translation.vector.x, at.translation.vector.y, top - 0.01),
        "box",
        &[0.04, 0.04, 0.02],
    );
    let item = add_shape("item", &at, "box", &[ITEM_M; 3]);
    let mut scene = scene_with(&mut voxels, &[item, table]);
    let field = DistanceField::of(&voxels);
    let raised = raised();
    let check_holding = |held: &PlanningScene, fingers: &[Rider]| {
        let objects = left_objects(held);
        let riders = riders(&objects, fingers);
        let arm = Arm::new(&LEFT_ARM, None)
            .riding(&riders)
            .expect("the riders ride the hand");
        check(
            &arm,
            &among(&voxels, &field, objects.objects(&[])),
            LOW,
            &[raised],
        )
    };
    let touching = |checked: &PathCheck, body: &str, object: &str| {
        matches!(
            &checked.fault,
            Some(PathFault::Start(StartFault::InCollision(Contact::Object { body: b, object: o })))
                if b == body && o == object
        )
    };
    let on_table = holding(&scene, &LOW, &GRIPPER);
    let resting = check_holding(&on_table, &[]);
    assert!(touching(&resting, "item", "table"), "{resting:?}");
    let mut allowed = on_table.clone();
    allowed
        .set_allowed_collision("item", &["table".to_string()], true)
        .expect("allowed");
    assert_eq!(check_holding(&allowed, &[]), passing(&[raised]));

    scene
        .apply_collision_objects(&[remove_object("table")], &mut voxels.clone())
        .expect("removed");
    let fingers = left_fingers();
    let between = check_holding(&scene, &fingers);
    assert!(
        matches!(
            &between.fault,
            Some(PathFault::Start(StartFault::InCollision(Contact::Object { body, object })))
                if GRIPPER.contains(&body.as_str()) && object == "item"
        ),
        "{between:?}"
    );
    let gripper: Vec<String> = GRIPPER.iter().map(|link| link.to_string()).collect();
    scene
        .set_allowed_collision("item", &gripper, true)
        .expect("allowed");
    assert_eq!(check_holding(&scene, &fingers), passing(&[raised]));
}

/// Where the right arm's grasp link stands while the left arm plans.
fn right_hand() -> Isometry3<f64> {
    Isometry3::translation(0.3, -0.3, 0.3)
}

/// The scene of both arms after `requests`, applied to `map`, with the object `id`
/// attached to the right arm's grasp link where it stands, touching `touch_links`.
fn held_by_the_right_arm(
    map: &mut VoxelMap,
    requests: &[ObjectRequest],
    id: &str,
    touch_links: &[&str],
) -> PlanningScene {
    let mut scene = scene_of_grasp_links(&[GRASP_LINK, RIGHT_GRASP_LINK]);
    scene
        .apply_collision_objects(requests, map)
        .expect("the objects apply");
    attached(&scene, id, RIGHT_GRASP_LINK, right_hand(), touch_links)
}

/// The scene as a plan of the left arm meets it, the right arm's grasp link at
/// `right_hand`.
fn left_objects_beside_the_right(scene: &PlanningScene) -> ArmObjects<'_> {
    scene.for_arm(GRASP_LINK, |link| {
        assert_eq!(link, RIGHT_GRASP_LINK, "the other arm's grasp link");
        right_hand()
    })
}

// The right arm holds the box across the left arm's path: the left arm's straight path
// fails on its run where the right arm holds the box, and the path around it keeps the
// clearance under a dense check. The right arm holds an item between the left gripper's
// fingers: the left arm's start is in contact with it until the item's touch links are
// the left gripper's links, and a path from the start then passes its check.
#[test]
fn an_object_the_other_arm_holds_stands_where_it_holds_it() {
    let mut voxels = map(VOXEL_M, 1.6);
    let boxed = held_by_the_right_arm(&mut voxels, &[box_object()], "box", &[]);
    let field = DistanceField::of(&voxels);
    let objects = left_objects_beside_the_right(&boxed);
    let arm = Arm::new(&LEFT_ARM, None);
    let scene = among(&voxels, &field, objects.objects(&[]));
    let straight = check(&arm, &scene, LOW, &[HIGH]);
    assert!(breaks_first_run_on(&straight, 0), "{straight:?}");
    let around = around_the_box(&arm, &scene);
    let near_the_box = Surroundings::new(&voxels, &field, term_clearances(MIN_CLEARANCE_M))
        .with_objects(objects.objects(&[]));
    assert_dense_clear_of(&arm, &near_the_box, &around);

    let item = add_shape("item", &grasp_pose(&LOW), "box", &[ITEM_M; 3]);
    let fingers = left_fingers();
    let gripped = Arm::new(&LEFT_ARM, None)
        .riding(&fingers)
        .expect("the fingers ride the hand");
    let check_held = |touch_links: &[&str]| {
        let mut voxels = map(VOXEL_M, 1.6);
        let held = held_by_the_right_arm(
            &mut voxels,
            std::slice::from_ref(&item),
            "item",
            touch_links,
        );
        let field = DistanceField::of(&voxels);
        let objects = left_objects_beside_the_right(&held);
        check(
            &gripped,
            &among(&voxels, &field, objects.objects(&[])),
            LOW,
            &[raised()],
        )
    };
    let untouchable = check_held(&[]);
    assert!(
        matches!(
            &untouchable.fault,
            Some(PathFault::Start(StartFault::InCollision(Contact::Object { body, object })))
                if GRIPPER.contains(&body.as_str()) && object == "item"
        ),
        "{untouchable:?}"
    );
    assert_eq!(check_held(&GRIPPER), passing(&[raised()]));
}

/// A mesh of `vertices` and `triangles`, as apply_collision_objects takes one.
fn mesh(vertices: Vec<[f64; 3]>, triangles: Vec<[u32; 3]>) -> ShapeFields {
    ShapeFields {
        kind: "mesh".to_string(),
        dimensions: Vec::new(),
        mesh_vertices: vertices.into_iter().flatten().collect(),
        mesh_triangles: triangles.into_iter().flatten().collect(),
        position: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
    }
}

/// An L of two bars, each 20 cm long and 2 cm wide, meeting at the frame's origin along
/// x and y, from z = 0 to 10 cm: a closed solid of 20 triangles.
fn l_mesh() -> ShapeFields {
    let corners = [
        [0.0, 0.0],
        [0.2, 0.0],
        [0.2, 0.02],
        [0.02, 0.02],
        [0.02, 0.2],
        [0.0, 0.2],
    ];
    let vertices = [0.0, 0.1]
        .iter()
        .flat_map(|&z| corners.map(|[x, y]| [x, y, z]))
        .collect();
    let caps = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]]
        .into_iter()
        .flat_map(|[a, b, c]| [[a, c, b], [a + 6, b + 6, c + 6]]);
    let sides = (0..6).flat_map(|i| {
        let j = (i + 1) % 6;
        [[i, j, j + 6], [i, j + 6, i + 6]]
    });
    mesh(vertices, caps.chain(sides).collect())
}

/// A cube of `side` about the frame's origin as a closed mesh of 12 triangles.
fn cube_mesh(side: f64) -> ShapeFields {
    let half = side / 2.0;
    let vertices = (0..8)
        .map(|i| {
            let at = |bit: usize| if i & bit == 0 { -half } else { half };
            [at(1), at(2), at(4)]
        })
        .collect();
    let faces = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let triangles = faces
        .into_iter()
        .flat_map(|[a, b, c, d]| [[a, b, c], [a, c, d]])
        .collect();
    mesh(vertices, triangles)
}

// A 2 cm cube the hand holds 15 cm past the grasp point lies in the open corner of an
// L-shaped mesh, inside the convex hull of the L and 3 cm from each bar: the state keeps
// 5 mm from the L, where a box of the L's bounds holds the cube. Inside a closed cube
// mesh, clear of its triangles, the held cube is in collision.
#[test]
fn a_body_passes_the_corner_of_an_l_and_collides_inside_a_mesh() {
    let q = [0.0; JOINTS];
    let grasp = grasp_pose(&q);
    let hand = LEFT_ARM.chain().at(&q).link_pose_world(JOINTS - 1);
    let out = (grasp.translation.vector - hand.translation.vector).normalize();
    let center = grasp.translation.vector + out * 0.15;
    let turned = UnitQuaternion::rotation_between(&Vector3::z(), &out).unwrap_or_else(|| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    });
    let at_cube = Isometry3::from_parts(Translation3::from(center), turned);
    let voxels = map(VOXEL_M, 1.6);
    let field = DistanceField::of(&voxels);
    let holding_the_cube = |requests: &[ObjectRequest]| {
        let cube = add_shape("cube", &at_cube, "box", &[0.02; 3]);
        let scene = scene_with(&mut voxels.clone(), &[requests, &[cube]].concat());
        attached(&scene, "cube", GRASP_LINK, grasp, &GRIPPER)
    };
    let keeps = |requests: &[ObjectRequest]| {
        let scene = holding_the_cube(requests);
        let objects = left_objects(&scene);
        let riders = riders(&objects, &[]);
        let arm = Arm::new(&LEFT_ARM, None)
            .riding(&riders)
            .expect("the cube rides the hand");
        Surroundings::new(&voxels, &field, term_clearances(0.005))
            .with_objects(objects.objects(&[]))
            .clear(&arm, &q)
    };
    let corner = at_cube * Isometry3::translation(-0.04, -0.04, -0.05);
    assert!(keeps(&[add_object("corner", &corner, vec![l_mesh()])]));
    let bounds = at_cube * Isometry3::translation(0.06, 0.06, 0.0);
    assert!(!keeps(&[add_shape(
        "bounds",
        &bounds,
        "box",
        &[0.2, 0.2, 0.1]
    )]));
    let scene = holding_the_cube(&[add_object("crate", &at_cube, vec![cube_mesh(0.1)])]);
    let objects = left_objects(&scene);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the cube rides the hand");
    let checked = check(
        &arm,
        &among(&voxels, &field, objects.objects(&[])),
        q,
        &[raised()],
    );
    assert!(
        matches!(
            &checked.fault,
            Some(PathFault::Start(StartFault::InCollision(Contact::Object { body, object })))
                if body == "cube" && object == "crate"
        ),
        "{checked:?}"
    );
}

// The slab as an object of a box at each of its voxels, on an empty map: from the start a
// centimeter from it, a path that moves nearer it breaks the start rule on the map and the
// objects, while the planned path away from it passes whole. An object is a term of the
// start rule as the map is.
#[test]
fn a_path_from_a_start_near_an_object_must_move_away() {
    let NearTheSlab {
        map: slab,
        start,
        toward,
        ..
    } = near_the_slab(0.01);
    let side = slab.voxel_size();
    let boxes = slab
        .occupied()
        .map(|key| primitive("box", &[side; 3], slab.center(key)))
        .collect();
    let mut voxels = map(VOXEL_M, 1.6);
    let planning = scene_with(
        &mut voxels,
        &[add_object("slab", &Isometry3::identity(), boxes)],
    );
    let field = DistanceField::of(&voxels);
    let objects = left_objects(&planning);
    let arm = Arm::new(&LEFT_ARM, None);
    assert!(
        !Surroundings::new(&voxels, &field, [MIN_CLEARANCE_M + PLAN_SPARE_M, 0.0, 0.0])
            .with_objects(objects.objects(&[]))
            .clear(&arm, &start),
        "the start keeps the plan's clearance from the slab"
    );
    let scene = among(&voxels, &field, objects.objects(&[]));
    assert!(breaks_first_run_on(
        &check(&arm, &scene, start, &[toward]),
        0
    ));
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let waypoints = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, start, &goal, &sampler),
    )
    .expect("a path away from the slab");
    assert_eq!(
        check(&arm, &scene, start, &waypoints[1..]),
        passing(&waypoints[1..])
    );
}

// A 4 cm item held 15 cm past the grasp point, a voxel the map marks inside it after the
// attach, from three frames: the start is in contact, the item and the map, until a pair
// of the item and "occupancy_map" lets it touch the map, and then a path from it passes
// its check.
#[test]
fn a_pair_with_the_map_lets_a_held_item_touch_it() {
    let ahead = ahead_of_the_grasp_point(&LOW, 0.15);
    let mut voxels = map(0.01, 1.6);
    let item = add_shape("item", &ahead, "box", &[0.04; 3]);
    let held = holding(&scene_with(&mut voxels, &[item]), &LOW, &[]);
    // The add left the voxel at the lowest log-odds, which three frames' hits raise past
    // the occupancy threshold.
    let inside: [f64; 3] = ahead.translation.vector.into();
    for _ in 0..3 {
        occupied_at(&mut voxels, [0.0, 0.0, 3.0], &[inside]);
    }
    assert!(voxels.is_occupied(voxels.key(inside).expect("a point inside the map")));
    let field = DistanceField::of(&voxels);
    let check_held = |held: &PlanningScene| {
        let objects = left_objects(held);
        let riders = riders(&objects, &[]);
        let arm = Arm::new(&LEFT_ARM, None)
            .riding(&riders)
            .expect("the item rides the hand");
        check(
            &arm,
            &among(&voxels, &field, objects.objects(&[])),
            LOW,
            &[raised()],
        )
    };
    let touching = check_held(&held);
    assert!(
        matches!(
            &touching.fault,
            Some(PathFault::Start(StartFault::InCollision(Contact::Map { body }))) if body == "item"
        ),
        "{touching:?}"
    );
    let mut allowed = held.clone();
    allowed
        .set_allowed_collision("item", &["occupancy_map".to_string()], true)
        .expect("allowed");
    assert_eq!(check_held(&allowed), passing(&[raised()]));
}

// Two items the hand holds 15 cm past the grasp point, the second overlapping the first,
// each with the gripper as its touch links: objects one arm carries are never checked
// against each other, so the start is clear and a path from it passes its check, with no
// pair that lets them touch.
#[test]
fn two_held_items_are_never_checked_against_each_other() {
    let ahead = ahead_of_the_grasp_point(&LOW, 0.15);
    let mut voxels = map(VOXEL_M, 1.6);
    let first = add_shape("item", &ahead, "box", &[0.04; 3]);
    let second = add_shape(
        "second",
        &(ahead * Isometry3::translation(0.03, 0.0, 0.0)),
        "box",
        &[0.04; 3],
    );
    let both = scene_with(&mut voxels, &[first, second]);
    let held = attached(
        &holding(&both, &LOW, &GRIPPER),
        "second",
        GRASP_LINK,
        grasp_pose(&LOW),
        &GRIPPER,
    );
    let field = DistanceField::of(&voxels);
    let objects = left_objects(&held);
    let riders = riders(&objects, &[]);
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the items ride the hand");
    assert_eq!(
        check(
            &arm,
            &among(&voxels, &field, objects.objects(&[])),
            LOW,
            &[raised()],
        ),
        passing(&[raised()])
    );
}
