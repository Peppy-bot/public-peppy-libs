//! Plans of OpenArm v2's left arm around boxes of voxels: the rounded path of every
//! returned path keeps its clearance under a dense check and passes the check of a
//! caller's path, the same request on the same map gives the same path, of one attempt
//! or several, a stop ends a plan, and a goal or a world that admits no path says why.
//! Checks of a caller's path: where each fault lies and the waypoints before it.

mod common;

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread::ThreadId;
use std::time::{Duration, Instant};

use control_core::path_timing::RoundedPath;
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, CheckFailure, CheckRequest, Contact, Goal, GoalSampler, IkGoalSampler, JointTarget,
    MAX_STEP_M, PathCheck, PathFault, Place, PlanFailure, PlanRequest, Scene, StartFault,
    Surroundings, check_arm_path, plan_arm_path,
};
use nalgebra::Vector3;
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::Planner;

use common::{
    JOINTS, LEFT_ARM, an_hour_from_now, distance_to_map, left_fingers, map, occupied_at,
    voxel_in_a_finger,
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
            above: 0.01,
            below: 0.01,
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

/// Asserts that the rounded path through `waypoints` keeps `MIN_CLEARANCE_M` at every
/// state of a dense check: no point of a link moves a tenth of a voxel between two.
fn assert_dense_clear(
    arm: &Arm<JOINTS>,
    map: &VoxelMap,
    field: &DistanceField,
    waypoints: &[[f64; JOINTS]],
) {
    let path = RoundedPath::joint_path(&vectors(waypoints));
    let spacing = PositiveFinite::parse(VOXEL_M / 10.0 / arm.lever()).expect("positive");
    let surroundings = Surroundings::new(map, field, MIN_CLEARANCE_M, MIN_CLEARANCE_M);
    for probe in probes(&path, spacing) {
        let q: [f64; JOINTS] = probe.as_slice().try_into().expect("a configuration");
        assert!(
            surroundings.clear(arm, &q),
            "the rounded path touches the map at {q:?}"
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
    };
    let straight = Surroundings::new(&map, &field, MIN_CLEARANCE_M, MIN_CLEARANCE_M);
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
// meeting it is in collision, nearer than the plan's clearance it names.
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
    assert!(
        message.ends_with(
            "that meet the goal is in collision, nearer than the plan's clearance, 0.0210 m from the map"
        ),
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
             plan's clearance, 0.0210 m from the map"
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

/// A slab under the arm, and a start `start_distance_m` from it.
struct NearTheSlab {
    map: VoxelMap,
    field: DistanceField,
    start: [f64; JOINTS],
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
    NearTheSlab { map, field, start }
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
    let surroundings = Surroundings::new(map, field, MIN_CLEARANCE_M, MIN_CLEARANCE_M);
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
    let surroundings = Surroundings::new(&map, &field, MIN_CLEARANCE_M, MIN_CLEARANCE_M);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
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
    };
    let refused = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    );
    let Err(PlanFailure::Start(StartFault::InCollision(Contact::Map { link }))) = refused else {
        panic!("a start inside the box gives {refused:?}");
    };
    assert!(
        LEFT_ARM.links().iter().any(|l| l.name() == link),
        "{link} is not a link of the arm"
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
    let NearTheSlab { map, field, start } = near_the_slab(distance);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
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
    };
    let refused = plan_arm_path(
        &arm,
        &scene,
        &request(Planner::RrtConnect, LOW, &goal, &sampler),
    );
    let Err(PlanFailure::Start(StartFault::InCollision(Contact::Map { link }))) = refused else {
        panic!("a start whose finger holds a voxel gives {refused:?}");
    };
    assert!(
        riders.iter().any(|rider| rider.link == link),
        "{link} is not a finger"
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

/// Asserts that `check` finds a state short of the plan's clearance from the map at
/// `place`, leaving `passing` waypoints before it.
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
            && reason.ends_with("m from the map, nearer than the plan's clearance, 0.0210 m"),
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
    let NearTheSlab { map, field, start } = near_the_slab(0.01);
    let arm = Arm::new(&LEFT_ARM, None);
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = at(&HIGH);
    let scene = Scene {
        map: &map,
        field: &field,
        stamp: 1,
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
