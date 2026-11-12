//! Plans that hold orientations along the path: OpenArm v2 carries a hand as it holds an
//! upright cup across its body, past a wall, free about the vertical or not, from a start near
//! a table and with an orientation of a link
//! the later joints do not move, one arm and both; every state of the rounded path holds
//! each orientation in any order of the list, and a start, a goal or a passage that no path
//! can hold fails with no path. A goal that names a position takes the path's orientation.

mod common;
mod openarm_v2;

use std::f64::consts::PI;
use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::time::{Duration, Instant};

use chain_kinematics::{Aims, ChainFrame, ChainSpec, Draws, Ends, IkSolver, JointSelection};
use collision_geometry::chain::ChainGeometry;
use control_core::path_timing::RoundedPath;
use motion_planning_core::{
    Arm, ArmGoal, Arms, Axis, Frame, Goal, GoalSampler, IkGoalSampler, JointTarget, Objects,
    OrientationTarget, PathOrientation, PlanFailure, PlanRequest, PositionTarget, Scene, plan_path,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::Planner;

use common::{
    LEFT_ARM, PLAN_SPARE_M, RIGHT_ARM, an_hour_from_now, distance_to_map, map, occupied_at,
    plan_arm, positive,
};
use openarm_v2::{JOINTS, LEFT, RIGHT};

const VOXEL_M: f64 = 0.04;
/// The half side of the map, in meters: past the arms' reach.
const MAP_HALF_M: f64 = 1.6;
const MIN_CLEARANCE_M: f64 = 0.02;
/// The joints of both arms.
const BOTH: usize = 2 * JOINTS;
/// The tolerance of the orientations held here, in radians.
const TOLERANCE_RAD: f64 = 0.1;
/// The farthest a frame turns between two states of a dense check of a rounded path, in
/// radians.
const PROBE_RAD: f64 = 1e-3;
/// The most goals each plan samples: the launch's value.
const GOAL_SAMPLES: usize = 10;
/// The configurations each `configuration_at` tries.
const IK_ATTEMPTS: usize = 64;
/// The tolerances of the configuration `configuration_at` finds.
const AT_POSE_M: f64 = 1e-3;
const AT_POSE_RAD: f64 = 1e-3;
/// In front of the left arm, on its own side of the body: where its hand starts.
const NEAR: [f64; 3] = [0.3, 0.25, 0.85];
/// Across the body, past the robot's middle: where its hand ends.
const FAR: [f64; 3] = [0.3, -0.1, 0.85];
/// A short move from `NEAR` up and toward the body, which any orientation test reaches.
const NEARBY: [f64; 3] = [0.3, 0.15, 0.9];
/// The farthest the hand moves between two states of `walk_of_the_line`, in meters.
const LINE_STEP_M: f64 = 0.003;
/// The most a joint may change between two states of `walk_of_the_line`: a walk along one
/// branch of the arm, in radians.
const LINE_JOINT_STEP_RAD: f64 = 0.2;
/// The restarts of each step's search of `walk_of_the_line`.
const LINE_SEARCH_ITERATIONS: usize = 64;

/// The grasp point as it holds an upright cup by its side: its approach axis, +z of its frame,
/// forward along x, and its x axis straight up.
fn upright() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1.0, 0.0, 1.0)), PI)
}

fn target(
    frame: Frame,
    orientation: UnitQuaternion<f64>,
    tolerance_rad: f64,
    free_axis: Option<Axis>,
) -> OrientationTarget {
    OrientationTarget {
        frame,
        orientation,
        tolerance_rad: positive(tolerance_rad),
        free_axis,
    }
}

/// The grasp point held upright, free about the vertical or not.
fn cup_upright(free_axis: Option<Axis>) -> OrientationTarget {
    target(Frame::GraspPoint, upright(), TOLERANCE_RAD, free_axis)
}

/// `target` held by the left hand's grasp point.
fn of_the_left_hand(target: OrientationTarget) -> [PathOrientation<'static>; 1] {
    [PathOrientation {
        arm: 0,
        link_name: &openarm_v2::arm(LEFT).grasp_link,
        target,
    }]
}

/// A goal of the grasp point's position and orientation.
fn pose_goal(position: [f64; 3], orientation: &OrientationTarget) -> Goal {
    Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            position: position.into(),
            tolerance_m: positive(AT_POSE_M),
        }],
        vec![*orientation],
    )
    .expect("a valid goal")
}

/// A goal of the grasp point's position alone.
fn position_goal(position: [f64; 3]) -> Goal {
    Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            position: position.into(),
            tolerance_m: positive(AT_POSE_M),
        }],
        Vec::new(),
    )
    .expect("a valid goal")
}

/// A configuration of `geometry`'s arm whose grasp point stands at `position` with
/// `orientation`.
fn configuration_at(
    geometry: &ChainGeometry<JOINTS>,
    position: [f64; 3],
    orientation: UnitQuaternion<f64>,
) -> [f64; JOINTS] {
    found_at(geometry, position, orientation)
        .unwrap_or_else(|| panic!("the arm reaches {position:?}"))
}

/// A configuration of `geometry`'s arm whose grasp point stands at `position` with
/// `orientation`, where inverse kinematics finds one.
fn found_at(
    geometry: &ChainGeometry<JOINTS>,
    position: [f64; 3],
    orientation: UnitQuaternion<f64>,
) -> Option<[f64; JOINTS]> {
    let goal = pose_goal(
        position,
        &target(Frame::GraspPoint, orientation, AT_POSE_RAD, None),
    );
    IkGoalSampler::new(geometry.chain().clone()).sample_from(
        &goal,
        &[0.0; JOINTS],
        IK_ATTEMPTS,
        &mut Draws::new(1),
        &|_| true,
    )
}

/// The configuration at the end of a dense walk by inverse kinematics of the grasp point in a
/// straight line from where `start` holds it to `end`, turned as `start` holds it, each step
/// on the branch of the one before: the proof that a path from `start` keeps that orientation
/// the whole way. None where the walk leaves the branch or the arm's reach.
fn walk_of_the_line(start: &[f64; JOINTS], end: [f64; 3]) -> Option<[f64; JOINTS]> {
    let chain = LEFT_ARM.chain();
    let ik = IkSolver::new(chain, ChainFrame::EndEffector, Aims::Both, Ends::Converged);
    let pose = chain.world_ee_pose(start);
    let line = Vector3::from(end) - pose.translation.vector;
    let steps = (line.norm() / LINE_STEP_M).ceil() as usize;
    (1..=steps).try_fold(*start, |at, k| {
        let position = pose.translation.vector + line * (k as f64 / steps as f64);
        let near =
            |q: &[f64; JOINTS]| (0..JOINTS).all(|i| (q[i] - at[i]).abs() <= LINE_JOINT_STEP_RAD);
        ik.search(
            at,
            &Isometry3::from_parts(Translation3::from(position), pose.rotation),
            &mut Draws::new(k as u64),
            &near,
            LINE_SEARCH_ITERATIONS,
        )
    })
}

/// The map of a solid box from `min` to `max` of voxels, and its distance field.
fn world(boxes: &[([f64; 3], [f64; 3])]) -> (VoxelMap, DistanceField) {
    let mut voxels = map(VOXEL_M, MAP_HALF_M);
    let points: Vec<[f64; 3]> = boxes
        .iter()
        .flat_map(|(min, max)| {
            let steps: [usize; 3] =
                std::array::from_fn(|i| ((max[i] - min[i]) / VOXEL_M).ceil() as usize);
            (0..=steps[0]).flat_map(move |i| {
                (0..=steps[1]).flat_map(move |j| {
                    (0..=steps[2]).map(move |k| {
                        [
                            (min[0] + i as f64 * VOXEL_M).min(max[0]),
                            (min[1] + j as f64 * VOXEL_M).min(max[1]),
                            (min[2] + k as f64 * VOXEL_M).min(max[2]),
                        ]
                    })
                })
            })
        })
        .collect();
    occupied_at(&mut voxels, [0.0, 0.0, 2.0], &points);
    let field = DistanceField::of(&voxels);
    (voxels, field)
}

/// A wall between `NEAR` and `FAR`, up to just under the hand's height.
const WALL: ([f64; 3], [f64; 3]) = ([0.2, 0.06, 0.4], [0.4, 0.1, 0.72]);

/// Plans `arms` from `start` around `boxes`, holding `orientations`, each arm to its goal
/// of `goals`.
fn plan_holding<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    boxes: &[([f64; 3], [f64; 3])],
    start: [f64; D],
    goals: &[&Goal],
    samplers: &[IkGoalSampler<N>],
    orientations: &[PathOrientation],
) -> Result<Vec<[f64; D]>, PlanFailure> {
    let goals = goals
        .iter()
        .zip(samplers)
        .map(|(goal, sampler)| ArmGoal { goal, sampler })
        .collect();
    plan_until(arms, boxes, start, goals, orientations, an_hour_from_now())
}

/// `plan_holding`, ended by `deadline`.
fn plan_until<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    boxes: &[([f64; 3], [f64; 3])],
    start: [f64; D],
    goals: Vec<ArmGoal<'_, N>>,
    orientations: &[PathOrientation],
    deadline: Instant,
) -> Result<Vec<[f64; D]>, PlanFailure> {
    let (voxels, field) = world(boxes);
    let answer = plan_path(
        arms,
        &Scene {
            map: &voxels,
            field: &field,
            stamp: 1,
            objects: Objects::none(),
        },
        &PlanRequest {
            start,
            start_tolerance: [0.0; D],
            goals,
            path_orientations: orientations,
            goal_samples: NonZeroUsize::new(GOAL_SAMPLES).expect("nonzero"),
            planner: Planner::RrtConnect,
            refiner: None,
            deadline,
            max_iterations: NonZeroU64::new(20_000).expect("nonzero"),
            attempts: NonZeroU32::MIN,
            seed: 7,
            min_clearance_m: MIN_CLEARANCE_M,
            stop: None,
        },
    );
    if let Ok(path) = &answer {
        let limits = arms.limits();
        assert!(
            path.iter()
                .flatten()
                .zip(limits.iter().cycle())
                .all(|(joint, limit)| (limit.lo..=limit.hi).contains(joint)),
            "a waypoint of the path stands outside a joint limit"
        );
    }
    answer
}

/// The left arm as a group of one, and its goal sampler.
fn left_arm() -> (Arms<'static, JOINTS, JOINTS>, [IkGoalSampler<JOINTS>; 1]) {
    (
        Arms::new(vec![plan_arm(LEFT)], None),
        [IkGoalSampler::new(LEFT_ARM.chain().clone())],
    )
}

/// The largest error of `target` of `geometry`'s arm over the rounded path through
/// `waypoints`, the arm at its joints from `first` in each.
fn largest_error<const D: usize>(
    waypoints: &[[f64; D]],
    first: usize,
    geometry: &ChainGeometry<JOINTS>,
    target: &OrientationTarget,
) -> f64 {
    let vectors: Vec<Vec<f64>> = waypoints.iter().map(|q| q.to_vec()).collect();
    let path = RoundedPath::joint_path(&vectors);
    let step = PROBE_RAD / (JOINTS as f64).sqrt();
    let count = (path.length() / step).ceil() as usize;
    (0..=count)
        .map(|k| {
            let q = path.at((k as f64 * step).min(path.length()));
            let joints: [f64; JOINTS] = std::array::from_fn(|j| q[first + j]);
            target.error_at(geometry.chain(), &joints)
        })
        .fold(0.0, f64::max)
}

/// The largest error of `target` along the straight joint motion from `from` to `to`.
fn largest_error_straight(
    from: &[f64; JOINTS],
    to: &[f64; JOINTS],
    target: &OrientationTarget,
) -> f64 {
    largest_error(&[*from, *to], 0, &LEFT_ARM, target)
}

// Held upright, free about the vertical or whole, the hand goes from one side of a wall to the
// other, and every state of the rounded path keeps the tolerance, where the straight joint
// motion between the same ends leaves it.
#[test]
fn every_state_of_the_rounded_path_holds_the_orientation() {
    let (arms, samplers) = left_arm();
    for free_axis in [Some(Axis::X), None] {
        let hand = cup_upright(free_axis);
        let start = configuration_at(&LEFT_ARM, NEAR, upright());
        let end = walk_of_the_line(&start, FAR).expect("the hand reaches across the body held");
        assert!(
            largest_error_straight(&start, &end, &hand) > TOLERANCE_RAD,
            "the straight joint motion holds the orientation: this scene needs no correction"
        );
        let goal = pose_goal(FAR, &hand);
        let orientations = of_the_left_hand(hand);
        let path = plan_holding(&arms, &[WALL], start, &[&goal], &samplers, &orientations)
            .unwrap_or_else(|failure| panic!("free about {free_axis:?}: {failure}"));
        let largest = largest_error(&path, 0, &LEFT_ARM, &hand);
        assert!(
            largest <= TOLERANCE_RAD,
            "free about {free_axis:?}: the path turns the hand {largest} rad"
        );
        let last = path.last().expect("a path");
        assert!(goal.met(LEFT_ARM.chain(), last));
    }
}

// A hand that starts in the spare above the clearance from a table departs from it by the
// start rule and holds its orientation at every state.
#[test]
fn a_hand_in_the_spare_above_a_tables_clearance_departs_holding_its_orientation() {
    /// The top of the table, on the voxels' grid: 18 voxels up.
    const TABLE_TOP_M: f64 = 18.0 * VOXEL_M;
    let (arms, samplers) = left_arm();
    let hand = cup_upright(Some(Axis::X));
    let table = (
        [0.2, 0.2, TABLE_TOP_M - 0.2],
        [0.4, 0.3, TABLE_TOP_M - VOXEL_M / 2.0],
    );
    let (voxels, _) = world(&[table]);
    let start = (0..1000)
        .find_map(|k| {
            let height = TABLE_TOP_M + f64::from(k) * 2e-4;
            let start = found_at(&LEFT_ARM, [NEAR[0], NEAR[1], height], upright())?;
            let distance = distance_to_map(&arms, &voxels, &start);
            (distance > MIN_CLEARANCE_M && distance < MIN_CLEARANCE_M + PLAN_SPARE_M)
                .then_some(start)
        })
        .expect("a height of the hand in the spare above the table's clearance");
    let goal = pose_goal(NEARBY, &hand);
    let orientations = of_the_left_hand(hand);
    let path = plan_holding(&arms, &[table], start, &[&goal], &samplers, &orientations)
        .unwrap_or_else(|failure| panic!("{failure}"));
    let largest = largest_error(&path, 0, &LEFT_ARM, &hand);
    assert!(largest <= TOLERANCE_RAD, "the hand turns {largest} rad");
}

// A start past the angle a corrected state allows, inside the tolerance, plans: the plan
// corrects its states to the start's error, and every state keeps the tolerance.
#[test]
fn a_start_in_the_tolerances_outer_share_plans() {
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let tilted =
        upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.95 * TOLERANCE_RAD);
    let start = configuration_at(&LEFT_ARM, NEAR, tilted);
    let goal = pose_goal(NEARBY, &hand);
    let orientations = of_the_left_hand(hand);
    let path = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations)
        .unwrap_or_else(|failure| panic!("{failure}"));
    let largest = largest_error(&path, 0, &LEFT_ARM, &hand);
    assert!(largest <= TOLERANCE_RAD, "the hand turns {largest} rad");
}

// A goal that names a position alone takes the orientation of the path's constraint on its
// frame, and ends inside the angle a corrected state allows, as MoveIt merges them.
#[test]
fn a_goal_of_a_position_takes_the_paths_orientation() {
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    let goal = position_goal(NEARBY);
    let orientations = of_the_left_hand(hand);
    let path = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations)
        .unwrap_or_else(|failure| panic!("{failure}"));
    let last = path.last().expect("a path");
    let error = hand.error_at(LEFT_ARM.chain(), last);
    assert!(
        error <= 0.9 * TOLERANCE_RAD,
        "the goal ends {error} rad from the path's orientation"
    );
}

// The order of the list carries no meaning: two orientations of one arm, of the grasp point and
// of a link the later joints do not move, hold along the path in either order. A plan of the hand
// alone turns the upper arm past the tolerance, so this plan holds a state the other leaves.
#[test]
fn the_orientations_hold_in_either_order() {
    /// The tolerance of the upper arm's orientation: the goal's nearest configuration, 0.18 rad
    /// from it, stands inside 0.9 of this.
    const UPPER_ARM_TOLERANCE_RAD: f64 = 0.22;
    let (arms, samplers) = left_arm();
    let hand = cup_upright(Some(Axis::X));
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    let posed = LEFT_ARM.chain().at(&start);
    let frame = Frame::Link {
        segment: 2,
        offset: Isometry3::identity(),
    };
    let upper_arm = target(
        frame,
        posed.link_pose_world(2).rotation,
        UPPER_ARM_TOLERANCE_RAD,
        None,
    );
    let goal = pose_goal(NEARBY, &hand);
    let [of_the_hand] = of_the_left_hand(hand);
    let of_the_upper_arm = PathOrientation {
        arm: 0,
        link_name: posed.link_name(2),
        target: upper_arm,
    };
    let hand_alone = plan_holding(&arms, &[], start, &[&goal], &samplers, &[of_the_hand])
        .unwrap_or_else(|failure| panic!("{failure}"));
    assert!(
        largest_error(&hand_alone, 0, &LEFT_ARM, &upper_arm) > UPPER_ARM_TOLERANCE_RAD,
        "the hand alone keeps the upper arm inside its tolerance: this scene needs no second row"
    );
    for orientations in [
        [of_the_hand, of_the_upper_arm],
        [of_the_upper_arm, of_the_hand],
    ] {
        let path = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations)
            .unwrap_or_else(|failure| panic!("{failure}"));
        for (target, tolerance) in [
            (&hand, TOLERANCE_RAD),
            (&upper_arm, UPPER_ARM_TOLERANCE_RAD),
        ] {
            let largest = largest_error(&path, 0, &LEFT_ARM, target);
            assert!(largest <= tolerance, "{largest} rad, past {tolerance}");
        }
    }
}

/// Both arms as one group.
fn both_arms() -> Arms<'static, JOINTS, BOTH> {
    Arms::new(vec![plan_arm(LEFT), plan_arm(RIGHT)], None)
}

/// `position` on the robot's other side.
fn mirror([x, y, z]: [f64; 3]) -> [f64; 3] {
    [x, -y, z]
}

/// Both arms standing with each hand upright at `NEAR`.
fn both_upright_at_near() -> [f64; BOTH] {
    Arms::<JOINTS, BOTH>::join(&[
        configuration_at(&LEFT_ARM, NEAR, upright()),
        configuration_at(&RIGHT_ARM, mirror(NEAR), upright()),
    ])
}

/// A plan of both arms, each hand upright from `NEAR` toward the robot's middle, its goal
/// `NEARBY`, holding `orientations`; the goal of the right hand with it.
fn plan_both_arms_holding(orientations: &[PathOrientation]) -> (Vec<[f64; BOTH]>, Goal) {
    let samplers = [
        IkGoalSampler::new(LEFT_ARM.chain().clone()),
        IkGoalSampler::new(RIGHT_ARM.chain().clone()),
    ];
    let hand = cup_upright(Some(Axis::X));
    let goals = [pose_goal(NEARBY, &hand), pose_goal(mirror(NEARBY), &hand)];
    let path = plan_holding(
        &both_arms(),
        &[],
        both_upright_at_near(),
        &[&goals[0], &goals[1]],
        &samplers,
        orientations,
    )
    .unwrap_or_else(|failure| panic!("{failure}"));
    let [_, right_goal] = goals;
    (path, right_goal)
}

// Both arms carry a hand held upright, each toward the robot's middle, and each holds its
// orientation at every state of the one path.
#[test]
fn each_of_two_arms_holds_its_orientation() {
    let hand = cup_upright(Some(Axis::X));
    let [of_the_left_hand] = of_the_left_hand(hand);
    let (path, _) = plan_both_arms_holding(&[
        of_the_left_hand,
        PathOrientation {
            arm: 1,
            link_name: &openarm_v2::arm(RIGHT).grasp_link,
            target: hand,
        },
    ]);
    for (first, geometry) in [(0, &*LEFT_ARM), (JOINTS, &*RIGHT_ARM)] {
        let largest = largest_error(&path, first, geometry, &hand);
        assert!(largest <= TOLERANCE_RAD, "an arm turns {largest} rad");
    }
}

// One arm of two holds an orientation: it holds it at every state, and the other arm reaches its
// goal in the same path.
#[test]
fn one_of_two_arms_holds_its_orientation() {
    let hand = cup_upright(Some(Axis::X));
    let (path, right_goal) = plan_both_arms_holding(&of_the_left_hand(hand));
    let largest = largest_error(&path, 0, &LEFT_ARM, &hand);
    assert!(largest <= TOLERANCE_RAD, "the left arm turns {largest} rad");
    let last = path.last().expect("a path");
    let right: [f64; JOINTS] = std::array::from_fn(|j| last[JOINTS + j]);
    assert!(right_goal.met(RIGHT_ARM.chain(), &right));
}

// A start past the tolerance fails with no path, and says what to send.
#[test]
fn a_start_that_breaks_an_orientation_fails_with_no_path() {
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let tilted =
        upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.0 * TOLERANCE_RAD);
    let start = configuration_at(&LEFT_ARM, NEAR, tilted);
    let goal = pose_goal(FAR, &hand);
    let orientations = of_the_left_hand(hand);
    let failure = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations);
    let Err(PlanFailure::NoPath(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(
        message
            .starts_with("the start holds openarm_left_tcp 0.2000 rad from its path orientation"),
        "{message}"
    );
}

// A goal of joints whose configuration breaks an orientation fails with no path, naming it.
#[test]
fn a_goal_of_joints_that_breaks_an_orientation_fails_with_no_path() {
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    let elsewhere = [0.4, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0];
    let goal = Goal::new(
        JOINTS,
        (0..JOINTS)
            .map(|joint| JointTarget {
                joint,
                position: elsewhere[joint],
                above: positive(1e-3),
                below: positive(1e-3),
            })
            .collect(),
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let orientations = of_the_left_hand(hand);
    let failure = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations);
    let Err(PlanFailure::NoPath(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(
        message.contains("each breaking a path orientation"),
        "{message}"
    );
}

/// The grasp point's goal at `NEARBY`, turned `angle` from upright, within `AT_POSE_RAD`.
fn goal_turned_by(angle: f64) -> Goal {
    pose_goal(
        NEARBY,
        &target(
            Frame::GraspPoint,
            upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle),
            AT_POSE_RAD,
            None,
        ),
    )
}

// A goal's orientation that lies past the angle the path allows a goal fails with no path at
// once, whatever time the plan has: no state meets both. It stands inside the path's tolerance
// or past it.
#[test]
fn a_goal_orientation_the_path_breaks_fails_with_no_path() {
    let (arms, samplers) = left_arm();
    let orientations = of_the_left_hand(cup_upright(None));
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    for (angle, named) in [(0.5, "0.5000"), (0.095, "0.0950")] {
        let goal = goal_turned_by(angle);
        let failure = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations);
        let Err(PlanFailure::NoPath(message)) = failure else {
            panic!("{failure:?}");
        };
        assert!(
            message.starts_with(&format!(
                "the goal's orientation of openarm_left_tcp stands {named} rad"
            )),
            "{message}"
        );
    }
}

// A goal's orientation inside the angle the path allows a goal plans, off the middle of the
// path's orientation.
#[test]
fn a_goal_orientation_inside_the_allowed_angle_plans() {
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let orientations = of_the_left_hand(hand);
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    let goal = goal_turned_by(0.085);
    let path = plan_holding(&arms, &[], start, &[&goal], &samplers, &orientations)
        .unwrap_or_else(|failure| panic!("{failure}"));
    let largest = largest_error(&path, 0, &LEFT_ARM, &hand);
    assert!(largest <= TOLERANCE_RAD, "the hand turns {largest} rad");
    assert!(goal.met(LEFT_ARM.chain(), path.last().expect("a path")));
}

/// The goal sampler of one configuration of the left arm: it finds the configuration where a
/// goal holds it and nothing else.
struct OnlyAt([f64; JOINTS]);

impl GoalSampler<JOINTS> for OnlyAt {
    fn sample_from(
        &self,
        goal: &Goal,
        _: &[f64; JOINTS],
        _: usize,
        _: &mut Draws,
        valid: &dyn Fn(&[f64; JOINTS]) -> bool,
    ) -> Option<[f64; JOINTS]> {
        (goal.met(LEFT_ARM.chain(), &self.0) && valid(&self.0)).then_some(self.0)
    }
}

/// The goal sampler of a goal whose draws meet it only where the goal holds the path's
/// orientation: `turned` meets a goal of the position alone, which it answers with nothing, and
/// a goal that holds the orientation takes `upright`.
struct OnlyWhenHeld {
    turned: [f64; JOINTS],
    upright: [f64; JOINTS],
}

impl GoalSampler<JOINTS> for OnlyWhenHeld {
    fn sample_from(
        &self,
        goal: &Goal,
        _: &[f64; JOINTS],
        _: usize,
        _: &mut Draws,
        valid: &dyn Fn(&[f64; JOINTS]) -> bool,
    ) -> Option<[f64; JOINTS]> {
        (!goal.met(LEFT_ARM.chain(), &self.turned) && valid(&self.upright)).then_some(self.upright)
    }
}

/// How a plan from `start` ends, holding the hand upright, to a goal of the position `position`
/// that only `sampler` samples.
fn plan_sampled_by(
    position: [f64; 3],
    sampler: &dyn GoalSampler<JOINTS>,
) -> Result<Vec<[f64; JOINTS]>, PlanFailure> {
    let (arms, _) = left_arm();
    let goal = position_goal(position);
    plan_until(
        &arms,
        &[],
        configuration_at(&LEFT_ARM, NEAR, upright()),
        vec![ArmGoal {
            goal: &goal,
            sampler,
        }],
        &of_the_left_hand(cup_upright(None)),
        an_hour_from_now(),
    )
}

// A position a configuration reaches only with the hand turned past the path's orientation
// fails with no path, naming the link, as soon as the goal sampling's samples end.
#[test]
fn a_position_only_a_turned_hand_reaches_fails_with_no_path() {
    let turned = upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3);
    let configuration = configuration_at(&LEFT_ARM, NEARBY, turned);
    let failure = plan_sampled_by(NEARBY, &OnlyAt(configuration));
    let Err(PlanFailure::NoPath(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(
        message.starts_with("the goal can be met")
            && message.contains("path orientations of openarm_left_tcp"),
        "{message}"
    );
}

// A position whose goal alone finds nothing plans where the goal that holds the orientation
// finds a state: the plan's own sampling answers it.
#[test]
fn a_position_only_the_held_goal_finds_plans() {
    let sampler = OnlyWhenHeld {
        turned: configuration_at(
            &LEFT_ARM,
            NEARBY,
            upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3),
        ),
        upright: configuration_at(&LEFT_ARM, NEARBY, upright()),
    };
    let path = plan_sampled_by(NEARBY, &sampler).unwrap_or_else(|failure| panic!("{failure}"));
    assert_eq!(path.last(), Some(&sampler.upright));
}

/// The goal sampler that finds nothing.
struct Nowhere;

impl<const N: usize> GoalSampler<N> for Nowhere {
    fn sample_from(
        &self,
        _: &Goal,
        _: &[f64; N],
        _: usize,
        _: &mut Draws,
        _: &dyn Fn(&[f64; N]) -> bool,
    ) -> Option<[f64; N]> {
        None
    }
}

// A plan of both arms names the arm no sample reaches: the left arm's goal alone has a state, and
// no sample of the right arm's has one.
#[test]
fn a_plan_of_both_arms_names_the_arm_no_sample_reaches() {
    let turned = upright() * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3);
    let left_sampler = OnlyAt(configuration_at(&LEFT_ARM, NEARBY, turned));
    let (left_goal, right_goal) = (position_goal(NEARBY), position_goal(mirror(NEARBY)));
    let failure = plan_until(
        &both_arms(),
        &[],
        both_upright_at_near(),
        vec![
            ArmGoal {
                goal: &left_goal,
                sampler: &left_sampler,
            },
            ArmGoal {
                goal: &right_goal,
                sampler: &Nowhere,
            },
        ],
        &of_the_left_hand(cup_upright(None)),
        an_hour_from_now(),
    );
    let Err(PlanFailure::Unreachable(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(message.contains("of right_arm"), "{message}");
}

// A position no sample reaches is unreachable once its samples are spent.
#[test]
fn a_position_no_sample_reaches_is_unreachable() {
    let configuration = configuration_at(&LEFT_ARM, NEARBY, upright());
    let failure = plan_sampled_by(FAR, &OnlyAt(configuration));
    let Err(PlanFailure::Unreachable(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(
        message.starts_with("inverse kinematics found no configuration"),
        "{message}"
    );
}

// A position no configuration reaches is unreachable by the plan's deadline, whatever the path
// holds: the samples of the goal alone find no state by then.
#[test]
fn a_position_out_of_reach_stays_unreachable() {
    /// The time the sampling takes before it ends: the outcome does not depend on it.
    const SAMPLING_TIME: Duration = Duration::from_secs(1);
    let (arms, samplers) = left_arm();
    let hand = cup_upright(None);
    let start = configuration_at(&LEFT_ARM, NEAR, upright());
    let goal = position_goal([1.5, 1.5, 1.5]);
    let orientations = of_the_left_hand(hand);
    let failure = plan_until(
        &arms,
        &[],
        start,
        vec![ArmGoal {
            goal: &goal,
            sampler: &samplers[0],
        }],
        &orientations,
        Instant::now() + SAMPLING_TIME,
    );
    assert!(
        matches!(failure, Err(PlanFailure::Unreachable(_))),
        "{failure:?}"
    );
}

/// A planar arm of two revolute joints about z, links of 30 cm and 25 cm along x, so its
/// second link's orientation is the sum of its joints.
fn planar_arm() -> ChainGeometry<2> {
    let urdf = r#"<robot name="planar">
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
          <limit lower="-2.5" upper="2.5" effort="1" velocity="1"/>
        </joint>
      </robot>"#;
    ChainGeometry::from_urdf(
        urdf,
        "",
        &ChainSpec {
            base_link: None,
            tip_link: "link_2",
            joints: JointSelection::PathOrder,
        },
    )
    .expect("a planar arm")
}

/// The second link of the planar arm held along x.
fn along_x() -> OrientationTarget {
    target(
        Frame::Link {
            segment: 1,
            offset: Isometry3::identity(),
        },
        UnitQuaternion::identity(),
        TOLERANCE_RAD,
        None,
    )
}

// Held along x, the planar arm's second link sweeps straight through y = 0 from one side to
// the other, where a box stands: the start and the goal hold the orientation and no path
// between them does, so the plan fails with no path, naming the link. The same plan holding
// nothing goes around the box.
#[test]
fn a_passage_no_path_can_hold_fails_with_no_path() {
    let geometry = planar_arm();
    let arms: Arms<2, 2> = Arms::new(vec![Arm::new("planar", &geometry)], None);
    let samplers = [IkGoalSampler::new(geometry.chain().clone())];
    let goal = Goal::new(
        2,
        [(0, 1.0), (1, -1.0)]
            .into_iter()
            .map(|(joint, position)| JointTarget {
                joint,
                position,
                above: positive(1e-3),
                below: positive(1e-3),
            })
            .collect(),
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let orientations = [PathOrientation {
        arm: 0,
        link_name: "link_2",
        target: along_x(),
    }];
    let in_the_way = ([0.44, -0.02, -0.02], [0.46, 0.02, 0.02]);
    plan_holding(&arms, &[in_the_way], [-1.0, 1.0], &[&goal], &samplers, &[])
        .unwrap_or_else(|failure| panic!("holding nothing: {failure}"));
    let failure = plan_holding(
        &arms,
        &[in_the_way],
        [-1.0, 1.0],
        &[&goal],
        &samplers,
        &orientations,
    );
    let Err(PlanFailure::NoPath(message)) = failure else {
        panic!("{failure:?}");
    };
    assert!(
        message.ends_with("and the path orientations of link_2"),
        "{message}"
    );
}

// A list as long as the state's joints is refused: OMPL needs fewer rows than dimensions.
#[test]
fn as_many_orientations_as_joints_are_refused() {
    let geometry = planar_arm();
    let arms: Arms<2, 2> = Arms::new(vec![Arm::new("planar", &geometry)], None);
    let samplers = [IkGoalSampler::new(geometry.chain().clone())];
    let goal = Goal::new(
        2,
        vec![JointTarget {
            joint: 0,
            position: 0.0,
            above: positive(0.1),
            below: positive(0.1),
        }],
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let held = PathOrientation {
        arm: 0,
        link_name: "link_2",
        target: along_x(),
    };
    assert_eq!(
        plan_holding(
            &arms,
            &[],
            [0.0, 0.0],
            &[&goal],
            &samplers,
            &[held, held],
        ),
        Err(PlanFailure::Request(
            "path_orientation_constraints holds 2 constraints, and a plan of 2 joints takes at most 1: send fewer"
                .to_string()
        ))
    );
}
