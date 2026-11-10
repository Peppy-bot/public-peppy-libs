//! An arm of five joints, SO-101's: a goal sample meets a position alone, and an orientation
//! within its tolerance where the arm's joints come that near it, a Cartesian path covers
//! the lines on which the joints hold the orientation, and a plan departs from the park pose
//! within the real arm's joint error.

use std::path::Path;
use std::sync::LazyLock;

use chain_kinematics::{ChainSpec, Draws, JointSelection};
use collision_geometry::chain::ChainGeometry;
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, Arms, Axis, CartesianRequest, Collisions, FollowedFrame, Frame, Goal, GoalSampler,
    IkGoalSampler, MIN_DEPARTURE_STEP_M, OrientationTarget, PositionTarget, cartesian_path,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use self_collision_model::so101;
use so101_description::{ARM_DOF, BASE_LINK, TCP_LINK, URDF, write_meshes_to};

/// The goals each count runs: the position of a configuration drawn inside the limits.
const GOALS: usize = 100;
/// The goals a sampler meets of them, at the attempts and the goal samples of a plan.
const MOST_GOALS: usize = 98;
/// The attempts of a goal sample: the plan module's `MAX_STATE_SAMPLING_ATTEMPTS`, MoveIt's
/// `max_state_sampling_attempts` default.
const ATTEMPTS: usize = 4;
/// The goal samples each goal runs: MoveIt's `max_goal_samples`.
const GOAL_SAMPLES: usize = 10;
/// How near its position a goal holds the frame: the 1 mm of the harness's pose goals.
const TOLERANCE_M: f64 = 0.001;
const SEED: u64 = 7;
/// A posture of the arm in its vertical plane, every joint inside its limits and off them.
const IN_ITS_PLANE: [f64; ARM_DOF] = [0.0, -0.4, 0.8, 0.6, 0.0];
/// How far a goal turns the frame about an axis it holds, past the tolerance.
const AXIS_TURN_RAD: f64 = 0.8;
/// The goals a sampler meets of them with an axis held: each aims at a turn about the axis
/// the arm may not hold, and takes the end whose axis keeps its tolerance.
const MOST_GOALS_AXIS_HELD: usize = 90;
/// How far the lines of the Cartesian paths run.
const LINE_M: f64 = 0.03;

/// The park pose (postures.py's `HOME_POSITIONS_RAD`): the arm folded onto itself as it
/// rests, unfolded on the shoulder lift, the elbow and the wrist.
const HOME: [f64; ARM_DOF] = [-0.05, -1.51, 1.47, 1.03, 0.32];
/// The real arm's largest joint error at a goal over A13's three runs on hardware.
const ARM_ERROR_RAD: f64 = 0.046;
/// The moving bodies of a pair of the robot's own body: its `k` in the plan module's start
/// rule.
const OWN_PAIR_MOVING_BODIES: f64 = 2.0;

/// SO-101's collision meshes, written once per test binary.
static MESHES: LazyLock<String> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("so101_meshes");
    write_meshes_to(&dir).expect("the meshes write");
    dir.to_str()
        .expect("the mesh directory is UTF-8")
        .to_string()
});

/// SO-101's arm with its collision pieces.
static ARM: LazyLock<ChainGeometry<ARM_DOF>> = LazyLock::new(|| {
    ChainGeometry::from_urdf(
        URDF,
        &MESHES,
        &ChainSpec {
            base_link: Some(BASE_LINK),
            tip_link: TCP_LINK,
            joints: JointSelection::PathOrder,
        },
    )
    .expect("SO-101's arm fits")
});

fn positive(value: f64) -> PositiveFinite {
    PositiveFinite::parse(value).expect("a positive value")
}

/// A unit axis drawn from `draws`.
fn drawn_axis(draws: &mut Draws) -> Unit<Vector3<f64>> {
    Unit::new_normalize(Vector3::from_fn(|_, _| draws.between(-1.0, 1.0)))
}

/// How many of `GOALS` goals a sampler meets, each the goal `goal_of` makes of the pose of a
/// drawn configuration, from a drawn working state.
fn goals_met(goal_of: impl Fn(&Isometry3<f64>, &mut Draws) -> Goal) -> usize {
    let chain = ARM.chain();
    let limits = chain.limits();
    let sampler = IkGoalSampler::new(chain.clone());
    let mut draws = Draws::new(SEED);
    (0..GOALS)
        .filter(|_| {
            let goal = goal_of(&chain.world_ee_pose(&draws.within(&limits)), &mut draws);
            let working = draws.within(&limits);
            (0..GOAL_SAMPLES).any(|_| {
                sampler
                    .sample_from(&goal, &working, ATTEMPTS, &mut draws, &|_| true)
                    .is_some_and(|q| goal.met(chain, &q))
            })
        })
        .count()
}

/// The goal of `pose`'s position, within `TOLERANCE_M`, and of `orientation`.
fn at_the_position(pose: &Isometry3<f64>, orientation: Option<OrientationTarget>) -> Goal {
    Goal::new(
        ARM_DOF,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            position: pose.translation.vector,
            tolerance_m: positive(TOLERANCE_M),
        }],
        orientation.into_iter().collect(),
    )
    .expect("a goal of one frame")
}

/// The orientation tilted by `tilt_rad` about a drawn axis, held within `tolerance_rad`.
fn tilted(
    orientation: UnitQuaternion<f64>,
    tilt_rad: f64,
    tolerance_rad: f64,
    draws: &mut Draws,
) -> OrientationTarget {
    OrientationTarget {
        frame: Frame::GraspPoint,
        orientation: orientation * UnitQuaternion::from_axis_angle(&drawn_axis(draws), tilt_rad),
        tolerance_rad: positive(tolerance_rad),
        free_axis: None,
    }
}

// A goal of a position alone is met wherever the arm reaches it.
#[test]
fn a_goal_of_a_position_alone_is_met() {
    assert_eq!(goals_met(|pose, _| at_the_position(pose, None)), GOALS);
}

// A goal that holds the orientation the drawn configuration has is met at a tight tolerance.
#[test]
fn an_orientation_the_arm_holds_is_met_at_a_tight_tolerance() {
    let met = goals_met(|pose, draws| {
        at_the_position(pose, Some(tilted(pose.rotation, 0.0, 0.02, draws)))
    });
    println!("held at 0.02 rad: {met} of {GOALS}");
    assert!(met >= MOST_GOALS, "{met} of {GOALS}");
}

// A goal whose orientation lies off every orientation the arm holds at the position, within
// the tolerance's reach of one it holds, is met.
#[test]
fn an_orientation_within_its_tolerance_of_one_the_arm_holds_is_met() {
    let met = goals_met(|pose, draws| {
        at_the_position(pose, Some(tilted(pose.rotation, 0.05, 0.1, draws)))
    });
    println!("tilted 0.05 within 0.1 rad: {met} of {GOALS}");
    assert!(met >= MOST_GOALS, "{met} of {GOALS}");
}

// A goal turned far about an axis of the frame the arm's other joints cannot turn it about, the
// axis held within its tolerance and the frame free to turn about it, is met; the same goal
// with no axis free is met seldom.
#[test]
fn an_axis_held_frees_a_turn_about_it_the_tolerance_alone_misses() {
    let turned = |pose: &Isometry3<f64>, free_axis| {
        at_the_position(
            pose,
            Some(OrientationTarget {
                frame: Frame::GraspPoint,
                orientation: pose.rotation
                    * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), AXIS_TURN_RAD),
                tolerance_rad: positive(0.1),
                free_axis,
            }),
        )
    };
    let held = goals_met(|pose, _| turned(pose, Some(Axis::X)));
    let not_held = goals_met(|pose, _| turned(pose, None));
    println!("turned {AXIS_TURN_RAD} rad: {held} with x held, {not_held} with no axis free");
    assert!(held >= MOST_GOALS_AXIS_HELD, "{held} of {GOALS}");
    assert!(not_held < held / 2, "{not_held} of {GOALS}");
}

// A goal of an orientation alone, one a drawn configuration holds, is met.
#[test]
fn a_goal_of_an_orientation_alone_is_met() {
    let met = goals_met(|pose, draws| {
        Goal::new(
            ARM_DOF,
            Vec::new(),
            Vec::new(),
            vec![tilted(pose.rotation, 0.0, 0.02, draws)],
        )
        .expect("a goal of one frame")
    });
    println!("an orientation alone within 0.02 rad: {met} of {GOALS}");
    assert!(met >= MOST_GOALS, "{met} of {GOALS}");
}

/// The fraction of the Cartesian path from `IN_ITS_PLANE` along `by`, the orientation kept.
fn fraction_along(by: Vector3<f64>) -> f64 {
    let start = ARM.chain().world_ee_pose(&IN_ITS_PLANE);
    let arms = Arms::<ARM_DOF, ARM_DOF>::new(vec![Arm::new("arm", &ARM)], None);
    let waypoints = [Isometry3::from_parts(
        Translation3::from(start.translation.vector + by),
        start.rotation,
    )];
    cartesian_path(
        &arms,
        &CartesianRequest {
            start: IN_ITS_PLANE,
            start_tolerance: [0.0; ARM_DOF],
            frame: FollowedFrame::of(ARM.chain(), Frame::GraspPoint)
                .expect("a Cartesian path follows the grasp point"),
            waypoints: &waypoints,
            collisions: Collisions::Unchecked,
            seed: SEED,
            stop: None,
        },
    )
    .expect("a path from a start inside the limits")
    .fraction
}

// A line in the arm's vertical plane is one its lift, elbow and wrist hold the orientation
// along. Sideways, the pan turns the orientation with it, and the path ends short.
#[test]
fn a_cartesian_path_covers_the_lines_on_which_the_joints_hold_the_orientation() {
    assert_eq!(fraction_along(Vector3::new(0.0, 0.0, LINE_M)), 1.0);
    let sideways = fraction_along(Vector3::new(0.0, LINE_M, 0.0));
    assert!(sideways < 1.0, "{sideways}");
}

// At the park pose, at every corner of the real arm's joint error around it and the jaw shut
// or open, every pair of the arm's own bodies stands farther apart than the start rule's
// least departure, `2k` of its least step for a pair whose two bodies move: a plan departs
// from there.
#[test]
fn a_plan_departs_from_the_park_pose_within_the_arms_error() {
    let least_departure_m = 2.0 * OWN_PAIR_MOVING_BODIES * MIN_DEPARTURE_STEP_M;
    let mut model = so101(&MESHES).expect("SO-101's self model builds");
    let corners = 3usize.pow(ARM_DOF as u32);
    for opening in [0.0, 1.0] {
        model.set_gripper_openings(&[opening]);
        for corner in 0..corners {
            let q: [f64; ARM_DOF] = std::array::from_fn(|i| {
                let side = (corner / 3usize.pow(i as u32)) % 3;
                HOME[i] + ARM_ERROR_RAD * (side as f64 - 1.0)
            });
            let nearest = model.min_distance(&[q]).expect("a distance");
            assert!(
                nearest.distance > least_departure_m,
                "at {q:?}, opening {opening}: {} - {} at {}",
                nearest.link_a,
                nearest.link_b,
                nearest.distance
            );
        }
    }
}
