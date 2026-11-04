//! A goal's constraints: what an orientation target measures, the goals refused, and the
//! frame of a goal on a part the arm carries; and the goal sampling: its count and its
//! end, each sample from the working state the valid goals before it leave.

mod common;

use std::num::NonZeroUsize;
use std::sync::Mutex;

use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::{
    Arm, ArmGoal, Axis, Body, Draws, Frame, Goal, GoalError, GoalSampler, IkGoalSampler,
    JointTarget, OrientationTarget, PositionTarget, sample_goals,
};
use nalgebra::{Isometry3, Unit, UnitQuaternion, Vector3};
use openarm_description::{HardwareVersion, Side};
use srs_model::ArmAnglePolicy;

use common::{JOINTS, LEFT_ARM, branch, left_fingers, off_the_branch_seams, positive};

/// The start a sample sets out from: every joint at 0.
const ZERO: [f64; JOINTS] = [0.0; JOINTS];

/// An axis drawn by its polar and azimuth angles.
fn drawn_axis(tc: &TestCase) -> Unit<Vector3<f64>> {
    let polar: f64 = tc.draw(gs::floats().min_value(0.0).max_value(std::f64::consts::PI));
    let azimuth: f64 = tc.draw(gs::floats().min_value(0.0).max_value(std::f64::consts::TAU));
    Unit::new_normalize(Vector3::new(
        polar.sin() * azimuth.cos(),
        polar.sin() * azimuth.sin(),
        polar.cos(),
    ))
}

fn drawn_orientation(tc: &TestCase) -> UnitQuaternion<f64> {
    let angle: f64 = tc.draw(gs::floats().min_value(-3.0).max_value(3.0));
    UnitQuaternion::from_axis_angle(&drawn_axis(tc), angle)
}

fn target(orientation: UnitQuaternion<f64>, free_axis: Option<Axis>) -> OrientationTarget {
    OrientationTarget {
        frame: Frame::GraspPoint,
        orientation,
        tolerance_rad: positive(0.1),
        free_axis,
    }
}

// With no free axis, the error of an orientation turned by an angle about any axis is
// that angle.
#[hegel::test]
fn the_error_is_the_angle_of_the_turn(tc: TestCase) {
    let orientation = drawn_orientation(&tc);
    let angle: f64 = tc.draw(gs::floats().min_value(0.0).max_value(3.1));
    let actual = orientation * UnitQuaternion::from_axis_angle(&drawn_axis(&tc), angle);
    let error = target(orientation, None).error(&actual);
    assert!(
        (error - angle).abs() < 1e-9,
        "error {error} for a turn of {angle}"
    );
}

// With a free axis, a turn about that axis of the frame is no error, and a turn about
// an axis across it is an error of its angle.
#[hegel::test]
fn a_turn_about_the_free_axis_is_no_error(tc: TestCase) {
    let orientation = drawn_orientation(&tc);
    let about: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
    let across: f64 = tc.draw(gs::floats().min_value(0.0).max_value(3.1));
    let free = target(orientation, Some(Axis::Z));
    let spun = orientation * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), about);
    assert!(free.error(&spun) < 1e-9);
    let tilted = spun * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), across);
    assert!((free.error(&tilted) - across).abs() < 1e-9);
}

fn joint(joint: usize) -> JointTarget {
    JointTarget {
        joint,
        position: 0.0,
        above: positive(0.1),
        below: positive(0.1),
    }
}

/// The child link of the fourth joint.
fn elbow() -> Frame {
    Frame::Link {
        segment: 3,
        offset: Isometry3::identity(),
    }
}

fn position(frame: Frame, tolerance_m: f64) -> PositionTarget {
    PositionTarget {
        frame,
        position: Vector3::new(0.3, 0.1, 0.2),
        tolerance_m: positive(tolerance_m),
    }
}

// A goal that names nothing, mixes joints with poses, names a joint or a frame twice in
// one list, or names a position and an orientation of two frames, is refused, the last
// naming both frames.
#[test]
fn malformed_goals_are_refused() {
    let new = |joints, positions, orientations| Goal::new(JOINTS, joints, positions, orientations);
    assert_eq!(new(vec![], vec![], vec![]), Err(GoalError::Empty));
    assert_eq!(
        new(
            vec![joint(0)],
            vec![position(Frame::GraspPoint, 0.01)],
            vec![]
        ),
        Err(GoalError::Mixed)
    );
    assert!(matches!(
        new(vec![joint(2), joint(2)], vec![], vec![]),
        Err(GoalError::Repeated(_))
    ));
    assert!(matches!(
        new(
            vec![],
            vec![position(elbow(), 0.01), position(elbow(), 0.02)],
            vec![]
        ),
        Err(GoalError::Repeated(_))
    ));
    assert_eq!(
        new(
            vec![],
            vec![position(Frame::GraspPoint, 0.01)],
            vec![OrientationTarget {
                frame: elbow(),
                ..target(UnitQuaternion::identity(), None)
            }]
        ),
        Err(GoalError::Frames(
            "the grasp point".to_string(),
            "the child link of joint 3".to_string()
        ))
    );
    assert_eq!(
        GoalError::Frames(
            "the grasp point".to_string(),
            "the child link of joint 3".to_string()
        )
        .to_string(),
        "a goal's positions and orientations name one frame of the arm, and this one names \
         the grasp point and the child link of joint 3: send the position and the \
         orientation of one of them"
    );
    assert!(matches!(
        new(
            vec![],
            vec![position(Frame::GraspPoint, 0.01), position(elbow(), 0.01)],
            vec![]
        ),
        Err(GoalError::Frames(..))
    ));
    assert!(new(vec![joint(0), joint(1)], vec![], vec![]).is_ok());
    assert!(
        new(
            vec![],
            vec![position(Frame::GraspPoint, 0.01)],
            vec![target(UnitQuaternion::identity(), Some(Axis::X))]
        )
        .is_ok()
    );
}

/// A goal of joint 3 within 0.1 rad of 1 rad.
fn joint_goal() -> Goal {
    Goal::new(
        JOINTS,
        vec![JointTarget {
            joint: 3,
            position: 1.0,
            above: positive(0.1),
            below: positive(0.1),
        }],
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal")
}

// The goal sampling keeps its count of goals while its end does not hold, none once it
// holds, and none that its validity rejects.
#[test]
fn goal_sampling_keeps_its_count_until_its_end() {
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = joint_goal();
    let eight = NonZeroUsize::new(8).expect("nonzero");
    let all = |_: &[f64; JOINTS], _| true;
    let arm = [ArmGoal {
        goal: &goal,
        sampler: &sampler,
    }];
    let goals = sample_goals(&arm, &ZERO, eight, 1, &|| false, &all);
    assert_eq!(goals.len(), 8);
    assert!(goals.iter().all(|q| goal.met(LEFT_ARM.chain(), q)));
    assert!(sample_goals(&arm, &ZERO, eight, 1, &|| true, &all).is_empty());
    assert!(sample_goals(&arm, &ZERO, eight, 1, &|| false, &|_, _| false).is_empty());
}

/// A goal sampler that answers its script in turn, each configuration where it is valid,
/// and keeps the working state each sample runs from.
struct Scripted {
    script: Vec<[f64; JOINTS]>,
    workings: Mutex<Vec<[f64; JOINTS]>>,
}

impl GoalSampler<JOINTS> for Scripted {
    fn sample_from(
        &self,
        _: &Goal,
        working: &[f64; JOINTS],
        _: usize,
        _: &mut Draws,
        valid: &dyn Fn(&[f64; JOINTS]) -> bool,
    ) -> Option<[f64; JOINTS]> {
        let mut workings = self.workings.lock().expect("the lock holds");
        let answer = self.script.get(workings.len()).copied();
        workings.push(*working);
        answer.filter(|q| valid(q))
    }
}

// Each goal sample runs from the working state: the start, then each goal a sample
// finds; a configuration the validity rejects never becomes it, and a goal equal to one
// kept joins the goals once.
#[test]
fn each_goal_sample_runs_from_the_last_valid_goal() {
    let (first, rejected, again, last) =
        ([0.1; JOINTS], [0.2; JOINTS], [0.1; JOINTS], [0.3; JOINTS]);
    let scripted = Scripted {
        script: vec![first, rejected, again, last],
        workings: Mutex::new(Vec::new()),
    };
    let goal = joint_goal();
    let goals = sample_goals(
        &[ArmGoal {
            goal: &goal,
            sampler: &scripted,
        }],
        &ZERO,
        NonZeroUsize::new(2).expect("nonzero"),
        1,
        &|| false,
        &|q, _| *q != rejected,
    );
    assert_eq!(goals, vec![first, last]);
    assert_eq!(
        *scripted.workings.lock().expect("the lock holds"),
        vec![ZERO, first, first, again]
    );
}

/// MoveIt's `max_goal_sampling_attempts` default (moveit2 2.15.2
/// planning_context_manager.cpp:260).
const MAX_GOAL_SAMPLING_ATTEMPTS: usize = 1000;

// The goal sampling ends once the samples since the last goal found reach 1000 less the
// goals found so far: three goals found, then 997 samples that find none.
#[test]
fn goal_sampling_ends_once_its_failing_samples_reach_1000_less_its_goals() {
    let found = [[0.1; JOINTS], [0.2; JOINTS], [0.3; JOINTS]];
    let scripted = Scripted {
        script: found.to_vec(),
        workings: Mutex::new(Vec::new()),
    };
    let goals = sample_goals(
        &[ArmGoal {
            goal: &joint_goal(),
            sampler: &scripted,
        }],
        &ZERO,
        NonZeroUsize::new(10).expect("nonzero"),
        1,
        &|| false,
        &|_, _| true,
    );
    assert_eq!(goals, found.to_vec());
    assert_eq!(
        scripted.workings.lock().expect("the lock holds").len(),
        MAX_GOAL_SAMPLING_ATTEMPTS
    );
}

/// The goal of the grasp point's pose at `q`, within 5 mm and 0.01 rad.
fn pose_goal_at(q: &[f64; JOINTS]) -> Goal {
    let pose = LEFT_ARM.chain().world_ee_pose(q);
    Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            position: pose.translation.vector,
            tolerance_m: positive(0.005),
        }],
        vec![OrientationTarget {
            frame: Frame::GraspPoint,
            orientation: pose.rotation,
            tolerance_rad: positive(0.01),
            free_axis: None,
        }],
    )
    .expect("a valid goal")
}

/// The poses a test reaches from two branches.
const TWO_BRANCH_POSES: usize = 10;

// A goal sample's first attempt sets out from its working state: of a pose that two
// branches reach, a working state on either branch gives a goal on that branch.
#[test]
fn a_goal_sample_sets_out_from_its_working_state() {
    let version = HardwareVersion::V2;
    let srs = srs_model::Arm::from_urdf(version.urdf(), version.base_link(Side::Left))
        .expect("an SRS arm")
        .with_tool_link(version.tcp_link(Side::Left))
        .expect("its grasp point");
    let chain = LEFT_ARM.chain();
    let limits = chain.limits();
    let sampler = IkGoalSampler::new(chain.clone());
    let within = |q: &[f64; JOINTS]| (0..JOINTS).all(|i| limits[i].contains(q[i]));
    let mut reached = 0;
    for (k, q) in off_the_branch_seams(20261104).take(200).enumerate() {
        if reached == TWO_BRANCH_POSES {
            break;
        }
        let pose = chain.world_ee_pose(&q);
        // The pose on the other shoulder branch, from a seed whose shoulder turns past
        // the quarter turn.
        let mut seed = q;
        seed[1] = std::f64::consts::PI - q[1];
        let Some(other) = (0..360)
            .filter_map(|step| {
                let psi = -std::f64::consts::PI + std::f64::consts::TAU * f64::from(step) / 360.0;
                srs.solve_ik(&srs.base_pose(&pose), ArmAnglePolicy::Fixed(psi), &seed)
                    .map(|solution| solution.q)
            })
            .find(|other| {
                within(other)
                    && branch(other) != branch(&q)
                    && [other[1], other[5]]
                        .iter()
                        .all(|v| v.cos().abs() >= std::f64::consts::FRAC_1_SQRT_2)
            })
        else {
            continue;
        };
        let goal = pose_goal_at(&q);
        for working in [q, other] {
            let found = sampler
                .sample_from(&goal, &working, 1, &mut Draws::new(k as u64), &|_| true)
                .unwrap_or_else(|| panic!("no goal from {working:?}"));
            assert_eq!(
                branch(&found),
                branch(&working),
                "from {working:?}: {found:?}"
            );
        }
        reached += 1;
    }
    assert_eq!(
        reached, TWO_BRANCH_POSES,
        "too few poses reach two branches"
    );
}

/// A goal's tolerances, ten times KDL's convergence, 1e-5 in meters and radians
/// (kdl_kinematics_parameters.yaml:27-31): a descent that brings the grasp point's
/// orientation to the target with its position meets them.
const CONVERGED_TOLERANCE: f64 = 1e-4;
/// How far the goal's orientation stands turned from the working state's, about the grasp
/// point's own z axis: 0.2 rad, two thousand times the tolerance.
const TURN_RAD: f64 = 0.2;
/// How near the working state the validity takes a goal: half a radian in the joints' L2
/// norm, above the 0.42 rad at most that the descents from these working states move
/// (measured on this test's poses with every configuration valid), and a small share of
/// the joint limits' box, where the restarts draw.
const NEAR_RAD: f64 = 0.5;

// A goal sample on the grasp point aims at the goal's orientation together with its
// position: from drawn working states whose grasp point stands at the goal's position, its
// orientation turned from the goal's, the descent from the working state meets goals of
// 0.1 mm and 0.1 mrad, half a radian from it at most (the validity takes no other).
#[test]
fn a_goal_sample_on_the_grasp_point_aims_at_its_orientation_too() {
    let chain = LEFT_ARM.chain();
    let sampler = IkGoalSampler::new(chain.clone());
    for (k, q) in off_the_branch_seams(11).take(10).enumerate() {
        let here = chain.world_ee_pose(&q);
        let goal = Goal::new(
            JOINTS,
            Vec::new(),
            vec![PositionTarget {
                frame: Frame::GraspPoint,
                position: here.translation.vector,
                tolerance_m: positive(CONVERGED_TOLERANCE),
            }],
            vec![OrientationTarget {
                frame: Frame::GraspPoint,
                orientation: here.rotation
                    * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), TURN_RAD),
                tolerance_rad: positive(CONVERGED_TOLERANCE),
                free_axis: None,
            }],
        )
        .expect("a valid goal");
        let near = |found: &[f64; JOINTS]| {
            (0..JOINTS)
                .map(|i| (found[i] - q[i]).powi(2))
                .sum::<f64>()
                .sqrt()
                <= NEAR_RAD
        };
        let found = sampler
            .sample_from(&goal, &q, 1, &mut Draws::new(k as u64), &near)
            .unwrap_or_else(|| panic!("no goal from {q:?}"));
        assert!(goal.met(chain, &found), "{found:?}");
    }
}

// A goal sample answers no configuration that its validity rejects: with every
// configuration valid it answers one, and with none valid none.
#[test]
fn a_goal_sample_answers_only_what_its_validity_accepts() {
    let chain = LEFT_ARM.chain();
    let sampler = IkGoalSampler::new(chain.clone());
    let q = off_the_branch_seams(7).next().expect("a configuration");
    let goal = pose_goal_at(&q);
    assert!(
        sampler
            .sample_from(&goal, &q, 1, &mut Draws::new(1), &|_| true)
            .is_some()
    );
    assert!(
        sampler
            .sample_from(&goal, &q, 1, &mut Draws::new(1), &|_| false)
            .is_none()
    );
}

// A goal on a finger, in the frame its rider gives, holds where the arm's clearance
// checks place the finger's piece, and not with the last joint turned a tenth of a
// radian.
#[test]
fn a_goal_on_a_rider_stands_where_the_checks_place_it() {
    let fingers = left_fingers();
    let arm = Arm::new("left_arm", &LEFT_ARM)
        .riding(&fingers)
        .expect("the fingers ride the hand");
    let q = [0.3, -0.2, 0.1, 1.0, 0.2, 0.1, 0.0];
    let Some((_, Body::Piece(finger))) = arm
        .bodies(&q)
        .into_iter()
        .find(|(name, _)| *name == fingers[0].name)
    else {
        panic!("a piece of the finger");
    };
    let frame = Frame::from(&fingers[0]);
    let goal = Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame,
            position: finger.pose().translation.vector,
            tolerance_m: positive(1e-9),
        }],
        vec![OrientationTarget {
            frame,
            orientation: finger.pose().rotation,
            tolerance_rad: positive(1e-9),
            free_axis: None,
        }],
    )
    .expect("a valid goal");
    assert!(goal.met(LEFT_ARM.chain(), &q));
    let mut turned = q;
    turned[JOINTS - 1] += 0.1;
    assert!(!goal.met(LEFT_ARM.chain(), &turned));
}
