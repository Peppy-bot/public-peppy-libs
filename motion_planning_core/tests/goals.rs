//! A goal's constraints: what an orientation target measures, the goals refused, and the
//! frame of a goal on a part the arm carries; and the goal sampler's deadline.

mod common;

use std::sync::atomic::AtomicBool;
use std::time::Instant;

use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::{
    Arm, Axis, Frame, Goal, GoalError, GoalSampler, IkGoalSampler, JointTarget, OrientationTarget,
    PositionTarget,
};
use nalgebra::{Isometry3, Unit, UnitQuaternion, Vector3};

use common::{JOINTS, LEFT_ARM, an_hour_from_now, left_fingers};

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
        tolerance_rad: 0.1,
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
        above: 0.1,
        below: 0.1,
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
        tolerance_m,
    }
}

// A goal that names nothing, mixes joints with poses, holds a tolerance that is not
// above 0, names a joint or a frame twice in one list, or names a position and an
// orientation of two frames, is refused, the last naming both frames.
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
    assert_eq!(
        new(vec![], vec![position(Frame::GraspPoint, 0.0)], vec![]),
        Err(GoalError::Tolerance(0.0))
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

// The sampler draws its count while its deadline lies ahead and its stop flag is clear,
// and nothing once the deadline has passed or the flag is set.
#[test]
fn the_sampler_draws_until_its_deadline_or_its_stop() {
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goal = Goal::new(
        JOINTS,
        vec![JointTarget {
            joint: 3,
            position: 1.0,
            above: 0.1,
            below: 0.1,
        }],
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let clear = AtomicBool::new(false);
    let set = AtomicBool::new(true);
    assert_eq!(
        sampler.sample(&goal, 8, 1, an_hour_from_now(), None).len(),
        8
    );
    assert_eq!(
        sampler
            .sample(&goal, 8, 1, an_hour_from_now(), Some(&clear))
            .len(),
        8
    );
    assert!(sampler.sample(&goal, 8, 1, Instant::now(), None).is_empty());
    assert!(
        sampler
            .sample(&goal, 8, 1, an_hour_from_now(), Some(&set))
            .is_empty()
    );
}

// A goal on a finger, in the frame its rider gives, holds where the arm's clearance
// checks place the finger's piece, and not with the last joint turned a tenth of a
// radian.
#[test]
fn a_goal_on_a_rider_stands_where_the_checks_place_it() {
    let fingers = left_fingers();
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&fingers)
        .expect("the fingers ride the hand");
    let q = [0.3, -0.2, 0.1, 1.0, 0.2, 0.1, 0.0];
    let (_, finger) = arm
        .pieces(&q)
        .into_iter()
        .find(|(link, _)| *link == fingers[0].link)
        .expect("a piece of the finger");
    let frame = Frame::from(&fingers[0]);
    let goal = Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame,
            position: finger.pose().translation.vector,
            tolerance_m: 1e-9,
        }],
        vec![OrientationTarget {
            frame,
            orientation: finger.pose().rotation,
            tolerance_rad: 1e-9,
            free_axis: None,
        }],
    )
    .expect("a valid goal");
    assert!(goal.met(LEFT_ARM.chain(), &q));
    let mut turned = q;
    turned[JOINTS - 1] += 0.1;
    assert!(!goal.met(LEFT_ARM.chain(), &turned));
}
