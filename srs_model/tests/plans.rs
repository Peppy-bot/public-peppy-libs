//! What any planned line promises, on the fixture arms: its walk starts at the seed and
//! keeps the hand within MoveIt's Cartesian precision of the straight line to the goal,
//! and played as a joint path it keeps the hand on that line.

use std::time::{Duration, Instant};

use control_core::path_timing::{JointTrajectory, distinct_joint_waypoints};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{configs, unit_vectors};
use srs_model::chain_kinematics::{PRECISION_M, PRECISION_RAD, interpolate, pose_miss};
use srs_model::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use srs_model::{ELBOW, JointVec, plan_line};

mod common;
use common::arm;
use common::properties::sides;

/// The joint whose zero is the wrist's singular surface.
const WRIST_BEND: usize = 5;
/// How far (m) a played line's hand may stray from the exact line: twice the precision
/// of its walk, which holds at the walk's states and their midpoints, where a joint path
/// rounds its corners.
const STRAY_M: f64 = 2.0 * PRECISION_M;
/// How far a played line's hand may turn from the exact line's, likewise.
const STRAY_RAD: f64 = 2.0 * PRECISION_RAD;

// A goal near any posture of either fixture arm, the wrist or the elbow beside its
// singular surface among them: a line, when there is one, starts at the seed and lands
// on the goal within the precision, inside the joint limits, and a second plan gives the
// same walk. Played as a joint path at any joint limits, the hand stays on the line between
// the samples and lands on the goal.
#[hegel::test(test_cases = 200, seed = Some(69))]
fn a_planned_line_walks_the_line_and_plays_on_it(tc: TestCase) {
    let arm = arm(tc.draw(sides()));
    let joint_limits = arm.limits();
    let mut seed: JointVec = tc.draw(configs(joint_limits));
    match tc.draw(gs::integers::<u8>().max_value(2)) {
        1 => seed[WRIST_BEND] = tc.draw(gs::floats().min_value(-0.05).max_value(0.05)),
        2 => {
            let floor = joint_limits[ELBOW].lo;
            seed[ELBOW] = tc.draw(gs::floats().min_value(floor).max_value(floor + 0.2));
        }
        _ => {}
    }
    let start = arm.world_pose(&arm.at(&seed).ee_pose());
    let travel: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.4));
    let direction = tc.draw(unit_vectors());
    let turn: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.6));
    let axis = tc.draw(unit_vectors());
    let end = Isometry3::from_parts(
        Translation3::from(start.translation.vector + direction * travel),
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), turn) * start.rotation,
    );
    let Some(walk) = plan_line(arm, &end, seed) else {
        tc.event("no line");
        return;
    };
    assert_eq!(walk[0], seed, "the walk starts at the seed");
    assert_eq!(
        plan_line(arm, &end, seed).as_ref(),
        Some(&walk),
        "a plan repeats"
    );
    for (k, q) in walk.iter().enumerate() {
        for (i, (&v, limit)) in q.iter().zip(&joint_limits).enumerate() {
            assert!(limit.contains(v), "joint {i} at {v} on state {k}");
        }
    }
    let landed = pose_miss(&arm.world_ee_pose(&walk[walk.len() - 1]), &end);
    assert!(
        landed.position_m <= PRECISION_M && landed.orientation_rad <= PRECISION_RAD,
        "the walk lands {landed:?} off the goal"
    );
    let t0 = Instant::now();
    let played = JointTrajectory::through(
        &distinct_joint_waypoints(&walk),
        &tc.draw(gs::arrays(gs::floats().min_value(0.5).max_value(20.0))),
        &tc.draw(gs::arrays(gs::floats().min_value(0.5).max_value(50.0))),
        Duration::ZERO,
        t0,
    );
    let duration = played.clock().duration();
    let ab = end.translation.vector - start.translation.vector;
    for k in 0..=400u32 {
        let ee = arm.world_ee_pose(&played.sample(t0 + duration * k / 400));
        let from_start = ee.translation.vector - start.translation.vector;
        let along = if ab.norm_squared() > 0.0 {
            (from_start.dot(&ab) / ab.norm_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let off_m = (from_start - ab * along).norm();
        assert!(off_m <= STRAY_M, "the hand strays {off_m} m at {k}/400");
        let off_rad = (0..=100)
            .map(|j| {
                let on_the_turn = interpolate(&start, &end, f64::from(j) / 100.0).rotation;
                ee.rotation.angle_to(&on_the_turn)
            })
            .fold(f64::INFINITY, f64::min);
        assert!(
            off_rad <= STRAY_RAD,
            "the hand turns {off_rad} rad off the line at {k}/400"
        );
    }
    let miss = pose_miss(&arm.world_ee_pose(&played.sample(t0 + duration)), &end);
    assert!(
        miss.position_m <= PRECISION_M && miss.orientation_rad <= PRECISION_RAD,
        "the line ends {miss:?} from the goal"
    );
}

// A goal out of reach has no line.
#[test]
fn a_goal_out_of_reach_has_no_line() {
    let arm = arm("left");
    let seed: JointVec = std::array::from_fn(|i| arm.limits()[i].midpoint());
    let start = arm.world_ee_pose(&seed);
    let far = Isometry3::from_parts(
        Translation3::from(start.translation.vector + srs_model::nalgebra::Vector3::x() * 3.0),
        start.rotation,
    );
    assert_eq!(plan_line(arm, &far, seed), None);
}
