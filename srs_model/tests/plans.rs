//! What any planned line promises, on the fixture arms: it lasts at least the
//! request, starts where the arm is and ends at the goal, and the configurations
//! it commands one control period apart stay within the joint velocity and
//! acceleration limits it was sized against.

use std::time::{Duration, Instant};

use control_core::timing::tick_fractions;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs, unit_vectors};
use srs_model::chain_kinematics::pose_miss;
use srs_model::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use srs_model::{ELBOW, JointVec, LineCursor, LineTrajectory, PlanLimits, plan_line};

mod common;
use common::arm;
use common::properties::sides;

const PERIOD: Duration = Duration::from_millis(10);
const LONGEST_MOVE: Duration = Duration::from_secs(60);

/// The joint whose zero is the wrist's singular surface.
const WRIST_BEND: usize = 5;

// A goal near any posture of either fixture arm, the wrist or the elbow beside its
// singular surface among them, at any request and any joint limits: a line, when
// there is one, lasts at least the request, starts at the seed's own solution of
// the start pose and ends on the goal, and the configurations the runtime commands,
// one period apart, keep every joint within its velocity limit and within its
// acceleration limit, as the plan checked.
#[hegel::test]
fn a_planned_line_plays_within_the_limits_it_was_sized_against(tc: TestCase) {
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
    let requested: f64 = tc.draw(gs::floats().min_value(0.0).max_value(5.0));
    let limits = PlanLimits {
        max_joint_velocity_rad_s: tc.draw(gs::arrays(gs::floats().min_value(0.5).max_value(20.0))),
        max_joint_acceleration_rad_s2: tc
            .draw(gs::arrays(gs::floats().min_value(0.5).max_value(50.0))),
        control_period: PERIOD,
        max_duration: LONGEST_MOVE,
    };
    let Some(line) = plan_line(arm, &end, seed, &limits, Duration::from_secs_f64(requested)) else {
        tc.event("no line");
        return;
    };
    tc.event(format!("line {:?}", line.elbow));
    let duration_s = line.duration.as_secs_f64();
    assert!(
        duration_s >= requested - 1e-9,
        "a {duration_s}s plan under a {requested}s request"
    );
    assert!(
        line.duration <= LONGEST_MOVE,
        "a {duration_s}s plan past the longest a move may take"
    );
    let normalized = arm
        .solve_ik(
            &arm.base_pose(&start),
            srs_model::ArmAnglePolicy::FromSeed,
            &seed,
        )
        .expect("the seed's own pose solves")
        .q;
    assert_eq!(
        line.start_q, normalized,
        "the line starts off the seed's own solution"
    );
    let period_s = PERIOD.as_secs_f64();
    let mut cursor = LineCursor::new(line.start_q, line.swivel);
    let t0 = Instant::now();
    let traj = LineTrajectory::new(start, end, line.path.clone(), line.duration, t0);
    let ticks: Vec<JointVec> = tick_fractions(duration_s, period_s)
        .map(|f| {
            let blend = traj.blend(t0 + Duration::from_secs_f64(f * duration_s));
            cursor
                .advance(arm, |s| traj.pose_at_blend(s), blend)
                .expect("the plan validated every tick of its line")
        })
        .collect();
    let landed = ticks[ticks.len() - 1];
    let miss = pose_miss(&arm.world_pose(&arm.at(&landed).ee_pose()), &end);
    assert!(
        miss.position_m <= TOLERANCE && miss.orientation_rad <= TOLERANCE,
        "the line ends {miss:?} from the goal"
    );
    let joint_limits = limits
        .max_joint_velocity_rad_s
        .iter()
        .zip(&limits.max_joint_acceleration_rad_s2);
    for (k, w) in ticks.windows(3).enumerate() {
        for (i, (&v_max, &a_max)) in joint_limits.clone().enumerate() {
            let speed = (w[1][i] - w[0][i]).abs() / period_s;
            assert!(
                speed <= v_max * (1.0 + 1e-6),
                "joint {i} at {speed} rad/s at tick {k}, past its {v_max}"
            );
            // A second difference over a period magnifies the positions'
            // rounding ten-thousandfold.
            let acceleration = (w[2][i] - 2.0 * w[1][i] + w[0][i]).abs() / (period_s * period_s);
            assert!(
                acceleration <= a_max * (1.0 + 1e-6),
                "joint {i} at {acceleration} rad/s² at tick {k}, past its {a_max}"
            );
        }
    }
}

// A line that tracks is refused once it cannot run within the longest a move may
// take, and planned once it can.
#[test]
fn a_line_past_the_longest_a_move_may_take_has_no_plan() {
    let arm = arm("left");
    let seed: JointVec = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
    let mut end = arm.world_pose(&arm.at(&seed).ee_pose());
    end.translation.vector.z += 0.05;
    let limits = |max_duration| PlanLimits {
        max_joint_velocity_rad_s: [2.0; 7],
        max_joint_acceleration_rad_s2: [5.0; 7],
        control_period: PERIOD,
        max_duration,
    };
    let plan = |max_duration| plan_line(arm, &end, seed, &limits(max_duration), Duration::ZERO);
    let line = plan(LONGEST_MOVE).expect("a 5 cm lift tracks");
    assert_eq!(plan(line.duration / 2), None);
}
