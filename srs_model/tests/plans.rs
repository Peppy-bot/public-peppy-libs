//! What any accepted Cartesian plan promises, on the fixture arms: it lasts at
//! least the request, starts where the arm is and ends at the goal, and the
//! configurations it commands one control period apart stay within the joint
//! velocity and acceleration limits it was sized against.

use std::time::{Duration, Instant};

use chain_kinematics::{EeCaps, ServoTolerances, command_smoothing, joints};
use control_core::timing::tick_fractions;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs, unit_vectors};
use srs_model::chain_kinematics::pose_miss;
use srs_model::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use srs_model::{
    CartesianPlan, JointVec, LineCursor, LineTrajectory, PlanLimits, PlanRejection, plan_cartesian,
};

mod common;
use common::arm;
use common::properties::sides;

const PERIOD: Duration = Duration::from_millis(10);

// A goal near any posture of either fixture arm, at any request, any joint
// limits and any end-effector caps: a plan, when there is one, lasts at least
// the request; a line starts at the seed's own solution of the start pose and
// a servo path at the seed; both end on the goal, to the servo's slack; and the
// configurations the runtime commands, one period apart, keep every joint
// within its velocity limit and within its acceleration limit, as the plan
// checked. A refusal is the law failing to
// converge: the request and the limits drawn here are always sane budgets.
#[hegel::test]
fn an_accepted_plan_plays_within_the_limits_it_was_sized_against(tc: TestCase) {
    let arm = arm(tc.draw(sides()));
    let seed = tc.draw(configs(arm.limits()));
    let start = arm.world_pose(&arm.at(&seed).ee_pose());
    let travel: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.1));
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
        ee: EeCaps {
            linear_m_s: tc.draw(gs::floats().min_value(0.05).max_value(1.0)),
            angular_rad_s: tc.draw(gs::floats().min_value(0.2).max_value(1.5)),
        },
        control_period: PERIOD,
        smoothing: command_smoothing(PERIOD).expect("a 100 Hz period carries the cutoff"),
    };
    let tolerance = ServoTolerances::new(1e-3, 1e-2).expect("a reachable tolerance");
    let plan = match plan_cartesian(
        arm,
        &end,
        seed,
        &limits,
        Duration::from_secs_f64(requested),
        Ok(tolerance),
    ) {
        Ok(plan) => plan,
        Err(PlanRejection::Unreachable) => {
            tc.event("refused: unreachable");
            return;
        }
        Err(refusal) => panic!("a sane budget was refused as {refusal}"),
    };
    let duration_s = plan.duration().as_secs_f64();
    assert!(
        duration_s >= requested - 1e-9,
        "a {duration_s}s plan under a {requested}s request"
    );
    let period_s = PERIOD.as_secs_f64();
    let (ticks, slack): (Vec<JointVec>, f64) = match &plan {
        CartesianPlan::Line(line) => {
            tc.event(format!("line {:?}", line.elbow));
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
            let mut cursor = LineCursor::new(line.start_q, line.swivel);
            let t0 = Instant::now();
            let traj = LineTrajectory::new(
                start,
                end,
                line.path.clone(),
                Duration::from_secs_f64(duration_s),
                t0,
            );
            let ticks = tick_fractions(duration_s, period_s)
                .map(|f| {
                    let blend = traj.blend(t0 + Duration::from_secs_f64(f * duration_s));
                    cursor
                        .advance(arm, |s| traj.pose_at_blend(s), blend)
                        .expect("the plan validated every tick of its line")
                })
                .collect();
            (ticks, 1.0 + 1e-6)
        }
        CartesianPlan::Servo(servo) => {
            tc.event("servo");
            let configuration = |f: f64| -> JointVec { joints(&servo.path.at_fraction(f)) };
            assert_eq!(
                configuration(0.0),
                seed,
                "the servo path starts away from the seed"
            );
            let ticks = tick_fractions(duration_s, period_s)
                .map(configuration)
                .collect();
            (ticks, 1.0 + 1e-6)
        }
    };
    let landed = ticks[ticks.len() - 1];
    let miss = pose_miss(&arm.world_pose(&arm.at(&landed).ee_pose()), &end);
    assert!(
        miss.position_m <= tolerance.position_m() + TOLERANCE
            && miss.orientation_rad <= tolerance.orientation_rad() + TOLERANCE,
        "the plan ends {miss:?} from the goal"
    );
    let joint_limits = limits
        .max_joint_velocity_rad_s
        .iter()
        .zip(&limits.max_joint_acceleration_rad_s2);
    for (k, w) in ticks.windows(3).enumerate() {
        for (i, (&v_max, &a_max)) in joint_limits.clone().enumerate() {
            let speed = (w[1][i] - w[0][i]).abs() / period_s;
            assert!(
                speed <= v_max * slack,
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
