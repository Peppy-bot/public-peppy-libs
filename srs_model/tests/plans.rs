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
use srs_model::nalgebra::Vector3;
use srs_model::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use srs_model::{
    ARM_DOF, Arm, CartesianPlan, JOINT_PATH_CORRIDOR_M, JointVec, LineCursor, LineTrajectory,
    PlanLimits, PlanRejection, plan_cartesian,
};

mod common;
use common::arm;
use common::properties::sides;

const PERIOD: Duration = Duration::from_millis(10);

// A goal near any posture of either fixture arm, at any request, any joint
// limits and any end-effector caps: a plan, when there is one, lasts at least
// the request; a line starts at the seed's own solution of the start pose and
// a servo or joint path at the seed; each ends on the goal, to the slack; and the
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
        Err(PlanRejection::JointPathLeavesTheLine { .. }) => {
            tc.event("refused: the joint path leaves the line");
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
        CartesianPlan::Servo(servo) | CartesianPlan::Joint(servo) => {
            tc.event(match plan {
                CartesianPlan::Servo(_) => "servo",
                _ => "joint",
            });
            let configuration = |f: f64| -> JointVec { joints(&servo.path.at_fraction(f)) };
            assert_eq!(
                configuration(0.0),
                seed,
                "the path starts away from the seed"
            );
            if matches!(plan, CartesianPlan::Joint(_)) {
                let swing_m = widest_swing_m(arm, &configuration);
                assert!(
                    swing_m <= JOINT_PATH_CORRIDOR_M + 1e-4,
                    "the joint path takes the grasp point {swing_m} m from the line"
                );
            }
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

/// The farthest the grasp point of the path `configuration`, read at 201
/// fractions of it, stands from the straight segment between its two ends (m).
fn widest_swing_m(arm: &Arm, configuration: &dyn Fn(f64) -> JointVec) -> f64 {
    let at = |f: f64| -> Vector3<f64> { arm.at(&configuration(f)).ee_pose().translation.vector };
    let (start, end) = (at(0.0), at(1.0));
    let along = end - start;
    (0..=200)
        .map(|k| {
            let point = at(f64::from(k) / 200.0);
            let share = if along.norm_squared() == 0.0 {
                0.0
            } else {
                ((point - start).dot(&along) / along.norm_squared()).clamp(0.0, 1.0)
            };
            (point - (start + share * along)).norm()
        })
        .fold(0.0, f64::max)
}

/// Joint limits that a slow end effector, at caps of 1 mm/s and 1 mrad/s,
/// cannot meet on a line or with the servo within their time.
fn slow_end_effector_limits() -> PlanLimits {
    PlanLimits {
        max_joint_velocity_rad_s: [2.0; ARM_DOF],
        max_joint_acceleration_rad_s2: [5.0; ARM_DOF],
        ee: EeCaps {
            linear_m_s: 0.001,
            angular_rad_s: 0.001,
        },
        control_period: PERIOD,
        smoothing: command_smoothing(PERIOD).expect("a 100 Hz period carries the cutoff"),
    }
}

/// The left fixture arm, the middle of its joint limits as the seed, and the
/// seed with `moves` added to its joints.
fn seed_and_moved(moves: &[(usize, f64)]) -> (&'static Arm, JointVec, JointVec) {
    let arm = arm("left");
    let joint_limits = arm.limits();
    let seed: JointVec = std::array::from_fn(|j| (joint_limits[j].lo + joint_limits[j].hi) / 2.0);
    let mut moved = seed;
    for &(j, by) in moves {
        moved[j] += by;
    }
    (arm, seed, moved)
}

/// The pose of an in-limit configuration, which neither a line nor the servo
/// reaches in its time at end-effector caps of 1 mm/s and 1 mrad/s, plans on
/// the joint tier: from the seed, straight in joint space, so each joint stays
/// between its start and its end, to a configuration on the pose within the
/// slack, with the grasp point within the corridor of the straight line, and
/// no faster than the caps allow.
#[test]
fn a_pose_that_no_line_or_servo_reaches_in_time_plans_on_the_joint_tier() {
    let (arm, seed, moved) = seed_and_moved(&[(0, 0.3), (1, 0.3), (3, 0.3)]);
    let start = arm.world_pose(&arm.at(&seed).ee_pose());
    let end = arm.world_pose(&arm.at(&moved).ee_pose());
    let limits = slow_end_effector_limits();
    let tolerance = ServoTolerances::new(1e-3, 1e-2).expect("a reachable tolerance");
    let plan = plan_cartesian(arm, &end, seed, &limits, Duration::ZERO, Ok(tolerance))
        .expect("the exact solver reaches the pose");
    let CartesianPlan::Joint(joint) = &plan else {
        panic!("{plan:?} is not on the joint tier")
    };
    assert_eq!(joints::<ARM_DOF>(&joint.path.at_fraction(0.0)), seed);
    let landed: JointVec = joints(&joint.path.at_fraction(1.0));
    let miss = pose_miss(&arm.world_pose(&arm.at(&landed).ee_pose()), &end);
    assert!(
        miss.position_m <= tolerance.position_m() + TOLERANCE && miss.orientation_rad <= TOLERANCE,
        "the joint path ends {miss:?} from the pose"
    );
    for k in 0..=200 {
        let q: JointVec = joints(&joint.path.at_fraction(f64::from(k) / 200.0));
        for j in 0..ARM_DOF {
            let (low, high) = (seed[j].min(landed[j]), seed[j].max(landed[j]));
            assert!(
                (low - TOLERANCE..=high + TOLERANCE).contains(&q[j]),
                "joint {j} at {} leaves [{low}, {high}] at {k}/200",
                q[j]
            );
        }
    }
    let swing_m = widest_swing_m(arm, &|f| joints(&joint.path.at_fraction(f)));
    assert!(
        swing_m <= JOINT_PATH_CORRIDOR_M,
        "the joint path takes the grasp point {swing_m} m from the line"
    );
    let travel_m = (end.translation.vector - start.translation.vector).norm();
    assert!(
        plan.duration().as_secs_f64() >= travel_m / limits.ee.linear_m_s * (1.0 - 1e-6),
        "a {:?} plan for {travel_m} m at 1 mm/s",
        plan.duration()
    );
}

/// A turn of the shoulder that sweeps the grasp point on an arc, which
/// neither a line nor the servo reaches in its time at the slow caps, is no
/// plan on the joint tier either: its joint path takes the grasp point
/// farther than the corridor from the straight line, and the refusal names
/// how far.
#[test]
fn a_joint_path_that_swings_wider_than_the_corridor_is_refused_with_its_swing() {
    let (arm, seed, moved) = seed_and_moved(&[(0, 1.2)]);
    let joint_limits = arm.limits();
    assert!(
        joint_limits[0].contains(moved[0]),
        "the turn stays within its limits"
    );
    let end = arm.world_pose(&arm.at(&moved).ee_pose());
    let tolerance = ServoTolerances::new(1e-3, 1e-2).expect("a reachable tolerance");
    let refused = plan_cartesian(
        arm,
        &end,
        seed,
        &slow_end_effector_limits(),
        Duration::ZERO,
        Ok(tolerance),
    );
    let Err(PlanRejection::JointPathLeavesTheLine { swing_m }) = refused else {
        panic!("{refused:?} is no refusal for the swing")
    };
    assert!(swing_m > JOINT_PATH_CORRIDOR_M, "a swing of {swing_m} m");
    let straight =
        |f: f64| -> JointVec { std::array::from_fn(|j| seed[j] + f * (moved[j] - seed[j])) };
    let measured_m = widest_swing_m(arm, &straight);
    assert!(
        (swing_m - measured_m).abs() < 0.005,
        "the refusal names {swing_m} m, the path swings {measured_m} m"
    );
    assert!(
        PlanRejection::JointPathLeavesTheLine { swing_m }
            .to_string()
            .ends_with(&format!(
                "takes the grasp point {:.1} cm from the straight line, farther than 2.0 cm",
                swing_m * 100.0
            ))
    );
}
