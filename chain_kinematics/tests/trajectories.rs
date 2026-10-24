//! The straight line between two poses and the timing of a move's
//! configurations with the hand, on chains of any shape; and the joint limits
//! every configuration they command is kept in.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use chain_kinematics::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion};
use chain_kinematics::{
    COINCIDENT_ARC_EPS, EeCaps, Limit, ServoTrajectory, all_within, clamp_into, ee_progress,
    interpolate, limit_margin, timed_with_ee,
};
use control_core::path_timing::TimedPath;
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, configs, points, poses, unit_vectors};
use urdf_rs::Robot;

mod common;
use common::properties::{ChainProperty, chain_of, on_a_drawn_robot};

// The line between any two poses puts, at every blend `s`, the position at the
// fraction `s` of the way and the orientation at the fraction `s` of one
// geodesic turn, or, for two orientations slerp cannot separate, at the goal's
// from the first blend on.
#[hegel::test]
fn a_line_blends_the_position_and_the_orientation_along_the_geodesic(tc: TestCase) {
    let start = tc.draw(poses());
    let axis = Unit::new_normalize(tc.draw(unit_vectors()));
    let turn: f64 = tc.draw(gs::floats().min_value(0.0).max_value(PI));
    let end = Isometry3::from_parts(
        Translation3::from(tc.draw(points())),
        UnitQuaternion::from_axis_angle(&axis, turn) * start.rotation,
    );
    let turn = start.rotation.angle_to(&end.rotation);
    let separable = start
        .rotation
        .try_slerp(&end.rotation, 0.5, COINCIDENT_ARC_EPS)
        .is_some();
    for k in 0..=20 {
        let s = f64::from(k) / 20.0;
        let got = interpolate(&start, &end, s);
        let along = start.translation.vector.lerp(&end.translation.vector, s);
        assert!(
            (got.translation.vector - along).norm() <= TOLERANCE,
            "at blend {s}"
        );
        let on_the_turn = if separable {
            (got.rotation.angle_to(&start.rotation) - s * turn).abs() <= TOLERANCE
                && (got.rotation.angle_to(&end.rotation) - (1.0 - s) * turn).abs() <= TOLERANCE
        } else {
            got.rotation == end.rotation
        };
        assert!(
            on_the_turn,
            "at blend {s} the orientation is off the geodesic of a {turn} rad turn"
        );
    }
}

/// A path shaped like a rollout's: up to 40 joint steps of up to 0.05 rad or
/// m each, on a nanoradian grid, from any configuration of `chain`, with the
/// velocity limits, acceleration limits and end-effector caps drawn for it.
struct DrawnPath<const N: usize> {
    configs: Vec<[f64; N]>,
    max_joint_velocity: [f64; N],
    max_joint_acceleration: [f64; N],
    ee: EeCaps,
}

impl<const N: usize> DrawnPath<N> {
    fn draw(tc: &TestCase, chain: &chain_kinematics::Chain<N>) -> Self {
        // A step under a nanoradian is no motion.
        const GRID: f64 = 1e-9;
        let seed = tc.draw(configs(chain.limits()));
        let steps: Vec<[f64; N]> = tc
            .draw(gs::vecs(gs::arrays(gs::floats().min_value(-0.05).max_value(0.05))).max_size(40))
            .into_iter()
            .map(|step: [f64; N]| step.map(|v| (v / GRID).round() * GRID))
            .collect();
        let configs = std::iter::once(seed)
            .chain(steps.iter().scan(seed, |q, step| {
                *q = std::array::from_fn(|i| q[i] + step[i]);
                Some(*q)
            }))
            .collect();
        Self {
            configs,
            max_joint_velocity: tc.draw(gs::arrays(gs::floats().min_value(0.5).max_value(20.0))),
            max_joint_acceleration: tc
                .draw(gs::arrays(gs::floats().min_value(0.5).max_value(50.0))),
            ee: EeCaps {
                linear_m_s: tc.draw(gs::floats().min_value(0.05).max_value(1.0)),
                angular_rad_s: tc.draw(gs::floats().min_value(0.2).max_value(1.5)),
            },
        }
    }

    fn timed(&self, chain: &chain_kinematics::Chain<N>) -> TimedPath {
        timed_with_ee(
            chain,
            &self.configs,
            &self.max_joint_velocity,
            &self.max_joint_acceleration,
            &self.ee,
        )
    }

    /// The time the end effector takes along the path at its caps: each step's travel
    /// at the binding cap, summed.
    fn ee_time(&self, chain: &chain_kinematics::Chain<N>) -> f64 {
        self.configs
            .windows(2)
            .map(|w| {
                self.ee
                    .time_between(&chain.world_ee_pose(&w[0]), &chain.world_ee_pose(&w[1]))
            })
            .sum()
    }
}

// A drawn path timed with the hand takes at least the end effector's time at its caps.
// Played over the longer of a request and its own time, it rests on its first
// configuration at the start and on its last from its duration on; and, sampled
// through the duration, every joint stays within its velocity limit,
// accelerates within its acceleration limit and leaves and reaches rest at no
// more than one sample's worth of that acceleration, and the end effector's progress
// runs no faster than real time.
struct PlaysFromRestToRestWithinTheLimits;

impl ChainProperty for PlaysFromRestToRestWithinTheLimits {
    fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
        let chain = chain_of::<N>(robot);
        let drawn = DrawnPath::draw(tc, &chain);
        let requested: f64 = tc.draw(gs::floats().min_value(0.0).max_value(5.0));
        let path = drawn.timed(&chain);
        let ee_s = drawn.ee_time(&chain);
        // The hand's time is a sum of steps, each rounded.
        assert!(
            path.duration_s() >= ee_s * (1.0 - 1e-12),
            "a {}s path under the {ee_s}s the end effector needs at its caps",
            path.duration_s()
        );
        let (first, last) = (drawn.configs[0], drawn.configs[drawn.configs.len() - 1]);
        let duration_s = requested.max(path.duration_s());
        let traj = ServoTrajectory::<N>::new(
            path.clone(),
            Duration::from_secs_f64(duration_s),
            Instant::now(),
        );
        let (t0, duration) = (traj.clock().motion_start(), traj.clock().duration());
        // A motion that lasts no time is at its end throughout.
        if !duration.is_zero() {
            assert_eq!(traj.sample(t0), first);
        }
        assert_eq!(traj.sample(t0 + duration), last);

        const SAMPLES: u32 = 400;
        let step = duration / SAMPLES;
        if step.is_zero() {
            return;
        }
        let dt = step.as_secs_f64();
        // The clock keeps whole nanoseconds, which can shorten the duration by
        // one; the profile's arithmetic over a thousand cells carries a part in
        // a billion; and a position carries its own rounding, a few ulps of its
        // size, into the differences over a sample and a sample squared.
        let slack = 1.0 + 2e-9 / duration.as_secs_f64() + 1e-9;
        let samples: Vec<[f64; N]> = (0..=SAMPLES).map(|k| traj.sample(t0 + step * k)).collect();
        let rounding =
            |i: usize| 4.0 * f64::EPSILON * samples.iter().map(|q| q[i].abs()).fold(0.0, f64::max);
        // The samples sit on whole nanoseconds, so the path is read at the
        // fraction of the duration each one is at.
        let progress: Vec<f64> = (0..=SAMPLES)
            .map(|k| {
                ee_progress(&path.at_fraction((step * k).as_secs_f64() / duration.as_secs_f64()))
            })
            .collect();
        for (k, w) in progress.windows(2).enumerate() {
            assert!(
                w[1] - w[0] <= dt * slack,
                "the end effector's progress ran {} s in one {dt} s sample at sample {k}",
                w[1] - w[0]
            );
        }
        let limits = || {
            drawn
                .max_joint_velocity
                .iter()
                .zip(&drawn.max_joint_acceleration)
        };
        for (k, w) in samples.windows(3).enumerate() {
            for (i, (&v_max, &a_max)) in limits().enumerate() {
                let speed = (w[1][i] - w[0][i]).abs() / dt;
                assert!(
                    speed <= v_max * slack + rounding(i) / dt,
                    "joint {i} at {speed} rad/s past its {v_max} rad/s"
                );
                let acceleration = (w[2][i] - 2.0 * w[1][i] + w[0][i]).abs() / (dt * dt);
                assert!(
                    acceleration <= a_max * slack + rounding(i) / (dt * dt),
                    "joint {i} at {acceleration} rad/s² between samples {k} and {}, past its {a_max}",
                    k + 2
                );
            }
        }
        // The samples stop short of the end by the step's truncation to whole
        // nanoseconds, so the arrival is read over the last step before the
        // end itself.
        let leaving = &samples[..2];
        let arriving = [
            traj.sample(t0 + duration - step),
            traj.sample(t0 + duration),
        ];
        for (i, (_, &a_max)) in limits().enumerate() {
            for (end, w) in [("leaves", leaving), ("reaches", &arriving[..])] {
                let speed = (w[1][i] - w[0][i]).abs() / dt;
                assert!(
                    speed <= a_max * dt * slack + rounding(i) / dt,
                    "joint {i} {end} rest at {speed} rad/s, past the {} one sample at its \
                     acceleration limit allows",
                    a_max * dt
                );
            }
        }
    }
}

#[hegel::test]
fn a_servo_trajectory_plays_its_path_from_rest_to_rest_within_the_limits(tc: TestCase) {
    on_a_drawn_robot::<PlaysFromRestToRestWithinTheLimits>(&tc);
}

// A drawn servo trajectory's probes start on its first configuration at its
// start and end on its last at its end; each stands where the trajectory plays
// it at its instant, and no joint changes by more than the step between
// neighbours.
struct ProbesWalkTheTrajectory;

impl ChainProperty for ProbesWalkTheTrajectory {
    fn check<const N: usize>(tc: &TestCase, robot: &Robot) {
        let chain = chain_of::<N>(robot);
        let drawn = DrawnPath::draw(tc, &chain);
        let step: f64 = tc.draw(gs::floats().min_value(0.01).max_value(1.0));
        let path = drawn.timed(&chain);
        let traj = ServoTrajectory::<N>::new(
            path.clone(),
            Duration::from_secs_f64(path.duration_s()),
            Instant::now(),
        );
        let probes = traj.probes_every(step);
        let (first, last) = (drawn.configs[0], drawn.configs[drawn.configs.len() - 1]);
        let start = traj.clock().motion_start();
        assert_eq!(probes[0].0, start);
        assert_eq!(probes[0].1, first);
        assert_eq!(probes[probes.len() - 1].1, last);
        for pair in probes.windows(2) {
            let ((t0, q0), (t1, q1)) = (pair[0], pair[1]);
            assert!(t1 >= t0, "a probe before the one ahead of it");
            for (a, b) in q0.iter().zip(&q1) {
                assert!(
                    (b - a).abs() <= step * (1.0 + 1e-6),
                    "{a} to {b} over a step of {step}"
                );
            }
        }
        for (at, q) in &probes {
            let played = traj.sample(*at);
            for (a, b) in played.iter().zip(q) {
                assert!(
                    (a - b).abs() <= 1e-6,
                    "{q:?} probed where the trajectory plays {played:?}"
                );
            }
        }
    }
}

#[hegel::test]
fn a_servo_trajectory_is_probed_from_its_start_to_its_end(tc: TestCase) {
    on_a_drawn_robot::<ProbesWalkTheTrajectory>(&tc);
}

/// Seven joint limits and a configuration, each anywhere within 10 rad of
/// zero, so the configuration lands inside, on and outside its limits.
fn limits_and_configuration(tc: &TestCase) -> ([Limit; 7], [f64; 7]) {
    let span = || gs::floats().min_value(-10.0).max_value(10.0);
    let q: [f64; 7] = tc.draw(gs::arrays(span()));
    let ends: [f64; 14] = tc.draw(gs::arrays(span()));
    let limits = std::array::from_fn(|i| Limit {
        lo: ends[2 * i].min(ends[2 * i + 1]),
        hi: ends[2 * i].max(ends[2 * i + 1]),
    });
    (limits, q)
}

// A clamped configuration lies inside every joint's limits, a joint already
// inside them is left where it was, and a configuration is within its limits
// exactly when clamping leaves it as it was.
#[hegel::test]
fn a_clamped_configuration_lies_inside_the_limits(tc: TestCase) {
    let (limits, q) = limits_and_configuration(&tc);
    let clamped = clamp_into(&limits, &q);
    assert!(all_within(&limits, &clamped));
    for i in 0..7 {
        if limits[i].contains(q[i]) {
            assert_eq!(clamped[i], q[i], "joint {i} inside its limits moved");
        }
    }
    assert_eq!(all_within(&limits, &q), clamped == q);
}

// A configuration's limit margin is non-negative exactly when it lies inside
// its limits, and for one outside them it is minus the distance clamping
// moves the joint it moves furthest.
#[hegel::test]
fn the_limit_margin_is_signed_by_whether_the_limits_hold(tc: TestCase) {
    let (limits, q) = limits_and_configuration(&tc);
    let margin = limit_margin(&limits, &q);
    assert_eq!(margin >= 0.0, all_within(&limits, &q));
    if margin < 0.0 {
        let clamped = clamp_into(&limits, &q);
        let furthest = (0..7)
            .map(|i| (clamped[i] - q[i]).abs())
            .fold(0.0, f64::max);
        assert_eq!(-margin, furthest);
    }
}
