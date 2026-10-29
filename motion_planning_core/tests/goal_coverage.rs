//! The generic goal sampler against OpenArm's closed form: on reachable poses, it
//! reaches each shoulder and wrist branch that `srs_model` finds. It reaches the
//! reachable poses of the links above the grasp point, and of a part a link carries.

mod common;

use std::collections::BTreeSet;

use motion_planning_core::{
    Frame, Goal, GoalSampler, IkGoalSampler, OrientationTarget, PositionTarget,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use openarm_description::{HardwareVersion, Side};
use srs_model::ArmAnglePolicy;

use common::{JOINTS, LEFT_ARM, an_hour_from_now};

/// The poses drawn.
const POSES: usize = 200;
/// The configurations the generic sampler draws per pose.
const GOAL_SAMPLES: usize = 64;
/// The arm angles at which `srs_model` is asked for a solution on each branch.
const ARM_ANGLES: usize = 720;
/// The share of the poses on which the generic sampler reaches every branch.
const COVERED_SHARE: f64 = 0.95;
/// How near the grasp point lies to the pose for the generic sampler's goal.
const POSITION_TOLERANCE_M: f64 = 0.005;
const ORIENTATION_TOLERANCE_RAD: f64 = 0.01;

/// The shoulder and wrist branch of `q`: on which side of a quarter turn the middle
/// joint of each spherical limb stands. The two solutions of a limb differ there.
fn branch(q: &[f64; JOINTS]) -> (bool, bool) {
    (q[1].cos() >= 0.0, q[5].cos() >= 0.0)
}

/// SplitMix64 draws in `[0, 1)`.
struct Draws(u64);

impl Draws {
    fn unit(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1_u64 << 53) as f64
    }
}

// On drawn reachable poses of OpenArm v2's left arm, the generic sampler reaches each
// branch that srs_model finds on at least 95% of them.
#[test]
fn the_generic_sampler_reaches_each_branch_srs_model_finds() {
    let version = HardwareVersion::V2;
    let srs = srs_model::Arm::from_urdf(version.urdf(), version.base_link(Side::Left))
        .expect("an SRS arm")
        .with_tool_link(version.tcp_link(Side::Left))
        .expect("its grasp point");
    let chain = LEFT_ARM.chain();
    let sampler = IkGoalSampler::new(chain.clone());
    let limits = chain.limits();
    let mut draws = Draws(20261028);
    let mut covered = 0;
    // The poses on which srs_model finds more than one branch.
    let mut several = 0;
    let mut missed = Vec::new();
    for pose_index in 0..POSES {
        let q: [f64; JOINTS] =
            std::array::from_fn(|i| limits[i].lo + (limits[i].hi - limits[i].lo) * draws.unit());
        let pose = chain.world_ee_pose(&q);
        // A seed in each branch: the drawn configuration with its limbs' middle joints
        // turned to each side of a quarter turn where their limits allow.
        let sides = |joint: usize| -> Vec<f64> {
            [
                q[joint],
                std::f64::consts::PI - q[joint],
                -std::f64::consts::PI - q[joint],
            ]
            .into_iter()
            .filter(|v| limits[joint].lo <= *v && *v <= limits[joint].hi)
            .collect()
        };
        let mut found = BTreeSet::new();
        for shoulder in sides(1) {
            for wrist in sides(5) {
                let mut seed = q;
                (seed[1], seed[5]) = (shoulder, wrist);
                for k in 0..ARM_ANGLES {
                    let psi = -std::f64::consts::PI
                        + std::f64::consts::TAU * k as f64 / ARM_ANGLES as f64;
                    if let Some(solution) =
                        srs.solve_ik(&srs.base_pose(&pose), ArmAnglePolicy::Fixed(psi), &seed)
                    {
                        found.insert(branch(&solution.q));
                    }
                }
            }
        }
        let goal = Goal::new(
            JOINTS,
            Vec::new(),
            vec![PositionTarget {
                frame: Frame::GraspPoint,
                position: pose.translation.vector,
                tolerance_m: POSITION_TOLERANCE_M,
            }],
            vec![OrientationTarget {
                frame: Frame::GraspPoint,
                orientation: pose.rotation,
                tolerance_rad: ORIENTATION_TOLERANCE_RAD,
                free_axis: None,
            }],
        )
        .expect("a valid goal");
        let reached: BTreeSet<(bool, bool)> = sampler
            .sample(&goal, GOAL_SAMPLES, pose_index as u64, an_hour_from_now())
            .iter()
            .map(branch)
            .collect();
        several += usize::from(found.len() > 1);
        if found.is_subset(&reached) {
            covered += 1;
        } else {
            missed.push((pose_index, found, reached));
        }
    }
    let share = covered as f64 / POSES as f64;
    println!(
        "covered {covered} of {POSES} poses, {several} with more than one branch; missed {:?}",
        missed.iter().take(5).collect::<Vec<_>>()
    );
    assert!(
        share >= COVERED_SHARE,
        "the generic sampler covers {share} of the poses"
    );
}

/// The reachable poses drawn for each link frame.
const LINK_POSES: usize = 10;
/// The configurations the sampler draws per link pose.
const LINK_SAMPLES: usize = 16;

// On drawn reachable poses of the forearm, of the wrist link, and of a part the hand
// carries off its frame, the sampler finds configurations whose frame meets the pose.
#[test]
fn the_sampler_reaches_the_poses_of_links_and_carried_parts() {
    let chain = LEFT_ARM.chain();
    let sampler = IkGoalSampler::new(chain.clone());
    let limits = chain.limits();
    let carried = Isometry3::from_parts(
        Translation3::new(0.0, 0.03, -0.1),
        UnitQuaternion::from_euler_angles(0.3, 0.0, 0.0),
    );
    let mut draws = Draws(20261029);
    for frame in [
        Frame::Link {
            segment: 3,
            offset: Isometry3::identity(),
        },
        Frame::Link {
            segment: 5,
            offset: Isometry3::identity(),
        },
        Frame::Link {
            segment: JOINTS - 1,
            offset: carried,
        },
    ] {
        for pose_index in 0..LINK_POSES {
            let q: [f64; JOINTS] = std::array::from_fn(|i| {
                limits[i].lo + (limits[i].hi - limits[i].lo) * draws.unit()
            });
            let Frame::Link { segment, offset } = frame else {
                unreachable!("each frame drawn is a link's")
            };
            let pose = chain.at(&q).link_pose_world(segment) * offset;
            let goal = Goal::new(
                JOINTS,
                Vec::new(),
                vec![PositionTarget {
                    frame,
                    position: pose.translation.vector,
                    tolerance_m: POSITION_TOLERANCE_M,
                }],
                vec![OrientationTarget {
                    frame,
                    orientation: pose.rotation,
                    tolerance_rad: ORIENTATION_TOLERANCE_RAD,
                    free_axis: None,
                }],
            )
            .expect("a valid goal");
            let reached =
                sampler.sample(&goal, LINK_SAMPLES, pose_index as u64, an_hour_from_now());
            assert!(
                !reached.is_empty(),
                "no configuration brings {frame:?} to its pose at {q:?}"
            );
            assert!(reached.iter().all(|q| goal.met(chain, q)));
        }
    }
}
