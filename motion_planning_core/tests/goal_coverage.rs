//! The generic goal sampler against OpenArm's closed form: on reachable poses, with the
//! configurations of every other shoulder and wrist branch invalid, one goal sample
//! reaches each branch that `srs_model` finds. It reaches the reachable poses of the
//! links above the grasp point, and of a part a link carries.

mod common;
mod openarm_v2;

use std::collections::BTreeSet;
use std::num::NonZeroUsize;

use chain_kinematics::Draws;
use motion_planning_core::{
    ArmGoal, Frame, Goal, GoalSampler, IkGoalSampler, OrientationTarget, PositionTarget,
    sample_goals,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use srs_model::ArmAnglePolicy;

use common::{LEFT_ARM, branch, left_srs_arm, positive};
use openarm_v2::JOINTS;

/// The poses drawn.
const POSES: usize = 200;
/// The working state each goal sample sets out from, every joint at 0: a start unrelated
/// to the drawn pose.
const START: [f64; JOINTS] = [0.0; JOINTS];
/// The attempts of a goal sample: the plan's, MoveIt's `max_state_sampling_attempts`
/// default (moveit2 2.15.2 planning_context_manager.cpp:259).
const ATTEMPTS: usize = 4;
/// The arm angles at which `srs_model` is asked for a solution on each branch.
const ARM_ANGLES: usize = 720;
/// How near the grasp point lies to the pose for the generic sampler's goal.
const POSITION_TOLERANCE_M: f64 = 0.005;
const ORIENTATION_TOLERANCE_RAD: f64 = 0.01;

// On each of 200 drawn reachable poses of OpenArm v2's left arm, for each branch that
// srs_model finds, one goal sample from a working state unrelated to the pose, every
// configuration off that branch invalid, reaches a goal on it: the goal sampling reaches
// a branch that the scene leaves valid when the branches its seeds lead to are not.
#[test]
fn one_goal_sample_reaches_each_branch_srs_model_finds_when_the_others_are_invalid() {
    let srs = left_srs_arm();
    let chain = LEFT_ARM.chain();
    let sampler = IkGoalSampler::new(chain.clone());
    let limits = chain.limits();
    let mut draws = Draws::new(20261028);
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
                tolerance_m: positive(POSITION_TOLERANCE_M),
            }],
            vec![OrientationTarget {
                frame: Frame::GraspPoint,
                orientation: pose.rotation,
                tolerance_rad: positive(ORIENTATION_TOLERANCE_RAD),
                free_axis: None,
            }],
        )
        .expect("a valid goal");
        let reached: BTreeSet<(bool, bool)> = found
            .iter()
            .copied()
            .filter(|&only| {
                sampler
                    .sample_from(
                        &goal,
                        &START,
                        ATTEMPTS,
                        &mut Draws::new(pose_index as u64),
                        &|q| branch(q) == only,
                    )
                    .is_some()
            })
            .collect();
        several += usize::from(found.len() > 1);
        if found == reached {
            covered += 1;
        } else {
            missed.push((pose_index, found, reached));
        }
    }
    println!("covered {covered} of {POSES} poses, {several} with more than one branch");
    assert!(
        missed.is_empty(),
        "one goal sample misses a branch on {} poses: {:?}",
        missed.len(),
        missed.iter().take(5).collect::<Vec<_>>()
    );
}

/// The reachable poses drawn for each link frame.
const LINK_POSES: usize = 10;
/// The goals sampled per link pose.
const LINK_GOALS: usize = 4;

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
    let mut draws = Draws::new(20261029);
    for frame in [
        Frame::Link {
            segment: 3,
            offset: Isometry3::identity(),
        },
        Frame::Link {
            segment: 5,
            offset: Isometry3::identity(),
        },
        Frame::Part {
            segment: JOINTS - 1,
            offset: carried,
        },
    ] {
        for pose_index in 0..LINK_POSES {
            let q: [f64; JOINTS] = std::array::from_fn(|i| {
                limits[i].lo + (limits[i].hi - limits[i].lo) * draws.unit()
            });
            let (Frame::Link { segment, offset } | Frame::Part { segment, offset }) = frame else {
                unreachable!("each frame drawn is a link's or a part's")
            };
            let pose = chain.at(&q).link_pose_world(segment) * offset;
            let goal = Goal::new(
                JOINTS,
                Vec::new(),
                vec![PositionTarget {
                    frame,
                    position: pose.translation.vector,
                    tolerance_m: positive(POSITION_TOLERANCE_M),
                }],
                vec![OrientationTarget {
                    frame,
                    orientation: pose.rotation,
                    tolerance_rad: positive(ORIENTATION_TOLERANCE_RAD),
                    free_axis: None,
                }],
            )
            .expect("a valid goal");
            let reached = sample_goals(
                &[ArmGoal {
                    goal: &goal,
                    sampler: &sampler,
                }],
                &START,
                NonZeroUsize::new(LINK_GOALS).expect("nonzero"),
                pose_index as u64,
                &|| false,
                &|_, _| true,
            );
            assert!(
                !reached.is_empty(),
                "no configuration brings {frame:?} to its pose at {q:?}"
            );
            assert!(reached.iter().all(|q| goal.met(chain, q)));
        }
    }
}
