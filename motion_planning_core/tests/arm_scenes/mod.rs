//! The scenes of the ROS reference harness's arm planning scenario as this crate plans
//! them: the arm, the parts its links carry, each scene's map and its plan. The example
//! `plan_arm_scenes` and the parity test share them, so the paths the scenario carries
//! are the ones the test plans.
#![allow(dead_code)]

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::path::Path;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, Goal, IkGoalSampler, JointTarget, PlanFailure, PlanRequest, Scene, StartFault,
    plan_arm_path,
};
use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};
use ompl_bridge::Planner;
use ros_reference::arm_planning::{Ours, Scene as ReferenceScene, Scenes};

pub const JOINTS: usize = 7;
/// The configurations the goal sampler draws for a scene's joint goal.
const GOAL_SAMPLES: usize = 8;
/// The share of a voxel a link point moves at most between two probes of a path.
const PROBE_SHARE_OF_VOXEL: f64 = 0.1;
/// The frame of OpenArm's URDF root, in which the scenes' boxes stand.
const ROOT_FRAME: &str = "world";
/// The range of the scan that fills a scene's map, from the root: beyond every voxel of
/// the map.
const MAX_RANGE_M: f64 = 10.0;

/// The collision pieces of the scenario's arm, from the URDF text `urdf`, whose meshes
/// lie in `meshes`.
pub fn geometry(scenes: &Scenes, urdf: &str, meshes: &Path) -> ChainGeometry<JOINTS> {
    let joints: Vec<&str> = scenes.joints.iter().map(String::as_str).collect();
    ChainGeometry::from_urdf(
        urdf,
        meshes.to_str().expect("a UTF-8 mesh directory"),
        &ChainSpec {
            base_link: Some(&scenes.base_link),
            tip_link: &scenes.tip_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .expect("the arm fits")
}

/// `with` run on the scenario's arm, from the URDF text `urdf` whose meshes lie in
/// `meshes`: its chain, and the parts its links carry, its gripper's fingers closed, as
/// MoveIt's default state holds them.
pub fn on_the_arm<R>(
    scenes: &Scenes,
    urdf: &str,
    meshes: &Path,
    with: impl FnOnce(&Arm<JOINTS>) -> R,
) -> R {
    let geometry = geometry(scenes, urdf, meshes);
    let riders: Vec<Rider> = geometry.carried().iter().map(CarriedPart::rider).collect();
    let arm = Arm::new(&geometry, None)
        .riding(&riders)
        .expect("the parts ride the arm's links");
    with(&arm)
}

/// The map of the voxels of `scene`'s boxes at the scenario's voxel size, each occupied.
pub fn map_of<O>(scenes: &Scenes, scene: &ReferenceScene<O>) -> VoxelMap {
    let size = scenes.voxel_size;
    let half = scenes.map_half_side;
    let frame = Frame::parse(ROOT_FRAME).expect("a frame name");
    let mut map = VoxelMap::new(
        frame.clone(),
        PositiveFinite::parse(size).expect("a positive voxel size"),
        Bounds {
            min: [-half; 3],
            max: [half; 3],
        },
    )
    .expect("a valid map");
    let points: Vec<[f32; 3]> = scene
        .boxes
        .iter()
        .flat_map(|b| {
            (b.first[0]..=b.last[0]).flat_map(move |x| {
                (b.first[1]..=b.last[1])
                    .flat_map(move |y| (b.first[2]..=b.last[2]).map(move |z| [x, y, z]))
            })
        })
        .map(|key| key.map(|k| ((f64::from(k) + 0.5) * size) as f32))
        .collect();
    map.insert(
        &Scan {
            frame: &frame,
            sensor: Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose"),
            points: &points,
            max_range_m: PositiveFinite::parse(MAX_RANGE_M).expect("a positive range"),
        },
        &[],
    )
    .expect("the scan stands in the map's frame");
    map
}

/// The joints' L2 distance between two probes of a path at which no point of `arm`'s
/// links or of its riders moves more than [`PROBE_SHARE_OF_VOXEL`] of a voxel: the
/// reference probes every path at it.
pub fn probe_spacing(arm: &Arm<JOINTS>, scenes: &Scenes) -> f64 {
    scenes.voxel_size * PROBE_SHARE_OF_VOXEL / arm.lever()
}

fn configuration(values: &[f64]) -> [f64; JOINTS] {
    values.try_into().expect("one position per joint")
}

/// This crate's answer to `scene`, among the voxels its boxes occupy.
pub fn plan_scene<O>(
    arm: &Arm<JOINTS>,
    sampler: &IkGoalSampler<JOINTS>,
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Ours {
    let map = map_of(scenes, scene);
    let field = DistanceField::of(&map);
    let goal_q = configuration(&scene.goal);
    let goal = Goal::new(
        JOINTS,
        (0..JOINTS)
            .map(|joint| JointTarget {
                joint,
                position: goal_q[joint],
                above: scenes.goal_tolerance,
                below: scenes.goal_tolerance,
            })
            .collect(),
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let answer = plan_arm_path(
        arm,
        &Scene {
            map: &map,
            field: &field,
            stamp: 0,
        },
        &PlanRequest {
            start: configuration(&scene.start),
            start_tolerance: [0.0; JOINTS],
            goal: &goal,
            sampler,
            goal_samples: NonZeroUsize::new(GOAL_SAMPLES).expect("nonzero"),
            planner: Planner::RrtConnect,
            deadline: Instant::now() + Duration::from_secs_f64(scenes.time_limit),
            max_iterations: NonZeroU64::new(scenes.iterations).expect("nonzero"),
            attempts: NonZeroU32::MIN,
            seed: scene.seed.get(),
            min_clearance_m: scenes.min_clearance,
            stop: None,
        },
    );
    let failure = |name: &str| Ours::Failure {
        failure: name.to_string(),
    };
    match answer {
        Ok(waypoints) => Ours::Path {
            waypoints: waypoints.iter().map(|q| q.to_vec()).collect(),
        },
        Err(PlanFailure::Start(StartFault::OutsideLimits { .. })) => {
            failure("start_outside_limits")
        }
        Err(PlanFailure::Start(StartFault::InCollision(_))) => failure("start_in_collision"),
        Err(PlanFailure::Unreachable(_)) => failure("unreachable"),
        Err(PlanFailure::NoPath(_)) => failure("no_path"),
        Err(
            failure @ (PlanFailure::Request(_) | PlanFailure::Planner(_) | PlanFailure::Stopped),
        ) => {
            panic!("the scene's request fails: {failure}")
        }
    }
}
