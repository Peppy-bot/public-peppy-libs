//! The scenes of the ROS reference harness's arm planning scenario as this crate plans
//! them: the arm, the parts its links carry, the robot around it that its plans do not
//! move, each scene's map, its planning scene of objects, of the object the arm holds and
//! of the one the other arm holds, with its allowed pairs, and its plan. The example
//! `plan_arm_scenes` and the parity test share them, so the paths the scenario carries
//! are the ones the test plans. The planning scene parity test shares the reading of a
//! harness object as an entry of apply_collision_objects.
#![allow(dead_code)]

use std::collections::BTreeSet;
use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::path::Path;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, AttachRequest, Goal, IkGoalSampler, JointTarget, ObjectFields, ObjectRequest, PlanFailure,
    PlanRequest, PlanningScene, RobotLinks, RobotPart, Scene, ShapeFields, StartFault,
    plan_arm_path, pose_from_parts, pose_parts,
};
use nalgebra::Isometry3;
use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};
use ompl_bridge::Planner;
use openarm_description::{ArmPair, HardwareVersion, Side};
use ros_reference::arm_planning::{
    Objective, Ours, Scene as ReferenceScene, Scenes, Termination, VoxelBox,
};
use ros_reference::planning_scene::{Object, Shape};
use self_collision_model::SelfCollisionModel;

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
/// MoveIt's default state holds them; and the robot around it.
pub fn on_the_arm<R>(
    scenes: &Scenes,
    urdf: &str,
    meshes: &Path,
    with: impl FnOnce(&Arm<JOINTS>, &mut Standing) -> R,
) -> R {
    let geometry = geometry(scenes, urdf, meshes);
    let riders = parts(&geometry);
    let arm = Arm::new(&geometry, None)
        .riding(&riders)
        .expect("the parts ride the arm's links");
    let mut standing = Standing::new(urdf, meshes, &scenes.tip_link);
    with(&arm, &mut standing)
}

/// The robot around the scenario's arm, which its plans do not move: OpenArm v2's self
/// model, whose bodies off the arm stand as the parts of the robot a plan meets, the
/// other arm at its zero configuration, as MoveIt's default state holds it; and the URDF,
/// whose chain to the other arm's grasp link places the object that link holds.
pub struct Standing {
    model: SelfCollisionModel,
    /// The side of the arm the scenario plans.
    planned: Side,
    urdf: String,
    meshes: String,
}

impl Standing {
    /// The robot around the arm whose tip link is `tip_link`, either grasp link of OpenArm
    /// v2: `parts` places that arm of the self model at a plan's start and the other arm at
    /// zero.
    fn new(urdf: &str, meshes: &Path, tip_link: &str) -> Self {
        let planned = Side::BOTH
            .into_iter()
            .find(|&side| HardwareVersion::V2.tcp_link(side) == tip_link)
            .unwrap_or_else(|| {
                panic!(
                    "the scenario plans an arm of OpenArm v2, whose tip link is {} or {}, and \
                     its tip link is {tip_link}",
                    HardwareVersion::V2.tcp_link(Side::Left),
                    HardwareVersion::V2.tcp_link(Side::Right)
                )
            });
        let meshes = meshes.to_str().expect("a UTF-8 mesh directory").to_string();
        Self {
            model: self_collision_model::openarm(HardwareVersion::V2, &meshes)
                .expect("the self model of OpenArm v2"),
            planned,
            urdf: urdf.to_string(),
            meshes,
        }
    }

    /// The pose of the grasp link `link` of the other arm, at its zero configuration.
    fn other_grasp_pose(&self, link: &str) -> Isometry3<f64> {
        ChainGeometry::<JOINTS>::from_urdf(
            &self.urdf,
            &self.meshes,
            &ChainSpec {
                base_link: None,
                tip_link: link,
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the other arm's chain")
        .chain()
        .world_ee_pose(&[0.0; JOINTS])
    }

    /// The pieces of the bodies of the robot that `arm` at `start` does not move, the
    /// other arm at its zero configuration: each but the arm's links and the parts they
    /// carry.
    pub fn parts<'s>(&'s mut self, arm: &Arm<JOINTS>, start: &[f64; JOINTS]) -> Vec<RobotPart<'s>> {
        let geometry = arm.geometry();
        let moving: BTreeSet<&str> = geometry
            .links()
            .iter()
            .map(|link| link.name())
            .chain(geometry.carried().iter().map(|part| part.rider().name))
            .collect();
        let planned = self.planned;
        let q = ArmPair::from_fn(|side| {
            if side == planned {
                *start
            } else {
                [0.0; JOINTS]
            }
        });
        self.model
            .world_pieces(&q[Side::Left], &q[Side::Right])
            .expect("finite joint positions")
            .into_iter()
            .filter(|(link, _)| !moving.contains(link))
            .flat_map(|(link, pieces)| {
                pieces
                    .into_iter()
                    .map(move |piece| RobotPart { link, piece })
            })
            .collect()
    }
}

/// The parts the links of `geometry`'s chain carry, as riders.
fn parts(geometry: &ChainGeometry<JOINTS>) -> Vec<Rider<'_>> {
    geometry.carried().iter().map(CarriedPart::rider).collect()
}

/// The fields of apply_collision_objects that a harness shape writes.
fn shape_fields(shape: &Shape) -> ShapeFields {
    ShapeFields {
        kind: shape.kind.clone(),
        dimensions: shape.dimensions.clone(),
        mesh_vertices: shape.mesh_vertices.clone(),
        mesh_triangles: shape.mesh_triangles.clone(),
        position: shape.position,
        orientation: shape.orientation,
    }
}

/// The entry of apply_collision_objects that a harness object writes.
pub fn object_request(object: &Object) -> ObjectRequest {
    ObjectRequest {
        operation: object.operation.clone(),
        object: ObjectFields {
            id: object.id.clone(),
            position: object.position,
            orientation: object.orientation,
            shapes: object.shapes.iter().map(shape_fields).collect(),
        },
    }
}

/// The entry of apply_collision_objects that adds `held`, whose pose is in the frame of
/// a link at `link_pose`, where that link holds it.
fn held_request(held: &Object, link_pose: &Isometry3<f64>) -> ObjectRequest {
    let mut request = object_request(held);
    let in_the_link = pose_from_parts(request.object.position, request.object.orientation)
        .expect("the held object's pose in its link's frame");
    let (position, orientation) = pose_parts(&(link_pose * in_the_link));
    request.object.position = position;
    request.object.orientation = orientation;
    request
}

/// The planning scene of `scene` on `map`, which its objects clear where they stand: its
/// objects; the object it attaches to the arm's tip link with the arm at its start, and
/// the one it attaches to the other arm's grasp link, at its zero configuration where
/// `other_grasp` places that link, each at its pose in its link's frame; and its allowed
/// pairs.
fn planning_scene_of<O>(
    arm: &Arm<JOINTS>,
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
    other_grasp: Option<Isometry3<f64>>,
    map: &mut VoxelMap,
) -> PlanningScene {
    let tree = arm.geometry().chain().tree();
    let links = (0..tree.link_count()).map(|i| tree.link(i).name.clone());
    let other_link = scene
        .other_arm_holds
        .as_ref()
        .map(|_| scenes.other_tip_link());
    let grasp_links = std::iter::once(scenes.tip_link.clone()).chain(other_link.clone());
    let mut planning = PlanningScene::new(RobotLinks::new(links, grasp_links));
    let tip = arm
        .geometry()
        .chain()
        .world_ee_pose(&configuration(&scene.start));
    let held = scene
        .attached
        .iter()
        .map(|held| held_request(&held.object, &tip));
    let other = scene.other_arm_holds.iter().map(|other| {
        held_request(
            &other.object,
            &other_grasp.expect("the other arm's grasp link placed"),
        )
    });
    let requests: Vec<ObjectRequest> = scene
        .objects
        .iter()
        .map(object_request)
        .chain(held)
        .chain(other)
        .collect();
    planning
        .apply_collision_objects(&requests, map)
        .expect("the scene's objects apply");
    let attachments = scene
        .attached
        .iter()
        .map(|held| (&held.object.id, &scenes.tip_link, &held.touch_links, tip))
        .chain(
            other_link
                .iter()
                .zip(&scene.other_arm_holds)
                .map(|(link, other)| {
                    let at = other_grasp.expect("the other arm's grasp link placed");
                    (&other.object.id, link, &other.touch_links, at)
                }),
        );
    for (id, link, touch_links, link_pose) in attachments {
        planning
            .apply_attached_collision_object(
                &AttachRequest {
                    object_id: id.clone(),
                    link_name: link.clone(),
                    touch_links: touch_links.clone(),
                    operation: "attach".to_string(),
                },
                |_| Some(link_pose),
                map,
            )
            .expect("the held object attaches to its link");
    }
    for [name, other] in &scene.allowed {
        planning
            .set_allowed_collision(name, std::slice::from_ref(other), true)
            .expect("the scene's pair is allowed");
    }
    planning
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

/// The occupied voxels of `map` as boxes, each run of them along x one box: the boxes
/// whose voxels [`map_of`] fills a scene's map with.
pub fn boxes_of(map: &VoxelMap) -> Vec<VoxelBox> {
    let mut keys: Vec<[i32; 3]> = map.occupied().map(|key| key.0).collect();
    keys.sort_by_key(|k| (k[1], k[2], k[0]));
    keys.into_iter()
        .fold(Vec::new(), |mut boxes: Vec<VoxelBox>, key| {
            match boxes.last_mut() {
                Some(run) if run.last[1..] == key[1..] && run.last[0] + 1 == key[0] => {
                    run.last = key;
                }
                _ => boxes.push(VoxelBox {
                    first: key,
                    last: key,
                }),
            }
            boxes
        })
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

/// The planner of `objective`: RRTConnect for `any`, as the motion planner node plans it,
/// and RRTstar for `shortest`.
fn planner_of(objective: Objective) -> Planner {
    match objective {
        Objective::Any => Planner::RrtConnect,
        Objective::Shortest => Planner::RrtStar,
    }
}

/// The iteration bound of this crate's plans of `scenes`, as their termination says.
fn max_iterations(scenes: &Scenes) -> NonZeroU64 {
    let bound = NonZeroU64::new(scenes.iterations).expect("nonzero");
    match (scenes.termination, scenes.objective) {
        (Termination::Iterations, _) | (Termination::Time, Objective::Any) => bound,
        (Termination::Time, Objective::Shortest) => NonZeroU64::MAX,
    }
}

/// This crate's answer to a scene, and whether its plan returned at or after its
/// deadline, the scenes' time limit after the plan started.
pub struct Planned {
    pub ours: Ours,
    pub at_the_deadline: bool,
}

/// This crate's answer to `scene`, among the voxels its boxes occupy, its objects and the
/// parts of `standing` the arm does not move, holding the object it attaches: `arm`'s
/// parts ride its links, and the held object its tip link.
pub fn plan_scene<O>(
    arm: &Arm<JOINTS>,
    standing: &mut Standing,
    sampler: &IkGoalSampler<JOINTS>,
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    let mut map = map_of(scenes, scene);
    let other_grasp = scene
        .other_arm_holds
        .as_ref()
        .map(|_| standing.other_grasp_pose(&scenes.other_tip_link()));
    let planning = planning_scene_of(arm, scenes, scene, other_grasp, &mut map);
    let field = DistanceField::of(&map);
    let objects = planning.for_arm(&scenes.tip_link, |_| {
        other_grasp.expect("the other arm's grasp link placed")
    });
    let standing_parts = standing.parts(arm, &configuration(&scene.start));
    let geometry = arm.geometry();
    let riders: Vec<Rider> = parts(geometry)
        .into_iter()
        .chain(objects.riders(geometry))
        .collect();
    let arm = &Arm::new(geometry, None)
        .riding(&riders)
        .expect("the parts and the held object ride the arm's links");
    let goal_q = configuration(&scene.goal);
    let tolerance =
        PositiveFinite::parse(scenes.goal_tolerance).expect("the scenes' goal tolerance");
    let goal = Goal::new(
        JOINTS,
        (0..JOINTS)
            .map(|joint| JointTarget {
                joint,
                position: goal_q[joint],
                above: tolerance,
                below: tolerance,
            })
            .collect(),
        Vec::new(),
        Vec::new(),
    )
    .expect("a valid goal");
    let deadline = Instant::now() + Duration::from_secs_f64(scenes.time_limit);
    let answer = plan_arm_path(
        arm,
        &Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: objects.objects(&standing_parts),
        },
        &PlanRequest {
            start: configuration(&scene.start),
            start_tolerance: [0.0; JOINTS],
            goal: &goal,
            sampler,
            goal_samples: NonZeroUsize::new(GOAL_SAMPLES).expect("nonzero"),
            planner: planner_of(scenes.objective),
            deadline,
            max_iterations: max_iterations(scenes),
            attempts: NonZeroU32::MIN,
            seed: scene.seed.get(),
            min_clearance_m: scenes.min_clearance,
            stop: None,
        },
    );
    let at_the_deadline = Instant::now() >= deadline;
    let failure = |name: &str| Ours::Failure {
        failure: name.to_string(),
    };
    let ours = match answer {
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
    };
    Planned {
        ours,
        at_the_deadline,
    }
}
