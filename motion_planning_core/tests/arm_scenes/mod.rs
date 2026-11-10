//! The scenes of the ROS reference harness's arm planning scenarios as this crate plans
//! them: the arms, the parts their links carry, the robot around them that their plans do
//! not move, OpenArm v2 or SO-101, with its self model where a scenario plans every chain
//! of the robot, each scene's map, its planning scene of objects and of the objects the
//! robot's grasp links hold, with its allowed pairs, and its plan. The example `plan_arm_scenes` and the parity
//! test share them, so the paths, the checked pairs and the probe spacing each scenario
//! carries are the ones the test plans. The planning scene parity test shares the reading
//! of a harness object as an entry of apply_collision_objects, and the Cartesian paths
//! scenes (`cartesian_scenes`) share the arms and a scene's parts as this crate meets them.
#![allow(dead_code)]

use std::collections::BTreeSet;
use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::path::Path;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, ArmGoal, Arms, AttachRequest, Axis, Goal, IkGoalSampler, JointTarget, Moving,
    ObjectFields, ObjectRequest, OrientationTarget, PathOrientation, PlanFailure, PlanRequest,
    PlanningScene, PositionTarget, RobotLinks, RobotPart, Scene, SelfModel, ShapeFields,
    StartFault, orientation_from_parts, plan_path, pose_from_parts, pose_parts,
};
use nalgebra::Isometry3;
use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};
use ompl_bridge::Planner;
use openarm_description::{ARM_DOF, ARMS, DUAL_DOF, HardwareVersion, Side};
use ros_reference::arm_planning::{
    ArmChain, Held, HeldOrientation, Objective, Ours, Scene as ReferenceScene, SceneGoal, Scenes,
    Termination, VoxelBox,
};
use ros_reference::planning_scene::{Object, Shape};
use self_collision_model::{CollisionError, Joints, Proximity, SelfCollisionModel};

/// The joints of an arm of OpenArm v2, which the scenarios that name no other robot plan.
pub const JOINTS: usize = ARM_DOF;
/// The fewest scenes of a drawn set whose statistics a parity test compares: where the
/// normal approximation of the arm planning test's intervals holds.
pub const MIN_SCENES: usize = 30;
/// The most goals a scene's plan samples: the motion planner's launch's (launchers-hub
/// openarm/fragments/motion_planner.json5, `goal_samples`).
const LAUNCH_GOAL_SAMPLES: usize = 10;
/// The share of a voxel a link point moves at most between two probes of a path.
const PROBE_SHARE_OF_VOXEL: f64 = 0.1;
/// The name of the frame of a scene's map, in which its boxes stand.
const ROOT_FRAME: &str = "world";
/// The range of the scan that fills a scene's map, from the root: beyond every voxel of
/// the map.
const MAX_RANGE_M: f64 = 10.0;
/// The opening of OpenArm v2's grippers in its self model: closed, where its finger joints
/// stand at 0, as MoveIt's default state holds them.
const FINGERS_CLOSED: f64 = 0.0;

/// What runs at the joints of a scenario's arms, `N` an arm's and `D` those of them all.
pub trait AtItsJoints {
    type Output;
    fn run<const N: usize, const D: usize>(self) -> Self::Output;
}

/// `at` at the joints of the arms of `chains`.
pub fn at_its_joints<A: AtItsJoints>(chains: &[ArmChain], at: A) -> A::Output {
    match chains[0].joints.len() {
        JOINTS => at.run::<JOINTS, DUAL_DOF>(),
        so101_description::ARM_DOF => {
            at.run::<{ so101_description::ARM_DOF }, { so101_description::ARM_DOF }>()
        }
        count => panic!("a scenario plans arms of 7 or 5 joints, and this one names {count}"),
    }
}

/// The collision pieces of the arm of `chain`, from the URDF text `urdf`, whose meshes
/// lie in `meshes`.
pub fn geometry<const N: usize>(chain: &ArmChain, urdf: &str, meshes: &Path) -> ChainGeometry<N> {
    let joints: Vec<&str> = chain.joints.iter().map(String::as_str).collect();
    ChainGeometry::from_urdf(
        urdf,
        meshes.to_str().expect("a UTF-8 mesh directory"),
        &ChainSpec {
            base_link: Some(&chain.base_link),
            tip_link: &chain.tip_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .expect("the arm fits")
}

/// `with` run on the arms of `chains`, from the URDF text `urdf` whose meshes lie in
/// `meshes`, in their order: each arm by its chain's name, with the parts its links carry,
/// its gripper's fingers closed, as MoveIt's default state holds them; and the robot around
/// them.
pub fn on_the_arms<const N: usize, R>(
    chains: &[ArmChain],
    urdf: &str,
    meshes: &Path,
    with: impl FnOnce(&[Arm<N>], &mut Standing) -> R,
) -> R {
    let geometries: Vec<ChainGeometry<N>> = chains
        .iter()
        .map(|chain| geometry(chain, urdf, meshes))
        .collect();
    let riders: Vec<Vec<Rider>> = geometries.iter().map(parts).collect();
    let arms: Vec<Arm<N>> = chains
        .iter()
        .zip(&geometries)
        .zip(&riders)
        .map(|((chain, geometry), riders)| {
            Arm::new(&chain.name, geometry)
                .riding(riders)
                .expect("the parts ride the arm's links")
        })
        .collect();
    let mut standing = Standing::new(urdf, meshes, chains);
    with(&arms, &mut standing)
}

/// Each arm's goal sampler, of the arms `arms`.
pub fn samplers<const N: usize>(arms: &[Arm<N>]) -> Vec<IkGoalSampler<N>> {
    arms.iter()
        .map(|arm| IkGoalSampler::new(arm.geometry().chain().clone()))
        .collect()
}

/// The robot around the scenario's arms, which their plans do not move: OpenArm v2 or
/// SO-101, whichever robot's chains the scenario's chains are.
pub enum Standing {
    OpenArm(Box<StandingRobot<JOINTS, ARMS>>),
    So101(Box<StandingRobot<{ so101_description::ARM_DOF }, 1>>),
}

impl Standing {
    /// The robot around the arms of `chains`, from the URDF text `urdf` whose meshes lie in
    /// `meshes`.
    pub fn new(urdf: &str, meshes: &Path, chains: &[ArmChain]) -> Self {
        let meshes = meshes.to_str().expect("a UTF-8 mesh directory");
        let version = HardwareVersion::V2;
        let openarm_ends = Side::BOTH.map(|side| [version.base_link(side), version.tcp_link(side)]);
        if let Some(planned) = planned_chains(chains, &openarm_ends) {
            assert!(
                urdf == version.urdf(),
                "the scenario of OpenArm v2's chains names OpenArm v2's URDF"
            );
            let model = self_collision_model::openarm(version, meshes)
                .expect("the self model of OpenArm v2");
            return Self::OpenArm(Box::new(StandingRobot::new(
                model,
                openarm_ends,
                planned,
                FINGERS_CLOSED,
                urdf,
                meshes,
            )));
        }
        let so101_ends = [[so101_description::BASE_LINK, so101_description::TCP_LINK]];
        let planned = planned_chains(chains, &so101_ends).unwrap_or_else(|| {
            panic!(
                "the scenario's chains are chains of OpenArm v2 or of SO-101, each from its \
                 base link to its grasp link"
            )
        });
        assert!(
            urdf == so101_description::URDF,
            "the scenario of SO-101's chain names SO-101's URDF"
        );
        let model = self_collision_model::so101(meshes).expect("the self model of SO-101");
        Self::So101(Box::new(StandingRobot::new(
            model,
            so101_ends,
            planned,
            // The jaw where its joint stands at 0, as MoveIt's default state holds it.
            so101_description::jaw_opening(0.0),
            urdf,
            meshes,
        )))
    }

    /// The pairs of the robot's own links the scenario's plans check.
    pub fn checked_pairs(&self) -> Vec<[String; 2]> {
        match self {
            Self::OpenArm(robot) => robot.checked_pairs(),
            Self::So101(robot) => robot.checked_pairs(),
        }
    }

    /// The least distance between the robot's own bodies, of the pairs a plan of every
    /// chain of the robot checks, the arms at `state`, which holds their joints in their
    /// order.
    pub fn self_clearance(&self, state: &[f64]) -> f64 {
        match self {
            Self::OpenArm(robot) => robot.self_clearance(state),
            Self::So101(robot) => robot.self_clearance(state),
        }
    }

    /// The pose of `link`, the grasp link of an arm of no chain of the scenario, at that
    /// arm's zero configuration.
    fn idle_grasp_pose(&self, link: &str) -> Isometry3<f64> {
        match self {
            Self::OpenArm(robot) => robot.idle_grasp_pose(link),
            Self::So101(robot) => robot.idle_grasp_pose(link),
        }
    }

    /// What a plan of `arms` from `start`, which holds their joints in their order, meets
    /// of the robot: the pieces of its bodies that the arms do not move; and the self model
    /// of the plan where the scenario plans every chain of the robot.
    pub fn around<'s, const N: usize, const D: usize>(
        &'s mut self,
        arms: &[Arm<N>],
        start: &[f64; D],
    ) -> (Vec<RobotPart<'s>>, Option<&'s dyn SelfModel<D>>) {
        let moving: BTreeSet<&str> = arms
            .iter()
            .flat_map(|arm| {
                let geometry = arm.geometry();
                geometry
                    .links()
                    .iter()
                    .map(|link| link.name())
                    .chain(geometry.carried().iter().map(|part| part.rider().name))
            })
            .collect();
        match self {
            Self::OpenArm(robot) => robot.around(&moving, start),
            Self::So101(robot) => robot.around(&moving, start),
        }
    }
}

/// The chain of the robot whose base and tip links `ends` holds, in its self model's
/// order, of each of `chains`, in their order; none where a chain is no chain of the robot.
fn planned_chains<const K: usize>(
    chains: &[ArmChain],
    ends: &[[&str; 2]; K],
) -> Option<Vec<usize>> {
    chains
        .iter()
        .map(|chain| {
            ends.iter()
                .position(|[base, tip]| chain.base_link == *base && chain.tip_link == *tip)
        })
        .collect()
}

/// Every chain of a robot of `K` chains of `N` joints at its zero configuration, but those
/// of `planned`, the chain of each arm of a scenario in its order, at their joints of
/// `state`, which holds the arms' joints in turn.
fn placed<const N: usize, const K: usize>(planned: &[usize], state: &[f64]) -> Joints<N, K> {
    std::array::from_fn(|chain| {
        planned
            .iter()
            .position(|of| *of == chain)
            .map_or([0.0; N], |k| {
                state[k * N..(k + 1) * N]
                    .try_into()
                    .expect("a state holds the joints of each arm")
            })
    })
}

/// A robot of `K` chains of `N` joints around the scenario's arms: its self model, whose
/// bodies off the arms stand as the parts of the robot a plan meets, a chain of no arm at
/// its zero configuration; where the scenario plans every chain, the self model of its
/// plans; and the URDF, whose chain to the grasp link of a chain of no arm places the
/// objects that link holds.
pub struct StandingRobot<const N: usize, const K: usize> {
    model: SelfCollisionModel<N, K>,
    /// The base and grasp links of each chain, in the self model's order.
    ends: [[&'static str; 2]; K],
    /// The chain of each arm of the scenario, in its order.
    planned: Vec<usize>,
    own_body: Option<OwnBody<N, K>>,
    urdf: String,
    meshes: String,
}

impl<const N: usize, const K: usize> StandingRobot<N, K> {
    /// The robot of `model`, whose chains run between the links of `ends`, around the arms
    /// on its chains `planned`, its fingers at the opening `fingers`.
    fn new(
        model: SelfCollisionModel<N, K>,
        ends: [[&'static str; 2]; K],
        planned: Vec<usize>,
        fingers: f64,
        urdf: &str,
        meshes: &str,
    ) -> Self {
        let own_body = (planned.len() == K).then(|| OwnBody::new(&model, &planned, fingers));
        Self {
            model,
            ends,
            planned,
            own_body,
            urdf: urdf.to_string(),
            meshes: meshes.to_string(),
        }
    }

    /// The pairs of the robot's own links the scenario's plans check: the self model's
    /// where it plans every chain, none where it plans some, which this crate plans
    /// without the self model.
    fn checked_pairs(&self) -> Vec<[String; 2]> {
        self.own_body
            .as_ref()
            .map_or_else(Vec::new, OwnBody::checked_pairs)
    }

    /// The least distance between the robot's own bodies, of the pairs the self model of a
    /// plan of every chain checks, the arms at `state`, which holds their joints in their
    /// order.
    fn self_clearance(&self, state: &[f64]) -> f64 {
        self.own_body
            .as_ref()
            .expect("the scenario plans every chain of the robot")
            .clearance(state, Moving::All)
    }

    /// The pose of `link`, the grasp link of a chain of no arm of the scenario, at that
    /// chain's zero configuration.
    fn idle_grasp_pose(&self, link: &str) -> Isometry3<f64> {
        assert!(
            self.ends
                .iter()
                .enumerate()
                .any(|(chain, [_, tip])| !self.planned.contains(&chain) && *tip == link),
            "{link} is the grasp link of a chain of no arm of the scenario"
        );
        ChainGeometry::<N>::from_urdf(
            &self.urdf,
            &self.meshes,
            &ChainSpec {
                base_link: None,
                tip_link: link,
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the chain to the grasp link")
        .chain()
        .world_ee_pose(&[0.0; N])
    }

    /// What a plan from `start`, which holds the joints of its arms in their order, meets of
    /// the robot: the pieces of its bodies that the arms do not move, a chain of no arm at
    /// its zero configuration, each body but `moving`, the arms' links and the parts they
    /// carry; and the self model of the plan where the scenario plans every chain.
    fn around<'s, const D: usize>(
        &'s mut self,
        moving: &BTreeSet<&str>,
        start: &[f64; D],
    ) -> (Vec<RobotPart<'s>>, Option<&'s dyn SelfModel<D>>) {
        let at = placed::<N, K>(&self.planned, start);
        let parts = self
            .model
            .world_pieces(&at)
            .expect("finite joint positions")
            .into_iter()
            .filter(|(link, _)| !moving.contains(link))
            .flat_map(|(link, pieces)| {
                pieces
                    .into_iter()
                    .map(move |piece| RobotPart { link, piece })
            })
            .collect();
        let own_body = self
            .own_body
            .as_ref()
            .map(|own_body| own_body as &dyn SelfModel<D>);
        (parts, own_body)
    }
}

/// A robot's self model as a plan of every chain of the robot meets it: its checked pairs,
/// each chain at the joints of its arm of the state asked about, and the fingers at the
/// opening at which MoveIt's default state holds them.
struct OwnBody<const N: usize, const K: usize> {
    model: Mutex<SelfCollisionModel<N, K>>,
    /// The chain of each arm of the plan, in its order.
    planned: Vec<usize>,
}

impl<const N: usize, const K: usize> OwnBody<N, K> {
    fn new(model: &SelfCollisionModel<N, K>, planned: &[usize], fingers: f64) -> Self {
        const {
            assert!(
                K <= 2,
                "a turn of one arm reads the last arm's pairs, and the first's of two"
            )
        };
        let mut model = model.clone();
        model.set_gripper_openings(&[fingers; K]);
        Self {
            model: Mutex::new(model),
            planned: planned.to_vec(),
        }
    }

    /// The model's checked pairs, by their links' names.
    fn checked_pairs(&self) -> Vec<[String; 2]> {
        let model = self.model.lock().expect("the self model");
        model
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| [a.to_string(), b.to_string()])
            .collect()
    }

    /// Runs `read` on the nearest checked pair that `moving` judges, the arms of the plan
    /// at the state `q`: the arm the plan holds last reads its pairs with every chain, and
    /// the first of two its pairs without the other.
    fn nearest<T>(
        &self,
        q: &[f64],
        moving: Moving,
        read: impl FnOnce(Result<Proximity<'_>, CollisionError>) -> T,
    ) -> T {
        let at = placed::<N, K>(&self.planned, q);
        let last = self.planned.len() - 1;
        let mut model = self.model.lock().expect("the self model");
        read(match moving {
            Moving::All => model.min_distance(&at),
            Moving::Arm(k) if k == last => model.min_distance_of_chain(self.planned[k], &at),
            Moving::Arm(0) if last == 1 => model.min_distance_of_chain_alone(self.planned[0], &at),
            Moving::Arm(k) => panic!(
                "the self model reads the pairs of the last arm of a plan, or of the first of two, and not of arm {k} of {}",
                last + 1
            ),
        })
    }

    /// The least distance of the nearest checked pair that `moving` judges, the arms at
    /// the state `q`: a state the model cannot measure reads as a collision.
    fn clearance(&self, q: &[f64], moving: Moving) -> f64 {
        self.nearest(q, moving, |nearest| {
            nearest.map_or(f64::NEG_INFINITY, |proximity| proximity.distance)
        })
    }
}

impl<const N: usize, const K: usize, const D: usize> SelfModel<D> for OwnBody<N, K> {
    fn clearance(&self, q: &[f64; D], moving: Moving) -> f64 {
        OwnBody::clearance(self, q, moving)
    }

    fn nearest_pair(&self, q: &[f64; D], moving: Moving) -> (String, String) {
        self.nearest(q, moving, |nearest| {
            let proximity = nearest.expect("a state the self model measures");
            (proximity.link_a.to_string(), proximity.link_b.to_string())
        })
    }
}

/// The states along the straight joint line of a scene at which
/// [`least_self_clearance_along`] measures it, the start and the goal among them.
const LINE_STATES: usize = 101;

/// The least distance between the robot's own bodies, as `standing`'s self model of a plan
/// of every chain measures it, at [`LINE_STATES`] states evenly spaced along the straight
/// joint line from `start` to `goal`.
pub fn least_self_clearance_along(standing: &Standing, start: &[f64], goal: &[f64]) -> f64 {
    (0..LINE_STATES)
        .map(|i| {
            let share = i as f64 / (LINE_STATES - 1) as f64;
            let state: Vec<f64> = start
                .iter()
                .zip(goal)
                .map(|(from, to)| from + share * (to - from))
                .collect();
            standing.self_clearance(&state)
        })
        .fold(f64::INFINITY, f64::min)
}

/// The parts the links of `geometry`'s chain carry, as riders.
fn parts<const N: usize>(geometry: &ChainGeometry<N>) -> Vec<Rider<'_>> {
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

/// What a scene of a scenario of any kind stands in: the voxel size and the half side of the
/// map, its boxes, its collision objects, the objects the robot's grasp links hold, and the
/// pairs allowed to touch.
pub struct SceneParts<'a> {
    pub voxel_size: f64,
    pub map_half_side: f64,
    pub boxes: &'a [VoxelBox],
    pub objects: &'a [Object],
    pub attached: &'a [Held],
    pub allowed: &'a [[String; 2]],
}

/// The planning scene of `scene` on `map`, which its objects clear where they stand: its
/// objects; each object it attaches to a grasp link, at its pose in the link's frame, the
/// link where `grasp_pose` puts it with the arms at the scene's start; and its allowed
/// pairs. The tip links of `arms` are `tips`.
fn planning_scene_of<const N: usize>(
    arms: &[Arm<N>],
    tips: &[&str],
    scene: &SceneParts,
    grasp_pose: &dyn Fn(&str) -> Isometry3<f64>,
    map: &mut VoxelMap,
) -> PlanningScene {
    let tree = arms[0].geometry().chain().tree();
    let links = (0..tree.link_count()).map(|i| tree.link(i).name.clone());
    let grasp_links = tips
        .iter()
        .map(|tip| tip.to_string())
        .chain(scene.attached.iter().map(|held| held.link.clone()));
    let mut planning = PlanningScene::new(RobotLinks::new(links, grasp_links));
    let poses: Vec<Isometry3<f64>> = scene
        .attached
        .iter()
        .map(|held| grasp_pose(&held.link))
        .collect();
    let requests: Vec<ObjectRequest> = scene
        .objects
        .iter()
        .map(object_request)
        .chain(
            scene
                .attached
                .iter()
                .zip(&poses)
                .map(|(held, pose)| held_request(&held.object, pose)),
        )
        .collect();
    planning
        .apply_collision_objects(&requests, map)
        .expect("the scene's objects apply");
    for (held, pose) in scene.attached.iter().zip(&poses) {
        planning
            .apply_attached_collision_object(
                &AttachRequest {
                    object_id: held.object.id.clone(),
                    link_name: held.link.clone(),
                    touch_links: held.touch_links.clone(),
                    operation: "attach".to_string(),
                },
                |_| Some(*pose),
                map,
            )
            .expect("the held object attaches to its link");
    }
    for [name, other] in scene.allowed {
        planning
            .set_allowed_collision(name, std::slice::from_ref(other), true)
            .expect("the scene's pair is allowed");
    }
    planning
}

/// The map of the voxels of `boxes` at the voxel size `size`, each occupied, over the cube of
/// half side `half` about the robot's origin.
pub fn map_of(size: f64, half: f64, boxes: &[VoxelBox]) -> VoxelMap {
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
    let points: Vec<[f32; 3]> = boxes
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

/// The joints' L2 distance between two probes of a path at which no point of the links of
/// `arms`, the scenario's arms in its order, or of their riders moves more than
/// [`PROBE_SHARE_OF_VOXEL`] of a voxel: the reference probes every path at it.
pub fn probe_spacing<const N: usize, const D: usize>(arms: &[Arm<N>], scenes: &Scenes) -> f64 {
    let lever = match arms.len() {
        1 => Arms::<N, N>::new(arms.to_vec(), None).lever(),
        2 => Arms::<N, D>::new(arms.to_vec(), None).lever(),
        count => arms_unplanned(count),
    };
    scenes.voxel_size * PROBE_SHARE_OF_VOXEL / lever
}

/// Panics for a scenario of `count` arms: a scenario plans one arm or both.
fn arms_unplanned(count: usize) -> ! {
    panic!("a scenario plans one arm or both, and this one names {count}")
}

/// The state of a group of D joints that `values` hold.
pub fn state<const D: usize>(values: &[f64]) -> [f64; D] {
    values
        .try_into()
        .expect("one position per joint of the arms")
}

/// The configuration of the arms that `goal` reaches for, where it is one.
pub fn joint_goal(goal: &SceneGoal) -> Option<&[f64]> {
    match goal {
        SceneGoal::Joints(q) => Some(q),
        SceneGoal::Poses(_) | SceneGoal::Positions(_) => None,
    }
}

/// Each arm's goal of `goal`, for the arms of a group of D joints in their order: each
/// joint within the scenes' goal tolerance of the arm's configuration in it, or its grasp
/// point within the tolerances of its pose or its position of them.
fn goals_of<const N: usize, const D: usize>(scenes: &Scenes, goal: &SceneGoal) -> Vec<Goal> {
    match goal {
        SceneGoal::Joints(q) => {
            let goal_state: [f64; D] = state(q);
            let tolerance =
                PositiveFinite::parse(scenes.goal_tolerance).expect("the scenes' goal tolerance");
            (0..D / N)
                .map(|k| {
                    let goal_q = Arms::<N, D>::slice(&goal_state, k);
                    Goal::new(
                        N,
                        (0..N)
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
                    .expect("a valid goal")
                })
                .collect()
        }
        SceneGoal::Poses(poses) => {
            assert_eq!(poses.len() * N, D, "a pose of each arm's grasp point");
            poses
                .iter()
                .map(|pose| {
                    let placed = pose_from_parts(pose.position, pose.orientation)
                        .expect("a pose goal's pose");
                    Goal::new(
                        N,
                        Vec::new(),
                        vec![PositionTarget {
                            frame: motion_planning_core::Frame::GraspPoint,
                            position: placed.translation.vector,
                            tolerance_m: PositiveFinite::parse(pose.position_tolerance)
                                .expect("a pose goal's position tolerance"),
                        }],
                        vec![OrientationTarget {
                            frame: motion_planning_core::Frame::GraspPoint,
                            orientation: placed.rotation,
                            tolerance_rad: PositiveFinite::parse(pose.orientation_tolerance)
                                .expect("a pose goal's orientation tolerance"),
                            free_axis: None,
                        }],
                    )
                    .expect("a valid goal")
                })
                .collect()
        }
        SceneGoal::Positions(positions) => {
            assert_eq!(
                positions.len() * N,
                D,
                "a position of each arm's grasp point"
            );
            positions
                .iter()
                .map(|goal| {
                    Goal::new(
                        N,
                        Vec::new(),
                        vec![PositionTarget {
                            frame: motion_planning_core::Frame::GraspPoint,
                            position: goal.position.into(),
                            tolerance_m: PositiveFinite::parse(goal.position_tolerance)
                                .expect("a position goal's tolerance"),
                        }],
                        Vec::new(),
                    )
                    .expect("a valid goal")
                })
                .collect()
        }
    }
}

/// What a plan holds of `held`, an orientation of a link of the chains `chains` with their
/// arms `arms`: of the grasp point for a chain's tip link, and of the link of a joint for any
/// other.
fn path_orientation<'a, const N: usize>(
    held: &'a HeldOrientation,
    chains: &[ArmChain],
    arms: &[Arm<N>],
) -> PathOrientation<'a> {
    let frame = if held.link_name == chains[held.chain].tip_link {
        motion_planning_core::Frame::GraspPoint
    } else {
        let posed = arms[held.chain].geometry().chain().at(&[0.0; N]);
        let segment = (0..N)
            .find(|&segment| posed.link_name(segment) == held.link_name)
            .unwrap_or_else(|| panic!("{} is a link of its chain", held.link_name));
        motion_planning_core::Frame::Link {
            segment,
            offset: Isometry3::identity(),
        }
    };
    PathOrientation {
        arm: held.chain,
        link_name: &held.link_name,
        target: OrientationTarget {
            frame,
            orientation: orientation_from_parts(held.orientation).expect("a unit quaternion"),
            tolerance_rad: PositiveFinite::parse(held.tolerance_rad)
                .expect("a path orientation's tolerance"),
            free_axis: Axis::free_named(&held.free_axis).expect("a free axis's name"),
        },
    }
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

/// This crate's answer to `scene` by `arms`, the scenario's arms in its order, as one
/// group, each drawing its goal configurations by its sampler of `samplers`.
pub fn plan_scene<const N: usize, const D: usize, O>(
    arms: &[Arm<N>],
    standing: &mut Standing,
    samplers: &[IkGoalSampler<N>],
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    match arms.len() {
        1 => plan_group::<N, N, O>(arms, standing, samplers, scenes, scene),
        2 => plan_group::<N, D, O>(arms, standing, samplers, scenes, scene),
        count => arms_unplanned(count),
    }
}

/// `with` run on the group of `arms`, the arms of `chains`, D the joints of them all, and
/// on the scene they meet from `start`: among the voxels the boxes of `scene` occupy, its
/// objects and the parts of `standing` the arms do not move, holding the objects it attaches,
/// with `standing`'s self model where the scenario plans every chain of the robot: each
/// arm's parts ride its links, and the objects its grasp link holds its tip link.
pub fn in_the_scene<const N: usize, const D: usize, R>(
    arms: &[Arm<N>],
    standing: &mut Standing,
    chains: &[ArmChain],
    scene: &SceneParts,
    start: &[f64; D],
    with: impl FnOnce(&Arms<N, D>, &Scene<'_>) -> R,
) -> R {
    let mut map = map_of(scene.voxel_size, scene.map_half_side, scene.boxes);
    let tips: Vec<&str> = chains.iter().map(|chain| chain.tip_link.as_str()).collect();
    let grasp_pose = |link: &str| match tips.iter().position(|tip| *tip == link) {
        Some(k) => arms[k]
            .geometry()
            .chain()
            .world_ee_pose(&Arms::<N, D>::slice(start, k)),
        None => standing.idle_grasp_pose(link),
    };
    let planning = planning_scene_of(arms, &tips, scene, &grasp_pose, &mut map);
    let field = DistanceField::of(&map);
    let objects = planning.for_arms(&tips, grasp_pose);
    let (standing_parts, self_model) = standing.around(arms, start);
    let riders: Vec<Vec<Rider>> = arms
        .iter()
        .zip(&tips)
        .map(|(arm, tip)| {
            parts(arm.geometry())
                .into_iter()
                .chain(planning.riders(tip, arm.geometry()))
                .collect()
        })
        .collect();
    let group = Arms::<N, D>::new(
        arms.iter()
            .zip(&riders)
            .map(|(arm, riders)| {
                Arm::new(arm.name(), arm.geometry())
                    .riding(riders)
                    .expect("the parts and the held objects ride the arm's links")
            })
            .collect(),
        self_model,
    );
    with(
        &group,
        &Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: objects.objects(&standing_parts),
        },
    )
}

/// The name of a scenario's answer for a start that `fault` refuses.
pub fn start_fault_name(fault: &StartFault) -> &'static str {
    match fault {
        StartFault::OutsideLimits { .. } => "start_outside_limits",
        StartFault::InCollision(_) | StartFault::InsideClearance { .. } => "start_in_collision",
    }
}

/// This crate's answer to `scene` by the group of `arms`, D the joints of them all, in the
/// scene `in_the_scene` holds.
fn plan_group<const N: usize, const D: usize, O>(
    arms: &[Arm<N>],
    standing: &mut Standing,
    samplers: &[IkGoalSampler<N>],
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    let start: [f64; D] = state(&scene.start);
    let world = SceneParts {
        voxel_size: scenes.voxel_size,
        map_half_side: scenes.map_half_side,
        boxes: &scene.boxes,
        objects: &scene.objects,
        attached: &scene.attached,
        allowed: &scene.allowed,
    };
    in_the_scene(
        arms,
        standing,
        &scenes.chains,
        &world,
        &start,
        |group, met| {
            let goals = goals_of::<N, D>(scenes, &scene.goal);
            let orientations: Vec<PathOrientation> = scene
                .path_orientations
                .iter()
                .map(|held| path_orientation(held, &scenes.chains, arms))
                .collect();
            let deadline = Instant::now() + Duration::from_secs_f64(scenes.time_limit);
            let answer = plan_path(
                group,
                met,
                &PlanRequest {
                    start,
                    start_tolerance: [0.0; D],
                    goals: goals
                        .iter()
                        .zip(samplers)
                        .map(|(goal, sampler)| ArmGoal { goal, sampler })
                        .collect(),
                    path_orientations: &orientations,
                    goal_samples: NonZeroUsize::new(LAUNCH_GOAL_SAMPLES).expect("nonzero"),
                    planner: planner_of(scenes.objective),
                    refiner: None,
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
                Err(PlanFailure::Start(fault)) => failure(start_fault_name(&fault)),
                Err(PlanFailure::Unreachable(_)) => failure("unreachable"),
                Err(PlanFailure::NoPath(_)) => failure("no_path"),
                Err(
                    failure @ (PlanFailure::Request(_)
                    | PlanFailure::Planner(_)
                    | PlanFailure::Stopped),
                ) => {
                    panic!("the scene's request fails: {failure}")
                }
            };
            Planned {
                ours,
                at_the_deadline,
            }
        },
    )
}
