//! The scenes of the ROS reference harness's arm planning scenarios as this crate plans
//! them: the arms of the scenario's group, the parts their links carry, the robot around
//! them that their plans do not move, with its self model where a scenario plans every arm
//! of the robot, each scene's map, its planning scene of objects and of the objects the
//! robot's grasp links hold, with its allowed pairs, and its plan.
//! The example `plan_arm_scenes` and the parity test share them, so the paths and the probe
//! spacing each scenario carries are the ones the test plans. The planning scene parity
//! test shares the reading of a harness object as an entry of apply_collision_objects and
//! the fingers' opening of MoveIt's default state, and the Cartesian paths scenes
//! (`cartesian_scenes`) share the arms and a scene's parts as this crate meets them.
#![allow(dead_code)]

use std::collections::BTreeSet;
use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, ArmGoal, Arms, AttachRequest, Axis, Goal, IkGoalSampler, JointTarget, Moving,
    ObjectFields, ObjectRequest, OrientationTarget, PathOrientation, PlanFailure, PlanRequest,
    PlanningScene, PositionTarget, Refiner, RobotLinks, RobotPart, Scene, SelfModel, ShapeFields,
    StartFault, iteration_bound, orientation_from_parts, plan_path, pose_from_parts, pose_parts,
};
use nalgebra::Isometry3;
use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};
use ompl_bridge::Planner;
use robot_description::{Arm as DescribedArm, RobotDescription};
use ros_reference::Robot;
use ros_reference::arm_planning::{
    Held, HeldOrientation, Ours, Planner as ReferencePlanner, Refiner as ReferenceRefiner,
    Scene as ReferenceScene, SceneGoal, Scenes, Termination, VoxelBox,
};
use ros_reference::planning_scene::{Object, Shape};
use self_collision_model::{CollisionError, Joints, Proximity, SelfCollisionModel};

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
/// The position of a joint in MoveIt's default state, which the scenarios' fingers stand at.
const DEFAULT_POSITION: f64 = 0.0;

/// The collision pieces of `arm`, an arm of `robot`, from its base link to its grasp link.
pub fn geometry<const N: usize>(arm: &DescribedArm, robot: &Robot) -> ChainGeometry<N> {
    let joints: Vec<&str> = arm.joints.iter().map(String::as_str).collect();
    ChainGeometry::from_urdf(
        robot.description.urdf(),
        robot.meshes.to_str().expect("a UTF-8 mesh directory"),
        &ChainSpec {
            base_link: Some(&arm.base_link),
            tip_link: &arm.grasp_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .expect("the arm fits")
}

/// `with` run on `planned`, the arms of `robot`, a robot of `K` arms of `N` joints, that a
/// scenario's group holds, in their order: each by its name, with the parts its links
/// carry, its gripper's fingers closed, as MoveIt's default state holds them; and the robot
/// around them.
pub fn on_the_arms<const N: usize, const K: usize, R>(
    planned: &[DescribedArm],
    robot: &Robot,
    with: impl FnOnce(&[Arm<N>], &mut Standing<N, K>) -> R,
) -> R {
    let geometries: Vec<ChainGeometry<N>> =
        planned.iter().map(|arm| geometry(arm, robot)).collect();
    let riders: Vec<Vec<Rider>> = geometries.iter().map(parts).collect();
    let arms: Vec<Arm<N>> = planned
        .iter()
        .zip(&geometries)
        .zip(&riders)
        .map(|((arm, geometry), riders)| {
            Arm::new(&arm.name, geometry)
                .riding(riders)
                .expect("the parts ride the arm's links")
        })
        .collect();
    let mut standing = Standing::new(robot, planned);
    with(&arms, &mut standing)
}

/// Each arm's goal sampler, of the arms `arms`.
pub fn samplers<const N: usize>(arms: &[Arm<N>]) -> Vec<IkGoalSampler<N>> {
    arms.iter()
        .map(|arm| IkGoalSampler::new(arm.geometry().chain().clone()))
        .collect()
}

/// Each arm's gripper opening where MoveIt's default state stands its fingers: the share of
/// the way from where the gripper's `closed` state puts its finger that mimics none to where
/// its `open` state puts it, the finger at `DEFAULT_POSITION`.
pub fn default_openings<const K: usize>(description: &RobotDescription) -> [f64; K] {
    let arms: &[robot_description::Arm; K] = description
        .arms()
        .try_into()
        .expect("an opening for each arm of the description");
    arms.each_ref().map(|arm| {
        let leading = arm
            .gripper
            .fingers
            .iter()
            .find(|finger| {
                description
                    .robot()
                    .joints
                    .iter()
                    .any(|joint| joint.name == finger.joint && joint.mimic.is_none())
            })
            .unwrap_or_else(|| panic!("{} has a finger that mimics none", arm.gripper.name));
        (DEFAULT_POSITION - leading.closed) / (leading.open - leading.closed)
    })
}

/// The links of the bodies of `geometry`'s chain: its links and the parts they carry.
pub fn bodies_of<const N: usize>(geometry: &ChainGeometry<N>) -> impl Iterator<Item = &str> {
    geometry
        .links()
        .iter()
        .map(|link| link.name())
        .chain(geometry.carried().iter().map(|part| part.rider().name))
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

/// The robot around the scenario's arms, which their plans do not move, a robot of `K`
/// chains of `N` joints: its self model, whose bodies off the arms stand as the parts of the
/// robot a plan meets, a chain of no arm at its zero configuration; where the scenario plans
/// every chain, the self model of its plans; and the URDF, whose chain to the grasp link of
/// a chain of no arm places the objects that link holds.
pub struct Standing<const N: usize, const K: usize> {
    model: SelfCollisionModel<N, K>,
    /// The robot's arms, in the order of its description, which the self model's chains
    /// hold.
    arms: Vec<DescribedArm>,
    /// The place among `arms` of each arm of the scenario, in its order.
    planned: Vec<usize>,
    own_body: Option<OwnBody<N, K>>,
    urdf: String,
    meshes: String,
}

impl<const N: usize, const K: usize> Standing<N, K> {
    /// The robot `robot` around `planned`, the arms of it a scenario's group holds, in
    /// their order: its self model, its fingers where MoveIt's default state stands them.
    pub fn new(robot: &Robot, planned: &[DescribedArm]) -> Self {
        let meshes = robot.meshes.to_str().expect("a UTF-8 mesh directory");
        let description = &robot.description;
        let model = SelfCollisionModel::from_description(description, meshes)
            .expect("the self model of the scenario's robot");
        let arms = description.arms().to_vec();
        let planned: Vec<usize> = planned
            .iter()
            .map(|arm| {
                arms.iter()
                    .position(|candidate| candidate == arm)
                    .unwrap_or_else(|| panic!("{} is an arm of the scenario's robot", arm.name))
            })
            .collect();
        let own_body = (planned.len() == K)
            .then(|| OwnBody::new(&model, &planned, default_openings(description)));
        Self {
            model,
            arms,
            planned,
            own_body,
            urdf: description.urdf().to_string(),
            meshes: meshes.to_string(),
        }
    }

    /// The grasp link of each arm of the scenario, in its order.
    pub fn grasp_links(&self) -> Vec<String> {
        self.planned
            .iter()
            .map(|&arm| self.arms[arm].grasp_link.clone())
            .collect()
    }

    /// The pairs of the robot's own links the scenario's plans check, each pair's names in
    /// order: the self model's where it plans every arm, none where it plans some, which
    /// this crate plans without the self model.
    pub fn checked_pairs(&self) -> BTreeSet<[String; 2]> {
        self.own_body
            .as_ref()
            .map_or_else(BTreeSet::new, OwnBody::checked_pairs)
    }

    /// The least distance between the robot's own bodies, of the pairs the self model of a
    /// plan of every chain checks, the arms at `state`, which holds their joints in their
    /// order.
    pub fn self_clearance(&self, state: &[f64]) -> f64 {
        self.own_body
            .as_ref()
            .expect("the scenario plans every chain of the robot")
            .clearance(state, Moving::All, f64::INFINITY)
    }

    /// The pose of `link`, the grasp link of an arm the scenario does not plan, at that
    /// arm's zero configuration.
    fn idle_grasp_pose(&self, link: &str) -> Isometry3<f64> {
        assert!(
            self.arms
                .iter()
                .enumerate()
                .any(|(arm, held)| !self.planned.contains(&arm) && held.grasp_link == link),
            "{link} is the grasp link of an arm the scenario does not plan"
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

    /// What a plan of `arms` from `start`, which holds their joints in their order, meets
    /// of the robot: the pieces of its bodies that the arms do not move, a chain of no arm
    /// at its zero configuration, each body but the arms' links and the parts they carry;
    /// and the self model of the plan where the scenario plans every chain.
    pub fn around<'s, const D: usize>(
        &'s mut self,
        arms: &[Arm<N>],
        start: &[f64; D],
    ) -> (Vec<RobotPart<'s>>, Option<&'s dyn SelfModel<D>>) {
        let moving: BTreeSet<&str> = arms
            .iter()
            .flat_map(|arm| bodies_of(arm.geometry()))
            .collect();
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
    fn new(model: &SelfCollisionModel<N, K>, planned: &[usize], openings: [f64; K]) -> Self {
        const {
            assert!(
                K <= 2,
                "a turn of one arm reads the last arm's pairs, and the first's of two"
            )
        };
        let mut model = model.clone();
        model.set_gripper_openings(&openings);
        Self {
            model: Mutex::new(model),
            planned: planned.to_vec(),
        }
    }

    /// The model's checked pairs, by their links' names, each pair's names in order.
    fn checked_pairs(&self) -> BTreeSet<[String; 2]> {
        let model = self.model.lock().expect("the self model");
        model
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| {
                let pair = [a.to_string(), b.to_string()];
                if a <= b {
                    pair
                } else {
                    [pair[1].clone(), pair[0].clone()]
                }
            })
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

    /// The least distance of the checked pairs that `moving` judges, the arms at the
    /// state `q`, as a check at `within` meters reads it, each arm reading the pairs
    /// `nearest` reads for it: a state the model cannot measure reads as a collision.
    fn clearance(&self, q: &[f64], moving: Moving, within: f64) -> f64 {
        let at = placed::<N, K>(&self.planned, q);
        let last = self.planned.len() - 1;
        let mut model = self.model.lock().expect("the self model");
        match moving {
            Moving::All => model.min_distance_within(&at, within),
            Moving::Arm(k) if k == last => {
                model.min_distance_of_chain_within(self.planned[k], &at, within)
            }
            Moving::Arm(0) if last == 1 => {
                model.min_distance_of_chain_alone_within(self.planned[0], &at, within)
            }
            Moving::Arm(k) => panic!(
                "the self model reads the pairs of the last arm of a plan, or of the first of two, and not of arm {k} of {}",
                last + 1
            ),
        }
        .unwrap_or(f64::NEG_INFINITY)
    }
}

impl<const N: usize, const K: usize, const D: usize> SelfModel<D> for OwnBody<N, K> {
    fn clearance(&self, q: &[f64; D], moving: Moving, within: f64) -> f64 {
        OwnBody::clearance(self, q, moving, within)
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
pub fn least_self_clearance_along<const N: usize, const K: usize>(
    standing: &Standing<N, K>,
    start: &[f64],
    goal: &[f64],
) -> f64 {
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

/// What a plan holds of `held`, an orientation of a link of `arms`, whose grasp links are
/// `grasp_links`: of the grasp point for an arm's grasp link, and of the link of a joint for
/// any other.
fn path_orientation<'a, const N: usize>(
    held: &'a HeldOrientation,
    grasp_links: &[String],
    arms: &[Arm<N>],
) -> PathOrientation<'a> {
    let frame = if held.link_name == grasp_links[held.arm] {
        motion_planning_core::Frame::GraspPoint
    } else {
        let posed = arms[held.arm].geometry().chain().at(&[0.0; N]);
        let segment = (0..N)
            .find(|&segment| posed.link_name(segment) == held.link_name)
            .unwrap_or_else(|| panic!("{} is a link of its chain", held.link_name));
        motion_planning_core::Frame::Link {
            segment,
            offset: Isometry3::identity(),
        }
    };
    PathOrientation {
        arm: held.arm,
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

/// This crate's planner of the scenario's `planner`: RRTConnect for `rrt_connect`, as the
/// motion planner node plans it, and RRTstar for `rrt_star`.
fn planner_of(planner: ReferencePlanner) -> Planner {
    match planner {
        ReferencePlanner::RrtConnect => Planner::RrtConnect,
        ReferencePlanner::RrtStar => Planner::RrtStar,
    }
}

/// This crate's refiner of the scenario's `refiner`: none for `none`, and CHOMP for `chomp`.
fn refiner_of(refiner: ReferenceRefiner) -> Option<Refiner> {
    match refiner {
        ReferenceRefiner::None => None,
        ReferenceRefiner::Chomp => Some(Refiner::Chomp),
    }
}

/// The iteration bound of this crate's plans of `scenes`, as their termination says: the
/// scenes' iterations, or for a plan the time ends, the bound its planner takes from them.
fn max_iterations(scenes: &Scenes) -> NonZeroU64 {
    let bound = NonZeroU64::new(scenes.iterations).expect("nonzero");
    match scenes.termination {
        Termination::Iterations => bound,
        Termination::Time => iteration_bound(planner_of(scenes.planner), bound),
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
pub fn plan_scene<const N: usize, const D: usize, const K: usize, O>(
    arms: &[Arm<N>],
    standing: &mut Standing<N, K>,
    samplers: &[IkGoalSampler<N>],
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    match arms.len() {
        1 => plan_group::<N, N, K, O>(arms, standing, samplers, scenes, scene),
        2 => plan_group::<N, D, K, O>(arms, standing, samplers, scenes, scene),
        count => arms_unplanned(count),
    }
}

/// `with` run on the group of `arms`, the arms of the scenario's group, D the joints of them
/// all, and on the scene they meet from `start`: among the voxels the boxes of `scene`
/// occupy, its objects and the parts of `standing` the arms do not move, holding the objects
/// it attaches, with `standing`'s self model where the scenario plans every arm of the
/// robot: each arm's parts ride its links, and the objects its grasp link holds its last
/// link.
pub fn in_the_scene<const N: usize, const D: usize, const K: usize, R>(
    arms: &[Arm<N>],
    standing: &mut Standing<N, K>,
    scene: &SceneParts,
    start: &[f64; D],
    with: impl FnOnce(&Arms<N, D>, &Scene<'_>) -> R,
) -> R {
    let mut map = map_of(scene.voxel_size, scene.map_half_side, scene.boxes);
    let grasp_links = standing.grasp_links();
    let tips: Vec<&str> = grasp_links.iter().map(String::as_str).collect();
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
fn plan_group<const N: usize, const D: usize, const K: usize, O>(
    arms: &[Arm<N>],
    standing: &mut Standing<N, K>,
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
    let grasp_links = standing.grasp_links();
    in_the_scene(arms, standing, &world, &start, |group, met| {
        let goals = goals_of::<N, D>(scenes, &scene.goal);
        let orientations: Vec<PathOrientation> = scene
            .path_orientations
            .iter()
            .map(|held| path_orientation(held, &grasp_links, arms))
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
                planner: planner_of(scenes.planner),
                refiner: refiner_of(scenes.refiner),
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
                failure
                @ (PlanFailure::Request(_) | PlanFailure::Planner(_) | PlanFailure::Stopped),
            ) => {
                panic!("the scene's request fails: {failure}")
            }
        };
        Planned {
            ours,
            at_the_deadline,
        }
    })
}
