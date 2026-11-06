//! The scenes of the ROS reference harness's arm planning scenarios as this crate plans
//! them: the arms, the parts their links carry, the robot around them that their plans do
//! not move, with OpenArm v2's self model where a scenario plans both arms, each scene's
//! map, its planning scene of objects and of the objects the robot's grasp links hold,
//! with its allowed pairs, and its plan. The example `plan_arm_scenes` and the parity
//! test share them, so the paths, the checked pairs and the probe spacing each scenario
//! carries are the ones the test plans. The planning scene parity test shares the reading
//! of a harness object as an entry of apply_collision_objects.
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
    Arm, ArmGoal, Arms, AttachRequest, Goal, IkGoalSampler, JointTarget, Moving, ObjectFields,
    ObjectRequest, OrientationTarget, PlanFailure, PlanRequest, PlanningScene, PositionTarget,
    RobotLinks, RobotPart, Scene, SelfModel, ShapeFields, StartFault, plan_path, pose_from_parts,
    pose_parts,
};
use nalgebra::Isometry3;
use occupancy_map::{Bounds, DistanceField, Frame, Scan, VoxelMap};
use ompl_bridge::Planner;
use openarm_description::{ARM_DOF, ArmOrder, ArmPair, DUAL_DOF, HardwareVersion, Side};
use ros_reference::arm_planning::{
    ArmChain, Objective, Ours, Scene as ReferenceScene, SceneGoal, Scenes, Termination, VoxelBox,
};
use ros_reference::planning_scene::{Object, Shape};
use self_collision_model::{CollisionError, Proximity, SelfCollisionModel};

pub const JOINTS: usize = ARM_DOF;
/// The most goals a scene's plan samples: the motion planner's launch's (launchers-hub
/// openarm/fragments/motion_planner.json5, `goal_samples`).
const LAUNCH_GOAL_SAMPLES: usize = 10;
/// The share of a voxel a link point moves at most between two probes of a path.
const PROBE_SHARE_OF_VOXEL: f64 = 0.1;
/// The frame of OpenArm's URDF root, in which the scenes' boxes stand.
const ROOT_FRAME: &str = "world";
/// The range of the scan that fills a scene's map, from the root: beyond every voxel of
/// the map.
const MAX_RANGE_M: f64 = 10.0;
/// Where an arm of no chain of a scenario stands: at its zero configuration, as MoveIt's
/// default state holds it.
const AT_ZERO: ArmPair<[f64; JOINTS]> = ArmPair {
    left: [0.0; JOINTS],
    right: [0.0; JOINTS],
};
/// The opening of the self model's grippers: closed, where OpenArm v2's finger joints
/// stand at 0, as MoveIt's default state holds them.
const FINGERS_CLOSED: f64 = 0.0;

/// The collision pieces of the arm of `chain`, from the URDF text `urdf`, whose meshes
/// lie in `meshes`.
pub fn geometry(chain: &ArmChain, urdf: &str, meshes: &Path) -> ChainGeometry<JOINTS> {
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

/// `with` run on the scenario's arms, from the URDF text `urdf` whose meshes lie in
/// `meshes`, in the order of its chains: each arm by its chain's name, with the parts its
/// links carry, its gripper's fingers closed, as MoveIt's default state holds them; and
/// the robot around them.
pub fn on_the_arms<R>(
    scenes: &Scenes,
    urdf: &str,
    meshes: &Path,
    with: impl FnOnce(&[Arm<JOINTS>], &mut Standing) -> R,
) -> R {
    let geometries: Vec<ChainGeometry<JOINTS>> = scenes
        .chains
        .iter()
        .map(|chain| geometry(chain, urdf, meshes))
        .collect();
    let riders: Vec<Vec<Rider>> = geometries.iter().map(parts).collect();
    let arms: Vec<Arm<JOINTS>> = scenes
        .chains
        .iter()
        .zip(&geometries)
        .zip(&riders)
        .map(|((chain, geometry), riders)| {
            Arm::new(&chain.name, geometry)
                .riding(riders)
                .expect("the parts ride the arm's links")
        })
        .collect();
    let mut standing = Standing::new(urdf, meshes, &scenes.chains);
    with(&arms, &mut standing)
}

/// Each arm's goal sampler, of the arms `arms`.
pub fn samplers(arms: &[Arm<JOINTS>]) -> Vec<IkGoalSampler<JOINTS>> {
    arms.iter()
        .map(|arm| IkGoalSampler::new(arm.geometry().chain().clone()))
        .collect()
}

/// The robot around the scenario's arms, which their plans do not move: OpenArm v2's self
/// model, whose bodies off the arms stand as the parts of the robot a plan meets, an arm
/// of no chain at its zero configuration; where the scenario plans both arms, the self
/// model of their plans; and the URDF, whose chain to the grasp link of an arm of no chain
/// places the objects that link holds.
pub struct Standing {
    model: SelfCollisionModel,
    own_body: Option<OwnBody>,
    order: ArmOrder,
    urdf: String,
    meshes: String,
}

impl Standing {
    /// The robot around the arms of `chains`, each one of OpenArm v2's arms from its base
    /// link to its grasp link, by its name.
    fn new(urdf: &str, meshes: &Path, chains: &[ArmChain]) -> Self {
        let version = HardwareVersion::V2;
        let names: Vec<&str> = chains.iter().map(|chain| chain.name.as_str()).collect();
        let order = ArmOrder::parse(&names)
            .unwrap_or_else(|refusal| panic!("the scenario's chains: {refusal}"));
        for (chain, side) in chains.iter().zip(order.sides()) {
            assert_eq!(
                [chain.base_link.as_str(), chain.tip_link.as_str()],
                [version.base_link(*side), version.tcp_link(*side)],
                "the chain {} runs from its arm's base link to its grasp link",
                chain.name
            );
        }
        let meshes = meshes.to_str().expect("a UTF-8 mesh directory").to_string();
        let model =
            self_collision_model::openarm(version, &meshes).expect("the self model of OpenArm v2");
        let own_body = (chains.len() > 1).then(|| OwnBody::new(&model, &order));
        Self {
            model,
            own_body,
            order,
            urdf: urdf.to_string(),
            meshes,
        }
    }

    /// The pairs of the robot's own links the scenario's plans check: the self model's
    /// where it plans both arms, none where it plans one arm, which this crate plans
    /// without the self model.
    pub fn checked_pairs(&self) -> Vec<[String; 2]> {
        self.own_body
            .as_ref()
            .map_or_else(Vec::new, OwnBody::checked_pairs)
    }

    /// The least distance between the robot's own bodies, of the pairs the self model of a
    /// plan of both arms checks, the arms at `state`, which holds their joints in their
    /// order.
    pub fn self_clearance(&self, state: &[f64]) -> f64 {
        self.own_body
            .as_ref()
            .expect("the scenario plans both arms")
            .clearance(state, Moving::All)
    }

    /// The pose of `link`, the grasp link of an arm of no chain of the scenario, at that
    /// arm's zero configuration.
    fn idle_grasp_pose(&self, link: &str) -> Isometry3<f64> {
        assert!(
            Side::BOTH.into_iter().any(|side| {
                !self.order.sides().contains(&side) && HardwareVersion::V2.tcp_link(side) == link
            }),
            "{link} is the grasp link of an arm of no chain of the scenario"
        );
        ChainGeometry::<JOINTS>::from_urdf(
            &self.urdf,
            &self.meshes,
            &ChainSpec {
                base_link: None,
                tip_link: link,
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the arm's chain")
        .chain()
        .world_ee_pose(&AT_ZERO.left)
    }

    /// What a plan of `arms` from `start`, which holds their joints in their order, meets
    /// of the robot: the pieces of its bodies that the arms do not move, an arm of no chain
    /// at its zero configuration, each body but the arms' links and the parts they carry;
    /// and the self model of the plan where the scenario plans both arms.
    pub fn around<'s, const D: usize>(
        &'s mut self,
        arms: &[Arm<JOINTS>],
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
        let at = self.order.placed(start, &AT_ZERO);
        let parts = self
            .model
            .world_pieces(&at.left, &at.right)
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

/// OpenArm v2's self model as a plan of the arms of an order meets it: its checked pairs,
/// each arm of the order at its joints of the state asked about, an arm of none at its
/// zero configuration, and the grippers' fingers closed, as MoveIt's default state holds
/// them.
struct OwnBody {
    model: Mutex<SelfCollisionModel>,
    order: ArmOrder,
}

impl OwnBody {
    fn new(model: &SelfCollisionModel, order: &ArmOrder) -> Self {
        let mut model = model.clone();
        model.set_gripper_openings(FINGERS_CLOSED, FINGERS_CLOSED);
        Self {
            model: Mutex::new(model),
            order: order.clone(),
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

    /// Runs `read` on the nearest checked pair that `moving` judges, the arms of the order
    /// at the state `q`: an arm the order holds last reads its pairs with the other arm,
    /// and one before it its pairs without it.
    fn nearest<T>(
        &self,
        q: &[f64],
        moving: Moving,
        read: impl FnOnce(Result<Proximity<'_>, CollisionError>) -> T,
    ) -> T {
        let at = self.order.placed(q, &AT_ZERO);
        let sides = self.order.sides();
        let mut model = self.model.lock().expect("the self model");
        read(match moving {
            Moving::All => model.min_distance(&at.left, &at.right),
            Moving::Arm(k) if k + 1 == sides.len() => {
                model.min_distance_of_arm(sides[k], &at.left, &at.right)
            }
            Moving::Arm(k) => model.min_distance_of_arm_alone(sides[k], &at.left, &at.right),
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

impl<const D: usize> SelfModel<D> for OwnBody {
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
/// of both arms measures it, at [`LINE_STATES`] states evenly spaced along the straight
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
/// objects; each object it attaches to a grasp link, at its pose in the link's frame, the
/// link where `grasp_pose` puts it with the arms at the scene's start; and its allowed
/// pairs. The tip links of `arms` are `tips`.
fn planning_scene_of<O>(
    arms: &[Arm<JOINTS>],
    tips: &[&str],
    scene: &ReferenceScene<O>,
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

/// The joints' L2 distance between two probes of a path at which no point of the links of
/// `arms`, the scenario's arms in its order, or of their riders moves more than
/// [`PROBE_SHARE_OF_VOXEL`] of a voxel: the reference probes every path at it.
pub fn probe_spacing(arms: &[Arm<JOINTS>], scenes: &Scenes) -> f64 {
    let lever = match arms.len() {
        1 => Arms::<JOINTS, JOINTS>::new(arms.to_vec(), None).lever(),
        2 => Arms::<JOINTS, DUAL_DOF>::new(arms.to_vec(), None).lever(),
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
        SceneGoal::Poses(_) => None,
    }
}

/// Each arm's goal of `goal`, for the arms of a group of D joints in their order: each
/// joint within the scenes' goal tolerance of the arm's configuration in it, or its grasp
/// point within the tolerances of its pose of them.
fn goals_of<const D: usize>(scenes: &Scenes, goal: &SceneGoal) -> Vec<Goal> {
    match goal {
        SceneGoal::Joints(q) => {
            let goal_state: [f64; D] = state(q);
            let tolerance =
                PositiveFinite::parse(scenes.goal_tolerance).expect("the scenes' goal tolerance");
            (0..D / JOINTS)
                .map(|k| {
                    let goal_q = Arms::<JOINTS, D>::slice(&goal_state, k);
                    Goal::new(
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
                    .expect("a valid goal")
                })
                .collect()
        }
        SceneGoal::Poses(poses) => {
            assert_eq!(poses.len() * JOINTS, D, "a pose of each arm's grasp point");
            poses
                .iter()
                .map(|pose| {
                    let placed = pose_from_parts(pose.position, pose.orientation)
                        .expect("a pose goal's pose");
                    Goal::new(
                        JOINTS,
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
pub fn plan_scene<O>(
    arms: &[Arm<JOINTS>],
    standing: &mut Standing,
    samplers: &[IkGoalSampler<JOINTS>],
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    match arms.len() {
        1 => plan_group::<JOINTS, O>(arms, standing, samplers, scenes, scene),
        2 => plan_group::<DUAL_DOF, O>(arms, standing, samplers, scenes, scene),
        count => arms_unplanned(count),
    }
}

/// This crate's answer to `scene` by the group of `arms`, D the joints of them all, among
/// the voxels its boxes occupy, its objects and the parts of `standing` the arms do not
/// move, holding the objects it attaches, with `standing`'s self model where the scenario
/// plans both arms: each arm's parts ride its links, and the objects its grasp link holds
/// its tip link.
fn plan_group<const D: usize, O>(
    arms: &[Arm<JOINTS>],
    standing: &mut Standing,
    samplers: &[IkGoalSampler<JOINTS>],
    scenes: &Scenes,
    scene: &ReferenceScene<O>,
) -> Planned {
    let mut map = map_of(scenes, scene);
    let start: [f64; D] = state(&scene.start);
    let tips: Vec<&str> = scenes
        .chains
        .iter()
        .map(|chain| chain.tip_link.as_str())
        .collect();
    let grasp_pose = |link: &str| match tips.iter().position(|tip| *tip == link) {
        Some(k) => arms[k]
            .geometry()
            .chain()
            .world_ee_pose(&Arms::<JOINTS, D>::slice(&start, k)),
        None => standing.idle_grasp_pose(link),
    };
    let planning = planning_scene_of(arms, &tips, scene, &grasp_pose, &mut map);
    let field = DistanceField::of(&map);
    let objects = planning.for_arms(&tips, grasp_pose);
    let (standing_parts, self_model) = standing.around(arms, &start);
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
    let group = Arms::<JOINTS, D>::new(
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
    let goals = goals_of::<D>(scenes, &scene.goal);
    let deadline = Instant::now() + Duration::from_secs_f64(scenes.time_limit);
    let answer = plan_path(
        &group,
        &Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: objects.objects(&standing_parts),
        },
        &PlanRequest {
            start,
            start_tolerance: [0.0; D],
            goals: goals
                .iter()
                .zip(samplers)
                .map(|(goal, sampler)| ArmGoal { goal, sampler })
                .collect(),
            goal_samples: NonZeroUsize::new(LAUNCH_GOAL_SAMPLES).expect("nonzero"),
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
