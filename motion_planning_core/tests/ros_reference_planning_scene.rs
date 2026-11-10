//! Parity with MoveIt's planning scene on the planning scene scenarios of the ROS
//! reference harness, on OpenArm v2. After a scenario's calls of motion_planning's scene
//! members, this crate's planning scene reports what MoveIt's planning scene message
//! holds. At each configuration of the left arm, the pairs that `Surroundings` checks of
//! the arm's bodies, the objects, the voxel map as MoveIt's octomap holds it and the
//! parts of the robot the arm does not move are the pairs FCL measures of them. The
//! pieces of a link hold its collision mesh, so this crate's distance of each pair is no
//! larger than FCL's, or than 0 where FCL finds the pair in contact, and below 0 where
//! FCL finds it in contact deeper than its tolerance; a link whose mesh is concave reads
//! nearer than FCL where its pieces fill the mesh's hollows. Between two shapes of
//! objects, or an object and the map, which both measure alike, the distance equals FCL's
//! within FCL's tolerance. The map the calls leave, which each object added, moved or
//! attached cleared where it stood, reads the clearing's departure from MoveIt's octomap,
//! which keeps those voxels. Where one body lies wholly inside the other's closed mesh,
//! which FCL measures by its triangles and this crate as a solid, the distance is below
//! 0. Each scenario reads each case of `Exercised` at least once. On each path of a
//! scenario, this crate's check of a caller's path finds its first fault no later than
//! the first waypoint MoveIt's path check finds invalid, and a path MoveIt's check passes
//! that this crate's fails is tallied by where it fails; a path MoveIt's dense probe finds
//! clear past a margin passes. Of the paths MoveIt's check passes with its links padded by
//! the launch clearance, this crate's check at that clearance passes one where the object
//! the left hand holds comes nearer than it to its own arm. At a scale of 1, the objects'
//! cut-out holds a probe MoveIt's shape mask does not only within a mesh's padding, and
//! each probe the mask holds alone lies in its sharper growth.

mod arm_scenes;
mod common;
mod openarm_v2;

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use collision_geometry::chain::{ChainGeometry, Padding, Rider};
use collision_geometry::gjk::{Hull, Placed};
use collision_geometry::hull::exact_hull;
use collision_geometry::shapes::{PlacedShape, ShapeKind};
use motion_planning_core::{
    Arm, Arms, AttachRequest, AttachedObjectFields, CheckRequest, Contact, ObjectFields, Objects,
    PathCheck, PathFault, Place, PlanningScene, Reading, RobotLinks, RobotPart, Scene,
    SceneSnapshot, ShapeFields, Surroundings, TERMS, check_path, pose_from_parts,
};
use nalgebra::{Isometry3, Point3};
use occupancy_map::{DistanceField, VoxelMap};
use openarm_description::{ARM_DOF, ARMS, HardwareVersion, Side};
use ros_reference::planning_scene::{
    Cases, Measured, MoveItAttached, MoveItObject, MoveItShape, Operation, Path,
};

use arm_scenes::object_request;
use common::{held_from_its_arm, map, occupied_at};
use openarm_v2::Robot;

const JOINTS: usize = ARM_DOF;

const VERSION: HardwareVersion = HardwareVersion::V2;
/// The difference allowed between this crate's poses and MoveIt's: the rounding of the
/// same compositions of transforms.
const POSE_TOLERANCE: f64 = 1e-9;
/// The difference allowed between this crate's distance and FCL's where both measure the
/// same geometry, in meters: the tolerance at which FCL's GJK solver stops.
const DISTANCE_TOLERANCE_M: f64 = 1e-6;
/// The clearance at which `Surroundings` reads every pair's distance exactly: farther
/// than any two bodies of a scenario lie apart, in meters.
const EXACT_WITHIN_M: f64 = 5.0;
/// The half side of the cube about the robot's origin that the maps cover, in meters:
/// past every voxel of a scenario.
const MAP_HALF_SIDE_M: f64 = 1.6;
/// The name a pair gives the voxel map.
const MAP: &str = "occupancy_map";

/// The names of a pair's bodies, in sorted order.
type NamePair = (String, String);

fn sorted(a: String, b: String) -> NamePair {
    if a <= b { (a, b) } else { (b, a) }
}

/// The names of the joints of `arm`.
fn joint_names(arm: &ChainGeometry<JOINTS>) -> BTreeSet<&str> {
    (0..JOINTS).map(|i| arm.chain().joint_name(i)).collect()
}

/// OpenArm v2 on the meshes of `cases`: its arms, its body and its self model, the
/// fingers closed, as the reference stands each finger joint at 0, where OpenArm v2's
/// fingers close.
fn robot_of(cases: &Cases) -> Robot {
    let mut robot = Robot::new(cases.meshes.to_str().expect("the mesh directory is UTF-8"));
    robot.self_model.set_gripper_openings(&[0.0; ARMS]);
    robot
}

/// Every planning scene scenario with OpenArm v2 on its meshes, each checked to name
/// OpenArm v2's URDF and to pose its left arm at each configuration.
fn scenarios() -> Vec<(Cases, Robot)> {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a planning scene scenario"
    );
    scenarios
        .into_iter()
        .map(|cases| {
            let robot = robot_of(&cases);
            let name = &cases.name;
            assert!(
                cases.urdf == VERSION.urdf(),
                "{name}: the parity test poses OpenArm {VERSION}, whose URDF the scenario \
                 names"
            );
            assert_eq!(
                cases.tip_link,
                VERSION.tcp_link(Side::Left),
                "{name}: the configurations pose the left arm"
            );
            for (index, configuration) in cases.configurations.iter().enumerate() {
                let joints: BTreeSet<&str> =
                    configuration.positions.keys().map(String::as_str).collect();
                assert_eq!(
                    joints,
                    joint_names(&robot.arms.left),
                    "{name} configuration {index}: the joints of the left arm"
                );
            }
            (cases, robot)
        })
        .collect()
}

/// The joint positions of MoveIt's default state of OpenArm v2's arms: each at 0, which
/// lies inside the limits of every joint of either arm.
fn default_state(robot: &Robot) -> BTreeMap<String, f64> {
    robot
        .arms
        .iter()
        .flat_map(|arm| (0..JOINTS).map(move |i| (arm.chain().joint_name(i).to_string(), 0.0)))
        .collect()
}

/// Sets each joint `positions` names to its position in `state`.
fn set(state: &mut BTreeMap<String, f64>, positions: &BTreeMap<String, f64>) {
    for (joint, position) in positions {
        *state
            .get_mut(joint)
            .unwrap_or_else(|| panic!("{joint} is a joint of neither arm")) = *position;
    }
}

/// The positions of the joints of `arm` in `state`, in the chain's order.
fn positions_of(arm: &ChainGeometry<JOINTS>, state: &BTreeMap<String, f64>) -> [f64; JOINTS] {
    std::array::from_fn(|i| state[arm.chain().joint_name(i)])
}

/// The arm of `robot` whose grasp link is `link`.
fn arm_of<'r>(robot: &'r Robot, link: &str) -> &'r ChainGeometry<JOINTS> {
    let side = Side::BOTH
        .into_iter()
        .find(|side| VERSION.tcp_link(*side) == link)
        .unwrap_or_else(|| panic!("{link} is no grasp link of OpenArm {VERSION}"));
    &robot.arms[side]
}

/// The pose of the grasp link `link` in the frame of the URDF root, the arms at `state`.
fn grasp_pose(robot: &Robot, link: &str, state: &BTreeMap<String, f64>) -> Isometry3<f64> {
    let arm = arm_of(robot, link);
    arm.chain().world_ee_pose(&positions_of(arm, state))
}

/// How a scene takes the calls of a scenario: each attach with the touch links
/// `touch_links` makes of its own, and the calls of set_allowed_collision or none.
#[derive(Clone, Copy)]
struct Taking {
    touch_links: fn(&[String]) -> Vec<String>,
    allows: bool,
}

/// The calls of a scenario as they are.
const AS_CALLED: Taking = Taking {
    touch_links: <[String]>::to_vec,
    allows: true,
};

/// The calls of a scenario with no touch link and no allowed pair: every pair checked.
const CHECKING_EVERY_PAIR: Taking = Taking {
    touch_links: |_| Vec::new(),
    allows: false,
};

/// The voxel map of `cases` as the scene holds it from the start: each voxel of its
/// octomap occupied, by one scan of the voxels' centers from the robot's origin.
fn initial_map(cases: &Cases) -> VoxelMap {
    let mut voxels = map(cases.octomap.voxel_size, MAP_HALF_SIDE_M);
    mark(cases, &mut voxels, &cases.octomap.keys);
    assert_eq!(
        voxels.occupied().count(),
        cases.octomap.keys.len(),
        "{}: the map holds each voxel of the octomap",
        cases.name
    );
    voxels
}

/// Marks each voxel of `keys` occupied in `voxels`, a map of the scenario's voxel size, as
/// MoveIt's octomap marks a node: hits seen from the voxel's center, so no ray crosses
/// another voxel, until the voxel is occupied, from whatever log-odds it held.
fn mark(cases: &Cases, voxels: &mut VoxelMap, keys: &[[i32; 3]]) {
    let size = cases.octomap.voxel_size;
    for key in keys {
        let center = key.map(|k| (f64::from(k) + 0.5) * size);
        for _ in 0..HITS_TO_OCCUPY {
            if occupied_at_center(voxels, center) {
                break;
            }
            occupied_at(voxels, center, &[center]);
        }
        assert!(
            occupied_at_center(voxels, center),
            "{}: the map marks {key:?}",
            cases.name
        );
    }
}

/// Whether the voxel of `voxels` that holds `center` is occupied.
fn occupied_at_center(voxels: &VoxelMap, center: [f64; 3]) -> bool {
    voxels
        .key(center)
        .is_some_and(|key| voxels.is_occupied(key))
}

/// The hits that make a voxel occupied from the lowest log-odds: OctoMap's lower clamp,
/// probability 0.1192 or log-odds -2.0, its hit of probability 0.7 or log-odds 0.85, and
/// occupancy at log-odds 0 ask three.
const HITS_TO_OCCUPY: usize = 3;

/// The voxel map as MoveIt's octomap holds it after the calls: each voxel of the octomap
/// and each the calls mark, none cleared.
fn moveit_octomap(cases: &Cases) -> VoxelMap {
    let mut voxels = initial_map(cases);
    for operation in &cases.operations {
        if let Operation::Mark { keys } = operation {
            mark(cases, &mut voxels, keys);
        }
    }
    voxels
}

/// The voxels each kind of call clears: the batches that only add, the batch that moves,
/// and the attaches.
#[derive(Debug, Default)]
struct ClearedBy {
    adds: usize,
    moves: usize,
    attaches: usize,
}

/// The scene after the calls of `cases`, as `taking` takes them, on a robot of every link
/// of OpenArm v2 whose grasp links are its arms' grasp points; the joint positions of
/// MoveIt's state after them: each call of apply_attached_collision_object sets the joints
/// it names, then places its grasp link where they put it; and the voxel map the calls
/// leave, each object added, moved or attached clearing the voxels it intersects, each
/// mark marking its voxels; and the voxels each kind of call clears.
fn scene_after(
    cases: &Cases,
    robot: &Robot,
    taking: Taking,
) -> (PlanningScene, BTreeMap<String, f64>, VoxelMap, ClearedBy) {
    let tree = robot.arms.left.chain().tree();
    let links = (0..tree.link_count()).map(|i| tree.link(i).name.clone());
    let grasp_links = Side::BOTH.map(|side| VERSION.tcp_link(side).to_string());
    let mut scene = PlanningScene::new(RobotLinks::new(links, grasp_links));
    let mut state = default_state(robot);
    let mut voxels = initial_map(cases);
    let mut cleared = ClearedBy::default();
    for operation in &cases.operations {
        let before = voxels.occupied().count();
        let applied = match operation {
            Operation::Mark { keys } => {
                mark(cases, &mut voxels, keys);
                Ok(())
            }
            Operation::Objects { objects } => scene.apply_collision_objects(
                &objects.iter().map(object_request).collect::<Vec<_>>(),
                &mut voxels,
            ),
            Operation::Allow { .. } if !taking.allows => Ok(()),
            Operation::Allow {
                name,
                other_names,
                allowed,
            } => scene.set_allowed_collision(name, other_names, *allowed),
            Operation::Attached {
                object_id,
                link_name,
                touch_links: own,
                operation,
                positions,
            } => {
                set(&mut state, positions);
                scene.apply_attached_collision_object(
                    &AttachRequest {
                        object_id: object_id.clone(),
                        link_name: link_name.clone(),
                        touch_links: (taking.touch_links)(own),
                        operation: operation.clone(),
                    },
                    |link| Some(grasp_pose(robot, link, &state)),
                    &mut voxels,
                )
            }
        };
        applied.unwrap_or_else(|e| panic!("{}: {operation:?}: {e}", cases.name));
        let count = before.saturating_sub(voxels.occupied().count());
        match operation {
            Operation::Objects { objects } if objects.iter().any(|o| o.operation == "move") => {
                cleared.moves += count;
            }
            Operation::Objects { .. } => cleared.adds += count,
            Operation::Attached { operation, .. } if operation == "attach" => {
                cleared.attaches += count;
            }
            Operation::Attached { .. } | Operation::Allow { .. } | Operation::Mark { .. } => {}
        }
    }
    (scene, state, voxels, cleared)
}

fn assert_pose(what: &str, ours: &ObjectFields, position: &[f64; 3], orientation: &[f64; 4]) {
    let pose = pose_from_parts(ours.position, ours.orientation).expect("a pose");
    let theirs = pose_from_parts(*position, *orientation).expect("a pose");
    let along = (pose.translation.vector - theirs.translation.vector).norm();
    let turned = pose.rotation.angle_to(&theirs.rotation);
    assert!(
        along < POSE_TOLERANCE && turned < POSE_TOLERANCE,
        "{what}: at {:?} {:?}, MoveIt's {position:?} {orientation:?}",
        ours.position,
        ours.orientation
    );
}

/// Whether a shape of `kind` is a mesh, which MoveIt's message lists after the primitives.
fn is_mesh(kind: &str) -> bool {
    kind == "mesh"
}

/// Whether `shapes` holds a mesh before a primitive.
fn mesh_first(shapes: &[ShapeFields]) -> bool {
    let first_mesh = shapes.iter().position(|shape| is_mesh(&shape.kind));
    let last_primitive = shapes.iter().rposition(|shape| !is_mesh(&shape.kind));
    matches!((first_mesh, last_primitive), (Some(mesh), Some(primitive)) if mesh < primitive)
}

/// Asserts that `ours`, in the caller's order, holds the shapes MoveIt's message holds,
/// which lists an object's primitives in their order and then its meshes in theirs.
fn assert_shapes(what: &str, ours: &[ShapeFields], theirs: &[MoveItShape]) {
    assert_eq!(ours.len(), theirs.len(), "{what}: the number of shapes");
    let (meshes, primitives): (Vec<&ShapeFields>, Vec<&ShapeFields>) =
        ours.iter().partition(|shape| is_mesh(&shape.kind));
    for (shape, moveit) in primitives.into_iter().chain(meshes).zip(theirs) {
        assert_eq!(shape.kind, moveit.kind, "{what}: the kind of a shape");
        assert_eq!(shape.dimensions, moveit.dimensions, "{what}: dimensions");
        assert_eq!(
            shape.mesh_vertices, moveit.mesh_vertices,
            "{what}: vertices"
        );
        assert_eq!(
            shape.mesh_triangles, moveit.mesh_triangles,
            "{what}: triangles"
        );
        let pose = pose_from_parts(shape.position, shape.orientation).expect("a pose");
        let theirs = pose_from_parts(moveit.position, moveit.orientation).expect("a pose");
        assert!(
            (pose.translation.vector - theirs.translation.vector).norm() < POSE_TOLERANCE
                && pose.rotation.angle_to(&theirs.rotation) < POSE_TOLERANCE,
            "{what}: a shape's pose"
        );
    }
}

fn assert_object(name: &str, ours: &ObjectFields, theirs: &MoveItObject) {
    let what = format!("{name}: {}", ours.id);
    assert_eq!(ours.id, theirs.id, "{name}: the objects in order");
    assert_pose(&what, ours, &theirs.position, &theirs.orientation);
    assert_shapes(&what, &ours.shapes, &theirs.shapes);
}

fn assert_attached(name: &str, ours: &AttachedObjectFields, theirs: &MoveItAttached) {
    let what = format!("{name}: {}", ours.object.id);
    assert_eq!(
        ours.object.id, theirs.id,
        "{name}: the attached objects in order"
    );
    assert_eq!(ours.link_name, theirs.link, "{what}: its link");
    assert_eq!(
        ours.touch_links, theirs.touch_links,
        "{what}: its touch links"
    );
    assert_pose(&what, &ours.object, &theirs.position, &theirs.orientation);
    assert_shapes(&what, &ours.object.shapes, &theirs.shapes);
}

// After the same calls, get_planning_scene reports what MoveIt's planning scene message
// holds: the objects, the attached objects under the attached objects alone, and the
// pairs allowed to touch. An object's shapes keep the caller's order, and compare with
// the message's primitives and then its meshes; each scenario holds an object whose mesh
// comes before a primitive.
#[test]
fn the_snapshot_holds_what_moveits_message_holds() {
    for (cases, robot) in scenarios() {
        let name = &cases.name;
        let SceneSnapshot {
            objects,
            attached_objects,
            allowed_pairs,
            ..
        } = scene_after(&cases, &robot, AS_CALLED)
            .0
            .snapshot(0.0, cases.octomap.voxel_size);
        assert_eq!(
            objects.len(),
            cases.scene.objects.len(),
            "{name}: the objects"
        );
        objects
            .iter()
            .zip(&cases.scene.objects)
            .for_each(|(ours, theirs)| assert_object(name, ours, theirs));
        assert!(
            objects.iter().any(|object| mesh_first(&object.shapes)),
            "{name} holds no object whose mesh comes before a primitive"
        );
        assert_eq!(
            attached_objects.len(),
            cases.scene.attached.len(),
            "{name}: the attached objects"
        );
        attached_objects
            .iter()
            .zip(&cases.scene.attached)
            .for_each(|(ours, theirs)| assert_attached(name, ours, theirs));
        assert_eq!(
            allowed_pairs, cases.scene.allowed,
            "{name}: the allowed pairs"
        );
    }
}

/// The links the left arm moves: its chain's, and its fingers'.
fn moving_links<'a>(robot: &'a Robot, fingers: &[Rider<'a>]) -> BTreeSet<&'a str> {
    robot
        .arms
        .left
        .links()
        .iter()
        .map(|link| link.name())
        .chain(fingers.iter().map(|rider| rider.name))
        .collect()
}

/// The least distance of each pair of `readings`, by the pair's names, the map's named
/// "occupancy_map".
fn least_by_pair(readings: Vec<Reading>) -> BTreeMap<NamePair, f64> {
    readings
        .into_iter()
        .map(|Reading { pair, distance }| match pair {
            Contact::Map { body } => (sorted(body, MAP.to_string()), distance),
            Contact::Object { body, object } => (sorted(body, object), distance),
            Contact::Own { a, b } => (sorted(a, b), distance),
        })
        .fold(BTreeMap::new(), |mut least, (pair, distance)| {
            least
                .entry(pair)
                .and_modify(|d: &mut f64| *d = d.min(distance))
                .or_insert(distance);
            least
        })
}

/// `read` of the left arm of `robot` at `state`, its fingers closed and carrying its
/// objects, and of what it keeps clear of besides the map: the objects of `scene` and the
/// parts of the robot it does not move, where the self model places them.
fn with_the_left_arm<R>(
    robot: &Robot,
    scene: &PlanningScene,
    state: &BTreeMap<String, f64>,
    read: impl FnOnce(&Arms<JOINTS, JOINTS>, Objects) -> R,
) -> R {
    let left = positions_of(&robot.arms.left, state);
    let right = positions_of(&robot.arms.right, state);
    let objects = scene.for_arms(&[VERSION.tcp_link(Side::Left)], |link| {
        grasp_pose(robot, link, state)
    });
    let fingers = robot.fingers(&robot.self_model).left;
    let moving = moving_links(robot, &fingers);
    let mut placed = robot.self_model.clone();
    let parts: Vec<RobotPart> = placed
        .world_pieces(&[left, right])
        .expect("finite joint positions")
        .into_iter()
        .filter(|(link, _)| !moving.contains(*link))
        .flat_map(|(link, pieces)| {
            pieces
                .into_iter()
                .map(move |piece| RobotPart { link, piece })
        })
        .collect();
    let riders: Vec<Rider> = fingers
        .iter()
        .copied()
        .chain(scene.riders(VERSION.tcp_link(Side::Left), &robot.arms.left))
        .collect();
    let arms = Arms::new(
        vec![
            Arm::new("arm", &robot.arms.left)
                .riding(&riders)
                .expect("the fingers and the objects ride the hand"),
        ],
        None,
    );
    read(&arms, objects.objects(&parts))
}

/// The distance of each pair that `Surroundings` checks of the left arm of `robot` at
/// `state`, as `with_the_left_arm` places it, and the voxel map `voxels`; by the pair's
/// names.
fn read_pairs(
    robot: &Robot,
    scene: &PlanningScene,
    state: &BTreeMap<String, f64>,
    voxels: &(VoxelMap, DistanceField),
) -> BTreeMap<NamePair, f64> {
    let (map, field) = voxels;
    with_the_left_arm(robot, scene, state, |arms, objects| {
        let surroundings =
            Surroundings::new(map, field, [EXACT_WITHIN_M; TERMS]).with_objects(objects);
        least_by_pair(surroundings.readings(arms, &positions_of(&robot.arms.left, state)))
    })
}

/// Whether `point` lies inside the box of the vertices of the mesh `mesh`, in the mesh's
/// frame; never for a shape of another kind.
fn in_the_box_of(mesh: &PlacedShape, point: &Point3<f64>) -> bool {
    let ShapeKind::Mesh { vertices, .. } = mesh.shape().kind() else {
        return false;
    };
    let local = mesh.pose().inverse_transform_point(point);
    (0..3).all(|axis| {
        let (low, high) = vertices
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), v| {
                (low.min(v[axis]), high.max(v[axis]))
            });
        (low..=high).contains(&local[axis])
    })
}

/// The cases a scenario reads, each by the count of the pairs that show it.
#[derive(Default)]
struct Exercised {
    /// A shape of an object that is no mesh in contact deeper than FCL's tolerance.
    primitive_in_contact: usize,
    /// Two shapes of objects apart, which measure alike.
    apart: usize,
    /// A link of the left arm that reads apart from a mesh object, the center of each of
    /// its pieces inside the box of the mesh: in the mesh's open corner.
    open_corner: usize,
    /// One body wholly inside the other's closed mesh.
    inside: usize,
    /// An object the left arm carries and a link of the arm it may not touch.
    carried_and_own_link: usize,
    /// An object the left arm carries and a part of the robot the arm does not move.
    carried_and_part: usize,
    /// An object the right arm holds and a body of the left arm.
    held_by_the_other_arm: usize,
    /// An object the left arm carries and one of its touch links, overlapping, which
    /// neither checks.
    touching: usize,
    /// An allowed pair, overlapping, which neither checks.
    allowed_overlapping: usize,
    /// An object of several shapes and a body of the left arm.
    several_shapes: usize,
    /// A body of the left arm and the voxel map.
    map: usize,
}

impl Exercised {
    fn each(&self) -> [(&'static str, usize); 11] {
        [
            ("a primitive in contact", self.primitive_in_contact),
            ("two shapes of objects apart", self.apart),
            ("a link in a mesh's open corner", self.open_corner),
            ("a body inside a mesh", self.inside),
            ("a carried object and its arm", self.carried_and_own_link),
            ("a carried object and a part", self.carried_and_part),
            ("an object the other arm holds", self.held_by_the_other_arm),
            ("a carried object on its touch link", self.touching),
            ("an allowed pair overlapping", self.allowed_overlapping),
            ("an object of several shapes", self.several_shapes),
            ("a body and the map", self.map),
        ]
    }
}

// At each configuration, `Surroundings` checks the pairs FCL measures of the left arm's
// bodies, each pair's distance bounds FCL's as the module says, and each scenario reads
// each case of `Exercised`.
#[test]
fn the_pairs_and_their_distances_bound_fcls() {
    for (cases, robot) in &scenarios() {
        assert_the_pairs_bound_fcls(cases, robot);
    }
}

fn assert_the_pairs_bound_fcls(cases: &Cases, robot: &Robot) {
    let name = &cases.name;
    let (scene, called, cleared, cleared_by) = scene_after(cases, robot, AS_CALLED);
    let (unexempt, ..) = scene_after(cases, robot, CHECKING_EVERY_PAIR);
    let snapshot = scene.snapshot(0.0, cases.octomap.voxel_size);
    let allowed: BTreeSet<NamePair> = snapshot
        .allowed_pairs
        .into_iter()
        .map(|[a, b]| (a, b))
        .collect();
    let left_grasp = VERSION.tcp_link(Side::Left);
    let carried: BTreeSet<&str> = snapshot
        .attached_objects
        .iter()
        .filter(|attached| attached.link_name == left_grasp)
        .map(|attached| attached.object.id.as_str())
        .collect();
    let held: BTreeSet<&str> = snapshot
        .attached_objects
        .iter()
        .filter(|attached| attached.link_name != left_grasp)
        .map(|attached| attached.object.id.as_str())
        .collect();
    let several: BTreeSet<&str> = snapshot
        .objects
        .iter()
        .filter(|object| object.shapes.len() > 1)
        .map(|object| object.id.as_str())
        .collect();
    let tree = robot.arms.left.chain().tree();
    let links: BTreeSet<&str> = (0..tree.link_count())
        .map(|i| tree.link(i).name.as_str())
        .collect();
    // The pairs are read on the voxel map as MoveIt's octomap holds it, from the start;
    // the map the calls leave, which each object added, moved or attached cleared where it
    // stood, reads the clearing's departure from MoveIt.
    let moveit = moveit_octomap(cases);
    let cleared_voxels = moveit.occupied().count() - cleared.occupied().count();
    let moveit_field = DistanceField::of(&moveit);
    let as_moveit_holds_it = (moveit, moveit_field);
    let cleared_field = DistanceField::of(&cleared);
    let as_cleared = (cleared, cleared_field);
    let fingers = robot.fingers(&robot.self_model).left;
    let mut exercised = Exercised::default();
    // The most this crate reads a pair that both read apart nearer than FCL, and the
    // deepest contact FCL measures of a pair this crate reads apart.
    let (mut nearer, mut apart_in_contact) = (0.0_f64, 0.0_f64);
    let mut clearing = Clearing {
        voxels: cleared_voxels,
        by: cleared_by,
        ..Clearing::default()
    };
    for (index, configuration) in cases.configurations.iter().enumerate() {
        let what = format!("{name} configuration {index}");
        let mut state = called.clone();
        set(&mut state, &configuration.positions);
        let ours = read_pairs(robot, &scene, &state, &as_moveit_holds_it);
        let unchecked = read_pairs(robot, &unexempt, &state, &as_moveit_holds_it);
        let after_clearing = read_pairs(robot, &scene, &state, &as_cleared);
        let objects = scene.for_arms(&[left_grasp], |link| grasp_pose(robot, link, &state));
        let meshes: BTreeMap<&str, Vec<&PlacedShape>> = objects
            .obstacles()
            .iter()
            .map(|obstacle| {
                let mesh_shapes: Vec<&PlacedShape> = obstacle
                    .shapes()
                    .iter()
                    .filter(|shape| matches!(shape.shape().kind(), ShapeKind::Mesh { .. }))
                    .collect();
                (obstacle.id(), mesh_shapes)
            })
            .filter(|(_, mesh_shapes)| !mesh_shapes.is_empty())
            .collect();
        let moving: BTreeSet<&str> = moving_links(robot, &fingers)
            .into_iter()
            .chain(carried.iter().copied())
            .collect();
        let fcl: BTreeMap<NamePair, &Measured> = configuration
            .measured
            .iter()
            .filter(|m| moving.contains(m.a.as_str()) || moving.contains(m.b.as_str()))
            .map(|m| ((m.a.clone(), m.b.clone()), m))
            .collect();
        let only_ours: Vec<&NamePair> = ours.keys().filter(|p| !fcl.contains_key(*p)).collect();
        let only_fcl: Vec<&NamePair> = fcl.keys().filter(|p| !ours.contains_key(*p)).collect();
        assert!(
            only_ours.is_empty() && only_fcl.is_empty(),
            "{what}: `Surroundings` checks {only_ours:?} alone, and FCL measures {only_fcl:?} \
             alone"
        );
        let pieces: Vec<(&str, Placed)> = robot
            .arms
            .left
            .place(&positions_of(&robot.arms.left, &state))
            .pieces()
            .collect();
        // Whether the link `link` lies in the open corner of a mesh of `object`.
        let in_open_corner = |link: &str, object: &str| {
            let centers: Vec<Point3<f64>> = pieces
                .iter()
                .filter(|(of, _)| *of == link)
                .map(|(_, piece)| piece.bounding_sphere().0)
                .collect();
            !centers.is_empty()
                && meshes.get(object).is_some_and(|mesh_shapes| {
                    mesh_shapes
                        .iter()
                        .any(|mesh| centers.iter().all(|center| in_the_box_of(mesh, center)))
                })
        };
        for (pair, measured) in &fcl {
            let (a, b) = (pair.0.as_str(), pair.1.as_str());
            let here = ours[pair];
            let theirs = measured.distance;
            let shown = format!("{what}: {a} and {b} lie {here} m apart here, {theirs} m by FCL");
            if measured.inside {
                assert!(here < 0.0, "{shown}, one wholly inside the other's mesh");
                exercised.inside += 1;
                continue;
            }
            assert!(
                here <= theirs.max(0.0) + DISTANCE_TOLERANCE_M,
                "{shown}: no farther"
            );
            if here > 0.0 && theirs > 0.0 {
                nearer = nearer.max(theirs - here);
            }
            if here >= 0.0 {
                apart_in_contact = apart_in_contact.max(-theirs);
            }
            if !links.contains(a) && !links.contains(b) && theirs > 0.0 {
                assert!(
                    here >= theirs - DISTANCE_TOLERANCE_M,
                    "{shown}: as far, two shapes measured alike"
                );
                exercised.apart += 1;
            }
            exercised.open_corner +=
                usize::from(here > 0.0 && (in_open_corner(a, b) || in_open_corner(b, a)));
            if theirs < -DISTANCE_TOLERANCE_M {
                assert!(here < 0.0, "{shown}: in contact");
                exercised.primitive_in_contact +=
                    usize::from(!meshes.contains_key(a) && !meshes.contains_key(b));
            }
            let names = [a, b];
            let carries = names.iter().any(|n| carried.contains(n));
            exercised.carried_and_own_link += usize::from(
                carries
                    && names
                        .iter()
                        .any(|n| links.contains(n) && moving.contains(n)),
            );
            exercised.carried_and_part += usize::from(
                carries
                    && names
                        .iter()
                        .any(|n| links.contains(n) && !moving.contains(n)),
            );
            exercised.held_by_the_other_arm += usize::from(names.iter().any(|n| held.contains(n)));
            exercised.several_shapes += usize::from(names.iter().any(|n| several.contains(n)));
            if names.contains(&MAP) {
                exercised.map += 1;
                let moved = after_clearing[pair] > here + DISTANCE_TOLERANCE_M;
                clearing.pairs_moved += usize::from(moved);
                clearing.contacts_cleared +=
                    usize::from(moved && theirs < 0.0 && after_clearing[pair] >= 0.0);
            }
        }
        for (pair, distance) in &unchecked {
            if *distance >= 0.0 || ours.contains_key(pair) {
                continue;
            }
            if allowed.contains(pair) {
                exercised.allowed_overlapping += 1;
            } else {
                exercised.touching += 1;
            }
        }
    }
    for (case, count) in exercised.each() {
        assert!(count >= 1, "{name} reads no pair of the case {case}");
    }
    assert!(
        clearing.voxels >= 1 && clearing.contacts_cleared >= 1,
        "{name}: no voxel the calls clear meets a body of the arm: {clearing:?}"
    );
    assert!(
        clearing.by.adds >= 1 && clearing.by.moves >= 1 && clearing.by.attaches >= 1,
        "{name}: an add, a move and an attach each clear a voxel: {clearing:?}"
    );
    println!("{name}: {:?}", exercised.each());
    println!(
        "{name}: of the pairs both read apart, this crate reads one at most {nearer:.4} m \
         nearer than FCL; of those it reads apart, FCL measures one at most \
         {apart_in_contact:e} m in contact"
    );
    println!("{name}: the clearing's departure from MoveIt's octomap: {clearing:?}");
}

/// What the clearing of the voxels an object intersects where it is added, moved or
/// attached moves, against MoveIt's octomap, which keeps them.
#[derive(Debug, Default)]
struct Clearing {
    /// The voxels the calls clear.
    voxels: usize,
    /// The voxels each kind of call clears.
    by: ClearedBy,
    /// The pairs of a body and the map, over every configuration, that read farther on the
    /// map the calls leave than on MoveIt's.
    pairs_moved: usize,
    /// Those that FCL finds in contact, and the map the calls leave apart: an object
    /// attached among voxels it filled, which MoveIt's octomap keeps.
    contacts_cleared: usize,
}

/// Where along a path a fault of this crate's check lies, in the path's order: at the
/// start or at a waypoint, or on the corner rounded at one, by the waypoint's index, the
/// start 0; or on the run from a waypoint, just past it.
fn along_the_path(fault: &PathFault) -> (usize, bool) {
    match fault {
        PathFault::Start(_) => (0, false),
        PathFault::OutsideLimits { waypoint, .. } => (*waypoint, false),
        PathFault::Clearance { place, .. } => match *place {
            Place::Waypoint(k) | Place::Corner(k) => (k, false),
            Place::Run(k) => (k, true),
        },
    }
}

/// What this crate's check of the scenario's paths finds against MoveIt's path check.
#[derive(Debug, Default)]
struct PathTally {
    /// Paths both find valid.
    both_pass: usize,
    /// Paths both fail, this crate's first fault at MoveIt's first invalid waypoint.
    same_waypoint: usize,
    /// Paths both fail, this crate's first fault before MoveIt's first invalid waypoint.
    earlier: usize,
    /// Paths MoveIt's check passes that this crate's fails on a run or a corner, between
    /// waypoints MoveIt checks alone.
    between_waypoints: usize,
    /// Paths MoveIt's check passes that this crate's fails at a waypoint: within the
    /// plan's clearance, or where a link's pieces fill its mesh's hollows.
    at_a_waypoint: usize,
    /// Paths MoveIt's dense probe finds clear, `DENSE_MARGIN_M` or more from the objects
    /// and the octomap at every probe, each of which this crate's check passes.
    clear_by_dense_probe: usize,
}

/// How far MoveIt's dense probe of a path must find it from the objects and the octomap
/// for this crate's check of it to pass: the most this crate reads a body nearer an object
/// than FCL where both read them apart, 16.6 mm, the pieces of openarm_left_link3, fitted
/// to its mesh, from the tote at the first configuration (as
/// `the_pairs_and_their_distances_bound_fcls` prints it), and the 1 mm the check's
/// clearance of 0 holds from the map and the objects, rounded up to the centimeter.
const DENSE_MARGIN_M: f64 = 0.02;

// On each path of each scenario, the left arm's check at a clearance of 0, its fingers
// closed and carrying its objects, among the objects, the parts that stand still and the
// voxel map as MoveIt's octomap holds it, finds its first fault no later than the first
// waypoint MoveIt's path check finds invalid; the path through the post fails on its run,
// which MoveIt checks at its waypoints alone, and passes there; and each path MoveIt's
// dense probe finds `DENSE_MARGIN_M` or more from the objects and the octomap, and in
// contact nowhere, passes, the paths drawn among the objects among them. The tally of the
// paths MoveIt passes and this crate fails says where.
#[test]
fn a_path_check_fails_no_later_than_moveits() {
    for (cases, robot) in &scenarios() {
        let name = &cases.name;
        let (scene, called, ..) = scene_after(cases, robot, AS_CALLED);
        let voxels = moveit_octomap(cases);
        let field = DistanceField::of(&voxels);
        let mut tally = PathTally::default();
        for (index, path) in cases.paths.iter().enumerate() {
            let what = format!("{name} path {index}");
            let checked =
                PathOf::new(robot, &called, path).check(robot, &scene, (&voxels, &field), 0.0);
            let ours = checked.fault.as_ref().map(along_the_path);
            if !path.dense.colliding && path.dense.clearance >= DENSE_MARGIN_M {
                assert!(
                    ours.is_none(),
                    "{what}: MoveIt's dense probe finds the path {} m from the objects, and \
                     this crate's check fails: {:?}",
                    path.dense.clearance,
                    checked.fault
                );
                tally.clear_by_dense_probe += 1;
            }
            match (path.invalid.first(), ours) {
                (None, None) => tally.both_pass += 1,
                (Some(&first), Some(at)) => {
                    assert!(
                        at <= (first, false),
                        "{what}: MoveIt finds waypoint {first} invalid, and this crate's \
                         check fails later: {:?}",
                        checked.fault
                    );
                    if at == (first, false) {
                        tally.same_waypoint += 1;
                    } else {
                        tally.earlier += 1;
                    }
                }
                (Some(first), None) => {
                    panic!(
                        "{what}: MoveIt finds waypoint {first} invalid, and this crate's check \
                         passes"
                    )
                }
                (None, Some((_, true))) => tally.between_waypoints += 1,
                (None, Some(_))
                    if matches!(
                        checked.fault,
                        Some(PathFault::Clearance {
                            place: Place::Corner(_),
                            ..
                        })
                    ) =>
                {
                    tally.between_waypoints += 1
                }
                (None, Some(_)) => tally.at_a_waypoint += 1,
            }
        }
        let through_the_post = cases.paths.first().expect("the path through the post");
        assert!(
            through_the_post.invalid.is_empty() && tally.between_waypoints >= 1,
            "{name}: the run through the post: {tally:?}"
        );
        assert!(
            tally.clear_by_dense_probe >= 1,
            "{name}: no path MoveIt's dense probe finds clear: {tally:?}"
        );
        println!("{name}: the paths, this crate's check against MoveIt's: {tally:?}");
    }
}

/// A path of a scenario as the left arm runs it: the state at each waypoint, the rest of
/// the robot as the calls leave it, and the left arm's joints there.
struct PathOf {
    states: Vec<BTreeMap<String, f64>>,
    joints: Vec<[f64; JOINTS]>,
}

impl PathOf {
    fn new(robot: &Robot, called: &BTreeMap<String, f64>, path: &Path) -> Self {
        let states: Vec<BTreeMap<String, f64>> = path
            .waypoints
            .iter()
            .map(|waypoint| {
                let mut state = called.clone();
                set(&mut state, waypoint);
                state
            })
            .collect();
        let joints = states
            .iter()
            .map(|state| positions_of(&robot.arms.left, state))
            .collect();
        Self { states, joints }
    }

    /// The left arm's check of the path at `clearance`, its fingers closed and carrying its
    /// objects, among `scene`'s objects, the parts that stand still and `voxels`.
    fn check(
        &self,
        robot: &Robot,
        scene: &PlanningScene,
        (voxels, field): (&VoxelMap, &DistanceField),
        clearance: f64,
    ) -> PathCheck {
        with_the_left_arm(robot, scene, &self.states[0], |arms, objects| {
            check_path(
                arms,
                &Scene {
                    map: voxels,
                    field,
                    stamp: 1,
                    objects,
                },
                &CheckRequest {
                    start: self.joints[0],
                    start_tolerance: [0.0; JOINTS],
                    waypoints: &self.joints[1..],
                    min_clearance_m: clearance,
                    deadline: Instant::now() + Duration::from_secs(3600),
                },
            )
            .expect("a check of a valid request answers")
        })
    }

    /// The least distance, along the path, between an object the left hand carries and a
    /// piece of the left arm it may not touch: outside its link, its touch links and the
    /// pairs allowed to touch, as `snapshot`, the scene's, names them. The joints step at
    /// most `HELD_STEP_RAD` between two states.
    fn held_from_the_arm(
        &self,
        robot: &Robot,
        scene: &PlanningScene,
        snapshot: &SceneSnapshot,
    ) -> f64 {
        let may_touch = |object: &str, link: &str| {
            snapshot.attached_objects.iter().any(|attached| {
                attached.object.id == object
                    && (attached.link_name == link
                        || attached.touch_links.iter().any(|l| l == link))
            }) || snapshot
                .allowed_pairs
                .iter()
                .any(|[a, b]| (a == object && b == link) || (a == link && b == object))
        };
        let states: Vec<[f64; JOINTS]> = self
            .joints
            .windows(2)
            .flat_map(|pair| {
                let widest = (0..JOINTS)
                    .map(|i| (pair[1][i] - pair[0][i]).abs())
                    .fold(0.0, f64::max);
                let steps = (widest / HELD_STEP_RAD).ceil().max(1.0) as usize;
                (0..=steps).map(move |s| {
                    let share = s as f64 / steps as f64;
                    std::array::from_fn(|i| pair[0][i] + (pair[1][i] - pair[0][i]) * share)
                })
            })
            .collect();
        with_the_left_arm(robot, scene, &self.states[0], |arms, _| {
            states
                .iter()
                .map(|q| held_from_its_arm(arms, q, may_touch))
                .fold(f64::INFINITY, f64::min)
        })
    }
}

/// The most the joints turn between two states at which a path's held distance is read:
/// MoveIt's dense probe's step.
const HELD_STEP_RAD: f64 = 1e-3;

// Of the paths of each scenario that MoveIt checks again with its links padded by the
// scenario's path padding, the motion planner's 4 cm, and finds valid at each waypoint and
// in contact nowhere at its dense probe, this crate's check at that clearance passes at
// least one with the item the left hand holds nearer than the clearance to a piece of the
// left arm it may not touch, out of contact: an attached object keeps 0 from the arm that
// carries it, as MoveIt checks it unpadded, its self check. The pair is rigid, the item and
// openarm_left_ee_base_link, which a fixed joint carries, 9.9 mm apart on every path. The
// test prints the fault of each path this crate's check fails, with its term and pair.
#[test]
fn at_the_launch_clearance_a_held_object_near_its_arm_passes_both_checks() {
    for (cases, robot) in &scenarios() {
        let name = &cases.name;
        let (scene, called, ..) = scene_after(cases, robot, AS_CALLED);
        let voxels = moveit_octomap(cases);
        let field = DistanceField::of(&voxels);
        let clearance = cases.path_padding;
        let snapshot = scene.snapshot(0.0, cases.octomap.voxel_size);
        let (mut held_near, mut moveit_passes, mut ours_fails) = (Vec::new(), 0, Vec::new());
        for (index, path) in cases.paths.iter().enumerate() {
            if !path.padded.invalid.is_empty() || path.padded.colliding {
                continue;
            }
            moveit_passes += 1;
            let of = PathOf::new(robot, &called, path);
            let checked = of.check(robot, &scene, (&voxels, &field), clearance);
            if let Some(fault) = &checked.fault {
                ours_fails.push(format!(
                    "path {index}, MoveIt {:.4} m clear: {fault}",
                    path.dense.clearance
                ));
            }
            let held = of.held_from_the_arm(robot, &scene, &snapshot);
            if checked.fault.is_none() && 0.0 < held && held < clearance {
                held_near.push((index, held));
            }
        }
        assert!(
            !held_near.is_empty(),
            "{name}: no path both checks pass at {clearance} m brings the held item nearer \
             than that to its arm"
        );
        println!(
            "{name}: MoveIt's padded check passes {moveit_passes} paths; this crate's at \
             {clearance} m fails {ours_fails:#?}, and passes these with the held item nearer \
             its arm than {clearance} m (index, least distance): {held_near:?}"
        );
    }
}

// Two objects attached to one link, the cap reaching into the item, touch: MoveIt's
// collision check never finds them in contact (bodies attached to one link always may
// touch), and `Surroundings` reads no pair of them, so the state the calls leave keeps
// the clearance between them.
#[test]
fn objects_one_arm_carries_are_never_checked_against_each_other() {
    for (cases, robot) in &scenarios() {
        let name = &cases.name;
        assert!(
            cases.one_link_pairs.iter().any(|pair| pair.distance < 0.0),
            "{name} holds no two objects on one link that touch"
        );
        let (scene, called, voxels, _) = scene_after(cases, robot, AS_CALLED);
        let field = DistanceField::of(&voxels);
        let readings = with_the_left_arm(robot, &scene, &called, |arms, objects| {
            Surroundings::new(&voxels, &field, [EXACT_WITHIN_M; TERMS])
                .with_objects(objects)
                .readings(arms, &positions_of(&robot.arms.left, &called))
        });
        let read: BTreeSet<NamePair> = least_by_pair(readings).into_keys().collect();
        for pair in &cases.one_link_pairs {
            let [a, b] = pair.pair.clone();
            assert!(!pair.contact, "{name}: MoveIt finds {a} and {b} in contact");
            assert!(
                !read.contains(&sorted(a.clone(), b.clone())),
                "{name}: `Surroundings` checks {a} against {b}"
            );
        }
    }
}

/// Whether `point` lies inside `shape` grown by sharp planes at `offset` with a scale of 1:
/// a box's half sides and a cylinder's radius and half height the offset longer, as
/// MoveIt's shape mask grows them, a sphere's radius the offset longer, and a mesh's convex
/// hull with each face moved out by the offset, which holds the mask's growth of the hull,
/// each of whose vertices it moves the offset out from the hull's center.
fn inside_sharp_growth(shape: &PlacedShape, point: &Point3<f64>, offset: f64) -> bool {
    let local = shape.pose().inverse_transform_point(point);
    match shape.shape().kind() {
        ShapeKind::Box { size } => (0..3).all(|i| local[i].abs() <= size[i] / 2.0 + offset),
        ShapeKind::Sphere { radius } => local.coords.norm() <= radius + offset,
        ShapeKind::Cylinder { radius, height } => {
            local.x.hypot(local.y) <= radius + offset && local.z.abs() <= height / 2.0 + offset
        }
        ShapeKind::Mesh { vertices, .. } => {
            let hull =
                Hull::new(&exact_hull(vertices).expect("a mesh's hull"), 0.0).expect("a hull");
            hull.planes().iter().all(|(normal, plane)| {
                normal.dot(&local.coords) - plane <= offset + GROWTH_ROUNDING_M
            })
        }
    }
}

/// How far past a plane of MoveIt's growth a point that MoveIt's shape mask holds may lie:
/// the rounding of the hull's planes and of placing the point in the shape's frame.
const GROWTH_ROUNDING_M: f64 = 1e-9;

/// What this crate's cut-out of the objects holds of the probes against MoveIt's shape
/// mask, at one padding.
#[derive(Debug, Default)]
struct CutOutTally {
    both: usize,
    neither: usize,
    /// Probes MoveIt's sharper growth holds and this crate's rounded one does not.
    moveit_only: usize,
    /// Probes this crate's growth holds and MoveIt's does not: at a scale of 1, within the
    /// offset of a mesh, whose hull's faces MoveIt's growth moves out by less than the
    /// offset.
    ours_only: usize,
}

// At each padding of each scenario, the probes the objects the calls leave hold, standing
// or attached, grown by this crate's rule (`PlacedShape::contains`) and by MoveIt's shape
// mask, which the planning scene monitor fills with both. At a
// scale of 1, each probe this crate's cut-out holds and MoveIt's does not lies within the
// offset of a mesh, whose hull MoveIt's mask grows by moving each vertex the offset out
// from the hull's center, so that a face moves out by less; and each probe MoveIt's holds
// alone lies inside the sharp growth of a shape, past this crate's rounded one: beyond a
// box's or a cylinder's edge, or inside a mesh's convex hull out of the mesh. The tally of
// each padding says how many.
#[test]
fn the_cut_out_of_the_objects_departs_from_moveits_at_edges_and_meshes_alone() {
    for (cases, robot) in &scenarios() {
        let name = &cases.name;
        let (scene, called, ..) = scene_after(cases, robot, AS_CALLED);
        // The objects where they stand or another arm holds them, and the objects the left
        // arm carries where the calls leave it: the riders an intake cuts out with them.
        let objects = scene.for_arms(&[VERSION.tcp_link(Side::Left)], |link| {
            grasp_pose(robot, link, &called)
        });
        let riders = scene.riders(VERSION.tcp_link(Side::Left), &robot.arms.left);
        let arms = Arms::new(
            vec![
                Arm::new("arm", &robot.arms.left)
                    .riding(&riders)
                    .expect("the objects ride the hand"),
            ],
            None,
        );
        let bodies = arms.bodies(&positions_of(&robot.arms.left, &called));
        let standing: Vec<&PlacedShape> = objects
            .obstacles()
            .iter()
            .flat_map(|obstacle| obstacle.shapes())
            .chain(bodies.iter().filter_map(|body| body.body.shape()))
            .collect();
        for (padding, theirs) in cases.cut_out.paddings.iter().zip(&cases.cut_out.inside) {
            let grown = Padding::new(padding.scale, padding.offset).expect("a padding");
            let mut tally = CutOutTally::default();
            for (probe, &moveit) in cases.cut_out.probes.iter().zip(theirs) {
                let point = Point3::from(*probe);
                let ours = standing.iter().any(|shape| shape.contains(&point, grown));
                match (ours, moveit) {
                    (true, true) => tally.both += 1,
                    (false, false) => tally.neither += 1,
                    (false, true) => tally.moveit_only += 1,
                    (true, false) => tally.ours_only += 1,
                }
                if padding.scale == 1.0 && moveit && !ours {
                    assert!(
                        standing.iter().any(|shape| inside_sharp_growth(
                            shape,
                            &point,
                            padding.offset
                        )),
                        "{name}: MoveIt's mask holds {probe:?} at {padding:?}, inside no sharp \
                         growth"
                    );
                }
                if padding.scale == 1.0 && ours && !moveit {
                    assert!(
                        standing.iter().any(|shape| matches!(
                            shape.shape().kind(),
                            ShapeKind::Mesh { .. }
                        ) && shape.contains(&point, grown)),
                        "{name}: the cut-out holds {probe:?} at {padding:?}, which MoveIt's mask \
                         does not, outside the padding of every mesh"
                    );
                }
            }
            assert!(
                tally.both > 0 && tally.neither > 0,
                "{name}: {padding:?} {tally:?}"
            );
            println!("{name}: the cut-out at {padding:?} against MoveIt's mask: {tally:?}");
        }
    }
}
