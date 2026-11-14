//! The rules of the planning scene, on a robot of two hands, each with two fingers, on a
//! torso: a batch of objects applies all or none; "add" replaces and keeps the pairs,
//! "move" and "remove" read only what they use, and "remove" drops every pair that names
//! the object; ids, attached objects and open meshes are refused; only a grasp link
//! carries an object, where it stands, and an attach and a detach keep the pairs; a pair
//! names an object; the snapshot is what was set, with the clearance and the voxel size
//! it is given; adding, moving or attaching an object clears the map where it stands, a
//! batch where it leaves the object; and an empty batch, or a pair of an object with no
//! other name, changes nothing.

mod common;
mod openarm_v2;

use motion_planning_core::{
    AttachRequest, OCCUPANCY_MAP, ObjectFields, ObjectRequest, PlanningScene, RobotLinks,
    SceneError, ShapeFields, pose_from_parts,
};
use nalgebra::{Isometry3, Vector3};
use occupancy_map::VoxelMap;

use common::{map, occupied_at};

const VOXEL_M: f64 = 0.02;
/// The clearance the scene's snapshot reports.
const CLEARANCE_M: f64 = 0.04;
/// The difference allowed between two poses composed of the same transforms in another
/// order: their rounding.
const POSE_TOLERANCE: f64 = 1e-12;
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

/// Two hands, each the grasp link of its arm, with two fingers each, on a torso.
fn robot() -> RobotLinks {
    RobotLinks::new(
        [
            "torso",
            "left_hand",
            "left_finger_a",
            "left_finger_b",
            "right_hand",
            "right_finger_a",
            "right_finger_b",
        ]
        .map(String::from),
        ["left_hand", "right_hand"].map(String::from),
    )
}

/// Where each hand stands: the left one 0.4 m ahead of the robot, the right one beside
/// it.
fn hands(link: &str) -> Option<Isometry3<f64>> {
    match link {
        "left_hand" => Some(Isometry3::translation(0.4, 0.2, 0.3)),
        "right_hand" => Some(Isometry3::translation(0.4, -0.2, 0.3)),
        _ => None,
    }
}

fn shape(kind: &str, dimensions: &[f64]) -> ShapeFields {
    ShapeFields {
        kind: kind.to_string(),
        dimensions: dimensions.to_vec(),
        mesh_vertices: Vec::new(),
        mesh_triangles: Vec::new(),
        position: [0.0; 3],
        orientation: IDENTITY,
    }
}

fn cube(side: f64) -> ShapeFields {
    shape("box", &[side; 3])
}

fn object(id: &str, position: [f64; 3], shapes: Vec<ShapeFields>) -> ObjectFields {
    ObjectFields {
        id: id.to_string(),
        position,
        orientation: IDENTITY,
        shapes,
    }
}

fn add(id: &str, position: [f64; 3], shapes: Vec<ShapeFields>) -> ObjectRequest {
    ObjectRequest {
        operation: "add".to_string(),
        object: object(id, position, shapes),
    }
}

/// A move of `id` to `position`, with a shape no add takes, which a move does not read.
fn move_to(id: &str, position: [f64; 3]) -> ObjectRequest {
    ObjectRequest {
        operation: "move".to_string(),
        object: object(id, position, vec![shape("cone", &[])]),
    }
}

/// A removal of `id`, at a position that is not one, which a removal does not read.
fn remove(id: &str) -> ObjectRequest {
    ObjectRequest {
        operation: "remove".to_string(),
        object: object(id, [f64::NAN; 3], Vec::new()),
    }
}

fn attach(id: &str, link: &str, touch_links: &[&str]) -> AttachRequest {
    AttachRequest {
        object_id: id.to_string(),
        link_name: link.to_string(),
        touch_links: touch_links.iter().map(|l| l.to_string()).collect(),
        operation: "attach".to_string(),
    }
}

/// A detach of `id`, naming a link and a touch link the robot does not have, which a
/// detach does not read.
fn detach(id: &str) -> AttachRequest {
    AttachRequest {
        object_id: id.to_string(),
        link_name: "no_link".to_string(),
        touch_links: vec!["no_link".to_string()],
        operation: "detach".to_string(),
    }
}

fn names(names: &[&str]) -> Vec<String> {
    names.iter().map(|n| n.to_string()).collect()
}

fn empty_map() -> VoxelMap {
    map(VOXEL_M, 1.0)
}

/// The scene after `requests`, which apply.
fn scene_of(requests: &[ObjectRequest]) -> PlanningScene {
    let mut scene = PlanningScene::new(robot());
    scene
        .apply_collision_objects(requests, &mut empty_map())
        .expect("the requests apply");
    scene
}

fn pairs(scene: &PlanningScene) -> Vec<[String; 2]> {
    scene.snapshot(CLEARANCE_M, VOXEL_M).allowed_pairs
}

fn pair(a: &str, b: &str) -> [String; 2] {
    [a.to_string(), b.to_string()]
}

/// Whether the voxel holding `point` of `map` is occupied.
fn occupied(map: &VoxelMap, point: [f64; 3]) -> bool {
    map.is_occupied(map.key(point).expect("inside the map"))
}

/// A map whose voxels holding `points` are occupied, seen from above. Each point lies at
/// a voxel's center, where the scan's single-precision ray ends in the voxel `occupied`
/// reads.
fn map_with(points: &[[f64; 3]]) -> VoxelMap {
    let mut map = empty_map();
    occupied_at(&mut map, [0.0, 0.0, 0.9], points);
    map
}

// A batch whose third object fails applies none of its objects, leaves the map as it
// was, and names the object that failed; the batch without it applies whole.
#[test]
fn a_batch_applies_all_or_none() {
    let mut scene = PlanningScene::new(robot());
    let mut map = map_with(&[[0.51, 0.01, 0.31]]);
    let batch = [
        add("block", [0.51, 0.01, 0.31], vec![cube(0.06)]),
        add(
            "table",
            [0.5, 0.0, 0.2],
            vec![shape("box", &[0.6, 1.0, 0.05])],
        ),
        move_to("missing", [0.0; 3]),
    ];
    assert_eq!(
        scene.apply_collision_objects(&batch, &mut map),
        Err(SceneError::Batch {
            index: 2,
            id: "missing".to_string(),
            error: Box::new(SceneError::NoObject("missing".to_string())),
        })
    );
    assert_eq!(scene, PlanningScene::new(robot()));
    assert!(
        occupied(&map, [0.51, 0.01, 0.31]),
        "a refused batch cleared the map"
    );
    scene
        .apply_collision_objects(&batch[..2], &mut map)
        .expect("the batch applies");
    let ids: Vec<String> = scene
        .snapshot(CLEARANCE_M, VOXEL_M)
        .objects
        .into_iter()
        .map(|object| object.id)
        .collect();
    assert_eq!(ids, names(&["block", "table"]));
}

// An add of an id the scene holds replaces the object and keeps its pairs; a move reads
// only the id and the pose, and a removal only the id, and drops every pair that names
// the object.
#[test]
fn add_replaces_and_remove_drops_the_pairs() {
    let mut scene = scene_of(&[
        add("block", [0.5, 0.0, 0.3], vec![cube(0.06)]),
        add(
            "cup",
            [0.6, 0.0, 0.3],
            vec![shape("cylinder", &[0.1, 0.04])],
        ),
    ]);
    scene
        .set_allowed_collision("block", &names(&["cup", "torso", OCCUPANCY_MAP]), true)
        .expect("allowed");
    scene
        .set_allowed_collision("cup", &names(&["left_finger_a"]), true)
        .expect("allowed");
    let mut map = empty_map();
    let replacing = add("block", [0.5, 0.1, 0.3], vec![shape("sphere", &[0.03])]);
    scene
        .apply_collision_objects(
            &[replacing.clone(), move_to("cup", [0.7, 0.0, 0.3])],
            &mut map,
        )
        .expect("a replacing add and a move apply");
    let snapshot = scene.snapshot(CLEARANCE_M, VOXEL_M);
    assert_eq!(snapshot.objects[0], replacing.object);
    assert_eq!(snapshot.objects[1].position, [0.7, 0.0, 0.3]);
    assert_eq!(snapshot.objects[1].shapes[0].kind, "cylinder");
    assert_eq!(
        snapshot.allowed_pairs,
        [
            pair("block", "cup"),
            pair("block", OCCUPANCY_MAP),
            pair("block", "torso"),
            pair("cup", "left_finger_a"),
        ]
    );
    scene
        .apply_collision_objects(&[remove("block")], &mut map)
        .expect("a removal applies");
    assert_eq!(pairs(&scene), [pair("cup", "left_finger_a")]);
}

// An empty id, and an id that names a link or the voxel map, are refused, and so is an id
// the scene does not hold for a move or a removal: an empty one among them, which MoveIt
// takes as every object.
#[test]
fn an_id_must_name_an_object() {
    let mut scene = PlanningScene::new(robot());
    let refused = |scene: &mut PlanningScene, request: ObjectRequest| {
        let error = scene
            .apply_collision_objects(&[request], &mut empty_map())
            .expect_err("a refusal");
        let SceneError::Batch { error, .. } = error else {
            panic!("{error:?} names no object of the batch");
        };
        *error
    };
    assert_eq!(
        refused(&mut scene, add("", [0.0; 3], vec![cube(0.1)])),
        SceneError::EmptyId
    );
    assert_eq!(
        refused(&mut scene, add("torso", [0.0; 3], vec![cube(0.1)])),
        SceneError::LinkId("torso".to_string())
    );
    assert_eq!(
        refused(&mut scene, add(OCCUPANCY_MAP, [0.0; 3], vec![cube(0.1)])),
        SceneError::MapId
    );
    assert_eq!(
        refused(&mut scene, move_to("block", [0.0; 3])),
        SceneError::NoObject("block".to_string())
    );
    assert_eq!(
        refused(&mut scene, remove("block")),
        SceneError::NoObject("block".to_string())
    );
    for empty in [move_to("", [0.0; 3]), remove("")] {
        assert_eq!(refused(&mut scene, empty), SceneError::EmptyId);
    }
}

// A shape or an operation the contract does not describe is refused, and the message
// says what to send: a kind of no shape, a box of two dimensions, a radius that is no
// length, an open mesh, a position that is no number, an orientation that is no unit
// quaternion, an object of no shape, and an operation of no name. An orientation within
// 0.001 of unit length is normalized.
#[test]
fn what_the_contract_does_not_describe_is_refused() {
    let mut scene = PlanningScene::new(robot());
    let refusal = |request: ObjectRequest| {
        PlanningScene::new(robot())
            .apply_collision_objects(&[request], &mut empty_map())
            .expect_err("a refusal")
            .to_string()
    };
    let one = |shape: ShapeFields| add("block", [0.5, 0.0, 0.3], vec![shape]);
    assert!(
        refusal(one(shape("cone", &[0.1])))
            .contains("\"box\", \"sphere\", \"cylinder\" or \"mesh\"")
    );
    assert!(refusal(one(shape("box", &[0.1, 0.2]))).contains("a box has 3 dimensions"));
    assert!(refusal(one(shape("sphere", &[-0.1]))).contains("radius is -0.1"));
    let walls = ShapeFields {
        mesh_vertices: vec![
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
            1.0,
        ],
        mesh_triangles: vec![0, 1, 2, 0, 2, 3, 0, 4, 5, 0, 5, 3],
        ..shape("mesh", &[])
    };
    assert!(refusal(one(walls)).contains("the mesh is open"));
    assert!(
        refusal(add("block", [0.5, f64::NAN, 0.3], vec![cube(0.06)]))
            .contains("a position is finite")
    );
    let skewed = ObjectRequest {
        object: ObjectFields {
            orientation: [0.0, 0.0, 0.0, 1.1],
            ..object("block", [0.5, 0.0, 0.3], vec![cube(0.06)])
        },
        ..add("block", [0.5, 0.0, 0.3], vec![cube(0.06)])
    };
    assert!(refusal(skewed).contains("unit quaternion"));
    assert!(refusal(add("block", [0.5, 0.0, 0.3], Vec::new())).contains("at least one shape"));
    let rotate = ObjectRequest {
        operation: "rotate".to_string(),
        ..remove("block")
    };
    assert!(refusal(rotate).contains("\"add\", \"move\" or \"remove\""));
    let nearly = ObjectRequest {
        object: ObjectFields {
            orientation: [0.0, 0.0, 0.0, 1.0005],
            ..object("block", [0.5, 0.0, 0.3], vec![cube(0.06)])
        },
        ..add("block", [0.5, 0.0, 0.3], vec![cube(0.06)])
    };
    scene
        .apply_collision_objects(&[nearly], &mut empty_map())
        .expect("a nearly unit orientation applies");
    assert_eq!(
        scene.snapshot(CLEARANCE_M, VOXEL_M).objects[0].orientation,
        IDENTITY
    );
}

// Only an arm's grasp link carries an object, with touch links of the robot; attaching
// an attached object and detaching one that is not attached are refused, and every
// operation of a batch on an attached object too. A detach reads only the object's id.
#[test]
fn only_a_grasp_link_carries_an_object() {
    let mut scene = scene_of(&[add("block", [0.45, 0.2, 0.3], vec![cube(0.04)])]);
    let mut map = empty_map();
    let mut apply = |scene: &mut PlanningScene, request: AttachRequest| {
        scene.apply_attached_collision_object(&request, hands, &mut map)
    };
    assert_eq!(
        apply(&mut scene, attach("block", "torso", &[])),
        Err(SceneError::NotGraspLink {
            link: "torso".to_string(),
            grasp_links: names(&["left_hand", "right_hand"]),
        })
    );
    assert_eq!(
        apply(&mut scene, attach("block", "left_hand", &["left_thumb"])),
        Err(SceneError::NoLink("left_thumb".to_string()))
    );
    assert_eq!(
        apply(&mut scene, attach("cup", "left_hand", &[])),
        Err(SceneError::NoObject("cup".to_string()))
    );
    assert_eq!(
        apply(&mut scene, detach("block")),
        Err(SceneError::NotAttached("block".to_string()))
    );
    let fingers = ["left_finger_a", "left_finger_b"];
    apply(&mut scene, attach("block", "left_hand", &fingers)).expect("attached");
    let attached = Err(SceneError::Attached {
        id: "block".to_string(),
        link: "left_hand".to_string(),
    });
    assert_eq!(
        apply(&mut scene, attach("block", "right_hand", &[])),
        attached
    );
    for request in [
        add("block", [0.0; 3], vec![cube(0.1)]),
        move_to("block", [0.0; 3]),
        remove("block"),
    ] {
        let error = scene
            .apply_collision_objects(&[request], &mut empty_map())
            .expect_err("a refusal");
        assert!(
            matches!(error, SceneError::Batch { error, .. } if Err(*error.clone()) == attached)
        );
    }
    apply(&mut scene, detach("block")).expect("detached");
    assert_eq!(
        apply(&mut scene, detach("block")),
        Err(SceneError::NotAttached("block".to_string()))
    );
    assert!(matches!(
        apply(
            &mut scene,
            AttachRequest {
                operation: "hold".to_string(),
                ..attach("block", "left_hand", &[])
            }
        ),
        Err(SceneError::Operation { .. })
    ));
}

// An attach keeps the object where it stands, in the frame of its link, under the
// attached objects alone; a detach leaves it where the link carried it. Its pairs stay
// through both.
#[test]
fn an_attached_object_keeps_where_it_stands_and_its_pairs() {
    let at = [0.45, 0.2, 0.3];
    let mut scene = scene_of(&[add("block", at, vec![cube(0.04)])]);
    scene
        .set_allowed_collision("block", &names(&["torso"]), true)
        .expect("allowed");
    let mut map = empty_map();
    scene
        .apply_attached_collision_object(
            &attach("block", "left_hand", &["left_finger_b", "left_finger_a"]),
            hands,
            &mut map,
        )
        .expect("attached");
    let snapshot = scene.snapshot(CLEARANCE_M, VOXEL_M);
    assert!(
        snapshot.objects.is_empty(),
        "an attached object is an object"
    );
    let [attached] = snapshot.attached_objects.as_slice() else {
        panic!("one attached object: {:?}", snapshot.attached_objects);
    };
    assert_eq!(attached.link_name, "left_hand");
    assert_eq!(
        attached.touch_links,
        names(&["left_finger_a", "left_finger_b"])
    );
    let hand = hands("left_hand").expect("a hand");
    let carried =
        pose_from_parts(attached.object.position, attached.object.orientation).expect("a pose");
    let off = ((hand * carried).translation.vector - Vector3::from(at)).norm();
    assert!(off < POSE_TOLERANCE, "the attached block moved {off} m");
    assert_eq!(snapshot.allowed_pairs, [pair("block", "torso")]);
    let moved = Isometry3::translation(0.0, 0.3, 0.1) * hand;
    scene
        .apply_attached_collision_object(&detach("block"), |_| Some(moved), &mut map)
        .expect("detached");
    let snapshot = scene.snapshot(CLEARANCE_M, VOXEL_M);
    assert!(snapshot.attached_objects.is_empty());
    let off = (Vector3::from(snapshot.objects[0].position) - Vector3::new(0.45, 0.5, 0.4)).norm();
    assert!(off < POSE_TOLERANCE, "the detached block lies {off} m off");
    assert_eq!(snapshot.allowed_pairs, [pair("block", "torso")]);
}

// A link whose pose is not known yet refuses an attach and a detach, and the scene
// stays as it was.
#[test]
fn an_unplaced_link_refuses_an_attach_and_a_detach() {
    let mut scene = scene_of(&[add("block", [0.45, 0.2, 0.3], vec![cube(0.04)])]);
    let before = scene.clone();
    assert_eq!(
        scene.apply_attached_collision_object(
            &attach("block", "left_hand", &[]),
            |_| None,
            &mut empty_map()
        ),
        Err(SceneError::Unplaced("left_hand".to_string()))
    );
    assert_eq!(scene, before);
    scene
        .apply_attached_collision_object(
            &attach("block", "left_hand", &[]),
            hands,
            &mut empty_map(),
        )
        .expect("attached");
    let attached = scene.clone();
    assert_eq!(
        scene.apply_attached_collision_object(&detach("block"), |_| None, &mut empty_map()),
        Err(SceneError::Unplaced("left_hand".to_string()))
    );
    assert_eq!(scene, attached);
}

// A pair names an object of the scene, attached or not, and links, objects or the voxel
// map, all or none; a name the robot and the scene do not have is refused, and so is an
// object paired with itself. A pair is the same pair either way round, reported once.
#[test]
fn a_pair_names_an_object() {
    let mut scene = scene_of(&[
        add("block", [0.45, 0.2, 0.3], vec![cube(0.04)]),
        add(
            "table",
            [0.5, 0.0, 0.2],
            vec![shape("box", &[0.6, 1.0, 0.05])],
        ),
    ]);
    assert_eq!(
        scene.set_allowed_collision("torso", &names(&["block"]), true),
        Err(SceneError::NoObject("torso".to_string()))
    );
    assert_eq!(
        scene.set_allowed_collision("block", &names(&["torso", "chair"]), true),
        Err(SceneError::NoName("chair".to_string()))
    );
    assert_eq!(
        pairs(&scene),
        Vec::<[String; 2]>::new(),
        "a refused call set a pair"
    );
    assert_eq!(
        scene.set_allowed_collision("block", &names(&["block"]), true),
        Err(SceneError::SelfPair("block".to_string()))
    );
    scene
        .set_allowed_collision("table", &names(&["block", OCCUPANCY_MAP]), true)
        .expect("allowed");
    scene
        .set_allowed_collision("block", &names(&["table"]), true)
        .expect("allowed again");
    assert_eq!(
        pairs(&scene),
        [pair("block", "table"), pair(OCCUPANCY_MAP, "table")]
    );
    scene
        .apply_attached_collision_object(
            &attach("block", "left_hand", &[]),
            hands,
            &mut empty_map(),
        )
        .expect("attached");
    scene
        .set_allowed_collision("block", &names(&["table"]), false)
        .expect("stopped for an attached object");
    assert_eq!(pairs(&scene), [pair(OCCUPANCY_MAP, "table")]);
}

// The snapshot holds exactly the objects that were added, each pose and shape as it was
// given, in the order of the ids, nothing attached, and the clearance and the voxel size
// it is given.
#[test]
fn the_snapshot_holds_what_was_set() {
    let tetrahedron = ShapeFields {
        mesh_vertices: vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        mesh_triangles: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
        position: [0.01, 0.02, 0.03],
        orientation: [1.0, 0.0, 0.0, 0.0],
        ..shape("mesh", &[])
    };
    let given = [
        object(
            "a_box",
            [0.5, -0.1, 0.3],
            vec![shape("box", &[0.1, 0.2, 0.3])],
        ),
        object(
            "b_can",
            [0.4, 0.1, 0.25],
            vec![
                shape("cylinder", &[0.12, 0.03]),
                ShapeFields {
                    position: [0.0, 0.0, 0.07],
                    orientation: [0.0, 1.0, 0.0, 0.0],
                    ..shape("sphere", &[0.03])
                },
            ],
        ),
        object("c_tetrahedron", [0.6, 0.0, 0.1], vec![tetrahedron]),
    ];
    let scene = scene_of(
        &given
            .iter()
            .rev()
            .map(|object| ObjectRequest {
                operation: "add".to_string(),
                object: object.clone(),
            })
            .collect::<Vec<_>>(),
    );
    let snapshot = scene.snapshot(CLEARANCE_M, VOXEL_M);
    assert_eq!(snapshot.objects, given);
    assert!(snapshot.attached_objects.is_empty());
    assert!(snapshot.allowed_pairs.is_empty());
    assert_eq!(
        (snapshot.clearance_m, snapshot.voxel_size_m),
        (CLEARANCE_M, VOXEL_M)
    );
}

// An empty batch of apply_collision_objects, and set_allowed_collision of an object with no
// other name, allowed or stopped, leave the snapshot and the map as they were: the object
// stands where it stood, with its pair, beside an occupied voxel.
#[test]
fn an_empty_call_changes_nothing() {
    let (here, beside) = ([0.51, 0.01, 0.31], [0.51, 0.31, 0.31]);
    let mut scene = PlanningScene::new(robot());
    let mut map = map_with(&[here, beside]);
    scene
        .apply_collision_objects(&[add("block", here, vec![cube(0.06)])], &mut map)
        .expect("added");
    scene
        .set_allowed_collision("block", &names(&["left_hand"]), true)
        .expect("allowed");
    let voxels =
        |map: &VoxelMap| -> Vec<_> { map.known().map(|key| (key, map.voxel(key))).collect() };
    let (snapshot, before) = (scene.snapshot(CLEARANCE_M, VOXEL_M), voxels(&map));
    scene
        .apply_collision_objects(&[], &mut map)
        .expect("an empty batch applies");
    for allowed in [true, false] {
        scene
            .set_allowed_collision("block", &[], allowed)
            .expect("no other name applies");
    }
    assert_eq!(scene.snapshot(CLEARANCE_M, VOXEL_M), snapshot);
    assert_eq!(voxels(&map), before);
    assert!(occupied(&map, beside));
}

// Adding, moving or attaching an object clears every voxel it intersects where it then
// stands; removing and detaching it clear nothing.
#[test]
fn an_object_clears_the_map_where_it_is_placed() {
    let (here, there) = ([0.51, 0.01, 0.31], [0.51, 0.31, 0.31]);
    let mut scene = PlanningScene::new(robot());
    let mut map = map_with(&[here, there]);
    scene
        .apply_collision_objects(&[add("block", here, vec![cube(0.06)])], &mut map)
        .expect("added");
    assert!(!occupied(&map, here) && occupied(&map, there));
    scene
        .apply_collision_objects(&[move_to("block", there)], &mut map)
        .expect("moved");
    assert!(!occupied(&map, there));
    occupied_at(&mut map, [0.0, 0.0, 0.9], &[there]);
    occupied_at(&mut map, [0.0, 0.0, 0.9], &[there]);
    occupied_at(&mut map, [0.0, 0.0, 0.9], &[there]);
    assert!(
        occupied(&map, there),
        "three hits mark a cleared voxel again"
    );
    let hand_at_block = |_: &str| Some(Isometry3::translation(there[0], there[1] - 0.05, there[2]));
    scene
        .apply_attached_collision_object(
            &attach("block", "left_hand", &[]),
            hand_at_block,
            &mut map,
        )
        .expect("attached");
    assert!(
        !occupied(&map, there),
        "an attach clears where the object stands"
    );
    let mut marked = map_with(&[there]);
    scene
        .apply_attached_collision_object(&detach("block"), hand_at_block, &mut marked)
        .expect("detached");
    scene
        .apply_collision_objects(&[remove("block")], &mut marked)
        .expect("removed");
    assert!(
        occupied(&marked, there),
        "a detach or a removal cleared the map"
    );
}

// A batch clears the map from the scene it leaves: an object a later operation of the
// batch removes clears nothing, and one a later operation moves clears where it ends
// alone.
#[test]
fn a_batch_clears_the_map_where_it_leaves_its_objects() {
    let (here, there) = ([0.51, 0.01, 0.31], [0.51, 0.31, 0.31]);
    let mut scene = PlanningScene::new(robot());
    let mut map = map_with(&[here, there]);
    scene
        .apply_collision_objects(
            &[add("passing", here, vec![cube(0.06)]), remove("passing")],
            &mut map,
        )
        .expect("added and removed");
    assert!(
        occupied(&map, here),
        "an object the batch removed cleared the map"
    );
    scene
        .apply_collision_objects(
            &[
                add("block", here, vec![cube(0.06)]),
                move_to("block", there),
            ],
            &mut map,
        )
        .expect("added and moved");
    assert!(
        occupied(&map, here),
        "an object cleared where the batch moved it from"
    );
    assert!(
        !occupied(&map, there),
        "an object left where the batch moved it to marked"
    );
}
