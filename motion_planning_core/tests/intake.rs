//! The map's intake replays a recorded stream of OpenArm v2: the joint positions measured
//! every 100 ms while the left elbow bends, and the frames a depth camera facing the robot
//! took between them, each arriving after the next measurement. Each frame, cut out where
//! the history puts the robot at its stamp, marks nothing; cut out where the robot stood at
//! the newest measurement, it marks the arm. A depth point inside an object of the planning
//! scene, or inside an object the left hand holds where the hand stood at the frame's
//! stamp, marks nothing.

mod openarm_v2;

use std::path::{Path, PathBuf};
use std::sync::LazyLock;
use std::time::{Duration, SystemTime};

use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use collision_geometry::chain::{BodyMask, Padding, PlacedMask, Rider, RiderBody};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    AttachRequest, DepthCamera, JointHistory, ObjectFields, ObjectRequest, PlanningScene,
    RobotLinks, RobotMask, ShapeFields, insert_cut_out,
};
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{Bounds, Frame, Scan, VoxelMap};
use openarm_description::{ARM_DOF, ArmPair, HardwareVersion, Side, ready};

use openarm_v2::Robot;

/// Both arms' joint positions, the left arm's first.
type Joints = [f64; 2 * ARM_DOF];

/// The rendered image's pixels across and down, and its focal length in pixels.
const WIDTH: u32 = 48;
const HEIGHT: u32 = 36;
const FOCAL_PX: f64 = 30.0;
/// Meters per count of a depth frame.
const DEPTH_UNIT: f32 = 0.001;
/// The depths a ray is sampled over, and the step between samples (m).
const NEAREST_M: f64 = 0.2;
const FARTHEST_M: f64 = 1.8;
const STEP_M: f64 = 0.004;

/// How far the left elbow bends between two measurements (rad): the hand moves some
/// 3 cm in half the interval.
const ELBOW_STEP_RAD: f64 = 0.2;

/// The padding the replay cuts the robot out with: under how far the hand moves in half
/// an interval.
const PADDING_OFFSET_M: f64 = 0.01;

/// OpenArm v2 as the map cuts it out, its meshes written once per test binary.
static OPENARM_V2: LazyLock<Robot> = LazyLock::new(|| {
    let meshes: PathBuf = Path::new(env!("CARGO_TARGET_TMPDIR")).join("intake_openarm_v2");
    HardwareVersion::V2
        .write_meshes_to(&meshes)
        .expect("write the meshes");
    Robot::new(meshes.to_str().expect("the mesh directory is UTF-8"))
});

/// The fingers of both arms where the self model hangs them.
fn fingers() -> ArmPair<Vec<Rider<'static>>> {
    OPENARM_V2.fingers(&OPENARM_V2.self_model)
}

/// The robot at `joints`, grown by `padding`.
fn robot_at<'a>(
    joints: &Joints,
    riders: &'a ArmPair<Vec<Rider<'a>>>,
    padding: Padding,
) -> RobotMask<'a, ARM_DOF> {
    OPENARM_V2.mask(&ArmPair::from_fn(|side| arm(joints, side)), riders, padding)
}

/// The arm on `side` of `joints`.
fn arm(joints: &Joints, side: Side) -> [f64; ARM_DOF] {
    let offset = match side {
        Side::Left => 0,
        Side::Right => ARM_DOF,
    };
    std::array::from_fn(|i| joints[offset + i])
}

/// Both arms at Ready, the left elbow bent by `bend` from it.
fn bent(bend: f64) -> Joints {
    let (mut left, right) = (ready(Side::Left), ready(Side::Right));
    left[3] += bend;
    std::array::from_fn(|i| {
        if i < ARM_DOF {
            left[i]
        } else {
            right[i - ARM_DOF]
        }
    })
}

/// The instant `ms` milliseconds into the recording.
fn at_ms(ms: u64) -> SystemTime {
    SystemTime::UNIX_EPOCH + Duration::from_secs(1_000) + Duration::from_millis(ms)
}

/// A camera 0.8 m in front of the robot at shoulder height facing it, image right
/// across its optical axis and the robot's +z.
fn camera_pose() -> Isometry3<f64> {
    let eye = Vector3::new(0.8, 0.0, 0.6);
    let forward = (Vector3::new(0.0, 0.0, 0.6) - eye).normalize();
    let across = forward.cross(&Vector3::z()).normalize();
    let axes = Matrix3::from_columns(&[across, forward.cross(&across), forward]);
    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(axes));
    Isometry3::from_parts(Translation3::from(eye), rotation)
}

fn pose_of(isometry: &Isometry3<f64>) -> Pose {
    let q = isometry.rotation.quaternion();
    Pose::new(isometry.translation.vector.into(), [q.i, q.j, q.k, q.w]).expect("a unit quaternion")
}

/// The camera: its color optical frame at `camera_pose`, its depth frame there too.
fn depth_camera() -> DepthCamera {
    let lens = Pinhole::parse(&Intrinsics {
        width: WIDTH,
        height: HEIGHT,
        fx: FOCAL_PX,
        fy: FOCAL_PX,
        cx: f64::from(WIDTH) / 2.0,
        cy: f64::from(HEIGHT) / 2.0,
        distortion_model: "none",
        distortion: &[],
    })
    .expect("a pinhole model");
    let identity = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("the identity");
    DepthCamera::new(
        &pose_of(&camera_pose()),
        &identity,
        lens,
        DepthModel::Z,
        DEPTH_UNIT,
    )
}

/// The z16 frame the camera takes of the robot at `joints`: each pixel's depth along its
/// ray to the first sample inside the robot's pieces, 0 where it sees nothing. The
/// renderer places the pieces itself.
fn frame_of(joints: &Joints) -> Vec<u8> {
    let none = Padding::new(1.0, 0.0).expect("the padding that grows nothing");
    let chains = Side::BOTH.map(|side| (side, OPENARM_V2.arms[side].mask(none)));
    let riders = fingers();
    let carried = BodyMask::of(
        Side::BOTH.into_iter().flat_map(|side| {
            let posed = OPENARM_V2.arms[side].chain().at(&arm(joints, side));
            riders[side].iter().map(move |finger| {
                let hand = posed.link_pose_world(finger.segment);
                let RiderBody::Part(part) = finger.body else {
                    panic!("a finger is a part of the robot");
                };
                (finger.name, part.pieces(), hand * finger.offset)
            })
        }),
        none,
    );
    let body = OPENARM_V2.body.mask(none);
    let parts: Vec<PlacedMask> = chains
        .iter()
        .map(|(side, mask)| mask.at(&arm(joints, *side)))
        .chain([carried.placed(), body.placed()])
        .collect();
    let camera = camera_pose();
    let samples = ((FARTHEST_M - NEAREST_M) / STEP_M) as usize;
    (0..HEIGHT)
        .flat_map(|v| (0..WIDTH).map(move |u| (u, v)))
        .map(|(u, v)| {
            let x = (f64::from(u) - f64::from(WIDTH) / 2.0) / FOCAL_PX;
            let y = (f64::from(v) - f64::from(HEIGHT) / 2.0) / FOCAL_PX;
            let ray = Vector3::new(x, y, 1.0).normalize();
            (0..=samples)
                .map(|k| ray * (NEAREST_M + STEP_M * k as f64))
                .find(|seen| {
                    let point = camera * Point3::from(*seen);
                    parts.iter().any(|part| part.contains(&point))
                })
                .map_or(0, |seen| (seen.z / f64::from(DEPTH_UNIT)).round() as u16)
        })
        .flat_map(u16::to_le_bytes)
        .collect()
}

/// The recording: a measurement every 100 ms from 0 to 600 ms, the left elbow bending by
/// `ELBOW_STEP_RAD` between two, and a frame at the middle of each interval, of the robot
/// where it stood then. Each event is in the order it arrived: a frame 70 ms after its
/// stamp, after the next measurement.
enum Event {
    Measured(SystemTime, Joints),
    Frame(SystemTime, Vec<u8>),
}

fn recording() -> Vec<Event> {
    (0..=6_u64)
        .flat_map(|k| {
            let measured = Event::Measured(at_ms(100 * k), bent(ELBOW_STEP_RAD * k as f64));
            let previous = (k > 0).then(|| {
                let between = bent(ELBOW_STEP_RAD * (k as f64 - 0.5));
                Event::Frame(at_ms(100 * k - 50), frame_of(&between))
            });
            std::iter::once(measured).chain(previous)
        })
        .collect()
}

/// Where the replay cuts the robot out of a frame: at the frame's stamp, or where the
/// newest measurement puts it.
#[derive(Clone, Copy)]
enum CutAt {
    Stamp,
    Newest,
}

/// The frame of OpenArm's URDF root, which the maps stand in.
fn world() -> Frame {
    Frame::parse("world").expect("a frame name")
}

/// An empty map of 2.5 cm voxels about the robot and the camera.
fn empty_map() -> VoxelMap {
    VoxelMap::new(
        world(),
        PositiveFinite::parse(0.025).expect("a positive voxel size"),
        Bounds {
            min: [-0.4, -1.0, 0.0],
            max: [1.2, 1.0, 1.6],
        },
    )
    .expect("a map")
}

/// The map the recording builds, each frame inserted with the robot cut out as `cut_at`
/// says.
fn replay(recording: &[Event], cut_at: CutAt) -> VoxelMap {
    let world = world();
    let mut map = empty_map();
    let camera = depth_camera();
    let riders = fingers();
    let padding = Padding::new(1.0, PADDING_OFFSET_M).expect("a padding");
    let mut history = JointHistory::new(Duration::from_secs(2));
    for event in recording {
        match event {
            Event::Measured(at, joints) => history.push(*at, *joints),
            Event::Frame(stamp, frame) => {
                let joints = match cut_at {
                    CutAt::Stamp => history.at(*stamp).expect("the history spans the stamp"),
                    CutAt::Newest => *history.latest().expect("a measurement"),
                };
                let points = camera.points(frame).expect("a frame of the lens's size");
                let scan = Scan {
                    frame: &world,
                    sensor: camera.sensor,
                    points: &points,
                    max_range_m: PositiveFinite::parse(2.0).expect("a positive range"),
                };
                insert_cut_out(&mut map, &scan, &robot_at(&joints, &riders, padding))
                    .expect("the scan stands in the map's frame");
            }
        }
    }
    map
}

#[test]
fn a_replayed_frame_cut_out_at_its_stamp_marks_nothing() {
    let recording = recording();
    let seen = recording
        .iter()
        .filter_map(|event| match event {
            Event::Frame(_, frame) => Some(
                depth_camera()
                    .points(frame)
                    .expect("a frame of the lens's size")
                    .len(),
            ),
            Event::Measured(..) => None,
        })
        .min()
        .expect("frames");
    assert!(
        seen > 100,
        "each frame sees the robot: {seen} points at least"
    );

    let at_stamps = replay(&recording, CutAt::Stamp);
    assert!(
        at_stamps.known().count() > 0,
        "the frames' rays reach the map"
    );
    assert_eq!(
        at_stamps.occupied().count(),
        0,
        "the robot's own points marked the map"
    );

    let at_newest = replay(&recording, CutAt::Newest);
    assert!(
        at_newest.occupied().count() > 10,
        "the arm marks the map where the newest measurement put it: {} voxels",
        at_newest.occupied().count()
    );
}

/// A box of `size` at `position`, as apply_collision_objects adds it.
fn add_box(id: &str, position: [f64; 3], size: [f64; 3]) -> ObjectRequest {
    ObjectRequest {
        operation: "add".to_string(),
        object: ObjectFields {
            id: id.to_string(),
            position,
            orientation: [0.0, 0.0, 0.0, 1.0],
            shapes: vec![ShapeFields {
                kind: "box".to_string(),
                dimensions: size.to_vec(),
                mesh_vertices: Vec::new(),
                mesh_triangles: Vec::new(),
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
            }],
        },
    }
}

/// Whether the voxel of `map` that holds `point` is occupied.
fn occupied(map: &VoxelMap, point: [f64; 3]) -> bool {
    map.is_occupied(map.key(point).expect("a point inside the map"))
}

// The left hand holds a block 8 cm past its grasp point, attached where the arm stands
// now; a table stands apart from the robot. A frame taken with the left elbow bent half a
// radian from now sees a point on the table, a point at the block's center where the hand
// then held it, and one at its center where the hand holds it now. Cut out with the
// scene, the first two mark nothing and the third is a hit; cut out with the robot alone,
// the first two are hits.
#[test]
fn a_depth_point_inside_an_object_or_a_held_one_marks_nothing() {
    let version = HardwareVersion::V2;
    let grasp_links = Side::BOTH.map(|side| version.tcp_link(side).to_string());
    let left_grasp = grasp_links[0].as_str();
    let mut scene = PlanningScene::new(RobotLinks::new(grasp_links.clone(), grasp_links.clone()));
    let mut map = empty_map();
    let (now, then) = (bent(0.0), bent(0.5));
    let left = &OPENARM_V2.arms.left;
    let hand_at = |joints: &Joints| left.chain().world_ee_pose(&arm(joints, Side::Left));
    let held_at = |joints: &Joints| -> [f64; 3] {
        (hand_at(joints) * Point3::new(0.0, 0.0, 0.08))
            .coords
            .into()
    };
    let table = [1.0, 0.0, 0.1];
    scene
        .apply_collision_objects(
            &[
                add_box("table", table, [0.3, 0.3, 0.04]),
                add_box("block", held_at(&now), [0.04; 3]),
            ],
            &mut map,
        )
        .expect("the objects apply");
    scene
        .apply_attached_collision_object(
            &AttachRequest {
                object_id: "block".to_string(),
                link_name: left_grasp.to_string(),
                touch_links: Vec::new(),
                operation: "attach".to_string(),
            },
            |link| (link == left_grasp).then(|| hand_at(&now)),
            &mut map,
        )
        .expect("the block attaches to the left hand");
    let points = [table, held_at(&then), held_at(&now)];
    let fingers = fingers();
    let riders = ArmPair::from_fn(|side| {
        let held = match side {
            Side::Left => scene.riders(left_grasp, left),
            Side::Right => Vec::new(),
        };
        fingers[side]
            .iter()
            .copied()
            .chain(held)
            .collect::<Vec<_>>()
    });
    let padding = Padding::new(1.0, 0.0).expect("the padding that grows nothing");
    let joints = ArmPair::from_fn(|side| arm(&then, side));
    let with_scene = OPENARM_V2
        .mask(&joints, &riders, padding)
        .with_objects(scene.standing_shapes());
    let robot_alone = OPENARM_V2.mask(&joints, &fingers, padding);
    let sensor = Pose::new([0.4, 0.0, 1.5], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let relative: Vec<[f32; 3]> = points
        .iter()
        .map(|p| std::array::from_fn(|i| (p[i] - sensor.origin()[i]) as f32))
        .collect();
    let world = world();
    let scan = Scan {
        frame: &world,
        sensor,
        points: &relative,
        max_range_m: PositiveFinite::parse(3.0).expect("a positive range"),
    };
    let marked = |robot: &RobotMask<ARM_DOF>| {
        let mut fresh = empty_map();
        insert_cut_out(&mut fresh, &scan, robot).expect("the scan stands in the map's frame");
        points.map(|point| occupied(&fresh, point))
    };
    assert_eq!(
        marked(&with_scene),
        [false, false, true],
        "cut out with the scene"
    );
    assert_eq!(
        marked(&robot_alone),
        [true, true, true],
        "cut out with the robot alone"
    );
}
