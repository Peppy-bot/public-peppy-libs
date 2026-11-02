//! The map's intake replays a recorded stream of OpenArm v2: the joint positions measured
//! every 100 ms while the left elbow bends, and the frames a depth camera facing the robot
//! took between them, each arriving after the next measurement. Each frame, cut out where
//! the history puts the robot at its stamp, marks nothing; cut out where the robot stood at
//! the newest measurement, it marks the arm.

use std::path::{Path, PathBuf};
use std::sync::LazyLock;
use std::time::{Duration, SystemTime};

use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{
    BodyGeometry, BodyMask, ChainGeometry, Padding, PlacedMask, Rider,
};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{DepthCamera, JointHistory, PosedChain, RobotMask, insert_cut_out};
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{Bounds, Frame, Scan, VoxelMap};
use openarm_description::{ARM_DOF, ArmPair, HardwareVersion, Side, ready};
use self_collision_model::SelfCollisionModel;

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

/// OpenArm v2 as the map cuts it out: both arms from their base links to their grasp
/// points, the self model's fingers at the grippers' openings, and the body.
struct Robot {
    arms: ArmPair<ChainGeometry<ARM_DOF>>,
    body: BodyGeometry,
    self_model: SelfCollisionModel,
}

static OPENARM_V2: LazyLock<Robot> = LazyLock::new(|| {
    let version = HardwareVersion::V2;
    let meshes: PathBuf = Path::new(env!("CARGO_TARGET_TMPDIR")).join("intake_openarm_v2");
    version.write_meshes_to(&meshes).expect("write the meshes");
    let meshes = meshes.to_str().expect("the mesh directory is UTF-8");
    let arm = |side: Side| {
        ChainGeometry::from_urdf(
            version.urdf(),
            meshes,
            &ChainSpec {
                base_link: Some(version.base_link(side)),
                tip_link: version.tcp_link(side),
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the arm fits")
    };
    Robot {
        arms: ArmPair::new(arm(Side::Left), arm(Side::Right)),
        body: BodyGeometry::from_urdf(
            version.urdf(),
            meshes,
            &[(version.torso_body(), version.torso_regions())],
        )
        .expect("the body fits"),
        self_model: self_collision_model::openarm(version, meshes).expect("the self model"),
    }
});

/// The fingers of the arm on `side`, as riders of its links.
fn fingers(side: Side) -> Vec<Rider<'static>> {
    OPENARM_V2
        .self_model
        .fingers(side, OPENARM_V2.arms[side].carried())
        .expect("each arm's chain carries its own fingers")
}

/// The robot at `joints`, grown by `padding`.
fn robot_at<'a>(
    joints: &Joints,
    riders: &'a ArmPair<Vec<Rider<'a>>>,
    padding: Padding,
) -> RobotMask<'a, ARM_DOF> {
    let chains: Vec<PosedChain<'a, ARM_DOF>> = Side::BOTH
        .into_iter()
        .map(|side| PosedChain {
            geometry: &OPENARM_V2.arms[side],
            joints: arm(joints, side),
            riders: &riders[side],
        })
        .collect();
    RobotMask::new(&chains, &OPENARM_V2.body, padding)
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
    let carried = BodyMask::of(
        Side::BOTH.into_iter().flat_map(|side| {
            let posed = OPENARM_V2.arms[side].chain().at(&arm(joints, side));
            fingers(side).into_iter().map(move |finger| {
                let hand = posed.link_pose_world(finger.segment);
                (finger.link, finger.pieces, hand * finger.offset)
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

/// The map the recording builds, each frame inserted with the robot cut out as `cut_at`
/// says.
fn replay(recording: &[Event], cut_at: CutAt) -> VoxelMap {
    let world = Frame::parse("world").expect("a frame name");
    let mut map = VoxelMap::new(
        world.clone(),
        PositiveFinite::parse(0.025).expect("a positive voxel size"),
        Bounds {
            min: [-0.4, -1.0, 0.0],
            max: [1.2, 1.0, 1.6],
        },
    )
    .expect("a map");
    let camera = depth_camera();
    let riders = ArmPair::from_fn(fingers);
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
