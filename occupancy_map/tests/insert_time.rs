//! The time one 640x360 depth frame of a table scene in front of OpenArm v2 takes, on
//! the machine that runs it, to deproject, to insert with both arms cut out, grown by a
//! padding, and with nothing cut out, and the time the map's distance field takes after
//! it. The chest camera sees the boxes on the table and both forearms, rendered from
//! the arms' placed pieces, and the test prints how many of the frame's points the arms'
//! mask holds: `cargo test --release --test insert_time -- --ignored --nocapture`
//! prints them.

use std::time::{Duration, Instant};

use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use chain_kinematics::Tree;
use collision_geometry::chain::Padding;
use collision_geometry::gjk::Placed;
use control_core::positive_finite::PositiveFinite;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{
    Bounds, Cutout, DistanceField, Frame, RobotCutout, Scan, VoxelMap, depth_points,
};
use robot_description::states;

#[path = "common/arms.rs"]
mod arms;
#[path = "common/pose.rs"]
mod pose;
#[path = "common/slab.rs"]
mod slab;
use arms::{OPENARM_V2, OPENARM_V2_ARMS, posture};
use pose::pose_of;
use slab::box_entry;

/// The frames the test times, each into a map of its own.
const RUNS: usize = 5;

/// The chest camera's depth stream at 640x360, an ideal pinhole.
const CAMERA: Intrinsics<'static> = Intrinsics {
    width: 640,
    height: 360,
    fx: 369.0547,
    fy: 369.0547,
    cx: 319.5,
    cy: 179.5,
    distortion_model: "none",
    distortion: &[],
};

/// Meters per count of the depth frame.
const DEPTH_UNIT: f32 = 0.001;

/// The camera the test renders the frame of: OpenArm v2's chest camera.
const CAMERA_NAME: &str = "chest";

/// The fewest of the frame's points the arms' mask holds: the camera sees both
/// forearms.
const MIN_MASKED: usize = 1000;

/// The shapes of the scene in the URDF root's frame: the floor, boxes on it, a ball at
/// each arm's grasp point, and the arms' pieces where they stand.
struct Scene<'a> {
    boxes: Vec<([f64; 3], [f64; 3])>,
    balls: Vec<([f64; 3], f64)>,
    pieces: Vec<Placed<'a>>,
}

impl Scene<'_> {
    /// How far along `direction` from `origin` the ray first meets the scene.
    fn first_hit(&self, origin: Point3<f64>, direction: Vector3<f64>) -> Option<f64> {
        let floor = (direction.z < 0.0).then(|| -origin.z / direction.z);
        let boxes = self
            .boxes
            .iter()
            .filter_map(|(min, max)| box_entry(origin.into(), direction.into(), *min, *max));
        let balls = self.balls.iter().filter_map(|(center, radius)| {
            let to = origin - Point3::from(*center);
            let (a, b) = (direction.dot(&direction), direction.dot(&to));
            let discriminant = b * b - a * (to.dot(&to) - radius * radius);
            (discriminant >= 0.0)
                .then(|| (-b - discriminant.sqrt()) / a)
                .filter(|t| *t > 0.0)
        });
        let pieces = self
            .pieces
            .iter()
            .filter_map(|piece| piece_entry(piece, origin, direction));
        floor
            .into_iter()
            .chain(boxes)
            .chain(balls)
            .chain(pieces)
            .min_by(f64::total_cmp)
    }
}

/// How far along `direction` from `origin` the ray first meets the convex core of
/// `piece`: the latest it enters the side of a face plane the core lies on, before the
/// earliest it leaves one.
fn piece_entry(piece: &Placed, origin: Point3<f64>, direction: Vector3<f64>) -> Option<f64> {
    let (center, radius) = piece.bounding_sphere();
    let to = origin - center;
    let along = direction.dot(&to);
    if along * along - direction.dot(&direction) * (to.dot(&to) - radius * radius) < 0.0 {
        return None;
    }
    let to_piece = piece.pose().inverse();
    let (origin, direction) = (to_piece * origin, to_piece * direction);
    let (mut near, mut far) = (0.0_f64, f64::INFINITY);
    for (normal, offset) in piece.hull().planes() {
        let (toward, room) = (normal.dot(&direction), offset - normal.dot(&origin.coords));
        if toward == 0.0 {
            if room < 0.0 {
                return None;
            }
        } else if toward < 0.0 {
            near = near.max(room / toward);
        } else {
            far = far.min(room / toward);
        }
    }
    (near <= far && near > 0.0).then_some(near)
}

/// The z16 frame a camera of model `camera` at `pose` reads of `scene`.
fn render(camera: &Pinhole, pose: &Isometry3<f64>, scene: &Scene) -> Vec<u8> {
    let origin = Point3::from(pose.translation.vector);
    (0..CAMERA.height)
        .flat_map(|v| (0..CAMERA.width).map(move |u| (u, v)))
        .flat_map(|(u, v)| {
            // The pixel's ray at unit depth: the distance along it is the depth along
            // the axis.
            let (x, y) = camera.ray(f64::from(u), f64::from(v));
            let direction = pose.rotation * Vector3::new(x, y, 1.0);
            let sample = scene
                .first_hit(origin, direction)
                .map_or(0, |depth| (depth / f64::from(DEPTH_UNIT)).round() as u16);
            sample.to_le_bytes()
        })
        .collect()
}

fn median(mut times: Vec<Duration>) -> Duration {
    times.sort();
    times[times.len() / 2]
}

#[test]
#[ignore = "a measurement: it prints the times on the machine that runs it"]
fn time_one_640x360_frame() {
    let tree = Tree::from_robot(OPENARM_V2.robot()).expect("the URDF is one tree");
    let root = Frame::parse(&tree.link(tree.root()).name).expect("the URDF root names a frame");
    // The chest camera's optical frame in the URDF root's, over the fixed joints that
    // hold it.
    let chest = OPENARM_V2
        .cameras()
        .iter()
        .find(|camera| camera.name == CAMERA_NAME)
        .expect("OpenArm v2 carries the chest camera");
    let [roll, pitch, yaw] = chest.rpy;
    let camera_in_root = tree
        .fixed_path_to(tree.root(), &chest.parent_link)
        .expect("the chest camera is fixed to the root")
        * Isometry3::from_parts(
            Translation3::from(chest.xyz),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        );
    let optical = pose_of(&camera_in_root);

    let arms = &*OPENARM_V2_ARMS;
    let ready = posture(states::READY);
    let placed: Vec<_> = arms
        .iter()
        .zip(&ready)
        .map(|(arm, q)| arm.place(q))
        .collect();
    let scene = Scene {
        boxes: vec![
            ([0.35, -0.3, 0.0], [0.55, -0.1, 0.15]),
            ([0.45, 0.05, 0.0], [0.6, 0.25, 0.3]),
            ([0.7, -0.15, 0.0], [0.8, 0.15, 0.45]),
        ],
        balls: arms
            .iter()
            .zip(&ready)
            .map(|(arm, q)| (arm.chain().at(q).ee_pose().translation.vector.into(), 0.035))
            .collect(),
        pieces: placed
            .iter()
            .flat_map(|arm| arm.pieces().map(|(_, piece)| piece))
            .collect(),
    };
    let camera = Pinhole::parse(&CAMERA).expect("a valid model");
    let frame = render(&camera, &camera_in_root, &scene);
    let padding = Padding::new(1.0, 0.03).expect("a valid padding");
    let masks: Vec<_> = arms.iter().map(|arm| arm.mask(padding)).collect();
    let empty_map = || {
        VoxelMap::new(
            root.clone(),
            PositiveFinite::parse(0.02).expect("positive"),
            Bounds {
                min: [-0.5, -1.0, -0.1],
                max: [1.5, 1.0, 1.9],
            },
        )
        .expect("a valid map")
    };
    let max_range_m = PositiveFinite::parse(3.0).expect("positive");
    let at_root = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose");

    let (mut deprojected, mut cut, mut bare, mut fields) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut points, mut masked) = (Vec::new(), 0);
    for _ in 0..RUNS {
        let started = Instant::now();
        points = depth_points(&frame, &camera, DEPTH_UNIT, DepthModel::Z).expect("a frame");
        deprojected.push(started.elapsed());
        let scan = Scan {
            frame: &root,
            sensor: optical,
            points: &points,
            max_range_m,
        };

        let mut map = empty_map();
        let started = Instant::now();
        let chains = masks
            .iter()
            .zip(&ready)
            .map(|(mask, q)| mask.at(q))
            .collect();
        let robot = RobotCutout::new(&root, at_root, chains);
        map.insert(&scan, &[&robot]).expect("the map's frame");
        cut.push(started.elapsed());
        masked = points
            .iter()
            .filter(|point| robot.contains(optical.apply(point.map(f64::from))))
            .count();

        let started = Instant::now();
        let field = DistanceField::of(&map);
        fields.push(started.elapsed());
        let below_the_camera = field.distance([0.5, 0.0, 0.0]).expect("a point");
        assert!(below_the_camera.is_finite(), "the map holds nothing");

        let mut map = empty_map();
        let started = Instant::now();
        map.insert(&scan, &[]).expect("the map's frame");
        bare.push(started.elapsed());
    }
    assert!(
        masked >= MIN_MASKED,
        "the arms' mask holds {masked} of the frame's points"
    );
    println!(
        "640x360 frame, {} points, {masked} of them in the arms' mask, at 2 cm voxels, \
         median of {RUNS}: deprojected in {:?}, inserted in {:?} with both arms cut out \
         and {:?} with nothing cut out; distance field in {:?}",
        points.len(),
        median(deprojected),
        median(cut),
        median(bare),
        median(fields),
    );
}
