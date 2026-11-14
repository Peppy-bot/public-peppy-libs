//! What the tests share: OpenArm v2's collision meshes, its arms and the fingers the left
//! hand carries, the left arm as `srs_model` models it, the shoulder and wrist branch of a
//! configuration and configurations drawn off the seams of the branches, a deadline no
//! sampling reaches, voxel maps of drawn points, the distance from an arm to every occupied
//! voxel, the distance from the objects an arm carries to its own pieces, and whether a pose
//! stands within the precision of a Cartesian path's line. The crate of each test that
//! shares it holds `openarm_v2` too.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use std::path::Path;
use std::sync::LazyLock;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{Draws, PRECISION_M, PRECISION_RAD, interpolate};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider, RiderBody};
use collision_geometry::gjk::{self, Placed};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{Arm, Arms, Body};
use nalgebra::{Isometry3, Point3};
use occupancy_map::{Bounds, Frame, Scan, VoxelBox, VoxelMap};
use robot_description::ArmId;

use crate::openarm_v2::{self, JOINTS, LEFT, RIGHT};

/// The samples of a line `on_line` compares a pose with: the pose's nearest sample lies at
/// most half a sample from its nearest point of the line, which `on_line` allows.
const LINE_SAMPLES: usize = 1000;

/// Whether `actual` stands within the precision of a pose of the line from `a` to `b`.
pub fn on_line(actual: &Isometry3<f64>, a: &Isometry3<f64>, b: &Isometry3<f64>) -> bool {
    let length = (b.translation.vector - a.translation.vector).norm();
    let turn = a.rotation.angle_to(&b.rotation);
    let half_sample = 0.5 / LINE_SAMPLES as f64;
    (0..=LINE_SAMPLES).any(|k| {
        let target = interpolate(a, b, k as f64 / LINE_SAMPLES as f64);
        (actual.translation.vector - target.translation.vector).norm()
            <= PRECISION_M + length * half_sample
            && actual.rotation.angle_to(&target.rotation) <= PRECISION_RAD + turn * half_sample
    })
}

/// The directory of OpenArm v2's collision meshes, written once per test binary.
pub static MESHES: LazyLock<String> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("openarm_v2_meshes");
    openarm_v2::model()
        .write_meshes_to(&dir)
        .expect("write the meshes");
    dir.to_str()
        .expect("the mesh directory is UTF-8")
        .to_string()
});

/// The collision pieces of OpenArm v2's left arm, from its base to its grasp point.
pub static LEFT_ARM: LazyLock<ChainGeometry<JOINTS>> =
    LazyLock::new(|| openarm_v2::arm_geometry(LEFT, &MESHES));

/// The collision pieces of OpenArm v2's right arm, from its base to its grasp point.
pub static RIGHT_ARM: LazyLock<ChainGeometry<JOINTS>> =
    LazyLock::new(|| openarm_v2::arm_geometry(RIGHT, &MESHES));

/// OpenArm v2's arm `id` as a plan holds it: by its name in the description, with the
/// collision pieces of `LEFT_ARM` or `RIGHT_ARM`.
pub fn plan_arm(id: ArmId) -> Arm<'static, JOINTS> {
    let geometry: &'static ChainGeometry<JOINTS> = if id == LEFT { &LEFT_ARM } else { &RIGHT_ARM };
    Arm::new(&openarm_v2::arm(id).name, geometry)
}

/// OpenArm v2's left arm as `srs_model` models it, from its base to its grasp point.
pub fn left_srs_arm() -> srs_model::Arm {
    let left = openarm_v2::arm(LEFT);
    srs_model::Arm::from_urdf(openarm_v2::DESCRIPTION.urdf(), &left.base_link)
        .expect("an SRS arm")
        .with_tool_link(&left.grasp_link)
        .expect("its grasp point")
}

/// The parts the left arm's links carry, the gripper's fingers, closed, as riders of its
/// hand.
pub fn left_fingers() -> Vec<Rider<'static>> {
    LEFT_ARM.carried().iter().map(CarriedPart::rider).collect()
}

/// The shoulder and wrist branch of `q`: on which side of a quarter turn the middle joint
/// of each spherical limb stands. The two solutions of a limb differ there.
pub fn branch(q: &[f64; JOINTS]) -> (bool, bool) {
    (q[1].cos() >= 0.0, q[5].cos() >= 0.0)
}

/// Configurations of the left arm drawn inside its limits from `seed`, the
/// middle joint of each spherical limb (joints 2 and 6) at least a quarter of pi from the
/// quarter turn its two branches meet at.
pub fn off_the_branch_seams(seed: u64) -> impl Iterator<Item = [f64; JOINTS]> {
    let limits = LEFT_ARM.chain().limits();
    let mut draws = Draws::new(seed);
    std::iter::repeat_with(move || -> [f64; JOINTS] {
        std::array::from_fn(|i| limits[i].lo + (limits[i].hi - limits[i].lo) * draws.unit())
    })
    .filter(|q| {
        [q[1], q[5]]
            .iter()
            .all(|v| v.cos().abs() >= std::f64::consts::FRAC_1_SQRT_2)
    })
}

/// `value`, a tolerance of the tests above 0.
pub fn positive(value: f64) -> PositiveFinite {
    PositiveFinite::parse(value).expect("a positive tolerance")
}

/// How much farther than the clearance asked of it a plan keeps its states from the map and
/// the objects, as its messages name the plan's clearance.
pub const PLAN_SPARE_M: f64 = 0.001;

/// A deadline no test's goal sampling reaches: an hour from now.
pub fn an_hour_from_now() -> Instant {
    Instant::now() + Duration::from_secs(3600)
}

/// The frame of OpenArm v2's URDF root, which the maps stand in.
pub fn world() -> Frame {
    Frame::parse(openarm_v2::root_link()).expect("the URDF root names a frame")
}

/// A map of `voxel_m` voxels over the cube of half side `half_m` about the robot's
/// origin.
pub fn map(voxel_m: f64, half_m: f64) -> VoxelMap {
    VoxelMap::new(
        world(),
        PositiveFinite::parse(voxel_m).expect("positive"),
        Bounds {
            min: [-half_m; 3],
            max: [half_m; 3],
        },
    )
    .expect("a valid map")
}

/// `map` after one scan of `points` from a sensor at `origin`: each point's voxel
/// occupied.
pub fn occupied_at(map: &mut VoxelMap, origin: [f64; 3], points: &[[f64; 3]]) {
    let relative: Vec<[f32; 3]> = points
        .iter()
        .map(|p| std::array::from_fn(|i| (p[i] - origin[i]) as f32))
        .collect();
    map.insert(
        &Scan {
            frame: &world(),
            sensor: Pose::new(origin, [0.0, 0.0, 0.0, 1.0]).expect("a pose"),
            points: &relative,
            max_range_m: PositiveFinite::parse(10.0).expect("positive"),
        },
        &[],
    )
    .expect("the scan stands in the map's frame");
}

/// The box of every occupied voxel of `map`.
pub fn voxel_boxes(map: &VoxelMap) -> Vec<VoxelBox> {
    map.occupied().map(|key| map.voxel_box(key)).collect()
}

/// The least distance at `q` between a shape of an object an arm of `arms` carries and a
/// piece of that arm's links and of the parts they carry that the object may not touch,
/// measured shape against piece with no screen: `may_touch` takes the object's id and the
/// link's name.
pub fn held_from_its_arm<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    q: &[f64; D],
    may_touch: impl Fn(&str, &str) -> bool,
) -> f64 {
    let bodies = arms.bodies(q);
    let may_touch = &may_touch;
    let shapes = bodies
        .iter()
        .filter_map(|held| held.body.shape().map(|shape| (held.arm, held.name, shape)));
    shapes
        .flat_map(|(arm, object, shape)| {
            bodies
                .iter()
                .filter(move |body| body.arm == arm && !may_touch(object, body.name))
                .filter_map(move |body| match &body.body {
                    Body::Piece(piece) => Some(shape.distance_to(piece, piece.bounding_sphere())),
                    Body::Shape(_) => None,
                })
        })
        .fold(f64::INFINITY, f64::min)
}

/// The least distance from every body of `arms` at `q` to the box of every occupied voxel
/// of `map`, with no screen.
pub fn distance_to_map<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    map: &VoxelMap,
    q: &[f64; D],
) -> f64 {
    let boxes = voxel_boxes(map);
    arms.bodies(q)
        .iter()
        .flat_map(|body| {
            boxes
                .iter()
                .map(|b| body.body.distance_to_convex(b, b.bounding_sphere()))
        })
        .fold(f64::INFINITY, f64::min)
}

/// The center of the voxel of `map` inside a finger of the left arm at `q` that lies
/// farthest from the bare arm: of the voxels holding a point a seventh of the way in from
/// a vertex of a finger's piece toward its center, one the finger touches. With that
/// distance from the bare arm. Each finger stands where its hand carries it.
pub fn voxel_in_a_finger(map: &VoxelMap, q: &[f64; JOINTS]) -> ([f64; 3], f64) {
    let bare = plan_arm(LEFT).bodies(q);
    let posed = LEFT_ARM.chain().at(q);
    let fingers: Vec<Placed> = left_fingers()
        .into_iter()
        .flat_map(|rider| {
            let pose = posed.link_pose_world(rider.segment) * rider.offset;
            let RiderBody::Part(part) = rider.body else {
                panic!("a finger is a part of the robot");
            };
            part.pieces()
                .iter()
                .map(move |hull| Placed::new(hull, pose))
        })
        .collect();
    let voxel_of = |point: Point3<f64>| {
        let key = map.key(point.into()).expect("a point inside the map");
        map.voxel_box(key)
    };
    fingers
        .iter()
        .flat_map(|piece| {
            let (center, _) = piece.bounding_sphere();
            piece
                .vertices()
                .map(move |vertex| voxel_of(vertex + (center - vertex) / 7.0))
                .filter(|voxel| gjk::distance(piece, voxel).distance <= 0.0)
                .collect::<Vec<_>>()
        })
        .map(|voxel| {
            let sphere = voxel.bounding_sphere();
            let away = bare
                .iter()
                .map(|(_, body)| body.distance_to_convex(&voxel, sphere))
                .fold(f64::INFINITY, f64::min);
            (sphere.0.into(), away)
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a voxel inside a finger")
}
