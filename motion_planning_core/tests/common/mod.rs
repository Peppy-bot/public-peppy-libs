//! What the tests share: OpenArm v2's left arm and the fingers its hand carries, the
//! shoulder and wrist branch of a configuration and configurations drawn off the seams of
//! the branches, a deadline no sampling reaches, voxel maps of drawn points, the distance
//! from an arm to every occupied voxel, and the distance from the objects an arm carries
//! to its own pieces.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::sync::LazyLock;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider, RiderBody};
use collision_geometry::gjk::{self, Placed};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{Arm, Arms, Body};
use nalgebra::Point3;
use occupancy_map::{Bounds, Frame, Scan, VoxelBox, VoxelMap};
use openarm_description::{HardwareVersion, Side};

pub const JOINTS: usize = 7;

/// OpenArm v2's collision meshes, written once per test binary.
static MESHES_DIR: LazyLock<PathBuf> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("openarm_v2_meshes");
    HardwareVersion::V2
        .write_meshes_to(&dir)
        .expect("write the meshes");
    dir
});

/// The collision pieces of OpenArm v2's left arm, from its base to its grasp point.
pub static LEFT_ARM: LazyLock<ChainGeometry<JOINTS>> = LazyLock::new(|| {
    let version = HardwareVersion::V2;
    ChainGeometry::from_urdf(
        version.urdf(),
        MESHES_DIR.to_str().expect("the mesh directory is UTF-8"),
        &ChainSpec {
            base_link: Some(version.base_link(Side::Left)),
            tip_link: version.tcp_link(Side::Left),
            joints: JointSelection::PathOrder,
        },
    )
    .expect("the left arm fits")
});

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

/// SplitMix64 draws in `[0, 1)`.
pub struct SplitMix64(pub u64);

impl SplitMix64 {
    pub fn unit(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1_u64 << 53) as f64
    }
}

/// Configurations of the left arm drawn inside its limits by SplitMix64 from `seed`, the
/// middle joint of each spherical limb (joints 2 and 6) at least a quarter of pi from the
/// quarter turn its two branches meet at.
pub fn off_the_branch_seams(seed: u64) -> impl Iterator<Item = [f64; JOINTS]> {
    let limits = LEFT_ARM.chain().limits();
    let mut draws = SplitMix64(seed);
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

/// A deadline no test's goal sampling reaches: an hour from now.
pub fn an_hour_from_now() -> Instant {
    Instant::now() + Duration::from_secs(3600)
}

/// The frame of OpenArm's URDF root, which the maps stand in.
pub fn world() -> Frame {
    Frame::parse("world").expect("a frame name")
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
    let bare = Arm::new("left_arm", &LEFT_ARM).bodies(q);
    let posed = LEFT_ARM.chain().at(q);
    let fingers: Vec<Placed> = left_fingers()
        .into_iter()
        .flat_map(|rider| {
            let pose = posed.link_pose_world(rider.segment) * rider.offset;
            let RiderBody::Pieces(pieces) = rider.body else {
                panic!("a finger is a part of the robot");
            };
            pieces.iter().map(move |hull| Placed::new(hull, pose))
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
