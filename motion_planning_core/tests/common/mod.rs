//! What the tests share: OpenArm v2's left arm and the fingers its hand carries, a
//! deadline no sampling reaches, voxel maps of drawn points, and the distance from an arm
//! to every occupied voxel.
//!
//! Each integration test is its own crate, so items unused by one suite are
//! expected; the allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::sync::LazyLock;
use std::time::{Duration, Instant};

use camera_core::pose::Pose;
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{CarriedPart, ChainGeometry, Rider};
use collision_geometry::gjk::{self, Placed, Support};
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::Arm;
use nalgebra::{Point3, Vector3};
use occupancy_map::{Bounds, Frame, Scan, VoxelMap};
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

/// The box of an occupied voxel, as GJK measures it.
pub struct VoxelBox {
    pub center: Point3<f64>,
    pub half: f64,
}

impl Support for VoxelBox {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        self.center + dir.map(|d| if d < 0.0 { -self.half } else { self.half })
    }
}

/// The box of every occupied voxel of `map`.
pub fn voxel_boxes(map: &VoxelMap) -> Vec<VoxelBox> {
    map.occupied()
        .map(|key| VoxelBox {
            center: map.center(key).into(),
            half: map.voxel_size() / 2.0,
        })
        .collect()
}

/// The least distance GJK measures from every piece of `arm` at `q` to the box of every
/// occupied voxel of `map`, with no screen.
pub fn distance_to_map<const N: usize>(arm: &Arm<N>, map: &VoxelMap, q: &[f64; N]) -> f64 {
    let boxes = voxel_boxes(map);
    arm.pieces(q)
        .iter()
        .flat_map(|(_, piece)| boxes.iter().map(|b| gjk::distance(piece, b).distance))
        .fold(f64::INFINITY, f64::min)
}

/// The center of the voxel of `map` inside a finger of the left arm at `q` that lies
/// farthest from the bare arm: of the voxels holding a point a seventh of the way in from
/// a vertex of a finger's piece toward its center, one the finger touches. With that
/// distance from the bare arm. Each finger stands where its hand carries it.
pub fn voxel_in_a_finger(map: &VoxelMap, q: &[f64; JOINTS]) -> ([f64; 3], f64) {
    let bare = Arm::new(&LEFT_ARM, None).pieces(q);
    let posed = LEFT_ARM.chain().at(q);
    let fingers: Vec<Placed> = left_fingers()
        .into_iter()
        .flat_map(|rider| {
            let pose = posed.link_pose_world(rider.segment) * rider.offset;
            rider.pieces.iter().map(move |hull| Placed::new(hull, pose))
        })
        .collect();
    let voxel_of = |point: Point3<f64>| {
        let key = map.key(point.into()).expect("a point inside the map");
        VoxelBox {
            center: map.center(key).into(),
            half: map.voxel_size() / 2.0,
        }
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
            let away = bare
                .iter()
                .map(|(_, piece)| gjk::distance(piece, &voxel).distance)
                .fold(f64::INFINITY, f64::min);
            (voxel.center.into(), away)
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a voxel inside a finger")
}
