//! The occupancy map against MoveIt's point cloud updater: `VoxelMap::insert` keys a
//! voxel, walks a ray and updates a log-odds in OctoMap's arithmetic, and places each
//! point by tf2's, so after the same clouds, with the same shapes cut out, it holds the
//! voxels of MoveIt's occupancy map, each at its log-odds bit for bit as MoveIt's amd64
//! build computes them. MoveIt's arm64 build fuses multiply-adds, and at most
//! `MAX_DIFFERING_SHARE` of its voxels differ. The evidence: after the clouds of every
//! occupancy scenario of the ROS reference harness, with its sphere cut out as the robot's
//! own body and no padding, the map holds exactly the voxels of the committed outputs,
//! those of MoveIt's amd64 build, each at their log-odds, on every architecture; against
//! the outputs that `ROS_REFERENCE_OUTPUTS` names, those of another image, the test counts
//! the voxels that differ and bounds their share.
//!
//! A scenario's body is a sphere, which the test cuts out as a [`Ball`]: a point nearer
//! its center than its radius lies inside, the strict test MoveIt's robot filter makes of
//! a body's bounding sphere, which for a sphere is the sphere. The test runs neither
//! `RobotCutout` nor collision_geometry's robot filter. The ball and MoveIt disagree only
//! on the sphere's surface, where the rounding of the frame that a point is judged in
//! decides: MoveIt judges a point in the sensor's frame, the map in its own. Every point
//! of a scenario within the range lies farther than [`SURFACE_CLEARANCE`] from that
//! surface, which the test checks, so no point lies where the frames disagree.

use std::collections::{BTreeMap, BTreeSet};

use camera_core::pose::Pose;
use control_core::positive_finite::PositiveFinite;
use occupancy_map::{Bounds, Frame, Scan, Voxel, VoxelMap};
use ros_reference::Outputs;
use ros_reference::occupancy::{Cases, Cloud};

#[path = "common/ball.rs"]
mod ball;
use ball::Ball;

/// The bounds of the map, which hold every voxel that a scenario's clouds reach.
const BOUNDS: Bounds = Bounds {
    min: [-4.0; 3],
    max: [4.0; 3],
};

/// The fewest voxels a scenario's map holds for the comparison to cover it: half the 398
/// of `occupancy_room`'s map.
const MIN_VOXELS: usize = 199;

/// How far every point of a scenario within the range lies from its sphere's surface, in
/// meters: far beyond the rounding of placing a point in the map's frame or in the
/// sensor's, about 1e-15 m a meter out, and a thousandth of the margin that
/// `draw_occupancy.py` keeps.
const SURFACE_CLEARANCE: f64 = 1e-6;

/// The largest share of the reference's voxels that may differ from the map's against the
/// outputs of another image. MoveIt's arm64 build fuses multiply-adds in its point cloud
/// callback (31 of them in `cloudMsgCallback`). Its map of `occupancy_room` equals the
/// amd64 one in all 398 voxels, and that of a drawn set of 60 points of each drawn case
/// (`draw_occupancy.py --drawn 60`) differs from the amd64 one in 6 of 7187 voxels
/// (0.08%).
const MAX_DIFFERING_SHARE: f64 = 0.01;

/// Where the sensor of `cloud` stands in the map's frame.
fn sensor_of(cloud: &Cloud) -> Pose {
    Pose::new(cloud.sensor.position, cloud.sensor.orientation).expect("a sensor pose")
}

/// How near the surface of `cases`'s sphere the nearest point of its clouds within the
/// range lies, each point placed in the map's frame as the map places it.
fn nearest_to_the_surface(cases: &Cases) -> f64 {
    let sphere = &cases.cut_out;
    cases
        .clouds
        .iter()
        .flat_map(|cloud| {
            let sensor = sensor_of(cloud);
            cloud
                .points
                .iter()
                .map(|point| point.map(f64::from))
                .filter(|local| local.iter().map(|c| c * c).sum::<f64>().sqrt() <= cases.max_range)
                .map(move |local| sensor.apply(local))
        })
        .map(|point| {
            let from_center = (0..3)
                .map(|i| (point[i] - sphere.center[i]).powi(2))
                .sum::<f64>()
                .sqrt();
            (from_center - sphere.radius).abs()
        })
        .fold(f64::INFINITY, f64::min)
}

/// The map after the clouds of `cases`, with its sphere cut out.
fn map_after(cases: &Cases) -> VoxelMap {
    let frame = Frame::parse("map").expect("a frame name");
    let voxel_size = PositiveFinite::parse(cases.voxel_size).expect("a positive voxel size");
    let mut map = VoxelMap::new(frame.clone(), voxel_size, BOUNDS).expect("a valid map");
    let ball = Ball {
        frame: frame.clone(),
        center: cases.cut_out.center,
        radius: cases.cut_out.radius,
    };
    let max_range_m = PositiveFinite::parse(cases.max_range).expect("a positive range");
    for cloud in &cases.clouds {
        map.insert(
            &Scan {
                frame: &frame,
                sensor: sensor_of(cloud),
                points: &cloud.points,
                max_range_m,
            },
            &[&ball],
        )
        .expect("the clouds stand in the map's frame");
    }
    map
}

/// The log-odds of each voxel that `map` knows, by its key.
fn voxels_of(map: &VoxelMap) -> BTreeMap<[i32; 3], f64> {
    map.known()
        .map(|key| match map.voxel(key) {
            Voxel::Unknown => unreachable!("the map knows voxel {key:?}"),
            Voxel::Free(log_odds) | Voxel::Occupied(log_odds) => (key.0, f64::from(log_odds)),
        })
        .collect()
}

/// The voxels that differ between `ours` and `reference`: those that one holds alone, and
/// those that both hold at other log-odds.
fn differing(ours: &BTreeMap<[i32; 3], f64>, reference: &BTreeMap<[i32; 3], f64>) -> Vec<[i32; 3]> {
    let keys: BTreeSet<[i32; 3]> = ours.keys().chain(reference.keys()).copied().collect();
    keys.into_iter()
        .filter(|key| ours.get(key).map(|v| v.to_bits()) != reference.get(key).map(|v| v.to_bits()))
        .collect()
}

// After each scenario's clouds, the map holds exactly the voxels of the committed outputs,
// each at their log-odds, and all but at most MAX_DIFFERING_SHARE of the reference's
// voxels against the outputs of another image. No point lies within SURFACE_CLEARANCE of
// the sphere's surface.
#[test]
fn the_map_holds_moveits_voxels_after_the_same_clouds() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds an occupancy scenario"
    );
    for cases in &scenarios {
        let name = &cases.name;
        assert!(
            cases.voxels.len() >= MIN_VOXELS,
            "{name}: only {} voxels to compare",
            cases.voxels.len()
        );
        let nearest = nearest_to_the_surface(cases);
        assert!(
            nearest > SURFACE_CLEARANCE,
            "{name}: a point lies {nearest} m from the sphere's surface, where the robot \
             filters or the frames may disagree"
        );
        let reference: BTreeMap<[i32; 3], f64> = cases.voxels.iter().copied().collect();
        let ours = voxels_of(&map_after(cases));
        let differing = differing(&ours, &reference);
        let first: Vec<_> = differing
            .iter()
            .take(5)
            .map(|key| (key, ours.get(key), reference.get(key)))
            .collect();
        match Outputs::from_environment() {
            Outputs::Committed => assert!(
                differing.is_empty(),
                "{name}: {} voxels differ from MoveIt's, the first (key, ours, MoveIt's): \
                 {first:?}",
                differing.len()
            ),
            Outputs::Named(directory) => assert!(
                differing.len() as f64 <= MAX_DIFFERING_SHARE * reference.len() as f64,
                "{name}: {} of MoveIt's {} voxels in {} differ, the first (key, ours, \
                 MoveIt's): {first:?}",
                differing.len(),
                reference.len(),
                directory.display()
            ),
        }
    }
}
