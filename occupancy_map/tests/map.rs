//! Properties of the voxel map on drawn scenes: what one point does to the voxels of its
//! ray, what an empty depth sample and a point that is not a number do, what repeated
//! scans of a still scene make of a surface, the robot cut out where it stands, its
//! chains and the body fixed around them, the frames a map refuses, the distance field
//! against a brute-force distance inside the map and outside it, a rendered depth frame
//! of a box, and a collision object clearing the voxels it intersects or touches, those
//! it covers in part among them, a mesh clearing the voxels each one measured on its own
//! finds, and cutting what the sensor sees of it, grown by a padding, out of a scan;
//! points that clear their rays and mark nothing; one update for each voxel of a scan
//! whose rays share voxels, the strongest; and the map's columns, each voxel's center
//! inside its column, and the clear of a column.

use std::collections::{BTreeMap, BTreeSet};

use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use collision_geometry::chain::{BodyGeometry, Padding};
use collision_geometry::gjk::Vertex;
use collision_geometry::shapes::{PlacedShape, Shape};
use control_core::positive_finite::PositiveFinite;
use frame_tree::FrameName;
use frame_tree::Transform;
use hegel::TestCase;
use hegel::generators as gs;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{
    Bounds, Columns, Cutout, DistanceField, InsertError, Key, NotANumber, Reading, RobotCutout,
    Scan, ShapeCutout, Voxel, VoxelMap, depth_points,
};

#[path = "common/arms.rs"]
mod arms;
#[path = "common/ball.rs"]
mod ball;
#[path = "common/meshes.rs"]
mod meshes;
#[path = "common/pose.rs"]
mod pose;
#[path = "common/slab.rs"]
mod slab;
use arms::{LEFT, OPENARM_V2_ARMS, RIGHT};
use ball::Ball;
use pose::pose_of;
use slab::box_entry;

const VOXEL_M: f64 = 0.05;
const RANGE_M: f64 = 3.0;
/// The map's frame.
const MAP: &str = "map";
/// The edge of the map's bounds on every axis, in meters.
const EDGE_M: f64 = 2.0;
/// How far two computations of one distance may differ by rounding alone.
const ROUNDING_M: f64 = 1e-9;

fn frame() -> FrameName {
    FrameName::parse(MAP).expect("a frame name")
}

fn range(meters: f64) -> PositiveFinite {
    PositiveFinite::parse(meters).expect("a positive range")
}

fn map() -> VoxelMap {
    map_of(VOXEL_M)
}

/// An empty map 4 m across of voxels `voxel_m` on a side.
fn map_of(voxel_m: f64) -> VoxelMap {
    VoxelMap::new(
        frame(),
        PositiveFinite::parse(voxel_m).expect("positive"),
        Bounds {
            min: [-EDGE_M; 3],
            max: [EDGE_M; 3],
        },
    )
    .expect("a valid map")
}

/// OctoMap's lower bound of a voxel's log-odds, the log-odds of 0.1192.
fn lowest() -> f32 {
    (0.1192_f64 / (1.0 - 0.1192)).ln() as f32
}

/// OctoMap's log-odds of one miss, the log-odds of 0.4.
fn one_miss() -> f32 {
    (0.4_f64 / (1.0 - 0.4)).ln() as f32
}

/// OctoMap's log-odds of one hit, the log-odds of 0.7.
fn one_hit() -> f32 {
    (0.7_f64 / (1.0 - 0.7)).ln() as f32
}

/// The pose that places nothing anywhere else.
fn identity() -> Transform {
    Transform::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose")
}

/// A sensor somewhere in the middle of the map, turned any way.
fn sensor(tc: &TestCase) -> Transform {
    let origin: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-0.5).max_value(0.5)));
    let q: [f64; 4] = tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)));
    let norm = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    let q = if norm > 0.1 {
        q.map(|c| c / norm)
    } else {
        [0.0, 0.0, 0.0, 1.0]
    };
    Transform::new(origin, q).expect("a unit quaternion")
}

/// A unit vector, any way.
fn direction(tc: &TestCase) -> Vector3<f64> {
    let raw = Vector3::from(tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0))));
    if raw.norm() > 0.1 {
        raw.normalize()
    } else {
        Vector3::z()
    }
}

/// A point at a distance of `near` to `far` from the sensor, any way.
fn point(tc: &TestCase, near: f64, far: f64) -> [f32; 3] {
    let direction = direction(tc);
    let distance: f64 = tc.draw(gs::floats().min_value(near).max_value(far));
    std::array::from_fn(|i| (direction[i] * distance) as f32)
}

/// The key `map` gives a sensor point that it holds.
fn key_of(map: &VoxelMap, sensor: &Transform, p: [f32; 3]) -> Key {
    map.key(sensor.apply(p.map(f64::from))).expect("in the map")
}

fn log_odds(map: &VoxelMap, key: Key) -> Option<f32> {
    match map.voxel(key) {
        Voxel::Unknown => None,
        Voxel::Free(v) | Voxel::Occupied(v) => Some(v),
    }
}

/// Inserts the points a sensor at `sensor` reads, cutting out `cutouts`.
fn insert(map: &mut VoxelMap, sensor: Transform, points: &[[f32; 3]], cutouts: &[&dyn Cutout]) {
    let frame = frame();
    map.insert(
        &Scan {
            frame: &frame,
            sensor,
            points,
            max_range_m: range(RANGE_M),
        },
        cutouts,
    )
    .expect("a scan in the map's frame");
}

/// The free voxels of a map.
fn free_keys(map: &VoxelMap) -> Vec<Key> {
    map.known()
        .filter(|key| matches!(map.voxel(*key), Voxel::Free(_)))
        .collect()
}

/// Every voxel the map knows, with what it holds.
fn voxels(map: &VoxelMap) -> Vec<(Key, Voxel)> {
    map.known().map(|key| (key, map.voxel(key))).collect()
}

// A point within range is a hit: its voxel gains log-odds and every voxel its
// ray crosses before it loses, the sensor's own voxel first.
#[hegel::test]
fn a_ray_clears_what_it_crosses_and_marks_its_hit(tc: TestCase) {
    let sensor = sensor(&tc);
    let p = point(&tc, 0.3, 1.2);
    let mut map = map();
    insert(&mut map, sensor, &[p], &[]);
    let hit = key_of(&map, &sensor, p);
    let after_one = log_odds(&map, hit).expect("the hit is known");
    assert!(after_one > 0.0, "the hit reads {after_one}");
    let ray = free_keys(&map);
    assert!(!ray.is_empty(), "the ray crosses no voxel");
    // The rays run from the origin in single precision.
    let origin = map
        .key(sensor.origin().map(|c| f64::from(c as f32)))
        .expect("in the map");
    assert!(ray.contains(&origin) || origin == hit);
    let before: Vec<f32> = ray
        .iter()
        .map(|k| log_odds(&map, *k).expect("known"))
        .collect();
    insert(&mut map, sensor, &[p], &[]);
    assert!(
        log_odds(&map, hit).expect("known") > after_one,
        "the hit did not gain"
    );
    for (key, value) in ray.iter().zip(before) {
        let now = log_odds(&map, *key).expect("known");
        assert!(now < value || now == lowest(), "{key:?} did not lose");
    }
}

// A point with a coordinate that is not a finite number changes nothing,
// within the range, beyond it, or inside the robot.
#[test]
fn a_point_that_is_not_a_finite_number_changes_nothing() {
    let mut map = map();
    let sensor = Transform::new([0.12, -0.07, 0.31], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let points = [
        [f32::NAN, 0.0, 1.0],
        [0.0, f32::NAN, 1.0],
        [0.0, 0.0, f32::NAN],
        [f32::INFINITY, 0.0, 1.0],
        [0.0, f32::NEG_INFINITY, 1.0],
    ];
    let everywhere = Ball {
        frame: frame(),
        center: [0.0; 3],
        radius: f64::MAX,
    };
    insert(&mut map, sensor, &points, &[]);
    insert(&mut map, sensor, &points, &[&everywhere]);
    assert_eq!(map.known().count(), 0);
}

/// The bytes of a z16 frame of `samples`.
fn z16(samples: &[u16]) -> Vec<u8> {
    samples.iter().flat_map(|s| s.to_le_bytes()).collect()
}

// A depth sample of 0, the no-reading value, makes no point and changes
// nothing: on a drawn frame each empty sample takes away its own pixel's
// point and leaves every other, and a frame of empty samples leaves a drawn
// map as it was.
#[hegel::test]
fn an_empty_sample_makes_no_point_and_changes_nothing(tc: TestCase) {
    let width: u32 = tc.draw(gs::integers().min_value(1).max_value(12));
    let height: u32 = tc.draw(gs::integers().min_value(1).max_value(8));
    let camera = Pinhole::parse(&Intrinsics {
        width,
        height,
        fx: 6.0,
        fy: 6.0,
        cx: f64::from(width - 1) / 2.0,
        cy: f64::from(height - 1) / 2.0,
        distortion_model: "none",
        distortion: &[],
    })
    .expect("a valid model");
    let pixels = (width * height) as usize;
    let read = |samples: &[u16]| {
        depth_points(&z16(samples), &camera, 0.001, DepthModel::Z).expect("a valid frame")
    };
    let full: Vec<u16> = (0..pixels)
        .map(|_| tc.draw(gs::integers().min_value(1)))
        .collect();
    let empty: Vec<bool> = (0..pixels).map(|_| tc.draw(gs::booleans())).collect();
    let holed: Vec<u16> = full
        .iter()
        .zip(&empty)
        .map(|(&sample, &empty)| if empty { 0 } else { sample })
        .collect();
    let all = read(&full);
    assert_eq!(all.len(), pixels);
    let kept: Vec<[f32; 3]> = all
        .iter()
        .zip(&empty)
        .filter(|(_, empty)| !**empty)
        .map(|(point, _)| *point)
        .collect();
    assert_eq!(read(&holed), kept);

    let sensor = sensor(&tc);
    let mut map = map();
    let drawn: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(8)))
        .map(|_| point(&tc, 0.2, 1.4))
        .collect();
    insert(&mut map, sensor, &drawn, &[]);
    let before = voxels(&map);
    let nothing = read(&vec![0; pixels]);
    assert!(nothing.is_empty());
    insert(&mut map, sensor, &nothing, &[]);
    assert_eq!(voxels(&map), before);
}

// A point beyond the range marks nothing: the voxels its ray crosses up to the
// range lose log-odds, the voxel at the range among them, the point at the
// range placed as tf2's normalize places it, and nothing is occupied.
#[hegel::test]
fn a_point_beyond_the_range_clears_to_the_range_and_marks_nothing(tc: TestCase) {
    let sensor = sensor(&tc);
    let max_range: f64 = tc.draw(gs::floats().min_value(0.3).max_value(1.2));
    let p = point(&tc, max_range * 1.2, max_range * 2.0);
    let mut map = map();
    let frame = frame();
    map.insert(
        &Scan {
            frame: &frame,
            sensor,
            points: &[p],
            max_range_m: range(max_range),
        },
        &[],
    )
    .expect("a scan in the map's frame");
    assert_eq!(
        map.occupied().count(),
        0,
        "a point beyond the range marked a voxel"
    );
    // tf2's normalize: each coordinate times the reciprocal of the point's
    // distance from the sensor, here times the range.
    let local = p.map(f64::from);
    let inverse = 1.0 / (local[0] * local[0] + local[1] * local[1] + local[2] * local[2]).sqrt();
    let at_range = map
        .key(sensor.apply(local.map(|c| c * inverse * max_range)))
        .expect("in the map");
    assert!(
        matches!(map.voxel(at_range), Voxel::Free(_)),
        "the voxel at the range reads {:?}",
        map.voxel(at_range)
    );
    if let Some(beyond) = map.key(sensor.apply(local)) {
        assert!(
            !matches!(map.voxel(beyond), Voxel::Free(_)),
            "the voxel of the point itself is cleared"
        );
    }
}

// A scan reads each of its drawn points, in their order, in the map's frame: one within
// the range is a return where the sensor places it, and one beyond the range no return at
// the range along its ray.
#[hegel::test]
fn a_scan_reads_each_point_as_a_return_or_no_return_at_the_range(tc: TestCase) {
    let sensor = sensor(&tc);
    let max_range: f64 = tc.draw(gs::floats().min_value(0.3).max_value(1.2));
    let points: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(20)))
        .map(|_| {
            if tc.draw(gs::booleans()) {
                point(&tc, 0.05, max_range * 0.9)
            } else {
                point(&tc, max_range * 1.1, max_range * 2.0)
            }
        })
        .collect();
    let frame = frame();
    let scan = Scan {
        frame: &frame,
        sensor,
        points: &points,
        max_range_m: range(max_range),
    };
    let readings: Vec<Reading> = scan.readings().collect();
    assert_eq!(readings.len(), points.len(), "a reading for each point");
    let origin = Vector3::from(sensor.origin());
    for (p, reading) in points.iter().zip(readings) {
        let local = p.map(f64::from);
        let beyond = Vector3::from(local).norm() > max_range;
        let along = Vector3::from(sensor.apply(local)) - origin;
        match reading {
            Reading::Return(at) => {
                assert!(!beyond, "{p:?} lies beyond {max_range} m and reads {at:?}");
                assert_eq!(at, sensor.apply(local));
            }
            Reading::NoReturn(at) => {
                assert!(
                    beyond,
                    "{p:?} lies within {max_range} m and reads no return"
                );
                let from_sensor = Vector3::from(at) - origin;
                assert!(
                    (from_sensor.norm() - max_range).abs() < ROUNDING_M,
                    "no return {} m from the sensor",
                    from_sensor.norm()
                );
                assert!(
                    from_sensor.normalize().dot(&along.normalize()) > 1.0 - ROUNDING_M,
                    "no return off the ray of {p:?}"
                );
            }
        }
    }
}

/// The scan of `points` a sensor at `sensor` reads out to `max_range_m`.
fn scan_of<'a>(
    frame: &'a FrameName,
    sensor: Transform,
    points: &'a [[f32; 3]],
    max_range_m: f64,
) -> Scan<'a> {
    Scan {
        frame,
        sensor,
        points,
        max_range_m: range(max_range_m),
    }
}

// Points that clear and mark nothing, drawn within the range and beyond it beside a drawn
// scan's points: each voxel the scan alone knows reads as the scan alone leaves it, a hit
// their rays cross among them; each voxel their rays cross up to their own, or up to the
// range, which a scan of them alone frees, is known; and each other voxel known reads one
// miss, so nothing more is occupied.
#[hegel::test]
fn points_that_clear_free_their_rays_and_mark_nothing(tc: TestCase) {
    let sensor = sensor(&tc);
    let max_range: f64 = tc.draw(gs::floats().min_value(0.3).max_value(1.2));
    let drawn = || -> Vec<[f32; 3]> {
        (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(8)))
            .map(|_| {
                if tc.draw(gs::booleans()) {
                    point(&tc, 0.05, max_range * 0.9)
                } else {
                    point(&tc, max_range * 1.1, max_range * 2.0)
                }
            })
            .collect()
    };
    let (points, clearing) = (drawn(), drawn());
    let frame = frame();
    let mut alone = map();
    alone
        .insert(&scan_of(&frame, sensor, &points, max_range), &[])
        .expect("a scan in the map's frame");
    let mut cleared_alone = map();
    cleared_alone
        .insert(&scan_of(&frame, sensor, &clearing, max_range), &[])
        .expect("a scan in the map's frame");
    let mut with_clearing = map();
    with_clearing
        .insert_clearing(&scan_of(&frame, sensor, &points, max_range), &clearing)
        .expect("a scan in the map's frame");

    for key in alone.known() {
        assert_eq!(with_clearing.voxel(key), alone.voxel(key), "{key:?}");
    }
    for key in free_keys(&cleared_alone) {
        assert!(
            with_clearing.voxel(key) != Voxel::Unknown,
            "{key:?}, on a clearing ray, is unknown"
        );
    }
    for key in with_clearing.known() {
        assert!(
            alone.voxel(key) != Voxel::Unknown
                || (cleared_alone.voxel(key) != Voxel::Unknown
                    && log_odds(&with_clearing, key) == Some(one_miss())),
            "{key:?} reads {:?}",
            with_clearing.voxel(key)
        );
    }
}

// A point that clears, alone on its ray but for a hit of the scan, marks nothing: its own
// voxel stays unknown, the scan's hit stays occupied, and every other voxel the ray crosses
// up to its own reads one miss. A scan in another frame than the map's is refused, and
// changes nothing.
#[test]
fn a_point_that_clears_frees_its_ray_up_to_its_voxel_and_marks_nothing() {
    let sensor = Transform::new([0.012; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let (hit, clearing) = ([0.5_f32, 0.0, 0.0], [1.0_f32, 0.0, 0.0]);
    let frame = frame();
    let mut map = map();
    map.insert_clearing(&scan_of(&frame, sensor, &[hit], RANGE_M), &[clearing])
        .expect("a scan in the map's frame");
    let along_the_ray: BTreeSet<Key> = (0..20)
        .map(|i| {
            map.key([0.025 + VOXEL_M * f64::from(i), 0.012, 0.012])
                .expect("in the map")
        })
        .collect();
    let hit_key = key_of(&map, &sensor, hit);
    assert_eq!(known(&map), along_the_ray);
    assert_eq!(map.voxel(key_of(&map, &sensor, clearing)), Voxel::Unknown);
    assert_eq!(map.occupied().collect::<Vec<Key>>(), vec![hit_key]);
    for key in along_the_ray.iter().filter(|&&key| key != hit_key) {
        assert_eq!(log_odds(&map, *key), Some(one_miss()), "{key:?}");
    }

    let before = voxels(&map);
    let elsewhere = FrameName::parse("elsewhere").expect("a frame name");
    assert!(matches!(
        map.insert_clearing(&scan_of(&elsewhere, sensor, &[hit], RANGE_M), &[clearing]),
        Err(InsertError::ScanFrame { .. })
    ));
    assert_eq!(voxels(&map), before);
}

/// The six directions along the map's axes.
const AXES: [[i32; 3]; 6] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
];

/// What a point of a scan is to the map.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Role {
    /// A point that the scan sees.
    Seen,
    /// A point that the scan sees and a cutout holds.
    CutOut,
    /// A point that clears its ray and marks nothing.
    Clearing,
}

/// An update that a scan gives a voxel, from the weakest to the strongest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Update {
    Miss,
    Hit,
    CutOut,
}

/// A cutout that holds the points it lists, in the map's frame, and no other.
struct Listed {
    frame: FrameName,
    points: Vec<[f64; 3]>,
}

impl Cutout for Listed {
    fn frame(&self) -> &FrameName {
        &self.frame
    }

    fn contains(&self, point: [f64; 3]) -> bool {
        self.points.contains(&point)
    }
}

/// The key of the voxel of the tree that holds `point`, inside the map or outside it,
/// for a point that lies far from the faces of its voxel.
fn tree_key(point: [f64; 3]) -> Key {
    Key(point.map(|c| (c / VOXEL_M).floor() as i32))
}

/// The voxels from `from` up to and not including `to`, which lies on an axis through
/// `from`: the voxels that a ray from the center of `from` to the center of `to` crosses.
fn voxels_along_an_axis(from: Key, to: Key) -> Vec<Key> {
    let step: [i32; 3] = std::array::from_fn(|i| (to.0[i] - from.0[i]).signum());
    assert!(
        step.iter().filter(|s| **s != 0).count() <= 1,
        "{to:?} lies on no axis through {from:?}"
    );
    let count: i32 = (0..3).map(|i| (to.0[i] - from.0[i]).abs()).sum();
    (0..count)
        .map(|n| Key(std::array::from_fn(|i| from.0[i] + n * step[i])))
        .collect()
}

// Each voxel takes one update for each scan, the strongest that the scan gives it, when
// the rays of the scan share voxels: a voxel that many rays cross reads one miss, a voxel
// that a point hits and other rays cross reads one hit, and a voxel that holds a cut-out
// point reads the lowest log-odds, whatever else lies in it or crosses it. A point beyond
// the range gives its voxel at the range one miss, and a point that clears gives its own
// voxel nothing. The sensor stands at the center of a voxel and each point lies on an
// axis through it, so each ray crosses the voxels from the sensor's up to its end's and
// no other, and the test knows each voxel that each ray crosses. The first two points lie
// on one axis, so the ray of the farther crosses the voxel of the nearer. The map is
// smaller than the reach of the points: a ray that ends outside it clears the voxels of
// the map that it crosses, and the map knows no other voxel.
#[hegel::test]
fn each_voxel_takes_the_strongest_update_of_a_scan_once(tc: TestCase) {
    let frame = frame();
    let mut map = VoxelMap::new(
        frame.clone(),
        PositiveFinite::parse(VOXEL_M).expect("positive"),
        Bounds {
            min: [-0.6; 3],
            max: [0.6; 3],
        },
    )
    .expect("a valid map");
    let sensor_key = Key(tc.draw(gs::arrays(gs::integers().min_value(-4).max_value(4))));
    let sensor = Transform::new(map.center(sensor_key), [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    // A whole number of voxels, so a point at the range lies far from a face of its
    // voxel.
    let max_range_m =
        f64::from(tc.draw(gs::integers::<i32>().min_value(6).max_value(30))) * VOXEL_M;
    let clears = tc.draw(gs::booleans());
    let other = if clears { Role::Clearing } else { Role::CutOut };
    let axis = || tc.draw(gs::integers::<usize>().max_value(AXES.len() - 1));
    let shared = axis();
    let near: i32 = tc.draw(gs::integers().min_value(1).max_value(10));
    let far: i32 = tc.draw(gs::integers().min_value(near + 1).max_value(near + 15));
    let mut placed = vec![(shared, near), (shared, far)];
    for _ in 0..tc.draw(gs::integers::<usize>().max_value(10)) {
        placed.push((axis(), tc.draw(gs::integers().min_value(0).max_value(30))));
    }
    let points: Vec<([f32; 3], Role)> = placed
        .into_iter()
        .map(|(axis, steps)| {
            let within: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-0.4).max_value(0.4)));
            let local = std::array::from_fn(|i| {
                ((f64::from(AXES[axis][i] * steps) + within[i]) * VOXEL_M) as f32
            });
            let role = if tc.draw(gs::booleans()) {
                other
            } else {
                Role::Seen
            };
            (local, role)
        })
        .collect();
    let of_role = |kept: fn(Role) -> bool| -> Vec<[f32; 3]> {
        points
            .iter()
            .filter(|(_, role)| kept(*role))
            .map(|(point, _)| *point)
            .collect()
    };
    let (seen, clearing) = (
        of_role(|role| role != Role::Clearing),
        of_role(|role| role == Role::Clearing),
    );
    let cut = Listed {
        frame: frame.clone(),
        points: of_role(|role| role == Role::CutOut)
            .iter()
            .map(|point| sensor.apply(point.map(f64::from)))
            .collect(),
    };
    let scan = scan_of(&frame, sensor, &seen, max_range_m);
    if clears {
        map.insert_clearing(&scan, &clearing)
    } else {
        map.insert(&scan, &[&cut])
    }
    .expect("a scan in the map's frame");

    let in_map = |key: Key| map.key(map.center(key)).is_some();
    let mut expected: BTreeMap<Key, Update> = BTreeMap::new();
    let mut give = |key: Key, update: Update| {
        if in_map(key) {
            let held = expected.entry(key).or_insert(update);
            *held = (*held).max(update);
        }
    };
    let seen_roles = points
        .iter()
        .map(|(_, role)| *role)
        .filter(|role| *role != Role::Clearing);
    let clearing_scan = scan_of(&frame, sensor, &clearing, max_range_m);
    let readings = scan.readings().zip(seen_roles).chain(
        clearing_scan
            .readings()
            .map(|reading| (reading, Role::Clearing)),
    );
    for (reading, role) in readings {
        let (Reading::Return(end) | Reading::NoReturn(end)) = reading;
        let end = tree_key(end);
        for crossed in voxels_along_an_axis(sensor_key, end) {
            give(crossed, Update::Miss);
        }
        match (reading, role) {
            (Reading::NoReturn(_), _) => give(end, Update::Miss),
            (Reading::Return(_), Role::Seen) => give(end, Update::Hit),
            (Reading::Return(_), Role::CutOut) => give(end, Update::CutOut),
            (Reading::Return(_), Role::Clearing) => {}
        }
    }
    let expected: BTreeMap<Key, f32> = expected
        .into_iter()
        .map(|(key, update)| {
            let value = match update {
                Update::Miss => one_miss(),
                Update::Hit => one_hit(),
                Update::CutOut => lowest(),
            };
            (key, value)
        })
        .collect();
    let known: BTreeMap<Key, f32> = map
        .known()
        .map(|key| (key, log_odds(&map, key).expect("known")))
        .collect();
    assert_eq!(known, expected);
}

// A point inside a cut-out shape marks nothing: its voxel takes the lowest
// log-odds, and its ray still clears.
#[hegel::test]
fn a_point_inside_a_cutout_marks_nothing(tc: TestCase) {
    let sensor = sensor(&tc);
    let p = point(&tc, 0.3, 1.0);
    let robot = Ball {
        frame: frame(),
        center: sensor.apply(p.map(f64::from)),
        radius: 0.1,
    };
    let mut map = map();
    insert(&mut map, sensor, &[p], &[&robot]);
    assert_eq!(map.occupied().count(), 0);
    let cut = log_odds(&map, key_of(&map, &sensor, p)).expect("the cut-out voxel is known");
    assert_eq!(cut, lowest(), "the cut-out voxel reads {cut}");
    assert!(free_keys(&map).len() > 1, "the ray did not clear");
}

// A point inside OpenArm's arm grown by a nonzero padding, and just outside
// the bare arm, is cut out where the robot stands, its root away from the
// map's origin and turned: it marks nothing, its voxel takes the lowest
// log-odds, and its ray clears. The bare arm leaves the same point a hit.
#[hegel::test]
fn the_padded_robot_is_cut_out_where_it_stands(tc: TestCase) {
    let side = if tc.draw(gs::booleans()) { LEFT } else { RIGHT };
    let arm = &OPENARM_V2_ARMS[side];
    let q = arm
        .chain()
        .limits()
        .map(|limit| tc.draw(gs::floats().min_value(limit.lo).max_value(limit.hi)));
    let padding = Padding::new(
        tc.draw(gs::floats().min_value(1.0).max_value(1.2)),
        tc.draw(gs::floats().min_value(0.02).max_value(0.05)),
    )
    .expect("a valid padding");
    // A tenth of a millimeter past a drawn vertex of a drawn piece, straight
    // out from the piece's center: outside the bare piece.
    let placed = arm.place(&q);
    let pieces: Vec<Vec<Point3<f64>>> = placed
        .pieces()
        .map(|(_, piece)| piece.vertices().collect())
        .collect();
    let piece = &pieces[tc.draw(gs::integers::<usize>().max_value(pieces.len() - 1))];
    let vertex = piece[tc.draw(gs::integers::<usize>().max_value(piece.len() - 1))];
    let center = Point3::from(
        piece.iter().fold(Vector3::zeros(), |sum, v| sum + v.coords) / piece.len() as f64,
    );
    let in_root = vertex + (vertex - center).normalize() * 1e-4;
    let none = Padding::new(1.0, 0.0).expect("the padding that grows nothing");
    let (padded, bare) = (arm.mask(padding), arm.mask(none));
    let (padded, bare) = (padded.at(&q), bare.at(&q));
    tc.assume(padded.contains(&in_root) && !bare.contains(&in_root));

    let root = Isometry3::from_parts(
        Translation3::new(0.4, -0.3, -0.6),
        UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
        ),
    );
    let in_map = root * in_root;
    // A sensor half a meter from the point, toward the map's middle.
    let sensor = Isometry3::from_parts(
        Translation3::from(in_map.coords - in_map.coords.normalize() * 0.5),
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4),
    );
    let reading: [f32; 3] = (sensor.inverse() * in_map).coords.map(|c| c as f32).into();
    let (frame, sensor, root) = (frame(), pose_of(&sensor), pose_of(&root));
    let key = map().key(in_map.coords.into()).expect("in the map");

    let mut cut = map();
    insert(
        &mut cut,
        sensor,
        &[reading],
        &[&RobotCutout::new(&frame, root, vec![padded])],
    );
    assert_eq!(cut.occupied().count(), 0, "the robot marked a voxel");
    assert_eq!(log_odds(&cut, key), Some(lowest()));
    assert!(!free_keys(&cut).is_empty(), "the ray did not clear");

    let mut bare_arm = map();
    insert(
        &mut bare_arm,
        sensor,
        &[reading],
        &[&RobotCutout::new(&frame, root, vec![bare])],
    );
    assert!(bare_arm.is_occupied(key), "the bare arm cut out the point");
}

/// A robot whose body is a box 20 cm on a side about its root.
const BOX_BODY: &str = r#"<robot name="box_body">
    <link name="base">
      <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
    </link>
  </robot>"#;

/// A robot whose body is a box 20 cm on a side 1 m along x from its root.
const FAR_BOX_BODY: &str = r#"<robot name="far_box_body">
    <link name="base">
      <collision><origin xyz="1 0 0"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
    </link>
  </robot>"#;

// A point inside the body fixed around the robot's chains, the root away from
// the map's origin, is cut out of a scan by the robot's parts, the first of
// which, a box 1 m away, does not hold it: it marks nothing and its voxel
// takes the lowest log-odds. The same scan with nothing cut out marks it.
#[test]
fn a_point_inside_the_robots_body_is_cut_out() {
    let body = BodyGeometry::from_urdf(BOX_BODY, "").expect("a box body");
    let far = BodyGeometry::from_urdf(FAR_BOX_BODY, "").expect("a box body");
    let none = Padding::new(1.0, 0.0).expect("the padding that grows nothing");
    let (grown, far) = (body.mask(none), far.mask(none));
    let root = Isometry3::translation(0.4, -0.3, -0.6);
    let in_map = root * Point3::new(0.05, -0.02, 0.03);
    // A sensor half a meter from the point, along the map's x axis.
    let sensor = Isometry3::translation(in_map.x - 0.5, in_map.y, in_map.z);
    let reading: [f32; 3] = (sensor.inverse() * in_map).coords.map(|c| c as f32).into();
    let (frame, sensor, root) = (frame(), pose_of(&sensor), pose_of(&root));
    let key = map().key(in_map.coords.into()).expect("in the map");

    let mut cut = map();
    insert(
        &mut cut,
        sensor,
        &[reading],
        &[&RobotCutout::new(
            &frame,
            root,
            vec![far.placed(), grown.placed()],
        )],
    );
    assert_eq!(cut.occupied().count(), 0, "the body marked a voxel");
    assert_eq!(log_odds(&cut, key), Some(lowest()));

    let mut marked = map();
    insert(&mut marked, sensor, &[reading], &[]);
    assert!(
        marked.is_occupied(key),
        "a scan with nothing cut out cut the point"
    );
}

// A map takes a scan whose sensor stands in its own frame and cutouts in its
// own frame, and refuses any other, changing nothing.
#[test]
fn a_map_refuses_a_scan_or_a_cutout_in_another_frame() {
    let mut map = map();
    let (ours, other) = (frame(), FrameName::parse("camera").expect("a frame name"));
    let points = [[0.0_f32, 0.0, 1.0]];
    let scan = |frame| Scan {
        frame,
        sensor: identity(),
        points: &points,
        max_range_m: range(RANGE_M),
    };
    let refused = map.insert(&scan(&other), &[]).unwrap_err();
    assert_eq!(
        refused,
        InsertError::ScanFrame {
            scan: other.clone(),
            map: ours.clone()
        }
    );
    assert_eq!(
        refused.to_string(),
        "the scan's sensor stands in frame \"camera\" and the map is in \"map\": give the \
         sensor's pose in \"map\""
    );
    let ball = Ball {
        frame: other.clone(),
        center: [0.0; 3],
        radius: 0.1,
    };
    assert_eq!(
        map.insert(&scan(&ours), &[&ball]),
        Err(InsertError::CutoutFrame {
            cutout: other.clone(),
            map: ours.clone()
        })
    );
    assert_eq!(map.known().count(), 0);
    map.insert(&scan(&ours), &[]).expect("the map's frame");
    assert!(map.known().count() > 0);
}

/// The points a sensor at the origin, looking along +Z, reads of a wall at
/// `z_m` across `extent_m`, one per `step_m`.
fn wall(z_m: f32, extent_m: f32, step_m: f32) -> Vec<[f32; 3]> {
    let n = (extent_m / step_m) as i32;
    (-n..=n)
        .flat_map(|i| (-n..=n).map(move |j| [i as f32 * step_m, j as f32 * step_m, z_m]))
        .collect()
}

/// The voxels of `map` the segment from `from` to `to` passes through, found by
/// stepping along it a hundredth of a voxel at a time and keeping the steps
/// that lie farther than a thousandth of a voxel from every face.
fn passed_through(map: &VoxelMap, from: [f64; 3], to: [f64; 3]) -> BTreeSet<Key> {
    let length = (0..3)
        .map(|i| (to[i] - from[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    let steps = (length / (VOXEL_M / 100.0)).ceil() as usize;
    (0..=steps)
        .map(|step| {
            let t = step as f64 / steps as f64;
            std::array::from_fn(|i| from[i] + t * (to[i] - from[i]))
        })
        .filter(|p: &[f64; 3]| {
            p.iter().all(|c| {
                let within = (c / VOXEL_M).rem_euclid(1.0);
                within > 1e-3 && within < 1.0 - 1e-3
            })
        })
        .map(|p| map.key(p).expect("in the map"))
        .collect()
}

// Repeated scans of a still wall make every voxel of its surface occupied and
// every voxel in front of it that a ray passes through free; one wrong far
// reading through an occupied voxel leaves it occupied.
#[test]
fn a_still_scene_settles_and_a_stray_reading_does_not_undo_it() {
    let mut map = map();
    // Off the voxels' faces, so no ray starts on one.
    let origin = [0.012, -0.013, 0.007];
    let sensor = Transform::new(origin, [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let points = wall(1.0, 0.3, 0.02);
    for _ in 0..10 {
        insert(&mut map, sensor, &points, &[]);
    }
    let surface: BTreeSet<Key> = points.iter().map(|p| key_of(&map, &sensor, *p)).collect();
    for key in &surface {
        assert!(
            map.is_occupied(*key),
            "surface voxel {key:?} is not occupied"
        );
    }
    // The rays run from the sensor to the centers of the voxels they hit.
    let crossed: BTreeSet<Key> = surface
        .iter()
        .flat_map(|key| passed_through(&map, origin, map.center(*key)))
        .filter(|key| !surface.contains(key))
        .collect();
    assert!(
        crossed.len() > surface.len(),
        "only {} voxels in front of the wall",
        crossed.len()
    );
    for key in &crossed {
        assert!(
            matches!(map.voxel(*key), Voxel::Free(_)),
            "{key:?} in front of the wall reads {:?}",
            map.voxel(*key)
        );
    }
    insert(&mut map, sensor, &[[0.0, 0.0, 1.5]], &[]);
    let through = key_of(&map, &sensor, [0.0, 0.0, 1.0]);
    assert!(map.is_occupied(through), "one stray reading undid the wall");
}

/// The distance from `q` to the nearest of `centers`, by brute force.
fn brute_force(centers: &[[f64; 3]], q: [f64; 3]) -> f64 {
    centers
        .iter()
        .map(|c| (0..3).map(|i| (q[i] - c[i]).powi(2)).sum::<f64>().sqrt())
        .fold(f64::INFINITY, f64::min)
}

/// A map holding the hits of drawn points from a sensor at the origin.
fn drawn_map(tc: &TestCase) -> VoxelMap {
    let points: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(12)))
        .map(|_| point(tc, 0.3, 1.5))
        .collect();
    let mut map = map();
    insert(&mut map, identity(), &points, &[]);
    map
}

/// The centers of a map's occupied voxels.
fn occupied_centers(map: &VoxelMap) -> Vec<[f64; 3]> {
    map.occupied().map(|key| map.center(key)).collect()
}

// On drawn maps, the distance from any point of the map to the nearest
// occupied voxel agrees with a brute-force distance to the centers of the
// occupied voxels within half a voxel's diagonal, the farthest a point lies
// from its voxel's center.
#[hegel::test]
fn inside_the_map_the_distance_agrees_with_brute_force(tc: TestCase) {
    let map = drawn_map(&tc);
    let centers = occupied_centers(&map);
    let field = DistanceField::of(&map);
    let half_diagonal = VOXEL_M * 3.0_f64.sqrt() / 2.0;
    for _ in 0..20 {
        let q: [f64; 3] = tc.draw(gs::arrays(
            gs::floats().min_value(-EDGE_M).max_value(EDGE_M),
        ));
        let (d, brute) = (
            field.distance(q).expect("a point"),
            brute_force(&centers, q),
        );
        assert!(
            (d - brute).abs() <= half_diagonal + ROUNDING_M,
            "{d} against brute force {brute} at {q:?}"
        );
    }
}

/// A point outside the map: on each axis anywhere up to three meters past
/// the map's bounds, and past them on at least one.
fn outside(tc: &TestCase) -> [f64; 3] {
    let far = EDGE_M + 3.0;
    let mut q: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-far).max_value(far)));
    let axis = tc.draw(gs::integers::<usize>().max_value(2));
    let past: f64 = tc.draw(gs::floats().min_value(VOXEL_M).max_value(3.0));
    // The voxel holding the bounds' maximum reaches a voxel past it.
    q[axis] = if tc.draw(gs::booleans()) {
        EDGE_M + VOXEL_M + past
    } else {
        -EDGE_M - past
    };
    q
}

// Outside the map the distance never exceeds the brute-force distance.
#[hegel::test]
fn outside_the_map_the_distance_is_a_lower_bound(tc: TestCase) {
    let map = drawn_map(&tc);
    let centers = occupied_centers(&map);
    let field = DistanceField::of(&map);
    for _ in 0..20 {
        let q = outside(&tc);
        assert_eq!(map.key(q), None, "{q:?} lies in the map");
        let (d, brute) = (
            field.distance(q).expect("a point"),
            brute_force(&centers, q),
        );
        assert!(
            d <= brute + ROUNDING_M,
            "{d} exceeds brute force {brute} at {q:?}"
        );
    }
}

// With every occupied voxel on the map's top face, the distance from a point
// straight above the face is a lower bound wherever on the face it stands, and
// the distance itself straight above a voxel center.
#[hegel::test]
fn straight_out_from_the_face_the_bound_is_the_distance(tc: TestCase) {
    let mut map = map();
    let top = map.key([0.0, 0.0, EDGE_M]).expect("in the map").0[2];
    // Within a meter of the middle on x and y, so every hit is in range.
    let (first, last) = (-20, 19);
    let on_face = |tc: &TestCase| {
        let [x, y]: [i32; 2] = tc.draw(gs::arrays(gs::integers().min_value(first).max_value(last)));
        Key([x, y, top])
    };
    let hits: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(6)))
        .map(|_| map.center(on_face(&tc)).map(|c| c as f32))
        .collect();
    insert(&mut map, identity(), &hits, &[]);
    let centers = occupied_centers(&map);
    assert!(!centers.is_empty());
    assert!(
        map.occupied().all(|key| key.0[2] == top),
        "a voxel off the face is occupied"
    );
    let field = DistanceField::of(&map);
    let above = |[x, y, z]: [f64; 3]| {
        let q = [
            x,
            y,
            z + tc.draw(gs::floats().min_value(VOXEL_M).max_value(3.0)),
        ];
        assert_eq!(map.key(q), None);
        (
            field.distance(q).expect("a point"),
            brute_force(&centers, q),
            q,
        )
    };
    let (d, brute, q) = above(map.center(on_face(&tc)));
    assert!(
        (d - brute).abs() <= ROUNDING_M,
        "{d} against brute force {brute} at {q:?}"
    );
    let span = |i: usize| {
        let (low, high) = (
            map.center(Key([first; 3]))[i],
            map.center(Key([last; 3]))[i],
        );
        tc.draw(gs::floats().min_value(low).max_value(high))
    };
    let (d, brute, q) = above([span(0), span(1), map.center(Key([0, 0, top]))[2]]);
    assert!(
        d <= brute + ROUNDING_M,
        "{d} exceeds brute force {brute} at {q:?}"
    );
}

#[test]
fn the_distance_at_a_point_that_is_not_a_number_is_refused() {
    let field = DistanceField::of(&map());
    for q in [
        [f64::NAN, 0.0, 0.0],
        [0.0, 0.0, f64::NAN],
        [f64::NAN, 5.0, 5.0],
    ] {
        assert!(matches!(field.distance(q), Err(NotANumber(_))), "{q:?}");
    }
    assert_eq!(field.distance([5.0, 0.0, 0.0]), Ok(f64::INFINITY));
}

fn cap(meters: f64) -> PositiveFinite {
    PositiveFinite::parse(meters).expect("a positive cap")
}

/// The voxel `steps` voxels from `key` along `axis`.
fn stepped(key: Key, axis: usize, steps: i32) -> Key {
    Key(std::array::from_fn(|i| {
        key.0[i] + if i == axis { steps } else { 0 }
    }))
}

// On drawn maps, at a point whose voxel has its neighbors on every axis, the
// gradient is the central difference of the brute-force distances at the
// neighbors' centers, each capped, and the distance the capped brute-force
// distance at the voxel's center.
#[hegel::test]
fn the_gradient_is_the_central_difference_of_the_capped_distances(tc: TestCase) {
    let map = drawn_map(&tc);
    let centers = occupied_centers(&map);
    let field = DistanceField::of(&map);
    let limit = tc.draw(gs::floats().min_value(VOXEL_M).max_value(3.0));
    let whole_voxels = (limit / VOXEL_M).ceil() * VOXEL_M;
    let capped = |key: Key| brute_force(&centers, map.center(key)).min(whole_voxels);
    for _ in 0..20 {
        let q: [f64; 3] = tc.draw(gs::arrays(
            gs::floats()
                .min_value(-EDGE_M + 2.0 * VOXEL_M)
                .max_value(EDGE_M - 2.0 * VOXEL_M),
        ));
        let key = map.key(q).expect("in the map");
        let at = field
            .gradient(q, cap(limit))
            .expect("a point")
            .expect("a voxel with neighbors on every axis");
        assert!(
            (at.distance - capped(key)).abs() <= ROUNDING_M,
            "{at:?} at {q:?}"
        );
        for axis in 0..3 {
            let want =
                (capped(stepped(key, axis, 1)) - capped(stepped(key, axis, -1))) / (2.0 * VOXEL_M);
            assert!(
                (at.gradient[axis] - want).abs() <= ROUNDING_M / VOXEL_M,
                "axis {axis}: {at:?} against {want} at {q:?}"
            );
        }
    }
}

// Three voxels from the one occupied voxel along x, the distance falls by one voxel
// toward it and rises by one away from it: a gradient of 1 along x, and 0 across it,
// where the neighbors stand alike. A cap of two and a half voxels rounds up to three, as
// MoveIt's propagation field rounds it, which stops the distance ahead there at three
// voxels: the gradient along x falls to 0.5, and the distance stays three voxels.
#[test]
fn a_cap_flattens_the_gradient_where_a_neighbor_reaches_it() {
    let mut map = map();
    let hit = map.key([1.0, 0.0, 0.0]).expect("in the map");
    let point = map.center(hit).map(|c| c as f32);
    insert(&mut map, identity(), &[point], &[]);
    assert_eq!(map.occupied().collect::<Vec<_>>(), vec![hit]);
    let field = DistanceField::of(&map);
    let q = map.center(stepped(hit, 0, 3));
    let at = |limit: f64| {
        field
            .gradient(q, cap(limit))
            .expect("a point")
            .expect("inside the map")
    };
    let uncapped = at(10.0);
    assert!(
        (uncapped.distance - 3.0 * VOXEL_M).abs() <= ROUNDING_M,
        "{uncapped:?}"
    );
    for (axis, want) in [1.0, 0.0, 0.0].into_iter().enumerate() {
        assert!(
            (uncapped.gradient[axis] - want).abs() <= 1e-12,
            "{uncapped:?}"
        );
    }
    let capped = at(2.5 * VOXEL_M);
    assert!(
        (capped.distance - 3.0 * VOXEL_M).abs() <= ROUNDING_M,
        "{capped:?}"
    );
    assert!((capped.gradient[0] - 0.5).abs() <= 1e-12, "{capped:?}");
}

// A map with nothing occupied reads the cap, rounded up to whole voxels, everywhere,
// with no slope.
#[test]
fn a_map_with_nothing_occupied_reads_the_cap_and_no_slope() {
    let field = DistanceField::of(&map());
    let at = field
        .gradient([0.3, -0.2, 0.1], cap(0.27))
        .expect("a point")
        .expect("inside the map");
    assert!((at.distance - 6.0 * VOXEL_M).abs() <= ROUNDING_M, "{at:?}");
    assert_eq!(at.gradient, [0.0; 3]);
}

// Outside the map, and on its outermost voxels, which lack a neighbor on one
// side, there is no gradient, where MoveIt reports the point out of bounds.
#[hegel::test]
fn outside_the_map_and_on_its_edge_there_is_no_gradient(tc: TestCase) {
    let map = drawn_map(&tc);
    let field = DistanceField::of(&map);
    let q = outside(&tc);
    assert_eq!(field.gradient(q, cap(0.25)), Ok(None), "{q:?}");
    let axis = tc.draw(gs::integers::<usize>().max_value(2));
    let edge = if tc.draw(gs::booleans()) {
        EDGE_M + VOXEL_M / 2.0
    } else {
        -EDGE_M + VOXEL_M / 2.0
    };
    let mut q = [0.0; 3];
    q[axis] = edge;
    let key = map.key(q).expect("the edge voxel lies in the map");
    let outward = if edge > 0.0 { 1 } else { -1 };
    assert_eq!(
        map.key(map.center(stepped(key, axis, outward))),
        None,
        "{q:?} lies on an outermost voxel"
    );
    assert_eq!(field.gradient(q, cap(0.25)), Ok(None), "{q:?}");
}

#[test]
fn the_gradient_at_a_point_that_is_not_a_number_is_refused() {
    let field = DistanceField::of(&map());
    let q = [0.0, f64::NAN, 0.0];
    assert!(matches!(field.gradient(q, cap(0.25)), Err(NotANumber(_))));
}

/// The camera of the rendered frames: an ideal pinhole, 160 by 90 pixels.
const BOX_CAMERA: Intrinsics<'static> = Intrinsics {
    width: 160,
    height: 90,
    fx: 92.0,
    fy: 92.0,
    cx: 79.5,
    cy: 44.5,
    distortion_model: "none",
    distortion: &[],
};

/// Meters per count of the rendered frames.
const BOX_UNIT: f32 = 1e-4;

/// Where the box from `min` to `max`, in the map's frame, meets the ray of
/// each pixel of a depth camera at `camera` in the map's frame, row by row:
/// the point it meets first and its depth along the camera's axis, from the
/// pixel's ray worked out here.
fn box_hits(camera: &Isometry3<f64>, min: [f64; 3], max: [f64; 3]) -> Vec<Option<([f64; 3], f64)>> {
    let Intrinsics { fx, fy, cx, cy, .. } = BOX_CAMERA;
    (0..BOX_CAMERA.height)
        .flat_map(|v| (0..BOX_CAMERA.width).map(move |u| (u, v)))
        .map(|(u, v)| {
            // The pixel's ray at unit depth: the distance along it is the
            // depth along the axis.
            let ray = Vector3::new((f64::from(u) - cx) / fx, (f64::from(v) - cy) / fy, 1.0);
            let (origin, direction) = (camera.translation.vector, camera.rotation * ray);
            box_entry(origin.into(), direction.into(), min, max)
                .map(|near| ((origin + direction * near).into(), near))
        })
        .collect()
}

// A rendered depth frame of a box off the camera's axis, from a depth imager
// that sits beside the color one and turned against it, marks every voxel of
// the faces the camera sees and no other: the depth frame stands where the
// color frame's pose composed with the depth-to-color pose puts it. The
// inside of the box stays unknown.
#[test]
fn a_rendered_box_marks_the_faces_the_camera_sees() {
    // The color camera above the map's middle, looking down and to +X.
    let color_in_map = Isometry3::from_parts(
        Translation3::new(0.1, -0.3, 1.2),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI - 0.35),
    );
    let depth_in_color = Isometry3::from_parts(
        Translation3::new(-0.063, 0.002, 0.004),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.08),
    );
    // A box whose faces and edges lie on voxel centers, so a depth rounded to
    // a count keeps each point in its voxel across the face.
    let (min, max) = ([0.075, 0.075, 0.025], [0.425, 0.375, 0.325]);
    let hits = box_hits(&(color_in_map * depth_in_color), min, max);
    let frame: Vec<u8> = hits
        .iter()
        .flat_map(|hit| {
            let sample = hit.map_or(0, |(_, depth)| (depth / f64::from(BOX_UNIT)).round() as u16);
            sample.to_le_bytes()
        })
        .collect();

    let camera = Pinhole::parse(&BOX_CAMERA).expect("a valid model");
    let points = depth_points(&frame, &camera, BOX_UNIT, DepthModel::Z).expect("a valid frame");
    let sensor = pose_of(&color_in_map).compose(&pose_of(&depth_in_color));
    let mut map = map();
    for _ in 0..5 {
        insert(&mut map, sensor, &points, &[]);
    }
    let seen: BTreeSet<Key> = hits
        .iter()
        .flatten()
        .map(|(point, _)| map.key(*point).expect("in the map"))
        .collect();
    assert!(
        seen.len() > 50,
        "the camera sees {} voxels of the box",
        seen.len()
    );
    let marked: BTreeSet<Key> = map.occupied().collect();
    assert_eq!(marked, seen, "the marked voxels are not the faces seen");
    let inside = map.key([0.275, 0.225, 0.175]).expect("in the map");
    assert_eq!(
        map.voxel(inside),
        Voxel::Unknown,
        "the inside of the box was seen"
    );
}

/// Whether the box from `min` to `max` holds the center of voxel `key`.
fn holds_center(map: &VoxelMap, min: [f64; 3], max: [f64; 3], key: Key) -> bool {
    let center = map.center(key);
    (0..3).all(|i| center[i] >= min[i] && center[i] <= max[i])
}

// The occupied voxels inside a box are the occupied voxels whose centers it
// holds, for a box inside the map, across its edge, or reaching past the
// voxel tree.
#[hegel::test]
fn the_occupied_voxels_in_a_box_are_those_whose_centers_it_holds(tc: TestCase) {
    let sensor = sensor(&tc);
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(30));
    let points: Vec<[f32; 3]> = (0..count).map(|_| point(&tc, 0.2, 1.4)).collect();
    let mut map = map();
    insert(&mut map, sensor, &points, &[]);
    let corner = |tc: &TestCase| -> [f64; 3] {
        tc.draw(gs::arrays(gs::floats().min_value(-3.0).max_value(3.0)))
    };
    let (a, b) = (corner(&tc), corner(&tc));
    let min: [f64; 3] = std::array::from_fn(|i| a[i].min(b[i]));
    let max: [f64; 3] = std::array::from_fn(|i| a[i].max(b[i]));
    let max = if tc.draw(gs::booleans()) {
        max.map(|c| c + 1000.0)
    } else {
        max
    };
    let mut found = map.occupied_in(min, max);
    found.sort();
    let want: Vec<Key> = map
        .occupied()
        .filter(|key| holds_center(&map, min, max, *key))
        .collect();
    assert_eq!(found, want);
}

// A cleared map knows no voxel, and a scan after it marks the map as it marks
// an empty one.
#[test]
fn a_cleared_map_is_an_empty_one() {
    let points = wall(1.0, 0.5, 0.02);
    let mut cleared = map();
    insert(&mut cleared, identity(), &points, &[]);
    insert(&mut cleared, identity(), &points, &[]);
    cleared.clear();
    assert_eq!(cleared.known().count(), 0);
    insert(&mut cleared, identity(), &points, &[]);
    let mut empty = map();
    insert(&mut empty, identity(), &points, &[]);
    assert!(!voxels(&empty).is_empty());
    assert_eq!(voxels(&cleared), voxels(&empty));
}

/// The shape of a collision object: a box, a sphere or a cylinder, one to eight voxels
/// across.
fn object_shape(tc: &TestCase) -> Shape {
    let length = || gs::floats().min_value(VOXEL_M).max_value(8.0 * VOXEL_M);
    match tc.draw(gs::integers::<usize>().min_value(0).max_value(2)) {
        0 => Shape::cuboid(tc.draw(gs::arrays(length()))),
        1 => Shape::sphere(tc.draw(length()) / 2.0),
        _ => Shape::cylinder(tc.draw(length()) / 2.0, tc.draw(length())),
    }
    .expect("a shape of lengths above 0")
}

/// A rotation drawn from its Euler angles.
fn turned(tc: &TestCase) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(
        tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
        tc.draw(gs::floats().min_value(-1.5).max_value(1.5)),
        tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
    )
}

// An object placed over a scanned cloud clears every voxel whose center it holds, each
// to the lowest log-odds, and leaves every voxel whose box lies wholly beyond it as it
// was.
#[hegel::test]
fn an_object_clears_the_voxels_it_intersects(tc: TestCase) {
    let sensor = sensor(&tc);
    let points: Vec<[f32; 3]> = (0..60).map(|_| point(&tc, 0.2, 1.5)).collect();
    let mut map = map();
    insert(&mut map, sensor, &points, &[]);
    let occupied: Vec<Key> = map.occupied().collect();
    tc.assume(!occupied.is_empty());
    let at = map.center(
        occupied[tc.draw(
            gs::integers::<usize>()
                .min_value(0)
                .max_value(occupied.len() - 1),
        )],
    );
    let shape = object_shape(&tc);
    let placed = PlacedShape::new(
        &shape,
        Isometry3::from_parts(Translation3::new(at[0], at[1], at[2]), turned(&tc)),
    );
    let before: Vec<(Key, Voxel)> = map.known().map(|key| (key, map.voxel(key))).collect();
    map.clear_intersecting(&placed);
    let at_key = map.key(at).expect("inside the map");
    assert_eq!(log_odds(&map, at_key), Some(lowest()));
    let half_diagonal = VOXEL_M * 3.0_f64.sqrt() / 2.0;
    for (key, voxel) in before {
        let center = Point3::from(map.center(key));
        let distance = placed.distance_to(&Vertex(center), (center, 0.0));
        if distance <= 0.0 {
            assert_eq!(
                log_odds(&map, key),
                Some(lowest()),
                "{key:?} stays as it was inside the object"
            );
        } else if distance > half_diagonal {
            assert_eq!(map.voxel(key), voxel, "{key:?} beyond the object changed");
        }
    }
}

/// OctoMap's upper bound of a voxel's log-odds, the log-odds of 0.971.
fn highest() -> f32 {
    (0.971_f64 / (1.0 - 0.971)).ln() as f32
}

// An object occupies the voxels it clears: the voxels that clear_intersecting reaches on
// an empty map, occupy_intersecting takes to the highest log-odds on a map a scan has
// filled, and it leaves every other voxel as it was; the distance field of the map then
// reads 0 at the object's center.
#[hegel::test]
fn an_object_occupies_the_voxels_it_clears(tc: TestCase) {
    let sensor = sensor(&tc);
    let points: Vec<[f32; 3]> = (0..60).map(|_| point(&tc, 0.2, 1.5)).collect();
    let mut scanned = map();
    insert(&mut scanned, sensor, &points, &[]);
    let at: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)));
    let shape = object_shape(&tc);
    let placed = PlacedShape::new(
        &shape,
        Isometry3::from_parts(Translation3::new(at[0], at[1], at[2]), turned(&tc)),
    );
    let mut cleared = map();
    cleared.clear_intersecting(&placed);
    let touched: BTreeSet<Key> = cleared.known().collect();
    let center = cleared.key(at).expect("inside the map");
    assert!(
        touched.contains(&center),
        "the object holds its center's voxel"
    );
    let mut occupied = scanned.clone();
    occupied.occupy_intersecting(&placed);
    let keys: BTreeSet<Key> = occupied.known().chain(scanned.known()).collect();
    for key in keys {
        let want = if touched.contains(&key) {
            Some(highest())
        } else {
            log_odds(&scanned, key)
        };
        assert_eq!(log_odds(&occupied, key), want, "{key:?}");
    }
    assert_eq!(DistanceField::of(&occupied).distance(at), Ok(0.0));
}

// A point inside a collision object grown by its padding marks nothing, and a point
// beyond the padding is a hit: the band a ball's padding cuts reaches its radius times
// the scale less 1, and the offset farther.
#[hegel::test]
fn a_point_inside_an_object_marks_nothing(tc: TestCase) {
    let sensor = sensor(&tc);
    let p = point(&tc, 0.3, 1.0);
    let at = Vector3::from(sensor.apply(p.map(f64::from)));
    let radius: f64 = tc.draw(gs::floats().min_value(VOXEL_M).max_value(0.2));
    let scale: f64 = tc.draw(gs::floats().min_value(1.0).max_value(1.5));
    let offset: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.05));
    let band = radius * (scale - 1.0) + offset;
    let gap: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.2));
    tc.assume((gap - band).abs() > 1e-6);
    let ball = Shape::sphere(radius).expect("a sphere");
    let center = at + direction(&tc) * (radius + gap);
    let frame = frame();
    let object = ShapeCutout {
        frame: &frame,
        shape: PlacedShape::new(&ball, Isometry3::translation(center.x, center.y, center.z)),
        padding: Padding::new(scale, offset).expect("a scale above 0 and an offset of 0 or more"),
    };
    let mut map = map();
    insert(&mut map, sensor, &[p], &[&object]);
    assert_eq!(
        map.occupied().count() == 0,
        gap < band,
        "a point {gap} m from the object, whose padding cuts a band of {band} m"
    );
}

/// The voxels `map` knows.
fn known(map: &VoxelMap) -> BTreeSet<Key> {
    map.known().collect()
}

/// The voxels from `lo` to `hi`, both included.
fn keys(lo: [i32; 3], hi: [i32; 3]) -> BTreeSet<Key> {
    (lo[0]..=hi[0])
        .flat_map(|x| {
            (lo[1]..=hi[1]).flat_map(move |y| (lo[2]..=hi[2]).map(move |z| Key([x, y, z])))
        })
        .collect()
}

// A box whose faces lie on voxel faces clears the voxels it fills and those whose faces,
// edges or corners touch its own, and no other, as a box and as a mesh: voxels of 12.5 cm
// and a box from 12.5 to 37.5 cm on each axis, and voxels of 10 cm and a box from 10 to
// 20 cm, whose lower faces the sum of its center and half its size puts a rounding above
// 10 cm.
#[test]
fn an_object_clears_the_voxels_whose_faces_touch_it() {
    for (voxel_m, first, last) in [(0.125, 1, 2), (0.1, 1, 1)] {
        let size = f64::from(last - first + 1) * voxel_m;
        let center = f64::from(first + last + 1) / 2.0 * voxel_m;
        let shapes = [
            ("a box", Shape::cuboid([size; 3]).expect("a box")),
            ("a box mesh", meshes::box_mesh([size; 3])),
        ];
        for (form, shape) in shapes {
            let mut map = map_of(voxel_m);
            map.clear_intersecting(&PlacedShape::new(
                &shape,
                Isometry3::translation(center, center, center),
            ));
            assert_eq!(
                known(&map),
                keys([first - 1; 3], [last + 1; 3]),
                "{form} in {voxel_m} m voxels"
            );
            assert!(map.known().all(|key| log_odds(&map, key) == Some(lowest())));
        }
    }
}

// A box of 10 cm about the corner the voxels of 12.5 cm from 12.5 to 37.5 cm share
// covers each of those eight voxels in part, none of their centers among it: it clears
// the eight, and no other.
#[test]
fn an_object_clears_the_voxels_it_covers_in_part() {
    let mut map = map_of(0.125);
    let cube = Shape::cuboid([0.1; 3]).expect("a box");
    map.clear_intersecting(&PlacedShape::new(
        &cube,
        Isometry3::translation(0.25, 0.25, 0.25),
    ));
    assert_eq!(known(&map), keys([1; 3], [2; 3]));
}

/// A closed mesh one to twelve voxels across: a sphere whose vertices lie at drawn
/// radii, an L, or a box with a box hollowed out of it.
fn drawn_mesh(tc: &TestCase) -> Shape {
    let across = || gs::floats().min_value(VOXEL_M).max_value(12.0 * VOXEL_M);
    match tc.draw(gs::integers::<usize>().min_value(0).max_value(2)) {
        0 => {
            let rings = tc.draw(gs::integers::<usize>().min_value(1).max_value(12));
            let around = tc.draw(gs::integers::<usize>().min_value(3).max_value(16));
            let radius: f64 = tc.draw(across()) / 2.0;
            meshes::sphere(rings, around, || {
                tc.draw(gs::floats().min_value(radius / 2.0).max_value(radius))
            })
        }
        1 => meshes::l_mesh(tc.draw(across())),
        _ => meshes::hollow_box(
            tc.draw(gs::arrays(across())),
            tc.draw(gs::floats().min_value(0.2).max_value(0.8)),
        ),
    }
}

/// The voxels of `map` that `shape` touches, each measured on its own: every voxel of the
/// map within a voxel of the shape's bounds.
fn touched_one_by_one(map: &VoxelMap, shape: &PlacedShape) -> BTreeSet<Key> {
    let (min, max) = shape.bounds();
    let index = |c: f64| (c / map.voxel_size()).floor() as i32;
    let lo = std::array::from_fn(|i| index(min[i]) - 1);
    let hi = std::array::from_fn(|i| index(max[i]) + 1);
    keys(lo, hi)
        .into_iter()
        .filter(|key| map.key(map.center(*key)) == Some(*key) && map.touches(shape, *key))
        .collect()
}

// A mesh clears the voxels it touches, as each measured on its own finds them: those its
// triangles cross or touch and those it holds whole, and no voxel of an L's open corner or
// of a hollow box's hollow; on drawn meshes, turned any way, standing anywhere in the map
// or across its faces, edges and corners.
#[hegel::test]
fn a_mesh_clears_the_voxels_each_one_measured_alone_finds(tc: TestCase) {
    let shape = drawn_mesh(&tc);
    let reach = EDGE_M + 6.0 * VOXEL_M;
    let center: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-reach).max_value(reach)));
    let placed = PlacedShape::new(
        &shape,
        Isometry3::from_parts(
            Translation3::new(center[0], center[1], center[2]),
            turned(&tc),
        ),
    );
    let mut map = map();
    map.clear_intersecting(&placed);
    assert_eq!(known(&map), touched_one_by_one(&map, &placed));
}

/// The column of `columns` that holds the center of voxel `key` of `map`, by the
/// column's corner and side.
fn column_of(map: &VoxelMap, columns: Columns, key: Key) -> [u32; 2] {
    let center = map.center(key);
    std::array::from_fn(|i| {
        let offset = (center[i] - columns.origin[i]) / columns.side_m;
        assert!(offset > 0.0, "voxel {key:?} lies below the columns' corner");
        offset as u32
    })
}

// Every voxel's center lies inside the column of its x and y keys, and the columns
// cover the map's extent; clearing drawn columns of a drawn scene takes every voxel of
// them to one miss's log-odds, unknown ones included, and leaves every other voxel as
// it was.
#[hegel::test(test_cases = 50)]
fn clearing_columns_frees_their_voxels_and_no_other(tc: TestCase) {
    let mut map = map();
    let sensor = sensor(&tc);
    let points: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(40)))
        .map(|_| point(&tc, 0.2, 1.4))
        .collect();
    insert(&mut map, sensor, &points, &[]);
    assert_eq!(map.frame(), &frame());
    let columns = map.columns();
    let [lowest_key, highest_key] = [[-EDGE_M; 3], [EDGE_M; 3]].map(|corner| {
        map.key(corner)
            .expect("the map holds the corners of its bounds")
    });
    let keys_on = |axis: usize| (highest_key.0[axis] - lowest_key.0[axis] + 1) as u32;
    assert_eq!((columns.size_x, columns.size_y), (keys_on(0), keys_on(1)));
    assert_eq!(columns.side_m, VOXEL_M);
    let cleared: BTreeSet<[u32; 2]> = (0..tc.draw(gs::integers::<usize>().max_value(30)))
        .map(|_| {
            [
                tc.draw(gs::integers::<u32>().max_value(columns.size_x - 1)),
                tc.draw(gs::integers::<u32>().max_value(columns.size_y - 1)),
            ]
        })
        .collect();
    let before = voxels(&map);
    map.clear_columns(cleared.iter().copied());
    for (key, voxel) in &before {
        if !cleared.contains(&column_of(&map, columns, *key)) {
            assert_eq!(
                &map.voxel(*key),
                voxel,
                "voxel {key:?} outside the cleared columns"
            );
        }
    }
    for &[x, y] in &cleared {
        let low = map.key([
            columns.origin[0] + (f64::from(x) + 0.5) * VOXEL_M,
            columns.origin[1] + (f64::from(y) + 0.5) * VOXEL_M,
            -EDGE_M,
        ]);
        let low = low.expect("a column's bottom voxel lies in the map");
        assert_eq!(column_of(&map, columns, low), [x, y]);
        for z in 0..keys_on(2) as i32 {
            let key = Key([low.0[0], low.0[1], low.0[2] + z]);
            assert_eq!(
                map.voxel(key),
                Voxel::Free(one_miss()),
                "voxel {key:?} of column [{x}, {y}]"
            );
        }
    }
}

// A voxel that ten scans hit, its column cleared, reads free, and the next scan that hits
// it marks it occupied again, as Nav2's obstacle layer marks a cleared cell on the next
// frame that sees it.
#[test]
fn a_cleared_voxel_is_marked_again_by_one_hit() {
    let mut map = map();
    let point = [0.52_f32, 0.21, 0.33];
    for _ in 0..10 {
        insert(&mut map, identity(), &[point], &[]);
    }
    let key = map
        .key(point.map(f64::from))
        .expect("the map holds the point");
    assert!(map.is_occupied(key), "{:?}", map.voxel(key));
    let column = column_of(&map, map.columns(), key);
    map.clear_columns([column]);
    assert_eq!(map.voxel(key), Voxel::Free(one_miss()));
    insert(&mut map, identity(), &[point], &[]);
    assert!(map.is_occupied(key), "{:?}", map.voxel(key));
}

// A column outside the map is no column of it.
#[test]
#[should_panic(expected = "lies outside the map's")]
fn a_column_outside_the_map_is_refused() {
    let mut map = map();
    let columns = map.columns();
    map.clear_columns([[columns.size_x, 0]]);
}

// A map recentered over bounds moved by a drawn offset keeps what it held of each voxel the
// new bounds hold, and knows no other voxel.
#[hegel::test(test_cases = 50)]
fn a_recentered_map_keeps_the_voxels_its_new_bounds_hold(tc: TestCase) {
    let mut map = map();
    let sensor = sensor(&tc);
    let points: Vec<[f32; 3]> = (0..tc.draw(gs::integers::<usize>().min_value(1).max_value(40)))
        .map(|_| point(&tc, 0.2, 1.4))
        .collect();
    insert(&mut map, sensor, &points, &[]);
    let offset: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-1.5).max_value(1.5)));
    let moved = map
        .recentered(Bounds {
            min: offset.map(|c| c - EDGE_M),
            max: offset.map(|c| c + EDGE_M),
        })
        .expect("bounds the tree holds");
    assert_eq!(moved.frame(), map.frame());
    assert_eq!(moved.voxel_size(), map.voxel_size());
    let held = |key: Key| moved.key(moved.center(key)) == Some(key);
    for (key, voxel) in voxels(&map) {
        if held(key) {
            assert_eq!(moved.voxel(key), voxel, "{key:?}");
        } else {
            assert_eq!(moved.voxel(key), Voxel::Unknown, "{key:?}");
        }
    }
    assert!(
        moved.known().all(|key| map.voxel(key) != Voxel::Unknown),
        "the recentered map knows a voxel the map did not"
    );
}
