//! Properties of the voxel map on drawn scenes: what one point does to the
//! voxels of its ray, what an empty depth sample and a point that is not a
//! number do, what repeated scans of a still scene make of a surface, the
//! robot cut out where it stands, the frames a map refuses, the distance
//! field against a brute-force distance inside the map and outside it, and a
//! rendered depth frame of a box.

use std::collections::BTreeSet;

use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use collision_geometry::chain::Padding;
use control_core::positive_finite::PositiveFinite;
use hegel::TestCase;
use hegel::generators as gs;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{
    Bounds, Cutout, DistanceField, Frame, InsertError, Key, NotANumber, RobotCutout, Scan, Voxel,
    VoxelMap, depth_points,
};
use openarm_description::Side;

#[path = "common/arms.rs"]
mod arms;
#[path = "common/ball.rs"]
mod ball;
#[path = "common/pose.rs"]
mod pose;
#[path = "common/slab.rs"]
mod slab;
use arms::OPENARM_V2_ARMS;
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

fn frame() -> Frame {
    Frame::parse(MAP).expect("a frame name")
}

fn range(meters: f64) -> PositiveFinite {
    PositiveFinite::parse(meters).expect("a positive range")
}

fn map() -> VoxelMap {
    VoxelMap::new(
        frame(),
        PositiveFinite::parse(VOXEL_M).expect("positive"),
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

/// The pose that places nothing anywhere else.
fn identity() -> Pose {
    Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose")
}

/// A sensor somewhere in the middle of the map, turned any way.
fn sensor(tc: &TestCase) -> Pose {
    let origin: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-0.5).max_value(0.5)));
    let q: [f64; 4] = tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)));
    let norm = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    let q = if norm > 0.1 {
        q.map(|c| c / norm)
    } else {
        [0.0, 0.0, 0.0, 1.0]
    };
    Pose::new(origin, q).expect("a unit quaternion")
}

/// A point at a distance of `near` to `far` from the sensor, any way.
fn point(tc: &TestCase, near: f64, far: f64) -> [f32; 3] {
    let direction: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0)));
    let norm = direction.iter().map(|c| c * c).sum::<f64>().sqrt();
    let direction = if norm > 0.1 {
        direction.map(|c| c / norm)
    } else {
        [0.0, 0.0, 1.0]
    };
    let distance: f64 = tc.draw(gs::floats().min_value(near).max_value(far));
    direction.map(|c| (c * distance) as f32)
}

/// The key `map` gives a sensor point that it holds.
fn key_of(map: &VoxelMap, sensor: &Pose, p: [f32; 3]) -> Key {
    map.key(sensor.apply(p.map(f64::from))).expect("in the map")
}

fn log_odds(map: &VoxelMap, key: Key) -> Option<f32> {
    match map.voxel(key) {
        Voxel::Unknown => None,
        Voxel::Free(v) | Voxel::Occupied(v) => Some(v),
    }
}

/// Inserts the points a sensor at `sensor` reads, cutting out `cutouts`.
fn insert(map: &mut VoxelMap, sensor: Pose, points: &[[f32; 3]], cutouts: &[&dyn Cutout]) {
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
    let sensor = Pose::new([0.12, -0.07, 0.31], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
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
    let side = if tc.draw(gs::booleans()) {
        Side::Left
    } else {
        Side::Right
    };
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

// A map takes a scan whose sensor stands in its own frame and cutouts in its
// own frame, and refuses any other, changing nothing.
#[test]
fn a_map_refuses_a_scan_or_a_cutout_in_another_frame() {
    let mut map = map();
    let (ours, other) = (frame(), Frame::parse("camera").expect("a frame name"));
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
    let sensor = Pose::new(origin, [0.0, 0.0, 0.0, 1.0]).expect("a pose");
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
