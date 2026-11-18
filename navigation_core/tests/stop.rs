//! The stop on drawn scenes: a base driving at a wall of occupied cells slows so that no
//! cell's center lies inside the footprint at any step of the projection the slowed
//! velocity covers within the time before collision, and goes on at full speed when the
//! wall lies beyond what that time covers; one occupied cell is a contact, and one inside
//! the footprint now brings the base to rest; a drop that `LiveObstacles` marks ahead stops
//! the base before its footprint reaches it; a post and a drop at the padded front edge,
//! seen by a frame at every step, stop the base before contact on the newest frame's
//! columns, which the footprint's clearing leaves, as does a post in the padding, each
//! source's newest frame kept apart, and kept by a clear that empties the grid until the
//! source's next frame; the base's own body inside its outline never slows it, seen by a
//! depth sensor, or by a lidar while the base drives or turns, which marks no cell, and a
//! body drawn on its outline holds it; a source whose readings stop arriving stops the
//! base, as does every source from a decision that finds the clock gone back until its next
//! reading, a dead source among them, and the stop gives that source back; a frame that
//! arrived after the now of the decision that reads it is live, and a reading that arrived
//! after a decision's now holds for the timeout from the first decision that reads it,
//! before any decision and when given after a decision that finds the clock gone back; a
//! reading of a source the live obstacles do not read is refused; and a simulation step
//! past the time before collision, or too short to advance every time of the projection,
//! makes no stop.

mod common;

use std::collections::BTreeSet;
use std::num::NonZeroUsize;

use camera_core::laser_scan::{LaserScan, Sweep};
use camera_core::pose::{PlanarPose, Pose};
use common::finite::positive;
use common::footprints::padded;
use common::layers::{layer, lethal_cells};
use common::poses::{moved, on_base, planar};
use control_core::twist::Twist;
use frame_tree::FrameName;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::PaddedFootprint;
use navigation_core::obstacles::{
    GridPoses, LiveObstacles, ObstacleHeights, ObstacleHeightsParams,
};
use navigation_core::stop::{Stop, StopCause, StopDecision, StopError, StopParams, UnknownSource};
use occupancy_map::Bounds;

/// The front of the footprint, in meters ahead of the base's center.
const FRONT: f64 = 0.25;
/// The half width of the footprint, in meters.
const HALF_WIDTH: f64 = 0.2;
/// The time before collision and the simulation step, in seconds.
const TIME_BEFORE_COLLISION: f64 = 2.0;
const STEP: f64 = 0.1;
/// The source timeout, in seconds and in nanoseconds.
const TIMEOUT_S: f64 = 0.5;
const TIMEOUT_NS: i64 = 500_000_000;
/// The time now of the cases that read no clock, in nanoseconds.
const NOW_NS: i64 = 10_000_000_000;
/// The slack of a comparison of two computed distances.
const TOLERANCE: f64 = 1e-9;
/// The shares of a reach that keep a drawn gap off the distance the projection's last step
/// reaches, whose count of steps the summed step times decide.
const WITHIN: f64 = 0.98;
const BEYOND: f64 = 1.02;

fn footprint() -> PaddedFootprint {
    padded(
        &[
            [-FRONT, -HALF_WIDTH],
            [FRONT, -HALF_WIDTH],
            [FRONT, HALF_WIDTH],
            [-FRONT, HALF_WIDTH],
        ],
        0.0,
    )
}

fn params() -> StopParams {
    StopParams {
        time_before_collision_s: positive(TIME_BEFORE_COLLISION),
        simulation_time_step_s: positive(STEP),
        source_timeout_s: positive(TIMEOUT_S),
    }
}

/// The voxel side and the grid's cell, in meters.
const VOXEL_M: f64 = 0.05;

/// Empty live obstacles of `padded_footprint` whose `sources` sources give its frames, in
/// cells of `VOXEL_M`.
fn live_obstacles(padded_footprint: PaddedFootprint, sources: usize) -> LiveObstacles {
    let heights = ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: 0.1,
        max_obstacle_height_m: 0.6,
        max_step_down_m: 0.1,
        voxel_m: positive(VOXEL_M),
    })
    .expect("heights of whole voxels");
    LiveObstacles::new(
        FrameName::parse("odom").expect("a frame name"),
        Bounds {
            min: [-1.5, -1.5, -0.5],
            max: [3.0, 1.5, 1.5],
        },
        heights,
        padded_footprint,
        NonZeroUsize::new(sources).expect("a source or more"),
    )
    .expect("a map")
}

/// The stop of `params()`, no reading yet.
fn fresh_stop() -> Stop {
    Stop::parse(params()).expect("a step within the time before collision")
}

/// The stop of a base of `footprint()` with one source, whose reading arrived at
/// `arrived_ns`, and the live obstacles it reads.
fn stop_read_at(arrived_ns: i64) -> (Stop, LiveObstacles) {
    let live = live_obstacles(footprint(), 1);
    let mut stop = fresh_stop();
    stop.reading_arrived(&live, 0, arrived_ns)
        .expect("the live obstacles' one source");
    (stop, live)
}

/// The centers of a column of cells across the footprint's width, `gap` ahead of its front.
fn wall(gap: f64) -> Vec<[f64; 2]> {
    (0..9)
        .map(|i| [FRONT + gap, -0.16 + f64::from(i) * 0.04])
        .collect()
}

fn forward(speed: f64) -> Twist {
    Twist::new(speed, 0.0, 0.0).expect("finite")
}

/// The decision for `occupied` at `velocity`, one source's reading just arrived.
fn decide(occupied: &[[f64; 2]], velocity: Twist) -> StopDecision {
    let (mut stop, live) = stop_read_at(NOW_NS);
    stop.decide(&live, velocity, occupied, NOW_NS)
}

// A wall the projection reaches slows the base so that, at the slowed speed, the time
// before collision covers no more than the gap, and no less than the gap less one step at
// the commanded speed. A wall beyond the projection leaves the speed as it is.
#[hegel::test]
fn the_stop_slows_so_no_step_of_the_slowed_projection_holds_a_contact(tc: TestCase) {
    let speed = tc.draw(gs::floats().min_value(0.1).max_value(1.0));
    let reach = speed * TIME_BEFORE_COLLISION;
    let near = tc.draw(gs::booleans());
    let gap = if near {
        tc.draw(
            gs::floats()
                .min_value(0.01)
                .max_value(WITHIN * (reach - speed * STEP)),
        )
    } else {
        tc.draw(
            gs::floats()
                .min_value(BEYOND * (reach + 2.0 * speed * STEP))
                .max_value(5.0),
        )
    };
    let decision = decide(&wall(gap), forward(speed));
    if !near {
        assert_eq!(
            decision,
            StopDecision::Continue {
                velocity: forward(speed)
            },
            "a wall {gap} m ahead"
        );
        return;
    }
    let StopDecision::Slow { velocity, .. } = decision else {
        panic!("a wall {gap} m ahead at {speed} m/s gives {decision:?}");
    };
    let covered = velocity.linear_x() * TIME_BEFORE_COLLISION;
    assert!(
        covered <= gap + TOLERANCE && covered >= gap - speed * STEP - TOLERANCE,
        "a wall {gap} m ahead at {speed} m/s: {velocity:?} covers {covered} m"
    );
    assert_eq!((velocity.linear_y(), velocity.angular_z()), (0.0, 0.0));
}

// One occupied cell's center the footprint reaches is a contact: no occupied cell's center
// may lie inside the padded footprint at any step of the projection.
#[test]
fn one_occupied_cell_is_a_contact() {
    assert!(matches!(
        decide(&[[FRONT + 0.3, 0.0]], forward(0.5)),
        StopDecision::Slow { .. }
    ));
    assert_eq!(
        decide(&[[FRONT + 0.3, HALF_WIDTH + 0.1]], forward(0.5)),
        StopDecision::Continue {
            velocity: forward(0.5)
        }
    );
}

// An occupied cell already inside the footprint, or reached at the first step, brings the
// base to rest.
#[test]
fn a_contact_now_scales_the_velocity_to_rest() {
    for gap in [-0.1, 0.02] {
        assert_eq!(
            decide(&wall(gap), forward(0.5)),
            StopDecision::Slow {
                velocity: Twist::ZERO,
                collision_time_s: 0.0
            },
            "a wall {gap} m ahead"
        );
    }
}

/// Where the floor drops away, along x, in meters: a fifth of a cell into a cell.
const EDGE_M: f64 = 1.41;

/// The live obstacles of a sensor 1.2 m over the edge of a floor that drops 0.3 m, its
/// points every 2 cm.
fn ledge() -> LiveObstacles {
    let mut live = live_obstacles(footprint(), 1);
    let sensor = Pose::new([1.0, 0.0, 1.2], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let points: Vec<[f32; 3]> = (0..150)
        .flat_map(|i| (0..50).map(move |j| (i, j)))
        .map(|(i, j)| {
            let x = -0.5 + f64::from(i) * 0.02;
            let y = -0.49 + f64::from(j) * 0.02;
            let z = if x < EDGE_M { 0.0 } else { -0.3 };
            [x - 1.0, y, z - 1.2].map(|c| c as f32)
        })
        .collect();
    let away = planar(-10.0, 0.0, 0.0);
    live.insert_depth_frame(0, &points, away, on_base(away, &sensor), positive(5.0))
        .expect("the live obstacles' one source");
    live
}

// A base drives at 0.5 m/s toward a step down 0.3 m deep, deeper than the 0.1 m limit, that
// the live obstacles mark: the stop slows it as the drop's cells come within the projected
// footprint and brings it to rest before its footprint reaches them.
#[test]
fn a_drop_cell_within_the_projected_footprint_stops_the_base() {
    let live = ledge();
    let grid = layer(&live).grid().clone();
    let commanded = forward(0.5);
    let mut stop = fresh_stop();
    let dt_ns: i64 = 50_000_000;
    let mut base = planar(0.0, 0.0, 0.0);
    let mut executed = commanded;
    for step in 0..400 {
        let now = i64::from(step) * dt_ns;
        let occupied = live.occupied_seen_from(&grid, standing_on_the_map(base));
        assert!(!occupied.is_empty(), "the drop's cells are lethal");
        assert!(
            occupied.iter().all(|center| center[0] > FRONT),
            "the footprint at {base:?} reaches a drop cell"
        );
        stop.reading_arrived(&live, 0, now)
            .expect("the live obstacles' one source");
        executed = stop.decide(&live, commanded, &occupied, now).velocity();
        base = planar(base.x() + executed.linear_x() * 0.05, base.y(), base.yaw());
    }
    assert_eq!(executed, Twist::ZERO, "the base comes to rest, at {base:?}");
    assert!(
        base.x() + FRONT < EDGE_M,
        "the base stops before the edge, at {base:?}"
    );
}

/// The padding of the footprint the stop reads a source's newest frame by, in meters: a
/// fifth of a cell.
const PADDING_M: f64 = 0.01;
/// The time before collision and the simulation step of those stops, in seconds.
const APPROACH_HORIZON_S: f64 = 1.2;
const APPROACH_STEP_S: f64 = 0.1;
/// The time between two frames, and two decisions, of a drive, in nanoseconds and seconds.
const FRAME_NS: i64 = 50_000_000;
const FRAME_S: f64 = 0.05;

/// `footprint()` padded by `PADDING_M`.
fn padded_footprint() -> PaddedFootprint {
    padded(
        &[
            [-FRONT, -HALF_WIDTH],
            [FRONT, -HALF_WIDTH],
            [FRONT, HALF_WIDTH],
            [-FRONT, HALF_WIDTH],
        ],
        PADDING_M,
    )
}

/// The stop of a time before collision `APPROACH_HORIZON_S` and a step `APPROACH_STEP_S`.
fn approach_stop() -> Stop {
    Stop::parse(StopParams {
        time_before_collision_s: positive(APPROACH_HORIZON_S),
        simulation_time_step_s: positive(APPROACH_STEP_S),
        source_timeout_s: positive(TIMEOUT_S),
    })
    .expect("a step within the time before collision")
}

/// The poses by which the stop reads the points of live obstacles whose map is the grid's
/// frame, the base at `base`.
fn standing_on_the_map(base: PlanarPose) -> GridPoses {
    GridPoses {
        grid_from_map: PlanarPose::IDENTITY,
        base,
    }
}

/// The points the stop reads of `live`, projected onto the grid of its columns, the base at
/// `base`.
fn occupied_of(live: &LiveObstacles, base: PlanarPose) -> Vec<[f64; 2]> {
    live.occupied_seen_from(layer(live).grid(), standing_on_the_map(base))
}

/// A sensor `height_m` up over the base's center, facing along +X.
fn sensor_on_base(height_m: f64) -> Pose {
    Pose::new([0.0, 0.0, height_m], [0.0, 0.0, 0.0, 1.0]).expect("a pose")
}

/// `points`, in the map's frame, seen from the sensor `sensor_on_base` of a base at `base`.
fn frame_of(points: &[[f64; 3]], base: PlanarPose, sensor_on_base: &Pose) -> Vec<[f32; 3]> {
    let to_sensor = base.to_pose().compose(sensor_on_base).inverse();
    points
        .iter()
        .map(|&point| to_sensor.apply(point).map(|c| c as f32))
        .collect()
}

/// The decisions of a base that drives from `start` at `commanded` for `frames` frames, a
/// frame of `scene`'s points for the base where it stands inserted before each decision, the
/// stop reading `LiveObstacles::occupied_seen_from` of the projected grid: each pose and the
/// velocity the stop let the base execute there.
fn drive(
    start: PlanarPose,
    commanded: Twist,
    frames: i64,
    sensor_height_m: f64,
    scene: impl Fn(PlanarPose) -> Vec<[f64; 3]>,
) -> Vec<(PlanarPose, Twist)> {
    let mut live = live_obstacles(padded_footprint(), 1);
    let mut stop = approach_stop();
    let mount = sensor_on_base(sensor_height_m);
    let mut base = start;
    (0..frames)
        .map(|frame| {
            let now = frame * FRAME_NS;
            let points = frame_of(&scene(base), base, &mount);
            live.insert_depth_frame(0, &points, base, mount, positive(5.0))
                .expect("the live obstacles' one source");
            stop.reading_arrived(&live, 0, now)
                .expect("the live obstacles' one source");
            let occupied = occupied_of(&live, base);
            let executed = stop.decide(&live, commanded, &occupied, now).velocity();
            let here = base;
            base = planar(
                base.x() + executed.linear_x() * FRAME_S,
                base.y(),
                base.yaw(),
            );
            (here, executed)
        })
        .collect()
}

/// Where the post stands, along x: 2.8 cm past the padded front edge of a base whose
/// padded front edge lies at 1.012 m, both in the cell from 1.00 m to 1.05 m.
const POST_X_M: f64 = 1.040;

/// The face of a post at `x_m` along x, 4 cm wide, from the floor's band to 0.5 m, its
/// points every 2 cm.
fn post_face(x_m: f64) -> Vec<[f64; 3]> {
    (0..3)
        .flat_map(|j| (0..10).map(move |k| (j, k)))
        .map(|(j, k)| [x_m, -0.02 + f64::from(j) * 0.02, 0.12 + f64::from(k) * 0.04])
        .collect()
}

// A base whose padded front edge stands at 1.012 m, in the cell of a post at 1.040 m that
// no saved map holds, commanded at 0.3 m/s toward it, a frame of the post inserted at every
// step: the footprint's clearing frees that cell in the grid, and the stop, reading the
// post's column of the newest frame, brings the base to rest at once, its body short of the
// post, as Nav2's monitor stops it on the post's point.
#[test]
fn a_post_at_the_padded_front_edge_stops_the_base_before_contact() {
    let padded_front = FRONT + PADDING_M;
    let start = planar(1.012 - padded_front, 0.0, 0.0);
    let decisions = drive(start, forward(0.3), 40, 0.8, |_| post_face(POST_X_M));
    for (base, executed) in &decisions {
        assert!(
            base.x() + FRONT < POST_X_M,
            "the base at {base:?} meets the post, executing {executed:?}"
        );
    }
    let (rest, executed) = decisions.last().expect("a drive");
    assert_eq!(*executed, Twist::ZERO, "the base rests, at {rest:?}");
}

/// Where the floor drops away 0.3 m, along x, in meters: two fifths of a cell into the
/// cell from 1.40 m to 1.45 m.
const DROP_EDGE_M: f64 = 1.42;
/// The depth of the drop, in meters, deeper than the 0.1 m step down.
const DROP_DEPTH_M: f64 = 0.3;
/// The height of the sensor that looks down over the drop, in meters.
const DROP_SENSOR_M: f64 = 1.2;

/// The floor a sensor `DROP_SENSOR_M` over the base's center at `base` sees ahead of the
/// base, from 0.3 m to 1.6 m ahead, 0.24 m to either side, its points every 2 cm on a grid
/// fixed to the floor: the floor before `DROP_EDGE_M`, and past it the lower floor that the
/// sensor sees over the edge.
fn floor_with_a_drop(base: PlanarPose) -> Vec<[f64; 3]> {
    let sensor_x = base.x();
    let over_the_edge = |x: f64| {
        sensor_x + (x - sensor_x) * DROP_SENSOR_M / (DROP_SENSOR_M + DROP_DEPTH_M) >= DROP_EDGE_M
    };
    (0..200)
        .map(|i| f64::from(i) * 0.02)
        .filter(|&x| x >= sensor_x + 0.3 && x <= sensor_x + 1.6)
        .flat_map(|x| (0..25).map(move |j| (x, -0.24 + f64::from(j) * 0.02)))
        .filter_map(|(x, y)| {
            if x < DROP_EDGE_M {
                return Some([x, y, 0.0]);
            }
            over_the_edge(x).then_some([x, y, -DROP_DEPTH_M])
        })
        .collect()
}

// A base commanded at 0.2 m/s toward a drop, a step of the projection 2 cm, under half a
// cell, a frame of the floor inserted at every step: each frame's clearing frees the cell of
// the padded front edge, and the stop, reading the drop's columns of the newest frame,
// brings the base to rest, its body short of the edge.
#[test]
fn a_slow_approach_over_a_drop_stops_the_base_before_the_edge() {
    let start = planar(1.10 - FRONT - PADDING_M, 0.0, 0.0);
    let decisions = drive(start, forward(0.2), 300, DROP_SENSOR_M, floor_with_a_drop);
    for (base, executed) in &decisions {
        assert!(
            base.x() + FRONT < DROP_EDGE_M,
            "the base at {base:?} stands over the drop, executing {executed:?}"
        );
    }
    let (rest, executed) = decisions.last().expect("a drive");
    assert_eq!(*executed, Twist::ZERO, "the base rests, at {rest:?}");
}

// A base whose front overhangs the drop's edge by 10 cm, its sensor seeing the lower floor
// under that overhang and past it: the drop's columns whose centers lie inside its unpadded
// outline, floor it stands on, give the stop no point, and those past its front give
// points ahead of it.
#[test]
fn a_drop_column_under_the_base_gives_no_point() {
    let base = planar(DROP_EDGE_M + 0.1 - FRONT, 0.0, 0.0);
    let mount = sensor_on_base(DROP_SENSOR_M);
    let floor: Vec<[f64; 3]> = (0..200)
        .map(|i| f64::from(i) * 0.02)
        .filter(|&x| x >= base.x() - 0.2 && x <= base.x() + 1.0)
        .flat_map(|x| (0..25).map(move |j| (x, -0.24 + f64::from(j) * 0.02)))
        .map(|(x, y)| [x, y, if x < DROP_EDGE_M { 0.0 } else { -DROP_DEPTH_M }])
        .collect();
    let mut live = live_obstacles(padded_footprint(), 1);
    live.insert_depth_frame(
        0,
        &frame_of(&floor, base, &mount),
        base,
        mount,
        positive(5.0),
    )
    .expect("the live obstacles' one source");
    let occupied = occupied_of(&live, base);
    assert!(
        occupied
            .iter()
            .all(|&[x, y]| x.abs() > FRONT || y.abs() > HALF_WIDTH),
        "a point inside the outline: {occupied:?}"
    );
    assert!(
        occupied.iter().any(|&[x, _]| x > FRONT),
        "no drop ahead: {occupied:?}"
    );
}

/// A displacement of up to a millimeter, a fixed draw for each `index`, in meters.
fn millimeter_noise(index: usize) -> f64 {
    ((index * 7919) % 2001) as f64 / 1e6 - 1e-3
}

/// The base's own front bumper and deck as its sensor sees them, at `base`: the bumper
/// `inset_m` inside the front edge, from 0.15 m to 0.3 m up, and the deck at 0.3 m, every
/// 2 cm, each point moved along x and along y by `millimeter_noise`.
fn own_body(base: PlanarPose, inset_m: f64) -> Vec<[f64; 3]> {
    let across = (0..20).map(|j| -HALF_WIDTH + 0.01 + f64::from(j) * 0.02);
    let bumper = across
        .clone()
        .flat_map(|y| (0..4).map(move |k| [FRONT - inset_m, y, 0.15 + f64::from(k) * 0.05]));
    let deck =
        across.flat_map(|y| (0..24).map(move |i| [-FRONT + 0.01 + f64::from(i) * 0.02, y, 0.3]));
    bumper
        .chain(deck)
        .enumerate()
        .map(|(index, [x, y, z])| {
            let noisy = [
                x + millimeter_noise(index),
                y + millimeter_noise(index + 1000),
            ];
            let [x, y] = base.apply(noisy);
            [x, y, z]
        })
        .collect()
}

// A base whose sensor sees its own bumper and deck just inside its unpadded outline, each
// return up to a millimeter off its place, the bumper 1.1 mm inside the front edge, drives
// on an open floor at 0.3 m/s: a frame of its body inserted at every step never slows it.
#[test]
fn the_base_own_body_inside_its_outline_never_stops_it() {
    let decisions = drive(planar(-0.5, 0.0, 0.0), forward(0.3), 40, 0.8, |base| {
        own_body(base, 0.0011)
    });
    for (base, executed) in &decisions {
        assert_eq!(
            *executed,
            forward(0.3),
            "the base at {base:?} slows for its body"
        );
    }
}

// A bumper drawn on the front edge of the outline, each return up to a millimeter off it,
// scatters returns across the edge: those outside the outline are points the stop reads, and
// they hold the base at rest from the first frame, as they hold Nav2's monitor.
#[test]
fn the_base_own_body_on_its_outline_holds_it() {
    let decisions = drive(planar(-0.5, 0.0, 0.0), forward(0.3), 10, 0.8, |base| {
        own_body(base, 0.0)
    });
    for (base, executed) in &decisions {
        assert_eq!(*executed, Twist::ZERO, "the base at {base:?} moves");
    }
}

/// Where a post stands in the padding, along x: 5 mm past the unpadded front edge of a base
/// at `IN_THE_PADDING_BASE_X`, at the center of the cell from 1.00 m to 1.05 m.
const IN_THE_PADDING_X_M: f64 = 1.025;
const IN_THE_PADDING_BASE_X: f64 = IN_THE_PADDING_X_M - FRONT - 0.005;

// A base at rest, commanded at 0.3 m/s, whose sensor sees a post 5 mm past its unpadded
// front edge, inside the 1 cm padding: the footprint's clearing frees the post's cell in the
// grid, and the stop, reading the newest frame's column, whose center lies inside the padded
// footprint, holds the base at rest, as Nav2's monitor holds it on the post's points.
#[test]
fn a_post_in_the_padding_holds_the_base() {
    let start = planar(IN_THE_PADDING_BASE_X, 0.0, 0.0);
    let decisions = drive(start, forward(0.3), 10, 0.8, |_| {
        post_face(IN_THE_PADDING_X_M)
    });
    for (base, executed) in &decisions {
        assert_eq!(*executed, Twist::ZERO, "the base at {base:?} moves");
    }
}

/// The pose of a base at rest whose padding holds the post at `IN_THE_PADDING_X_M`.
fn by_the_post_in_the_padding() -> PlanarPose {
    planar(IN_THE_PADDING_BASE_X, 0.0, 0.0)
}

/// Inserts a frame of `source` that sees `points`, from a sensor 0.8 m up over the center of
/// the base at `by_the_post_in_the_padding()`, and gives the points the stop then reads.
fn insert_by_the_post(
    live: &mut LiveObstacles,
    source: usize,
    points: &[[f64; 3]],
) -> Vec<[f64; 2]> {
    let base = by_the_post_in_the_padding();
    let mount = sensor_on_base(0.8);
    live.insert_depth_frame(
        source,
        &frame_of(points, base, &mount),
        base,
        mount,
        positive(5.0),
    )
    .expect("a source of the live obstacles");
    occupied_of(live, base)
}

// Two sources feed the stop of a base at rest: source 0's frame sees the post in the
// padding, and source 1's frame after it sees nothing. The live obstacles keep each source's
// newest frame apart, so the post's column still holds the base; source 0's next frame,
// empty too, forgets it, and the base drives on.
#[test]
fn each_source_keeps_its_newest_frame() {
    let mut live = live_obstacles(padded_footprint(), 2);
    let mut stop = approach_stop();
    let mut decide = |live: &LiveObstacles, occupied: &[[f64; 2]]| {
        for source in 0..2 {
            stop.reading_arrived(live, source, NOW_NS)
                .expect("a source of the live obstacles");
        }
        stop.decide(live, forward(0.3), occupied, NOW_NS)
    };
    insert_by_the_post(&mut live, 0, &post_face(IN_THE_PADDING_X_M));
    let occupied = insert_by_the_post(&mut live, 1, &[]);
    assert_eq!(
        decide(&live, &occupied).velocity(),
        Twist::ZERO,
        "{occupied:?}"
    );
    let occupied = insert_by_the_post(&mut live, 0, &[]);
    assert_eq!(
        decide(&live, &occupied),
        StopDecision::Continue {
            velocity: forward(0.3)
        }
    );
}

/// Where a second post stands, along x, ahead of the padded footprint of the base at
/// `by_the_post_in_the_padding()`: at the center of the cell from 1.50 m to 1.55 m.
const AHEAD_X_M: f64 = 1.525;

// A base at rest, commanded at 0.3 m/s, whose sensor sees the post in the padding and a post
// ahead of its padded footprint: the grid marks the post ahead lethal, and the footprint's
// clearing frees the post in the padding. A clear empties the grid, which then holds no
// lethal cell, and keeps the source's newest frame: the stop still reads the centers of both
// posts' columns, either side of y = 0, and the post in the padding holds the base. The
// source's next frame, which sees nothing there, replaces it, and the base drives on.
#[test]
fn a_clear_empties_the_grid_and_keeps_the_source_newest_frame() {
    let mut live = live_obstacles(padded_footprint(), 1);
    let mut stop = approach_stop();
    let base = by_the_post_in_the_padding();
    let mut decide_at = |live: &LiveObstacles, occupied: &[[f64; 2]], now_ns: i64| {
        stop.reading_arrived(live, 0, now_ns)
            .expect("the live obstacles' one source");
        stop.decide(live, forward(0.3), occupied, now_ns)
    };
    let in_mm = |point: [f64; 2]| point.map(|c| (c * 1000.0).round() as i64);
    let posts: Vec<[f64; 3]> = post_face(IN_THE_PADDING_X_M)
        .into_iter()
        .chain(post_face(AHEAD_X_M))
        .collect();
    insert_by_the_post(&mut live, 0, &posts);
    assert!(
        !lethal_cells(&layer(&live)).is_empty(),
        "the grid marks the post ahead"
    );
    live.clear();
    assert!(
        lethal_cells(&layer(&live)).is_empty(),
        "the clear empties the grid"
    );
    let occupied = occupied_of(&live, base);
    let read: BTreeSet<[i64; 2]> = occupied
        .iter()
        .map(|&point| in_mm(base.apply(point)))
        .collect();
    let posts_columns: BTreeSet<[i64; 2]> = [IN_THE_PADDING_X_M, AHEAD_X_M]
        .into_iter()
        .flat_map(|x| [-0.5, 0.5].map(|side| in_mm([x, side * VOXEL_M])))
        .collect();
    assert_eq!(read, posts_columns);
    assert_eq!(decide_at(&live, &occupied, NOW_NS).velocity(), Twist::ZERO);
    let occupied = insert_by_the_post(&mut live, 0, &[]);
    assert!(occupied.is_empty(), "{occupied:?}");
    assert_eq!(
        decide_at(&live, &occupied, NOW_NS + FRAME_NS),
        StopDecision::Continue {
            velocity: forward(0.3)
        }
    );
}

/// A lidar's scan: 361 rays a degree apart from -pi, swept counterclockwise in
/// `LIDAR_SWEEP_S`, in seconds and in nanoseconds.
const LIDAR_RAYS: usize = 361;
const LIDAR_SWEEP_S: f64 = 0.1;
const LIDAR_SWEEP_NS: i64 = 100_000_000;

/// The scan of a lidar that sees the base's own mast alone: ray `mast_ray` returns at
/// `mast_m`, and every other ray met nothing.
fn mast_scan(mast_ray: usize, mast_m: f32) -> LaserScan {
    let mut ranges = vec![10000.0_f32; LIDAR_RAYS];
    ranges[mast_ray] = mast_m;
    let intervals = (LIDAR_RAYS - 1) as f64;
    LaserScan::parse(
        -std::f32::consts::PI,
        (std::f64::consts::TAU / intervals) as f32,
        (LIDAR_SWEEP_S / intervals) as f32,
        0.05,
        8.0,
        &ranges,
    )
    .expect("a scan of the mast")
}

/// The decisions of a base that starts at `start`, commanded at `commanded`, whose lidar,
/// 0.3 m up over its center, gives `scan` over each sweep while the base moves at the
/// velocity the stop last let it execute, the scan's first ray where the base stood as the
/// sweep began and its last where it stands as it ends, the stop deciding then on the
/// points `LiveObstacles::occupied_seen_from` gives: each pose, the velocity the stop let
/// the base execute there, and the points it read.
fn lidar_drive(
    start: PlanarPose,
    commanded: Twist,
    sweeps: i64,
    scan: &LaserScan,
) -> Vec<(PlanarPose, Twist, Vec<[f64; 2]>)> {
    let mut live = live_obstacles(padded_footprint(), 1);
    let mut stop = approach_stop();
    let lidar = sensor_on_base(0.3);
    let mut base = start;
    let mut executed = commanded;
    (1..=sweeps)
        .map(|sweep| {
            let ended = moved(base, executed.axes(), scan.sweep_s());
            let swept = Sweep {
                first_ray: base,
                last_ray: ended,
            };
            live.insert_laser_scan(0, scan, swept, lidar, positive(5.0))
                .expect("the live obstacles' one source");
            let now = sweep * LIDAR_SWEEP_NS;
            stop.reading_arrived(&live, 0, now)
                .expect("the live obstacles' one source");
            let occupied = occupied_of(&live, ended);
            executed = stop.decide(&live, commanded, &occupied, now).velocity();
            base = ended;
            (ended, executed, occupied)
        })
        .collect()
}

/// Asserts that the stop read no point at any sweep of `drive` and let the base execute
/// `commanded` at each.
fn assert_never_slowed(drive: &[(PlanarPose, Twist, Vec<[f64; 2]>)], commanded: Twist) {
    for (base, executed, occupied) in drive {
        assert!(
            occupied.is_empty() && *executed == commanded,
            "the base at {base:?} reads {occupied:?} and executes {executed:?}"
        );
    }
}

// A lidar over the base's center sees the base's own mast while the base drives at 1 m/s
// and at 0.5 m/s from starts a centimeter apart across a cell, and while it turns in place
// at 1 rad/s, each scan swept in 0.1 s. Driving, the mast stands 2 cm inside the front
// edge, and the base moves 5 cm or 2.5 cm from the scan's first ray to the mast's ray,
// which places the mast past the outline at the first ray's pose, and at 1 m/s past the
// padded one. Turning, a mast 2 cm inside the front edge and 7 mm inside the left side,
// and one 9 mm inside the front edge and 1.2 cm inside the left side, turn past the
// outline's side at the first ray's pose. Each return is judged where the lidar stood on
// the base at its ray's time, inside the outline, so it marks nothing: the stop reads no
// point, neither a lethal cell of the grid nor a column of the newest frame, and no scan
// slows the base.
#[test]
fn a_lidar_that_sees_the_base_own_mast_marks_nothing_and_never_slows_it() {
    for speed in [1.0, 0.5] {
        for start_x in [-0.5, -0.49, -0.48, -0.47, -0.46] {
            let ahead = lidar_drive(
                planar(start_x, 0.0, 0.0),
                forward(speed),
                20,
                &mast_scan(180, 0.23),
            );
            assert_never_slowed(&ahead, forward(speed));
        }
    }
    let turning = Twist::new(0.0, 0.0, 1.0).expect("finite");
    for mast in [mast_scan(220, 0.3002), mast_scan(218, 0.3061)] {
        assert_never_slowed(
            &lidar_drive(planar(0.0, 0.0, 0.0), turning, 20, &mast),
            turning,
        );
    }
}

/// Panics unless `decision` stops the base for the stale newest reading of `source`.
fn assert_stale(decision: StopDecision, source: usize) {
    assert!(
        matches!(
            decision,
            StopDecision::Stop(StopCause::StaleSource { source: stale, .. }) if stale == source
        ),
        "{decision:?}"
    );
}

// A source whose last reading arrived at 99.2 s reads stale at a decision at 100 s; the
// clock then goes back to 99.5 s, within the timeout after that reading: the source reads
// silent there, before the decision and at it, and on until a reading arrives on the new
// clock.
#[test]
fn a_dead_source_stays_silent_when_the_clock_goes_back_within_its_timeout() {
    let (mut stop, live) = stop_read_at(99_200_000_000);
    assert_stale(stop.decide(&live, forward(0.5), &[], 100_000_000_000), 0);
    let silent = StopCause::NoReading { source: 0 };
    assert_eq!(stop.silent_source(&live, 99_500_000_000), Some(silent));
    for now in [99_500_000_000, 99_600_000_000, 99_700_000_000] {
        assert_eq!(
            stop.decide(&live, forward(0.5), &[], now),
            StopDecision::Stop(silent),
            "at {now} ns"
        );
    }
    stop.reading_arrived(&live, 0, 99_750_000_000)
        .expect("the live obstacles' one source");
    assert_eq!(
        stop.decide(&live, forward(0.5), &[], 99_800_000_000),
        StopDecision::Continue {
            velocity: forward(0.5)
        }
    );
}

// A reading arrives at 100 s and a decision there reads it live; the clock then goes back,
// by less than the timeout, to 99.7 s, or by more, to 40 s: from the decision that finds the
// step back, the source reads silent, and its next reading on the new clock reads live.
#[test]
fn a_clock_that_goes_back_silences_a_source_until_its_next_reading() {
    let moving = StopDecision::Continue {
        velocity: forward(0.5),
    };
    let silent = StopDecision::Stop(StopCause::NoReading { source: 0 });
    for back_ns in [99_700_000_000, 40_000_000_000] {
        let (mut stop, live) = stop_read_at(100_000_000_000);
        let decide_at =
            |stop: &mut Stop, now_ns: i64| stop.decide(&live, forward(0.5), &[], now_ns);
        assert_eq!(decide_at(&mut stop, 100_000_000_000), moving);
        assert_eq!(
            decide_at(&mut stop, back_ns),
            silent,
            "back to {back_ns} ns"
        );
        assert_eq!(decide_at(&mut stop, back_ns + 10_000_000), silent);
        stop.reading_arrived(&live, 0, back_ns + 20_000_000)
            .expect("the live obstacles' one source");
        assert_eq!(decide_at(&mut stop, back_ns + 30_000_000), moving);
    }
}

// Two sources, source 0's newest reading at 99.2 s and source 1's at 99.6 s, read live at a
// decision at 99.6 s; the clock goes back to 99.5 s, after source 0's reading and before
// source 1's: both read silent from there, and each reads live again with its own next
// reading alone.
#[test]
fn a_clock_that_goes_back_silences_every_source_until_each_reads_again() {
    let live = live_obstacles(footprint(), 2);
    let mut stop = fresh_stop();
    let arrived = |stop: &mut Stop, source: usize, arrived_ns: i64| {
        stop.reading_arrived(&live, source, arrived_ns)
            .expect("a source of the live obstacles")
    };
    let decide_at = |stop: &mut Stop, now_ns: i64| stop.decide(&live, forward(0.5), &[], now_ns);
    let moving = StopDecision::Continue {
        velocity: forward(0.5),
    };
    let silent = |source| StopDecision::Stop(StopCause::NoReading { source });
    arrived(&mut stop, 0, 99_200_000_000);
    arrived(&mut stop, 1, 99_600_000_000);
    assert_eq!(decide_at(&mut stop, 99_600_000_000), moving);
    assert_eq!(decide_at(&mut stop, 99_500_000_000), silent(0));
    arrived(&mut stop, 1, 99_550_000_000);
    assert_eq!(decide_at(&mut stop, 99_560_000_000), silent(0));
    arrived(&mut stop, 0, 99_570_000_000);
    assert_eq!(decide_at(&mut stop, 99_580_000_000), moving);
}

/// The period of a caller's decisions, how far into its slot each runs, and the time
/// between the frames of a 30 Hz camera, in nanoseconds.
const PERIOD_NS: i64 = 50_000_000;
const PHASE_NS: i64 = 25_000_000;
const CAMERA_FRAME_NS: i64 = 33_333_333;

// A reading that arrived 25 ms after the now of a decision, a caller deciding at the slot
// of its period, counts as one that arrived now: the source reads live at that decision
// and at the next slot's. A caller that stamps each frame of a 30 Hz camera as it arrives
// and decides at the start of each 50 ms slot, running 25 ms into it, gives the frames
// that arrived by then: the source reads live at every slot.
#[test]
fn a_frame_that_arrived_after_its_decision_began_reads_live() {
    let (mut stop, live) = stop_read_at(NOW_NS + PHASE_NS);
    let moving = StopDecision::Continue {
        velocity: forward(0.5),
    };
    assert_eq!(stop.silent_source(&live, NOW_NS), None);
    assert_eq!(stop.decide(&live, forward(0.5), &[], NOW_NS), moving);
    assert_eq!(
        stop.decide(&live, forward(0.5), &[], NOW_NS + PERIOD_NS),
        moving
    );

    let live = live_obstacles(footprint(), 1);
    let mut stop = fresh_stop();
    let mut frames = (1..).map(|frame| frame * CAMERA_FRAME_NS).peekable();
    for slot in 1..=60 {
        let due_ns = slot * PERIOD_NS;
        while let Some(arrived_ns) = frames.next_if(|&arrived_ns| arrived_ns <= due_ns + PHASE_NS) {
            stop.reading_arrived(&live, 0, arrived_ns)
                .expect("the live obstacles' one source");
        }
        assert_eq!(
            stop.decide(&live, forward(0.5), &[], due_ns),
            moving,
            "at the slot from {due_ns} ns"
        );
    }
}

// A reading stamped 1000 s, the stop's first decision at 5 s, the clock having gone back
// before any decision: the source reads live at 5 s and a timeout later, at 5.5 s, and stale
// 1 ns after, with no reading since.
#[test]
fn a_reading_stamped_after_now_holds_for_the_timeout_from_the_decision_that_reads_it() {
    let (mut stop, live) = stop_read_at(1_000_000_000_000);
    let moving = StopDecision::Continue {
        velocity: forward(0.5),
    };
    let last_live_ns = 5_000_000_000 + TIMEOUT_NS;
    for now_ns in [5_000_000_000, last_live_ns] {
        assert_eq!(
            stop.decide(&live, forward(0.5), &[], now_ns),
            moving,
            "at {now_ns} ns"
        );
    }
    assert_stale(stop.decide(&live, forward(0.5), &[], last_live_ns + 1), 0);
}

// A decision at 100 s, the clock then running on to 100.05 s and going back 0.5 s, and a
// decision at 99.55 s that finds it gone back; a reading stamped 100.04 s on the old clock,
// given after that decision, then the source dies: the next decision, at 99.6 s, reads it
// live and gives it that now, so it reads live a timeout later, at 100.1 s, and stale 1 ns
// after.
#[test]
fn a_reading_from_before_a_step_back_given_after_its_decision_holds_for_the_timeout() {
    let (mut stop, live) = stop_read_at(100_000_000_000);
    let decide_at = |stop: &mut Stop, now_ns: i64| stop.decide(&live, forward(0.5), &[], now_ns);
    let moving = StopDecision::Continue {
        velocity: forward(0.5),
    };
    assert_eq!(decide_at(&mut stop, 100_000_000_000), moving);
    assert_eq!(
        decide_at(&mut stop, 99_550_000_000),
        StopDecision::Stop(StopCause::NoReading { source: 0 })
    );
    stop.reading_arrived(&live, 0, 100_040_000_000)
        .expect("the live obstacles' one source");
    let last_live_ns = 99_600_000_000 + TIMEOUT_NS;
    for now_ns in [99_600_000_000, last_live_ns] {
        assert_eq!(decide_at(&mut stop, now_ns), moving, "at {now_ns} ns");
    }
    assert_stale(decide_at(&mut stop, last_live_ns + 1), 0);
}

// Each source's newest reading must have arrived within the source timeout: a source with
// no reading, and one whose newest reading came longer ago, stop the base whatever the
// cells; a reading exactly the timeout old still counts, as Nav2 compares the age.
#[test]
fn a_source_whose_readings_stop_arriving_stops_the_base() {
    let clear: [[f64; 2]; 0] = [];
    let moving = forward(0.5);
    let live = live_obstacles(footprint(), 2);
    let mut stop = fresh_stop();
    assert_eq!(
        stop.decide(&live, moving, &clear, NOW_NS),
        StopDecision::Stop(StopCause::NoReading { source: 0 })
    );
    stop.reading_arrived(&live, 1, NOW_NS)
        .expect("source 1 of 2");
    assert_eq!(
        stop.decide(&live, moving, &clear, NOW_NS),
        StopDecision::Stop(StopCause::NoReading { source: 0 })
    );
    stop.reading_arrived(&live, 0, NOW_NS)
        .expect("source 0 of 2");
    stop.reading_arrived(&live, 1, NOW_NS - TIMEOUT_NS)
        .expect("source 1 of 2");
    assert_eq!(
        stop.decide(&live, moving, &clear, NOW_NS),
        StopDecision::Continue { velocity: moving }
    );
    let stale = stop.decide(&live, moving, &clear, NOW_NS + 1);
    assert_stale(stale, 1);
    assert_eq!(stale.velocity(), Twist::ZERO);
}

// The stop gives back the source whose readings stopped arriving, the one a decision stops
// for, so a caller keeps no copy of the arrivals: the first with no reading, then one whose
// newest reading is older than the timeout, and none while each is within it, a reading
// exactly the timeout old among them.
#[test]
fn the_stop_gives_back_its_silent_source() {
    let live = live_obstacles(footprint(), 2);
    let mut stop = fresh_stop();
    assert_eq!(
        stop.silent_source(&live, NOW_NS),
        Some(StopCause::NoReading { source: 0 })
    );
    stop.reading_arrived(&live, 0, NOW_NS)
        .expect("source 0 of 2");
    assert_eq!(
        stop.silent_source(&live, NOW_NS),
        Some(StopCause::NoReading { source: 1 })
    );
    stop.reading_arrived(&live, 1, NOW_NS - TIMEOUT_NS)
        .expect("source 1 of 2");
    assert_eq!(stop.silent_source(&live, NOW_NS), None);
    let stale = stop.silent_source(&live, NOW_NS + 1);
    assert!(
        matches!(stale, Some(StopCause::StaleSource { source: 1, .. })),
        "{stale:?}"
    );
}

// The stop reads the sources of the live obstacles it is given: a reading of another is
// refused, naming the sources to give, and leaves the stop as it was.
#[test]
fn a_reading_of_a_source_the_live_obstacles_do_not_read_is_refused() {
    let live = live_obstacles(footprint(), 2);
    let mut stop = fresh_stop();
    let refused = stop.reading_arrived(&live, 2, NOW_NS).unwrap_err();
    assert_eq!(
        refused,
        UnknownSource {
            index: 2,
            sources: 2
        }
    );
    assert!(
        refused.to_string().contains("give a source from 0 to 1"),
        "{refused}"
    );
    assert_eq!(stop, fresh_stop());
}

// A simulation step longer than the time before collision reaches past it in one step, and
// one too short to advance every time of the projection never ends it: each makes no stop,
// the refusal naming both parameters. The refusal of a short step names the least step that
// advances each time, which the stop takes, the double below it refused, and the count of
// steps each decision projects. That holds for a time before collision whose last bit is
// odd too, which a step of half its spacing advances where the power of two below it stays.
#[test]
fn a_step_past_the_time_or_too_short_to_advance_it_is_refused() {
    let with = |step_s: f64, horizon_s: f64| {
        Stop::parse(StopParams {
            simulation_time_step_s: positive(step_s),
            time_before_collision_s: positive(horizon_s),
            ..params()
        })
    };
    let with_step = |step_s: f64| with(step_s, TIME_BEFORE_COLLISION);
    let past = with_step(1.5 * TIME_BEFORE_COLLISION).unwrap_err();
    assert_eq!(
        past,
        StopError::StepPastHorizon {
            step_s: 1.5 * TIME_BEFORE_COLLISION,
            horizon_s: TIME_BEFORE_COLLISION
        }
    );
    assert!(
        past.to_string()
            .contains("give a simulation_time_step_s of at most time_before_collision_s"),
        "{past}"
    );
    assert!(with_step(TIME_BEFORE_COLLISION).is_ok());
    let short = with_step(1e-17).unwrap_err();
    let StopError::StepTooShort { least_s, .. } = short else {
        panic!("{short:?}");
    };
    let message = short.to_string();
    assert!(
        message.contains(&format!("the least step that advances each is {least_s} s"))
            && message.contains(
                "one step more than the whole number of simulation_time_step_s in \
                 time_before_collision_s"
            ),
        "{short}"
    );
    for horizon_s in [TIME_BEFORE_COLLISION, 0.3] {
        let Err(StopError::StepTooShort { least_s, .. }) = with(1e-18, horizon_s) else {
            panic!("a step of 1e-18 s against {horizon_s} s");
        };
        assert!(with(least_s, horizon_s).is_ok(), "{horizon_s}");
        assert!(
            matches!(
                with(least_s.next_down(), horizon_s),
                Err(StopError::StepTooShort { .. })
            ),
            "{horizon_s}: {least_s}"
        );
    }
}
