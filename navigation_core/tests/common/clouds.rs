//! What the parity tests of the obstacle layer share: the live obstacles of a scenario's
//! grid, with voxels of the grid's cells, a cloud inserted into them, and the margins that
//! keep a point's cell and band from turning on the rounding of the frame it is judged in.

use std::num::NonZeroUsize;

use camera_core::pose::Pose;
use control_core::positive_finite::PositiveFinite;
use navigation_core::PaddedFootprint;
use navigation_core::grid::GridGeometry;
use navigation_core::obstacles::{LiveObstacles, ObstacleHeights, ObstacleHeightsParams};
use occupancy_map::{Bounds, Frame};
use ros_reference::{Grid, SensorTransform};

use super::footprints;
use super::poses::{on_base, planar};

/// How far, in meters, each of Nav2's points lies at least from a cell's edge and from the
/// band's ends: Nav2 transforms the points in single precision and the voxel map in double,
/// a difference far below it.
pub const EDGE_MARGIN_M: f64 = 1e-4;

/// How far below and above the floor, in meters, the voxel map of a cloud reaches.
const MAP_DEPTH_M: [f64; 2] = [-1.0, 3.0];

/// The step down of the parity scenes, two of their 5 cm voxels: Nav2's obstacle layer
/// marks no drop, and no point of the scenes lies below it.
const STEP_DOWN_M: f64 = 0.1;

/// The geometry of `grid`.
pub fn geometry_of(grid: &Grid) -> GridGeometry {
    GridGeometry::new(
        grid.size_x,
        grid.size_y,
        PositiveFinite::parse(grid.resolution).expect("a positive resolution"),
        grid.origin,
    )
    .expect("the scenario's grid")
}

/// How far, in meters, a value may lie from a whole number of steps and count as on one:
/// far inside `EDGE_MARGIN_M`, and past the rounding of a step that single precision
/// carries.
const ON_STEP_M: f64 = 1e-6;

/// Panics unless `value` lies within `ON_STEP_M` of a whole number of `step`s.
pub fn assert_on_step(value: f64, step: f64, what: &str) {
    let off = (value - (value / step).round() * step).abs();
    assert!(
        off <= ON_STEP_M,
        "{what}, {value}, lies {off} m from a whole number of {step} m steps"
    );
}

/// The heights of a scenario's band, its ends on boundaries of voxels of `voxel_m`, with
/// `STEP_DOWN_M`.
pub fn heights_of(
    min_obstacle_height: f64,
    max_obstacle_height: f64,
    voxel_m: f64,
) -> ObstacleHeights {
    ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: min_obstacle_height,
        max_obstacle_height_m: max_obstacle_height,
        max_step_down_m: STEP_DOWN_M,
        voxel_m: PositiveFinite::parse(voxel_m).expect("a positive voxel size"),
    })
    .expect("the scenario's band, of whole voxels")
}

/// The live obstacles of `padded_footprint` over `grid`, of voxels `voxel_m` on a side,
/// one column a cell of the grid, judged by `heights`.
pub fn live_obstacles_over(
    grid: &Grid,
    voxel_m: f64,
    heights: ObstacleHeights,
    padded_footprint: PaddedFootprint,
) -> LiveObstacles {
    assert_on_step(grid.origin[0], voxel_m, "the grid's origin");
    assert_on_step(grid.origin[1], voxel_m, "the grid's origin");
    let half = voxel_m / 2.0;
    let bounds = Bounds {
        min: [grid.origin[0] + half, grid.origin[1] + half, MAP_DEPTH_M[0]],
        max: [
            grid.origin[0] + (f64::from(grid.size_x) - 0.5) * voxel_m,
            grid.origin[1] + (f64::from(grid.size_y) - 0.5) * voxel_m,
            MAP_DEPTH_M[1],
        ],
    };
    let live = LiveObstacles::new(
        Frame::parse("map").expect("a frame name"),
        bounds,
        heights,
        padded_footprint,
        NonZeroUsize::MIN,
    )
    .expect("a map over the grid");
    let columns = live.columns();
    assert_eq!(
        (columns.size_x(), columns.size_y()),
        (grid.size_x, grid.size_y),
        "the map's columns are the grid's cells"
    );
    live
}

/// Inserts the points `points` that a sensor at `sensor` reads out to `range_m`, the base at
/// `base`, x, y and yaw.
pub fn insert_cloud(
    live: &mut LiveObstacles,
    sensor: &SensorTransform,
    points: &[[f32; 3]],
    base: [f64; 3],
    range_m: f64,
) {
    let pose = Pose::new(sensor.position, sensor.orientation).expect("the sensor's pose");
    let [x, y, yaw] = base;
    live.insert_depth_frame(
        0,
        points,
        planar(x, y, yaw),
        on_base(planar(x, y, yaw), &pose),
        PositiveFinite::parse(range_m).expect("a positive range"),
    )
    .expect("the live obstacles' one source");
}

/// Panics unless each of `points`, in the grid's frame, lies `EDGE_MARGIN_M` or more from
/// every edge of the grid's cells and from both ends of the band of `heights`, and above the
/// step down.
pub fn assert_clear_of_edges(
    what: &str,
    grid: &Grid,
    heights: ObstacleHeights,
    points: &[[f32; 3]],
) {
    let ends = [
        heights.min_obstacle_height_m(),
        heights.max_obstacle_height_m(),
    ];
    for (index, point) in points.iter().enumerate() {
        let point = point.map(f64::from);
        for axis in 0..2 {
            let offset = (point[axis] - grid.origin[axis]) / grid.resolution;
            let margin = (offset - offset.round()).abs() * grid.resolution;
            assert!(
                margin >= EDGE_MARGIN_M,
                "{what} point {index}, {point:?}, lies {margin} m from a cell's edge"
            );
        }
        for end in ends {
            assert!(
                (point[2] - end).abs() >= EDGE_MARGIN_M,
                "{what} point {index}, {point:?}, lies within {EDGE_MARGIN_M} m of the band's end"
            );
        }
        assert!(
            point[2] > -heights.max_step_down_m(),
            "{what} point {index}, {point:?}, lies below the step down, a drop Nav2 does not mark"
        );
    }
}

/// Panics unless our padding of `footprint` by `padding` is Nav2's `padded`, bit for bit.
pub fn padded_as_nav2(
    footprint: &[[f64; 2]],
    padding: f64,
    padded: &[[f64; 2]],
    what: &str,
) -> PaddedFootprint {
    let ours = footprints::padded(footprint, padding);
    let bits = |points: &[[f64; 2]]| -> Vec<[u64; 2]> {
        points.iter().map(|p| p.map(f64::to_bits)).collect()
    };
    assert_eq!(
        bits(ours.points()),
        bits(padded),
        "{what}: our padded footprint {:?}, Nav2's {padded:?}",
        ours.points()
    );
    ours
}
