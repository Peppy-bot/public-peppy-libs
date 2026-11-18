//! The live obstacles on drawn scenes: a table top above the band marks only its legs and
//! the floor marks no cell, while a box that rises into the band marks the cells under it;
//! a step down deeper than the limit marks where the sensor's rays to the lower floor
//! cross the floor's plane and a shallower one marks none; a forward camera driving toward
//! a drop marks the cells past its edge where its rays cross the floor's plane, before it
//! sees their floor, a shallow step marks none, a footprint cleared over the drop holds
//! none, recentered obstacles keep the drop and a clear leaves it out of the grid; a lone
//! wrong far reading in each frame marks no cell, nor do a fiftieth of a forward camera's
//! readings wrong and far before a box, spread evenly over each frame, and wrong far
//! readings placed at random mark a cell only where three of one frame cross its floor; a
//! base that sees its own deck as it drives and turns marks no cell; the footprint's
//! clearing leaves no occupied cell under it and every cell of a saved grid; and a laser
//! scan marks each hit where the sensor stood at its ray's time, a ray that met nothing
//! marking and freeing nothing.

mod common;

use std::collections::{BTreeMap, BTreeSet};
use std::num::NonZeroUsize;

use camera_core::laser_scan::{LaserScan, Sweep};
use camera_core::pose::{PlanarPose, Pose};
use common::footprints::{inside_by, padded};
use common::layers::{layer, lethal_cells};
use common::poses::{on_base, planar};
use control_core::positive_finite::PositiveFinite;
use frame_tree::FrameName;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::PaddedFootprint;
use navigation_core::costmap::compose;
use navigation_core::grid::{GridGeometry, LETHAL, UNKNOWN};
use navigation_core::obstacles::{LiveObstacles, ObstacleHeights, ObstacleHeightsParams};
use navigation_core::saved_grid::{MapThresholds, SavedGrid};
use occupancy_map::Bounds;

/// The side of a voxel and of a cell, in meters.
const VOXEL_M: f64 = 0.05;
/// The spacing of the points a surface shows, in meters: finer than a voxel.
const SPACING_M: f64 = 0.02;
/// How far inside a footprint's outline a cell's center lies for the outline's raster,
/// Bresenham's cells between its vertices' cells, to hold it: a cell's diagonal and half
/// a cell more.
const RASTER_MARGIN_M: f64 = 0.11;
/// Where the overhead sensor stands, above the scene, in the map's frame.
const SENSOR: [f64; 3] = [0.0, 0.0, 1.5];
/// The table top, from its low corner to its high one, in meters, its height drawn: each
/// edge a fifth of a cell inside a cell, so no rounding of a point on it changes its cell.
const TABLE: [[f64; 2]; 2] = [[0.61, -0.29], [1.19, 0.29]];
/// The table's four legs under the corners of its top, their edges inside cells as the
/// top's.
const LEGS: [[[f64; 2]; 2]; 4] = [
    [[0.61, -0.29], [0.69, -0.21]],
    [[1.11, -0.29], [1.19, -0.21]],
    [[0.61, 0.21], [0.69, 0.29]],
    [[1.11, 0.21], [1.19, 0.29]],
];
/// The box, from its low corner to its high one, in meters, its height drawn, its edges
/// inside cells as the table's.
const BOX: [[f64; 2]; 2] = [[-0.61, -0.81], [-0.29, -0.49]];
/// The farthest range a frame reads, past every point of these scenes.
const RANGE_M: f64 = 5.0;
/// A pose of the base off the map, where its footprint clears no column.
fn away() -> PlanarPose {
    planar(-10.0, 0.0, 0.0)
}

fn voxel() -> PositiveFinite {
    PositiveFinite::parse(VOXEL_M).expect("positive")
}

fn range() -> PositiveFinite {
    PositiveFinite::parse(RANGE_M).expect("positive")
}

/// The band from 0.1 m to 0.6 m and a step of 0.1 m, whole voxels.
fn heights() -> ObstacleHeights {
    ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: 0.1,
        max_obstacle_height_m: 0.6,
        max_step_down_m: 0.1,
        voxel_m: voxel(),
    })
    .expect("heights of whole voxels")
}

/// The odometry frame, which the live obstacles' map is in.
fn odom() -> FrameName {
    FrameName::parse("odom").expect("a frame name")
}

/// The bounds of the live obstacles' map: from 1.5 m behind and beside the origin to 2 m
/// ahead, and from 0.5 m below the floor to 2 m above it.
fn bounds() -> Bounds {
    Bounds {
        min: [-1.5, -1.5, -0.5],
        max: [2.0, 1.5, 2.0],
    }
}

/// The footprint of `half_x` by `half_y` meters either side of the base's center, padded
/// by `padding_m`.
fn rectangle(half_x: f64, half_y: f64, padding_m: f64) -> PaddedFootprint {
    padded(
        &[
            [-half_x, -half_y],
            [half_x, -half_y],
            [half_x, half_y],
            [-half_x, half_y],
        ],
        padding_m,
    )
}

/// A square footprint 0.2 m across, unpadded.
fn small_square() -> PaddedFootprint {
    rectangle(0.1, 0.1, 0.0)
}

/// The pose of a sensor at `position` turned by `yaw` about +Z.
fn sensor_at(position: [f64; 3], yaw: f64) -> Pose {
    let half = yaw / 2.0;
    Pose::new(position, [0.0, 0.0, half.sin(), half.cos()]).expect("a turn about +Z")
}

/// `points`, in the map's frame, in the frame of `sensor`, in single precision.
fn seen_from(sensor: &Pose, points: &[[f64; 3]]) -> Vec<[f32; 3]> {
    let to_sensor = sensor.inverse();
    points
        .iter()
        .map(|&p| to_sensor.apply(p).map(|c| c as f32))
        .collect()
}

/// Points every `SPACING_M` over the rectangle from `low` to `high` at height `z`.
fn surface(low: [f64; 2], high: [f64; 2], z: f64) -> Vec<[f64; 3]> {
    let steps = |a: f64, b: f64| ((b - a) / SPACING_M).floor() as usize;
    (0..=steps(low[0], high[0]))
        .flat_map(|i| (0..=steps(low[1], high[1])).map(move |j| (i, j)))
        .map(|(i, j)| {
            [
                low[0] + i as f64 * SPACING_M,
                low[1] + j as f64 * SPACING_M,
                z,
            ]
        })
        .collect()
}

/// The four sides of the rectangle from `low` to `high`, as rectangles of no width.
fn edges([low, high]: [[f64; 2]; 2]) -> [[[f64; 2]; 2]; 4] {
    [
        [[low[0], low[1]], [high[0], low[1]]],
        [[low[0], high[1]], [high[0], high[1]]],
        [[low[0], low[1]], [low[0], high[1]]],
        [[high[0], low[1]], [high[0], high[1]]],
    ]
}

/// The points of the four sides of `rectangle`, from the floor to `height`.
fn sides(rectangle: [[f64; 2]; 2], height: f64) -> Vec<[f64; 3]> {
    let levels = (height / SPACING_M).floor() as usize;
    (0..=levels)
        .flat_map(|k| {
            let z = k as f64 * SPACING_M;
            edges(rectangle)
                .into_iter()
                .flat_map(move |[low, high]| surface(low, high, z))
        })
        .collect()
}

/// The cells of `geometry` under the rectangle from `low` to `high`.
fn cells_under(geometry: GridGeometry, low: [f64; 2], high: [f64; 2]) -> BTreeSet<(u32, u32)> {
    surface(low, high, 0.0)
        .into_iter()
        .chain([
            [high[0], high[1], 0.0],
            [low[0], high[1], 0.0],
            [high[0], low[1], 0.0],
        ])
        .filter_map(|p| geometry.cell_of([p[0], p[1]]))
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}

/// The cells under the four sides of `rectangle`.
fn rim(geometry: GridGeometry, rectangle: [[f64; 2]; 2]) -> BTreeSet<(u32, u32)> {
    edges(rectangle)
        .into_iter()
        .flat_map(|[low, high]| cells_under(geometry, low, high))
        .collect()
}

// With the band's ends on voxel boundaries, a table top drawn above the band marks only
// the cells of its legs, the floor below the band marks none, and a box whose top lies in
// the band marks every cell under its sides and no other.
#[hegel::test(test_cases = 50)]
fn a_table_above_the_band_marks_only_its_legs_and_the_floor_none(tc: TestCase) {
    let heights = heights();
    let table_m = tc.draw(
        gs::floats()
            .min_value(heights.max_obstacle_height_m() + VOXEL_M)
            .max_value(1.2),
    );
    let box_m = tc.draw(
        gs::floats()
            .min_value(heights.min_obstacle_height_m() + VOXEL_M)
            .max_value(heights.max_obstacle_height_m() - VOXEL_M / 2.0),
    );
    let points: Vec<[f64; 3]> = surface([-1.0, -1.0], [1.5, 1.0], 0.0)
        .into_iter()
        .chain(surface(TABLE[0], TABLE[1], table_m))
        .chain(LEGS.into_iter().flat_map(|leg| sides(leg, table_m)))
        .chain(surface(BOX[0], BOX[1], box_m))
        .chain(sides(BOX, box_m))
        .collect();
    let mut live = LiveObstacles::new(odom(), bounds(), heights, small_square(), NonZeroUsize::MIN)
        .expect("whole voxels");
    let sensor = sensor_at(SENSOR, 0.0);
    live.insert_depth_frame(
        0,
        &seen_from(&sensor, &points),
        away(),
        on_base(away(), &sensor),
        range(),
    )
    .expect("the live obstacles' one source");

    let geometry = live.columns();
    let lethal = lethal_cells(&layer(&live));
    let under_legs: BTreeSet<(u32, u32)> = LEGS
        .iter()
        .flat_map(|[low, high]| cells_under(geometry, *low, *high))
        .collect();
    let under_top: BTreeSet<(u32, u32)> = cells_under(geometry, TABLE[0], TABLE[1])
        .difference(&under_legs)
        .copied()
        .collect();
    assert!(
        lethal.is_disjoint(&under_top),
        "the table top {table_m} m up marks {:?}",
        lethal.intersection(&under_top).collect::<Vec<_>>()
    );
    for leg in LEGS {
        assert!(
            rim(geometry, leg).is_subset(&lethal),
            "the leg {leg:?} marks its sides"
        );
    }
    assert!(
        rim(geometry, BOX).is_subset(&lethal),
        "the box {box_m} m up marks its sides"
    );
    let under_box = cells_under(geometry, BOX[0], BOX[1]);
    let marked: BTreeSet<(u32, u32)> = under_box.union(&under_legs).copied().collect();
    assert!(
        lethal.is_subset(&marked),
        "the floor or the table top marks {:?}",
        lethal.difference(&marked).collect::<Vec<_>>()
    );
}

// The floor drops by a drawn depth beyond an edge ahead of an overhead sensor, which sees
// each point of the lower floor that the edge does not hide, and the base drives down a
// drawn step, off the voxels. A depth a millimeter or more past the step marks every cell
// wholly inside where the sensor sees through the floor's plane to the lower floor, its
// rays to the lower floor crossing the plane there, and no cell wholly before the edge; a
// depth a millimeter or more short of the step marks no cell.
#[hegel::test(test_cases = 50)]
fn a_step_down_deeper_than_the_limit_marks_where_the_rays_cross_the_floor_and_a_shallower_one_none(
    tc: TestCase,
) {
    let step_m = tc.draw(gs::floats().min_value(0.03).max_value(0.2));
    let heights = ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: 0.1,
        max_obstacle_height_m: 0.6,
        max_step_down_m: step_m,
        voxel_m: voxel(),
    })
    .expect("a band of whole voxels");
    let deeper = tc.draw(gs::booleans());
    let depth_m = if deeper {
        tc.draw(
            gs::floats()
                .min_value(step_m + 0.001)
                .max_value(step_m + 0.15),
        )
    } else {
        tc.draw(gs::floats().min_value(0.0125).max_value(step_m - 0.001))
    };
    let edge = 0.41;
    // The lower floor's points lie a tenth of a cell off every cell edge, so no rounding
    // of a point moves its cell.
    let lower = [[edge + 0.005, -0.485], [1.385, 0.485]];
    let height = 1.0;
    let sensor_position = [0.0, 0.0, height];
    let step_down = StepDown {
        edge_x: edge,
        depth_m,
    };
    let seen_lower: Vec<[f64; 3]> = surface(lower[0], lower[1], -depth_m)
        .into_iter()
        .filter(|&point| step_down.sees(sensor_position, point))
        .collect();
    let points: Vec<[f64; 3]> = surface([-1.0, -1.0], [edge - SPACING_M, 1.0], 0.0)
        .into_iter()
        .chain(seen_lower.iter().copied())
        .collect();
    let mut live = LiveObstacles::new(odom(), bounds(), heights, small_square(), NonZeroUsize::MIN)
        .expect("whole voxels");
    let sensor = sensor_at(sensor_position, 0.0);
    live.insert_depth_frame(
        0,
        &seen_from(&sensor, &points),
        away(),
        on_base(away(), &sensor),
        range(),
    )
    .expect("the live obstacles' one source");
    let lethal = lethal_cells(&layer(&live));
    if !deeper {
        assert!(
            lethal.is_empty(),
            "a step down of {depth_m} m, within {step_m} m, marks {lethal:?}"
        );
        return;
    }
    // A ray from the sensor, above the map's origin, to a point of the lower floor crosses
    // the floor's plane at the point scaled toward the sensor by `to_floor`, the edge
    // bounding the crossings on the near side.
    let to_floor = height / (height + depth_m);
    let seen_floor = [
        [edge, lower[0][1] * to_floor],
        [lower[1][0] * to_floor, lower[1][1] * to_floor],
    ];
    let crossed = cells_within(live.columns(), seen_floor[0], seen_floor[1]);
    assert!(!crossed.is_empty());
    assert!(
        crossed.is_subset(&lethal),
        "a step down of {depth_m} m, past {step_m} m, leaves {:?} of the plane the sensor sees \
         through it unmarked",
        crossed.difference(&lethal).collect::<Vec<_>>()
    );
    let before_the_edge = cells_before(live.columns(), edge);
    assert!(
        lethal.is_disjoint(&before_the_edge),
        "a step down of {depth_m} m marks {:?} before the edge",
        lethal.intersection(&before_the_edge).collect::<Vec<_>>()
    );
}

/// The edge of the drop the overhead sensor sees: across the map at x = 0.41 m, the
/// lower floor 0.2 m down, past a step of 0.1 m.
const DROP_EDGE: StepDown = StepDown {
    edge_x: 0.41,
    depth_m: 0.2,
};

/// The live obstacles of an overhead sensor 1 m above the origin that sees a box in the band
/// and, beyond [`DROP_EDGE`], the lower floor.
fn a_box_before_a_drop() -> LiveObstacles {
    let sensor_position = [0.0, 0.0, 1.0];
    let lower: Vec<[f64; 3]> = surface([0.415, -0.485], [1.385, 0.485], -DROP_EDGE.depth_m)
        .into_iter()
        .filter(|&point| DROP_EDGE.sees(sensor_position, point))
        .collect();
    let box_m = 0.3;
    let points: Vec<[f64; 3]> = surface([-1.0, -1.0], [DROP_EDGE.edge_x - SPACING_M, 1.0], 0.0)
        .into_iter()
        .chain(lower)
        .chain(surface(BOX[0], BOX[1], box_m))
        .chain(sides(BOX, box_m))
        .collect();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        small_square(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let sensor = sensor_at(sensor_position, 0.0);
    live.insert_depth_frame(
        0,
        &seen_from(&sensor, &points),
        away(),
        on_base(away(), &sensor),
        range(),
    )
    .expect("the live obstacles' one source");
    live
}

/// The columns `live` marks on the map's own grid past [`DROP_EDGE`], the drop's.
fn drop_columns(live: &LiveObstacles) -> BTreeSet<(u32, u32)> {
    let columns = live.columns();
    lethal_cells(&layer(live))
        .into_iter()
        .filter(|&(x, _)| columns.origin()[0] + f64::from(x) * VOXEL_M >= DROP_EDGE.edge_x)
        .collect()
}

// An overhead sensor sees a box in the band and, beyond an edge, a floor 0.2 m down, past a
// step of 0.1 m. Projected onto a grid placed at a pose other than the map's, half a turn
// about it and moved by whole cells, the live obstacles mark each cell that the pose carries
// the center of a column marked on the map's own grid to, the box's and the drop's alike,
// and no other.
#[test]
fn a_grid_placed_at_a_pose_marks_the_cells_its_pose_carries_the_marked_columns_to() {
    let live = a_box_before_a_drop();
    let columns = live.columns();
    let marked = lethal_cells(&layer(&live));
    assert!(
        rim(columns, BOX).is_subset(&marked),
        "the box marks its sides"
    );
    assert!(
        !drop_columns(&live).is_empty(),
        "the drop marks a cell past the edge"
    );

    let grid_from_map = planar(0.3, -0.2, std::f64::consts::PI);
    let geometry = GridGeometry::new(80, 80, voxel(), [-2.0, -2.0]).expect("a grid");
    let carried: BTreeSet<(u32, u32)> = marked
        .iter()
        .map(|&(x, y)| {
            let column = columns.cell(x, y).expect("a column of the map");
            let cell = geometry
                .cell_of(grid_from_map.apply(columns.center_of(column)))
                .expect("the grid holds every column the pose carries");
            (cell.x(), cell.y())
        })
        .collect();
    let projected = live
        .project(grid_from_map, geometry)
        .expect("a finite placement");
    assert_eq!(lethal_cells(&projected), carried);
}

// The box before the drop, projected onto a grid of half a voxel's side turned a twelfth
// of a turn against the map: a drop's column marks every cell under its square as an
// obstacle's does, so every point within 0.024 m of a drop column's center on each axis,
// inside its square, falls in a lethal cell.
#[test]
fn a_drop_marks_every_cell_under_its_column_on_a_turned_finer_grid() {
    let live = a_box_before_a_drop();
    let columns = live.columns();
    let drops = drop_columns(&live);
    assert!(!drops.is_empty(), "the drop marks a cell past the edge");
    let grid_from_map = planar(0.0, 0.0, std::f64::consts::FRAC_PI_6);
    let half_a_voxel = PositiveFinite::parse(VOXEL_M / 2.0).expect("positive");
    let geometry = GridGeometry::new(240, 240, half_a_voxel, [-3.0, -3.0]).expect("a grid");
    let projected = live
        .project(grid_from_map, geometry)
        .expect("a finite placement");
    let offsets: Vec<f64> = (-12..=12).map(|k| f64::from(k) * 0.002).collect();
    let unmarked: Vec<[f64; 2]> = drops
        .iter()
        .flat_map(|&(x, y)| {
            let [center_x, center_y] =
                columns.center_of(columns.cell(x, y).expect("a column of the map"));
            let offsets = &offsets;
            offsets.iter().flat_map(move |&dx| {
                offsets
                    .iter()
                    .map(move |&dy| [center_x + dx, center_y + dy])
            })
        })
        .filter(|&point| {
            let cell = geometry
                .cell_of(grid_from_map.apply(point))
                .expect("the grid holds the scene");
            projected.grid().cost(cell) != LETHAL
        })
        .collect();
    assert!(unmarked.is_empty(), "the drop unmarked at {unmarked:?}");
}

// A wall one voxel thick and the floor beside it, seen from above, projected onto a grid
// turned a twelfth of a turn against the map, onto a grid of cells half a voxel's side,
// and onto a grid both turned and finer: a column marks every cell its square overlaps,
// the square a sliver in from the column's sides, 0.28 mm at these voxels, so every cell
// a point more than that sliver inside the wall's voxels falls in is lethal, here every
// point of the 1 mm lattice from 0.5 mm inside, cell (109, 138) of the turned finer grid
// among them, two thirds under one wall column and a third under the next, and every cell
// a point over the seen floor falls in is known.
#[test]
fn a_turned_or_finer_grid_marks_every_cell_under_a_wall_and_knows_the_seen_floor() {
    let points: Vec<[f64; 3]> = surface([-1.0, 0.51], [1.5, 0.54], 0.3)
        .into_iter()
        .chain(surface([-1.0, -1.0], [1.5, 0.45], 0.0))
        .collect();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        small_square(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let sensor = sensor_at(SENSOR, 0.0);
    live.insert_depth_frame(
        0,
        &seen_from(&sensor, &points),
        away(),
        on_base(away(), &sensor),
        range(),
    )
    .expect("the live obstacles' one source");
    let twelfth_of_a_turn = planar(0.0, 0.0, std::f64::consts::FRAC_PI_6);
    let half_a_voxel = PositiveFinite::parse(VOXEL_M / 2.0).expect("positive");
    let turned = (
        twelfth_of_a_turn,
        GridGeometry::new(120, 120, voxel(), [-3.0, -3.0]).expect("a grid"),
    );
    let finer = (
        PlanarPose::IDENTITY,
        GridGeometry::new(140, 120, half_a_voxel, [-1.5, -1.5]).expect("a grid"),
    );
    let turned_and_finer = (
        twelfth_of_a_turn,
        GridGeometry::new(240, 240, half_a_voxel, [-3.0, -3.0]).expect("a grid"),
    );
    let millimeters = |from: f64, count: u32| (0..count).map(move |k| from + f64::from(k) * 1e-3);
    for (grid_from_map, geometry) in [turned, finer, turned_and_finer] {
        let projected = live
            .project(grid_from_map, geometry)
            .expect("a finite placement");
        let cost_at = |point: [f64; 2]| {
            let cell = geometry
                .cell_of(grid_from_map.apply(point))
                .expect("the grid holds the scene");
            projected.grid().cost(cell)
        };
        let unmarked: Vec<[f64; 2]> = millimeters(-0.9995, 2500)
            .flat_map(|x| millimeters(0.5005, 49).map(move |y| [x, y]))
            .filter(|&point| cost_at(point) != LETHAL)
            .collect();
        assert!(
            unmarked.is_empty(),
            "{grid_from_map:?}: the wall unmarked at {unmarked:?}"
        );
        let unseen: Vec<[f64; 2]> = millimeters(-0.95, 2400)
            .step_by(10)
            .flat_map(|x| millimeters(-0.95, 1300).step_by(10).map(move |y| [x, y]))
            .filter(|&point| cost_at(point) == UNKNOWN)
            .collect();
        assert!(
            unseen.is_empty(),
            "{grid_from_map:?}: the seen floor unknown at {unseen:?}"
        );
    }
    let (grid_from_map, geometry) = turned_and_finer;
    let projected = live
        .project(grid_from_map, geometry)
        .expect("a finite placement");
    let between_two_columns = geometry.cell(109, 138).expect("a cell of the grid");
    assert_eq!(projected.grid().cost(between_two_columns), LETHAL);
}

/// The floor up to an edge across the map at x = `edge_x`, and a lower floor `depth_m`
/// below it beyond, as a sensor before the edge sees them: the edge hides the lower floor
/// nearest it, and the drop's wall faces away from the sensor.
struct StepDown {
    edge_x: f64,
    depth_m: f64,
}

impl StepDown {
    /// Where the ray from `origin`, before the edge, along `direction` meets a floor: the
    /// upper floor before the edge, else the lower floor, and none for a ray that does not
    /// point down.
    fn hit(&self, origin: [f64; 3], direction: [f64; 3]) -> Option<[f64; 3]> {
        if direction[2] >= 0.0 {
            return None;
        }
        let along = |t: f64| std::array::from_fn(|i| origin[i] + t * direction[i]);
        let on_upper: [f64; 3] = along(-origin[2] / direction[2]);
        if on_upper[0] < self.edge_x {
            return Some(on_upper);
        }
        Some(along((-self.depth_m - origin[2]) / direction[2]))
    }

    /// Whether a sensor at `origin` sees `point` of the lower floor: the ray to it meets
    /// no floor before.
    fn sees(&self, origin: [f64; 3], point: [f64; 3]) -> bool {
        let direction = std::array::from_fn(|i| point[i] - origin[i]);
        self.hit(origin, direction)
            .is_some_and(|hit| (hit[2] - point[2]).abs() < 1e-9)
    }
}

/// The cells of `geometry` that hold one of `points`.
fn cells_of(geometry: GridGeometry, points: &[[f64; 3]]) -> BTreeSet<(u32, u32)> {
    points
        .iter()
        .filter_map(|p| geometry.cell_of([p[0], p[1]]))
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}

/// The cells of `geometry` wholly inside the rectangle from `low` to `high`.
fn cells_within(geometry: GridGeometry, low: [f64; 2], high: [f64; 2]) -> BTreeSet<(u32, u32)> {
    let side = geometry.resolution_m();
    geometry
        .cells()
        .filter(|&cell| {
            let corner = [cell.x(), cell.y()]
                .map(f64::from)
                .map(|index| index * side);
            (0..2).all(|axis| {
                let start = geometry.origin()[axis] + corner[axis];
                start >= low[axis] && start + side <= high[axis]
            })
        })
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}

/// The cells of `geometry` wholly before x = `edge_x`.
fn cells_before(geometry: GridGeometry, edge_x: f64) -> BTreeSet<(u32, u32)> {
    let side = geometry.resolution_m();
    geometry
        .cells()
        .filter(|cell| geometry.origin()[0] + f64::from(cell.x() + 1) * side <= edge_x)
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}

/// A forward depth camera on a base: 0.3 m up at the base's front, 0.2 m ahead of its
/// center, pitched 10 degrees down, its field 36.9 degrees either side of its axis up and
/// down and 30 degrees either side across, a ray every 0.45 degree up and down and every
/// degree across, so the field's edges hold rays.
const CAMERA_AHEAD_M: f64 = 0.2;
const CAMERA_HEIGHT_M: f64 = 0.3;
const CAMERA_PITCH_DEG: f64 = 10.0;
const CAMERA_HALF_FIELD_DEG: [f64; 2] = [36.9, 30.0];
const CAMERA_RAY_STEP_DEG: [f64; 2] = [0.45, 1.0];

/// The camera's position with the base's center at `base_x` on the x axis, facing +x.
fn camera_at(base_x: f64) -> [f64; 3] {
    [base_x + CAMERA_AHEAD_M, 0.0, CAMERA_HEIGHT_M]
}

/// The base at `base_x` on the x axis, facing +x.
fn base_at(base_x: f64) -> PlanarPose {
    planar(base_x, 0.0, 0.0)
}

/// The directions of the camera's rays, in the map's frame, the base facing +x.
fn camera_rays() -> Vec<[f64; 3]> {
    let [down, across] =
        [0, 1].map(|axis| (CAMERA_HALF_FIELD_DEG[axis] / CAMERA_RAY_STEP_DEG[axis]).round() as i32);
    (-down..=down)
        .flat_map(|i| (-across..=across).map(move |j| (i, j)))
        .map(|(i, j)| {
            let elevation = (f64::from(i) * CAMERA_RAY_STEP_DEG[0] - CAMERA_PITCH_DEG).to_radians();
            let azimuth = (f64::from(j) * CAMERA_RAY_STEP_DEG[1]).to_radians();
            [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ]
        })
        .collect()
}

/// The base's footprint, 0.4 m long and 0.3 m wide around its center, unpadded.
fn drive_footprint() -> PaddedFootprint {
    rectangle(0.2, 0.15, 0.0)
}

/// What the camera shows in each frame of a drive along +x toward the edge of
/// `step_down`, the camera from 1.5 m before the edge until it stands 0.14 m before it, a
/// frame every 2 cm, with the live obstacles after each frame and the points of the
/// lower floor its frames have shown so far.
fn drive_toward(
    step_down: &StepDown,
    heights: ObstacleHeights,
) -> Vec<(LiveObstacles, Vec<[f64; 3]>)> {
    let rays = camera_rays();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights,
        drive_footprint(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let mut lower_seen = Vec::new();
    let frames = 68;
    (0..=frames)
        .map(|frame| {
            let base_x = step_down.edge_x - CAMERA_AHEAD_M - 1.5 + f64::from(frame) * 0.02;
            let camera = camera_at(base_x);
            let points: Vec<[f64; 3]> = rays
                .iter()
                .filter_map(|&direction| step_down.hit(camera, direction))
                .collect();
            lower_seen.extend(points.iter().filter(|p| p[2] < 0.0).copied());
            let sensor = sensor_at(camera, 0.0);
            let base = base_at(base_x);
            live.insert_depth_frame(
                0,
                &seen_from(&sensor, &points),
                base,
                on_base(base, &sensor),
                range(),
            )
            .expect("the live obstacles' one source");
            (live.clone(), lower_seen.clone())
        })
        .collect()
}

/// The edge of the drives' floors: a fifth of a cell into a cell.
const DRIVE_EDGE_X: f64 = 1.01;

// The forward camera drives toward an edge where the floor drops 0.25 m, past a step of
// 0.1 m. In no frame does a cell wholly before the edge mark. Rays that pass over the edge
// cross the floor's plane just past it and read past the step's plane, marking the cells
// past the edge before any of their own lower floor shows: the nearest marked cell begins
// less than a cell past the edge.
#[test]
fn a_forward_camera_marks_the_cells_past_an_edge_before_it_sees_the_lower_floor() {
    let step_down = StepDown {
        edge_x: DRIVE_EDGE_X,
        depth_m: 0.25,
    };
    let frames = drive_toward(&step_down, heights());
    let mut marked_unseen = false;
    for (frame, (live, lower_seen)) in frames.iter().enumerate() {
        let lethal = lethal_cells(&layer(live));
        let before_the_edge = cells_before(live.columns(), DRIVE_EDGE_X);
        assert!(
            lethal.is_disjoint(&before_the_edge),
            "frame {frame} marks {:?} before the edge",
            lethal.intersection(&before_the_edge).collect::<Vec<_>>()
        );
        let seen = cells_of(live.columns(), lower_seen);
        marked_unseen |= lethal.difference(&seen).next().is_some();
    }
    assert!(
        marked_unseen,
        "no frame marks a cell whose lower floor it has not shown"
    );
    let (last, _) = frames.last().expect("frames");
    let geometry = last.columns();
    let nearest_m = lethal_cells(&layer(last))
        .iter()
        .map(|&(x, _)| geometry.origin()[0] + f64::from(x) * geometry.resolution_m())
        .fold(f64::INFINITY, f64::min)
        - DRIVE_EDGE_X;
    assert!(
        nearest_m < VOXEL_M,
        "the nearest marked cell begins {nearest_m} m past the edge, a cell or more"
    );
}

// The same drive toward a step of 0.075 m, half a voxel short of the 0.1 m step the base
// drives down, marks no cell in any frame.
#[test]
fn a_forward_camera_marks_no_cell_at_a_shallow_step() {
    let step_down = StepDown {
        edge_x: DRIVE_EDGE_X,
        depth_m: 0.075,
    };
    for (frame, (live, _)) in drive_toward(&step_down, heights()).iter().enumerate() {
        let lethal = lethal_cells(&layer(live));
        assert!(lethal.is_empty(), "frame {frame} marks {lethal:?}");
    }
}

// After the drive toward the 0.25 m drop, a frame with the base standing past the edge
// clears the drop under its footprint: no cell under it is lethal, and the drop's cells
// beyond it stay lethal.
#[test]
fn a_cleared_footprint_holds_no_drop() {
    let step_down = StepDown {
        edge_x: DRIVE_EDGE_X,
        depth_m: 0.25,
    };
    let (mut live, _) = drive_toward(&step_down, heights()).pop().expect("frames");
    let base_x = DRIVE_EDGE_X + 0.4;
    let base = base_at(base_x);
    let sensor = sensor_at(camera_at(base_x), 0.0);
    live.insert_depth_frame(0, &[], base, on_base(base, &sensor), range())
        .expect("the live obstacles' one source");
    let lethal = lethal_cells(&layer(&live));
    let geometry = live.columns();
    let under = cells_under(
        geometry,
        [base_x - 0.2 + RASTER_MARGIN_M, -0.15 + RASTER_MARGIN_M],
        [base_x + 0.2 - RASTER_MARGIN_M, 0.15 - RASTER_MARGIN_M],
    );
    assert!(!under.is_empty());
    assert!(
        lethal.is_disjoint(&under),
        "the footprint holds the drop's cells {:?}",
        lethal.intersection(&under).collect::<Vec<_>>()
    );
    let beyond = cells_under(geometry, [base_x + 0.3, -0.1], [base_x + 0.4, 0.1]);
    assert!(
        beyond.is_subset(&lethal),
        "the drop's cells {:?} beyond the footprint are unmarked",
        beyond.difference(&lethal).collect::<Vec<_>>()
    );
}

// After the drive toward the 0.25 m drop, the live obstacles recentered by whole cells mark
// the same places the new bounds hold, and a clear leaves no drop in the grid.
#[test]
fn recentered_obstacles_keep_their_drops_and_a_clear_leaves_none_in_the_grid() {
    let step_down = StepDown {
        edge_x: DRIVE_EDGE_X,
        depth_m: 0.25,
    };
    let (mut live, _) = drive_toward(&step_down, heights()).pop().expect("frames");
    let places = |live: &LiveObstacles| -> BTreeSet<[i64; 2]> {
        let geometry = live.columns();
        lethal_cells(&layer(live))
            .into_iter()
            .map(|(x, y)| {
                let cell = geometry.cell(x, y).expect("a cell of the grid");
                geometry
                    .center_of(cell)
                    .map(|c| (c * 1000.0).round() as i64)
            })
            .collect()
    };
    let before = places(&live);
    assert!(!before.is_empty(), "the drive marks the drop");
    let moved = live
        .recentered(Bounds {
            min: [-1.2, -1.7, -0.5],
            max: [1.6, 1.3, 2.0],
        })
        .expect("bounds the map holds");
    let held = |place: [i64; 2]| {
        let center = place.map(|c| c as f64 / 1000.0);
        moved.columns().cell_of(center).is_some()
    };
    let kept: BTreeSet<[i64; 2]> = before
        .iter()
        .copied()
        .filter(|&place| held(place))
        .collect();
    assert!(
        kept.len() < before.len(),
        "the new bounds drop part of the drop"
    );
    assert_eq!(places(&moved), kept);
    live.clear();
    assert!(
        lethal_cells(&layer(&live)).is_empty(),
        "a clear leaves a lethal cell in the grid"
    );
}

// Over a flat floor, one drawn ray of each frame of the forward camera, the base at a
// drawn place, reads wrong and far, a drawn 1.05 to 10 times its distance: no frame marks
// a cell, as the wrong ray is the one ray of its frame past the step's plane.
#[hegel::test(test_cases = 50)]
fn a_lone_wrong_far_reading_in_each_frame_marks_no_cell(tc: TestCase) {
    let floor = StepDown {
        edge_x: f64::INFINITY,
        depth_m: 0.25,
    };
    let rays = camera_rays();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        drive_footprint(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    for frame in 0..tc.draw(gs::integers::<u32>().min_value(1).max_value(5)) {
        let base_x = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
        let camera = camera_at(base_x);
        let hits: Vec<[f64; 3]> = rays
            .iter()
            .filter_map(|&direction| floor.hit(camera, direction))
            .collect();
        let wrong = tc.draw(gs::integers::<usize>().max_value(hits.len() - 1));
        let factor = tc.draw(gs::floats().min_value(1.05).max_value(10.0));
        let points: Vec<[f64; 3]> = hits
            .iter()
            .enumerate()
            .map(|(index, &hit)| {
                if index == wrong {
                    std::array::from_fn(|i| camera[i] + factor * (hit[i] - camera[i]))
                } else {
                    hit
                }
            })
            .collect();
        let sensor = sensor_at(camera, 0.0);
        let base = base_at(base_x);
        live.insert_depth_frame(
            0,
            &seen_from(&sensor, &points),
            base,
            on_base(base, &sensor),
            range(),
        )
        .expect("the live obstacles' one source");
        let lethal = lethal_cells(&layer(&live));
        assert!(lethal.is_empty(), "frame {frame} marks {lethal:?}");
    }
}

/// How far along `direction` from `origin` a ray enters the box over the rectangle from
/// `low` to `high`, from the floor up to `height`, when it meets it ahead.
fn box_entry(
    origin: [f64; 3],
    direction: [f64; 3],
    [low, high]: [[f64; 2]; 2],
    height: f64,
) -> Option<f64> {
    let (low, high) = ([low[0], low[1], 0.0], [high[0], high[1], height]);
    let (near, far) = (0..3).try_fold((0.0_f64, f64::INFINITY), |(near, far), axis| {
        if direction[axis] == 0.0 {
            return (low[axis]..=high[axis])
                .contains(&origin[axis])
                .then_some((near, far));
        }
        let [a, b] = [low[axis], high[axis]].map(|bound| (bound - origin[axis]) / direction[axis]);
        Some((near.max(a.min(b)), far.min(a.max(b))))
    })?;
    (near <= far).then_some(near)
}

/// The box in the band before which the forward camera drives over a flat floor, from its
/// low corner to its high one, its edges inside cells as the table's, and its height, in
/// meters.
const FLAT_DRIVE_BOX: [[f64; 2]; 2] = [[1.21, -0.19], [1.39, 0.19]];
const FLAT_DRIVE_BOX_HEIGHT_M: f64 = 0.3;

/// How far along the ray from `camera` along the unit `direction` its line crosses the
/// floor's plane, and how far along it the ray meets the box or that floor first; none for
/// a ray that does not point down.
fn toward_the_box_over_a_flat_floor(camera: [f64; 3], direction: [f64; 3]) -> Option<(f64, f64)> {
    if direction[2] >= 0.0 {
        return None;
    }
    let floor_t = -camera[2] / direction[2];
    let t = box_entry(camera, direction, FLAT_DRIVE_BOX, FLAT_DRIVE_BOX_HEIGHT_M)
        .map_or(floor_t, |box_t| box_t.min(floor_t));
    Some((floor_t, t))
}

/// The point `t` along the unit `direction` from `origin`.
fn along(origin: [f64; 3], direction: [f64; 3], t: f64) -> [f64; 3] {
    std::array::from_fn(|i| origin[i] + t * direction[i])
}

/// One ray in this many of the forward camera's reads wrong and far: a fiftieth.
const WRONG_EVERY: usize = 50;

// The forward camera drives 1 m over a flat floor toward a box in the band while a
// fiftieth of its rays, spread evenly over each frame, read wrong and far, two to five
// times the distance at which their line meets the floor and some beyond the range, below
// the floor where the box hides it as where it shows: no frame marks a cell but the box's,
// the wrong readings fewer than three through each column, or outnumbered there by the
// floor the frame sees in it and the rays that see the box above the floor.
#[test]
fn wrong_far_readings_among_a_forward_cameras_rays_mark_no_drop() {
    let rays = camera_rays();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        drive_footprint(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let box_cells = cells_under(live.columns(), FLAT_DRIVE_BOX[0], FLAT_DRIVE_BOX[1]);
    for frame in 0..=50_usize {
        let base_x = -0.5 + frame as f64 * 0.02;
        let camera = camera_at(base_x);
        let points: Vec<[f64; 3]> = rays
            .iter()
            .enumerate()
            .filter_map(|(index, &direction)| {
                let (floor_t, t) = toward_the_box_over_a_flat_floor(camera, direction)?;
                let wrong = (frame * rays.len() + index).is_multiple_of(WRONG_EVERY);
                let t = if wrong {
                    floor_t * (2.0 + (index % 4) as f64)
                } else {
                    t
                };
                Some(along(camera, direction, t))
            })
            .collect();
        let sensor = sensor_at(camera, 0.0);
        live.insert_depth_frame(
            0,
            &seen_from(&sensor, &points),
            base_at(base_x),
            on_base(base_at(base_x), &sensor),
            range(),
        )
        .expect("the live obstacles' one source");
        let lethal = lethal_cells(&layer(&live));
        assert!(
            lethal.is_subset(&box_cells),
            "frame {frame} marks {:?} off the box",
            lethal.difference(&box_cells).collect::<Vec<_>>()
        );
    }
}

/// The columns of `geometry` that at least three of `crossings` lie within `rounding_m` of,
/// each crossing counted once in a column.
fn columns_crossed_by_three(
    geometry: GridGeometry,
    crossings: &[[f64; 2]],
    rounding_m: f64,
) -> BTreeSet<(u32, u32)> {
    let crossings_per_column = crossings
        .iter()
        .flat_map(|&[x, y]| {
            [[-1.0, -1.0], [-1.0, 1.0], [1.0, -1.0], [1.0, 1.0]]
                .into_iter()
                .filter_map(|[sx, sy]| geometry.cell_of([x + sx * rounding_m, y + sy * rounding_m]))
                .map(|cell| (cell.x(), cell.y()))
                .collect::<BTreeSet<_>>()
        })
        .fold(BTreeMap::new(), |mut counts, column| {
            *counts.entry(column).or_insert(0_u32) += 1;
            counts
        });
    crossings_per_column
        .into_iter()
        .filter(|&(_, count)| count >= 3)
        .map(|(column, _)| column)
        .collect()
}

// The forward camera, the base at drawn places before a box in the band on a flat floor,
// reads a drawn share of its rays wrong and far, each at a depth drawn from where its line
// crosses the floor's plane out to the range, below the floor where the box hides it as
// where it shows, the wrong rays placed at random: a cell off the box marks only in a
// column where at least three wrong rays of one frame cross the floor's plane, so one or
// two wrong far readings through a column mark none wherever they fall.
#[hegel::test(test_cases = 50)]
fn drawn_wrong_far_readings_mark_a_drop_only_where_three_of_one_frame_cross(tc: TestCase) {
    // How far from a column a crossing of the floor's plane may lie and count as crossing
    // it, in meters: a reading just past the step at the range moves its crossing by up to
    // 0.3 x 5 / 0.4^2, 9.4 times its height's single-precision rounding there, 2.4e-7 m.
    const CROSSING_ROUNDING_M: f64 = 1e-5;
    let rays = camera_rays();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        drive_footprint(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let columns = live.columns();
    let box_cells = cells_under(columns, FLAT_DRIVE_BOX[0], FLAT_DRIVE_BOX[1]);
    let wrong_share = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    let mut crossed_by_three = BTreeSet::new();
    for frame in 0..tc.draw(gs::integers::<u32>().min_value(1).max_value(5)) {
        let base_x = tc.draw(gs::floats().min_value(-0.5).max_value(0.5));
        let camera = camera_at(base_x);
        let readings: Vec<([f64; 3], Option<[f64; 2]>)> = rays
            .iter()
            .filter_map(|&direction| {
                let (floor_t, t) = toward_the_box_over_a_flat_floor(camera, direction)?;
                let wrong = floor_t < RANGE_M
                    && tc.draw(gs::floats().min_value(0.0).max_value(1.0)) < wrong_share;
                if !wrong {
                    return Some((along(camera, direction, t), None));
                }
                let depth_share: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
                let [x, y, _] = along(camera, direction, floor_t);
                Some((
                    along(
                        camera,
                        direction,
                        floor_t + depth_share * (RANGE_M - floor_t),
                    ),
                    Some([x, y]),
                ))
            })
            .collect();
        let points: Vec<[f64; 3]> = readings.iter().map(|&(point, _)| point).collect();
        let crossings: Vec<[f64; 2]> = readings.iter().filter_map(|&(_, at)| at).collect();
        let sensor = sensor_at(camera, 0.0);
        live.insert_depth_frame(
            0,
            &seen_from(&sensor, &points),
            base_at(base_x),
            on_base(base_at(base_x), &sensor),
            range(),
        )
        .expect("the live obstacles' one source");
        crossed_by_three.extend(columns_crossed_by_three(
            columns,
            &crossings,
            CROSSING_ROUNDING_M,
        ));
        let unexplained: Vec<_> = lethal_cells(&layer(&live))
            .into_iter()
            .filter(|cell| !box_cells.contains(cell) && !crossed_by_three.contains(cell))
            .collect();
        assert!(
            unexplained.is_empty(),
            "frame {frame} marks {unexplained:?} off the box, where no three wrong rays of \
             one frame cross the floor's plane"
        );
    }
}

// A box in the band that ten frames saw, cleared by the footprint of a base standing over
// it, marks no cell; the next frame that sees it, the base away, marks its cells again, as
// Nav2's obstacle layer marks a cleared cell on the next frame.
#[test]
fn a_cleared_obstacle_is_marked_again_by_the_next_frame() {
    let box_ = [[0.31, -0.09], [0.49, 0.09]];
    let points = surface(box_[0], box_[1], 0.3);
    let sensor = sensor_at(SENSOR, 0.0);
    let frame = seen_from(&sensor, &points);
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        small_square(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let insert = |live: &mut LiveObstacles, points: &[[f32; 3]], base: PlanarPose| {
        live.insert_depth_frame(0, points, base, on_base(base, &sensor), range())
            .expect("the live obstacles' one source")
    };
    for _ in 0..10 {
        insert(&mut live, &frame, away());
    }
    let under = cells_under(live.columns(), box_[0], box_[1]);
    assert!(under.is_subset(&lethal_cells(&layer(&live))));
    let over_the_box = planar(0.4, 0.0, 0.0);
    insert(&mut live, &[], over_the_box);
    assert!(lethal_cells(&layer(&live)).is_disjoint(&under));
    insert(&mut live, &frame, away());
    assert!(
        under.is_subset(&lethal_cells(&layer(&live))),
        "one frame leaves {:?} of the box unmarked",
        under
            .difference(&lethal_cells(&layer(&live)))
            .collect::<Vec<_>>()
    );
}

// A sensor on the base, above its deck, sees the deck in every frame while the base drives
// 1 m and then turns in place, its footprint padded by a voxel or more: the clearing under
// the padded footprint after each frame leaves no occupied cell.
#[hegel::test(test_cases = 30)]
fn a_base_that_sees_its_own_deck_as_it_drives_and_turns_marks_no_cell(tc: TestCase) {
    let half_x = tc.draw(gs::floats().min_value(0.15).max_value(0.3));
    let half_y = tc.draw(gs::floats().min_value(0.1).max_value(0.25));
    let padding = tc.draw(gs::floats().min_value(VOXEL_M).max_value(0.08));
    let footprint = rectangle(half_x, half_y, padding);
    let deck_m = tc.draw(gs::floats().min_value(0.12).max_value(0.5));
    let mount_m = deck_m + tc.draw(gs::floats().min_value(0.1).max_value(0.4));
    let deck: Vec<[f64; 2]> = surface(
        [-half_x + SPACING_M, -half_y + SPACING_M],
        [half_x - SPACING_M, half_y - SPACING_M],
        0.0,
    )
    .into_iter()
    .map(|p| [p[0], p[1]])
    .collect();
    let drive_steps = tc.draw(gs::integers::<u32>().min_value(5).max_value(20));
    let turn_steps = tc.draw(gs::integers::<u32>().min_value(4).max_value(16));
    let turn_rad = tc.draw(gs::floats().min_value(1.0).max_value(std::f64::consts::TAU));
    let start = planar(
        -0.5,
        tc.draw(gs::floats().min_value(-0.3).max_value(0.3)),
        0.0,
    );
    let driving = (0..=drive_steps).map(|k| {
        planar(
            start.x() + f64::from(k) / f64::from(drive_steps),
            start.y(),
            start.yaw(),
        )
    });
    let turning = (1..=turn_steps).map(|k| {
        planar(
            start.x() + 1.0,
            start.y(),
            turn_rad * f64::from(k) / f64::from(turn_steps),
        )
    });
    let mut live = LiveObstacles::new(odom(), bounds(), heights(), footprint, NonZeroUsize::MIN)
        .expect("whole voxels");
    for base in driving.chain(turning) {
        let sensor = sensor_at([base.x(), base.y(), mount_m], base.yaw());
        let seen: Vec<[f64; 3]> = deck
            .iter()
            .map(|&point| {
                let [x, y] = base.apply(point);
                [x, y, deck_m]
            })
            .collect();
        live.insert_depth_frame(
            0,
            &seen_from(&sensor, &seen),
            base,
            on_base(base, &sensor),
            range(),
        )
        .expect("the live obstacles' one source");
        let marked = lethal_cells(&layer(&live));
        assert!(
            marked.is_empty(),
            "the deck {deck_m} m up marks {marked:?} with the base at {base:?}"
        );
    }
}

// Obstacles drawn in the band around a base drawn anywhere on the map: after the frame,
// no cell under the padded footprint is lethal, and over a drawn saved grid every lethal
// cell of it stays lethal in the composed grid, under the footprint too. A cell counts as
// under the footprint when its center lies inside the outline by more than the outline's
// raster can miss it by.
#[hegel::test(test_cases = 50)]
fn the_footprint_clearing_leaves_no_occupied_cell_under_it_and_every_saved_cell(tc: TestCase) {
    let footprint = rectangle(
        tc.draw(gs::floats().min_value(0.15).max_value(0.4)),
        tc.draw(gs::floats().min_value(0.15).max_value(0.4)),
        tc.draw(gs::floats().min_value(0.0).max_value(0.05)),
    );
    let base = planar(
        tc.draw(gs::floats().min_value(-0.5).max_value(0.5)),
        tc.draw(gs::floats().min_value(-0.5).max_value(0.5)),
        tc.draw(gs::floats().min_value(-3.2).max_value(3.2)),
    );
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(200));
    let points: Vec<[f64; 3]> = (0..count)
        .map(|_| {
            [
                tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
                tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
                tc.draw(gs::floats().min_value(0.2).max_value(0.4)),
            ]
        })
        .collect();
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        footprint.clone(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let sensor = sensor_at(SENSOR, 0.0);
    live.insert_depth_frame(
        0,
        &seen_from(&sensor, &points),
        base,
        on_base(base, &sensor),
        range(),
    )
    .expect("the live obstacles' one source");

    let columns = live.columns();
    let (width, height) = (columns.size_x(), columns.size_y());
    let pixels: Vec<u8> = (0..width * height)
        .map(|_| tc.draw(gs::sampled_from(&[0_u8, 254, 205])))
        .collect();
    let saved = SavedGrid::from_image(
        width,
        height,
        &pixels,
        PositiveFinite::parse(VOXEL_M).expect("positive"),
        columns.origin(),
        MapThresholds::parse(0.65, 0.196).expect("map_saver's thresholds"),
    )
    .expect("a saved grid over the map's columns");
    let geometry = saved.grid().geometry();
    let obstacles = live
        .project(PlanarPose::IDENTITY, geometry)
        .expect("a finite placement");
    let placed = footprint.placed_at(base);
    for cell in geometry.cells() {
        if inside_by(&placed, geometry.center_of(cell), RASTER_MARGIN_M) {
            assert_ne!(
                obstacles.grid().cost(cell),
                LETHAL,
                "{cell:?} under the footprint at {base:?}"
            );
        }
    }
    let grid = compose(Some(&saved), &obstacles, None).expect("one grid");
    for cell in geometry.cells() {
        if saved.grid().cost(cell) == LETHAL {
            assert_eq!(grid.cost(cell), LETHAL, "the saved cell {cell:?}");
        }
    }
}

/// A 2-D lidar 0.3 m up on its base, in the band.
fn lidar_on_base() -> Pose {
    sensor_at([0.0, 0.0, 0.3], 0.0)
}

/// A base that stands at `base` through a sweep.
fn standing(base: PlanarPose) -> Sweep<PlanarPose> {
    Sweep {
        first_ray: base,
        last_ray: base,
    }
}

// A scan whose rays all carry lidar_2d's marks, one that met nothing, one not measured and
// one past the farthest range, marks and frees nothing; one hit among such rays marks its
// own cell alone.
#[test]
fn a_ray_that_met_nothing_frees_and_marks_nothing() {
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        small_square(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let base = standing(away());
    let lidar = on_base(away(), &sensor_at([-0.5, 0.0, 0.3], 0.0));
    let marks =
        LaserScan::parse(-0.1, 0.1, 0.0, 0.05, 8.0, &[10000.0, 0.0, 9.0]).expect("a scan of marks");
    live.insert_laser_scan(0, &marks, base, lidar, range())
        .expect("the live obstacles' one source");
    assert!(
        layer(&live)
            .grid()
            .costs()
            .iter()
            .all(|&cost| cost == UNKNOWN),
        "a scan of marks makes a cell known"
    );
    let one_hit = LaserScan::parse(-0.1, 0.1, 0.0, 0.05, 8.0, &[10000.0, 1.012, 0.0])
        .expect("a scan of one hit");
    live.insert_laser_scan(0, &one_hit, base, lidar, range())
        .expect("the live obstacles' one source");
    let hit = live
        .columns()
        .cell_of([0.512, 0.0])
        .expect("the hit lies on the map");
    assert_eq!(
        lethal_cells(&layer(&live)),
        BTreeSet::from([(hit.x(), hit.y())])
    );
}

// A lidar on a base that turns by 0.4 rad while the lidar sweeps a wall 1.012 m ahead: each
// ray's range is the distance along the direction it pointed at its time, and each hit
// marks the cell of the wall that ray met, where the sensor stood at the ray's time. The
// wall's points lie at least 1e-4 m from every cell edge, so rounding moves none.
#[test]
fn each_ray_of_a_turning_scan_lands_where_the_sensor_stood_at_its_time() {
    let (wall_x, lidar_x, rays, turn) = (0.512, -0.5, 21_usize, 0.4);
    let (angle_min, increment) = (-0.5_f64, 0.05_f64);
    let distance = wall_x - lidar_x;
    let pointing =
        |i: usize| angle_min + i as f64 * increment + turn * i as f64 / (rays - 1) as f64;
    let ranges: Vec<f32> = (0..rays)
        .map(|i| (distance / pointing(i).cos()) as f32)
        .collect();
    let scan = LaserScan::parse(
        angle_min as f32,
        increment as f32,
        0.005,
        0.05,
        8.0,
        &ranges,
    )
    .expect("a scan of the wall");
    let mut live = LiveObstacles::new(
        odom(),
        bounds(),
        heights(),
        small_square(),
        NonZeroUsize::MIN,
    )
    .expect("whole voxels");
    let base = Sweep {
        first_ray: planar(lidar_x, 0.0, 0.0),
        last_ray: planar(lidar_x, 0.0, turn),
    };
    live.insert_laser_scan(0, &scan, base, lidar_on_base(), range())
        .expect("the live obstacles' one source");
    let columns = live.columns();
    let wall: BTreeSet<(u32, u32)> = (0..rays)
        .map(|i| {
            let cell = columns
                .cell_of([wall_x, distance * pointing(i).tan()])
                .expect("the wall lies on the map");
            (cell.x(), cell.y())
        })
        .collect();
    assert_eq!(lethal_cells(&layer(&live)), wall);
}
