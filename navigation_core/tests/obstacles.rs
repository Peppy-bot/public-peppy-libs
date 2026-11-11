//! The live obstacles on drawn scenes: a table top above the band marks only its legs and
//! the floor marks no cell, while a box that rises into the band marks the cells under
//! it; a step down deeper than the limit marks where the lower floor is seen and a
//! shallower one marks none; a base that sees its own deck as it drives and turns marks
//! no cell; the footprint's clearing leaves no occupied cell under it and every cell of a
//! saved grid; and a laser scan marks each hit where the sensor stood at its ray's time,
//! a ray that met nothing marking and freeing nothing.

mod common;

use std::collections::BTreeSet;

use camera_core::laser_scan::LaserScan;
use camera_core::pose::{PlanarPose, Pose};
use common::footprints::{inside_by, padded};
use common::layers::{layer, lethal_cells};
use control_core::positive_finite::PositiveFinite;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::PaddedFootprint;
use navigation_core::costmap::compose;
use navigation_core::grid::{GridGeometry, LETHAL, UNKNOWN};
use navigation_core::obstacles::{LiveObstacles, ObstacleHeights};
use navigation_core::saved_grid::{MapThresholds, SavedGrid};
use occupancy_map::{Bounds, Frame, VoxelMap};

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
const AWAY: PlanarPose = PlanarPose {
    x: -10.0,
    y: 0.0,
    yaw: 0.0,
};

fn voxel() -> PositiveFinite {
    PositiveFinite::parse(VOXEL_M).expect("positive")
}

fn range() -> PositiveFinite {
    PositiveFinite::parse(RANGE_M).expect("positive")
}

/// The band from 0.1 m to 0.6 m and a step of 0.1 m, whole voxels.
fn heights() -> ObstacleHeights {
    ObstacleHeights::parse(0.1, 0.6, 0.1, voxel()).expect("heights of whole voxels")
}

/// An empty map in the odometry frame, from 1.5 m behind and beside the origin to 2 m
/// ahead, and from 0.5 m below the floor to 2 m above it.
fn map() -> VoxelMap {
    VoxelMap::new(
        Frame::parse("odom").expect("a frame name"),
        voxel(),
        Bounds {
            min: [-1.5, -1.5, -0.5],
            max: [2.0, 1.5, 2.0],
        },
    )
    .expect("a map")
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
    let mut live = LiveObstacles::new(map(), heights, small_square()).expect("whole voxels");
    let sensor = sensor_at(SENSOR, 0.0);
    live.insert_depth_frame(&seen_from(&sensor, &points), sensor, AWAY, range())
        .expect("the scan stands in the map's frame");

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

// The floor drops by a drawn depth beyond an edge ahead of an overhead sensor. A depth
// more than max_step_down_m, by at least half a voxel, marks every cell where the sensor
// sees the lower floor; a depth at least half a voxel short of it marks no cell.
#[hegel::test(test_cases = 50)]
fn a_step_down_deeper_than_the_limit_marks_the_lower_floor_and_a_shallower_one_none(tc: TestCase) {
    let step_voxels = tc.draw(gs::integers::<u32>().min_value(1).max_value(4));
    let step_m = f64::from(step_voxels) * VOXEL_M;
    let heights =
        ObstacleHeights::parse(0.1, 0.6, step_m, voxel()).expect("heights of whole voxels");
    let deeper = tc.draw(gs::booleans());
    let depth_m = if deeper {
        tc.draw(
            gs::floats()
                .min_value(step_m + VOXEL_M / 2.0)
                .max_value(step_m + 3.0 * VOXEL_M),
        )
    } else {
        tc.draw(
            gs::floats()
                .min_value(VOXEL_M / 4.0)
                .max_value(step_m - VOXEL_M / 2.0),
        )
    };
    let edge = 0.41;
    let lower = [[edge, -0.49], [1.39, 0.49]];
    let points: Vec<[f64; 3]> = surface([-1.0, -1.0], [edge - SPACING_M, 1.0], 0.0)
        .into_iter()
        .chain(surface(lower[0], lower[1], -depth_m))
        .collect();
    let mut live = LiveObstacles::new(map(), heights, small_square()).expect("whole voxels");
    let sensor = sensor_at([0.0, 0.0, 1.0], 0.0);
    live.insert_depth_frame(&seen_from(&sensor, &points), sensor, AWAY, range())
        .expect("the scan stands in the map's frame");
    let lethal = lethal_cells(&layer(&live));
    if deeper {
        assert_eq!(
            lethal,
            cells_under(live.columns(), lower[0], lower[1]),
            "a step down of {depth_m} m, deeper than {step_m} m"
        );
    } else {
        assert!(
            lethal.is_empty(),
            "a step down of {depth_m} m, within {step_m} m, marks {lethal:?}"
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
    let mut live = LiveObstacles::new(map(), heights(), small_square()).expect("whole voxels");
    let insert = |live: &mut LiveObstacles, points: &[[f32; 3]], base: PlanarPose| {
        live.insert_depth_frame(points, sensor, base, range())
            .expect("the scan stands in the map's frame")
    };
    for _ in 0..10 {
        insert(&mut live, &frame, AWAY);
    }
    let under = cells_under(live.columns(), box_[0], box_[1]);
    assert!(under.is_subset(&lethal_cells(&layer(&live))));
    let over_the_box = PlanarPose {
        x: 0.4,
        y: 0.0,
        yaw: 0.0,
    };
    insert(&mut live, &[], over_the_box);
    assert!(lethal_cells(&layer(&live)).is_disjoint(&under));
    insert(&mut live, &frame, AWAY);
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
    let start = PlanarPose {
        x: -0.5,
        y: tc.draw(gs::floats().min_value(-0.3).max_value(0.3)),
        yaw: 0.0,
    };
    let driving = (0..=drive_steps).map(|k| PlanarPose {
        x: start.x + f64::from(k) / f64::from(drive_steps),
        ..start
    });
    let turning = (1..=turn_steps).map(|k| PlanarPose {
        x: start.x + 1.0,
        yaw: turn_rad * f64::from(k) / f64::from(turn_steps),
        ..start
    });
    let mut live = LiveObstacles::new(map(), heights(), footprint).expect("whole voxels");
    for base in driving.chain(turning) {
        let sensor = sensor_at([base.x, base.y, mount_m], base.yaw);
        let seen: Vec<[f64; 3]> = deck
            .iter()
            .map(|&point| {
                let [x, y] = base.apply(point);
                [x, y, deck_m]
            })
            .collect();
        live.insert_depth_frame(&seen_from(&sensor, &seen), sensor, base, range())
            .expect("the scan stands in the map's frame");
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
    let base = PlanarPose {
        x: tc.draw(gs::floats().min_value(-0.5).max_value(0.5)),
        y: tc.draw(gs::floats().min_value(-0.5).max_value(0.5)),
        yaw: tc.draw(gs::floats().min_value(-3.2).max_value(3.2)),
    };
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
    let mut live = LiveObstacles::new(map(), heights(), footprint.clone()).expect("whole voxels");
    let sensor = sensor_at(SENSOR, 0.0);
    live.insert_depth_frame(&seen_from(&sensor, &points), sensor, base, range())
        .expect("the scan stands in the map's frame");

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
    let obstacles = live.project(PlanarPose::IDENTITY, geometry);
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

/// A 2-D lidar 0.3 m up, in the band, at `x` meters along the map's x, turned by `yaw`.
fn lidar_at(x: f64, yaw: f64) -> Pose {
    sensor_at([x, 0.0, 0.3], yaw)
}

// A scan whose rays all carry lidar_2d's marks, one that met nothing, one not measured and
// one past the farthest range, marks and frees nothing; one hit among such rays marks its
// own cell alone.
#[test]
fn a_ray_that_met_nothing_frees_and_marks_nothing() {
    let mut live = LiveObstacles::new(map(), heights(), small_square()).expect("whole voxels");
    let lidar = lidar_at(-0.5, 0.0);
    let marks =
        LaserScan::parse(-0.1, 0.1, 0.0, 0.05, 8.0, &[10000.0, 0.0, 9.0]).expect("a scan of marks");
    live.insert_laser_scan(&marks, &lidar, &lidar, AWAY, range())
        .expect("the scan stands in the map's frame");
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
    live.insert_laser_scan(&one_hit, &lidar, &lidar, AWAY, range())
        .expect("the scan stands in the map's frame");
    let hit = live
        .columns()
        .cell_of([0.512, 0.0])
        .expect("the hit lies on the map");
    assert_eq!(
        lethal_cells(&layer(&live)),
        BTreeSet::from([(hit.x(), hit.y())])
    );
}

// A lidar that turns by 0.4 rad while it sweeps a wall 1.012 m ahead: each ray's range is
// the distance along the direction it pointed at its time, and each hit marks the cell of
// the wall that ray met, where the sensor stood at the ray's time. The wall's points lie
// at least 1e-4 m from every cell edge, so rounding moves none.
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
    let mut live = LiveObstacles::new(map(), heights(), small_square()).expect("whole voxels");
    let base = PlanarPose {
        x: lidar_x,
        y: 0.0,
        yaw: 0.0,
    };
    live.insert_laser_scan(
        &scan,
        &lidar_at(lidar_x, 0.0),
        &lidar_at(lidar_x, turn),
        base,
        range(),
    )
    .expect("the scan stands in the map's frame");
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
