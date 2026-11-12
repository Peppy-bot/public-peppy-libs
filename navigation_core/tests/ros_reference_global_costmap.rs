//! The composed cost grid against a Nav2 global cost map of a static, an obstacle and an
//! inflation layer (nav2_costmap_2d 1.5.2): `SavedGrid::from_image` reads a saved map as
//! map_io and the static layer read it, `LiveObstacles` marks the cloud's occupied voxels
//! in the band as the obstacle layer marks its points and clears the columns under the
//! padded footprint as the obstacle layer clears its cells there, and `compose` merges
//! them and inflates as the layered cost map does, so for the same saved map, points,
//! footprint, base pose and parameters the cost grid is Nav2's, bit for bit. The cells
//! under the footprint read free over the saved map's unknown cells in both, and the saved
//! map's lethal cells stay. The obstacle layer under test also frees each cell over a
//! voxel its rays cleared, where Nav2's, with clearing off, leaves a cell unknown; the
//! scenario keeps the saved map's unknown cells out of the camera's view, and the test
//! checks that none lies under such a cell outside the footprint. The evidence: every
//! global costmap scenario of the ROS reference harness, whose committed outputs hold the
//! bits of Nav2's amd64 build.
#![cfg(target_arch = "x86_64")]

mod common;

use camera_core::pose::PlanarPose;

use common::clouds::{
    assert_clear_of_edges, assert_on_step, geometry_of, heights_of, insert_cloud,
    live_obstacles_over, padded_as_nav2,
};
use common::footprints::inside_by;
use common::poses::planar;
use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;
use navigation_core::costmap::compose;
use navigation_core::grid::{FREE, LETHAL, UNKNOWN};
use navigation_core::inflation::{Inflation, InflationParams};
use navigation_core::saved_grid::{MapThresholds, SavedGrid};
use ros_reference::global_costmap::Cases;

/// How far past the footprint's outline, in meters, the cells its raster clears reach: one
/// cell's diagonal.
const REACH_M: f64 = 0.08;

/// The cost the static layer gives an occupancy grid's value, for a trinary map with
/// unknown cells tracked.
fn static_cost(value: i8) -> u8 {
    match value {
        -1 => UNKNOWN,
        100 => LETHAL,
        0 => FREE,
        other => panic!("map_io gives a trinary map no value {other}"),
    }
}

// The saved grid's costs are the static layer's, and the composed and inflated grid is
// Nav2's global cost map.
#[test]
fn the_cost_grid_is_nav2s_global_cost_map() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a global costmap scenario"
    );
    for cases in &scenarios {
        let name = &cases.name;
        let (map, globals) = (&cases.map, &cases.globals);
        assert!(
            map.mode == "trinary" && !map.negate,
            "{name}: map_io reads the map as the saved grid does: trinary, not negated"
        );
        let thresholds = MapThresholds::parse(map.occupied_thresh, map.free_thresh)
            .expect("the scenario's thresholds");
        assert!(
            globals.track_unknown_space
                && globals.trinary_costmap
                && !globals.use_maximum
                && globals.lethal_cost_threshold == 100
                && globals.unknown_cost_value == 255
                && !cases.static_layer.footprint_clearing_enabled,
            "{name}: the static layer takes the map as the saved grid does"
        );
        assert!(
            cases.obstacle.combination_method == 1
                && cases.obstacle.footprint_clearing_enabled
                && cases.obstacle.depth.marking
                && !cases.obstacle.depth.clearing,
            "{name}: the obstacle layer marks, its rays clear nothing, its footprint clears, \
             and it merges by the maximum"
        );
        let inflation_parameters = cases.inflation;
        assert!(
            inflation_parameters.enabled
                && !inflation_parameters.inflate_unknown
                && !inflation_parameters.inflate_around_unknown
                && inflation_parameters.custom_inscribed_radius < 0.0,
            "{name}: the inflation layer runs as the port does"
        );

        let height = map.image.len() as u32;
        let width = map.image[0].len() as u32;
        let pixels: Vec<u8> = map.image.iter().flatten().copied().collect();
        let saved = SavedGrid::from_image(
            width,
            height,
            &pixels,
            PositiveFinite::parse(map.resolution).expect("a positive resolution"),
            [map.origin[0], map.origin[1]],
            thresholds,
        )
        .expect("the scenario's image");
        let geometry = geometry_of(&cases.grid);
        assert_eq!(
            saved.grid().geometry(),
            geometry,
            "{name}: the saved grid's geometry"
        );
        let static_costs: Vec<u8> = cases.map_values.iter().map(|&v| static_cost(v)).collect();
        assert_eq!(
            saved.grid().costs(),
            static_costs,
            "{name}: the static layer's costs"
        );

        // The voxels take the map's resolution as its YAML file gives it, a whole divisor
        // of the band; the grid holds it in single precision.
        let voxel_m = map.resolution;
        let heights = heights_of(
            cases.obstacle.min_obstacle_height,
            cases.obstacle.max_obstacle_height,
            voxel_m,
        );
        assert_on_step(
            heights.min_obstacle_height_m(),
            voxel_m,
            "the band's lower end",
        );
        assert_on_step(
            heights.max_obstacle_height_m(),
            voxel_m,
            "the band's upper end",
        );
        assert_clear_of_edges(name, &cases.grid, heights, &cases.points_in_grid);
        let footprint = padded_as_nav2(
            &cases.footprint,
            cases.footprint_padding,
            &cases.padded_footprint,
            name,
        );
        let mut live = live_obstacles_over(&cases.grid, voxel_m, heights, footprint.clone());
        insert_cloud(
            &mut live,
            &cases.sensor,
            &cases.points,
            cases.base,
            cases.obstacle.depth.obstacle_max_range,
        );
        let obstacles = live
            .project(PlanarPose::IDENTITY, geometry)
            .expect("a finite placement");
        let [x, y, yaw] = cases.base;
        let under_footprint: Vec<[f64; 2]> = footprint.placed_at(planar(x, y, yaw));
        let freed_unknown: Vec<_> = geometry
            .cells()
            .filter(|&cell| {
                saved.grid().cost(cell) == UNKNOWN
                    && obstacles.grid().cost(cell) == FREE
                    && !inside_by(&under_footprint, geometry.center_of(cell), -REACH_M)
            })
            .collect();
        assert!(
            freed_unknown.is_empty(),
            "{name}: the cloud's rays clear {} cells the saved map leaves unknown, the first \
             {:?}; keep the saved map's unknown cells out of the camera's view",
            freed_unknown.len(),
            freed_unknown.first()
        );

        let inflation = Inflation::parse(
            InflationParams {
                inflation_radius_m: NonNegativeFinite::parse(inflation_parameters.inflation_radius)
                    .expect("a radius of at least 0"),
                cost_scaling_factor: NonNegativeFinite::parse(
                    inflation_parameters.cost_scaling_factor,
                )
                .expect("a factor of at least 0"),
            },
            &footprint,
        )
        .expect("the scenario's radius covers the padded footprint's inscribed radius");
        let ours = compose(Some(&saved), &obstacles, Some(&inflation)).expect("one geometry");
        let differing: Vec<(usize, u8, u8)> = ours
            .costs()
            .iter()
            .zip(&cases.costs)
            .enumerate()
            .filter(|(_, (a, b))| a != b)
            .map(|(i, (&a, &b))| (i, a, b))
            .collect();
        assert!(
            differing.is_empty(),
            "{name}: {} cells differ from Nav2's, the first (index, ours, Nav2's): {:?}",
            differing.len(),
            &differing[..differing.len().min(5)]
        );
    }
}
