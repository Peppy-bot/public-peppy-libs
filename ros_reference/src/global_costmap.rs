//! Global costmap. The reference is a Nav2 global cost map (nav2_costmap_2d 1.5.2) of a
//! static layer, an obstacle layer and an inflation layer, updated once: the static layer
//! takes the scenario's saved map as nav2_map_server's `loadMapFromFile` reads its image as
//! a trinary map, the obstacle layer marks the scenario's cloud from its sensor's pose and
//! clears the cells under the footprint, padded as Costmap2DROS pads it, at the scenario's
//! base pose, and the inflation layer inflates the result for the padded footprint. It
//! gives the map as map_io read it, the padded footprint, the cloud's points in the map's
//! frame as the observation buffer transformed them, and the cost map's costs.

use crate::inflation::InflationParameters;
use crate::obstacle_projection::ObstacleParameters;
use crate::{
    Grid, Kind, SensorTransform, read_reference, read_scenario, read_single, scenario_names,
};
use serde::Deserialize;

/// The `kind` of a global costmap scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "global_costmap")]
    GlobalCostmap,
}

/// A saved map as its YAML file gives it to map_io: its image, one row of 8-bit gray pixels
/// a member from the top, the side of a pixel, the pose of its bottom left pixel's corner,
/// x, y and yaw, and how map_io reads it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SavedMap {
    pub image: Vec<Vec<u8>>,
    pub resolution: f64,
    pub origin: [f64; 3],
    pub free_thresh: f64,
    pub occupied_thresh: f64,
    pub mode: String,
    pub negate: bool,
}

/// The static layer's parameters, by Nav2's names.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StaticParameters {
    pub enabled: bool,
    pub subscribe_to_updates: bool,
    pub footprint_clearing_enabled: bool,
    pub restore_cleared_footprint: bool,
    pub map_topic: String,
    pub map_subscribe_transient_local: bool,
}

/// The cost map's parameters that its layers read, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Globals {
    pub track_unknown_space: bool,
    pub use_maximum: bool,
    pub lethal_cost_threshold: i64,
    pub inscribed_obstacle_cost_value: i64,
    pub unknown_cost_value: i64,
    pub trinary_costmap: bool,
    pub transform_tolerance: f64,
    pub allow_parameter_qos_overrides: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    map: SavedMap,
    #[serde(rename = "static")]
    static_layer: StaticParameters,
    obstacle: ObstacleParameters,
    inflation: InflationParameters,
    globals: Globals,
    footprint: Vec<[f64; 2]>,
    footprint_padding: f64,
    /// The base's pose, x, y and yaw, in the map's frame.
    base: [f64; 3],
    cloud: CloudInput,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CloudInput {
    sensor: SensorTransform,
    /// The points in the sensor's frame.
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    map: Vec<Vec<i8>>,
    grid: Grid,
    /// The footprint as Nav2 padded it.
    footprint: Vec<[f64; 2]>,
    /// The points in the map's frame, each coordinate a single precision number.
    points: Vec<[f64; 3]>,
    costs: Vec<Vec<u8>>,
}

/// A global costmap scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub map: SavedMap,
    pub static_layer: StaticParameters,
    pub obstacle: ObstacleParameters,
    pub inflation: InflationParameters,
    pub globals: Globals,
    pub footprint: Vec<[f64; 2]>,
    pub footprint_padding: f64,
    /// The footprint as Nav2 padded it.
    pub padded_footprint: Vec<[f64; 2]>,
    /// The base's pose, x, y and yaw, in the map's frame.
    pub base: [f64; 3],
    /// The grid of the cost map: the saved map's, its resolution in single precision.
    pub grid: Grid,
    /// The occupancy grid map_io read, -1 unknown and 0 to 100 known, row by row from the
    /// first cell.
    pub map_values: Vec<i8>,
    pub sensor: SensorTransform,
    /// The cloud's points in the sensor's frame, in the single precision in which the
    /// layer reads them.
    pub points: Vec<[f32; 3]>,
    /// The same points in the map's frame, as Nav2 transformed them.
    pub points_in_grid: Vec<[f32; 3]>,
    /// The cost map's costs, row by row from the first cell.
    pub costs: Vec<u8>,
}

impl Cases {
    /// Every global costmap scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: the reference padded every point of the
    /// footprint, transformed every point, and its cost map lies over the saved map's
    /// grid.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::GlobalCostmap)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::GlobalCostmap,
        map,
        static_layer,
        obstacle,
        inflation,
        globals,
        footprint,
        footprint_padding,
        base,
        cloud,
    } = scenario;
    let Reference {
        kind: ThisKind::GlobalCostmap,
        map: map_read,
        grid: grid_read,
        footprint: footprint_read,
        points: points_read,
        costs,
    } = reference;
    let grid = grid_read;
    let height = map.image.len();
    let width = map.image.first().map_or(0, Vec::len);
    assert_eq!(
        (
            grid.size_x as usize,
            grid.size_y as usize,
            grid.resolution,
            grid.origin
        ),
        (
            width,
            height,
            f64::from(map.resolution as f32),
            [map.origin[0], map.origin[1]]
        ),
        "{name}: the cost map lies over the saved map's grid, at the resolution in the single \
         precision of the occupancy grid's metadata"
    );
    assert_eq!(map_read.len(), height, "{name}: map_io read every row");
    assert_eq!(
        footprint_read.len(),
        footprint.len(),
        "{name}: the reference padded every point of the footprint"
    );
    assert_eq!(
        points_read.len(),
        cloud.points.len(),
        "{name}: the reference transformed every point"
    );
    assert_eq!(costs.len(), height, "{name}: the cost map holds every row");
    Cases {
        name,
        map,
        static_layer,
        obstacle,
        inflation,
        globals,
        footprint,
        footprint_padding,
        padded_footprint: footprint_read,
        base,
        grid,
        map_values: map_read.into_iter().flatten().collect(),
        sensor: cloud.sensor,
        points: cloud.points.iter().map(|p| p.map(|c| c as f32)).collect(),
        points_in_grid: points_read.iter().map(|p| p.map(read_single)).collect(),
        costs: costs.into_iter().flatten().collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_global_costmap_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a global costmap scenario"
        );
        for cases in scenarios {
            assert_eq!(
                cases.costs.len(),
                cases.map_values.len(),
                "{}: a cost a cell of the map",
                cases.name
            );
        }
    }
}
