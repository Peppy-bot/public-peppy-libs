//! Obstacle projection. The reference is Nav2's obstacle layer (nav2_costmap_2d 1.5.2): it
//! marks, on a scenario's grid, the points of each cloud of the scenario, seen from its
//! sensor's pose, through one observation source with the layer's height band and clearing
//! off, then clears the cells under the scenario's footprint, padded as Costmap2DROS pads
//! it, at the cloud's base pose, and gives the padded footprint, the lethal cells and each
//! point as the observation buffer transformed it into the grid's frame, in single
//! precision.

use std::collections::BTreeSet;

use crate::{
    Grid, Kind, SensorTransform, read_reference, read_scenario, read_single, scenario_names,
};
use serde::Deserialize;

/// The `kind` of an obstacle projection scenario and of its reference output, which
/// refuses every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "obstacle_projection")]
    ObstacleProjection,
}

/// The obstacle layer's parameters, by Nav2's names, with its one observation source.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObstacleParameters {
    pub enabled: bool,
    pub footprint_clearing_enabled: bool,
    pub min_obstacle_height: f64,
    pub max_obstacle_height: f64,
    pub combination_method: i64,
    pub observation_sources: String,
    pub tf_filter_tolerance: f64,
    pub depth: SourceParameters,
}

/// The parameters of an observation source, by Nav2's names.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceParameters {
    pub topic: String,
    pub sensor_frame: String,
    pub observation_persistence: f64,
    pub expected_update_rate: f64,
    pub data_type: String,
    pub min_obstacle_height: f64,
    pub max_obstacle_height: f64,
    pub inf_is_valid: bool,
    pub marking: bool,
    pub clearing: bool,
    pub transport_type: String,
    pub obstacle_max_range: f64,
    pub obstacle_min_range: f64,
    pub raytrace_max_range: f64,
    pub raytrace_min_range: f64,
}

/// The parameters of the cost map the layer reads beyond its own, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Globals {
    pub track_unknown_space: bool,
    pub transform_tolerance: f64,
    pub allow_parameter_qos_overrides: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    grid: Grid,
    footprint: Vec<[f64; 2]>,
    footprint_padding: f64,
    obstacle: ObstacleParameters,
    globals: Globals,
    clouds: Vec<CloudInput>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CloudInput {
    sensor: SensorTransform,
    /// The base's pose, x, y and yaw, in the grid's frame.
    base: [f64; 3],
    /// The points in the sensor's frame.
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    grid: Grid,
    /// The footprint as Nav2 padded it.
    footprint: Vec<[f64; 2]>,
    layer_band: [f64; 2],
    source_band: [f64; 2],
    clouds: Vec<CloudOutput>,
}

/// A cloud as the reference read it: its points in the grid's frame in single precision,
/// and the cells the layer marked lethal, each its column and row.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CloudOutput {
    points: Vec<[f64; 3]>,
    lethal: Vec<[u32; 2]>,
}

/// One cloud: where its sensor and the base stand, its points in the sensor's frame in the
/// single precision in which the layer reads them, the same points in the grid's frame as
/// Nav2 transformed them, and the cells Nav2's layer marked lethal after it cleared the
/// cells under the padded footprint.
pub struct Cloud {
    pub sensor: SensorTransform,
    /// The base's pose, x, y and yaw, in the grid's frame.
    pub base: [f64; 3],
    pub points: Vec<[f32; 3]>,
    pub points_in_grid: Vec<[f32; 3]>,
    pub lethal: BTreeSet<(u32, u32)>,
}

/// An obstacle projection scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub grid: Grid,
    pub footprint: Vec<[f64; 2]>,
    pub footprint_padding: f64,
    /// The footprint as Nav2 padded it.
    pub padded_footprint: Vec<[f64; 2]>,
    pub obstacle: ObstacleParameters,
    pub globals: Globals,
    pub clouds: Vec<Cloud>,
}

impl Cases {
    /// Every obstacle projection scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read the grid, the layer's
    /// band and its source's, which is the layer's, padded every point of the footprint,
    /// and transformed every point of each cloud.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ObstacleProjection)
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
        kind: ThisKind::ObstacleProjection,
        grid,
        footprint,
        footprint_padding,
        obstacle,
        globals,
        clouds,
    } = scenario;
    let Reference {
        kind: ThisKind::ObstacleProjection,
        grid: grid_read,
        footprint: footprint_read,
        layer_band,
        source_band,
        clouds: clouds_read,
    } = reference;
    assert_eq!(grid_read, grid, "{name}: the reference read the grid");
    assert_eq!(
        footprint_read.len(),
        footprint.len(),
        "{name}: the reference padded every point of the footprint"
    );
    let band = [obstacle.min_obstacle_height, obstacle.max_obstacle_height];
    assert_eq!(
        layer_band, band,
        "{name}: the reference read the layer's band"
    );
    assert_eq!(
        source_band,
        [
            obstacle.depth.min_obstacle_height,
            obstacle.depth.max_obstacle_height
        ],
        "{name}: the reference read the source's band"
    );
    assert_eq!(
        source_band, band,
        "{name}: the source's band is the layer's"
    );
    assert_eq!(
        clouds.len(),
        clouds_read.len(),
        "{name}: the reference read every cloud"
    );
    let clouds = clouds
        .into_iter()
        .zip(clouds_read)
        .enumerate()
        .map(|(index, (input, read))| {
            assert_eq!(
                read.points.len(),
                input.points.len(),
                "{name} cloud {index}: the reference transformed every point"
            );
            let lethal: BTreeSet<(u32, u32)> = read.lethal.iter().map(|&[x, y]| (x, y)).collect();
            assert_eq!(
                lethal.len(),
                read.lethal.len(),
                "{name} cloud {index}: the layer marks each cell once"
            );
            Cloud {
                sensor: input.sensor,
                base: input.base,
                points: input
                    .points
                    .iter()
                    .map(|point| point.map(|c| c as f32))
                    .collect(),
                points_in_grid: read.points.iter().map(|p| p.map(read_single)).collect(),
                lethal,
            }
        })
        .collect();
    Cases {
        name,
        grid,
        footprint,
        footprint_padding,
        padded_footprint: footprint_read,
        obstacle,
        globals,
        clouds,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_obstacle_projection_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds an obstacle projection scenario"
        );
        for cases in scenarios {
            assert!(!cases.clouds.is_empty(), "{} holds a cloud", cases.name);
            for cloud in &cases.clouds {
                assert!(
                    !cloud.lethal.is_empty(),
                    "{}: a cloud marks a cell",
                    cases.name
                );
            }
        }
    }
}
