//! Inflation. The reference is Nav2's inflation layer (nav2_costmap_2d 1.5.2): it inflates
//! the costs of each grid of a scenario, over the whole grid, for the scenario's footprint
//! padded by the scenario's padding as Costmap2DROS pads it, and the layer's parameters,
//! and gives the padded footprint, the inflated costs and the inscribed radius it took from
//! the padded footprint.

use crate::{Grid, Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of an inflation scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "inflation")]
    Inflation,
}

/// The inflation layer's parameters, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InflationParameters {
    pub enabled: bool,
    pub inflation_radius: f64,
    pub custom_inscribed_radius: f64,
    pub cost_scaling_factor: f64,
    pub inflate_unknown: bool,
    pub inflate_around_unknown: bool,
    pub num_threads: i64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    footprint: Vec<[f64; 2]>,
    footprint_padding: f64,
    inflation: InflationParameters,
    grids: Vec<GridInput>,
}

/// A grid and its costs before inflation, one row of cells a member from the first row.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GridInput {
    grid: Grid,
    costs: Vec<Vec<u8>>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    /// The footprint as Nav2 padded it.
    footprint: Vec<[f64; 2]>,
    inflation: InflationParameters,
    grids: Vec<GridOutput>,
}

/// A grid as the reference read it, the inscribed radius the layer took from the
/// footprint, and the grid's costs once inflated.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GridOutput {
    grid: Grid,
    given: Vec<Vec<u8>>,
    inscribed_radius: f64,
    inflated: Vec<Vec<u8>>,
}

/// One grid: its costs before and after Nav2's inflation, each row by row from the first
/// cell, and the inscribed radius Nav2 took from the padded footprint.
pub struct InflatedGrid {
    pub grid: Grid,
    pub given: Vec<u8>,
    pub inscribed_radius: f64,
    pub inflated: Vec<u8>,
}

/// An inflation scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub footprint: Vec<[f64; 2]>,
    pub footprint_padding: f64,
    /// The footprint as Nav2 padded it.
    pub padded_footprint: Vec<[f64; 2]>,
    pub inflation: InflationParameters,
    pub grids: Vec<InflatedGrid>,
}

impl Cases {
    /// Every inflation scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: the reference read the parameters and each grid
    /// with its costs, and padded the footprint, a point for each of the scenario's.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::Inflation)
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
        kind: ThisKind::Inflation,
        footprint,
        footprint_padding,
        inflation,
        grids,
    } = scenario;
    let Reference {
        kind: ThisKind::Inflation,
        footprint: footprint_read,
        inflation: inflation_read,
        grids: grids_read,
    } = reference;
    assert_eq!(
        footprint_read.len(),
        footprint.len(),
        "{name}: the reference padded every point of the footprint"
    );
    assert_eq!(
        inflation_read, inflation,
        "{name}: the reference read the layer's parameters"
    );
    assert_eq!(
        grids.len(),
        grids_read.len(),
        "{name}: the reference inflated every grid"
    );
    let grids = grids
        .into_iter()
        .zip(grids_read)
        .enumerate()
        .map(|(index, (input, read))| {
            assert_eq!(
                read.grid, input.grid,
                "{name} grid {index}: the reference read it"
            );
            assert_eq!(
                read.given, input.costs,
                "{name} grid {index}: the reference read its costs"
            );
            assert_eq!(
                read.inflated.len(),
                read.given.len(),
                "{name} grid {index}: the inflated grid holds every row"
            );
            InflatedGrid {
                grid: input.grid,
                given: input.costs.into_iter().flatten().collect(),
                inscribed_radius: read.inscribed_radius,
                inflated: read.inflated.into_iter().flatten().collect(),
            }
        })
        .collect();
    Cases {
        name,
        footprint,
        footprint_padding,
        padded_footprint: footprint_read,
        inflation,
        grids,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "smac_2d", "footprint": [], "footprint_padding": 0.0,
            "inflation": {"enabled": true,
            "inflation_radius": 0.5, "custom_inscribed_radius": -1.0,
            "cost_scaling_factor": 1.0, "inflate_unknown": false,
            "inflate_around_unknown": false, "num_threads": 1}, "grids": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn every_inflation_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds an inflation scenario"
        );
        for cases in scenarios {
            assert!(!cases.grids.is_empty(), "{} holds a grid", cases.name);
            for grid in &cases.grids {
                assert_eq!(grid.inflated.len(), grid.given.len(), "{}", cases.name);
            }
        }
    }
}
