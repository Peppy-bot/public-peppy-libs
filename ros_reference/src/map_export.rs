//! Map export. The reference is Nav2's map saver: it writes each occupancy grid of a
//! scenario as a trinary map (`nav2_map_server::saveMapToFile`), and gives the bytes of
//! the image and the text of the description it wrote.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a map export scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "map_export")]
    MapExport,
}

/// An occupancy grid as a map's message holds it: its size in cells, the side of a cell
/// (m) in single precision, the corner of its first cell and its heading, `[x, y, yaw]`
/// (m, rad), and each cell row by row from the first, -1 for unknown and 0 to 100.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Grid {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin: [f64; 3],
    pub cells: Vec<i8>,
}

/// The map saver's thresholds and the grids it writes.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    free_thresh: f64,
    occupied_thresh: f64,
    grids: Vec<Grid>,
}

/// The files the map saver wrote for a grid.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Export {
    /// The bytes of the image (`map.pgm`).
    pub image: Vec<u8>,
    /// The text of the description (`map.yaml`).
    pub description: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    maps: Vec<Export>,
}

/// A map export scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub free_thresh: f64,
    pub occupied_thresh: f64,
    /// Each grid with what the map saver wrote for it, in order.
    pub maps: Vec<(Grid, Export)>,
}

impl Cases {
    /// Every map export scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output, a map a grid.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::MapExport)
            .into_iter()
            .map(|name| {
                let Scenario {
                    kind: ThisKind::MapExport,
                    free_thresh,
                    occupied_thresh,
                    grids,
                } = read_scenario(&name);
                let Reference {
                    kind: ThisKind::MapExport,
                    maps,
                } = read_reference(&name);
                assert_eq!(
                    grids.len(),
                    maps.len(),
                    "{name}: the map saver wrote every grid"
                );
                Cases {
                    name,
                    free_thresh,
                    occupied_thresh,
                    maps: grids.into_iter().zip(maps).collect(),
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "occupancy", "free_thresh": 0.25, "occupied_thresh": 0.65,
                       "grids": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn every_map_export_scenario_pairs_with_its_reference() {
        let all = Cases::load_all();
        assert!(!all.is_empty(), "scenarios/ holds a map export scenario");
        for cases in all {
            assert!(!cases.maps.is_empty(), "{} holds a grid", cases.name);
        }
    }
}
