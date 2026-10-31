//! Self distance. The reference is MoveIt's FCL collision environment: posed at each
//! configuration of the robot, it gives the signed distance between the collision meshes
//! of each pair of links, negative where they overlap.

use crate::{Kind, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

/// The `kind` of a self distance scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "self_distance")]
    SelfDistance,
}

/// A robot and the configurations to pose it at, each a position per joint name.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root.
    meshes: String,
    configurations: Vec<BTreeMap<String, f64>>,
}

/// The pairs of links FCL measures, and for each configuration, in the scenario's order,
/// the positions as the reference read them and one distance per pair, in the order of
/// the pairs.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    pairs: Vec<[String; 2]>,
    configurations: Vec<Row>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Row {
    positions: BTreeMap<String, f64>,
    distances: Vec<f64>,
}

/// A self distance scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The directory of the collision meshes.
    pub meshes: PathBuf,
    pub configurations: Vec<Configuration>,
}

/// One configuration of the robot and FCL's distances at it.
pub struct Configuration {
    /// The position of each joint the scenario sets, by name.
    pub positions: BTreeMap<String, f64>,
    /// The signed distance between the meshes of each pair of links, by the pair's names
    /// in either order.
    distances: BTreeMap<(String, String), f64>,
}

impl Configuration {
    /// FCL's signed distance between the meshes of links `a` and `b`; `None` for a pair it
    /// did not measure.
    pub fn distance(&self, a: &str, b: &str) -> Option<f64> {
        let key = |a: &str, b: &str| (a.to_owned(), b.to_owned());
        self.distances
            .get(&key(a, b))
            .or_else(|| self.distances.get(&key(b, a)))
            .copied()
    }
}

impl Cases {
    /// Every self distance scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: one row per configuration, which read its
    /// positions, holds one distance per pair.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::SelfDistance)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::SelfDistance,
            urdf,
            meshes,
            configurations,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::SelfDistance,
            pairs,
            configurations: rows,
        } = read_reference(&name);
        Cases {
            urdf: read_urdf(&name, &urdf),
            meshes: read_mesh_directory(&name, &meshes),
            configurations: pair(&name, &pairs, configurations, rows),
            name,
        }
    }
}

fn pair(
    name: &str,
    pairs: &[[String; 2]],
    configurations: Vec<BTreeMap<String, f64>>,
    rows: Vec<Row>,
) -> Vec<Configuration> {
    assert_eq!(
        configurations.len(),
        rows.len(),
        "{name}: one output per configuration"
    );
    let mut measured = BTreeSet::new();
    for [a, b] in pairs {
        let unordered = if a <= b { (a, b) } else { (b, a) };
        assert!(
            measured.insert(unordered),
            "{name}: FCL measures {a} and {b} twice"
        );
    }
    configurations
        .into_iter()
        .zip(rows)
        .enumerate()
        .map(|(index, (positions, row))| {
            assert_eq!(
                row.positions, positions,
                "{name} configuration {index}: the reference read the positions"
            );
            assert_eq!(
                row.distances.len(),
                pairs.len(),
                "{name} configuration {index}: one distance per pair"
            );
            let distances = pairs
                .iter()
                .zip(row.distances)
                .map(|([a, b], distance)| ((a.clone(), b.clone()), distance))
                .collect();
            Configuration {
                positions,
                distances,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(positions: &[(&str, f64)], distances: &[f64]) -> Row {
        Row {
            positions: positions
                .iter()
                .map(|&(joint, position)| (joint.to_owned(), position))
                .collect(),
            distances: distances.to_vec(),
        }
    }

    #[test]
    fn a_pair_reads_in_either_order() {
        let pairs = [["a".to_owned(), "b".to_owned()]];
        let configurations = pair(
            "scenario",
            &pairs,
            vec![BTreeMap::new()],
            vec![row(&[], &[0.5])],
        );
        assert_eq!(configurations[0].distance("a", "b"), Some(0.5));
        assert_eq!(configurations[0].distance("b", "a"), Some(0.5));
        assert_eq!(configurations[0].distance("a", "c"), None);
    }

    #[test]
    #[should_panic(expected = "FCL measures b and a twice")]
    fn a_pair_measured_twice_does_not_pair() {
        let pairs = [
            ["a".to_owned(), "b".to_owned()],
            ["b".to_owned(), "a".to_owned()],
        ];
        pair("scenario", &pairs, vec![], vec![]);
    }

    #[test]
    #[should_panic(expected = "one distance per pair")]
    fn a_row_missing_a_distance_does_not_pair() {
        let pairs = [["a".to_owned(), "b".to_owned()]];
        pair(
            "scenario",
            &pairs,
            vec![BTreeMap::new()],
            vec![row(&[], &[])],
        );
    }

    #[test]
    #[should_panic(expected = "the reference read the positions")]
    fn a_row_that_read_other_positions_does_not_pair() {
        let pairs = [["a".to_owned(), "b".to_owned()]];
        let positions = BTreeMap::from([("j".to_owned(), 0.5)]);
        pair(
            "scenario",
            &pairs,
            vec![positions],
            vec![row(&[("j", 0.25)], &[1.0])],
        );
    }

    #[test]
    fn every_self_distance_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a self distance scenario"
        );
        for cases in scenarios {
            assert!(
                !cases.configurations.is_empty(),
                "{} holds a configuration",
                cases.name
            );
        }
    }
}
