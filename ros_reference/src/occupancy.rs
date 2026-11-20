//! Occupancy. The reference is MoveIt's point cloud updater: it inserts each point cloud
//! of a scenario into MoveIt's occupancy map, with a sphere cut out as the robot's own
//! body, and the map gives the log-odds of every voxel it then holds.

use crate::{Kind, SensorTransform, read_reference, read_scenario, scenario_names};
use serde::Deserialize;
use std::collections::BTreeSet;

/// The `kind` of an occupancy scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "occupancy")]
    Occupancy,
}

/// The map's voxel size, the updater's range and padding, the sphere it cuts out, and the
/// clouds it inserts, in their order.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    voxel_size: f64,
    max_range: f64,
    padding_offset: f64,
    padding_scale: f64,
    cut_out: Sphere,
    clouds: Vec<CloudInput>,
}

/// A sphere in the map frame.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sphere {
    pub center: [f64; 3],
    pub radius: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CloudInput {
    sensor: SensorTransform,
    /// The points in the sensor's frame.
    points: Vec<[f64; 3]>,
}

/// Each voxel that the map holds after the clouds.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    /// Each voxel the map holds: its key on the three axes and its log-odds.
    voxels: Vec<(i32, i32, i32, f64)>,
}

/// One cloud: where its sensor stands, and its points in the sensor's frame, in the single
/// precision in which the updater reads them.
pub struct Cloud {
    pub sensor: SensorTransform,
    pub points: Vec<[f32; 3]>,
}

/// An occupancy scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub voxel_size: f64,
    pub max_range: f64,
    /// The cut-out sphere as the updater grows it: its radius times the padding scale, plus
    /// the padding offset.
    pub cut_out: Sphere,
    /// The clouds, in the order in which the updater inserts them.
    pub clouds: Vec<Cloud>,
    /// Each voxel that MoveIt's map holds after the clouds, by its key, with its log-odds.
    pub voxels: Vec<([i32; 3], f64)>,
}

impl Cases {
    /// Every occupancy scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output, which holds each voxel of the map once.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::Occupancy)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

/// `points` as the updater reads them from a cloud, whose coordinates are single
/// precision numbers.
fn in_single_precision(points: &[[f64; 3]]) -> Vec<[f32; 3]> {
    points
        .iter()
        .map(|point| point.map(|coordinate| coordinate as f32))
        .collect()
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::Occupancy,
        voxel_size,
        max_range,
        padding_offset,
        padding_scale,
        cut_out,
        clouds,
    } = scenario;
    let Reference {
        kind: ThisKind::Occupancy,
        voxels,
    } = reference;
    let clouds = clouds
        .into_iter()
        .map(|cloud| Cloud {
            sensor: cloud.sensor,
            points: in_single_precision(&cloud.points),
        })
        .collect();
    let voxels: Vec<([i32; 3], f64)> = voxels
        .into_iter()
        .map(|(x, y, z, log_odds)| ([x, y, z], log_odds))
        .collect();
    let keys: BTreeSet<[i32; 3]> = voxels.iter().map(|(key, _)| *key).collect();
    assert_eq!(
        keys.len(),
        voxels.len(),
        "{name}: the map holds each voxel once"
    );
    Cases {
        name,
        voxel_size,
        max_range,
        cut_out: Sphere {
            center: cut_out.center,
            radius: cut_out.radius * padding_scale + padding_offset,
        },
        clouds,
        voxels,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A point of the first cloud of `occupancy_room`, and its coordinates as the reference
    /// program reads them from the cloud: the doubles that hold their single precision
    /// numbers.
    const ROOM_POINT: [f64; 3] = [0.18336085911150296, 1.7327322845055468, 0.534251987237059];
    const ROOM_POINT_READ: [f64; 3] = [0.18336085975170135, 1.7327322959899902, 0.5342519879341125];

    /// A scenario of one cloud of one point, at 0.1 on each axis, from a sensor at the
    /// origin, unturned.
    fn scenario() -> Scenario {
        serde_json::from_str(
            r#"{"kind": "occupancy", "voxel_size": 1.0, "max_range": 1.0,
                "padding_offset": 0.0, "padding_scale": 1.0,
                "cut_out": {"center": [0.0, 0.0, 0.0], "radius": 1.0},
                "clouds": [{"sensor": {"position": [0.0, 0.0, 0.0],
                                       "orientation": [0.0, 0.0, 0.0, 1.0]},
                            "points": [[0.1, 0.1, 0.1]]}]}"#,
        )
        .expect("parses")
    }

    /// The output of the scenario of [`scenario`] that holds the voxels `voxels`.
    fn reference(voxels: &str) -> Reference {
        serde_json::from_str(&format!(r#"{{"kind": "occupancy", "voxels": {voxels}}}"#))
            .expect("parses")
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "self_distance", "voxel_size": 1.0, "max_range": 1.0,
            "padding_offset": 0.0, "padding_scale": 1.0,
            "cut_out": {"center": [0.0, 0.0, 0.0], "radius": 1.0}, "clouds": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn a_cloud_pairs_with_its_points_in_single_precision() {
        let cases = pair("scenario".into(), scenario(), reference("[[0, 0, 0, 1]]"));
        assert_eq!(cases.clouds[0].points, [[0.1f32; 3]]);
        assert_eq!(cases.voxels, [([0, 0, 0], 1.0)]);
    }

    #[test]
    fn the_cut_out_grows_by_the_padding_scale_and_offset() {
        let scenario = Scenario {
            padding_scale: 2.0,
            padding_offset: 0.25,
            ..scenario()
        };
        let cases = pair("scenario".into(), scenario, reference("[[0, 0, 0, 1]]"));
        assert_eq!(cases.cut_out.radius, 2.25);
    }

    #[test]
    fn a_point_reads_in_single_precision_as_the_reference_program_reads_it() {
        let read = in_single_precision(&[ROOM_POINT]);
        assert_eq!(read[0].map(f64::from), ROOM_POINT_READ);
    }

    #[test]
    #[should_panic(expected = "the map holds each voxel once")]
    fn an_output_holding_a_voxel_twice_does_not_pair() {
        pair(
            "scenario".into(),
            scenario(),
            reference("[[0, 0, 0, 1], [0, 0, 0, 0]]"),
        );
    }

    #[test]
    fn every_occupancy_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds an occupancy scenario"
        );
        for cases in scenarios {
            assert!(!cases.clouds.is_empty(), "{} holds a cloud", cases.name);
            assert!(!cases.voxels.is_empty(), "{} holds a voxel", cases.name);
        }
    }
}
