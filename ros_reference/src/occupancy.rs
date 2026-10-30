//! Occupancy. The reference is MoveIt's point cloud updater: it inserts each point cloud
//! of a scenario into MoveIt's occupancy map, with a sphere cut out as the robot's own
//! body, and the map gives the log-odds of every voxel it then holds.

use crate::{Kind, read_reference, read_scenario, scenario_names};
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
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sphere {
    pub center: [f64; 3],
    pub radius: f64,
}

/// Where a sensor stands in the map frame: its position, and its orientation as a unit
/// quaternion [x, y, z, w].
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorTransform {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CloudInput {
    sensor: SensorTransform,
    /// The points in the sensor's frame.
    points: Vec<[f64; 3]>,
}

/// The parameters, the sphere and the clouds as the reference read them, each cloud's
/// points in the single precision of the cloud, and each voxel that the map holds after
/// the clouds.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    voxel_size: f64,
    max_range: f64,
    padding_offset: f64,
    padding_scale: f64,
    cut_out: Sphere,
    clouds: Vec<CloudInput>,
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
    /// paired with its reference output: the reference read the scenario's parameters, its
    /// sphere and each of its clouds, and holds each voxel of the map once.
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
        voxel_size: voxel_size_read,
        max_range: max_range_read,
        padding_offset: padding_offset_read,
        padding_scale: padding_scale_read,
        cut_out: cut_out_read,
        clouds: clouds_read,
        voxels,
    } = reference;
    assert_eq!(
        [
            voxel_size_read,
            max_range_read,
            padding_offset_read,
            padding_scale_read,
        ],
        [voxel_size, max_range, padding_offset, padding_scale],
        "{name}: the reference read the voxel size, the range and the padding"
    );
    assert_eq!(
        cut_out_read, cut_out,
        "{name}: the reference read the sphere"
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
        .map(|(index, (cloud, read))| {
            assert_eq!(
                read.sensor, cloud.sensor,
                "{name} cloud {index}: the reference read the sensor's pose"
            );
            let points: Vec<[f32; 3]> = cloud
                .points
                .iter()
                .map(|point| point.map(|coordinate| coordinate as f32))
                .collect();
            let points_widened: Vec<[f64; 3]> =
                points.iter().map(|point| point.map(f64::from)).collect();
            assert_eq!(
                read.points, points_widened,
                "{name} cloud {index}: the reference read the points in single precision"
            );
            Cloud {
                sensor: cloud.sensor,
                points,
            }
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

    /// 0.1, and 0.1 in single precision, as the reference writes each.
    const TENTH: &str = "0.1";
    const TENTH_IN_SINGLE: &str = "0.10000000149011612";

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

    /// The output that read the scenario of [`scenario`], with each coordinate of the
    /// point read as `coordinate`, and the voxels `voxels`.
    fn reference(coordinate: &str, voxels: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "occupancy", "voxel_size": 1, "max_range": 1,
                 "padding_offset": 0, "padding_scale": 1,
                 "cut_out": {{"center": [0, 0, 0], "radius": 1}},
                 "clouds": [{{"sensor": {{"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}},
                              "points": [[{coordinate}, {coordinate}, {coordinate}]]}}],
                 "voxels": {voxels}}}"#
        ))
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
    fn a_cloud_pairs_with_the_output_that_read_its_points_in_single_precision() {
        let cases = pair(
            "scenario".into(),
            scenario(),
            reference(TENTH_IN_SINGLE, "[[0, 0, 0, 1]]"),
        );
        assert_eq!(cases.clouds[0].points, [[0.1f32; 3]]);
        assert_eq!(cases.voxels, [([0, 0, 0], 1.0)]);
    }

    #[test]
    #[should_panic(expected = "the reference read the points in single precision")]
    fn an_output_that_read_the_points_in_double_precision_does_not_pair() {
        pair(
            "scenario".into(),
            scenario(),
            reference(TENTH, "[[0, 0, 0, 1]]"),
        );
    }

    #[test]
    #[should_panic(expected = "the map holds each voxel once")]
    fn an_output_holding_a_voxel_twice_does_not_pair() {
        pair(
            "scenario".into(),
            scenario(),
            reference(TENTH_IN_SINGLE, "[[0, 0, 0, 1], [0, 0, 0, 0]]"),
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
