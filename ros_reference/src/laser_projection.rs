//! Laser projection. The reference is pointcloud_to_laserscan: it projects each point
//! cloud of a scenario to a 2-D scan, with the scenario's height band, angles and range
//! limits and no target frame, and gives the angles and the range limits that it writes
//! into every scan, and each scan's ranges.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a laser projection scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "laser_projection")]
    LaserProjection,
}

/// pointcloud_to_laserscan's parameters, by the names of its node.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaserScanParameters {
    pub min_height: f64,
    pub max_height: f64,
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    pub scan_time: f64,
    pub range_min: f64,
    pub range_max: f64,
    /// Whether a bin with no point holds +inf.
    pub use_inf: bool,
}

/// pointcloud_to_laserscan's parameters and the clouds it projects, each a list of points
/// [x, y, z] in the scan's frame.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    laser_scan: LaserScanParameters,
    clouds: Vec<Vec<[f64; 3]>>,
}

/// The angles and the range limits that the node wrote into every scan, each widened from
/// single precision.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct HeaderRead {
    angle_min: f64,
    angle_max: f64,
    angle_increment: f64,
    range_min: f64,
    range_max: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    scan: HeaderRead,
    ranges: Vec<Vec<RangeRead>>,
}

/// The angles (rad) and the range limits (m) that pointcloud_to_laserscan wrote into every
/// scan of a scenario, in the single precision of the message.
#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceHeader {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
}

/// A cloud, its points in the single precision in which the node reads them, and the ranges
/// of the scan that the node gave for it, in the single precision of the message.
#[derive(Debug, Clone, PartialEq)]
pub struct Cloud {
    pub points: Vec<[f32; 3]>,
    pub ranges: Vec<f32>,
}

/// A laser projection scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub laser_scan: LaserScanParameters,
    pub scan: ReferenceHeader,
    /// The clouds, in the order in which the node projected them.
    pub clouds: Vec<Cloud>,
}

impl Cases {
    /// Every laser projection scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output, which projected every cloud.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::LaserProjection)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

/// A range of a scan as a reference writes it: a number widened from single precision,
/// or a mark of REP 117, `inf`, `-inf` or `nan`, which JSON has no number for.
#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub(crate) enum RangeRead {
    Number(f64),
    Mark(String),
}

/// `value`, which the reference widened from single precision, back in single precision.
pub(crate) fn single(name: &str, what: &str, value: f64) -> f32 {
    let narrowed = value as f32;
    assert!(
        f64::from(narrowed).to_bits() == value.to_bits(),
        "{name}: the reference wrote {what} {value}, which is not a single precision value"
    );
    narrowed
}

/// A range that the reference wrote, in single precision.
pub(crate) fn range_of(name: &str, what: &str, range: RangeRead) -> f32 {
    match range {
        RangeRead::Number(value) => single(name, what, value),
        RangeRead::Mark(mark) => match mark.as_str() {
            "inf" => f32::INFINITY,
            "-inf" => f32::NEG_INFINITY,
            "nan" => f32::NAN,
            _ => panic!("{name}: the reference wrote {what} {mark:?}, which is no mark of REP 117"),
        },
    }
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::LaserProjection,
        laser_scan,
        clouds,
    } = scenario;
    let Reference {
        kind: ThisKind::LaserProjection,
        scan,
        ranges,
    } = reference;
    assert_eq!(
        clouds.len(),
        ranges.len(),
        "{name}: the reference projected every cloud"
    );
    let header = "the scans' header";
    let scan = ReferenceHeader {
        angle_min: single(&name, header, scan.angle_min),
        angle_max: single(&name, header, scan.angle_max),
        angle_increment: single(&name, header, scan.angle_increment),
        range_min: single(&name, header, scan.range_min),
        range_max: single(&name, header, scan.range_max),
    };
    let clouds = clouds
        .into_iter()
        .zip(ranges)
        .enumerate()
        .map(|(index, (points, ranges))| {
            let what = format!("cloud {index}'s scan");
            Cloud {
                points: points
                    .iter()
                    .map(|point| point.map(|coordinate| coordinate as f32))
                    .collect(),
                ranges: ranges
                    .into_iter()
                    .map(|range| range_of(&name, &what, range))
                    .collect(),
            }
        })
        .collect();
    Cases {
        name,
        laser_scan,
        scan,
        clouds,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 0.1, and 0.1 in single precision widened.
    const TENTH: &str = "0.1";
    const TENTH_IN_SINGLE: &str = "0.10000000149011612";
    /// The mark of REP 117 for a ray that met nothing.
    const INFINITY: &str = r#""inf""#;

    const PARAMETERS: &str = r#"{"min_height": 0.1, "max_height": 1.0, "angle_min": -0.5,
        "angle_max": 0.485, "angle_increment": 0.01, "scan_time": 0.2, "range_min": 0.2,
        "range_max": 5.0, "use_inf": true}"#;

    /// A scenario of one cloud of one point, at 0.1 on each axis.
    fn scenario() -> Scenario {
        serde_json::from_str(&format!(
            r#"{{"kind": "laser_projection", "laser_scan": {PARAMETERS},
                 "clouds": [[[0.1, 0.1, 0.1]]]}}"#
        ))
        .expect("parses")
    }

    /// The output of [`scenario`], the range `range` in each bin.
    fn reference(range: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "laser_projection",
                 "scan": {{"angle_min": -0.5, "angle_max": 0.48500001430511475,
                           "angle_increment": 0.009999999776482582,
                           "range_min": 0.20000000298023224, "range_max": 5.0}},
                 "ranges": [[{range}, {INFINITY}]]}}"#
        ))
        .expect("parses")
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = format!(r#"{{"kind": "occupancy", "laser_scan": {PARAMETERS}, "clouds": []}}"#);
        assert!(serde_json::from_str::<Scenario>(&text).is_err());
    }

    #[test]
    fn each_cloud_pairs_with_its_scan_in_single_precision() {
        let cases = pair("scenario".into(), scenario(), reference(TENTH_IN_SINGLE));
        assert_eq!(cases.scan.range_min, 0.2f32);
        assert_eq!(cases.clouds[0].points, [[0.1f32; 3]]);
        assert_eq!(cases.clouds[0].ranges, [0.1f32, f32::INFINITY]);
    }

    #[test]
    #[should_panic(expected = "which is not a single precision value")]
    fn an_output_whose_scan_holds_a_double_precision_range_does_not_pair() {
        pair("scenario".into(), scenario(), reference(TENTH));
    }

    #[test]
    fn every_laser_projection_scenario_pairs_with_its_reference() {
        let all = Cases::load_all();
        assert!(
            !all.is_empty(),
            "scenarios/ holds a laser projection scenario"
        );
        for cases in all {
            assert!(!cases.clouds.is_empty(), "{} holds a cloud", cases.name);
        }
    }
}
