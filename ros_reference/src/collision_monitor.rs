//! Collision monitor. The reference is Nav2's collision monitor (nav2_collision_monitor
//! 1.5.2) with one footprint polygon in approach mode and one point cloud source: each case
//! of a scenario runs in a monitor of its own, at the case's time on the node's clock, with
//! the case's cloud, if it has one, stamped at its arrival, the case's age before that
//! time, in the base's frame, and gives the action the monitor takes for the case's
//! velocity and the velocity it allows. A case's points are points the stop under test
//! reads as it reads the centers of a cost grid's occupied cells, or the points of a depth
//! frame, which it reads through navigation_core's `LiveObstacles`.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a collision monitor scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "collision_monitor")]
    CollisionMonitor,
}

/// The monitor's parameters, by Nav2's names: every parameter the node declares, its
/// polygon's under `FootprintApproach` and its source's under `depth`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MonitorParameters {
    pub cmd_vel_in_topic: String,
    pub cmd_vel_out_topic: String,
    pub state_topic: String,
    pub base_frame_id: String,
    pub odom_frame_id: String,
    pub transform_tolerance: f64,
    pub source_timeout: f64,
    pub base_shift_correction: bool,
    pub collision_points_marker_3d: bool,
    pub stop_pub_timeout: f64,
    pub polygons: Vec<String>,
    pub observation_sources: Vec<String>,
    pub enabled: bool,
    pub use_realtime_priority: bool,
    pub enable_stamped_cmd_vel: bool,
    pub allow_parameter_qos_overrides: bool,
    pub introspection_mode: String,
    #[serde(rename = "FootprintApproach")]
    pub footprint_approach: PolygonParameters,
    pub depth: SourceParameters,
}

/// The polygon's parameters, by Nav2's names; `points` is the footprint as Nav2 parses
/// it, a list of [x, y] in meters, each in single precision.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolygonParameters {
    #[serde(rename = "type")]
    pub polygon_type: String,
    pub points: String,
    pub action_type: String,
    pub enabled: bool,
    pub min_points: u32,
    pub trigger_consecutive_points: u32,
    pub release_consecutive_points: u32,
    pub time_before_collision: f64,
    pub simulation_time_step: f64,
    pub visualize: bool,
    pub polygon_subscribe_transient_local: bool,
    pub sources_names: Vec<String>,
}

/// The point cloud source's parameters, by Nav2's names.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceParameters {
    #[serde(rename = "type")]
    pub source_type: String,
    pub topic: String,
    pub enabled: bool,
    pub source_timeout: f64,
    pub min_height: f64,
    pub max_height: f64,
    pub min_range: f64,
    pub use_global_height: bool,
    pub transport_type: String,
    pub exclusion_zones: Vec<String>,
}

/// What a scenario's points are: points the stop under test reads as they are, as it
/// reads the centers of a cost grid's occupied cells (`cell_centers`), or a depth frame's
/// points, which it reads through navigation_core's `LiveObstacles`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PointsAre {
    CellCenters,
    Depth,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    monitor: MonitorParameters,
    /// How far below the floor, in meters, the `LiveObstacles` under test let a voxel lie
    /// before it is a drop; the collision monitor has no such parameter.
    max_step_down: f64,
    points_are: PointsAre,
    cases: Vec<CaseInput>,
}

/// A case: the time, in nanoseconds, how old the cloud is then, none when no cloud came,
/// the cloud's points in the base's frame, z up from the floor plane, and the velocity,
/// linear x, linear y and angular z.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CaseInput {
    now_ns: i64,
    /// Null when no cloud came; the field is required.
    #[serde(deserialize_with = "Option::deserialize")]
    cloud_age_ns: Option<i64>,
    points: Vec<[f64; 3]>,
    velocity: [f64; 3],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    polygon: Vec<[f64; 2]>,
    cases: Vec<CaseRead>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CaseRead {
    now_ns: i64,
    /// Null when no cloud came; the field is required.
    #[serde(deserialize_with = "Option::deserialize")]
    cloud_age_ns: Option<i64>,
    velocity: [f64; 3],
    action: Action,
    polygon_name: String,
    allowed: [f64; 3],
}

/// The action the monitor takes, by Nav2's action types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Action {
    DoNothing,
    Stop,
    Slowdown,
    Approach,
    Limit,
}

/// One case as the monitor decided it.
#[derive(Debug, Clone, PartialEq)]
pub struct Case {
    pub now_ns: i64,
    pub cloud_age_ns: Option<i64>,
    /// The points of the case's cloud as the reference program hands them to the monitor's
    /// source, each coordinate in single precision.
    pub points: Vec<[f64; 3]>,
    pub velocity: [f64; 3],
    pub action: Action,
    /// The polygon that gave the action, `invalid source` for a stop by the source, or
    /// empty for no action.
    pub polygon_name: String,
    /// The velocity the monitor allows, linear x, linear y and angular z.
    pub allowed: [f64; 3],
}

/// A collision monitor scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub monitor: MonitorParameters,
    pub max_step_down: f64,
    pub points_are: PointsAre,
    /// The polygon's points as the monitor parsed them.
    pub polygon: Vec<[f64; 2]>,
    pub cases: Vec<Case>,
}

impl Cases {
    /// Every collision monitor scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read every case's time,
    /// cloud age and velocity.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::CollisionMonitor)
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
        kind: ThisKind::CollisionMonitor,
        monitor,
        max_step_down,
        points_are,
        cases,
    } = scenario;
    let Reference {
        kind: ThisKind::CollisionMonitor,
        polygon,
        cases: cases_read,
    } = reference;
    assert_eq!(
        cases.len(),
        cases_read.len(),
        "{name}: the reference decided every case"
    );
    let cases = cases
        .into_iter()
        .zip(cases_read)
        .enumerate()
        .map(|(index, (input, read))| {
            let what = format!("{name} case {index}");
            assert_eq!(
                (read.now_ns, read.cloud_age_ns, read.velocity),
                (input.now_ns, input.cloud_age_ns, input.velocity),
                "{what}: the reference read the case"
            );
            assert!(
                input.cloud_age_ns.is_some() || input.points.is_empty(),
                "{what}: a case with no cloud holds no points, as the reference program hands \
                 its source none"
            );
            Case {
                now_ns: input.now_ns,
                cloud_age_ns: input.cloud_age_ns,
                points: points_handed_to_source(&input),
                velocity: input.velocity,
                action: read.action,
                polygon_name: read.polygon_name,
                allowed: read.allowed,
            }
        })
        .collect();
    Cases {
        name,
        monitor,
        max_step_down,
        points_are,
        polygon,
        cases,
    }
}

/// The points of a case's cloud as the reference program hands them to the monitor's
/// source, in a PointCloud2 of float x, y and z: each coordinate of the scenario's points
/// in single precision; none when no cloud came.
fn points_handed_to_source(input: &CaseInput) -> Vec<[f64; 3]> {
    match input.cloud_age_ns {
        Some(_) => input
            .points
            .iter()
            .map(|point| point.map(|c| f64::from(c as f32)))
            .collect(),
        None => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "pure_pursuit", "monitor": {}, "max_step_down": 0.1,
            "points_are": "depth", "cases": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    // A case with a cloud hands the source each coordinate of its points in single
    // precision, bit for bit, and a case with no cloud hands it none.
    #[test]
    fn a_cases_points_are_the_scenarios_in_single_precision() {
        let with_cloud = CaseInput {
            now_ns: 0,
            cloud_age_ns: Some(0),
            points: vec![[0.1, -0.2, 0.3]],
            velocity: [0.0; 3],
        };
        let handed: Vec<[u64; 3]> = points_handed_to_source(&with_cloud)
            .iter()
            .map(|point| point.map(f64::to_bits))
            .collect();
        let single = [0.1_f32, -0.2_f32, 0.3_f32].map(|c| f64::from(c).to_bits());
        assert_eq!(handed, [single]);
        let no_cloud = CaseInput {
            cloud_age_ns: None,
            points: Vec::new(),
            ..with_cloud
        };
        assert!(points_handed_to_source(&no_cloud).is_empty());
    }

    // A case with points and no cloud is refused: the reference program hands its source
    // none of them.
    #[test]
    #[should_panic(expected = "a case with no cloud holds no points")]
    fn a_case_with_points_and_no_cloud_is_refused() {
        let name = "collision_monitor_approach".to_string();
        let mut scenario: Scenario = read_scenario(&name);
        let reference: Reference = read_reference(&name);
        scenario
            .cases
            .iter_mut()
            .find(|case| case.cloud_age_ns.is_none())
            .expect("the approach scenario holds a case with no cloud")
            .points
            .push([1.0, 0.0, 0.2]);
        pair(name, scenario, reference);
    }

    #[test]
    fn every_collision_monitor_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a collision monitor scenario"
        );
        for cases in scenarios {
            assert!(
                cases.polygon.len() >= 4,
                "{} holds the monitor's polygon",
                cases.name
            );
            assert!(!cases.cases.is_empty(), "{} holds a case", cases.name);
        }
    }
}
