//! Planning scene. The scenario holds a voxel map, calls of motion_planning's scene
//! members on a robot, and configurations of the robot to measure the scene at. The
//! reference is MoveIt's planning scene: it holds the voxel map as its octomap from the
//! start, takes the calls in order, as collision object and attached collision object
//! messages and entries of its allowed collision matrix, each call of
//! apply_attached_collision_object with the joints it names at their positions, and gives
//! its objects, its attached objects and its allowed pairs that name an object, as its
//! planning scene message holds them. At each configuration, FCL's collision environment
//! measures the signed distance of each pair of a body of the robot, a link or an
//! attached object, and an object or the octomap, named "occupancy_map", and of an
//! attached object and a link it may not touch; and the reference tells, of each pair FCL
//! finds apart, whether one body lies wholly inside the other's closed mesh. MoveIt's
//! path check (`PlanningScene::isPathValid`) checks each path of the scenario, the rest
//! of the robot standing still, and gives the waypoints it finds invalid; and MoveIt's
//! shape mask, as the octomap updater cuts the objects out of a cloud, tells whether each
//! probe of the scenario lies inside an object the calls leave standing, grown by each
//! padding.

use crate::{Kind, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

/// The `kind` of a planning scene scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "planning_scene")]
    PlanningScene,
}

/// A shape as motion_planning writes it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Shape {
    pub kind: String,
    pub dimensions: Vec<f64>,
    pub mesh_vertices: Vec<f64>,
    pub mesh_triangles: Vec<u32>,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// One entry of apply_collision_objects, as motion_planning writes it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Object {
    pub id: String,
    pub operation: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub shapes: Vec<Shape>,
}

/// One call of the scenario, as motion_planning's scene members take it.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
pub enum Operation {
    /// apply_collision_objects.
    Objects { objects: Vec<Object> },
    /// set_allowed_collision.
    Allow {
        name: String,
        other_names: Vec<String>,
        allowed: bool,
    },
    /// apply_attached_collision_object, made with each joint `positions` names at its
    /// position.
    Attached {
        object_id: String,
        link_name: String,
        touch_links: Vec<String>,
        operation: String,
        positions: BTreeMap<String, f64>,
    },
    /// The voxels of `keys` marked occupied, as a depth frame marks them, each the voxel
    /// from `key * voxel_size` to `(key + 1) * voxel_size` on each axis.
    Mark { keys: Vec<[i32; 3]> },
}

/// The voxel map the scene holds from the start: the side of its voxels, in meters, and
/// the key of each occupied voxel, the voxel from `key * voxel_size` to
/// `(key + 1) * voxel_size` on each axis.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Octomap {
    pub voxel_size: f64,
    pub keys: Vec<[i32; 3]>,
}

/// A robot, its voxel map, the calls to apply to its scene in order, and the
/// configurations to measure the scene at, each a position per joint name.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root.
    meshes: String,
    tip_link: String,
    octomap: Octomap,
    operations: Vec<Operation>,
    configurations: Vec<BTreeMap<String, f64>>,
    /// Paths of the arm the configurations pose, each its waypoints in order, the first
    /// its start.
    paths: Vec<Vec<BTreeMap<String, f64>>>,
    /// The padding of MoveIt's links in its second check of each path, and the clearance
    /// of this crate's: the motion planner's launch min_clearance_m.
    path_padding: f64,
    cut_out: CutOutProbes,
}

/// The paddings to grow the objects by, and the points to test against the objects so
/// grown, in the model frame.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CutOutProbes {
    paddings: Vec<GrowthPadding>,
    probes: Vec<[f64; 3]>,
}

/// A padding of MoveIt's shape mask: a scale, and an offset in meters.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GrowthPadding {
    pub scale: f64,
    pub offset: f64,
}

/// MoveIt's answer to the probes: for each padding, whether each probe lies inside an
/// object grown by it.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CutOutAnswer {
    inside: Vec<Vec<bool>>,
}

/// The cut-out probes of a scenario and MoveIt's shape mask's answer to them: for each
/// padding, whether each probe lies inside a shape of an object the calls leave standing,
/// grown by it.
pub struct CutOut {
    pub paddings: Vec<GrowthPadding>,
    pub probes: Vec<[f64; 3]>,
    /// One row per padding, one answer per probe.
    pub inside: Vec<Vec<bool>>,
}

/// A shape as MoveIt's message gives it back.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveItShape {
    pub kind: String,
    pub dimensions: Vec<f64>,
    pub mesh_vertices: Vec<f64>,
    pub mesh_triangles: Vec<u32>,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// An object of MoveIt's scene: its pose in the model frame.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveItObject {
    pub id: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub shapes: Vec<MoveItShape>,
}

/// An attached object of MoveIt's scene: its pose in the frame of its link.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveItAttached {
    pub id: String,
    pub link: String,
    /// Sorted.
    pub touch_links: Vec<String>,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub shapes: Vec<MoveItShape>,
}

/// MoveIt's scene after the calls: its objects and its attached objects, each sorted by
/// id, and its allowed pairs that name an object, by motion_planning's names, each sorted,
/// in sorted order.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveItScene {
    pub objects: Vec<MoveItObject>,
    pub attached: Vec<MoveItAttached>,
    pub allowed: Vec<[String; 2]>,
}

/// MoveIt's scene, the pairs FCL measures, by their names in sorted order, and for each
/// configuration, in the scenario's order, the positions as the reference read them, and
/// one distance and one insideness per pair, in the order of the pairs.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    scene: MoveItScene,
    pairs: Vec<[String; 2]>,
    configurations: Vec<Row>,
    paths: Vec<CheckedPath>,
    cut_out: CutOutAnswer,
    one_link_pairs: Vec<OneLinkPair>,
}

/// MoveIt's check of a path: the indices of the waypoints it finds invalid, the start 0,
/// and its dense probe of the path; and its check with its links padded.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CheckedPath {
    invalid: Vec<usize>,
    dense: Dense,
    padded: Padded,
}

/// MoveIt's check of a path with its links padded by the scenario's path padding against
/// the objects and the octomap, the robot meeting itself unpadded: the indices of the
/// waypoints it finds invalid, and whether its dense probe finds a state in collision.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Padded {
    pub invalid: Vec<usize>,
    pub colliding: bool,
}

/// MoveIt's dense probe of a path, straight in the joints between its waypoints, a step
/// of 1 mrad of the joint that moves most: whether its collision check finds a probe in
/// collision, and, for a path in contact nowhere, the least distance it measures from the
/// robot to the objects and the octomap, in meters; -1 for a path in collision.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Dense {
    pub colliding: bool,
    pub clearance: f64,
}

/// A path of the scenario and MoveIt's check of it.
pub struct Path {
    /// The waypoints, each a position per joint name, the first the path's start.
    pub waypoints: Vec<BTreeMap<String, f64>>,
    /// The indices of the waypoints MoveIt's path check finds invalid, in order.
    pub invalid: Vec<usize>,
    pub dense: Dense,
    pub padded: Padded,
}

/// Two objects attached to one link, after the calls: FCL's signed distance between
/// them, and whether MoveIt's collision check of the robot against itself finds them in
/// contact.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OneLinkPair {
    pub pair: [String; 2],
    pub distance: f64,
    pub contact: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Row {
    positions: BTreeMap<String, f64>,
    distances: Vec<f64>,
    inside: Vec<bool>,
}

/// FCL's reading of one pair at a configuration.
pub struct Measured {
    /// The names of the pair's bodies, each a link, an object or an attached object, in
    /// sorted order.
    pub a: String,
    pub b: String,
    /// FCL's smallest signed distance between the bodies, in meters: where they touch,
    /// the depth of the deepest touching pair of triangles.
    pub distance: f64,
    /// Whether one body lies wholly inside the other's closed mesh, which FCL measures by
    /// its triangles: only where FCL finds the bodies apart.
    pub inside: bool,
}

/// One configuration of the robot and FCL's readings at it.
pub struct Configuration {
    /// The position of each joint the configuration sets, by name.
    pub positions: BTreeMap<String, f64>,
    /// One reading per pair FCL measures.
    pub measured: Vec<Measured>,
}

/// A planning scene scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The directory of the collision meshes.
    pub meshes: PathBuf,
    /// The grasp link of the arm the configurations pose.
    pub tip_link: String,
    pub octomap: Octomap,
    pub operations: Vec<Operation>,
    pub scene: MoveItScene,
    pub configurations: Vec<Configuration>,
    pub paths: Vec<Path>,
    /// The padding of MoveIt's second check of each path, and the clearance of this
    /// crate's.
    pub path_padding: f64,
    pub cut_out: CutOut,
    pub one_link_pairs: Vec<OneLinkPair>,
}

impl Cases {
    /// Every planning scene scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: one row per configuration, which read its
    /// positions, holds one distance and one insideness per pair.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::PlanningScene)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::PlanningScene,
            urdf,
            meshes,
            tip_link,
            octomap,
            operations,
            configurations,
            paths,
            path_padding,
            cut_out: CutOutProbes { paddings, probes },
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::PlanningScene,
            scene,
            pairs,
            configurations: rows,
            paths: checked,
            cut_out: CutOutAnswer { inside },
            one_link_pairs,
        } = read_reference(&name);
        assert!(
            inside.len() == paddings.len() && inside.iter().all(|row| row.len() == probes.len()),
            "{name}: one answer per padding and probe"
        );
        assert_eq!(paths.len(), checked.len(), "{name}: one check per path");
        let paths = paths
            .into_iter()
            .zip(checked)
            .map(
                |(
                    waypoints,
                    CheckedPath {
                        invalid,
                        dense,
                        padded,
                    },
                )| {
                    assert!(
                        invalid
                            .iter()
                            .chain(&padded.invalid)
                            .all(|&index| index < waypoints.len()),
                        "{name}: MoveIt names a waypoint of each path"
                    );
                    Path {
                        waypoints,
                        invalid,
                        dense,
                        padded,
                    }
                },
            )
            .collect();
        Cases {
            urdf: read_urdf(&name, &urdf),
            meshes: read_mesh_directory(&name, &meshes),
            tip_link,
            octomap,
            operations,
            scene,
            configurations: pair(&name, &pairs, configurations, rows),
            paths,
            path_padding,
            cut_out: CutOut {
                paddings,
                probes,
                inside,
            },
            one_link_pairs,
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
    let mut distinct = BTreeSet::new();
    for [a, b] in pairs {
        assert!(
            a < b,
            "{name}: FCL's pair {a} and {b} names its bodies in sorted order"
        );
        assert!(
            distinct.insert((a, b)),
            "{name}: FCL measures {a} and {b} twice"
        );
    }
    configurations
        .into_iter()
        .zip(rows)
        .enumerate()
        .map(|(index, (positions, row))| {
            let what = format!("{name} configuration {index}");
            assert_eq!(
                row.positions, positions,
                "{what}: the reference read the positions"
            );
            assert_eq!(
                row.distances.len(),
                pairs.len(),
                "{what}: one distance per pair"
            );
            assert_eq!(
                row.inside.len(),
                pairs.len(),
                "{what}: one insideness per pair"
            );
            let measured = pairs
                .iter()
                .zip(row.distances)
                .zip(row.inside)
                .map(|(([a, b], distance), inside)| {
                    assert!(
                        !inside || distance > 0.0,
                        "{what}: {a} and {b} lie inside one another only where FCL finds \
                         them apart"
                    );
                    Measured {
                        a: a.clone(),
                        b: b.clone(),
                        distance,
                        inside,
                    }
                })
                .collect();
            Configuration {
                positions,
                measured,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(positions: &[(&str, f64)], distances: &[f64], inside: &[bool]) -> Row {
        Row {
            positions: positions
                .iter()
                .map(|&(joint, position)| (joint.to_owned(), position))
                .collect(),
            distances: distances.to_vec(),
            inside: inside.to_vec(),
        }
    }

    fn one_pair() -> [[String; 2]; 1] {
        [["a".to_owned(), "b".to_owned()]]
    }

    #[test]
    fn a_row_reads_one_distance_and_one_insideness_per_pair() {
        let pairs = [
            ["a".to_owned(), "b".to_owned()],
            ["a".to_owned(), "c".to_owned()],
        ];
        let positions = BTreeMap::from([("j".to_owned(), 0.5)]);
        let configurations = pair(
            "scenario",
            &pairs,
            vec![positions.clone()],
            vec![row(&[("j", 0.5)], &[-0.25, 0.75], &[false, true])],
        );
        let [configuration] = configurations.as_slice() else {
            panic!("one configuration");
        };
        assert_eq!(configuration.positions, positions);
        let read: Vec<(&str, &str, f64, bool)> = configuration
            .measured
            .iter()
            .map(|m| (m.a.as_str(), m.b.as_str(), m.distance, m.inside))
            .collect();
        assert_eq!(read, [("a", "b", -0.25, false), ("a", "c", 0.75, true)]);
    }

    #[test]
    #[should_panic(expected = "names its bodies in sorted order")]
    fn a_pair_out_of_order_does_not_pair() {
        pair(
            "scenario",
            &[["b".to_owned(), "a".to_owned()]],
            vec![],
            vec![],
        );
    }

    #[test]
    #[should_panic(expected = "FCL measures a and b twice")]
    fn a_pair_measured_twice_does_not_pair() {
        let pairs = [
            ["a".to_owned(), "b".to_owned()],
            ["a".to_owned(), "b".to_owned()],
        ];
        pair("scenario", &pairs, vec![], vec![]);
    }

    #[test]
    #[should_panic(expected = "one distance per pair")]
    fn a_row_missing_a_distance_does_not_pair() {
        pair(
            "scenario",
            &one_pair(),
            vec![BTreeMap::new()],
            vec![row(&[], &[], &[false])],
        );
    }

    #[test]
    #[should_panic(expected = "one insideness per pair")]
    fn a_row_missing_an_insideness_does_not_pair() {
        pair(
            "scenario",
            &one_pair(),
            vec![BTreeMap::new()],
            vec![row(&[], &[1.0], &[])],
        );
    }

    #[test]
    #[should_panic(expected = "only where FCL finds them apart")]
    fn a_pair_in_contact_and_inside_does_not_pair() {
        pair(
            "scenario",
            &one_pair(),
            vec![BTreeMap::new()],
            vec![row(&[], &[-0.5], &[true])],
        );
    }

    #[test]
    #[should_panic(expected = "the reference read the positions")]
    fn a_row_that_read_other_positions_does_not_pair() {
        let positions = BTreeMap::from([("j".to_owned(), 0.5)]);
        pair(
            "scenario",
            &one_pair(),
            vec![positions],
            vec![row(&[("j", 0.25)], &[1.0], &[false])],
        );
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let scenario = |kind: &str| {
            format!(
                r#"{{"kind": "{kind}", "urdf": "a.urdf", "meshes": "meshes", "tip_link": "tip",
                    "octomap": {{"voxel_size": 0.04, "keys": []}}, "operations": [],
                    "configurations": [], "paths": [], "path_padding": 0.04,
                    "cut_out": {{"paddings": [], "probes": []}}}}"#
            )
        };
        assert!(serde_json::from_str::<Scenario>(&scenario("self_distance")).is_err());
        assert!(serde_json::from_str::<Scenario>(&scenario("planning_scene")).is_ok());
    }

    #[test]
    fn every_planning_scene_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a planning scene scenario"
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
