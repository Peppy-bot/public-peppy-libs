//! Cartesian paths. The scenario holds cases of one arm among boxes of voxels and collision
//! objects, each with a start, the link that follows its lines, the end of each line an
//! offset from that link's pose at the start, so each side places it by its own forward
//! kinematics, whether it avoids collisions, a seed, and motion_planning_core's answer, which
//! its example `cartesian_scenes` gives `tools/draw_cartesian_paths.py`. The reference is
//! MoveIt's Cartesian interpolator, as move_group's compute_cartesian_path service runs it
//! with KDL as the inverse kinematics, each state held clear of the case's voxels as an
//! octomap and of its collision objects where the case avoids collisions: the fraction of
//! each path, and the link's pose and the joints at each of its states.

use crate::arm_planning::{ArmChain, Held, VoxelBox};
use crate::planning_scene::Object;
use crate::{
    Kind, LinkPose, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names,
};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// The `kind` of a Cartesian paths scenario and of its reference output, which refuses every
/// other kind.
#[derive(Debug, Deserialize)]
enum ThisKind {
    #[serde(rename = "cartesian_paths")]
    CartesianPaths,
}

/// The arm, the map and the step of a scenario, and its cases. A scenario that
/// `tools/draw_cartesian_paths.py` has drawn holds no checked pairs, no step and no answer of
/// motion_planning_core yet; the example `cartesian_scenes` reads it so.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Paths {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    pub urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root.
    pub meshes: String,
    /// The arm the paths move.
    pub chains: [ArmChain; 1],
    /// The pairs of the robot's own links, by their names, that motion_planning_core's self
    /// model checks and MoveIt's collision matrix checks too; empty where the paths check no
    /// such pair, MoveIt's matrix allowing each.
    pub checked_pairs: Option<Vec<[String; 2]>>,
    pub voxel_size: f64,
    /// The half side of the cube about the robot's origin that motion_planning_core's map
    /// covers.
    pub map_half_side: f64,
    /// The clearance of the paths: motion_planning_core's min_clearance_m, and the padding of
    /// every link of MoveIt's robot against the octomap and the objects, the robot meeting
    /// itself unpadded.
    pub min_clearance: f64,
    /// The longest step of a line (m): motion_planning_core's, and the request's max_step of
    /// compute_cartesian_path, 3.5 rad of turn for each meter of it (`MaxEEFStep`).
    pub step: Option<f64>,
    pub cases: Vec<PathRequest>,
}

/// A pose given from the link's pose at a case's start: a translation in the robot frame, and
/// a rotation vector in the link's frame.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Offset {
    pub translation: [f64; 3],
    pub rotation: [f64; 3],
}

/// One case: the path's start, the link that follows its lines, the poses they run through,
/// its boxes, its collision objects, the objects the robot's grasp links hold, the pairs
/// allowed to touch, whether its states keep clear of them, its seed, and `ours`,
/// motion_planning_core's answer to it: an `Option<Ours>` in a scenario, null in one the
/// example `cartesian_scenes` reads before it answers, and an [`Ours`] in a [`PathCase`].
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PathRequest<O = Option<Ours>> {
    pub start: Vec<f64>,
    /// The chain's tip link, the grasp link, or a link fixed to the tip link that the chain's
    /// last joint moves.
    pub link: String,
    /// The pose each line ends at, in turn.
    pub offsets: Vec<Offset>,
    pub boxes: Vec<VoxelBox>,
    /// The collision objects, each as apply_collision_objects adds it, in the robot frame.
    pub objects: Vec<Object>,
    /// The objects the robot's grasp links hold through the path.
    pub attached: Vec<Held>,
    /// The pairs of names allowed to touch, as set_allowed_collision allows them.
    pub allowed: Vec<[String; 2]>,
    /// Whether the states keep clear of the case's boxes, objects and held objects: the
    /// request's avoid_collisions of compute_cartesian_path.
    pub avoid_collisions: bool,
    /// The seed of both inverse kinematics' draws.
    pub seed: u32,
    pub ours: O,
}

impl PathRequest {
    /// The case with the answer it holds; a case of `what` that holds none panics.
    fn answered(self, what: &str) -> PathRequest<Ours> {
        let PathRequest {
            start,
            link,
            offsets,
            boxes,
            objects,
            attached,
            allowed,
            avoid_collisions,
            seed,
            ours,
        } = self;
        PathRequest {
            start,
            link,
            offsets,
            boxes,
            objects,
            attached,
            allowed,
            avoid_collisions,
            seed,
            ours: ours.unwrap_or_else(|| panic!("{what}: motion_planning_core's answer")),
        }
    }
}

/// motion_planning_core's answer to a case: the share of its lines covered with the waypoints
/// of its path after the start, or the name of its failure.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(untagged, deny_unknown_fields)]
pub enum Ours {
    Path {
        fraction: f64,
        waypoints: Vec<Vec<f64>>,
    },
    Failure {
        failure: String,
    },
}

/// MoveIt's Cartesian path of a case: the share of its lines covered, and the link's pose and
/// the joints at each of its states, the start first. The states of a step that fails count
/// in no fraction, and those of the halves of it that passed stand in the path.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveIt {
    pub fraction: f64,
    pub poses: Vec<LinkPose>,
    pub states: Vec<Vec<f64>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    cases: Vec<MoveIt>,
}

/// A case, with motion_planning_core's answer to it, and MoveIt's path of it.
#[derive(Debug)]
pub struct PathCase {
    pub request: PathRequest<Ours>,
    pub moveit: MoveIt,
}

/// A Cartesian paths scenario paired with its reference output.
#[derive(Debug)]
pub struct Case {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The scenario's mesh directory.
    pub meshes: PathBuf,
    pub paths: Paths,
    /// The scenario's checked pairs.
    pub checked_pairs: Vec<[String; 2]>,
    /// The scenario's step.
    pub step: f64,
    pub cases: Vec<PathCase>,
}

impl Case {
    /// Every Cartesian paths scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: an answer of motion_planning_core in every case, and
    /// MoveIt's path of each from its start, a pose of the case's link and the joints at each
    /// state.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::CartesianPaths)
            .into_iter()
            .map(|name| {
                let paths: Paths = read_scenario(&name);
                let urdf = read_urdf(&name, &paths.urdf);
                let meshes = read_mesh_directory(&name, &paths.meshes);
                let reference = read_reference(&name);
                pair(name, urdf, meshes, paths, reference)
            })
            .collect()
    }
}

fn pair(name: String, urdf: String, meshes: PathBuf, paths: Paths, reference: Reference) -> Case {
    let Paths {
        kind: ThisKind::CartesianPaths,
        ..
    } = &paths;
    let Reference {
        kind: ThisKind::CartesianPaths,
        cases: answers,
    } = reference;
    let checked_pairs = paths
        .checked_pairs
        .clone()
        .unwrap_or_else(|| panic!("{name}: the scenario's checked pairs"));
    let step = paths
        .step
        .unwrap_or_else(|| panic!("{name}: the scenario's step"));
    assert_eq!(
        paths.cases.len(),
        answers.len(),
        "{name}: an answer per case"
    );
    let cases = paths
        .cases
        .iter()
        .zip(answers)
        .enumerate()
        .map(|(index, (request, moveit))| {
            let what = format!("{name} case {index}");
            assert_eq!(
                moveit.poses.len(),
                moveit.states.len(),
                "{what}: MoveIt's pose and joints of each state"
            );
            assert!(
                moveit.poses.iter().all(|pose| pose.link == request.link),
                "{what}: MoveIt's poses of the case's link"
            );
            assert_eq!(
                moveit.states.first(),
                Some(&request.start),
                "{what}: MoveIt's path from the case's start"
            );
            PathCase {
                request: request.clone().answered(&what),
                moveit,
            }
        })
        .collect();
    Case {
        name,
        urdf,
        meshes,
        paths,
        checked_pairs,
        step,
        cases,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PATH: &str = r#"{"fraction": 0.5, "waypoints": [[1.0]]}"#;
    const FAILURE: &str = r#"{"failure": "start_in_collision"}"#;

    /// The text of a scenario of the kind `kind`, whose one member `chains` holds, and of one
    /// case, our answer to it `ours`.
    fn scenario(kind: &str, chains: &str, ours: &str) -> String {
        format!(
            r#"{{"kind": "{kind}", "urdf": "a.urdf", "meshes": "meshes", "chains": [{chains}],
                "checked_pairs": [], "voxel_size": 0.04, "map_half_side": 1.6,
                "min_clearance": 0.0, "step": 0.01,
                "cases": [{{"start": [0.0], "link": "tip",
                            "offsets": [{{"translation": [0.0, 0.0, 0.1],
                                          "rotation": [0.0, 0.0, 0.0]}}],
                            "boxes": [{{"first": [0, 0, 0], "last": [1, 1, 1]}}],
                            "objects": [], "attached": [], "allowed": [],
                            "avoid_collisions": true, "seed": 1, "ours": {ours}}}]}}"#
        )
    }

    const CHAIN: &str =
        r#"{"name": "arm", "base_link": "base", "tip_link": "tip", "joints": ["j"]}"#;

    fn paths(ours: &str) -> Paths {
        serde_json::from_str(&scenario("cartesian_paths", CHAIN, ours)).expect("parses")
    }

    /// The output of one case: MoveIt's fraction, its poses of `link` and its states.
    fn reference(link: &str, states: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "cartesian_paths", "cases": [
                {{"fraction": 1.0,
                  "poses": [{{"link": "{link}", "translation": [0.0, 0.0, 0.0],
                              "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]}}],
                  "states": {states}}}]}}"#
        ))
        .expect("parses")
    }

    fn paired(ours: &str, link: &str, states: &str) -> Case {
        pair(
            "scenario".into(),
            String::new(),
            PathBuf::new(),
            paths(ours),
            reference(link, states),
        )
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let of_kind = |kind| serde_json::from_str::<Paths>(&scenario(kind, CHAIN, PATH));
        assert!(of_kind("arm_planning").is_err());
        assert!(of_kind("cartesian_paths").is_ok());
    }

    #[test]
    fn a_scenario_of_two_chains_is_refused() {
        let chains = format!("{CHAIN}, {CHAIN}");
        assert!(
            serde_json::from_str::<Paths>(&scenario("cartesian_paths", &chains, PATH)).is_err()
        );
    }

    #[test]
    fn a_case_pairs_with_moveits_path_from_its_start() {
        let case = paired(PATH, "tip", "[[0.0]]");
        let path = &case.cases[0];
        assert_eq!(
            path.request.ours,
            Ours::Path {
                fraction: 0.5,
                waypoints: vec![vec![1.0]],
            }
        );
        assert_eq!(path.moveit.fraction, 1.0);
        assert_eq!(case.step, 0.01);
        let refused = paired(FAILURE, "tip", "[[0.0]]");
        assert_eq!(
            refused.cases[0].request.ours,
            Ours::Failure {
                failure: "start_in_collision".into(),
            }
        );
    }

    #[test]
    #[should_panic(expected = "MoveIt's path from the case's start")]
    fn a_path_of_moveit_from_another_start_does_not_pair() {
        paired(PATH, "tip", "[[1.0]]");
    }

    #[test]
    #[should_panic(expected = "MoveIt's pose and joints of each state")]
    fn a_path_of_moveit_with_more_states_than_poses_does_not_pair() {
        paired(PATH, "tip", "[[0.0], [1.0]]");
    }

    #[test]
    #[should_panic(expected = "MoveIt's poses of the case's link")]
    fn a_pose_of_another_link_does_not_pair() {
        paired(PATH, "base", "[[0.0]]");
    }

    #[test]
    #[should_panic(expected = "motion_planning_core's answer")]
    fn a_case_with_no_answer_of_ours_does_not_pair() {
        paired("null", "tip", "[[0.0]]");
    }

    #[test]
    fn an_answer_of_ours_is_a_path_or_a_failure() {
        let ours = |text: &str| serde_json::from_str::<Ours>(text);
        assert!(matches!(ours(PATH), Ok(Ours::Path { .. })));
        assert!(matches!(ours(FAILURE), Ok(Ours::Failure { .. })));
        assert!(ours(r#"{"fraction": 0.5}"#).is_err());
    }

    #[test]
    fn an_answer_that_leaves_out_moveits_states_is_refused() {
        let text = r#"{"kind": "cartesian_paths", "cases": [{"fraction": 1.0, "poses": []}]}"#;
        assert!(serde_json::from_str::<Reference>(text).is_err());
    }

    #[test]
    fn every_cartesian_paths_scenario_pairs_with_its_reference() {
        let cases = Case::load_all();
        assert!(
            !cases.is_empty(),
            "scenarios/ holds a Cartesian paths scenario"
        );
        for case in cases {
            assert!(!case.cases.is_empty(), "{} holds a case", case.name);
        }
    }
}
