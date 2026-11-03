//! Arm planning. The scenario holds scenes of an arm among boxes of voxels and collision
//! objects, each with a start, a goal and a seed, and motion_planning_core's answer to
//! each, which its example `plan_arm_scenes` gives `tools/draw_arm_planning.py`. The
//! reference is MoveIt's planning pipeline, with the OMPL planner of the scenario's
//! objective, its default request adapters and the response adapters
//! AddTimeOptimalParameterization and ValidateSolution, planning each scene with the
//! voxels as an octomap, among its collision objects and holding its attached object, to
//! the goal state itself, seeded from the scene and ended as the scenario's termination
//! says, and MoveIt's collision check and distance probing each planned path: MoveIt's
//! own, and motion_planning_core's with its corners rounded as MoveIt's time-optimal
//! trajectory generation rounds them. Both check every link that moves with the arm's
//! joints, the gripper's fingers among them, closed, as MoveIt's default state holds them.

use crate::planning_scene::Object;
use crate::{Kind, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names};
use serde::{Deserialize, Serialize};
use std::num::NonZeroU32;
use std::path::PathBuf;

/// The `kind` of an arm planning scenario and of its reference output, which refuses
/// every other kind.
#[derive(Debug, Deserialize)]
enum ThisKind {
    #[serde(rename = "arm_planning")]
    ArmPlanning,
}

/// The arm, the map and the plans' parameters of a scenario, and its scenes. A scenario
/// that `tools/draw_arm_planning.py` has drawn holds no probe spacing and no answer of
/// motion_planning_core yet; the example `plan_arm_scenes` reads it so.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenes {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    pub urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root.
    pub meshes: String,
    pub base_link: String,
    pub tip_link: String,
    pub joints: Vec<String>,
    pub voxel_size: f64,
    /// The half side of the cube about the robot's origin that motion_planning_core's
    /// map covers.
    pub map_half_side: f64,
    /// The clearance of the plans: motion_planning_core's min_clearance_m, and the padding
    /// of every link of MoveIt's robot against the octomap and the objects, the robot
    /// meeting itself unpadded.
    pub min_clearance: f64,
    /// The maximum deviation at which execute_joint_path and MoveIt's time-optimal path
    /// round a path's corners.
    pub corner_tolerance: f64,
    /// The tolerance of each joint of motion_planning_core's goal, above and below its
    /// target, above 0 as a goal's tolerance is. MoveIt plans to the goal state itself.
    pub goal_tolerance: f64,
    pub objective: Objective,
    pub termination: Termination,
    /// The time limit of each plan, motion_planning_core's and MoveIt's, in seconds: the
    /// request's allowed planning time.
    pub time_limit: f64,
    /// The iteration bound of each plan, as [`Termination`] applies it: the motion
    /// planner node's `max_iterations`.
    pub iterations: u64,
    /// The joints' L2 distance between two probes of a path: no point of a link moves a
    /// tenth of a voxel between them.
    pub probe_spacing: Option<f64>,
    pub scenes: Vec<Scene>,
}

/// What each plan optimizes, and the OMPL planner that plans for it: `any`, as
/// plan_joint_path's `objective` names it and the motion planner node plans it, and
/// `shortest`, a mode the harness measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Objective {
    /// The first valid path found: RRTConnect.
    Any,
    /// The path's joint-space length, shortened until the plan's time ends: RRTstar.
    Shortest,
}

/// What ends each plan, motion_planning_core's and MoveIt's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Termination {
    /// The scenario's `iterations` (MoveIt's termination condition Iteration), its
    /// `time_limit` far beyond them, so a plan repeats under its seed.
    Iterations,
    /// The scenario's `time_limit`, as a robot's plan ends: MoveIt's by the time alone,
    /// its default termination, and motion_planning_core's by `iterations` for `any`, as
    /// the motion planner node bounds it, and by no iteration count for `shortest`.
    Time,
}

/// The voxels from `first` to `last`, by their keys, on every axis: the voxel of key k
/// on an axis runs from k to k + 1 voxel sizes along it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VoxelBox {
    pub first: [i32; 3],
    pub last: [i32; 3],
}

impl Scenes {
    /// The grasp link of the other arm, which holds a scene's `other_arm_holds`: the tip
    /// link's mirror, OpenArm's name of it with `left` and `right` swapped, as
    /// arm_planning.cpp names it.
    pub fn other_tip_link(&self) -> String {
        let (side, other) = if self.tip_link.contains("left") {
            ("left", "right")
        } else {
            ("right", "left")
        };
        assert!(
            self.tip_link.contains(side),
            "the tip link {} names no side, left or right, of OpenArm",
            self.tip_link
        );
        self.tip_link.replacen(side, other, 1)
    }
}

/// One scene: its boxes, its collision objects, the object the arm holds, the object the
/// other arm holds, the pairs allowed to touch, the plan's start, goal and seed, and
/// `ours`, motion_planning_core's answer to it: an `Option<Ours>` in a scenario, null in
/// one the example `plan_arm_scenes` reads before it answers, and an [`Ours`] in a
/// [`SceneCase`].
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scene<O = Option<Ours>> {
    pub boxes: Vec<VoxelBox>,
    /// The collision objects, each as apply_collision_objects adds it, in the robot frame.
    pub objects: Vec<Object>,
    /// The object the arm's tip link holds through the plan; null for none.
    #[serde(deserialize_with = "Option::deserialize")]
    pub attached: Option<Held>,
    /// The object the other arm's grasp link holds ([`Scenes::other_tip_link`]), the
    /// other arm at its zero configuration; null for none.
    #[serde(deserialize_with = "Option::deserialize")]
    pub other_arm_holds: Option<Held>,
    /// The pairs of names allowed to touch, as set_allowed_collision allows them.
    pub allowed: Vec<[String; 2]>,
    pub start: Vec<f64>,
    pub goal: Vec<f64>,
    /// Both planners' seed, above 0: OMPL takes no seed of 0.
    pub seed: NonZeroU32,
    pub ours: O,
}

impl Scene {
    /// The scene with the answer it holds; a scene of `what` that holds none panics.
    fn answered(self, what: &str) -> Scene<Ours> {
        let Scene {
            boxes,
            objects,
            attached,
            other_arm_holds,
            allowed,
            start,
            goal,
            seed,
            ours,
        } = self;
        Scene {
            boxes,
            objects,
            attached,
            other_arm_holds,
            allowed,
            start,
            goal,
            seed,
            ours: ours.unwrap_or_else(|| panic!("{what}: motion_planning_core's answer")),
        }
    }
}

/// An object a grasp link holds: the object, its pose in the link's frame, and the links
/// it may touch, as apply_attached_collision_object attaches it with the arm at the
/// scene's start.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Held {
    pub object: Object,
    pub touch_links: Vec<String>,
}

/// motion_planning_core's answer to a scene: the waypoints of its path, or the name of
/// its failure.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(untagged, deny_unknown_fields)]
pub enum Ours {
    Path { waypoints: Vec<Vec<f64>> },
    Failure { failure: String },
}

/// MoveIt's collision check and distance along a path, probed at the scenario's spacing.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probed {
    /// The probes MoveIt finds in collision, its links padded by the scenario's clearance.
    pub collisions: u64,
    /// The smallest distance MoveIt measures from the arm to the voxels and the objects,
    /// unpadded: -1 where a probe is in collision, as MoveIt's distanceToCollisionUnpadded
    /// answers it, and in place of a distance below -1, which FCL can read of a body that
    /// touches the octomap.
    pub clearance: f64,
    /// The smallest distance MoveIt measures from the object the arm holds to the voxels
    /// and the objects, unpadded and held at -1 or above as `clearance` is; none for a
    /// scene where the arm holds none.
    pub held_clearance: Option<f64>,
    /// The path's length in the joints' L2 norm.
    pub length: f64,
}

/// MoveIt's answer to a scene as the reference writes it: its error code, and the probe
/// of its path, null where it planned none.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MoveItRead {
    status: i32,
    #[serde(deserialize_with = "Option::deserialize")]
    collisions: Option<u64>,
    #[serde(deserialize_with = "Option::deserialize")]
    clearance: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    held_clearance: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    length: Option<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SceneAnswer {
    moveit: MoveItRead,
    #[serde(deserialize_with = "Option::deserialize")]
    ours: Option<Probed>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    scenes: Vec<SceneAnswer>,
}

/// MoveIt's error code of a plan that succeeds.
pub const MOVEIT_SUCCESS: i32 = 1;

/// MoveIt's answer to a scene: its error code, and the probe of its path when it planned
/// one.
#[derive(Debug)]
pub struct MoveIt {
    pub status: i32,
    pub probed: Option<Probed>,
}

/// A scene, with motion_planning_core's answer to it, and what the reference found of it.
#[derive(Debug)]
pub struct SceneCase {
    pub scene: Scene<Ours>,
    pub moveit: MoveIt,
    /// MoveIt's probe of motion_planning_core's path, when it planned one.
    pub ours_probed: Option<Probed>,
}

/// An arm planning scenario paired with its reference output.
#[derive(Debug)]
pub struct Case {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The scenario's mesh directory.
    pub meshes: PathBuf,
    pub scenes: Scenes,
    /// The scenario's probe spacing.
    pub probe_spacing: f64,
    pub cases: Vec<SceneCase>,
}

impl Case {
    /// Every arm planning scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: an answer of motion_planning_core in every
    /// scene, MoveIt's probe of its path exactly where it is a path, and the probe of
    /// MoveIt's own path exactly where MoveIt succeeds.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ArmPlanning)
            .into_iter()
            .map(|name| {
                let scenes: Scenes = read_scenario(&name);
                let urdf = read_urdf(&name, &scenes.urdf);
                let meshes = read_mesh_directory(&name, &scenes.meshes);
                let reference = read_reference(&name);
                pair(name, urdf, meshes, scenes, reference)
            })
            .collect()
    }
}

fn pair(name: String, urdf: String, meshes: PathBuf, scenes: Scenes, reference: Reference) -> Case {
    let Scenes {
        kind: ThisKind::ArmPlanning,
        ..
    } = &scenes;
    let Reference {
        kind: ThisKind::ArmPlanning,
        scenes: answers,
    } = reference;
    let probe_spacing = scenes
        .probe_spacing
        .unwrap_or_else(|| panic!("{name}: the scenario's probe spacing"));
    assert_eq!(
        scenes.scenes.len(),
        answers.len(),
        "{name}: an answer per scene"
    );
    let cases = scenes
        .scenes
        .iter()
        .zip(answers)
        .enumerate()
        .map(|(index, (scene, answer))| {
            let what = format!("{name} scene {index}");
            let scene = scene.clone().answered(&what);
            assert_eq!(
                answer.ours.is_some(),
                matches!(scene.ours, Ours::Path { .. }),
                "{what}: MoveIt's probe of our path exactly where ours is a path"
            );
            SceneCase {
                scene,
                moveit: moveit_of(&what, answer.moveit),
                ours_probed: answer.ours,
            }
        })
        .collect();
    Case {
        name,
        urdf,
        meshes,
        scenes,
        probe_spacing,
        cases,
    }
}

/// MoveIt's answer `read`, whose probe the reference gives exactly where MoveIt succeeds.
fn moveit_of(what: &str, read: MoveItRead) -> MoveIt {
    let MoveItRead {
        status,
        collisions,
        clearance,
        held_clearance,
        length,
    } = read;
    let probed = match (collisions, clearance, length) {
        (Some(collisions), Some(clearance), Some(length)) => Some(Probed {
            collisions,
            clearance,
            held_clearance,
            length,
        }),
        (None, None, None) if held_clearance.is_none() => None,
        _ => panic!("{what}: MoveIt's probe whole or not at all"),
    };
    assert_eq!(
        probed.is_some(),
        status == MOVEIT_SUCCESS,
        "{what}: MoveIt's probe exactly where MoveIt succeeds"
    );
    MoveIt { status, probed }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A scenario of one scene, our answer to it `ours`.
    fn scenes(ours: &str) -> Scenes {
        serde_json::from_str(&scenario("arm_planning", "any", "iterations", ours)).expect("parses")
    }

    /// The text of a scenario of the kind `kind`, its plans for `objective` ended by
    /// `termination`, and of one scene, our answer to it `ours`.
    fn scenario(kind: &str, objective: &str, termination: &str, ours: &str) -> String {
        format!(
            r#"{{"kind": "{kind}", "urdf": "a.urdf", "meshes": "meshes",
                "base_link": "base", "tip_link": "tip", "joints": ["j"], "voxel_size": 0.04,
                "map_half_side": 1.6, "min_clearance": 0.0, "corner_tolerance": 0.1,
                "goal_tolerance": 0.01, "objective": "{objective}",
                "termination": "{termination}", "time_limit": 5.0, "iterations": 10,
                "probe_spacing": 0.01,
                "scenes": [{{"boxes": [{{"first": [0, 0, 0], "last": [1, 1, 1]}}],
                             "objects": [], "attached": null, "other_arm_holds": null,
                             "allowed": [], "start": [0.0],
                             "goal": [1.0], "seed": 1, "ours": {ours}}}]}}"#
        )
    }

    const PATH: &str = r#"{"waypoints": [[0.0], [1.0]]}"#;
    const FAILURE: &str = r#"{"failure": "no_path"}"#;
    const PROBED: &str =
        r#"{"collisions": 0, "clearance": 0.03125, "held_clearance": null, "length": 1.0}"#;

    /// The output of one scene: MoveIt's `status` and the fields of its probe, and the
    /// probe of ours.
    fn reference(status: i32, moveit_probe: &str, ours: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "arm_planning", "scenes": [
                {{"moveit": {{"status": {status}, {moveit_probe}}}, "ours": {ours}}}]}}"#
        ))
        .expect("parses")
    }

    const MOVEIT_PROBED: &str =
        r#""collisions": 0, "clearance": 0.03125, "held_clearance": null, "length": 1.0"#;
    const MOVEIT_NONE: &str =
        r#""collisions": null, "clearance": null, "held_clearance": null, "length": null"#;

    fn paired(ours: &str, status: i32, moveit_probe: &str, ours_probe: &str) -> Case {
        pair(
            "scenario".into(),
            String::new(),
            PathBuf::new(),
            scenes(ours),
            reference(status, moveit_probe, ours_probe),
        )
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let of_kind =
            |kind| serde_json::from_str::<Scenes>(&scenario(kind, "any", "iterations", PATH));
        assert!(of_kind("planning").is_err());
        assert!(of_kind("arm_planning").is_ok());
    }

    #[test]
    fn a_scenario_names_its_objective_and_its_termination() {
        let planned = |objective, termination| {
            serde_json::from_str::<Scenes>(&scenario("arm_planning", objective, termination, PATH))
                .map(|s| (s.objective, s.termination))
        };
        assert_eq!(
            planned("any", "iterations").ok(),
            Some((Objective::Any, Termination::Iterations))
        );
        assert_eq!(
            planned("shortest", "time").ok(),
            Some((Objective::Shortest, Termination::Time))
        );
        assert!(planned("fastest", "time").is_err());
        assert!(planned("any", "deadline").is_err());
    }

    // The other arm's grasp link is the tip link's mirror, left and right swapped, either
    // way; a tip link that names no side has none.
    #[test]
    fn the_other_arm_mirrors_the_tip_link() {
        let with_tip = |tip: &str| -> Scenes {
            serde_json::from_str(
                &scenario("arm_planning", "any", "iterations", PATH)
                    .replace(r#""tip_link": "tip""#, &format!(r#""tip_link": "{tip}""#)),
            )
            .expect("a scenario")
        };
        assert_eq!(
            with_tip("openarm_left_tcp").other_tip_link(),
            "openarm_right_tcp"
        );
        assert_eq!(
            with_tip("openarm_right_tcp").other_tip_link(),
            "openarm_left_tcp"
        );
        let sideless = with_tip("tip");
        assert!(std::panic::catch_unwind(|| sideless.other_tip_link()).is_err());
    }

    #[test]
    fn a_scene_seeded_with_0_is_refused() {
        let text = r#"{"boxes": [], "objects": [], "attached": null, "other_arm_holds": null,
                             "allowed": [], "start": [0.0],
            "goal": [1.0], "seed": 0, "ours": null}"#;
        assert!(serde_json::from_str::<Scene>(text).is_err());
    }

    #[test]
    fn a_scene_pairs_with_the_answers_of_moveit_and_its_probe_of_ours() {
        let case = paired(PATH, MOVEIT_SUCCESS, MOVEIT_PROBED, PROBED);
        let scene = &case.cases[0];
        assert_eq!(scene.moveit.status, MOVEIT_SUCCESS);
        assert!(scene.moveit.probed.is_some() && scene.ours_probed.is_some());
        assert_eq!(case.probe_spacing, 0.01);
        let refused = paired(FAILURE, -10, MOVEIT_NONE, "null");
        assert!(refused.cases[0].moveit.probed.is_none() && refused.cases[0].ours_probed.is_none());
    }

    #[test]
    #[should_panic(expected = "MoveIt's probe of our path exactly where ours is a path")]
    fn a_probe_of_a_failure_of_ours_does_not_pair() {
        paired(FAILURE, MOVEIT_SUCCESS, MOVEIT_PROBED, PROBED);
    }

    #[test]
    #[should_panic(expected = "MoveIt's probe exactly where MoveIt succeeds")]
    fn a_failure_of_moveit_with_a_probe_does_not_pair() {
        paired(PATH, -1, MOVEIT_PROBED, PROBED);
    }

    #[test]
    fn an_answer_that_leaves_out_moveits_probe_is_refused() {
        let text = r#"{"kind": "arm_planning", "scenes": [
            {"moveit": {"status": -1, "collisions": null, "clearance": null}, "ours": null}]}"#;
        assert!(serde_json::from_str::<Reference>(text).is_err());
    }

    #[test]
    fn every_arm_planning_scenario_pairs_with_its_reference() {
        let cases = Case::load_all();
        assert!(
            !cases.is_empty(),
            "scenarios/ holds an arm planning scenario"
        );
        for case in cases {
            assert!(!case.cases.is_empty(), "{} holds a scene", case.name);
        }
    }
}
