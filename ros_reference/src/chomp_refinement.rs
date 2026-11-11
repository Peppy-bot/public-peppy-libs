//! CHOMP refinement. The scenario holds scenes of one arm among boxes of voxels and collision
//! objects, each with a start, a seed, an input path from that start, the path
//! motion_planning_core's RRTConnect plans through the scene, and motion_planning_core's
//! CHOMP refinement of it, which its example `plan_arm_scenes` gives
//! `tools/draw_chomp_refinement.py`. The reference is MoveIt's CHOMP optimizer at the
//! parameters MoveIt's CHOMP plugin loads, refining each input path in the scene's hybrid
//! collision environment, seeded from the scene: its error code and its waypoints, and
//! MoveIt's collision check and distances probing the input path and CHOMP's, each straight
//! between its waypoints, and ours with its corners rounded as MoveIt's time-optimal
//! trajectory generation rounds them, as arm planning probes our paths, with the measures of
//! each rounded at the scenario's corner tolerance, as [`Probed`] holds them.

use crate::arm_planning::{
    self, ArmChain, Held, MOVEIT_SUCCESS, Planner, Probed, Refiner, Scene, SceneGoal, Scenes,
    Termination, VoxelBox,
};
use crate::planning_scene::Object;
use crate::{Kind, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names};
use serde::Deserialize;
use std::num::NonZeroU32;
use std::path::PathBuf;

/// The `kind` of a CHOMP refinement scenario and of its reference output, which refuses every
/// other kind.
#[derive(Debug, Deserialize)]
enum ThisKind {
    #[serde(rename = "chomp_refinement")]
    ChompRefinement,
}

/// The waypoints of CHOMP's path: 3 s at 0.03 s between waypoints, both ends among them
/// (moveit2 2.15.2 chomp_planner.cpp:86).
const CHOMP_WAYPOINTS: usize = 101;

/// The arm, the map and the probes' parameters of a scenario, and its scenes.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Refinements {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    pub urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root.
    pub meshes: String,
    /// The arm whose paths CHOMP refines.
    pub chains: [ArmChain; 1],
    pub voxel_size: f64,
    /// The half side of the cube about the robot's origin that motion_planning_core's map
    /// covers.
    pub map_half_side: f64,
    /// The clearance of the input paths, motion_planning_core's min_clearance_m, and the
    /// padding of every link of MoveIt's robot against the octomap and the objects in its
    /// probes.
    pub min_clearance: f64,
    /// The maximum deviation at which execute_joint_path and MoveIt's time-optimal path round
    /// a path's corners, for the measures of the rounded path.
    pub corner_tolerance: f64,
    /// The tolerance of each joint of motion_planning_core's goal, the input path's end,
    /// above and below it.
    pub goal_tolerance: f64,
    /// The iteration bound on which motion_planning_core's plans end.
    pub iterations: u64,
    /// The time limit of motion_planning_core's plans, in seconds, far beyond their
    /// iterations, so a plan repeats under its seed.
    pub time_limit: f64,
    /// The joints' L2 distance between two probes of a path: no point of a link moves a
    /// tenth of a voxel between them.
    pub probe_spacing: f64,
    pub scenes: Vec<RefinementScene>,
}

impl Refinements {
    /// The arm planning scenario of the plans that give each scene's paths:
    /// motion_planning_core's RRTConnect from the scene's start to its goal, ended on the
    /// scenario's iterations, its path refined by `refiner`. [`Refiner::None`]
    /// gives the input path and [`Refiner::Chomp`] our refinement of it.
    pub fn plans(&self, refiner: Refiner) -> Scenes {
        Scenes {
            kind: arm_planning::ThisKind::ArmPlanning,
            urdf: self.urdf.clone(),
            meshes: self.meshes.clone(),
            chains: self.chains.to_vec(),
            checked_pairs: Some(Vec::new()),
            voxel_size: self.voxel_size,
            map_half_side: self.map_half_side,
            min_clearance: self.min_clearance,
            corner_tolerance: self.corner_tolerance,
            goal_tolerance: self.goal_tolerance,
            planner: Planner::RrtConnect,
            refiner,
            termination: Termination::Iterations,
            time_limit: self.time_limit,
            iterations: self.iterations,
            probe_spacing: Some(self.probe_spacing),
            scenes: self.scenes.iter().map(RefinementScene::planned).collect(),
        }
    }
}

/// One scene: its boxes, its collision objects, the objects the robot's grasp links hold, the
/// pairs allowed to touch, the arm's start and goal, the input path, our refinement of it and
/// the seed of the plans and of CHOMP's draws.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RefinementScene {
    pub boxes: Vec<VoxelBox>,
    /// The collision objects, each as apply_collision_objects adds it, in the robot frame.
    pub objects: Vec<Object>,
    /// The objects the robot's grasp links hold through the path.
    pub attached: Vec<Held>,
    /// The pairs of names allowed to touch, as set_allowed_collision allows them.
    pub allowed: Vec<[String; 2]>,
    pub start: Vec<f64>,
    /// The configuration motion_planning_core's plans reach for, within the scenario's goal
    /// tolerance.
    pub goal: Vec<f64>,
    /// The path CHOMP refines: the arm's joints at each waypoint, at least two, the start
    /// first. Its last waypoint is CHOMP's goal.
    pub path: Vec<Vec<f64>>,
    /// motion_planning_core's CHOMP refinement of `path`: the arm's joints at each waypoint,
    /// from the start to the input path's end.
    pub refined: Vec<Vec<f64>>,
    pub seed: NonZeroU32,
}

impl RefinementScene {
    /// The scene as a scene of arm planning.
    fn planned(&self) -> Scene {
        Scene {
            boxes: self.boxes.clone(),
            objects: self.objects.clone(),
            attached: self.attached.clone(),
            allowed: self.allowed.clone(),
            path_orientations: Vec::new(),
            start: self.start.clone(),
            goal: SceneGoal::Joints(self.goal.clone()),
            seed: self.seed,
            ours: None,
        }
    }
}

/// MoveIt's answer to a scene as the reference writes it: CHOMP's error code, its waypoints
/// and MoveIt's probe of them, null where CHOMP's optimizer did not run, and MoveIt's probe
/// of the input path and of our refinement.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SceneAnswer {
    status: i32,
    #[serde(deserialize_with = "Option::deserialize")]
    waypoints: Option<Vec<Vec<f64>>>,
    #[serde(deserialize_with = "Option::deserialize")]
    refined: Option<Probed>,
    input: Probed,
    ours: Probed,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    scenes: Vec<SceneAnswer>,
}

/// CHOMP's answer to a scene: its error code, MoveIt's of a plan, and MoveIt's probe of its
/// path where its optimizer ran, whether it succeeded or not.
#[derive(Debug)]
pub struct Chomp {
    pub status: i32,
    pub probed: Option<Probed>,
}

/// A scene, with CHOMP's answer to it and MoveIt's probe of its input path and of our
/// refinement.
#[derive(Debug)]
pub struct RefinementCase {
    pub scene: RefinementScene,
    pub chomp: Chomp,
    pub input_probed: Probed,
    pub ours_probed: Probed,
}

/// A CHOMP refinement scenario paired with its reference output.
#[derive(Debug)]
pub struct Case {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The scenario's mesh directory.
    pub meshes: PathBuf,
    pub refinements: Refinements,
    pub cases: Vec<RefinementCase>,
}

impl Case {
    /// Every CHOMP refinement scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: an input path of at least two waypoints from each
    /// scene's start and our refinement of it to the input path's end, MoveIt's probe of
    /// both, and CHOMP's path from that start to the input path's end with MoveIt's probe of
    /// it exactly where CHOMP gives one, as it does where it succeeds.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ChompRefinement)
            .into_iter()
            .map(|name| {
                let refinements: Refinements = read_scenario(&name);
                let urdf = read_urdf(&name, &refinements.urdf);
                let meshes = read_mesh_directory(&name, &refinements.meshes);
                let reference = read_reference(&name);
                pair(name, urdf, meshes, refinements, reference)
            })
            .collect()
    }
}

fn pair(
    name: String,
    urdf: String,
    meshes: PathBuf,
    refinements: Refinements,
    reference: Reference,
) -> Case {
    let Refinements {
        kind: ThisKind::ChompRefinement,
        ..
    } = &refinements;
    let Reference {
        kind: ThisKind::ChompRefinement,
        scenes: answers,
    } = reference;
    assert_eq!(
        refinements.scenes.len(),
        answers.len(),
        "{name}: an answer per scene"
    );
    let cases = refinements
        .scenes
        .iter()
        .zip(answers)
        .enumerate()
        .map(|(index, (scene, answer))| {
            case_of(&format!("{name} scene {index}"), scene.clone(), answer)
        })
        .collect();
    Case {
        name,
        urdf,
        meshes,
        refinements,
        cases,
    }
}

/// The scene `scene` of `what` with `answer`, the reference's: an input path of at least two
/// waypoints from the scene's start, our refinement of it from that start to the input
/// path's end, and CHOMP's path and MoveIt's probe of it, which the reference gives together,
/// from the scene's start to the input path's end, and gives where CHOMP succeeds.
fn case_of(what: &str, scene: RefinementScene, answer: SceneAnswer) -> RefinementCase {
    assert!(
        scene.path.len() >= 2 && scene.path.first() == Some(&scene.start),
        "{what}: an input path of at least two waypoints from the scene's start"
    );
    assert!(
        scene.refined.len() >= 2
            && scene.refined.first() == Some(&scene.start)
            && scene.refined.last() == scene.path.last(),
        "{what}: our refinement from the scene's start to the input path's end"
    );
    let SceneAnswer {
        status,
        waypoints,
        refined,
        input,
        ours,
    } = answer;
    let probed = match (waypoints, refined) {
        (Some(waypoints), Some(probed)) => {
            assert!(
                waypoints.len() == CHOMP_WAYPOINTS
                    && waypoints.first() == Some(&scene.start)
                    && waypoints.last() == scene.path.last(),
                "{what}: CHOMP's {CHOMP_WAYPOINTS} waypoints from the scene's start to the \
                 input path's end"
            );
            Some(probed)
        }
        (None, None) => None,
        _ => panic!("{what}: MoveIt's probe of CHOMP's path exactly where CHOMP gives one"),
    };
    assert!(
        status != MOVEIT_SUCCESS || probed.is_some(),
        "{what}: CHOMP's path where CHOMP succeeds"
    );
    RefinementCase {
        scene,
        chomp: Chomp { status, probed },
        input_probed: input,
        ours_probed: ours,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAIN: &str =
        r#"{"name": "arm", "base_link": "base", "tip_link": "tip", "joints": ["j"]}"#;
    const PATH: &str = "[[0.0], [0.5], [1.0]]";
    /// Our refinement of PATH.
    const REFINED: &str = "[[0.0], [0.25], [0.75], [1.0]]";

    /// The text of a scenario of the kind `kind`, whose one member `chains` holds, and of one
    /// scene, its input path `path`, our refinement `refined` and its seed `seed`.
    fn scenario(kind: &str, chains: &str, path: &str, refined: &str, seed: u32) -> String {
        format!(
            r#"{{"kind": "{kind}", "urdf": "a.urdf", "meshes": "meshes", "chains": [{chains}],
                "voxel_size": 0.04, "map_half_side": 1.6, "min_clearance": 0.0,
                "corner_tolerance": 0.1, "goal_tolerance": 0.01, "iterations": 100,
                "time_limit": 600.0, "probe_spacing": 0.01,
                "scenes": [{{"boxes": [{{"first": [0, 0, 0], "last": [1, 1, 1]}}],
                             "objects": [], "attached": [], "allowed": [],
                             "start": [0.0], "goal": [1.005], "path": {path},
                             "refined": {refined},
                             "seed": {seed}}}]}}"#
        )
    }

    fn refinements(path: &str, refined: &str) -> Refinements {
        serde_json::from_str(&scenario("chomp_refinement", CHAIN, path, refined, 1))
            .expect("parses")
    }

    const PROBED: &str = r#"{"collisions": 0, "clearance": 0.03125, "held_clearance": null,
                             "self_clearance": null, "share_of_tolerance": null,
                             "length": 1.0, "turning": 0.0, "bending": 0.0,
                             "duration": null}"#;

    /// CHOMP's waypoints from 0 to `end`, CHOMP_WAYPOINTS of them.
    fn chomp_path(end: f64) -> String {
        let waypoints: Vec<String> = (0..CHOMP_WAYPOINTS)
            .map(|k| format!("[{}]", k as f64 * end / (CHOMP_WAYPOINTS - 1) as f64))
            .collect();
        format!("[{}]", waypoints.join(", "))
    }

    /// The output of one scene: CHOMP's `status`, its `waypoints` and MoveIt's probe of them
    /// `refined`, and its probe of the input path and of ours.
    fn reference(status: i32, waypoints: &str, refined: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "chomp_refinement", "scenes": [
                {{"status": {status}, "waypoints": {waypoints}, "refined": {refined},
                  "input": {PROBED}, "ours": {PROBED}}}]}}"#
        ))
        .expect("parses")
    }

    /// MoveIt's error code of a state outside the joint limits, which CHOMP gives before its
    /// optimizer runs.
    const INVALID_ROBOT_STATE: i32 = -17;

    /// The scenario of the input path `path` and our refinement `ours`, paired with CHOMP's
    /// `status`, `waypoints` and MoveIt's probe of them `refined`.
    fn paired_with(path: &str, ours: &str, status: i32, waypoints: &str, refined: &str) -> Case {
        pair(
            "scenario".into(),
            String::new(),
            PathBuf::new(),
            refinements(path, ours),
            reference(status, waypoints, refined),
        )
    }

    fn paired(path: &str, status: i32, waypoints: &str, refined: &str) -> Case {
        paired_with(path, REFINED, status, waypoints, refined)
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let of_kind =
            |kind| serde_json::from_str::<Refinements>(&scenario(kind, CHAIN, PATH, REFINED, 1));
        assert!(of_kind("arm_planning").is_err());
        assert!(of_kind("chomp_refinement").is_ok());
    }

    #[test]
    fn a_scenario_of_two_chains_is_refused() {
        let chains = format!("{CHAIN}, {CHAIN}");
        assert!(
            serde_json::from_str::<Refinements>(&scenario(
                "chomp_refinement",
                &chains,
                PATH,
                REFINED,
                1
            ))
            .is_err()
        );
    }

    #[test]
    fn a_scene_of_seed_0_is_refused() {
        let of_seed = |seed| {
            serde_json::from_str::<Refinements>(&scenario(
                "chomp_refinement",
                CHAIN,
                PATH,
                REFINED,
                seed,
            ))
        };
        assert!(of_seed(0).is_err());
        assert!(of_seed(1).is_ok());
    }

    #[test]
    fn the_plans_of_a_scenario_reach_for_each_scenes_goal_refined_as_asked() {
        let plans = refinements(PATH, REFINED).plans(Refiner::Chomp);
        assert_eq!(plans.planner, Planner::RrtConnect);
        assert_eq!(plans.refiner, Refiner::Chomp);
        assert_eq!(plans.termination, Termination::Iterations);
        assert_eq!((plans.iterations, plans.time_limit), (100, 600.0));
        let SceneGoal::Joints(goal) = &plans.scenes[0].goal else {
            panic!("a goal of joints");
        };
        assert_eq!(goal, &vec![1.005]);
        assert_eq!(plans.scenes[0].start, vec![0.0]);
        assert_eq!(
            refinements(PATH, REFINED).plans(Refiner::None).refiner,
            Refiner::None
        );
    }

    #[test]
    fn a_scene_pairs_with_chomps_path_and_the_probes_of_both_paths() {
        let case = paired(PATH, MOVEIT_SUCCESS, &chomp_path(1.0), PROBED);
        let scene = &case.cases[0];
        assert_eq!(scene.chomp.status, MOVEIT_SUCCESS);
        let probed = scene.chomp.probed.expect("MoveIt's probe of CHOMP's path");
        assert_eq!(probed.length, 1.0);
        assert_eq!(scene.input_probed.length, 1.0);
        assert_eq!(scene.ours_probed.length, 1.0);
        let refused = paired(PATH, INVALID_ROBOT_STATE, "null", "null");
        assert!(refused.cases[0].chomp.probed.is_none());
    }

    #[test]
    #[should_panic(expected = "an input path of at least two waypoints from the scene's start")]
    fn an_input_path_of_one_waypoint_does_not_pair() {
        paired("[[0.0]]", MOVEIT_SUCCESS, &chomp_path(0.0), PROBED);
    }

    #[test]
    #[should_panic(expected = "an input path of at least two waypoints from the scene's start")]
    fn an_input_path_from_another_start_does_not_pair() {
        paired("[[0.5], [1.0]]", MOVEIT_SUCCESS, &chomp_path(1.0), PROBED);
    }

    #[test]
    #[should_panic(expected = "our refinement from the scene's start to the input path's end")]
    fn a_refinement_from_another_start_does_not_pair() {
        paired_with(
            PATH,
            "[[0.5], [1.0]]",
            MOVEIT_SUCCESS,
            &chomp_path(1.0),
            PROBED,
        );
    }

    #[test]
    #[should_panic(expected = "our refinement from the scene's start to the input path's end")]
    fn a_refinement_to_another_end_does_not_pair() {
        paired_with(
            PATH,
            "[[0.0], [0.5]]",
            MOVEIT_SUCCESS,
            &chomp_path(1.0),
            PROBED,
        );
    }

    #[test]
    #[should_panic(expected = "MoveIt's probe of CHOMP's path exactly where CHOMP gives one")]
    fn a_path_of_chomp_with_no_probe_does_not_pair() {
        paired(PATH, MOVEIT_SUCCESS, &chomp_path(1.0), "null");
    }

    #[test]
    #[should_panic(expected = "to the input path's end")]
    fn a_path_of_chomp_to_another_end_does_not_pair() {
        paired(PATH, MOVEIT_SUCCESS, &chomp_path(0.5), PROBED);
    }

    #[test]
    #[should_panic(expected = "to the input path's end")]
    fn a_path_of_chomp_of_other_than_its_waypoints_does_not_pair() {
        paired(PATH, MOVEIT_SUCCESS, PATH, PROBED);
    }

    #[test]
    #[should_panic(expected = "CHOMP's path where CHOMP succeeds")]
    fn a_success_of_chomp_with_no_path_does_not_pair() {
        paired(PATH, MOVEIT_SUCCESS, "null", "null");
    }

    #[test]
    fn an_answer_that_leaves_out_the_probe_of_the_input_path_is_refused() {
        let text = r#"{"kind": "chomp_refinement", "scenes": [
            {"status": -17, "waypoints": null, "refined": null}]}"#;
        assert!(serde_json::from_str::<Reference>(text).is_err());
    }

    #[test]
    fn an_answer_that_leaves_out_the_probe_of_our_refinement_is_refused() {
        let text = format!(
            r#"{{"kind": "chomp_refinement", "scenes": [
                {{"status": -17, "waypoints": null, "refined": null, "input": {PROBED}}}]}}"#
        );
        assert!(serde_json::from_str::<Reference>(&text).is_err());
    }

    #[test]
    fn every_chomp_refinement_scenario_pairs_with_its_reference() {
        let cases = Case::load_all();
        assert!(
            !cases.is_empty(),
            "scenarios/ holds a CHOMP refinement scenario"
        );
        for case in cases {
            assert!(!case.cases.is_empty(), "{} holds a scene", case.name);
        }
    }
}
