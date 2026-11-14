//! Arm planning. The scenario holds scenes of one or more arms among boxes of voxels and
//! collision objects, each with a start, a goal and a seed, and motion_planning_core's
//! answer to each, which its example `plan_arm_scenes` gives
//! `tools/draw_arm_planning.py`. The reference is MoveIt's planning pipeline, with the
//! OMPL planner the scenario names, its default request adapters and the
//! response adapters AddTimeOptimalParameterization and ValidateSolution, planning each
//! scene's arms as one group with the voxels as an octomap, among its collision objects
//! and holding its attached objects, to the goal state itself, seeded from the scene and
//! ended as the scenario's termination says, the path refined by MoveIt's CHOMP where the
//! scenario's refiner is `chomp`, and MoveIt's collision check and distances
//! probing each planned path: MoveIt's own, and motion_planning_core's with its corners
//! rounded as MoveIt's time-optimal trajectory generation rounds them. Both check every
//! link that moves with the arms' joints, the grippers' fingers among them, closed, as
//! MoveIt's default state holds them, and, where the scenario's group holds every arm of
//! the robot, the pairs of the robot's own bodies that the SRDF leaves checked.

use crate::planning_scene::Object;
use crate::{Kind, Robot, read_reference, read_robot, read_scenario, scenario_names};
use robot_description::Arm;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::num::NonZeroU32;

/// The `kind` of an arm planning scenario and of its reference output, which refuses
/// every other kind.
#[derive(Debug, Deserialize)]
pub(crate) enum ThisKind {
    #[serde(rename = "arm_planning")]
    ArmPlanning,
}

/// The robot, its group, the map and the plans' parameters of a scenario, and its scenes. A
/// scenario that `tools/draw_arm_planning.py` has drawn holds no probe spacing and no
/// answer of motion_planning_core yet; the example `plan_arm_scenes` reads it so.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenes {
    pub(crate) kind: ThisKind,
    /// The robot's model directory, relative to the repository root.
    pub description: String,
    /// The SRDF group the plans move: its arms, the group itself when it is an arm, else
    /// the arms it holds, in the order a state holds their joints, a scene's start and goal
    /// holding each arm's joints in turn.
    pub group: String,
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
    pub planner: Planner,
    pub refiner: Refiner,
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

/// The OMPL planner of each plan, as plan_joint_path's `planner` names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Planner {
    /// RRTConnect: the first valid path found.
    RrtConnect,
    /// RRTstar: the path's joint-space length, shortened until the plan's time ends.
    RrtStar,
}

/// What refines the path the planner answers, as plan_joint_path's `refiner` names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Refiner {
    /// Nothing: the planner's path.
    None,
    /// CHOMP: MoveIt's CHOMP at the parameters of its plugin, its initial trajectory the path
    /// MoveIt's pipeline answers, and motion_planning_core's CHOMP refinement.
    Chomp,
}

/// What ends each plan, motion_planning_core's and MoveIt's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Termination {
    /// The scenario's `iterations` (MoveIt's termination condition Iteration), its
    /// `time_limit` far beyond them, so a plan repeats under its seed.
    Iterations,
    /// The scenario's `time_limit`, as a robot's plan ends: MoveIt's by the time alone,
    /// its default termination, and motion_planning_core's by `iterations` for
    /// `rrt_connect`, as the motion planner node bounds it, and by no iteration count for
    /// `rrt_star`.
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

/// One scene: its boxes, its collision objects, the objects the robot's grasp links
/// hold, the pairs allowed to touch, the plan's start, goal and seed, and
/// `ours`, motion_planning_core's answer to it: an `Option<Ours>` in a scenario, null in
/// one the example `plan_arm_scenes` reads before it answers, and an [`Ours`] in a
/// [`SceneCase`].
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scene<O = Option<Ours>> {
    pub boxes: Vec<VoxelBox>,
    /// The collision objects, each as apply_collision_objects adds it, in the robot frame.
    pub objects: Vec<Object>,
    /// The objects the robot's grasp links hold through the plan.
    pub attached: Vec<Held>,
    /// The pairs of names allowed to touch, as set_allowed_collision allows them.
    pub allowed: Vec<[String; 2]>,
    /// The orientations the plan's path holds.
    pub path_orientations: Vec<HeldOrientation>,
    pub start: Vec<f64>,
    pub goal: SceneGoal,
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
            allowed,
            path_orientations,
            start,
            goal,
            seed,
            ours,
        } = self;
        Scene {
            boxes,
            objects,
            attached,
            allowed,
            path_orientations,
            start,
            goal,
            seed,
            ours: ours.unwrap_or_else(|| panic!("{what}: motion_planning_core's answer")),
        }
    }
}

/// An orientation a plan's path holds of a link of the arm at `arm` among the arms of the
/// scenario's group, as plan_joint_path's path_orientation_constraints names it for
/// motion_planning_core, and MoveIt's rotation-vector tolerances about x, y and z of the robot
/// frame for MoveIt's path constraint.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeldOrientation {
    pub arm: usize,
    pub link_name: String,
    pub orientation: [f64; 4],
    pub tolerance_rad: f64,
    pub free_axis: String,
    pub moveit_tolerances: [f64; 3],
}

/// What a scene's plan reaches for: a configuration of the arms, each joint within the
/// scenario's `goal_tolerance` of it for motion_planning_core and at it for MoveIt; or a
/// pose or a position of each arm's grasp link, in the order of the arms, which both reach
/// through inverse kinematics, MoveIt's KDL.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum SceneGoal {
    Joints(Vec<f64>),
    Poses(Vec<PoseGoal>),
    Positions(Vec<PositionGoal>),
}

/// A pose of a grasp link in the robot frame, its position (m) and its orientation `[x, y,
/// z, w]`, and how near each the goal holds: within `position_tolerance` (m) of the
/// position, and turned within `orientation_tolerance` (rad) of the orientation, which
/// MoveIt's `constructGoalConstraints` takes as the radius of its position region and as
/// the tolerance of each component of the rotation vector of its orientation (moveit2
/// 2.15.2 utils.cpp:220-226).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseGoal {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub position_tolerance: f64,
    pub orientation_tolerance: f64,
}

/// A position of a grasp link in the robot frame (m), which the goal holds within
/// `position_tolerance` (m), the grasp link free to turn: MoveIt's `constructGoalConstraints`
/// of a point.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PositionGoal {
    pub position: [f64; 3],
    pub position_tolerance: f64,
}

/// An object a grasp link holds: the object, its pose in the link's frame, the link, and
/// the links it may touch, as apply_attached_collision_object attaches it with the arms
/// at the scene's start. A link that is the grasp link of an arm of the group is held by
/// an arm the plan moves; any other is held by an arm the plan does not move, at its zero
/// configuration, as MoveIt's default state holds it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Held {
    pub object: Object,
    pub link: String,
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

/// MoveIt's collision check and distance along a path, probed at the scenario's spacing, and
/// the measures of the path rounded by MoveIt's time-optimal path at the scenario's corner
/// tolerance: MoveIt's own path straight between its waypoints rounded so, and ours as the
/// probe runs along it.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probed {
    /// The probes MoveIt finds in collision, its links padded by the scenario's clearance.
    pub collisions: u64,
    /// The smallest distance MoveIt measures from the arms to the voxels and the objects,
    /// unpadded: -1 where a probe is in collision, as MoveIt's distanceToCollisionUnpadded
    /// answers it, and in place of a distance below -1, which FCL can read of a body that
    /// touches the octomap.
    pub clearance: f64,
    /// The smallest distance MoveIt measures from the objects the arms hold to the voxels
    /// and the objects, unpadded and held at -1 or above as `clearance` is; none for a
    /// scene where the arms hold none.
    #[serde(deserialize_with = "Option::deserialize")]
    pub held_clearance: Option<f64>,
    /// The smallest distance MoveIt measures between the robot's own bodies and the
    /// objects they hold, unpadded, over the pairs its collision matrix checks, -1 in
    /// place of every distance below it; null where the plans check no such pair.
    #[serde(deserialize_with = "Option::deserialize")]
    pub self_clearance: Option<f64>,
    /// The largest share of its tolerance that any orientation the scene holds reaches along
    /// the path, measured as plan_joint_path states the tolerance; null where it holds none.
    #[serde(deserialize_with = "Option::deserialize")]
    pub share_of_tolerance: Option<f64>,
    /// The path's length in the joints' L2 norm.
    pub length: f64,
    /// The integral of the norm of the rounded path's curvature over its length (rad), summed
    /// at the midpoints of equal steps at most the scenario's probe spacing long.
    pub turning: f64,
    /// The integral of the squared norm of the rounded path's curvature over its length
    /// (1/rad), summed as `turning` is.
    pub bending: f64,
    /// The time (s) MoveIt's time-optimal trajectory takes along the rounded path at each
    /// joint's velocity and acceleration bounds of the robot model; null where it times none.
    #[serde(deserialize_with = "Option::deserialize")]
    pub duration: Option<f64>,
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
    self_clearance: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    share_of_tolerance: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    length: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    turning: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    bending: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    duration: Option<f64>,
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

impl SceneCase {
    /// The probes of the scene: MoveIt's of its own path and of ours, where each is one.
    fn probes(&self) -> impl Iterator<Item = &Probed> {
        self.moveit.probed.iter().chain(&self.ours_probed)
    }
}

/// An arm planning scenario paired with its reference output.
#[derive(Debug)]
pub struct Case {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub robot: Robot,
    /// The arms of the scenario's group, in the order a state holds their joints.
    pub arms: Vec<Arm>,
    pub scenes: Scenes,
    /// The pairs of the robot's own links the plans check, each pair's names in order.
    pub checked_pairs: BTreeSet<[String; 2]>,
    /// The scenario's probe spacing.
    pub probe_spacing: f64,
    pub cases: Vec<SceneCase>,
}

impl Case {
    /// Every arm planning scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: an answer of motion_planning_core in every
    /// scene, MoveIt's probe of its path exactly where it is a path, the probe of
    /// MoveIt's own path exactly where MoveIt succeeds, and a self distance in each probe
    /// exactly where the plans check pairs of the robot's own links.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ArmPlanning)
            .into_iter()
            .map(|name| {
                let scenes: Scenes = read_scenario(&name);
                let robot = read_robot(&name, &scenes.description);
                let arms = robot.group_arms(&name, &scenes.group);
                let checked_pairs = robot.checked_pairs(&name, &scenes.group);
                let (probe_spacing, cases) = pair(
                    &name,
                    &scenes,
                    !checked_pairs.is_empty(),
                    read_reference(&name),
                );
                Case {
                    name,
                    robot,
                    arms,
                    scenes,
                    checked_pairs,
                    probe_spacing,
                    cases,
                }
            })
            .collect()
    }
}

/// The probe spacing of `scenes`, the scenario `name`, and each of its scenes paired with
/// its answer of `reference`, the plans checking pairs of the robot's own links where
/// `checks_own_pairs`.
fn pair(
    name: &str,
    scenes: &Scenes,
    checks_own_pairs: bool,
    reference: Reference,
) -> (f64, Vec<SceneCase>) {
    let Scenes {
        kind: ThisKind::ArmPlanning,
        ..
    } = scenes;
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
            let holds = !scene.path_orientations.is_empty();
            let case = SceneCase {
                scene,
                moveit: moveit_of(&what, answer.moveit),
                ours_probed: answer.ours,
            };
            assert!(
                case.probes()
                    .all(|p| p.self_clearance.is_some() == checks_own_pairs),
                "{what}: a self distance in each probe exactly where the plans check pairs"
            );
            assert!(
                case.probes()
                    .all(|p| p.share_of_tolerance.is_some() == holds),
                "{what}: a share of the tolerance in each probe exactly where the scene holds an \
                 orientation"
            );
            case
        })
        .collect();
    (probe_spacing, cases)
}

/// MoveIt's answer `read`, whose probe the reference gives exactly where MoveIt succeeds.
fn moveit_of(what: &str, read: MoveItRead) -> MoveIt {
    let MoveItRead {
        status,
        collisions,
        clearance,
        held_clearance,
        self_clearance,
        share_of_tolerance,
        length,
        turning,
        bending,
        duration,
    } = read;
    let probed = match (collisions, clearance, length, turning, bending) {
        (Some(collisions), Some(clearance), Some(length), Some(turning), Some(bending)) => {
            Some(Probed {
                collisions,
                clearance,
                held_clearance,
                self_clearance,
                share_of_tolerance,
                length,
                turning,
                bending,
                duration,
            })
        }
        (None, None, None, None, None)
            if held_clearance.is_none()
                && self_clearance.is_none()
                && share_of_tolerance.is_none()
                && duration.is_none() =>
        {
            None
        }
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
        serde_json::from_str(&scenario("arm_planning", "rrt_connect", "iterations", ours))
            .expect("parses")
    }

    /// The text of a scenario of the kind `kind`, its plans by `planner`, refined by no
    /// refiner, ended by `termination`, and of one scene, our answer to it `ours`.
    fn scenario(kind: &str, planner: &str, termination: &str, ours: &str) -> String {
        format!(
            r#"{{"kind": "{kind}", "description": "d", "group": "arm", "voxel_size": 0.04,
                "map_half_side": 1.6, "min_clearance": 0.0, "corner_tolerance": 0.1,
                "goal_tolerance": 0.01, "planner": "{planner}", "refiner": "none",
                "termination": "{termination}", "time_limit": 5.0, "iterations": 10,
                "probe_spacing": 0.01,
                "scenes": [{{"boxes": [{{"first": [0, 0, 0], "last": [1, 1, 1]}}],
                             "objects": [], "attached": [], "allowed": [],
                             "path_orientations": [],
                             "start": [0.0], "goal": [1.0], "seed": 1, "ours": {ours}}}]}}"#
        )
    }

    const PATH: &str = r#"{"waypoints": [[0.0], [1.0]]}"#;
    const FAILURE: &str = r#"{"failure": "no_path"}"#;
    const PROBED: &str = r#"{"collisions": 0, "clearance": 0.03125, "held_clearance": null,
                             "self_clearance": null, "share_of_tolerance": null,
                             "length": 1.0, "turning": 0.5, "bending": 0.25,
                             "duration": 2.0}"#;

    /// The output of one scene: MoveIt's `status` and the fields of its probe, and the
    /// probe of ours.
    fn reference(status: i32, moveit_probe: &str, ours: &str) -> Reference {
        serde_json::from_str(&format!(
            r#"{{"kind": "arm_planning", "scenes": [
                {{"moveit": {{"status": {status}, {moveit_probe}}}, "ours": {ours}}}]}}"#
        ))
        .expect("parses")
    }

    const MOVEIT_PROBED: &str = r#""collisions": 0, "clearance": 0.03125,
                                   "held_clearance": null, "self_clearance": null,
                                   "share_of_tolerance": null, "length": 1.0,
                                   "turning": 0.5, "bending": 0.25, "duration": 2.0"#;
    const MOVEIT_NONE: &str = r#""collisions": null, "clearance": null,
                                 "held_clearance": null, "self_clearance": null,
                                 "share_of_tolerance": null, "length": null,
                                 "turning": null, "bending": null, "duration": null"#;

    fn paired(
        ours: &str,
        status: i32,
        moveit_probe: &str,
        ours_probe: &str,
    ) -> (f64, Vec<SceneCase>) {
        pair(
            "scenario",
            &scenes(ours),
            false,
            reference(status, moveit_probe, ours_probe),
        )
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let of_kind = |kind| {
            serde_json::from_str::<Scenes>(&scenario(kind, "rrt_connect", "iterations", PATH))
        };
        assert!(of_kind("planning").is_err());
        assert!(of_kind("arm_planning").is_ok());
    }

    #[test]
    fn a_scenario_names_its_planner_and_its_termination() {
        let planned = |planner, termination| {
            serde_json::from_str::<Scenes>(&scenario("arm_planning", planner, termination, PATH))
                .map(|s| (s.planner, s.termination))
        };
        assert_eq!(
            planned("rrt_connect", "iterations").ok(),
            Some((Planner::RrtConnect, Termination::Iterations))
        );
        assert_eq!(
            planned("rrt_star", "time").ok(),
            Some((Planner::RrtStar, Termination::Time))
        );
        assert!(planned("prm", "time").is_err());
        assert!(planned("rrt_connect", "deadline").is_err());
    }

    #[test]
    fn a_scenario_names_its_refiner() {
        let refined = |refiner: &str| {
            serde_json::from_str::<Scenes>(
                &scenario("arm_planning", "rrt_connect", "iterations", PATH).replace(
                    r#""refiner": "none""#,
                    &format!(r#""refiner": "{refiner}""#),
                ),
            )
            .map(|s| s.refiner)
        };
        assert_eq!(refined("none").ok(), Some(Refiner::None));
        assert_eq!(refined("chomp").ok(), Some(Refiner::Chomp));
        assert!(refined("stomp").is_err());
    }

    // A probe holds a self distance exactly where the plans check pairs of the robot's own
    // links: plans that check none pair with probes that hold none, plans that check pairs
    // with probes that hold one, and either the other way round panics.
    #[test]
    fn a_self_distance_pairs_exactly_where_the_plans_check_pairs() {
        let pairs = |checks_own_pairs: bool, self_clearance: &str| {
            let scenes = scenes(PATH);
            let probe = r#""self_clearance": null"#;
            let probe_with = format!(r#""self_clearance": {self_clearance}"#);
            let reference = reference(
                MOVEIT_SUCCESS,
                &MOVEIT_PROBED.replace(probe, &probe_with),
                &PROBED.replace(probe, &probe_with),
            );
            std::panic::catch_unwind(|| pair("scenario", &scenes, checks_own_pairs, reference))
                .is_ok()
        };
        assert!(pairs(false, "null"));
        assert!(pairs(true, "0.25"));
        assert!(!pairs(false, "0.25"));
        assert!(!pairs(true, "null"));
    }

    #[test]
    fn a_scene_seeded_with_0_is_refused() {
        let text = r#"{"boxes": [], "objects": [], "attached": [], "allowed": [],
            "start": [0.0], "goal": [1.0], "seed": 0, "ours": null}"#;
        assert!(serde_json::from_str::<Scene>(text).is_err());
    }

    #[test]
    fn a_scenes_goal_is_a_configuration_or_a_pose_of_each_grasp_link() {
        let goal = |text: &str| serde_json::from_str::<SceneGoal>(text);
        assert!(matches!(goal("[1.0, 2.0]"), Ok(SceneGoal::Joints(q)) if q == [1.0, 2.0]));
        let pose = r#"{"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0],
                       "position_tolerance": 0.005, "orientation_tolerance": 0.05}"#;
        let poses = format!("[{pose}, {}]", pose.replace("0.05}", "0.01}"));
        assert!(matches!(
            goal(&poses),
            Ok(SceneGoal::Poses(p))
                if p.len() == 2 && p[0].orientation_tolerance == 0.05
                    && p[1].orientation_tolerance == 0.01
        ));
        let position = r#"[{"position": [0.1, 0.2, 0.3], "position_tolerance": 0.001}]"#;
        assert!(matches!(
            goal(position),
            Ok(SceneGoal::Positions(p)) if p.len() == 1 && p[0].position_tolerance == 0.001
        ));
        assert!(goal(pose).is_err());
        assert!(
            goal(&format!(
                "[{}]",
                pose.replace('}', r#", "frame": "world"}"#)
            ))
            .is_err()
        );
    }

    #[test]
    fn a_scene_pairs_with_the_answers_of_moveit_and_its_probe_of_ours() {
        let (probe_spacing, cases) = paired(PATH, MOVEIT_SUCCESS, MOVEIT_PROBED, PROBED);
        let scene = &cases[0];
        assert_eq!(scene.moveit.status, MOVEIT_SUCCESS);
        assert!(scene.moveit.probed.is_some() && scene.ours_probed.is_some());
        assert_eq!(probe_spacing, 0.01);
        let (_, refused) = paired(FAILURE, -10, MOVEIT_NONE, "null");
        assert!(refused[0].moveit.probed.is_none() && refused[0].ours_probed.is_none());
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
    fn moveits_probe_of_a_path_it_times_none_pairs_with_no_duration() {
        let untimed = MOVEIT_PROBED.replace(r#""duration": 2.0"#, r#""duration": null"#);
        let (_, cases) = paired(PATH, MOVEIT_SUCCESS, &untimed, PROBED);
        let probed = cases[0].moveit.probed.expect("MoveIt's probe");
        assert_eq!((probed.turning, probed.duration), (0.5, None));
    }

    #[test]
    #[should_panic(expected = "MoveIt's probe whole or not at all")]
    fn a_probe_of_moveit_with_no_bending_does_not_pair() {
        let partial = MOVEIT_PROBED.replace(r#""bending": 0.25"#, r#""bending": null"#);
        paired(PATH, MOVEIT_SUCCESS, &partial, PROBED);
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
