//! Parity with MoveIt's planning pipeline on the arm planning scenarios of the ROS
//! reference harness, at one attempt, the gripper's fingers carried by the arm in both,
//! each plan with the planner of the scenario's objective. The committed scenarios plan at
//! a clearance of 0: this crate still plans the paths each of them carries; MoveIt's
//! collision check finds every probe of each of them free; this crate refuses each start
//! that MoveIt refuses outside a joint limit or in collision; and each holds a scene of
//! each outcome the README names. Over the scenes of a drawn set, at its clearance, which
//! pads MoveIt's links, the success rate and the mean smallest clearance lie within or
//! above MoveIt's 95% intervals, and the mean path length within or below its interval.

mod arm_scenes;

use std::collections::{BTreeMap, BTreeSet};

use control_core::path_timing::JOINT_PATH_CORNER_DEVIATION;
use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::IkGoalSampler;
use ros_reference::arm_planning::{
    Case, MOVEIT_SUCCESS, Ours, Probed, Scene, SceneCase, Scenes, Termination, VoxelBox,
};

use arm_scenes::{boxes_of, geometry, map_of, on_the_arm, plan_scene, probe_spacing};
use nalgebra::Point3;
use openarm_description::{HardwareVersion, Side};

/// MoveIt's error codes of a refusal: a start outside a joint limit, a start in
/// collision, and a goal state that is not valid; and of a planned path that MoveIt's
/// own check finds invalid.
const MOVEIT_START_STATE_INVALID: i32 = -26;
const MOVEIT_START_STATE_IN_COLLISION: i32 = -10;
const MOVEIT_GOAL_STATE_INVALID: i32 = -27;
const MOVEIT_INVALID_MOTION_PLAN: i32 = -2;
/// The quantile of the standard normal distribution that bounds a two-sided 95%
/// interval.
const Z_95: f64 = 1.959_963_984_540_054;
/// The fewest scenes whose statistics the test compares: where the normal approximation
/// of the intervals holds.
const MIN_SCENES: usize = 30;

/// The 95% interval of the mean of `values`.
fn mean_interval(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let half = Z_95 * (variance / n).sqrt();
    (mean - half, mean + half)
}

/// The Wilson score interval of a success rate, at 95%.
fn rate_interval(successes: usize, trials: usize) -> (f64, f64) {
    let n = trials as f64;
    let p = successes as f64 / n;
    let z2 = Z_95 * Z_95;
    let center = (p + z2 / (2.0 * n)) / (1.0 + z2 / n);
    let half = Z_95 / (1.0 + z2 / n) * (p * (1.0 - p) / n + z2 / (4.0 * n * n)).sqrt();
    (center - half, center + half)
}

/// What the outcomes of a scene read: its case, the waypoints of our path (0 for a
/// failure), and whether its start lies below a joint's lower limit or above an upper.
struct Shown<'a> {
    case: &'a SceneCase,
    waypoints: usize,
    below: bool,
    above: bool,
}

/// An outcome of a scene, by its name, and whether a scene shows it.
type Outcome = (&'static str, fn(&Shown) -> bool);

/// The name of this crate's failure in `ours`, or none for a path.
fn failure(ours: &Ours) -> Option<&str> {
    match ours {
        Ours::Path { .. } => None,
        Ours::Failure { failure } => Some(failure),
    }
}

// The occupied voxels of a map, as boxes of runs along x, fill a scene's map with them
// and no other: the example scenes_from_episodes writes the map an episode's frames build
// as its scene's boxes. Each box is one whole run: no two boxes of a row touch or share a
// voxel.
#[hegel::test]
fn a_maps_voxels_as_boxes_fill_a_scenes_map_with_them(tc: TestCase) {
    let scenes: Scenes = serde_json::from_str(
        r#"{"kind": "arm_planning", "urdf": "", "meshes": "", "base_link": "", "tip_link": "",
            "joints": [], "voxel_size": 0.04, "map_half_side": 0.2, "min_clearance": 0.0,
            "corner_tolerance": 0.1, "goal_tolerance": 0.01, "objective": "any",
            "termination": "iterations", "time_limit": 1.0, "iterations": 1, "scenes": []}"#,
    )
    .expect("a scenario's header");
    // Keys -5 to 4 lie inside the map of 0.04 m voxels 0.2 m about the origin.
    let keys: Vec<[i32; 3]> = tc
        .draw(gs::vecs(gs::arrays(gs::integers::<i32>().min_value(-5).max_value(4))).max_size(60));
    let scene_of = |boxes: Vec<VoxelBox>| Scene {
        boxes,
        objects: Vec::new(),
        attached: None,
        other_arm_holds: None,
        allowed: Vec::new(),
        start: Vec::new(),
        goal: Vec::new(),
        seed: std::num::NonZeroU32::MIN,
        ours: (),
    };
    let voxels = scene_of(
        keys.iter()
            .map(|&key| VoxelBox {
                first: key,
                last: key,
            })
            .collect(),
    );
    let map = map_of(&scenes, &voxels);
    let boxes = boxes_of(&map);
    let occupied = |map: &occupancy_map::VoxelMap| -> BTreeSet<[i32; 3]> {
        map.occupied().map(|key| key.0).collect()
    };
    assert_eq!(occupied(&map), keys.iter().copied().collect());
    assert!(
        boxes
            .iter()
            .all(|b| b.first[1..] == b.last[1..] && b.first[0] <= b.last[0]),
        "each box a run along x: {boxes:?}"
    );
    let held: usize = boxes
        .iter()
        .map(|b| (b.last[0] - b.first[0] + 1) as usize)
        .sum();
    assert_eq!(held, occupied(&map).len(), "no two boxes share a voxel");
    let mut rows: Vec<&VoxelBox> = boxes.iter().collect();
    rows.sort_by_key(|b| (b.first[1], b.first[2], b.first[0]));
    assert!(
        rows.windows(2).all(|pair| {
            pair[0].first[1..] != pair[1].first[1..] || pair[0].last[0] + 1 < pair[1].first[0]
        }),
        "no two boxes of a row touch: {boxes:?}"
    );
    assert_eq!(occupied(&map_of(&scenes, &scene_of(boxes))), occupied(&map));
}

// The robot around a scenario's arm, the left or the right one, holds the other arm at its
// zero configuration: of the planned arm's side only its base link, which no joint moves,
// stands, some parts lie on the other arm's, and each lies where the self model at zero
// places it, however far the planned arm stands from zero.
#[test]
fn the_robot_around_either_arm_holds_the_other_arm_at_zero() {
    let mut case = Case::load_all()
        .into_iter()
        .find(|case| case.name == "arm_planning_openarm_v2")
        .expect("the scenario among boxes");
    let meshes = case.meshes.to_str().expect("a UTF-8 mesh directory");
    let mut model = self_collision_model::openarm(HardwareVersion::V2, meshes)
        .expect("the self model of OpenArm v2");
    let at_zero: BTreeMap<String, Vec<Point3<f64>>> = model
        .world_pieces(&[0.0; 7], &[0.0; 7])
        .expect("finite joint positions")
        .into_iter()
        .map(|(link, pieces)| {
            let centers = pieces
                .iter()
                .map(|piece| piece.bounding_sphere().0)
                .collect();
            (link.to_string(), centers)
        })
        .collect();
    let start = [0.3, -0.4, 0.2, 0.9, -0.1, 0.5, 0.2];
    let prefix = |side| HardwareVersion::V2.tcp_link(side).trim_end_matches("tcp");
    for side in Side::BOTH {
        let named = |name: &str| name.replace(prefix(Side::Left), prefix(side));
        case.scenes.base_link = named(&case.scenes.base_link);
        case.scenes.tip_link = named(&case.scenes.tip_link);
        case.scenes.joints = case
            .scenes
            .joints
            .iter()
            .map(|joint| named(joint))
            .collect();
        on_the_arm(&case.scenes, &case.urdf, &case.meshes, |arm, standing| {
            let parts = standing.parts(arm, &start);
            let base = HardwareVersion::V2.base_link(side);
            for part in &parts {
                assert!(
                    !part.link.starts_with(prefix(side)) || part.link == base,
                    "{side:?}: {} of the planned arm stands",
                    part.link
                );
            }
            assert!(
                parts
                    .iter()
                    .any(|part| part.link.starts_with(prefix(side.other()))),
                "{side:?}: no part of the other arm stands"
            );
            let mut given: BTreeMap<String, Vec<Point3<f64>>> = BTreeMap::new();
            for part in &parts {
                given
                    .entry(part.link.to_string())
                    .or_default()
                    .push(part.piece.bounding_sphere().0);
            }
            for (link, centers) in given {
                assert_eq!(centers, at_zero[&link], "{side:?}: {link}");
            }
        });
    }
}

// This crate plans, scene by scene, the answer each scenario carries for it: the paths
// the reference probed are this crate's, rounded as execute_joint_path rounds them, and
// probed at the spacing this crate's arm gives. Each scenario's plans end on its
// iterations, so they repeat.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_carries_the_paths_this_crate_plans() {
    for case in Case::load_all() {
        assert_eq!(
            case.scenes.termination,
            Termination::Iterations,
            "{}: a committed scenario's plans end on its iterations; one ended by time runs \
             locally, as the harness README's rule 3 says",
            case.name
        );
        assert_eq!(case.scenes.corner_tolerance, JOINT_PATH_CORNER_DEVIATION);
        on_the_arm(&case.scenes, &case.urdf, &case.meshes, |arm, standing| {
            let sampler = IkGoalSampler::new(arm.geometry().chain().clone());
            assert_eq!(
                case.probe_spacing,
                probe_spacing(arm, &case.scenes),
                "{}: the scenario's probe spacing is its arm's",
                case.name
            );
            for (k, c) in case.cases.iter().enumerate() {
                assert_eq!(
                    plan_scene(arm, standing, &sampler, &case.scenes, &c.scene).ours,
                    c.scene.ours,
                    "{} scene {k}",
                    case.name
                );
            }
        });
    }
}

// Each scenario holds a scene of each outcome: a straight path and one with corners that
// MoveIt plans too, a path of ours where MoveIt's own fails its check, a start in a box
// and a goal in a box that both refuse, and a start below a joint's lower limit and one
// above an upper limit that both refuse. The committed outputs hold MoveIt's amd64
// answers, whose outcomes the scenes were picked by, so the test runs on amd64.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_holds_a_scene_of_each_outcome() {
    let outcomes: [Outcome; 7] = [
        ("a straight path", |s| {
            s.waypoints == 2 && s.case.moveit.status == MOVEIT_SUCCESS
        }),
        ("a path with corners", |s| {
            s.waypoints > 2 && s.case.moveit.status == MOVEIT_SUCCESS
        }),
        ("a path where MoveIt's fails its check", |s| {
            s.waypoints >= 2 && s.case.moveit.status == MOVEIT_INVALID_MOTION_PLAN
        }),
        ("a start in a box", |s| {
            failure(&s.case.scene.ours) == Some("start_in_collision")
                && s.case.moveit.status == MOVEIT_START_STATE_IN_COLLISION
        }),
        ("a goal in a box", |s| {
            failure(&s.case.scene.ours) == Some("unreachable")
                && s.case.moveit.status == MOVEIT_GOAL_STATE_INVALID
        }),
        ("a start below a lower limit", |s| {
            s.below
                && failure(&s.case.scene.ours) == Some("start_outside_limits")
                && s.case.moveit.status == MOVEIT_START_STATE_INVALID
        }),
        ("a start above an upper limit", |s| {
            s.above
                && failure(&s.case.scene.ours) == Some("start_outside_limits")
                && s.case.moveit.status == MOVEIT_START_STATE_INVALID
        }),
    ];
    for case in Case::load_all() {
        let geometry = geometry(&case.scenes, &case.urdf, &case.meshes);
        let limits = geometry.chain().limits();
        let shown: Vec<Shown> = case
            .cases
            .iter()
            .map(|c| Shown {
                case: c,
                waypoints: match &c.scene.ours {
                    Ours::Path { waypoints } => waypoints.len(),
                    Ours::Failure { .. } => 0,
                },
                below: c.scene.start.iter().zip(limits).any(|(q, l)| *q < l.lo),
                above: c.scene.start.iter().zip(limits).any(|(q, l)| *q > l.hi),
            })
            .collect();
        for (outcome, holds) in outcomes {
            assert!(
                shown.iter().any(holds),
                "{} holds no scene of {outcome}",
                case.name
            );
        }
    }
}

// Each scenario plans at a clearance of 0. MoveIt's collision check finds every probe of
// every path of this crate free; this crate refuses each start that MoveIt refuses
// outside a joint limit or in collision, and MoveIt each start that this crate refuses
// outside a joint limit. The test prints how often MoveIt's check finds a probe of
// MoveIt's own paths in collision.
#[test]
fn moveit_finds_every_path_of_this_crate_free() {
    for case in Case::load_all() {
        assert_eq!(
            case.scenes.min_clearance, 0.0,
            "{}: the parity plans at a clearance of 0",
            case.name
        );
        let (mut theirs, mut colliding) = (0, 0);
        for (k, c) in case.cases.iter().enumerate() {
            let what = format!("{} scene {k}", case.name);
            if let Some(probed) = c.ours_probed {
                assert_eq!(
                    probed.collisions, 0,
                    "{what}: MoveIt finds our path in collision"
                );
            }
            let refused = match c.moveit.status {
                MOVEIT_START_STATE_INVALID => Some("start_outside_limits"),
                MOVEIT_START_STATE_IN_COLLISION => Some("start_in_collision"),
                _ => None,
            };
            if let Some(refused) = refused {
                assert_eq!(
                    failure(&c.scene.ours),
                    Some(refused),
                    "{what}: MoveIt refuses the start"
                );
            }
            if failure(&c.scene.ours) == Some("start_outside_limits") {
                assert_eq!(
                    c.moveit.status, MOVEIT_START_STATE_INVALID,
                    "{what}: MoveIt plans from outside the limits"
                );
            }
            if let Some(probed) = c.moveit.probed {
                theirs += 1;
                colliding += usize::from(probed.collisions > 0);
            }
        }
        println!(
            "{}: MoveIt's check finds {colliding} of MoveIt's {theirs} paths in collision at a \
             probe",
            case.name
        );
    }
}

/// The spare a plan keeps above its clearance from an object, 1 mm (the plan module's
/// `MIN_SPARE_M`, for a term of one moving body): a held object keeps the clearance and this
/// from an object at each checked state.
const OBJECT_SPARE_M: f64 = 1e-3;

/// A group of scenes, by its name, and whether a scene belongs to it.
type Group = (&'static str, fn(&SceneCase) -> bool);

/// What `scenes` show: their number; this crate's paths and failures by name, and
/// MoveIt's paths and error codes; the mean smallest clearance and path length of each
/// one's paths; how many of each one's paths MoveIt's check finds in collision at a
/// probe; and how many bring the object the arm holds nearer an object or a voxel than
/// `held_within`.
fn tally(scenes: &[&SceneCase], held_within: f64) -> String {
    let mut failures: BTreeMap<&str, usize> = BTreeMap::new();
    let mut codes: BTreeMap<i32, usize> = BTreeMap::new();
    for c in scenes {
        if let Some(name) = failure(&c.scene.ours) {
            *failures.entry(name).or_default() += 1;
        }
        *codes.entry(c.moveit.status).or_default() += 1;
    }
    let ours: Vec<Probed> = scenes.iter().filter_map(|c| c.ours_probed).collect();
    let theirs: Vec<Probed> = scenes.iter().filter_map(|c| c.moveit.probed).collect();
    let mean = |paths: &[Probed], of: fn(&Probed) -> f64| {
        paths.iter().map(of).sum::<f64>() / paths.len() as f64
    };
    let colliding = |paths: &[Probed]| paths.iter().filter(|p| p.collisions > 0).count();
    let held_near = |paths: &[Probed]| {
        paths
            .iter()
            .filter(|p| p.held_clearance.is_some_and(|held| held < held_within))
            .count()
    };
    format!(
        "{} scenes; ours {} paths, failures {failures:?}, clearance {:.4}, length {:.3}, {} \
         in collision at a probe, {} holding an object nearer than {held_within} m; MoveIt's \
         {} paths, codes {codes:?}, clearance {:.4}, length {:.3}, {} in collision at a \
         probe, {} holding an object nearer than {held_within} m",
        scenes.len(),
        ours.len(),
        mean(&ours, |p| p.clearance),
        mean(&ours, |p| p.length),
        colliding(&ours),
        held_near(&ours),
        theirs.len(),
        mean(&theirs, |p| p.clearance),
        mean(&theirs, |p| p.length),
        colliding(&theirs),
        held_near(&theirs),
    )
}

/// Asserts that over the scenes of the drawn set in the scenario `name`, this crate's
/// success rate and mean smallest clearance lie within or above MoveIt's 95% intervals,
/// and its mean path length within or below MoveIt's interval.
fn lies_on_its_side_of_moveits(name: &str) {
    let case = Case::load_all()
        .into_iter()
        .find(|case| case.name == name)
        .unwrap_or_else(|| panic!("scenarios/ holds no scenario {name}"));
    let scenes: &[SceneCase] = &case.cases;
    assert!(
        scenes.len() >= MIN_SCENES,
        "{}: {} scenes; the statistics need a drawn set of at least {MIN_SCENES}: \
         draw_arm_planning.py --drawn 200, run.sh, then this test",
        case.name,
        scenes.len()
    );
    println!(
        "{}: {} scenes, objective {:?}, ended by {:?}, time limit {} s, clearance {} m",
        case.name,
        scenes.len(),
        case.scenes.objective,
        case.scenes.termination,
        case.scenes.time_limit,
        case.scenes.min_clearance
    );
    let held_within = case.scenes.min_clearance + OBJECT_SPARE_M;
    let groups: [Group; 5] = [
        ("all", |_| true),
        ("holding", |c| c.scene.attached.is_some()),
        ("not holding", |c| c.scene.attached.is_none()),
        ("the other arm holding", |c| {
            c.scene.other_arm_holds.is_some()
        }),
        ("with an allowed pair", |c| !c.scene.allowed.is_empty()),
    ];
    for (group, holds) in groups {
        let kept: Vec<&SceneCase> = scenes.iter().filter(|c| holds(c)).collect();
        if !kept.is_empty() {
            println!("{}: {group}: {}", case.name, tally(&kept, held_within));
        }
    }
    let ours: Vec<Probed> = scenes.iter().filter_map(|c| c.ours_probed).collect();
    let theirs: Vec<Probed> = scenes.iter().filter_map(|c| c.moveit.probed).collect();
    let (low, high) = rate_interval(theirs.len(), scenes.len());
    let rate = ours.len() as f64 / scenes.len() as f64;
    let moveit_rate = theirs.len() as f64 / scenes.len() as f64;
    println!(
        "{}: success rate ours {rate}, MoveIt's {moveit_rate} in {low}..{high}",
        case.name
    );
    assert!(
        rate >= low,
        "{}: a success rate of {rate}, below MoveIt's interval from {low}",
        case.name
    );
    let interval = |of: fn(&Probed) -> f64| {
        let values: Vec<f64> = theirs.iter().map(of).collect();
        let mean = ours.iter().map(of).sum::<f64>() / ours.len() as f64;
        (mean, mean_interval(&values))
    };
    let (clearance, (low, high)) = interval(|p| p.clearance);
    println!(
        "{}: clearance ours {clearance}, MoveIt's {} in {low}..{high}",
        case.name,
        (low + high) / 2.0
    );
    assert!(
        clearance >= low,
        "{}: a mean clearance of {clearance}, below MoveIt's interval from {low}",
        case.name
    );
    let (length, (low, high)) = interval(|p| p.length);
    println!(
        "{}: length ours {length}, MoveIt's {} in {low}..{high}",
        case.name,
        (low + high) / 2.0
    );
    assert!(
        length <= high,
        "{}: a mean length of {length}, above MoveIt's interval to {high}",
        case.name
    );
}

// Over the scenes of a drawn set, among boxes of voxels or among collision objects, this
// crate's success rate and mean smallest clearance lie within or above MoveIt's 95%
// intervals, and its mean path length within or below MoveIt's; a probe of MoveIt's in
// collision measures a clearance of -1. The committed scenes, one of each outcome, are too
// few for the intervals: each test runs on its scenario's drawn set, locally, as the
// harness README's rule 3 says, and the pull request states its tally at its head, for
// each objective, termination and clearance it ran.
#[test]
#[ignore = "statistics of a drawn set among boxes, run locally as the harness README's rule 3 says"]
fn among_boxes_the_success_rate_the_clearance_and_the_length_lie_on_their_side_of_moveits() {
    lies_on_its_side_of_moveits("arm_planning_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set among objects, run locally as the harness README's rule 3 says"]
fn among_objects_the_success_rate_the_clearance_and_the_length_lie_on_their_side_of_moveits() {
    lies_on_its_side_of_moveits("arm_planning_objects_openarm_v2");
}
