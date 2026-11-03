//! Parity with MoveIt's planning pipeline on the arm planning scenarios of the ROS
//! reference harness, at a clearance of 0 and one attempt, the gripper's fingers carried
//! by the arm in both. This crate still plans the paths each scenario carries; MoveIt's
//! collision check finds every probe of each of them free; this crate refuses each start
//! that MoveIt refuses outside a joint limit or in collision; and over the scenes of a
//! drawn set, the success rate and the mean smallest clearance lie within or above
//! MoveIt's 95% intervals, and the mean path length within or below its interval. Each
//! scenario holds a scene of each outcome the README names.

mod arm_scenes;

use std::collections::BTreeSet;

use control_core::path_timing::JOINT_PATH_CORNER_DEVIATION;
use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::IkGoalSampler;
use ros_reference::arm_planning::{
    Case, MOVEIT_SUCCESS, Ours, Probed, Scene, SceneCase, Scenes, VoxelBox,
};

use arm_scenes::{boxes_of, geometry, map_of, on_the_arm, plan_scene, probe_spacing};

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
            "corner_tolerance": 0.1, "goal_tolerance": 0.01, "time_limit": 1.0,
            "iterations": 1, "scenes": []}"#,
    )
    .expect("a scenario's header");
    // Keys -5 to 4 lie inside the map of 0.04 m voxels 0.2 m about the origin.
    let keys: Vec<[i32; 3]> = tc
        .draw(gs::vecs(gs::arrays(gs::integers::<i32>().min_value(-5).max_value(4))).max_size(60));
    let scene_of = |boxes: Vec<VoxelBox>| Scene {
        boxes,
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

// This crate plans, scene by scene, the answer each scenario carries for it: the paths
// the reference probed are this crate's, rounded as execute_joint_path rounds them, and
// probed at the spacing this crate's arm gives.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_carries_the_paths_this_crate_plans() {
    for case in Case::load_all() {
        assert_eq!(case.scenes.corner_tolerance, JOINT_PATH_CORNER_DEVIATION);
        on_the_arm(&case.scenes, &case.urdf, &case.meshes, |arm| {
            let sampler = IkGoalSampler::new(arm.geometry().chain().clone());
            assert_eq!(
                case.probe_spacing,
                probe_spacing(arm, &case.scenes),
                "{}: the scenario's probe spacing is its arm's",
                case.name
            );
            for (k, c) in case.cases.iter().enumerate() {
                assert_eq!(
                    plan_scene(arm, &sampler, &case.scenes, &c.scene),
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

// Over the scenes of a drawn set, this crate's success rate and mean smallest clearance
// lie within or above MoveIt's 95% intervals, and its mean path length within or below
// MoveIt's; a probe of MoveIt's in collision measures a clearance of -1. The committed
// scenes, one of each outcome, are too few for the intervals: the test runs on a drawn
// set, locally, as the harness README's rule 3 says, and the pull request states its
// tally at its head.
#[test]
#[ignore = "statistics of a drawn set, run locally as the harness README's rule 3 says"]
fn the_success_rate_the_clearance_and_the_length_lie_on_their_side_of_moveits() {
    for case in Case::load_all() {
        let scenes: &[SceneCase] = &case.cases;
        assert!(
            scenes.len() >= MIN_SCENES,
            "{}: {} scenes; the statistics need a drawn set of at least {MIN_SCENES}: \
             draw_arm_planning.py --drawn 200, run.sh, then cargo test -- --ignored",
            case.name,
            scenes.len()
        );
        let ours: Vec<Probed> = scenes.iter().filter_map(|c| c.ours_probed).collect();
        let theirs: Vec<Probed> = scenes.iter().filter_map(|c| c.moveit.probed).collect();
        let (low, high) = rate_interval(theirs.len(), scenes.len());
        let rate = ours.len() as f64 / scenes.len() as f64;
        println!(
            "{}: success rate ours {rate}, MoveIt's interval {low}..{high}",
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
            "{}: clearance ours {clearance}, MoveIt's interval {low}..{high}",
            case.name
        );
        assert!(
            clearance >= low,
            "{}: a mean clearance of {clearance}, below MoveIt's interval from {low}",
            case.name
        );
        let (length, (low, high)) = interval(|p| p.length);
        println!(
            "{}: length ours {length}, MoveIt's interval {low}..{high}",
            case.name
        );
        assert!(
            length <= high,
            "{}: a mean length of {length}, above MoveIt's interval to {high}",
            case.name
        );
    }
}
