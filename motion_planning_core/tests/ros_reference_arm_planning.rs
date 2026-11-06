//! Parity with MoveIt's planning pipeline on the arm planning scenarios of the ROS
//! reference harness, of one arm and of both, at one attempt, the grippers' fingers
//! carried by the arms in both, each plan with the planner of the scenario's objective,
//! and a plan of both arms checking the pairs of the robot's own links that this crate's
//! self model checks in both. The committed scenarios plan at a clearance of 0: this crate
//! still plans the paths each of them carries; MoveIt's collision check finds every probe
//! of each of them free; this crate refuses each start that MoveIt refuses outside a joint
//! limit or in collision; and each holds a scene of each outcome the README names. The
//! robot around a scenario's arm, either arm, holds the other arm at its zero
//! configuration. Over the scenes of a drawn set, at its clearance, which pads MoveIt's
//! links, the success rate and the mean smallest clearances, to the scene and between the
//! robot's own bodies, lie within or above MoveIt's 95% intervals, and the mean path length
//! within or below its interval, of both arms at a clearance above 0 on the scenes both plan
//! where MoveIt's path keeps the clearances. Over a drawn set whose scenes hold path orientations,
//! MoveIt's own forward kinematics finds every probe of each rounded path of ours inside each
//! tolerance, and the tally states the share of the tolerance MoveIt's own paths reach.

mod arm_scenes;

use std::collections::{BTreeMap, BTreeSet};

use control_core::path_timing::JOINT_PATH_CORNER_DEVIATION;
use hegel::TestCase;
use hegel::generators as gs;
use ros_reference::arm_planning::{
    Case, Held, MOVEIT_SUCCESS, Ours, Probed, SceneCase, SceneGoal, Scenes, Termination, VoxelBox,
};

use arm_scenes::{
    JOINTS, MIN_SCENES, boxes_of, geometry, joint_goal, least_self_clearance_along, map_of,
    on_the_arms, plan_scene, probe_spacing, samplers, state,
};
use nalgebra::{Point3, Vector3};
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

/// The 95% interval of the mean of `values`.
fn mean_interval(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let half = Z_95 * (variance / n).sqrt();
    (mean - half, mean + half)
}

/// The fewest values `mean_interval` draws an interval from: it divides by one less than their
/// count.
const LEAST_TO_COMPARE: usize = 2;

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

/// The least distance between the robot's own bodies in a scene of both arms, in this
/// crate's self model of their plans: at its start, and, for a goal of joints, at its goal
/// and along the straight joint line between them.
struct OwnDistances {
    start: f64,
    goal: Option<f64>,
    line: Option<f64>,
}

/// An outcome of a scene of both arms, by its name, and whether a scene shows it.
type OutcomeOfBoth = (&'static str, fn(&SceneCase, &OwnDistances) -> bool);

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
    // Keys -5 to 4 lie inside the map of 0.04 m voxels 0.2 m about the origin.
    let keys: Vec<[i32; 3]> = tc
        .draw(gs::vecs(gs::arrays(gs::integers::<i32>().min_value(-5).max_value(4))).max_size(60));
    let map_of_boxes = |boxes: &[VoxelBox]| map_of(0.04, 0.2, boxes);
    let voxels: Vec<VoxelBox> = keys
        .iter()
        .map(|&key| VoxelBox {
            first: key,
            last: key,
        })
        .collect();
    let map = map_of_boxes(&voxels);
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
    assert_eq!(occupied(&map_of_boxes(&boxes)), occupied(&map));
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
        let chain = &mut case.scenes.chains[0];
        chain.name = side.arm_name().to_string();
        chain.base_link = named(&chain.base_link);
        chain.tip_link = named(&chain.tip_link);
        chain.joints = chain.joints.iter().map(|joint| named(joint)).collect();
        let chains = &case.scenes.chains;
        on_the_arms(chains, &case.urdf, &case.meshes, |arms, standing| {
            let (parts, _) = standing.around(arms, &start);
            let base = HardwareVersion::V2.base_link(side);
            assert!(
                parts.iter().any(|part| part.link == base),
                "{side:?}: the planned arm's base link {base} does not stand"
            );
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
// the reference probed are this crate's, rounded as execute_joint_path rounds them,
// probed at the spacing this crate's arms give, against the pairs of the robot's own
// links this crate's plans check. Each scenario's plans end on its iterations, so they
// repeat.
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
        let chains = &case.scenes.chains;
        on_the_arms(chains, &case.urdf, &case.meshes, |arms, standing| {
            let samplers = samplers(arms);
            assert_eq!(
                case.probe_spacing,
                probe_spacing(arms, &case.scenes),
                "{}: the scenario's probe spacing is its arms'",
                case.name
            );
            assert_eq!(
                case.checked_pairs,
                standing.checked_pairs(),
                "{}: the scenario's checked pairs are those its plans check",
                case.name
            );
            for (k, c) in case.cases.iter().enumerate() {
                assert_eq!(
                    plan_scene(arms, standing, &samplers, &case.scenes, &c.scene).ours,
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
// above an upper limit that both refuse. The scenario of both arms to poses holds each
// but a path of ours where MoveIt's own fails its check: none of its first 200 drawn
// scenes, planned on the committed scenes' iterations, gives one. The committed outputs
// hold MoveIt's amd64 answers, whose outcomes the scenes were picked by, so the test runs
// on amd64.
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
        (MOVEIT_PATH_INVALID, |s| {
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
        let geometries: Vec<_> = case
            .scenes
            .chains
            .iter()
            .map(|chain| geometry(chain, &case.urdf, &case.meshes))
            .collect();
        let limits: Vec<_> = geometries
            .iter()
            .flat_map(|geometry| geometry.chain().limits())
            .collect();
        let shown: Vec<Shown> = case
            .cases
            .iter()
            .map(|c| Shown {
                case: c,
                waypoints: match &c.scene.ours {
                    Ours::Path { waypoints } => waypoints.len(),
                    Ours::Failure { .. } => 0,
                },
                below: c.scene.start.iter().zip(&limits).any(|(q, l)| *q < l.lo),
                above: c.scene.start.iter().zip(&limits).any(|(q, l)| *q > l.hi),
            })
            .collect();
        let both_to_poses = case.scenes.chains.len() > 1 && to_poses(&case);
        for (outcome, holds) in outcomes {
            if both_to_poses && outcome == MOVEIT_PATH_INVALID {
                continue;
            }
            assert!(
                shown.iter().any(holds),
                "{} holds no scene of {outcome}",
                case.name
            );
        }
    }
}

// Each scenario of both arms holds a scene of each outcome of a plan of both: a start
// where the robot's own bodies meet and a goal where they meet, which both refuse, and a
// straight joint line from the start to the goal on which they meet, which both plan
// around; a scenario to poses holds the first two, its drawn goals holding no
// configuration and so no joint line. So the pairs of the robot's own links that this
// crate's self model checks, and MoveIt's collision matrix with them, hold both plans'
// bodies apart. The committed outputs hold MoveIt's amd64 answers, whose outcomes the
// scenes were picked by, so the test runs on amd64.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_of_both_arms_holds_a_scene_of_each_of_their_outcomes() {
    let outcomes: [OutcomeOfBoth; 3] = [
        ("a start where the robot's own bodies meet", |c, own| {
            own.start < 0.0
                && failure(&c.scene.ours) == Some("start_in_collision")
                && c.moveit.status == MOVEIT_START_STATE_IN_COLLISION
        }),
        ("a goal where the robot's own bodies meet", |c, own| {
            own.goal.is_some_and(|goal| goal < 0.0)
                && failure(&c.scene.ours) == Some("unreachable")
                && c.moveit.status == MOVEIT_GOAL_STATE_INVALID
        }),
        (
            "a straight joint line through the robot's own body, planned around",
            |c, own| {
                own.line.is_some_and(|line| line < 0.0)
                    && matches!(c.scene.ours, Ours::Path { .. })
                    && c.moveit.status == MOVEIT_SUCCESS
            },
        ),
    ];
    let both: Vec<Case> = Case::load_all()
        .into_iter()
        .filter(|case| case.scenes.chains.len() == 2)
        .collect();
    assert!(!both.is_empty(), "scenarios/ holds a scenario of both arms");
    for case in both {
        let chains = &case.scenes.chains;
        on_the_arms(chains, &case.urdf, &case.meshes, |_, standing| {
            let shown: Vec<OwnDistances> = case
                .cases
                .iter()
                .map(|c| {
                    let goal = joint_goal(&c.scene.goal);
                    OwnDistances {
                        start: standing.self_clearance(&c.scene.start),
                        goal: goal.map(|goal| standing.self_clearance(goal)),
                        line: goal
                            .map(|goal| least_self_clearance_along(standing, &c.scene.start, goal)),
                    }
                })
                .collect();
            let held = if to_poses(&case) {
                &outcomes[..2]
            } else {
                &outcomes[..]
            };
            for (outcome, holds) in held {
                assert!(
                    case.cases.iter().zip(&shown).any(|(c, own)| holds(c, own)),
                    "{} holds no scene of {outcome}",
                    case.name
                );
            }
        });
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

/// The spare a plan keeps above its clearance between the robot's own bodies, 2 mm (the plan
/// module's `MIN_SPARE_M` over a pair of which both bodies move).
const OWN_BODY_SPARE_M: f64 = 2e-3;

/// Whether a scene of `case` reaches for poses.
fn to_poses(case: &Case) -> bool {
    case.cases
        .iter()
        .any(|c| matches!(c.scene.goal, SceneGoal::Poses(_)))
}

/// The outcome of a path of ours where MoveIt's own fails its check.
const MOVEIT_PATH_INVALID: &str = "a path where MoveIt's fails its check";

/// A group of scenes, by its name, and whether a scene belongs to it.
type Group<'a> = (&'static str, &'a dyn Fn(&SceneCase) -> bool);

/// The reach of the pose scenario's short moves: how far the drawing tool moves a short
/// move's grasp point (draw_arm_planning.py's SHORT_REACH_M).
const SHORT_REACH_M: f64 = 0.1;

/// Whether an arm of `scenes`'s chains holds `held`.
fn held_by_an_arm_of_the_plan(held: &Held, scenes: &Scenes) -> bool {
    scenes
        .chains
        .iter()
        .any(|chain| chain.tip_link == held.link)
}

/// The mean of `values`.
fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

/// The mean smallest distance between the robot's own bodies along `paths`, as a tally
/// writes it: none where no path holds one.
fn self_clearance(paths: &[Probed]) -> String {
    let values: Vec<f64> = paths.iter().filter_map(|p| p.self_clearance).collect();
    if values.is_empty() {
        return String::new();
    }
    format!(", self clearance {:.4}", mean(&values))
}

/// What `scenes` show: their number; this crate's paths and failures by name, and
/// MoveIt's paths and error codes; the mean smallest clearance, to the scene and between
/// the robot's own bodies where a scenario checks them, and path length of each one's
/// paths; how many of each one's paths MoveIt's check finds in collision at a probe; and
/// how many bring an object an arm holds nearer an object or a voxel than `held_within`.
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
    let mean_of = |paths: &[Probed], of: fn(&Probed) -> f64| {
        mean(&paths.iter().map(of).collect::<Vec<f64>>())
    };
    let colliding = |paths: &[Probed]| paths.iter().filter(|p| p.collisions > 0).count();
    let held_near = |paths: &[Probed]| {
        paths
            .iter()
            .filter(|p| p.held_clearance.is_some_and(|held| held < held_within))
            .count()
    };
    format!(
        "{} scenes; ours {} paths, failures {failures:?}, clearance {:.4}{}, length {:.3}, \
         {} in collision at a probe, {} holding an object nearer than {held_within} m; \
         MoveIt's {} paths, codes {codes:?}, clearance {:.4}{}, length {:.3}, {} in \
         collision at a probe, {} holding an object nearer than {held_within} m",
        scenes.len(),
        ours.len(),
        mean_of(&ours, |p| p.clearance),
        self_clearance(&ours),
        mean_of(&ours, |p| p.length),
        colliding(&ours),
        held_near(&ours),
        theirs.len(),
        mean_of(&theirs, |p| p.clearance),
        self_clearance(&theirs),
        mean_of(&theirs, |p| p.length),
        colliding(&theirs),
        held_near(&theirs),
    )
}

/// Asserts that over the scenes of the drawn set in the scenario `name`, this crate's
/// success rate and mean smallest clearances, to the scene and between the robot's own
/// bodies where the scenario checks them, lie within or above MoveIt's 95% intervals, and
/// its mean path length within or below MoveIt's interval. A plan of both arms at a clearance
/// above 0 holds the clearance where MoveIt's path may come nearer, which lengthens ours on
/// those scenes, so its length is judged on the scenes both plan where MoveIt's path keeps the
/// clearances (`keeps_the_clearances`), its length over every path printed beside it. A plan
/// that holds path orientations answers scenes MoveIt's does not, whose means say nothing of
/// the two, so its clearances and its length are judged on the scenes both plan where MoveIt's
/// path keeps the clearances. A set of path orientations takes MoveIt's rate as it comes: where
/// fewer than `LEAST_TO_COMPARE` such scenes remain, the rate alone judges it.
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
    let geometries: Vec<_> = case
        .scenes
        .chains
        .iter()
        .map(|chain| geometry(chain, &case.urdf, &case.meshes))
        .collect();
    // How far a goal's grasp point stands from the start's, the farthest of the arms', for a
    // goal of poses.
    let reach = |c: &SceneCase| match &c.scene.goal {
        SceneGoal::Poses(poses) => Some(
            poses
                .iter()
                .zip(&geometries)
                .zip(c.scene.start.chunks_exact(JOINTS))
                .map(|((pose, geometry), start)| {
                    (Vector3::from(pose.position)
                        - geometry
                            .chain()
                            .world_ee_pose(&state(start))
                            .translation
                            .vector)
                        .norm()
                })
                .fold(0.0, f64::max),
        ),
        SceneGoal::Joints(_) => None,
    };
    let held_by_the_plan = |c: &SceneCase| {
        c.scene
            .attached
            .iter()
            .any(|held| held_by_an_arm_of_the_plan(held, &case.scenes))
    };
    let groups: [Group; 7] = [
        ("all", &|_| true),
        ("holding", &held_by_the_plan),
        ("not holding", &|c| !held_by_the_plan(c)),
        ("an arm off the plan holding", &|c| {
            c.scene
                .attached
                .iter()
                .any(|held| !held_by_an_arm_of_the_plan(held, &case.scenes))
        }),
        ("with an allowed pair", &|c| !c.scene.allowed.is_empty()),
        ("a pose within the short reach of the start's", &|c| {
            reach(c).is_some_and(|r| r <= SHORT_REACH_M)
        }),
        ("a pose beyond it", &|c| {
            reach(c).is_some_and(|r| r > SHORT_REACH_M)
        }),
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
    let both_plan = || -> (Vec<Probed>, Vec<Probed>) {
        scenes
            .iter()
            .filter_map(|c| Some((c.ours_probed?, c.moveit.probed?)))
            .filter(|(_, theirs)| keeps_the_clearances(theirs, case.scenes.min_clearance))
            .unzip()
    };
    let holds_orientations = scenes.iter().any(|c| !c.scene.path_orientations.is_empty());
    let (ours, theirs) = if holds_orientations {
        both_plan()
    } else {
        (ours, theirs)
    };
    if holds_orientations {
        println!(
            "{}: on the {} scenes both plan where MoveIt's path keeps the clearances:",
            case.name,
            ours.len()
        );
        if ours.len() < LEAST_TO_COMPARE {
            println!(
                "{}: too few to judge: the rate alone judges this set",
                case.name
            );
            return;
        }
    }
    let interval = |of: fn(&Probed) -> f64| {
        let values: Vec<f64> = theirs.iter().map(of).collect();
        let mean = mean(&ours.iter().map(of).collect::<Vec<f64>>());
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
    if !case.checked_pairs.is_empty() {
        let (clearance, (low, high)) = interval(|p| {
            p.self_clearance
                .expect("a self distance where the scenario checks pairs")
        });
        println!(
            "{}: self clearance ours {clearance}, MoveIt's {} in {low}..{high}",
            case.name,
            (low + high) / 2.0
        );
        assert!(
            clearance >= low,
            "{}: a mean self clearance of {clearance}, below MoveIt's interval from {low}",
            case.name
        );
    }
    let (length, (low, high)) = interval(|p| p.length);
    println!(
        "{}: length ours {length}, MoveIt's {} in {low}..{high}",
        case.name,
        (low + high) / 2.0
    );
    if holds_orientations || case.scenes.min_clearance == 0.0 || case.checked_pairs.is_empty() {
        assert!(
            length <= high,
            "{}: a mean length of {length}, above MoveIt's interval to {high}",
            case.name
        );
        return;
    }
    let (ours, theirs) = both_plan();
    let (ours, theirs): (Vec<f64>, Vec<f64>) = (
        ours.iter().map(|p| p.length).collect(),
        theirs.iter().map(|p| p.length).collect(),
    );
    assert!(
        ours.len() >= MIN_SCENES,
        "{}: {} scenes both plan where MoveIt's path keeps the clearances; the statistics need \
         at least {MIN_SCENES}",
        case.name,
        ours.len()
    );
    let (length, (low, high)) = (mean(&ours), mean_interval(&theirs));
    println!(
        "{}: length on the {} scenes both plan where MoveIt's path keeps the clearances: ours \
         {length}, MoveIt's {} in {low}..{high}",
        case.name,
        ours.len(),
        (low + high) / 2.0
    );
    assert!(
        length <= high,
        "{}: where MoveIt's path keeps the clearances, a mean length of {length}, above its \
         interval to {high}",
        case.name
    );
}

/// Whether a path of MoveIt keeps the clearances a plan of both arms keeps at `min_clearance`:
/// from the scene, of the objects the arms hold from the scene, and between the robot's own
/// bodies, as `Probed` measures them unpadded. `Probed::self_clearance` also holds an object
/// against the arm that carries it, which a plan keeps 2 mm apart, so this leaves out more of
/// MoveIt's paths than the plan's clearances alone do.
fn keeps_the_clearances(path: &Probed, min_clearance: f64) -> bool {
    let from_the_scene = min_clearance + OBJECT_SPARE_M;
    path.clearance >= from_the_scene
        && path
            .held_clearance
            .is_none_or(|held| held >= from_the_scene)
        && path
            .self_clearance
            .is_none_or(|own| own >= min_clearance + OWN_BODY_SPARE_M)
}

// Over the scenes of a drawn set, of one arm or of both, among boxes of voxels, among
// collision objects or to a pose of each grasp point, this crate's success rate and mean
// smallest clearances, to the scene and between the robot's own bodies where the scenario
// checks them, lie within or above MoveIt's 95% intervals, and its mean path length within
// or below MoveIt's, of both arms at a clearance above 0 on the scenes both plan where
// MoveIt's path keeps the clearances; a probe of MoveIt's in collision measures a clearance
// of -1. The
// committed scenes, one of each outcome, are too few for the intervals: each test runs on
// its scenario's drawn set, locally, as the harness README's rule 3 says, and the pull
// request states its tally at its head, for each objective, termination and clearance it
// ran.
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

#[test]
#[ignore = "statistics of a drawn set of pose goals, run locally as the harness README's rule 3 says"]
fn among_poses_the_success_rate_the_clearance_and_the_length_lie_on_their_side_of_moveits() {
    lies_on_its_side_of_moveits("arm_planning_poses_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set of both arms among boxes, run locally as the harness README's rule 3 says"]
fn both_arms_with_boxes_the_success_rate_the_clearances_and_the_length_lie_on_their_side_of_moveits()
 {
    lies_on_its_side_of_moveits("arm_planning_both_arms_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set of both arms among objects, run locally as the harness README's rule 3 says"]
fn both_arms_with_objects_the_success_rate_the_clearances_and_the_length_lie_on_their_side_of_moveits()
 {
    lies_on_its_side_of_moveits("arm_planning_both_arms_objects_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set of both arms to poses, run locally as the harness README's rule 3 says"]
fn both_arms_to_poses_the_success_rate_the_clearances_and_the_length_lie_on_their_side_of_moveits()
{
    lies_on_its_side_of_moveits("arm_planning_both_arms_poses_openarm_v2");
}

/// How far past its tolerance a probe of a path of ours may read, in shares of the tolerance:
/// 1e-7 rad at a tolerance of 0.1 rad, a slack over the rounding of MoveIt's forward kinematics,
/// some 4e-15 per coordinate, and of the points of its rounded corners, 1e-12 in control_core's
/// path timing parity. The plan's own walks leave a hundredth of the tolerance for it.
const PROBE_SHARE_ROUNDING: f64 = 1e-6;

/// Asserts that each path of ours in the scenario `name` keeps every orientation its scene holds,
/// as MoveIt's forward kinematics finds it along the rounded path, and prints what MoveIt's own
/// paths reach; then the tally of `lies_on_its_side_of_moveits`.
fn holds_its_orientations_and_lies_on_its_side_of_moveits(name: &str) {
    let case = Case::load_all()
        .into_iter()
        .find(|case| case.name == name)
        .unwrap_or_else(|| panic!("scenarios/ holds no scenario {name}"));
    let shares = |of: &dyn Fn(&SceneCase) -> Option<Probed>| -> Vec<f64> {
        case.cases
            .iter()
            .filter_map(of)
            .map(|probed| {
                probed
                    .share_of_tolerance
                    .expect("a share where the scene holds an orientation")
            })
            .collect()
    };
    let held = &case.cases[0].scene.path_orientations[0];
    println!(
        "{name}: tolerance_rad {}, MoveIt's box {:?}",
        held.tolerance_rad, held.moveit_tolerances
    );
    let ours = shares(&|c| c.ours_probed);
    let theirs = shares(&|c| c.moveit.probed);
    let largest = |values: &[f64]| values.iter().copied().fold(0.0, f64::max);
    let past = |values: &[f64]| values.iter().filter(|share| **share > 1.0).count();
    println!(
        "{name}: share of the tolerance: ours largest {} of {} paths, {} past it; MoveIt's largest {} \
         of {} paths, {} past it",
        largest(&ours),
        ours.len(),
        past(&ours),
        largest(&theirs),
        theirs.len(),
        past(&theirs)
    );
    assert!(
        largest(&ours) <= 1.0 + PROBE_SHARE_ROUNDING,
        "{name}: a path of ours reaches {} of its tolerance",
        largest(&ours)
    );
    lies_on_its_side_of_moveits(name);
}

#[test]
#[ignore = "statistics of a drawn set holding a whole orientation, run locally as the harness README's rule 3 says"]
fn holding_a_whole_orientation_a_path_lies_on_its_side_of_moveits() {
    holds_its_orientations_and_lies_on_its_side_of_moveits("arm_planning_path_whole_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set holding a free axis, run locally as the harness README's rule 3 says"]
fn holding_a_free_axis_a_path_lies_on_its_side_of_moveits() {
    holds_its_orientations_and_lies_on_its_side_of_moveits("arm_planning_path_free_openarm_v2");
}

#[test]
#[ignore = "statistics of a drawn set holding two links, run locally as the harness README's rule 3 says"]
fn holding_two_links_a_path_lies_on_its_side_of_moveits() {
    holds_its_orientations_and_lies_on_its_side_of_moveits(
        "arm_planning_path_two_links_openarm_v2",
    );
}

#[test]
#[ignore = "statistics of a drawn set of both arms each holding an orientation, run locally as the harness README's rule 3 says"]
fn both_arms_holding_whole_orientations_a_path_lies_on_its_side_of_moveits() {
    holds_its_orientations_and_lies_on_its_side_of_moveits(
        "arm_planning_both_arms_path_whole_openarm_v2",
    );
}

#[test]
#[ignore = "statistics of a drawn set of both arms each holding a free axis, run locally as the harness README's rule 3 says"]
fn both_arms_holding_free_axes_a_path_lies_on_its_side_of_moveits() {
    holds_its_orientations_and_lies_on_its_side_of_moveits(
        "arm_planning_both_arms_path_free_openarm_v2",
    );
}
