//! Parity with MoveIt's Cartesian interpolator, as move_group's compute_cartesian_path
//! service runs it, on the Cartesian paths scenarios of the ROS reference harness: an arm of
//! OpenArm v2, KDL solving MoveIt's inverse kinematics, this crate's paths at the scenario's
//! clearance, which pads MoveIt's links. This crate still computes the paths each scenario
//! carries. Each pose of MoveIt's paths lies within the precision of the lines from the start
//! through the case's poses, the lines this crate follows. Both take the same steps of each
//! line, solved by one inverse kinematics from the state before, so with collisions unchecked
//! this crate covers the share of each path that MoveIt covers. With collisions checked, a
//! step passes on each side's own check, MoveIt's of its padded meshes at the step's state,
//! and this crate's of its pieces' clearance from the map and the objects at each state and
//! along the motion between them, so at a clearance of 0 a line ends within one step of where
//! MoveIt's ends, and at the robot's clearance no more than one step past it. Over the cases of
//! a drawn set, the tests tally how many cases keep each bound.

mod arm_scenes;
mod cartesian_scenes;
mod common;

use chain_kinematics::{CARTESIAN_STEP_M, CARTESIAN_STEP_RAD};
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use openarm_description::{HardwareVersion, Side};
use ros_reference::LinkPose;
use ros_reference::cartesian_paths::{Case, Ours, PathCase};

#[cfg(target_arch = "x86_64")]
use arm_scenes::on_the_arms;
use arm_scenes::{JOINTS, MIN_SCENES, geometry};
use cartesian_scenes::corners;
#[cfg(target_arch = "x86_64")]
use cartesian_scenes::path_answer;
use common::on_line;

/// The difference allowed between two fractions of the same steps: their rounding.
const FRACTION_TOLERANCE: f64 = 1e-12;
/// This crate's failure of a start in contact, which MoveIt, checking no start, answers with
/// a path.
const START_IN_COLLISION: &str = "start_in_collision";

/// The steps of each line between `corners`, as many as MoveIt takes: the more of its length
/// over the step and its turn over the step's turn, each rounded down, plus one (moveit2
/// 2.15.2 cartesian_interpolator.cpp:237-245).
fn line_steps(corners: &[Isometry3<f64>]) -> Vec<usize> {
    corners
        .windows(2)
        .map(|line| {
            let length = (line[1].translation.vector - line[0].translation.vector).norm();
            let turn = line[0].rotation.angle_to(&line[1].rotation);
            let steps = (length / CARTESIAN_STEP_M)
                .floor()
                .max((turn / CARTESIAN_STEP_RAD).floor());
            steps as usize + 1
        })
        .collect()
}

/// How many steps apart two fractions of a path stand, whose lines take `line_steps` steps:
/// each line, an equal share of the path, counts its own steps.
fn steps_apart(line_steps: &[usize], a: f64, b: f64) -> f64 {
    let lines = line_steps.len() as f64;
    let (near, far) = (a.min(b) * lines, a.max(b) * lines);
    line_steps
        .iter()
        .enumerate()
        .map(|(k, steps)| {
            let line = k as f64;
            (far.min(line + 1.0) - near.max(line)).max(0.0) * *steps as f64
        })
        .sum()
}

/// The steps a fraction's rounding moves it at most, on lines of `line_steps` steps.
fn rounding_in_steps(line_steps: &[usize]) -> f64 {
    let most = line_steps.iter().copied().max().unwrap_or(0);
    FRACTION_TOLERANCE * line_steps.len() as f64 * most as f64
}

/// Our fraction of `ours`, none where this crate refuses its start.
fn fraction(ours: &Ours) -> Option<f64> {
    match ours {
        Ours::Path { fraction, .. } => Some(*fraction),
        Ours::Failure { .. } => None,
    }
}

/// A case with what the tests read of it, its place in its scenario, the poses its lines run
/// through and the steps of each line.
struct Compared<'a> {
    index: usize,
    case: &'a PathCase,
    corners: Vec<Isometry3<f64>>,
    line_steps: Vec<usize>,
}

impl Compared<'_> {
    fn ours(&self) -> Option<f64> {
        fraction(&self.case.request.ours)
    }

    /// Whether this crate covers the share MoveIt covers.
    fn same_fraction(&self) -> bool {
        self.ours()
            .is_some_and(|ours| (ours - self.case.moveit.fraction).abs() <= FRACTION_TOLERANCE)
    }

    /// How many steps this crate's path ends from MoveIt's, none where this crate refuses the
    /// start.
    fn steps_from_moveits(&self) -> Option<f64> {
        self.ours()
            .map(|ours| steps_apart(&self.line_steps, ours, self.case.moveit.fraction))
    }

    /// Whether this crate's path ends within one step of MoveIt's.
    fn within_one_step(&self) -> bool {
        self.steps_from_moveits()
            .is_some_and(|apart| apart <= 1.0 + rounding_in_steps(&self.line_steps))
    }

    /// Whether this crate's path ends more than one step past MoveIt's.
    fn past_moveits(&self) -> bool {
        self.ours()
            .is_some_and(|ours| ours > self.case.moveit.fraction && !self.within_one_step())
    }
}

/// The cases of `case`, each as the tests read it.
fn compared(case: &Case) -> Vec<Compared<'_>> {
    let chain = &case.paths.chains[0];
    let geometry = geometry::<JOINTS>(chain, &case.urdf, &case.meshes);
    case.cases
        .iter()
        .enumerate()
        .map(|(index, c)| {
            let corners = corners(geometry.chain(), &chain.tip_link, &c.request);
            Compared {
                index,
                case: c,
                line_steps: line_steps(&corners),
                corners,
            }
        })
        .collect()
}

/// The pose a reference output gives.
fn pose_of(link: &LinkPose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(Vector3::from(link.translation)),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
            Matrix3::from_row_slice(&link.rotation),
        )),
    )
}

/// Whether `c` holds a box of voxels or a collision object.
fn among_obstacles(c: &PathCase) -> bool {
    !c.request.boxes.is_empty() || !c.request.objects.is_empty()
}

/// Whether `c` follows the left arm's grasp point, the scenario's chain's tip link.
fn follows_the_grasp_point(c: &PathCase) -> bool {
    c.request.link == HardwareVersion::V2.tcp_link(Side::Left)
}

/// Whether `c` turns its link in place on each line.
fn turns_in_place(c: &PathCase) -> bool {
    c.request.offsets.iter().all(|o| o.translation == [0.0; 3])
}

/// Whether `c` is one line of the grasp point that moves, among no obstacle.
#[cfg(target_arch = "x86_64")]
fn one_free_line(c: &PathCase) -> bool {
    c.request.offsets.len() == 1
        && follows_the_grasp_point(c)
        && !turns_in_place(c)
        && !among_obstacles(c)
}

/// Whether the first line of `c` neither moves nor turns.
#[cfg(target_arch = "x86_64")]
fn first_line_stands_still(c: &PathCase) -> bool {
    let first = &c.request.offsets[0];
    first.translation == [0.0; 3] && first.rotation == [0.0; 3]
}

/// Whether `c` holds two lines and its first leaves the start.
#[cfg(target_arch = "x86_64")]
fn two_lines_that_move(c: &PathCase) -> bool {
    c.request.offsets.len() == 2 && !first_line_stands_still(c)
}

/// Whether this crate and MoveIt each cover a share of `c` that `holds` holds of.
#[cfg(target_arch = "x86_64")]
fn both(c: &PathCase, holds: impl Fn(f64) -> bool) -> bool {
    fraction(&c.request.ours).is_some_and(&holds) && holds(c.moveit.fraction)
}

/// Whether this crate and MoveIt each cover the whole of `c`.
#[cfg(target_arch = "x86_64")]
fn both_whole(c: &PathCase) -> bool {
    both(c, |share| share == 1.0)
}

/// Whether this crate's path and MoveIt's each leave the start of `c` and end before its
/// last pose.
#[cfg(target_arch = "x86_64")]
fn both_end_along(c: &PathCase) -> bool {
    both(c, |share| 0.0 < share && share < 1.0)
}

/// A kind of case, by its name, and whether a case is of it: a behavior a scenario holds, or
/// a group a tally prints.
type KindOfCase = (&'static str, fn(&PathCase) -> bool);

// This crate computes, case by case, the answer each scenario carries for it, at the step and
// against the pairs of the robot's own links this crate's paths check. Its restarts draw from
// the case's seed and its search ends on its passes, so a path repeats.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_carries_the_paths_this_crate_computes() {
    for case in Case::load_all() {
        assert_eq!(
            case.step, CARTESIAN_STEP_M,
            "{}: the scenario's step is this crate's",
            case.name
        );
        on_the_arms::<JOINTS, _>(
            &case.paths.chains,
            &case.urdf,
            &case.meshes,
            |arms, standing| {
                assert_eq!(
                    case.checked_pairs,
                    standing.checked_pairs(),
                    "{}: the scenario's checked pairs are those its paths check",
                    case.name
                );
                for (k, c) in case.cases.iter().enumerate() {
                    assert_eq!(
                        path_answer(arms, standing, &case.paths, &c.request),
                        c.request.ours,
                        "{} case {k}",
                        case.name
                    );
                }
            },
        );
    }
}

// Each scenario holds a case of each behavior, shown by both sides where both answer: a free
// line both cover whole, a line past the reach both end along, collisions unchecked, a line
// into a box both end along and the same line unchecked, which both cover whole, a line into
// a collision object, a turn in place, two lines whole, two lines of which both end the
// second, a line of the last joint's link, and a start in contact, which this crate refuses.
// The committed outputs hold MoveIt's amd64 answers, so the test runs on amd64.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_holds_a_case_of_each_behavior() {
    let behaviors: [KindOfCase; 11] = [
        ("a free line whole", |c| {
            c.request.avoid_collisions && one_free_line(c) && both_whole(c)
        }),
        ("a line past the reach", |c| {
            !c.request.avoid_collisions && one_free_line(c) && both(c, |share| share < 1.0)
        }),
        ("a line that a box ends", |c| {
            c.request.avoid_collisions && !c.request.boxes.is_empty() && both_end_along(c)
        }),
        ("a line through a box, unchecked, whole", |c| {
            !c.request.avoid_collisions && !c.request.boxes.is_empty() && both_whole(c)
        }),
        ("a line that a collision object ends", |c| {
            c.request.avoid_collisions && !c.request.objects.is_empty() && both_end_along(c)
        }),
        ("a turn in place", |c| turns_in_place(c) && both_whole(c)),
        ("two lines whole", |c| {
            two_lines_that_move(c) && both_whole(c)
        }),
        ("a line that stands still, then a line", |c| {
            first_line_stands_still(c) && both_whole(c)
        }),
        ("two lines, the second ended", |c| {
            two_lines_that_move(c) && both(c, |share| (0.5..1.0).contains(&share))
        }),
        ("a line of the last joint's link", |c| {
            !follows_the_grasp_point(c) && both_whole(c)
        }),
        ("a start in contact", |c| {
            c.request.ours
                == Ours::Failure {
                    failure: START_IN_COLLISION.to_string(),
                }
        }),
    ];
    for case in Case::load_all() {
        for (behavior, shows) in behaviors {
            assert!(
                case.cases.iter().any(shows),
                "{} holds no case of {behavior}",
                case.name
            );
        }
    }
}

// MoveIt's paths follow the lines this crate follows: each pose of each of its paths, the
// states of a step that failed among them, stands within the precision of a line from the
// case's start through its poses, each of which each side places by its own forward
// kinematics.
#[test]
fn moveits_poses_lie_on_this_crates_lines() {
    for case in Case::load_all() {
        for c in compared(&case) {
            for (k, pose) in c.case.moveit.poses.iter().enumerate() {
                let actual = pose_of(pose);
                assert!(
                    c.corners
                        .windows(2)
                        .any(|line| on_line(&actual, &line[0], &line[1])),
                    "{} case {} state {k} stands off its lines: {actual:?}",
                    case.name,
                    c.index
                );
            }
        }
    }
}

// With collisions unchecked, no collision ends a path, so this crate covers the share of each
// path that MoveIt covers, within their rounding.
#[test]
fn with_collisions_unchecked_the_fraction_is_moveits() {
    for case in Case::load_all() {
        for c in compared(&case) {
            if c.case.request.avoid_collisions {
                continue;
            }
            assert!(
                c.same_fraction(),
                "{} case {}: this crate covers {:?}, MoveIt {}",
                case.name,
                c.index,
                c.case.request.ours,
                c.case.moveit.fraction
            );
        }
    }
}

// With collisions checked, each path ends within one step of MoveIt's, counting across the
// end of a line. A start in contact is this crate's refusal: MoveIt checks no start, and the
// test prints the fraction it answers. The test names every case past the bound.
#[test]
fn with_collisions_checked_the_line_ends_within_one_step_of_moveits() {
    for case in Case::load_all() {
        let mut exceptions = Vec::new();
        for c in compared(&case) {
            if !c.case.request.avoid_collisions {
                continue;
            }
            match &c.case.request.ours {
                Ours::Failure { failure } if failure == START_IN_COLLISION => println!(
                    "{} case {}: this crate refuses the start in contact, MoveIt covers {}",
                    case.name, c.index, c.case.moveit.fraction
                ),
                Ours::Failure { failure } => {
                    exceptions.push(format!("case {}: this crate fails: {failure}", c.index));
                }
                Ours::Path { fraction, .. } if !c.within_one_step() => {
                    exceptions.push(format!(
                        "case {}: this crate covers {fraction}, MoveIt {}: {:?} steps apart",
                        c.index,
                        c.case.moveit.fraction,
                        c.steps_from_moveits()
                    ));
                }
                Ours::Path { .. } => {}
            }
        }
        assert!(exceptions.is_empty(), "{}: {exceptions:#?}", case.name);
    }
}

// How many steps apart two fractions stand counts each line's own steps, across the end of a
// line too.
#[test]
fn steps_apart_counts_each_lines_own_steps() {
    let lines = [10, 20];
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-9;
    assert!(close(steps_apart(&lines, 0.0, 0.25), 5.0));
    assert!(close(steps_apart(&lines, 0.25, 0.0), 5.0));
    assert!(close(steps_apart(&lines, 0.5, 0.75), 10.0));
    assert!(close(steps_apart(&lines, 0.45, 0.525), 2.0));
    assert!(close(steps_apart(&lines, 0.3, 0.3), 0.0));
}

// A line takes as many steps as MoveIt takes: its length or its turn over the step's, the
// larger, rounded down, plus one.
#[test]
fn a_line_takes_the_steps_of_its_length_or_its_turn() {
    let start = Isometry3::identity();
    let moved = |x: f64| Isometry3::translation(x, 0.0, 0.0);
    let turned = |angle: f64| {
        Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle),
        )
    };
    assert_eq!(line_steps(&[start, start]), [1]);
    assert_eq!(line_steps(&[start, moved(0.105)]), [11]);
    assert_eq!(line_steps(&[start, turned(0.1)]), [3]);
    assert_eq!(line_steps(&[start, moved(0.105), moved(0.21)]), [11, 11]);
}

/// How the cases of a group of a drawn set compare with MoveIt's paths.
#[derive(Debug)]
struct Tally {
    cases: usize,
    refused: usize,
    same_fraction: usize,
    within_one_step: usize,
    past_moveits: usize,
}

fn tally<'a, 'b: 'a>(cases: impl Iterator<Item = &'a Compared<'b>>) -> Tally {
    cases.fold(
        Tally {
            cases: 0,
            refused: 0,
            same_fraction: 0,
            within_one_step: 0,
            past_moveits: 0,
        },
        |sum, c| Tally {
            cases: sum.cases + 1,
            refused: sum.refused + usize::from(c.ours().is_none()),
            same_fraction: sum.same_fraction + usize::from(c.same_fraction()),
            within_one_step: sum.within_one_step + usize::from(c.within_one_step()),
            past_moveits: sum.past_moveits + usize::from(c.past_moveits()),
        },
    )
}

/// The scenario of a drawn set, which its tally refuses when it is too small.
fn drawn_set() -> Case {
    let mut cases = Case::load_all();
    assert_eq!(
        cases.len(),
        1,
        "scenarios/ holds one Cartesian paths scenario"
    );
    let case = cases.remove(0);
    assert!(
        case.cases.len() >= MIN_SCENES,
        "{}: {} cases; the tally needs a drawn set of at least {MIN_SCENES}: \
         draw_cartesian_paths.py --drawn 200, run.sh, then this test",
        case.name,
        case.cases.len()
    );
    case
}

/// Prints the tally of the cases of `case` whose checking is `checked`, whole and by group,
/// and gives the whole.
fn printed_tally(case: &Case, cases: &[Compared], checked: bool) -> Tally {
    let groups: [KindOfCase; 7] = [
        ("no obstacle", |c| !among_obstacles(c)),
        ("among boxes", |c| !c.request.boxes.is_empty()),
        ("among objects", |c| !c.request.objects.is_empty()),
        ("a turn in place", turns_in_place),
        ("one line", |c| c.request.offsets.len() == 1),
        ("two lines", |c| c.request.offsets.len() == 2),
        ("the last joint's link", |c| !follows_the_grasp_point(c)),
    ];
    let kept: Vec<&Compared> = cases
        .iter()
        .filter(|c| c.case.request.avoid_collisions == checked)
        .collect();
    for (group, holds) in groups {
        println!(
            "{}: collisions checked {checked}, clearance {} m: {group}: {:?}",
            case.name,
            case.paths.min_clearance,
            tally(kept.iter().copied().filter(|c| holds(c.case)))
        );
    }
    let whole = tally(kept.iter().copied());
    println!(
        "{}: collisions checked {checked}, clearance {} m: all: {whole:?}",
        case.name, case.paths.min_clearance
    );
    whole
}

// Over the cases of a drawn set whose collisions go unchecked, this crate covers the share of
// each path that MoveIt covers. The committed cases, one of each behavior, are too few for a
// rate: the test runs on a drawn set, locally, as the harness README's rule 3 says, and the
// pull request states its tally at its head.
#[test]
#[ignore = "a tally of a drawn set, run locally as the harness README's rule 3 says"]
fn over_a_drawn_set_with_collisions_unchecked_the_fraction_is_moveits() {
    let case = drawn_set();
    let cases = compared(&case);
    let whole = printed_tally(&case, &cases, false);
    assert_eq!(whole.same_fraction, whole.cases, "{whole:?}");
}

// Over the cases of a drawn set whose collisions are checked, a start in contact this crate
// refuses set aside: at a clearance of 0, this crate's path ends within one step of MoveIt's.
// At the clearance of the robot's launch, as rule 7 says, this crate's path ends no more than
// one step past MoveIt's. There MoveIt's path ends later: no state of this crate's after the
// start comes nearer than the clearance, and over the 27 drawn cases at a padding of 0.04 m
// where this crate's path ends first, the last state of MoveIt's stood 0 to 39 mm from the
// obstacle, 21 mm at the median.
#[test]
#[ignore = "a tally of a drawn set, run locally as the harness README's rule 3 says"]
fn over_a_drawn_set_with_collisions_checked_the_line_ends_at_most_one_step_past_moveits() {
    let case = drawn_set();
    let cases = compared(&case);
    let whole = printed_tally(&case, &cases, true);
    if case.paths.min_clearance == 0.0 {
        assert_eq!(
            whole.within_one_step,
            whole.cases - whole.refused,
            "{whole:?}"
        );
    } else {
        assert_eq!(whole.past_moveits, 0, "{whole:?}");
    }
}
