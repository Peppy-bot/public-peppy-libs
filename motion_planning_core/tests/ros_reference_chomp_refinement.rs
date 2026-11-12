//! Parity with MoveIt's CHOMP on the CHOMP refinement scenarios of the ROS reference harness:
//! MoveIt's CHOMP optimizer, at the parameters of its planner plugin, and this crate's CHOMP
//! refinement refine the same input path, the path this crate's RRTConnect plans through each
//! scene. This crate still plans each input path and each refinement a scenario carries;
//! MoveIt's collision check finds every probe of each refinement of ours free; and where
//! MoveIt's CHOMP answers a path of a committed scene, ours bends no more than MoveIt's. Over
//! the scenes of a drawn set whose input path turns at a waypoint, where both refinements are
//! free, ours bends less than MoveIt's on more scenes than it bends more, by a two-sided sign
//! test at the 5% level.

mod arm_scenes;
mod openarm_v2;

use ros_reference::arm_planning::{MOVEIT_SUCCESS, Probed};
#[cfg(target_arch = "x86_64")]
use ros_reference::arm_planning::{Ours, Refiner};
use ros_reference::chomp_refinement::{Case, RefinementCase};

#[cfg(target_arch = "x86_64")]
use arm_scenes::{on_the_arms, plan_scene, samplers};
#[cfg(target_arch = "x86_64")]
use openarm_v2::{ARMS, JOINTS};

/// The level of the sign test: the chance at even odds of a tally as far from even.
const SIGNIFICANCE: f64 = 0.05;

// Each scenario carries the paths this crate plans for its scenes: the input path,
// RRTConnect's plan ended on the scenario's iterations, and our refinement, the same plan
// refined by CHOMP. Plans ended on iterations repeat under their seeds. The committed outputs
// hold the paths of an amd64 build, so the test runs on amd64, as the arm planning parity
// test does.
#[cfg(target_arch = "x86_64")]
#[test]
fn each_scenario_carries_the_paths_this_crate_plans() {
    for case in Case::load_all() {
        let refinements = &case.refinements;
        on_the_arms::<JOINTS, ARMS, _>(
            std::slice::from_ref(&case.arm),
            &case.robot,
            |arms, standing| {
                let samplers = samplers(arms);
                for refiner in [Refiner::None, Refiner::Chomp] {
                    let plans = refinements.plans(refiner);
                    for (k, (scene, c)) in plans.scenes.iter().zip(&case.cases).enumerate() {
                        let carried = match refiner {
                            Refiner::None => &c.scene.path,
                            Refiner::Chomp => &c.scene.refined,
                        };
                        assert_eq!(
                            plan_scene::<JOINTS, JOINTS, ARMS, _>(
                                arms, standing, &samplers, &plans, scene
                            )
                            .ours,
                            Ours::Path {
                                waypoints: carried.clone()
                            },
                            "{} scene {k}, refined by {refiner:?}",
                            case.name
                        );
                    }
                }
            },
        );
    }
}

// Each scenario plans at a clearance of 0, and MoveIt's collision check finds every probe of
// each refinement of ours free.
#[test]
fn moveit_finds_every_refinement_of_this_crate_free() {
    for case in Case::load_all() {
        assert_eq!(
            case.refinements.min_clearance, 0.0,
            "{}: the parity plans at a clearance of 0",
            case.name
        );
        for (k, c) in case.cases.iter().enumerate() {
            assert_eq!(
                c.ours_probed.collisions, 0,
                "{} scene {k}: MoveIt finds our refinement in collision",
                case.name
            );
        }
    }
}

// Where MoveIt's CHOMP answers a path of a scene, our refinement of the same input path bends
// no more than MoveIt's path. The test prints the bending of the input path, of MoveIt's and of
// ours.
#[test]
fn where_moveits_chomp_answers_a_path_ours_bends_no_more() {
    for case in Case::load_all() {
        for (k, c) in case.cases.iter().enumerate() {
            let Some(theirs) = &c.chomp.probed else {
                continue;
            };
            let what = format!("{} scene {k}", case.name);
            println!(
                "{what}: bending of the input path {:.4}, of MoveIt's CHOMP {:.4}, of ours {:.4}",
                c.input_probed.bending, theirs.bending, c.ours_probed.bending
            );
            assert!(
                c.ours_probed.bending <= theirs.bending,
                "{what}: our refinement bends more than MoveIt's CHOMP"
            );
        }
    }
}

/// The two-sided p-value of the sign test of `lower` scenes against `higher`: twice the chance
/// at even odds of at most the fewer of them among their sum, and at most 1. The chance of k
/// of n starts from that of none, 2^-n, and C(n, k + 1) = C(n, k) (n - k) / (k + 1).
fn sign_test(lower: usize, higher: usize) -> f64 {
    let n = lower + higher;
    let none = 0.5_f64.powi(i32::try_from(n).expect("a count of scenes"));
    let tail: f64 = (0..=lower.min(higher))
        .scan(none, |chance, k| {
            let this = *chance;
            *chance *= (n - k) as f64 / (k + 1) as f64;
            Some(this)
        })
        .sum();
    (2.0 * tail).min(1.0)
}

/// MoveIt's probe of its CHOMP path and of our refinement of a scene, where MoveIt's CHOMP
/// succeeds and its collision check finds both free.
fn both_free(c: &RefinementCase) -> Option<(&Probed, &Probed)> {
    let theirs = c.chomp.probed.as_ref()?;
    (c.chomp.status == MOVEIT_SUCCESS && theirs.collisions == 0 && c.ours_probed.collisions == 0)
        .then_some((theirs, &c.ours_probed))
}

/// A measure of a probed path, where the probe gives one.
type Measure = fn(&Probed) -> Option<f64>;

fn bending(probed: &Probed) -> Option<f64> {
    Some(probed.bending)
}

/// The measures the drawn set's tally prints, by name.
const MEASURES: [(&str, Measure); 5] = [
    ("length", |p| Some(p.length)),
    ("turning", |p| Some(p.turning)),
    ("bending", bending),
    ("duration", |p| p.duration),
    ("clearance", |p| Some(p.clearance)),
];

/// The scenes where a measure of ours is lower than MoveIt's, equal and higher.
struct Tally {
    lower: usize,
    equal: usize,
    higher: usize,
}

/// The tally of `measure` over `pairs`, each MoveIt's probe and ours.
fn tally(pairs: &[(&Probed, &Probed)], measure: Measure) -> Tally {
    let values: Vec<(f64, f64)> = pairs
        .iter()
        .filter_map(|(theirs, ours)| Some((measure(theirs)?, measure(ours)?)))
        .collect();
    let lower = values.iter().filter(|(theirs, ours)| ours < theirs).count();
    let higher = values.iter().filter(|(theirs, ours)| ours > theirs).count();
    Tally {
        lower,
        equal: values.len() - lower - higher,
        higher,
    }
}

/// Whether the input path of a scene turns at a waypoint.
fn turns(c: &RefinementCase) -> bool {
    c.scene.path.len() > 2
}

// Over the scenes of a drawn set where both refinements are free and the input path turns at
// a waypoint, ours bends less than MoveIt's on more scenes than it bends more, by a two-sided
// sign test at SIGNIFICANCE. The test prints, for the scenes whose input path turns and for
// those whose input path is straight, each measure's scenes where ours is lower, equal and
// higher.
#[test]
#[ignore = "statistics of a drawn set, run locally as the harness README's rule 3 says"]
fn over_a_drawn_set_our_refinement_of_a_turning_path_bends_less_than_moveits_chomp() {
    for case in Case::load_all() {
        let of_input = |turning: bool| -> Vec<(&Probed, &Probed)> {
            case.cases
                .iter()
                .filter(|c| turns(c) == turning)
                .filter_map(both_free)
                .collect()
        };
        for (input, pairs) in [("turns", of_input(true)), ("is straight", of_input(false))] {
            println!(
                "{}: {} scenes where both refinements are free and the input path {input}",
                case.name,
                pairs.len()
            );
            for (name, measure) in MEASURES {
                let Tally {
                    lower,
                    equal,
                    higher,
                } = tally(&pairs, measure);
                println!(
                    "  {name}: ours lower {lower}, equal {equal}, higher {higher}, p = {:.2e}",
                    sign_test(lower, higher)
                );
            }
        }
        let Tally { lower, higher, .. } = tally(&of_input(true), bending);
        assert!(
            lower > higher && sign_test(lower, higher) < SIGNIFICANCE,
            "{}: of the turning input paths, ours bends less on {lower} scenes and more on \
             {higher}",
            case.name
        );
    }
}

#[test]
fn the_sign_test_of_a_tally_is_its_binomial_tail() {
    assert_eq!(sign_test(0, 0), 1.0);
    assert_eq!(sign_test(3, 3), 1.0);
    // 2 (1 + 10) / 2^10.
    assert!((sign_test(9, 1) - 22.0 / 1024.0).abs() < 1e-15);
    assert_eq!(sign_test(1, 9), sign_test(9, 1));
    assert!(sign_test(5, 0) > SIGNIFICANCE && sign_test(6, 0) < SIGNIFICANCE);
}
