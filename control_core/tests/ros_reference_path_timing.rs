//! Parity with MoveIt's time-optimal trajectory generation (TOTG) on the paths of every
//! path timing scenario of the ROS reference harness, blended at deviations from 0.01 to
//! 0.2, the deviation of execute_joint_path among them: where TOTG blends every corner,
//! the rounded path is TOTG's blended path, point for point at equal arc length; where
//! TOTG runs a corner straight on, a turn of under about 0.081 degrees, the rounded path
//! rounds it and lies within the path's deviation of TOTG's; and its timing is no faster
//! than TOTG's, to a part in 1e9, and within the grid's slack of it on a straight run. A
//! path that TOTG refuses because it turns back on itself comes to rest at that waypoint
//! and turns back. TOTG's turning-back threshold lies between the turns of the
//! scenario's reversal-boundary paths, which control_core rounds and stops at as TOTG
//! blends and refuses them. The scenarios hold no repeated waypoint: MoveIt's
//! `computeTimeStamps` removes repeated points before it builds TOTG's path (moveit2
//! 2.15.2), and control_core's rest at a repeated waypoint is its own rule, which its unit
//! tests check.

use control_core::path_timing::{JOINT_PATH_CORNER_DEVIATION, RoundedPath, TimedPath};
use control_core::positive_finite::PositiveFinite;
use ros_reference::path_timing::{Cases, PathCase, Totg};

/// The arc and the straight pieces are computed by different formulas here and in
/// TOTG (a turn from the start of the arc against a center and two axes), so the
/// points agree to the rounding of those formulas over paths a few radians long.
const POINT_TOLERANCE: f64 = 1e-12;

/// The deviations a scenario blends its cornered paths at.
const SCENARIO_DEVIATIONS: [f64; 4] = [0.01, 0.05, 0.1, 0.2];

/// The slack of the timing grid on a straight run: a straight run's time lies within
/// three parts in a thousand of its trapezoid (`CELLS_PER_PATH`), and TOTG's lies on
/// its trapezoid to its millisecond integration step.
const STRAIGHT_RUN_SLACK: f64 = 1.003;

/// The rounding of a duration computed by different formulas here and in TOTG, relative
/// to the duration: on a straight run both land on the run's trapezoid time.
const DURATION_ROUNDING: f64 = 1e-9;

/// The cosine of a turn above which TOTG runs a corner straight on through its
/// waypoint, a turn of under about 0.081 degrees (`CircularPathSegment`, moveit2
/// 2.15.2).
const TOTG_STRAIGHT_ON_COS: f64 = 0.999999;

/// The widest a scenario's turns may bracket TOTG's turning-back threshold, in degrees:
/// the sharpest turn TOTG blends and the mildest it refuses lie within this of each
/// other.
const TURNING_BACK_BRACKET_DEG: f64 = 0.02;

/// The even steps of its time at which a test samples a timed path for the speed at
/// which it reaches a turn.
const SPEED_STEPS: u32 = 100_000;

fn rounded(case: &PathCase) -> RoundedPath {
    RoundedPath::through(
        &case.waypoints,
        PositiveFinite::parse(case.max_deviation).expect("a positive deviation"),
    )
}

fn timed(case: &PathCase) -> TimedPath {
    TimedPath::along(rounded(case), &case.max_velocity, &case.max_acceleration)
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// The cosine of the turn of the corner at waypoint `k` of `waypoints`.
fn corner_cos(waypoints: &[Vec<f64>], k: usize) -> f64 {
    let leg = |from: &[f64], to: &[f64]| -> Vec<f64> {
        let length = distance(from, to);
        to.iter().zip(from).map(|(b, a)| (b - a) / length).collect()
    };
    let (before, after) = (
        leg(&waypoints[k - 1], &waypoints[k]),
        leg(&waypoints[k], &waypoints[k + 1]),
    );
    before.iter().zip(&after).map(|(a, b)| a * b).sum()
}

/// The turn, in degrees, of the corner at waypoint `k` of `waypoints`.
fn turn_deg(waypoints: &[Vec<f64>], k: usize) -> f64 {
    corner_cos(waypoints, k)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

/// Whether TOTG runs a corner of `case` straight on.
fn has_a_straight_on_corner(case: &PathCase) -> bool {
    (1..case.waypoints.len().saturating_sub(1))
        .any(|k| corner_cos(&case.waypoints, k) > TOTG_STRAIGHT_ON_COS)
}

/// The waypoint of `case`'s sharpest corner, and its turn in degrees; `None` for a
/// path with no corner.
fn sharpest_corner(case: &PathCase) -> Option<(usize, f64)> {
    (1..case.waypoints.len().saturating_sub(1))
        .map(|k| (k, turn_deg(&case.waypoints, k)))
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
}

/// Whether `path` comes to rest at `turn`. Along straight legs into and out of `turn`,
/// its distance from `turn` falls until it reaches it and grows after, so its sample
/// nearest `turn`, over [`SPEED_STEPS`] even steps of its time, lies within a step of
/// that instant, and the samples beside it within two. From rest there, at an
/// acceleration of at most `a`, the norm of `max_acceleration`, a path moves at most
/// `2·a·step²` in two steps, and the three samples lie that near `turn`; reaching `turn`
/// at a speed `v`, or stopping short of it, the path lies about `v·step` from it, or
/// farther, at one of them.
fn comes_to_rest_at(path: &TimedPath, turn: &[f64], max_acceleration: &[f64]) -> bool {
    let away: Vec<f64> = (0..=SPEED_STEPS)
        .map(|k| {
            distance(
                &path.at_fraction(f64::from(k) / f64::from(SPEED_STEPS)),
                turn,
            )
        })
        .collect();
    let nearest = (0..away.len())
        .min_by(|&i, &j| away[i].total_cmp(&away[j]))
        .expect("the path has samples");
    let a = max_acceleration.iter().map(|x| x * x).sum::<f64>().sqrt();
    let step = path.duration_s() / f64::from(SPEED_STEPS);
    let reach = 2.0 * a * step * step + POINT_TOLERANCE;
    (nearest.saturating_sub(1)..=nearest + 1)
        .filter_map(|k| away.get(k))
        .all(|&d| d <= reach)
}

/// Every path timing scenario of the harness.
fn scenarios() -> Vec<Cases> {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a path timing scenario"
    );
    scenarios
}

// On every path TOTG blends each corner of, the rounded path has TOTG's length, and its
// point at each fraction of that length is TOTG's. The cornered paths compared span the
// scenario's deviations.
#[test]
fn the_rounded_path_is_totgs_blended_path() {
    for cases in scenarios() {
        let mut deviations = Vec::new();
        for case in &cases.paths {
            let Totg::Timed {
                length, configs, ..
            } = &case.totg
            else {
                continue;
            };
            if has_a_straight_on_corner(case) {
                continue;
            }
            let path = rounded(case);
            assert!(
                (path.length() - length).abs() <= POINT_TOLERANCE * length.max(1.0),
                "{} {}: length {} against TOTG's {length}",
                cases.name,
                case.name,
                path.length()
            );
            for (fraction, config) in case.fractions.iter().zip(configs) {
                let ours = path.at(fraction * length);
                let off = distance(&ours, config);
                assert!(
                    off <= POINT_TOLERANCE,
                    "{} {}: {off} off TOTG at {fraction} of the length",
                    cases.name,
                    case.name
                );
            }
            if case.waypoints.len() > 2 {
                deviations.push(case.max_deviation);
            }
        }
        deviations.sort_by(f64::total_cmp);
        deviations.dedup();
        assert_eq!(
            deviations, SCENARIO_DEVIATIONS,
            "{}: the deviations of the cornered paths compared",
            cases.name
        );
    }
}

// TOTG runs a corner that turns by less than its straight-on threshold straight on
// through its waypoint, so its path is longer than the rounded path, which rounds the
// corner on an arc within the path's deviation; at each fraction of TOTG's length, the
// rounded path lies within that deviation of TOTG's point.
#[test]
fn a_corner_totg_runs_straight_on_is_rounded_within_the_deviation() {
    for cases in scenarios() {
        let mut compared = 0;
        for case in cases
            .paths
            .iter()
            .filter(|case| has_a_straight_on_corner(case))
        {
            let at = format!("{} {}", cases.name, case.name);
            let Totg::Timed {
                length, configs, ..
            } = &case.totg
            else {
                panic!("{at}: TOTG refuses a path it runs straight on at a corner");
            };
            let path = rounded(case);
            assert!(
                length - path.length() > POINT_TOLERANCE,
                "{at}: TOTG's length {length} against the rounded path's {}",
                path.length()
            );
            for (fraction, config) in case.fractions.iter().zip(configs) {
                let off = distance(&path.at(fraction * length), config);
                assert!(
                    off <= case.max_deviation,
                    "{at}: {off} off TOTG at {fraction} of the length, past the deviation"
                );
            }
            compared += 1;
        }
        assert!(
            compared > 0,
            "{}: no path with a corner TOTG runs straight on",
            cases.name
        );
    }
}

// The timing is no faster than TOTG's on any path, to a part in 1e9, and on a straight
// run no slower than the grid's slack over it.
#[test]
fn the_timing_is_no_faster_than_totg_and_matches_it_on_a_straight_run() {
    for cases in scenarios() {
        let mut straight_runs = 0;
        for case in &cases.paths {
            let Totg::Timed { duration_s, .. } = case.totg else {
                continue;
            };
            let ours = timed(case).duration_s();
            assert!(
                ours >= duration_s * (1.0 - DURATION_ROUNDING),
                "{} {}: {ours} s against TOTG's {duration_s} s",
                cases.name,
                case.name
            );
            if case.waypoints.len() == 2 {
                assert!(
                    ours <= duration_s * STRAIGHT_RUN_SLACK,
                    "{} {}: {ours} s against TOTG's {duration_s} s on a straight run",
                    cases.name,
                    case.name
                );
                straight_runs += 1;
            }
        }
        assert!(
            straight_runs > 0,
            "{}: no straight run compared",
            cases.name
        );
    }
}

// A path TOTG refuses because it turns back on itself comes to rest at the waypoint of
// its sharpest corner and turns back there: it reaches the waypoint at speed 0, and none
// of its points lies farther from it than the start.
#[test]
fn a_path_totg_refuses_for_turning_back_comes_to_rest_and_turns_back() {
    for cases in scenarios() {
        let refused: Vec<&PathCase> = cases
            .paths
            .iter()
            .filter(|case| case.totg == Totg::Refused)
            .collect();
        assert!(
            !refused.is_empty(),
            "{}: the scenario holds a path that turns back",
            cases.name
        );
        for case in refused {
            let (k, _) = sharpest_corner(case).expect("a refused path has a corner");
            let turn = &case.waypoints[k];
            assert!(
                comes_to_rest_at(&timed(case), turn, &case.max_acceleration),
                "{} {}: reaches its turn at speed, or short of it",
                cases.name,
                case.name
            );
            let path = rounded(case);
            assert!(
                path.arcs_every(PositiveFinite::parse(1e-3).expect("positive"))
                    .iter()
                    .all(|&s| {
                        distance(&path.at(s), turn) <= distance(&case.waypoints[0], turn) + 1e-9
                    }),
                "{} {}: the path runs past the waypoint it turns back at",
                cases.name,
                case.name
            );
        }
    }
}

// TOTG refuses a path whose sharpest corner turns farther than that of every path it
// blends, and the scenario brackets that turning-back threshold within
// TURNING_BACK_BRACKET_DEG. control_core rounds each corner TOTG blends, as the parity
// of the points shows, and stops at each TOTG refuses, as the test above shows, so its
// threshold lies in the same bracket.
#[test]
fn totgs_turning_back_threshold_lies_between_the_scenarios_turns() {
    for cases in scenarios() {
        let sharpest = |refused: bool| {
            cases
                .paths
                .iter()
                .filter(move |case| (case.totg == Totg::Refused) == refused)
                .filter_map(sharpest_corner)
                .map(|(_, turn)| turn)
        };
        let blended = sharpest(false).fold(f64::NEG_INFINITY, f64::max);
        let refused = sharpest(true).fold(f64::INFINITY, f64::min);
        assert!(
            blended < refused,
            "{}: TOTG blends a turn of {blended} degrees and refuses one of {refused}",
            cases.name
        );
        assert!(
            refused - blended <= TURNING_BACK_BRACKET_DEG,
            "{}: the turns {blended} and {refused} degrees bracket TOTG's threshold more \
             widely than {TURNING_BACK_BRACKET_DEG} degrees",
            cases.name
        );
    }
}

// Each scenario blends paths at the deviation of execute_joint_path, so the parity above
// covers the corners that move runs.
#[test]
fn each_scenario_blends_at_the_joint_path_deviation() {
    for cases in scenarios() {
        let blended = cases
            .paths
            .iter()
            .filter(|case| {
                case.max_deviation == JOINT_PATH_CORNER_DEVIATION
                    && matches!(case.totg, Totg::Timed { .. })
                    && case.waypoints.len() > 2
            })
            .count();
        assert!(
            blended > 0,
            "{}: no cornered path at the joint path's deviation",
            cases.name
        );
    }
}
