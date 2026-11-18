//! The metrics that score a planar trajectory, a localization and a global localization
//! against ground truth, and summarize them over seeds.

/// A pose on the floor: a position (m) and a heading (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// A rigid motion of the plane: a turn about the origin, then a translation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarMotion {
    pub turn: f64,
    pub translation: [f64; 2],
}

impl PlanarMotion {
    /// `pose` moved by the motion.
    pub fn apply(&self, pose: PlanarPose) -> PlanarPose {
        let (sin, cos) = self.turn.sin_cos();
        PlanarPose {
            x: cos * pose.x - sin * pose.y + self.translation[0],
            y: sin * pose.x + cos * pose.y + self.translation[1],
            yaw: wrapped(pose.yaw + self.turn),
        }
    }
}

/// `angle` wrapped into [-pi, pi].
fn wrapped(angle: f64) -> f64 {
    let turns = (angle / std::f64::consts::TAU).round();
    angle - turns * std::f64::consts::TAU
}

/// The rigid motion that moves the positions of `from` closest to those of `to`, pair by
/// pair, in the least-squares sense: the turn that aligns the centered positions, and the
/// translation that then takes the centroid of `from` to that of `to`. None when there are
/// fewer than two pairs or the positions of either side all coincide.
pub fn best_alignment(from: &[PlanarPose], to: &[PlanarPose]) -> Option<PlanarMotion> {
    assert_eq!(from.len(), to.len(), "the alignment pairs two equal sets");
    if from.len() < 2 {
        return None;
    }
    let count = from.len() as f64;
    let centroid = |poses: &[PlanarPose]| {
        let (x, y) = poses
            .iter()
            .fold((0.0, 0.0), |(x, y), pose| (x + pose.x, y + pose.y));
        [x / count, y / count]
    };
    let (a, b) = (centroid(from), centroid(to));
    let (dot, cross) = from
        .iter()
        .zip(to)
        .fold((0.0, 0.0), |(dot, cross), (p, q)| {
            let (px, py) = (p.x - a[0], p.y - a[1]);
            let (qx, qy) = (q.x - b[0], q.y - b[1]);
            (dot + px * qx + py * qy, cross + px * qy - py * qx)
        });
    if dot == 0.0 && cross == 0.0 {
        return None;
    }
    let turn = cross.atan2(dot);
    let (sin, cos) = turn.sin_cos();
    Some(PlanarMotion {
        turn,
        translation: [
            b[0] - (cos * a[0] - sin * a[1]),
            b[1] - (sin * a[0] + cos * a[1]),
        ],
    })
}

/// The distance (m) between the positions of two poses, and the magnitude of the turn
/// (rad) between their headings.
pub fn pose_error(estimate: PlanarPose, truth: PlanarPose) -> (f64, f64) {
    (
        (estimate.x - truth.x).hypot(estimate.y - truth.y),
        wrapped(estimate.yaw - truth.yaw).abs(),
    )
}

/// The errors of a set of estimates against their ground truth poses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Errors {
    /// The root mean square of the position errors (m).
    pub position_rmse_m: f64,
    /// The largest position error (m).
    pub position_max_m: f64,
    /// The root mean square of the heading errors (rad).
    pub yaw_rmse_rad: f64,
}

/// The errors of `estimates` against `truths`, pair by pair. Panics on no pair.
pub fn errors(estimates: &[PlanarPose], truths: &[PlanarPose]) -> Errors {
    assert_eq!(
        estimates.len(),
        truths.len(),
        "each estimate has its ground truth"
    );
    assert!(!estimates.is_empty(), "the errors of no estimate");
    let pairs: Vec<(f64, f64)> = estimates
        .iter()
        .zip(truths)
        .map(|(&estimate, &truth)| pose_error(estimate, truth))
        .collect();
    let count = pairs.len() as f64;
    let mean_square = |select: fn(&(f64, f64)) -> f64| {
        pairs.iter().map(|pair| select(pair).powi(2)).sum::<f64>() / count
    };
    Errors {
        position_rmse_m: mean_square(|(position, _)| *position).sqrt(),
        position_max_m: pairs
            .iter()
            .map(|(position, _)| *position)
            .fold(0.0, f64::max),
        yaw_rmse_rad: mean_square(|(_, yaw)| *yaw).sqrt(),
    }
}

/// The trajectory error of a map: the errors of the poses of its scans after the rigid
/// motion that best aligns the ground truth with them. None when no alignment exists.
pub fn trajectory_errors(estimates: &[PlanarPose], truths: &[PlanarPose]) -> Option<Errors> {
    let motion = best_alignment(truths, estimates)?;
    let aligned: Vec<PlanarPose> = truths.iter().map(|&truth| motion.apply(truth)).collect();
    Some(errors(estimates, &aligned))
}

/// The standard deviations of a pose estimate: of the position along its least certain
/// direction (m), and of the heading (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deviations {
    pub position_std_m: f64,
    pub yaw_std_rad: f64,
}

/// The largest standard deviations of an estimate that count it localized.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LocalizationLimits {
    pub position_std_m: f64,
    pub yaw_std_rad: f64,
}

impl LocalizationLimits {
    /// Whether `deviations` lie within these limits.
    pub fn hold(&self, deviations: Deviations) -> bool {
        deviations.position_std_m <= self.position_std_m
            && deviations.yaw_std_rad <= self.yaw_std_rad
    }
}

/// One estimate of a global localization: when it came (s from the run's first scan),
/// the pose, whether the filter's convergence test held, and the standard deviations of
/// the filter's covariance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalEstimate {
    pub time_s: f64,
    pub pose: PlanarPose,
    pub converged: bool,
    pub deviations: Deviations,
}

/// How near the ground truth an estimate lies to count as localized.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LocalizedWithin {
    pub position_m: f64,
    pub yaw_rad: f64,
}

impl LocalizedWithin {
    /// Whether `pose` lies within this bound of `truth`.
    pub fn holds(&self, pose: PlanarPose, truth: PlanarPose) -> bool {
        let (position, yaw) = pose_error(pose, truth);
        position <= self.position_m && yaw <= self.yaw_rad
    }
}

/// The outcome of a global localization run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalOutcome {
    pub estimates: usize,
    /// The estimates at which the filter's convergence test held.
    pub converged_estimates: usize,
    /// The estimates whose deviations lay within the search limits.
    pub within_limits_estimates: usize,
    /// The smallest position and heading deviations of the run's estimates, each over
    /// every estimate.
    pub least_deviations: Deviations,
    /// When the run localized for good by the convergence test (s from its first scan):
    /// the first estimate from which every estimate converged and lay within the bound.
    /// None when the last estimate did not.
    pub localized_after_s: Option<f64>,
    /// When the run localized for good by the search limits (s from its first scan): the
    /// first estimate from which every estimate's deviations lay within the limits and its
    /// pose within the bound. None when the last estimate's did not.
    pub localized_by_limits_after_s: Option<f64>,
    /// The position error (m) and the heading error (rad) of the last estimate.
    pub final_position_error_m: f64,
    pub final_yaw_error_rad: f64,
}

/// The index of the first of a run's estimates from which every estimate `held` and lay
/// `near` the truth, as a run counts as localized for good; none when the last did not.
pub fn localized_for_good_from(held: &[bool], near: &[bool]) -> Option<usize> {
    assert_eq!(
        held.len(),
        near.len(),
        "a flag of each kind for each estimate"
    );
    let first_of_last_run = held
        .iter()
        .zip(near)
        .rposition(|(&held, &near)| !(held && near))
        .map_or(0, |last_miss| last_miss + 1);
    (first_of_last_run < held.len()).then_some(first_of_last_run)
}

/// The outcome of the estimates `estimates` against `truths`, pair by pair, an estimate
/// localized when its pose lies `within` the truth and, by the convergence test, the test
/// held or, by the search limits, its deviations lay within `limits`. Panics on no
/// estimate.
pub fn global_outcome(
    estimates: &[GlobalEstimate],
    truths: &[PlanarPose],
    within: LocalizedWithin,
    limits: LocalizationLimits,
) -> GlobalOutcome {
    assert_eq!(
        estimates.len(),
        truths.len(),
        "each estimate has its ground truth"
    );
    let (last, &last_truth) = estimates
        .last()
        .zip(truths.last())
        .expect("the outcome of no estimate");
    let near: Vec<bool> = estimates
        .iter()
        .zip(truths)
        .map(|(estimate, &truth)| within.holds(estimate.pose, truth))
        .collect();
    let converged: Vec<bool> = estimates.iter().map(|e| e.converged).collect();
    let within_limits: Vec<bool> = estimates
        .iter()
        .map(|e| limits.hold(e.deviations))
        .collect();
    let localized_for_good_after_s =
        |held: &[bool]| localized_for_good_from(held, &near).map(|first| estimates[first].time_s);
    let count = |held: &[bool]| held.iter().filter(|&&held| held).count();
    let least = |deviation: fn(&Deviations) -> f64| {
        estimates
            .iter()
            .map(|e| deviation(&e.deviations))
            .fold(f64::INFINITY, f64::min)
    };
    let (final_position_error_m, final_yaw_error_rad) = pose_error(last.pose, last_truth);
    GlobalOutcome {
        estimates: estimates.len(),
        converged_estimates: count(&converged),
        within_limits_estimates: count(&within_limits),
        least_deviations: Deviations {
            position_std_m: least(|d| d.position_std_m),
            yaw_std_rad: least(|d| d.yaw_std_rad),
        },
        localized_after_s: localized_for_good_after_s(&converged),
        localized_by_limits_after_s: localized_for_good_after_s(&within_limits),
        final_position_error_m,
        final_yaw_error_rad,
    }
}

/// The mean of values over seeds, with its standard error: the sample standard deviation
/// over the square root of the count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mean {
    pub mean: f64,
    pub standard_error: f64,
}

/// The mean of `values` with its standard error. None for no value; the standard error of
/// one value is 0.
pub fn mean(values: &[f64]) -> Option<Mean> {
    if values.is_empty() {
        return None;
    }
    let count = values.len() as f64;
    let mean = values.iter().sum::<f64>() / count;
    let standard_error = if values.len() < 2 {
        0.0
    } else {
        let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (count - 1.0);
        (variance / count).sqrt()
    };
    Some(Mean {
        mean,
        standard_error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn a_run_is_localized_for_good_from_the_estimate_after_its_last_miss() {
        let every = [true; 4];
        assert_eq!(localized_for_good_from(&every, &every), Some(0));
        assert_eq!(
            localized_for_good_from(&[true, false, true, true], &every),
            Some(2)
        );
        assert_eq!(
            localized_for_good_from(&every, &[true, true, true, false]),
            None
        );
        assert_eq!(localized_for_good_from(&[], &[]), None);
    }

    fn pose(x: f64, y: f64, yaw: f64) -> PlanarPose {
        PlanarPose { x, y, yaw }
    }

    #[test]
    fn the_best_alignment_recovers_a_rigid_motion() {
        let motion = PlanarMotion {
            turn: 0.7,
            translation: [1.5, -2.0],
        };
        let from = [
            pose(0.0, 0.0, 0.1),
            pose(2.0, 0.5, 0.2),
            pose(-1.0, 3.0, -0.4),
        ];
        let to: Vec<PlanarPose> = from.iter().map(|&p| motion.apply(p)).collect();
        let found = best_alignment(&from, &to).expect("an alignment");
        assert!((found.turn - motion.turn).abs() < 1e-12);
        assert!((found.translation[0] - 1.5).abs() < 1e-12);
        assert!((found.translation[1] + 2.0).abs() < 1e-12);
        let errors = trajectory_errors(&to, &from).expect("an alignment");
        assert!(errors.position_rmse_m < 1e-12 && errors.yaw_rmse_rad < 1e-12);
    }

    #[test]
    fn coincident_positions_have_no_alignment() {
        let same = [pose(1.0, 1.0, 0.0), pose(1.0, 1.0, 1.0)];
        assert_eq!(best_alignment(&same, &same), None);
        assert_eq!(best_alignment(&same[..1], &same[..1]), None);
    }

    #[test]
    fn errors_are_the_root_mean_square_and_the_largest() {
        let truths = [pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0)];
        let estimates = [pose(3.0, 4.0, 0.1), pose(0.0, 0.0, -0.3)];
        let errors = errors(&estimates, &truths);
        assert!((errors.position_rmse_m - 12.5f64.sqrt()).abs() < 1e-12);
        assert_eq!(errors.position_max_m, 5.0);
        assert!((errors.yaw_rmse_rad - 0.05f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn a_heading_error_wraps_across_pi() {
        let (_, yaw) = pose_error(pose(0.0, 0.0, PI - 0.1), pose(0.0, 0.0, -PI + 0.1));
        assert!((yaw - 0.2).abs() < 1e-12);
        assert!((wrapped(3.0 * FRAC_PI_2) + FRAC_PI_2).abs() < 1e-12);
    }

    const WITHIN: LocalizedWithin = LocalizedWithin {
        position_m: 0.5,
        yaw_rad: 0.3,
    };

    const LIMITS: LocalizationLimits = LocalizationLimits {
        position_std_m: 0.3,
        yaw_std_rad: 0.3,
    };

    const NARROW: Deviations = Deviations {
        position_std_m: 0.1,
        yaw_std_rad: 0.2,
    };

    const WIDE: Deviations = Deviations {
        position_std_m: 2.0,
        yaw_std_rad: 1.5,
    };

    fn estimate(time_s: f64, x: f64, converged: bool, deviations: Deviations) -> GlobalEstimate {
        GlobalEstimate {
            time_s,
            pose: pose(x, 0.0, 0.0),
            converged,
            deviations,
        }
    }

    #[test]
    fn a_run_localizes_from_the_first_estimate_after_its_last_miss() {
        let truths = [pose(0.0, 0.0, 0.0); 5];
        let estimates = [
            estimate(0.0, 0.1, true, NARROW),
            estimate(1.0, 3.0, true, NARROW),
            estimate(2.0, 0.1, false, NARROW),
            estimate(3.0, 0.2, true, NARROW),
            estimate(4.0, 0.1, true, NARROW),
        ];
        let outcome = global_outcome(&estimates, &truths, WITHIN, LIMITS);
        assert_eq!(outcome.localized_after_s, Some(3.0));
        assert_eq!(outcome.localized_by_limits_after_s, Some(2.0));
        assert!((outcome.final_position_error_m - 0.1).abs() < 1e-12);
    }

    #[test]
    fn a_run_converged_far_from_the_truth_is_not_localized() {
        let truths = [pose(0.0, 0.0, 0.0); 2];
        let estimates = [
            estimate(0.0, 0.1, true, NARROW),
            estimate(1.0, 2.0, true, NARROW),
        ];
        let outcome = global_outcome(&estimates, &truths, WITHIN, LIMITS);
        assert_eq!(outcome.localized_after_s, None);
        assert_eq!(outcome.localized_by_limits_after_s, None);
        assert_eq!(outcome.final_position_error_m, 2.0);
    }

    #[test]
    fn the_counts_and_the_least_deviations_cover_every_estimate() {
        let truths = [pose(0.0, 0.0, 0.0); 3];
        let estimates = [
            estimate(0.0, 4.0, true, NARROW),
            estimate(1.0, 4.0, false, WIDE),
            estimate(
                2.0,
                0.1,
                false,
                Deviations {
                    position_std_m: 0.5,
                    yaw_std_rad: 0.05,
                },
            ),
        ];
        let outcome = global_outcome(&estimates, &truths, WITHIN, LIMITS);
        assert_eq!(
            (
                outcome.estimates,
                outcome.converged_estimates,
                outcome.within_limits_estimates
            ),
            (3, 1, 1)
        );
        assert_eq!(
            outcome.least_deviations,
            Deviations {
                position_std_m: 0.1,
                yaw_std_rad: 0.05,
            }
        );
        assert_eq!(outcome.localized_after_s, None);
        assert_eq!(outcome.localized_by_limits_after_s, None);
    }

    #[test]
    fn the_limits_hold_up_to_and_at_each_deviation() {
        let at = |position_std_m, yaw_std_rad| {
            LIMITS.hold(Deviations {
                position_std_m,
                yaw_std_rad,
            })
        };
        assert!(at(0.3, 0.3) && at(0.0, 0.0));
        assert!(!at(0.31, 0.1) && !at(0.1, 0.31));
    }

    #[test]
    fn the_standard_error_is_the_sample_deviation_over_the_root_of_the_count() {
        let summary = mean(&[1.0, 2.0, 3.0, 4.0]).expect("a mean");
        assert_eq!(summary.mean, 2.5);
        assert!((summary.standard_error - (5.0f64 / 3.0 / 4.0).sqrt()).abs() < 1e-15);
        assert_eq!(mean(&[]), None);
        assert_eq!(mean(&[2.0]).map(|m| m.standard_error), Some(0.0));
    }
}
