// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! The pose of the base in a map, with the standard deviations that `room_map` judges
//! localization by, and the limits within which they hold the base localized.

use crate::geometry::Matrix3;
use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;

/// The standard deviations of a pose estimate: of its position along its least certain
/// direction (m), and of its heading (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deviations {
    pub position_std_m: f64,
    pub yaw_std_rad: f64,
}

impl Deviations {
    /// The deviations of a 3 x 3 covariance of x, y and yaw: the square root of the
    /// larger eigenvalue of its 2 x 2 position block, and the square root of its yaw
    /// variance, as slam_toolbox's pose and AMCL's `amcl_pose` carry a covariance. A
    /// variance that rounding leaves below 0, as AMCL's of particles at one pose, reads as
    /// 0.
    pub(crate) fn of_covariance(covariance: &Matrix3) -> Self {
        let [[xx, xy, _], [_, yy, _], [_, _, yaw]] = covariance.0;
        Self {
            position_std_m: deviation_of(larger_eigenvalue(xx, xy, yy)),
            yaw_std_rad: deviation_of(yaw),
        }
    }

    /// Whether both deviations lie within `limits`.
    pub fn within(&self, limits: &LocalizationLimits) -> bool {
        self.position_std_m <= limits.position_std_m.get()
            && self.yaw_std_rad <= limits.yaw_std_rad.get()
    }
}

/// The square root of `variance`, 0 for a variance below 0; NaN for NaN.
fn deviation_of(variance: f64) -> f64 {
    if variance < 0.0 { 0.0 } else { variance.sqrt() }
}

/// The larger eigenvalue of the symmetric matrix [[xx, xy], [xy, yy]], from its trace and
/// the root of its discriminant, which keeps it exact for a diagonal matrix.
fn larger_eigenvalue(xx: f64, xy: f64, yy: f64) -> f64 {
    let half_trace = 0.5 * (xx + yy);
    let half_difference = 0.5 * (xx - yy);
    half_trace + half_difference.hypot(xy)
}

/// The largest standard deviations with which an estimate holds the base localized, as a
/// navigator's launch sets them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalizationLimits {
    pub position_std_m: PositiveFinite,
    pub yaw_std_rad: PositiveFinite,
}

/// Where the robot frame stands in a map at a scan: the estimate, its deviations, and
/// whether the base is localized, as `room_map`'s `map_poses` carries it. The heading lies
/// in (-pi, pi].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapPose {
    /// The time of the scan this pose is estimated from (ns).
    pub stamp_ns: i64,
    pub pose: PlanarPose,
    pub deviations: Deviations,
    pub localized: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    #[hegel::test]
    fn the_position_deviation_is_the_root_of_the_larger_eigenvalue(tc: TestCase) {
        // An ellipse of deviations `larger` and `smaller`, its larger axis at `angle`.
        let larger: f64 = tc.draw(gs::floats().min_value(1e-3).max_value(10.0));
        let smaller = larger * tc.draw(gs::floats::<f64>().min_value(0.0).max_value(1.0));
        let angle: f64 = tc.draw(gs::floats().min_value(-3.2).max_value(3.2));
        let yaw: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let (sin, cos) = angle.sin_cos();
        let (a, b) = (larger * larger, smaller * smaller);
        let covariance = Matrix3([
            [a * cos * cos + b * sin * sin, (a - b) * sin * cos, 0.0],
            [(a - b) * sin * cos, a * sin * sin + b * cos * cos, 0.0],
            [0.0, 0.0, yaw * yaw],
        ]);
        let deviations = Deviations::of_covariance(&covariance);
        assert!(
            (deviations.position_std_m - larger).abs() <= 1e-9 * larger,
            "{deviations:?} of an ellipse of {larger} and {smaller} m at {angle} rad"
        );
        assert!((deviations.yaw_std_rad - yaw).abs() <= 1e-12);
    }

    #[test]
    fn a_diagonal_covariance_gives_the_root_of_its_larger_position_variance() {
        let covariance = Matrix3([[0.04, 0.0, 0.0], [0.0, 0.09, 0.0], [0.0, 0.0, 0.01]]);
        let deviations = Deviations::of_covariance(&covariance);
        assert_eq!(deviations.position_std_m, 0.09f64.sqrt());
        assert_eq!(deviations.yaw_std_rad, 0.1);
    }

    #[test]
    fn a_variance_that_rounding_leaves_below_zero_reads_as_no_deviation() {
        let covariance = Matrix3([[-1e-17, 0.0, 0.0], [0.0, -2e-17, 0.0], [0.0, 0.0, -4e-16]]);
        let deviations = Deviations::of_covariance(&covariance);
        assert_eq!(
            (deviations.position_std_m, deviations.yaw_std_rad),
            (0.0, 0.0)
        );
        let unknown = Matrix3([[f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, f64::NAN]]);
        let deviations = Deviations::of_covariance(&unknown);
        assert!(deviations.position_std_m.is_nan() && deviations.yaw_std_rad.is_nan());
    }

    #[test]
    fn deviations_lie_within_limits_while_each_is_at_most_its_own_limit() {
        let limit = |value: f64| PositiveFinite::parse(value).expect("a limit");
        let limits = LocalizationLimits {
            position_std_m: limit(0.1),
            yaw_std_rad: limit(0.2),
        };
        let deviations = |position_std_m: f64, yaw_std_rad: f64| Deviations {
            position_std_m,
            yaw_std_rad,
        };
        assert!(deviations(0.1, 0.2).within(&limits));
        assert!(!deviations(0.11, 0.2).within(&limits));
        assert!(!deviations(0.1, 0.21).within(&limits));
    }
}
