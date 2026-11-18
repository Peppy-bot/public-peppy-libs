// Software License Agreement (BSD License)
//
//  Copyright (c) 2010-2012, Willow Garage, Inc.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// This file translates into Rust pointcloud_to_laserscan 2.1.1's projection of a cloud,
// `PointCloudToLaserScanNode::cloudCallback` of src/pointcloud_to_laserscan_node.cpp (Copyright
// (c) 2010-2012, Willow Garage, Inc.).
//
// Rust translation by Peppy, 2026-11-13.
//
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

//! Points to a 2-D scan, as pointcloud_to_laserscan 2.1.1 projects a cloud with no
//! target frame (`PointCloudToLaserScanNode::cloudCallback`,
//! src/pointcloud_to_laserscan_node.cpp, with `use_inf` true): each point inside the
//! height band and the range limits falls in the bin of its bearing, and each bin holds
//! the range of its nearest point, or +inf when no point falls in it, which the scan that
//! `lidar_2d`'s `scan_stream` carries gives as its no-return mark, 10000.
//!
//! The arithmetic is the callback's: the points are single precision, the range and the
//! bearing of a point are computed in single precision (the callback calls `hypotf`
//! and `atan2f`) and compared in double precision with the band and the limits, which
//! are double, and with the angles of the scan, which are single. Here each of the two
//! is computed in double precision and rounded to single precision.

use camera_core::laser_scan::{LaserScan, NO_RETURN_MARK_M};
use thiserror::Error;

/// pointcloud_to_laserscan's parameters, by the names of its node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaserProjectionParameters {
    /// The height band (m) on the z axis: a point below `min_height` or above
    /// `max_height` is left out.
    pub min_height: f64,
    pub max_height: f64,
    /// The bearings (rad) of the scan, which are kept in single precision.
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    /// The range limits (m): a point nearer than `range_min` or farther than
    /// `range_max` on the floor plane is left out.
    pub range_min: f64,
    pub range_max: f64,
}

/// Why parameters give no projection.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum LaserProjectionError {
    #[error("{name} is {value}, and every parameter of the projection is finite")]
    NotFinite { name: &'static str, value: f64 },
    #[error("angle_increment is {0} in single precision, and it must be above 0")]
    IncrementNotPositive(f32),
    #[error("angle_max ({angle_max}) is not above angle_min ({angle_min}) in single precision")]
    EmptyFan { angle_min: f32, angle_max: f32 },
    #[error(
        "a point at angle_max falls in bin {last_bin} of a scan of {bins} bins: set \
         angle_max at least one increment below angle_min plus {bins} increments"
    )]
    LastBinOutsideScan { last_bin: usize, bins: usize },
    #[error(
        "range_min ({range_min}) is below 0, or range_max ({range_max}) is not above it or \
         not below lidar_2d's no-return mark, 10000, in the single precision of a scan: give \
         a range_max above range_min and below 10000"
    )]
    RangeLimits { range_min: f64, range_max: f64 },
}

/// A scan as `lidar_2d`'s `scan_stream` carries it, every ray at the scan's timestamp: the
/// angle of its first ray (rad), the angle between one ray and the next (rad), its range
/// limits (m) and the range of each ray, a ray that met nothing at the no-return mark, each
/// in single precision.
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectedScan {
    min_angle_rad: f32,
    angle_increment_rad: f32,
    min_range_m: f32,
    max_range_m: f32,
    ranges: Vec<f32>,
}

impl ProjectedScan {
    /// The angle of the first ray (rad).
    pub fn min_angle_rad(&self) -> f32 {
        self.min_angle_rad
    }

    /// The angle between one ray and the next (rad).
    pub fn angle_increment_rad(&self) -> f32 {
        self.angle_increment_rad
    }

    /// The nearest range a ray holds (m).
    pub fn min_range_m(&self) -> f32 {
        self.min_range_m
    }

    /// The farthest range of the scan (m), at and beyond which a range reads as no return.
    pub fn max_range_m(&self) -> f32 {
        self.max_range_m
    }

    /// The range of each ray (m), the no-return mark for a ray that met nothing.
    pub fn ranges(&self) -> &[f32] {
        &self.ranges
    }

    /// The scan as `camera_core` reads it.
    pub fn laser_scan(&self) -> LaserScan {
        LaserScan::parse(
            self.min_angle_rad,
            self.angle_increment_rad,
            0.0,
            self.min_range_m,
            self.max_range_m,
            &self.ranges,
        )
        .expect("a projection's angles, limits and ranges make a scan")
    }
}

/// A projection of points to a scan, from parameters whose every bearing up to
/// `angle_max` falls in a bin of the scan.
#[derive(Debug, Clone, PartialEq)]
pub struct LaserProjection {
    min_height: f64,
    max_height: f64,
    angle_min: f32,
    angle_max: f32,
    angle_increment: f32,
    range_min: f64,
    range_max: f64,
    bins: usize,
}

impl LaserProjection {
    pub fn new(parameters: LaserProjectionParameters) -> Result<Self, LaserProjectionError> {
        let LaserProjectionParameters {
            min_height,
            max_height,
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
        } = parameters;
        [
            ("min_height", min_height),
            ("max_height", max_height),
            ("angle_min", angle_min),
            ("angle_max", angle_max),
            ("angle_increment", angle_increment),
            ("range_min", range_min),
            ("range_max", range_max),
        ]
        .into_iter()
        .find(|(_, value)| !value.is_finite())
        .map_or(Ok(()), |(name, value)| {
            Err(LaserProjectionError::NotFinite { name, value })
        })?;
        // The scan carries the limits in single precision, a range_max above range_min and
        // below the no-return mark, as lidar_2d's scan_stream does.
        let (single_min, single_max) = (range_min as f32, range_max as f32);
        if !(single_max < NO_RETURN_MARK_M && single_min >= 0.0 && single_max > single_min) {
            return Err(LaserProjectionError::RangeLimits {
                range_min,
                range_max,
            });
        }
        let (angle_min, angle_max, angle_increment) =
            (angle_min as f32, angle_max as f32, angle_increment as f32);
        if angle_increment <= 0.0 {
            return Err(LaserProjectionError::IncrementNotPositive(angle_increment));
        }
        if angle_max <= angle_min {
            return Err(LaserProjectionError::EmptyFan {
                angle_min,
                angle_max,
            });
        }
        // The callback sizes the scan by the ceiling of the span over the increment, in
        // single precision, and puts a point in the bin of its bearing's span over the
        // increment, in double precision, truncated.
        let bins = ((angle_max - angle_min) / angle_increment).ceil() as usize;
        let last_bin = bin_of(f64::from(angle_max), angle_min, angle_increment);
        if last_bin >= bins {
            return Err(LaserProjectionError::LastBinOutsideScan { last_bin, bins });
        }
        Ok(Self {
            min_height,
            max_height,
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
            bins,
        })
    }

    /// The number of bins of every scan the projection gives.
    pub fn bins(&self) -> usize {
        self.bins
    }

    /// The scan of `points`, each `[x, y, z]` in the scan's frame, in their order, every ray
    /// at the scan's timestamp: a point replaces the range of its bin when it is nearer, and
    /// a bin with no point holds the no-return mark.
    pub fn project(&self, points: &[[f32; 3]]) -> ProjectedScan {
        let mut ranges = vec![NO_RETURN_MARK_M; self.bins];
        for &[x, y, z] in points {
            let Some((bin, range)) = self.bin_and_range(x, y, z) else {
                continue;
            };
            if range < f64::from(ranges[bin]) {
                ranges[bin] = range as f32;
            }
        }
        ProjectedScan {
            min_angle_rad: self.angle_min,
            angle_increment_rad: self.angle_increment,
            min_range_m: self.range_min as f32,
            max_range_m: self.range_max as f32,
            ranges,
        }
    }

    /// The bin of a point and its range on the floor plane, widened to double precision,
    /// or none for a point that the callback leaves out.
    fn bin_and_range(&self, x: f32, y: f32, z: f32) -> Option<(usize, f64)> {
        if x.is_nan() || y.is_nan() || z.is_nan() {
            return None;
        }
        let height = f64::from(z);
        if height > self.max_height || height < self.min_height {
            return None;
        }
        let range = f64::from(single_precision_hypot(x, y));
        if range < self.range_min || range > self.range_max {
            return None;
        }
        let angle = f64::from(single_precision_atan2(y, x));
        if angle < f64::from(self.angle_min) || angle > f64::from(self.angle_max) {
            return None;
        }
        Some((bin_of(angle, self.angle_min, self.angle_increment), range))
    }
}

/// The bin of a bearing at or above `angle_min`: its span over the increment, in double
/// precision, truncated as the callback converts it to an int.
fn bin_of(angle: f64, angle_min: f32, angle_increment: f32) -> usize {
    ((angle - f64::from(angle_min)) / f64::from(angle_increment)) as usize
}

/// sqrt(x^2 + y^2) of single precision values: computed in double precision, where the
/// squares are exact, and rounded to single precision.
fn single_precision_hypot(x: f32, y: f32) -> f32 {
    let (x, y) = (f64::from(x), f64::from(y));
    (x * x + y * y).sqrt() as f32
}

/// atan2(y, x) of single precision values: computed in double precision and rounded to
/// single precision.
fn single_precision_atan2(y: f32, x: f32) -> f32 {
    f64::from(y).atan2(f64::from(x)) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parameters() -> LaserProjectionParameters {
        LaserProjectionParameters {
            min_height: 0.1,
            max_height: 1.0,
            angle_min: -0.5,
            angle_max: 0.485,
            angle_increment: 0.01,
            range_min: 0.2,
            range_max: 5.0,
        }
    }

    #[test]
    fn the_nearest_point_of_a_bin_gives_its_range() {
        let projection = LaserProjection::new(parameters()).expect("valid parameters");
        let scan = projection.project(&[[2.0, 0.0, 0.5], [1.0, 0.0, 0.5], [3.0, 0.0, 0.5]]);
        let bin = bin_of(0.0, -0.5, 0.01f64 as f32);
        let hits: Vec<(usize, f32)> = scan
            .ranges()
            .iter()
            .copied()
            .enumerate()
            .filter(|&(_, range)| range != NO_RETURN_MARK_M)
            .collect();
        assert_eq!(hits, [(bin, 1.0)]);
    }

    #[test]
    fn points_outside_the_band_the_limits_or_the_fan_are_left_out() {
        let projection = LaserProjection::new(parameters()).expect("valid parameters");
        let left_out = [
            [1.0, 0.0, 0.05],
            [1.0, 0.0, 1.5],
            [0.1, 0.0, 0.5],
            [6.0, 0.0, 0.5],
            [0.0, 1.0, 0.5],
            [f32::NAN, 0.0, 0.5],
        ];
        let scan = projection.project(&left_out);
        assert!(scan.ranges().iter().all(|&range| range == NO_RETURN_MARK_M));
    }

    // A projected scan reads as the laser scan of its fields: a ray of each bin, a hit where
    // a point fell and no return elsewhere.
    #[test]
    fn a_projected_scan_reads_as_the_laser_scan_of_its_fields() {
        let projection = LaserProjection::new(parameters()).expect("valid parameters");
        let projected = projection.project(&[[1.0, 0.0, 0.5]]);
        let identity = camera_core::pose::PlanarPose::IDENTITY.to_pose();
        let still = camera_core::laser_scan::Sweep {
            first_ray: identity,
            last_ray: identity,
        };
        let scan = projected.laser_scan();
        assert_eq!(
            [
                scan.min_angle_rad(),
                scan.angle_increment_rad(),
                scan.min_range_m(),
                scan.max_range_m()
            ],
            [
                projected.min_angle_rad(),
                projected.angle_increment_rad(),
                projected.min_range_m(),
                projected.max_range_m()
            ]
        );
        let hits: Vec<Option<f32>> = scan
            .placed_rays(&still)
            .expect("every ray placed")
            .iter()
            .map(|ray| ray.hit_m)
            .collect();
        let expected: Vec<Option<f32>> = projected
            .ranges()
            .iter()
            .map(|&range| (range != NO_RETURN_MARK_M).then_some(range))
            .collect();
        assert_eq!(hits.len(), projection.bins());
        assert_eq!(hits, expected);
        assert_eq!(hits.iter().flatten().count(), 1);
    }

    #[test]
    fn parameters_whose_last_bin_lies_past_the_scan_are_refused() {
        let at_the_edge = LaserProjectionParameters {
            angle_max: 0.5,
            ..parameters()
        };
        assert!(matches!(
            LaserProjection::new(at_the_edge),
            Err(LaserProjectionError::LastBinOutsideScan { .. })
        ));
    }

    #[test]
    fn parameters_that_are_not_finite_are_refused_by_name() {
        let infinite = LaserProjectionParameters {
            max_height: f64::INFINITY,
            ..parameters()
        };
        assert_eq!(
            LaserProjection::new(infinite),
            Err(LaserProjectionError::NotFinite {
                name: "max_height",
                value: f64::INFINITY
            })
        );
    }

    #[test]
    fn range_limits_that_a_scan_cannot_carry_are_refused() {
        let limits = |range_min, range_max| LaserProjectionParameters {
            range_min,
            range_max,
            ..parameters()
        };
        let refused = |range_min, range_max| {
            LaserProjection::new(limits(range_min, range_max))
                == Err(LaserProjectionError::RangeLimits {
                    range_min,
                    range_max,
                })
        };
        assert!(refused(-0.1, 8.0));
        assert!(refused(2.0, 2.0));
        assert!(refused(1.0, 1.0 + 1e-12));
        assert!(refused(0.5, 1e39));
        assert!(refused(0.5, 10000.0));
        let projection = LaserProjection::new(limits(0.5, 0.5 + 1e-6)).expect("a projection");
        assert_eq!(
            projection.project(&[[0.5, 0.0, 0.5]]).ranges().len(),
            projection.bins()
        );
    }
}
