//  Copyright (c) 2008, Willow Garage, Inc.
//  All rights reserved.
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This file translates into Rust Nav2 1.5.2's AMCL node flow of a scan,
// nav2_amcl/src/amcl_node.cpp (Copyright (c) 2008, Willow Garage, Inc.).
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

//! The parts of Nav2 1.5.2's AMCL node that global localization runs (nav2_amcl
//! src/amcl_node.cpp): the map that `handleMapMessage` converts, with its likelihood field
//! and free cells; the filter of `initParticleFilter`; `globalLocalizationCallback`, which
//! spreads the particles over the free cells; and `laserReceived` with
//! `shouldUpdateFilter` and `updateFilter`: the first scan after a start weighs the
//! particles where they lie, and each later scan whose odometry moved past `update_min_d`
//! in x or y or turned past `update_min_a` since the last update moves them by the motion
//! model and weighs them, every `resample_interval`-th update resampling them. AMCL
//! publishes the mean of the heaviest cluster as its pose, with the covariance of the whole
//! filter, after the first scan of a start, after each scan whose update resampled the
//! filter, and after every scan until it first publishes one (`laserReceived`,
//! `publishAmclPose`).

use super::drand48::Drand48;
use super::likelihood_field::LikelihoodField;
use super::map::Map;
use super::motion_model::MotionModel;
use super::parameters::AmclSettings;
use super::particle_filter::ParticleFilter;
use super::pose_arithmetic::{angle_diff, odometry_yaw};
use crate::geometry::{Matrix3, Pose2};
use crate::occupancy_grid::OccupancyGrid;
use crate::placed_scan::PlacedScan;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Amcl {
    map: Map,
    filter: ParticleFilter,
    motion: MotionModel,
    laser: LikelihoodField,
    update_min_a: f64,
    update_min_d: f64,
    resample_interval: u64,
    /// `pf_odom_pose_`: the odometry at the last update, after the first scan of a start.
    odometry_at_update: Option<Pose2>,
    resample_count: u64,
    /// `first_pose_sent_`: whether AMCL has published a pose.
    pose_sent: bool,
}

impl Amcl {
    /// AMCL with the map of `grid`, started on `stream`; none for a grid with no free cell.
    pub fn new(
        grid: &OccupancyGrid,
        parameters: AmclSettings,
        stream: &mut Drand48,
    ) -> Option<Self> {
        let map = Map::of_grid(grid, parameters.laser_likelihood_max_dist);
        if map.free_cells().is_empty() {
            return None;
        }
        let mut amcl = Self {
            map,
            filter: ParticleFilter::new(parameters.filter),
            motion: parameters.motion,
            laser: parameters.laser,
            update_min_a: parameters.update_min_a,
            update_min_d: parameters.update_min_d,
            resample_interval: u64::from(parameters.resample_interval),
            odometry_at_update: None,
            resample_count: 0,
            pose_sent: false,
        };
        amcl.start(stream);
        Some(amcl)
    }

    /// `globalLocalizationCallback`: the particles spread over the free cells by draws on
    /// `stream`, and the next scan the first of the start.
    pub fn start(&mut self, stream: &mut Drand48) {
        let map = &self.map;
        self.filter
            .init_model(stream, |generator| map.uniform_pose(generator));
        self.odometry_at_update = None;
    }

    /// `laserReceived` of `scan`, its random draws on `stream`: the mean of the heaviest
    /// cluster after it when AMCL publishes it; none after a scan that AMCL publishes no
    /// pose after, while no cluster weighs more than 0, and for a pose that
    /// `publishAmclPose` finds a NaN in.
    pub fn add(&mut self, scan: &PlacedScan, stream: &mut Drand48) -> Option<Pose2> {
        let odometry = scan.odometry;
        let pose = Pose2::new(odometry.x, odometry.y, odometry_yaw(odometry.yaw));
        let first_of_start = self.odometry_at_update.is_none();
        let resampled = match self.odometry_at_update {
            None => {
                self.resample_count = 0;
                self.update_filter(scan, pose, stream)
            }
            Some(last) => {
                let delta = Pose2::new(
                    pose.x - last.x,
                    pose.y - last.y,
                    angle_diff(pose.yaw, last.yaw),
                );
                let moved = delta.x.abs() > self.update_min_d
                    || delta.y.abs() > self.update_min_d
                    || delta.yaw.abs() > self.update_min_a;
                if moved {
                    self.motion
                        .odometry_update(self.filter.samples_mut(), stream, pose, delta);
                    self.update_filter(scan, pose, stream)
                } else {
                    false
                }
            }
        };
        let publishes = resampled || first_of_start || !self.pose_sent;
        if !publishes {
            return None;
        }
        self.published_pose()
    }

    /// `updateFilter` of `scan`, with the odometry at `pose`, and the resampling of every
    /// `resample_interval`-th update on `stream`: whether it resampled.
    fn update_filter(&mut self, scan: &PlacedScan, pose: Pose2, stream: &mut Drand48) -> bool {
        let data = self.laser.laser_data(scan);
        let (laser, map) = (&self.laser, &self.map);
        self.filter
            .update_sensor(|samples| laser.weigh(map, &data, samples));
        self.odometry_at_update = Some(pose);
        self.resample_count += 1;
        let resamples = self.resample_count.is_multiple_of(self.resample_interval);
        if resamples {
            self.filter.update_resample(stream);
        }
        resamples
    }

    /// `getMaxWeightHyp` and `publishAmclPose`: the mean of the heaviest cluster, published
    /// unless a NaN lies in it or in the filter's covariance.
    fn published_pose(&mut self) -> Option<Pose2> {
        let mean = self.filter.heaviest_cluster_mean()?;
        if holds_nan(mean, self.filter.covariance()) {
            return None;
        }
        self.pose_sent = true;
        Some(mean)
    }

    /// The covariance of the whole filter, as `amcl_pose` carries it.
    pub fn covariance(&self) -> &Matrix3 {
        self.filter.covariance()
    }
}

/// Whether `publishAmclPose` finds a NaN in `mean` with `covariance`: it sums, in single
/// precision, the terms of the 6 x 6 covariance it fills (x and y in its first two rows
/// and columns, the heading at its last term, 0 elsewhere), then the sum of the mean's x
/// and y.
fn holds_nan(mean: Pose2, covariance: &Matrix3) -> bool {
    let [[xx, xy, _], [yx, yy, _], [_, _, yaw]] = covariance.0;
    let sum = [xx, xy, yx, yy, yaw, mean.x + mean.y]
        .into_iter()
        .fold(0.0f32, |sum, term| (f64::from(sum) + term) as f32);
    sum.is_nan()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::global_localization::AmclParameters;
    use crate::global_localization::parameters::tests::nav2_parameters;
    use crate::global_localization::tests::{room_grid, room_scan};
    use control_core::planar_pose::PlanarPose;

    /// Where the base stands at the first scan, at binary fractions, so a move of
    /// `update_min_d` along x or y gives a delta of exactly `update_min_d`.
    fn first() -> PlanarPose {
        PlanarPose::new(0.5, -0.5, 0.25).expect("a finite pose")
    }

    fn started(parameters: AmclParameters, stream: &mut Drand48) -> Amcl {
        Amcl::new(
            &room_grid(),
            AmclSettings::parse(parameters).expect("valid parameters"),
            stream,
        )
        .expect("a room with free floor")
    }

    /// The scan of the room with the base at [`first`] moved by `x`, `y` (m) and `yaw` (rad),
    /// its odometry reading the same pose.
    fn moved(x: f64, y: f64, yaw: f64) -> PlacedScan {
        let pose = PlanarPose::new(first().x() + x, first().y() + y, first().yaw() + yaw)
            .expect("a finite pose");
        room_scan(0, pose, pose)
    }

    /// AMCL after its first scan, at [`first`], and the stream it draws on.
    fn after_the_first_scan(parameters: AmclParameters) -> (Amcl, Drand48) {
        let mut stream = Drand48::seeded(7);
        let mut amcl = started(parameters, &mut stream);
        let before = amcl.filter.clone();
        assert!(
            amcl.add(&moved(0.0, 0.0, 0.0), &mut stream).is_some(),
            "the first scan of a start publishes a pose"
        );
        assert_ne!(
            amcl.filter, before,
            "the first scan of a start weighs the particles"
        );
        (amcl, stream)
    }

    #[test]
    fn a_scan_whose_odometry_moved_no_farther_than_the_thresholds_changes_nothing() {
        let parameters = nav2_parameters();
        let (first, stream) = after_the_first_scan(parameters);
        let (d, a) = (parameters.update_min_d, parameters.update_min_a);
        for (x, y, yaw) in [
            (d, 0.0, 0.0),
            (0.0, -d, 0.0),
            (0.0, 0.0, 0.9 * a),
            (0.9 * d, -0.9 * d, 0.9 * a),
        ] {
            let (mut amcl, mut drawn) = (first.clone(), stream.clone());
            assert_eq!(
                amcl.add(&moved(x, y, yaw), &mut drawn),
                None,
                "moved by ({x}, {y}, {yaw})"
            );
            assert_eq!(amcl, first, "moved by ({x}, {y}, {yaw})");
            assert_eq!(drawn, stream, "moved by ({x}, {y}, {yaw}): no draw");
        }
    }

    #[test]
    fn a_scan_whose_odometry_moved_past_a_threshold_updates_the_filter() {
        let parameters = nav2_parameters();
        let (first, stream) = after_the_first_scan(parameters);
        let (d, a) = (parameters.update_min_d, parameters.update_min_a);
        for (x, y, yaw) in [
            (1.1 * d, 0.0, 0.0),
            (0.0, -1.1 * d, 0.0),
            (0.0, 0.0, 1.1 * a),
        ] {
            let mut amcl = first.clone();
            assert!(
                amcl.add(&moved(x, y, yaw), &mut stream.clone()).is_some(),
                "moved by ({x}, {y}, {yaw})"
            );
            assert_ne!(amcl.filter, first.filter, "moved by ({x}, {y}, {yaw})");
        }
    }

    #[test]
    fn until_a_pose_is_published_each_scan_publishes_one() {
        let (mut amcl, mut stream) = after_the_first_scan(nav2_parameters());
        // As after a first scan whose pose went unpublished.
        amcl.pose_sent = false;
        assert!(amcl.add(&moved(0.0, 0.0, 0.0), &mut stream).is_some());
        assert_eq!(amcl.add(&moved(0.0, 0.0, 0.0), &mut stream), None);
    }

    #[test]
    fn with_a_resample_interval_of_2_every_second_update_resamples_and_publishes() {
        let (mut amcl, mut stream) = after_the_first_scan(AmclParameters {
            resample_interval: 2,
            ..nav2_parameters()
        });
        // Each scan moves the base 0.3 m on, past update_min_d, so each updates the filter.
        let answers: Vec<bool> = (1..=4)
            .map(|step| {
                let before = amcl.filter.clone();
                let answer = amcl.add(&moved(0.3 * f64::from(step), 0.0, 0.0), &mut stream);
                assert_ne!(amcl.filter, before, "update {step}");
                answer.is_some()
            })
            .collect();
        assert_eq!(answers, [true, false, true, false]);
    }

    #[test]
    fn a_pose_whose_position_or_covariance_sums_to_a_nan_goes_unpublished() {
        let covariance = Matrix3([[0.1, 0.01, 0.0], [0.01, 0.2, 0.0], [0.0, 0.0, 0.3]]);
        let mean = Pose2::new(1.0, 2.0, 0.5);
        assert!(!holds_nan(mean, &covariance));
        assert!(holds_nan(Pose2::new(f64::NAN, 2.0, 0.5), &covariance));
        let with = |row: usize, column: usize, value: f64| {
            let mut terms = covariance.0;
            terms[row][column] = value;
            Matrix3(terms)
        };
        assert!(holds_nan(mean, &with(2, 2, f64::NAN)));
        // Opposite infinities sum to a NaN; one infinity alone does not.
        let mut opposite = with(0, 0, f64::INFINITY).0;
        opposite[1][1] = f64::NEG_INFINITY;
        assert!(holds_nan(mean, &Matrix3(opposite)));
        assert!(!holds_nan(mean, &with(0, 0, f64::INFINITY)));
        // publishAmclPose reads no term of the heading's mean.
        assert!(!holds_nan(Pose2::new(1.0, 2.0, f64::NAN), &covariance));
    }
}
