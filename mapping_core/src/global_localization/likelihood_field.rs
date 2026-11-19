//  Player - One Hell of a Robot Server
//  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
//                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This file translates into Rust Nav2 1.5.2's likelihood field model of AMCL:
// nav2_amcl/src/sensors/laser/likelihood_field_model.cpp (Copyright (C) 2000 Brian Gerkey &
// Kasper Stoy), the laser data of nav2_amcl/src/amcl_node.cpp (Copyright (c) 2008, Willow
// Garage, Inc.) and of nav2_amcl/include/nav2_amcl/sensors/laser/laser.hpp (Copyright (c) 2018
// Intel Corporation).
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

//! AMCL's likelihood field model of a laser (nav2_amcl
//! src/sensors/laser/likelihood_field_model.cpp, `LikelihoodFieldModel::sensorFunction`),
//! with each scan read into AMCL's laser data as `AmclNode::updateFilter` reads it
//! (src/amcl_node.cpp). A particle weighs the end of each beam it reads, every `step`-th of
//! the scan, by the distance from the end, placed from the particle, to the nearest
//! occupied cell: a Gaussian of that distance, weighted by `z_hit`, plus a uniform
//! likelihood of a random reading, weighted by `z_rand`; and the particle's weight grows by
//! one plus the sum of the cubes of its beams' likelihoods. A beam whose reading lies at
//! or below the shortest range or at or past the largest, or that reads no return, weighs
//! nothing.
//!
//! AMCL places every beam from one laser pose on the base, at the angle of its ray in the
//! base frame. Here each beam starts where the sensor stood at its ray's time
//! ([`PlacedScan`]), at AMCL's angle of its ray turned by the sensor's heading on the base
//! at that time: for a still sensor at the base's origin, AMCL's beams.

use super::map::Map;
use super::particle_filter::Sample;
use super::pose_arithmetic::coord_add;
use crate::geometry::Pose2;
use crate::mapping::tf2_yaw;
use crate::placed_scan::{Beam, PlacedScan, ScanGeometry};
use std::f64::consts::PI;

/// The parameters of the model: `z_hit`, `z_rand`, `sigma_hit` (m), `max_beams`, and the
/// range limits `laser_min_range` and `laser_max_range` (m) that tighten a scan's own when
/// above 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LikelihoodField {
    pub z_hit: f64,
    pub z_rand: f64,
    pub sigma_hit: f64,
    /// 2 or more.
    pub max_beams: usize,
    pub laser_min_range: f64,
    pub laser_max_range: f64,
}

/// `LaserData`: the range limits (m) within which a beam weighs a particle, in the single
/// precision of a scan; the first angle and the increment (rad) of the scan's rays in the
/// base frame, as `updateFilter` reads them; and the scan's beams.
#[derive(Debug)]
pub(crate) struct LaserData<'a> {
    range_min: f64,
    range_max: f64,
    angle_min: f64,
    angle_increment: f64,
    geometry: &'a ScanGeometry,
    beams: &'a [Beam],
}

impl<'a> LaserData<'a> {
    /// Whether `beam` weighs a particle: a reading between the range limits.
    fn weighs(&self, beam: &Beam) -> bool {
        beam.reading > self.range_min && beam.reading < self.range_max
    }

    /// The bearing (rad) of `beam`, the beam of ray `index`, in the base frame: AMCL's angle
    /// of the ray, turned by the sensor's heading on the base at the ray's time.
    fn bearing(&self, index: usize, beam: &Beam) -> f64 {
        let sensor_heading = beam.bearing - self.geometry.ray_angle(index);
        self.angle_min + index as f64 * self.angle_increment + sensor_heading
    }
}

impl LikelihoodField {
    /// The laser data that `updateFilter` makes of `scan`: the range limits of the scan,
    /// each tightened to the model's when that lies above 0 and within them, in single
    /// precision; and the scan's first angle and increment through tf2's quaternions, the
    /// increment wrapped into [-pi, pi).
    pub fn laser_data<'a>(&self, scan: &'a PlacedScan) -> LaserData<'a> {
        let geometry = &scan.geometry;
        let range_max = if self.laser_max_range > 0.0 {
            let limit = self.laser_max_range as f32;
            if limit < geometry.range_max {
                limit
            } else {
                geometry.range_max
            }
        } else {
            geometry.range_max
        };
        let range_min = if self.laser_min_range > 0.0 {
            let limit = self.laser_min_range as f32;
            if geometry.range_min < limit {
                limit
            } else {
                geometry.range_min
            }
        } else {
            geometry.range_min
        };
        let angle_min = tf2_yaw(f64::from(geometry.angle_min));
        let angle_increment =
            tf2_yaw(f64::from(geometry.angle_min + geometry.angle_increment)) - angle_min;
        LaserData {
            range_min: f64::from(range_min),
            range_max: f64::from(range_max),
            angle_min,
            angle_increment: (angle_increment + 5.0 * PI) % (2.0 * PI) - PI,
            geometry,
            beams: &scan.beams,
        }
    }

    /// `sensorFunction`: each particle's weight grown by its likelihood of `data` in `map`,
    /// and the sum of the weights.
    pub fn weigh(&self, map: &Map, data: &LaserData, samples: &mut [Sample]) -> f64 {
        let z_hit_denom = 2.0 * self.sigma_hit * self.sigma_hit;
        let z_rand_mult = 1.0 / data.range_max;
        let step = (data.beams.len().saturating_sub(1) / (self.max_beams - 1)).max(1);
        let mut total_weight = 0.0;
        for sample in samples {
            let mut p = 1.0;
            for (index, beam) in data.beams.iter().enumerate().step_by(step) {
                if !data.weighs(beam) {
                    continue;
                }
                // The beam's origin on the base, its heading in the beam's bearing.
                let sensor = Pose2::new(beam.origin.x, beam.origin.y, 0.0);
                let pose = coord_add(sensor, sample.pose);
                let bearing = data.bearing(index, beam);
                let hit_x = pose.x + beam.reading * (pose.yaw + bearing).cos();
                let hit_y = pose.y + beam.reading * (pose.yaw + bearing).sin();
                let z = map.occupied_distance_at(hit_x, hit_y);
                let mut pz = 0.0;
                pz += self.z_hit * (-(z * z) / z_hit_denom).exp();
                pz += self.z_rand * z_rand_mult;
                p += pz * pz * pz;
            }
            sample.weight *= p;
            total_weight += sample.weight;
        }
        total_weight
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::global_localization::tests::{room_grid, room_scan};
    use crate::occupancy_grid::{CellState, OccupancyGrid};
    use control_core::planar_pose::PlanarPose;

    fn model(laser_min_range: f64, laser_max_range: f64, max_beams: usize) -> LikelihoodField {
        LikelihoodField {
            z_hit: 0.5,
            z_rand: 0.5,
            sigma_hit: 0.2,
            max_beams,
            laser_min_range,
            laser_max_range,
        }
    }

    /// The scan of the room from `truth`, by odometry at the origin.
    fn scan_from(truth: PlanarPose) -> PlacedScan {
        room_scan(0, truth, PlanarPose::IDENTITY)
    }

    /// The weight of each particle at `particles`, each of weight 1, after `field` weighs
    /// `scan` in `map`.
    fn weights(
        field: &LikelihoodField,
        map: &Map,
        scan: &PlacedScan,
        particles: &[Pose2],
    ) -> Vec<f64> {
        let mut samples: Vec<Sample> = particles
            .iter()
            .map(|&pose| Sample { pose, weight: 1.0 })
            .collect();
        let total = field.weigh(map, &field.laser_data(scan), &mut samples);
        let weights: Vec<f64> = samples.iter().map(|sample| sample.weight).collect();
        assert_eq!(total, weights.iter().sum::<f64>());
        weights
    }

    #[test]
    fn the_models_range_limits_above_0_tighten_the_scans_and_others_leave_them() {
        // The scan's own limits: 0.1 m and 12 m.
        let scan = scan_from(PlanarPose::IDENTITY);
        let limits = |min: f64, max: f64| {
            let data = model(min, max, 60).laser_data(&scan);
            (data.range_min, data.range_max)
        };
        let scans_own = (f64::from(0.1f32), 12.0);
        assert_eq!(limits(0.5, 5.0), (f64::from(0.5f32), 5.0));
        assert_eq!(limits(0.05, 20.0), scans_own);
        assert_eq!(limits(0.0, 0.0), scans_own);
        assert_eq!(limits(-1.0, -1.0), scans_own);
    }

    #[test]
    fn a_beam_outside_the_range_limits_weighs_nothing() {
        // From the origin, the walls of the room lie 3 m to 5 m away.
        let map = Map::of_grid(&room_grid(), 2.0);
        let scan = scan_from(PlanarPose::IDENTITY);
        for field in [model(-1.0, 2.5, 60), model(5.5, 100.0, 60)] {
            assert_eq!(weights(&field, &map, &scan, &[Pose2::default()]), [1.0]);
        }
    }

    #[test]
    fn a_particle_at_the_true_pose_outweighs_particles_moved_or_turned_off_it() {
        let map = Map::of_grid(&room_grid(), 2.0);
        let truth = PlanarPose::new(0.5, -0.5, 0.25).expect("a finite pose");
        let at =
            |x: f64, y: f64, yaw: f64| Pose2::new(truth.x() + x, truth.y() + y, truth.yaw() + yaw);
        let particles = [
            at(0.0, 0.0, 0.0),
            at(0.3, 0.0, 0.0),
            at(0.0, -0.3, 0.0),
            at(0.0, 0.0, 0.2),
        ];
        let weights = weights(&model(-1.0, 100.0, 60), &map, &scan_from(truth), &particles);
        assert!(
            weights[1..].iter().all(|&off| off < weights[0]),
            "{weights:?}"
        );
    }

    #[test]
    fn a_particle_weighs_every_step_th_beam_so_at_most_max_beams_of_them() {
        // On a grid of occupied cells alone, each beam that ends on the map ends 0 m from
        // an occupied cell, and adds the cube of `z_hit` plus `z_rand` over the scan's
        // largest range (12 m) to the weight.
        let grid = room_grid();
        let cells = vec![CellState::Occupied; (grid.width() * grid.height()) as usize];
        let occupied = OccupancyGrid::from_cells(
            grid.width(),
            grid.height(),
            grid.resolution(),
            grid.origin(),
            cells,
        )
        .expect("a grid");
        let map = Map::of_grid(&occupied, 2.0);
        let scan = scan_from(PlanarPose::IDENTITY);
        // Of the scan's 360 beams: every 359th, every 6th (0 to 354), and every one.
        for (max_beams, weighed) in [(2, 2), (60, 60), (360, 360), (400, 360)] {
            let field = model(-1.0, 100.0, max_beams);
            let likelihood = field.z_hit + field.z_rand / 12.0;
            let weight = weights(&field, &map, &scan, &[Pose2::default()])[0];
            let beams = (weight - 1.0) / likelihood.powi(3);
            assert!(
                (beams - f64::from(weighed)).abs() < 1e-9,
                "max_beams {max_beams}: {beams} beams weighed"
            );
        }
    }
}
