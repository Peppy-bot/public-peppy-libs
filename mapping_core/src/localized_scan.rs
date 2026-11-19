// Copyright 2010 SRI International
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// laser_utils
// Copyright (c) 2019, Samsung Research America
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
// COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
// COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
// AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
//
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//
// slam_toolbox
// Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
// Copyright Work Modifications (c) 2019, Samsung Research America
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
// COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
// COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
// AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
//
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//
// This file translates into Rust karto's laser and localized scan as slam_toolbox 2.10.0
// carries them: lib/karto_sdk/include/karto_sdk/Karto.h and lib/karto_sdk/src/Karto.cpp
// (Copyright 2010 SRI International); and slam_toolbox 2.10.0's laser and scan of a message:
// src/laser_utils.cpp (Copyright (c) 2019, Samsung Research America) and
// src/slam_toolbox_common.cpp (Copyright Work Modifications (c) 2018, Simbe Robotics, Inc., and
// (c) 2019, Samsung Research America).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
//
// This translation takes slam_toolbox's own code under the GNU Lesser General Public License,
// version 2.1, of slam_toolbox's LICENSE; as section 3 of that license allows, the GNU General
// Public License, version 3 or later, applies in its place.
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

//! A scan where the base stood when it took it, as karto holds it (slam_toolbox 2.10.0,
//! lib/karto_sdk: Karto.h's `LaserRangeFinder` and `LocalizedRangeScan`, Karto.cpp's
//! `LaserRangeFinder::Validate`), with the laser that slam_toolbox's `LaserAssistant`
//! makes of the first scan (src/laser_utils.cpp, `makeLaser`). A scan's pose is the base's
//! at the scan's timestamp, and each of its beams is placed in the base's frame there
//! ([`crate::placed_scan`]): the laser's offset on the base is the identity, and a beam's
//! origin and bearing carry where the sensor stood at the beam's time.

use crate::geometry::{BoundingBox, Pose2, Transform, Vector2, normalize_angle, round};
use crate::placed_scan::{Beam, ScanGeometry};
use thiserror::Error;

/// Why a scan is not one the laser reads.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ScanError {
    #[error(
        "the scan holds {ranges} ranges, and the laser made from the first scan reads \
         {expected}: every scan takes the angles of the first"
    )]
    ReadingCount { ranges: usize, expected: usize },
    #[error(
        "the scan's angles or range limits, with min_laser_range and max_laser_range, make \
         another laser than the map's: localize with the scans and the laser ranges that \
         made the map"
    )]
    NotTheMapsLaser,
}

/// The id of a scan in a map: karto's unique id of the scan, which with one laser is also
/// its state id, counted from 0 in the order in which the map adds its scans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ScanId(pub(crate) u32);

impl ScanId {
    /// The id of the scan added before this one, karto's `GetStateId() - 1`.
    pub(crate) fn previous(self) -> Option<Self> {
        self.0.checked_sub(1).map(Self)
    }
}

/// `LaserRangeFinder`: the angles and the range limits of every scan of a map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Laser {
    pub minimum_angle: f64,
    pub maximum_angle: f64,
    pub angular_resolution: f64,
    pub minimum_range: f64,
    pub maximum_range: f64,
    pub range_threshold: f64,
    pub number_of_readings: usize,
}

/// `math::Clip`.
fn clip(value: f64, minimum: f64, maximum: f64) -> f64 {
    let at_least = if value > minimum { value } else { minimum };
    if at_least < maximum {
        at_least
    } else {
        maximum
    }
}

impl Laser {
    /// `LaserAssistant::makeLaser` of a scan of `geometry`, with slam_toolbox's
    /// `min_laser_range` and `max_laser_range`.
    pub fn of(geometry: &ScanGeometry, min_laser_range: f64, max_laser_range: f64) -> Self {
        let scan_range_min = f64::from(geometry.range_min);
        let scan_range_max = f64::from(geometry.range_max);
        let minimum_range = if min_laser_range < 0.0 || min_laser_range < scan_range_min {
            scan_range_min
        } else {
            min_laser_range
        };
        let maximum_range = scan_range_max;
        let threshold = if max_laser_range <= 0.0 || max_laser_range > scan_range_max {
            scan_range_max
        } else {
            max_laser_range
        };
        let (minimum_angle, maximum_angle, angular_resolution) = (
            f64::from(geometry.angle_min),
            f64::from(geometry.angle_max()),
            f64::from(geometry.angle_increment),
        );
        let residual = if is_360_laser(geometry) { 0.0 } else { 1.0 };
        let number_of_readings =
            (round((maximum_angle - minimum_angle) / angular_resolution) + residual) as usize;
        Self {
            minimum_angle,
            maximum_angle,
            angular_resolution,
            minimum_range,
            maximum_range,
            range_threshold: clip(threshold, minimum_range, maximum_range),
            number_of_readings,
        }
    }

    /// `LaserRangeFinder::Validate(scan)`: the scan holds the laser's number of readings.
    pub fn validate(&self, geometry: &ScanGeometry) -> Result<(), ScanError> {
        if geometry.rays == self.number_of_readings {
            Ok(())
        } else {
            Err(ScanError::ReadingCount {
                ranges: geometry.rays,
                expected: self.number_of_readings,
            })
        }
    }
}

/// The span (rad) above which `makeLaser` takes a laser whose rounded steps and one give
/// the number of ranges as one that does not see all around: 350 deg, as laser_utils.cpp
/// writes it.
const COUNTED_FULL_TURN_SPAN: f64 = 6.10865;

/// `makeLaser`'s test of a laser that sees all around: a span within an increment of a
/// full turn, but for a span over 350 deg whose rounded steps and one give the number of
/// ranges.
fn is_360_laser(geometry: &ScanGeometry) -> bool {
    let angular_range = (geometry.angle_max() - geometry.angle_min).abs();
    let within_a_step = (f64::from(angular_range) - 2.0 * std::f64::consts::PI).abs()
        < f64::from(geometry.angle_increment)
            - f64::from(f32::EPSILON) * 2.0 * std::f64::consts::PI;
    let counted = f64::from(angular_range) > COUNTED_FULL_TURN_SPAN
        && (angular_range / geometry.angle_increment).round() + 1.0 == geometry.rays as f32;
    within_a_step && !counted
}

/// `LocalizedRangeScan`: a scan's ranges, its time, the pose its odometry gives and the
/// pose the map corrects it to, and from the corrected pose its points, the barycenter of
/// its points within range and the box that holds them.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct LocalizedScan {
    /// The time of the scan (s).
    pub time: f64,
    pub beams: Vec<Beam>,
    odometric_pose: Pose2,
    corrected_pose: Pose2,
    unfiltered_point_readings: Vec<Vector2>,
    barycenter_pose: Pose2,
    bounding_box: BoundingBox,
}

impl LocalizedScan {
    /// The scan at `odometric_pose`, which is also its corrected pose, as slam_toolbox's
    /// `getLocalizedRangeScan` makes it.
    pub fn new(laser: &Laser, beams: Vec<Beam>, time: f64, odometric_pose: Pose2) -> Self {
        let mut scan = Self {
            time,
            beams,
            odometric_pose,
            corrected_pose: odometric_pose,
            unfiltered_point_readings: Vec::new(),
            barycenter_pose: Pose2::default(),
            bounding_box: BoundingBox::default(),
        };
        scan.update(laser);
        scan
    }

    /// The scan as a saved map holds it: at `odometric_pose` by odometry, corrected to
    /// `corrected_pose`.
    pub fn with_poses(
        laser: &Laser,
        beams: Vec<Beam>,
        time: f64,
        odometric_pose: Pose2,
        corrected_pose: Pose2,
    ) -> Self {
        let mut scan = Self::new(laser, beams, time, odometric_pose);
        scan.set_corrected_pose(corrected_pose, laser);
        scan
    }

    pub fn odometric_pose(&self) -> Pose2 {
        self.odometric_pose
    }

    pub fn set_odometric_pose(&mut self, pose: Pose2) {
        self.odometric_pose = pose;
    }

    pub fn corrected_pose(&self) -> Pose2 {
        self.corrected_pose
    }

    /// `SetCorrectedPose` and the update of what it moves.
    pub fn set_corrected_pose(&mut self, pose: Pose2, laser: &Laser) {
        self.corrected_pose = pose;
        self.update(laser);
    }

    /// `GetSensorPose`: the laser's pose at the corrected pose.
    pub fn sensor_pose(&self) -> Pose2 {
        sensor_at(self.corrected_pose)
    }

    /// `SetSensorPose`: the corrected pose at which the laser stands at `sensor_pose`.
    pub fn set_sensor_pose(&mut self, sensor_pose: Pose2, laser: &Laser) {
        self.corrected_pose = corrected_at(sensor_pose);
        self.update(laser);
    }

    /// `GetReferencePose`: the barycenter of the points within range, or the sensor's
    /// pose.
    pub fn reference_pose(&self, use_barycenter: bool) -> Pose2 {
        if use_barycenter {
            self.barycenter_pose
        } else {
            self.sensor_pose()
        }
    }

    /// `GetPointReadings(false)`: the point of every range, in the map frame.
    pub fn unfiltered_point_readings(&self) -> &[Vector2] {
        &self.unfiltered_point_readings
    }

    pub fn bounding_box(&self) -> &BoundingBox {
        &self.bounding_box
    }

    /// `LocalizedRangeScan::Update`: the points at the corrected pose, and the barycenter
    /// and the box of those of the ranges within the laser's limits.
    fn update(&mut self, laser: &Laser) {
        let scan_pose = self.sensor_pose();
        let unfiltered: Vec<Vector2> = self
            .beams
            .iter()
            .map(|beam| point_of(scan_pose, beam))
            .collect();
        let filtered: Vec<Vector2> = self
            .beams
            .iter()
            .zip(&unfiltered)
            .filter(|(beam, _)| {
                beam.reading >= laser.minimum_range && beam.reading <= laser.range_threshold
            })
            .map(|(_, &point)| point)
            .collect();
        let sum = filtered
            .iter()
            .fold(Vector2::default(), |sum, &point| sum.plus(point));
        self.barycenter_pose = if filtered.is_empty() {
            scan_pose
        } else {
            Pose2::at(sum.divided(filtered.len() as f64), 0.0)
        };
        let mut bounding_box = BoundingBox::default();
        bounding_box.add(scan_pose.position());
        for &point in &filtered {
            bounding_box.add(point);
        }
        self.bounding_box = bounding_box;
        self.unfiltered_point_readings = unfiltered;
    }
}

/// The point of `beam` with the laser at `scan_pose`: its origin carried by the pose, and
/// its reading along its bearing turned by the pose's heading, as karto's `Update` places a
/// reading along its angle from the laser.
pub(crate) fn point_of(scan_pose: Pose2, beam: &Beam) -> Vector2 {
    let (sin, cos) = scan_pose.yaw.sin_cos();
    let angle = scan_pose.yaw + beam.bearing;
    Vector2::new(
        scan_pose.x + cos * beam.origin.x - sin * beam.origin.y + beam.reading * angle.cos(),
        scan_pose.y + sin * beam.origin.x + cos * beam.origin.y + beam.reading * angle.sin(),
    )
}

/// `GetSensorAt(pose)`: the laser's pose when the base stands at `pose`, through karto's
/// transform of the identity offset.
pub(crate) fn sensor_at(pose: Pose2) -> Pose2 {
    Transform::to(pose).transform_pose(Pose2::default())
}

/// `GetCorrectedAt(sensor_pose)`: the base's pose when the laser stands at
/// `sensor_pose`, with the identity offset.
pub(crate) fn corrected_at(sensor_pose: Pose2) -> Pose2 {
    let corrected_heading = normalize_angle(sensor_pose.yaw);
    let offset = Pose2::new(
        0.0 * corrected_heading.cos(),
        0.0 * corrected_heading.sin(),
        0.0,
    );
    sensor_pose.minus(offset)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry(rays: usize) -> ScanGeometry {
        ScanGeometry {
            angle_min: -0.5,
            angle_increment: 0.25,
            rays,
            range_min: 0.2,
            range_max: 4.0,
        }
    }

    /// The beams of a still laser on the base, along its angles.
    fn beams(readings: &[f64]) -> Vec<Beam> {
        readings
            .iter()
            .enumerate()
            .map(|(index, &reading)| Beam {
                reading,
                origin: Vector2::new(0.0, 0.0),
                bearing: -0.5 + 0.25 * index as f64,
            })
            .collect()
    }

    #[test]
    fn the_laser_reads_the_rounded_steps_and_one_within_its_limits() {
        let laser = Laser::of(&geometry(5), 0.3, 10.0);
        assert_eq!(laser.number_of_readings, 5);
        assert_eq!(laser.minimum_range, 0.3);
        assert_eq!(laser.range_threshold, 4.0);
        assert!(laser.validate(&geometry(4)).is_err());
    }

    #[test]
    fn a_scan_holds_the_point_of_each_range_and_the_box_of_those_within_the_limits() {
        let laser = Laser::of(&geometry(5), 0.2, 4.0);
        let readings = [1.0, f64::INFINITY, 2.0, 0.1, 1.0];
        let scan = LocalizedScan::new(&laser, beams(&readings), 0.0, Pose2::new(1.0, 0.0, 0.0));
        assert_eq!(scan.unfiltered_point_readings().len(), 5);
        let straight_ahead = scan.unfiltered_point_readings()[2];
        assert!((straight_ahead.x - 3.0).abs() < 1e-12 && straight_ahead.y.abs() < 1e-12);
        let corners = scan.bounding_box();
        assert!((corners.maximum.x - 3.0).abs() < 1e-12, "{corners:?}");
        assert!(
            corners.minimum.x >= 1.0 && corners.minimum.x < 2.0,
            "{corners:?}"
        );
    }

    #[test]
    fn a_beam_from_an_origin_off_the_laser_is_carried_by_the_scan_pose() {
        let beam = Beam {
            reading: 1.0,
            origin: Vector2::new(0.5, 0.0),
            bearing: 0.0,
        };
        let point = point_of(Pose2::new(1.0, 1.0, std::f64::consts::FRAC_PI_2), &beam);
        assert!(
            (point.x - 1.0).abs() < 1e-12 && (point.y - 2.5).abs() < 1e-12,
            "{point:?}"
        );
    }
}
