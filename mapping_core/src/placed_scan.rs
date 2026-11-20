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

//! A scan of a 2-D lidar with each ray placed where the sensor stood at the ray's time, as
//! `lidar_2d`'s `time_increment_s` gives it, in the base's frame at the scan's timestamp:
//! the sensor's poses at the first and the last ray seen from the base at the scan's
//! timestamp, and its pose at each ray interpolated between them as laser_geometry
//! interpolates them (`camera_core::laser_scan::LaserScan::placed_rays`). The sensor's poses
//! are planar, so its scan plane lies level. Each ray keeps its range, karto's reading of
//! it, a mark of `lidar_2d` as +inf, which karto's readings beyond the laser's limits skip.

use crate::geometry::{Pose2, Vector2};
use camera_core::laser_scan::{LaserScan, Sweep};
use control_core::planar_pose::PlanarPose;
use frame_tree::NotATransform;
use frame_tree::arithmetic::yaw_of_quaternion;

/// One ray of a placed scan: its reading (m, +inf for a mark), and where the sensor stood at
/// its time and the heading of the ray, in the base's frame at the scan's timestamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Beam {
    pub reading: f64,
    pub origin: Vector2,
    pub bearing: f64,
}

/// The angles and the range limits of a scan, in the single precision of its message, and
/// its number of rays: what karto makes its laser of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ScanGeometry {
    pub angle_min: f32,
    pub angle_increment: f32,
    pub rays: usize,
    pub range_min: f32,
    pub range_max: f32,
}

impl ScanGeometry {
    /// The angle of the last ray, as a `sensor_msgs/LaserScan` carries it.
    pub fn angle_max(&self) -> f32 {
        self.angle_min + (self.rays - 1) as f32 * self.angle_increment
    }

    /// The angle of ray `index` in the sensor's frame, as
    /// `camera_core::laser_scan::PlacedRay::angle_rad` gives it.
    pub fn ray_angle(&self, index: usize) -> f64 {
        f64::from(self.angle_min) + index as f64 * f64::from(self.angle_increment)
    }
}

/// A scan with each ray placed where the sensor stood at the ray's time, and the base's
/// pose in the odometry frame at the scan's timestamp.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacedScan {
    pub(crate) stamp_ns: i64,
    pub(crate) odometry: Pose2,
    pub(crate) geometry: ScanGeometry,
    pub(crate) beams: Vec<Beam>,
}

impl PlacedScan {
    /// `scan`, taken at `stamp_ns` by a sensor that stood at `sensor.first_ray` and
    /// `sensor.last_ray` on the floor of the odometry frame, at the scan's timestamp and
    /// `scan.sweep_s()` after it, with the base at `base` in the odometry frame at the
    /// scan's timestamp. A scan projected into the base frame stands at the base: its
    /// sensor's poses are `base`. Refused with the error of the first ray at whose time the
    /// sensor's poses interpolate to no pose.
    pub fn new(
        stamp_ns: i64,
        scan: &LaserScan,
        sensor: &Sweep<PlanarPose>,
        base: PlanarPose,
    ) -> Result<Self, NotATransform> {
        let from_base = base.to_transform().inverse();
        let in_base = Sweep {
            first_ray: from_base.compose(&sensor.first_ray.to_transform()),
            last_ray: from_base.compose(&sensor.last_ray.to_transform()),
        };
        let beams = scan
            .placed_rays(&in_base)?
            .into_iter()
            .map(|ray| {
                let [x, y, _] = ray.sensor.origin();
                Beam {
                    reading: ray.hit_m.map_or(f64::INFINITY, f64::from),
                    origin: Vector2::new(x, y),
                    bearing: yaw_of_quaternion(ray.sensor.orientation()) + ray.angle_rad,
                }
            })
            .collect();
        Ok(Self {
            stamp_ns,
            odometry: base.into(),
            geometry: ScanGeometry {
                angle_min: scan.min_angle_rad(),
                angle_increment: scan.angle_increment_rad(),
                rays: scan.ray_count(),
                range_min: scan.min_range_m(),
                range_max: scan.max_range_m(),
            },
            beams,
        })
    }

    /// The scan's timestamp (ns).
    pub fn stamp_ns(&self) -> i64 {
        self.stamp_ns
    }

    /// The base's pose in the odometry frame at the scan's timestamp.
    pub fn odometry(&self) -> PlanarPose {
        self.odometry.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localized_scan::point_of;
    use hegel::TestCase;
    use hegel::generators as gs;

    /// Where the wall of [`a_scan_swept_while_the_sensor_moves_places_each_ray_on_the_wall`]
    /// stands: the line x = `WALL_X` of the odometry frame.
    const WALL_X: f64 = 3.0;

    fn pose(x: f64, y: f64, yaw: f64) -> PlanarPose {
        PlanarPose::new(x, y, yaw).expect("a finite pose")
    }

    fn still(at: PlanarPose) -> Sweep<PlanarPose> {
        Sweep {
            first_ray: at,
            last_ray: at,
        }
    }

    #[hegel::test]
    fn a_scan_swept_while_the_sensor_moves_places_each_ray_on_the_wall(tc: TestCase) {
        // The sensor moves at a constant velocity and turns at a constant rate, so the
        // poses between its first and last rays are those that laser_geometry interpolates.
        let velocity: [f64; 2] = [
            tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
            tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
        ];
        let turn_rate: f64 = tc.draw(gs::floats().min_value(-2.0).max_value(2.0));
        let time_increment: f32 = tc.draw(gs::floats().min_value(0.0).max_value(1e-3));
        let rays = 100;
        let (angle_min, angle_increment) = (-0.6f32, 0.012f32);
        let at = |time: f64| pose(velocity[0] * time, velocity[1] * time, turn_rate * time);
        let ranges: Vec<f32> = (0..rays)
            .map(|index| {
                let time = f64::from(time_increment) * index as f64;
                let sensor = at(time);
                let angle = f64::from(angle_min) + index as f64 * f64::from(angle_increment);
                ((WALL_X - sensor.x()) / (sensor.yaw() + angle).cos()) as f32
            })
            .collect();
        let scan = LaserScan::parse(
            angle_min,
            angle_increment,
            time_increment,
            0.0,
            20.0,
            &ranges,
        )
        .expect("a scan");
        let sweep = Sweep {
            first_ray: at(0.0),
            last_ray: at(scan.sweep_s()),
        };
        let placed = PlacedScan::new(0, &scan, &sweep, PlanarPose::IDENTITY).expect("finite poses");
        for (index, beam) in placed.beams.iter().enumerate() {
            let end = point_of(Pose2::default(), beam);
            // A range in single precision rounds by up to 20 m x 2^-24.
            assert!(
                (end.x - WALL_X).abs() <= 2e-6,
                "ray {index} lands at {end:?}, off the wall at x = {WALL_X}"
            );
        }
    }

    #[test]
    fn the_geometry_gives_each_ray_the_angle_of_its_ray_in_the_scan() {
        let (angle_min, angle_increment) = (-3.1f32, 0.017f32);
        let scan = LaserScan::parse(angle_min, angle_increment, 0.0, 0.05, 12.0, &[1.0; 365])
            .expect("a scan");
        let placed = PlacedScan::new(0, &scan, &still(PlanarPose::IDENTITY), PlanarPose::IDENTITY)
            .expect("finite poses");
        // The still sensor at the identity heads each ray along its angle in the scan, as
        // camera_core's placed rays give it.
        for (index, beam) in placed.beams.iter().enumerate() {
            assert_eq!(placed.geometry.ray_angle(index), beam.bearing);
        }
    }

    fn scan(time_increment_s: f32, ranges: &[f32]) -> LaserScan {
        LaserScan::parse(-1.0, 0.5, time_increment_s, 0.1, 10.0, ranges).expect("a scan")
    }

    #[test]
    fn a_still_sensor_on_the_base_places_each_ray_along_its_angle() {
        let base = pose(3.0, -1.0, 0.7);
        let placed =
            PlacedScan::new(7, &scan(0.0, &[2.0; 5]), &still(base), base).expect("finite poses");
        for (index, beam) in placed.beams.iter().enumerate() {
            assert!(beam.origin.x.abs() < 1e-12 && beam.origin.y.abs() < 1e-12);
            assert!((beam.bearing - (-1.0 + 0.5 * index as f64)).abs() < 1e-12);
            assert_eq!(beam.reading, 2.0);
        }
    }

    // lidar_2d's marks, 10000 for no return and 0 for a ray not measured, read as karto's
    // reading beyond the laser's limits.
    #[test]
    fn a_mark_reads_as_no_return() {
        let base = PlanarPose::IDENTITY;
        let placed = PlacedScan::new(0, &scan(0.0, &[10000.0, 0.0]), &still(base), base)
            .expect("finite poses");
        assert!(
            placed
                .beams
                .iter()
                .all(|beam| beam.reading == f64::INFINITY)
        );
    }

    #[test]
    fn the_last_ray_of_a_turning_sensor_is_placed_where_the_sensor_stood_at_its_time() {
        let base = PlanarPose::IDENTITY;
        let sweep = Sweep {
            first_ray: base,
            last_ray: pose(0.2, 0.1, 0.4),
        };
        let placed =
            PlacedScan::new(0, &scan(0.01, &[1.0; 5]), &sweep, base).expect("finite poses");
        let last = placed.beams[4];
        assert!((last.origin.x - 0.2).abs() < 1e-12 && (last.origin.y - 0.1).abs() < 1e-12);
        assert!((last.bearing - (0.4 + 1.0)).abs() < 1e-12);
    }
}
