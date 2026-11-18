//! The scans of a planar laser range finder, a 2-D lidar, as lidar_2d:v1
//! carries them: a fan of rays in the sensor frame's XY plane, ray `i` at
//! `min_angle_rad + i x angle_increment_rad` counterclockwise about +Z from +X,
//! measured `time_increment_s x i` after the scan's timestamp, the increment
//! below 0 for a sensor that sweeps clockwise, its range the distance to the
//! first surface it met, or one of the contract's two marks: 0 for a ray the
//! sensor could not measure or a surface nearer than `min_range_m`, and 10000
//! for a ray that met nothing within `max_range_m`. A range at `max_range_m`
//! is no return, as laser_geometry keeps only a range below its cutoff, the
//! scan's `range_max` (laser_geometry.cpp:208).
//!
//! A sensor that moves while it sweeps measures each ray from where it stood at
//! that ray's time. [`LaserScan::placed_hits`] places each ray's hit by the
//! sensor's pose at the ray's time, interpolated between its poses at the first
//! and the last ray by the share `i * (1 / (n - 1))` of the sweep, as
//! laser_geometry's `transformLaserScanToPointCloud` interpolates them
//! ([`Pose::interpolated`]). The caller gives the two poses as a [`Sweep`]:
//! where the sensor stood at the timestamp, its first ray, and
//! [`LaserScan::sweep_s`] after it, its last ray. A ray at whose share the
//! two poses interpolate to no pose places no hit. Each [`PlacedHit`] keeps
//! the hit in the sensor's frame at its ray's time too, where a body fixed to
//! the sensor holds it however the sensor moved while it swept.

use std::fmt;

use crate::pose::{Pose, PoseError};

/// lidar_2d:v1's range of a ray the sensor could not measure, or of a surface
/// nearer than `min_range_m`.
const UNMEASURED_MARK_M: f32 = 0.0;

/// lidar_2d:v1's range of a ray that met nothing within `max_range_m`, above
/// every `max_range_m` the contract allows.
const NO_RETURN_MARK_M: f32 = 10000.0;

/// Where a scan's sensor, or the base it rides on, stood at the scan's first
/// ray, its timestamp, and at its last ray, [`LaserScan::sweep_s`] after it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sweep<P> {
    pub first_ray: P,
    pub last_ray: P,
}

/// A ray's hit: where it lies in the sensor's frame where the sensor stood at
/// the ray's time, and where the sensor's pose at that time places it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlacedHit {
    /// The hit in the sensor's frame at the ray's time.
    pub in_sensor: [f64; 3],
    /// The hit in the frame of the sensor's poses, placed by the sensor's
    /// pose at the ray's time.
    pub placed: [f64; 3],
}

/// What one ray measured.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Range {
    /// The distance (m) to the first surface the ray met, from `min_range_m`
    /// up to below `max_range_m`.
    Hit(f32),
    /// The ray met nothing within `max_range_m`: a range at `max_range_m` or
    /// above it, the mark 10000 among them.
    NoReturn,
    /// The sensor could not measure the ray, or a surface stood nearer than
    /// `min_range_m`: the mark 0.
    Unmeasured,
}

impl Range {
    /// What the range `value` of a scan whose ranges lie from `min_range_m` to
    /// `max_range_m` measured; none for a value lidar_2d:v1 does not allow: one
    /// that is not finite, one below 0, or one above 0 and below `min_range_m`.
    fn of(value: f32, min_range_m: f32, max_range_m: f32) -> Option<Range> {
        if value == UNMEASURED_MARK_M {
            return Some(Range::Unmeasured);
        }
        if value >= max_range_m {
            return value.is_finite().then_some(Range::NoReturn);
        }
        (value >= min_range_m).then_some(Range::Hit(value))
    }
}

/// One ray of a scan.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Ray {
    /// Its place in the scan, from 0.
    index: usize,
    /// Its angle in the sensor frame (rad), `min_angle_rad + index x
    /// angle_increment_rad` in double precision.
    angle_rad: f64,
    /// Its time after the scan's timestamp (s), `time_increment_s x index`.
    time_s: f64,
    range: Range,
}

impl Ray {
    /// The point the ray hit, in the sensor frame where the sensor stood at the
    /// ray's time; none for a mark.
    fn hit(&self) -> Option<[f64; 3]> {
        let Range::Hit(range) = self.range else {
            return None;
        };
        let (sin, cos) = self.angle_rad.sin_cos();
        let range = f64::from(range);
        Some([range * cos, range * sin, 0.0])
    }
}

/// A scan whose fields lidar_2d:v1 allows: at least one ray, finite angles
/// with an increment above 0, a finite time increment, a nearest range of 0
/// or above and a farthest one above it and below 10000, and each range
/// read as what its ray measured.
#[derive(Debug, Clone, PartialEq)]
pub struct LaserScan {
    min_angle_rad: f32,
    angle_increment_rad: f32,
    time_increment_s: f32,
    max_range_m: f32,
    ranges: Vec<Range>,
}

/// Why the fields of a scan make no scan.
#[derive(Debug, Clone, PartialEq)]
pub enum LaserScanError {
    NoRays,
    Angles {
        min_angle_rad: f32,
        angle_increment_rad: f32,
    },
    TimeIncrement(f32),
    Limits {
        min_range_m: f32,
        max_range_m: f32,
    },
    OutsideLimits {
        index: usize,
        range_m: f32,
        min_range_m: f32,
        max_range_m: f32,
    },
}

impl fmt::Display for LaserScanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LaserScanError::NoRays => write!(f, "a scan of no rays: give at least one range"),
            LaserScanError::Angles {
                min_angle_rad,
                angle_increment_rad,
            } => write!(
                f,
                "min_angle_rad {min_angle_rad} and angle_increment_rad {angle_increment_rad} \
                 make no fan of rays: give finite angles and an increment above 0"
            ),
            LaserScanError::TimeIncrement(value) => write!(
                f,
                "time_increment_s {value} is no time between rays: give a finite time, below 0 \
                 for a sensor that sweeps clockwise"
            ),
            LaserScanError::Limits {
                min_range_m,
                max_range_m,
            } => write!(
                f,
                "min_range_m {min_range_m} and max_range_m {max_range_m} make no range: give a \
                 min_range_m of 0 or above and a max_range_m above it and below 10000"
            ),
            LaserScanError::OutsideLimits {
                index,
                range_m,
                min_range_m,
                max_range_m,
            } => write!(
                f,
                "ray {index} measured {range_m} m, outside the sensor's {min_range_m} m to \
                 {max_range_m} m: give 0 for a ray not measured or a surface nearer than \
                 min_range_m, and 10000 for a ray that met nothing"
            ),
        }
    }
}

impl std::error::Error for LaserScanError {}

impl LaserScan {
    /// The scan of lidar_2d:v1's fields, its ranges in ray order.
    pub fn parse(
        min_angle_rad: f32,
        angle_increment_rad: f32,
        time_increment_s: f32,
        min_range_m: f32,
        max_range_m: f32,
        ranges: &[f32],
    ) -> Result<Self, LaserScanError> {
        if ranges.is_empty() {
            return Err(LaserScanError::NoRays);
        }
        if !(min_angle_rad.is_finite()
            && angle_increment_rad.is_finite()
            && angle_increment_rad > 0.0)
        {
            return Err(LaserScanError::Angles {
                min_angle_rad,
                angle_increment_rad,
            });
        }
        if !time_increment_s.is_finite() {
            return Err(LaserScanError::TimeIncrement(time_increment_s));
        }
        if !(min_range_m >= 0.0 && max_range_m > min_range_m && max_range_m < NO_RETURN_MARK_M) {
            return Err(LaserScanError::Limits {
                min_range_m,
                max_range_m,
            });
        }
        let ranges = ranges
            .iter()
            .enumerate()
            .map(|(index, &range_m)| {
                Range::of(range_m, min_range_m, max_range_m).ok_or(LaserScanError::OutsideLimits {
                    index,
                    range_m,
                    min_range_m,
                    max_range_m,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            min_angle_rad,
            angle_increment_rad,
            time_increment_s,
            max_range_m,
            ranges,
        })
    }

    /// The farthest range the sensor measures (m).
    pub fn max_range_m(&self) -> f32 {
        self.max_range_m
    }

    /// The time from the scan's timestamp to its last ray (s), below 0 for a
    /// sensor that sweeps clockwise: where the caller reads the sensor's pose
    /// at the last ray.
    pub fn sweep_s(&self) -> f64 {
        self.ray(self.ranges.len() - 1).time_s
    }

    /// Ray `index`, which the scan holds.
    fn ray(&self, index: usize) -> Ray {
        let step = index as f64;
        Ray {
            index,
            angle_rad: f64::from(self.min_angle_rad) + step * f64::from(self.angle_increment_rad),
            time_s: f64::from(self.time_increment_s) * step,
            range: self.ranges[index],
        }
    }

    /// Every ray of the scan, in order.
    fn rays(&self) -> impl Iterator<Item = Ray> + '_ {
        (0..self.ranges.len()).map(|index| self.ray(index))
    }

    /// Each ray's hit, the marks left out, in ray order: in the sensor's frame
    /// at its ray's time, and placed in the frame of the sensor's poses
    /// `sensor` where the sensor stood at its ray's time; a ray at whose time
    /// the poses interpolate to no pose places none.
    pub fn placed_hits(&self, sensor: &Sweep<Pose>) -> Vec<PlacedHit> {
        self.rays()
            .filter_map(|ray| {
                let in_sensor = ray.hit()?;
                let placed = self.sensor_at(ray.index, sensor).ok()?.apply(in_sensor);
                Some(PlacedHit { in_sensor, placed })
            })
            .collect()
    }

    /// Where the sensor stood at ray `index`'s time: its poses at the first and
    /// the last ray interpolated by the share `index * (1 / (n - 1))` of the
    /// sweep, 0 for a scan of one ray, as laser_geometry computes it
    /// (laser_geometry.cpp:334, 346).
    fn sensor_at(&self, index: usize, sensor: &Sweep<Pose>) -> Result<Pose, PoseError> {
        let intervals = self.ranges.len() - 1;
        let ranges_norm = if intervals > 0 {
            1.0 / intervals as f64
        } else {
            0.0
        };
        sensor
            .first_ray
            .interpolated(&sensor.last_ray, index as f64 * ranges_norm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::close;

    fn identity() -> Pose {
        Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("the identity")
    }

    fn sweep(first_ray: Pose, last_ray: Pose) -> Sweep<Pose> {
        Sweep {
            first_ray,
            last_ray,
        }
    }

    fn scan(ranges: &[f32]) -> LaserScan {
        LaserScan::parse(-0.5, 0.25, 0.01, 0.1, 10.0, ranges).expect("a scan")
    }

    #[test]
    fn a_scan_wants_rays_increasing_angles_and_ranges_within_its_limits() {
        assert_eq!(
            LaserScan::parse(0.0, 0.1, 0.0, 0.1, 5.0, &[]),
            Err(LaserScanError::NoRays)
        );
        for (min_angle, increment) in [(f32::NAN, 0.1), (0.0, 0.0), (0.0, -0.1)] {
            assert!(
                matches!(
                    LaserScan::parse(min_angle, increment, 0.0, 0.1, 5.0, &[1.0]),
                    Err(LaserScanError::Angles { .. })
                ),
                "{min_angle} {increment}"
            );
        }
        for increment in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(
                matches!(
                    LaserScan::parse(0.0, 0.1, increment, 0.1, 5.0, &[1.0]),
                    Err(LaserScanError::TimeIncrement(_))
                ),
                "{increment}"
            );
        }
        for (min, max) in [
            (-0.1, 5.0),
            (5.0, 5.0),
            (f32::NAN, 5.0),
            (0.1, f32::INFINITY),
            (0.1, 10000.0),
        ] {
            assert!(
                matches!(
                    LaserScan::parse(0.0, 0.1, 0.0, min, max, &[1.0]),
                    Err(LaserScanError::Limits { .. })
                ),
                "{min} {max}"
            );
        }
        for range in [0.05, -1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(
                matches!(
                    LaserScan::parse(0.0, 0.1, 0.0, 0.1, 5.0, &[1.0, range]),
                    Err(LaserScanError::OutsideLimits { index: 1, .. })
                ),
                "{range}"
            );
        }
        assert_eq!(
            LaserScan::parse(0.0, 0.1, 0.0, 0.1, 5.0, &[1.0, 0.05])
                .unwrap_err()
                .to_string(),
            "ray 1 measured 0.05 m, outside the sensor's 0.1 m to 5 m: give 0 for a ray not \
             measured or a surface nearer than min_range_m, and 10000 for a ray that met nothing"
        );
    }

    // Ray i points along min_angle + i x increment at time_increment x i; 0 marks a ray
    // not measured, and a range at max_range or above, the mark 10000 among them, a ray
    // that met nothing; neither has a hit.
    #[test]
    fn each_ray_points_along_its_angle_at_its_time() {
        let scan = scan(&[2.0, 10000.0, 0.0, 0.1, 10.0, 9.999, 25.0, 20000.0]);
        let rays: Vec<Ray> = scan.rays().collect();
        assert_eq!(rays.len(), 8);
        assert_eq!(rays[4].angle_rad, -0.5 + 4.0 * 0.25);
        assert_eq!(rays[4].time_s, f64::from(0.01_f32) * 4.0);
        assert_eq!(scan.sweep_s(), rays[7].time_s);
        assert_eq!(
            rays.iter().map(|ray| ray.range).collect::<Vec<_>>(),
            [
                Range::Hit(2.0),
                Range::NoReturn,
                Range::Unmeasured,
                Range::Hit(0.1),
                Range::NoReturn,
                Range::Hit(9.999),
                Range::NoReturn,
                Range::NoReturn,
            ]
        );
        let hit = rays[0].hit().expect("a hit");
        assert!(close(
            hit,
            [2.0 * 0.5_f64.cos(), -2.0 * 0.5_f64.sin(), 0.0],
            1e-12
        ));
        let marked = [1, 2, 4, 6, 7];
        assert!(marked.iter().all(|&i| rays[i].hit().is_none()));
    }

    // A sensor whose nearest range is 0 reads 0 as the mark of a ray not measured, never as
    // a hit at its own origin.
    #[test]
    fn a_range_of_0_is_a_mark_where_the_nearest_range_is_0() {
        let scan = LaserScan::parse(0.0, 0.1, 0.0, 0.0, 5.0, &[0.0, 0.001]).expect("a scan");
        assert_eq!(
            scan.rays().map(|ray| ray.range).collect::<Vec<_>>(),
            [Range::Unmeasured, Range::Hit(0.001)]
        );
    }

    // A sensor that sweeps clockwise gives its rays in rising angle with a time increment
    // below 0: ray 0 stands at the timestamp, each later ray before the one ahead of it,
    // and the sweep ends before the timestamp, where the caller reads the pose of the last
    // ray.
    #[test]
    fn a_clockwise_sweep_places_each_later_ray_before_the_timestamp() {
        let scan = LaserScan::parse(-0.5, 0.25, -0.01, 0.1, 10.0, &[1.0; 5]).expect("a scan");
        let times: Vec<f64> = scan.rays().map(|ray| ray.time_s).collect();
        assert_eq!(times[0], 0.0);
        assert!(times.windows(2).all(|pair| pair[1] < pair[0]), "{times:?}");
        assert_eq!(scan.sweep_s(), f64::from(-0.01_f32) * 4.0);
    }

    // A sensor that stands still places each hit along its ray; one that
    // turns while it sweeps places each where it stood at that ray's time,
    // and keeps each hit in its own frame along the ray as it measured it.
    #[test]
    fn each_hit_lands_where_the_sensor_stood_at_its_ray() {
        let scan = scan(&[1.0; 5]);
        let along_ray = |index: usize| {
            let angle = -0.5 + index as f64 * 0.25;
            [angle.cos(), angle.sin(), 0.0]
        };
        let still = scan.placed_hits(&sweep(identity(), identity()));
        assert_eq!(still.len(), 5);
        for (index, hit) in still.iter().enumerate() {
            assert!(close(hit.placed, along_ray(index), 1e-12));
            assert!(close(hit.in_sensor, along_ray(index), 1e-12));
        }
        // The sensor turns back by the whole fan while it sweeps, so every ray
        // points along the first ray's direction in the frame of the poses.
        let half = -0.5_f64;
        let turned = Pose::new([0.0; 3], [0.0, 0.0, half.sin(), half.cos()]).expect("a turn");
        for (index, hit) in scan
            .placed_hits(&sweep(identity(), turned))
            .into_iter()
            .enumerate()
        {
            assert!(
                close(hit.placed, [0.5_f64.cos(), -0.5_f64.sin(), 0.0], 1e-12),
                "ray {index}: {hit:?}"
            );
            assert!(
                close(hit.in_sensor, along_ray(index), 1e-12),
                "ray {index}: {hit:?}"
            );
        }
        // The mark of ray 1 places no hit; rays 0 and 2 stand at -0.5 and
        // 0 rad.
        let marked = scan_with_marks().placed_hits(&sweep(identity(), identity()));
        assert_eq!(marked.len(), 2);
        assert!(close(
            marked[0].placed,
            [0.5_f64.cos(), -0.5_f64.sin(), 0.0],
            1e-12
        ));
        assert!(close(marked[1].placed, [3.0, 0.0, 0.0], 1e-12));
    }

    // The sensor moves 1 m along +X while it sweeps six rays, so each ray's
    // pose stands at its share of the sweep along X. laser_geometry's share
    // `i * (1 / 5)` differs from `i / 5` in its last bit at ray 3.
    #[test]
    fn each_ray_stands_at_laser_geometrys_share_of_the_sweep() {
        let scan = scan(&[1.0; 6]);
        let moved = Pose::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let shares: Vec<f64> = (0..6)
            .map(|index| {
                scan.sensor_at(index, &sweep(identity(), moved))
                    .expect("a share of the sweep")
                    .origin()[0]
            })
            .collect();
        let expected: Vec<f64> = (0..6).map(|i| f64::from(i) * (1.0 / 5.0)).collect();
        assert_eq!(shares, expected);
        assert_ne!(3.0 / 5.0, 3.0 * (1.0 / 5.0));
    }

    // A sensor whose heading turns from 2.6 rad to 2.9 rad while it sweeps,
    // past the 120 degrees beyond which tf2 reads a pose's quaternion by its
    // largest diagonal entry, places each hit of its fan of one ray a degree
    // along the ray's angle plus the heading at the ray's share of the sweep.
    #[test]
    fn a_hit_turns_with_a_heading_past_120_degrees() {
        const RAYS: usize = 31;
        let degree = 1.0_f32.to_radians();
        let scan = LaserScan::parse(-0.25, degree, 0.001, 0.1, 10.0, &[1.0; RAYS])
            .expect("a scan of one ray a degree");
        let heading = |yaw: f64| {
            Pose::new([0.0; 3], [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()])
                .expect("a turn about +Z")
        };
        let hits = scan.placed_hits(&sweep(heading(2.6), heading(2.9)));
        assert_eq!(hits.len(), RAYS);
        for (index, hit) in hits.into_iter().enumerate() {
            let share = index as f64 / (RAYS - 1) as f64;
            let angle = f64::from(-0.25_f32) + index as f64 * f64::from(degree);
            let along = angle + 2.6 + 0.3 * share;
            assert!(
                close(hit.placed, [along.cos(), along.sin(), 0.0], 1e-12),
                "ray {index}: {hit:?}"
            );
        }
    }

    fn scan_with_marks() -> LaserScan {
        scan(&[1.0, 10000.0, 3.0])
    }

    // One ray stands at the scan's timestamp: its pose is the first.
    #[test]
    fn a_scan_of_one_ray_places_its_hit_by_the_first_pose() {
        let scan = scan(&[1.0]);
        let moved = Pose::new([5.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let hits = scan.placed_hits(&sweep(identity(), moved));
        assert_eq!(hits.len(), 1);
        assert!(close(
            hits[0].placed,
            [0.5_f64.cos(), -0.5_f64.sin(), 0.0],
            1e-12
        ));
        assert_eq!(scan.sweep_s(), 0.0);
    }
}
