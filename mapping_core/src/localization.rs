// slam_toolbox
// Copyright Work Modifications (c) 2019, Steve Macenski
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
// This file translates into Rust slam_toolbox 2.10.0's localization mode:
// src/slam_toolbox_localization.cpp (Copyright Work Modifications (c) 2019, Steve Macenski) and
// src/slam_toolbox_common.cpp (Copyright Work Modifications (c) 2018, Simbe Robotics, Inc., and
// (c) 2019, Samsung Research America), with karto's lib/karto_sdk/src/Mapper.cpp (Copyright
// 2010 SRI International).
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

//! The base's pose in a map, as slam_toolbox 2.10.0 keeps it in its localization mode
//! (src/slam_toolbox_localization.cpp: `laserCallback`, `addScan`, `localizePoseCallback`;
//! src/slam_toolbox_common.cpp: `loadSerializedPoseGraph`, `setTransformFromPoses`,
//! `getLocalizedRangeScan`; lib/karto_sdk/src/Mapper.cpp: `ProcessAgainstNodesNearBy`,
//! `ProcessLocalization`): the map's graph and scans, its optimizer solved once; a scan
//! after a start pose placed there and matched against the map's scan nearest it; each
//! later scan carried into the map by the motion that took the base's odometry to that
//! scan's pose, and processed as karto processes a scan in a loaded map: added to the
//! graph, linked, its loops closed, and the oldest added scan beyond the buffer dropped.
//!
//! A localization starts from a pose with a search radius and a search angle
//! (`set_initial_pose`): the scan is matched within the search as karto matches a loop,
//! and the match is taken when its standard deviations lie within the matching limits. It
//! also goes on from a mapping's estimate on the graph the mapping built. The base is
//! localized at a scan while the standard deviations of its match lie within the matching
//! limits.

use crate::geometry::{Pose2, normalize_angle};
use crate::localized_scan::{Laser, LocalizedScan, ScanError, corrected_at};
use crate::map_pose::{Deviations, LocalizationLimits, MapPose};
use crate::mapper::{Mapper, MapperState, Processed};
use crate::mapping::{MappingError, MappingParameters, ParameterError, ScanGate, seconds, tf2_yaw};
use crate::placed_scan::PlacedScan;
use crate::saved_map::RoomMap;
use crate::scan_matcher::MatcherError;
use camera_core::pose::PlanarPose;
use control_core::positive_finite::{NotPositiveFinite, PositiveFinite};
use std::f64::consts::PI;
use thiserror::Error;

/// The parameters of a localization: slam_toolbox's, by the names of its node, and the
/// matching limits, within which the standard deviations of a match hold the base
/// localized.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalizationParameters {
    pub slam_toolbox: MappingParameters,
    pub matching_limits: LocalizationLimits,
}

/// Why parameters or a map make no localization.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum LocalizationError {
    #[error(transparent)]
    Parameters(#[from] ParameterError),
    #[error(transparent)]
    Matcher(#[from] MatcherError),
}

/// Where the robot frame stands in a map, searched within a radius and an angle of it, as
/// `room_map`'s `set_initial_pose` gives it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitialPose {
    pose: Pose2,
    search_radius_m: PositiveFinite,
    search_angle_rad: PositiveFinite,
}

/// Why a search radius and angle make no initial pose.
#[derive(Debug, Clone, Copy, Error)]
pub enum InitialPoseError {
    #[error("search_radius_m: {0}")]
    SearchRadius(NotPositiveFinite),
    #[error("search_angle_rad: {0}")]
    SearchAngle(NotPositiveFinite),
}

impl InitialPose {
    pub fn parse(
        pose: PlanarPose,
        search_radius_m: f64,
        search_angle_rad: f64,
    ) -> Result<Self, InitialPoseError> {
        Ok(Self {
            pose: pose.into(),
            search_radius_m: PositiveFinite::parse(search_radius_m)
                .map_err(InitialPoseError::SearchRadius)?,
            search_angle_rad: PositiveFinite::parse(search_angle_rad)
                .map_err(InitialPoseError::SearchAngle)?,
        })
    }

    /// Whether `pose` lies within the search of `center`, this pose carried to it.
    fn holds(&self, center: Pose2, pose: Pose2) -> bool {
        let search_angle = self.search_angle_rad.get();
        (pose.x - center.x).hypot(pose.y - center.y) <= self.search_radius_m.get()
            && (search_angle >= PI || normalize_angle(pose.yaw - center.yaw).abs() <= search_angle)
    }
}

/// Why a scan did not start a localization at an initial pose.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum InitialPoseFailure {
    /// No match within the search has standard deviations within the matching limits. `best`
    /// is the match within the search, not localized, when the scan matched within the
    /// search.
    #[error("no match of the scan within the search lies within the matching limits")]
    NoMatch { best: Option<MapPose> },
    #[error(transparent)]
    Localization(#[from] LocalizationError),
    #[error(transparent)]
    Scan(#[from] ScanError),
}

/// The smear deviations of the sequential matcher's kernel, and the coarse angle steps,
/// by which its reach about a pose passes its search space and its coarse angle offset.
const REACH_STEPS: f64 = 3.0;

/// How far past an initial pose's search the matcher reaches, in position (m) and in
/// yaw (rad): a true pose farther than that past the search gives no match.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatcherReach {
    pub position_m: f64,
    pub yaw_rad: f64,
}

impl MappingParameters {
    /// The matcher's reach past an initial pose's search: in position, half the
    /// correlation search space and three of its smear deviations; in yaw, the coarse
    /// angle offset and three coarse angle steps.
    pub fn matcher_reach(&self) -> MatcherReach {
        MatcherReach {
            position_m: 0.5 * self.correlation_search_space_dimension
                + REACH_STEPS * self.correlation_search_space_smear_deviation,
            yaw_rad: self.coarse_search_angle_offset + REACH_STEPS * self.coarse_angle_resolution,
        }
    }
}

/// A rigid motion of the floor as a pose: a turn by `yaw`, then a translation.
pub(crate) fn compose(a: Pose2, b: Pose2) -> Pose2 {
    let (sin, cos) = a.yaw.sin_cos();
    Pose2::new(
        a.x + cos * b.x - sin * b.y,
        a.y + sin * b.x + cos * b.y,
        tf2_yaw(a.yaw + b.yaw),
    )
}

pub(crate) fn inverse(a: Pose2) -> Pose2 {
    let (sin, cos) = a.yaw.sin_cos();
    Pose2::new(
        -cos * a.x - sin * a.y,
        sin * a.x - cos * a.y,
        tf2_yaw(-a.yaw),
    )
}

/// `pose` with its heading in (-pi, pi], as `room_map` carries a heading.
pub(crate) fn in_half_open_turn(pose: Pose2) -> Pose2 {
    let yaw = normalize_angle(pose.yaw);
    Pose2::new(pose.x, pose.y, if yaw == -PI { PI } else { yaw })
}

/// How the next scan that passes the gate is placed in the map.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Placement {
    /// At the pose that the transform from the odometry frame to the map carries its
    /// odometry to, matched against the map's scan nearest it, as after a start pose.
    NearBy,
    /// At the pose that slam_toolbox's reprocessing transform carries its odometry to,
    /// corrected by the last scan's correction and matched against the running buffer.
    Carried,
}

/// The base kept in a map.
#[derive(Debug, Clone, PartialEq)]
pub struct Localization {
    parameters: LocalizationParameters,
    gate: ScanGate,
    mapper: Mapper,
    placement: Placement,
    /// slam_toolbox's reprocessing transform: from the odometry frame to the frame of the
    /// odometric poses that karto holds.
    reprocessing: Pose2,
    /// From the odometry frame to the map, at the last scan the localization added.
    map_from_odometry: Pose2,
}

impl Localization {
    /// A localization on the graph and scans of `state`, whose next scan that passes the
    /// gate is placed where `map_from_odometry` carries its odometry, and matched against
    /// the map's scan nearest it.
    pub(crate) fn continuing(
        state: &MapperState,
        parameters: LocalizationParameters,
        map_from_odometry: Pose2,
    ) -> Result<Self, LocalizationError> {
        let karto = parameters.slam_toolbox.karto()?;
        Ok(Self {
            parameters,
            gate: ScanGate::new(&parameters.slam_toolbox),
            mapper: Mapper::restore(karto, state)?,
            placement: Placement::NearBy,
            reprocessing: map_from_odometry,
            map_from_odometry,
        })
    }

    /// The base at `estimate` in `map` when its odometry read `odometry`: the next scan
    /// that passes the gate is placed where the odometry carries the estimate, and matched
    /// against the map's scan nearest it, as slam_toolbox starts from an initial pose
    /// (`localizePoseCallback`).
    pub fn from_estimate(
        map: &RoomMap,
        parameters: LocalizationParameters,
        estimate: PlanarPose,
        odometry: PlanarPose,
    ) -> Result<Self, LocalizationError> {
        Self::continuing(
            &map.state,
            parameters,
            compose(estimate.into(), inverse(odometry.into())),
        )
    }

    /// `set_initial_pose`: the base at `initial` in `map` when the base's odometry read
    /// `odometry_at_call`, carried by the odometry to `scan`. The scan is matched within the
    /// search, and the localization starts at the pose of the match when it lies within the
    /// search and its standard deviations within the matching limits.
    pub fn from_initial_pose(
        map: &RoomMap,
        parameters: LocalizationParameters,
        initial: &InitialPose,
        odometry_at_call: PlanarPose,
        scan: &PlacedScan,
    ) -> Result<(Self, MapPose), InitialPoseFailure> {
        let carried = compose(
            compose(initial.pose, inverse(odometry_at_call.into())),
            scan.odometry,
        );
        let mut localization = Self::continuing(
            &map.state,
            parameters,
            compose(carried, inverse(scan.odometry)),
        )?;
        let laser = localization.checked_laser(scan)?;
        let placed =
            LocalizedScan::new(&laser, scan.beams.clone(), seconds(scan.stamp_ns), carried);
        let found = localization
            .mapper
            .match_within(
                &placed,
                initial.search_radius_m.get(),
                initial.search_angle_rad.get(),
            )
            .map_err(LocalizationError::from)?;
        let matched = corrected_at(found.mean);
        if !initial.holds(carried, matched) {
            return Err(InitialPoseFailure::NoMatch { best: None });
        }
        let passed = localization.gate.passes(scan.stamp_ns, scan.odometry);
        assert!(passed, "a localization's fresh gate passes its first scan");
        let localized = localization
            .place_near_by(scan, &laser, matched)
            .map_err(LocalizationError::from)?;
        if !(localized.localized && initial.holds(carried, localized.pose.into())) {
            return Err(InitialPoseFailure::NoMatch {
                best: Some(MapPose {
                    localized: false,
                    ..localized
                }),
            });
        }
        Ok((localization, localized))
    }

    /// `scan`: the base's pose in the map, when the scan passes the gate and the base moved
    /// enough for karto. A scan must read as the map's scans do: with their angles and
    /// within their range limits.
    pub fn add(&mut self, scan: &PlacedScan) -> Result<Option<MapPose>, MappingError> {
        let laser = self.checked_laser(scan)?;
        if !self.gate.passes(scan.stamp_ns, scan.odometry) {
            return Ok(None);
        }
        match self.placement {
            Placement::NearBy => {
                let pose = compose(self.map_from_odometry, scan.odometry);
                Ok(Some(self.place_near_by(scan, &laser, pose)?))
            }
            Placement::Carried => {
                let pose = compose(self.reprocessing, scan.odometry);
                let localized =
                    LocalizedScan::new(&laser, scan.beams.clone(), seconds(scan.stamp_ns), pose);
                let Some(processed) = self.mapper.process(localized, true)? else {
                    return Ok(None);
                };
                Ok(Some(self.answer(scan, processed)))
            }
        }
    }

    /// From the odometry frame to the map, at the last scan the localization added: the
    /// transform slam_toolbox publishes from `map` to `odom`.
    pub fn map_from_odometry(&self) -> PlanarPose {
        self.map_from_odometry.into()
    }

    /// The map's laser, when `scan` reads as the map's scans do.
    fn checked_laser(&self, scan: &PlacedScan) -> Result<Laser, ScanError> {
        let laser = *self.mapper.laser();
        let slam_toolbox = &self.parameters.slam_toolbox;
        if Laser::of(
            &scan.geometry,
            slam_toolbox.min_laser_range,
            slam_toolbox.max_laser_range,
        ) != laser
        {
            return Err(ScanError::NotTheMapsLaser);
        }
        laser.validate(&scan.geometry)?;
        Ok(laser)
    }

    /// `ProcessAgainstNodesNearBy` of `scan` placed at `pose` in the map; the transform
    /// from the odometry frame to the map that it gives becomes the reprocessing transform.
    fn place_near_by(
        &mut self,
        scan: &PlacedScan,
        laser: &Laser,
        pose: Pose2,
    ) -> Result<MapPose, MatcherError> {
        let placed = LocalizedScan::with_poses(
            laser,
            scan.beams.clone(),
            seconds(scan.stamp_ns),
            pose,
            pose,
        );
        let processed = self.mapper.process_against_nodes_near_by(placed)?;
        let localized = self.answer(scan, processed);
        self.reprocessing = self.map_from_odometry;
        self.placement = Placement::Carried;
        Ok(localized)
    }

    /// The pose of a processed scan, its deviations and whether they lie within the
    /// limits; the transform from the odometry frame to the map moves to the scan.
    fn answer(&mut self, scan: &PlacedScan, processed: Processed) -> MapPose {
        let pose = self
            .mapper
            .scans()
            .get(&processed.id)
            .map(LocalizedScan::corrected_pose)
            .expect("the mapper holds the scan it processed");
        self.map_from_odometry = compose(pose, inverse(scan.odometry));
        let deviations = Deviations::of_covariance(&processed.covariance);
        MapPose {
            stamp_ns: scan.stamp_ns,
            pose: in_half_open_turn(pose).into(),
            deviations,
            localized: deviations.within(&self.parameters.matching_limits),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pose_composed_with_its_inverse_is_the_origin() {
        let pose = Pose2::new(1.5, -0.5, 2.5);
        let origin = compose(pose, inverse(pose));
        assert!(origin.x.abs() < 1e-12 && origin.y.abs() < 1e-12 && origin.yaw.abs() < 1e-12);
        let moved = compose(
            Pose2::new(1.0, 0.0, std::f64::consts::FRAC_PI_2),
            Pose2::new(1.0, 0.0, 0.0),
        );
        assert!((moved.x - 1.0).abs() < 1e-12 && (moved.y - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_heading_of_minus_pi_is_carried_as_pi() {
        assert_eq!(in_half_open_turn(Pose2::new(0.0, 0.0, -PI)).yaw, PI);
        assert_eq!(in_half_open_turn(Pose2::new(0.0, 0.0, PI)).yaw, PI);
    }

    #[test]
    fn a_search_angle_of_pi_holds_every_heading() {
        let initial = InitialPose::parse(PlanarPose::IDENTITY, 0.5, PI).expect("valid");
        let center = Pose2::new(0.0, 0.0, 0.0);
        assert!(initial.holds(center, Pose2::new(0.1, 0.0, 3.0)));
        assert!(!initial.holds(center, Pose2::new(0.6, 0.0, 0.0)));
    }
}
