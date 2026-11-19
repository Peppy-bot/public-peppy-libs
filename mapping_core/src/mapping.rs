// slam_toolbox
// Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
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
// This file translates into Rust slam_toolbox 2.10.0's synchronous mapping:
// src/slam_toolbox_sync.cpp (Copyright Work Modifications (c) 2018, Simbe Robotics, Inc., and
// (c) 2019, Steve Macenski), src/slam_toolbox_common.cpp (Copyright Work Modifications (c)
// 2018, Simbe Robotics, Inc., and (c) 2019, Samsung Research America) and src/laser_utils.cpp
// (Copyright (c) 2019, Samsung Research America), with karto's parameter setters of
// lib/karto_sdk/src/Mapper.cpp (Copyright 2010 SRI International).
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

//! A room mapped from scans, as slam_toolbox 2.10.0 maps in its synchronous mode
//! (src/slam_toolbox_sync.cpp: `laserCallback`; src/slam_toolbox_common.cpp:
//! `shouldProcessScan`, `getLocalizedRangeScan`, `addScanImpl` with `PROCESS`;
//! src/laser_utils.cpp: `makeLaser`): the laser made of the first scan, each scan gated
//! by its time and the base's travel since the last scan that passed, and karto's mapper
//! processing the scans that pass, at their odometric poses.

use crate::geometry::{Pose2, normalize_angle};
use crate::localization::{
    Localization, LocalizationError, LocalizationParameters, compose, in_half_open_turn, inverse,
};
use crate::localized_scan::{Laser, LocalizedScan, ScanError, ScanId};
use crate::map_pose::{Deviations, MapPose};
use crate::mapper::{KartoParameters, Mapper};
use crate::occupancy_grid::{GridError, GridExtent, GridParameters, OccupancyGrid};
use crate::placed_scan::PlacedScan;
use crate::saved_map::RoomMap;
use crate::scan_matcher::{MatchParameters, MatcherError, MatcherSizes};
use control_core::planar_pose::PlanarPose;
use frame_tree::arithmetic::{quaternion_of_yaw, yaw_of_quaternion};
use serde::Deserialize;
use thiserror::Error;

/// The parameters of slam_toolbox's mapping, by the names of its node. Each is required.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappingParameters {
    /// Every `throttle_scans`-th scan is considered, from 1.
    pub throttle_scans: u32,
    /// The least time (s) between the scans that pass.
    pub minimum_time_interval: f64,
    /// Whether a scan passes when the base moved `minimum_travel_distance` or turned
    /// `minimum_travel_heading`, or, when false, on 80% of the squared distance alone.
    pub check_min_dist_and_heading_precisely: bool,
    /// The range limits (m) of the laser's readings within the scan's own.
    pub min_laser_range: f64,
    pub max_laser_range: f64,
    /// The cell of the map's grid (m).
    pub resolution: f64,
    pub use_scan_barycenter: bool,
    pub minimum_travel_distance: f64,
    pub minimum_travel_heading: f64,
    pub scan_buffer_size: u32,
    pub scan_buffer_maximum_scan_distance: f64,
    pub link_match_minimum_response_fine: f64,
    pub link_scan_maximum_distance: f64,
    pub loop_search_maximum_distance: f64,
    pub do_loop_closing: bool,
    pub loop_match_minimum_chain_size: u32,
    pub loop_match_maximum_variance_coarse: f64,
    pub loop_match_minimum_response_coarse: f64,
    pub loop_match_minimum_response_fine: f64,
    pub correlation_search_space_dimension: f64,
    pub correlation_search_space_resolution: f64,
    pub correlation_search_space_smear_deviation: f64,
    pub loop_search_space_dimension: f64,
    pub loop_search_space_resolution: f64,
    pub loop_search_space_smear_deviation: f64,
    pub distance_variance_penalty: f64,
    pub angle_variance_penalty: f64,
    pub fine_search_angle_offset: f64,
    pub coarse_search_angle_offset: f64,
    pub coarse_angle_resolution: f64,
    pub minimum_angle_penalty: f64,
    pub minimum_distance_penalty: f64,
    pub use_response_expansion: bool,
    pub min_pass_through: u32,
    pub occupancy_threshold: f64,
}

/// Why parameters make no mapping.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ParameterError {
    #[error("{name} is {value}, and every parameter of the mapping is finite")]
    NotFinite { name: &'static str, value: f64 },
    #[error("{name} is {value}, and it must be above 0")]
    NotPositive { name: &'static str, value: f64 },
    #[error("{name} is 0, and it must be 1 or more")]
    ZeroCount { name: &'static str },
    #[error("the {search} search space: {error}")]
    SearchSpace {
        search: &'static str,
        error: MatcherError,
    },
}

/// Why mapping ended with no map, or with a map and no localization.
#[derive(Debug, Error)]
pub enum FinishError {
    #[error(transparent)]
    Grid(#[from] GridError),
    /// The map stands, and the localization parameters keep no pose on it.
    #[error("the map stands, and its localization does not start: {error}")]
    Localization {
        map: Box<RoomMap>,
        #[source]
        error: LocalizationError,
    },
}

/// Why a scan was not added.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum MappingError {
    #[error(transparent)]
    Scan(#[from] ScanError),
    #[error(transparent)]
    Matcher(#[from] MatcherError),
}

impl MappingParameters {
    /// The parameters checked, with karto's values: slam_toolbox squares the variances
    /// it sets.
    pub(crate) fn karto(&self) -> Result<KartoParameters, ParameterError> {
        let finite = [
            ("minimum_time_interval", self.minimum_time_interval),
            ("min_laser_range", self.min_laser_range),
            ("max_laser_range", self.max_laser_range),
            ("resolution", self.resolution),
            ("minimum_travel_distance", self.minimum_travel_distance),
            ("minimum_travel_heading", self.minimum_travel_heading),
            (
                "scan_buffer_maximum_scan_distance",
                self.scan_buffer_maximum_scan_distance,
            ),
            (
                "link_match_minimum_response_fine",
                self.link_match_minimum_response_fine,
            ),
            (
                "link_scan_maximum_distance",
                self.link_scan_maximum_distance,
            ),
            (
                "loop_search_maximum_distance",
                self.loop_search_maximum_distance,
            ),
            (
                "loop_match_maximum_variance_coarse",
                self.loop_match_maximum_variance_coarse,
            ),
            (
                "loop_match_minimum_response_coarse",
                self.loop_match_minimum_response_coarse,
            ),
            (
                "loop_match_minimum_response_fine",
                self.loop_match_minimum_response_fine,
            ),
            ("distance_variance_penalty", self.distance_variance_penalty),
            ("angle_variance_penalty", self.angle_variance_penalty),
            ("fine_search_angle_offset", self.fine_search_angle_offset),
            (
                "coarse_search_angle_offset",
                self.coarse_search_angle_offset,
            ),
            ("coarse_angle_resolution", self.coarse_angle_resolution),
            ("minimum_angle_penalty", self.minimum_angle_penalty),
            ("minimum_distance_penalty", self.minimum_distance_penalty),
            ("occupancy_threshold", self.occupancy_threshold),
        ];
        if let Some((name, value)) = finite.into_iter().find(|(_, value)| !value.is_finite()) {
            return Err(ParameterError::NotFinite { name, value });
        }
        let positive = [
            ("resolution", self.resolution),
            (
                "scan_buffer_maximum_scan_distance",
                self.scan_buffer_maximum_scan_distance,
            ),
            ("distance_variance_penalty", self.distance_variance_penalty),
            ("angle_variance_penalty", self.angle_variance_penalty),
            ("fine_search_angle_offset", self.fine_search_angle_offset),
            (
                "coarse_search_angle_offset",
                self.coarse_search_angle_offset,
            ),
            ("coarse_angle_resolution", self.coarse_angle_resolution),
        ];
        if let Some((name, value)) = positive.into_iter().find(|(_, value)| *value <= 0.0) {
            return Err(ParameterError::NotPositive { name, value });
        }
        let counts = [
            ("throttle_scans", self.throttle_scans),
            ("scan_buffer_size", self.scan_buffer_size),
        ];
        if let Some((name, _)) = counts.into_iter().find(|(_, count)| *count == 0) {
            return Err(ParameterError::ZeroCount { name });
        }
        let search_space = |search, dimension, resolution, smear_deviation| {
            MatcherSizes::parse(dimension, resolution, smear_deviation)
                .map_err(|error| ParameterError::SearchSpace { search, error })
        };
        Ok(KartoParameters {
            use_scan_barycenter: self.use_scan_barycenter,
            minimum_travel_distance: self.minimum_travel_distance,
            minimum_travel_heading: self.minimum_travel_heading,
            scan_buffer_size: self.scan_buffer_size,
            scan_buffer_maximum_scan_distance: self.scan_buffer_maximum_scan_distance,
            link_match_minimum_response_fine: self.link_match_minimum_response_fine,
            link_scan_maximum_distance: self.link_scan_maximum_distance,
            loop_search_maximum_distance: self.loop_search_maximum_distance,
            do_loop_closing: self.do_loop_closing,
            loop_match_minimum_chain_size: self.loop_match_minimum_chain_size,
            loop_match_maximum_variance_coarse: self.loop_match_maximum_variance_coarse.powi(2),
            loop_match_minimum_response_coarse: self.loop_match_minimum_response_coarse,
            loop_match_minimum_response_fine: self.loop_match_minimum_response_fine,
            correlation_search_space: search_space(
                "correlation",
                self.correlation_search_space_dimension,
                self.correlation_search_space_resolution,
                self.correlation_search_space_smear_deviation,
            )?,
            loop_search_space: search_space(
                "loop",
                self.loop_search_space_dimension,
                self.loop_search_space_resolution,
                self.loop_search_space_smear_deviation,
            )?,
            match_parameters: MatchParameters {
                coarse_angle_resolution: self.coarse_angle_resolution,
                coarse_search_angle_offset: self.coarse_search_angle_offset,
                fine_search_angle_offset: self.fine_search_angle_offset,
                distance_variance_penalty: self.distance_variance_penalty.powi(2),
                angle_variance_penalty: self.angle_variance_penalty.powi(2),
                minimum_distance_penalty: self.minimum_distance_penalty,
                minimum_angle_penalty: self.minimum_angle_penalty,
                use_response_expansion: self.use_response_expansion,
            },
        })
    }
}

/// slam_toolbox's `shouldProcessScan`, whose state is the function's statics: the first
/// scan passes; then a scan passes when it is a throttled one, the minimum time interval
/// has passed since the last scan that passed, it is the fifth scan or later, and the
/// base traveled far enough, by odometry.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ScanGate {
    first_measurement: bool,
    last_pose: Pose2,
    last_scan_time_ns: i64,
    scan_counter: u64,
    throttle_scans: u64,
    minimum_time_interval_ns: i64,
    min_dist2: f64,
    min_rotation: f64,
    check_min_dist_and_heading_precisely: bool,
}

impl ScanGate {
    pub fn new(parameters: &MappingParameters) -> Self {
        Self {
            first_measurement: true,
            last_pose: Pose2::default(),
            last_scan_time_ns: 0,
            scan_counter: 0,
            throttle_scans: u64::from(parameters.throttle_scans),
            minimum_time_interval_ns: (parameters.minimum_time_interval * 1e9) as i64,
            min_dist2: parameters.minimum_travel_distance * parameters.minimum_travel_distance,
            min_rotation: parameters.minimum_travel_heading,
            check_min_dist_and_heading_precisely: parameters.check_min_dist_and_heading_precisely,
        }
    }

    /// Whether the scan at `stamp_ns`, with the base at `pose` by odometry, passes.
    pub fn passes(&mut self, stamp_ns: i64, pose: Pose2) -> bool {
        self.scan_counter += 1;
        if self.first_measurement {
            self.last_scan_time_ns = stamp_ns;
            self.last_pose = pose;
            self.first_measurement = false;
            return true;
        }
        if !self.scan_counter.is_multiple_of(self.throttle_scans) {
            return false;
        }
        if stamp_ns - self.last_scan_time_ns < self.minimum_time_interval_ns {
            return false;
        }
        if self.scan_counter < STABILIZATION_SCANS {
            return false;
        }
        let dist2 = self.last_pose.squared_distance(pose);
        if self.check_min_dist_and_heading_precisely {
            let heading_diff = normalize_angle(pose.yaw - self.last_pose.yaw).abs();
            if dist2 < self.min_dist2 && heading_diff < self.min_rotation {
                return false;
            }
        } else if dist2 < TRAVEL_CORRECTION * self.min_dist2 {
            return false;
        }
        self.last_pose = pose;
        self.last_scan_time_ns = stamp_ns;
        true
    }
}

/// The scans that `shouldProcessScan` counts before it passes one after the first, "for
/// initial stabilization".
const STABILIZATION_SCANS: u64 = 5;
/// The share of the squared travel distance that `shouldProcessScan` takes as enough,
/// "within 10% for correction error".
const TRAVEL_CORRECTION: f64 = 0.8;

/// The time of a stamp (s), as `rclcpp::Time::nanoseconds() / 1e9`.
pub(crate) fn seconds(stamp_ns: i64) -> f64 {
    stamp_ns as f64 / 1e9
}

/// The heading that tf2 gives back for the quaternion that `setRPY(0, 0, yaw)` makes:
/// `getYaw` of it, in [-pi, pi].
pub(crate) fn tf2_yaw(yaw: f64) -> f64 {
    yaw_of_quaternion(quaternion_of_yaw(yaw))
}

/// A room mapped from a sequence of scans: the laser and karto's mapper, made of the
/// first scan, and the gate that each scan passes first. The map frame is the odometry
/// frame where mapping started, as karto keeps the odometric pose of its first scan
/// (`Mapper::Process`): each scan's odometric pose is its odometry.
#[derive(Debug, Clone, PartialEq)]
pub struct Mapping {
    parameters: MappingParameters,
    karto: KartoParameters,
    gate: ScanGate,
    mapper: Option<Mapper>,
    /// The scans the mapping read: each scan `add` took, whether or not it passed the
    /// gate and joined the graph. It stays at `u32::MAX` once there.
    scans: u32,
    /// From the odometry frame to the map, at the last scan the mapping added.
    map_from_odometry: Pose2,
}

impl Mapping {
    /// Mapping in the odometry frame of the scans it takes.
    pub fn start(parameters: MappingParameters) -> Result<Self, ParameterError> {
        Ok(Self {
            parameters,
            karto: parameters.karto()?,
            gate: ScanGate::new(&parameters),
            mapper: None,
            scans: 0,
            map_from_odometry: Pose2::new(0.0, 0.0, 0.0),
        })
    }

    /// `scan`: its pose in the map when it passes the gate and the base moved enough for
    /// karto, so it is added to the graph, a scan of the map. The base is localized while
    /// mapping runs. Every scan takes the angles of the first; a scan with another number
    /// of rays is refused and changes nothing.
    pub fn add(&mut self, scan: &PlacedScan) -> Result<Option<MapPose>, MappingError> {
        let mapper = match self.mapper.take() {
            Some(mapper) => mapper,
            None => Mapper::new(
                self.karto,
                Laser::of(
                    &scan.geometry,
                    self.parameters.min_laser_range,
                    self.parameters.max_laser_range,
                ),
            )?,
        };
        let mapper = self.mapper.insert(mapper);
        mapper.laser().validate(&scan.geometry)?;
        self.scans = self.scans.saturating_add(1);
        let odometry = scan.odometry;
        if !self.gate.passes(scan.stamp_ns, odometry) {
            return Ok(None);
        }
        let pose = Pose2::new(odometry.x, odometry.y, tf2_yaw(odometry.yaw));
        let localized = LocalizedScan::new(
            mapper.laser(),
            scan.beams.clone(),
            seconds(scan.stamp_ns),
            pose,
        );
        let Some(processed) = mapper.process(localized, false)? else {
            return Ok(None);
        };
        let corrected = mapper
            .scans()
            .get(&processed.id)
            .map(LocalizedScan::corrected_pose)
            .expect("the mapper holds the scan it processed");
        self.map_from_odometry = compose(corrected, inverse(scan.odometry));
        Ok(Some(MapPose {
            stamp_ns: scan.stamp_ns,
            pose: in_half_open_turn(corrected).into(),
            deviations: Deviations::of_covariance(&processed.covariance),
            localized: true,
        }))
    }

    /// The number of scans the mapping read, whether or not each joined the graph: the
    /// `scans` of room_map's map object.
    pub fn scans(&self) -> u32 {
        self.scans
    }

    /// The extent of the map's grid as it stands, as karto's `ComputeDimensions` gives it
    /// of the scans' points; none before a scan was added.
    pub fn extent(&self) -> Result<Option<GridExtent>, GridError> {
        let Some(mapper) = self.mapper.as_ref() else {
            return Ok(None);
        };
        GridExtent::of_scans(mapper.scans().values(), self.parameters.resolution)
    }

    /// From the odometry frame to the map, at the last scan the mapping added.
    pub fn map_from_odometry(&self) -> PlanarPose {
        self.map_from_odometry.into()
    }

    /// The parameters of the map's grid.
    fn grid_parameters(&self) -> GridParameters {
        GridParameters {
            resolution: self.parameters.resolution,
            min_pass_through: self.parameters.min_pass_through,
            occupancy_threshold: self.parameters.occupancy_threshold,
        }
    }

    /// The grid of the map as it stands, the grid of the map that `finish` gives; none
    /// before a scan was added.
    pub fn grid(&self) -> Result<Option<OccupancyGrid>, GridError> {
        let Some(mapper) = self.mapper.as_ref() else {
            return Ok(None);
        };
        OccupancyGrid::from_scans(
            mapper.scans().values(),
            mapper.laser(),
            self.grid_parameters(),
        )
    }

    /// The map as it stands, whose mapping started at `mapping_start_unix_ns`; none before a
    /// scan was added.
    pub(crate) fn map(&self, mapping_start_unix_ns: i64) -> Result<Option<RoomMap>, GridError> {
        let Some(mapper) = self.mapper.as_ref() else {
            return Ok(None);
        };
        RoomMap::new(
            mapper.state(),
            self.grid_parameters(),
            self.scans,
            mapping_start_unix_ns,
        )
    }

    /// The end of mapping: the map, whose mapping started at `mapping_start_unix_ns` on the
    /// wall clock, and the localization that keeps the base's pose on the graph the
    /// mapping built, from the mapping's estimate. None before a scan was added. When
    /// `localization` keeps no pose, the error holds the map.
    pub fn finish(
        self,
        mapping_start_unix_ns: i64,
        localization: LocalizationParameters,
    ) -> Result<Option<(RoomMap, Localization)>, FinishError> {
        let Some(map) = self.map(mapping_start_unix_ns)? else {
            return Ok(None);
        };
        match Localization::continuing(&map.state, localization, self.map_from_odometry) {
            Ok(kept) => Ok(Some((map, kept))),
            Err(error) => Err(FinishError::Localization {
                map: Box::new(map),
                error,
            }),
        }
    }

    /// The corrected pose of each scan of the map, by id.
    pub fn poses(&self) -> Vec<(ScanId, PlanarPose)> {
        self.mapper
            .iter()
            .flat_map(|mapper| mapper.scans())
            .map(|(&id, scan)| (id, scan.corrected_pose().into()))
            .collect()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn the_gate_passes_the_first_scan_and_then_travel_after_the_fifth() {
        let mut gate = ScanGate::new(&parameters());
        let at = |x: f64| Pose2::new(x, 0.0, 0.0);
        assert!(gate.passes(0, at(0.0)));
        assert!(!gate.passes(1_000_000_000, at(1.0)), "the second scan");
        assert!(!gate.passes(2_000_000_000, at(1.0)), "the third scan");
        assert!(!gate.passes(3_000_000_000, at(1.0)), "the fourth scan");
        assert!(gate.passes(4_000_000_000, at(1.0)), "the fifth scan");
        assert!(!gate.passes(4_100_000_000, at(2.0)), "too soon");
        assert!(!gate.passes(5_000_000_000, at(1.3)), "too near");
        assert!(gate.passes(6_000_000_000, at(1.5)));
    }

    #[test]
    fn tf2_gives_the_heading_back_in_minus_pi_to_pi() {
        assert!((tf2_yaw(0.3) - 0.3).abs() < 1e-15);
        assert!((tf2_yaw(2.0 * std::f64::consts::PI + 0.3) - 0.3).abs() < 1e-14);
    }

    pub(crate) fn parameters() -> MappingParameters {
        crate::slam_toolbox_parameters::mapper_params_online_sync(0.1, 8.0)
    }

    #[test]
    fn a_zero_count_is_refused_by_name() {
        let parameters = MappingParameters {
            scan_buffer_size: 0,
            ..parameters()
        };
        assert_eq!(
            Mapping::start(parameters).map(|_| ()),
            Err(ParameterError::ZeroCount {
                name: "scan_buffer_size"
            })
        );
    }
}
