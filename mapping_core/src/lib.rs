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

//! Room maps from 2-D scans, as Nav2 makes them, and the pose of a base in a map.
//!
//! - [`LaserProjection`]: depth points to a [`ProjectedScan`], the scan that `lidar_2d`'s
//!   `scan_stream` carries, as pointcloud_to_laserscan 2.1.1 projects a cloud: the points
//!   inside a height band and range limits, binned by bearing, the nearest point of each
//!   bin, and the no-return mark in a bin with no point. After the clouds of the ROS reference
//!   harness, the scans hold pointcloud_to_laserscan's ranges bit for bit on amd64
//!   (`tests/ros_reference_laser_projection.rs`).
//! - [`PlacedScan`]: a scan of `camera_core::laser_scan` with each ray placed where the
//!   sensor stood at the ray's time, in the base's frame at the scan's timestamp.
//! - [`Mapping`]: a room mapped from scans and odometry as slam_toolbox 2.10.0 maps in its
//!   synchronous mode, its map frame the robot frame where mapping started: each scan that
//!   passes slam_toolbox's gate is matched against a running buffer of recent scans with
//!   karto's correlative matcher, linked to the scan before it, to the buffer and to the
//!   near chains of scans it matches, and each loop it closes against a chain of scans
//!   nearby closes, by optimizing the pose graph with Ceres's Levenberg-Marquardt
//!   schedule as slam_toolbox configures it. Its [`RoomMap`] is the pose graph with its
//!   scans and an [`OccupancyGrid`] made of them as karto makes it.
//! - [`RoomMap::save`] and [`RoomMap::load`]: a map as a directory of the pose graph, a
//!   description, and the grid as Nav2's map saver writes a trinary map.
//! - [`Localization`]: the base's pose in a map, kept as slam_toolbox keeps it in its
//!   localization mode, from an [`InitialPose`] and its search, from an estimate, or from
//!   a mapping's estimate. Each [`MapPose`] carries the standard deviations of its match,
//!   and the base is localized while they lie within its matching limits
//!   ([`LocalizationLimits`]).
//! - [`GlobalLocalization`]: the base found in a map from no known pose, as Nav2 1.5.2's
//!   AMCL localizes globally once `reinitialize_global_localization` starts it:
//!   `max_particles` particles spread over the free cells of the map, weighed by AMCL's
//!   likelihood field model, moved by its differential or omnidirectional motion model and
//!   resampled by KLD sampling, every random draw on glibc's `drand48` stream of a seed. Each
//!   [`MapPose`] is the mean of the heaviest cluster of particles with the deviations of the
//!   whole filter's covariance, as `amcl_pose` carries them; once they lie within the search
//!   limits, [`Localization::from_estimate`] keeps the pose within its matching limits.
//!
//! Each module names the file and the function of slam_toolbox, karto, Ceres or Nav2 that
//! it follows, and follows its arithmetic in its order, with three departures: a scan is
//! checked against the map's laser before slam_toolbox's gate, which slam_toolbox runs
//! first (`shouldProcessScan`); a beam's angle is the pose's heading plus the beam's
//! bearing, where karto adds the first angle to the heading before the steps; and
//! [`Localization::from_initial_pose`] searches a disc with a coarse matcher of its own
//! (`ScanMatcher::match_scan_within`), with no response expansion. Poses of the floor are
//! `control_core::planar_pose::PlanarPose`.
#![forbid(unsafe_code)]

mod cholesky;
mod geometry;
mod global_localization;
mod grid;
mod laser_projection;
mod localization;
mod localized_scan;
mod map_pose;
mod mapper;
mod mapping;
mod occupancy_grid;
mod optimizer;
mod placed_scan;
mod pose_graph;
mod saved_map;
mod scan_matcher;
#[cfg(test)]
mod slam_toolbox_parameters;

pub use global_localization::{
    AmclParameterError, AmclParameters, GlobalLocalization, GlobalLocalizationError,
    GlobalLocalizationParameters, RobotModelType, SearchStep,
};
pub use laser_projection::{
    LaserProjection, LaserProjectionError, LaserProjectionParameters, ProjectedScan,
};
pub use localization::{
    InitialPose, InitialPoseError, InitialPoseFailure, Localization, LocalizationError,
    LocalizationParameters, MatcherReach,
};
pub use localized_scan::{ScanError, ScanId};
pub use map_pose::{Deviations, LocalizationLimits, MapPose};
pub use mapping::{FinishError, Mapping, MappingError, MappingParameters, ParameterError};
pub use occupancy_grid::{CellState, GridError, GridExtent, MapSaverFiles, OccupancyGrid};
pub use placed_scan::PlacedScan;
pub use saved_map::{LoadError, MapSummary, RoomMap, SaveError};
pub use scan_matcher::MatcherError;
