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

//! slam_toolbox's parameters of `mapper_params_online_sync.yaml`, read by the unit tests
//! and, through `#[path]`, by the integration tests' `common` module, each with the laser
//! ranges of its scans.

use super::MappingParameters;

/// slam_toolbox's parameters of `mapper_params_online_sync.yaml`, with the laser ranges
/// `min_laser_range` and `max_laser_range` (m).
pub fn mapper_params_online_sync(min_laser_range: f64, max_laser_range: f64) -> MappingParameters {
    MappingParameters {
        throttle_scans: 1,
        minimum_time_interval: 0.5,
        check_min_dist_and_heading_precisely: false,
        min_laser_range,
        max_laser_range,
        resolution: 0.05,
        use_scan_barycenter: true,
        minimum_travel_distance: 0.5,
        minimum_travel_heading: 0.5,
        scan_buffer_size: 10,
        scan_buffer_maximum_scan_distance: 10.0,
        link_match_minimum_response_fine: 0.1,
        link_scan_maximum_distance: 1.5,
        loop_search_maximum_distance: 3.0,
        do_loop_closing: true,
        loop_match_minimum_chain_size: 10,
        loop_match_maximum_variance_coarse: 3.0,
        loop_match_minimum_response_coarse: 0.35,
        loop_match_minimum_response_fine: 0.45,
        correlation_search_space_dimension: 0.5,
        correlation_search_space_resolution: 0.01,
        correlation_search_space_smear_deviation: 0.1,
        loop_search_space_dimension: 8.0,
        loop_search_space_resolution: 0.05,
        loop_search_space_smear_deviation: 0.03,
        distance_variance_penalty: 0.5,
        angle_variance_penalty: 1.0,
        fine_search_angle_offset: 0.00349,
        coarse_search_angle_offset: 0.349,
        coarse_angle_resolution: 0.0349,
        minimum_angle_penalty: 0.9,
        minimum_distance_penalty: 0.5,
        use_response_expansion: true,
        min_pass_through: 2,
        occupancy_threshold: 0.1,
    }
}
