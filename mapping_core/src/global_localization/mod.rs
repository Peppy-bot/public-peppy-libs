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

//! The base found in a map from no known pose: global localization as Nav2 1.5.2's AMCL
//! runs it once `reinitialize_global_localization` starts it. Its particles spread over the
//! free cells of the map's grid, are weighed by AMCL's likelihood field model, moved by the
//! motion model of `robot_model_type` and resampled by KLD sampling. After each scan that
//! AMCL publishes a pose after, the estimate is the mean of the heaviest cluster, with the
//! deviations of the whole filter's covariance as `amcl_pose` carries it; once they lie
//! within the search limits, the search ends in a [`Localization`] that keeps the pose
//! within its matching limits.
//!
//! Each module follows the nav2_amcl files that it names, in their arithmetic and its
//! order: `drand48` the generator, `particle_filter` and `kdtree` the filter, `map` and
//! `cspace` the map and its distance field, `likelihood_field` and `motion_model` the
//! sensor and the motion models, `pose_arithmetic` the pose arithmetic, and `amcl` the
//! node's flow of a scan.

mod amcl;
mod cspace;
mod drand48;
mod kdtree;
mod likelihood_field;
mod map;
mod motion_model;
mod parameters;
mod particle_filter;
mod pose_arithmetic;

pub use motion_model::RobotModelType;
pub use parameters::{AmclParameterError, AmclParameters};

use crate::localization::{
    Localization, LocalizationError, LocalizationParameters, RestoredLocalization, compose,
    in_half_open_turn, inverse,
};
use crate::map_pose::{Deviations, LocalizationLimits, MapPose};
use crate::placed_scan::PlacedScan;
use crate::saved_map::RoomMap;
use amcl::Amcl;
use thiserror::Error;

/// The parameters of global localization: AMCL's; the search limits, within which the
/// deviations of the whole filter hold the base localized; and the parameters of the
/// localization that keeps the pose once the search ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalLocalizationParameters {
    pub amcl: AmclParameters,
    pub search_limits: LocalizationLimits,
    pub localization: LocalizationParameters,
}

/// Why parameters or a map start no global localization.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GlobalLocalizationError {
    #[error(transparent)]
    Amcl(#[from] AmclParameterError),
    #[error(transparent)]
    Localization(#[from] LocalizationError),
    #[error(
        "the map's grid holds no free cell to spread the particles over: localize in a map \
         whose scans saw free floor"
    )]
    NoFreeCell,
}

/// A search for the base in a map.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalLocalization {
    amcl: Amcl,
    search_limits: LocalizationLimits,
    /// The localization in the map that keeps the pose once the search ends, placed at
    /// the pose it ends at.
    localization: RestoredLocalization,
}

/// A search after a scan. Each variant holds the whole state that goes on, the search's or
/// the localization's, and the caller takes it out at once.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum SearchStep {
    /// The search goes on, with the pose that AMCL publishes after the scan, whose
    /// deviations lie outside the search limits; none after a scan that AMCL publishes no
    /// pose after.
    Searching(GlobalLocalization, Option<MapPose>),
    /// The search ended at the pose, whose deviations lie within the search limits: the
    /// localization keeps the pose from the next scan on.
    Found(Localization, MapPose),
}

impl GlobalLocalization {
    /// `reinitialize_global_localization` in `map`: the particles spread over the free
    /// cells of its grid, every random draw on the stream of `srand48(seed)`. The next scan
    /// weighs them where they lie.
    pub fn start(
        map: &RoomMap,
        parameters: GlobalLocalizationParameters,
        seed: u32,
    ) -> Result<Self, GlobalLocalizationError> {
        let amcl = parameters.amcl.checked()?;
        let localization = RestoredLocalization::new(&map.state, parameters.localization)?;
        let amcl = Amcl::new(map.grid(), amcl, seed).ok_or(GlobalLocalizationError::NoFreeCell)?;
        Ok(Self {
            amcl,
            search_limits: parameters.search_limits,
            localization,
        })
    }

    /// `laserReceived` of `scan`: the search goes on, or ends once the pose that AMCL
    /// publishes after it has deviations within the search limits. It takes a scan, as
    /// `Mapping::add` and `Localization::add` do.
    #[allow(clippy::should_implement_trait)]
    pub fn add(mut self, scan: &PlacedScan) -> SearchStep {
        let Some(pose) = self.amcl.add(scan) else {
            return SearchStep::Searching(self, None);
        };
        let deviations = Deviations::of_covariance(self.amcl.covariance());
        let estimate = in_half_open_turn(pose);
        let localized = deviations.within(&self.search_limits);
        let map_pose = MapPose {
            stamp_ns: scan.stamp_ns,
            pose: estimate.into(),
            deviations,
            localized,
        };
        if !localized {
            return SearchStep::Searching(self, Some(map_pose));
        }
        let map_from_odometry = compose(estimate, inverse(scan.odometry));
        SearchStep::Found(self.localization.placed(map_from_odometry), map_pose)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::mapping::Mapping;
    use crate::occupancy_grid::{CellState, GridParameters, OccupancyGrid};
    use camera_core::laser_scan::{LaserScan, Sweep};
    use camera_core::pose::PlanarPose;
    use control_core::positive_finite::PositiveFinite;
    use std::f64::consts::{PI, TAU};

    /// The half width and the half depth (m) of the test room, a rectangle about the
    /// origin, and the side (m) of its grid's cells.
    const HALF_WIDTH: f64 = 4.0;
    const HALF_DEPTH: f64 = 3.0;
    const RESOLUTION: f64 = 0.05;
    /// The rays of a scan of the room, all around the sensor.
    const RAYS: usize = 360;

    /// The scan at `stamp_ns` of the room, by a sensor at the origin of the base, the base at
    /// `truth` in the room and at `odometry` by odometry.
    pub(crate) fn room_scan(stamp_ns: i64, truth: PlanarPose, odometry: PlanarPose) -> PlacedScan {
        let (angle_min, angle_increment) = (-PI as f32, (TAU / RAYS as f64) as f32);
        let ranges: Vec<f32> = (0..RAYS)
            .map(|ray| {
                let angle =
                    truth.yaw() + f64::from(angle_min) + ray as f64 * f64::from(angle_increment);
                let (along_x, along_y) = (angle.cos(), angle.sin());
                let to_wall =
                    |half: f64, from: f64, along: f64| (half - from * along.signum()) / along.abs();
                let x_wall = to_wall(HALF_WIDTH, truth.x(), along_x);
                let y_wall = to_wall(HALF_DEPTH, truth.y(), along_y);
                x_wall.min(y_wall) as f32
            })
            .collect();
        let scan = LaserScan::parse(angle_min, angle_increment, 0.0, 0.1, 12.0, &ranges)
            .expect("a scan of the room");
        let still = Sweep {
            first_ray: odometry,
            last_ray: odometry,
        };
        PlacedScan::new(stamp_ns, &scan, &still, odometry).expect("a still laser's scan")
    }

    /// The grid of the room: its walls occupied, the floor inside free.
    pub(crate) fn room_grid() -> OccupancyGrid {
        let (width, height) = (
            (2.0 * HALF_WIDTH / RESOLUTION) as u32 + 1,
            (2.0 * HALF_DEPTH / RESOLUTION) as u32 + 1,
        );
        let cells = (0..height)
            .flat_map(|j| (0..width).map(move |i| (i, j)))
            .map(|(i, j)| {
                let wall = i == 0 || j == 0 || i == width - 1 || j == height - 1;
                if wall {
                    CellState::Occupied
                } else {
                    CellState::Free
                }
            })
            .collect();
        OccupancyGrid::from_cells(width, height, RESOLUTION, [-HALF_WIDTH, -HALF_DEPTH], cells)
            .expect("a grid")
    }

    /// The map of the room that a scan from its center makes.
    fn room_map() -> RoomMap {
        let parameters = crate::mapping::tests::parameters();
        let mut mapping =
            Mapping::start(parameters, PlanarPose::IDENTITY).expect("valid parameters");
        let center = PlanarPose::IDENTITY;
        mapping.add(&room_scan(0, center, center)).expect("a scan");
        mapping
            .finish(0, localization_parameters())
            .expect("valid parameters")
            .expect("a map of the scans")
            .0
    }

    fn limits(position_std_m: f64, yaw_std_rad: f64) -> LocalizationLimits {
        let limit = |value| PositiveFinite::parse(value).expect("a positive limit");
        LocalizationLimits {
            position_std_m: limit(position_std_m),
            yaw_std_rad: limit(yaw_std_rad),
        }
    }

    fn localization_parameters() -> LocalizationParameters {
        LocalizationParameters {
            slam_toolbox: crate::mapping::tests::parameters(),
            matching_limits: limits(0.1, 0.1),
        }
    }

    fn parameters() -> GlobalLocalizationParameters {
        GlobalLocalizationParameters {
            amcl: parameters::tests::nav2_parameters(),
            search_limits: limits(0.5, 0.5),
            localization: localization_parameters(),
        }
    }

    #[test]
    fn a_search_starts_in_a_map_with_free_floor() {
        assert!(GlobalLocalization::start(&room_map(), parameters(), 0).is_ok());
    }

    #[test]
    fn a_map_whose_grid_holds_no_free_cell_starts_no_search() {
        let map = room_map();
        // No cell is passed by more rays than the most a count holds, so every cell is
        // unknown.
        let unknown = RoomMap::new(
            map.state.clone(),
            GridParameters {
                resolution: RESOLUTION,
                min_pass_through: u32::MAX,
                occupancy_threshold: 0.1,
            },
            0,
        )
        .expect("a grid karto can count")
        .expect("a grid of the scans");
        assert_eq!(
            GlobalLocalization::start(&unknown, parameters(), 0).err(),
            Some(GlobalLocalizationError::NoFreeCell)
        );
    }

    #[test]
    fn parameters_that_amcl_or_the_localization_refuses_start_no_search() {
        let map = room_map();
        let amcl = GlobalLocalizationParameters {
            amcl: AmclParameters {
                resample_interval: 0,
                ..parameters::tests::nav2_parameters()
            },
            ..parameters()
        };
        assert_eq!(
            GlobalLocalization::start(&map, amcl, 0).err(),
            Some(GlobalLocalizationError::Amcl(
                AmclParameterError::NoResampleInterval
            ))
        );
        let mut localization = parameters();
        localization.localization.slam_toolbox.scan_buffer_size = 0;
        assert!(matches!(
            GlobalLocalization::start(&map, localization, 0).err(),
            Some(GlobalLocalizationError::Localization(
                LocalizationError::Parameters(_)
            ))
        ));
    }
}
