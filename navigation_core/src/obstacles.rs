//! What a mobile base's sensors see, kept in a voxel map and projected to a
//! cost grid as Nav2 1.5.2's obstacle layer marks the points of its sensors
//! (nav2_costmap_2d plugins/obstacle_layer.cpp, src/observation_buffer.cpp).
//!
//! [`LiveObstacles`] holds an occupancy map in a frame fixed to the floor, its
//! z up from the floor, and inserts each sensor frame into it, as
//! occupancy_map inserts a scan. A depth frame's points all stand at the
//! frame's time. A laser scan's hits each stand where the sensor stood at
//! their ray's time ([`LaserScan::placed_hits`]), carried into the sensor's
//! frame at the first ray, from whose origin the map traces every ray, as the
//! observation buffer traces a scan from the sensor's origin at its stamp
//! (`ObservationBuffer::bufferCloud`). A depth sample of 0 and a ray that
//! carries a mark of lidar_2d:v1 make no point: a ray that met nothing frees
//! and marks nothing, as the obstacle layer leaves it with `inf_is_valid`
//! false (`ObstacleLayer::laserScanCallback`).
//!
//! After each frame, the voxels of every column under the padded footprint,
//! placed at the base's pose at the frame's time, take one miss's log-odds, as
//! the obstacle layer writes `FREE_SPACE` under its footprint
//! (`ObstacleLayer::updateCosts`, `Costmap2D::setConvexPolygonCost`), the
//! columns found as Nav2 finds the cells under a footprint. A voxel the base
//! stood on reads free, and the next frame that sees an obstacle there marks
//! it again, as the obstacle layer marks a cleared cell; what the sensors see
//! of the robot itself within the footprint marks nothing.
//!
//! [`LiveObstacles::project`] gives the obstacle layer over a grid: a cell is
//! lethal under the center of an occupied voxel whose height lies between
//! `min_obstacle_height_m` and `max_obstacle_height_m`, both included, as the
//! observation buffer keeps a point, or more than `max_step_down_m` below the
//! floor, a drop the sensors saw; free where the map knows a voxel over it;
//! and unknown otherwise. Each height is a whole number of the map's voxels,
//! at least one, so a voxel's center judges as its points do, and the floor's
//! own voxels, either side of the floor, are neither an obstacle nor a drop:
//! [`ObstacleHeights::parse`] takes the voxel's side with the heights, and
//! [`LiveObstacles::new`] a map of that side. [`LiveObstacles::recentered`]
//! moves the map over other bounds, keeping what it knows of the voxels both
//! hold, for a grid that moves with the base.

use camera_core::laser_scan::LaserScan;
use camera_core::pose::{PlanarPose, Pose};
use control_core::positive_finite::PositiveFinite;
use occupancy_map::{Bounds, InsertError, MapError, Scan, VoxelMap};

use crate::footprint::PaddedFootprint;
use crate::grid::{CostGrid, FREE, GridError, GridGeometry, LETHAL, MAX_AXIS_CELLS, UNKNOWN};
use crate::raster::polygon_cells;

/// How far a height may lie from a whole number of voxels, in voxels: the
/// rounding of a height of meters over the voxel's side.
const WHOLE_VOXEL_TOLERANCE: f64 = 1e-9;

/// The heights, in meters above the floor, that judge an occupied voxel of a
/// side `voxel_m`: an obstacle from `min_obstacle_height_m` to
/// `max_obstacle_height_m`, both included, and a drop more than
/// `max_step_down_m` below the floor, each a whole number of voxels, at least
/// one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleHeights {
    min_obstacle_height_m: f64,
    max_obstacle_height_m: f64,
    max_step_down_m: f64,
    voxel_m: f64,
}

/// Why three heights judge no voxel.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum ObstacleHeightsError {
    #[error(
        "min_obstacle_height_m {min_obstacle_height_m}, max_obstacle_height_m \
         {max_obstacle_height_m} and max_step_down_m {max_step_down_m} are not all numbers: \
         give finite heights in meters"
    )]
    NotFinite {
        min_obstacle_height_m: f64,
        max_obstacle_height_m: f64,
        max_step_down_m: f64,
    },
    #[error(
        "min_obstacle_height_m {0} m is not above the floor: give the height up to which a \
         voxel counts as floor, above 0"
    )]
    FloorBand(f64),
    #[error(
        "max_obstacle_height_m {max_obstacle_height_m} m is not above min_obstacle_height_m \
         {min_obstacle_height_m} m: give the higher height as the maximum"
    )]
    Inverted {
        min_obstacle_height_m: f64,
        max_obstacle_height_m: f64,
    },
    #[error(
        "max_step_down_m {0} m is no step: give the deepest step the base drives down, above 0"
    )]
    StepDown(f64),
    #[error(
        "{name} {height_m} m lies within the floor's own voxels, {voxel_m} m either side of \
         the floor: give at least {voxel_m} m"
    )]
    WithinTheFloorVoxels {
        name: &'static str,
        height_m: f64,
        voxel_m: f64,
    },
    #[error(
        "{name} {height_m} m is not a whole number of the map's {voxel_m} m voxels, which judge \
         a height by their centers: give {lower_m} m or {upper_m} m"
    )]
    NotWholeVoxels {
        name: &'static str,
        height_m: f64,
        voxel_m: f64,
        lower_m: f64,
        upper_m: f64,
    },
}

impl ObstacleHeights {
    /// The heights of a navigator's launch, in meters, for a voxel map of
    /// voxels `voxel_m` on a side.
    pub fn parse(
        min_obstacle_height_m: f64,
        max_obstacle_height_m: f64,
        max_step_down_m: f64,
        voxel_m: PositiveFinite,
    ) -> Result<Self, ObstacleHeightsError> {
        let heights = [
            min_obstacle_height_m,
            max_obstacle_height_m,
            max_step_down_m,
        ];
        if !heights.iter().all(|h| h.is_finite()) {
            return Err(ObstacleHeightsError::NotFinite {
                min_obstacle_height_m,
                max_obstacle_height_m,
                max_step_down_m,
            });
        }
        if min_obstacle_height_m <= 0.0 {
            return Err(ObstacleHeightsError::FloorBand(min_obstacle_height_m));
        }
        if max_obstacle_height_m <= min_obstacle_height_m {
            return Err(ObstacleHeightsError::Inverted {
                min_obstacle_height_m,
                max_obstacle_height_m,
            });
        }
        if max_step_down_m <= 0.0 {
            return Err(ObstacleHeightsError::StepDown(max_step_down_m));
        }
        let heights = Self {
            min_obstacle_height_m,
            max_obstacle_height_m,
            max_step_down_m,
            voxel_m: voxel_m.get(),
        };
        heights
            .named()
            .into_iter()
            .try_for_each(|(name, height_m)| whole_voxels(name, height_m, heights.voxel_m))?;
        Ok(heights)
    }

    pub fn min_obstacle_height_m(&self) -> f64 {
        self.min_obstacle_height_m
    }

    pub fn max_obstacle_height_m(&self) -> f64 {
        self.max_obstacle_height_m
    }

    pub fn max_step_down_m(&self) -> f64 {
        self.max_step_down_m
    }

    /// Whether an occupied voxel centered `height_m` above the floor marks its
    /// cell: an obstacle in the band, or a drop below the step.
    fn marks(&self, height_m: f64) -> bool {
        (self.min_obstacle_height_m..=self.max_obstacle_height_m).contains(&height_m)
            || height_m < -self.max_step_down_m
    }

    /// Each height by the name a launch gives it.
    fn named(&self) -> [(&'static str, f64); 3] {
        [
            ("min_obstacle_height_m", self.min_obstacle_height_m),
            ("max_obstacle_height_m", self.max_obstacle_height_m),
            ("max_step_down_m", self.max_step_down_m),
        ]
    }
}

/// `height_m`, the height `name`, as a whole number of voxels of `voxel_m`,
/// at least one.
fn whole_voxels(
    name: &'static str,
    height_m: f64,
    voxel_m: f64,
) -> Result<(), ObstacleHeightsError> {
    let voxels = height_m / voxel_m;
    if voxels < 1.0 - WHOLE_VOXEL_TOLERANCE {
        return Err(ObstacleHeightsError::WithinTheFloorVoxels {
            name,
            height_m,
            voxel_m,
        });
    }
    if (voxels - voxels.round()).abs() > WHOLE_VOXEL_TOLERANCE {
        return Err(ObstacleHeightsError::NotWholeVoxels {
            name,
            height_m,
            voxel_m,
            lower_m: voxels.floor() * voxel_m,
            upper_m: voxels.ceil() * voxel_m,
        });
    }
    Ok(())
}

/// Why a map and heights keep no live obstacles, or keep none over other
/// bounds.
#[derive(Debug, thiserror::Error)]
pub enum LiveObstaclesError {
    #[error(
        "the heights judge voxels of {heights_voxel_m} m and the map holds voxels of \
         {map_voxel_m} m: parse the heights with the map's voxel size"
    )]
    VoxelSize {
        heights_voxel_m: f64,
        map_voxel_m: f64,
    },
    #[error(
        "the map's bottom lies at {bottom_m} m, and a drop shows in a voxel more than \
         max_step_down_m {max_step_down_m} m below the floor: give the map bounds that reach \
         down to {lowest_needed_m} m or lower"
    )]
    AboveTheDrops {
        bottom_m: f64,
        max_step_down_m: f64,
        lowest_needed_m: f64,
    },
    #[error(
        "the map holds {size_x} by {size_y} columns of voxels, more than {} on an axis, the \
         most a cost grid's axis holds: give the map narrower bounds or larger voxels",
        MAX_AXIS_CELLS
    )]
    TooManyColumns { size_x: u32, size_y: u32 },
    #[error(
        "the bounds put a corner of the map's columns at {0:?}, which single precision does \
         not hold, where the grid of the columns places its cells: give bounds nearer the \
         map's origin or smaller voxels"
    )]
    ColumnsPastSinglePrecision([f64; 2]),
    #[error(transparent)]
    Map(#[from] MapError),
}

/// The live obstacles of a mobile base: its voxel map, the heights that judge
/// its voxels, its padded footprint, and the grid of the map's columns.
#[derive(Debug, Clone)]
pub struct LiveObstacles {
    map: VoxelMap,
    heights: ObstacleHeights,
    footprint: PaddedFootprint,
    columns: GridGeometry,
}

impl LiveObstacles {
    /// The live obstacles of `map`, a map in a frame fixed to the floor, its z
    /// up from the floor, judged by `heights`, which judge voxels of the map's
    /// side, for a base of `padded_footprint`. The map's bottom lies a voxel or
    /// more below `max_step_down_m` under the floor, so it holds the voxels a
    /// drop shows in. A map of more columns on an axis than a cost grid's axis
    /// holds is refused.
    pub fn new(
        map: VoxelMap,
        heights: ObstacleHeights,
        padded_footprint: PaddedFootprint,
    ) -> Result<Self, LiveObstaclesError> {
        if map.voxel_size() != heights.voxel_m {
            return Err(LiveObstaclesError::VoxelSize {
                heights_voxel_m: heights.voxel_m,
                map_voxel_m: map.voxel_size(),
            });
        }
        let lowest_needed_m = -(heights.max_step_down_m + heights.voxel_m);
        let bottom_m = map.columns().bottom_m;
        if bottom_m > lowest_needed_m + WHOLE_VOXEL_TOLERANCE * heights.voxel_m {
            return Err(LiveObstaclesError::AboveTheDrops {
                bottom_m,
                max_step_down_m: heights.max_step_down_m,
                lowest_needed_m,
            });
        }
        Ok(Self {
            columns: columns_of(&map)?,
            map,
            heights,
            footprint: padded_footprint,
        })
    }

    /// The live obstacles over `bounds`, in the map's frame: each voxel the
    /// old bounds and the new hold keeps what the map knew of it, and every
    /// other voxel is unknown, as a grid that moves with the base keeps the
    /// obstacles around it. Bounds of more columns on an axis than a cost
    /// grid's axis holds are refused.
    pub fn recentered(&self, bounds: Bounds) -> Result<Self, LiveObstaclesError> {
        let map = self.map.recentered(bounds)?;
        Ok(Self {
            columns: columns_of(&map)?,
            map,
            heights: self.heights,
            footprint: self.footprint.clone(),
        })
    }

    /// The grid of the map's columns, one cell a column: the grid a navigator
    /// in the map's frame plans on.
    pub fn columns(&self) -> GridGeometry {
        self.columns
    }

    /// Inserts the points of a depth frame, in the sensor's frame, the sensor
    /// at `sensor` and the base at `base` at the frame's time, in the map's
    /// frame, a point farther than `max_range_m` clearing to it as occupancy_map
    /// clears one; then clears the columns under the footprint at `base`.
    pub fn insert_depth_frame(
        &mut self,
        points: &[[f32; 3]],
        sensor: Pose,
        base: PlanarPose,
        max_range_m: PositiveFinite,
    ) -> Result<(), InsertError> {
        self.insert(points, sensor, max_range_m)?;
        self.clear_footprint(base);
        Ok(())
    }

    /// Inserts the hits of a laser scan, the sensor at `sensor_at_first_ray`
    /// and at `sensor_at_last_ray` at the times of its first and last rays,
    /// and the base at `base` at the scan's time, in the map's frame, each hit
    /// placed at its ray's time and traced from the sensor's origin at the
    /// first ray; then clears the columns under the footprint at `base`.
    pub fn insert_laser_scan(
        &mut self,
        scan: &LaserScan,
        sensor_at_first_ray: &Pose,
        sensor_at_last_ray: &Pose,
        base: PlanarPose,
        max_range_m: PositiveFinite,
    ) -> Result<(), InsertError> {
        let to_first_ray = sensor_at_first_ray.inverse();
        let points: Vec<[f32; 3]> = scan
            .placed_hits(sensor_at_first_ray, sensor_at_last_ray)
            .into_iter()
            .map(|hit| to_first_ray.apply(hit).map(|c| c as f32))
            .collect();
        self.insert(&points, *sensor_at_first_ray, max_range_m)?;
        self.clear_footprint(base);
        Ok(())
    }

    /// Forgets everything the sensors saw: every voxel unknown.
    pub fn clear(&mut self) {
        self.map.clear();
    }

    /// The obstacle layer over a grid of `geometry`, the map placed in the
    /// grid's frame at `grid_from_map`. The map's frame and the grid's share
    /// the floor, so the pose is planar and the heights read the map's z.
    pub fn project(&self, grid_from_map: PlanarPose, geometry: GridGeometry) -> ObstacleLayer {
        let mut grid = CostGrid::filled(geometry, UNKNOWN);
        let costs = grid.costs_mut();
        for key in self.map.known() {
            let [x, y, z] = self.map.center(key);
            let Some(cell) = geometry.cell_of(grid_from_map.apply([x, y])) else {
                continue;
            };
            let cost = &mut costs[geometry.index(cell)];
            if self.map.is_occupied(key) && self.heights.marks(z) {
                *cost = LETHAL;
            } else if *cost == UNKNOWN {
                *cost = FREE;
            }
        }
        ObstacleLayer { grid }
    }

    fn insert(
        &mut self,
        points: &[[f32; 3]],
        sensor: Pose,
        max_range_m: PositiveFinite,
    ) -> Result<(), InsertError> {
        let frame = self.map.frame().clone();
        let scan = Scan {
            frame: &frame,
            sensor,
            points,
            max_range_m,
        };
        self.map.insert(&scan, &[])
    }

    /// Clears every voxel of the columns under the footprint at `base`; none
    /// when a vertex lies off the map, as Nav2 then clears no cell.
    fn clear_footprint(&mut self, base: PlanarPose) {
        let Some(cells) = polygon_cells(&self.columns, &self.footprint.placed_at(base)) else {
            return;
        };
        self.map
            .clear_columns(cells.into_iter().map(|cell| [cell.x(), cell.y()]));
    }
}

/// The grid of `map`'s columns, one cell a column, when a grid's axis holds
/// as many cells as the map holds columns on each axis.
fn columns_of(map: &VoxelMap) -> Result<GridGeometry, LiveObstaclesError> {
    let columns = map.columns();
    GridGeometry::new(
        columns.size_x,
        columns.size_y,
        PositiveFinite::parse(columns.side_m).expect("a voxel's side lies above 0"),
        columns.origin,
    )
    .map_err(|error| match error {
        GridError::AxisTooLong { size_x, size_y } => {
            LiveObstaclesError::TooManyColumns { size_x, size_y }
        }
        GridError::Origin(corner) | GridError::Extent(corner) => {
            LiveObstaclesError::ColumnsPastSinglePrecision(corner)
        }
        error => panic!("a map's columns, one or more an axis from a finite corner: {error}"),
    })
}

/// A grid of what the sensors saw over each cell: [`LETHAL`] under an obstacle
/// or a drop, [`FREE`] where the map knows a voxel and none marks the cell,
/// and [`UNKNOWN`] where it knows none.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleLayer {
    grid: CostGrid,
}

impl ObstacleLayer {
    pub fn grid(&self) -> &CostGrid {
        &self.grid
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_core::footprint::Footprint;
    use control_core::non_negative_finite::NonNegativeFinite;
    use occupancy_map::Frame;

    fn map(voxel_m: f64) -> VoxelMap {
        VoxelMap::new(
            Frame::parse("odom").unwrap(),
            PositiveFinite::parse(voxel_m).unwrap(),
            Bounds {
                min: [-1.0, -1.0, -0.5],
                max: [1.0, 1.0, 1.0],
            },
        )
        .unwrap()
    }

    fn square() -> PaddedFootprint {
        PaddedFootprint::new(
            &Footprint::parse(&[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap()
    }

    fn twentieth() -> PositiveFinite {
        PositiveFinite::parse(0.05).unwrap()
    }

    #[test]
    fn heights_want_numbers_a_band_above_the_floor_and_a_step() {
        assert!(matches!(
            ObstacleHeights::parse(f64::NAN, 0.5, 0.1, twentieth()),
            Err(ObstacleHeightsError::NotFinite { .. })
        ));
        assert_eq!(
            ObstacleHeights::parse(0.0, 0.5, 0.1, twentieth()),
            Err(ObstacleHeightsError::FloorBand(0.0))
        );
        assert!(matches!(
            ObstacleHeights::parse(0.5, 0.5, 0.1, twentieth()),
            Err(ObstacleHeightsError::Inverted { .. })
        ));
        assert_eq!(
            ObstacleHeights::parse(0.1, 0.5, 0.0, twentieth()),
            Err(ObstacleHeightsError::StepDown(0.0))
        );
        let heights = ObstacleHeights::parse(0.1, 0.5, 0.1, twentieth()).unwrap();
        assert!(heights.marks(0.1) && heights.marks(0.5) && heights.marks(-0.125));
        assert!(!heights.marks(0.075) && !heights.marks(0.525) && !heights.marks(-0.075));
    }

    // A map whose bottom lies less than a voxel below the step under the floor
    // holds no voxel a drop shows in: refused, naming the lowest bottom that
    // serves; a map a voxel deeper is kept.
    #[test]
    fn a_map_reaches_a_voxel_below_the_step() {
        let heights = ObstacleHeights::parse(0.1, 0.5, 0.1, twentieth()).unwrap();
        let reaching_down_to = |min_z: f64| {
            VoxelMap::new(
                Frame::parse("odom").unwrap(),
                twentieth(),
                Bounds {
                    min: [-1.0, -1.0, min_z],
                    max: [1.0, 1.0, 1.0],
                },
            )
            .unwrap()
        };
        let refused = LiveObstacles::new(reaching_down_to(-0.08), heights, square()).unwrap_err();
        let LiveObstaclesError::AboveTheDrops {
            bottom_m,
            lowest_needed_m,
            ..
        } = refused
        else {
            panic!("{refused:?}");
        };
        assert!((bottom_m + 0.1).abs() < 1e-12 && (lowest_needed_m + 0.15).abs() < 1e-12);
        assert!(refused.to_string().contains("down to -0.15"), "{refused}");
        assert!(LiveObstacles::new(reaching_down_to(-0.12), heights, square()).is_ok());
    }

    // A height off the voxels names the whole values either side of it, and one
    // within the floor's voxels names the least it may be; live obstacles take a
    // map of the voxels the heights judge.
    #[test]
    fn heights_are_whole_voxels_of_the_map() {
        let refused = ObstacleHeights::parse(0.12, 0.5, 0.1, twentieth()).unwrap_err();
        let ObstacleHeightsError::NotWholeVoxels {
            name,
            lower_m,
            upper_m,
            ..
        } = refused
        else {
            panic!("{refused:?}");
        };
        assert_eq!(name, "min_obstacle_height_m");
        assert!((lower_m - 0.1).abs() < 1e-12 && (upper_m - 0.15).abs() < 1e-12);
        assert_eq!(
            ObstacleHeights::parse(0.1, 0.5, 0.05, PositiveFinite::parse(0.1).unwrap()),
            Err(ObstacleHeightsError::WithinTheFloorVoxels {
                name: "max_step_down_m",
                height_m: 0.05,
                voxel_m: 0.1,
            })
        );
        let whole = ObstacleHeights::parse(0.1, 0.5, 0.1, twentieth()).unwrap();
        let refused = LiveObstacles::new(map(0.1), whole, square()).unwrap_err();
        let LiveObstaclesError::VoxelSize {
            heights_voxel_m,
            map_voxel_m,
        } = refused
        else {
            panic!("{refused:?}");
        };
        assert_eq!((heights_voxel_m, map_voxel_m), (0.05, 0.1));
        let live = LiveObstacles::new(map(0.05), whole, square()).unwrap();
        assert_eq!(live.columns().resolution_m(), 0.05);
        assert_eq!(live.columns().origin(), [-1.0, -1.0]);
        let moved = live
            .recentered(Bounds {
                min: [0.0, -1.0, -0.5],
                max: [2.0, 1.0, 1.0],
            })
            .unwrap();
        assert_eq!(moved.columns().origin(), [0.0, -1.0]);
    }

    // A map of more columns on an axis than a cost grid's axis holds keeps no
    // live obstacles, and live obstacles take no bounds of so many columns:
    // each is refused, naming the map's columns.
    #[test]
    fn a_map_of_more_columns_an_axis_than_a_grid_holds_is_refused() {
        let tenth = PositiveFinite::parse(0.1).unwrap();
        let half_m = f64::from(MAX_AXIS_CELLS / 2 + 1) * tenth.get();
        let long = Bounds {
            min: [-half_m, 0.0, -0.5],
            max: [half_m, 0.05, 1.0],
        };
        let heights = ObstacleHeights::parse(0.1, 0.5, 0.1, tenth).unwrap();
        let too_many = |refused: LiveObstaclesError| {
            assert!(
                matches!(
                    refused,
                    LiveObstaclesError::TooManyColumns { size_x, size_y: 1 }
                        if size_x > MAX_AXIS_CELLS
                ),
                "{refused:?}"
            );
        };
        let wide = VoxelMap::new(Frame::parse("odom").unwrap(), tenth, long).unwrap();
        too_many(LiveObstacles::new(wide, heights, square()).unwrap_err());
        let live = LiveObstacles::new(map(0.1), heights, square()).unwrap();
        too_many(live.recentered(long).unwrap_err());
    }

    // Columns of voxels 1e35 m wide reach past single precision's range: a
    // map whose first column starts past it, at 1e39 m, and one whose column
    // starts below f32::MAX and ends past it, give no grid of columns, each
    // refused saying what to give; a map at 1e37 m gives one.
    #[test]
    fn columns_past_single_precision_are_refused() {
        let side = PositiveFinite::parse(1e35).unwrap();
        let columns_at = |x: f64| {
            let one_voxel = Bounds {
                min: [x, 0.0, 0.0],
                max: [x, 0.0, 0.0],
            };
            columns_of(&VoxelMap::new(Frame::parse("odom").unwrap(), side, one_voxel).unwrap())
        };
        for x in [1e39, 3.4025e38] {
            let refused = columns_at(x).unwrap_err();
            assert!(
                matches!(refused, LiveObstaclesError::ColumnsPastSinglePrecision(_))
                    && refused
                        .to_string()
                        .starts_with("the bounds put a corner of the map's columns at")
                    && refused
                        .to_string()
                        .ends_with("give bounds nearer the map's origin or smaller voxels"),
                "{refused:?}"
            );
        }
        assert!(columns_at(1e37).is_ok());
    }
}
