//! What a mobile base's sensors see, kept in a voxel map and projected to a
//! cost grid as Nav2 1.5.2's obstacle layer marks the points of its sensors
//! (nav2_costmap_2d plugins/obstacle_layer.cpp, src/observation_buffer.cpp).
//!
//! [`LiveObstacles`] holds an occupancy map, the voxel map, in a frame fixed
//! to the floor, its z up from the floor, and inserts each sensor frame into
//! it, as occupancy_map inserts a scan, the sensor placed by the base's pose
//! on the floor at the frame's time and the sensor's pose on the base. A
//! depth frame's points all stand at the frame's time. A laser scan's hits
//! each stand where the sensor stood at their ray's time
//! ([`LaserScan::placed_hits`]), the base's poses at the scan's first and
//! last rays given as a [`Sweep`], carried into the sensor's frame at the
//! first ray, from whose origin the map traces every ray, as the observation
//! buffer traces a scan from the sensor's origin at its stamp
//! (`ObservationBuffer::bufferCloud`). A depth sample of 0 and a ray that
//! carries a mark of lidar_2d:v1 make no point: a ray that met nothing frees
//! and marks nothing, as the obstacle layer leaves it with `inf_is_valid`
//! false (`ObstacleLayer::laserScanCallback`).
//!
//! Three departures from the obstacle layer's marking
//! (`ObstacleLayer::updateBounds`, obstacle_layer.cpp:512-581) follow from
//! occupancy_map's insert and from placing each hit at its ray's time:
//!
//! - The obstacle layer projects a scan in the scan's own frame
//!   (obstacle_layer.cpp:398), so laser_geometry's two transforms are the
//!   identity and every hit stands where the sensor stood at the scan's
//!   stamp; here each hit stands where the sensor stood at its own ray's
//!   time, so a sensor that moves while it sweeps places each hit where it
//!   measured it.
//! - The obstacle layer skips an observation whose sensor origin lies off its
//!   grid (obstacle_layer.cpp:516-519); here every frame goes into the voxel
//!   map, whose bounds clip its rays.
//! - occupancy_map reads a point farther from the sensor than the sensor's
//!   range, by their distance in 3-D, as no return at the range
//!   (`Scan::readings`), which clears its ray to the range and marks nothing;
//!   the obstacle layer marks the cell of each point within
//!   `obstacle_max_range` of the sensor's origin in the plane, however far it
//!   lies in 3-D.
//!
//! A return farther from its sensor's origin than the obstacle range,
//! `obstacle_max_range_m`, by their distance in the plane, marks nothing, as
//! the obstacle layer marks no point farther than `obstacle_max_range` from the
//! observation's origin in the plane (obstacle_layer.cpp:549-553): its ray
//! clears as a return's ray does, its voxel takes nothing from it, and it takes
//! no part in the votes of a drop. One range holds for every source, given to
//! [`LiveObstacles::new`]. A source's newest points hold such a return as any
//! other, as the collision monitor keeps a point of its source within its
//! heights at any distance past its `min_range` (src/pointcloud.cpp:160-176).
//!
//! A return inside the base's unpadded outline is the robot's own body, judged
//! in the base's frame at the return's own time, where the sensor's pose on
//! the base places it (a depth frame's point, or a laser scan's hit in the
//! sensor's frame at its ray's time): its ray clears free space up to it, and
//! it marks nothing, so the body enters neither the map nor a source's newest
//! frame, however the base moves while a lidar sweeps. Here the live obstacles
//! depart from the obstacle layer, which marks the body's returns as any other
//! and frees them by its clearing under the padded footprint at one pose
//! (`ObstacleLayer::updateCosts`), which leaves a body return lethal where the
//! footprint's raster leaves its cell out; and with each hit placed at its
//! ray's time, a body hit that a lidar measured late in the sweep of a moving
//! base lies past the footprint at the sweep's first ray, where the clearing
//! runs.
//!
//! After each frame, the voxels of every column under the padded footprint,
//! placed at the base's pose at the frame's time (at a laser scan's first
//! ray), take one miss's log-odds, as the obstacle layer writes `FREE_SPACE`
//! under its footprint (`ObstacleLayer::updateCosts`,
//! `Costmap2D::setConvexPolygonCost`), the columns found as Nav2 finds the
//! cells under a footprint, and none of them is a drop: the base stands on
//! floor there. A voxel the base stood on reads free, and the next frame that
//! sees an obstacle there marks it again, as the obstacle layer marks a
//! cleared cell.
//!
//! A drop is a column whose floor lies more than `max_step_down_m` down, the
//! step's plane, which each frame's rays find through the floor's plane, each
//! ray whose reading lies within the obstacle range. Each such ray that points
//! down from above the floor reads the floor's plane in the column where its
//! line crosses it: past, when its reading lies below the step's plane (a
//! return there, or no return with its range reaching there), and short, when
//! it returns above the floor's plane, before the crossing. A column where at
//! least three rays of one frame read past, more than the rays that read short
//! and the returns of that frame within the obstacle range that stand in it at
//! or above the step's plane together, is a drop until the footprint clears it:
//! one or two wrong far readings through a column mark none, and wrong far
//! readings among rays that see an obstacle before the floor, or through a
//! column whose floor the frame sees, are outnumbered there. A ray over an edge
//! crosses the floor's plane just past the edge, so a drop shows from the edge
//! on before a sensor sees the lower floor, the cell that holds the edge too
//! when more of the frame's rays read past the step through it than see its
//! floor.
//!
//! Wrong far readings fall at random, so three of one frame can fall in a
//! column whose floor the frame sees little of and mark a drop there. Measured
//! on rendered depth frames of a flat floor, from two camera mounts at two
//! speeds, with 2% and with 4% of the readings nearer than 2 m reading a depth
//! drawn uniformly from their own out to the camera's range, ten seeds each,
//! 24,000 frames in all: the rule marks 52 false drop cells, and one of them
//! lies in the base's path within the stop's horizon for 33 frames of one
//! drive. On the same frames, a count of two marks 152, a count of one 3,839,
//! and a count of three that need not outnumber the rays short and standing
//! 516,867.
//!
//! [`LiveObstacles::project`] gives the obstacle layer over a grid, the voxel
//! map placed in the grid's frame. A voxel stands for every point inside it,
//! and the obstacle layer marks the cell of every point
//! (obstacle_layer.cpp:525-581), so a column of the voxel map marks every
//! cell whose square overlaps the column's square, on a grid turned against
//! the map or finer than it too: a cell is lethal where its square overlaps
//! the square of a column that holds an occupied voxel whose center lies
//! between `min_obstacle_height_m` and `max_obstacle_height_m`, both included,
//! as the observation buffer keeps a point, or of a drop's column; free where
//! it overlaps the square of any other column the map knows a voxel of; and
//! unknown otherwise. Two squares overlap when they share more than a line or
//! a point, which the separating axes of convex polygons decide exactly: the
//! two axes of the grid and the two of the turned square. Nav2 marks points,
//! so the overlap of a column's square with a cell is the crate's own. Each
//! column's square stands a sliver in from the column's sides, so on a grid of
//! the map's own lattice a column's square overlaps the column's cell alone.
//! A placement that puts a corner of a column's square at a point that is not
//! finite is refused ([`PlacementNotFinite`]); a column whose square's extent
//! on the grid's lattice, its corners' offsets from the grid's origin over the
//! resolution, is not finite marks no cell, the cells of the grid that its
//! square overlaps among them, as the overlap is decided on that extent.
//!
//! Each height of the band is a whole number of the map's voxels, at least
//! one, so a voxel's center judges as its points do but one: a point exactly
//! at `max_obstacle_height_m` lies in the voxel above the band and marks
//! nothing, where the obstacle layer keeps it (obstacle_layer.cpp:535). The
//! floor's own voxels, either side of the floor, are no obstacle.
//! [`ObstacleHeights::parse`] takes the voxel's side with the heights
//! ([`ObstacleHeightsParams`]), and [`LiveObstacles::new`] makes its voxel map
//! of that side.
//! [`LiveObstacles::recentered`] moves the map and the drops over other
//! bounds, keeping what it knows of the columns both hold, for a grid that
//! moves with the base.
//!
//! Each frame comes from a source, sources 0 to one fewer than the count
//! [`LiveObstacles::new`] takes, the indexes the stop
//! ([`Stop`](crate::stop::Stop)) names them by, reading the count from the
//! live obstacles it decides with. For each source the live obstacles keep
//! the points its newest frame saw, in the map's frame on the floor's plane,
//! as the frame saw them before the clearing under the footprint: each return
//! of the frame but the robot's own body whose height lies in the band, both
//! ends included, as Nav2's collision monitor keeps a point of its point
//! cloud source (`PointCloud::getSourceData`, nav2_collision_monitor
//! src/pointcloud.cpp:174); and the center of each column the frame's rays
//! show a drop in, but for one whose center lies inside the unpadded outline
//! at the frame's base pose, floor the base stands on, which the monitor has
//! no counterpart of. A source's newest points hold whatever bounds the map
//! takes, as the monitor reads its sources' points whatever its cost map
//! holds. [`LiveObstacles::occupied_seen_from`] gives what the stop reads:
//! those points, and a cost grid. The monitor reads each source's newest
//! points (`CollisionMonitor::process`, collision_monitor_node.cpp:454;
//! `PointCloud::getSourceData`, src/pointcloud.cpp:107-117), and the obstacle
//! layer's clearing under the footprint sets the cost map's cells alone
//! (obstacle_layer.cpp:620-621), so an obstacle the padded footprint stands
//! on, outside the unpadded outline, is a point the stop reads, as it is one
//! the monitor reads. A return inside the unpadded outline is a point the
//! monitor reads and the stop does not, a real obstacle there among them, as
//! the stop's module gives it. [`LiveObstacles::clear`] empties the map and
//! forgets the drops, as base_navigation:v1's clear_grid empties the grid, and
//! keeps each source's newest points until the source's next frame replaces
//! them, as the monitor reads its sources' newest points from their topics,
//! which a clear of the cost map leaves as they are.

use std::collections::{BTreeSet, HashMap};
use std::num::NonZeroUsize;

use camera_core::laser_scan::{LaserScan, Sweep};
use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;
use frame_tree::FrameName;
use frame_tree::Transform;
use occupancy_map::{Bounds, MapError, Reading, Scan, VoxelMap};

use crate::footprint::PaddedFootprint;
use crate::grid::{
    Cell, CostGrid, FREE, GridError, GridGeometry, LETHAL, MAX_AXIS_CELLS, PlacementNotFinite,
    UNKNOWN, cells_under,
};
use crate::raster::polygon_cells;
use crate::stop::{Occupied, UnknownSource};

/// How far a height may lie from a whole number of voxels, in voxels: the
/// rounding of a height of meters over the voxel's side.
const WHOLE_VOXEL_TOLERANCE: f64 = 1e-9;

/// The fewest rays of one frame that read past the step's plane through a
/// column for the column to be a drop: three, so one or two wrong far readings
/// through a column that no other ray reads mark none, measured as the module
/// doc gives.
const MIN_RAYS_PAST_THE_STEP: u32 = 3;

/// The heights of a navigator's launch, in meters above the floor, and the
/// side of the voxels they judge, by a launch's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleHeightsParams {
    /// The lowest height of an obstacle.
    pub min_obstacle_height_m: f64,
    /// The highest height of an obstacle.
    pub max_obstacle_height_m: f64,
    /// The deepest step down the base drives down, below the floor.
    pub max_step_down_m: f64,
    /// The side of the voxel map's voxels.
    pub voxel_m: PositiveFinite,
}

/// The heights, in meters above the floor, that judge the voxels of a side
/// `voxel_m` and the rays of a frame: an obstacle in an occupied voxel from
/// `min_obstacle_height_m` to `max_obstacle_height_m`, both included, each a
/// whole number of voxels, at least one, and a drop where rays read past the
/// plane `max_step_down_m` below the floor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleHeights {
    min_obstacle_height_m: f64,
    max_obstacle_height_m: f64,
    max_step_down_m: f64,
    voxel_m: PositiveFinite,
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
    /// The heights of `params`, finite, a band above the floor of whole
    /// voxels, at least one, and a step down below it.
    pub fn parse(params: ObstacleHeightsParams) -> Result<Self, ObstacleHeightsError> {
        let ObstacleHeightsParams {
            min_obstacle_height_m,
            max_obstacle_height_m,
            max_step_down_m,
            voxel_m,
        } = params;
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
            voxel_m,
        };
        heights
            .band()
            .into_iter()
            .try_for_each(|(name, height_m)| whole_voxels(name, height_m, voxel_m.get()))?;
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

    /// Whether `height_m` above the floor lies in the band, both ends
    /// included: an occupied voxel centered there, and a return of a frame
    /// there, are obstacles.
    fn is_obstacle(&self, height_m: f64) -> bool {
        (self.min_obstacle_height_m..=self.max_obstacle_height_m).contains(&height_m)
    }

    /// The height of the step's plane: `max_step_down_m` below the floor.
    fn step_plane_z(&self) -> f64 {
        -self.max_step_down_m
    }

    /// Each height of the band by the name a launch gives it.
    fn band(&self) -> [(&'static str, f64); 2] {
        [
            ("min_obstacle_height_m", self.min_obstacle_height_m),
            ("max_obstacle_height_m", self.max_obstacle_height_m),
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

/// Why bounds keep no live obstacles.
#[derive(Debug, thiserror::Error)]
pub enum LiveObstaclesError {
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

/// The poses in a grid's frame by which the stop's points are read: where the
/// voxel map is placed, and where the base stands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridPoses {
    /// The voxel map's pose in the grid's frame.
    pub grid_from_map: PlanarPose,
    /// The base's pose in the grid's frame.
    pub base: PlanarPose,
}

/// The live obstacles of a mobile base: its voxel map, the heights that judge
/// its voxels and its rays, its padded footprint, the grid of the map's
/// columns, the columns of that grid that are drops, and the points each
/// source's newest frame saw.
#[derive(Debug, Clone)]
pub struct LiveObstacles {
    map: VoxelMap,
    heights: ObstacleHeights,
    footprint: PaddedFootprint,
    columns: GridGeometry,
    drops: BTreeSet<Cell>,
    /// How far from its sensor's origin in the plane a return marks.
    obstacle_max_range_m: PositiveFinite,
    /// For each source, in order, the points its newest frame saw in the
    /// map's frame on the floor's plane, as the module gives them; none
    /// before its first frame.
    newest_points: Vec<Vec<[f64; 2]>>,
}

impl LiveObstacles {
    /// The live obstacles of an empty map in `frame`, a frame fixed to the
    /// floor, its z up from the floor, over `bounds`, of the voxels `heights`
    /// judge, each source's returns marking within `obstacle_max_range_m` of
    /// its origin in the plane, for a base of `padded_footprint` whose
    /// `sources` sources give its frames, sources 0 to `sources - 1`, with no
    /// drop and no frame. Bounds of more columns on an axis than a cost grid's
    /// axis holds, or whose columns reach past single precision's range, are
    /// refused.
    pub fn new(
        frame: FrameName,
        bounds: Bounds,
        heights: ObstacleHeights,
        obstacle_max_range_m: PositiveFinite,
        padded_footprint: PaddedFootprint,
        sources: NonZeroUsize,
    ) -> Result<Self, LiveObstaclesError> {
        let map = VoxelMap::new(frame, heights.voxel_m, bounds)?;
        Ok(Self {
            columns: columns_of(&map)?,
            map,
            heights,
            footprint: padded_footprint,
            drops: BTreeSet::new(),
            obstacle_max_range_m,
            newest_points: vec![Vec::new(); sources.get()],
        })
    }

    /// The live obstacles over `bounds`, in the map's frame: each voxel and
    /// each column the old bounds and the new hold keeps what the map knew of
    /// it and whether it is a drop, and every other voxel is unknown and every
    /// other column no drop, as a grid that moves with the base keeps the
    /// obstacles around it. Each source's newest points stay as they are.
    /// Bounds of more columns on an axis than a cost grid's axis holds, or
    /// whose columns reach past single precision's range, are refused.
    pub fn recentered(&self, bounds: Bounds) -> Result<Self, LiveObstaclesError> {
        let map = self.map.recentered(bounds)?;
        let columns = columns_of(&map)?;
        let drops = self
            .drops
            .iter()
            .filter_map(|&cell| columns.cell_of(self.columns.center_of(cell)))
            .collect();
        Ok(Self {
            drops,
            newest_points: self.newest_points.clone(),
            columns,
            map,
            heights: self.heights,
            footprint: self.footprint.clone(),
            obstacle_max_range_m: self.obstacle_max_range_m,
        })
    }

    /// The grid of the voxel map's columns, one cell a column: the grid a
    /// navigator plans on in the voxel map's frame.
    pub fn columns(&self) -> GridGeometry {
        self.columns
    }

    /// The padded footprint of the base, which the stop projects.
    pub(crate) fn padded_footprint(&self) -> &PaddedFootprint {
        &self.footprint
    }

    /// How many sources give the frames, sources 0 to one fewer.
    pub(crate) fn sources(&self) -> usize {
        self.newest_points.len()
    }

    /// Inserts the points of a depth frame of `source`, in the sensor's frame,
    /// the base at `base` in the map's frame and the sensor at `sensor_on_base`
    /// in the base's frame at the frame's time, a point farther than
    /// `max_range_m` clearing to it as occupancy_map clears one, a return
    /// beyond the obstacle range clearing its ray and marking nothing, and a
    /// point inside the unpadded outline where `sensor_on_base` places it, the
    /// robot's own body, clearing its ray and marking nothing; keeps the points
    /// the frame saw as the source's newest; then clears the columns under the
    /// footprint at `base`. A source the live obstacles do not read is refused,
    /// and nothing changes.
    pub fn insert_depth_frame(
        &mut self,
        source: usize,
        points: &[[f32; 3]],
        base: PlanarPose,
        sensor_on_base: Transform,
        max_range_m: PositiveFinite,
    ) -> Result<(), UnknownSource> {
        self.known_source(source)?;
        let sensor = base.to_transform().compose(&sensor_on_base);
        let on_base: Vec<[f64; 2]> = points
            .iter()
            .map(|point| on_the_floor(sensor_on_base.apply(point.map(f64::from))))
            .collect();
        self.insert(source, points, &on_base, sensor, base, max_range_m);
        self.clear_footprint(base);
        Ok(())
    }

    /// Inserts the hits of a laser scan of `source`, the base at `base`'s poses
    /// in the map's frame at the times of the scan's first and last rays and
    /// the sensor at `sensor_on_base` in the base's frame, each hit placed at
    /// its ray's time and traced from the sensor's origin at the first ray, a
    /// hit beyond the obstacle range of the sensor's origin at the first ray
    /// clearing its ray and marking nothing, and a hit inside the unpadded
    /// outline where `sensor_on_base` places it at its ray's time, the robot's
    /// own body, clearing its ray and marking nothing; keeps the points the
    /// scan saw as the source's newest; then clears the columns under the
    /// footprint at the base's pose at the first ray, the scan's time. A source
    /// the live obstacles do not read is refused, and nothing changes.
    pub fn insert_laser_scan(
        &mut self,
        source: usize,
        scan: &LaserScan,
        base: Sweep<PlanarPose>,
        sensor_on_base: Transform,
        max_range_m: PositiveFinite,
    ) -> Result<(), UnknownSource> {
        self.known_source(source)?;
        let sensor = Sweep {
            first_ray: base.first_ray.to_transform().compose(&sensor_on_base),
            last_ray: base.last_ray.to_transform().compose(&sensor_on_base),
        };
        let to_first_ray = sensor.first_ray.inverse();
        let (points, on_base): (Vec<[f32; 3]>, Vec<[f64; 2]>) = scan
            .placed_hits(&sensor)
            .into_iter()
            .map(|hit| {
                (
                    to_first_ray.apply(hit.placed).map(|c| c as f32),
                    on_the_floor(sensor_on_base.apply(hit.in_sensor)),
                )
            })
            .unzip();
        self.insert(
            source,
            &points,
            &on_base,
            sensor.first_ray,
            base.first_ray,
            max_range_m,
        );
        self.clear_footprint(base.first_ray);
        Ok(())
    }

    /// Empties the map and forgets the drops, as base_navigation:v1's
    /// clear_grid empties the grid, so the map projects to a grid with no
    /// obstacle. Each source's newest points, which the stop reads beside the
    /// grid, stay until the source's next frame replaces them.
    pub fn clear(&mut self) {
        self.map.clear();
        self.drops.clear();
    }

    /// What the stop reads of a base at `poses.base` in the frame of `grid`,
    /// the map placed in that frame at `poses.grid_from_map`: each source's
    /// newest points, as the module gives them, in the base's frame, and
    /// `grid` with the base's pose on it.
    pub fn occupied_seen_from<'g>(&self, grid: &'g CostGrid, poses: GridPoses) -> Occupied<'g> {
        let GridPoses {
            grid_from_map,
            base,
        } = poses;
        let points = self
            .newest_points
            .iter()
            .flatten()
            .map(|&point| base.apply_inverse(grid_from_map.apply(point)))
            .collect();
        Occupied { points, grid, base }
    }

    /// Nothing when the live obstacles read `source`, else its refusal.
    pub(crate) fn known_source(&self, source: usize) -> Result<(), UnknownSource> {
        let sources = self.newest_points.len();
        if source >= sources {
            return Err(UnknownSource {
                index: source,
                sources,
            });
        }
        Ok(())
    }

    /// The obstacle layer over a grid of `geometry`, the map placed in the
    /// grid's frame at `grid_from_map`, as the module gives it: every cell
    /// whose square overlaps the square of a column that holds an obstacle or
    /// is a drop lethal, every other cell whose square overlaps the square of
    /// a column the map knows a voxel of free. The map's frame and the grid's
    /// share the floor, so the pose is planar and the heights read the map's
    /// z. A placement that puts a corner of a column's square at a point that
    /// is not finite is refused.
    pub fn project(
        &self,
        grid_from_map: PlanarPose,
        geometry: GridGeometry,
    ) -> Result<ObstacleLayer, PlacementNotFinite> {
        let mut known_columns = BTreeSet::new();
        let mut marked_columns = self.drops.clone();
        for key in self.map.known() {
            let [x, y, z] = self.map.center(key);
            let column = self
                .columns
                .cell_of([x, y])
                .expect("a voxel's center lies over its column");
            known_columns.insert(column);
            if self.map.is_occupied(key) && self.heights.is_obstacle(z) {
                marked_columns.insert(column);
            }
        }
        let mut grid = CostGrid::filled(geometry, UNKNOWN);
        for (columns, cost) in [(&known_columns, FREE), (&marked_columns, LETHAL)] {
            for &column in columns {
                for cell in cells_under(&self.columns, column, grid_from_map, &geometry)? {
                    grid.costs_mut()[geometry.index(cell)] = cost;
                }
            }
        }
        Ok(ObstacleLayer { grid })
    }

    /// Inserts `points` of `source`'s frame as a scan in the map's own frame,
    /// the one scan occupancy_map's insert takes without a refusal, each point
    /// of the robot's own body ([`LiveObstacles::split_off_the_body`]) and
    /// each return beyond the obstacle range
    /// ([`LiveObstacles::split_off_beyond_range`]) clearing its ray and
    /// marking nothing; marks the drops the frame's rays show; and keeps as
    /// the source's newest points the place of each of its returns but the
    /// body's whose height lies in the band, and the center of each drop's
    /// column that, seen from the base at `base`, lies outside the unpadded
    /// outline.
    fn insert(
        &mut self,
        source: usize,
        points: &[[f32; 3]],
        points_on_base: &[[f64; 2]],
        sensor: Transform,
        base: PlanarPose,
        max_range_m: PositiveFinite,
    ) {
        let (body, seen_points) = self.split_off_the_body(points, points_on_base);
        let frame = self.map.frame().clone();
        let seen = Scan {
            frame: &frame,
            sensor,
            points: &seen_points,
            max_range_m,
        };
        let (within_range, beyond_range) = self.split_off_beyond_range(&seen);
        let marking = Scan {
            points: &within_range,
            ..seen
        };
        self.map
            .insert_clearing(&marking, &[body, beyond_range].concat())
            .expect("a scan in the map's own frame inserts");
        let drops = self.drops_shown(&Scan { points, ..seen });
        let off_the_base_drops = drops
            .iter()
            .map(|&column| self.columns.center_of(column))
            .filter(|&center| !self.footprint.outline_holds(base.apply_inverse(center)));
        self.newest_points[source] = self
            .obstacle_points(&seen)
            .chain(off_the_base_drops)
            .collect();
        self.drops.extend(drops);
    }

    /// `points` split into the robot's own body, each point whose place in
    /// the base's frame at its own time, `points_on_base` in the order of the
    /// points, lies inside the unpadded outline, and the points the sensor
    /// sees of everything else, each in its order.
    fn split_off_the_body(
        &self,
        points: &[[f32; 3]],
        points_on_base: &[[f64; 2]],
    ) -> (Vec<[f32; 3]>, Vec<[f32; 3]>) {
        assert_eq!(
            points.len(),
            points_on_base.len(),
            "each point of a frame has its place on the base"
        );
        split_points(
            points,
            points_on_base
                .iter()
                .map(|&on_base| self.footprint.outline_holds(on_base)),
        )
    }

    /// The points of `scan` split into those whose readings lie within the
    /// obstacle range ([`LiveObstacles::within_obstacle_range`]) and the
    /// others, each in its order.
    fn split_off_beyond_range(&self, scan: &Scan) -> (Vec<[f32; 3]>, Vec<[f32; 3]>) {
        let origin = scan.sensor.position();
        split_points(
            scan.points,
            scan.readings()
                .map(|reading| self.within_obstacle_range(origin, reading)),
        )
    }

    /// Whether `reading` lies within the obstacle range of the sensor at
    /// `origin` by their distance in the plane, the range included, as the
    /// obstacle layer keeps a point (obstacle_layer.cpp:549-553); a reading
    /// that is not a number lies beyond it.
    fn within_obstacle_range(&self, origin: [f64; 3], reading: Reading) -> bool {
        let (Reading::Return(at) | Reading::NoReturn(at)) = reading;
        (at[0] - origin[0]).hypot(at[1] - origin[1]) <= self.obstacle_max_range_m.get()
    }

    /// The place on the floor's plane of each return of `scan` whose height
    /// lies in the band, as the collision monitor keeps a point of its source
    /// (src/pointcloud.cpp:174).
    fn obstacle_points<'s>(&'s self, scan: &'s Scan) -> impl Iterator<Item = [f64; 2]> + 's {
        scan.readings().filter_map(|reading| match reading {
            Reading::Return([x, y, z]) if self.heights.is_obstacle(z) => Some([x, y]),
            _ => None,
        })
    }

    /// The columns the votes of `scan`'s readings within the obstacle range
    /// show a drop in ([`FloorVotes`]).
    fn drops_shown(&self, scan: &Scan) -> BTreeSet<Cell> {
        let origin = scan.sensor.position();
        let step_z = self.heights.step_plane_z();
        let mut votes: HashMap<Cell, FloorVotes> = HashMap::new();
        let within_range = scan
            .readings()
            .filter(|&reading| self.within_obstacle_range(origin, reading));
        for reading in within_range {
            if let Some(cell) = surface_at(reading, step_z).and_then(|at| self.columns.cell_of(at))
            {
                votes.entry(cell).or_default().surfaces += 1;
            }
            if let Some((crossing, read)) = read_through_floor(origin, reading, step_z)
                && let Some(cell) = self.columns.cell_of(crossing)
            {
                votes.entry(cell).or_default().count(read);
            }
        }
        votes
            .into_iter()
            .filter(|(_, votes)| votes.show_a_drop())
            .map(|(cell, _)| cell)
            .collect()
    }

    /// Clears every voxel of the columns under the footprint at `base`, and
    /// unmarks every drop among them, the base standing on floor there; none
    /// when a vertex lies off the map, as Nav2 then clears no cell.
    fn clear_footprint(&mut self, base: PlanarPose) {
        let Some(cells) = polygon_cells(&self.columns, &self.footprint.placed_at(base)) else {
            return;
        };
        for cell in &cells {
            self.drops.remove(cell);
        }
        self.map
            .clear_columns(cells.into_iter().map(|cell| [cell.x(), cell.y()]));
    }
}

/// `point` on the floor's plane: its x and y.
fn on_the_floor([x, y, _]: [f64; 3]) -> [f64; 2] {
    [x, y]
}

/// `points` split by `holds`, one flag a point in the order of the points:
/// the points it holds, then the others, each in its order.
fn split_points(
    points: &[[f32; 3]],
    holds: impl Iterator<Item = bool>,
) -> (Vec<[f32; 3]>, Vec<[f32; 3]>) {
    let (held, others): (Vec<_>, Vec<_>) = points.iter().zip(holds).partition(|&(_, holds)| holds);
    let points_of = |pairs: Vec<(&[f32; 3], bool)>| -> Vec<[f32; 3]> {
        pairs.into_iter().map(|(&point, _)| point).collect()
    };
    (points_of(held), points_of(others))
}

/// Where a return stands at or above the step's plane, on x and y: a surface
/// within the step of the floor or above it.
fn surface_at(reading: Reading, step_z: f64) -> Option<[f64; 2]> {
    match reading {
        Reading::Return([x, y, z]) if z >= step_z => Some([x, y]),
        _ => None,
    }
}

/// How a ray reads the floor's plane where its line crosses it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum FloorRead {
    /// Its reading lies below the step's plane.
    Past,
    /// It returned above the floor's plane, before the crossing.
    Short,
}

/// Where the line of the ray from `origin` to `reading` crosses the floor's
/// plane, and how the ray reads it there: past, for a reading below the step's
/// plane at `step_z`, a return or no return at the range, and short, for a
/// return above the floor's plane; none for any other reading, for a ray that
/// does not point down from above the floor, and for a reading that is not a
/// number.
fn read_through_floor(
    origin: [f64; 3],
    reading: Reading,
    step_z: f64,
) -> Option<([f64; 2], FloorRead)> {
    let (end, read) = match reading {
        Reading::Return(end) | Reading::NoReturn(end) if end[2] < step_z => (end, FloorRead::Past),
        Reading::Return(end) if end[2] > 0.0 => (end, FloorRead::Short),
        _ => return None,
    };
    if !(origin[2] > 0.0 && end[2] < origin[2]) {
        return None;
    }
    let along = origin[2] / (origin[2] - end[2]);
    Some((
        [0, 1].map(|i| origin[i] + along * (end[i] - origin[i])),
        read,
    ))
}

/// What the readings of one frame tell of one column: how many rays read past
/// the step's plane through its floor's plane, how many aimed there returned
/// short, above the floor's plane, and how many returns stand in it at or above
/// the step's plane, on its floor or above it.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct FloorVotes {
    past: u32,
    short: u32,
    surfaces: u32,
}

impl FloorVotes {
    fn count(&mut self, read: FloorRead) {
        match read {
            FloorRead::Past => self.past += 1,
            FloorRead::Short => self.short += 1,
        }
    }

    /// Whether the column is a drop: at least [`MIN_RAYS_PAST_THE_STEP`] rays
    /// read past the step's plane through it, more than returned short of it
    /// and stand in it together.
    fn show_a_drop(self) -> bool {
        self.past >= MIN_RAYS_PAST_THE_STEP && self.past > self.short + self.surfaces
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
    use frame_tree::UnitQuaternion;

    fn odom() -> FrameName {
        FrameName::parse("odom").unwrap()
    }

    fn bounds() -> Bounds {
        Bounds {
            min: [-1.0, -1.0, -0.5],
            max: [1.0, 1.0, 1.0],
        }
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

    /// An obstacle range of 5 m, past every column of [`bounds`] from a
    /// sensor over the map.
    fn past_the_map() -> PositiveFinite {
        PositiveFinite::parse(5.0).unwrap()
    }

    /// The band from 0.1 m to 0.5 m of voxels 0.05 m on a side, and a step of 0.1 m.
    fn heights() -> ObstacleHeights {
        ObstacleHeights::parse(ObstacleHeightsParams {
            min_obstacle_height_m: 0.1,
            max_obstacle_height_m: 0.5,
            max_step_down_m: 0.1,
            voxel_m: twentieth(),
        })
        .unwrap()
    }

    /// The live obstacles of one occupied voxel, in the band, over the
    /// column from (0, 0) to (0.05, 0.05), seen from straight above.
    fn one_occupied_voxel() -> LiveObstacles {
        let mut live = LiveObstacles::new(
            odom(),
            bounds(),
            heights(),
            past_the_map(),
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
        let away = PlanarPose::new(-5.0, 0.0, 0.0).unwrap();
        let sensor_on_base = Transform::new([5.025, 0.025, 1.0], UnitQuaternion::IDENTITY).unwrap();
        live.insert_depth_frame(
            0,
            &[[0.0, 0.0, -0.7]],
            away,
            sensor_on_base,
            PositiveFinite::parse(5.0).unwrap(),
        )
        .unwrap();
        live
    }

    // On grids of half the voxel's side, turned against the map, one column
    // marks exactly the cells whose squares share area with its square, each
    // share found apart from this code by clipping the cell against the square:
    // at a turn of 0.42237 rad, cell (101, 101), half under the square, among
    // them; at the pose (0.46, -0.42, 2.88), cell (418, 381), under it by
    // 0.85%, among them, and cell (415, 382), which the square never reaches,
    // not. Each set holds under a change of 1e-9 m in the square's side.
    #[test]
    fn a_column_marks_exactly_the_cells_its_square_overlaps() {
        let live = one_occupied_voxel();
        let half_a_voxel = PositiveFinite::parse(0.025).unwrap();
        let turned = (
            PlanarPose::new(0.0, 0.0, 0.42237).unwrap(),
            200,
            [-2.5, -2.5],
            vec![
                (99, 100),
                (99, 101),
                (99, 102),
                (100, 100),
                (100, 101),
                (100, 102),
                (101, 100),
                (101, 101),
                (101, 102),
            ],
        );
        let moved = (
            PlanarPose::new(0.46, -0.42, 2.88).unwrap(),
            800,
            [-10.0, -10.0],
            vec![
                (415, 381),
                (416, 381),
                (416, 382),
                (416, 383),
                (417, 381),
                (417, 382),
                (417, 383),
                (418, 381),
                (418, 382),
                (418, 383),
            ],
        );
        for (grid_from_map, size, origin, expected) in [turned, moved] {
            let geometry = GridGeometry::new(size, size, half_a_voxel, origin).unwrap();
            let layer = live.project(grid_from_map, geometry).unwrap();
            let lethal: BTreeSet<(u32, u32)> = geometry
                .cells()
                .filter(|&cell| layer.grid().cost(cell) == LETHAL)
                .map(|cell| (cell.x(), cell.y()))
                .collect();
            assert_eq!(lethal, expected.into_iter().collect(), "{grid_from_map:?}");
        }
    }

    // A finite placement whose corners lie past the largest double cells from the grid, on
    // a grid of 0.05 m cells, or past the grid, on a grid of 2 m cells, marks no cell; a map
    // that knows no voxel places none, and its layer is unknown.
    #[test]
    fn a_placement_past_the_grid_marks_no_cell() {
        let live = one_occupied_voxel();
        let geometry = GridGeometry::new(40, 40, twentieth(), [-1.0, -1.0]).unwrap();
        let far = PlanarPose::new(f64::MAX, 0.0, 0.0).unwrap();
        let two_meters =
            GridGeometry::new(40, 40, PositiveFinite::parse(2.0).unwrap(), [-1.0, -1.0]).unwrap();
        for geometry in [geometry, two_meters] {
            let layer = live.project(far, geometry).unwrap();
            assert!(
                layer.grid().costs().iter().all(|&cost| cost == UNKNOWN),
                "{geometry:?}"
            );
        }
        let empty = LiveObstacles::new(
            odom(),
            bounds(),
            heights(),
            past_the_map(),
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
        let layer = empty.project(far, geometry).unwrap();
        assert!(layer.grid().costs().iter().all(|&cost| cost == UNKNOWN));
    }

    // The stop's points of the newest frame are its returns placed in the grid's frame and
    // seen from the base: the one return, at (0.025, 0.025) in the map's frame, placed a
    // quarter turn about (1, 0) and seen from a base at (0.975, 0) heading along x, stands at
    // (0, 0.025), and the stop reads the grid it is given with the base on it. Live obstacles
    // recentered over bounds that hold the return, and over bounds that do not, keep it, as
    // the monitor reads a source's points whatever its cost map holds; a frame of a source
    // the live obstacles do not read is refused and changes nothing.
    #[test]
    fn the_newest_frame_points_are_placed_and_kept() {
        let mut live = one_occupied_voxel();
        let free = CostGrid::filled(live.columns(), FREE);
        let grid_from_map = PlanarPose::new(1.0, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        let base = PlanarPose::new(0.975, 0.0, 0.0).unwrap();
        let poses = GridPoses {
            grid_from_map,
            base,
        };
        let occupied = live.occupied_seen_from(&free, poses);
        assert!(std::ptr::eq(occupied.grid, &free) && occupied.base == base);
        let seen = |live: &LiveObstacles| live.occupied_seen_from(&free, poses).points;
        let at_the_return = |points: &[[f64; 2]]| matches!(points, [[x, y]] if x.abs() < 1e-12 && (y - 0.025).abs() < 1e-12);
        let points = seen(&live);
        assert!(at_the_return(&points), "{points:?}");
        for min_x in [0.0, 2.0] {
            let moved = live
                .recentered(Bounds {
                    min: [min_x, -1.0, -0.5],
                    max: [min_x + 2.0, 1.0, 1.0],
                })
                .unwrap();
            assert!(at_the_return(&seen(&moved)), "{:?}", seen(&moved));
        }
        let away = PlanarPose::new(-5.0, 0.0, 0.0).unwrap();
        let elsewhere = Transform::new([5.5, 0.5, 1.0], UnitQuaternion::IDENTITY).unwrap();
        let refused = live
            .insert_depth_frame(
                1,
                &[[0.0, 0.0, -0.7]],
                away,
                elsewhere,
                PositiveFinite::parse(5.0).unwrap(),
            )
            .unwrap_err();
        assert_eq!(
            refused,
            UnknownSource {
                index: 1,
                sources: 1
            }
        );
        assert_eq!(seen(&live), points);
        let lethal = |live: &LiveObstacles| {
            let layer = live.project(PlanarPose::IDENTITY, live.columns()).unwrap();
            layer.grid().costs().to_vec()
        };
        assert_eq!(lethal(&live), lethal(&one_occupied_voxel()));
    }

    // A frame's newest points are its returns' own places on the floor's plane whose heights
    // lie in the band, both ends included, as the monitor keeps a point: of four returns
    // over (0.01, 0.04), off its column's center, at 0.075 m, just above 0.1 m, at 0.5 m and
    // at 0.525 m, the two between the band's ends, the one at 0.5 m among them, which lies in
    // the voxel above the band and marks no cell.
    #[test]
    fn the_newest_points_are_the_returns_in_the_band_where_they_stand() {
        let mut live = LiveObstacles::new(
            odom(),
            bounds(),
            heights(),
            past_the_map(),
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
        let away = PlanarPose::new(-5.0, 0.0, 0.0).unwrap();
        let sensor_on_base = Transform::new([5.01, 0.04, 1.0], UnitQuaternion::IDENTITY).unwrap();
        live.insert_depth_frame(
            0,
            &[
                [0.0, 0.0, -0.925],
                [0.0, 0.0, -0.9],
                [0.0, 0.0, -0.5],
                [0.0, 0.0, -0.475],
            ],
            away,
            sensor_on_base,
            PositiveFinite::parse(5.0).unwrap(),
        )
        .unwrap();
        let points = &live.newest_points[0];
        assert_eq!(points.len(), 2, "{points:?}");
        for &[x, y] in points {
            assert!(
                (x - 0.01).abs() < 1e-12 && (y - 0.04).abs() < 1e-12,
                "{points:?}"
            );
        }
        let layer = live.project(PlanarPose::IDENTITY, live.columns()).unwrap();
        let lethal = layer.grid().costs().iter().filter(|&&cost| cost == LETHAL);
        assert_eq!(lethal.count(), 1);
    }

    // A grid whose last cells end inside a column's square marks the one cell
    // of the square it holds, (19, 19); a grid that ends a sliver before the
    // square, the square within the cell past its last, marks none, every
    // cell unknown.
    #[test]
    fn a_grid_that_ends_at_a_column_marks_only_the_cells_it_holds() {
        let live = one_occupied_voxel();
        let through = GridGeometry::new(20, 20, twentieth(), [-0.975, -0.975]).unwrap();
        let layer = live.project(PlanarPose::IDENTITY, through).unwrap();
        let lethal: Vec<Cell> = through
            .cells()
            .filter(|&cell| layer.grid().cost(cell) == LETHAL)
            .collect();
        assert_eq!(lethal, vec![through.cell(19, 19).unwrap()]);
        let before = GridGeometry::new(20, 20, twentieth(), [-1.0, -1.0]).unwrap();
        let layer = live.project(PlanarPose::IDENTITY, before).unwrap();
        assert!(layer.grid().costs().iter().all(|&cost| cost == UNKNOWN));
    }

    #[test]
    fn heights_want_numbers_a_band_above_the_floor_and_a_step() {
        assert!(matches!(
            ObstacleHeights::parse(ObstacleHeightsParams {
                min_obstacle_height_m: f64::NAN,
                max_obstacle_height_m: 0.5,
                max_step_down_m: 0.1,
                voxel_m: twentieth()
            }),
            Err(ObstacleHeightsError::NotFinite { .. })
        ));
        assert_eq!(
            ObstacleHeights::parse(ObstacleHeightsParams {
                min_obstacle_height_m: 0.0,
                max_obstacle_height_m: 0.5,
                max_step_down_m: 0.1,
                voxel_m: twentieth()
            }),
            Err(ObstacleHeightsError::FloorBand(0.0))
        );
        assert!(matches!(
            ObstacleHeights::parse(ObstacleHeightsParams {
                min_obstacle_height_m: 0.5,
                max_obstacle_height_m: 0.5,
                max_step_down_m: 0.1,
                voxel_m: twentieth()
            }),
            Err(ObstacleHeightsError::Inverted { .. })
        ));
        assert_eq!(
            ObstacleHeights::parse(ObstacleHeightsParams {
                min_obstacle_height_m: 0.1,
                max_obstacle_height_m: 0.5,
                max_step_down_m: 0.0,
                voxel_m: twentieth()
            }),
            Err(ObstacleHeightsError::StepDown(0.0))
        );
        let heights = heights();
        assert!(heights.is_obstacle(0.1) && heights.is_obstacle(0.5));
        assert!(!heights.is_obstacle(0.075) && !heights.is_obstacle(0.525));
        assert_eq!(heights.step_plane_z(), -0.1);
    }

    // A ray down from above the floor reads the floor's plane where its line
    // crosses it: past, for a return or a ray with no return that ends below
    // the step's plane, and short, for a return above the floor's plane, the
    // crossing then beyond the return along the ray.
    #[test]
    fn a_ray_down_reads_the_floor_plane_where_its_line_crosses_it() {
        let origin = [0.0, 0.0, 1.0];
        let step_z = -0.5;
        for (reading, read) in [
            (Reading::Return([4.0, 2.0, -1.0]), FloorRead::Past),
            (Reading::NoReturn([4.0, 2.0, -1.0]), FloorRead::Past),
            (Reading::Return([1.0, 0.5, 0.5]), FloorRead::Short),
        ] {
            assert_eq!(
                read_through_floor(origin, reading, step_z),
                Some(([2.0, 1.0], read)),
                "{reading:?}"
            );
        }
    }

    // A return on the floor or within the step below it, a ray with no return
    // that ends above the step's plane, a ray that points level or up, a sensor
    // at or below the floor and a reading that is not a number read nothing of
    // the floor's plane.
    #[test]
    fn a_ray_that_reads_neither_past_the_step_nor_above_the_floor_reads_nothing() {
        let origin = [0.0, 0.0, 1.0];
        let step_z = -0.5;
        for reading in [
            Reading::Return([2.0, 0.0, 0.0]),
            Reading::Return([2.0, 0.0, -0.5]),
            Reading::NoReturn([2.0, 0.0, -0.2]),
            Reading::Return([1.0, 0.0, 1.0]),
            Reading::Return([1.0, 0.0, 1.5]),
            Reading::Return([f64::NAN, 0.0, f64::NAN]),
            Reading::NoReturn([f64::NAN; 3]),
        ] {
            assert_eq!(
                read_through_floor(origin, reading, step_z),
                None,
                "{reading:?}"
            );
        }
        for below in [0.0, -0.2] {
            assert_eq!(
                read_through_floor([0.0, 0.0, below], Reading::Return([1.0, 0.0, -1.0]), step_z),
                None
            );
        }
    }

    // A return at or above the step's plane stands in its column; one below it,
    // a ray with no return and a reading that is not a number stand nowhere.
    #[test]
    fn a_surface_stands_where_a_return_lies_at_or_above_the_step_plane() {
        let step_z = -0.1;
        assert_eq!(
            surface_at(Reading::Return([1.0, 2.0, 0.0]), step_z),
            Some([1.0, 2.0])
        );
        assert_eq!(
            surface_at(Reading::Return([1.0, 2.0, -0.1]), step_z),
            Some([1.0, 2.0])
        );
        assert_eq!(
            surface_at(Reading::Return([1.0, 2.0, 0.4]), step_z),
            Some([1.0, 2.0])
        );
        for reading in [
            Reading::Return([1.0, 2.0, -0.2]),
            Reading::NoReturn([1.0, 2.0, 0.0]),
            Reading::Return([1.0, 2.0, f64::NAN]),
        ] {
            assert_eq!(surface_at(reading, step_z), None, "{reading:?}");
        }
    }

    // A column is a drop where at least three rays of a frame read past the
    // step's plane through it, more than returned short and stand in it
    // together: two past alone, or as many short or standing as past, marks
    // none.
    #[test]
    fn a_drop_needs_three_rays_past_outnumbering_those_short_and_standing() {
        let votes = |past: u32, short: u32, surfaces: u32| {
            FloorVotes {
                past,
                short,
                surfaces,
            }
            .show_a_drop()
        };
        assert!(!votes(2, 0, 0) && !votes(3, 3, 0) && !votes(5, 0, 5) && !votes(4, 2, 2));
        assert!(votes(3, 0, 0) && votes(3, 2, 0) && votes(5, 2, 2) && votes(9, 0, 8));
        let mut counted = FloorVotes::default();
        for read in [FloorRead::Past, FloorRead::Short, FloorRead::Past] {
            counted.count(read);
        }
        assert_eq!(
            counted,
            FloorVotes {
                past: 2,
                short: 1,
                surfaces: 0
            }
        );
    }

    // A height of the band off the voxels names the whole values either side of
    // it, and one within the floor's voxels names the least it may be, while the
    // step, a plane the rays read, takes any height above the floor; live
    // obstacles keep a map of the voxels the heights judge.
    #[test]
    fn the_band_heights_are_whole_voxels_of_the_map() {
        let refused = ObstacleHeights::parse(ObstacleHeightsParams {
            min_obstacle_height_m: 0.12,
            max_obstacle_height_m: 0.5,
            max_step_down_m: 0.1,
            voxel_m: twentieth(),
        })
        .unwrap_err();
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
            ObstacleHeights::parse(ObstacleHeightsParams {
                min_obstacle_height_m: 0.05,
                max_obstacle_height_m: 0.5,
                max_step_down_m: 0.1,
                voxel_m: PositiveFinite::parse(0.1).unwrap()
            }),
            Err(ObstacleHeightsError::WithinTheFloorVoxels {
                name: "min_obstacle_height_m",
                height_m: 0.05,
                voxel_m: 0.1,
            })
        );
        for step_m in [0.03, 0.12] {
            assert!(
                ObstacleHeights::parse(ObstacleHeightsParams {
                    min_obstacle_height_m: 0.1,
                    max_obstacle_height_m: 0.5,
                    max_step_down_m: step_m,
                    voxel_m: twentieth()
                })
                .is_ok()
            );
        }
        let whole = heights();
        let live = LiveObstacles::new(
            odom(),
            bounds(),
            whole,
            past_the_map(),
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
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
        let heights = ObstacleHeights::parse(ObstacleHeightsParams {
            min_obstacle_height_m: 0.1,
            max_obstacle_height_m: 0.5,
            max_step_down_m: 0.1,
            voxel_m: tenth,
        })
        .unwrap();
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
        too_many(
            LiveObstacles::new(
                odom(),
                long,
                heights,
                past_the_map(),
                square(),
                NonZeroUsize::MIN,
            )
            .unwrap_err(),
        );
        let live = LiveObstacles::new(
            odom(),
            bounds(),
            heights,
            past_the_map(),
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
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
            columns_of(&VoxelMap::new(FrameName::parse("odom").unwrap(), side, one_voxel).unwrap())
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

    /// The live obstacles over [`bounds`] whose returns mark within
    /// `obstacle_max_range_m` of their sensor in the plane, after one depth
    /// frame of `points` from a sensor at `sensor` in the map's frame, its axes
    /// the map's, the base off the map.
    fn seen_within(
        obstacle_max_range_m: f64,
        sensor: [f64; 3],
        points: &[[f32; 3]],
    ) -> LiveObstacles {
        let range = PositiveFinite::parse(obstacle_max_range_m).unwrap();
        let mut live = LiveObstacles::new(
            odom(),
            bounds(),
            heights(),
            range,
            square(),
            NonZeroUsize::MIN,
        )
        .unwrap();
        let away = PlanarPose::new(-5.0, 0.0, 0.0).unwrap();
        let [x, y, z] = sensor;
        let sensor_on_base = Transform::new([x + 5.0, y, z], UnitQuaternion::IDENTITY).unwrap();
        live.insert_depth_frame(0, points, away, sensor_on_base, past_the_map())
            .unwrap();
        live
    }

    /// The lethal columns of `live` on the grid of its columns.
    fn lethal_columns(live: &LiveObstacles) -> Vec<(u32, u32)> {
        let layer = live.project(PlanarPose::IDENTITY, live.columns()).unwrap();
        live.columns()
            .cells()
            .filter(|&cell| layer.grid().cost(cell) == LETHAL)
            .map(|cell| (cell.x(), cell.y()))
            .collect()
    }

    // A return marks within the obstacle range by its distance from the sensor's origin in
    // the plane, the range included, as the obstacle layer keeps a point: a return in the
    // band 1 m from the sensor in the plane and 1.18 m in 3-D marks its column, (22, 20),
    // with a range of 1 m, and no column with a range a nanometer shorter. The stop reads
    // the return as one of the source's newest points with either range.
    #[test]
    fn a_return_marks_within_the_obstacle_range_by_its_distance_in_the_plane() {
        let sensor = [-0.875, 0.025, 0.9];
        let ahead = [[1.0, 0.0, -0.625]];
        let at_the_range = seen_within(1.0, sensor, &ahead);
        assert_eq!(lethal_columns(&at_the_range), vec![(22, 20)]);
        let just_short = seen_within(1.0 - 1e-9, sensor, &ahead);
        assert_eq!(lethal_columns(&just_short), Vec::new());
        for live in [at_the_range, just_short] {
            assert_eq!(live.newest_points[0].len(), 1, "{:?}", live.newest_points);
        }
    }

    // Rays vote for a drop only where their readings lie within the obstacle range by
    // their distance in the plane: three returns of a floor 0.325 m down, 1 m from a sensor
    // 0.3 m up in the plane and 1.18 m in 3-D, read past the step's plane where their line
    // crosses the floor's plane, column (12, 20), and mark a drop there with a range of 1 m,
    // and none with a range a nanometer shorter.
    #[test]
    fn rays_vote_for_a_drop_only_within_the_obstacle_range_in_the_plane() {
        let sensor = [-0.875, 0.025, 0.3];
        let lower_floor = [[1.0, 0.0, -0.625]; 3];
        assert_eq!(
            lethal_columns(&seen_within(1.0, sensor, &lower_floor)),
            vec![(12, 20)]
        );
        assert_eq!(
            lethal_columns(&seen_within(1.0 - 1e-9, sensor, &lower_floor)),
            Vec::new()
        );
    }
}
