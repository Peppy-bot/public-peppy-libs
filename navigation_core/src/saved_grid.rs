//! A saved occupancy grid as Nav2 1.5.2 takes one into its static layer, from
//! a map's image or from an occupancy grid's cells. The image is read as
//! nav2_map_server's `loadMapFromFile` reads a trinary map that its YAML file
//! describes with `negate` 0 (src/map_io.cpp), each pixel's occupancy
//! `1 - p / 255` in single precision, occupied at or above the map's
//! `occupied_thresh` and free at or below its `free_thresh`, each threshold in
//! single precision as Eigen compares a float array with it, and unknown
//! between; the image's top row the grid's last, and the resolution in the
//! single precision of the occupancy grid's metadata, which the static layer
//! gives the cost map. The cells are an occupancy grid's values of a trinary
//! map, 0, 100 and -1 read unsigned as 255, as room_map:v1's map_grids carries
//! them, row by row from the first cell, at the resolution given. Each cell's
//! cost is the one `StaticLayer::interpretValue` gives it (nav2_costmap_2d
//! plugins/static_layer.cpp:282-299) with `track_unknown_space` and
//! `trinary_costmap` true and a lethal threshold of 100: lethal, free or
//! unknown.
//!
//! [`SavedGrid::placed_on`] places a saved grid's occupied cells on a grid in
//! another frame, as a voxel map's columns project onto a grid
//! ([`crate::grid`]), for a grid that rolls with the base under a saved map.
//!
//! [`MapExtent`] is the rectangle of the map frame that room_map:v1 gives a
//! map's grid, and tells whether a point lies on it.

use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;

use crate::grid::{
    CostGrid, FREE, GridError, GridGeometry, LETHAL, PlacementNotFinite, UNKNOWN,
    cells_overlapping, cells_under,
};

/// The thresholds of a trinary map, as its YAML file gives them: the
/// occupancy at or above which a pixel is occupied (`occupied_thresh`) and
/// the one at or below which it is free (`free_thresh`), each from 0 to 1,
/// the free one below the occupied one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapThresholds {
    occupied: f64,
    free: f64,
}

/// Why two numbers make no thresholds of a trinary map.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum MapThresholdsError {
    #[error("{field} {value} is no occupancy: give a number from 0 to 1")]
    OutOfRange { field: &'static str, value: f64 },
    #[error(
        "free_thresh {free} is not below occupied_thresh {occupied}: give a free threshold \
         below the occupied one"
    )]
    FreeNotBelowOccupied { free: f64, occupied: f64 },
}

impl MapThresholds {
    /// The thresholds `occupied_thresh` and `free_thresh` of a map's YAML
    /// file.
    pub fn parse(occupied: f64, free: f64) -> Result<Self, MapThresholdsError> {
        for (field, value) in [("occupied_thresh", occupied), ("free_thresh", free)] {
            if !(0.0..=1.0).contains(&value) {
                return Err(MapThresholdsError::OutOfRange { field, value });
            }
        }
        if free >= occupied {
            return Err(MapThresholdsError::FreeNotBelowOccupied { free, occupied });
        }
        Ok(Self { occupied, free })
    }

    /// A pixel's cost: its occupancy as map_io reads it, against the
    /// thresholds in single precision, as the static layer interprets the
    /// occupancy grid's value.
    fn cost_of(&self, pixel: u8) -> u8 {
        let occupancy = 1.0_f32 - f32::from(pixel) / 255.0_f32;
        if occupancy <= self.free as f32 {
            return FREE;
        }
        if occupancy >= self.occupied as f32 {
            return LETHAL;
        }
        UNKNOWN
    }
}

/// A saved grid's costs: [`LETHAL`], [`FREE`] or [`UNKNOWN`] a cell.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedGrid {
    grid: CostGrid,
}

/// Why an image or an occupancy grid's cells make no saved grid.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SavedGridError {
    #[error(transparent)]
    Grid(#[from] GridError),
    #[error(
        "{pixels} pixels fill no image of {width} by {height}: give one gray pixel a cell, row \
         by row from the top"
    )]
    PixelCount {
        pixels: usize,
        width: u32,
        height: u32,
    },
    #[error(
        "the resolution {0} m is no single precision number above 0, as an occupancy grid's \
         metadata carries it: give a resolution from 1e-38 m to 1e38 m"
    )]
    Resolution(f64),
    #[error(
        "{cells} cells fill no grid of {columns} columns and {rows} rows: give columns times \
         rows cells, row by row from the first"
    )]
    CellCount {
        cells: usize,
        columns: u32,
        rows: u32,
    },
    #[error(
        "cell {index} holds {value}, a value no cell of a trinary occupancy grid holds: give 0 \
         (free), 100 (occupied) or 255 (unknown, the grid's -1 read unsigned)"
    )]
    CellValue { index: usize, value: u8 },
}

impl SavedGrid {
    /// The grid of the 8-bit gray image `pixels`, `width` by `height`, row by
    /// row from the top, dark occupied, read at `thresholds`, of cells of side
    /// `resolution_m` in single precision whose first, the bottom left pixel,
    /// has its corner at `origin`: the image and the description of a map's
    /// YAML file.
    pub fn from_image(
        width: u32,
        height: u32,
        pixels: &[u8],
        resolution_m: PositiveFinite,
        origin: [f64; 2],
        thresholds: MapThresholds,
    ) -> Result<Self, SavedGridError> {
        let single = f64::from(resolution_m.get() as f32);
        let resolution = PositiveFinite::parse(single)
            .map_err(|_| SavedGridError::Resolution(resolution_m.get()))?;
        let geometry = GridGeometry::new(width, height, resolution, origin)?;
        if pixels.len() != geometry.cell_count() {
            return Err(SavedGridError::PixelCount {
                pixels: pixels.len(),
                width,
                height,
            });
        }
        let costs: Vec<u8> = pixels
            .chunks(width as usize)
            .rev()
            .flatten()
            .map(|&pixel| thresholds.cost_of(pixel))
            .collect();
        Ok(Self {
            grid: CostGrid::from_costs(geometry, costs)?,
        })
    }

    /// The grid of an occupancy grid's cells as the static layer takes them with
    /// `track_unknown_space` and `trinary_costmap` true and a lethal threshold of 100:
    /// `columns` by `rows` cells of side `cell_size_m` as given, the first with its corner at
    /// `origin`, row by row from it, each 0 (free), 100 (occupied) or 255 (unknown, the
    /// occupancy grid's -1 read unsigned). A count of cells other than `columns` times `rows`
    /// is refused, and so is another value, naming its index.
    pub fn from_cells(
        columns: u32,
        rows: u32,
        cell_size_m: PositiveFinite,
        origin: [f64; 2],
        cells: &[u8],
    ) -> Result<Self, SavedGridError> {
        let geometry = GridGeometry::new(columns, rows, cell_size_m, origin)?;
        if cells.len() != geometry.cell_count() {
            return Err(SavedGridError::CellCount {
                cells: cells.len(),
                columns,
                rows,
            });
        }
        let costs = cells
            .iter()
            .enumerate()
            .map(|(index, &value)| {
                static_cost(value).ok_or(SavedGridError::CellValue { index, value })
            })
            .collect::<Result<Vec<u8>, SavedGridError>>()?;
        Ok(Self {
            grid: CostGrid::from_costs(geometry, costs)?,
        })
    }

    /// The occupied cells of this grid on a grid of `geometry` in another frame, this grid
    /// placed in that frame at `grid_from_saved`: [`LETHAL`] each cell whose square overlaps
    /// the square of a lethal cell of this grid, set a sliver in from the cell's sides as a
    /// voxel map's column is, and [`UNKNOWN`] every other cell, so the placed grid, composed
    /// as the static layer, changes the cells under this grid's occupied cells alone. Only
    /// the cells of this grid whose squares overlap the target grid are read. A placement
    /// that puts a corner of a square at a point that is not finite is refused.
    pub fn placed_on(
        &self,
        grid_from_saved: PlanarPose,
        geometry: GridGeometry,
    ) -> Result<SavedGrid, PlacementNotFinite> {
        let saved = self.grid.geometry();
        let [x, y] = geometry.origin();
        let [width, height] = [geometry.size_x(), geometry.size_y()]
            .map(|cells| f64::from(cells) * geometry.resolution_m());
        let bounds = [
            [x, y],
            [x + width, y],
            [x + width, y + height],
            [x, y + height],
        ]
        .map(|corner| grid_from_saved.apply_inverse(corner));
        let under_the_target = cells_overlapping(&saved, bounds).ok_or(PlacementNotFinite {
            grid_from_map: grid_from_saved,
        })?;
        let mut grid = CostGrid::filled(geometry, UNKNOWN);
        for cell in under_the_target
            .into_iter()
            .filter(|&cell| self.grid.cost(cell) == LETHAL)
        {
            for marked in cells_under(&saved, cell, grid_from_saved, &geometry)? {
                grid.costs_mut()[geometry.index(marked)] = LETHAL;
            }
        }
        Ok(Self { grid })
    }

    pub fn grid(&self) -> &CostGrid {
        &self.grid
    }
}

/// The cost the static layer gives a trinary occupancy grid's value read unsigned, with
/// unknown space tracked and a lethal threshold of 100: free at 0, lethal at 100, and
/// unknown at 255, the grid's -1; none for a value no cell of a trinary grid holds.
fn static_cost(value: u8) -> Option<u8> {
    match value {
        0 => Some(FREE),
        100 => Some(LETHAL),
        255 => Some(UNKNOWN),
        _ => None,
    }
}

/// A map's extent as room_map:v1 gives it, `[x_min, x_max, y_min, y_max]` in meters in the
/// map frame: the closed rectangle that the map's grid covers, each edge finite and the
/// least edge of each axis at or below its greatest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapExtent {
    min: [f64; 2],
    max: [f64; 2],
}

/// Why four numbers make no map extent.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum MapExtentError {
    #[error(
        "the extent {0:?} holds an edge that is not a finite number: give [x_min, x_max, \
         y_min, y_max] in meters"
    )]
    NotFinite([f64; 4]),
    #[error(
        "{axis}_min {min} m lies above {axis}_max {max} m: give the least edge of each axis \
         before its greatest"
    )]
    Inverted {
        axis: &'static str,
        min: f64,
        max: f64,
    },
}

impl MapExtent {
    /// The extent `[x_min, x_max, y_min, y_max]` of room_map's map object and map_grids.
    pub fn parse(extent: [f64; 4]) -> Result<Self, MapExtentError> {
        if !extent.iter().all(|edge| edge.is_finite()) {
            return Err(MapExtentError::NotFinite(extent));
        }
        let [x_min, x_max, y_min, y_max] = extent;
        for (axis, min, max) in [("x", x_min, x_max), ("y", y_min, y_max)] {
            if min > max {
                return Err(MapExtentError::Inverted { axis, min, max });
            }
        }
        Ok(Self {
            min: [x_min, y_min],
            max: [x_max, y_max],
        })
    }

    /// The corner of least x and least y, `[x_min, y_min]`.
    pub fn min(&self) -> [f64; 2] {
        self.min
    }

    /// The corner of greatest x and greatest y, `[x_max, y_max]`.
    pub fn max(&self) -> [f64; 2] {
        self.max
    }

    /// Whether `point`, in meters in the map frame, lies on the extent's closed rectangle,
    /// its edges and corners among it.
    pub fn contains(&self, point: [f64; 2]) -> bool {
        (0..2).all(|axis| self.min[axis] <= point[axis] && point[axis] <= self.max[axis])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tenth() -> PositiveFinite {
        PositiveFinite::parse(0.1).unwrap()
    }

    /// The thresholds 0.65 and 0.25.
    fn ticket() -> MapThresholds {
        MapThresholds::parse(0.65, 0.25).unwrap()
    }

    #[test]
    fn thresholds_are_occupancies_with_the_free_one_below() {
        assert_eq!(
            MapThresholds::parse(1.2, 0.25),
            Err(MapThresholdsError::OutOfRange {
                field: "occupied_thresh",
                value: 1.2
            })
        );
        assert!(matches!(
            MapThresholds::parse(0.65, f64::NAN),
            Err(MapThresholdsError::OutOfRange {
                field: "free_thresh",
                ..
            })
        ));
        assert_eq!(
            MapThresholds::parse(0.4, 0.4),
            Err(MapThresholdsError::FreeNotBelowOccupied {
                free: 0.4,
                occupied: 0.4
            })
        );
        assert_eq!(
            MapThresholds::parse(0.4, 0.5).unwrap_err().to_string(),
            "free_thresh 0.5 is not below occupied_thresh 0.4: give a free threshold below the \
             occupied one"
        );
    }

    // At 0.65 and 0.25 no 8-bit pixel's occupancy equals a threshold in single
    // precision, so map_io's "at or above" and "at or below" pick the pixels
    // above and below them.
    #[test]
    fn no_pixel_lies_on_the_thresholds_of_0_65_and_0_25() {
        let thresholds = ticket();
        for pixel in 0..=u8::MAX {
            let occupancy = 1.0_f32 - f32::from(pixel) / 255.0_f32;
            assert_ne!(occupancy, thresholds.occupied as f32, "pixel {pixel}");
            assert_ne!(occupancy, thresholds.free as f32, "pixel {pixel}");
        }
    }

    // Black is lethal, white free and the gray between unknown, the image's
    // top row is the grid's last, and the resolution is single precision.
    #[test]
    fn dark_is_lethal_light_free_and_the_top_row_last() {
        let saved = SavedGrid::from_image(
            3,
            2,
            &[0, 89, 90, 254, 192, 191],
            tenth(),
            [0.0, 0.0],
            ticket(),
        )
        .unwrap();
        assert_eq!(
            saved.grid().costs(),
            [FREE, FREE, UNKNOWN, LETHAL, LETHAL, UNKNOWN]
        );
        assert!(matches!(
            SavedGrid::from_image(3, 2, &[0; 5], tenth(), [0.0; 2], ticket()),
            Err(SavedGridError::PixelCount { pixels: 5, .. })
        ));
        assert_eq!(saved.grid().geometry().resolution_m(), f64::from(0.1_f32));
        assert!(matches!(
            SavedGrid::from_image(
                3,
                2,
                &[0; 6],
                PositiveFinite::parse(1e-50).unwrap(),
                [0.0; 2],
                ticket()
            ),
            Err(SavedGridError::Resolution(_))
        ));
    }

    // map_io compares a pixel's single precision occupancy with each threshold
    // made single precision: a pixel whose occupancy lies a hair below
    // occupied_thresh in double precision, and on it in single, is occupied.
    #[test]
    fn a_threshold_compares_in_single_precision() {
        let occupancy = f64::from(1.0_f32 - f32::from(89_u8) / 255.0_f32);
        let thresholds = MapThresholds::parse(occupancy + 1e-12, 0.25).unwrap();
        let saved = SavedGrid::from_image(1, 1, &[89], tenth(), [0.0; 2], thresholds).unwrap();
        assert_eq!(saved.grid().costs(), [LETHAL]);
    }

    // An occupancy grid's cells read free, lethal and unknown, row by row from the first
    // with no row turned over, at the resolution given in double precision.
    #[test]
    fn cells_read_free_lethal_and_unknown_from_the_first() {
        let saved =
            SavedGrid::from_cells(3, 2, tenth(), [-1.0, 2.0], &[0, 100, 255, 255, 0, 100]).unwrap();
        assert_eq!(
            saved.grid().costs(),
            [FREE, LETHAL, UNKNOWN, UNKNOWN, FREE, LETHAL]
        );
        let geometry = saved.grid().geometry();
        assert_eq!(geometry.resolution_m(), 0.1);
        assert_eq!(geometry.origin(), [-1.0, 2.0]);
        assert_eq!((geometry.size_x(), geometry.size_y()), (3, 2));
    }

    // A value no cell of a trinary grid holds is refused naming its index and its value, a
    // count of cells other than columns times rows is refused, and a grid of no cell too.
    #[test]
    fn cells_want_trinary_values_one_a_cell() {
        for (value, index) in [(1, 2), (99, 0), (101, 5), (254, 3)] {
            let mut cells = [0; 6];
            cells[index] = value;
            assert_eq!(
                SavedGrid::from_cells(3, 2, tenth(), [0.0; 2], &cells),
                Err(SavedGridError::CellValue { index, value })
            );
        }
        assert_eq!(
            SavedGrid::from_cells(3, 2, tenth(), [0.0; 2], &[0, 50, 0, 0, 0, 0])
                .unwrap_err()
                .to_string(),
            "cell 1 holds 50, a value no cell of a trinary occupancy grid holds: give 0 (free), 100 \
             (occupied) or 255 (unknown, the grid's -1 read unsigned)"
        );
        assert_eq!(
            SavedGrid::from_cells(3, 2, tenth(), [0.0; 2], &[0; 5]),
            Err(SavedGridError::CellCount {
                cells: 5,
                columns: 3,
                rows: 2
            })
        );
        assert!(matches!(
            SavedGrid::from_cells(0, 2, tenth(), [0.0; 2], &[]),
            Err(SavedGridError::Grid(GridError::Empty { .. }))
        ));
    }

    /// The grid of `cells`, `columns` by `rows` of 1 m from the origin.
    fn meters(columns: u32, rows: u32, cells: &[u8]) -> SavedGrid {
        let meter = PositiveFinite::parse(1.0).unwrap();
        SavedGrid::from_cells(columns, rows, meter, [0.0; 2], cells).unwrap()
    }

    // On its own lattice each occupied cell marks its own cell alone, and a free or unknown
    // cell marks nothing; a quarter turn about the origin carries the cells of a row to a
    // column; and a cell placed half a cell along each axis marks the four cells its square
    // overlaps.
    #[test]
    fn a_placed_grid_marks_the_cells_its_occupied_squares_overlap() {
        let row = meters(4, 1, &[100, 0, 255, 100]);
        let own = row
            .placed_on(PlanarPose::IDENTITY, row.grid().geometry())
            .unwrap();
        assert_eq!(own.grid().costs(), [LETHAL, UNKNOWN, UNKNOWN, LETHAL]);
        let column =
            GridGeometry::new(1, 4, PositiveFinite::parse(1.0).unwrap(), [-1.0, 0.0]).unwrap();
        let quarter = PlanarPose::new(0.0, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        let turned = row.placed_on(quarter, column).unwrap();
        assert_eq!(turned.grid().costs(), [LETHAL, UNKNOWN, UNKNOWN, LETHAL]);
        let three =
            GridGeometry::new(3, 3, PositiveFinite::parse(1.0).unwrap(), [-1.0, -1.0]).unwrap();
        let half_along = PlanarPose::new(0.5, 0.5, 0.0).unwrap();
        let moved = meters(1, 1, &[100]).placed_on(half_along, three).unwrap();
        let lethal: Vec<usize> = moved
            .grid()
            .costs()
            .iter()
            .enumerate()
            .filter(|&(_, &cost)| cost == LETHAL)
            .map(|(index, _)| index)
            .collect();
        assert_eq!(lethal, [4, 5, 7, 8]);
    }

    // A grid placed away from the target marks nothing, and a placement that carries the
    // target's corners past the finite numbers in the saved grid's frame is refused, naming
    // the placement.
    #[test]
    fn a_grid_placed_away_marks_nothing_and_past_the_finite_numbers_is_refused() {
        let row = meters(4, 1, &[100, 100, 100, 100]);
        let target = row.grid().geometry();
        let away = PlanarPose::new(10.0, 0.0, 0.0).unwrap();
        let placed = row.placed_on(away, target).unwrap();
        assert!(placed.grid().costs().iter().all(|&cost| cost == UNKNOWN));
        let far = PlanarPose::new(-f64::MAX, -f64::MAX, std::f64::consts::FRAC_PI_4).unwrap();
        let refused = row.placed_on(far, target).unwrap_err();
        assert_eq!(refused, PlacementNotFinite { grid_from_map: far });
        assert!(
            refused
                .to_string()
                .starts_with("the map placed in the grid's frame at x "),
            "{refused}"
        );
    }

    // An extent takes finite edges, the least of each axis at or below its greatest, and
    // holds every point of its closed rectangle, its edges and corners among them, and no
    // point past an edge by a double or not a number.
    #[test]
    fn an_extent_is_a_closed_rectangle_of_finite_edges() {
        let extent = MapExtent::parse([-1.0, 2.0, 0.5, 0.5]).unwrap();
        assert_eq!((extent.min(), extent.max()), ([-1.0, 0.5], [2.0, 0.5]));
        for point in [[-1.0, 0.5], [2.0, 0.5], [0.3, 0.5]] {
            assert!(extent.contains(point), "{point:?}");
        }
        for point in [
            [(-1.0_f64).next_down(), 0.5],
            [2.0_f64.next_up(), 0.5],
            [0.0, 0.5_f64.next_up()],
            [0.0, 0.5_f64.next_down()],
            [f64::NAN, 0.5],
        ] {
            assert!(!extent.contains(point), "{point:?}");
        }
        assert!(matches!(
            MapExtent::parse([0.0, f64::INFINITY, 0.0, 1.0]),
            Err(MapExtentError::NotFinite(_))
        ));
        assert_eq!(
            MapExtent::parse([0.0, 1.0, 2.0, 1.5]),
            Err(MapExtentError::Inverted {
                axis: "y",
                min: 2.0,
                max: 1.5
            })
        );
        assert_eq!(
            MapExtent::parse([3.0, 1.0, 0.0, 1.0])
                .unwrap_err()
                .to_string(),
            "x_min 3 m lies above x_max 1 m: give the least edge of each axis before its greatest"
        );
    }

    // A map as map_saver writes it in trinary mode, its occupied pixels 0, its
    // free ones 254 and its unknown ones 205, with occupied_thresh 0.65 and
    // free_thresh 0.196 in its YAML file, reads back lethal, free and unknown.
    #[test]
    fn a_map_saver_map_reads_back_lethal_free_and_unknown() {
        let thresholds = MapThresholds::parse(0.65, 0.196).unwrap();
        let saved =
            SavedGrid::from_image(3, 1, &[0, 254, 205], tenth(), [0.0; 2], thresholds).unwrap();
        assert_eq!(saved.grid().costs(), [LETHAL, FREE, UNKNOWN]);
    }
}
