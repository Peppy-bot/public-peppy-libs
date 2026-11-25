//! The cost grid, as Nav2 1.5.2 holds one (nav2_costmap_2d `Costmap2D`,
//! src/costmap_2d.cpp): one byte of cost per square cell, the cells laid out
//! row by row from the cell at the grid's origin, and Nav2's cost values
//! (include/nav2_costmap_2d/cost_values.hpp). [`GridGeometry::cell_of`] is
//! `Costmap2D::worldToMap`, [`GridGeometry::center_of`] is `mapToWorld` and
//! the planner locates its start and goal as `worldToMapContinuous` does,
//! each in its arithmetic.
//!
//! A cell of one grid placed in another grid's frame lies under each cell of
//! that grid whose square its own square overlaps by more than a line or a
//! point, its square set a sliver in from the cell's sides; a voxel map's
//! columns project onto a grid this way, and so do a saved grid's occupied
//! cells. A placement that puts a corner of such a square at a point that is
//! not finite is refused ([`PlacementNotFinite`]).

use std::ops::RangeInclusive;

use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;

/// A cell nothing occupies.
pub const FREE: u8 = 0;
/// The highest cost of a cell that the base's center may cross with its
/// footprint clear of every obstacle (`MAX_NON_OBSTACLE`).
pub const MAX_NON_OBSTACLE: u8 = 252;
/// A cell within the footprint's inscribed radius of an obstacle: the base's
/// center there puts the base on it (`INSCRIBED_INFLATED_OBSTACLE`).
pub const INSCRIBED: u8 = 253;
/// A cell an obstacle occupies (`LETHAL_OBSTACLE`).
pub const LETHAL: u8 = 254;
/// A cell that no sensor has seen (`NO_INFORMATION`).
pub const UNKNOWN: u8 = 255;

/// The most cells a grid's axis holds: the inflation's distance transform
/// squares a cell's index in a 32-bit signed integer, as Nav2 does
/// (`DistanceTransform::distanceTransform1D`), and the last index of an axis
/// of 46,341 cells squares to 46,340² = 2,147,395,600, at most 2³¹ − 1.
pub const MAX_AXIS_CELLS: u32 = 46_341;

/// How far in from its cell's sides the square of a cell placed on another
/// grid sets its corners, as a share of the cell's side: single precision's
/// epsilon times the most cells a grid's axis holds. A saved grid read from a
/// map's image holds its resolution in single precision, as an `OccupancyGrid`
/// does, so its cells and the voxels of the same side part by at most half
/// that epsilon a cell, and its lattice lies within half this share of the
/// voxel lattice over any axis a grid holds: the square of each column
/// overlaps the column's own cell alone there, as on a grid of the map's own
/// lattice.
pub(crate) const CELL_SQUARE_INSET: f64 = f32::EPSILON as f64 * MAX_AXIS_CELLS as f64;

/// Where a grid lies: its size in cells on each axis, the side of a cell, and
/// the corner of its first cell, in meters in the grid's frame. The corner of
/// its first cell and the far corner of its last, the origin plus its size in
/// cells times their side on each axis, lie within single precision's range,
/// as Smac 2D places a grid's cells from its origin in single precision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridGeometry {
    size_x: u32,
    size_y: u32,
    resolution_m: f64,
    origin: [f64; 2],
}

/// Why a size, a resolution and an origin make no grid.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum GridError {
    #[error("a grid of {size_x} by {size_y} cells holds no cell: give each axis at least one cell")]
    Empty { size_x: u32, size_y: u32 },
    #[error(
        "a grid of {size_x} by {size_y} cells has an axis of more than {} cells, whose last \
         index Nav2's distance transform cannot square: give fewer cells or larger ones",
        MAX_AXIS_CELLS
    )]
    AxisTooLong { size_x: u32, size_y: u32 },
    #[error(
        "the origin {0:?} is not a point single precision holds, as Smac 2D places a grid's \
         cells from it: give finite coordinates in meters from -{max:e} to {max:e}",
        max = f32::MAX
    )]
    Origin([f64; 2]),
    #[error(
        "the grid's far corner {0:?}, its origin plus its size in cells times their side, is \
         not a point single precision holds, as Smac 2D places a grid's cells: give fewer \
         cells, smaller ones, or an origin nearer 0"
    )]
    Extent([f64; 2]),
    #[error(
        "{costs} costs fill no grid of {size_x} by {size_y} cells: give one cost a cell, row by \
         row"
    )]
    CostCount {
        costs: usize,
        size_x: u32,
        size_y: u32,
    },
}

impl GridGeometry {
    /// The grid of `size_x` by `size_y` cells of side `resolution_m` whose
    /// first cell has its corner at `origin`, the origin and the far corner
    /// within single precision's range.
    pub fn new(
        size_x: u32,
        size_y: u32,
        resolution_m: PositiveFinite,
        origin: [f64; 2],
    ) -> Result<Self, GridError> {
        if size_x == 0 || size_y == 0 {
            return Err(GridError::Empty { size_x, size_y });
        }
        if size_x > MAX_AXIS_CELLS || size_y > MAX_AXIS_CELLS {
            return Err(GridError::AxisTooLong { size_x, size_y });
        }
        if !held_in_single_precision(origin) {
            return Err(GridError::Origin(origin));
        }
        let far_corner = [
            origin[0] + f64::from(size_x) * resolution_m.get(),
            origin[1] + f64::from(size_y) * resolution_m.get(),
        ];
        if !held_in_single_precision(far_corner) {
            return Err(GridError::Extent(far_corner));
        }
        Ok(Self {
            size_x,
            size_y,
            resolution_m: resolution_m.get(),
            origin,
        })
    }

    pub fn size_x(&self) -> u32 {
        self.size_x
    }

    pub fn size_y(&self) -> u32 {
        self.size_y
    }

    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    pub fn origin(&self) -> [f64; 2] {
        self.origin
    }

    /// How many cells the grid holds.
    pub fn cell_count(&self) -> usize {
        self.size_x as usize * self.size_y as usize
    }

    /// The cell `x` along and `y` up, when the grid holds it.
    pub fn cell(&self, x: u32, y: u32) -> Option<Cell> {
        (x < self.size_x && y < self.size_y).then_some(Cell { x, y })
    }

    /// The cell holding `point`, when the grid holds it: the point less the
    /// origin, divided by the resolution and truncated on each axis.
    pub fn cell_of(&self, point: [f64; 2]) -> Option<Cell> {
        let [wx, wy] = point;
        if !(wx >= self.origin[0] && wy >= self.origin[1]) {
            return None;
        }
        let mx = ((wx - self.origin[0]) / self.resolution_m) as u32;
        let my = ((wy - self.origin[1]) / self.resolution_m) as u32;
        self.cell(mx, my)
    }

    /// The center of `cell`.
    pub fn center_of(&self, cell: Cell) -> [f64; 2] {
        [
            self.origin[0] + (f64::from(cell.x) + 0.5) * self.resolution_m,
            self.origin[1] + (f64::from(cell.y) + 0.5) * self.resolution_m,
        ]
    }

    /// `point` in the grid's continuous cell coordinates, in single
    /// precision, when the grid holds it.
    pub(crate) fn locate(&self, point: [f64; 2]) -> Option<GridPoint> {
        let [wx, wy] = point;
        if !(wx >= self.origin[0] && wy >= self.origin[1]) {
            return None;
        }
        let mx = ((wx - self.origin[0]) / self.resolution_m) as f32;
        let my = ((wy - self.origin[1]) / self.resolution_m) as f32;
        (mx < self.size_x as f32 && my < self.size_y as f32).then_some(GridPoint {
            world: point,
            map: [mx, my],
        })
    }

    /// Where `cell` lies in the grid's row-by-row storage.
    pub(crate) fn index(&self, cell: Cell) -> usize {
        cell.y as usize * self.size_x as usize + cell.x as usize
    }

    /// The cell at `index` of the grid's row-by-row storage.
    pub(crate) fn cell_at(&self, index: usize) -> Cell {
        let size_x = self.size_x as usize;
        Cell {
            x: (index % size_x) as u32,
            y: (index / size_x) as u32,
        }
    }

    /// Every cell of the grid, row by row.
    pub fn cells(&self) -> impl Iterator<Item = Cell> + use<> {
        let geometry = *self;
        (0..self.cell_count()).map(move |index| geometry.cell_at(index))
    }
}

/// Whether each coordinate of `point` stays finite in single precision, as
/// `as f32` rounds it.
fn held_in_single_precision(point: [f64; 2]) -> bool {
    point.iter().all(|&c| (c as f32).is_finite())
}

/// A cell of a grid: one that the grid's geometry gave, so it lies inside
/// that grid and every grid of the same size.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Cell {
    x: u32,
    y: u32,
}

impl Cell {
    pub fn x(self) -> u32 {
        self.x
    }

    pub fn y(self) -> u32 {
        self.y
    }
}

/// A point inside a grid: in meters in the grid's frame, and in the grid's
/// continuous cell coordinates, the point less the origin over the
/// resolution, in single precision as `worldToMapContinuous` gives them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GridPoint {
    world: [f64; 2],
    map: [f32; 2],
}

impl GridPoint {
    pub(crate) fn world(self) -> [f64; 2] {
        self.world
    }

    pub(crate) fn map(self) -> [f32; 2] {
        self.map
    }

    /// The cell holding the point: its cell coordinates truncated.
    pub(crate) fn cell(self) -> Cell {
        Cell {
            x: self.map[0] as u32,
            y: self.map[1] as u32,
        }
    }
}

/// A grid of costs: one cost a cell over a [`GridGeometry`].
#[derive(Debug, Clone, PartialEq)]
pub struct CostGrid {
    geometry: GridGeometry,
    costs: Vec<u8>,
}

impl CostGrid {
    /// The grid over `geometry` with every cell at `cost`.
    pub fn filled(geometry: GridGeometry, cost: u8) -> Self {
        Self {
            geometry,
            costs: vec![cost; geometry.cell_count()],
        }
    }

    /// The grid over `geometry` of `costs`, one a cell, row by row from the
    /// first cell.
    pub fn from_costs(geometry: GridGeometry, costs: Vec<u8>) -> Result<Self, GridError> {
        if costs.len() != geometry.cell_count() {
            return Err(GridError::CostCount {
                costs: costs.len(),
                size_x: geometry.size_x,
                size_y: geometry.size_y,
            });
        }
        Ok(Self { geometry, costs })
    }

    pub fn geometry(&self) -> GridGeometry {
        self.geometry
    }

    /// The cost of `cell`.
    ///
    /// # Panics
    ///
    /// On a cell outside the grid: a cell of a larger grid.
    pub fn cost(&self, cell: Cell) -> u8 {
        let geometry = self.geometry;
        assert!(
            cell.x < geometry.size_x && cell.y < geometry.size_y,
            "cell ({}, {}) lies outside the grid's {} by {} cells",
            cell.x,
            cell.y,
            geometry.size_x,
            geometry.size_y
        );
        self.costs[geometry.index(cell)]
    }

    /// Every cell's cost, row by row from the first cell.
    pub fn costs(&self) -> &[u8] {
        &self.costs
    }

    pub(crate) fn costs_mut(&mut self) -> &mut [u8] {
        &mut self.costs
    }
}

/// A placement of a map's grid in another grid's frame that puts a corner of
/// a cell's square at a point that is not finite.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "the map placed in the grid's frame at x {} m, y {} m, yaw {} rad puts a cell's corner at \
     a point that is not finite: give an x and a y that, moved by the map's extent, stay finite",
    .grid_from_map.x(),
    .grid_from_map.y(),
    .grid_from_map.yaw()
)]
pub struct PlacementNotFinite {
    pub grid_from_map: PlanarPose,
}

/// The cells of `target` whose squares overlap the square of `cell` of
/// `source`, the source grid placed in the target's frame at
/// `target_from_source`, the square's corners [`CELL_SQUARE_INSET`] of a cell
/// in from the cell's sides. A placement that puts a corner of the square at a
/// point that is not finite is refused.
pub(crate) fn cells_under(
    source: &GridGeometry,
    cell: Cell,
    target_from_source: PlanarPose,
    target: &GridGeometry,
) -> Result<Vec<Cell>, PlacementNotFinite> {
    let [x, y] = source.center_of(cell);
    let half = source.resolution_m() * (0.5 - CELL_SQUARE_INSET);
    let square = [[-half, -half], [half, -half], [half, half], [-half, half]]
        .map(|[dx, dy]| target_from_source.apply([x + dx, y + dy]));
    cells_overlapping(target, square).ok_or(PlacementNotFinite {
        grid_from_map: target_from_source,
    })
}

/// The cells of `geometry` whose squares overlap the parallelogram of
/// `corners`, in order, in meters in the grid's frame, by more than a line or
/// a point: the cells within the corners' extent along each of the grid's
/// axes that no normal of the parallelogram's sides parts from it, as the
/// separating axes of two convex polygons find their overlap. None when a
/// corner is not finite. No cell when the parallelogram's extent on the
/// grid's lattice, its corners' offsets from the origin over the resolution,
/// is not finite, the cells of the grid that it overlaps among them, as the
/// separating axes decide the overlap on that extent.
pub(crate) fn cells_overlapping(
    geometry: &GridGeometry,
    corners: [[f64; 2]; 4],
) -> Option<Vec<Cell>> {
    if !all_finite(&corners) {
        return None;
    }
    let origin = geometry.origin();
    let resolution = geometry.resolution_m();
    let lattice = corners.map(|corner| [0, 1].map(|i| (corner[i] - origin[i]) / resolution));
    if !all_finite(&lattice) {
        return Some(Vec::new());
    }
    let cells_along = |axis: [f64; 2], size: u32| -> Option<RangeInclusive<u32>> {
        let [low, high] = extent_along(axis, &lattice);
        let first = low.floor().max(0.0);
        let last = (high.ceil() - 1.0).min(f64::from(size) - 1.0);
        (first <= last).then_some(first as u32..=last as u32)
    };
    let (Some(xs), Some(ys)) = (
        cells_along([1.0, 0.0], geometry.size_x()),
        cells_along([0.0, 1.0], geometry.size_y()),
    ) else {
        return Some(Vec::new());
    };
    let normals = [[0, 1], [1, 2]].map(|[from, to]| {
        let [dx, dy] = [0, 1].map(|i| lattice[to][i] - lattice[from][i]);
        [-dy, dx]
    });
    let overlaps = |x: u32, y: u32| {
        let (x, y) = (f64::from(x), f64::from(y));
        let cell = [[x, y], [x + 1.0, y], [x + 1.0, y + 1.0], [x, y + 1.0]];
        normals.iter().all(|&normal| {
            let [cell_low, cell_high] = extent_along(normal, &cell);
            let [low, high] = extent_along(normal, &lattice);
            cell_low < high && low < cell_high
        })
    };
    Some(
        xs.flat_map(|x| ys.clone().map(move |y| (x, y)))
            .filter(|&(x, y)| overlaps(x, y))
            .map(|(x, y)| geometry.cell(x, y).expect("an extent clamped to the grid"))
            .collect(),
    )
}

/// Whether every coordinate of `points` is finite.
fn all_finite(points: &[[f64; 2]]) -> bool {
    points
        .iter()
        .flatten()
        .all(|coordinate| coordinate.is_finite())
}

/// The least and the greatest of `points` along `axis`.
fn extent_along(axis: [f64; 2], points: &[[f64; 2]]) -> [f64; 2] {
    points
        .iter()
        .map(|point| point[0] * axis[0] + point[1] * axis[1])
        .fold([f64::INFINITY, f64::NEG_INFINITY], |[low, high], along| {
            [low.min(along), high.max(along)]
        })
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    fn geometry() -> GridGeometry {
        GridGeometry::new(4, 3, PositiveFinite::parse(0.5).unwrap(), [-1.0, 2.0]).unwrap()
    }

    #[test]
    fn a_grid_wants_cells_an_index_holds_and_a_finite_origin() {
        let half = PositiveFinite::parse(0.5).unwrap();
        assert_eq!(
            GridGeometry::new(0, 3, half, [0.0; 2]),
            Err(GridError::Empty {
                size_x: 0,
                size_y: 3
            })
        );
        assert_eq!(
            GridGeometry::new(MAX_AXIS_CELLS + 1, 3, half, [0.0; 2]),
            Err(GridError::AxisTooLong {
                size_x: MAX_AXIS_CELLS + 1,
                size_y: 3
            })
        );
        assert!(GridGeometry::new(3, MAX_AXIS_CELLS, half, [0.0; 2]).is_ok());
        assert!(matches!(
            GridGeometry::new(2, 2, half, [f64::NAN, 0.0]),
            Err(GridError::Origin(_))
        ));
        assert!(matches!(
            CostGrid::from_costs(geometry(), vec![0; 11]),
            Err(GridError::CostCount { costs: 11, .. })
        ));
    }

    // An origin or a far corner that single precision does not hold is
    // refused, as Smac 2D places the cells from the origin in single
    // precision: an origin past f32::MAX, and four cells of 1e38 m from 0,
    // whose far corner lies at 4e38 m; three such cells, and a cell whose
    // origin and far corner lie at f32::MAX, are held.
    #[test]
    fn a_grid_wants_an_origin_and_an_extent_single_precision_holds() {
        let largest = f64::from(f32::MAX);
        let past = f64::from(f32::MAX) * (1.0 + f64::from(f32::EPSILON));
        let half = PositiveFinite::parse(0.5).unwrap();
        for origin in [[past, 0.0], [0.0, -past], [1e39, 0.0]] {
            let refused = GridGeometry::new(2, 2, half, origin).unwrap_err();
            assert_eq!(refused, GridError::Origin(origin));
            assert!(
                refused
                    .to_string()
                    .contains("from -3.4028235e38 to 3.4028235e38"),
                "{refused}"
            );
        }
        let wide = PositiveFinite::parse(1e38).unwrap();
        assert_eq!(
            GridGeometry::new(4, 1, wide, [0.0; 2]),
            Err(GridError::Extent([4e38, 1e38]))
        );
        assert!(GridGeometry::new(3, 1, wide, [0.0; 2]).is_ok());
        assert!(GridGeometry::new(1, 1, half, [largest, -largest]).is_ok());
    }

    // A point maps to the cell holding it, a cell's center maps back to it,
    // and a point off the grid or not a number has no cell.
    #[test]
    fn a_point_lies_in_the_cell_holding_it() {
        let grid = geometry();
        let cell = grid.cell_of([0.2, 2.9]).expect("inside");
        assert_eq!((cell.x(), cell.y()), (2, 1));
        assert_eq!(grid.center_of(cell), [0.25, 2.75]);
        assert_eq!(grid.cell_of(grid.center_of(cell)), Some(cell));
        for outside in [
            [-1.01, 2.5],
            [0.0, 1.99],
            [1.0, 2.5],
            [0.0, 3.5],
            [f64::NAN, 2.5],
        ] {
            assert_eq!(grid.cell_of(outside), None, "{outside:?}");
            assert_eq!(grid.locate(outside), None, "{outside:?}");
        }
        let point = grid.locate([0.2, 2.9]).expect("inside");
        assert_eq!(point.map(), [2.4, 1.8]);
        assert_eq!(point.cell(), cell);
        assert_eq!(grid.cell_at(grid.index(cell)), cell);
        assert_eq!(grid.cells().count(), 12);
    }

    // A cell that meets a square at a line or a point alone is not under it:
    // on a grid of 1 m cells, a square whose sides lie on cell boundaries
    // covers its own cells and not the cells beside it; a square turned an
    // eighth of a turn, its corners at the middles of cells' sides, covers the
    // five cells it shares area with and not the four whose corners touch it;
    // and one whose corner lies on a boundary between two cells' sides covers
    // the six cells it shares area with and not the cell its corner touches,
    // which only the grid's own axis parts from it.
    #[test]
    fn a_cell_that_meets_a_square_at_a_line_or_a_point_alone_is_not_under_it() {
        let geometry =
            GridGeometry::new(5, 5, PositiveFinite::parse(1.0).unwrap(), [0.0; 2]).unwrap();
        let cells = |corners| -> BTreeSet<(u32, u32)> {
            cells_overlapping(&geometry, corners)
                .unwrap()
                .into_iter()
                .map(|cell| (cell.x(), cell.y()))
                .collect()
        };
        let on_boundaries = [[1.0, 1.0], [3.0, 1.0], [3.0, 2.0], [1.0, 2.0]];
        assert_eq!(cells(on_boundaries), BTreeSet::from([(1, 1), (2, 1)]));
        let turned = [[1.5, 0.5], [2.5, 1.5], [1.5, 2.5], [0.5, 1.5]];
        assert_eq!(
            cells(turned),
            BTreeSet::from([(1, 0), (0, 1), (1, 1), (2, 1), (1, 2)])
        );
        let pointed = [[2.0, 0.5], [3.0, 1.5], [2.0, 2.5], [1.0, 1.5]];
        assert_eq!(
            cells(pointed),
            BTreeSet::from([(1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)])
        );
    }

    // A square with a corner that is not finite overlaps no cell and is
    // refused; a square standing on a corner at the origin of a grid of cells
    // the least normal double wide, which overlaps three of the grid's cells,
    // and whose top corner alone lies past the largest double cells up the
    // grid's lattice, the corner neither side's normal is drawn from, overlaps
    // none.
    #[test]
    fn a_square_is_refused_only_where_a_corner_is_not_finite() {
        let geometry =
            GridGeometry::new(40, 40, PositiveFinite::parse(0.05).unwrap(), [-1.0, -1.0]).unwrap();
        for corner in [[f64::NAN, 0.0], [0.0, f64::INFINITY]] {
            let square = [corner, [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
            assert_eq!(cells_overlapping(&geometry, square), None, "{corner:?}");
        }
        let finest = GridGeometry::new(
            2,
            2,
            PositiveFinite::parse(f64::MIN_POSITIVE).unwrap(),
            [0.0; 2],
        )
        .unwrap();
        let standing = [[-2.0, 2.0], [0.0, 0.0], [2.0, 2.0], [0.0, 4.0]];
        assert_eq!(cells_overlapping(&finest, standing), Some(Vec::new()));
    }

    // A cell placed half a cell along each axis lies under the four cells its
    // square overlaps, and one placed on a cell of the target's lattice under
    // that cell alone.
    #[test]
    fn a_placed_cell_lies_under_the_cells_its_square_overlaps() {
        let meter = PositiveFinite::parse(1.0).unwrap();
        let source = GridGeometry::new(1, 1, meter, [0.0; 2]).unwrap();
        let target = GridGeometry::new(3, 3, meter, [-1.0, -1.0]).unwrap();
        let cell = source.cell(0, 0).unwrap();
        let half_along = PlanarPose::new(0.5, 0.5, 0.0).unwrap();
        let under: BTreeSet<(u32, u32)> = cells_under(&source, cell, half_along, &target)
            .unwrap()
            .into_iter()
            .map(|cell| (cell.x(), cell.y()))
            .collect();
        assert_eq!(under, BTreeSet::from([(1, 1), (2, 1), (1, 2), (2, 2)]));
        let one_along = PlanarPose::new(1.0, 0.0, 0.0).unwrap();
        let on_lattice = cells_under(&source, cell, one_along, &target).unwrap();
        assert_eq!(on_lattice, vec![target.cell(2, 1).unwrap()]);
    }
}
