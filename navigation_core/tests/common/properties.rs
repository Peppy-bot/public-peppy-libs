//! The Hegel generators of this crate's property tests: small cost grids with cells of
//! every kind, the cells of a grid a base's center may stand on, and the tolerance the
//! properties compare path costs at.

use control_core::positive_finite::PositiveFinite;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::grid::{
    Cell, CostGrid, FREE, GridGeometry, INSCRIBED, LETHAL, MAX_NON_OBSTACLE, UNKNOWN,
};

/// Largest difference between two costs of one path the properties allow: the planner
/// adds its step costs in single precision, the properties in double. A defect these
/// properties look for misses by a step's cost, 1 or more.
pub const COST_TOLERANCE: f64 = 1e-4;

/// The cell sides a drawn grid takes, in meters: powers of two, so a cell's corner, an
/// origin a whole number of cells from 0, lies on the bits single precision holds.
const RESOLUTIONS: [f64; 3] = [0.25, 0.5, 1.0];

/// A cost of any kind: free, one the inflation of an obstacle gives, the inscribed cost,
/// lethal or unknown.
#[hegel::composite]
pub fn costs(tc: &TestCase) -> u8 {
    match tc.draw(gs::integers::<u8>().min_value(0).max_value(4)) {
        0 => FREE,
        1 => tc.draw(
            gs::integers::<u8>()
                .min_value(1)
                .max_value(MAX_NON_OBSTACLE),
        ),
        2 => INSCRIBED,
        3 => LETHAL,
        _ => UNKNOWN,
    }
}

/// A grid of 1 to `max_side` cells on each axis, of a resolution of [`RESOLUTIONS`] and an
/// origin a whole number of cells from 0, each cell's cost drawn by [`costs`].
#[hegel::composite]
pub fn grids(tc: &TestCase, max_side: u32) -> CostGrid {
    let size_x = tc.draw(gs::integers::<u32>().min_value(1).max_value(max_side));
    let size_y = tc.draw(gs::integers::<u32>().min_value(1).max_value(max_side));
    let resolution = tc.draw(gs::sampled_from(&RESOLUTIONS));
    let origin: [i32; 2] = tc.draw(gs::arrays(gs::integers::<i32>().min_value(-8).max_value(8)));
    let geometry = GridGeometry::new(
        size_x,
        size_y,
        PositiveFinite::parse(resolution).expect("a positive resolution"),
        origin.map(|cells| f64::from(cells) * resolution),
    )
    .expect("a grid of 1 to 8 cells a side");
    let cells = (size_x * size_y) as usize;
    let costs: Vec<u8> = tc.draw(gs::vecs(costs()).min_size(cells).max_size(cells));
    CostGrid::from_costs(geometry, costs).expect("a cost a cell")
}

/// Whether a base's center may stand on a cell of `cost`: below the inscribed cost, or
/// unknown when unknown cells are passable.
pub fn passable(cost: u8, allow_unknown: bool) -> bool {
    cost < INSCRIBED || (cost == UNKNOWN && allow_unknown)
}

/// The cells of `grid` a base's center may stand on.
pub fn passable_cells(grid: &CostGrid, allow_unknown: bool) -> Vec<Cell> {
    grid.geometry()
        .cells()
        .filter(|&cell| passable(grid.cost(cell), allow_unknown))
        .collect()
}

/// A cost a base's center may not stand on: the inscribed cost, lethal, or unknown when
/// unknown cells are not passable.
#[hegel::composite]
pub fn obstacle_costs(tc: &TestCase, allow_unknown: bool) -> u8 {
    if allow_unknown {
        tc.draw(gs::sampled_from(&[INSCRIBED, LETHAL]))
    } else {
        tc.draw(gs::sampled_from(&[INSCRIBED, LETHAL, UNKNOWN]))
    }
}

/// `grid` with `cell` at `cost`.
pub fn with_cost(grid: &CostGrid, cell: Cell, cost: u8) -> CostGrid {
    let geometry = grid.geometry();
    let mut costs = grid.costs().to_vec();
    costs[cell.y() as usize * geometry.size_x() as usize + cell.x() as usize] = cost;
    CostGrid::from_costs(geometry, costs).expect("the same grid")
}

/// The corner of `cell` with the least coordinates, in the grid's frame.
pub fn corner(grid: &CostGrid, cell: Cell) -> [f64; 2] {
    let geometry = grid.geometry();
    let resolution = geometry.resolution_m();
    let [x, y] = geometry.origin();
    [
        x + f64::from(cell.x()) * resolution,
        y + f64::from(cell.y()) * resolution,
    ]
}
