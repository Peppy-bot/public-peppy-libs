//! Properties of a saved grid placed on a grid in another frame, on drawn grids, cells and
//! placements: each cell the placed grid marks lethal lies on the square of an occupied cell
//! of the saved grid, each cell whose center lies inside such a square, a sliver in from
//! its sides, is lethal, and every other cell is unknown.

mod common;

use std::f64::consts::PI;

use common::finite::positive;
use common::layers::lethal_cells_of;
use common::placements::assert_marks_the_cells_the_occupied_squares_overlap;
use common::poses::planar;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::grid::{GridGeometry, LETHAL, UNKNOWN};
use navigation_core::saved_grid::SavedGrid;

/// The values of a trinary occupancy grid's cells: free, occupied, and unknown read
/// unsigned.
const VALUES: [u8; 3] = [0, 100, 255];

/// The half side, in meters, of the target grid about the origin, which holds most of each
/// drawn saved grid wherever a drawn placement puts it.
const TARGET_HALF_SIDE_M: f64 = 1.5;

// A saved grid of 1 to 6 cells a side, of a drawn side, origin and cells, placed at a drawn
// offset and turn on a grid of a drawn side about the origin, coarser or finer than its
// own: each lethal cell lies on an occupied square, each cell whose center lies inside one,
// a sliver in from its sides, is lethal, and every other cell is unknown.
#[hegel::test(test_cases = 100)]
fn a_placed_grid_marks_the_cells_its_occupied_squares_overlap(tc: TestCase) {
    let columns = tc.draw(gs::integers::<u32>().min_value(1).max_value(6));
    let rows = tc.draw(gs::integers::<u32>().min_value(1).max_value(6));
    let cell_m = tc.draw(gs::floats().min_value(0.05).max_value(0.25));
    let corner = || gs::floats().min_value(-1.0).max_value(0.0);
    let origin = [tc.draw(corner()), tc.draw(corner())];
    let count = (columns * rows) as usize;
    let cells: Vec<u8> = tc.draw(
        gs::vecs(gs::sampled_from(&VALUES))
            .min_size(count)
            .max_size(count),
    );
    let saved = SavedGrid::from_cells(columns, rows, positive(cell_m), origin, &cells)
        .expect("a trinary grid, a value a cell");
    let offset = || gs::floats().min_value(-0.5).max_value(0.5);
    let grid_from_saved = planar(
        tc.draw(offset()),
        tc.draw(offset()),
        tc.draw(gs::floats().min_value(-PI).max_value(PI)),
    );
    let target_m = tc.draw(gs::floats().min_value(0.03).max_value(0.3));
    let target_cells = (2.0 * TARGET_HALF_SIDE_M / target_m).ceil() as u32;
    let target = GridGeometry::new(
        target_cells,
        target_cells,
        positive(target_m),
        [-TARGET_HALF_SIDE_M; 2],
    )
    .expect("a grid about the origin");
    let placed = saved
        .placed_on(grid_from_saved, target)
        .expect("a finite placement");
    assert_eq!(placed.grid().geometry(), target);
    assert!(
        placed
            .grid()
            .costs()
            .iter()
            .all(|&cost| cost == LETHAL || cost == UNKNOWN),
        "a placed grid holds lethal and unknown cells alone"
    );
    assert_marks_the_cells_the_occupied_squares_overlap(
        &saved.grid().geometry(),
        &lethal_cells_of(saved.grid()),
        grid_from_saved,
        placed.grid(),
    );
}
