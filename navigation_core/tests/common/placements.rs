//! A grid's cells placed on a grid in another frame: the squares that the properties of
//! `LiveObstacles::project` and `SavedGrid::placed_on` compare, and the two properties.

use std::collections::BTreeSet;
use std::f64::consts::FRAC_1_SQRT_2;

use control_core::planar_pose::PlanarPose;
use navigation_core::grid::{Cell, CostGrid, GridGeometry, LETHAL};

/// How far apart, in meters, a target cell's square and an occupied cell's square may lie
/// for the first to count as overlapping the second: past the rounding of a placement.
pub const TOUCH_M: f64 = 1e-9;

/// How far in from an occupied cell's sides, as a share of the cell, a target cell's center
/// lies for the target cell to count as under it: past the sliver by which a placed square
/// stands in from its cell's sides, single precision's epsilon times the most cells a
/// grid's axis holds, and past the rounding of a placement.
pub const UNDER_SHARE: f64 = 0.01;

/// The corners of `cell`'s square on `geometry`, in order, in the grid's frame.
pub fn square(geometry: &GridGeometry, cell: Cell) -> [[f64; 2]; 4] {
    let side = geometry.resolution_m();
    let [x, y] = geometry.origin();
    let low = [
        x + f64::from(cell.x()) * side,
        y + f64::from(cell.y()) * side,
    ];
    [
        low,
        [low[0] + side, low[1]],
        [low[0] + side, low[1] + side],
        [low[0], low[1] + side],
    ]
}

/// The least and the greatest of `corners` along `axis`.
fn extent_along(axis: [f64; 2], corners: &[[f64; 2]; 4]) -> [f64; 2] {
    corners
        .iter()
        .map(|corner| corner[0] * axis[0] + corner[1] * axis[1])
        .fold([f64::INFINITY, f64::NEG_INFINITY], |[low, high], along| {
            [low.min(along), high.max(along)]
        })
}

/// How far apart the squares `a` and `b`, each four corners in order, lie along the unit
/// normal of their sides that parts them most, in meters: below 0 where they overlap.
pub fn gap(a: &[[f64; 2]; 4], b: &[[f64; 2]; 4]) -> f64 {
    [a, b]
        .into_iter()
        .flat_map(|corners| {
            [0, 1].map(|i| {
                let [dx, dy] = [0, 1].map(|axis| corners[i + 1][axis] - corners[i][axis]);
                let length = dx.hypot(dy);
                [-dy / length, dx / length]
            })
        })
        .map(|normal| {
            let [a_low, a_high] = extent_along(normal, a);
            let [b_low, b_high] = extent_along(normal, b);
            (b_low - a_high).max(a_low - b_high)
        })
        .fold(f64::NEG_INFINITY, f64::max)
}

/// Whether `point`, in the frame of `source`, lies inside the square of an `occupied` cell
/// of it shrunk by [`UNDER_SHARE`] of a cell on each side.
fn under_an_occupied_cell(
    source: &GridGeometry,
    occupied: &BTreeSet<(u32, u32)>,
    point: [f64; 2],
) -> bool {
    let Some(cell) = source.cell_of(point) else {
        return false;
    };
    let origin = source.origin();
    let side = source.resolution_m();
    let within = [(0, cell.x()), (1, cell.y())].iter().all(|&(axis, index)| {
        let share = (point[axis] - origin[axis]) / side - f64::from(index);
        (UNDER_SHARE..=1.0 - UNDER_SHARE).contains(&share)
    });
    within && occupied.contains(&(cell.x(), cell.y()))
}

/// Asserts the two properties of `placed`, a grid on which the `occupied` cells of `source`
/// are placed at `target_from_source`: each lethal cell's square lies within [`TOUCH_M`] of
/// the square of an occupied cell, and each cell whose center lies inside the square of an
/// occupied cell shrunk by [`UNDER_SHARE`] of a cell on each side is lethal.
pub fn assert_marks_the_cells_the_occupied_squares_overlap(
    source: &GridGeometry,
    occupied: &BTreeSet<(u32, u32)>,
    target_from_source: PlanarPose,
    placed: &CostGrid,
) {
    let target = placed.geometry();
    let occupied_squares: Vec<([f64; 2], [[f64; 2]; 4])> = occupied
        .iter()
        .map(|&(x, y)| {
            let cell = source.cell(x, y).expect("a cell of the source");
            (source.center_of(cell), square(source, cell))
        })
        .collect();
    // Two squares overlap only where their centers lie within the sum of their half
    // diagonals.
    let reach = (target.resolution_m() + source.resolution_m()) * FRAC_1_SQRT_2 + TOUCH_M;
    for cell in target.cells() {
        let in_source = |point: [f64; 2]| target_from_source.apply_inverse(point);
        let center = in_source(target.center_of(cell));
        let lethal = placed.cost(cell) == LETHAL;
        if lethal {
            let placed_square = square(&target, cell).map(in_source);
            let nearest = occupied_squares
                .iter()
                .filter(|(occupied_center, _)| {
                    (center[0] - occupied_center[0]).hypot(center[1] - occupied_center[1]) <= reach
                })
                .map(|(_, occupied)| gap(&placed_square, occupied))
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest <= TOUCH_M,
                "cell ({}, {}) is lethal {nearest} m from the nearest occupied square",
                cell.x(),
                cell.y()
            );
        }
        assert!(
            lethal || !under_an_occupied_cell(source, occupied, center),
            "cell ({}, {}) is not lethal, its center inside an occupied square",
            cell.x(),
            cell.y()
        );
    }
}
