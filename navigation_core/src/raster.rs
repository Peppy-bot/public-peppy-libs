//! The cells under a polygon, as Nav2 1.5.2's cost map finds the cells it
//! sets under a footprint (nav2_costmap_2d src/costmap_2d.cpp
//! `getMapRegionOccupiedByPolygon`, `polygonOutlineCells` and
//! `convexFillCells`, with nav2_util's `raytraceLine` and `bresenham2D`,
//! include/nav2_util/raytrace_line_2d.hpp), step for step: each vertex's cell,
//! the outline's cells from each vertex's cell to the next one's, the last
//! back to the first, sorted by column, and each column filled between its
//! lowest and highest outline cells. A concave polygon's column takes the whole
//! span between them, as Nav2 fills it.
//!
//! [`polygon_cells`] sets no cell for a polygon with a vertex off the grid, as
//! Nav2 does.

use crate::grid::{Cell, GridGeometry};

/// The cells of `geometry` under the polygon `polygon`, in meters in the
/// grid's frame, each outline cell as often as the outline crosses it; none
/// when a vertex lies off the grid, as Nav2 then sets no cell.
pub(crate) fn polygon_cells(geometry: &GridGeometry, polygon: &[[f64; 2]]) -> Option<Vec<Cell>> {
    let vertices: Vec<(u32, u32)> = polygon
        .iter()
        .map(|&point| geometry.cell_of(point).map(|cell| (cell.x(), cell.y())))
        .collect::<Option<_>>()?;
    Some(
        convex_fill(&vertices)
            .into_iter()
            .map(|(x, y)| {
                geometry
                    .cell(x, y)
                    .expect("a polygon's cells lie between its vertices' cells")
            })
            .collect(),
    )
}

/// `raytraceLine` with no length bounds: the cells from `(x0, y0)` to
/// `(x1, y1)`, both included, along the axis of the longer step, the other
/// moved when its error term reaches the longer step.
pub(crate) fn raytrace_line(
    (x0, y0): (u32, u32),
    (x1, y1): (u32, u32),
    cells: &mut Vec<(u32, u32)>,
) {
    let dx = i64::from(x1) - i64::from(x0);
    let dy = i64::from(y1) - i64::from(y0);
    let (abs_dx, abs_dy) = (dx.unsigned_abs(), dy.unsigned_abs());
    let (step_x, step_y) = (dx.signum(), dy.signum());
    let (mut x, mut y) = (i64::from(x0), i64::from(y0));
    let mut push = |x: i64, y: i64| cells.push((x as u32, y as u32));
    if abs_dx >= abs_dy {
        let mut error = abs_dx / 2;
        for _ in 0..abs_dx {
            push(x, y);
            x += step_x;
            error += abs_dy;
            if error >= abs_dx {
                y += step_y;
                error -= abs_dx;
            }
        }
        push(x, y);
        return;
    }
    let mut error = abs_dy / 2;
    for _ in 0..abs_dy {
        push(x, y);
        y += step_y;
        error += abs_dx;
        if error >= abs_dy {
            x += step_x;
            error -= abs_dy;
        }
    }
    push(x, y);
}

/// `polygonOutlineCells`: the cells from each vertex's cell to the next one's,
/// the last back to the first.
fn outline_cells(vertices: &[(u32, u32)]) -> Vec<(u32, u32)> {
    let mut cells = Vec::new();
    for pair in vertices.windows(2) {
        raytrace_line(pair[0], pair[1], &mut cells);
    }
    if let (Some(&last), Some(&first)) = (vertices.last(), vertices.first()) {
        raytrace_line(last, first, &mut cells);
    }
    cells
}

/// `convexFillCells`: the outline's cells, sorted by column with the swaps of
/// Nav2's sort, and each column from its first to its last filled between the
/// lowest and the highest of the two cells the walk reaches first in it and
/// the column's other outline cells, the fill appended after the outline.
fn convex_fill(vertices: &[(u32, u32)]) -> Vec<(u32, u32)> {
    if vertices.len() < 3 {
        return Vec::new();
    }
    let mut cells = outline_cells(vertices);
    let mut i = 0;
    while i < cells.len() - 1 {
        if cells[i].0 > cells[i + 1].0 {
            cells.swap(i, i + 1);
            i = i.saturating_sub(1);
        } else {
            i += 1;
        }
    }
    let (min_x, max_x) = (cells[0].0, cells[cells.len() - 1].0);
    i = 0;
    for x in min_x..=max_x {
        if i >= cells.len() - 1 {
            break;
        }
        let (mut low, mut high) = if cells[i].1 < cells[i + 1].1 {
            (cells[i], cells[i + 1])
        } else {
            (cells[i + 1], cells[i])
        };
        i += 2;
        while i < cells.len() && cells[i].0 == x {
            if cells[i].1 < low.1 {
                low = cells[i];
            } else if cells[i].1 > high.1 {
                high = cells[i];
            }
            i += 1;
        }
        for y in low.1..=high.1 {
            cells.push((x, y));
        }
    }
    cells
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_core::positive_finite::PositiveFinite;
    use std::collections::BTreeSet;

    // Bresenham steps along the longer axis, the shorter one moving when its
    // error term, starting at half a step, reaches the longer step.
    #[test]
    fn a_line_runs_from_its_first_cell_to_its_last() {
        let mut cells = Vec::new();
        raytrace_line((0, 0), (4, 2), &mut cells);
        assert_eq!(cells, [(0, 0), (1, 1), (2, 1), (3, 2), (4, 2)]);
        cells.clear();
        raytrace_line((3, 5), (3, 2), &mut cells);
        assert_eq!(cells, [(3, 5), (3, 4), (3, 3), (3, 2)]);
        cells.clear();
        raytrace_line((1, 1), (1, 1), &mut cells);
        assert_eq!(cells, [(1, 1)]);
    }

    // A square's cells are every cell from its lowest corner's to its highest.
    #[test]
    fn the_cells_under_a_square_fill_it() {
        let filled: BTreeSet<(u32, u32)> = convex_fill(&[(1, 1), (4, 1), (4, 3), (1, 3)])
            .into_iter()
            .collect();
        let expected: BTreeSet<(u32, u32)> =
            (1..=4).flat_map(|x| (1..=3).map(move |y| (x, y))).collect();
        assert_eq!(filled, expected);
    }

    // A triangle fills each column between its outline's lowest and highest
    // cells; a polygon with a vertex off the grid sets no cell.
    #[test]
    fn the_cells_under_a_triangle_fill_each_column_between_its_edges() {
        let filled: BTreeSet<(u32, u32)> =
            convex_fill(&[(0, 0), (4, 0), (0, 4)]).into_iter().collect();
        let expected: BTreeSet<(u32, u32)> = (0..=4)
            .flat_map(|x| (0..=4 - x).map(move |y| (x, y)))
            .collect();
        assert_eq!(filled, expected);
        let geometry =
            GridGeometry::new(5, 5, PositiveFinite::parse(1.0).unwrap(), [0.0; 2]).unwrap();
        assert!(polygon_cells(&geometry, &[[0.5, 0.5], [5.5, 0.5], [0.5, 4.5]]).is_none());
        assert!(polygon_cells(&geometry, &[[0.5, 0.5], [4.5, 0.5], [0.5, 4.5]]).is_some());
    }
}
