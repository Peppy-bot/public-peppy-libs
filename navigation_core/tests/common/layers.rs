//! The obstacle layer the suites read of the live obstacles, and the lethal cells of a
//! layer or a grid.

use std::collections::BTreeSet;

use control_core::planar_pose::PlanarPose;
use navigation_core::grid::{CostGrid, LETHAL};
use navigation_core::obstacles::{LiveObstacles, ObstacleLayer};

/// The obstacle layer of `live` over the map's columns.
pub fn layer(live: &LiveObstacles) -> ObstacleLayer {
    live.project(PlanarPose::IDENTITY, live.columns())
        .expect("a finite placement")
}

/// The lethal cells of `layer`, each its column and row.
pub fn lethal_cells(layer: &ObstacleLayer) -> BTreeSet<(u32, u32)> {
    lethal_cells_of(layer.grid())
}

/// The lethal cells of `grid`, each its column and row.
pub fn lethal_cells_of(grid: &CostGrid) -> BTreeSet<(u32, u32)> {
    grid.geometry()
        .cells()
        .filter(|&cell| grid.cost(cell) == LETHAL)
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}
