//! The obstacle layer the suites read of the live obstacles, and its lethal cells.

use std::collections::BTreeSet;

use camera_core::pose::PlanarPose;
use navigation_core::grid::LETHAL;
use navigation_core::obstacles::{LiveObstacles, ObstacleLayer};

/// The obstacle layer of `live` over the map's columns.
pub fn layer(live: &LiveObstacles) -> ObstacleLayer {
    live.project(PlanarPose::IDENTITY, live.columns())
        .expect("a finite placement")
}

/// The lethal cells of `layer`, each its column and row.
pub fn lethal_cells(layer: &ObstacleLayer) -> BTreeSet<(u32, u32)> {
    let grid = layer.grid();
    grid.geometry()
        .cells()
        .filter(|&cell| grid.cost(cell) == LETHAL)
        .map(|cell| (cell.x(), cell.y()))
        .collect()
}
