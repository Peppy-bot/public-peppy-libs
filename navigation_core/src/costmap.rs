//! A cost map's layers composed as Nav2 1.5.2's layered cost map composes
//! them over the whole grid (nav2_costmap_2d src/layered_costmap.cpp
//! `LayeredCostmap::updateMap`, with `CostmapLayer::updateWithTrueOverwrite`
//! and `CostmapLayer::updateWithMax` of src/costmap_layer.cpp), with
//! `track_unknown_space` true: the master grid starts unknown, or as the saved
//! grid's costs, as the static layer writes them; then the obstacle layer's
//! costs, where a layer's unknown cell changes nothing, any other cost replaces
//! an unknown cell, and else the higher cost wins; then the inflation, when
//! the grid takes one.

use crate::grid::{CostGrid, GridGeometry, UNKNOWN};
use crate::inflation::{Inflation, InflationError};
use crate::obstacles::ObstacleLayer;
use crate::saved_grid::SavedGrid;

/// Why layers compose into no grid.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ComposeError {
    #[error(
        "the obstacle layer lies over {obstacles:?} and the saved grid over {saved:?}: \
         project the obstacles onto the saved grid's geometry"
    )]
    Geometry {
        saved: GridGeometry,
        obstacles: GridGeometry,
    },
    #[error(transparent)]
    Inflation(#[from] InflationError),
}

/// The cost grid of `obstacles`, over `saved` when a saved grid is loaded,
/// inflated by `inflation` when it takes one.
pub fn compose(
    saved: Option<&SavedGrid>,
    obstacles: &ObstacleLayer,
    inflation: Option<&Inflation>,
) -> Result<CostGrid, ComposeError> {
    let layer = obstacles.grid();
    let mut grid = match saved {
        Some(saved) if saved.grid().geometry() != layer.geometry() => {
            return Err(ComposeError::Geometry {
                saved: saved.grid().geometry(),
                obstacles: layer.geometry(),
            });
        }
        Some(saved) => saved.grid().clone(),
        None => CostGrid::filled(layer.geometry(), UNKNOWN),
    };
    for (master, &cost) in grid.costs_mut().iter_mut().zip(layer.costs()) {
        if cost == UNKNOWN {
            continue;
        }
        if *master == UNKNOWN || *master < cost {
            *master = cost;
        }
    }
    if let Some(inflation) = inflation {
        inflation.inflate(&mut grid)?;
    }
    Ok(grid)
}

#[cfg(test)]
mod tests {

    use camera_core::pose::{PlanarPose, Pose};
    use control_core::non_negative_finite::NonNegativeFinite;
    use control_core::positive_finite::PositiveFinite;
    use occupancy_map::{Bounds, Frame};

    use super::*;
    use crate::footprint::PaddedFootprint;
    use crate::grid::{FREE, LETHAL};
    use crate::obstacles::{LiveObstacles, ObstacleHeights, ObstacleHeightsParams};
    use crate::saved_grid::MapThresholds;
    use control_core::footprint::Footprint;

    fn tenth() -> PositiveFinite {
        PositiveFinite::parse(0.1).unwrap()
    }

    fn thresholds() -> MapThresholds {
        MapThresholds::parse(0.65, 0.25).unwrap()
    }

    // Over a row of four cells, the saved grid unknown, lethal, free and unknown, and a
    // cloud whose one ray clears the first two columns and hits the third in the band:
    // the cleared cell replaces the unknown one, the lethal saved cell stays, the hit
    // marks the free cell, and the cell no voxel reaches keeps its unknown cost. With no
    // saved grid, the obstacle layer's costs stand.
    #[test]
    fn a_known_cost_replaces_an_unknown_cell_and_else_the_higher_cost_wins() {
        let saved =
            SavedGrid::from_image(4, 1, &[128, 0, 254, 128], tenth(), [0.0; 2], thresholds())
                .unwrap();
        let bounds = Bounds {
            min: [0.0, 0.0, -0.25],
            max: [0.39, 0.09, 1.2],
        };
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.04, -0.04], [0.04, -0.04], [0.04, 0.04], [-0.04, 0.04]])
                .unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let heights = ObstacleHeights::parse(ObstacleHeightsParams {
            min_obstacle_height_m: 0.2,
            max_obstacle_height_m: 0.5,
            max_step_down_m: 0.1,
            voxel_m: tenth(),
        })
        .unwrap();
        let mut live =
            LiveObstacles::new(Frame::parse("map").unwrap(), bounds, heights, footprint).unwrap();
        let away = PlanarPose::new(-5.0, 0.0, 0.0).unwrap();
        let sensor_on_base = Pose::new([5.05, 0.05, 1.05], [0.0, 0.0, 0.0, 1.0]).unwrap();
        live.insert_depth_frame(
            &[[0.2, 0.0, -0.7]],
            away,
            sensor_on_base,
            PositiveFinite::parse(5.0).unwrap(),
        );
        let geometry = saved.grid().geometry();
        let obstacles = live
            .project(PlanarPose::IDENTITY, geometry)
            .expect("a finite placement");
        assert_eq!(obstacles.grid().costs(), [FREE, FREE, LETHAL, UNKNOWN]);
        let grid = compose(Some(&saved), &obstacles, None).unwrap();
        assert_eq!(grid.costs(), [FREE, LETHAL, LETHAL, UNKNOWN]);
        let alone = compose(None, &obstacles, None).unwrap();
        assert_eq!(alone.costs(), [FREE, FREE, LETHAL, UNKNOWN]);

        let other = SavedGrid::from_image(2, 2, &[0; 4], tenth(), [0.0; 2], thresholds()).unwrap();
        assert!(compose(Some(&other), &obstacles, None).is_err());
    }
}
