//! The footprint checks of Nav2 1.5.2's regulated pure pursuit
//! (nav2_regulated_pure_pursuit_controller src/collision_checker.cpp:
//! `CollisionChecker::inCollision` and `costAtPose`), on a cost grid in the
//! frame of the base's pose. The cost under the footprint reads lethal at the
//! first edge that holds a lethal cell, whatever the edges before it hold
//! (`PaddedFootprint`'s departure from Nav2's `footprintCost`), so a lethal
//! cell under the outline after an unknown edge is a collision here, where
//! Nav2's check reads the footprint's cost as unknown and passes it.

use camera_core::pose::PlanarPose;

use crate::footprint::PaddedFootprint;
use crate::grid::{CostGrid, LETHAL, UNKNOWN};

/// The grid the checks read and the base's footprint on it. The grid tracks
/// unknown cells, as every grid `compose` makes does (`track_unknown_space`
/// true), so a footprint whose cost is unknown, an unknown cell under its
/// outline and no lethal one, is no collision (`isTrackingUnknown`).
pub(super) struct FootprintCheck<'a> {
    pub grid: &'a CostGrid,
    pub footprint: &'a PaddedFootprint,
}

impl FootprintCheck<'_> {
    /// `inCollision`: whether the footprint at `x`, `y` and heading `theta`
    /// touches a lethal cell or has a vertex off the grid, which the footprint
    /// cost reads as lethal, as Nav2's `footprintCost` does (nav2_costmap_2d
    /// src/footprint_collision_checker.cpp:55-56, 66-67). A center off the
    /// grid is no collision; on the grid, a heading that is not finite places
    /// the footprint's vertices on no cell and collides.
    pub(super) fn in_collision(&self, x: f64, y: f64, theta: f64) -> bool {
        if self.grid.geometry().cell_of([x, y]).is_none() {
            return false;
        }
        let Ok(pose) = PlanarPose::new(x, y, theta) else {
            return true;
        };
        let footprint_cost = self.footprint.cost_at(self.grid, pose);
        if footprint_cost == f64::from(UNKNOWN) {
            return false;
        }
        footprint_cost >= f64::from(LETHAL)
    }

    /// `costAtPose`: the cost of the cell under `x`, `y`, none off the grid.
    pub(super) fn cost_at_pose(&self, x: f64, y: f64) -> Option<f64> {
        let cell = self.grid.geometry().cell_of([x, y])?;
        Some(f64::from(self.grid.cost(cell)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{FREE, GridGeometry};
    use control_core::footprint::Footprint;
    use control_core::non_negative_finite::NonNegativeFinite;
    use control_core::positive_finite::PositiveFinite;

    // On a free grid, a footprint at a finite heading is clear, and at a heading that is not
    // finite its vertices lie on no cell and it collides; a center off the grid is clear.
    #[test]
    fn a_heading_that_is_not_finite_collides() {
        let geometry =
            GridGeometry::new(20, 20, PositiveFinite::parse(0.1).unwrap(), [0.0; 2]).unwrap();
        let grid = CostGrid::filled(geometry, FREE);
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let check = FootprintCheck {
            grid: &grid,
            footprint: &footprint,
        };
        assert!(!check.in_collision(1.0, 1.0, 0.3));
        assert!(check.in_collision(1.0, 1.0, f64::INFINITY));
        assert!(!check.in_collision(-1.0, 1.0, f64::NAN));
    }
}
