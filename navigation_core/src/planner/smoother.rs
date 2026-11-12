//! Smac 2D's path smoother, ported from nav2_smac_planner 1.5.2
//! `Smoother::smooth` and `Smoother::smoothImpl` (src/smoother.cpp) as
//! SmacPlanner2D runs them, on a path of a holonomic search: one segment, the
//! whole path, smoothed when it holds more than 11 poses; each pass moves
//! every pose but the ends towards its own unsmoothed place and towards its
//! neighbors, Gauss-Seidel, and ends the smoothing once the poses move less
//! than the tolerance in a pass, or reverts the pass and ends it when a pose
//! lands where the cost under the footprint's outline is above
//! [`MAX_NON_OBSTACLE`] and known (src/smoother.cpp:182-207): a lethal cell
//! under the outline always among them, the footprint's cost reading lethal
//! first (`PaddedFootprint`'s departure from Nav2's `footprintCost`); the
//! iteration bound ends it with the last admissible pass. Headings follow
//! nav2_util's `updateApproximatePathOrientations`
//! (include/nav2_util/smoother_utils.hpp).

use std::num::NonZeroU32;

use control_core::non_negative_finite::NonNegativeFinite;

use crate::footprint::PaddedFootprint;
use crate::grid::{CostGrid, MAX_NON_OBSTACLE, UNKNOWN};
use crate::planner::PathPose;
use crate::pose::heading;

/// The smoother's settings, Smac's `smoother` parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SmootherParams {
    /// The total movement of a pass, in meters, below which the smoothing
    /// ends (`tolerance`): the sum over the poses of each one's move along x
    /// and along y (src/smoother.cpp:168).
    pub tolerance_m: NonNegativeFinite,
    /// The iteration bound (`max_iterations`): the passes that may run are
    /// one fewer.
    pub max_iterations: NonZeroU32,
    /// How strongly a pose keeps to its unsmoothed place (`w_data`).
    pub w_data: NonNegativeFinite,
    /// How strongly a pose moves towards its neighbors (`w_smooth`).
    pub w_smooth: NonNegativeFinite,
    /// How many times a converged smoothing runs again from its result
    /// (`refinement_num` when `do_refinement` is true, else 0).
    pub refinements: u32,
}

/// The most poses past the first that a segment holds and is left as it is:
/// the smoother smooths a segment of more, `end - start > 10`
/// (src/smoother.cpp:73).
const LONGEST_UNSMOOTHED: usize = 10;

/// Below this, on both axes, two poses lie on each other and the first keeps
/// its heading.
const OVERLAP_M: f64 = 1e-4;

/// `Smoother::smooth` on `path`, a holonomic path in `grid`'s frame.
pub(crate) fn smooth(
    path: &mut [PathPose],
    grid: &CostGrid,
    footprint: &PaddedFootprint,
    params: &SmootherParams,
) {
    if path.len() - 1 <= LONGEST_UNSMOOTHED {
        return;
    }
    let mut refinements_run = 0;
    let mut segment = path.to_vec();
    smooth_impl(&mut segment, grid, footprint, params, &mut refinements_run);
    path.copy_from_slice(&segment);
}

/// `Smoother::smoothImpl`.
fn smooth_impl(
    path: &mut Vec<PathPose>,
    grid: &CostGrid,
    footprint: &PaddedFootprint,
    params: &SmootherParams,
    refinements_run: &mut u32,
) {
    let tolerance = params.tolerance_m.get();
    let max_iterations = params.max_iterations.get();
    let data_w = params.w_data.get();
    let smooth_w = params.w_smooth.get();
    let path_size = path.len();
    let mut its = 0;
    let mut change = tolerance;
    let mut new_path = path.clone();
    let mut last_path = path.clone();
    while change >= tolerance {
        its += 1;
        change = 0.0;
        if its >= max_iterations {
            update_orientations(&mut last_path);
            *path = last_path;
            return;
        }
        for i in 1..path_size - 1 {
            for j in 0..2 {
                let x_i = path[i].xy[j];
                let mut y_i = new_path[i].xy[j];
                let y_m1 = new_path[i - 1].xy[j];
                let y_ip1 = new_path[i + 1].xy[j];
                let y_i_org = y_i;
                y_i += data_w * (x_i - y_i) + smooth_w * (y_ip1 + y_m1 - (2.0 * y_i));
                new_path[i].xy[j] = y_i;
                change += (y_i - y_i_org).abs();
            }
        }
        update_orientations(&mut new_path);
        if new_path[1..path_size - 1]
            .iter()
            .any(|pose| !admissible(grid, footprint, pose))
        {
            *path = last_path;
            return;
        }
        last_path = new_path.clone();
    }
    if *refinements_run < params.refinements {
        *refinements_run += 1;
        smooth_impl(&mut new_path, grid, footprint, params, refinements_run);
    }
    *path = new_path;
}

/// Whether the footprint at `pose` lies on no obstacle of `grid`: the cost
/// under its outline at most [`MAX_NON_OBSTACLE`], or unknown. A pose that is
/// not finite lies on no cell of the grid and is not admissible, as a vertex
/// off the grid reads lethal.
fn admissible(grid: &CostGrid, footprint: &PaddedFootprint, pose: &PathPose) -> bool {
    pose.placed().is_ok_and(|placed| {
        let cost = footprint.cost_at(grid, placed) as f32;
        !(cost > f32::from(MAX_NON_OBSTACLE) && cost != f32::from(UNKNOWN))
    })
}

/// `updateApproximatePathOrientations` for a holonomic path: each pose but
/// the last heads towards the next, unless the two lie on each other.
fn update_orientations(path: &mut [PathPose]) {
    for i in 0..path.len() - 1 {
        let dx = path[i + 1].xy[0] - path[i].xy[0];
        let dy = path[i + 1].xy[1] - path[i].xy[1];
        if dx.abs() < OVERLAP_M && dy.abs() < OVERLAP_M {
            continue;
        }
        path[i].yaw = heading(dy.atan2(dx));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{FREE, GridGeometry};
    use control_core::footprint::Footprint;
    use control_core::positive_finite::PositiveFinite;

    // A pose of a free grid is admissible, and one whose position or heading is not finite
    // lies on no cell and is not.
    #[test]
    fn a_pose_that_is_not_finite_is_not_admissible() {
        let geometry =
            GridGeometry::new(20, 20, PositiveFinite::parse(0.1).unwrap(), [0.0; 2]).unwrap();
        let grid = CostGrid::filled(geometry, FREE);
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let at = |xy: [f64; 2], yaw: f64| PathPose { xy, yaw };
        assert!(admissible(&grid, &footprint, &at([1.0, 1.0], 0.3)));
        assert!(!admissible(&grid, &footprint, &at([f64::NAN, 1.0], 0.3)));
        assert!(!admissible(
            &grid,
            &footprint,
            &at([1.0, 1.0], f64::INFINITY)
        ));
    }
}
