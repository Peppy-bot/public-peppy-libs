//! Smac 2D's planner on a cost grid, ported from nav2_smac_planner 1.5.2
//! `SmacPlanner2DT<Node2D>::createPlan` (include/nav2_smac_planner/
//! smac_planner_2d_impl.hpp): the start and the goal located in the grid it
//! plans on, in single precision, Smac's A* between them ([`search`]), the cells' corners
//! as poses in the grid's frame (`getWorldCoords`), the path smoothed by
//! Smac's smoother, the last pose moved onto the goal when the search
//! reached the goal's cell, and the last pose's heading.
//!
//! A start or a goal off the grid fails with [`PlanError::StartOffGrid`] or
//! [`PlanError::GoalOffGrid`], and a heading that is not finite with
//! [`PlanError::HeadingNotFinite`]. A cell at or above
//! [`INSCRIBED`](crate::grid::INSCRIBED) is an obstacle for the base's
//! center, and an unknown cell is passable only when
//! [`SearchParams::allow_unknown`] is true. Before the search, the planner
//! tests the start, then the goal, by the cost of its cell, as Smac 2D's
//! collision checker tests a cell for a base taken as its inscribed circle,
//! and fails with [`PlanError::StartOccupied`] or [`PlanError::GoalOccupied`]
//! for a cell in collision. The goal tolerance is 0, Smac 2D's `tolerance`
//! at 0, so the search plans to the goal's cell itself, and one that ends
//! without reaching it, its iteration bound included, fails with
//! [`PlanError::NoPath`]. The planner reads no clock: it runs as Smac 2D runs
//! with a `max_planning_time` that no search reaches.
//!
//! The failures follow base_navigation:v1's rule, which departs from Smac 2D
//! three ways:
//!
//! - Smac 2D never tests the start, which it takes as cleared
//!   (a_star_impl.hpp:318); here a start in collision fails
//!   [`PlanError::StartOccupied`], where Smac 2D plans from it.
//! - Smac 2D returns the goal as a one-pose plan for a start and a goal in one
//!   cell before any test (smac_planner_2d_impl.hpp:272-290); here both tests
//!   run first, so a start and a goal in one occupied cell fail
//!   [`PlanError::StartOccupied`].
//! - Smac 2D throws `StartOccupied` for any search that ends after one
//!   iteration (smac_planner_2d_impl.hpp:300-303), a free start walled in
//!   among them; here such a start fails [`PlanError::NoPath`], and only a
//!   start in collision fails [`PlanError::StartOccupied`].

mod search;
mod smoother;

use std::num::NonZeroU32;

use camera_core::pose::PlanarPose;

use crate::footprint::PaddedFootprint;
use crate::grid::{Cell, CostGrid, GridPoint};
use crate::planner::search::{SearchLimits, a_star, in_collision};
use crate::pose::heading;

pub use smoother::SmootherParams;

/// How much a cell's cost grows a step onto it (`cost_travel_multiplier`),
/// in single precision, as Smac 2D holds it (`SearchInfo::cost_penalty`,
/// set at smac_planner_2d_impl.hpp:76-77): a number from 0 to the largest
/// single precision number, `f32::MAX`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CostTravelMultiplier(f32);

/// A number that is no `cost_travel_multiplier`.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "cost_travel_multiplier {0} is not a number from 0 to {max:e}, the largest single \
     precision number, which Smac 2D holds it in: give a multiplier from 0 to {max:e}",
    max = f32::MAX
)]
pub struct CostTravelMultiplierError(pub f64);

impl CostTravelMultiplier {
    /// The multiplier `value`, in single precision; negative zero reads as
    /// zero.
    pub fn parse(value: f64) -> Result<Self, CostTravelMultiplierError> {
        (value >= 0.0 && value <= f64::from(f32::MAX))
            .then_some(Self(value as f32 + 0.0))
            .ok_or(CostTravelMultiplierError(value))
    }

    pub fn get(self) -> f32 {
        self.0
    }
}

/// The search's settings, Smac 2D's parameters of the same names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SearchParams {
    /// How much a cell's cost grows a step onto it: a step costs its length
    /// in cells times one plus this times the cell's cost over 252
    /// (`cost_travel_multiplier`).
    pub cost_travel_multiplier: CostTravelMultiplier,
    /// Whether an unknown cell is passable (`allow_unknown`).
    pub allow_unknown: bool,
    /// The most cells the search expands (`max_iterations`).
    pub max_iterations: NonZeroU32,
}

/// The planner's settings, Smac 2D's parameters of the same names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannerParams {
    pub search: SearchParams,
    /// Whether the last pose heads along the path's last step, in place of
    /// the goal's heading (`use_final_approach_orientation`).
    pub use_final_approach_orientation: bool,
    /// The smoother's settings, none to leave the path as the search gives
    /// it (`smoother.max_iterations` 0).
    pub smoother: Option<SmootherParams>,
}

/// Why the planner gives no path.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum PlanError {
    #[error(
        "the start, x {} m, y {} m, lies off the grid or is not a number: start the base on \
         the grid",
        .0[0],
        .0[1]
    )]
    StartOffGrid([f64; 2]),
    #[error(
        "the goal, x {} m, y {} m, lies off the grid or is not a number: give a goal on the \
         grid",
        .0[0],
        .0[1]
    )]
    GoalOffGrid([f64; 2]),
    #[error(
        "the start's heading {start_rad} rad or the goal's heading {goal_rad} rad is not \
         finite: give finite headings"
    )]
    HeadingNotFinite { start_rad: f64, goal_rad: f64 },
    #[error(
        "the start, cell ({}, {}), is an obstacle for the base's center: start the base clear \
         of every obstacle",
        .0.x(),
        .0.y()
    )]
    StartOccupied(Cell),
    #[error(
        "the goal, cell ({}, {}), is an obstacle for the base's center: give a goal clear of \
         every obstacle",
        .0.x(),
        .0.y()
    )]
    GoalOccupied(Cell),
    #[error(
        "no path reaches the goal's cell within the search's iteration bound: give a goal the \
         base can reach"
    )]
    NoPath,
}

/// The cells of an A* path, from the start's cell to the goal's cell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GridPath {
    cells: Vec<Cell>,
}

impl GridPath {
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }
}

/// Smac 2D's A* path on `grid` from `start` towards `goal`, points in meters
/// in the grid's frame: one cell when both lie in one cell.
pub fn search(
    grid: &CostGrid,
    start: [f64; 2],
    goal: [f64; 2],
    params: &SearchParams,
) -> Result<GridPath, PlanError> {
    let start_point = grid
        .geometry()
        .locate(start)
        .ok_or(PlanError::StartOffGrid(start))?;
    let goal_point = grid
        .geometry()
        .locate(goal)
        .ok_or(PlanError::GoalOffGrid(goal))?;
    search_located(grid, start_point, goal_point, params)
}

/// Smac 2D's A* path on `grid` between two points located in it.
fn search_located(
    grid: &CostGrid,
    start: GridPoint,
    goal: GridPoint,
    params: &SearchParams,
) -> Result<GridPath, PlanError> {
    let passable = |cell: Cell| !in_collision(grid.cost(cell), params.allow_unknown);
    if !passable(start.cell()) {
        return Err(PlanError::StartOccupied(start.cell()));
    }
    if !passable(goal.cell()) {
        return Err(PlanError::GoalOccupied(goal.cell()));
    }
    if start.cell() == goal.cell() {
        return Ok(GridPath {
            cells: vec![start.cell()],
        });
    }
    let limits = SearchLimits {
        cost_travel_multiplier: params.cost_travel_multiplier.get(),
        allow_unknown: params.allow_unknown,
        max_iterations: params.max_iterations.get(),
    };
    a_star(grid, start.cell(), goal, limits)
        .map(|cells| GridPath { cells })
        .ok_or(PlanError::NoPath)
}

/// Smac 2D's path on `grid` from `start` to `goal`, poses in the grid's
/// frame, for a base of `footprint`: its poses in the grid's frame, from the
/// start's cell to the goal.
pub fn plan(
    grid: &CostGrid,
    footprint: &PaddedFootprint,
    start: PlanarPose,
    goal: PlanarPose,
    params: &PlannerParams,
) -> Result<Vec<PlanarPose>, PlanError> {
    let start_xy = [start.x, start.y];
    let goal_xy = [goal.x, goal.y];
    let start_point = grid
        .geometry()
        .locate(start_xy)
        .ok_or(PlanError::StartOffGrid(start_xy))?;
    let goal_point = grid
        .geometry()
        .locate(goal_xy)
        .ok_or(PlanError::GoalOffGrid(goal_xy))?;
    if !(start.yaw.is_finite() && goal.yaw.is_finite()) {
        return Err(PlanError::HeadingNotFinite {
            start_rad: start.yaw,
            goal_rad: goal.yaw,
        });
    }
    let path = search_located(grid, start_point, goal_point, &params.search)?;
    let [goal_x, goal_y] = goal_point.world();
    if start_point.cell() == goal_point.cell() {
        let yaw = if start.yaw != goal.yaw && params.use_final_approach_orientation {
            start.yaw
        } else {
            goal.yaw
        };
        return Ok(vec![PlanarPose {
            x: goal_x,
            y: goal_y,
            yaw,
        }]);
    }
    let geometry = grid.geometry();
    let [origin_x, origin_y] = geometry.origin();
    let resolution = geometry.resolution_m();
    let mut poses: Vec<PlanarPose> = path
        .cells
        .iter()
        .map(|cell| PlanarPose {
            x: f64::from(origin_x as f32) + f64::from(cell.x() as f32) * resolution,
            y: f64::from(origin_y as f32) + f64::from(cell.y() as f32) * resolution,
            yaw: 0.0,
        })
        .collect();
    if let Some(smoother) = &params.smoother {
        smoother::smooth(&mut poses, grid, footprint, smoother);
    }
    let last = poses.len() - 1;
    poses[last].x = goal_x;
    poses[last].y = goal_y;
    poses[last].yaw = if params.use_final_approach_orientation {
        let (end, approach) = (poses[last], poses[last - 1]);
        heading((end.y - approach.y).atan2(end.x - approach.x))
    } else {
        goal.yaw
    };
    Ok(poses)
}
