//! The planner against Nav2's Smac 2D (nav2_smac_planner 1.5.2): `planner::search` ports
//! Smac 2D's A*, with its costs and cell coordinates in single precision and its open set
//! in the order of libstdc++'s `std::priority_queue`, so it expands the cells Smac expands
//! in Smac's order, and
//! `planner::plan` ports `SmacPlanner2D::createPlan` and Smac's smoother. On the same cost
//! grid, padded footprint, start, goal and parameters, a goal tolerance of 0, a start
//! Smac's checker passes, the A* path is Smac's with its smoother
//! off, cell for cell, its poses Smac's bit for bit, and the smoothed path at the same
//! iteration bound lies within `SMOOTHED_M` of Smac's. The scenario's footprint is the one
//! Smac reads, the padded footprint, so ours pads it by 0.
//!
//! Smac reports `StartOccupied` for any search that ends after one iteration
//! (include/nav2_smac_planner/smac_planner_2d_impl.hpp:300-303), a free start whose
//! neighbors are all blocked among them, where ours fails with `NoPath`, as the ticket's
//! failure rule says: `StartOccupied` for a start in collision only. The scenarios' one
//! start that Smac reports occupied is in collision, so both report `StartOccupied`
//! there; their other failures are `GoalOccupied` on both sides, and Smac's
//! `NoValidPathCouldBeFound` and `PlannerTimedOut`, ours `NoPath`. The evidence: every plan
//! of every Smac 2D scenario of the ROS reference harness, whose committed outputs hold the
//! bits of Nav2's amd64 build.
#![cfg(target_arch = "x86_64")]

mod common;

use std::num::NonZeroU32;

use camera_core::pose::PlanarPose;
use common::finite::{multiplier, non_negative};
use common::footprints::padded;
use common::poses::planar;
use control_core::positive_finite::PositiveFinite;
use navigation_core::grid::{Cell, CostGrid, GridGeometry};
use navigation_core::planner::{
    PlanError, PlannerParams, SearchParams, SmootherParams, plan, search,
};
use ros_reference::smac_2d::{Cases, Outcome};

/// How far a smoothed pose may lie from Smac's, in meters: the ticket's bound.
const SMOOTHED_M: f64 = 1e-9;

/// How far a pose's heading may lie from Smac's, in radians: Smac carries a heading as
/// its quaternion, which reads back within rounding of the angle that made it.
const HEADING_RAD: f64 = 1e-9;

fn count(value: u32) -> NonZeroU32 {
    NonZeroU32::new(value).expect("the scenario's bound is positive")
}

/// The planner settings of `cases`, with the smoother off.
fn raw_params(cases: &Cases) -> PlannerParams {
    let planner = &cases.planner;
    PlannerParams {
        search: SearchParams {
            cost_travel_multiplier: multiplier(planner.cost_travel_multiplier),
            allow_unknown: planner.allow_unknown,
            max_iterations: count(planner.max_iterations),
        },
        use_final_approach_orientation: planner.use_final_approach_orientation,
        smoother: None,
    }
}

/// The planner settings of `cases`, smoothed at its iteration bound.
fn smoothed_params(cases: &Cases) -> PlannerParams {
    let smoother = &cases.smoother;
    PlannerParams {
        smoother: Some(SmootherParams {
            tolerance_m: non_negative(smoother.tolerance),
            max_iterations: count(smoother.max_iterations),
            w_data: non_negative(smoother.w_data),
            w_smooth: non_negative(smoother.w_smooth),
            refinements: if smoother.do_refinement {
                smoother.refinement_num
            } else {
                0
            },
        }),
        ..raw_params(cases)
    }
}

fn grid_of(cases: &Cases) -> CostGrid {
    let grid = cases.grid;
    let resolution = PositiveFinite::parse(grid.resolution).expect("a positive resolution");
    let geometry = GridGeometry::new(grid.size_x, grid.size_y, resolution, grid.origin)
        .expect("the scenario's grid is one");
    CostGrid::from_costs(geometry, cases.costs.clone()).expect("a cost a cell")
}

fn pose_of(pose: [f64; 3]) -> PlanarPose {
    planar(pose[0], pose[1], pose[2])
}

/// The cell of `grid` holding the position of `pose`, a scenario's start or goal.
fn cell_of(grid: &CostGrid, pose: PlanarPose) -> Cell {
    grid.geometry()
        .cell_of([pose.x(), pose.y()])
        .expect("the scenario's start and goal lie in its grid")
}

/// The difference of two headings, on the circle.
fn heading_difference(a: f64, b: f64) -> f64 {
    let difference = (a - b).rem_euclid(std::f64::consts::TAU);
    difference.min(std::f64::consts::TAU - difference)
}

/// The cell whose corner Smac puts a pose of its unsmoothed path at: the pose less the
/// grid's origin in single precision, over the resolution, rounded.
fn corner_cell(grid: &CostGrid, pose: [f64; 3]) -> (u32, u32) {
    let geometry = grid.geometry();
    let [origin_x, origin_y] = geometry.origin();
    let resolution = geometry.resolution_m();
    let index = |coordinate: f64, origin: f64| {
        ((coordinate - f64::from(origin as f32)) / resolution).round() as u32
    };
    (index(pose[0], origin_x), index(pose[1], origin_y))
}

// The A* path is Smac's with the smoother off, cell for cell and pose for pose bit for bit,
// the smoothed path lies within SMOOTHED_M of Smac's, and a failure is Smac's failure.
#[test]
fn the_planner_gives_smac_2ds_paths() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a Smac 2D scenario"
    );
    for cases in &scenarios {
        assert_eq!(
            cases.planner.tolerance, 0.0,
            "{}: Smac plans with a goal tolerance of 0, as ours does",
            cases.name
        );
        let grid = grid_of(cases);
        let footprint = padded(&cases.footprint, 0.0);
        let (raw, smoothed) = (raw_params(cases), smoothed_params(cases));
        for (index, expected) in cases.plans.iter().enumerate() {
            let what = format!("{} plan {index}", cases.name);
            let (start, goal) = (pose_of(expected.start), pose_of(expected.goal));
            let ours_raw = plan(&grid, &footprint, start, goal, &raw);
            let ours_smoothed = plan(&grid, &footprint, start, goal, &smoothed);
            let failure = match expected.outcome {
                Outcome::Path => None,
                Outcome::StartOccupied => Some(PlanError::StartOccupied(cell_of(&grid, start))),
                Outcome::GoalOccupied => Some(PlanError::GoalOccupied(cell_of(&grid, goal))),
                Outcome::NoValidPath | Outcome::TimedOut => Some(PlanError::NoPath),
            };
            if let Some(failure) = failure {
                assert_eq!(
                    ours_raw,
                    Err(failure),
                    "{what}: Smac ended it {:?}",
                    expected.outcome
                );
                assert_eq!(ours_smoothed, Err(failure), "{what}: smoothed");
                continue;
            }
            let ours_raw = ours_raw.unwrap_or_else(|e| panic!("{what}: ours fails: {e}"));
            let ours_smoothed =
                ours_smoothed.unwrap_or_else(|e| panic!("{what}: smoothed fails: {e}"));
            assert_eq!(
                ours_raw.len(),
                expected.raw.len(),
                "{what}: the A* path's length"
            );
            assert_eq!(
                ours_smoothed.len(),
                expected.smoothed.len(),
                "{what}: the smoothed length"
            );

            let cells = search(
                &grid,
                [start.x(), start.y()],
                [goal.x(), goal.y()],
                &raw.search,
            )
            .unwrap_or_else(|e| panic!("{what}: the search fails: {e}"));
            let smac_cells: Vec<(u32, u32)> = expected
                .raw
                .iter()
                .enumerate()
                .map(|(i, pose)| {
                    let on_goal = i + 1 == expected.raw.len() && pose[..2] == expected.goal[..2];
                    if on_goal {
                        (cell_of(&grid, goal).x(), cell_of(&grid, goal).y())
                    } else {
                        corner_cell(&grid, *pose)
                    }
                })
                .collect();
            let our_cells: Vec<(u32, u32)> = cells
                .cells()
                .iter()
                .map(|cell| (cell.x(), cell.y()))
                .collect();
            if expected.raw.len() > 1 {
                assert_eq!(our_cells, smac_cells, "{what}: the A* cells");
            }

            for (i, (ours, smac)) in ours_raw.iter().zip(&expected.raw).enumerate() {
                assert!(
                    ours.x().to_bits() == smac[0].to_bits()
                        && ours.y().to_bits() == smac[1].to_bits(),
                    "{what} pose {i}: ours ({}, {}), Smac's ({}, {})",
                    ours.x(),
                    ours.y(),
                    smac[0],
                    smac[1]
                );
                assert!(
                    heading_difference(ours.yaw(), smac[2]) <= HEADING_RAD,
                    "{what} pose {i}: our heading {}, Smac's {}",
                    ours.yaw(),
                    smac[2]
                );
            }
            for (i, (ours, smac)) in ours_smoothed.iter().zip(&expected.smoothed).enumerate() {
                assert!(
                    (ours.x() - smac[0]).abs() <= SMOOTHED_M
                        && (ours.y() - smac[1]).abs() <= SMOOTHED_M,
                    "{what} smoothed pose {i}: ours ({}, {}), Smac's ({}, {})",
                    ours.x(),
                    ours.y(),
                    smac[0],
                    smac[1]
                );
                assert!(
                    heading_difference(ours.yaw(), smac[2]) <= HEADING_RAD,
                    "{what} smoothed pose {i}: our heading {}, Smac's {}",
                    ours.yaw(),
                    smac[2]
                );
            }
        }
    }
}
