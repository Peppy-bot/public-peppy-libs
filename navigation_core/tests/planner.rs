//! Properties of the planner on drawn grids: Smac 2D's A* finds a path as cheap as an
//! exhaustive search on small grids, crossing only cells the base's center may stand on;
//! a start or a goal in an obstacle, and a goal a wall closes off, give their failures; a
//! goal never seen plans with unknown cells passable and is occupied without; the
//! iteration bound ends a search; and on live obstacles recentered under the base, a plan
//! starts in the start's own cell of the new columns.

mod common;

use std::num::{NonZeroU32, NonZeroUsize};

use camera_core::pose::PlanarPose;
use common::finite::multiplier;
use common::footprints::padded;
use common::poses::planar;
use common::properties::{
    COST_TOLERANCE, corner, grids, obstacle_costs, passable, passable_cells, with_cost,
};
use control_core::positive_finite::PositiveFinite;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::PaddedFootprint;
use navigation_core::grid::{Cell, CostGrid, FREE, GridGeometry, UNKNOWN};
use navigation_core::obstacles::{LiveObstacles, ObstacleHeights, ObstacleHeightsParams};
use navigation_core::planner::{
    CostTravelMultiplier, PlanError, PlannerParams, SearchParams, plan, search,
};
use occupancy_map::{Bounds, Frame};

/// The largest side of a grid the exhaustive search runs on.
const SMALL_SIDE: u32 = 8;

/// Drawn search settings: a cost multiplier, whether unknown cells are passable, and a
/// bound no small grid reaches.
fn search_params(tc: &TestCase) -> SearchParams {
    SearchParams {
        cost_travel_multiplier: multiplier(tc.draw(gs::floats().min_value(0.0).max_value(5.0))),
        allow_unknown: tc.draw(gs::booleans()),
        max_iterations: NonZeroU32::new(1_000_000).expect("positive"),
    }
}

/// The pose at `point`, heading along +x.
fn at(point: [f64; 2]) -> PlanarPose {
    planar(point[0], point[1], 0.0)
}

/// The position of `pose`.
fn xy(pose: PlanarPose) -> [f64; 2] {
    [pose.x(), pose.y()]
}

/// The pose at the center of `cell`.
fn at_center(grid: &CostGrid, cell: Cell) -> PlanarPose {
    at(grid.geometry().center_of(cell))
}

/// A square footprint inside one cell of the smallest drawn grids.
fn small_footprint() -> PaddedFootprint {
    padded(&[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]], 0.0)
}

fn planner_params(search: SearchParams) -> PlannerParams {
    PlannerParams {
        search,
        use_final_approach_orientation: false,
        smoother: None,
    }
}

/// The cost of a step from `from` to `to`: its length in cells times one plus the
/// multiplier times `to`'s cost over 252, Smac 2D's step cost, in double precision.
fn step_cost(grid: &CostGrid, multiplier: f64, from: Cell, to: Cell) -> f64 {
    let diagonal = from.x() != to.x() && from.y() != to.y();
    let length = if diagonal {
        std::f64::consts::SQRT_2
    } else {
        1.0
    };
    length * (1.0 + multiplier * f64::from(grid.cost(to)) / 252.0)
}

/// The cells of `grid` one step from `cell`, on either axis or both.
fn neighbors(grid: &CostGrid, cell: Cell) -> Vec<Cell> {
    let geometry = grid.geometry();
    (-1_i64..=1)
        .flat_map(|dy| (-1_i64..=1).map(move |dx| (dx, dy)))
        .filter(|&step| step != (0, 0))
        .filter_map(|(dx, dy)| {
            let x = u32::try_from(i64::from(cell.x()) + dx).ok()?;
            let y = u32::try_from(i64::from(cell.y()) + dy).ok()?;
            geometry.cell(x, y)
        })
        .collect()
}

/// The least cost of a path from `start` to `goal` over passable cells, by Dijkstra's
/// search of every cell; none when no path joins them.
fn cheapest(grid: &CostGrid, params: &SearchParams, start: Cell, goal: Cell) -> Option<f64> {
    let multiplier = f64::from(params.cost_travel_multiplier.get());
    let cells: Vec<Cell> = grid.geometry().cells().collect();
    let index = |cell: Cell| {
        cells
            .iter()
            .position(|&c| c == cell)
            .expect("a cell of the grid")
    };
    let mut cost = vec![f64::INFINITY; cells.len()];
    let mut done = vec![false; cells.len()];
    cost[index(start)] = 0.0;
    while let Some(current) = (0..cells.len())
        .filter(|&i| !done[i] && cost[i].is_finite())
        .min_by(|&a, &b| cost[a].total_cmp(&cost[b]))
    {
        done[current] = true;
        for next in neighbors(grid, cells[current]) {
            if !passable(grid.cost(next), params.allow_unknown) {
                continue;
            }
            let through = cost[current] + step_cost(grid, multiplier, cells[current], next);
            let i = index(next);
            if through < cost[i] {
                cost[i] = through;
            }
        }
    }
    Some(cost[index(goal)]).filter(|cost| cost.is_finite())
}

/// The cost of the path through `cells`, each step a step to a neighbor.
fn path_cost(grid: &CostGrid, params: &SearchParams, cells: &[Cell]) -> f64 {
    let multiplier = f64::from(params.cost_travel_multiplier.get());
    cells
        .windows(2)
        .map(|step| {
            assert!(
                neighbors(grid, step[0]).contains(&step[1]),
                "{:?} to {:?} is no step to a neighbor",
                step[0],
                step[1]
            );
            step_cost(grid, multiplier, step[0], step[1])
        })
        .sum()
}

// With the goal on a cell's corner, the goal tolerance 0, the straight-line heuristic never
// overestimates, and the A* path costs no more than the cheapest path an exhaustive search
// finds: it starts at the start's cell, ends at the goal's, crosses only passable cells,
// and exists exactly when a path does.
#[hegel::test(test_cases = 300)]
fn a_star_is_as_cheap_as_brute_force_on_small_grids(tc: TestCase) {
    let grid = tc.draw(grids(SMALL_SIDE));
    let params = search_params(&tc);
    let cells = passable_cells(&grid, params.allow_unknown);
    tc.assume(!cells.is_empty());
    let start = tc.draw(gs::sampled_from(&cells));
    let goal = tc.draw(gs::sampled_from(&cells));
    let found = search(
        &grid,
        grid.geometry().center_of(start),
        corner(&grid, goal),
        &params,
    );
    match (found, cheapest(&grid, &params, start, goal)) {
        (Ok(path), Some(least)) => {
            let path = path.cells();
            assert_eq!(
                path.first(),
                Some(&start),
                "the path starts at the start's cell"
            );
            assert_eq!(path.last(), Some(&goal), "the path ends at the goal's cell");
            for &cell in path {
                assert!(
                    passable(grid.cost(cell), params.allow_unknown),
                    "the path crosses {cell:?}, of cost {}",
                    grid.cost(cell)
                );
            }
            let cost = path_cost(&grid, &params, path);
            assert!(
                cost <= least + COST_TOLERANCE,
                "the A* path costs {cost}, the cheapest {least}"
            );
        }
        (Err(PlanError::NoPath), None) => {}
        (found, least) => panic!("A* gives {found:?}, the exhaustive search {least:?}"),
    }
}

// A goal on a cell that is an obstacle for the base's center fails GoalOccupied, from any
// passable start.
#[hegel::test]
fn a_goal_in_an_obstacle_fails_goal_occupied(tc: TestCase) {
    let drawn = tc.draw(grids(SMALL_SIDE));
    let params = search_params(&tc);
    let goal = tc.draw(gs::sampled_from(
        drawn.geometry().cells().collect::<Vec<_>>(),
    ));
    let grid = with_cost(&drawn, goal, tc.draw(obstacle_costs(params.allow_unknown)));
    let starts = passable_cells(&grid, params.allow_unknown);
    tc.assume(!starts.is_empty());
    let start = at_center(&grid, tc.draw(gs::sampled_from(&starts)));
    let goal_pose = at_center(&grid, goal);
    assert_eq!(
        search(&grid, xy(start), xy(goal_pose), &params),
        Err(PlanError::GoalOccupied(goal))
    );
    assert_eq!(
        plan(
            &grid,
            &small_footprint(),
            start,
            goal_pose,
            &planner_params(params)
        ),
        Err(PlanError::GoalOccupied(goal))
    );
}

// A start on a cell that is an obstacle for the base's center fails StartOccupied, towards
// any goal.
#[hegel::test]
fn a_start_in_an_obstacle_fails_start_occupied(tc: TestCase) {
    let drawn = tc.draw(grids(SMALL_SIDE));
    let params = search_params(&tc);
    let all: Vec<Cell> = drawn.geometry().cells().collect();
    let start = tc.draw(gs::sampled_from(&all));
    let grid = with_cost(&drawn, start, tc.draw(obstacle_costs(params.allow_unknown)));
    let start_pose = at_center(&grid, start);
    let goal = at_center(&grid, tc.draw(gs::sampled_from(&all)));
    assert_eq!(
        search(&grid, xy(start_pose), xy(goal), &params),
        Err(PlanError::StartOccupied(start))
    );
    assert_eq!(
        plan(
            &grid,
            &small_footprint(),
            start_pose,
            goal,
            &planner_params(params)
        ),
        Err(PlanError::StartOccupied(start))
    );
}

// A wall of obstacle cells across the grid closes the start's side off from the goal's: no
// path reaches the goal's cell, and the planner fails NoPath.
#[hegel::test]
fn a_closed_grid_fails_no_path(tc: TestCase) {
    let drawn = tc.draw(grids(SMALL_SIDE));
    let size_x = drawn.geometry().size_x();
    tc.assume(size_x >= 3);
    let params = search_params(&tc);
    let wall_x = tc.draw(gs::integers::<u32>().min_value(1).max_value(size_x - 2));
    let wall_cost = tc.draw(obstacle_costs(params.allow_unknown));
    let geometry = drawn.geometry();
    let grid = (0..geometry.size_y())
        .filter_map(|y| geometry.cell(wall_x, y))
        .fold(drawn.clone(), |grid, cell| {
            with_cost(&grid, cell, wall_cost)
        });
    let cells = passable_cells(&grid, params.allow_unknown);
    let west: Vec<Cell> = cells.iter().copied().filter(|c| c.x() < wall_x).collect();
    let east: Vec<Cell> = cells.iter().copied().filter(|c| c.x() > wall_x).collect();
    tc.assume(!west.is_empty() && !east.is_empty());
    let start = at_center(&grid, tc.draw(gs::sampled_from(&west)));
    let goal_cell = tc.draw(gs::sampled_from(&east));
    let offset: [f64; 2] = tc.draw(gs::arrays(gs::floats().min_value(0.0).max_value(0.99)));
    let [x, y] = corner(&grid, goal_cell);
    let resolution = geometry.resolution_m();
    let goal = at([x + offset[0] * resolution, y + offset[1] * resolution]);
    assert_eq!(geometry.cell_of(xy(goal)), Some(goal_cell));
    assert_eq!(
        search(&grid, xy(start), xy(goal), &params),
        Err(PlanError::NoPath)
    );
    assert_eq!(
        plan(
            &grid,
            &small_footprint(),
            start,
            goal,
            &planner_params(params)
        ),
        Err(PlanError::NoPath)
    );
}

// On a grid of cells the base's center may stand on, a goal in a cell never seen plans to
// that cell while unknown cells are passable, and fails GoalOccupied while they are not,
// as Smac 2D's collision checker passes an unknown cell only with allow_unknown.
#[hegel::test]
fn an_unseen_goal_plans_with_unknown_allowed_and_fails_goal_occupied_without(tc: TestCase) {
    let drawn = tc.draw(grids(SMALL_SIDE));
    let geometry = drawn.geometry();
    let open: Vec<u8> = drawn
        .costs()
        .iter()
        .map(|&cost| if passable(cost, true) { cost } else { FREE })
        .collect();
    let open = CostGrid::from_costs(geometry, open).expect("the same grid");
    let all: Vec<Cell> = geometry.cells().collect();
    let goal = tc.draw(gs::sampled_from(&all));
    let grid = with_cost(&open, goal, UNKNOWN);
    let starts: Vec<Cell> = passable_cells(&grid, false)
        .into_iter()
        .filter(|&cell| cell != goal)
        .collect();
    tc.assume(!starts.is_empty());
    let start = at_center(&grid, tc.draw(gs::sampled_from(&starts)));
    let goal_pose = at_center(&grid, goal);
    let drawn_params = search_params(&tc);
    let with_unknown = SearchParams {
        allow_unknown: true,
        ..drawn_params
    };
    let path = search(&grid, xy(start), xy(goal_pose), &with_unknown)
        .expect("every cell is passable with unknown cells allowed");
    assert_eq!(path.cells().last(), Some(&goal));
    let without_unknown = SearchParams {
        allow_unknown: false,
        ..drawn_params
    };
    assert_eq!(
        search(&grid, xy(start), xy(goal_pose), &without_unknown),
        Err(PlanError::GoalOccupied(goal))
    );
}

// Along a row of ten free cells, the search expands the ten cells, the goal's last: a bound
// of nine ends it with NoPath, and a bound of ten reaches the goal.
#[test]
fn the_iteration_bound_ends_a_search() {
    let geometry = GridGeometry::new(10, 1, PositiveFinite::parse(1.0).unwrap(), [0.0; 2]).unwrap();
    let grid = CostGrid::filled(geometry, FREE);
    let start = at_center(&grid, geometry.cell(0, 0).unwrap());
    let goal = at_center(&grid, geometry.cell(9, 0).unwrap());
    let bounded = |iterations: u32| SearchParams {
        cost_travel_multiplier: multiplier(1.0),
        allow_unknown: false,
        max_iterations: NonZeroU32::new(iterations).unwrap(),
    };
    assert_eq!(
        search(&grid, xy(start), xy(goal), &bounded(9)),
        Err(PlanError::NoPath)
    );
    let path = search(&grid, xy(start), xy(goal), &bounded(10)).expect("room to finish");
    assert_eq!(path.cells().len(), 10);
}

// Live obstacles recentered by a drawn whole number of cells along each axis, their new
// bounds grown by a drawn number of cells: their columns are a grid of another size, whose
// origin moves with the shift. A plan on the new columns, from a start and to a goal in
// another cell, both drawn inside cells where both bounds hold, starts at the corner of the
// start's own cell of the new columns and ends at the goal, so a pose plans from its own
// cell on the grid it plans on.
#[hegel::test(test_cases = 50)]
fn a_recentered_grid_plans_from_the_starts_own_cell(tc: TestCase) {
    let side = 0.1;
    let voxel = PositiveFinite::parse(side).expect("positive");
    let bounds = |low: [f64; 2], high: [f64; 2]| Bounds {
        min: [low[0], low[1], -0.5],
        max: [high[0], high[1], 0.5],
    };
    let heights = ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: 0.1,
        max_obstacle_height_m: 0.3,
        max_step_down_m: 0.1,
        voxel_m: voxel,
    })
    .expect("whole voxels");
    let live = LiveObstacles::new(
        Frame::parse("odom").expect("a frame name"),
        bounds([-1.0, -1.0], [1.0, 1.0]),
        heights,
        small_footprint(),
        NonZeroUsize::MIN,
    )
    .expect("a map");
    let cells = |tc: &TestCase, low: i32, high: i32| {
        f64::from(tc.draw(gs::integers::<i32>().min_value(low).max_value(high))) * side
    };
    let shift = [cells(&tc, -5, 5), cells(&tc, -5, 5)];
    let grow = cells(&tc, 0, 5);
    let moved = live
        .recentered(bounds(
            [-1.0 + shift[0], -1.0 + shift[1]],
            [1.0 + shift[0] + grow, 1.0 + shift[1] + grow],
        ))
        .expect("bounds the tree holds");
    let new = moved.columns();
    // A point where both bounds hold, a fifth of a cell or more inside its cell, so the
    // single precision in which the planner locates it and the double precision of
    // `cell_of` find one cell.
    let shared = |axis: usize| {
        let low = -1.0 + shift[axis].max(0.0) + side;
        let high = 1.0 + shift[axis].min(0.0) - side;
        let drawn = tc.draw(gs::floats().min_value(low).max_value(high));
        let offset = tc.draw(gs::floats().min_value(-0.3).max_value(0.3));
        ((drawn / side).floor() + 0.5 + offset) * side
    };
    let start = at([shared(0), shared(1)]);
    let goal = at([shared(0), shared(1)]);
    let start_cell = new
        .cell_of(xy(start))
        .expect("the new columns hold the start");
    tc.assume(new.cell_of(xy(goal)) != Some(start_cell));
    let grid = CostGrid::filled(new, FREE);
    let params = planner_params(SearchParams {
        cost_travel_multiplier: multiplier(2.0),
        allow_unknown: false,
        max_iterations: NonZeroU32::new(1_000_000).expect("positive"),
    });
    let path = plan(&grid, &small_footprint(), start, goal, &params).expect("a free grid");
    let first = path.first().expect("a path holds a pose");
    assert_eq!(
        new.cell_of([first.x() + side / 2.0, first.y() + side / 2.0]),
        Some(start_cell),
        "the path starts at {first:?}, the start lies at {start:?}"
    );
    let last = path.last().expect("a path holds a pose");
    assert_eq!(xy(*last), xy(goal));
}

// A start or a goal off the grid, or a searched point that is not a number, is refused by
// name, the start first.
#[test]
fn a_start_or_a_goal_off_the_grid_is_refused() {
    let geometry = GridGeometry::new(4, 4, PositiveFinite::parse(1.0).unwrap(), [0.0; 2]).unwrap();
    let grid = CostGrid::filled(geometry, FREE);
    let params = planner_params(SearchParams {
        cost_travel_multiplier: multiplier(1.0),
        allow_unknown: false,
        max_iterations: NonZeroU32::new(100).unwrap(),
    });
    let on = at([1.5, 1.5]);
    let off = at([4.5, 1.5]);
    let footprint = small_footprint();
    assert_eq!(
        plan(&grid, &footprint, off, off, &params),
        Err(PlanError::StartOffGrid(xy(off)))
    );
    assert_eq!(
        plan(&grid, &footprint, on, off, &params),
        Err(PlanError::GoalOffGrid(xy(off)))
    );
    assert_eq!(
        search(&grid, xy(on), xy(off), &params.search),
        Err(PlanError::GoalOffGrid(xy(off)))
    );
    assert!(matches!(
        search(&grid, [f64::NAN, 1.5], xy(on), &params.search),
        Err(PlanError::StartOffGrid([x, _])) if x.is_nan()
    ));
}

// A grid that reaches the edge of single precision's range, its origin at 3e38 m and its
// four cells 1e37 m wide, plans: its cells' corners from Smac 2D's single precision origin
// are finite, the path runs through three cells, and its last pose stands on the goal.
#[test]
fn a_grid_at_the_edge_of_single_precision_plans() {
    let (origin, side) = (3e38, 1e37);
    let geometry =
        GridGeometry::new(4, 4, PositiveFinite::parse(side).unwrap(), [origin, 0.0]).unwrap();
    let grid = CostGrid::filled(geometry, FREE);
    let params = planner_params(SearchParams {
        cost_travel_multiplier: multiplier(1.0),
        allow_unknown: false,
        max_iterations: NonZeroU32::new(100).unwrap(),
    });
    let start = planar(origin + 0.5 * side, 0.5 * side, 0.0);
    let goal = planar(origin + 2.5 * side, 0.5 * side, 0.0);
    let path =
        plan(&grid, &small_footprint(), start, goal, &params).expect("a path on a free grid");
    assert_eq!(path.len(), 3);
    assert_eq!(path.last().map(|pose| xy(*pose)), Some(xy(goal)));
}

// A multiplier below 0, beyond single precision's range or not a number is refused at
// parse, naming the largest one to give, the double just past it included, and the largest
// one parses.
#[test]
fn a_multiplier_beyond_single_precision_is_refused() {
    let refused = CostTravelMultiplier::parse(1e39).unwrap_err();
    assert!(refused.to_string().contains("3.4028235e38"), "{refused}");
    for value in [
        f64::NAN,
        -1.0,
        -0.5,
        f64::from(f32::MAX).next_up(),
        f64::INFINITY,
    ] {
        assert!(CostTravelMultiplier::parse(value).is_err(), "{value}");
    }
    assert_eq!(
        CostTravelMultiplier::parse(f64::from(f32::MAX)).map(CostTravelMultiplier::get),
        Ok(f32::MAX)
    );
    assert_eq!(
        CostTravelMultiplier::parse(-0.0).map(|m| m.get().to_bits()),
        Ok(0.0_f32.to_bits())
    );
}

// A start and a goal in one cell plan the one pose of the goal, heading as the start
// does when the final approach's heading is asked for and as the goal does when not.
#[test]
fn a_start_and_a_goal_in_one_cell_keep_the_start_heading_on_a_final_approach() {
    let geometry = GridGeometry::new(4, 4, PositiveFinite::parse(1.0).unwrap(), [0.0; 2]).unwrap();
    let grid = CostGrid::filled(geometry, FREE);
    let start = planar(1.2, 1.3, 0.3);
    let goal = planar(1.7, 1.6, 1.2);
    let search = SearchParams {
        cost_travel_multiplier: multiplier(1.0),
        allow_unknown: false,
        max_iterations: NonZeroU32::new(100).unwrap(),
    };
    let approach = PlannerParams {
        use_final_approach_orientation: true,
        ..planner_params(search)
    };
    assert_eq!(
        plan(&grid, &small_footprint(), start, goal, &approach),
        Ok(vec![planar(goal.x(), goal.y(), 0.3)])
    );
    assert_eq!(
        plan(
            &grid,
            &small_footprint(),
            start,
            goal,
            &planner_params(search)
        ),
        Ok(vec![goal])
    );
}
