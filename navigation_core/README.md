# navigation_core

What a mobile base needs to plan a path around what it sees, ported from Nav2 1.5.2. No
ROS, hardware or messaging dependencies.

- `grid` holds the cost grid, Nav2's `Costmap2D`: one byte of cost a cell over square
  cells from an origin, Nav2's cost values (`FREE` 0, `INSCRIBED` 253, `LETHAL` 254,
  `UNKNOWN` 255), and a point's cell (`worldToMap`), a cell's center (`mapToWorld`) and a
  point's continuous cell coordinates (`worldToMapContinuous`); an origin or a far corner
  that single precision does not hold is refused, as Smac 2D places a grid's cells from
  the origin in single precision.
- `PaddedFootprint::new` pads a base's outline, `control_core::footprint::Footprint`, the
  outline base_state:v1's describe_base gives, as Nav2's `padFootprint` pads it, and gives
  the one footprint the cost grid, the planner and the collision checks take, as Nav2
  hands them only its padded footprint; a padding that makes the outline cross itself is
  refused, naming the padding. A `PaddedFootprint` gives its inscribed radius as Nav2's
  `calculateMinAndMaxDistances` takes it, its vertices placed at a pose
  (`transformFootprint`), and whether the base collides at a pose.
- `obstacles` keeps what the sensors see in an occupancy map, `LiveObstacles`: the points
  of depth frames and the hits of laser scans, each placed where the sensor stood at its
  time, the map's columns under the padded footprint cleared after each frame, and the
  map projected to a grid as Nav2's obstacle layer marks points, a drop below the floor
  marked too.
- `saved_grid` reads a saved occupancy grid as Nav2's static layer takes a trinary map.
- `inflation` inflates a grid around its lethal cells, as Nav2's inflation layer does.
- `costmap` composes an obstacle layer, over a saved grid when one is loaded, and inflates
  them, as Nav2's layered cost map composes its layers.
- `planner` holds Smac 2D's planner: `search`, its A* to the goal's cell, and `plan`, its
  path, smoothed by its smoother.

Poses on the floor are `camera_core::pose::PlanarPose`s, and a laser scan is
`camera_core::laser_scan::LaserScan`.

## Quick start

```rust
use std::num::NonZeroU32;

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;
use camera_core::pose::PlanarPose;
use control_core::footprint::Footprint;
use navigation_core::PaddedFootprint;
use navigation_core::grid::{CostGrid, FREE, GridGeometry};
use navigation_core::planner::{
    CostTravelMultiplier, PlannerParams, SearchParams, SmootherParams, plan,
};

let geometry = GridGeometry::new(200, 200, PositiveFinite::parse(0.05)?, [-5.0, -5.0])?;
let grid = CostGrid::filled(geometry, FREE);
let outline = Footprint::parse(&[[-0.22, -0.2], [0.22, -0.2], [0.22, 0.2], [-0.22, 0.2]])?;
let footprint = PaddedFootprint::new(&outline, NonNegativeFinite::parse(0.01)?)?;
let params = PlannerParams {
    search: SearchParams {
        cost_travel_multiplier: CostTravelMultiplier::parse(2.0)?,
        allow_unknown: true,
        max_iterations: NonZeroU32::new(1_000_000).unwrap(),
    },
    use_final_approach_orientation: false,
    smoother: Some(SmootherParams {
        tolerance_m: NonNegativeFinite::parse(1e-10)?,
        max_iterations: NonZeroU32::new(1000).unwrap(),
        w_data: NonNegativeFinite::parse(0.2)?,
        w_smooth: NonNegativeFinite::parse(0.3)?,
        refinements: 2,
    }),
};
let start = PlanarPose { x: 0.0, y: 0.0, yaw: 0.0 };
let goal = PlanarPose { x: 3.0, y: 1.5, yaw: 1.0 };
let path = plan(&grid, &footprint, start, goal, &params)?;
```

## The planner

`search` is Smac 2D's A* (`AStarAlgorithm<Node2D>::createPath`): 8-connected, a step costs
its length in cells, 1 or the square root of 2, times one plus `cost_travel_multiplier`
times the cost of the cell it enters over 252, and the heuristic is the straight-line
distance in cells from a cell's corner to the goal. A cell at or above `INSCRIBED` is an
obstacle for the base's center, and an unknown cell is passable only when `allow_unknown`
is true. The costs, the cell coordinates and `cost_travel_multiplier` are single precision
as Smac holds them, a multiplier beyond single precision's range refused, and the open set
gives the order of libstdc++'s `std::priority_queue`, which holds Smac's open set, so the
search expands Smac's cells in Smac's order and entries of equal cost come out as Smac's
do (`libstdcxx_heap`). The goal tolerance is 0, Smac's `tolerance` at 0: the search plans
to the goal's cell itself.

`plan` is `SmacPlanner2D::createPlan`: the cells' corners as poses in the grid's frame,
smoothed by Smac's smoother, the last pose on the goal, and the last heading the goal's,
or the last step's when `use_final_approach_orientation` is true. The smoother moves each
pose but the ends towards its unsmoothed place by `w_data` and towards its neighbors by
`w_smooth`, pass after pass, until the poses move less than `tolerance_m` in a pass; a
pass that puts a pose where the cost under the footprint's outline is above
`MAX_NON_OBSTACLE` and known, a lethal cell under the outline always among them, is undone
and ends the smoothing, and the iteration bound `max_iterations` ends it after one pass
fewer. A converged smoothing runs again from its result `refinements` times. A path of 11
poses or fewer is left as the search gives it.

Before its search, the planner tests the start, then the goal, by the cost of its cell,
as Smac 2D's collision checker tests a cell for a base taken as its inscribed circle: a
cell at or above `INSCRIBED` is in collision, and an unknown cell is not while
`allow_unknown` is true. It fails with `StartOccupied` or `GoalOccupied` for a cell in
collision, and with `NoPath` when the search ends without reaching the goal's cell, its
iteration bound included. A start or a goal off the grid fails `StartOffGrid` or
`GoalOffGrid`, and a heading that is not finite `HeadingNotFinite`. These failures
follow base_navigation:v1's rule and depart from Smac 2D three ways: Smac 2D never tests
the start, and plans from one in collision; it returns the goal for a start and a goal in
one cell before any test, where both tests run here first; and it fails `StartOccupied`
for any search that ends after one iteration, where a free start walled in fails `NoPath`
here. The planner reads no clock: it runs as Smac 2D runs with a `max_planning_time` no
search reaches.

## The layers

`LiveObstacles` holds an `occupancy_map` voxel map in a frame fixed to the floor, its z up
from the floor, the `ObstacleHeights` that judge its voxels, and the padded footprint.
`insert_depth_frame` inserts a depth frame's points, all at the frame's time, and
`insert_laser_scan` a laser scan's hits, each placed where the sensor stood at its ray's
time and traced from the sensor's origin at the first ray, as Nav2's observation buffer
traces a scan from the origin at its stamp. A depth sample of 0 and a ray that carries a
mark of lidar_2d:v1 make no point: a ray that met nothing frees and marks nothing, as
Nav2's obstacle layer leaves it with `inf_is_valid` false. After each frame, every voxel of the
map's columns under the padded footprint, at the base's pose at the frame's time, takes
one miss's log-odds, as the obstacle layer writes `FREE_SPACE` under its footprint, the
columns found as Nav2's `convexFillCells` finds the cells under a polygon. A voxel the base
stood on reads free, and the next frame that sees an obstacle there marks it again, as the
obstacle layer marks a cleared cell; what the sensors see of the robot itself within the
footprint marks nothing. `clear` forgets every voxel.

`project` gives the `ObstacleLayer` over a grid's geometry, the map placed in the grid's
frame at a `PlanarPose`: a cell is `LETHAL` under the center of an occupied voxel whose
height lies from `min_obstacle_height_m` to `max_obstacle_height_m`, both included, as
Nav2's observation buffer keeps a point, or more than `max_step_down_m` below the floor, a
drop the sensors saw; `FREE` where the map knows a voxel over it; `UNKNOWN` otherwise.
`ObstacleHeights::parse` takes the heights with the side of the map's voxels and refuses a
height that is not a whole number of voxels, at least one, naming the nearest whole
values, so a voxel's center judges as its points do and the floor's own voxels, either
side of the floor, are neither an obstacle nor a drop; `LiveObstacles::new` takes a map of
that side whose bottom lies a voxel or more below `max_step_down_m` under the floor, and
refuses another, naming the lowest bottom that serves. `columns` is the grid of the map's
columns, one cell a column: the grid a navigator in the map's frame plans on. `recentered`
moves the map over other bounds, keeping what it knows of the voxels both hold, for a grid
that moves with the base. `new` and `recentered` refuse a map of more columns on an axis
than a cost grid's axis holds (`MAX_AXIS_CELLS`). `PaddedFootprint::collides_at` tells
whether the base collides at a pose, as Nav2's spin tests each heading of its turn: the
cost under the padded footprint's outline at `LETHAL` or above, so a lethal cell, an
unknown cell or a vertex off the grid.

`SavedGrid::from_image` reads a saved map's 8-bit gray image at the `MapThresholds` of its
YAML file, as nav2_map_server's `loadMapFromFile` reads a trinary map: each pixel's
occupancy `1 - p / 255`, occupied at or above `occupied_thresh`, free at or below
`free_thresh`, unknown between, each in single precision, the image's top row the grid's
last; and each cell's cost as Nav2's static layer takes a trinary map with unknown cells
tracked: `LETHAL`, `FREE` or `UNKNOWN`. A map as map_saver writes it in trinary mode,
pixels 0, 254 and 205 with thresholds 0.65 and 0.196, reads back lethal, free and unknown.

`Inflation::inflate` inflates a grid as Nav2's inflation layer does, over the whole grid:
the distance from each cell to the nearest lethal cell by the single precision
Felzenszwalb-Huttenlocher transform, and the cost of each cell within
`inflation_radius_m` read from a table of 100 entries a cell. A cell's cost is `LETHAL`
on an obstacle, `INSCRIBED` within the footprint's inscribed radius of one, and beyond it
`252 * exp(-cost_scaling_factor * (d - inscribed radius))` at a distance `d` in meters,
truncated. A cell keeps the higher of its cost and the inflated one, and an unknown cell
takes the inflated one only at `INSCRIBED` or above; only lethal cells are obstacles.
`Inflation::parse` makes the inflation of a padded footprint from its `InflationParams`
and refuses an `inflation_radius_m` below the footprint's inscribed radius, 0 among them,
naming the least radius to give: a cell within the inscribed radius of an obstacle and
beyond the inflation radius keeps a cost below `INSCRIBED`, which the planner reads as
clear for the base's center. Nav2 warns on that radius and inflates; the comparison is
Nav2's, so a radius equal to the inscribed radius inflates.

`compose` builds a cost grid as Nav2's layered cost map does with unknown space tracked:
the saved grid's costs when a map is loaded, else every cell unknown, then the obstacle
layer's, where a layer's unknown cell changes nothing, any other cost replaces an unknown
cell and else the higher cost wins, then the inflation, when the grid takes one.

## Agreement with Nav2

Each parity test reads the scenarios of its kind of the
[ROS reference harness](../ros_reference) and the committed outputs of Nav2 1.5.2's
amd64 build, so it runs with no ROS on the machine.

- `tests/ros_reference_smac_2d.rs`: on the same cost grid, footprint, start, goal and
  parameters, a goal tolerance of 0, the A* path is Smac 2D's with its smoother off, cell
  for cell, its poses Smac's bit for bit, and the smoothed path at the same iteration
  bound lies within 1e-9 m of Smac's. It runs on amd64.
- `tests/ros_reference_inflation.rs`: for the same costs, footprint, padding and
  parameters, the padded footprint and its inscribed radius are Nav2's, and the inflated
  grid of a radius at least that inscribed radius is that of Nav2's inflation layer, bit
  for bit; a radius below it is refused, in exactly the scenarios where Nav2 warns and
  inflates. It runs on amd64.
- `tests/ros_reference_obstacle_projection.rs`: for one depth frame's points, the lethal
  cells are those of Nav2's obstacle layer with its rays' clearing off and its footprint
  clearing on, the padded footprint at the same base pose, with voxels of the grid's
  cells, the band's ends on voxel boundaries, and each of Nav2's points 0.1 mm or more
  from a cell's edge and the band's ends.
- `tests/ros_reference_global_costmap.rs`: for the same saved map, points, footprint,
  padding, base pose and parameters, the cost grid is that of a Nav2 global cost map of a
  static, an obstacle and an inflation layer, bit for bit, the cells under the footprint
  cleared over the saved map's unknown cells and its walls kept. The obstacle layer under
  test frees a cell over a voxel its rays cleared, where Nav2's, with its rays' clearing
  off, leaves it unknown; the scenario keeps the saved map's unknown cells out of the
  camera's view, and the test checks that it does. It runs on amd64.
