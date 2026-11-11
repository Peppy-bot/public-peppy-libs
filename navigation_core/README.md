# navigation_core

What a mobile base needs to plan a path around what it sees and to drive it, ported from
Nav2 1.5.2. No ROS, hardware or messaging dependencies.

- `grid` holds the cost grid, Nav2's `Costmap2D`: one byte of cost a cell over square
  cells from an origin, Nav2's cost values (`FREE` 0, `INSCRIBED` 253, `LETHAL` 254,
  `UNKNOWN` 255), and a point's cell (`worldToMap`), a cell's center (`mapToWorld`) and a
  point's continuous cell coordinates (`worldToMapContinuous`); an origin or a far corner
  that single precision does not hold is refused, as Smac 2D places a grid's cells from
  the origin in single precision.
- `PaddedFootprint::new` pads a base's outline, `control_core::footprint::Footprint`, the
  outline base_state:v1's describe_base gives, as Nav2's `padFootprint` pads it, and gives
  the one footprint the cost grid, the planner, the controller, the stop and the collision
  checks take, as Nav2 hands them only its padded footprint; a padding that makes the
  outline cross itself is refused, naming the padding. A `PaddedFootprint` gives its
  inscribed and circumscribed radii as Nav2's `calculateMinAndMaxDistances` takes them,
  its vertices placed at a pose (`transformFootprint`), and whether the base collides at a
  pose.
- `obstacles` keeps what the sensors see in an occupancy map, `LiveObstacles`: the points
  of depth frames and the hits of laser scans, each placed where the sensor stood at its
  time, the map's columns under the padded footprint cleared after each frame, and the
  map projected to a grid as Nav2's obstacle layer marks points, a drop below the floor
  marked too; and, for each source, the columns its newest frame marked, which the stop
  reads beside the grid.
- `saved_grid` reads a saved occupancy grid as Nav2's static layer takes a trinary map.
- `inflation` inflates a grid around its lethal cells, as Nav2's inflation layer does.
- `costmap` composes an obstacle layer, over a saved grid when one is loaded, and inflates
  them, as Nav2's layered cost map composes its layers.
- `planner` holds Smac 2D's planner: `search`, its A* to the goal's cell, and `plan`, its
  path, smoothed by its smoother.
- `path_handler` prunes a planner's path to the poses ahead of the base, as Nav2's
  controller server hands it to its controller (`GlobalPlan::prune`, the feasible path
  handler).
- `goal_checker` judges whether the base reached the goal of the plan it follows, as
  Nav2's simple goal checker with `stateful` true judges it: once the position is reached,
  only the heading must still converge. Each `GlobalPlan` holds its own goal checker,
  which judges the position afresh, as Nav2's controller server resets its goal checker
  as it takes a new plan. A plan ahead longer than `path_length_tolerance_m`
  keeps the position unreached; Nav2's default of 1 m is the value to keep, so a base near
  a goal its plan still leads away from has not reached it, and the pruned plan ends at
  the goal whose heading the controller then turns the base to.
- `progress_checker` judges whether the base makes progress, as Nav2's simple progress
  checker judges it.
- `controller` is Nav2's regulated pure pursuit (`PurePursuit::compute`): the forward
  speed and turn rate that follow the pruned plan.
- `stop` slows the commanded velocity so that no occupied point lies inside the padded
  footprint at any step of the projection the slowed velocity covers within a time, as
  Nav2's collision monitor slows it in its approach mode, one point a contact, and stops
  the base while a source's readings stop arriving, which source it gives back
  (`Stop::reading_arrived`, `Stop::silent_source`, `Stop::decide`), each reading the
  padded footprint and the sources of the `LiveObstacles` it is given. Its points are
  those `LiveObstacles::occupied_seen_from` gives, the base and the map placed in the
  grid's frame by `GridPoses`: the centers of the cost grid's lethal cells and of the
  columns each source's newest frame marked. A decision before the stop's previous one
  comes from a clock that went back: every source then reads silent until its next
  reading. A reading that arrived after a decision's now counts from that decision, as
  control_core's limiter gives a setpoint after its tick's time that time, so it holds for
  at most the source timeout from the first decision that reads it, after a step back the
  stop finds and after one it does not; Nav2's monitor holds a stamp after now valid until
  the clock passes the stamp plus its source timeout.

Poses on the floor are `camera_core::pose::PlanarPose`s, a laser scan is
`camera_core::laser_scan::LaserScan`, and a body velocity is `control_core::twist::Twist`.

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
let start = PlanarPose::new(0.0, 0.0, 0.0)?;
let goal = PlanarPose::new(3.0, 1.5, 1.0)?;
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

Before its search, the planner tests the start, then the goal, by the cost of its cell, as
Smac 2D's collision checker tests a cell for a base taken as its inscribed circle: a cell
at or above `INSCRIBED` is in collision, and an unknown cell is not while `allow_unknown`
is true. It fails with `StartOccupied` or `GoalOccupied` for a cell in collision, and with
`NoPath` when the search ends without reaching the goal's cell, its iteration bound
included. A start or a goal off the grid fails `StartOffGrid` or `GoalOffGrid`. These
failures follow base_navigation:v1's rule and depart from Smac 2D three ways: Smac 2D
never tests the start, and plans from one in collision; it returns the goal for a start
and a goal in one cell before any test, where both tests run here first; and it fails
`StartOccupied` for any search that ends after one iteration, where a free start walled in
fails `NoPath` here. The planner reads no clock: it runs as Smac 2D runs with a
`max_planning_time` no search reaches.

## The layers

`LiveObstacles` holds an `occupancy_map` voxel map in a frame fixed to the floor, its z up
from the floor, the `ObstacleHeights` that judge its voxels, the padded footprint, and the
count of the sources that give its frames. Each insert names its frame's source and places
the sensor by the base's pose on the floor at the frame's time and the sensor's pose on
the base. `insert_depth_frame` inserts a depth frame's points, all at the frame's time,
and `insert_laser_scan` a laser scan's hits, the base's poses at the scan's first and last
rays given as a `camera_core::laser_scan::Sweep`, each hit placed where the sensor stood
at its ray's time and traced from the sensor's origin at the first ray, as Nav2's
observation buffer traces a scan from the origin at its stamp. Nav2's obstacle layer
places every hit of a scan where the sensor stood at the scan's stamp; here a sensor that
moves while it sweeps places each hit where it measured it. Two more departures follow
from `occupancy_map`'s insert: every frame goes into the voxel map, where Nav2 skips an
observation whose sensor origin lies off its grid, and a point's range is its distance
from the sensor in 3-D, where Nav2's marking measures it in the plane, so a point beyond
the range in 3-D and within it in the plane clears to the range here. A depth sample of 0
and a ray that carries a mark of lidar_2d:v1 make no point: a ray that met nothing frees
and marks nothing, as Nav2's obstacle layer leaves it with `inf_is_valid` false. After
each frame, every voxel of the map's columns under the padded footprint, at the base's
pose at the frame's time, takes one miss's log-odds, as the obstacle layer writes
`FREE_SPACE` under its footprint, the columns found as Nav2's `convexFillCells` finds the
cells under a polygon, and none of them is a drop: the base stands on floor there. A voxel
the base stood on reads free, and the next frame that sees an obstacle there marks it
again, as the obstacle layer marks a cleared cell. `clear` empties the map and forgets the
drops, as base_navigation:v1's clear_grid empties the grid, and keeps each source's newest
frame, whose obstacles and drops stay the stop's points beside the grid until the source's
next frame replaces it, as Nav2's collision monitor reads its sources' newest points from
their topics, which a clear of the cost map leaves as they are.

A return inside the base's unpadded outline is the robot's own body, judged in the base's
frame at the return's own time, where the sensor's pose on the base places it: its ray
clears free space up to it, and it marks nothing, so the body enters neither the map nor a
source's newest frame, however the base moves while a lidar sweeps. Nav2's obstacle layer
marks the body's returns as any other and frees them by its clearing under the padded
footprint at one pose, which leaves a body return lethal where the footprint's raster
leaves its cell out; and with each hit placed at its ray's time, a body hit that a lidar
measured late in the sweep of a moving base lies past the footprint at the sweep's first
ray, where the clearing runs.

A drop is a column whose floor lies more than `max_step_down_m` down, the step's plane,
which each frame's rays find through the floor's plane. Each ray that points down from
above the floor reads the floor's plane in the column where its line crosses it: past,
when its reading lies below the step's plane (a return there, or no return with its range
reaching there, as `occupancy_map`'s `Scan::readings` reads it), and short, when it
returns above the floor's plane, before the crossing. A column where at least three rays
of one frame read past, more than the rays that read short and the returns of that frame
that stand in it at or above the step's plane together, is a drop until the footprint
clears it: one or two wrong far readings through a column mark none, and wrong far
readings among rays that see an obstacle before the floor, or through a column whose floor
the frame sees, are outnumbered there. A ray over an edge crosses the floor's plane just
past the edge, so a drop shows from the edge on before a sensor sees the lower floor, the
cell that holds the edge too when more of the frame's rays read past the step through it
than see its floor.

Wrong far readings fall at random, so three of one frame can fall in a column whose floor
the frame sees little of and mark a drop there. Measured on rendered depth frames of a
flat floor, from two camera mounts at two speeds, with 2% and with 4% of the readings
nearer than 2 m reading a depth drawn uniformly from their own out to the camera's range,
ten seeds each, 24,000 frames in all: the rule marks 52 false drop cells, and one of them
lies in the base's path within the stop's horizon for 33 frames of one drive. On the same
frames, a count of two marks 152, a count of one 3,839, and a count of three that need not
outnumber the rays short and standing 516,867.

`project` gives the `ObstacleLayer` over a grid's geometry, the voxel map placed in the
grid's frame at a `PlanarPose`. A voxel stands for every point inside it, and Nav2's
obstacle layer marks the cell of every point, so a column of the voxel map marks every
cell whose square overlaps the column's square, on a grid turned against the map or finer
than it too: a cell is `LETHAL` where its square overlaps the square of a column that
holds an occupied voxel whose center lies from `min_obstacle_height_m` to
`max_obstacle_height_m`, both included, as Nav2's observation buffer keeps a point, or of
a drop's column; `FREE` where it overlaps the square of any other column the map knows a
voxel of; `UNKNOWN` otherwise. Two squares overlap when they share more than a line or a
point, which the separating axes of convex polygons decide exactly, and each column's
square stands a sliver in from the column's sides, so on a grid of the map's own lattice a
column marks its own cell alone. A placement that puts a corner of a column's square at a
point that is not finite fails with `PlacementNotFinite`; a column whose square's extent
on the grid's lattice is not finite marks no cell, the cells of the grid that its square
overlaps among them, as the overlap is decided on that extent. `ObstacleHeights::parse`
takes the heights with the side of the voxels, `ObstacleHeightsParams`, and refuses a
height of the band that is not a whole number of voxels, at least one, naming the nearest
whole values, so a voxel's center judges as its points do, but for a point exactly at
`max_obstacle_height_m`, which lies in the voxel above the band and marks nothing where
Nav2 keeps it, and the floor's own voxels, either side of the floor, are no obstacle;
`max_step_down_m` takes any height above the floor. `LiveObstacles::new` makes its voxel
map in a frame and over bounds, of the voxels the heights judge and of no more columns on
an axis than a cost grid's axis holds (`MAX_AXIS_CELLS`), and `recentered` takes bounds of
no more. `columns` is the grid of the voxel map's columns, one cell a column: the grid a
navigator plans on in the voxel map's frame. `recentered` moves the map, the drops and
each source's newest frame over other bounds, keeping what it knows of the columns both
hold, for a grid that moves with the base.

For each source, `LiveObstacles` keeps the columns its newest frame marked, as the frame
marked them before the clearing under the footprint: the column of each return of the
frame but the robot's own body whose voxel's center lies in the band, the returns the map
marks; and each column the frame's rays show a drop in, but for one whose center lies
inside the unpadded outline at the frame's base pose, floor the base stands on.
`occupied_seen_from` gives the points the stop reads: the centers of a cost grid's lethal
cells and of those columns. Nav2's collision monitor reads each source's newest points,
which the obstacle layer's clearing under the footprint leaves as they are, so an obstacle
in a column the padded footprint stands in, outside the unpadded outline, is a point the
stop reads, as it is one the monitor reads. A return inside the unpadded outline gives the
stop no point, where the monitor reads it: a real obstacle there does not hold the base,
where the monitor stops the base for it, and the stop reads a column at its center, so the
padded footprint comes up to the center of a column whose obstacle's face stands nearer
than the center by more than the padding, and that face then lies inside the unpadded
outline. A body whose surface lies on the outline scatters returns across its edge, and
those outside it hold the base, as they hold the monitor.

`PaddedFootprint::collides_at` checks one pose of base_navigation:v1's sweep, under which
a turn and a straight drive fail when the padded footprint placed at a step of the
controller meets an occupied cell: a lethal cell under the padded footprint's outline
collides, as does a vertex off the grid, which Nav2's `footprintCost` reads as lethal, and
a cell never seen counts as free, as base_navigation:v1 counts it, where Nav2's spin
collides on an unknown cell too. The cost under the outline reads lethal at the first edge
that holds a lethal cell, whatever the edges before it hold: Nav2's `footprintCost` ends
early only once its highest cost so far is lethal, so an unknown cell on an earlier edge
makes a later lethal cell read as unknown there. The smoother and the controller's arc
check read the same cost. The check reads the cells under the outline's edges alone, a
raster one cell thick, so a sweep checked at poses a step apart is sure to find every
lethal cell its footprint passes over only if no point of the outline moves farther than
the grid's cell from one checked pose to the next. A sweep that keeps that bound can still
pass a lethal cell: the raster moves by whole cells as its edges cross the lattice, so at
a step of under a cell, on a straight drive as on a turn, it can step over a lethal cell
of one cell. A step that moves the base's center a length along an arc of curvature kappa
moves a point of the outline about kappa times its distance from the arc's center times
that length, 1 + kappa |y| for a point beside the center at lateral offset y on the outer
side. Nav2 checks a sweep at poses alike: its arc check, and the controller's, which ports
it, step the center a cell along a drive whatever its curvature, and its spin and its
drive on a heading check the poses a cycle of their frequency apart.

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
  cells, the band's ends on voxel boundaries, each of Nav2's points 0.1 mm or more from a
  cell's edge and the band's ends, and each point in the band inside the unpadded outline,
  or 0.1 mm or less past it, at its frame's base pose, the robot's own body, which marks
  nothing here, in a cell Nav2 holds not lethal.
- `tests/ros_reference_global_costmap.rs`: for the same saved map, points, footprint,
  padding, base pose and parameters, the cost grid is that of a Nav2 global cost map of a
  static, an obstacle and an inflation layer, bit for bit, the cells under the footprint
  cleared over the saved map's unknown cells and its walls kept. The obstacle layer under
  test frees a cell over a voxel its rays cleared, where Nav2's, with its rays' clearing
  off, leaves it unknown; the scenario keeps the saved map's unknown cells out of the
  camera's view, and the test checks that it does. It runs on amd64.
