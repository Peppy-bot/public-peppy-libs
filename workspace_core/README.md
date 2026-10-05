# workspace_core

Where a robot can work: the definitions, the limits and the judgement that a
robot's own answer (its backbone's `workspace:v1`, from its design) and a
simulation's answer (Waldo's `scene_workspace:v1`, from a simulated world) share,
so that both give the same kind of answer, with the same limits and in the same
words. Pure Rust on plain arrays (`[f64; 3]` points, row-major `[f64; 9]`
rotations): no math library, no robot description, no world.

A backbone in Python uses it through its bindings,
[`workspace_core_py`](../workspace_core_py).

## Definitions

| Item | Definition |
|---|---|
| Robot frame | Fixed to the robot's base: origin where the base stands, +X the way the robot faces, +Y to its left, +Z up. |
| Grid (`Grid::STANDARD`) | x from 0.0 to 1.0 m every 0.02 m, y from -0.30 to +0.30 m every 0.05 m: 51 rows of 13 lanes. |
| Grasp frame | Origin at the grasp point, +Z along the gripper's approach (out of the gripper), +Y along the axis its jaws close on. |
| Grasp orientations (`GraspOrientation::all`) | The approach straight down (-Z) or straight forward (+X), each at 8 rolls about it, every 45°. |
| Reachable (`Reach`) | Some arm brings its grasp point within `REACH_TOLERANCE` (0.01 m) of the target with its approach axis within `GRASP_ANGLE_TOLERANCE` (0.05 rad) of a grasp direction; the roll the gripper ends at about that direction is not checked. A solver of full poses tries each grasp orientation. |
| Surface target | `ABOVE_SURFACE` (0.04 m) above a point of the surface. |
| Perception camera (`perception_camera`) | The robot's one camera that gives depth and that no arm carries. None: no view check. Several: an error naming them. |
| Visible (`View`) | Inside the perception camera's field of view, and where its depth stream reads a depth: its sample of the point, the optical z or the straight-line distance as its depth model says (`DepthModel`), lies inside its depth range (`Camera::view_of`). Each side adds its own test of what hides a point; only a simulation says `View::HiddenBy`. |
| Workable | Reachable and visible; reachable alone for a robot without a perception camera. |
| Room to work | The workable points cover at least `MIN_WORKABLE_AREA` (0.03 m²). |

## What it gives

| | |
|---|---|
| `fit(points, limits)` | the `Fit` of a surface from its measured `GridPoint`s: workable with its area and the largest workable rectangle, or why not |
| `largest_workable_rectangle` | the largest axis-aligned rectangle of workable grid points; of rectangles as large, the one nearest the robot's midline, then the nearest |
| `reach_bounds`, `view_bounds` | the rectangles bounding the points the arms reach and the camera sees |
| `nearest_workable(points, target)` | the workable point nearest a target, where to move an object to |
| `Camera::view_of` | the field-of-view and depth test of a point, for a camera in the optical convention (+X right, +Y down, +Z along the view) with OpenCV intrinsics |
| `Intrinsics::from_vertical_fov` | the pinhole model of a rendered camera from its vertical field of view and image size |
| `workable(reach, view)` | whether a point or an object is workable: reached, and seen or not asked |
| `messages::{point_message, surface_message, robot_frame_placement, answer_message, unchecked_view_message}` | the one-line text of each verdict, of how many checked points, objects or surfaces are workable, and of why no view is checked |

## The answer of a robot from its design

The `design` module gives the `workspace:v1` answer of a robot from its
design. The robot gives what is its own: whether an arm reaches a target
(`Reach`), and how the view of a point is checked (`ViewCheck`). The module
gives the rest, so that every robot answers alike.

| | |
|---|---|
| `SurfaceHeight::from_wire`, `Positions::from_wire` | the parsing of a describe_workspace and a check_positions request: a finite surface height within `MAX_COORDINATE` (1000 m) of the robot's base point, to the millimetre, and at least one point of three finite coordinates, each within `MAX_COORDINATE` of the robot's base point; the bound itself is within it, and applies to the values as the request gives them; a refusal reads as the contract says (`SurfaceHeightError`, `PositionsError`) |
| `SurfaceHeight::grid_targets` | the targets an arm reaches for on a surface: `ABOVE_SURFACE` above each point of `Grid::STANDARD`, in the grid's order |
| `SurfaceReach`, `PositionsReach` | the reach of each target of a surface or of a request, in order: measured now by the robot's solver (`measure`), or measured elsewhere and given one per target (`from_grid_order`, `from_request_order`; else `ReachCountError`) |
| `ReachMemo` | the reach of the surfaces measured last, one per height: past `REACH_MEMO_HEIGHTS` (32) heights it drops the height it stored first, also when a caller asked that height again after it was stored |
| `ViewCheck` | how the view of a point is checked: not at all, as the robot has no perception camera or no camera geometry is linked for it, or through its perception camera placed where the design fixes it |
| `describe_surface(surface, view_check)` | the `SurfaceAnswer` of a surface: whether it is workable, its workable area, the largest workable rectangle, the bounds of the reach and of the view, and the message, which adds where to put objects on a workable surface and why no view is checked |
| `check_positions(positions, view_check)` | the `PositionsAnswer` of a request: the `PointAnswer` of each point (its reach, its view and its message), and the message of how many are workable and of why no view is checked |
