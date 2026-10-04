# workspace_core

Where a robot can work: the definitions, the limits and the judgement that a
robot's own answer (the OpenArm backbone's `workspace:v1`, from its design) and a
simulation's answer (Waldo's `scene_workspace:v1`, from a simulated world) share,
so that both give the same kind of answer, with the same limits and in the same
words. Pure Rust on plain arrays (`[f64; 3]` points, row-major `[f64; 9]`
rotations): no math library, no robot description, no world.

## Definitions

| Item | Definition |
|---|---|
| Robot frame | Fixed to the robot's base: origin where the base stands, +X the way the robot faces, +Y to its left, +Z up. |
| Grid (`Grid::STANDARD`) | x from 0.0 to 1.0 m every 0.02 m, y from -0.30 to +0.30 m every 0.05 m: 51 rows of 13 lanes. |
| Grasp frame | Origin at the grasp point, +Z along the gripper's approach (out of the gripper), +Y along the axis its jaws close on. |
| Grasp orientations (`GraspOrientation::all`) | The approach straight down (-Z) or straight forward (+X), each at 8 rolls about it, every 45°. |
| Reachable (`Reach`) | Some arm brings its grasp point within `REACH_TOLERANCE` (0.01 m) of the target in one of the grasp orientations, its approach axis within `GRASP_ANGLE_TOLERANCE` (0.05 rad). |
| Surface target | `ABOVE_SURFACE` (0.04 m) above a point of the surface. |
| Perception camera (`perception_camera`) | The robot's one camera that gives depth and that no arm carries. None: no view check. Several: an error naming them. |
| Visible (`View`) | Inside the perception camera's field of view and depth range (`Camera::view_of`). Each side adds its own test of what hides a point; only a simulation says `View::HiddenBy`. |
| Workable | Reachable and visible; reachable alone for a robot without a perception camera. |
| Room to work | The workable points cover at least `MIN_WORKABLE_AREA` (0.03 m²). |

## What it gives

| | |
|---|---|
| `fit(points, limits)` | the `Fit` of a surface from its measured `GridPoint`s: workable with its area and the largest workable rectangle, or why not |
| `largest_workable_rectangle` | the largest axis-aligned rectangle of workable grid points; of rectangles as large, the one nearest the robot's midline, then the nearest |
| `reach_bounds`, `view_bounds` | the rectangles bounding the points the arms reach and the camera sees |
| `nearest_workable(points, target)` | the workable point nearest a target, where to move an object to |
| `Camera::view_of`, `view_of` | the field-of-view and depth test of a point, for a camera in the optical convention (+X right, +Y down, +Z along the view) with OpenCV intrinsics |
| `Intrinsics::from_vertical_fov` | the pinhole model of a rendered camera from its vertical field of view and image size |
| `workable(reach, view)` | whether a point or an object is workable: reached, and seen or not asked |
| `messages::{point_message, surface_message, robot_frame_placement, workable_count_message}` | the one-line text of each verdict, and of how many checked points or objects are workable |
