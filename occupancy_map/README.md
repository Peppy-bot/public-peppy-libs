# occupancy_map

A voxel map of what a depth sensor sees, in a frame the caller names: log-odds occupancy
over a bounded grid of voxels, each scan's points clearing the voxels their rays cross
and marking the voxels they hit, with the robot cut out, and the distance from any point
to the nearest occupied voxel. No hardware or messaging dependencies.

- `VoxelMap` holds the voxels in its `Frame`, inserts a `Scan` with the shapes it cuts
  out, answers each voxel's state and the occupied voxels inside a box, and `clear`s.
- `RobotCutout` is the robot's body to cut out: its chains placed and grown by
  `collision_geometry`, where the robot's URDF root stands in the map's frame.
- `depth_points` reads a z16 depth frame as the points of its valid samples, from its
  stream's pinhole model (`camera_core::pinhole`).
- `DistanceField` holds, for a map as it stands, the distance from each voxel's center
  to the center of the nearest occupied voxel.

## Quick start

```rust
use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use collision_geometry::chain::Padding;
use control_core::positive_finite::PositiveFinite;
use occupancy_map::{Bounds, DistanceField, Frame, RobotCutout, Scan, VoxelMap, depth_points};

let world = Frame::parse("world")?;
let mut map = VoxelMap::new(
    world.clone(),
    PositiveFinite::parse(0.02)?,
    Bounds { min: [-1.5, -1.5, -0.5], max: [1.5, 1.5, 2.5] },
)?;

// The depth stream's model, from its get_depth_intrinsics answer, reads the frame.
let depth = Pinhole::parse(&Intrinsics {
    width: intrinsics.width,
    height: intrinsics.height,
    fx: intrinsics.fx,
    fy: intrinsics.fy,
    cx: intrinsics.cx,
    cy: intrinsics.cy,
    distortion_model: &intrinsics.distortion_model,
    distortion: &intrinsics.distortion,
})?;
let depth_model = DepthModel::parse(&intrinsics.depth_model)?;
let points = depth_points(&frame, &depth, depth_unit, depth_model)?;

// The depth stream stands where the color camera's pose in the map's frame, composed
// with get_depth_to_color_extrinsics, the depth frame's pose in the color frame, puts it.
let depth_to_color = Pose::new(
    extrinsics.depth_to_color_position,
    extrinsics.depth_to_color_orientation,
)?;
let sensor = color_in_world.compose(&depth_to_color);

// Both arms at the frame's joint positions, each collision element scaled by 1.1 about
// its center and grown by 3 cm, with the URDF root where it stands in the map's frame.
let padding = Padding::new(1.1, 0.03)?;
let (left_grown, right_grown) = (left.mask(padding), right.mask(padding));
let robot = RobotCutout::new(
    &world,
    root_in_world,
    vec![left_grown.at(&left_q), right_grown.at(&right_q)],
);
let max_range_m = PositiveFinite::parse(3.0)?;
map.insert(&Scan { frame: &world, sensor, points: &points, max_range_m }, &[&robot])?;

let field = DistanceField::of(&map);
let clearance = field.distance([0.4, 0.1, 0.9])?;
let obstacles = map.occupied_in([0.2, -0.2, 0.6], [0.6, 0.2, 1.0]);
```

## Frames

A map keeps its voxels in the frame it is made in. A scan's points are in its sensor's
own frame, and its `sensor` pose places that frame in the scan's `frame`; a cutout names
the frame it stands in. `insert` takes a scan whose frame and cutouts' frames are the
map's, and refuses any other, naming the frame to give instead and changing nothing. A
frame's name is one tf takes: not empty, and not starting with `/`.

## The voxels

A voxel's edge is the map's voxel size, and voxels lie at its multiples: voxel `k` along
an axis spans `[k * size, (k + 1) * size)`, keyed as OctoMap keys a coordinate. Each
voxel holds a log-odds of occupancy in single precision, updated as OctoMap updates it:
a hit adds `logit(0.7)`, a miss adds `logit(0.4)`, and the sum is clamped between
`logit(0.1192)` and `logit(0.971)`. A voxel at or above even odds is occupied, a voxel
below them free. A voxel that no ray has reached is unknown and holds nothing to collide
with.

## A scan

A scan is a sensor's points in its own frame, where the sensor stands, and how far it
reads. It goes in by the rules of MoveIt's point cloud updater
(`PointCloudOctomapUpdater::cloudMsgCallback`, moveit2 2.15.2):

- A point with a coordinate that is not a finite number changes nothing.
- A point beyond `max_range_m` is a miss along its ray up to `max_range_m`, and no hit.
- A point inside a cut-out shape is cut out: its voxel takes the lowest log-odds, and
  its ray still clears.
- Any other point is a hit, and each voxel its ray crosses up to the center of the hit
  voxel is a miss, walked as OctoMap's `computeRayKeys` walks it.
- In one scan each voxel takes one update: a cut-out voxel is free, else a hit voxel is
  a hit, else a crossed voxel is a miss.

A cut-out shape is any `Cutout`. A `RobotCutout` is the robot's body: the `PlacedMask`
of each of its chains from `collision_geometry`, the chain's links at the frame's joint
positions grown by a padding, every point within the offset of each collision element
scaled about its center (collision_geometry's README, The robot filter), in the frame of
the URDF root, and the root's pose in the map's frame.

## The distance field

`DistanceField::of` takes the exact Euclidean distance transform of the grid
(Felzenszwalb and Huttenlocher) between voxel centers. Inside the map a point reads the
distance from its voxel's center to the center of the nearest occupied voxel, within
half a voxel's diagonal of its own distance. Outside the map it reads a lower bound of
its own distance: its distance to `p`, the nearest point of the box of the map's voxel
centers, joined at a right angle to a lower bound of `p`'s distance. Straight out from a
voxel center on the map's face, with every occupied voxel on that face, the bound is the
distance itself. A map with nothing occupied reads infinity everywhere, and a point that
is not a number is refused.

## Timing

`cargo test --release --test insert_time -- --ignored --nocapture` prints, on the
machine that runs it, the time one 640x360 depth frame of a table scene in front of
OpenArm v2 takes to deproject, and to insert at 2 cm voxels with both arms cut out and
with nothing cut out, and the time the map's distance field takes after it. The chest
camera sees both forearms, and the test prints how many of the frame's points the arms'
mask holds.
