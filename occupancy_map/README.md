# occupancy_map

A voxel map of what a depth sensor sees, in a frame the caller names: log-odds occupancy
over a bounded grid of voxels, each scan's points clearing the voxels their rays cross
and marking the voxels they hit, with the robot and the collision objects cut out, the
voxels a collision object intersects cleared, and the distance from any point to the
nearest occupied voxel, with its gradient. No hardware or messaging dependencies.

- `VoxelMap` holds the voxels in its `Frame`, inserts a `Scan` with the shapes it cuts
  out, answers each voxel's state, its box and the occupied voxels inside a box,
  `clear`s, and clears the voxels a collision object's shape intersects or touches.
- `RobotCutout` is the robot's body to cut out: its chains placed and the body fixed
  around them, grown by `collision_geometry`, where the robot's URDF root stands in the
  map's frame; a `ShapeCutout` is a collision object's shape to cut out.
- `depth_points` reads a z16 depth frame as the points of its valid samples, from its
  stream's pinhole model (`camera_core::pinhole`).
- `DistanceField` holds, for a map as it stands, the distance from each voxel's center
  to the center of the nearest occupied voxel, and gives its gradient capped as a
  propagation field caps it.

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

// Both arms at the frame's joint positions and the body fixed around them, each collision
// element scaled by 1.1 about its center and grown by 3 cm, with the URDF root where it
// stands in the map's frame.
let padding = Padding::new(1.1, 0.03)?;
let (left_grown, right_grown) = (left.mask(padding), right.mask(padding));
let body_grown = body.mask(padding);
let robot = RobotCutout::new(
    &world,
    root_in_world,
    vec![left_grown.at(&left_q), right_grown.at(&right_q), body_grown.placed()],
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

A cut-out shape is any `Cutout`. A `RobotCutout` is the robot's body: a `PlacedMask`
from `collision_geometry` for each of its chains, the chain's links at the frame's joint
positions, and for the body fixed around them (`BodyGeometry`), each grown by a padding,
every point within the offset of each collision element scaled about its center
(collision_geometry's README, The robot filter), in the frame of the URDF root, and the
root's pose in the map's frame.

A `ShapeCutout` is a collision object's shape (`collision_geometry::shapes`) placed in
the map's frame and grown by a padding by the same rule: a point inside it is the sensor
seeing the object.

## Collision objects

`clear_intersecting` clears every voxel a collision object's shape touches: whose box it
intersects or comes within 1e-9 m of, ten times the gap at which GJK counts two bodies as
touching, since GJK measures a face that touches a voxel's face as a gap of rounding. Each
takes the lowest log-odds, as a clip point's voxel does, so the map holds nothing where
the object stands. `occupy_intersecting` takes the same voxels to the highest log-odds,
so a distance field of the map holds the object, as MoveIt's CHOMP holds each object of
its world in its field. A primitive is measured against each voxel whose box meets its
bounds, faces included, by GJK. A mesh is measured by its surface: a voxel near one of its
triangles' bounds is touched when the triangle touches it, and every other voxel lies
wholly inside the mesh or wholly outside it, as does each region of them joined by their
faces: the first and the last voxel of a region decide it where the test answers them
alike, and each voxel decides its own where it answers them otherwise, as a wrong inside
test of one of them makes it. `tests/map.rs` checks a mesh's voxels against each voxel
measured on its own, on drawn meshes standing anywhere in the map and across its bounds.

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

`DistanceField::gradient` gives what MoveIt 2.15.2's `DistanceField::getDistanceGradient`
gives CHOMP from its propagation field, whose distances stop at its propagation distance:
every distance capped at the caller's cap rounded up to whole voxels, as MoveIt's field
holds distances up to `ceil(cap / resolution)` voxels, the capped distance at the center
of a point's voxel, and the gradient there by central differences, along each axis the
capped distance at the voxel's neighbor ahead less that at its neighbor behind, over twice
the voxel size.
Outside the map, and on its outermost voxels, which lack a neighbor on one side, it gives
none, where MoveIt reports the point out of bounds.

## Agreement with MoveIt

A voxel's key, a ray's walk and a log-odds update follow OctoMap 1.10's arithmetic, its
mix of single and double precision included, and a scan goes in by the rules of MoveIt's
point cloud updater. `tests/ros_reference_occupancy.rs` is the evidence against MoveIt:
after the clouds of every occupancy scenario of the
[ROS reference harness](../ros_reference), read from a turned sensor and a level one with
a sphere cut out and no padding, the map holds exactly the voxels of MoveIt's occupancy
map, each at MoveIt's log-odds bit for bit. It reads the harness's committed reference
outputs, so it runs with no ROS on the machine. They hold the bits of MoveIt's amd64
build, and the map gives them on every architecture. Against the outputs that
`ROS_REFERENCE_OUTPUTS` names, those of another image, the test counts the voxels that
differ, held by one map alone or at other log-odds, and fails when more than 1% of the
reference's voxels do: MoveIt's arm64 build fuses multiply-adds in its point cloud
callback (31 of them in `cloudMsgCallback`). Its map of `occupancy_room` equals ours in
all 398 voxels, and that of a drawn set of 60 points of each drawn case differs from
ours in 6 of 7187 voxels (0.08%).

The scenario cuts out a sphere and no chain, and the test cuts it out as a ball of its
own (`tests/common/ball.rs`): a point nearer its center than its radius lies inside, the
strict test MoveIt's robot filter makes of a body's bounding sphere, which for a sphere
is the sphere. So the test runs neither `RobotCutout` nor collision_geometry's robot
filter. Every point of a scenario lies more than 1 µm from the sphere's surface, which
the test checks, so the rounding of the frame a point is judged in, the sensor's for
MoveIt and the map's for the map, decides no point. A `RobotCutout` grows each collision
element by collision_geometry's robot filter, and MoveIt's robot filter grows a box, a
cylinder and a mesh by sharp planes, so the two disagree on some points near such a
body: that crate's parity test counts them.

A scan with no shape cut out still clears its far points to `max_range_m`: MoveIt's
updater marks them beyond range only while its shape mask holds a body.

## Timing

`cargo test --release --test insert_time -- --ignored --nocapture` prints, on the
machine that runs it, the time one 640x360 depth frame of a table scene in front of
OpenArm v2 takes to deproject, and to insert at 2 cm voxels with both arms cut out and
with nothing cut out, and the time the map's distance field takes after it. The chest
camera sees both forearms, and the test prints how many of the frame's points the arms'
mask holds.
