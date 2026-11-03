# collision_geometry

Collision geometry for any robot, read from its URDF and its collision meshes: the
convex pieces of each body, the distance between convex shapes, the pieces of a serial
chain placed at a joint configuration, and the shapes of collision objects. No hardware
or messaging dependencies.

- `fit::fit_pieces` fits the convex pieces of one body to its collision elements: one
  circumscribing polytope per `<collision>` element, which holds its element by
  construction, or one per `ClipRegion` for a concave body, where the fit checks that
  the pieces together hold every vertex and every face of the elements.
- `gjk` measures the distance between two convex shapes, signed through contact.
- `chain::ChainGeometry` holds the pieces of every link that a serial chain's joints
  move, places them at a configuration, each with a bounding sphere, and tests
  whether a point lies inside a link, or inside the chain, grown by a padding: every
  point within the offset of each element scaled about its center.
- `chain::Rider` is a body that a link of a chain carries rigidly: a part of the robot,
  such as a gripper finger at the gripper's opening, or a shape of a collision object
  attached to the link. It holds its name, the part's link name or the object's id, the
  segment of the link that carries it, its offset in that link's frame, and its body in
  its own frame (`RiderBody`): a part's pieces, or the object's shape.
  `ChainGeometry::carried` gives the parts below the chain's links across movable
  joints off its path, each a `CarriedPart` placed with those joints at 0, as riders.
- `chain::BodyGeometry` holds the pieces of the links that fixed joints alone join to
  the URDF root, the robot's body around its chains, and grows them by the same rule.
- `shapes` holds the shapes of collision objects: a box, a sphere, a cylinder, and a
  closed triangle mesh in a bounding volume hierarchy, as FCL holds one. It measures a
  convex body against a shape, a mesh by its triangles, so a body passes through the
  open corner of an L-shaped mesh, and a body wholly inside a mesh lies at a negative
  distance; it tests whether a point lies inside a shape grown by a padding by the
  robot filter's rule, and gives the box along the frame's axes that holds a placed
  shape.
- `urdf_collision`, `stl` and `hull` read the collision elements of a URDF (meshes,
  boxes, cylinders and spheres) and build the exact convex hull of a point cloud.

[`self_collision_model`](../self_collision_model) builds the self-collision model of
OpenArm on this crate.

## Quick start

```rust
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{BodyGeometry, ChainGeometry, Padding};

let left = ChainGeometry::<7>::from_urdf(&urdf, &meshes_dir, &ChainSpec {
    base_link: Some("openarm_left_link0"),
    tip_link: "openarm_left_tcp",
    joints: JointSelection::PathOrder,
})?;

// Every piece of every link, placed in the frame of the URDF root.
for (link, piece) in left.place(&q).pieces() {
    let (center, radius) = piece.bounding_sphere();
}

// The robot filter: every point within 3 cm of each element, scaled by 1.0.
let mask = left.mask(Padding::new(1.0, 0.03)?);
let grown = mask.at(&q);
let on_the_arm = grown.contains(&point);
for link in grown.links() {
    let on_this_link = link.contains(&point);
}

// The body fixed around the arms, its torso decomposed into clip regions.
let body = BodyGeometry::from_urdf(&urdf, &meshes_dir, &[(torso, torso_regions)])?;
let grown_body = body.mask(Padding::new(1.0, 0.03)?);
let on_the_body = grown_body.placed().contains(&point);
```

## The links of a chain

A chain carries the links whose pose its driven joints decide: the child link of each
joint it drives, every link fixed below one, down to the tip and past it, and links
below a movable joint of the path that the chain does not drive, which it holds at 0.
It leaves out a link below a movable joint off the path, such as a gripper finger, and
a link fixed to the root. A link has one piece per `<collision>` element, fitted to
that element in the link's frame.

A `<mesh>` element gives its triangles, scaled by its `scale`. A `<box>` gives its own
faces, a `<cylinder>` the faces of a 64-sided prism whose faces touch it, and a
`<sphere>` the faces of a polytope of 1024 points whose nearest face touches it, so
each piece holds its primitive. Each element is moved by its `<origin>`. A
`<capsule>`, and a primitive whose size is not positive and finite, is refused with an
error naming its link. `ChainGeometry::with_lower_floor` raises a joint's lower limit as
`Chain::with_lower_floor` does, so the chain a planner checks keeps the limits a robot's
controller enforces.

## The body

A body holds the links that fixed joints alone join to the URDF root, the root among
them, each with collision elements: the parts of the robot that stand where they are
whatever its joints do, such as a torso and the mounts of its arms. A link has one piece
per `<collision>` element, in the link's frame, as a chain's link has, or, for a link
the caller names with its clip regions, one piece per region, fitted to the region's
slice of the link's elements in the frame of the URDF root, the frame the regions are
given in, by `fit::fit_fixed_link`, which a self model's fixed bodies are fitted by too.
Regions that name a link outside the body, or one link twice, are refused.

## The fit

Every collision element is wrapped in a convex polytope. A body fits one polytope per
element; for a concave body, whose one polytope bulges far past it, the caller names clip
regions (`ClipRegion::by_body` reads each body's from their corners, and refuses a body
named twice), each region's slice of the body's elements gets its own fit, and the build
checks that the pieces jointly contain the elements.

The fit is conservative, so a distance to it is a lower bound on the distance to the
mesh:

- Every plane of a fit is a supporting plane of the mesh cloud: its offset is the
  cloud's own support value along its normal, so its halfspace holds the whole mesh,
  and so does the intersection of any number of them.
- The fit (`simplify`) refines outward from the cloud's axis-aligned bounding box by
  Kamenev's estimate refinement: it takes the polytope vertex farthest outside the
  mesh's convex hull and cuts it off with the supporting plane that touches the hull
  at that vertex's nearest point, until every vertex lies within 1 mm of the hull.
  Faces land where the surface curves: a flat needs one plane however large it is.
- The polytope's vertices come from the convex hull of the planes' dual points.
- For a body fitted in clip regions, the build checks that every mesh vertex lies
  inside a piece and that every face, subdivided down to 1 mm, lies inside a single
  piece, so no face escapes between the pieces.

The exact convex hulls, of a mesh and of the dual points, are Qhull's (`hull`), the
library MoveIt builds its convex meshes with, with triangulated output (`Qt`). Qhull
checks that every point lies inside its hull. A cloud that fails this on rounding is
hulled again from joggled points (`QJ`, a fixed seed), whose planes then move out to
hold every point; a cloud that fails the joggled pass too has no hull, and the fit
returns Qhull's error, which names the kind of failure and carries Qhull's first error
message. Qhull writes its messages, the check of the points and the report of a failed
pass, to a stream in memory, so a fit prints nothing to stdout or stderr, with one
exception: the `qhull` crate creates a temporary file when a pass fails, and panics
when it cannot create one (no writable temporary directory, or no free file
descriptor), and the process's panic hook, by default, prints that panic to stderr
before the hull catches it. A caught panic counts as the error that Qhull's report
names: singular input, for an initial simplex that is flat or not convex, and a
precision error otherwise, so the cloud is hulled again joggled. GJK and EPA are pure
nalgebra.

## Distance

GJK answers distance from a support function alone (`gjk::Support`), an exact scan
over a piece's vertices. On overlap, EPA grows the simplex GJK ends on to a
tetrahedron of support points, then expands it toward the nearest face of the
Minkowski difference to recover the penetration depth and direction. The signed
distance is therefore continuous through contact: it falls smoothly through zero, so a
caller can tell separating motion from approaching motion even inside an overlap.
`gjk::Vertex` is a single point as a body, for the distance from a point to a piece.

## The robot filter

`Mask` holds every collision element grown by a `Padding`: the element scaled by `scale`
about its center, and every point within `offset` meters of the scaled element. Placed at
a configuration, it answers for each link (`PlacedGrownLink::contains`) and for the chain
(`PlacedMask::contains`, any link). A `BodyMask` grows a body's pieces by the same rule, a
region's piece as a mesh's piece, and stands where the body stands as a `PlacedMask`
(`BodyMask::placed`). `BodyMask::of` grows parts a caller places, such as the parts that
ride a chain where they stand, each piece as a mesh's piece. Each piece is a fitted one,
which carries no rounding, and a rounded piece panics.

- A mesh scales from its piece, about the mean of the piece's vertices. A point inside
  every face of the scaled piece is inside it, and a point beyond the piece's bounding
  sphere or a face moved out by the offset is farther than the offset from it. A point
  between the two is inside when it lies within the offset of the face it lies highest
  above, and otherwise when its distance from the scaled piece, by GJK
  (`gjk::distance` of a `gjk::Vertex`), is at most the offset. A query of a link screens
  every piece of the link, and a query of the chain every piece whose outer ball holds
  the point (below), before either measures a distance.
- A box, a cylinder and a sphere scale as themselves, about the element's origin, and a
  point is inside when its distance from the scaled primitive is at most the offset: a
  grown box and a grown cylinder round their edges. The piece of a primitive, the
  polytope about it, stays its surface for distances.

So a grown element holds every point within the offset of the scaled element, and no
point farther from it, nor a point with a coordinate that is not a finite number. A
mesh's piece holds the convex hull of the mesh and lies within
its fit error (1 mm) of it, so at a scale of 1 a grown mesh holds every point within the
offset of the mesh's convex hull, and no point farther from that hull than the offset
plus the fit error.

A collision object's shape grows by the same rule (`PlacedShape::contains`): a box, a
cylinder and a sphere scale as themselves about their own origin, and a point is inside
when its distance from the scaled primitive is at most the offset, by the same
functions as a link's primitive; a mesh, a closed solid that may be concave, scales
about the center of its bounds, and a point is inside when it lies inside the scaled
mesh or within the offset of one of its triangles.

A placed mask holds the reach of each grown piece in the frame of the URDF root, two
balls about the center of its scaled element: the largest ball that the element holds,
grown by the offset less 1 nm, and the smallest that holds the element, grown by the
offset and 1 nm, a nanometer far beyond the rounding of placing a point in the link's
frame. A mesh's largest ball reaches the face of its scaled piece nearest the center; a
box's reaches its nearest faces, a cylinder's its side or its caps, whichever is nearer,
and a sphere's is the sphere. `PlacedMask::contains` answers as the links' `contains` do
one by one: a point outside the box, square to the root's axes, that holds every outer
ball lies inside no link, a point within an inner ball lies inside its piece's link, and
every other point is screened by the pieces whose outer ball holds it.

MoveIt's robot filter (`point_containment_filter::ShapeMask` over the bodies of
geometric_shapes 2.3.4, `bodies.cpp`) grows a body by sharp planes, and departs from
this growth:

- A box's half extents, and a cylinder's radius and half length, become `scale` times
  their own plus `offset`, so their edges reach past the offset: a box's corner by up
  to `sqrt(3)` times it. A sphere grows alike in both.
- A convex mesh moves each vertex along its line from the mean of the hull's vertices
  onto the plane of each face about the vertex, scaled and moved out by the offset, and
  takes the mean of those points (`computeScaledVerticesFromPlaneProjections`). The
  plane of each face passes through the moved first vertex of the face, and a point is
  inside when it is inside every such plane (`isPointInsidePlanes`) and inside the
  mesh's box, scaled about its own center and grown by the offset on each side. Its
  planes meet in edges that reach past the offset, and a plane that the mean places
  nearer than the offset keeps points within it.

On the probes of the ROS reference harness (see Testing), this filter deletes all 121
probes that lie within 3 cm of the hull of an OpenArm v2 mesh at an offset of 3 cm,
where MoveIt keeps 31 probes within 3 cm of a hull; MoveIt deletes 2 probes of the
primitive arm that lie beyond 3 cm of every element, which this filter keeps.

## Building

Qhull is C: the `qhull-sys` crate vendors Qhull 2020.2 (`qhull_r 8.1-alpha3`, 2020.2.r
of 2023/01/02) and builds it from source, and bindgen generates its bindings at build
time. A build therefore needs libclang (`libclang-dev` on Debian and Ubuntu); a node
that depends on this crate installs it in its `apptainer.def`.

The C compiler may fuse a multiply and an add of Qhull's into one instruction on
aarch64, which rounds once where x86_64 rounds twice, so an arm64 build fits pieces that
differ from an x86_64 build's in the last bits.

Qhull is free software under its own license, `COPYING.txt` in the `qhull/`
directory of the `qhull-sys` crate (also at [qhull.org](http://www.qhull.org/COPYING.txt)).
A binary distributed with this crate in it includes Qhull, so, as that license asks, it
ships Qhull's `COPYING.txt` with it and states that Qhull's source can be obtained from
[qhull.org](http://www.qhull.org).

## Layout

```text
src/
  chain.rs           ChainGeometry, PlacedChain, Padding, Mask, PlacedMask,
                     PlacedGrownLink: a serial chain's pieces and the robot filter;
                     Rider, RiderBody, CarriedPart: a body a link of a chain
                     carries;
                     BodyGeometry, BodyMask: the body fixed to the root
  fit.rs             fit_pieces: one body's pieces, and the containment check
  simplify.rs        the supporting-plane fit to a deviation budget
  clip.rs            ClipRegion and the clipping of a mesh to it
  hull.rs            the exact convex hull, by Qhull
  hull/report.rs     the stream in memory that Qhull writes its messages to
  gjk.rs             GJK distance, EPA penetration, Hull, Placed and Vertex
  shapes.rs          Shape, PlacedShape, PlacedMesh, the closed triangle mesh's hierarchy
                     and inside test
  stl.rs             binary STL reader
  urdf_collision.rs  URDF collision elements and their surfaces, fixed poses, joint
                     transforms
  error.rs           GeometryError, ContainmentFailure
tests/
  chain_geometry.rs                  properties of the pieces of drawn chains, the
                                     robot filter on one link, and the body
  shapes.rs                          box meshes against their boxes, the L's open corner,
                                     paddings, scales, bounds, a mirrored mesh
  common/properties.rs               the dressed robots and paddings the properties draw
  common/growth.rs                   MoveIt's growth of a primitive, which the
                                     properties and the MoveIt comparison read
  fixtures/thin_plate.rs             a cloud whose first hull fails on rounding
  fit_prints_nothing.rs              a fit prints nothing to stdout or stderr
  hull_without_a_free_descriptor.rs  a hull with no file descriptor free
  ros_reference_chain_collision.rs   link frames and the robot filter against MoveIt
```

## Testing

`cargo test` covers the hull, the fit and GJK analytically (containment of every input
point, containment after the fit, box and sphere distances, EPA depth and direction,
signed continuity through contact, isometry invariance), the clipping, the surfaces of
the primitives, the near-degenerate box clouds the property tests shrank to, and
properties of chains of one to seven joints that
`kinematics_properties::on_a_drawn_robot` draws, each mounted on a continuous joint the
chain does not drive, with a second such joint between two drawn links, and dressed in
meshes (a cube, and a wedge turned about its origin on one link), boxes, cylinders and
spheres: the chain carries exactly the links its joints move, the link on the skipped
joint among them, with one piece per collision element, each piece holds its element,
each bounding sphere holds its piece, every point within the offset of a scaled element
lies inside its grown link, no point farther than the offset from every scaled element
of a link lies inside the grown link, and the grown chain answers for a point as its
links do one by one, each screening every piece. The three filter properties draw a
point of the scaled element's surface, at a vertex, on an edge or on a face of a mesh's
piece, on a face, an edge or a corner of a box, and on the side, a cap or a rim of a
cylinder, and step from it along a drawn direction of the surface's normal cone, so the
step is the point's distance from the scaled element; the third steps up to one and a
half offsets, or toward the element's center, and asks for the same point with a drawn
coordinate not a number, or infinite, which lies inside neither the chain nor a link.
Three cases on one link ask the chain and the link alike. A point 1.13 offsets past the
rim of a scaled cylinder lies outside the grown link, one 0.9 offsets past the rim along
the same diagonal inside, and one 1.1 offsets out from the middle of its side outside. A
point 1.21 offsets past a corner of a scaled box lies outside, one 0.9 offsets past the
corner along the same diagonal inside, and one 1.1 offsets out from the middle of its
nearest face outside. Each vertex of a box mesh's scaled piece, moved out by half the
offset along the normal of each face through it, or by 0.9 offsets away from the piece's
center, lies inside, and a point 1.1 offsets out from the middle of its nearest face
outside. At the 100 cases a property runs by default, derandomized as on CI (hegel's
`ci` profile), a filter that grows a cylinder with a sharp rim fails the cylinder case
and the second property; one that grows a box with sharp edges fails the box case, the
second property and the MoveIt test below; one that compares a mesh's distance with zero
fails the mesh case and the MoveIt test; one that holds every point of a mesh within its
faces moved out by the offset fails the second property and the MoveIt test; and screens
that take a point with a coordinate that is not a finite number fail the third property.
A placed mask fails the cases and properties named here and the MoveIt test when its
outer balls are not grown by the offset, or grown by 0.8 of it (the three cases); when
its box holds the inner balls alone (the three cases and the first and third
properties); when the outer ball of a box reaches its largest half extent (the box case
and the third property), that of a cylinder the larger of its radius and half length
(the cylinder case), or that of a mesh 0.9 of its bounding sphere (the mesh case); when
the inner ball of a cylinder reaches the larger of its radius and half length (the
cylinder case), that of a box its largest half extent (the box case and the third
property), or that of a mesh its farthest face (the mesh case); when its inner balls are
grown by 1.2 or twice the offset (the three cases and the third property); and when a
primitive's balls stand about its origin negated (the first and third properties). A
margin of 0 fails the third property alone.

The body's properties draw the dressed robots with the mount hung from a new root by a
fixed joint at a drawn placement. The body holds the mount alone, and answers for each
point that the filter properties draw near a scaled element of the mount, off the grown
element's surface, as the grown chain that a revolute joint at that placement carries the
mount on answers at 0. With the mount split into two clip regions across a drawn axis,
overlapping by 5 mm, the body holds a drawn point inside the hull of a surface of the
mount, off its boundary, and no point beyond the reach of the grown regions. One case
places a box at the root, a cylinder on a turned fixed joint and a cube mesh split into
two regions on a fixed joint above it, and a link a revolute joint moves with a link
fixed below it: the body holds each element and region to 0.9 of the offset and not to
1.1, a region scaled by 1.5 about its center, and no point of the moving links. Regions
for a link the body does not hold, or twice for one link, are refused. A body that takes
the links a movable joint carries, that stands its links or its regions at the root's
frame or its regions at their link's, that grows its elements or its regions by no
offset or its regions by no scale, or that takes regions off the body or twice, fails
these. A cube of half side 0.1 m, placed turned and moved, holds at no padding a point
past its middle by 0.09 m along each axis and not by 0.11 m, and scaled by 2, by 0.19 m
and not 0.21 m; one that does not scale it, or stands the part at the root, fails it. A
rounded cube is refused.
`tests/fit_prints_nothing.rs` runs a hull, a cloud with no hull and the fit of
an OpenArm arm in a child process of its own and reads that it prints nothing, and
`tests/hull_without_a_free_descriptor.rs` builds a hull with no file descriptor free.
For the shapes of collision objects, at drawn poses: a box mesh measures a drawn
sphere as its box does and holds the points its box holds, two box meshes measure each
other as their boxes do, a primitive grown by a padding holds every point within the
offset of it and no point farther, a shape grown by a scale holds a point along each of
its axes up to the scale times its reach and no farther, and a shape's bounds hold it
and reach it on each side; a box mesh inside another lies at a negative distance from
either; an L-shaped mesh leaves its open corner free and holds what lies inside its
bars; a mesh whose mirrored half meets the other with -0.0 is closed; a point with a
coordinate that is not a finite number lies inside no shape; and a shape refuses what
it cannot be, an open mesh among them. `HEGEL_TEST_CASES` sets the cases each property
runs, 100 by default.

`tests/ros_reference_chain_collision.rs` is the evidence for the chain geometry and
the robot filter against MoveIt: at every configuration of the chain collision
scenarios of the [ROS reference harness](../ros_reference), the OpenArm v2 arms dressed
in their meshes and an arm dressed in boxes, cylinders and spheres and a turned wedge,
with probes 10 µm either side of the surface of each grown primitive, the chain
carries the links that MoveIt's chain carries, each at MoveIt's frame within 1e-12.
For each padding, the filter holds to its rule on every probe: a probe within the
offset of a mesh's hull, scaled about its piece's center, or of a scaled primitive lies
inside its grown link, and a probe farther than the offset plus the scaled fit error
from every hull, and than the offset from every scaled primitive, lies outside the
grown chain. The filter answers as MoveIt's robot filter on every probe outside the
band of each body: a probe inside a mesh's hull, or within the offset of a scaled
primitive, both hold, and neither holds a probe that the rule puts outside and that
lies outside MoveIt's grown box of every mesh and beyond the sharp growth of every
primitive. Inside the bands, the test holds the counts of the probes by how the two
answer: on OpenArm v2, 13, 119 and 56 probes at the three paddings (none, 3 cm, and a
scale of 1.1 with 1 cm), where MoveIt deletes none beyond the offset of every scaled
body, and keeps 31 probes at 3 cm and 6 at 1.1 with 1 cm within the offset of a scaled
hull, and 2 more within a fit error past it; on the primitive arm, 2, 2 and 0 probes,
where MoveIt deletes the 2 at 3 cm beyond the offset of every element. A filter that
holds every point of a mesh within its faces moved out by the offset puts a probe of
OpenArm v2 past the offset of every body inside the grown chain, one that grows a box
with sharp edges a probe of the primitive arm, and one that compares a mesh's distance
with zero leaves a probe of OpenArm v2 within the offset of a body outside its grown
link, and the test fails each. The test reads the harness's committed reference
outputs, so it runs with no ROS on the machine.
