# collision_geometry

Collision geometry for any robot, read from its URDF and its collision meshes: the
convex pieces of each body, the distance between convex shapes, and the pieces of a
serial chain placed at a joint configuration. No hardware or messaging dependencies.

- `fit::fit_pieces` fits the convex pieces of one body to its collision elements: one
  circumscribing polytope per `<collision>` element, which holds its element by
  construction, or one per `ClipRegion` for a concave body, where the fit checks that
  the pieces together hold every vertex and every face of the elements.
- `gjk` measures the distance between two convex shapes, signed through contact.
- `chain::ChainGeometry` holds the pieces of every link that a serial chain's joints
  move, places them at a configuration, each with a bounding sphere, and tests
  whether a point lies inside a link, or inside the chain, grown by a padding: every
  point within the offset of each element scaled about its center.
- `urdf_collision`, `stl` and `hull` read the collision elements of a URDF (meshes,
  boxes, cylinders and spheres) and build the exact convex hull of a point cloud.

[`self_collision_model`](../self_collision_model) builds the self-collision model of
OpenArm on this crate.

## Quick start

```rust
use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{ChainGeometry, Padding};

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
error naming its link.

## The fit

Every collision element is wrapped in a convex polytope. A body fits one polytope per
element; for a concave body, whose one polytope bulges far past it, the caller names clip
regions (`ClipRegion::all_from_bounds` reads them from their corners), each region's
slice of the body's elements gets its own fit, and the build checks that the pieces
jointly contain the elements.

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

`Mask` holds every collision element grown by a `Padding`: the element scaled by
`scale` about its center, and every point within `offset` meters of the scaled
element. Placed at a configuration, it answers for each link
(`PlacedGrownLink::contains`) and for the chain (`PlacedMask::contains`, any link).

- A mesh scales from its piece, about the mean of the piece's vertices. A point inside
  every face of the scaled piece is inside it, and a point beyond the piece's bounding
  sphere or a face moved out by the offset is farther than the offset from it. A point
  between the two is inside when it lies within the offset of the face it lies highest
  above, and otherwise when its distance from the scaled piece, by GJK
  (`gjk::distance` of a `gjk::Vertex`), is at most the offset. A query screens every
  piece of the link, or of the chain, before it measures a distance.
- A box, a cylinder and a sphere scale as themselves, about the element's origin, and a
  point is inside when its distance from the scaled primitive is at most the offset: a
  grown box and a grown cylinder round their edges. The piece of a primitive, the
  polytope about it, stays its surface for distances.

So a grown element holds every point within the offset of the scaled element, and no
point farther from it. A mesh's piece holds the convex hull of the mesh and lies within
its fit error (1 mm) of it, so at a scale of 1 a grown mesh holds every point within the
offset of the mesh's convex hull, and no point farther from that hull than the offset
plus the fit error.

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

On a 640x360 depth frame of a table scene in front of OpenArm v2, 230,400 points with
both arms at Ready and a padding of 1.0 and 3 cm, on one core, a query takes 105 ns a
point, 24.1 ms a frame: 378 ns for one of the 33,855 points the arms' mask holds, and
58 ns for one it does not. Placing the mask of both arms takes 0.5 µs.

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
                     PlacedGrownLink: a serial chain's pieces and the robot filter
  fit.rs             fit_pieces: one body's pieces, and the containment check
  simplify.rs        the supporting-plane fit to a deviation budget
  clip.rs            ClipRegion and the clipping of a mesh to it
  hull.rs            the exact convex hull, by Qhull
  hull/report.rs     the stream in memory that Qhull writes its messages to
  gjk.rs             GJK distance, EPA penetration, Hull, Placed and Vertex
  stl.rs             binary STL reader
  urdf_collision.rs  URDF collision elements and their surfaces, fixed poses, joint
                     transforms
  error.rs           GeometryError, ContainmentFailure
tests/
  chain_geometry.rs                  properties of the pieces of drawn chains, and
                                     the robot filter on one link
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
`kinematics_properties::on_a_drawn_robot` draws, each mounted on a continuous joint
the chain does not drive, with a second such joint between two drawn links, and
dressed in meshes (a cube, and a wedge turned about its origin on one link), boxes,
cylinders and spheres: the chain carries exactly the links its joints move, the link
on the skipped joint among them, with one piece per collision element, each piece
holds its element, each bounding sphere holds its piece, every point within the offset
of a scaled element lies inside its grown link, and no point farther than the offset
from every scaled element of a link lies inside the grown link. The two filter
properties draw a point of the scaled element's surface, at a vertex, on an edge or on
a face of a mesh's piece, on a face, an edge or a corner of a box, and on the side, a
cap or a rim of a cylinder, and step from it along a drawn direction of the surface's
normal cone, so the step is the point's distance from the scaled element. Three cases
on one link: a point 1.13 offsets past the rim of a scaled cylinder and one 1.21
offsets past a corner of a scaled box lie outside the grown link, and each vertex of a
box mesh's scaled piece, moved out by half the offset along the normal of each face
through it, lies inside. At the 100 cases a property runs by default, derandomized as
on CI (hegel's `ci` profile), a filter that grows a cylinder with a sharp rim fails the
cylinder case and the second property; one that grows a box with sharp edges fails the
box case, the second property and the MoveIt test below; one that compares a mesh's
distance with zero fails the mesh case and the MoveIt test; and one that holds every
point of a mesh within its faces moved out by the offset fails the second property and
the MoveIt test.
`tests/fit_prints_nothing.rs` runs a hull, a cloud with no hull and the fit of
an OpenArm arm in a child process of its own and reads that it prints nothing, and
`tests/hull_without_a_free_descriptor.rs` builds a hull with no file descriptor free.
`HEGEL_TEST_CASES` sets the cases each property runs, 100 by default.

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
