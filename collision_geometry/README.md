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
  whether a point lies inside a link, or inside the chain, grown as MoveIt's robot
  filter grows it.
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

// MoveIt's robot filter: each piece scaled by 1.0 and padded by 3 cm.
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

`Mask` holds every collision element grown by a `Padding`, as MoveIt's robot filter
(`point_containment_filter::ShapeMask` over the bodies of geometric_shapes 2.3.4)
holds a body. Placed at a configuration, it answers for each link
(`PlacedGrownLink::contains`) and for the chain (`PlacedMask::contains`, any link).

A mesh grows as MoveIt grows a `bodies::ConvexMesh`, from the piece fitted about it:

- each vertex moves away from the mean of the piece's vertices to `scale` times its
  distance from it, and `offset` meters farther;
- the piece's box along the axes of the mesh element's frame, as MoveIt aligns a
  mesh's box, grows to `scale` times its size about its own center, and `offset`
  meters farther on each side;
- a point lies inside when it lies in the grown box and behind every face plane, each
  moved out through the grown first vertex of its face. Both tests allow 1 nm, the
  slack MoveIt allows the faces.

A face moves out through one grown vertex: by `scale - 1` times its distance from the
center, and by at most `offset` more. Past a sharp edge, where two moved faces meet,
the grown piece reaches farther than its faces moved, as MoveIt's does.

A primitive grows as MoveIt grows its body, about the element's origin, with no slack:
a box's half extents, a cylinder's radius and half length, and a sphere's radius each
become `scale` times their own, plus `offset`. Its piece, the polytope about the
primitive, stays its surface for distances.

## Building

Qhull is C: the `qhull-sys` crate vendors Qhull 2020.2 (`qhull_r 8.1-alpha3`, 2020.2.r
of 2023/01/02) and builds it from source, and bindgen generates its bindings at build
time. A build therefore needs libclang (`libclang-dev` on Debian and Ubuntu); a node
that depends on this crate installs it in its `apptainer.def`.

The C compiler may fuse a multiply and an add of Qhull's into one instruction on
aarch64, which rounds once where x86_64 rounds twice, so an arm64 build fits pieces that
differ from an x86_64 build's in the last bits. On the 51,984 distances of
`self_collision_model`'s merge-base test, release builds of the two architectures differ
by at most 4.53e-12 m, inside every distance tolerance of this crate and of
`self_collision_model`, the least of them 1 nm.

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
  chain_geometry.rs                  properties of the pieces of drawn chains
  common/                            the dressed robots and paddings the properties draw
  fixtures/thin_plate.rs             a cloud whose first hull fails on rounding
  fit_prints_nothing.rs              a fit prints nothing to stdout or stderr
  hull_without_a_free_descriptor.rs  a hull with no file descriptor free
```

## Testing

`cargo test` covers the hull, the fit and GJK analytically (containment of every
input point, containment after the fit, box and sphere distances, EPA depth and
direction, signed continuity through contact, isometry invariance), the clipping, the
surfaces of the primitives, the near-degenerate box clouds the property tests shrank
to, and properties of chains of one to seven joints that
`kinematics_properties::on_a_drawn_robot` draws, each mounted on a continuous joint
the chain does not drive, with a second such joint between two drawn links, and
dressed in meshes (a cube, and a wedge turned about its origin on one link), boxes,
cylinders and spheres: the chain carries exactly the links its joints move, the link
on the skipped joint among them, with one piece per collision element, each piece
holds its element, each bounding sphere holds its piece, a point between an element
and the element grown by the padding lies inside its grown link, and a point farther
outside every element than the padding grows it lies outside every grown link; for a
primitive, a point 1 µm inside the surface of the grown primitive lies inside, and one
past it outside. A wedge whose vertices' mean lies off the center of its box holds the
padded points that the inside property draws, and the box of the wedge turned 45
degrees grows along the axes of its element's frame. `tests/fit_prints_nothing.rs` runs a hull, a cloud with no hull and the fit of
an OpenArm arm in a child process of its own and reads that it prints nothing, and
`tests/hull_without_a_free_descriptor.rs` builds a hull with no file descriptor free.
`HEGEL_TEST_CASES` sets the cases each property runs, 100 by default.
