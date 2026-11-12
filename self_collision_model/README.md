# self_collision_model

Runtime self-collision detection for a robot of serial chains. Given the joint
configuration of every chain, it reports the minimum distance between two chains,
between a chain and the robot's fixed bodies, or between two bodies of one chain,
with the gradient of that distance with respect to the joints. The caller holds the
proximity band and the law that gates commanded motion on the reported distance.

The library has no hardware or messaging dependencies. The model is built once,
at construction, from a robot's description and its collision meshes, then queried
every control tick at under 0.1 ms per query. The
geometry, the fit of each body's convex pieces and the distance between them, is
[`collision_geometry`](../collision_geometry)'s, which builds its exact convex
hulls with Qhull, a C library, so a build needs libclang.

`SelfCollisionModel::from_description` builds a robot's model from its description,
a URDF and an SRDF read together (`robot_description::RobotDescription`), and the
collision meshes the URDF names: a chain per arm of the SRDF, from the arm's base link
to its grasp link over the arm's joints; the fingers of each arm's gripper, each
placed between where the gripper's `closed` and `open` states put it; and every pair
of bodies checked but the pairs whose collisions the SRDF disables. A model of `K`
arms of `N` joints each is a `SelfCollisionModel<N, K>`, queried with `Joints<N, K>`,
every arm's joints in the order of the description's arms. A description of another
count of arms (`BuildError::Arms`), or with an arm of another count of joints
(`BuildError::ArmJoints`), is refused. `robot_description` bundles the description
of each robot model of this repository by its model id: `openarm_v1`, `openarm_v2`
and `so101`.

## The proximity band

The model reports distances. The caller holds the band its governor gates on,
because the geometry alone cannot choose it. The band is two distances:

- `d_stop` is the hard floor. At or below it, commanded motion that would
  approach is fully stopped.
- `d_safe` is the full-speed threshold. At or above it, motion runs unthrottled.

These two numbers carry the safety buffer that the geometry does not know about:
the worst-case closing speed, the watchdog latency, and the tracking drift of
the controller. A fitted piece sits at most 1 mm outside its mesh's convex hull,
so the band, not the geometry, is where the real margin lives. On OpenArm the
`Governor` of `openarm_backbone`'s `governor` module holds the band. It takes
`d_stop` and `d_safe` at `Governor::build`, which refuses a band that is not a
finite `0 < d_stop < d_safe`, and at `Governor::set_band`, which warns of such
a band and keeps the band it holds.

On the OpenArm fixture, OpenArm v1 as `robot_description` bundles it, the closest
checked pair at home stands about 33 mm apart (`home_clearance_is_stable` pins it), so
a `d_safe` below that runs home at full speed while still throttling the genuine close
approaches the arms reach elsewhere. The two numbers are the caller's to tune for
their robot and controller.

## Quick start

The description holds every input. This is SO-101's model: one arm of 5 joints, and
the jaw on a joint of its own.

```rust
use self_collision_model::SelfCollisionModel;

// SO-101 as robot_description bundles it, its collision meshes written where the
// model reads them (robot_description's `meshes` feature).
let so101 = robot_description::model("so101")?;
so101.write_meshes_to(std::path::Path::new(meshes_dir))?;
let mut model = SelfCollisionModel::<5, 1>::from_description(&so101.description(), meshes_dir)?;

// The caller's hard floor, 10 mm. The model reports distance; the caller decides
// what to do with it.
let d_stop = 0.01;

// Watchdog: evaluate the live joint states of every arm.
let p = model.min_distance(&[q_arm])?;
if p.distance <= d_stop {
    // Hold. p.link_a and p.link_b name the offending pair; p.on_a and p.on_b are
    // the closest points in the world frame.
}

// Command vetting: the clearance a step would land at, and the gradient of the
// clearance with respect to each arm's joints for a velocity-barrier caller.
let d_next = model.min_distance(&[q_arm_next])?.distance;
let g = model.distance_gradient(&[q_arm])?;
// g.joints[0][j] is d(distance)/d(q_arm[j]); separating motion has a positive gradient.
```

OpenArm's model, from `openarm_v1` or `openarm_v2`, is built the same way, a
`SelfCollisionModel<7, 2>` queried with `&[q_left, q_right]`.

## The geometry

Each body is wrapped at construction in convex pieces that `collision_geometry`
fits to its collision elements: one polytope per `<collision>` element of the link
and of the links fixed to it. A concave body, such as OpenArm's torso, is several
collision elements in its URDF, one per convex part. Every plane of a fit supports the
mesh, so the pieces contain it and the reported distance is a true lower bound on
the mesh distance, less at most the fit's 1 mm deviation: the model alarms early,
never late. At runtime the only geometry is GJK distance between pieces, with EPA
recovering the penetration depth and direction where two pieces overlap, so the
signed distance is continuous through contact. The fit, its guarantees and the
distance are described in [`collision_geometry`](../collision_geometry).

A bounding-sphere broadphase keeps the per-tick cost down. Each body has a local
bounding sphere; at query time the model places the centers, sorts the candidate
pairs by their lower-bound separation, and stops once that bound exceeds the
running minimum. On the OpenArm fixture this culls about 88% of pairs before any
GJK runs, and a release-mode test asserts a full query stays under 1 ms. A piece pair
the spheres leave in meets a second bound that follows the pieces' shapes, their
separation along the line of their centers (`gjk::separation_along`), and GJK
measures the pair where that bound leaves it in too.

## Construction: from URDF to pieces

```text
URDF collision elements
  mesh, box, cylinder, sphere   one surface per element           (collision_geometry::urdf_collision, stl)
  supporting-plane fit          one polytope per element          (collision_geometry::fit)
  fixed children                their pieces join the parent body (assemble)
  fingers                       own bodies, placed per query      (assemble)
  fixed bodies                  root-frame pieces                 (collision_geometry::fit)
```

- A finger, a collision-bearing link on a movable joint off a chain link, is fit as
  its own body and placed at the live opening of its chain that `set_gripper_openings`
  sets: at an opening of 0 its joint stands where the gripper's `closed` state puts
  it, at 1 where its `open` state does, and in between in proportion. Every such link
  is a finger of its arm's gripper in the SRDF; a link that no gripper names
  (`BuildError::UnnamedFinger`), and a gripper's finger that rides no movable joint off
  a chain link (`BuildError::NoSuchFinger`), are refused at build.
  `fingers(chain, carried)` takes the parts a chain carries
  (`ChainGeometry::carried` of `collision_geometry`), placed there with their joints
  at 0, and gives each as its `Rider` at the offset where the chain's opening places
  it, so a planner of that chain carries the fingers where they hang. A part the
  model holds as no finger of that chain, on the same link, is refused
  (`FingerError`), naming the part.
- Every collision-bearing link is accounted for. The moving-link names come from
  walking each chain, fixed children fold into their parents, and every
  remaining collision link must be world-fixed or construction fails loudly.

## Checked pairs

The description decides which pairs are checked: every pair of bodies but the pairs
whose collisions the SRDF disables (`disable_collisions`), whatever its reason
(`Adjacent`, `Never`, `Default`, `Always` or `User`). A chain's base link is a fixed
body, as is every link fixed to the world. The model trusts the SRDF: disabling a pair
that can in fact collide silently removes that protection, so a robot's list is a
small, deliberate, reviewed set. `checked_pairs` lists the pairs the model checks.

The hulls are tight, so `min_distance` reports the raw signed distance of the
nearest pair, and the caller's band thresholds apply to it directly.

## What a governor builds on

The model gives a governor three reads, and the governor owns the law:

- `min_distance` is the signed distance over the checked pairs, with the pair
  and its witness points. Non-finite joint values are rejected, so a caller
  fails safe.
- `distance_gradient` is the gradient of that distance with respect to each
  chain's joints and each chain's opening, computed analytically from the
  nearest pair's witness points at the cost of one distance query.
- `clearance_step_bound` bounds how much the minimum distance can change along a
  straight joint-space step, so a step whose start clearance exceeds a floor by
  more than the bound cannot cross that floor.

On OpenArm the law is the backbone's `Governor`, a closing-velocity barrier. Each
tick it limits only the component of the commanded step that closes the nearest
gap, by the gradient, and leaves tangential and separating motion at full speed.
The approach speed it permits ramps linearly from full at `d_safe` to zero at
`d_stop`, and an exact floor scan along the step, by `min_distance`, is its
backstop.

The signed distance is what keeps separating motion free inside an overlap,
where a hull is still a hair larger than the real part. Plain GJK reports zero on
overlap and cannot tell separating from approaching, but EPA's signed depth can,
so a separating step out of a torso overlap still reads as separating.

## What a planner builds on

A planner that moves one chain while the others hold where they stand reads
`min_distance_of_chain(chain, ...)`: `min_distance` over the checked pairs one of
whose bodies rides that chain, its fingers included, the distance a motion of that
chain changes. `min_distance_of_chain_alone(chain, ...)` reads those of them no other
chain's bodies ride: the chain against the fixed bodies and against itself. The
planner carries the chain's fingers with `fingers` (Construction, above).

A planner's check asks whether the robot keeps a clearance, and how far under it a
state stands. `min_distance_within(..., within)`, `min_distance_of_chain_within` and
`min_distance_of_chain_alone_within` read each of those distances for such a check:
exact where it lies under `within`, and a lower bound at or above `within` elsewhere.
The scan sets a pair aside as soon as a bound puts it at `within`, so GJK measures the
pairs near the clearance alone.

## What the caller supplies

| Supplied by the caller | Why it cannot be derived |
|---|---|
| robot description (URDF and SRDF) | the robot itself: its links and joints, its arms and their grippers, the gripper states, and the pairs whose collisions are disabled |
| collision mesh directory | `package://` URIs are not filesystem paths |

The proximity band is the governor's, outside the model (see "The proximity
band"). Everything else is derived at construction: the chains from the arms, the
hulls decomposed from the meshes, the body set, and the checked pairs. Construction
fails loudly on anything it cannot account for (an unreadable mesh, an unmodeled
collision link, a finger no gripper names, a description of another count of arms or
joints), so nothing is left silently unchecked.

## Calling from the backbone

The consumer is whichever node sees every chain and sits in their bidirectional
joint stream: it reads each arm's measured `joint_states` (keyed by `arm_id`,
0 = left, 1 = right) and produces the `joint_commands` both arms follow. For
OpenArm that is the backbone.

```rust
use self_collision_model::{Joints, SelfCollisionModel};

// Bringup, once: hull decomposition and pair derivation, from the robot's
// description, the collision meshes written to meshes_dir.
let description = robot_description::model("openarm_v2")?.description();
let mut model = SelfCollisionModel::<7, 2>::from_description(&description, &meshes_dir)?;

// The governor's hard floor; the governor holds the band.
let d_stop = 0.01;

// Latest measured joints of each arm, by arm_id, updated from joint_states.
// Queries take &mut self (forward kinematics in place), so one task owns the
// model; it is Send.
let mut q: Joints<7, 2> = [[0.0; 7]; 2];

// On every joint_states message, keep the arm it addresses:
q[msg.arm_id as usize] = msg.positions;

// Watchdog on the measured state, the backstop. min_distance borrows the model,
// so copy out what must outlive the call.
let here = model.min_distance(&q)?;
if here.distance <= d_stop {
    // Hold both arms; here.link_a / here.link_b name the pair, witnesses in root frame.
}

// Streaming to both arms, per command tick: the producer has a target pose `cmd`
// for each arm. The gradient over both arms at once is what the governor's
// barrier projects the step against, so the closest pair anywhere paces the
// whole motion.
let g = model.distance_gradient(&q)?;
// g.joints[0], g.joints[1]: d(distance)/dq for each arm's joints.
```

The governor slows only motion that would close the gap, to a stop at `d_stop`,
so an arm backing out of a tight spot is never throttled. The band, the law, the
hold and abort wiring, and the joint_states / joint_commands plumbing live with
the backbone, in `openarm_backbone`'s `governor` module.

## Visualization

```sh
cargo run --release --example visualize -- --model openarm_v1 \
    --joints left_arm=0,0,1.2,0.4,0,0,0 --joints right_arm=0,0,-1.2,0.4,0,0,0 \
    --wireframes -o scene.html
```

This writes an interactive HTML scene of a bundled robot model, which you open in a
browser. `--model` names the model by its id, `--joints <arm>=<positions>` poses an
arm, named as its SRDF group (an arm left out stands at its `home` state), and
`--gripper <fraction>` or `--gripper <arm>=<fraction>` opens the grippers. The solids
are the collision pieces, the geometry the distance query measures against. The
pieces are colored by arm, and the closest pair is highlighted with its GJK or EPA
witness segment and the signed distance. With `--wireframes`, the decimated source
meshes draw underneath, so the gap between a mesh and its pieces is the conservative
margin, visible directly for judging the fit. `--d-stop` and `--d-safe` set the HUD
thresholds.

## Layout

```text
src/
  assemble.rs        construction-time fit of all bodies
  model.rs           SelfCollisionModel: construction from a description, checked pairs, queries
  error.rs           BuildError, CollisionError, FingerError
examples/
  visualize.rs       the HTML scene of a bundled model
tests/
  models.rs                       every bundled model: builds, checked pairs, never-touching pairs
  dual_arm.rs                     two-arm scenarios: converge, fold, separate, with budgets
  distance_oracle.rs              the distance query against support-function bounds
  openarm.rs                      OpenArm's fingers, and v2's arms, distances and home
  so101.rs                        SO-101's folded and ready arm, and its jaw
  ros_reference_self_distance.rs  each checked distance against MoveIt's FCL
  fixtures/
    models.rs                     each bundled model by its id, the check run on every model, and OpenArm
```

The tests read the descriptions `robot_description` bundles and the collision meshes
of its models directory. `tests/models.rs` runs its checks on every bundled model, at
the shape of its self model, so a model added to `robot_description` is tested as it
lands. The visualization example writes a model's meshes with
`robot_description`'s `meshes` feature. The model places world-fixed bodies once, in
the world frame, so it models robots whose mount does not move.

## Testing

`cargo test` covers each bundled model's build, its checked pairs from its SRDF's
disabled pairs, the
broadphase against a brute force over every checked pair, a distance read at a bound
against the exact one, the distance gradient against finite differences, the fixture
integration scenarios (arms converging monotonically into collision, folding inward,
separating sweeps holding the floor, and non-finite rejection), the distance query
against independent support-function bounds, the pairs each bundled model's SRDF
disables as never touching over a grid of the joints between them, and each checked
distance against MoveIt's FCL. `cargo test --release` additionally asserts the
per-query time budget. The fit and the distance are tested in `collision_geometry`.

`tests/ros_reference_self_distance.rs` is the evidence that the pieces of each body
hold its link's collision meshes: at every configuration of the self distance
scenarios of the [ROS reference harness](../ros_reference), OpenArm v2 with both
grippers posed and SO-101 with its jaw posed, the distance of each checked pair, the
fingers' pairs included, is no larger than the distance that MoveIt's FCL measures
between the meshes where they lie apart, and no larger than zero where they touch.
It reads the harness's committed reference outputs, so it runs with no ROS on the
machine.
