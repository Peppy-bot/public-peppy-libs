# self_collision_model

Runtime self-collision detection for a robot of serial chains. Given the joint
configuration of every chain, it reports the minimum distance between two chains,
between a chain and the robot's fixed bodies, or between two bodies of one chain,
with the gradient of that distance with respect to the joints. The caller holds the
proximity band and the law that gates commanded motion on the reported distance.

The library has no hardware or messaging dependencies. The model is built once
from a URDF and its collision meshes (there is no intermediate artifact that can
go stale), then queried every control tick at under 0.1 ms per query. The
geometry, the fit of each body's convex pieces and the distance between them, is
[`collision_geometry`](../collision_geometry)'s, which builds its exact convex
hulls with Qhull, a C library, so a build needs libclang.

A robot's model is built from data its description holds: the URDF and its
collision meshes, each chain from its base link to its tip, the fingers its links
carry, its yoked joints, the pairs its joint limits keep apart, and the clip regions
of its concave bodies. A model of `K` chains of `N` joints each is a
`SelfCollisionModel<N, K>`, queried with `Joints<N, K>`, every chain's joints in the
order the builder names the chains. Two robots' models are built here from their
descriptions:

- `openarm(version, meshes_dir)` builds OpenArm's from `openarm_description`: both
  arms, left first, their grippers' fingers, and the torso in its clip regions.
- `so101(meshes_dir)` builds SO-101's from `so101_description`: its arm on its base,
  and the jaw.

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

On the OpenArm fixture, the closest checked pair at the rest pose stands about
33 mm apart (`rest_pose_clearance_is_stable` pins it), so a `d_safe` below that
runs the rest pose at full speed while still throttling the genuine close
approaches the arms reach elsewhere. The two numbers are the caller's to tune for
their robot and controller.

## Quick start

The builder takes every input from the robot's description. This is SO-101's model,
the one `so101` builds: one chain of 5 joints, and the jaw on a joint of its own.

```rust
use chain_kinematics::{ChainSpec, JointSelection};
use self_collision_model::{Finger, PairSpec, SelfCollisionModel};
use so101_description::{
    BASE_LINK, JAW_LINK, NEVER_TOUCHING_PAIRS, TCP_LINK, URDF, YOKED_JOINTS, jaw_shut_rad,
};

// The caller's hard floor, 10 mm. The model reports distance; the caller decides
// what to do with it.
let d_stop = 0.01;

let chains = [ChainSpec {
    base_link: Some(BASE_LINK),
    tip_link: TCP_LINK,
    joints: JointSelection::PathOrder,
}];
let never_touching: Vec<PairSpec> = NEVER_TOUCHING_PAIRS
    .iter()
    .map(|[a, b]| PairSpec::new(*a, *b))
    .collect();
let mut model = SelfCollisionModel::builder(URDF, &meshes_dir, chains, YOKED_JOINTS)
    // Every link on a moving joint off a chain link that has collision geometry is a
    // finger, named with the position of its joint at which it is shut.
    .fingers(&[Finger { link: JAW_LINK, shut: jaw_shut_rad() }])
    // Pairs the robot's joint limits keep apart, dropped from checking (see below).
    .exclude(&never_touching)
    .build()?;

// Watchdog: evaluate the live joint states of every chain.
let p = model.min_distance(&[q_arm])?;
if p.distance <= d_stop {
    // Hold. p.link_a and p.link_b name the offending pair; p.on_a and p.on_b are
    // the closest points in the world frame.
}

// Command vetting: the clearance a step would land at, and the gradient of the
// clearance with respect to each chain's joints for a velocity-barrier caller.
let d_next = model.min_distance(&[q_arm_next])?.distance;
let g = model.distance_gradient(&[q_arm])?;
// g.joints[0][j] is d(distance)/d(q_arm[j]); separating motion has a positive gradient.
```

`openarm` builds OpenArm's model of two chains the same way, a
`SelfCollisionModel<7, 2>` queried with `&[q_left, q_right]`, with the torso's clip
regions from `Builder::regions`.

## The geometry

Each body is wrapped at construction in convex pieces that `collision_geometry`
fits to its collision elements: one polytope per `<collision>` element of the link
and of the links fixed to it, or one per clip region for a concave body (a torso,
`Builder::regions`). Every plane of a fit supports the
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
  clip regions (supplied)       one fit per region's slice        (collision_geometry::fit, assemble)
  fixed children                their pieces join the parent body (assemble)
  fingers                       own bodies, placed per query      (assemble)
  fixed bodies                  root-frame pieces                 (collision_geometry::fit)
```

- A clip region is an axis-aligned box in the body's fit frame, its `[min, max]`
  corners, with infinite bounds for open sides. Adjacent regions should overlap by
  a few millimeters. `Builder::regions` takes one list of regions for each body; a body
  named twice is refused at build, as `collision_geometry`'s `BodyGeometry` refuses it.
- A finger, a collision-bearing link on a movable joint off a chain link, is fit as
  its own body and placed at the live opening of its chain that `set_gripper_openings`
  sets. `Builder::fingers` names each with the position of its joint at which it is
  shut, one limit of the joint; the finger opens toward the other. A finger left
  unnamed, a named link that is no finger, and a shut position off the joint's limits
  are refused at build. `fingers(chain, carried)` takes the parts a chain carries
  (`ChainGeometry::carried` of `collision_geometry`), placed there with their joints
  at 0, and gives each as its `Rider` at the offset where the chain's opening places
  it, so a planner of that chain carries the fingers where they hang. A part the
  model holds as no finger of that chain, on the same link, is refused
  (`FingerError`), naming the part.
- Every collision-bearing link is accounted for. The moving-link names come from
  walking each chain, fixed children fold into their parents, and every
  remaining collision link must be world-fixed or construction fails loudly.

## Checked pairs

Which pairs are checked is structural and independent of the geometry. Two
world-fixed bodies are skipped because their distance never changes. A chain's base
link is a body of that chain, ahead of its first moving joint. Two bodies of one
chain within the robot's yoked joints of each other, and a fixed body against the
links of a chain within its yoked joints of the root, are skipped as joint-yoked: the
members of such a cluster orbit each other through their whole range, so their
distance swings with every legitimate motion while real contact between them is
blocked by the link in between. Each robot's description holds its yoked joints.
Bodies of two chains are always checked, and each chain of a model starts at a base
link of its own.

Some pairs the structural rules keep can still never collide over the robot's
actual range (a base link and the upper arm a few joints down, for instance):
checking them only ever throttles spuriously. `Builder::exclude` drops such
pairs. The model trusts the assertion, so getting the list right is the caller's
responsibility, and excluding a pair that can in fact collide silently removes that
protection. Each excluded pair must be one the structural rules check, else the
build refuses it. The dropped pairs are reported by `excluded_pairs` for the
consumer to log, and `checked_pairs` lists the pairs the model checks. A robot's
list is a small, deliberate, reviewed set its description holds, from a
reachability analysis over its joint ranges (`HardwareVersion::never_touching_pairs`
of `openarm_description`, `NEVER_TOUCHING_PAIRS` of `so101_description`).

There is no per-pair rebasing: because the hulls are tight, `min_distance`
reports the raw signed distance, and the caller's band thresholds apply directly.

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
| URDF | the robot description itself |
| collision mesh directory | `package://` URIs are not filesystem paths |
| each chain's base and tip links | which links bound a chain, and the chains' order, is robot identity, not geometry |
| yoked joints | how far apart along a chain two bodies stay blocked from contact is a property of the robot's links |
| each finger's shut position | which limit of a finger's joint shuts it is the robot's convention |
| excluded pairs (optional) | pairs the caller asserts can never collide, dropped from checking; trusted, not re-derived |
| clip regions (optional) | how to split a concave body is a choice about the shape |

The proximity band is the governor's, outside the model (see "The proximity
band"). Everything else is derived at construction: the hulls decomposed from the meshes,
the body set, and the checked pairs from the structural rules. Construction fails
loudly on anything it cannot account for (an unreadable mesh, an unmodeled
collision link, an unnamed finger, an excluded pair the rules do not check), so
nothing is left silently unchecked.

## Calling from the backbone

The consumer is whichever node sees every chain and sits in their bidirectional
joint stream: it reads each arm's measured `joint_states` (keyed by `arm_id`,
0 = left, 1 = right) and produces the `joint_commands` both arms follow. For
OpenArm that is the backbone. It writes the collision meshes of its generation and
builds the model from `openarm_description`.

```rust
use openarm_description::{ARM_DOF, HardwareVersion};
use self_collision_model::{Joints, openarm};

// Bringup, once: hull decomposition and pair derivation, the never-touching
// pairs of the generation left out.
// The governor's hard floor; the governor holds the band.
let d_stop = 0.01;

let mut model = openarm(HardwareVersion::V2, &meshes_dir)?;
for (a, b) in model.excluded_pairs() {
    // info!("collision: not checking {a} vs {b}");  visible at startup
}

// Latest measured joints of each arm, by arm_id, updated from joint_states.
// Queries take &mut self (forward kinematics in place), so one task owns the
// model; it is Send.
let mut q: Joints<ARM_DOF, 2> = [[0.0; ARM_DOF]; 2];

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
cargo run --release --example visualize -- \
    --urdf tests/fixtures/openarm_v10.urdf --meshes ../openarm_description/assets/meshes \
    --left-base openarm_left_link0 --left-tip openarm_left_link7 \
    --right-base openarm_right_link0 --right-tip openarm_right_link7 \
    --yoked-joints 2 \
    --finger openarm_left_left_finger,0 --finger openarm_left_right_finger,0 \
    --finger openarm_right_left_finger,0 --finger openarm_right_right_finger,0 \
    --left 0,0,1.2,0.4,0,0,0 --right 0,0,-1.2,0.4,0,0,0 \
    --wireframes -o scene.html
```

This writes an interactive HTML scene of a robot of two chains, which you open in a
browser. The solids are the collision pieces, the geometry the distance query
measures against. The pieces are colored by chain, and the closest pair is
highlighted with its GJK or EPA witness segment and the signed distance. With
`--wireframes`, the decimated source meshes draw underneath, so the gap between a
mesh and its pieces is the conservative margin, visible directly for judging the
fit. `--d-stop` and `--d-safe` set the HUD thresholds.

## Layout

```text
src/
  assemble.rs        construction-time fit of all bodies
  pairs.rs           pair specs (explicit lists for tests and tools)
  model.rs           SelfCollisionModel queries, pair derivation, exclusions
  openarm.rs         OpenArm's model from openarm_description
  so101.rs           SO-101's model from so101_description
  error.rs           BuildError, CollisionError, FingerError
examples/
  visualize.rs       the HTML scene
tests/
  dual_arm.rs                     two-arm scenarios: converge, fold, separate, with budgets
  distance_oracle.rs              the distance query against support-function bounds
  openarm.rs                      OpenArm's model from openarm_description
  so101.rs                        SO-101's model from so101_description
  ros_reference_self_distance.rs  each checked distance against MoveIt's FCL
  fixtures/
    openarm_v10.urdf              the OpenArm v1.0 URDF of the tests
    openarm.rs                    the generation, the meshes and the builder of the fixture
```

The tests read the collision meshes, the URDFs and the torso regions of
`openarm_description`, whose `meshes` feature the tests turn on to write its meshes,
and the meshes `so101_description` writes. The model places world-fixed bodies once,
in the world frame, so it models robots whose mount does not move.

## Testing

`cargo test` covers pair derivation and exclusion, the broadphase against a brute
force over every checked pair, a distance read at a bound against the exact one,
the distance gradient against finite differences,
the fixture integration scenarios (arms converging monotonically into collision,
folding inward, separating sweeps holding the floor, and non-finite rejection), the
distance query against independent support-function bounds, SO-101's checked pairs
and its never-touching pairs over a grid of its joint ranges, and each checked
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
