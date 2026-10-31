# self_collision_model

Runtime self-collision detection for a bimanual robot. Given a pair of joint
configurations, it reports the minimum distance between the two arms, between an
arm and the torso, or between an arm and itself, with the gradient of that
distance with respect to the joints. The caller holds the proximity band and the
law that gates commanded motion on the reported distance.

The library has no hardware or messaging dependencies. The model is built once
from a URDF and its collision meshes (there is no intermediate artifact that can
go stale), then queried every control tick at roughly 0.1 ms per query. The
geometry, the fit of each body's convex pieces and the distance between them, is
[`collision_geometry`](../collision_geometry)'s, which builds its exact convex
hulls with Qhull, a C library, so a build needs libclang. The library contains no robot
names: the caller supplies the URDF, the mesh directory, and the two chain base
links. Any bimanual URDF whose arms are 7-DOF SRS chains (the `srs_model`
contract) is supported.

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

On the OpenArm fixture, the closest pair at the rest pose is each arm's base
link against its own forearm (`link0` vs `link3`), about 24 mm apart. That pair
is kinematically blocked from real contact, so the caller excludes it (see
below); with it gone, the closest checked pair at rest is the forearm against
the torso, about 34 mm apart. A 30 mm `d_safe` therefore runs the rest pose at
full speed while still throttling the genuine close approaches the arms reach
elsewhere. The two numbers are the caller's to tune for their robot and
controller.

## Quick start

```rust
use self_collision_model::{SelfCollisionModel, PairSpec};

// The caller's hard floor, 10 mm. The model does not hold it: the model reports
// distance, the caller decides what to do with it.
let d_stop = 0.01;

let mut model = SelfCollisionModel::builder_from_file(&urdf_path, &meshes_dir, &left_base, &right_base)?
    // Pairs the caller knows can never collide, dropped from checking (see below).
    .exclude(&[PairSpec::new("openarm_left_link0", "openarm_left_link3")])
    // Clip regions for a concave body, each its [min, max] corners: each region's
    // slice of the body gets its own fit, and the build checks that the pieces
    // jointly cover it. OpenArm's are `openarm_description`'s.
    .regions(version.torso_body(), version.torso_regions())
    .build()?;

// Watchdog: evaluate the live joint states of both arms.
let p = model.min_distance(&q_left, &q_right)?;
if p.distance <= d_stop {
    // Hold. p.link_a and p.link_b name the offending pair; p.on_a and p.on_b are
    // the closest points in the world frame.
}

// Command vetting: the clearance a step would land at, and the gradient of the
// clearance with respect to each arm's joints for a velocity-barrier caller.
let d_next = model.min_distance(&q_left_next, &q_right)?.distance;
let g = model.distance_gradient(&q_left, &q_right)?;
// g.grad_left[j] is d(distance)/d(q_left[j]); separating motion has a positive gradient.
```

## The geometry

Each body is wrapped at construction in convex pieces that `collision_geometry`
fits to its collision elements: one polytope per `<collision>` element of the link
and of the links fixed to it, or one per clip region for a concave body (the torso,
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
running minimum. On the fixture this culls about 88% of pairs before any GJK
runs, and a release-mode test asserts a full query stays under 1 ms.

## Construction: from URDF to pieces

```text
URDF collision elements
  mesh, box, cylinder, sphere   one surface per element           (collision_geometry::urdf_collision, stl)
  supporting-plane fit          one polytope per element          (collision_geometry::fit)
  clip regions (supplied)       one fit per region's slice        (collision_geometry::fit, assemble)
  fixed children                their pieces join the parent body (assemble)
  gripper fingers               own bodies, placed per query      (assemble)
  fixed bodies                  root-frame pieces
```

- A clip region is an axis-aligned box in the body's fit frame, its `[min, max]`
  corners, with infinite bounds for open sides. Adjacent regions should overlap by
  a few millimeters.
- A gripper finger, a collision-bearing link on a movable joint off a chain
  link, is fit as its own body and placed at the live opening that
  `set_gripper_openings` sets. The meshes orient the two travel extremes: the
  extreme with the larger between-finger separation is open.
- Every collision-bearing link is accounted for. The moving-link names come from
  walking each chain, fixed children fold into their parents, and every
  remaining collision link must be world-fixed or construction fails loudly.

## Checked pairs

Which pairs are checked is structural and independent of the geometry. Two
world-fixed bodies are skipped because their distance never changes. Pairs within
two moving joints of each other, whether same-side or the torso against a chain's
first links, are skipped as joint-yoked: the members of such a cluster orbit each
other through their whole range, so their distance swings with every legitimate
motion while real contact between them is blocked by the link in between.
Cross-arm pairs are always checked.

Some pairs the structural rules keep can still never collide over the robot's
actual range (a base link and the upper arm a few joints down, for instance):
checking them only ever throttles spuriously. `Builder::exclude` drops such
pairs. The model trusts the assertion rather than re-deriving it, so
getting the list right is the caller's responsibility, and excluding a pair that
can in fact collide silently removes that protection. Only the names are checked
(they must be real bodies). The dropped pairs are reported by `excluded_pairs`
for the consumer to log. The list is a small, deliberate, reviewed set, computed
from a one-off reachability analysis outside this build.

There is no per-pair rebasing: because the hulls are tight, `min_distance`
reports the raw signed distance, and the caller's band thresholds apply directly.

## What a governor builds on

The model gives a governor three reads, and the governor owns the law:

- `min_distance` is the signed distance over the checked pairs, with the pair
  and its witness points. Non-finite joint values are rejected, so a caller
  fails safe.
- `distance_gradient` is the gradient of that distance with respect to each
  arm's joints and each gripper's opening, computed analytically from the
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

## What the caller supplies

| Supplied by the caller | Why it cannot be derived |
|---|---|
| URDF path | the robot description itself |
| collision mesh directory | `package://` URIs are not filesystem paths |
| left and right base links | which chain is "left" is robot identity, not geometry |
| excluded pairs (optional) | pairs the caller asserts can never collide, dropped from checking; trusted, not re-derived |

The proximity band is the governor's, outside the model (see "The proximity
band"). Everything else is derived at construction: the hulls decomposed from the meshes,
the body set, and the checked pairs from the structural rules. Construction fails
loudly on anything it cannot account for (an unreadable mesh, an unmodeled
collision link, an excluded pair naming a body that does not exist), so nothing
is left silently unchecked.

## Calling from the backbone

The consumer is whichever node sees both arms and sits in their bidirectional
joint stream: it reads each arm's measured `joint_states` (keyed by `arm_id`,
0 = left, 1 = right) and produces the `joint_commands` both arms follow. For
OpenArm that is the backbone. It vendors the description (URDF and collision
meshes) and exposes the paths and base links as parameters.

```rust
use self_collision_model::{SelfCollisionModel, PairSpec};
use srs_model::JointVec;

// Bringup, once: hull decomposition and pair derivation.
// The governor's hard floor; the governor holds the band.
let d_stop = 0.01;

// On OpenArm, each arm's base shoulder support and its upper arm (link3) come
// within ~12 mm at folded-shoulder poses but cannot touch over the whole joint
// range, so checking the pair only ever throttles spuriously. Drop it on each
// arm. (A sweep of the joints between them confirmed no contact; that analysis
// lives outside this build, which trusts the assertion.)
let exclude = [
    PairSpec::new("openarm_left_link0", "openarm_left_link3"),
    PairSpec::new("openarm_right_link0", "openarm_right_link3"),
];
let mut model = SelfCollisionModel::builder_from_file(
    &params.urdf_path,            // the same description the arm nodes load
    &params.collision_meshes_dir,
    &params.left_base_link,       // for example "openarm_left_link0"
    &params.right_base_link,
)?
.exclude(&exclude)
.build()?;
for (a, b) in model.excluded_pairs() {
    // info!("collision: not checking {a} vs {b}");  visible at startup
}

// Latest measured joints of each arm, by arm_id, updated from joint_states.
// Queries take &mut self (forward kinematics in place), so one task owns the
// model; it is Send.
let mut q: [JointVec; 2] = [[0.0; 7]; 2];

// On every joint_states message, keep the arm it addresses:
q[msg.arm_id as usize] = msg.positions;

// Watchdog on the measured state, the backstop. min_distance borrows the model,
// so copy out what must outlive the call.
let here = model.min_distance(&q[0], &q[1])?;
if here.distance <= d_stop {
    // Hold both arms; here.link_a / here.link_b name the pair, witnesses in root frame.
}

// Streaming to both arms, per command tick: the producer has a target pose `cmd`
// for each arm. The gradient over both arms at once is what the governor's
// barrier projects the step against, so the closest pair anywhere paces the
// whole motion.
let g = model.distance_gradient(&q[0], &q[1])?;
// g.grad_left, g.grad_right: d(distance)/dq for each arm's joints.
```

The governor slows only motion that would close the gap, to a stop at `d_stop`,
so an arm backing out of a tight spot is never throttled. The band, the law, the
hold and abort wiring, and the joint_states / joint_commands plumbing live with
the backbone, in `openarm_backbone`'s `governor` module.

## Visualization

```sh
cargo run --release --example visualize -- \
    --urdf tests/fixtures/openarm_v10.urdf --meshes ../openarm_description/assets/meshes \
    --left-base openarm_left_link0 --right-base openarm_right_link0 \
    --left 0,0,1.2,0.4,0,0,0 --right 0,0,-1.2,0.4,0,0,0 \
    --wireframes -o scene.html
```

This writes an interactive HTML scene you open in a browser. The solids
are the collision pieces, the geometry the distance query measures against. The pieces are colored by side, and the closest pair is highlighted
with its GJK or EPA witness segment and the signed distance. With `--wireframes`,
the decimated source meshes draw underneath, so the gap between a mesh and its
pieces is the conservative margin, visible directly for judging the fit.
`--d-stop` and `--d-safe` set the HUD thresholds.

## Layout

```text
src/
  assemble.rs        construction-time fit of all bodies
  pairs.rs           pair specs (explicit lists for tests and tools)
  model.rs           SelfCollisionModel queries, pair derivation, exclusions
  error.rs           BuildError, CollisionError
examples/
  visualize.rs       the HTML scene
tests/
  dual_arm.rs                     two-arm scenarios: converge, fold, separate, with budgets
  distance_oracle.rs              the distance query against support-function bounds
  ros_reference_self_distance.rs  each checked distance against MoveIt's FCL
  fixtures/
    openarm_v10.urdf              the OpenArm v1.0 URDF of the tests, srs_model-style
    openarm.rs                    the generation and the meshes of openarm_description
```

The tests read the collision meshes, the URDFs and the torso regions of
`openarm_description`, a dev-dependency. The model places world-fixed bodies once, in
the world frame, so it models robots whose mount does not move.

## Testing

`cargo test` covers pair derivation and exclusion, the broadphase against a brute
force over every checked pair, the distance gradient against finite differences,
the fixture integration scenarios (arms converging monotonically into collision,
folding inward, separating sweeps holding the floor, and non-finite rejection), the
distance query against independent support-function bounds, and each checked
distance against MoveIt's FCL. `cargo test --release` additionally asserts the
per-query time budget. The fit and the distance are tested in `collision_geometry`.

`tests/ros_reference_self_distance.rs` is the evidence that the pieces of each body
hold its link's collision meshes: at every configuration of the self distance
scenarios of the [ROS reference harness](../ros_reference), OpenArm v2 with both
grippers posed, the distance of each checked pair, the fingers' pairs included, is no
larger than the distance that MoveIt's FCL measures between the meshes where they lie
apart, and no larger than zero where they touch. It reads the harness's committed
reference outputs, so it runs with no ROS on the machine.
