# collision_model

Runtime self-collision detection for a bimanual robot: the minimum distance
between the two arms, an arm and the torso, or an arm and itself, evaluated
at a pair of joint configurations, plus a proximity law for scaling commands
near contact.

Pure Rust, no hardware or messaging dependencies. The model is built once
from a URDF and a generated capsule config, then queried every control tick.
The library contains no robot names; the caller supplies the URDF, the two
chain base links, and the config. Any bimanual URDF whose arms are 7-DOF SRS
chains (the `srs_model` contract) is supported.

```rust
use collision_model::{CollisionConfig, DualArmCollisionModel, GovernorBand, MarginPolicy};

// Config and URDF come from the caller; from_file and from_json are both
// provided, the model itself only consumes the parsed values. The margin
// policy names the poses that must read as clear and by how much.
let config = CollisionConfig::from_file(&collision_config_path)?.parse()?;
let policy = MarginPolicy { headroom: 0.04, references: vec![home_pose, ready_pose] };
let mut model =
    DualArmCollisionModel::from_urdf_file(&urdf_path, &left_base, &right_base, &config, &policy)?;

// Watchdog: evaluate the live joint states of both arms.
let p = model.min_distance(&q_left, &q_right)?;
if p.distance <= 0.0 {
    // p.link_a / p.link_b name the offending pair,
    // p.on_a / p.on_b are the closest world-frame points.
}

// Command vetting: scale a step by where it would land.
let band = GovernorBand::new(0.01, 0.03)?; // d_stop, d_safe
let d_now = model.min_distance(&q_left, &q_right)?.distance;
let d_next = model.min_distance(&q_left_next, &q_right)?.distance;
let allowed_fraction = band.scale(d_now, d_next);
```

## Capsules

Every link is wrapped in one or more capsules (sphere-swept segments) fitted
offline from the URDF's collision meshes. Capsule-capsule distance is closed
form and branch-light; a full query over the fixture robot (99 checked
pairs, ~390 capsule-capsule distances) measures ~23 us in release, and a
release-mode test asserts the budget stays under 1 ms.

Capsules strictly contain their meshes, verified by test on every mesh
vertex and on sampled face points, so capsule distance is a lower bound on
true mesh distance: the model alarms early, never late.

## The fit pipeline (offline)

```
URDF collision entries
  mesh + origin + scale   -->   per-link vertex clouds      (urdf_collision, stl)
  PCA axis + shrink scan  -->   one capsule per cloud       (fit_capsule)
  adaptive axial banding  -->   compound bodies split while (fit_capsules_adaptive)
                                it keeps reducing volume
  attached children       -->   baked into the parent link
  fixed bodies            -->   world-frame capsules
                          -->   capsule config JSON
```

- Mirrored mesh references (negative scale components) are read per
  collision entry from the URDF; there is no runtime mirroring logic.
- Banding is adaptive: band counts from one up to a per-body ceiling are
  tried, and a larger count wins only by reducing total capsule volume at
  least 5%. Volume is the phantom space a proxy adds, and each extra band
  pays for its own end caps, so uniform shapes stay single-capsule while
  compound shapes split. A capsule union is not convex, so a face-coverage
  repair pass then samples every mesh face and grows leaking bands until
  the union contains faces, not only vertices.
- A movable child hanging off a chain link (a gripper finger) is fitted
  over the union of its travel extremes and stored in the parent link's
  capsule list. Travel is a joint-space line, so containing both extremes
  contains every intermediate opening: any gripper position is covered
  conservatively and the runtime needs no gripper state.
- Fixed bodies (a torso, the mount links) never move; their capsules are
  baked in world frame.

The fit tool is robot-agnostic. Chains and fixed bodies are command-line
inputs; moving-link names come from walking each chain in the URDF. The
fixture invocation is the worked example:

```sh
cargo run --release --bin fit_capsules -- \
    --urdf tests/fixtures/openarm_v10.urdf --meshes tests/fixtures/meshes \
    --chain openarm_left_link0 --chain openarm_right_link0 \
    --fixed openarm_body_link0 --fixed openarm_left_link0 --fixed openarm_right_link0 \
    --out tests/fixtures/openarm_v10_capsules.json
```

Containment tests pin the checked-in fixture config to the fixture assets.

## Pairs and margins (derived at construction)

The config carries geometry only. Which pairs are checked, and with what
margins, is derived when the model is built, so it can never go stale
against the geometry:

- Structural rule: two world-fixed bodies are skipped (their distance never
  changes), and same-side pairs within two moving joints of each other are
  skipped as joint-yoked: cluster members orbit each other through their
  whole range, so their capsule distance swings with every legitimate
  motion while real contact between them is blocked by the link in
  between. Cross-arm pairs are always checked. For the fixture robot this
  yields the cross-arm grid, the elbow-fold pairs, the torso and the
  mounts against each arm from the upper arm out.
- Margins come from the caller's `MarginPolicy`: reference poses that must
  read as clear (clamped into each arm's own joint limits) and a headroom.
  A pair closer than the headroom at a reference gets
  `margin = baseline - headroom`; reported distance is `raw - margin`, so
  a structurally snug pair reads the headroom at rest and reaches zero
  only when it gets closer than its rest baseline. For pairs whose
  capsules already overlap at reference (the torso against the upper arm)
  the alarm is baseline-relative, not an absolute pre-contact guarantee,
  because the margin spends part of the conservative cushion.
  `pair_margins()` exposes the derived set for inspection.

Tuning consequence: the margined floor caps the rest-pose global minimum at
the headroom, so any watchdog threshold or governor `d_safe` must stay
below it or rest poses read as warnings. With 40 mm headroom,
`GovernorBand::new(0.01, 0.03)` leaves working space.

## The governor law

`GovernorBand::scale(d_now, d_next)` returns the fraction of a commanded
step to allow and is direction-aware:

- separating (`d_next > d_now`): always 1, even inside the stop band, so a
  tangled state (arms crossed at power-on) is recoverable by an ordinary
  separating move;
- approaching: 1 at or above `d_safe`, 0 at or below `d_stop`, linear in
  between; non-finite input fails safe to 0.

The law sees only step endpoints, so steps must stay small against capsule
radii plus margins (true for per-tick control steps), and it is
intentionally discontinuous at `d_next == d_now` inside the band; consumers
tracking a tangential path should rate-limit the commanded step if chatter
matters downstream.

## Integration shape

The consumer is whichever node sees both arms (for OpenArm, the backbone):
it evaluates `min_distance` on every joint-state update as a watchdog,
holds or aborts both arms below threshold, and vets commands at the routing
point by rejecting goals that land in violation and scaling streamed
setpoints with the governor. Everything robot-specific arrives as
parameters and vendored files: the URDF path, the capsule config JSON, and
the two base-link names.

## Visualization

```sh
cargo run --release --bin visualize -- \
    --urdf tests/fixtures/openarm_v10.urdf \
    --config tests/fixtures/openarm_v10_capsules.json \
    --left-base openarm_left_link0 --right-base openarm_right_link0 \
    --reference 0,0,0,0,0,0,0 --reference 0,0,0,0.1,0,0,0 \
    --left 0,0,0.9,0.4,0,0,0 --right 0,0,-0.9,0.4,0,0,0 \
    --meshes tests/fixtures/meshes -o scene.html
```

Writes a self-contained interactive page (three.js from CDN): capsules
colored by side, the closest pair highlighted with its witness segment and
margin-adjusted distance, and with `--meshes <dir>` the decimated source
meshes as wireframes for judging fit quality.

## Layout

```
tests/fixtures/                     OpenArm V1.0 fixture, srs_model-style
  openarm_v10.urdf                  fixture URDF
  meshes/*.stl                      fixture collision meshes
  openarm_v10_capsules.json         generated capsule config
src/
  geometry.rs        capsule primitive, closed-form distances
  fit.rs             PCA capsule fit, adaptive banding, face repair
  stl.rs             binary STL reader
  urdf_collision.rs  URDF collision extraction, fixed poses, child transforms
  config.rs          JSON schema, validated loading
  pairs.rs           pair specs (explicit lists for tests and tools)
  model.rs           DualArmCollisionModel queries, pair and margin derivation
  governor.rs        direction-aware proximity scaling
  bin/               fit_capsules, visualize
tests/
  config_containment.rs  capsules contain their meshes; config pinned to fixtures
  dual_arm.rs            two-arm scenarios: converge, fold, separate; budgets
```

The fixture meshes are vendored copies of the Enactic `openarm_description`
v1.0 collision proxies; nothing reads outside the crate at build or run
time. A robot with a moving mount (a lift axis) needs a model extension:
fixed bodies are currently baked in world frame.

## Testing

`cargo test` covers the primitives analytically (degenerate segments,
penetration, symmetry, isometry invariance), the fit (containment by
construction, banding, face repair), the config boundary, and the fixture
integration scenarios: arms converging monotonically into collision, elbows
folding inward, separating sweeps holding the floor, a governor halt before
contact, witness consistency, and non-finite rejection. `cargo test
--release` additionally asserts the per-query time budget.
