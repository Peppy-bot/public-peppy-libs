# collision_model

Runtime self-collision detection for the bimanual OpenArm: are the two arms
(or an arm and the torso, or an arm and itself) too close, right now, at
these joint configurations?

Pure Rust, no hardware or messaging dependencies. Built once from the URDF,
queried every control tick. Sim and real load identical geometry.

```rust
use collision_model::{DualArmCollisionModel, GovernorBand};

let mut model = DualArmCollisionModel::openarm_v10()?;

// Watchdog: evaluate the live joint states of both arms.
let p = model.min_distance(&q_left, &q_right)?;
if p.distance <= 0.0 {
    // p.link_a / p.link_b name the offending pair,
    // p.on_a / p.on_b are the closest world-frame points.
}

// Command vetting: scale a step by where it would land.
let band = GovernorBand::new(0.01, 0.03)?;   // d_stop, d_safe
let d_now = model.min_distance(&q_left, &q_right)?.distance;
let d_next = model.min_distance(&q_left_next, &q_right)?.distance;
let allowed_fraction = band.scale(d_now, d_next);
```

## How it works

### Capsules, not meshes

Every link is wrapped in one or more capsules (sphere-swept segments) fitted
offline from the URDF's collision meshes. Capsule-capsule distance is closed
form and branch-light, so a full dual-arm query (99 checked pairs, ~390
capsule-capsule distances) measures ~23 us in release on the target class of
hardware. The query budget is asserted under 1 ms in a release-mode test.

Capsules strictly contain their meshes (every mesh vertex is inside, checked
by test), so capsule distance is a lower bound on true mesh distance: the
model can alarm early, never late.

### The fit pipeline (offline)

```
URDF collision entries          assets/openarm_v10.urdf
  mesh + origin + scale   -->   per-link vertex clouds      (urdf_collision, stl)
  PCA axis + shrink scan  -->   one capsule per cloud       (fit_capsule)
  adaptive axial banding  -->   tapered bodies split until  (fit_capsules_adaptive)
                                volume stops improving
  fingers at full travel  -->   baked into the wrist link
  fixed links             -->   world-frame capsules
                          -->   assets/openarm_v10_capsules.json
```

Notes that matter:

- Left and right links reference the same STLs but several left entries are
  mirrored (negative Y scale); the fit reads origin and scale per entry from
  the URDF, so there is no runtime mirroring logic.
- Banding is adaptive: band counts from one to a per-body ceiling are tried
  and a larger count wins only by reducing total capsule volume at least 5%
  (volume is the phantom space a proxy adds, and each extra band pays for
  its own end caps, so uniform shapes stay single). A capsule union is not
  convex, so after banding a face-coverage repair pass samples every mesh
  face and grows the bands that leak until the union contains faces, not
  just vertices. The torso lands on seven bands: a single capsule needs the
  base-plate radius (180 mm) everywhere and swallows the space the arms
  hang in; banded, the column gets its true ~42 mm. The tapered elbow link
  drops from a 91 mm to a 61 mm worst radius, the upper-arm link from
  51 mm to 36 mm; blob-shaped links stay single-capsule.
- Gripper fingers are prismatic children of link7. Each finger is fitted
  over the union of its travel extremes and stored in the wrist's capsule
  list, so worst-case-open is always covered and the runtime needs no
  gripper state.
- The body and the two mount links never move; their capsules are baked in
  world frame.

Regenerate after changing the URDF, the meshes, or the fit:

```sh
cargo run --release --bin fit_capsules     # capsules
cargo run --release --bin classify_pairs   # pair margins (after geometry changes)
```

Containment tests pin the checked-in JSON to the assets (capsules must
contain every mesh vertex and sampled face point), and the pairs section
carries a fingerprint of the capsules it was classified against, so a refit
without reclassification fails at load instead of running stale margins.

### Pair classification and margins

Checking all body pairs is wrong twice over: adjacent links always "collide",
and some pairs sit close by construction (the upper arm hangs centimeters
from the shoulder yoke; the two mount capsules permanently overlap). Pairs
are therefore data in the config, produced by `classify_pairs`:

- Start from the structural set: cross-arm everything (49), each arm against
  both mounts, torso against elbow-out links, and shoulder cluster against
  wrist cluster within each arm (the elbow fold). 99 pairs.
- Sample 30k reachable configurations per arm's own limits (deterministic
  seed). Sampling can prove a pair approaches, never that it cannot, so
  nothing is dropped on distance evidence; never-approaching pairs are only
  reported.
- Pairs closer than `HEADROOM` (40 mm) at the reference poses (home and
  ready, clamped into joint limits) get `margin = baseline - HEADROOM`. The
  reported distance for a pair is `raw - margin`, so a structurally snug
  pair reads `HEADROOM` at rest and goes to zero only when it gets closer
  than its rest baseline. Pairs whose capsules already overlap at reference
  (the torso against the upper arm; the two mount capsules) are flagged by
  the classifier: for them the alarm is baseline-relative, not an absolute
  pre-contact guarantee, because the margin spends part of the conservative
  cushion.

Consequence for tuning: the margined floor caps the rest-pose global
minimum at `HEADROOM`. Any watchdog threshold or governor `d_safe` must stay
below it, or rest poses read as warnings. With `HEADROOM = 0.04`, a band of
`GovernorBand::new(0.01, 0.03)` leaves working space.

### The governor law

`GovernorBand::scale(d_now, d_next)` returns the fraction of a commanded
step to allow and is direction-aware:

- separating (`d_next > d_now`): always 1, even inside the stop band, so a
  tangled state (arms crossed at power-on) is recoverable by an ordinary
  separating move;
- approaching: 1 at or above `d_safe`, 0 at or below `d_stop`, linear in
  between; non-finite input fails safe to 0.

### Who calls this

The `openarm01_backbone` node, which is the one node that depends on both
arms (`left_arm` / `right_arm`): it consumes both arms' joint states for a
watchdog (`min_distance` on every update, hold/abort both arms below
threshold) and vets commands at the routing point (reject a goal landing in
violation; scale streamed setpoints with the governor). See
`collision_avoidance_PLAN.md` at the repo root for the integration design.

## Visualization

```sh
cargo run --release --bin visualize -- \
    --left 0,0,0.9,0.4,0,0,0 --right 0,0,-0.9,0.4,0,0,0 \
    --meshes -o scene.html
```

Writes a self-contained interactive page (three.js from CDN): capsules
colored by side, the closest pair highlighted with its witness segment and
margin-adjusted distance, and with `--meshes` the decimated source meshes as
wireframes for judging fit quality. Omitting `--left` / `--right` renders
the home pose.

## Layout

```
assets/
  openarm_v10.urdf            vendored copy (also the test fixture)
  meshes/*.stl                vendored v1.0 collision meshes
  openarm_v10_capsules.json   generated capsules + classified pairs (checked in)
src/
  geometry.rs    capsule primitive, closed-form distances
  fit.rs         PCA capsule fit, adaptive banding
  stl.rs         binary STL reader
  urdf_collision.rs  URDF collision extraction, fixed poses, finger transforms
  config.rs      JSON schema and validated loading
  pairs.rs       pair specs, OpenArm structural set
  model.rs       DualArmCollisionModel runtime queries
  governor.rs    direction-aware proximity scaling
  bin/fit_capsules.rs, bin/classify_pairs.rs, bin/visualize.rs
tests/
  config_containment.rs  capsules contain their meshes; config pinned to assets
  dual_arm.rs            two-arm scenarios: converge, fold, separate; budgets
```

Assets are vendored copies of the Enactic `openarm_description` v1.0
collision proxies; nothing reads outside the crate at build or run time.

## Testing

`cargo test` covers the primitives analytically (degenerate segments,
penetration, symmetry, isometry invariance), the fit (containment by
construction, banding), the config boundary, and the integration scenarios:
arms converging monotonically into collision, elbows folding inward,
separating sweeps holding the floor, witness consistency, NaN rejection.
`cargo test --release` additionally asserts the per-query time budget.
