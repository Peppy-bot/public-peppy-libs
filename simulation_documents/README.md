# simulation_documents

The documents of the simulation side: worlds and world patterns, sweeps, the
setup record a simulation keeps, and the judges that check ground truth in a
live world.

```rust
use simulation_documents::{Judge, Seed, SetupRecord, Sweep, SweepDocument, Verdicts, World, WorldDocument};

let world = World::parse(text)?;
let text = world.to_json5()?;
let digest = world.document()?.digest();

let pattern = WorldDocument::parse(pattern_text)?.resolve_pattern(&bases)?;
let expansion = pattern.expansion(Seed(7), &assets_of_kind)?;
let placements = expansion.placements()?;
let world = expansion.world(&placed)?;

let sweep = Sweep::new(SweepDocument::parse(sweep_text)?, &workflow, &patterns)?;

let record = SetupRecord::parse(&setup_record_text)?;
let live = World::from_setup(manifest, catalogue, &record, &stack_robots)?;

let judges = Judge::parse_list(&judges_text)?;
let verdicts = Verdicts::parse(&verdicts_text)?;
```

A `World` (`world/v1`) names its catalogue release and its scene, the robots
with the join each takes on the stack, where each stands and where its limbs
stand, the scene objects it moves or removes, the objects it spawns, the lighting, the materials, the
daylight, its regions and the named locations. A robot stands at a side of a
work surface of an object, `{ object, landmark, side, offset }` (`side` one of
`"+x"`, `"-x"`, `"+y"`, `"-y"` of the frame of the landmark's body; without
it, the simulation takes the side), at a named pose of the scene, `{ at,
offset }`, or at `{ position, yaw }`; `offset` is optional. A location is a
landmark of an object, `{ object, landmark }`, which names a work surface by
the landmark it stands on, a named pose, `{ at }`, or a region, `{ in }`. A
region is a box in the world frame, `{ centre, size, yaw }`, metres and
radians, with `yaw` 0 when left out. A location `{ in: <region> }`, a sampler
`{ in: <region> }` and a judge name a region of `regions`. The limbs of a robot, `{ arms: { <arm>: [joint
positions] }, grippers: { <gripper>: opening } }`, name each limb as the
robot names it (limb_state): an arm with its joint positions, radians or
metres, in its joint order, and a gripper with its opening, a fraction of
full jaw travel from 0 (closed) to 1. They name at least one limb. A robot
without limbs stands at home, its grippers closed. The daylight sets the settings of the
scene's site in place of the scene's own: the local date
(`YYYY-MM-DD`), the hour (`HH:MM` or `HH:MM:SS`, written `HH:MM:SS`), the
clock rate and the weather (`{ cloud_cover, rain_mm_h }`). `World::parse`
reads a concrete world, one that extends no base. `WorldDocument::parse`
reads any world document, and `resolve` folds in its bases: a base gives each
field the document leaves out, an entry replaces the base's entry of its key,
and a daylight replaces the base's daylight whole. `{ removed: true }` drops
the base's entry or daylight, except in `scene_objects`, where it marks the
scene object removed and `{ kept: true }` drops the base's entry. A
`{ removed: true }` outside `scene_objects`, or a `{ kept: true }`, whose key
the bases leave no entry under, after their own marks, is refused, as is a
`{ removed: true }` daylight over bases that leave none. A refusal that comes of a base names
each base on the way to it, and a chain of more than 128 bases is refused.
`same_setup_as` compares two worlds whatever their manifests say, with the
limbs of each robot alike rather than equal: each joint within `JOINT_SLACK`
and each opening within `OPENING_SLACK` of the other's, since a measured
joint moves by a hair from one read to the next.

The world carries the root key `simulation_schema` and takes the root key
rule, the manifest and the digest from `documents_core`. Two joins whose
features differ in order are one join, and write one text.

Names are lowercase letters, digits and `_`. A robot name follows peppy's
copy names: 1 to 63 letters, digits, `_` and `-`, and `self` is reserved. No
robot name followed by `_` begins another, and an object never shares a name
with a scene object, one a base keeps or removes among them; both rules hold
in each document and at each base folded in.

A world pattern (`WorldPattern`) is a world with samplers in place of some
values. `{ one_of: [v, ...] }` draws one of its values and `{ range: [min,
max] }` a number from `min` to `max`, in place of any value of an entry but
the join of a robot. In an object pose, `{ on: <{ object, landmark } or {
one_of } of them>, reachable_by: <robot> }` draws a pose on a landmark of an
object that faces up, such as a work surface, where the robot reaches and sees
it, and `{ in: <region> }` a pose in
a region of the world; `{ in: <region> }` also draws the position of a stand. An
entry of `objects` with `{ item_kind, count }` adds `<name>_1` up to
`<name>_<count>`, each with an asset of the item kind; the count is a number
from 0 to 1000, `{ range }` or `{ one_of }`. A pattern also has
`work_cell_parameters`, each a `control: { object, id }` or a `distance: {
object, landmark, axis }` from the base of each robot of `per_robot` (or the
one robot with a stand) to a landmark of an object, with its unit, and
`min_spacing_m`, the least distance between
sampled objects. Each value that a sampler can draw is checked as the
concrete entry checks it. `WorldDocument::resolve_pattern` reads any world
document, and `WorldDocument::resolve` refuses a pattern.
`WorldPattern::to_json5` writes a pattern with its bases folded in as a
document that extends no base. `WorkCellParameters` and `Measurements` are
the JSON5 texts that `measure` of simulation_world takes and gives: the
parameters, and one value per parameter, or one per robot of its
`per_robot`; `Measurements::check` refuses measurements of another shape.

The pose of an entry of `objects` may be `{ furnish: <rule>, stations:
[<station>, ...] }`, `stations` optional: a piece that the simulation lays
out in the room together with every other such piece, in one call of its
solver. The rule is an object with one key:

- `{ against_wall: { on_wall } }`: the back of the piece, the side opposite
  its front, against the wall whose id `on_wall` gives, or against any wall
  when `on_wall` is left out;
- `{ in_corner: {} }`: the back against one wall and one side against
  another wall that meets it;
- `{ free: { near: [x, y] } }`: walkway room on every side, nearest the
  point `near` when it is given;
- `{ beside: { object, gap_m } }`: against the same wall as `object`, side
  by side with it at a gap of `gap_m` metres, at least 0;
- `{ facing: { object, distance_m } }`: front to front with `object`,
  `distance_m` metres between the two fronts, above 0.

The `object` of `beside` and `facing` is another object of the pattern
whose pose is `furnish`, `beside` names a piece whose rule is
`against_wall` or `in_corner`, and the rules name each other in no cycle. A
station is the room that a robot needs to work a work surface of the piece
from one of its sides: `{ landmark, side, robot }` for a robot of the
pattern, or `{ landmark, side, depth_m, width_m }` for a room of that size,
`depth_m` deep out of the edge of the side and `width_m` wide along it, each
above 0. No two stations of a piece work one side of one landmark. The robot
of a station stands at it: its stand is `{ object: <the piece>, landmark,
side }` of the station, without an offset, one station names it, and a
robot that stands at a piece is the robot of a station of that piece. A
piece with a station stands still: its `physics` is `static` or left out,
and the concrete world gives it `static`. The `scale` of a piece is a number
or `{ min, max }`, each above 0 and `min` at most `max`, the range in which
the solver picks the scale; a range whose ends differ is for a piece with
the station of a robot, since the solver picks the scale at which the robots
of its stations work its surfaces. The scale of a piece takes no sampler. An
entry of an item kind and a scene object take no `furnish` pose, and an
object whose pose is not `furnish` takes no scale `{ min, max }`.

A pattern lists its fixed stations in `stations` at its root, each `{ object, landmark, side, depth_m, width_m }` (`FixedStation`): the room at the side of a work surface of an object of `objects` with a fixed pose, or of a scene object, `depth_m` deep out of the edge of the side and `width_m` wide along it, each above 0, such as the room of a robot at the belt of a conveyor line that stands at a fixed pose. The furnish layout keeps each fixed station free: the solver takes it as a station of no piece, so each piece and each station of the layout stays clear of its room, and the base of each station of the layout stands as far from its centre as from the base of another station. No two fixed stations work one side of one landmark of one object, and a fixed station works no object whose pose is `furnish`, no entry of an item kind, no object or scene object whose pose holds a sampler, and no scene object that the pattern removes. The simulation checks a station on a scene object against the drawn scene. `WorldPattern::stations` gives them, and a concrete world takes none, as it takes no `min_spacing_m`.

An `Expansion` of a pattern and a `Seed` draws each sampler that the seed
alone draws: `one_of`, `range`, and the count and the asset of each object of
an item kind. Each sampler draws from its own `Stream`, which the seed and the
path of the sampler in the concrete world give (xoshiro256** from a sha256),
so the same pattern and seed always give the same values, and an added entry
changes no other value. The simulation draws each `PlacementSite`, the
furnish layout, a pose `on` a surface or `in` a region or a stand, in a model
of the drawn world, in the order of `Expansion::placements`. The furnish
layout comes first of all, in a pattern with a furnish pose: one placement at
the path `furnish`, `PlacementKind::Furnish`, with a `FurnishPiece` for each
piece in the order of the names (its asset, physics, mass and controls after
the draws of the seed, its scale range, its rule and its stations). The
other placements come after what each depends on (a stand at a work surface
after the pose of its object, a pose on a work surface after its object, a
pose with `reachable_by` after the stand of its robot, and the furnish
layout gives the pose of each piece), then the draws without
`reachable_by`, the stands and the draws with `reachable_by`. So the stand of
the robot of a station, a pose on a piece and a stand in a region come after
the furnish layout. A cycle is refused, naming its placements. The
simulation seeds its solver with `expansion.stream(&path).next_u64()`, the
first value of the stream of the path of the furnish layout, and places the
layout as one value: `{ "<object>": { position: [x, y, z], yaw, scale }, ...
}`, with the place of each piece, its scale within its range. Before each
draw, the simulation builds its model from `Placements::world_so_far`, the
world of the placements drawn so far: it leaves out each piece, and the
stand of each robot at a piece, until the furnish layout has its value.
`Expansion::world` gives the concrete world, named `<pattern>_seed_<seed>`,
each piece at the pose `{ position, yaw }` and the scale of its place, and
refuses a furnish layout without a place for each piece, with a place of an
object that is no piece, or with a scale outside the range of its piece. An
expansion draws the world at once and takes no steps. The furnish poses of a
pattern draw together: an added furnish pose can move the other pieces and
what stands on them, and changes no other value.
`WorldPattern::candidate_worlds` gives the worlds that check each value that
a sampler can draw: one with each sampler at its first value, then one for
each other value of each sampler, each placement at the origin and each
piece at the least scale of its range; a check of a pattern checks each of
them.

A `SweepDocument` (`sweep/v1`) runs one workflow revision in many worlds: the
launcher of its stack, the robot of each role, each world pattern with its
seeds and its expected ending (`completed`, `degraded`, `failed` or
`{ refused: <assumption> }`), the faults it injects (`{ push: { object, force,
duration_s } }` or `{ remove_robot }` at `{ step_starts }` or `{ step_ends }`
of a step `<role>.<id>`), its judges (`success`, `minimum`, one per assumption
and the `unsafe` judges), its `watch` and `incidents` as in `run/v1`, and the
thresholds of sign-off. `Sweep::new` checks it against its workflow and its
world patterns, and names each misfit.

A `SetupRecord` is the last value of each setup call a simulation took, as
JSON5 text without a root key. `World::from_setup` makes the world of a live
simulation from the record and the robots of the stack (`StackRobot`), each
with a body with the limbs that it measures, and `StartSetup::world` makes the
start world from the setup the simulation started with and where each robot
first joined, at home. `Judge::parse_list` reads a
JSON5 list of judges, `true` or one of the checks, and `Verdicts` holds a
`holds` and a `why` for each. The check `{ clear_ways: { walkway_m } }`
holds when each doorway of the scene is clear over its width and 1.0 m into
the room, no object but a wall enters the zone of a window, and a walkway
`walkway_m` wide, above 0, joins the entry of the free floor to the box,
seen from above, of each robot that stands, by the rules of the furnish
layout: robots are no obstacle.

A `ClaimHolder` names the call of an applier of worlds (`world_applier:v1`
and `sweep_runner:v1`) that holds the setup claim of a simulation: the
apply, the run or the sweep (`ClaimCall`), the label of its caller, the
sweep, and the run of the workflow director that runs now, as "the run of
'Tidy the table (c2)' (run r3)" or "the sweep of 'nightly' (sweep s2, run
r14)". `ApplyFailureKind` names the failure kinds of an apply and of the
first step of a run, `seed_refused` for the seed of a world pattern that
gives no world among them.
