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
let placements = expansion.placements(&holder_of_surface)?;
let world = expansion.world(&placed)?;

let sweep = Sweep::new(SweepDocument::parse(sweep_text)?, &workflow, &patterns)?;

let record = SetupRecord::parse(&setup_record_text)?;
let live = World::from_setup(manifest, catalogue, &record, &stack_robots)?;

let judges = Judge::parse_list(&judges_text)?;
let verdicts = Verdicts::parse(&verdicts_text)?;
```

A `World` (`world/v1`) names its catalogue release and its scene, the robots
with the join each takes on the stack and where each stands, the scene objects
it moves or removes, the objects it spawns, the lighting, the materials, the
daylight and the named locations. The daylight sets the settings of the
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
`same_setup_as` compares two worlds whatever their manifests say.

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
the join of a robot. In an object pose, `{ on: <surface label, { object,
landmark } or { one_of } of them>, reachable_by: <robot> }` draws a pose on a
surface where the robot reaches and sees it, and `{ in: <region> }` a pose in
a named region; `{ in: <region> }` also draws the position of a stand. An
entry of `objects` with `{ item_kind, count }` adds `<name>_1` up to
`<name>_<count>`, each with an asset of the item kind; the count is a number
from 0 to 1000, `{ range }` or `{ one_of }`. A pattern also has
`work_cell_parameters`, each a `control: { object, id }` or a `distance: {
surface, axis }` from the base of each robot of `per_robot` (or the one robot
with a stand), with its unit, and `min_spacing_m`, the least distance between
sampled objects. Each value that a sampler can draw is checked as the
concrete entry checks it. `WorldDocument::resolve_pattern` reads any world
document, and `WorldDocument::resolve` refuses a pattern.
`WorldPattern::to_json5` writes a pattern with its bases folded in as a
document that extends no base. `WorkCellParameters` and `Measurements` are
the JSON5 texts that `measure` of simulation_world takes and gives: the
parameters, and one value per parameter, or one per robot of its
`per_robot`; `Measurements::check` refuses measurements of another shape.

An `Expansion` of a pattern and a `Seed` draws each sampler that the seed
alone draws: `one_of`, `range`, and the count and the asset of each object of
an item kind. Each sampler draws from its own `Stream`, which the seed and the
path of the sampler in the concrete world give (xoshiro256** from a sha256),
so the same pattern and seed always give the same values, and an added entry
changes no other value. The simulation draws each `PlacementSite`, a pose
`on` a surface or `in` a region or a stand, in a model of the drawn world, in
the order of `Expansion::placements`: after what each depends on (a stand at
a surface after the pose of the object of that surface, a pose on a surface
after its object, a pose with `reachable_by` after the stand of its robot),
then the draws without `reachable_by`, the stands and the draws with
`reachable_by`. A cycle is refused, naming its placements.
`Expansion::world` gives the concrete world, named `<pattern>_seed_<seed>`.

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
simulation from the record and the robots of the stack (`StackRobot`), and
`StartSetup::world` makes the start world from the setup the simulation
started with and where each robot first joined. `Judge::parse_list` reads a
JSON5 list of judges, `true` or one of the checks, and `Verdicts` holds a
`holds` and a `why` for each.

A `ClaimHolder` names the call of an applier of worlds (`world_applier:v1`
and `sweep_runner:v1`) that holds the setup claim of a simulation: the
apply, the run or the sweep (`ClaimCall`), the label of its caller, the
sweep, and the run of the workflow director that runs now, as "the run of
'Tidy the table (c2)' (run r3)" or "the sweep of 'nightly' (sweep s2, run
r14)". `ApplyFailureKind` names the failure kinds of an apply and of the
first step of a run, `seed_refused` for the seed of a world pattern that
gives no world among them.
