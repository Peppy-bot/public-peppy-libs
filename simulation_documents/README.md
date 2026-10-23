# simulation_documents

The documents of the simulation side: worlds, the setup record a simulation
keeps, and the judges that check ground truth in a live world.

```rust
use simulation_documents::{Judge, SetupRecord, Verdicts, World};

let world = World::parse(text)?;
let text = world.to_json5()?;
let digest = world.document()?.digest();

let record = SetupRecord::parse(&setup_record_text)?;
let live = World::from_setup(manifest, catalogue, &record, &stack_robots)?;

let judges = Judge::parse_list(&judges_text)?;
let verdicts = Verdicts::parse(&verdicts_text)?;
```

A `World` (`world/v1`) names its catalogue release and its scene, the robots
with the join each takes on the stack and where each stands, the scene objects
it moves or removes, the objects it spawns, the lighting, the materials and
the named locations. `World::parse` reads a concrete world, one that extends
no base. `WorldDocument::parse` reads any world document, and `resolve` folds
in its bases: a base gives each field the document leaves out and an entry
replaces the base's entry of its key. `{ removed: true }` drops the base's
entry, except in `scene_objects`, where it marks the scene object removed and
`{ kept: true }` drops the base's entry. A `{ removed: true }` outside
`scene_objects`, or a `{ kept: true }`, whose key the bases leave no entry
under, after their own marks, is refused. A refusal that comes of a base names
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

A `SetupRecord` is the last value of each setup call a simulation took, as
JSON5 text without a root key. `World::from_setup` makes the world of a live
simulation from the record and the robots of the stack (`StackRobot`), and
`StartSetup::world` makes the start world from the setup the simulation
started with and where each robot first joined. `Judge::parse_list` reads a
JSON5 list of judges, `true` or one of the checks, and `Verdicts` holds a
`holds` and a `why` for each.
