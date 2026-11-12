# robot_description

Each robot model of Peppy as two files that every node builds from: its URDF,
which holds its links, joints, limits, inertias and collision geometry, and its
SRDF, which holds its groups, its end effectors, its named states and the pairs
of links whose collisions are disabled. The crate reads the two together and
bundles the models of `models/` by their model id, the id robot_identity
carries.

## The models

Each directory of `models/` is one model, named by its model id, and holds
`<id>.urdf`, `<id>.srdf` and the collision meshes the URDF names under
`meshes/`:

| Model id | Robot |
|---|---|
| `openarm_v1` | OpenArm v1.0, two seven-joint arms with a parallel gripper each |
| `openarm_v2` | OpenArm v2.0, two seven-joint arms with a two-finger revolute gripper each, two wrist cameras and a chest camera |
| `so101` | SO-101, one five-joint arm with a one-jaw gripper and a wrist camera |

The OpenArm URDFs are Enactic's (`openarm_description`), and their SRDFs start
from Enactic's `openarm_bimanual_moveit_config`, both Apache-2.0. The SO-101
URDF is TheRobotStudio's `so101_new_calib.urdf` (Apache-2.0), with the convex
hull of each part's mesh as its collision mesh, which
`tools/so101_collision_hulls.py` writes. Each file says what it adds to its
upstream or changes in it, such as the optical frame of each camera. The
OpenArm torso collides as eight pieces, each a collision element of the torso
link: the slices of the stand's mesh that collision_geometry's example
`openarm_torso_pieces` writes into both OpenArm models.

The SRDF of every model gives:

- each arm as a group of one chain from its base link to its grasp link, with
  an end effector whose parent link is the grasp link, and each gripper as the
  group of its finger joints. The group names are the names limb_motion
  addresses the limbs by: `left_arm`, `right_arm`, `left_gripper`,
  `right_gripper` on an OpenArm, `arm` and `gripper` on an SO-101.
- the states `ready` and `home` of each arm, `park` where the arm has one, and
  `closed` and `open` of each gripper ([`states`]).
- the disabled pairs, each with its reason: `Adjacent` for two links on one
  joint, `Never` for two links the joint limits keep apart, `Default` for two
  that touch with every joint at zero, `Always` for two that touch wherever the
  robot stands.

A camera the robot carries is a link `<camera>_optical_frame` of the URDF, on
a fixed joint, in the optical frame of REP-103: +z along the view, +x to the
right of the picture and +y down it. A camera that measures depth carries a
depth sensor in the form of Gazebo's URDF extension, on its optical frame: the
`<near>` and `<far>` of its `<clip>` are the nearest and the farthest depths it
measures, meters along its view ([`DepthRange`]):

```xml
<gazebo reference="chest_optical_frame">
  <sensor name="chest" type="depth">
    <camera><clip><near>0.1</near><far>15</far></clip></camera>
  </sensor>
</gazebo>
```

## Reading a description

```rust
let model = robot_description::model("openarm_v2")?;
let description = model.description();
for arm in description.arms() {
    let ready = description.state(&arm.name, robot_description::states::READY);
    println!("{}: {:?} from {} to {}", arm.name, ready, arm.base_link, arm.grasp_link);
}
```

[`RobotDescription::parse`] reads any URDF and SRDF, such as the ones the
attach goal of simulation_robot carries. The reader is strict: an element or
an attribute of the SRDF or of a `<gazebo>` element of the URDF that it does
not read, and a name the URDF lacks, is a refusal ([`DescriptionError`]) that
names it. The URDF's links and joints are held to
one tree by the rules of chain_kinematics, which poses every chain of it.
[`position_limits`] gives a joint's position limits, the URDF's, unbounded for
a continuous joint, the limits each [`Arm`] carries. [`Model::digest`] is the
SHA-256 of a model's URDF, SRDF and meshes, which robot_identity answers beside
the files, and [`Model::directory`] the directory of `models/` the build
bundled the model from, for a reader on the building machine that takes the
files from disk.

The `meshes` feature embeds the collision meshes: [`Model::meshes`] gives them
and [`Model::write_meshes_to`] writes them where a builder of the URDF's
collision geometry reads them, by the file name the URDF gives each
([`mesh_name`]).

Code compiled for the shapes of a robot's arms runs at a description's with
[`on_shape`]: an [`OnShape`] visitor runs at `K` arms of `N` joints each, `D`
joints in all, and a description of no arm, of arms of two joint counts or of a
shape no code is compiled for is refused ([`UnsupportedShape`]). A node compiled
for its own arms holds the table of them, [`NodeArms`]: each arm's name and its
gripper's name, the names its pairing slots carry, in place order. It names an
arm by its [`ArmId`] and gives the bundled model a launcher names as a
[`Driven`]: the model, its description, and each arm's joint limits and states,
one value per joint. A model whose arms are not these is refused
([`UndrivenModel`]), naming both and the bundled models whose arms are these. A
node that holds something of its own for each model it drives, such as the
friction constants of its arms, lists them by model id in a [`DrivenModels`]
table, which gives a listed model with what the node holds for it and refuses
any other, naming the listed ones. [`ArmId`] names an arm by its place among
the description's arms, and [`PerArm`] holds a value for each, in place.
[`write_meshes`] writes the meshes a robot_identity answer carries, refusing a
name that is not one file's.

`robot_description_py` gives the descriptions from Python.

## Adding a robot

1. Put its files in `models/<id>/`: `<id>.urdf`, `<id>.srdf` and the collision
   meshes the URDF names under `meshes/`. A description holds its meshes by
   file name, so two collision meshes of one file name from different paths
   are refused.
2. The URDF is one tree: its links and its joints are each named once, each
   joint is revolute, continuous, prismatic or fixed and names two of its
   links, and the links hang from one root link, every other link on one
   joint. Each arm's grasp link stands at its grasp point, its +z pointing from
   the grasp point away from the hand, so the origin of every link of the
   gripper with collision geometry lies at -z of it; the motion planner
   approaches an item along that +z.
3. The SRDF gives each arm as a group of one chain from its base link to its
   grasp link, with an end effector on the grasp link; each gripper as the
   group of its finger joints, with its `closed` and `open` states, which put
   each finger at one of its joint's limits, a different one each; each arm's
   `ready` and `home` states; and every pair of links whose collisions it
   disables, with its reason.
4. Build: `build.rs` bundles the model by its id with the digest of its
   files, and the crate's tests hold every bundled model to the rules above,
   naming none of them.
5. The generic suites take it with no edit when its arms are one or two of
   five, six or seven joints each, all of one joint count, the shapes
   [`on_shape`] runs at. self_collision_model's `tests/models.rs` builds its
   self model, which stands clear at home and checks every pair of bodies but
   the disabled ones, and holds each `Never` pair apart over a grid of the
   joints between the pair's two bodies, which walks a pair of at most three
   such joints. The motion planner's model tests build its planning model.
6. The self model takes each link with collision geometry on a moving joint
   off an arm's chain as a finger, which the arm's gripper group names, and
   gives each arm its own base link. The motion planner plans an arm whose
   base link stands fixed to the root and whose gripper moves one or more
   fingers, each one link, closing along an axis of the grasp frame.
7. A robot_initializer whose `model` names the id serves it to the robot's
   nodes; the planner and the self model need nothing more. The robot's
   drivers are nodes of their own: a node compiled for its arms lists them in
   a [`NodeArms`] table, and a node that holds constants of its own for each
   model lists the models in a [`DrivenModels`] table.
8. The simulation engines' needs: see sim_robot_core's README.

## Testing

```sh
cargo test --all-features
cargo test
```

The tests hold every bundled model to the rules above: its arms with their
`ready` and `home` states, each finger's `closed` and `open` positions at its
joint's limits, the origin of each gripper link with collision geometry at -z
of its grasp frame, its digest, its meshes the ones its URDF names, and its
files in its directory; each bundled model drives a node compiled for its own
arms, and a node's table of the models it drives gives each listed model. They
run each shape [`on_shape`] names at its counts, read each disabled pair's
reason back from its name, read each camera's depth range from its depth
sensor, and refuse, by name, every element and attribute the reader does not
read, a `<gazebo>` element that is not one camera's depth sensor, a clip that is
not a range of depths, every name the URDF lacks, a link or a joint given twice,
a joint naming a link the URDF lacks, a joint of more than one position, a link
on two joints, joints in a loop, links on no joint beside the root, two meshes
of one file name, a model of other arms than a node's and a model a node's
table does not list.
