# sim_robot_core

What a simulation engine node knows about the robots it stands, whatever its physics. The MuJoCo and Isaac Sim nodes of `nodes-hub` both install it in their base image and keep only what is their own: their gains, their model files, their physics and their transport.

| Module | What it holds |
|---|---|
| `registry` | The robots in the scene, by the name each stands under: who may join, who holds which robot, and the pair grace time of each robot: how long it stays without the pairs its model asks for, running from the first time a limb reaches it. A refused join raises `ValueError`, and `StillJoining` when the caller attaches its own robot again while the robot still joins: that one refusal ends once the robot stands, so an engine tells it apart. |
| `pairs` | The four pairing slots an engine declares (`arms`, `grippers`, `rgb_cameras`, `rgbd_cameras`) and what each pair names: its robot, and its limb or its camera. |
| `models` | One entry per robot model, with its checks, and the table that pairs those entries with an engine's own. |
| `cameras` | The camera configuration of a model's entry, the z16 depth wire format, and per-camera frame pacing. |

## The four slots

An engine declares one slot per kind of pair, each holding any number of pairs. Nothing in a slot's name says which robot, limb or camera a pair is, so every pair is read for it:

- a pair's robot is the copy it carries, which is the name the robot attached under. A pair with no copy belongs to a robot launched outside one, and outside a copy there is one robot in the scene for it to belong to;
- a limb pair's limb is the link the pair comes from on the robot's side, so a backbone names its downstream links after its limbs: `left_arm: "simulation_inst/arms"` for an OpenArm, `arm: "simulation_inst/arms"` for an SO-101;
- a camera pair's camera is its relay's name in the copy (`wrist_left`, `wrist`), which is the relay's instance id without the copy's prefix.

A new robot, limb or camera changes no engine's interface.

## Model entries

A robot's entry comes from its description, the URDF and SRDF its attach goal carries, read by [`robot_description_py`](../robot_description_py): each arm group under the name the robot answers to in `limb_motion` and `limb_state` with the joints it moves, each gripper group with where its `closed` and `open` states put each finger, the optical frame of each camera the robot carries with the depths a depth camera measures, and the posture the robot starts in, every arm at its `ready` state and every gripper closed. `src/sim_robot_core/models/<model>.json5` holds what the description does not: how the engines render each camera of the model, by the camera's name, its field of view and its streams, the size of its depth image on a camera that measures depth. The package ships one for `openarm_v2` and one for `so101`; a model whose description carries no camera, such as `openarm_v1`, needs none. An entry that renders a camera the description lacks, or that leaves one of the description's cameras out, is refused at the attach, naming the cameras of both, and a camera of a model this package ships no file for is refused naming the file to create. A camera rendered with a depth stream is one whose description gives it a depth range, the depth sensor of its URDF, and one rendered without is one whose description gives it none; any other is refused at the attach, naming the camera and what to write.

An engine keeps one entry of its own per model, `<model>.json5` in a directory beside its code, with what it alone knows: the file it loads, its gains, its extras. `Models.read(directory)` reads the engine's entries and the entries this package ships for the same models once, as the engine starts, so an entry that does not read, a key no entry has or a bad render stops the engine before a robot attaches. An engine stands the models it has an entry for: a model with no entry is refused naming the ones the engine stands, and none falls back to another's layout. Every refusal of an entry or a model, at the engine's start or at an attach, raises `ModelEntryError` (`errors`), a `ValueError` that names the entry or the model and why.

```python
from robot_description_py import RobotDescription
from sim_robot_core.models import Models

models = Models.read(engine_dir / "models")
known = models.of("so101")      # raises, naming the models this engine stands
entry = known.entry(RobotDescription.parse(goal.urdf, goal.srdf))
entry.arm_names()                # ["arm"]
known.engine                     # the engine's own entry, as written
```

A camera's pose is its optical frame turned half a turn about its own X (`cameras.camera_pose`): the optical frame looks along +Z with +Y down the picture, and the engines' camera frame looks along -Z with +Y up it.

## The match

`ModelEntry.mismatch(held)` compares what a robot holds on the four slots with what its model has, and names both lists when they differ: a limb or a camera the model lacks, or a limb the model has and no pair drives. A camera the model has may go unpaired, since a camera nobody views is not rendered. An engine starts a robot's pair grace time again only while its pairs match, so a robot paired as another model does not stay; a robot no limb reached yet has no pair grace time running (`Registry.note_limbs_reached` starts it, `Robot.pair_grace_ran_out` reads it), its stay is its attach goal's until one does, and `ModelEntry.holds_every_limb(held)` is what a robot's readiness asks.

## Adding a robot

A robot stands in a simulation once its model's description is in [`robot_description`](../robot_description) and each engine has a model of it. Its id is the model's id in `robot_description`, which names every file below:

1. When its description carries a camera, write `src/sim_robot_core/models/<id>.json5` with how each camera renders, under the camera's name in the description, with a depth stream on each camera its URDF gives a depth sensor. A robot with no camera needs no file here.
2. Give the MuJoCo node of `nodes-hub` its model: a robot in Waldo's catalogue, packed and published as a release whose `robots.json` entry names the robot's description, its MJCF, its mount body, the MJCF body of each URDF link the MJCF names otherwise and the gains of its arms; that release pinned in Waldo's `web/public/assets/catalogue.lock.json` and in `sim_base_images/waldo_catalogue.py` (`RELEASE_DIGEST`); and `sim_mujoco/engine/models/<id>.json5`. [sim_mujoco's README](https://github.com/Peppy-bot/nodes-hub/tree/main/sim_mujoco#models) says what each holds.
3. Give the Isaac Sim node its model: a USD bundle of the robot, pinned by key and SHA-256 in `sim_base_images/isaac_assets.env` and staged by `sim_base_images/download_assets.sh`, and `sim_isaac/engine/models/<id>.json5`. [sim_isaac's README](https://github.com/Peppy-bot/nodes-hub/tree/main/sim_isaac#adding-a-model) says what each holds.
4. Move the pins of this package and of `robot_description_py` in `sim_base_images/requirements.mujoco.txt`, `sim_base_images/requirements.isaac.txt` and each node's `tests/pyproject.toml` to the commit that carries the description and step 1, bump `MUJOCO_IMAGE_REV` and `ISAAC_IMAGE_REV` in `sim_base_images/build_base_images.sh`, and run that script, so each base image bakes the new pins.

## Tests

```sh
uv run pytest
```
