# robot_description_py

The Python bindings of [`robot_description`](../robot_description), for the
Python nodes and the simulation engines. They give each bundled robot model by
its model id, and a robot's description read from its URDF and SRDF with the
same reader as the Rust nodes: its arms with their grippers, its named states,
its cameras and its joint limits. A file the reader refuses raises the same
refusal the Rust nodes give.

## Build and install

The package is a PyO3 extension module that maturin builds for the stable ABI
of Python 3.12 (`abi3`): one build loads on every Python from 3.12. An install
compiles the module from the committed `Cargo.lock` exactly (`locked`), so the
host needs a Rust toolchain (`cargo`). The module does not link libpython: the
Python that loads it supplies the Python API. The bundled models, their
collision meshes included, are compiled into the module.

A consumer pins a commit of this repository:

```
robot_description_py @ git+https://github.com/Peppy-bot/public-peppy-libs.git@<commit>#subdirectory=robot_description_py
```

or, with uv:

```toml
[tool.uv.sources]
robot_description_py = { git = "https://github.com/Peppy-bot/public-peppy-libs", subdirectory = "robot_description_py", rev = "<commit>" }
```

## What it gives

| Name | What it is |
|---|---|
| `model(id)` | The bundled `Model` of a model id, as robot_identity gives it. An id of no model raises `ValueError`, naming the models. |
| `models()` | Every bundled `Model`, in the order of their ids. |
| `Model` | A model's `id`, `urdf`, `srdf`, `digest` and `meshes` (`(file name, bytes)` pairs), `write_to(directory)`, which writes `<id>.urdf`, `<id>.srdf` and `meshes/` for a reader that takes files from disk, and `description()`. |
| `RobotDescription.parse(urdf, srdf)` | The description two files make. A refusal raises `ValueError` with the reader's message. |
| `RobotDescription` | Its `urdf` and `srdf`, its `arms`, `state(group, name)` (a position per joint of the group, or `None`), `cameras` and `joint_limits(joint)` (`KeyError` for a joint the URDF lacks). |
| `Arm` | An arm group: `name`, `base_link`, `grasp_link`, `joints` (base to tip) and `gripper`. |
| `Gripper` | A gripper group: `name` and `fingers`. |
| `Finger` | A finger's `joint`, `link`, where the gripper's `closed` and `open` states put the joint, and `mimic`, the `Mimic` it follows, or `None` for a joint that moves of its own. |
| `Mimic` | How a finger joint follows another of its gripper, as its URDF `<mimic>` gives it: the `joint` followed, and the `multiplier` and `offset` that place the finger at `multiplier` times that joint's position plus `offset`. |
| `CameraFrame` | A camera's optical frame (+z along the view, +x right, +y down): `name`, `parent_link`, `xyz` and `rpy` in the parent link's frame, and `depth_range`, the `DepthRange` the URDF's depth sensor on the frame gives, or `None` for a camera the URDF gives no depth sensor. |
| `DepthRange` | The depths a camera measures, meters along its view: its `near` and `far`. |
| `JointLimits` | A joint's `lower` and `upper` position limits, the URDF's, unbounded for a continuous joint, and its `velocity` and `effort` limits, as the URDF gives them. |
| `READY`, `HOME`, `PARK`, `CLOSED`, `OPEN` | The names of the states the SRDFs give. |

## Testing

```sh
uv sync --group dev
uv run pytest
```

`tests/test_types.py` holds `robot_description_py.pyi` to the module with
mypy's stubtest.
