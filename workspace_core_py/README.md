# workspace_core_py

The Python bindings of [`workspace_core`](../workspace_core), for a robot's
backbone in Python. They give what the backbone needs to give the
`workspace:v1` answer of a robot without a perception camera, around the reach
its own solver gives: the limits, the grasp directions, the parsing of a
request, the grid targets of a surface, the reach memo, and the answers with
their messages. The definitions are those of `workspace_core` (see its
README), and the answers and their messages are the same as those of a
backbone in Rust, byte for byte.

## Build and install

The package is a PyO3 extension module that maturin builds for the stable ABI
of Python 3.12 (`abi3`): one build loads on every Python from 3.12. An install
compiles the module from the committed `Cargo.lock` exactly (`locked`), so the
host needs a Rust toolchain (`cargo`). The module does not link libpython: the
Python that loads it supplies the Python API.

A consumer pins a commit of this repository:

```
workspace_core_py @ git+https://github.com/Peppy-bot/public-peppy-libs.git@<commit>#subdirectory=workspace_core_py
```

or, with uv:

```toml
[tool.uv.sources]
workspace_core_py = { git = "https://github.com/Peppy-bot/public-peppy-libs", subdirectory = "workspace_core_py", rev = "<commit>" }
```

## What it gives

Points and directions are `(x, y, z)` tuples in the robot frame, in metres.
Rectangles are `(x_min, x_max, y_min, y_max)` tuples. A refusal of a request
is a `ValueError` whose text is the refusal that the answer carries.

| Name | What it is |
|---|---|
| `REACH_TOLERANCE`, `GRASP_ANGLE_TOLERANCE`, `ABOVE_SURFACE` | the limits of `workspace_core`: 0.01 m, 0.05 rad and 0.04 m |
| `GRASP_DIRECTIONS` | the grasp directions in the order a reach tries them, each a `GraspDirection` with its `name` (`down`, `forward`) and its unit `approach` vector |
| `angle_between(a, b)` | the angle between two directions, each an iterable of three numbers (a tuple, a list, a numpy array), in radians from 0 to pi; pi when one is zero |
| `SurfaceHeight.from_wire(metres)` | the surface height of a describe_workspace request, a finite number within 1000 m of the robot's base point, to the millimetre: `metres`, and `grid_targets()`, the targets `ABOVE_SURFACE` above each point of the standard grid, in the grid's order. Two heights that round alike are equal and hash alike |
| `Positions.from_wire(values)` | the points of a check_positions request, three numbers per point: `points`, in the request's order |
| `Reach.reached_by(arm)`, `Reach.short(by)` | whether an arm reaches a target: `reached`, `arm` (`""` when none does) and `short_by` (0 when an arm does). An empty arm name, and a shortfall that is not a finite distance of at least 0, are refused |
| `SurfaceReach.from_grid_order(height, reaches)` | the reach of the surface at `height`: one `Reach` per grid target, in the grid's order; another count is refused |
| `ReachMemo()` | the reach of the surfaces measured last, one per height: `get(height)`, `insert(surface)`, which gives back the reach stored first for the height, and `len(memo)`. Past 32 heights it drops the height it stored first, also when a caller asked that height again after it was stored |
| `describe_without_perception_camera(surface)` | the `SurfaceAnswer` of the surface: `workable`, `area`, `rectangle`, `reach`, `view` (always None) and `message` |
| `check_without_perception_camera(positions, reaches)` | the `PositionsAnswer` of the request: `points`, each a `PointAnswer` with `position`, `reach`, `view` (`no_camera`), `in_view` (False), `workable` and `message`, then `all_workable` and `message`. One `Reach` per point, in the request's order; another count is refused |

The messages say that the view is not checked, because the robot has no
perception camera. `Reach` and `SurfaceHeight` pickle, so that a worker
process can send them. `Reach` and `GraspDirection` compare by value and do
not hash. The types of the module are in `workspace_core_py.pyi`.

## How a backbone answers

The backbone measures the reach of a target with its own solver: the target
is reached when an arm brings its grasp point within `REACH_TOLERANCE` of it
with the approach axis of its gripper within `GRASP_ANGLE_TOLERANCE` of one of
`GRASP_DIRECTIONS` (`angle_between`). Otherwise the reach is short by how far
the closest arm stops from the target in any orientation.

describe_workspace:

1. `height = SurfaceHeight.from_wire(surface_height)`.
2. `surface = memo.get(height)`. When it is None, measure the reach of each of
   `height.grid_targets()`, then
   `surface = memo.insert(SurfaceReach.from_grid_order(height, reaches))`.
3. `describe_without_perception_camera(surface)`.

check_positions:

1. `positions = Positions.from_wire(values)`.
2. Measure the reach of each of `positions.points`.
3. `check_without_perception_camera(positions, reaches)`.

## Tests

```
uv sync
uv run pytest
```

`uv sync` builds the module and installs it into `.venv`, and builds it again
when a Rust source of this package or of `workspace_core` changes. `cargo
test` builds and links the module as maturin does, without libpython
(`.cargo/config.toml`); the tests of the bindings are in Python.
`tests/test_types.py` compares the types in `workspace_core_py.pyi` with the
module: through mypy's stubtest, and for the classes that do not hash, which
stubtest does not compare.
