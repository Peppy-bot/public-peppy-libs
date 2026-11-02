# path_timing_py

The Python bindings of [`control_core`](../control_core)'s path timing
(`control_core::path_timing`), for a robot's backbone in Python. They give the
joint trajectory of `limb_motion`'s `execute_joint_path`: the path through the
waypoints with each corner rounded, timed under each joint's velocity and
acceleration limits, and played on the backbone's clock. The rounding and the
timing are those of `control_core`, which the OpenArm backbone plays in Rust,
so a joint path runs alike on both robots.

## Build and install

The package is a PyO3 extension module that maturin builds for the stable ABI
of Python 3.12 (`abi3`): one build loads on every Python from 3.12. An install
compiles the module from the committed `Cargo.lock` exactly (`locked`), so the
host needs a Rust toolchain (`cargo`). The module does not link libpython: the
Python that loads it supplies the Python API.

A consumer pins a commit of this repository:

```
path_timing_py @ git+https://github.com/Peppy-bot/public-peppy-libs.git@<commit>#subdirectory=path_timing_py
```

or, with uv:

```toml
[tool.uv.sources]
path_timing_py = { git = "https://github.com/Peppy-bot/public-peppy-libs", subdirectory = "path_timing_py", rev = "<commit>" }
```

## What it gives

A configuration of the joints is a tuple of floats, one position per joint, in
radians (meters for a prismatic joint). Times are seconds on the caller's
clock, from the start of the trajectory. A refusal is a `ValueError` whose text
says what it refuses.

| Name | What it is |
|---|---|
| `JOINT_PATH_CORNER_DEVIATION`, `MIN_JOINT_CHANGE_RAD` | the constants of `control_core`: 0.1, the farthest the midpoint of a corner's arc lies from the corner, Euclidean over the joints, and 0.001, the most a waypoint differs in every joint from the waypoint given before it and still repeats it |
| `JointTrajectory.through(waypoints, velocity_limits, acceleration_limits, requested_s)` | the trajectory through `waypoints` (one or more, where the joints stand first), as `control_core`'s `JointTrajectory::through` makes it: each corner rounded within `JOINT_PATH_CORNER_DEVIATION`, resting where the path turns back on itself and at each waypoint within `MIN_JOINT_CHANGE_RAD` in every joint of the one before it, timed under the limits (one finite positive number per joint) and played over the longer of `requested_s` and the path's own time. The waypoints are any iterable of iterables of numbers, and each set of limits any iterable of numbers (a tuple, a list, a numpy array). The rounding and the timing run without the GIL, so a backbone that makes the trajectory on a thread keeps its event loop running |
| `duration_s`, `start`, `end` | the seconds the trajectory plays over, its first waypoint and its last |
| `sample(t_s)` | the joints at `t_s`: `start` at or before 0, `end` from `duration_s` on, and the path between them in between |
| `done(t_s)` | whether the trajectory has played to its end at `t_s` |

`through` refuses no waypoint, waypoints of different numbers of joints, a
position that is not a finite number, limits that do not hold one finite
positive number per joint, a `requested_s` that is not a finite number of at
least 0, and a path whose arithmetic fails in `control_core`, where a sum
overflows or a speed underflows (waypoints some 1e154 apart, limits near the
smallest positive double), with the assertion it fails. Limits too small to
time the path in a finite number of seconds give a `duration_s` of infinity,
and the joints stand at `start` throughout. `sample` refuses a time that is
not finite. The types of the module are in `path_timing_py.pyi`.

## How a backbone plays a joint path

1. `trajectory = JointTrajectory.through((measured, *waypoints), velocity_limits, acceleration_limits, duration_s)`,
   on a thread.
2. Each control tick, `t_s` seconds after the move started, command
   `trajectory.sample(t_s)`.
3. From `trajectory.done(t_s)` on, the move holds `trajectory.end` and settles
   there.

## Tests

```
uv sync
uv run pytest
```

`uv sync` builds the module and installs it into `.venv`, and builds it again
when a Rust source of this package or of `control_core` changes. `cargo test`
builds and links the module as maturin does, without libpython
(`.cargo/config.toml`); the tests of the bindings are in Python, and those of
the rounding and the timing are `control_core`'s. `tests/test_types.py`
compares the types in `path_timing_py.pyi` with the module through mypy's
stubtest.
