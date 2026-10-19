# brain_core

The code every brain node shares. A brain serves `item_perception:v1` and
`item_manipulation:v1` over a robot that `limb_motion:v1` drives and a camera
that `rgbd_camera:v1` streams. brain_core holds the contract rules (item ids,
the gripper table, the lanes, the result rules), the robot proxy, the camera
frames and geometry, and the loop of each exposed member. A node brings its
generated peppygen modules and its backends, and calls `node.setup`.

Pure Python on numpy and Pillow. brain_core never imports peppygen: peppygen is
generated per node, so the node passes its modules in through `Bindings`.

## Using it

A node depends on brain_core by commit:

```
brain_core @ git+https://github.com/Peppy-bot/public-peppy-libs.git@<commit>#subdirectory=brain_core
```

Its entry point builds `Bindings` from its peppygen package and hands them to
`brain_core.node.setup` with its two backends:

```python
from functools import partial

from peppygen import NodeBuilder, QoSProfile, clock
from peppygen.consumed_actions.limb_motion import move_arm, move_gripper
from peppygen.consumed_services.camera import depth_stream_info
from peppygen.consumed_services.camera_mounts import get_camera_poses
from peppygen.consumed_services.geometry import get_color_intrinsics, get_depth_intrinsics
from peppygen.consumed_topics.camera import depth_stream, video_stream
from peppygen.exposed_actions.item_manipulation import abort, drop_item, grab_item, place_item
from peppygen.exposed_actions.item_perception import identify_item, scan_items
from peppygen.exposed_services.item_manipulation import get_state

from brain_core import node
from brain_core.bindings import Bindings

BINDINGS = Bindings(
    clock=clock, standard_qos=QoSProfile.Standard, move_arm=move_arm, move_gripper=move_gripper,
    video_stream=video_stream, depth_stream=depth_stream, depth_stream_info=depth_stream_info,
    get_color_intrinsics=get_color_intrinsics, get_depth_intrinsics=get_depth_intrinsics,
    get_camera_poses=get_camera_poses, scan_items=scan_items, identify_item=identify_item,
    grab_item=grab_item, drop_item=drop_item, place_item=place_item, abort=abort, get_state=get_state,
)


async def setup(params, node_runner):
    return await node.setup(params, node_runner, BINDINGS, MyDetector(), MyManipulator())


def main() -> None:
    node.configure_logging("my_brain")
    NodeBuilder().run(setup)
```

The parameters the core reads are `limbs` and `camera_name`.

## The parts

| Module | What it holds |
|---|---|
| `ports` | The plain data that crosses the seams (`Box`, `Detection`, `Item`, `Gripper`, `Outcome`, `Pose`), the roles a model serves (`Role`, `parse_role`), `Refusal`, `ExpectedFailure`, `Cancelled`, `CancelToken`, `Deadline`, `SearchTimeout`, and the two ports a backend implements: `Detector` (an awaited `detect`, and `why_unavailable`) and `Manipulator` |
| `words` | The one rule for whether a description names a label |
| `limbs` | The `limbs` parameter parsed: each gripper with the arm that carries it |
| `state` | The grippers, the known items and their ids, and the rules that resolve a goal's names |
| `sequencer` | One sequence at a time per lane (a lane is a `Role`), lane-less jobs beside them, and the way to stop the ones that run |
| `robot` | The proxy to the backbone: one limb_motion goal per move, and `stop`, which cancels every move in flight |
| `brain` | The parts assembled, and `run_guarded`, where every goal is admitted, completed the way the contract asks and logged |
| `handlers` | One module per exposed member |
| `serve`, `waiting` | The loop of an exposed action or service, and the one way a loop waits on the node's token |
| `perception` | The `Perceiver` (frames in, world detections out), the frame store, the camera model and the geometry learners, `in_daemon_thread` for a detector whose model blocks, and `NoneDetector` |
| `manipulation` | `NoneManipulator` |
| `bindings`, `node` | The generated modules a node passes in, and the shared entry |

## Tests

```
uv sync --frozen && uv run --frozen pytest
```

The tests run every core rule against fakes of the generated modules
(`tests/fake_peppygen.py`): no peppygen, no router, no robot and no model.
