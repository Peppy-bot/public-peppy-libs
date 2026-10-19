# brain_core

The code every brain node shares. A brain serves `item_perception:v1`,
`item_manipulation:v1` and `brain_model:v1` over a robot that `limb_motion:v1`
drives and a camera that `rgbd_camera:v1` streams. brain_core holds the contract rules (item ids,
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
`brain_core.node.setup` with its `ModelSource`, the models the node can run
for each role. `Bindings.from_generated` finds each module where every
brain's peppygen puts it, so the node's manifest names its slots
`limb_motion`, `camera`, `geometry` and `camera_mounts`, as both brains do:

```python
import peppygen
from peppygen import NodeBuilder

from brain_core import node
from brain_core.bindings import Bindings

BINDINGS = Bindings.from_generated(peppygen)


async def setup(params, node_runner):
    return await node.setup(params, node_runner, BINDINGS, MyModels(params))


def main() -> None:
    node.configure_logging("my_brain")
    NodeBuilder().run(setup)
```

The parameters the core reads are `limbs`, `camera_name`, and the starting
model of each role, `perception_model` and `manipulation_model`.

## The parts

| Module | What it holds |
|---|---|
| `ports` | The plain data that crosses the seams (`Box`, `Detection`, `Item`, `Gripper`, `Outcome`, `Pose`), the roles a model serves (`Role`, `parse_role`), `Refusal`, `ExpectedFailure`, `Cancelled`, `CancelToken`, `Deadline`, `SearchTimeout`, the two ports a backend implements, `Detector` (an awaited `detect`, and `why_unavailable`) and `Manipulator`, and the port a node implements, `ModelSource` |
| `words` | The one rule for whether a description names a label |
| `limbs` | The `limbs` parameter parsed: each gripper with the arm that carries it |
| `state` | The grippers, the known items and their ids, and the rules that resolve a goal's names |
| `sequencer` | One sequence at a time per lane (a lane is a `Role`), lane-less jobs beside them, and the way to stop the ones that run |
| `robot` | The proxy to the backbone: one limb_motion goal per move, and `stop`, which cancels every move in flight |
| `models` | The model of each role, and the switch rule: a goal uses only its own role's model, and a switch never changes the model of a goal that runs |
| `brain` | The parts assembled, and `run_guarded`, where every goal is admitted, completed the way the contract asks and logged |
| `handlers` | One module per exposed member |
| `serve`, `waiting` | The loop of an exposed action or service, and the one way a part waits unless a stop comes first |
| `perception` | The `Perceiver` (frames in, world detections out), the frame store, the camera model and the geometry learners, `in_daemon_thread` for a detector whose model blocks, and `NoneDetector` |
| `manipulation` | `NoneManipulator` |
| `bindings`, `node` | The generated modules a node passes in, and the shared entry |

## Tests

```
uv sync --frozen && uv run --frozen pytest
```

The tests run every core rule against fakes of the generated modules
(`tests/fake_peppygen.py`): no peppygen, no router, no robot and no model.

## How it works

### Structure

```mermaid
classDiagram
    direction LR
    class Brain {
        State state
        Robot robot
        Perceiver perceiver
        Models models
        Sequencer sequencer
        start()
        background(token)
        shutdown()
        run_guarded(ctx, lane, action, body, zero, asked)
        look(job, detector, phrases, timeout_s) Look
        manipulator_or_refuse(manipulator)
    }
    class State {
        grippers
        items
        remember(detections, now_ns, coverage)
        gripper_for_grab(name, near)
        holder(name)
        forget_dropped(gripper)
        clear_held_placed(gripper, position, orientation)
    }
    class Sequencer {
        start(lane_or_none, action, now_ns) Job
        finish(job)
        cancel(job, reason, by_caller)
        stop(lane, reason)
        stop_laneless(reason)
        close()
    }
    class Job {
        lane
        action
        CancelToken cancel
        notes
    }
    class Models {
        start(names)
        roles()
        names(role)
        current(role)
        current_name(role)
        switch(role, name, job)
    }
    class ModelSource {
        <<interface>>
        roles()
        names(role)
        offered(role, name) entry
        start(entry)
        prepare(entry) async
    }
    class Perceiver {
        FrameStore frames
        CameraModel camera
        scan(detector, phrases, cancel, timeout_s) Look
    }
    class Detector {
        <<interface>>
        why_unavailable()
        detect(image, phrases, deadline) async
        scan_coverage()
    }
    class Manipulator {
        <<interface>>
        why_unavailable()
        start(robot)
        grab()
        drop()
        place()
        stop()
    }
    class Robot {
        move_arm()
        set_gripper()
        stop()
    }
    class Bindings {
        the generated modules
        from_generated(package)
    }
    Brain --> State
    Brain --> Sequencer
    Brain --> Models
    Brain --> Perceiver
    Brain --> Robot
    Brain --> Bindings
    Sequencer --> Job
    Models --> ModelSource
    Models --> Detector : perception
    Models --> Manipulator : manipulation
    Manipulator --> Robot
```

A node implements `ModelSource`, and its models implement `Detector` or
`Manipulator`. `NoneDetector` and `NoneManipulator` are the models of a role
with none.

### A goal, from its arrival to its result

`identify_item`; `scan_items` runs the same way, and the manipulation goals
run in the manipulation lane on the manipulation model.

```mermaid
sequenceDiagram
    autonumber
    participant C as caller
    participant S as serve_action
    participant T as goal task
    participant B as Brain.run_guarded
    participant Q as Sequencer
    participant M as Models
    participant P as Perceiver
    participant D as Detector
    participant St as State
    C->>S: identify_item goal
    S->>S: accept it
    S-)T: create_task(run(ctx))
    S->>S: wait for the next goal
    T->>B: run_guarded(ctx, PERCEPTION, "identify_item", body, ZERO)
    B->>Q: start(PERCEPTION, "identify_item")
    alt the lane runs a goal, or the node is shutting down
        Q-->>B: Refusal: "scan_items is running"
        B-->>C: complete(success=false, message, action_time)
    else admitted
        Q-->>B: Job
        B-)B: cancel watcher on ctx.cancel_signal()
        B->>T: body(job)
        T->>M: current(PERCEPTION), once, at the start of the body
        M-->>T: the perception model
        T->>B: look(job, detector, phrases, timeout_s)
        B->>D: why_unavailable(): a reason refuses the search
        B->>P: scan(detector, phrases, job.cancel, timeout_s)
        P->>P: why_unavailable(): the frames and the camera model
        P->>D: await detect(image, phrases, deadline)
        D-->>P: boxes, or SearchTimeout, or an ExpectedFailure
        P-->>B: Look, the boxes placed in the robot frame
        B-->>T: Look
        T->>St: best_match, then remember
        T-->>B: result fields
        B-->>C: complete(success=true, fields, action_time)
        B->>Q: finish(job)
    end
```

A Refusal, an ExpectedFailure or a Cancelled raised in the body goes up to
`run_guarded`, which completes the goal with the message; `action_time` is the
elapsed time whatever the outcome.

### Cancel, abort and shutdown

```mermaid
sequenceDiagram
    participant C as caller
    participant A as abort handler
    participant W as cancel watcher
    participant Q as Sequencer
    participant J as job.cancel
    participant L as Brain stop of a lane
    participant Mn as manipulation model
    participant R as Robot
    participant H as body and run_guarded
    participant S as Brain.shutdown
    Note over C,H: A cancel by the goal's own caller
    C->>W: cancel on the goal handle
    W->>Q: cancel(job, "cancelled by the caller", by_caller=true)
    Q->>J: cancel(reason, by_caller=true)
    Q->>L: the lane's stop
    L->>Mn: stop(), manipulation lane only
    L->>R: stop(): cancel every limb_motion goal in flight
    H->>J: check() raises Cancelled(by_caller=true)
    H-->>C: complete_cancelled(success=false, reason)
    Note over C,H: abort, from any caller
    C->>A: abort(reason)
    A->>Q: stop(MANIPULATION, "aborted: reason")
    Q->>J: cancel(reason, by_caller=false)
    Q->>L: the lane's stop
    H->>J: check() raises Cancelled(by_caller=false)
    H-->>C: the running goal completes success=false "aborted: reason"
    Q-->>A: returns once the job is done, with its action
    A-->>C: success=true, aborted_action
    Note over C,S: The node's shutdown hook
    S->>Q: close(): every new goal is refused "the node is shutting down"
    S->>Q: stop(MANIPULATION, "aborted: the node is shutting down")
    S->>Q: stop(PERCEPTION, "aborted: the node is shutting down")
    S->>Q: stop_laneless(...): a switch_model stops, its role keeps its model
    S->>R: stop()
```
