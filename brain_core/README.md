# brain_core

The code every brain node shares. A brain serves `item_perception:v1`,
`item_manipulation:v1` and `brain_model:v1` over a robot that `limb_motion:v1`
drives and `limb_state:v1` measures, and a camera that `rgbd_camera:v1` streams. brain_core holds the contract rules (item ids,
the gripper table, the lanes, the failure kinds and the result rules), the
robot proxy, the camera frames and geometry, and the loop of each exposed
member. A node brings its generated peppygen modules and its backends, and
calls `node.setup`.

item_perception and brain_model report a refusal or a failure by the status
of the goal: a refused goal and a failed goal carry a failure kind and a
message, and a failed goal gives no result. item_manipulation reports both in
its result, with `success` false and a message.

Pure Python on numpy and Pillow. brain_core never imports peppygen: peppygen is
generated per node, so the node passes its modules in through `Bindings`.

## Using it

A node depends on brain_core by commit. Both brains pin it in
`[tool.uv.sources]`, the way `so101/backbone` pins its shared libraries:

```toml
[tool.uv.sources]
brain_core = { git = "https://github.com/Peppy-bot/public-peppy-libs", subdirectory = "brain_core", rev = "<commit>" }
```

uv fetches a git source with git, and the `peppybot/python-uv-base` image has
no git, so the node's `apptainer.def` installs it in `%post` before
`uv sync`, as nodes-hub's `ai_brain_vla/apptainer.def` does.

Its entry point builds `Bindings` from its peppygen package and hands them to
`brain_core.node.setup` with its `ModelSource`, the models the node can run
for each role. `Bindings.from_generated` finds each module where every
brain's peppygen puts it, so the node's manifest names its slots
`limb_motion`, `limb_state`, `camera`, `geometry` and `camera_mounts`, as both
brains do:

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
| `ports` | The plain data that crosses the seams (`Box`, `Detection`, `Item`, `Gripper`, `Outcome`, `Pose`, and `Senses`, what a manipulator is started with: the robot, the limbs, the chest perceiver, the wrist frames and the camera poses), the roles a model serves (`Role`, `parse_role`), the failure kinds a goal ends with (`FailureKind`), `Refusal` and `ExpectedFailure`, each with its kind, `Cancelled`, `CancelToken`, `Deadline`, `SearchTimeout`, the port a detector implements, `Detector` (an awaited `detect`, and `why_unavailable`), and the port a node implements, `ModelSource` |
| `words` | The one rule for whether a description names a label |
| `limbs` | The `limbs` parameter parsed: each gripper with the arm that carries it |
| `state` | The grippers, the known items and their ids, and the rules that resolve a goal's names |
| `sequencer` | One sequence at a time per lane (a lane is a `Role`), lane-less jobs beside them, and the way to stop the ones that run |
| `robot` | The proxy to the backbone: one limb_motion goal per move, how each one ended (completed, cancelled with its reason, refused or failed with its kind, or lost), `check_arm_move`, which asks whether a move has a plan and moves nothing, and `stop`, which cancels every move in flight |
| `limb_state` | The robot's measured limbs: each limb_states message parsed into a `LimbSnapshot` (each arm's joints and grasp pose, each gripper's opening), the latest one kept, the wait for one captured after another, and the openings get_state reports |
| `models` | The model of each role, and the switch rule: a goal uses only its own role's model, and a switch never changes the model of a goal that runs |
| `brain` | The parts assembled, `decider`, where a goal is refused before it is accepted, and `run_guarded`, where every goal is admitted, ended the way its contract asks and logged |
| `handlers` | One module per exposed member |
| `serve`, `waiting` | The loop of an exposed action or service, and the one way a part waits unless a stop comes first |
| `perception` | The `Perceiver` (frames in, world detections out), the frame store, the camera model and the geometry learners, `in_daemon_thread` for a detector whose model blocks, `NoneDetector`; `wrist`, the newest frame of each arm's wrist camera and the wait for one captured after a move |
| `manipulators` | The `Manipulator` port (started with the `Senses`; `grab`, `drop`, `place`, `stop` and `why_unavailable`); `none`, the `NoneManipulator`; `scripted`, the `ScriptedManipulator`, a grab from above through a fixed sequence of moves, with the `CentreProposer`, the grasp from the item's position, the geometry (`grasp`: the orientations workspace_core tries, the pre-grasp and lifted points, the turn about the vertical, `Grasp` and the `GraspProposer` port) and the `Aligner` port (`aligning`) of the step a node may put between the move above the item and the descent |
| `bindings`, `node` | The generated modules a node passes in, and the shared entry |

## Tests

```
uv sync --frozen && uv run --frozen pytest
```

The tests run every core rule against fakes of the generated modules
(`tests/fake_peppygen.py`): no peppygen, no router, no robot and no model.

## How it works

### Where it sits

Both brains are thin: each brings only its models, and brain_core holds the
shared work. A dotted arrow is a manifest relation or a hand-over at run time.

```mermaid
flowchart TB
    subgraph MH["mcp-hub"]
        MCP["robot_control:v1<br/>MCP exposure"]
    end
    subgraph CH["contracts-hub"]
        direction LR
        BM["brain_model:v1"]
        IC["item_perception:v1 +<br/>item_manipulation:v1"]
    end
    subgraph NH["nodes-hub"]
        direction LR
        VLA["ai_brain_vla<br/>SAM 3 + SigLIP"]
        VLM["vlm_router_brain<br/>catalog → any-llm"]
    end
    TORCH["torch +<br/>transformers"]
    BC["brain_core<br/>public-peppy-libs"]
    ANY["any-llm-sdk"]
    MCP -->|exposes| BM
    MCP -->|exposes| IC
    BM -.->|implemented by| VLA
    BM -.->|implemented by| VLM
    IC -.->|implemented by| VLA
    IC -.->|implemented by| VLM
    VLA -->|imports| TORCH
    VLA -->|pinned import| BC
    VLA -.->|Bindings| BC
    VLM -->|pinned import| BC
    VLM -.->|Bindings| BC
    VLM -->|imports| ANY
```

brain_core receives each node's `Bindings` and its `ModelSource`; the
catalog and any-llm stay inside the router, and torch inside ai_brain_vla.

### Structure

```mermaid
classDiagram
    direction LR
    class Brain {
        State state
        Robot robot
        LimbStateStore limb_states
        Perceiver perceiver
        Models models
        Sequencer sequencer
        start()
        background(token)
        shutdown()
        check_search()
        run_guarded(ctx, lane, action, body, zero, ends, asked)
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
        check(lane_or_none)
        start(lane_or_none, action, now_ns) Job
        finish(job)
        cancel(job, reason, signalled)
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
        check_switch(role, name)
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
        start(robot, limb_states)
        grab()
        drop()
        place()
        stop()
    }
    class Robot {
        move_arm()
        set_gripper()
        check_arm_move()
        stop()
    }
    class LimbStateStore {
        latest
        why_unavailable()
        newer_than(timestamp, timeout_s) async
        openings(grippers)
        run(node_runner, token, limb_states)
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
    Brain --> LimbStateStore
    Brain --> Bindings
    Sequencer --> Job
    Models --> ModelSource
    Models --> Detector : perception
    Models --> Manipulator : manipulation
    Manipulator --> Robot
    Manipulator --> LimbStateStore
```

A node implements `ModelSource`, and its models implement `Detector` or
`Manipulator`. `NoneDetector` and `NoneManipulator` are the models of a role
with none; `ScriptedManipulator` is a manipulator any node can offer, built
on a `GraspProposer`, as it is or with an `Aligner` of the node's own.

### A goal, from its arrival to its result

`identify_item`; `scan_items` runs the same way, and `switch_model` is
decided by its role and model (`Models.check_switch`). The manipulation
goals accept every goal at once, run in the manipulation lane on the
manipulation model, and report a refusal or a failure in their result.

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
    S->>B: decider: check_search()
    alt the node shuts down, or the lane runs a search
        B-->>C: refused, precondition_failed: "scan_items is running"
    else no perception source
        B-->>C: refused, resource_unavailable ("no perception source: ...")<br/>or precondition_failed for the model "none"
    else accepted
        S-)T: create_task(run(ctx))
        S->>S: wait for the next goal
        T->>B: run_guarded(ctx, PERCEPTION, "identify_item", body, ZERO, STATUS_ENDS)
        B->>Q: start(PERCEPTION, "identify_item")
        alt another goal won the lane after the decision
            Q-->>B: Refusal, precondition_failed
            B-->>C: fail(precondition_failed, "scan_items is running")
        else admitted
            Q-->>B: Job
            B-)B: cancel watcher on ctx.cancel_signal()
            B->>T: body(job)
            T->>M: current(PERCEPTION), once, at the start of the body
            M-->>T: the perception model
            T->>B: look(job, detector, phrases, timeout_s)
            B->>D: why_unavailable(): checked again
            B->>P: scan(detector, phrases, job.cancel, timeout_s)
            P->>P: the frames and the camera model, checked again
            P->>D: await detect(image, phrases, deadline)
            D-->>P: boxes, or SearchTimeout, or an ExpectedFailure
            P-->>B: Look, the boxes placed in the robot frame
            B-->>T: Look
            T->>St: best_match, then remember
            alt no item matches
                T-->>B: ExpectedFailure, not_found
                B-->>C: fail(not_found, "no item matches 'cup'")
            else found
                T-->>B: result fields
                B-->>C: complete(fields, action_time)
            end
            B->>Q: finish(job)
        end
    end
```

A Refusal or an ExpectedFailure raised in the body goes up to
`run_guarded`, which ends the goal through the `Ends` of its contract:
`STATUS_ENDS` fails the goal with the kind and the message and gives no
result, `RESULT_ENDS` completes it with success false and the message. A
search past its bound fails with `timeout`, and any other exception with
`attempt_failed`. `action_time` is the elapsed time whatever the outcome.

### A scripted grab

`ScriptedManipulator.grab`, after the handler has resolved the item and the
gripper. Each step that fails ends the grab with the backbone's own words. A
step that fails once the descent was sent first retreats: the gripper opens
and the arm moves straight back up to the pre-grasp point. A goal that a stop
ended, a goal that ends cancelled, retreats from nothing.

With an `aligner`, one more step sits between the move to the pre-grasp
point and the descent. The manipulator hands it the item, the gripper, the
pre-grasp, grasp and lifted points and the orientation (`AlignRequest`, in
`manipulators/scripted/aligning`), and the aligner, a node's own behind the
`Aligner` port (vlm_router_brain's `ModelAligner`, a model that looks through
the wrist and chest cameras), may move the gripper sideways and turn it at
the same height through the manipulator's `refusal_for` (whether the
backbone has a plan for points at an orientation) and `move_to`, so each of
its moves carries the bars of every arm move. It ends `Aligned` with the
points the grab goes on with, `GaveUp`, `OutOfSteps` or `Stopped` when a
stop ended a move; the last three fail the grab there, before the descent.
A place runs the same step above the spot the held item goes to, measured
from the target point, and then lowers the item until it rests or the point
is reached: an arm the backbone stops short anywhere from the pre-place
point down, within 5 cm of the line down, has set the item on what is under
it, and the place opens there.

Every move names its bars, as the limb_motion contract has the caller do. An
arm move, the retreat's and a place's too, plans within 1 mm and 0.01 rad of
its pose and settles within `settle_position_m` (2 cm) and
`settle_orientation_rad` (0.15 rad), the settle the robots' endpoint gives for
the OpenArm v2, or fails with `timeout` `settle_timeout_s` (2 s) after
its planned end, with the backbone's words, such as `still 8.0 mm / 0.02 rad
from the target, 2.0 s after the planned end`. A check asks with the same
plan bars. A gripper move waits up to `grip_settle_timeout_s` (2 s) for the
jaws to stand still.

```mermaid
sequenceDiagram
    participant H as grab_item handler
    participant SM as ScriptedManipulator
    participant P as GraspProposer
    participant R as Robot
    participant L as LimbStateStore
    H->>SM: grab(item, gripper, max_effort, cancel)
    SM->>P: propose(item)
    P-->>SM: Grasp: the point, the orientations to try
    loop each orientation, until one plans
        SM->>R: check_arm_move: pre-grasp, grasp, lifted points
    end
    Note over SM: none plans: "no plan reaches item 'x' with gripper 'g': <last refusal>"
    SM->>R: set_gripper(g, the item's width and 1.5 cm each side, else 1)
    SM->>R: move_arm(pre-grasp), move_arm(grasp point): each settles, or fails
    SM->>R: set_gripper(g, 0, max_effort)
    Note over SM: final_opening below hold_opening_min: "gripper 'g' closed on nothing"
    SM->>R: move_arm(lifted point)
    SM->>L: newer_than(the latest stamp at the lift's result, snapshot_wait_s)
    Note over SM: none: "no limb_states snapshot after the lift"<br/>opening below hold_opening_min, or more than hold_drop_max below the close's: "item 'x' slipped while lifting"
    alt a step from the descent on failed, and no stop ended it
        SM->>R: set_gripper(g, 1), move_arm(pre-grasp)
        Note over SM: either fails: "<failure>; the retreat to the pre-grasp point failed: <why>"
        SM-->>H: Outcome(false, the failure)
    else every step went through
        SM-->>H: Outcome(true): the handler sets the held flag
    end
```

Before the descent the jaws open only as wide as the item and 1.5 cm on each
side (`approach_margin_m`), as a fraction of their 14.8 cm span
(`jaw_travel_m`, the OpenArm v2's in Waldo), so the fingers come down beside
the item and not on what stands next to it. The width is the one the look that
last saw the item measured: the box's left and right edges at the depth of its
seen point. An item of no known width, one addressed by pose or put down by
place_item, gets the jaws fully open.

Each grab logs one line: the orientation that planned, with the number and
the total time of the check_arm_move calls; how wide the jaws opened for the
item's width; where the descent's result put the
grasp point against the commanded one (`[dx, dy, dz] cm`), measured once the
arm settled; limb_state's grasp point at the close; the two openings it
judged; and the retreat.

A place whose jaws stand closed on air (an opening below `hold_opening_min`
in the latest snapshot: the item slipped since the grab) fails before any
move and says so; a drop lets the brain forget the item. Otherwise a place
carries the item to the pose through its pre-place point,
`clearance_m` and `held_item_depth_m` (7.5 cm, how far a held item may
reach below the grasp point) above the pose, so the item clears the floor of
a container and what already stands in it until the descent, or `clearance_m`
alone above it where the backbone has no plan that high; it opens there and
climbs back to the pre-place point. An arm that stops short on its way down
does not settle, so its move fails with `timeout`. When the latest
limb_states snapshot puts its grasp point anywhere from the pre-place point
down and within 5 cm (`RELEASE_OFF_LINE_M`) of the line down, the arm has
set the item on what is under it, the rim or the floor of a container or the
items already in it included (an item slides off a round top by a few
centimetres before it rests): the place opens there,
logs how far short it stopped, and reports that point. A stop of the
descent, or an arm stopped more than 5 cm above the pre-place point or off
the line down, fails the place with the jaws closed. A climb that fails is
logged and the place still succeeds, since the item is down and the jaws are
open.

`CentreProposer` lowers a seen item's position by `grasp_depth_m` and tries
the eight rolls of a grasp from above; an item whose position is a grasp point
(a grab by pose, or an item place_item put down) is grasped there unlowered,
in its own orientation alone when it has one. After every await the sequence
checks its cancel token, so a cancel, an abort or a shutdown ends it at the
next step.

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
    Note over C,H: The goal's own cancel signal: its caller cancels it, or its sender is gone
    C->>W: cancel on the goal handle
    W->>Q: cancel(job, "cancelled by its caller" or "its sender is gone", signalled=true)
    Q->>J: cancel(reason, signalled=true)
    Q->>L: the lane's stop
    L->>Mn: stop(), manipulation lane only
    L->>R: stop(): cancel every limb_motion goal in flight
    H->>J: check() raises Cancelled(signalled=true)
    H-->>C: complete_cancelled(reason, the zero fields)
    Note over C,H: abort, from any caller, item_manipulation
    C->>A: abort(reason)
    A->>Q: stop(MANIPULATION, "aborted: reason")
    Q->>J: cancel(reason, signalled=false)
    Q->>L: the lane's stop
    H->>J: check() raises Cancelled(signalled=false)
    H-->>C: the running goal completes success=false "aborted: reason"
    Q-->>A: returns once the job is done, with its action
    A-->>C: success=true, aborted_action
    Note over C,S: The node's shutdown hook
    S->>Q: close(): every new goal is refused "the node is shutting down"
    S->>Q: stop(MANIPULATION, "aborted: the node is shutting down")
    S->>Q: stop(PERCEPTION, "the node is shutting down"): a search ends cancelled
    S->>Q: stop_laneless(...): a switch_model ends cancelled, its role keeps its model
    S->>R: stop()
```
