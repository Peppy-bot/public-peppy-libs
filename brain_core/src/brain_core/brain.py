"""The brain assembled: the core parts, the model of each role from the
node's model source, the one way every goal is decided, and the one way
every action runs a sequence and ends its goal.

`decider` is where a goal is refused before it is accepted. A member whose
contract reports by failure kinds checks at once what it can, and a
Refusal there refuses the goal with its kind; item_manipulation accepts
every goal and reports in its result.

`run_guarded` is where the contracts' result rules are applied for every
goal that runs a sequence, through the `Ends` of the member's contract:
item_perception and brain_model end a goal by its status (`STATUS_ENDS`),
item_manipulation in its result (`RESULT_ENDS`). A check that passed at
the decision is made again when the goal starts, and a goal that fails it
then fails with the same kind. The fields the contract says are zero or
empty for a goal with no result of its own are the `zero` dict each
handler passes; and `action_time` is the time from the goal's start to its
end, whatever the outcome. It is also where each goal leaves its one line
in the log, so a search that found nothing says there what it looked at.

The planning scene of the motion planner follows the held flags of
`State`. One method of `Brain` stands for each change of a flag (`hold`)
and writes the flag, then the scene; `make_scene_follow_holds` makes the
scene equal to the flags. Each
item a gripper holds has its object, `brain/<item_id>`, attached to the
grasp link of its arm, and no other object of the brain stays. With the
planner slot vacant it does nothing.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Mapping, Optional, Protocol, Sequence

from .bindings import Bindings
from .limb_state import LimbStateStore
from .limbs import parse_limbs
from .manipulators import Manipulator
from .perception.camera import CameraModel
from .perception.frames import FrameStore
from .perception.geometry import MOUNTS_VACANT, camera_pose, learn_camera_pose, learn_intrinsics
from .perception.none import NoneDetector
from .perception.perceiver import Look, Perceiver
from .perception.wrist import WristFrames, parse_wrist_cameras
from .models import Models
from .ports import Cancelled, Detector, ExpectedFailure, FailureKind, GoalFailure, Gripper, ModelSource, Refusal, Role, Senses, Vec3
from .robot import ObjectOperation, PlannerModules, Robot, brain_object_id, item_object, item_of_object
from .sequencer import SHUTTING_DOWN, Job, Sequencer
from .state import State, new_run_token

logger = logging.getLogger(__name__)

Body = Callable[[Job], Awaitable[dict]]
# A check of a goal before it is accepted: the brain and the goal's request
# data in, the Refusal that refuses the goal raised.
Check = Callable[["Brain", object], None]

ABORTED_BY_SHUTDOWN = "aborted: the node is shutting down"

class Ends(Protocol):
    """How the contract of a member ends a goal. `fields` and `zero` carry
    `action_time`."""

    async def complete(self, ctx, fields: dict) -> None: ...

    async def fail(self, ctx, failure: GoalFailure, zero: dict) -> None: ...

    async def cancel(self, ctx, cancelled: Cancelled, zero: dict) -> None: ...


class EndsByStatus:
    """How item_perception and brain_model end a goal: by its status. A
    success completes with its fields. A refusal or a failure once the goal
    runs fails it, with its kind and no result. A stop of any cause, the
    goal's own cancel, an abort or a shutdown, ends it cancelled, with the
    reason and the zero fields."""

    async def complete(self, ctx, fields: dict) -> None:
        await ctx.complete(**fields)

    async def fail(self, ctx, failure: GoalFailure, zero: dict) -> None:
        await ctx.fail(failure.kind.value, failure.message)

    async def cancel(self, ctx, cancelled: Cancelled, zero: dict) -> None:
        await ctx.complete_cancelled(cancelled.reason, **zero)


class EndsInResult:
    """How item_manipulation ends a goal: in its result, whose `success`
    and `message` say how the goal went. A refusal or a failure completes
    with success false, its message and the zero fields. A stop by the
    goal's own cancel ends it cancelled; any other stop, an abort or a
    shutdown, completes it with success false and the reason."""

    async def complete(self, ctx, fields: dict) -> None:
        await ctx.complete(success=True, message="", **fields)

    async def fail(self, ctx, failure: GoalFailure, zero: dict) -> None:
        await ctx.complete(success=False, message=failure.message, **zero)

    async def cancel(self, ctx, cancelled: Cancelled, zero: dict) -> None:
        if cancelled.signalled:
            await ctx.complete_cancelled(cancelled.reason, success=False, message=cancelled.reason, **zero)
            return
        await ctx.complete(success=False, message=cancelled.reason, **zero)


STATUS_ENDS = EndsByStatus()
RESULT_ENDS = EndsInResult()


def decider(action_module, brain: "Brain", check: Optional[Check]):
    """The decider the generated module `action_module` takes: it accepts a
    goal `check` passes, and refuses a goal `check` raises a Refusal for,
    with its kind. Without a `check` it accepts every goal. A check that
    breaks refuses the goal with `attempt_failed`, and its log line carries
    the traceback."""
    name = action_module.ACTION_NAME

    def decide(request):
        if check is None:
            return action_module.GoalDecision.accept()
        try:
            check(brain, request.data)
        except Refusal as refusal:
            logger.info("%s refused: %s", name, refusal.message)
            return action_module.GoalDecision.refuse(refusal.kind.value, refusal.message)
        except Exception as error:
            message = f"{name} could not be decided: {error!r}"
            logger.error("%s", message, exc_info=error)
            return action_module.GoalDecision.refuse(FailureKind.ATTEMPT_FAILED.value, message)
        return action_module.GoalDecision.accept()

    return decide


class Brain:
    def __init__(
        self,
        params,
        node_runner,
        bindings: Bindings,
        source: ModelSource,
        *,
        now: Optional[Callable[[], int]] = None,
        run_token: Optional[str] = None,
    ) -> None:
        self.params = params
        self.node_runner = node_runner
        self.bindings = bindings
        limbs = parse_limbs(params.limbs)
        self.state = State(limbs, new_run_token() if run_token is None else run_token)
        self.robot = Robot(
            node_runner, bindings.move_arm, bindings.move_gripper, bindings.check_arm_move,
            planner=PlannerModules.from_bindings(bindings),
        )
        # The robot's measured limbs, which a manipulator reads.
        self.limb_states = LimbStateStore()
        # The perceiver alone holds the frames and the camera model: the
        # geometry learners give it each new camera model.
        self.perceiver = Perceiver(FrameStore(), CameraModel())
        # The wrist cameras, by arm: a brain whose parameters name none has
        # no wrist pictures.
        self.wrists = WristFrames(parse_wrist_cameras(getattr(params, "wrist_cameras", ""), [limb.arm for limb in limbs]))
        self.senses = Senses(self.robot, self.limb_states, self.perceiver, self.wrists, self._camera_pose)
        self.sequencer = Sequencer(stopper=self._stop_lane)
        self.models = Models(source, self.sequencer, self.senses)
        self._now = now or bindings.clock.now_ns

    def now(self) -> int:
        return self._now()

    def elapsed_s(self, started_ns: int) -> float:
        """The time from `started_ns` to now, in seconds."""
        return max(0.0, (self.now() - started_ns) / 1e9)

    async def start(self) -> None:
        """Sets the model of each role the parameters name, once. A
        ValueError names what stops the node."""
        await self.models.start({
            Role.PERCEPTION: self.params.perception_model,
            Role.MANIPULATION: self.params.manipulation_model,
        })

    def background(self, token) -> list[asyncio.Task]:
        """The loops that run beside the action loops: the camera follower,
        the one asking the camera where its pixels point, the one asking
        the robot where the camera stands, the follower of the robot's
        measured limbs, and the follower of the wrist cameras."""
        b = self.bindings
        return [
            asyncio.create_task(
                self.perceiver.frames.run(self.node_runner, token, b.video_stream, b.depth_stream, b.depth_stream_info)
            ),
            asyncio.create_task(
                learn_intrinsics(self.node_runner, token, self.perceiver, b.get_color_intrinsics, b.get_depth_intrinsics)
            ),
            asyncio.create_task(
                learn_camera_pose(self.node_runner, token, self.perceiver, self.params.camera_name, b.get_camera_poses)
            ),
            asyncio.create_task(self.limb_states.run(self.node_runner, token, b.limb_states)),
            asyncio.create_task(self.wrists.run(self.node_runner, token, b.wrist_video_stream)),
        ]

    async def _camera_pose(self, camera_name: str):
        """Where `camera_name` stands on the robot now, (position,
        orientation) in the robot frame from camera_mounts; a Refusal when
        the slot is vacant or the robot cannot say."""
        module = self.bindings.get_camera_poses
        producer = module.bound_producer(self.node_runner)
        if producer is None:
            raise Refusal(MOUNTS_VACANT)
        return await camera_pose(self.node_runner, producer, camera_name, module)

    async def shutdown(self) -> None:
        """The shutdown hook: refuse every new goal first, so none finds a
        lane the stops below just freed, then stop what runs while the
        messenger is still connected, so in-flight moves end as cancelled
        rather than abandoned. The manipulation lane runs the goals of
        item_manipulation, which names a stop of the brain an abort; a
        search and a switch end cancelled, the reason the shutdown
        itself."""
        self.sequencer.close()
        await self.sequencer.stop(Role.MANIPULATION, ABORTED_BY_SHUTDOWN)
        await self.sequencer.stop(Role.PERCEPTION, SHUTTING_DOWN)
        await self.sequencer.stop_laneless(SHUTTING_DOWN)
        await self.robot.stop()

    def check_search(self) -> None:
        """Raises the Refusal of a search that cannot start now, before it
        is accepted: `precondition_failed` while the node shuts down or a
        search runs; the refusal of the perception model while it cannot
        search (`refuse_unless_searchable`); `resource_unavailable` while
        the frames or the camera model cannot serve it."""
        self.sequencer.check(Role.PERCEPTION)
        self.refuse_unless_searchable(self.models.current(Role.PERCEPTION))
        self.perceiver.refuse_unless_ready()

    def starting(self, role: Role) -> str:
        """Why `role` cannot serve a goal yet while the brain starts it, as
        get_model of brain_model gives it, or "": once the role can serve a
        goal, and when it does not become able to (its model is "none", its
        model could not load, a slot that it reads is vacant, or a source
        gave what the brain cannot read). Perception waits for its model,
        then for the frames and the camera model, unless one of those does
        not become able to serve; manipulation for its manipulator."""
        if role is Role.MANIPULATION:
            return self.models.current(Role.MANIPULATION).starting()
        sources_starting = self.perceiver.starting()
        if self.perceiver.why_unavailable() and not sources_starting:
            return ""
        detector = self.models.current(Role.PERCEPTION)
        detector_starting = detector.starting()
        if detector_starting:
            return f"no perception source: {detector_starting}"
        if detector.why_unavailable():
            return ""
        return sources_starting

    def refuse_unless_searchable(self, detector: Detector) -> None:
        """Raises the Refusal of a search through `detector` while it cannot
        search: `precondition_failed` for the model "none", as brain_model
        asks of a role whose model is "none"; `resource_unavailable` for a
        model that loads or could not load, as item_perception asks when no
        perception source is available."""
        reason = detector.why_unavailable()
        if not reason:
            return
        kind = FailureKind.PRECONDITION_FAILED if isinstance(detector, NoneDetector) else FailureKind.RESOURCE_UNAVAILABLE
        raise Refusal(kind, f"no perception source: {reason}")

    async def look(self, job: Job, detector: Detector, phrases: Sequence[str], timeout_s: float) -> Look:
        """One look at the camera through `detector` for the search `job`
        runs, as `Perceiver.scan` gives it. The detector says first whether
        it can search, then the perceiver whether its frames and camera
        can; what the look saw goes on the job's notes for the log."""
        self.refuse_unless_searchable(detector)
        seen = await self.perceiver.scan(detector, phrases, job.cancel, timeout_s)
        job.notes.append(seen.describe(job.started_ns))
        return seen

    def manipulator_or_refuse(self, manipulator: Manipulator) -> Manipulator:
        """`manipulator`, or the refusal, `precondition_failed`, it gives
        while it cannot move."""
        reason = manipulator.why_unavailable()
        if reason:
            raise Refusal(FailureKind.PRECONDITION_FAILED, reason)
        return manipulator

    async def hold(self, gripper: Gripper, item_id: str, closed_at: Optional[Vec3]) -> None:
        """A grab held `item_id` in `gripper`: the held flag in `State`, then
        the planning scene made to follow it, the item's object moved from
        the table by what the grasp point moved since the jaws closed at
        `closed_at`."""
        self.state.set_held(gripper, item_id)
        await self.make_scene_follow_holds({item_id: closed_at} if closed_at is not None else None)

    async def make_scene_follow_holds(self, closed_at: Optional[Mapping[str, Vec3]] = None) -> None:
        """Makes the planning scene equal to the held flags, from what
        get_planning_scene says it holds:

        - an object of the brain that no gripper holds, free or attached, is
          detached and removed, and so is one attached to another arm than
          the one that holds its item;
        - an item a gripper holds whose object stands free has its object
          moved by what the grasp point of the arm moved since the jaws
          closed, where `closed_at` gives where the grasp point stood then,
          and attached to the arm's grasp link, the gripper's links its touch
          links;
        - an item a gripper holds with no object of its own, or whose
          `closed_at` is not given, gets its object at the hand, centred on
          the grasp point: the box of its shape, or a sphere for an item with
          none, attached the same way.

        Nothing happens with the planner slot vacant. A scene call that fails
        is logged and the next one is made: the held flags stay as they are,
        and the next call of this function corrects the scene."""
        if not self.robot.has_planner:
            return
        arms = await self.robot.describe_arms()
        scene = await self.robot.get_planning_scene()
        reason = arms.reason or scene.reason
        if reason:
            logger.warning("planning scene: not made to follow the holds: %s", reason)
            return
        holders = {gripper.held_item_id: gripper for gripper in self.state.grippers.values() if gripper.holding}
        attached = set()
        for found in scene.attached_objects:
            item_id = item_of_object(found.id)
            if item_id is None:
                continue
            holder = holders.get(item_id)
            arm = arms.arm(holder.arm) if holder is not None else None
            if arm is not None and found.link_name == arm.grasp_link:
                attached.add(item_id)
                continue
            await self._write_scene(f"detach {found.id} from {found.link_name}", self.robot.apply_attached_collision_object(found.id, found.link_name, (), "detach"))
            await self._write_scene(f"remove {found.id}", self.robot.apply_collision_objects([ObjectOperation.remove(found.id)]))
        free = {}
        for found in scene.objects:
            item_id = item_of_object(found.id)
            if item_id is None:
                continue
            if item_id in holders and item_id not in attached:
                free[item_id] = found
                continue
            await self._write_scene(f"remove {found.id}", self.robot.apply_collision_objects([ObjectOperation.remove(found.id)]))
        for item_id, gripper in holders.items():
            if item_id not in attached:
                await self._attach_held(item_id, gripper, arms.arm(gripper.arm), free.get(item_id), (closed_at or {}).get(item_id))

    async def _attach_held(self, item_id: str, gripper, arm, table_object, closed_at: Optional[Vec3]) -> None:
        """Puts the object of the held item `item_id` at the hand of
        `gripper` and attaches it: its object on the table moved by what the
        grasp point moved since `closed_at`, or a new one centred on the
        grasp point."""
        object_id = brain_object_id(item_id)
        if arm is None:
            logger.warning("planning scene: %s not attached: the motion planner does not describe arm '%s'", object_id, gripper.arm)
            return
        snapshot = self.limb_states.latest
        hand = snapshot.arms.get(gripper.arm) if snapshot is not None else None
        if hand is None:
            logger.warning("planning scene: %s not attached: limb_states gives no grasp point of arm '%s'", object_id, gripper.arm)
            return
        if table_object is not None and closed_at is not None:
            moved = tuple(at + now - then for at, now, then in zip(table_object.position, hand.position, closed_at))
            await self._write_scene(
                f"move {object_id} by what the grasp point moved since the jaws closed, to {_numbers(moved)}",
                self.robot.apply_collision_objects([ObjectOperation.move(object_id, moved, table_object.orientation)]),
            )
        else:
            item = self.state.items.get(item_id)
            add = item_object(item_id, item.shape if item is not None else None, hand.position)
            await self._write_scene(f"add {object_id} at the hand of arm '{gripper.arm}', {_numbers(hand.position)}", self.robot.apply_collision_objects([add]))
        await self._write_scene(
            f"attach {object_id} to {arm.grasp_link}",
            self.robot.apply_attached_collision_object(object_id, arm.grasp_link, arm.gripper_links, "attach"),
        )

    async def _write_scene(self, what: str, write: Awaitable[str]) -> bool:
        """Makes one write of the planning scene and logs it, or logs why it
        failed. Returns whether it was done."""
        reason = await write
        if reason:
            logger.warning("planning scene: %s failed: %s", what, reason)
            return False
        logger.info("planning scene: %s", what)
        return True

    async def _stop_lane(self, lane: Role) -> None:
        # The manipulation model is the one the running goal read: the lane
        # rule keeps a switch from changing it while the goal runs.
        if lane == Role.MANIPULATION:
            await self.models.current(Role.MANIPULATION).stop()
            await self.robot.stop()

    async def run_guarded(
        self, ctx, lane: Optional[Role], action: str, body: Body, zero: dict, *, ends: Ends, asked: str = ""
    ) -> None:
        """Admits the goal into its lane, or as a lane-less job when `lane`
        is none, runs `body` under a cancel watcher, and ends the goal the
        way its contract wants, through `ends`. Before it ends, the goal
        leaves its line in the log: the action and `asked`, what the goal
        asked for in a few words, how the goal ended and after how long,
        then the notes the body put on its job."""
        started = self.now()
        goal = f"{action} {asked}" if asked else action
        try:
            job = self.sequencer.start(lane, action, started)
        except Refusal as refusal:
            elapsed = self._log_end(goal, started, "refused", refusal.message)
            await ends.fail(ctx, refusal, {**zero, "action_time": elapsed})
            return
        watcher = asyncio.create_task(self._watch_cancel(ctx, job))
        try:
            fields = await body(job)
            job.cancel.check()
        except Refusal as refusal:
            elapsed = self._log_end(goal, started, "refused", refusal.message, job.notes)
            await ends.fail(ctx, refusal, {**zero, "action_time": elapsed})
        except ExpectedFailure as failure:
            elapsed = self._log_end(goal, started, "failed", failure.message, job.notes, level=logging.WARNING)
            await ends.fail(ctx, failure, {**zero, "action_time": elapsed})
        except Cancelled as cancelled:
            elapsed = self._log_end(goal, started, "stopped", cancelled.reason, job.notes)
            await ends.cancel(ctx, cancelled, {**zero, "action_time": elapsed})
        except Exception as error:
            fault = GoalFailure(FailureKind.ATTEMPT_FAILED, f"{action} failed: {error!r}")
            elapsed = self._log_end(goal, started, "failed", repr(error), job.notes, error=error)
            await ends.fail(ctx, fault, {**zero, "action_time": elapsed})
        else:
            elapsed = self._log_end(goal, started, "succeeded", "", job.notes)
            await ends.complete(ctx, {**fields, "action_time": elapsed})
        finally:
            watcher.cancel()
            self.sequencer.finish(job)

    def _log_end(
        self,
        goal: str,
        started_ns: int,
        ended: str,
        reason: str,
        notes: Sequence[str] = (),
        *,
        level: int = logging.INFO,
        error: Optional[Exception] = None,
    ) -> float:
        """Logs the one line a goal leaves: `goal`, how it `ended` and after
        how long, the reason when there is one, then `notes`. Returns that
        time, so the line and the result read the clock once. A fault,
        `error`, logs at error level with its traceback."""
        elapsed = self.elapsed_s(started_ns)
        line = f"{goal} {ended} after {elapsed:.2f} s"
        if reason:
            line += f": {reason}"
        line = "; ".join([line, *notes])
        if error is None:
            logger.log(level, "%s", line)
        else:
            logger.error("%s", line, exc_info=error)
        return elapsed

    async def _watch_cancel(self, ctx, job: Job) -> None:
        await ctx.cancel_signal()
        await self.sequencer.cancel(job, ctx.cancel_reason(), signalled=True)


def _numbers(values: Sequence[float]) -> str:
    """A point for the log: in brackets, with 3 decimals each."""
    return f"[{', '.join(f'{value:.3f}' for value in values)}]"
