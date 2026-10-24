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
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Optional, Protocol, Sequence

from .bindings import Bindings
from .limbs import parse_limbs
from .perception.camera import CameraModel
from .perception.frames import FrameStore
from .perception.geometry import learn_camera_pose, learn_intrinsics
from .perception.none import NoneDetector
from .perception.perceiver import Look, Perceiver
from .models import Models
from .ports import Cancelled, Detector, ExpectedFailure, FailureKind, GoalFailure, Manipulator, ModelSource, Refusal, Role
from .robot import Robot
from .sequencer import SHUTTING_DOWN, Job, Sequencer
from .state import State, new_run_token

logger = logging.getLogger(__name__)

Body = Callable[[Job], Awaitable[dict]]
# A check of a goal before it is accepted: the brain and the goal's request
# data in, the Refusal that refuses the goal raised.
Check = Callable[["Brain", object], None]

ABORTED_BY_SHUTDOWN = "aborted: the node is shutting down"

# The reason of a goal's own cancel for each cause its goal context gives,
# in peppy's words.
CANCEL_REASONS = {"caller": "cancelled by its caller", "sender_gone": "its sender is gone"}


def cancel_reason(cause: Optional[str]) -> str:
    """The reason of a goal's own cancel, from the cause its goal context
    gives; "cancelled" for a cause it does not name."""
    return CANCEL_REASONS.get(cause, "cancelled")


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
        self.state = State(parse_limbs(params.limbs), new_run_token() if run_token is None else run_token)
        self.robot = Robot(node_runner, bindings.move_arm, bindings.move_gripper)
        # The perceiver alone holds the frames and the camera model: the
        # geometry learners give it each new camera model.
        self.perceiver = Perceiver(FrameStore(), CameraModel())
        self.sequencer = Sequencer(stopper=self._stop_lane)
        self.models = Models(source, self.sequencer, self.robot)
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
        the one asking the camera where its pixels point, and the one
        asking the robot where the camera stands."""
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
        ]

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
        await self.sequencer.cancel(job, cancel_reason(ctx.cancel_cause()), signalled=True)
