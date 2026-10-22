"""The brain assembled: the core parts, the model of each role from the
node's model source, and the one way every action runs a sequence and
completes its goal.

`run_guarded` is where the contracts' result rules are applied for every
goal that runs a sequence: a refusal or a failure completes with success
false and the message; a cancel by the caller completes as cancelled; an
abort completes with success false and "aborted: <reason>"; the fields the
contract says are zero or empty on failure are the `zero` dict each
handler passes; and `action_time` is the time from the goal's arrival to
its completion, whatever the outcome. It is also where each goal leaves
its one line in the log, so a search that found nothing says there what it
looked at.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Optional, Sequence

from .bindings import Bindings
from .limb_state import LimbStateStore
from .limbs import parse_limbs
from .perception.camera import CameraModel
from .perception.frames import FrameStore
from .perception.geometry import learn_camera_pose, learn_intrinsics
from .perception.perceiver import Look, Perceiver
from .models import Models
from .ports import Cancelled, Detector, ExpectedFailure, Manipulator, ModelSource, Refusal, Role
from .robot import Robot
from .sequencer import Job, Sequencer
from .state import State, new_run_token

logger = logging.getLogger(__name__)

Body = Callable[[Job], Awaitable[dict]]

ABORTED_BY_SHUTDOWN = "aborted: the node is shutting down"


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
        self.robot = Robot(
            node_runner, bindings.move_arm, bindings.move_gripper, bindings.check_arm_move, bindings.standard_qos
        )
        # The robot's measured limbs, which a manipulator reads.
        self.limb_states = LimbStateStore()
        # The perceiver alone holds the frames and the camera model: the
        # geometry learners give it each new camera model.
        self.perceiver = Perceiver(FrameStore(), CameraModel())
        self.sequencer = Sequencer(stopper=self._stop_lane)
        self.models = Models(source, self.sequencer, self.robot, self.limb_states)
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
        the robot where the camera stands, and the follower of the robot's
        measured limbs."""
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
        ]

    async def shutdown(self) -> None:
        """The shutdown hook: refuse every new goal first, so none finds a
        lane the stops below just freed, then stop what runs while the
        messenger is still connected, so in-flight moves end as cancelled
        rather than abandoned."""
        self.sequencer.close()
        await self.sequencer.stop(Role.MANIPULATION, ABORTED_BY_SHUTDOWN)
        await self.sequencer.stop(Role.PERCEPTION, ABORTED_BY_SHUTDOWN)
        await self.sequencer.stop_laneless(ABORTED_BY_SHUTDOWN)
        await self.robot.stop()

    async def look(self, job: Job, detector: Detector, phrases: Sequence[str], timeout_s: float) -> Look:
        """One look at the camera through `detector` for the search `job`
        runs, as `Perceiver.scan` gives it. The detector says first whether
        it can search, then the perceiver whether its frames and camera
        can; what the look saw goes on the job's notes for the log."""
        reason = detector.why_unavailable()
        if reason:
            raise Refusal(f"no perception source: {reason}")
        seen = await self.perceiver.scan(detector, phrases, job.cancel, timeout_s)
        job.notes.append(seen.describe(job.started_ns))
        return seen

    def manipulator_or_refuse(self, manipulator: Manipulator) -> Manipulator:
        """`manipulator`, or the refusal it gives while it cannot move."""
        reason = manipulator.why_unavailable()
        if reason:
            raise Refusal(reason)
        return manipulator

    async def _stop_lane(self, lane: Role) -> None:
        # The manipulation model is the one the running goal read: the lane
        # rule keeps a switch from changing it while the goal runs.
        if lane == Role.MANIPULATION:
            await self.models.current(Role.MANIPULATION).stop()
            await self.robot.stop()

    async def run_guarded(self, ctx, lane: Optional[Role], action: str, body: Body, zero: dict, asked: str = "") -> None:
        """Admits the goal into its lane, or as a lane-less job when `lane`
        is none, runs `body` under a cancel watcher, and completes the goal
        the way the contract wants. Before it completes, the goal leaves its
        line in the log: the action and `asked`, what the goal asked for in
        a few words, how the goal ended and after how long, then the notes
        the body put on its job."""
        started = self.now()
        goal = f"{action} {asked}" if asked else action
        try:
            job = self.sequencer.start(lane, action, started)
        except Refusal as refusal:
            await self._end(ctx, goal, started, "refused", refusal.message, zero)
            return
        watcher = asyncio.create_task(self._watch_cancel(ctx, job))
        try:
            fields = await body(job)
            job.cancel.check()
        except Refusal as refusal:
            await self._end(ctx, goal, started, "refused", refusal.message, zero, job.notes)
        except ExpectedFailure as failure:
            await self._end(ctx, goal, started, "failed", failure.message, zero, job.notes, level=logging.WARNING)
        except Cancelled as cancelled:
            await self._end(ctx, goal, started, "stopped", cancelled.reason, zero, job.notes, by_caller=cancelled.by_caller)
        except Exception as error:
            await self._end(
                ctx, goal, started, "failed", f"{action} failed: {error!r}", zero, job.notes, reason=repr(error), error=error
            )
        else:
            await self._end(ctx, goal, started, "succeeded", "", fields, job.notes, success=True)
        finally:
            watcher.cancel()
            self.sequencer.finish(job)

    async def _end(
        self,
        ctx,
        goal: str,
        started_ns: int,
        ended: str,
        message: str,
        fields: dict,
        notes: Sequence[str] = (),
        *,
        success: bool = False,
        by_caller: bool = False,
        reason: Optional[str] = None,
        level: int = logging.INFO,
        error: Optional[Exception] = None,
    ) -> None:
        """Logs the goal's line and completes it, both with one reading of
        the time it took. A cancel by the goal's own caller completes as
        cancelled."""
        elapsed = self.elapsed_s(started_ns)
        self._log_end(goal, ended, elapsed, message if reason is None else reason, notes, level, error)
        complete = ctx.complete_cancelled if by_caller else ctx.complete
        await complete(success=success, message=message, action_time=elapsed, **fields)

    def _log_end(
        self, goal: str, ended: str, elapsed_s: float, reason: str, notes: Sequence[str], level: int, error: Optional[Exception]
    ) -> None:
        """The one line a goal leaves in the log: `goal`, how it `ended`
        and after how long, the reason when there is one, then `notes`. A
        fault, `error`, logs at error level with its traceback."""
        line = f"{goal} {ended} after {elapsed_s:.2f} s"
        if reason:
            line += f": {reason}"
        line = "; ".join([line, *notes])
        if error is None:
            logger.log(level, "%s", line)
        else:
            logger.error("%s", line, exc_info=error)

    async def _watch_cancel(self, ctx, job: Job) -> None:
        await ctx.cancel_signal()
        await self.sequencer.cancel(job, "cancelled by the caller", by_caller=True)
