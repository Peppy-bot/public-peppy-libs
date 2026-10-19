"""The generated modules a brain node passes in, faked with the fields the
contracts give them, so the core runs in plain tests with no peppygen, no
router and no robot.

Each exposed action is a class named as the generated module is, whose
`GoalContext.complete` takes the fields of the contract's result: the fake
goal context checks every completion against it. `ActionHandle.expose`
gives an action that never receives a goal, and `handle_next_request` a
service that never receives a request, for a node that boots and stops.
The limb_motion clients are scripted by the test (`FakeLimbAction`).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import IntEnum
from types import SimpleNamespace
from typing import Optional

from brain_core.bindings import Bindings


class _NoGoals:
    async def handle_goal_next_request(self, decide):
        await asyncio.Event().wait()


class _ExposedAction:
    class ActionHandle:
        @classmethod
        async def expose(cls, node_runner):
            return _NoGoals()

    class GoalDecision:
        @staticmethod
        def accept():
            return "accept"


class scan_items(_ExposedAction):
    class GoalContext:
        async def complete(
            self, *, success, message, item_ids, labels, positions, confidences, regions, camera, image_width,
            image_height, frame_timestamp, action_time,
        ): ...


class identify_item(_ExposedAction):
    class GoalContext:
        async def complete(
            self, *, success, message, item_id, label, position, orientation, confidence, region, camera, image_width,
            image_height, frame_timestamp, action_time,
        ): ...


class grab_item(_ExposedAction):
    class GoalContext:
        async def complete(self, *, success, message, gripper_name, item_id, action_time): ...


class drop_item(_ExposedAction):
    class GoalContext:
        async def complete(self, *, success, message, gripper_name, item_id, action_time): ...


class place_item(_ExposedAction):
    class GoalContext:
        async def complete(self, *, success, message, gripper_name, item_id, final_position, action_time): ...


class abort(_ExposedAction):
    class GoalContext:
        async def complete(self, *, success, message, aborted_action, action_time): ...


class get_state:
    SERVICE_NAME = "get_state"

    @dataclass
    class Response:
        current_action: str
        current_action_elapsed_s: float
        gripper_names: list[str]
        holding: list[bool]
        held_item_ids: list[str]

    @staticmethod
    async def handle_next_request(node_runner, handler):
        await asyncio.Event().wait()


class ResultStatus(IntEnum):
    COMPLETED = 0
    CANCELLED = 1
    ABANDONED = 2
    EXPIRED = 3


@dataclass
class MoveArmRequest:
    arm_name: str
    position: list[float]
    orientation: list[float]
    duration_s: float
    plan_position_tolerance_m: float
    plan_orientation_tolerance_rad: float


@dataclass
class MoveGripperRequest:
    gripper_name: str
    opening: float
    max_effort: float


class FiredGoal:
    """One goal the brain fired at the backbone, as its handle: accepted,
    or refused with `reason`; the test completes it, cancels it, or makes
    its result fail."""

    def __init__(self, request, qos, *, accepted: bool = True, reason: str = "") -> None:
        self.request = request
        self.qos = qos
        self.accepted = accepted
        self.reason = reason
        self.result_timeouts: list[float] = []
        self.cancel_asked = asyncio.Event()
        self._result: asyncio.Future = asyncio.get_running_loop().create_future()

    async def get_result(self, timeout: float):
        self.result_timeouts.append(timeout)
        return await self._result

    async def cancel_goal(self, timeout: float):
        self.cancel_asked.set()

    def complete(self, data) -> None:
        self._result.set_result(SimpleNamespace(status=ResultStatus.COMPLETED, data=data))

    def complete_cancelled(self, data) -> None:
        self._result.set_result(SimpleNamespace(status=ResultStatus.CANCELLED, data=data))

    def end(self, status: ResultStatus) -> None:
        self._result.set_result(SimpleNamespace(status=status, data=None))

    def lose_result(self, error: Exception) -> None:
        self._result.set_exception(error)


class FakeLimbAction:
    """A generated limb_motion action client: every goal the brain fires is
    a `FiredGoal` the test takes with `next_goal`. `refuse_next` makes the
    backbone refuse the next goal, `fail_next_send` makes the next send
    fail before it reaches the backbone."""

    def __init__(self, name: str, request_type) -> None:
        self.TARGET_ACTION_NAME = name
        self.GoalRequest = request_type
        self.ResultStatus = ResultStatus
        self.ActionHandle = SimpleNamespace(fire_goal=self._fire_goal)
        self._fired: asyncio.Queue = asyncio.Queue()
        self._refusal: Optional[str] = None
        self._send_error: Optional[Exception] = None

    def bound_producer(self, node_runner) -> str:
        return "backbone"

    def refuse_next(self, reason: str) -> None:
        self._refusal = reason

    def fail_next_send(self, error: Exception) -> None:
        self._send_error = error

    async def next_goal(self) -> FiredGoal:
        return await self._fired.get()

    async def _fire_goal(self, node_runner, producer, request, timeout, qos) -> FiredGoal:
        if self._send_error is not None:
            error, self._send_error = self._send_error, None
            raise error
        if self._refusal is not None:
            reason, self._refusal = self._refusal, None
            return FiredGoal(request, qos, accepted=False, reason=reason)
        goal = FiredGoal(request, qos)
        await self._fired.put(goal)
        return goal


class VacantSlot:
    """A consumed module whose slot the launcher left vacant."""

    @staticmethod
    def bound_producer(node_runner) -> None:
        return None


class FakeClock:
    """The node's clock: `now_ns` as the test sets it, and the init the
    node's setup awaits."""

    def __init__(self, now_ns: int = 0) -> None:
        self.now_ns_value = now_ns
        self.initialised_with = None

    def now_ns(self) -> int:
        return self.now_ns_value

    async def init(self, node_runner) -> None:
        self.initialised_with = node_runner


def fake_bindings(**overrides) -> Bindings:
    """Bindings to the fakes above: the camera, geometry and mounts slots
    vacant, limb_motion scripted. `overrides` replace any of them."""
    modules = dict(
        clock=FakeClock(),
        standard_qos="standard",
        move_arm=FakeLimbAction("move_arm", MoveArmRequest),
        move_gripper=FakeLimbAction("move_gripper", MoveGripperRequest),
        video_stream=VacantSlot,
        depth_stream=VacantSlot,
        depth_stream_info=VacantSlot,
        get_color_intrinsics=VacantSlot,
        get_depth_intrinsics=VacantSlot,
        get_camera_poses=VacantSlot,
        scan_items=scan_items,
        identify_item=identify_item,
        grab_item=grab_item,
        drop_item=drop_item,
        place_item=place_item,
        abort=abort,
        get_state=get_state,
    )
    modules.update(overrides)
    return Bindings(**modules)
