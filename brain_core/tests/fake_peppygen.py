"""The generated modules a brain node passes in, faked with the fields and
the failure kinds the contracts give them, so the core runs in plain tests
with no peppygen, no router and no robot.

Each exposed action is a `FakeAction` named as the generated module is:
its result's fields, which the fake goal context checks every end against,
its `FailureKind` and `Failure`, which refuse an undeclared kind as the
generated ones do, and its `GoalDecision`. `ActionHandle.expose` gives an
action that never receives a goal, and `handle_next_request` a service
that never receives a request, for a node that boots and stops. The
limb_motion clients are scripted by the test (`FakeLimbAction`), and a
consumed service answers what the test's `poll` gives
(`consumed_service`).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum, IntEnum
from types import SimpleNamespace
from typing import Callable, Optional

from brain_core.bindings import Bindings

# The kinds of peppy, which every member declares first.
PEPPY_KINDS = ("attempt_failed", "timeout", "precondition_failed", "resource_unavailable", "rejected")


def failure_types(member: str, *contract_kinds: str):
    """The `FailureKind` and the `Failure` of a member, as its generated
    module has them: the kinds of peppy, then `contract_kinds`, each member
    the kind in upper case; a Failure of any other kind raises
    ValueError."""
    failure_kind = Enum("FailureKind", {kind.upper(): kind for kind in (*PEPPY_KINDS, *contract_kinds)}, type=str)

    class Failure(Exception):
        def __init__(self, kind, message: str) -> None:
            try:
                parsed = failure_kind(kind)
            except ValueError:
                raise ValueError(f"{kind!r} is not a failure kind of {member}") from None
            super().__init__(parsed.value, message)
            self.kind = parsed
            self.message = message

        def __eq__(self, other: object) -> bool:
            if not isinstance(other, Failure):
                return NotImplemented
            return self.kind is other.kind and self.message == other.message

        def __hash__(self) -> int:
            return hash((self.kind, self.message))

        def __repr__(self) -> str:
            return f"Failure({self.kind.value!r}, {self.message!r})"

    return failure_kind, Failure


class _NoGoals:
    async def handle_goal_next_request(self, decide):
        await asyncio.Event().wait()


async def _expose_no_goals(node_runner) -> _NoGoals:
    return _NoGoals()


class FakeAction:
    """A generated exposed action module: `ACTION_NAME`, the fields of its
    result, `FailureKind` and `Failure`, `GoalDecision`, and an
    `ActionHandle` that exposes an action receiving no goal."""

    def __init__(self, name: str, result_fields: tuple[str, ...], *contract_kinds: str) -> None:
        self.ACTION_NAME = name
        self.RESULT_FIELDS = frozenset(result_fields)
        self.FailureKind, self.Failure = failure_types(name, *contract_kinds)
        failure = self.Failure

        class GoalDecision:
            def __init__(self, refusal) -> None:
                self.response = None
                self.refusal = refusal

            @staticmethod
            def accept() -> "GoalDecision":
                return GoalDecision(None)

            @staticmethod
            def refuse(kind, message: str) -> "GoalDecision":
                return GoalDecision(failure(kind, message))

        self.GoalDecision = GoalDecision
        self.ActionHandle = SimpleNamespace(expose=_expose_no_goals)


IMAGE_FIELDS = ("camera", "image_width", "image_height", "frame_timestamp", "action_time")

# item_perception:v1: no success field, and the kind `not_found`, which
# identify_item lists.
scan_items = FakeAction("scan_items", ("item_ids", "labels", "positions", "confidences", "regions", *IMAGE_FIELDS))
identify_item = FakeAction(
    "identify_item", ("item_id", "label", "position", "orientation", "confidence", "region", *IMAGE_FIELDS), "not_found"
)

# item_manipulation:v1: success and message in each result.
grab_item = FakeAction("grab_item", ("success", "message", "gripper_name", "item_id", "action_time"))
drop_item = FakeAction("drop_item", ("success", "message", "gripper_name", "item_id", "action_time"))
place_item = FakeAction("place_item", ("success", "message", "gripper_name", "item_id", "final_position", "action_time"))
abort = FakeAction("abort", ("success", "message", "aborted_action", "action_time"))

# brain_model:v1: no success field, the kinds of peppy alone.
switch_model = FakeAction("switch_model", ("role", "model", "action_time"))


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


class get_model:
    SERVICE_NAME = "get_model"

    @dataclass
    class Response:
        roles: list[str]
        models: list[str]

    @staticmethod
    async def handle_next_request(node_runner, handler):
        await asyncio.Event().wait()


class list_models:
    SERVICE_NAME = "list_models"

    @dataclass
    class Response:
        roles: list[str]
        models: list[str]

    @staticmethod
    async def handle_next_request(node_runner, handler):
        await asyncio.Event().wait()


class ResultStatus(IntEnum):
    COMPLETED = 0
    CANCELLED = 1
    ABANDONED = 2
    EXPIRED = 3
    FAILED = 4


@dataclass
class MoveArmRequest:
    arm_name: str
    position: list[float]
    orientation: list[float]
    duration_s: float
    plan_position_tolerance_m: float
    plan_orientation_tolerance_rad: float
    settle_position_tolerance_m: float
    settle_orientation_tolerance_rad: float
    settle_timeout_s: float


@dataclass
class MoveGripperRequest:
    gripper_name: str
    opening: float
    max_effort: float
    settle_timeout_s: float


# The kinds of limb_motion:v1 that each of its moves lists beside the kinds
# of peppy.
LIMB_MOTION_KINDS = {
    "move_arm": ("unreachable", "leader_streaming"),
    "move_gripper": ("leader_streaming",),
}


class FiredGoal:
    """One goal the brain fired at the backbone, as its handle: accepted,
    or refused with `refusal`; the test completes it, cancels it, fails it,
    ends it another way, or makes its result fail."""

    def __init__(self, failure, request, feedback_bound, *, refusal=None) -> None:
        self._failure = failure
        self.request = request
        self.feedback_bound = feedback_bound
        self.accepted = refusal is None
        self.refusal = refusal
        self.result_timeouts: list[float] = []
        self.cancel_asked = asyncio.Event()
        self._result: asyncio.Future = asyncio.get_running_loop().create_future()

    async def get_result(self, timeout: float):
        self.result_timeouts.append(timeout)
        return await self._result

    async def cancel_goal(self, timeout: float):
        self.cancel_asked.set()

    def complete(self, data) -> None:
        self._end(ResultStatus.COMPLETED, data=data)

    def complete_cancelled(self, reason: str, data) -> None:
        self._end(ResultStatus.CANCELLED, data=data, cancel_reason=reason)

    def fail(self, kind: str, message: str) -> None:
        self._end(ResultStatus.FAILED, failure=self._failure(kind, message))

    def end(self, status: ResultStatus) -> None:
        self._end(status)

    def lose_result(self, error: Exception) -> None:
        self._result.set_exception(error)

    def _end(self, status: ResultStatus, *, data=None, failure=None, cancel_reason: Optional[str] = None) -> None:
        self._result.set_result(SimpleNamespace(status=status, data=data, failure=failure, cancel_reason=cancel_reason))


class FakeLimbAction:
    """A generated limb_motion action client: every goal the brain fires is
    a `FiredGoal` the test takes with `next_goal`. `refuse_next` makes the
    backbone refuse the next goal with a kind and a message,
    `fail_next_send` makes the next send fail before it reaches the
    backbone."""

    def __init__(self, name: str, request_type) -> None:
        self.TARGET_ACTION_NAME = name
        self.GoalRequest = request_type
        self.ResultStatus = ResultStatus
        self.FailureKind, self.Failure = failure_types(name, *LIMB_MOTION_KINDS[name])
        self.ActionHandle = SimpleNamespace(fire_goal=self._fire_goal)
        self._fired: asyncio.Queue = asyncio.Queue()
        self._refusal = None
        self._send_error: Optional[Exception] = None

    def bound_producer(self, node_runner) -> str:
        return "backbone"

    def refuse_next(self, kind: str, message: str) -> None:
        self._refusal = self.Failure(kind, message)

    def fail_next_send(self, error: Exception) -> None:
        self._send_error = error

    async def next_goal(self) -> FiredGoal:
        return await self._fired.get()

    async def _fire_goal(self, node_runner, producer, request, timeout, feedback_bound=None) -> FiredGoal:
        if self._send_error is not None:
            error, self._send_error = self._send_error, None
            raise error
        if self._refusal is not None:
            refusal, self._refusal = self._refusal, None
            return FiredGoal(self.Failure, request, feedback_bound, refusal=refusal)
        goal = FiredGoal(self.Failure, request, feedback_bound)
        await self._fired.put(goal)
        return goal


def consumed_service(name: str, poll: Callable, *, producer: str = "producer") -> SimpleNamespace:
    """A generated consumed service module whose slot is bound to
    `producer`: `poll(node_runner, producer, timeout)` answers each poll,
    and `Failure`, the kinds of peppy, is the error reply it raises."""
    _, failure = failure_types(name)
    return SimpleNamespace(SERVICE_NAME=name, bound_producer=lambda node_runner: producer, poll=poll, Failure=failure)


def answering(name: str, *answers) -> SimpleNamespace:
    """A consumed service bound to one producer that answers each poll with
    the next of `answers`, the last one again once they run out: a
    response's data, or an exception to raise. A callable answer is called
    with the service's `Failure` and gives the error reply to raise."""
    pending = list(answers)
    service = consumed_service(name, None)

    async def poll(node_runner, producer, timeout):
        answer = pending.pop(0) if len(pending) > 1 else pending[0]
        if callable(answer):
            answer = answer(service.Failure)
        if isinstance(answer, BaseException):
            raise answer
        return SimpleNamespace(data=answer)

    service.poll = poll
    return service


def error_reply(kind: str, message: str) -> Callable:
    """An answer of `answering`: the service's error reply of `kind`."""
    return lambda failure: failure(kind, message)


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
        switch_model=switch_model,
        get_model=get_model,
        list_models=list_models,
    )
    modules.update(overrides)
    return Bindings(**modules)
