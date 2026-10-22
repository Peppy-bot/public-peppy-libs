"""The only part of the brain that talks to the backbone. One call per
limb_motion move, each returning a plain outcome, and the goals in flight
remembered so a stop can cancel them whatever backend asked for them.
`check_arm_move` asks whether a move has a plan and moves nothing, so a
stop has nothing of it to cancel.

The backbone plans and governs every move, so nothing here checks
collisions or limits: a target the backbone cannot reach comes back as a
refused goal, which the caller reports in its own result. A move that
gives no result within `result_timeout_s` is cancelled and reported as
having given none, so the arm does not keep going unwatched.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .ports import Quat, Vec3

GOAL_TIMEOUT_S = 5.0
RESULT_TIMEOUT_S = 120.0
CANCEL_TIMEOUT_S = 3.0
CHECK_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class MoveResult:
    success: bool
    message: str
    final_position: Optional[Vec3] = None
    final_orientation: Optional[Quat] = None


@dataclass(frozen=True)
class GripResult:
    success: bool
    message: str
    final_opening: Optional[float] = None


@dataclass(frozen=True)
class MoveCheck:
    """check_arm_move's answer: whether a move_arm goal with the same
    fields has a plan from where the arm is commanded now, the backbone's
    words when it has none, and the time the move would take."""

    success: bool
    message: str
    duration_s: float = 0.0


@dataclass(frozen=True)
class Ended:
    """How one limb_motion goal ended: success and message as the
    contract's result carries them, a refusal, a cancel and a lost result
    included, and the result's data when the backbone sent any."""

    success: bool
    message: str
    data: object = None


class Robot:
    """`move_arm`, `move_gripper` and `check_arm_move` are the node's
    generated limb_motion modules, and `qos` the QoS of a goal's
    feedback."""

    def __init__(
        self, node_runner, move_arm, move_gripper, check_arm_move, qos, *, result_timeout_s: float = RESULT_TIMEOUT_S
    ) -> None:
        self._node_runner = node_runner
        self._move_arm = move_arm
        self._move_gripper = move_gripper
        self._check_arm_move = check_arm_move
        self._qos = qos
        self._result_timeout_s = result_timeout_s
        self._live: set = set()

    async def move_arm(
        self,
        arm: str,
        position: Vec3,
        orientation: Quat,
        *,
        duration_s: float = 0.0,
        plan_position_tolerance_m: float = 0.0,
        plan_orientation_tolerance_rad: float = 0.0,
    ) -> MoveResult:
        """Plans and runs one grasp-point move of `arm`, robot frame, and
        waits for it to end. Zero tolerances and duration take the
        backbone's defaults."""
        request = self._move_arm.GoalRequest(
            **_arm_move_fields(arm, position, orientation, duration_s, plan_position_tolerance_m, plan_orientation_tolerance_rad)
        )
        ended = await self._run(self._move_arm, request)
        data = ended.data
        return MoveResult(
            ended.success,
            ended.message,
            _vec3(data.final_position) if data is not None else None,
            _quat(data.final_orientation) if data is not None else None,
        )

    async def set_gripper(self, gripper: str, opening: float, *, max_effort: float = 0.0) -> GripResult:
        """Drives one gripper to an opening fraction, 0 closed to 1 open,
        and waits for it to end."""
        request = self._move_gripper.GoalRequest(gripper_name=gripper, opening=opening, max_effort=max_effort)
        ended = await self._run(self._move_gripper, request)
        data = ended.data
        return GripResult(ended.success, ended.message, float(data.final_opening) if data is not None else None)

    async def check_arm_move(
        self,
        arm: str,
        position: Vec3,
        orientation: Quat,
        *,
        duration_s: float = 0.0,
        plan_position_tolerance_m: float = 0.0,
        plan_orientation_tolerance_rad: float = 0.0,
    ) -> MoveCheck:
        """Whether `move_arm` with the same arguments has a plan from where
        the arm is commanded now. A check that gets no answer within
        `CHECK_TIMEOUT_S` is reported as having none, with the reason."""
        request = self._check_arm_move.Request(
            **_arm_move_fields(arm, position, orientation, duration_s, plan_position_tolerance_m, plan_orientation_tolerance_rad)
        )
        try:
            answer = await self._check_arm_move.poll(
                self._node_runner, self._check_arm_move.bound_producer(self._node_runner), request, CHECK_TIMEOUT_S
            )
        except Exception as error:
            return MoveCheck(False, f"check_arm_move gave no answer: {error!r}")
        data = answer.data
        return MoveCheck(bool(data.success), data.message, float(data.duration_s))

    @property
    def moves_in_flight(self) -> int:
        """How many goals the backbone has accepted and not yet ended."""
        return len(self._live)

    async def stop(self) -> None:
        """Cancels every move in flight. The waiting calls return with the
        cancelled outcome once the backbone has brought the limb to rest."""
        for handle in list(self._live):
            await _cancel(handle)

    async def _run(self, module, request) -> Ended:
        """Fires one goal of `module` at the bound producer, keeps its
        handle while it runs, and reads how it ended."""
        name = module.TARGET_ACTION_NAME
        try:
            handle = await module.ActionHandle.fire_goal(
                self._node_runner, module.bound_producer(self._node_runner), request, GOAL_TIMEOUT_S, self._qos
            )
        except Exception as error:
            return Ended(False, f"{name} could not be sent: {error!r}")
        if not handle.accepted:
            return Ended(False, handle.reason or f"{name} was refused by the backbone")
        self._live.add(handle)
        try:
            result = await handle.get_result(self._result_timeout_s)
        except Exception as error:
            await _cancel(handle)
            return Ended(False, f"{name} gave no result: {error!r}")
        finally:
            self._live.discard(handle)
        data = result.data
        if result.status == module.ResultStatus.COMPLETED and data is not None:
            return Ended(data.success, data.message, data)
        if result.status == module.ResultStatus.CANCELLED:
            message = data.message if data is not None else ""
            return Ended(False, message or f"{name} was cancelled", data)
        return Ended(False, f"{name} ended as {result.status.name.lower()}")


def _arm_move_fields(
    arm: str,
    position: Vec3,
    orientation: Quat,
    duration_s: float,
    plan_position_tolerance_m: float,
    plan_orientation_tolerance_rad: float,
) -> dict:
    """The fields a move_arm goal and a check_arm_move request share."""
    return dict(
        arm_name=arm,
        position=list(position),
        orientation=list(orientation),
        duration_s=duration_s,
        plan_position_tolerance_m=plan_position_tolerance_m,
        plan_orientation_tolerance_rad=plan_orientation_tolerance_rad,
    )


async def _cancel(handle) -> None:
    try:
        await handle.cancel_goal(CANCEL_TIMEOUT_S)
    except Exception:
        pass


def _vec3(values) -> Optional[Vec3]:
    if values is None or len(values) != 3:
        return None
    return (float(values[0]), float(values[1]), float(values[2]))


def _quat(values) -> Optional[Quat]:
    if values is None or len(values) != 4:
        return None
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
