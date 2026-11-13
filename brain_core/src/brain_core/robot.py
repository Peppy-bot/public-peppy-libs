"""The only part of the brain that talks to the backbone and to the motion
planner. One call per limb_motion move, each returning how the move ended,
and the goals in flight remembered so a stop can cancel them whatever
backend asked for them. `check_arm_move` asks whether a move has a plan and
moves nothing, so a stop has nothing of it to cancel.

The motion planner is optional: `has_planner` says whether the brain
consumes it and the launcher bound its slot. Without it the brain moves by
move_arm lines alone.

The backbone plans and governs every move, so nothing here checks
collisions or limits: a target the backbone cannot reach comes back as a
failed or refused goal with its failure kind, `unreachable`, which the
caller reports in its own result. A move that gives no result within
`result_timeout_s` is cancelled and reported as lost, so the arm does not
keep going unwatched.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import StrEnum
from typing import Any, Optional, Sequence

from .ports import Quat, Vec3

GOAL_TIMEOUT_S = 5.0
# The longest a move may take, the longest duration and settle timeout the
# limb_motion contract admits together, and 5 s for its result to arrive.
RESULT_TIMEOUT_S = 125.0
CANCEL_TIMEOUT_S = 3.0
CHECK_TIMEOUT_S = 5.0


class MoveEnd(StrEnum):
    """How one limb_motion goal ended, as the robot proxy reads it."""

    # The move ran to its end: the result says where the limb stands.
    COMPLETED = "completed"
    # A cancel ended it, the brain's own or one of the stop service of the
    # backbone: the message is the reason of the cancel, and the result
    # says where the limb stopped.
    CANCELLED = "cancelled"
    # The backbone refused the goal: the failure kind and the message say
    # why.
    REFUSED = "refused"
    # The move started and gave no result: the failure kind and the
    # message say why.
    FAILED = "failed"
    # The goal could not be sent, gave no result in time, or ended
    # abandoned or expired: the message says which.
    LOST = "lost"


@dataclass(frozen=True)
class Moved:
    """How one limb_motion goal ended. `failure_kind` is the limb_motion
    kind of a refused or failed move, such as "unreachable" or
    "leader_streaming", and empty otherwise; `message` says why a move did
    not complete, and is empty for one that did."""

    end: MoveEnd
    message: str = ""
    failure_kind: str = ""

    @property
    def completed(self) -> bool:
        return self.end is MoveEnd.COMPLETED

    @property
    def cancelled(self) -> bool:
        """A cancel ended the move: the brain's stop, or the backbone's stop
        service."""
        return self.end is MoveEnd.CANCELLED

    @property
    def words(self) -> str:
        """Why the goal did not complete, with its failure kind first when
        it has one: "start_in_collision: <the producer's message>"."""
        return f"{self.failure_kind}: {self.message}" if self.failure_kind else self.message


@dataclass(frozen=True)
class MoveResult(Moved):
    """An arm move's end, and where the grasp point stood when it ended:
    for a completed or a cancelled move."""

    final_position: Optional[Vec3] = None
    final_orientation: Optional[Quat] = None


@dataclass(frozen=True)
class GripResult(Moved):
    """A gripper move's end, and how the jaws stood when it ended: for a
    completed or a cancelled move. `settled` is true when the jaws stood
    still before the settle timeout, false when the move completed at it."""

    settled: Optional[bool] = None
    final_opening: Optional[float] = None


@dataclass(frozen=True)
class Waypoint:
    """One state of a planned path: the arm's joint positions in its joint
    order of limb_state, and where its grasp point stands there."""

    joint_positions: tuple[float, ...]
    position: Vec3
    orientation: Quat


@dataclass(frozen=True)
class JointPlan(Moved):
    """plan_joint_path's answer for one arm: how the plan ended, and for a
    completed plan the start it used and its waypoints after the start, at
    least one, the last one at the goal. Nothing moved, whatever the end. A
    failed plan's kind is one of motion_planning's, such as
    `start_in_collision`, and its message the planner's words."""

    start_joint_positions: tuple[float, ...] = ()
    waypoints: tuple[Waypoint, ...] = ()


@dataclass(frozen=True)
class CartesianPath(Moved):
    """plan_cartesian_path's answer: how it ended, and for a completed one
    the start it used, the waypoints of the part of the requested lines
    that is valid (none when not even the first step is), and `fraction`,
    the share of the lines that part covers, 0 to 1, each line counting
    equally. Nothing moved, whatever the end."""

    start_joint_positions: tuple[float, ...] = ()
    waypoints: tuple[Waypoint, ...] = ()
    fraction: float = 0.0


@dataclass(frozen=True)
class MoveCheck:
    """check_arm_move's answer: whether a move_arm goal with the same
    fields has a plan from where the arm is commanded now, why it has none,
    and the time the move would take. A check that failed or got no answer
    is not reachable, and `reason` says why."""

    reachable: bool
    reason: str = ""
    duration_s: float = 0.0


@dataclass(frozen=True)
class PlannerModules:
    """The node's generated modules of the motion planner, motion_planning:v1
    on its `motion_planning` slot, with limb_motion's execute_joint_path,
    which runs a plan. Each field is named as its field of `Bindings`."""

    execute_joint_path: Any
    plan_joint_path: Any
    plan_cartesian_path: Any
    describe_arms: Any
    get_planning_scene: Any
    apply_collision_objects: Any
    apply_attached_collision_object: Any
    set_allowed_collision: Any

    @classmethod
    def from_bindings(cls, bindings) -> Optional["PlannerModules"]:
        """The planner modules of `bindings`, or None for a brain whose
        manifest lacks any of them."""
        modules = {field.name: getattr(bindings, field.name) for field in fields(cls)}
        if any(module is None for module in modules.values()):
            return None
        return cls(**modules)


class Robot:
    """`move_arm`, `move_gripper` and `check_arm_move` are the node's
    generated limb_motion modules, and `planner` its modules of the motion
    planner, none for a brain that does not consume it."""

    def __init__(
        self,
        node_runner,
        move_arm,
        move_gripper,
        check_arm_move,
        *,
        planner: Optional[PlannerModules] = None,
        result_timeout_s: float = RESULT_TIMEOUT_S,
    ) -> None:
        self._node_runner = node_runner
        self._move_arm = move_arm
        self._move_gripper = move_gripper
        self._check_arm_move = check_arm_move
        self._planner = planner
        self._result_timeout_s = result_timeout_s
        self._live: set = set()

    @property
    def has_planner(self) -> bool:
        """Whether the planner slot is bound: the brain consumes the motion
        planner, and the launcher bound its slot to a producer. The binding
        is fixed when the node starts."""
        if self._planner is None:
            return False
        return self._planner.plan_joint_path.bound_producer(self._node_runner) is not None

    async def move_arm(
        self,
        arm: str,
        position: Vec3,
        orientation: Quat,
        *,
        duration_s: float,
        settle_position_tolerance_m: float,
        settle_orientation_tolerance_rad: float,
        settle_timeout_s: float,
    ) -> MoveResult:
        """Plans and runs one grasp-point move of `arm`, robot frame, and
        waits for it to settle within the settle tolerances or to fail its
        settle timeout. The caller names every tolerance and time, as the
        limb_motion contract has them: the plan ends within 1 mm and 0.01 rad of
        the pose, the move completes once the measured grasp point stands within
        the settle tolerances at or after the planned end, and fails with
        `timeout` `settle_timeout_s` after it otherwise.
        A duration of 0 is the fastest the joint limits allow."""
        request = self._move_arm.GoalRequest(
            **_arm_move_fields(arm, position, orientation, duration_s),
            settle_position_tolerance_m=settle_position_tolerance_m,
            settle_orientation_tolerance_rad=settle_orientation_tolerance_rad,
            settle_timeout_s=settle_timeout_s,
        )
        moved, data = await self._run(self._move_arm, request)
        if data is None:
            return MoveResult(moved.end, moved.message, moved.failure_kind)
        return MoveResult(
            moved.end, moved.message, moved.failure_kind, _vec3(data.final_position), _quat(data.final_orientation)
        )

    async def set_gripper(
        self,
        gripper: str,
        opening: float,
        *,
        max_effort: float,
        settle_timeout_s: float,
    ) -> GripResult:
        """Drives one gripper to an opening fraction, 0 closed to 1 open,
        and waits for the jaws to stand still, at most `settle_timeout_s`
        after the commanded travel. The result's final_opening says where
        they stand then: on the opening, where an item stopped them, or
        wherever they were at the timeout, which `settled` false tells. A
        `max_effort` of 0 leaves the gripper's own effort ceiling in
        charge."""
        request = self._move_gripper.GoalRequest(
            gripper_name=gripper,
            opening=opening,
            max_effort=max_effort,
            settle_timeout_s=settle_timeout_s,
        )
        moved, data = await self._run(self._move_gripper, request)
        if data is None:
            return GripResult(moved.end, moved.message, moved.failure_kind)
        return GripResult(moved.end, moved.message, moved.failure_kind, bool(data.settled), float(data.final_opening))

    async def check_arm_move(
        self,
        arm: str,
        position: Vec3,
        orientation: Quat,
        *,
        duration_s: float,
    ) -> MoveCheck:
        """Whether `move_arm` with the same pose and duration has a plan from
        where the arm is commanded now. A check that fails, or that
        gets no answer within `CHECK_TIMEOUT_S`, is reported as not
        reachable, with the reason."""
        request = self._check_arm_move.Request(
            **_arm_move_fields(arm, position, orientation, duration_s)
        )
        try:
            answer = await self._check_arm_move.poll(
                self._node_runner, self._check_arm_move.bound_producer(self._node_runner), request, CHECK_TIMEOUT_S
            )
        except self._check_arm_move.Failure as failure:
            return MoveCheck(False, f"check_arm_move failed with {failure.kind.value}: {failure.message}")
        except Exception as error:
            return MoveCheck(False, f"check_arm_move gave no answer: {error!r}")
        data = answer.data
        return MoveCheck(bool(data.reachable), data.reason, float(data.duration_s))

    async def plan_joint_path(
        self,
        arm: str,
        link: str,
        position: Vec3,
        orientation: Quat,
        *,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        planner: str,
        refiner: str,
        allowed_planning_time_s: float,
        start_joint_positions: Sequence[float] = (),
    ) -> JointPlan:
        """Plans a joint path of `arm` around what the planning scene and
        the planner's map hold, from where the arm stands, or from
        `start_joint_positions`, to a state where the origin of `link`
        stands within `position_tolerance_m` of `position` and its
        orientation within `orientation_tolerance_rad` of `orientation`.
        The caller names the planner, the refiner and the time the plan may
        take, as motion_planning has them. Nothing moves. The plan is a goal
        in flight like a move, so a stop cancels it."""
        module = self._modules().plan_joint_path
        request = module.GoalRequest(
            arms=[module.GoalRequestArmsItem(arm_name=arm, start_joint_positions=[float(v) for v in start_joint_positions])],
            goal_joint_constraints=[],
            goal_position_constraints=[
                module.GoalRequestGoalPositionConstraintsItem(link_name=link, position=list(position), tolerance_m=position_tolerance_m)
            ],
            goal_orientation_constraints=[
                module.GoalRequestGoalOrientationConstraintsItem(
                    link_name=link, orientation=list(orientation), tolerance_rad=orientation_tolerance_rad, free_axis="none",
                )
            ],
            path_orientation_constraints=[],
            planner=planner,
            refiner=refiner,
            allowed_planning_time_s=allowed_planning_time_s,
        )
        moved, data = await self._run(module, request)
        if not moved.completed:
            return JointPlan(moved.end, moved.message, moved.failure_kind)
        planned = next((found for found in data.arms if found.arm_name == arm), None)
        if planned is None or not planned.waypoints:
            return JointPlan(MoveEnd.LOST, f"plan_joint_path answered no waypoints for arm '{arm}'")
        return JointPlan(
            MoveEnd.COMPLETED,
            start_joint_positions=_joints(planned.start_joint_positions),
            waypoints=tuple(_waypoint(waypoint) for waypoint in planned.waypoints),
        )

    async def plan_cartesian_path(
        self,
        arm: str,
        link: str,
        poses: Sequence[tuple[Vec3, Quat]],
        *,
        avoid_collisions: bool,
        start_joint_positions: Sequence[float] = (),
    ) -> CartesianPath:
        """Asks the planner for the joint path along which `link` of `arm`
        follows straight lines through `poses`, each a (position,
        orientation), from where the arm stands, or from
        `start_joint_positions`. With `avoid_collisions` false nothing is
        checked but the reach and the precision, as a reach test; with it
        true the path ends before the first collision. Nothing moves. The
        call is a goal in flight like a move, so a stop cancels it."""
        module = self._modules().plan_cartesian_path
        request = module.GoalRequest(
            arm_name=arm,
            start_joint_positions=[float(v) for v in start_joint_positions],
            link_name=link,
            waypoints=[module.GoalRequestWaypointsItem(position=list(position), orientation=list(orientation)) for position, orientation in poses],
            avoid_collisions=avoid_collisions,
        )
        moved, data = await self._run(module, request)
        if not moved.completed:
            return CartesianPath(moved.end, moved.message, moved.failure_kind)
        return CartesianPath(
            MoveEnd.COMPLETED,
            start_joint_positions=_joints(data.start_joint_positions),
            waypoints=tuple(_waypoint(waypoint) for waypoint in data.waypoints),
            fraction=float(data.fraction),
        )

    @property
    def moves_in_flight(self) -> int:
        """How many goals the backbone has accepted and not yet ended."""
        return len(self._live)

    async def stop(self) -> None:
        """Cancels every move in flight. The waiting calls return with the
        cancelled outcome once the backbone has brought the limb to rest."""
        for handle in list(self._live):
            await _cancel(handle)

    def _modules(self) -> PlannerModules:
        """The planner's modules; a call of a planner member by a brain that
        does not consume the planner is a fault of its caller."""
        if self._planner is None:
            raise RuntimeError("this brain does not consume the motion planner")
        return self._planner

    async def _run(self, module, request) -> tuple[Moved, object]:
        """Fires one goal of `module` at the bound producer, keeps its
        handle while it runs, and reads how it ended: the end, and the
        result's data of a completed or a cancelled move, none otherwise."""
        name = module.TARGET_ACTION_NAME
        try:
            handle = await module.ActionHandle.fire_goal(
                self._node_runner, module.bound_producer(self._node_runner), request, GOAL_TIMEOUT_S
            )
        except Exception as error:
            return Moved(MoveEnd.LOST, f"{name} could not be sent: {error!r}"), None
        if not handle.accepted:
            refusal = handle.refusal
            return Moved(MoveEnd.REFUSED, refusal.message, refusal.kind.value), None
        self._live.add(handle)
        try:
            result = await handle.get_result(self._result_timeout_s)
        except Exception as error:
            await _cancel(handle)
            return Moved(MoveEnd.LOST, f"{name} gave no result: {error!r}"), None
        finally:
            self._live.discard(handle)
        if result.status == module.ResultStatus.COMPLETED:
            return Moved(MoveEnd.COMPLETED), result.data
        if result.status == module.ResultStatus.CANCELLED:
            return Moved(MoveEnd.CANCELLED, result.cancel_reason or f"{name} was cancelled"), result.data
        if result.status == module.ResultStatus.FAILED:
            failure = result.failure
            return Moved(MoveEnd.FAILED, failure.message, failure.kind.value), None
        return Moved(MoveEnd.LOST, f"{name} ended as {result.status.name.lower()}"), None


def _arm_move_fields(
    arm: str,
    position: Vec3,
    orientation: Quat,
    duration_s: float,
) -> dict:
    """The fields a move_arm goal and a check_arm_move request share."""
    return dict(
        arm_name=arm,
        position=list(position),
        orientation=list(orientation),
        duration_s=duration_s,
    )


def _joints(values) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _waypoint(waypoint) -> Waypoint:
    """A waypoint of a plan or a path, as motion_planning gives it."""
    return Waypoint(_joints(waypoint.joint_positions), _vec3(waypoint.position), _quat(waypoint.orientation))


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
