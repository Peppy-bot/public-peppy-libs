"""The scripted manipulator: `manipulation_model: "scripted"`. No model
call: the grasp comes from a `GraspProposer`, and every move from a fixed
sequence through the `Robot`, so abort, cancel, the backbone's stop and
the node's shutdown stop it as they stop any manipulator.

A grab:

1. asks the proposer for the grasp point and the orientations to try;
2. opens the gripper as wide as the item and `approach_margin_m` on each
   side, fully for an item of no known width, so the fingers come down
   beside the item and not on what stands next to it;
3. takes the first orientation for which the backbone has a plan to the
   pre-grasp point (`clearance_m` back along the approach), to the grasp
   point and to the lifted point (`lift_m` above it), each checked with
   check_arm_move from where the arm is commanded now; then moves to the
   pre-grasp point; with an `aligner` (its port is `aligning.py`), a model then looks
   through the wrist and chest cameras and corrects the gripper's place
   and turn above the item, and the pre-grasp, grasp and lifted points
   become the aligned ones; then moves to the grasp point on a line down
   that stops on contact;
4. closes where the line down ended, at the grasp point or on contact,
   with the goal's `max_effort` and judges the hold from the result's
   `final_opening`: below `hold_opening_min` the jaws met nothing;
5. lifts, then judges the hold again from the first limb_states snapshot
   captured after the lift's result, waiting up to `snapshot_wait_s` for
   it: below `hold_opening_min`, or more than `hold_drop_max` below the
   opening at the close, the item slipped, since jaws that lose the item
   close further.

Every move names its bars, as the limb_motion contract has the caller do:
an arm move plans within 1 mm and 0.01 rad of its pose and completes once
the measured grasp point stands within `settle_position_m` and
`settle_orientation_rad` of it, or fails `settle_timeout_s` after its
planned end; a gripper move waits up to `grip_settle_timeout_s` for the
jaws to stand still. A check asks with the arm move's pose and duration.
Every arm move names the contact gap 0, no contact check, and the fastest
duration, but a line down, the descent of a grab and of a place: it stops
on contact, with `line_down_contact_gap_m`, and runs slowly, its duration
its length divided by `line_down_speed_m_s`.

The transit, the move to the pre-grasp point and to the pre-place point,
is the one step that depends on the motion planner (`_transit`). Without
it, the transit is the move_arm line of today, and each point is checked
with check_arm_move. With it (`Robot.has_planner`), each orientation is
asked for a plan_joint_path to the pre point, the grasp link within 1 mm
and 0.01 rad, `rrt_connect` with no refiner, in `allowed_planning_time_s`,
and then for a plan_cartesian_path with no collision check from the end of
that plan through the points that follow, a test of their reach alone; the
first orientation that passes both runs its plan with execute_joint_path,
the plan's start as the expected start. A plan that fails, such as one
from a start in collision, gives the planner's kind and words, and no line
replaces it. The descent, the lift, the retreat and the climb stay move_arm
lines either way. Before a grab plans, it adds the item's object,
`brain/<item_id>`, to the planning scene at the item's place, the box its
look measured or a sphere for an item no look measured, and lets the
gripper's links touch it; the add clears the item's cubes from the
planner's map, so none stay under the hand after the lift. Each way a grab
fails or stops removes the object; a grab that holds leaves it for the
brain to attach. Just before the descent, after the aligner, a grab with
the planner asks it what a checked descent would answer: a
plan_cartesian_path with the collision check from where the arm stands
through the grasp and lifted points. It logs the share of the lines that
path covers and moves by the lines of today either way.

Each step that fails ends the grab with the backbone's own words. A grab
or a place that no orientation plans names the last orientation it tried,
the point of it that has no plan and the backbone's words for it. A step
that fails once the descent has begun (the descent itself, the close, the
two hold judgements, the lift, the wait for a snapshot) first retreats:
the gripper opens and the arm moves straight back up to the pre-grasp
point, so the jaws let go of what they touch and the next look sees the
item. A retreat that stops short adds why to the failure. A step whose
goal a stop ended (the brain's stop or the backbone's stop service)
retreats from nothing: the arm stays where the stop left it. A step that
fails before the descent leaves the arm where the backbone stopped it; a
grab that no orientation reaches leaves the jaws open, since a grab takes
a gripper that holds nothing.

Each grab leaves one log line: the orientation that planned, with how many
check_arm_move calls ran and their total time; how wide the jaws opened for
the item's width; where the descent's result put the grasp point against
the commanded one, and whether the descent ended on contact; limb_state's
grasp point at the close; the two openings it judged; and the retreat.
Each place logs one line before its line down: a pre-place point lower than
the first, the orientation that planned with the calls it took, and the
aligner's correction.

A drop opens the gripper where it is; a drop above a target item ends at
once with `TARGETS_REFUSED`. A place first reads the gripper's
opening from the latest snapshot: jaws below `hold_opening_min` stand
closed on air, the item slipped since the grab, and the place fails
before any move, naming it. It then carries the held item above the
pose, through its pre-place point, `clearance_m` and `held_item_depth_m`
above the pose so the held item clears what it is set on until the
descent, or `clearance_m` alone above it where the backbone has no plan
that high; in the pose's orientation alone when it has one, else from
above at each roll; with an `aligner`, the model then
corrects the held item's place above the spot as it does above an item in
a grab. The line down runs from the pre-place point to the pose and stops
on contact, so the arm does not press the item on what it is set on. Each
check of the line down (the reach test and the aligner's) uses the pose.
When the line down completes, settled at the pose or on contact, the item
resting on what is under it (the rim or the floor of a container, or an
item already in it), the place opens there, logs a contact, and climbs
back to the pre-place point. A line down that fails, with `timeout` or
another kind, or that a stop ended, fails the place, and the gripper still
holds the item. A place reports where the grasp point stood when the jaws
opened. A climb that fails is logged, and the place still succeeds: the
jaws are open and the item is down.

A place into a target item (a `TargetItem`, what the brain knew of the
item when the goal started) carries the held item from above, at each roll,
to its pre-place point `above_target_m` over the target's top, or
`clearance_m` over it where the backbone has no plan that high. The reach
test checks the line down to the target's bottom, the surface under it.
With an aligner, the model then corrects the held item's place above the
target, told the target's label and width. The line down runs from the
pre-place point to the target's bottom and stops on contact: the item
rests on the target's floor or on what the target holds, and the jaws open
where the arm stopped, or at the bottom when the line met nothing; the
place logs which, and climbs back. A target that no orientation reaches
fails the place before any move, with the target's id in the words.

A place and a grab carry the arm to their pre point on the carry step:
the transit and the move there. With an aligner, each then runs the align
step above its point. The line down of a place and of a grab
runs on the descend step. A drop and the release of a place run on the
release step: it opens the gripper
and, for a place, climbs back to the pre-place point. A step returns its `StepRecord`: its name, what it was asked, its result, why it
failed and its measured numbers. The sequence keeps the records in order
and stops at the first one with a failure, whose message ends the
outcome; the outcome keeps the records in `steps`. A cancel raises
`Cancelled`, as in every step.

The manipulator keeps nothing between goals: ids and held flags live in
`State`.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Awaitable, Callable, Generic, Mapping, Optional, Sequence, TypeVar, Union

from ...limb_state import LimbSnapshot
from ...ports import Cancelled, CancelToken, Gripper, Item, Outcome, Pose, Quat, Senses, TargetItem, Vec3
from ...robot import ArmDescription, JointPlan, MoveResult, ObjectOperation, brain_object_id, item_object
from ...waiting import until
from .aligning import Aligned, Aligner, AlignmentFailed, AlignRequest, GaveUp, OutOfSteps, Stopped
from .grasp import Grasp, GraspProposer, jaw_line_deg, lifted, orientations_for, pre_grasp

logger = logging.getLogger(__name__)

# The pre-grasp and lifted points stand 7.5 cm from the grasp point: in
# Waldo, 10 cm above the top of a 12 cm can leaves the OpenArm's grasp
# point no plan pointing down, and 7.5 cm does.
CLEARANCE_M = 0.075
LIFT_M = 0.075
# How far below the grasp point a held item may reach: the pre-place point
# stands this and the clearance above the pose, so the item clears the
# floor of a container and what already stands in it until the descent. A
# 10 cm apple grasped 2.5 cm under its top reaches 7.8 cm. In Waldo the
# OpenArm has no plan pointing down more than about 15 cm above a desk spot
# near its body, so where this is too high the pre-place point stands at
# the clearance alone.
HELD_ITEM_DEPTH_M = 0.075
# How high the pre-place point of a place into a target item stands above
# the item's top, before it tries `clearance_m`: in Waldo the OpenArm has a
# plan pointing down 12.5 cm above the rim of a bowl near its body with
# either arm, and 15 cm has none for the left arm (the box session of
# 2026-11-18).
ABOVE_TARGET_M = 0.125
HOLD_OPENING_MIN = 0.05
# How far the opening may fall from the close to after the lift while the
# jaws still hold the item. In Waldo a held fruit's fell 0.003 to 0.013 (the
# bowl runs of 2026-10-24), and an apple that slipped out let it fall 0.316,
# from 0.378 to 0.062.
HOLD_DROP_MAX = 0.05
SNAPSHOT_WAIT_S = 1.0
# The bars every arm move names. The move completes once the measured grasp
# point stands within 2 cm and 0.15 rad of the commanded pose, the settle the
# robots' endpoint gives for the OpenArm v2, and fails 2 s after the planned
# end otherwise. 0 s is the fastest move the joint limits allow.
MOVE_DURATION_S = 0.0
SETTLE_POSITION_M = 0.02
SETTLE_ORIENTATION_RAD = 0.15
SETTLE_TIMEOUT_S = 2.0
# The goal of a planned transit: the grasp link within 1 mm and 0.01 rad of
# the pre point, as move_arm plans, by RRTConnect with no refiner.
PLAN_POSITION_TOLERANCE_M = 0.001
PLAN_ORIENTATION_TOLERANCE_RAD = 0.01
PLANNER = "rrt_connect"
REFINER = "none"
# The time each plan of a transit may take. A plan that exists took 0.03 to
# 0.42 s on the VLA box, and a goal with no plan takes the whole time: at
# most 4 s for the 8 rolls of a grab (the spike of 2026-11-13).
ALLOWED_PLANNING_TIME_S = 0.5
# How far each joint of a planned transit may stand from the plan's last
# waypoint when the run completes: the start tolerance of the backbone and
# of the planner.
SETTLE_JOINT_RAD = 0.05
# A reach test passes when its lines are covered to within this share.
FULL_REACH = 1.0 - 1e-9
# The contact gap every arm move but a line down, and every planned
# transit, names: limb_motion's 0, no contact check, so a move ends by its
# settle alone.
NO_CONTACT_CHECK_M = 0.0
# A line down, the descent of a grab or of a place, has its own contact gap
# and its own speed. The gap is the settle position tolerance, 2 cm, so no
# arm stands between a settle and a contact. The duration of a line down is
# its length divided by the speed (`line_down_duration_s`): 3 s for the
# 15 cm from the high pre-place point.
LINE_DOWN_CONTACT_GAP_M = 0.02
LINE_DOWN_SPEED_M_S = 0.05
# How long a gripper move waits for the jaws to stand still.
GRIP_SETTLE_TIMEOUT_S = 2.0
# The span of the jaws fully open: the OpenArm v2's in Waldo, where an
# opening grows 0.0677 a centimetre between the jaws (the gripper tests of
# 2026-10-23). A grab opens to the item's width and this margin on each
# side.
JAW_TRAVEL_M = 0.148
APPROACH_MARGIN_M = 0.015
OPEN = 1.0
CLOSED = 0.0
NOT_STARTED = "the scripted manipulator has not been started"
# What a drop above an item ends with.
TARGETS_REFUSED = "the scripted manipulator drops where the arm stands: it takes no target item"


class _Failed(Exception):
    """A step of a sequence that did not go through: `message` ends the
    sequence's outcome. `stopped` when the step's goal ended as cancelled,
    by a stop, so nothing more moves."""

    def __init__(self, message: str, *, stopped: bool = False) -> None:
        super().__init__(message)
        self.message = message
        self.stopped = stopped


class _NoReach(_Failed):
    """No orientation of a grasp reaches: nothing moved, and a place tries
    its lower pre-place point."""


@dataclass
class _Checks:
    """The calls of a reach: how many check_arm_move calls, plans and reach
    tests of the planner ran, and their total time."""

    count: int = 0
    plans: int = 0
    lines: int = 0
    seconds: float = 0.0

    def __str__(self) -> str:
        counted = [(self.count, "check"), (self.plans, "plan"), (self.lines, "reach test")]
        parts = [f"{number} {word}{'' if number == 1 else 's'}" for number, word in counted if number]
        return ", ".join([*parts, f"{1000 * self.seconds:.0f} ms"])


@dataclass(frozen=True)
class _Reach:
    """The first orientation of a grasp whose moves reach, as `_transit`
    finds it: the orientation, the points of its moves in order, the pre
    point first, `go`, the transit, the move that takes the arm to the pre
    point, `abandon` and `log_descent`."""

    orientation: Quat
    points: tuple[Vec3, ...]
    go: Callable[[CancelToken], Awaitable[object]]
    # What a grab that fails or stops undoes of the transit: the item's
    # object it added to the planning scene, nothing with no planner.
    abandon: Callable[[], Awaitable[None]]
    # The log of what a checked descent from where the arm stands would
    # answer, given the grasp point, the lifted point and the orientation;
    # nothing with no planner.
    log_descent: Callable[[Vec3, Vec3, Quat, CancelToken], Awaitable[None]]


class StepFailure(StrEnum):
    """Why a step did not go through."""

    # No orientation has a plan: nothing moved.
    NO_REACH = "no_reach"
    # A move, a plan or a jaw move did not complete.
    MOVE = "move"
    # The aligner gave up, ran out of steps or could not observe.
    ALIGNMENT = "alignment"
    # A stop ended the goal of the step.
    STOPPED = "stopped"


StepResult = TypeVar("StepResult")


@dataclass(frozen=True)
class StepRecord(Generic[StepResult]):
    """One step of a sequence as it ran: `step` its name ("carry", "align",
    "descend" or "release"), `inputs` what it was asked, `result` what the
    next step reads, none when the step failed, `failure` and `message` why
    it failed, and `measured` its numbers, the unit in each name."""

    step: str
    inputs: Mapping[str, object]
    result: Optional[StepResult] = None
    failure: Optional[StepFailure] = None
    message: str = ""
    measured: Mapping[str, float] = field(default_factory=dict)


@dataclass
class Carried:
    """Where a carry left the arm: the orientation that planned, and the
    points of the moves that follow it in order, the pre point first, where
    the arm stands now. `abandon` undoes what the carry added for a grab
    that fails or stops later, and `log_descent` logs what a checked descent
    from where the arm stands would answer; each does nothing with no
    planner."""

    orientation: Quat
    points: tuple[Vec3, ...]
    abandon: Callable[[], Awaitable[None]]
    log_descent: Callable[[Vec3, Vec3, Quat, CancelToken], Awaitable[None]]


@dataclass(frozen=True)
class Landed:
    """Where a line down ended: the grasp point its result gives, none when
    it gives none, and whether it ended on contact."""

    position: Optional[Vec3]
    on_contact: bool


@dataclass(frozen=True)
class Released:
    """How a release ended: the opening the jaws stood at once open, and
    for a release that climbs, whether the arm got back to its climb
    point; none for one that does not climb."""

    opening: float
    climbed: Optional[bool] = None


def _failed_step(step: str, inputs: Mapping[str, object], failure: _Failed, kind: StepFailure = StepFailure.MOVE) -> StepRecord:
    """The record of `step` that `failure` ended: `kind`, or `STOPPED` when
    a stop ended its goal."""
    return StepRecord(step, inputs, failure=StepFailure.STOPPED if failure.stopped else kind, message=failure.message)


def _passed(record: StepRecord[StepResult], steps: list[StepRecord]) -> StepResult:
    """Keeps `record` in `steps` and gives its result. A record with a
    failure ends the sequence: `_Failed` with its message."""
    steps.append(record)
    if record.failure is not None:
        raise _Failed(record.message, stopped=record.failure is StepFailure.STOPPED)
    return record.result


async def _nothing(*_) -> None:
    return None


class ScriptedManipulator:
    name = "scripted"

    def __init__(
        self,
        proposer: GraspProposer,
        *,
        clearance_m: float = CLEARANCE_M,
        lift_m: float = LIFT_M,
        held_item_depth_m: float = HELD_ITEM_DEPTH_M,
        above_target_m: float = ABOVE_TARGET_M,
        hold_opening_min: float = HOLD_OPENING_MIN,
        hold_drop_max: float = HOLD_DROP_MAX,
        snapshot_wait_s: float = SNAPSHOT_WAIT_S,
        settle_position_m: float = SETTLE_POSITION_M,
        settle_orientation_rad: float = SETTLE_ORIENTATION_RAD,
        settle_timeout_s: float = SETTLE_TIMEOUT_S,
        grip_settle_timeout_s: float = GRIP_SETTLE_TIMEOUT_S,
        jaw_travel_m: float = JAW_TRAVEL_M,
        approach_margin_m: float = APPROACH_MARGIN_M,
        allowed_planning_time_s: float = ALLOWED_PLANNING_TIME_S,
        line_down_contact_gap_m: float = LINE_DOWN_CONTACT_GAP_M,
        line_down_speed_m_s: float = LINE_DOWN_SPEED_M_S,
        aligner: Optional[Aligner] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not line_down_contact_gap_m > 0.0:
            raise ValueError(f"line_down_contact_gap_m must be above 0, so a line down stops on contact: got {line_down_contact_gap_m}")
        if not line_down_speed_m_s > 0.0:
            raise ValueError(f"line_down_speed_m_s must be above 0: got {line_down_speed_m_s}")
        self.proposer = proposer
        # The model that corrects the gripper above the item before the
        # descent; none for the grab with no model.
        self.aligner = aligner
        self.clearance_m = clearance_m
        self.lift_m = lift_m
        self.held_item_depth_m = held_item_depth_m
        self.above_target_m = above_target_m
        self.hold_opening_min = hold_opening_min
        self.hold_drop_max = hold_drop_max
        self.snapshot_wait_s = snapshot_wait_s
        self.settle_position_m = settle_position_m
        self.settle_orientation_rad = settle_orientation_rad
        self.settle_timeout_s = settle_timeout_s
        self.grip_settle_timeout_s = grip_settle_timeout_s
        self.jaw_travel_m = jaw_travel_m
        self.approach_margin_m = approach_margin_m
        self.allowed_planning_time_s = allowed_planning_time_s
        self.line_down_contact_gap_m = line_down_contact_gap_m
        self.line_down_speed_m_s = line_down_speed_m_s
        # The brain's monotonic clock, for the checks' time in the log line.
        self._clock = clock
        self._robot = None
        self._limb_states = None

    def why_unavailable(self) -> str:
        if self._robot is None or self._limb_states is None:
            return NOT_STARTED
        return self._limb_states.why_unavailable()

    def starting(self) -> str:
        if self._robot is None or self._limb_states is None:
            return NOT_STARTED
        return self._limb_states.starting()

    async def start(self, senses: Senses) -> None:
        self._robot = senses.robot
        self._limb_states = senses.limb_states
        if self.aligner is not None:
            self.aligner.start(senses)

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken) -> Outcome:
        notes: list[str] = []
        steps: list[StepRecord] = []
        try:
            outcome = await self._grab(item, gripper, max_effort, cancel, notes, steps)
        except _Failed as failure:
            notes.append(f"failed: {failure.message}")
            outcome = Outcome(False, failure.message, steps=tuple(steps))
        except Cancelled as cancelled:
            notes.append(f"stopped: {cancelled.reason}")
            raise
        finally:
            logger.info("grab of item '%s' with gripper '%s': %s", item.item_id, gripper.name, "; ".join(notes))
        return outcome

    async def drop(self, gripper: Gripper, cancel: CancelToken, above: Optional[TargetItem] = None) -> Outcome:
        steps: list[StepRecord] = []
        try:
            if above is not None:
                raise _Failed(TARGETS_REFUSED)
            self._require_started()
            _passed(await self._release(gripper, cancel), steps)
        except _Failed as failure:
            return Outcome(False, failure.message, steps=tuple(steps))
        return Outcome(True, steps=tuple(steps))

    async def place(self, gripper: Gripper, to: Union[Pose, TargetItem], cancel: CancelToken) -> Outcome:
        steps: list[StepRecord] = []
        try:
            if isinstance(to, TargetItem):
                return await self._place_into(gripper, to, cancel, steps)
            return await self._place(gripper, to, cancel, steps)
        except _Failed as failure:
            return Outcome(False, failure.message, steps=tuple(steps))

    async def stop(self) -> None:
        """Nothing of its own: the lane's stop cancels the robot's move in
        flight, and the sequence ends at its next cancel check."""
        return None

    async def _grab(
        self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken, notes: list[str], steps: list[StepRecord]
    ) -> Outcome:
        self._require_started()
        # Before any move: a gripper limb_states does not name could never
        # be judged after the lift.
        _opening_of(self._limb_states.latest, gripper.name)
        if self.aligner is not None:
            reason = self.aligner.why_unavailable(gripper.arm)
            if reason:
                raise _Failed(f"no alignment: {reason}")
        grasp = self._propose(item)
        # The jaws open before the checks or the plan: a plan uses the
        # openings measured when it is asked.
        opening = self._approach_opening(item)
        if item.width_m is not None:
            notes.append(f"opened to {opening:.2f} for its {100 * item.width_m:.1f} cm width")
        await self._grip(gripper.name, opening, 0.0, cancel)
        carried = _passed(
            await self._carry(
                gripper, grasp, f"item '{item.item_id}'", cancel, notes,
                heights_m=(self.clearance_m,), with_lift=True, item=item,
            ),
            steps,
        )
        try:
            return await self._run_grab(item, gripper, max_effort, carried, cancel, notes, steps)
        except (_Failed, Cancelled):
            await carried.abandon()
            raise

    async def _run_grab(
        self, item: Item, gripper: Gripper, max_effort: float, carried: Carried, cancel: CancelToken, notes: list[str],
        steps: list[StepRecord],
    ) -> Outcome:
        """The grab from its pre-grasp point on: the alignment, then the
        descent, the close and the lift, with the retreat after a step from
        the descent on that fails."""
        orientation, (pre, point, lift) = carried.orientation, carried.points
        if self.aligner is not None:
            request = AlignRequest(item, gripper, pre, point, orientation, self.lift_m)
            aligned = _passed(await self._align(self.aligner, request, cancel, notes), steps)
            pre, point, lift, orientation = aligned.pre, aligned.point, aligned.lift, aligned.orientation
        await carried.log_descent(point, lift, orientation, cancel)
        try:
            return await self._descend_close_lift(item, gripper, max_effort, orientation, pre, point, lift, cancel, notes, steps)
        except _Failed as failure:
            if failure.stopped:
                raise
            short = await self._retreat(gripper, pre, orientation, cancel)
            if short:
                notes.append(f"the retreat stopped short: {short}")
                raise _Failed(f"{failure.message}; the retreat to the pre-grasp point failed: {short}") from None
            notes.append("retreated to the pre-grasp point")
            raise

    async def _descend_close_lift(
        self, item: Item, gripper: Gripper, max_effort: float, orientation: Quat, pre: Vec3, point: Vec3, lift: Vec3,
        cancel: CancelToken, notes: list[str], steps: list[StepRecord],
    ) -> Outcome:
        """The grab from its descent on: the descend step from `pre` to
        `point`, close where it ended, judge, lift to `lift`, judge again. A
        hold reports where limb_states put the grasp point when the jaws
        closed."""
        landed = _passed(await self._descend(gripper.arm, pre, point, orientation, cancel), steps)
        on_contact = ", on contact" if landed.on_contact else ""
        notes.append(f"landed {_offset_cm(landed.position, point)} from the grasp point{on_contact}")
        closed = await self._grip(gripper.name, CLOSED, max_effort, cancel)
        closed_at = self._arm_position(gripper.arm)
        notes.append(f"limb_state at the close {_offset_cm(closed_at, point)} from it")
        notes.append(f"closed at {closed:.3f}")
        if closed < self.hold_opening_min:
            raise _Failed(f"gripper '{gripper.name}' closed on nothing")
        await self._move(gripper.arm, lift, orientation, cancel)
        after = await self._snapshot_after(self._limb_states.latest, cancel)
        opening = _opening_of(after, gripper.name)
        notes.append(f"{opening:.3f} after the lift")
        if self._slipped(closed, opening):
            raise _Failed(f"item '{item.item_id}' slipped while lifting")
        notes.append("held")
        return Outcome(True, closed_at=closed_at, steps=tuple(steps))

    async def _align(self, aligner: Aligner, request: AlignRequest, cancel: CancelToken, notes: list[str]) -> StepRecord[Aligned]:
        """The align step: `aligner` corrects the gripper above the point of
        `request`, through this manipulator's moves, and `notes` says how
        far the grasp point moved and along which line the jaws close. It
        goes through with the points and the orientation the sequence goes
        on with. A model that gives up, runs out of steps or cannot observe
        fails it, and so does a stop that ended one of its moves: each ends
        the sequence here, before the descent, so nothing retreats."""
        inputs = {"request": request}
        try:
            alignment = await aligner.align(request, self, cancel)
        except AlignmentFailed as failure:
            return StepRecord("align", inputs, failure=StepFailure.ALIGNMENT, message=f"the alignment failed: {failure.message}")
        if isinstance(alignment, GaveUp):
            message = f"the model gave up after {alignment.steps} step{'' if alignment.steps == 1 else 's'}"
            return StepRecord("align", inputs, failure=StepFailure.ALIGNMENT, message=message, measured={"steps": alignment.steps})
        if isinstance(alignment, OutOfSteps):
            message = f"no ready in {alignment.steps} steps"
            return StepRecord("align", inputs, failure=StepFailure.ALIGNMENT, message=message, measured={"steps": alignment.steps})
        if isinstance(alignment, Stopped):
            return StepRecord("align", inputs, failure=StepFailure.STOPPED, message=alignment.message, measured={"steps": alignment.steps})
        moved_cm = 100 * math.dist(alignment.pre[:2], request.pre[:2])
        jaw_deg = jaw_line_deg(alignment.orientation)
        notes.append(
            f"aligned in {alignment.steps} step{'' if alignment.steps == 1 else 's'}: the grasp point moved {moved_cm:.1f} cm, "
            f"the jaws close along {jaw_deg:.0f} degrees"
        )
        return StepRecord("align", inputs, alignment, measured={"steps": alignment.steps, "shift_cm": moved_cm, "jaw_line_deg": jaw_deg})

    async def refusal_for(self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken) -> str:
        """For the aligner: the backbone's words for the first of `points`
        it has no plan for in `orientation`, or "" when each has one."""
        return await self._first_refusal(arm, tuple(points), orientation, cancel, _Checks())

    async def move_to(self, arm: str, point: Vec3, orientation: Quat) -> MoveResult:
        """For the aligner: one arm move with the bars of every arm move;
        the result says how it ended."""
        return await self._move_arm(arm, point, orientation)

    def _slipped(self, closed: float, after_lift: float) -> bool:
        """Whether the item left the jaws during the lift: they stand near
        closed, or closed further than the item let them at the close."""
        return after_lift < self.hold_opening_min or closed - after_lift > self.hold_drop_max

    def _approach_opening(self, item: Item) -> float:
        """How wide the jaws open before the descent: the item's width and
        the margin on each side, as a fraction of the jaws' span; fully open
        for an item of no known width."""
        if item.width_m is None:
            return OPEN
        return min(OPEN, (item.width_m + 2 * self.approach_margin_m) / self.jaw_travel_m)

    async def _retreat(self, gripper: Gripper, pre: Vec3, orientation: Quat, cancel: CancelToken) -> str:
        """Opens `gripper` and moves its arm straight up to `pre`, after a
        grab that failed past its descent. Returns "" once there, or why the
        retreat stopped short; a cancel raises Cancelled."""
        try:
            await self._grip(gripper.name, OPEN, 0.0, cancel)
            await self._move(gripper.arm, pre, orientation, cancel)
        except _Failed as failure:
            return failure.message
        return ""

    def _arm_position(self, arm: str) -> Optional[Vec3]:
        """The grasp point of `arm` in the latest limb_states snapshot."""
        snapshot = self._limb_states.latest
        state = snapshot.arms.get(arm) if snapshot is not None else None
        return state.position if state is not None else None

    def _refuse_a_place_before_any_move(self, gripper: Gripper) -> None:
        """What ends a place before any move: a manipulator not started,
        jaws closed on air since the grab (the item slipped on the way, and a
        place would carry nothing; the caller drops to let the brain forget
        the item), and an aligner that cannot align the arm now."""
        self._require_started()
        opening = self._limb_states.latest.opening(gripper.name)
        if opening is not None and opening < self.hold_opening_min:
            raise _Failed(f"gripper '{gripper.name}' holds nothing now: its jaws stand at {opening:.2f}, closed on air; drop to let it go")
        if self.aligner is not None:
            reason = self.aligner.why_unavailable(gripper.arm)
            if reason:
                raise _Failed(f"no alignment: {reason}")

    async def _place(self, gripper: Gripper, pose: Pose, cancel: CancelToken, steps: list[StepRecord]) -> Outcome:
        self._refuse_a_place_before_any_move(gripper)
        try:
            orientations = orientations_for(pose.orientation)
        except ValueError as error:
            raise _Failed(str(error)) from None
        target = f"the pose {_numbers(pose.position)}"
        notes: list[str] = []
        carried = _passed(
            await self._carry(
                gripper, Grasp(pose.position, orientations), target, cancel, notes,
                heights_m=(self.clearance_m + self.held_item_depth_m, self.clearance_m),
            ),
            steps,
        )
        orientation, (pre, point) = carried.orientation, carried.points
        if self.aligner is not None:
            request = AlignRequest(None, gripper, pre, point, orientation, self.lift_m, target=pose.position)
            aligned = _passed(await self._align(self.aligner, request, cancel, notes), steps)
            pre, point, orientation = aligned.pre, aligned.point, aligned.orientation
        if notes:
            logger.info("place with gripper '%s': %s", gripper.name, "; ".join(notes))
        landed = _passed(await self._descend(gripper.arm, pre, point, orientation, cancel), steps)
        if landed.on_contact:
            logger.info(
                "place with gripper '%s' ended its line down on contact %s from the place point: it releases there",
                gripper.name, _offset_cm(landed.position, point),
            )
        _passed(await self._release(gripper, cancel, climb=(pre, orientation)), steps)
        return Outcome(True, "", landed.position, steps=tuple(steps))

    async def _place_into(self, gripper: Gripper, target: TargetItem, cancel: CancelToken, steps: list[StepRecord]) -> Outcome:
        """A place into or onto `target`: the carry to the pre-place point
        `above_target_m` over the target's top (`clearance_m` where that has
        no plan), from above at each roll, with the line down to the
        target's bottom in its reach test; the alignment, told the target;
        the line down, which ends on contact where the item comes to rest, or
        at the target's bottom when it meets nothing; the release and the
        climb back. It reports where the line down left the grasp point."""
        self._refuse_a_place_before_any_move(gripper)
        notes: list[str] = []
        end = (target.centre[0], target.centre[1], target.bottom_z)
        carried = _passed(
            await self._carry(
                gripper, Grasp(end, orientations_for(None)), f"item '{target.item_id}'", cancel, notes,
                heights_m=(self.above_target_m, self.clearance_m), rise_m=max(0.0, target.top_z - target.bottom_z),
                height_words=f"the top of item '{target.item_id}'",
            ),
            steps,
        )
        orientation, (pre, point) = carried.orientation, carried.points
        if self.aligner is not None:
            top = (target.centre[0], target.centre[1], target.top_z)
            request = AlignRequest(None, gripper, pre, point, orientation, self.lift_m, target=top, into=target)
            aligned = _passed(await self._align(self.aligner, request, cancel, notes), steps)
            pre, point, orientation = aligned.pre, aligned.point, aligned.orientation
        logger.info("place with gripper '%s' into item '%s': %s", gripper.name, target.item_id, "; ".join(notes))
        landed = _passed(await self._descend(gripper.arm, pre, point, orientation, cancel), steps)
        if landed.on_contact:
            logger.info(
                "place with gripper '%s' into item '%s' ended its line down on contact %s from the item's bottom: it releases there",
                gripper.name, target.item_id, _offset_cm(landed.position, point),
            )
        else:
            logger.info(
                "place with gripper '%s' into item '%s' met nothing down to the item's bottom: it releases there",
                gripper.name, target.item_id,
            )
        _passed(await self._release(gripper, cancel, climb=(pre, orientation)), steps)
        return Outcome(True, "", landed.position, steps=tuple(steps))

    async def _release(
        self, gripper: Gripper, cancel: CancelToken, climb: Optional[tuple[Vec3, Quat]] = None
    ) -> StepRecord[Released]:
        """The release step: opens `gripper` where its arm stands, then, given
        `climb`, a point and an orientation, moves the arm straight up to it.
        A jaw move that does not complete fails the step. A climb that fails
        is logged and the step still goes through: the jaws are open and the
        item is down, and the arm stays where the climb stopped."""
        inputs = {"gripper": gripper.name, "climb_to": climb[0] if climb is not None else None}
        try:
            opening = await self._grip(gripper.name, OPEN, 0.0, cancel)
        except _Failed as failure:
            return _failed_step("release", inputs, failure)
        measured = {"opening": opening}
        if climb is None:
            return StepRecord("release", inputs, Released(opening), measured=measured)
        point, orientation = climb
        try:
            await self._move(gripper.arm, point, orientation, cancel)
        except _Failed as failure:
            logger.warning(
                "place with gripper '%s' released the item, but the climb back to the pre-place point failed: %s",
                gripper.name, failure.message,
            )
            return StepRecord("release", inputs, Released(opening, climbed=False), measured=measured)
        return StepRecord("release", inputs, Released(opening, climbed=True), measured=measured)

    def _require_started(self) -> None:
        reason = self.why_unavailable()
        if reason:
            raise _Failed(reason)

    def _propose(self, item: Item) -> Grasp:
        try:
            return self.proposer.propose(item)
        except ValueError as error:
            raise _Failed(f"no grasp for item '{item.item_id}': {error}") from None

    async def _carry(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], *,
        heights_m: tuple[float, ...], rise_m: float = 0.0, height_words: str = "the pose", with_lift: bool = False,
        item: Optional[Item] = None,
    ) -> StepRecord[Carried]:
        """The carry step: the transit to the pre point `rise_m` and the
        first of `heights_m` back along the approach from the grasp point of
        `grasp`, then the move there; at each next height when no orientation
        reaches the one before, which `notes` says, the heights above
        `height_words`. `rise_m` is how far the reference of the heights
        stands above the grasp point: 0 for a grab and for a place at a pose,
        the depth of the target for a place into it, whose line down ends at
        its bottom. Its points: the pre point, the grasp point (the end of
        the line down for a place), and for a grab (`with_lift`) the lifted
        point. `notes` gets the orientation that planned and the calls it
        took. No orientation that reaches at any height fails the step
        before anything moves; a transit that does not complete fails it
        and undoes what it added for `item`, and so does a cancel before it
        raises."""
        inputs = {"reference": grasp.point, "orientations": grasp.orientations, "heights_m": heights_m, "rise_m": rise_m}
        checks = _Checks()
        for index, height in enumerate(heights_m):
            try:
                reach = await self._transit(
                    gripper, grasp, target, cancel, notes, checks, clearance_m=rise_m + height, with_lift=with_lift, item=item,
                )
                break
            except _NoReach as failure:
                if index + 1 == len(heights_m):
                    return _failed_step("carry", inputs, failure, StepFailure.NO_REACH)
                notes.append(
                    f"no plan {100 * height:.1f} cm above {height_words}: "
                    f"the pre-place point stands {100 * heights_m[index + 1]:.1f} cm above it"
                )
            except _Failed as failure:
                return _failed_step("carry", inputs, failure)
        try:
            await reach.go(cancel)
        except _Failed as failure:
            await reach.abandon()
            return _failed_step("carry", inputs, failure)
        except Cancelled:
            await reach.abandon()
            raise
        measured = {
            "checks": checks.count, "plans": checks.plans, "reach_tests": checks.lines, "seconds": checks.seconds,
            "height_above_reference_m": height,
        }
        return StepRecord("carry", inputs, Carried(reach.orientation, reach.points, reach.abandon, reach.log_descent), measured=measured)

    async def _transit(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], checks: _Checks, *,
        clearance_m: float, with_lift: bool = False, item: Optional[Item] = None,
    ) -> _Reach:
        """The first orientation of `grasp` whose moves reach, as
        `_first_reach` finds it, with its transit. The only place that asks
        whether the motion planner is bound. Without it, each point is
        checked with check_arm_move and the transit is the move_arm line
        from where the arm is commanded now to the pre point. With it, a
        grab's `item` first gets its object in the planning scene, each
        orientation is planned (`_planned_reach`), and the transit runs the
        plan; abandoning the reach removes the object. A failure that no
        orientation reaches is raised before anything moves, the object
        removed. Each call of a reach test counts in `checks`."""
        if not self._robot.has_planner:
            orientation, points, _ = await self._first_reach(
                gripper, grasp, target, cancel, notes, checks, clearance_m=clearance_m, with_lift=with_lift,
                test=lambda points, orientation, checks: self._checked_reach(gripper.arm, points, orientation, cancel, checks),
            )
            return _Reach(
                orientation, points,
                go=lambda cancel: self._move(gripper.arm, points[0], orientation, cancel), abandon=_nothing, log_descent=_nothing,
            )
        arm = await self._planner_arm(gripper)
        object_id = await self._add_item_object(item, arm) if item is not None else None
        try:
            orientation, points, plan = await self._first_reach(
                gripper, grasp, target, cancel, notes, checks, clearance_m=clearance_m, with_lift=with_lift,
                test=lambda points, orientation, checks: self._planned_reach(arm, points, orientation, cancel, checks),
            )
        except (_Failed, Cancelled):
            await self._remove_item_object(object_id)
            raise
        return _Reach(
            orientation, points,
            go=lambda cancel: self._run_plan(gripper.arm, plan, cancel),
            abandon=lambda: self._remove_item_object(object_id),
            log_descent=lambda point, lift, orientation, cancel: self._log_checked_descent(arm, target, point, lift, orientation, cancel),
        )

    async def _log_checked_descent(
        self, arm: ArmDescription, target: str, point: Vec3, lift: Vec3, orientation: Quat, cancel: CancelToken
    ) -> None:
        """Logs what a checked descent would answer: the share of the lines
        from where the arm stands down to `point` and up to `lift` that a
        plan_cartesian_path with the collision check covers. Log only: the
        grab descends by its lines whatever the answer."""
        checked = await self._robot.plan_cartesian_path(
            arm.name, arm.grasp_link, [(point, orientation), (lift, orientation)], avoid_collisions=True,
        )
        cancel.check()
        if checked.completed:
            answer = f"cover {checked.fraction:.2f} of their lines"
        else:
            answer = f"have no answer: {checked.words or 'plan_cartesian_path gave none'}"
        logger.info(
            "checked descent of %s with arm '%s' (log only, the grab descends by lines): the descent and the lift from where the arm stands %s",
            target, arm.name, answer,
        )

    async def _add_item_object(self, item: Item, arm: ArmDescription) -> Optional[str]:
        """Adds the object of `item` to the planning scene at its place, the
        box its look measured or a sphere at its position for an item no look
        measured, and lets the links of the gripper of `arm` touch it.
        Returns its id, or none when the add failed: the failure is logged,
        the grab goes on, and the hold puts the object at the hand."""
        object_id = brain_object_id(item.item_id)
        centre = item.shape.centre if item.shape is not None else item.position
        reason = await self._robot.apply_collision_objects([item_object(item.item_id, item.shape, centre)])
        if reason:
            logger.warning("planning scene: add %s at the item failed: %s", object_id, reason)
            return None
        what = "its box" if item.shape is not None else "a sphere, no look measured it"
        logger.info("planning scene: add %s at the item, %s, at %s", object_id, what, _numbers(centre))
        reason = await self._robot.set_allowed_collision(object_id, arm.gripper_links, True)
        if reason:
            logger.warning("planning scene: let the gripper links of %s touch %s failed: %s", arm.name, object_id, reason)
        else:
            logger.info("planning scene: let the gripper links of %s touch %s", arm.name, object_id)
        return object_id

    async def _remove_item_object(self, object_id: Optional[str]) -> None:
        """Removes the object a grab added, after the grab failed or
        stopped; nothing when it added none."""
        if object_id is None:
            return
        reason = await self._robot.apply_collision_objects([ObjectOperation.remove(object_id)])
        if reason:
            logger.warning("planning scene: remove %s after the grab failed: %s", object_id, reason)
        else:
            logger.info("planning scene: remove %s, the grab did not hold it", object_id)

    async def _planner_arm(self, gripper: Gripper) -> ArmDescription:
        """The planner's description of the arm of `gripper`: its grasp link
        and its gripper links."""
        described = await self._robot.describe_arms()
        arm = described.arm(gripper.arm)
        if arm is None:
            why = described.reason or "it names no such arm"
            raise _Failed(f"the motion planner does not describe arm '{gripper.arm}': {why}")
        return arm

    async def _checked_reach(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> tuple[tuple[int, str] | None, None]:
        """The reach test of an orientation with no planner: check_arm_move
        of each point, the first that has no plan with its words."""
        return await self._first_refused_point(arm, points, orientation, cancel, checks), None

    async def _planned_reach(
        self, arm: ArmDescription, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> tuple[tuple[int, str] | None, Optional[JointPlan]]:
        """The reach test of an orientation with the planner: a plan of the
        transit to the pre point, then the reach of the lines through the
        points that follow it, from the end of that plan, with no collision
        check. The index of the first point that does not pass with the
        planner's words, or none and the plan."""
        started = self._clock()
        plan = await self._robot.plan_joint_path(
            arm.name, arm.grasp_link, points[0], orientation,
            position_tolerance_m=PLAN_POSITION_TOLERANCE_M, orientation_tolerance_rad=PLAN_ORIENTATION_TOLERANCE_RAD,
            planner=PLANNER, refiner=REFINER, allowed_planning_time_s=self.allowed_planning_time_s,
        )
        checks.plans += 1
        checks.seconds += self._clock() - started
        cancel.check()
        if plan.cancelled:
            raise _Failed(plan.message or "plan_joint_path was cancelled", stopped=True)
        if not plan.completed:
            return (0, plan.words or "plan_joint_path found no path"), None
        started = self._clock()
        line = await self._robot.plan_cartesian_path(
            arm.name, arm.grasp_link, [(point, orientation) for point in points[1:]],
            avoid_collisions=False, start_joint_positions=plan.waypoints[-1].joint_positions,
        )
        checks.lines += 1
        checks.seconds += self._clock() - started
        cancel.check()
        if line.cancelled:
            raise _Failed(line.message or "plan_cartesian_path was cancelled", stopped=True)
        if not line.completed:
            return (1, line.words or "plan_cartesian_path gave no path"), None
        if line.fraction < FULL_REACH:
            lines = len(points) - 1
            short = min(lines, 1 + int(line.fraction * lines))
            return (short, f"plan_cartesian_path reaches {line.fraction:.2f} of the lines from the end of the plan"), None
        return None, plan

    async def _run_plan(self, arm: str, plan: JointPlan, cancel: CancelToken) -> None:
        """Runs the planned transit `plan` of `arm`; the result comes once
        each joint has settled at the plan's last waypoint. A transit that
        ran logs its own line, for a grab and for a place."""
        ran = await self._robot.execute_joint_path(
            arm, plan, duration_s=MOVE_DURATION_S, settle_tolerance_rad=SETTLE_JOINT_RAD, settle_timeout_s=self.settle_timeout_s,
            contact_gap_m=NO_CONTACT_CHECK_M,
        )
        cancel.check()
        if not ran.completed:
            raise _Failed(ran.message or "execute_joint_path failed", stopped=ran.cancelled)
        end = plan.waypoints[-1]
        logger.info(
            "planned transit of arm '%s' ran: %d waypoint%s to the grasp point %s", arm, len(plan.waypoints),
            "" if len(plan.waypoints) == 1 else "s", _numbers(end.position) if end.position is not None else "unknown",
        )

    async def _first_reach(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], checks: _Checks, *,
        clearance_m: float, with_lift: bool = False,
        test: Callable[[tuple[Vec3, ...], Quat, _Checks], Awaitable[tuple[tuple[int, str] | None, object]]],
    ) -> tuple[Quat, tuple[Vec3, ...], object]:
        """The first orientation of `grasp` whose moves all pass `test`, with
        the points of those moves in order and what `test` gave for them.
        The points: the pre point, `clearance_m` back along the approach from
        the grasp point; then the grasp point, the pose for a place; then for
        a grab (`with_lift`) the lifted point. With none, the failure names
        the last orientation tried, its point that does not pass and the
        words for it. Each call of `test` counts in `checks`."""
        names = ("pre-grasp point", "grasp point", "lifted point") if with_lift else ("pre-place point", "place point")
        last_refusal = ""
        for index, orientation in enumerate(grasp.orientations):
            points = (pre_grasp(grasp.point, orientation, clearance_m), grasp.point)
            if with_lift:
                points += (lifted(grasp.point, self.lift_m),)
            refused, found = await test(points, orientation, checks)
            tried = f"orientation {index + 1} of {len(grasp.orientations)}"
            if refused is None:
                notes.append(f"{tried} planned, {_numbers(orientation)} ({checks})")
                return orientation, points, found
            point, words = refused
            last_refusal = (
                f"{tried}, {_numbers(orientation)}, has no plan to its {names[point]} "
                f"{_numbers(points[point])}: {words}"
            )
        raise _NoReach(f"no plan reaches {target} with gripper '{gripper.name}': {last_refusal}")

    async def _first_refusal(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> str:
        """The backbone's words for the first of `points` it has no plan
        for, or "" when it has a plan for each. Each check counts in
        `checks`."""
        refused = await self._first_refused_point(arm, points, orientation, cancel, checks)
        return "" if refused is None else refused[1]

    async def _first_refused_point(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> tuple[int, str] | None:
        """The index of the first of `points` the backbone has no plan for,
        with its words, or None when it has a plan for each. Each check
        counts in `checks`."""
        for index, point in enumerate(points):
            started = self._clock()
            check = await until(
                cancel.wait(),
                self._robot.check_arm_move(
                    arm, point, orientation, duration_s=MOVE_DURATION_S,
                ),
            )
            checks.count += 1
            checks.seconds += self._clock() - started
            cancel.check()
            if not check.reachable:
                return index, check.reason or "check_arm_move found no plan"
        return None

    async def _move(
        self, arm: str, point: Vec3, orientation: Quat, cancel: CancelToken, *,
        duration_s: float = MOVE_DURATION_S, contact_gap_m: float = NO_CONTACT_CHECK_M,
    ) -> MoveResult:
        """Moves the grasp point of `arm` to `point` in `orientation`; the
        result comes once the arm has settled there, or once it ended on
        contact with a `contact_gap_m` above 0. Any other end fails."""
        moved = await self._move_arm(arm, point, orientation, duration_s=duration_s, contact_gap_m=contact_gap_m)
        cancel.check()
        if not moved.completed:
            raise _Failed(moved.message or "move_arm failed", stopped=moved.cancelled)
        return moved

    async def _descend(self, arm: str, start: Vec3, end: Vec3, orientation: Quat, cancel: CancelToken) -> StepRecord[Landed]:
        """The descend step: the line down of `arm` from `start`, where it
        stands, to `end`. It stops on contact with `line_down_contact_gap_m`
        and runs at `line_down_speed_m_s`. It goes through when the move
        completes, settled at `end` or on contact, and measures how far
        short of `end` the grasp point stopped; any other end fails it."""
        duration_s = line_down_duration_s(start, end, self.line_down_speed_m_s)
        inputs = {
            "start": start, "end": end, "orientation": orientation,
            "contact_gap_m": self.line_down_contact_gap_m, "duration_s": duration_s,
        }
        try:
            moved = await self._move(arm, end, orientation, cancel, duration_s=duration_s, contact_gap_m=self.line_down_contact_gap_m)
        except _Failed as failure:
            return _failed_step("descend", inputs, failure)
        measured = {} if moved.final_position is None else {"short_of_end_m": math.dist(moved.final_position, end)}
        return StepRecord("descend", inputs, Landed(moved.final_position, moved.ended_on_contact), measured=measured)

    async def _move_arm(
        self, arm: str, point: Vec3, orientation: Quat, *,
        duration_s: float = MOVE_DURATION_S, contact_gap_m: float = NO_CONTACT_CHECK_M,
    ) -> MoveResult:
        """One move_arm with the settle bars of every arm move, the fastest
        duration and no contact check unless the caller names others."""
        return await self._robot.move_arm(
            arm, point, orientation, duration_s=duration_s,
            settle_position_tolerance_m=self.settle_position_m, settle_orientation_tolerance_rad=self.settle_orientation_rad,
            settle_timeout_s=self.settle_timeout_s, contact_gap_m=contact_gap_m,
        )

    async def _grip(self, gripper: str, opening: float, max_effort: float, cancel: CancelToken) -> float:
        """Drives `gripper` to `opening` and returns its final opening."""
        gripped = await self._robot.set_gripper(gripper, opening, max_effort=max_effort, settle_timeout_s=self.grip_settle_timeout_s)
        cancel.check()
        if not gripped.completed:
            raise _Failed(gripped.message or "move_gripper failed", stopped=gripped.cancelled)
        if gripped.final_opening is None:
            raise _Failed(f"move_gripper gave no final opening for gripper '{gripper}'")
        return gripped.final_opening

    async def _snapshot_after(self, before: Optional[LimbSnapshot], cancel: CancelToken) -> LimbSnapshot:
        """The first snapshot captured after `before`, the latest one when
        the lift's result came back."""
        stamp = before.timestamp if before is not None else float("-inf")
        after = await until(cancel.wait(), self._limb_states.newer_than(stamp, self.snapshot_wait_s))
        cancel.check()
        if after is None:
            raise _Failed("no limb_states snapshot after the lift")
        return after


def line_down_duration_s(start: Vec3, end: Vec3, speed_m_s: float) -> float:
    """The duration of a line down from `start` to `end`: its length divided
    by `speed_m_s`."""
    return math.dist(start, end) / speed_m_s


def _offset_cm(position: Optional[Vec3], commanded: Vec3) -> str:
    """`position` minus `commanded` per axis, in centimetres, for the log."""
    if position is None:
        return "unknown"
    return "[" + ", ".join(f"{100 * (p - c):+.1f}" for p, c in zip(position, commanded)) + "] cm"


def _opening_of(snapshot: Optional[LimbSnapshot], gripper: str) -> float:
    opening = snapshot.opening(gripper) if snapshot is not None else None
    if opening is None:
        raise _Failed(f"limb_states names no gripper '{gripper}'")
    return opening


def _numbers(values: Sequence[float]) -> str:
    """`values` as a failure or a note names a point or an orientation: in
    brackets, with 3 decimals each."""
    return f"[{', '.join(f'{value:.3f}' for value in values)}]"
