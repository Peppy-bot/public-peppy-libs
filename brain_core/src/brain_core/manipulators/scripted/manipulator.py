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
replaces it. A grab plans with the jaws open as it opened them, so the
planner examines the open hand for the full transit. A refused start
(`start_in_collision`) ends the search at its orientation: the start is
where the arm stands, the same for each orientation. The grab then takes
its second path: it closes the jaws and plans each orientation again with
the hand closed. A grab takes a gripper that holds nothing, so the
close is safe, also when a place left the jaws open. The run of that plan
ends at the pre-grasp point, where the jaws open for the item and the grab
asks the planner whether the arm is clear where it stands, with the request
of the checked descent below: its start is where the arm stands, and the
planner refuses a start in collision as it refuses that of a plan. A path
in the answer lets the grab go on as on the first path. Any other answer
fails the grab with the planner's words, the arm stays at the pre-grasp
point, and the grab tries no other orientation: a start in collision says
that the arm is not clear, and any other answer that the planner cannot
tell, since the plan examined the closed hand alone. A refusal with the hand
closed too fails the grab with the planner's words before the arm moves.
Without the planner, a grab has the first path alone. The descent, the
lift, the retreat and the climb stay move_arm lines either way. Before a
grab plans, it adds the item's object, `brain/<item_id>`, to the planning
scene at the item's place, the box its look measured or a sphere for an
item no look measured, and lets the gripper's links touch it; the add
clears the item's cubes from the planner's map, so none stay under the
hand after the lift. Each way a grab fails or stops removes the object; a
grab that holds leaves it for the brain to attach. Just before the
descent, after the aligner, a grab with the planner asks it what a checked
descent would answer: a plan_cartesian_path with the collision check from
where the arm stands through the grasp and lifted points. On the second
path with no aligner, the check at the pre-grasp point is that request
from the same place, and the log reads its answer. It logs the share of
the lines that path covers and moves by the lines of today either way.

Each step that fails ends the grab with the backbone's own words. A grab
or a place that no orientation plans names the last orientation it tried,
the point of it that has no plan and the backbone's words for it. A step
that fails once the descent has begun (the descent itself, the close, the
two hold judgements, the lift, the wait for a snapshot) first retreats:
the gripper opens and the arm moves straight back up to the pre-grasp
point, so the jaws let go of what they touch and the next look sees the
item. A retreat that stops short adds why to the failure. A step whose
goal a stop ended (the brain's stop or the backbone's stop service)
retreats from nothing: the arm stays where the stop left it, and the jaws
stay as they are. A step that fails after the jaws opened and before the
descent, a grab that no orientation reaches included, leaves the arm where
the backbone stopped it and puts the jaws back to the opening that the
latest limb_states snapshot gave before the grab. Once the arm moved toward
the item, the put back does not open the jaws wider there: it closes them
to that opening when it is narrower than the opening of the latest
snapshot, and they stay as they stand otherwise. A put back that stops
short adds why to the failure, and one that a stop ended ends the grab as
a stop.

Each grab leaves one log line: the orientation that planned, with how many
check_arm_move calls ran and their total time; how wide the jaws opened for
the item's width; the second path and why the grab took it; where the
descent's result put the grasp point against the commanded one, and
whether the descent ended on contact; limb_state's grasp point at the
close; the two openings it judged; and the retreat or the put back of the
jaws.

A drop opens the gripper where it is. A place first reads the gripper's
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

The manipulator keeps nothing between goals: ids and held flags live in
`State`.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, replace
from typing import Awaitable, Callable, Optional, Sequence

from ...limb_state import LimbSnapshot
from ...ports import Cancelled, CancelToken, Gripper, Item, Outcome, Pose, Quat, Senses, Vec3
from ...robot import ArmDescription, CartesianPath, JointPlan, MoveResult, ObjectOperation, brain_object_id, item_object
from ...waiting import until
from .aligning import Aligned, Aligner, AlignmentFailed, AlignRequest, GaveUp, OutOfSteps
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
# The motion_planning failure kind of a plan or a path whose start keeps a
# part of the robot nearer to a thing than the clearance of the planner.
START_IN_COLLISION = "start_in_collision"
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


class _Failed(Exception):
    """A step of a sequence that did not go through: `message` ends the
    sequence's outcome. `stopped` when the step's goal ended as cancelled,
    by a stop, so nothing more moves."""

    def __init__(self, message: str, *, stopped: bool = False) -> None:
        super().__init__(message)
        self.message = message
        self.stopped = stopped


class _NoReach(_Failed):
    """No orientation of a grasp reaches: the arm did not move, and a place
    tries its lower pre-place point. `start_refused` when the planner
    refused the start of the plan to the pre point: where the arm stands,
    with the jaws as they stand, a part of it is too near a thing. The
    start is the same for each orientation, so no other orientation
    plans."""

    def __init__(self, message: str, *, start_refused: bool = False) -> None:
        super().__init__(message)
        self.start_refused = start_refused


@dataclass(frozen=True)
class _Refused:
    """The first point of the moves of an orientation that does not pass
    its reach test: its index among the points, the words for it, and
    whether the planner refused the start of the plan to the pre point
    (`start_in_collision`)."""

    point: int
    words: str
    start_refused: bool = False


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
    point (on the second path of a grab, also the open of the jaws there and
    the check that the arm is clear, whose answer it returns), `abandon`
    and `log_descent`."""

    orientation: Quat
    points: tuple[Vec3, ...]
    go: Callable[[CancelToken], Awaitable[Optional[CartesianPath]]]
    # What a grab that fails or stops undoes of the transit: the item's
    # object it added to the planning scene, nothing with no planner.
    abandon: Callable[[], Awaitable[None]]
    # The log of what a checked descent from where the arm stands would
    # answer, given the grasp point, the lifted point, the orientation and
    # the answer that `go` returned from where the arm stands, if any;
    # nothing with no planner.
    log_descent: Callable[[Vec3, Vec3, Quat, Optional[CartesianPath], CancelToken], Awaitable[None]]


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
        try:
            outcome = await self._grab(item, gripper, max_effort, cancel, notes)
        except _Failed as failure:
            notes.append(f"failed: {failure.message}")
            outcome = Outcome(False, failure.message)
        except Cancelled as cancelled:
            notes.append(f"stopped: {cancelled.reason}")
            raise
        finally:
            logger.info("grab of item '%s' with gripper '%s': %s", item.item_id, gripper.name, "; ".join(notes))
        return outcome

    async def drop(self, gripper: Gripper, cancel: CancelToken) -> Outcome:
        try:
            self._require_started()
            await self._grip(gripper.name, OPEN, 0.0, cancel)
        except _Failed as failure:
            return Outcome(False, failure.message)
        return Outcome(True)

    async def place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome:
        try:
            return await self._place(gripper, pose, cancel)
        except _Failed as failure:
            return Outcome(False, failure.message)

    async def stop(self) -> None:
        """Nothing of its own: the lane's stop cancels the robot's move in
        flight, and the sequence ends at its next cancel check."""
        return None

    async def _grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken, notes: list[str]) -> Outcome:
        self._require_started()
        # Before any move: a gripper limb_states does not name could never
        # be judged after the lift. A grab that fails before its descent
        # puts the jaws back to this opening.
        opening_before = _opening_of(self._limb_states.latest, gripper.name)
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
        try:
            reach = await self._transit(
                gripper, grasp, f"item '{item.item_id}'", cancel, notes,
                clearance_m=self.clearance_m, with_lift=True, item=item, opening=opening,
            )
        except _Failed as failure:
            raise await self._put_jaws_back(failure, gripper, opening_before, cancel, notes) from None
        try:
            reach = await self._approach(item, gripper, reach, cancel, notes)
        except _Failed as failure:
            raise await self._put_jaws_back_where_moved(failure, gripper, opening_before, cancel, notes) from None
        try:
            return await self._descend_or_retreat(item, gripper, max_effort, reach, cancel, notes)
        except (_Failed, Cancelled):
            await reach.abandon()
            raise

    async def _approach(self, item: Item, gripper: Gripper, reach: _Reach, cancel: CancelToken, notes: list[str]) -> _Reach:
        """The grab from its transit to its descent: the run of the transit
        of `reach` to the pre-grasp point, then the aligner's correction above
        the item and the log of the checked descent. Returns the reach with
        the points and the orientation that the descent goes on with. A step
        that fails removes the item's object."""
        try:
            checked = await reach.go(cancel)
            if self.aligner is not None:
                pre, point, _ = reach.points
                pre, point, lift, orientation = await self._align(item, gripper, pre, point, reach.orientation, cancel, notes)
                reach = replace(reach, orientation=orientation, points=(pre, point, lift))
                # The aligner moved the arm: an answer of the run is for
                # another start.
                checked = None
            _, point, lift = reach.points
            await reach.log_descent(point, lift, reach.orientation, checked, cancel)
        except (_Failed, Cancelled):
            await reach.abandon()
            raise
        return reach

    async def _put_jaws_back(
        self, failure: _Failed, gripper: Gripper, opening: float, cancel: CancelToken, notes: list[str]
    ) -> _Failed:
        """The failure that ends a grab that failed before its descent, once
        the jaws of `gripper` went back to `opening`, the opening that they
        had before the grab: `failure` itself when they got there, or
        `failure` with why not (`_after_undo`). A failure that a stop ended
        puts nothing back. A cancel raises Cancelled."""
        if failure.stopped:
            return failure
        # A measured opening can stand a little outside the 0 to 1 that a
        # gripper move takes.
        opening = min(OPEN, max(CLOSED, opening))
        try:
            await self._grip(gripper.name, opening, 0.0, cancel)
        except _Failed as undo:
            undone: Optional[_Failed] = undo
        else:
            undone = None
        return _after_undo(
            failure, undone, notes, undo=f"putting the jaws back to {opening:.2f}", done=f"put the jaws back to {opening:.2f}",
        )

    async def _put_jaws_back_where_moved(
        self, failure: _Failed, gripper: Gripper, opening_before: float, cancel: CancelToken, notes: list[str]
    ) -> _Failed:
        """`_put_jaws_back` for a grab that failed after its arm moved toward
        the item: the jaws close to `opening_before` when it is narrower than
        the opening that the latest limb_states snapshot gives, and stay as
        they are otherwise, so that they never open wider where the arm
        stopped, above the item."""
        standing = _opening_of(self._limb_states.latest, gripper.name)
        if opening_before < standing:
            return await self._put_jaws_back(failure, gripper, opening_before, cancel, notes)
        notes.append(
            f"the jaws stay at {standing:.2f}: the put back to {opening_before:.2f} does not open them wider where the arm stopped"
        )
        return failure

    async def _descend_or_retreat(
        self, item: Item, gripper: Gripper, max_effort: float, reach: _Reach, cancel: CancelToken, notes: list[str]
    ) -> Outcome:
        """The grab from its descent on: the descent, the close and the lift
        at the points of `reach`, with the retreat after a step that
        fails."""
        orientation, (pre, point, lift) = reach.orientation, reach.points
        try:
            return await self._descend_close_lift(item, gripper, max_effort, orientation, pre, point, lift, cancel, notes)
        except _Failed as failure:
            if failure.stopped:
                raise
            undone = await self._retreat(gripper, pre, orientation, cancel)
            raise _after_undo(
                failure, undone, notes, undo="the retreat to the pre-grasp point", done="retreated to the pre-grasp point",
            ) from None

    async def _descend_close_lift(
        self, item: Item, gripper: Gripper, max_effort: float, orientation: Quat, pre: Vec3, point: Vec3, lift: Vec3,
        cancel: CancelToken, notes: list[str],
    ) -> Outcome:
        """The grab from its descent on: the line down from `pre` to `point`,
        close where it ended, judge, lift to `lift`, judge again. A hold
        reports where limb_states put the grasp point when the jaws
        closed."""
        landed = await self._line_down(gripper.arm, pre, point, orientation, cancel)
        on_contact = ", on contact" if landed.ended_on_contact else ""
        notes.append(f"landed {_offset_cm(landed.final_position, point)} from the grasp point{on_contact}")
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
        return Outcome(True, closed_at=closed_at)

    async def _align(
        self, item: Item, gripper: Gripper, pre: Vec3, point: Vec3, orientation: Quat, cancel: CancelToken, notes: list[str]
    ) -> tuple[Vec3, Vec3, Vec3, Quat]:
        """The aligner's correction above the item: the pre-grasp, grasp
        and lifted points and the orientation the grab goes on with."""
        return await self._run_alignment(AlignRequest(item, gripper, pre, point, orientation, self.lift_m), cancel, notes)

    async def _run_alignment(self, request: AlignRequest, cancel: CancelToken, notes: list[str]) -> tuple[Vec3, Vec3, Vec3, Quat]:
        """One alignment above the point of `request`: the pre, point and
        lifted points and the orientation the sequence goes on with. A
        model that gives up, runs out of steps or has a move stopped ends
        the sequence here, before the descent, so nothing retreats."""
        pre = request.pre
        try:
            alignment = await self.aligner.align(request, self, cancel)
        except AlignmentFailed as failure:
            raise _Failed(f"the alignment failed: {failure.message}") from None
        if isinstance(alignment, Aligned):
            moved_cm = 100 * math.dist(alignment.pre[:2], pre[:2])
            notes.append(
                f"aligned in {alignment.steps} step{'' if alignment.steps == 1 else 's'}: the grasp point moved {moved_cm:.1f} cm, "
                f"the jaws close along {jaw_line_deg(alignment.orientation):.0f} degrees"
            )
            return alignment.pre, alignment.point, alignment.lift, alignment.orientation
        if isinstance(alignment, GaveUp):
            raise _Failed(f"the model gave up after {alignment.steps} step{'' if alignment.steps == 1 else 's'}")
        if isinstance(alignment, OutOfSteps):
            raise _Failed(f"no ready in {alignment.steps} steps")
        raise _Failed(alignment.message, stopped=True)

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

    async def _retreat(self, gripper: Gripper, pre: Vec3, orientation: Quat, cancel: CancelToken) -> Optional[_Failed]:
        """Opens `gripper` and moves its arm straight up to `pre`, after a
        grab that failed past its descent. Returns None once there, or the
        failure of the step that stopped the retreat short; a cancel raises
        Cancelled."""
        try:
            await self._grip(gripper.name, OPEN, 0.0, cancel)
            await self._move(gripper.arm, pre, orientation, cancel)
        except _Failed as failure:
            return failure
        return None

    def _arm_position(self, arm: str) -> Optional[Vec3]:
        """The grasp point of `arm` in the latest limb_states snapshot."""
        snapshot = self._limb_states.latest
        state = snapshot.arms.get(arm) if snapshot is not None else None
        return state.position if state is not None else None

    async def _place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome:
        self._require_started()
        # Jaws closed on air since the grab: the item slipped on the way, and
        # a place would carry nothing. The caller learns it here, before any
        # move, and drops to let the brain forget the item.
        opening = self._limb_states.latest.opening(gripper.name)
        if opening is not None and opening < self.hold_opening_min:
            raise _Failed(f"gripper '{gripper.name}' holds nothing now: its jaws stand at {opening:.2f}, closed on air; drop to let it go")
        if self.aligner is not None:
            reason = self.aligner.why_unavailable(gripper.arm)
            if reason:
                raise _Failed(f"no alignment: {reason}")
        try:
            orientations = orientations_for(pose.orientation)
        except ValueError as error:
            raise _Failed(str(error)) from None
        target = f"the pose {_numbers(pose.position)}"
        notes: list[str] = []
        reach = await self._place_reach(gripper, Grasp(pose.position, orientations), target, cancel, notes)
        orientation, (pre, point) = reach.orientation, reach.points
        await reach.go(cancel)
        if self.aligner is not None:
            request = AlignRequest(None, gripper, pre, point, orientation, self.lift_m, target=pose.position)
            pre, point, _lift, orientation = await self._run_alignment(request, cancel, notes)
        if notes:
            logger.info("place with gripper '%s': %s", gripper.name, "; ".join(notes))
        landed = await self._line_down(gripper.arm, pre, point, orientation, cancel)
        if landed.ended_on_contact:
            logger.info(
                "place with gripper '%s' ended its line down on contact %s from the place point: it releases there",
                gripper.name, _offset_cm(landed.final_position, point),
            )
        await self._grip(gripper.name, OPEN, 0.0, cancel)
        try:
            await self._move(gripper.arm, pre, orientation, cancel)
        except _Failed as failure:
            # The jaws are open and the item is down: the place went
            # through, and the arm stays where the climb stopped.
            logger.warning(
                "place with gripper '%s' released the item, but the climb back to the pre-place point failed: %s",
                gripper.name, failure.message,
            )
        return Outcome(True, "", landed.final_position)

    def _require_started(self) -> None:
        reason = self.why_unavailable()
        if reason:
            raise _Failed(reason)

    def _propose(self, item: Item) -> Grasp:
        try:
            return self.proposer.propose(item)
        except ValueError as error:
            raise _Failed(f"no grasp for item '{item.item_id}': {error}") from None

    async def _place_reach(self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str]) -> _Reach:
        """The transit of a place: to the pre-place point `clearance_m` and
        `held_item_depth_m` above the pose when the backbone has a plan that
        high, else `clearance_m` above it, which `notes` says. Its points are
        the pre-place point and the pose, where the line down ends."""
        high = self.clearance_m + self.held_item_depth_m
        try:
            return await self._transit(gripper, grasp, target, cancel, [], clearance_m=high)
        except _NoReach:
            notes.append(f"no plan {100 * high:.1f} cm above the pose: the pre-place point stands {100 * self.clearance_m:.1f} cm above it")
            return await self._transit(gripper, grasp, target, cancel, [], clearance_m=self.clearance_m)

    async def _transit(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], *,
        clearance_m: float, with_lift: bool = False, item: Optional[Item] = None,
        opening: Optional[float] = None,
    ) -> _Reach:
        """The first orientation of `grasp` whose moves reach, as
        `_first_reach` finds it, with its transit. The only place that asks
        whether the motion planner is bound. Without it, each point is
        checked with check_arm_move and the transit is the move_arm line
        from where the arm is commanded now to the pre point. With it, a
        grab's `item` first gets its object in the planning scene, each
        orientation is planned (`_planned_reach`), and the transit runs the
        plan; a grab, which gives the `opening` of its jaws above the item,
        can take the second path (`_planned_transit`). Abandoning the reach
        removes the object. A failure that no orientation reaches is raised
        before the arm moves, the object removed."""

        def first_reach(
            test: Callable[[tuple[Vec3, ...], Quat, _Checks], Awaitable[tuple[Optional[_Refused], object]]],
        ) -> Awaitable[tuple[Quat, tuple[Vec3, ...], object]]:
            return self._first_reach(
                gripper, grasp, target, cancel, notes, clearance_m=clearance_m, with_lift=with_lift,
                test=test,
            )

        if not self._robot.has_planner:
            orientation, points, _ = await first_reach(
                lambda points, orientation, checks: self._checked_reach(gripper.arm, points, orientation, cancel, checks),
            )

            async def move_to_pre(cancel: CancelToken) -> None:
                await self._move(gripper.arm, points[0], orientation, cancel)

            return _Reach(orientation, points, go=move_to_pre, abandon=_nothing, log_descent=_nothing)
        arm = await self._planner_arm(gripper)
        object_id = await self._add_item_object(item, arm) if item is not None else None
        try:
            orientation, points, go = await self._planned_transit(
                arm, gripper, opening, cancel, notes,
                plan=lambda: first_reach(
                    lambda points, orientation, checks: self._planned_reach(arm, points, orientation, cancel, checks),
                ),
            )
        except (_Failed, Cancelled):
            await self._remove_item_object(object_id)
            raise
        return _Reach(
            orientation, points,
            go=go,
            abandon=lambda: self._remove_item_object(object_id),
            log_descent=lambda point, lift, orientation, checked, cancel: self._log_checked_descent(
                arm, target, point, lift, orientation, checked, cancel,
            ),
        )

    async def _planned_transit(
        self, arm: ArmDescription, gripper: Gripper, opening: Optional[float], cancel: CancelToken, notes: list[str], *,
        plan: Callable[[], Awaitable[tuple[Quat, tuple[Vec3, ...], JointPlan]]],
    ) -> tuple[Quat, tuple[Vec3, ...], Callable[[CancelToken], Awaitable[Optional[CartesianPath]]]]:
        """The transit with the planner: the orientation and the points that
        `plan` finds, from where the arm stands with the jaws as they stand,
        and the run of their plan. The first path is that plan. A grab,
        which gives `opening`, takes the second path
        (`_closed_jaws_transit`) only when the planner refuses the start,
        where the arm stands; a place, and a refusal of another kind, fail
        with the planner's words."""
        try:
            orientation, points, joint_plan = await plan()
        except _NoReach as refused:
            if opening is None or not refused.start_refused:
                raise
            return await self._closed_jaws_transit(arm, gripper, opening, refused, cancel, notes, plan=plan)
        async def run(cancel: CancelToken) -> None:
            await self._run_plan(gripper.arm, joint_plan, cancel)

        return orientation, points, run

    async def _closed_jaws_transit(
        self, arm: ArmDescription, gripper: Gripper, opening: float, refused: _NoReach, cancel: CancelToken, notes: list[str], *,
        plan: Callable[[], Awaitable[tuple[Quat, tuple[Vec3, ...], JointPlan]]],
    ) -> tuple[Quat, tuple[Vec3, ...], Callable[[CancelToken], Awaitable[Optional[CartesianPath]]]]:
        """The second path of a grab, after the planner `refused` the start
        with the jaws open. The jaws close, as they
        stand after a place too: a grab takes a gripper that holds nothing,
        so the close is safe. `plan` then searches again with the jaws
        closed. Its run ends at the pre-grasp point, where the jaws open to
        `opening` and the planner must find the arm clear
        (`_open_where_clear`); it returns the planner's answer there. A
        search with the jaws closed that finds no orientation fails with the
        planner's words, before the arm moves."""
        notes.append(f"with the jaws open, {refused.message}")
        notes.append("the second path: the jaws close for the transit and open at the pre-grasp point")
        await self._grip(gripper.name, CLOSED, 0.0, cancel)
        try:
            orientation, points, joint_plan = await plan()
        except _NoReach as closed:
            raise _NoReach(
                f"the planner refuses the start where the arm stands with the jaws open, and with the jaws closed {closed.message}"
            ) from None

        async def run_then_open(cancel: CancelToken) -> CartesianPath:
            await self._run_plan(gripper.arm, joint_plan, cancel)
            return await self._open_where_clear(arm, gripper, opening, points, orientation, cancel, notes)

        return orientation, points, run_then_open

    async def _open_where_clear(
        self, arm: ArmDescription, gripper: Gripper, opening: float, points: tuple[Vec3, ...], orientation: Quat,
        cancel: CancelToken, notes: list[str],
    ) -> CartesianPath:
        """The end of the second path at the pre-grasp point, the first of
        `points`: opens the jaws to `opening`, then asks the planner whether
        the arm is clear where it stands, the open jaws included, with the
        request of the checked descent (`_checked_descent`), and returns its
        answer. Its start is where the arm stands, and the planner refuses a
        start in collision as it refuses that of a plan. A path in the
        answer lets the grab go on. Any other answer fails the grab with the
        planner's words, the arm at the pre-grasp point, and no other
        orientation is tried: a start in collision says that the arm is not
        clear, and any other answer that the planner cannot tell, since the
        plan of the transit examined the closed hand alone."""
        pre, point, lift = points
        await self._grip(gripper.name, opening, 0.0, cancel)
        checked = await self._checked_descent(arm, point, lift, orientation, cancel)
        if checked.cancelled:
            raise _Failed(checked.message or "plan_cartesian_path was cancelled", stopped=True)
        if checked.completed:
            notes.append("the jaws opened at the pre-grasp point, and the planner finds the arm clear there")
            return checked
        where = f"at the pre-grasp point {_numbers(pre)} with the jaws of gripper '{gripper.name}' open"
        words = checked.words or "plan_cartesian_path gave no answer"
        if checked.failure_kind == START_IN_COLLISION:
            raise _Failed(f"the planner does not find the arm clear {where}: {words}")
        raise _Failed(f"the planner cannot tell whether the arm is clear {where}: {words}")

    async def _checked_descent(
        self, arm: ArmDescription, point: Vec3, lift: Vec3, orientation: Quat, cancel: CancelToken
    ) -> CartesianPath:
        """The planner's answer for a checked descent: a plan_cartesian_path
        with the collision check from where the arm stands, with the jaws as
        they stand, down to `point` and up to `lift`."""
        checked = await self._robot.plan_cartesian_path(
            arm.name, arm.grasp_link, [(point, orientation), (lift, orientation)], avoid_collisions=True,
        )
        cancel.check()
        return checked

    async def _log_checked_descent(
        self, arm: ArmDescription, target: str, point: Vec3, lift: Vec3, orientation: Quat,
        checked: Optional[CartesianPath], cancel: CancelToken,
    ) -> None:
        """Logs what a checked descent would answer: the share of the lines
        from where the arm stands down to `point` and up to `lift` that a
        plan_cartesian_path with the collision check covers. `checked` is
        that answer when the transit already asked for it from where the
        arm stands; with none, the log asks. Log only: the grab descends by
        its lines whatever the answer."""
        if checked is None:
            checked = await self._checked_descent(arm, point, lift, orientation, cancel)
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
    ) -> tuple[Optional[_Refused], None]:
        """The reach test of an orientation with no planner: check_arm_move
        of each point, the first that has no plan with its words."""
        return await self._first_refused_point(arm, points, orientation, cancel, checks), None

    async def _planned_reach(
        self, arm: ArmDescription, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> tuple[Optional[_Refused], Optional[JointPlan]]:
        """The reach test of an orientation with the planner: a plan of the
        transit to the pre point, then the reach of the lines through the
        points that follow it, from the end of that plan, with no collision
        check. The first point that does not pass with the planner's words,
        or none and the plan."""
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
            words = plan.words or "plan_joint_path found no path"
            return _Refused(0, words, start_refused=plan.failure_kind == START_IN_COLLISION), None
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
            return _Refused(1, line.words or "plan_cartesian_path gave no path"), None
        if line.fraction < FULL_REACH:
            lines = len(points) - 1
            short = min(lines, 1 + int(line.fraction * lines))
            return _Refused(short, f"plan_cartesian_path reaches {line.fraction:.2f} of the lines from the end of the plan"), None
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
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], *,
        clearance_m: float, with_lift: bool = False,
        test: Callable[[tuple[Vec3, ...], Quat, _Checks], Awaitable[tuple[Optional[_Refused], object]]],
    ) -> tuple[Quat, tuple[Vec3, ...], object]:
        """The first orientation of `grasp` whose moves all pass `test`, with
        the points of those moves in order and what `test` gave for them.
        The points: the pre point, `clearance_m` back along the approach from
        the grasp point; then the grasp point, the pose for a place; then for
        a grab (`with_lift`) the lifted point. With none, the failure names
        the last orientation tried, its point that does not pass and the
        words for it. A refusal of the start (`_Refused.start_refused`) ends
        the search at its orientation: the start is where the arm stands, the
        same for each orientation."""
        names = ("pre-grasp point", "grasp point", "lifted point") if with_lift else ("pre-place point", "place point")
        last_refusal = ""
        checks = _Checks()
        for index, orientation in enumerate(grasp.orientations):
            points = (pre_grasp(grasp.point, orientation, clearance_m), grasp.point)
            if with_lift:
                points += (lifted(grasp.point, self.lift_m),)
            refused, found = await test(points, orientation, checks)
            tried = f"orientation {index + 1} of {len(grasp.orientations)}"
            if refused is None:
                notes.append(f"{tried} planned, {_numbers(orientation)} ({checks})")
                return orientation, points, found
            last_refusal = (
                f"{tried}, {_numbers(orientation)}, has no plan to its {names[refused.point]} "
                f"{_numbers(points[refused.point])}: {refused.words}"
            )
            if refused.start_refused:
                raise _NoReach(f"no plan reaches {target} with gripper '{gripper.name}': {last_refusal}", start_refused=True)
        raise _NoReach(f"no plan reaches {target} with gripper '{gripper.name}': {last_refusal}")

    async def _first_refusal(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> str:
        """The backbone's words for the first of `points` it has no plan
        for, or "" when it has a plan for each. Each check counts in
        `checks`."""
        refused = await self._first_refused_point(arm, points, orientation, cancel, checks)
        return "" if refused is None else refused.words

    async def _first_refused_point(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> Optional[_Refused]:
        """The first of `points` the backbone has no plan for, with its
        words, or None when it has a plan for each. Each check counts in
        `checks`."""
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
                return _Refused(index, check.reason or "check_arm_move found no plan")
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

    async def _line_down(self, arm: str, start: Vec3, end: Vec3, orientation: Quat, cancel: CancelToken) -> MoveResult:
        """The line down of `arm` from `start`, where it stands, to `end`: it
        stops on contact with `line_down_contact_gap_m` and runs at
        `line_down_speed_m_s`. Its result says where the grasp point
        stopped and whether on contact; any end but completed fails."""
        return await self._move(
            arm, end, orientation, cancel,
            duration_s=line_down_duration_s(start, end, self.line_down_speed_m_s), contact_gap_m=self.line_down_contact_gap_m,
        )

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


def _after_undo(failure: _Failed, undone: Optional[_Failed], notes: list[str], *, undo: str, done: str) -> _Failed:
    """The failure that ends a grab after `undo`, the step that puts back
    what the grab moved: `failure` itself when the undo went through
    (`undone` is None), which `notes` says with `done`, or `failure` with
    why the undo did not go through. An undo that a stop ended ends the
    grab as a stop."""
    if undone is None:
        notes.append(done)
        return failure
    if undone.stopped:
        notes.append(f"{undo} was stopped: {undone.message}")
        return _Failed(f"{failure.message}; {undo} was stopped: {undone.message}", stopped=True)
    notes.append(f"{undo} stopped short: {undone.message}")
    return _Failed(f"{failure.message}; {undo} failed: {undone.message}")


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
