"""The scripted manipulator: `manipulation_model: "scripted"`. No model
call: the grasp comes from a `GraspProposer`, and every move from a fixed
sequence through the `Robot`, so abort, cancel, the backbone's stop and
the node's shutdown stop it as they stop any manipulator.

A grab:

1. asks the proposer for the grasp point and the orientations to try;
2. takes the first orientation for which the backbone has a plan to the
   pre-grasp point (`clearance_m` back along the approach), to the grasp
   point and to the lifted point (`lift_m` above it), each checked with
   check_arm_move from where the arm is commanded now;
3. opens the gripper as wide as the item and `approach_margin_m` on each
   side, fully for an item of no known width, so the fingers come down
   beside the item and not on what stands next to it; then moves to the
   pre-grasp point; with an `aligner` (its port is `aligning.py`), a model then looks
   through the wrist and chest cameras and corrects the gripper's place
   and turn above the item, and the pre-grasp, grasp and lifted points
   become the aligned ones; then moves to the grasp point;
4. closes with the goal's `max_effort` and judges the hold from the
   result's `final_opening`: below `hold_opening_min` the jaws met nothing;
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
jaws to stand still. A check asks with the arm move's plan bars.

Each step that fails ends the grab with the backbone's own words. A step
that fails once the descent has begun (the descent itself, the close, the
two hold judgements, the lift, the wait for a snapshot) first retreats:
the gripper opens and the arm moves straight back up to the pre-grasp
point, so the jaws let go of what they touch and the next look sees the
item. A retreat that stops short adds why to the failure. A step whose
goal a stop ended (the brain's stop or the backbone's stop service)
retreats from nothing: the arm stays where the stop left it. A step that
fails before the descent leaves the arm where the backbone stopped it.

Each grab leaves one log line: the orientation that planned, with how many
check_arm_move calls ran and their total time; how wide the jaws opened for
the item's width; where the descent's result
put the grasp point against the commanded one; limb_state's grasp point at
the close; the two openings it judged; and the retreat.

A drop opens the gripper where it is. A place first reads the gripper's
opening from the latest snapshot: jaws below `hold_opening_min` stand
closed on air, the item slipped since the grab, and the place fails
before any move, naming it. It then carries the held item to the
pose, through its pre-place point, `clearance_m` and `held_item_depth_m`
above the pose so the held item clears what it is set on until the
descent, or `clearance_m` alone above it where the backbone has no plan
that high; in the pose's orientation alone when it has one, else from
above at each roll; with an `aligner`, the model then
corrects the held item's place above the spot as it does above an item in
a grab; the place opens there and climbs back to the pre-place point. An
arm that stops short on its way down to the pose does not settle, so its
move fails with `timeout`; when limb_states puts its grasp point anywhere
from the pre-place point down and within `RELEASE_OFF_LINE_M` of the line
down, the arm has set the item on what is under it, the rim or the floor
of a container included: the place opens there and logs it. It reports
where the grasp point stood when the jaws opened. A climb that fails is
logged, and the place still succeeds: the jaws are open and the item is
down.

The manipulator keeps nothing between goals: ids and held flags live in
`State`.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from ...limb_state import LimbSnapshot
from ...ports import Cancelled, CancelToken, Gripper, Item, Outcome, Pose, Quat, Senses, Vec3
from ...robot import MoveResult
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
# stands this and the clearance above the release point, so the item
# clears the floor of a container and what already stands in it until the
# descent. A 10 cm apple grasped 2.5 cm under its top reaches 7.8 cm. In
# Waldo the OpenArm has no plan pointing down more than about 15 cm above
# a desk spot near its body, so where this is too high the pre-place
# point stands at the clearance alone.
HELD_ITEM_DEPTH_M = 0.075
HOLD_OPENING_MIN = 0.05
# How far the opening may fall from the close to after the lift while the
# jaws still hold the item. In Waldo a held fruit's fell 0.003 to 0.013 (the
# bowl runs of 2026-10-24), and an apple that slipped out let it fall 0.316,
# from 0.378 to 0.062.
HOLD_DROP_MAX = 0.05
SNAPSHOT_WAIT_S = 1.0
# The bars every arm move names. The plan may sit 1 mm and 0.01 rad from
# the commanded pose, the backbone's servo tolerances; the move completes
# once the measured grasp point stands within 2 cm and 0.15 rad of it, the
# settle the robots' endpoint gives for the OpenArm v2, and fails 2 s after
# the planned end otherwise. 0 s is the fastest move the joint limits allow.
MOVE_DURATION_S = 0.0
PLAN_POSITION_M = 0.001
PLAN_ORIENTATION_RAD = 0.01
SETTLE_POSITION_M = 0.02
SETTLE_ORIENTATION_RAD = 0.15
SETTLE_TIMEOUT_S = 2.0
# The limb_motion failure kind of an arm move whose grasp point did not
# settle by its settle timeout.
SETTLE_TIMED_OUT = "timeout"
# How far from the line down an arm that stops short on its way down may
# stand for a place to open there: pushed further aside, it has not set
# the item down where it was going. An item whose bottom meets the floor
# right under the pre-place point tilts the arm a little as it stops, and
# one whose bottom meets an item already in the container slides off its
# round top by up to 4 or 5 cm before it rests.
RELEASE_OFF_LINE_M = 0.05
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


@dataclass
class _Checks:
    """The check_arm_move calls of a reach: how many, and their total time."""

    count: int = 0
    seconds: float = 0.0

    def __str__(self) -> str:
        return f"{self.count} check{'' if self.count == 1 else 's'}, {1000 * self.seconds:.0f} ms"


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
        aligner: Optional[Aligner] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
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
        # The brain's monotonic clock, for the checks' time in the log line.
        self._clock = clock
        self._robot = None
        self._limb_states = None

    def why_unavailable(self) -> str:
        if self._robot is None or self._limb_states is None:
            return NOT_STARTED
        return self._limb_states.why_unavailable()

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
        # be judged after the lift.
        _opening_of(self._limb_states.latest, gripper.name)
        if self.aligner is not None:
            reason = self.aligner.why_unavailable(gripper.arm)
            if reason:
                raise _Failed(f"no alignment: {reason}")
        grasp = self._propose(item)
        orientation, (pre, point, lift) = await self._first_reach(
            gripper, grasp, f"item '{item.item_id}'", cancel, notes, clearance_m=self.clearance_m, with_lift=True
        )
        opening = self._approach_opening(item)
        if item.width_m is not None:
            notes.append(f"opened to {opening:.2f} for its {100 * item.width_m:.1f} cm width")
        await self._grip(gripper.name, opening, 0.0, cancel)
        await self._move(gripper.arm, pre, orientation, cancel)
        if self.aligner is not None:
            pre, point, lift, orientation = await self._align(item, gripper, pre, point, orientation, cancel, notes)
        try:
            return await self._descend_close_lift(item, gripper, max_effort, orientation, point, lift, cancel, notes)
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
        self, item: Item, gripper: Gripper, max_effort: float, orientation: Quat, point: Vec3, lift: Vec3,
        cancel: CancelToken, notes: list[str],
    ) -> Outcome:
        """The grab from its descent on: down to `point`, close, judge, lift
        to `lift`, judge again."""
        landed = await self._move(gripper.arm, point, orientation, cancel)
        notes.append(f"landed {_offset_cm(landed.final_position, point)} from the grasp point")
        closed = await self._grip(gripper.name, CLOSED, max_effort, cancel)
        notes.append(f"limb_state at the close {_offset_cm(self._arm_position(gripper.arm), point)} from it")
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
        return Outcome(True)

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
        target = "the pose [{:.3f}, {:.3f}, {:.3f}]".format(*pose.position)
        notes: list[str] = []
        orientation, (pre, point) = await self._place_reach(gripper, Grasp(pose.position, orientations), target, cancel, notes)
        await self._move(gripper.arm, pre, orientation, cancel)
        if self.aligner is not None:
            request = AlignRequest(None, gripper, pre, point, orientation, self.lift_m, target=pose.position)
            pre, point, _lift, orientation = await self._run_alignment(request, cancel, notes)
        if notes:
            logger.info("place with gripper '%s': %s", gripper.name, "; ".join(notes))
        released_at = await self._lower_to_release(gripper, pre, point, orientation, cancel)
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
        return Outcome(True, "", released_at)

    def _require_started(self) -> None:
        reason = self.why_unavailable()
        if reason:
            raise _Failed(reason)

    def _propose(self, item: Item) -> Grasp:
        try:
            return self.proposer.propose(item)
        except ValueError as error:
            raise _Failed(f"no grasp for item '{item.item_id}': {error}") from None

    async def _place_reach(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str]
    ) -> tuple[Quat, tuple[Vec3, ...]]:
        """The orientation and the points of a place: the pre-place point
        `clearance_m` and `held_item_depth_m` above the pose when the backbone
        has a plan that high, else `clearance_m` above it, which `notes`
        says."""
        high = self.clearance_m + self.held_item_depth_m
        try:
            return await self._first_reach(gripper, grasp, target, cancel, [], clearance_m=high)
        except _Failed:
            notes.append(f"no plan {100 * high:.1f} cm above the pose: the pre-place point stands {100 * self.clearance_m:.1f} cm above it")
            return await self._first_reach(gripper, grasp, target, cancel, [], clearance_m=self.clearance_m)

    async def _first_reach(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], *,
        clearance_m: float, with_lift: bool = False,
    ) -> tuple[Quat, tuple[Vec3, ...]]:
        """The first orientation of `grasp` whose moves all plan, with the
        points of those moves in order: the pre point, `clearance_m` back
        along the approach, the grasp point, and the lifted point when
        `with_lift`."""
        last_refusal = ""
        checks = _Checks()
        for index, orientation in enumerate(grasp.orientations):
            points = (pre_grasp(grasp.point, orientation, clearance_m), grasp.point)
            if with_lift:
                points += (lifted(grasp.point, self.lift_m),)
            refusal = await self._first_refusal(gripper.arm, points, orientation, cancel, checks)
            if not refusal:
                notes.append(
                    f"orientation {index + 1} of {len(grasp.orientations)} planned, "
                    f"[{', '.join(f'{value:.3f}' for value in orientation)}] ({checks})"
                )
                return orientation, points
            last_refusal = refusal
        raise _Failed(f"no plan reaches {target} with gripper '{gripper.name}': {last_refusal}")

    async def _first_refusal(
        self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken, checks: _Checks
    ) -> str:
        """The backbone's words for the first of `points` it has no plan
        for, or "" when it has a plan for each. Each check counts in
        `checks`."""
        for point in points:
            started = self._clock()
            check = await until(
                cancel.wait(),
                self._robot.check_arm_move(
                    arm, point, orientation, duration_s=MOVE_DURATION_S,
                    plan_position_tolerance_m=PLAN_POSITION_M, plan_orientation_tolerance_rad=PLAN_ORIENTATION_RAD,
                ),
            )
            checks.count += 1
            checks.seconds += self._clock() - started
            cancel.check()
            if not check.reachable:
                return check.reason or "check_arm_move found no plan"
        return ""

    async def _move(self, arm: str, point: Vec3, orientation: Quat, cancel: CancelToken) -> MoveResult:
        """Moves the grasp point of `arm` to `point` in `orientation`; the
        result comes once the arm has settled there."""
        moved = await self._move_arm(arm, point, orientation)
        cancel.check()
        if not moved.completed:
            raise _Failed(moved.message or "move_arm failed", stopped=moved.cancelled)
        return moved

    async def _lower_to_release(
        self, gripper: Gripper, pre: Vec3, point: Vec3, orientation: Quat, cancel: CancelToken
    ) -> Optional[Vec3]:
        """Lowers the held item from `pre` to `point` and returns where the
        grasp point stands for the release: at `point`, or where the arm
        stopped short on the way down, the item set on what is under it. An
        arm that stops short does not settle, so its move fails with
        `timeout`; where it stopped is the grasp point of the latest
        limb_states snapshot, since a failed move gives no result. A move a
        stop ended, one that failed for another reason, or one that stopped
        more than `RELEASE_OFF_LINE_M` above the pre-place point or off the
        line down, fails."""
        moved = await self._move_arm(gripper.arm, point, orientation)
        cancel.check()
        if moved.completed:
            return moved.final_position
        at = self._arm_position(gripper.arm) if moved.failure_kind == SETTLE_TIMED_OUT else None
        if not _short_on_the_way_down(pre, point, at, RELEASE_OFF_LINE_M):
            raise _Failed(moved.message or "move_arm failed", stopped=moved.cancelled)
        logger.info(
            "place with gripper '%s' stopped %.1f cm short of the place point on the way down (%s): it releases there",
            gripper.name, 100 * math.dist(at, point), moved.message,
        )
        return at

    async def _move_arm(self, arm: str, point: Vec3, orientation: Quat) -> MoveResult:
        """One move_arm with the bars of every arm move."""
        return await self._robot.move_arm(
            arm, point, orientation, duration_s=MOVE_DURATION_S,
            plan_position_tolerance_m=PLAN_POSITION_M, plan_orientation_tolerance_rad=PLAN_ORIENTATION_RAD,
            settle_position_tolerance_m=self.settle_position_m, settle_orientation_tolerance_rad=self.settle_orientation_rad,
            settle_timeout_s=self.settle_timeout_s,
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


def _short_on_the_way_down(start: Vec3, end: Vec3, at: Optional[Vec3], off_line_m: float) -> bool:
    """Whether `at` lies between `start` and `end` along the line from one
    to the other, `start` itself included, within `off_line_m` of that
    line."""
    if at is None:
        return False
    line = [e - s for s, e in zip(start, end)]
    length = math.sqrt(sum(v * v for v in line))
    along = sum((a - s) * v for s, a, v in zip(start, at, line)) / length
    foot = [s + v * along / length for s, v in zip(start, line)]
    return -off_line_m <= along <= length and math.dist(at, foot) <= off_line_m


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
