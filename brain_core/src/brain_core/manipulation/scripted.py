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
3. opens the gripper, moves to the pre-grasp point, then to the grasp
   point;
4. closes with the goal's `max_effort` and judges the hold from the
   result's `final_opening`: below `hold_opening_min` the jaws met nothing;
5. lifts, then judges the hold again from the first limb_states snapshot
   captured after the lift's result, waiting up to `snapshot_wait_s` for
   it: below `hold_opening_min` the item slipped.

Each step that fails ends the grab with the backbone's own words, and the
arm stays where the backbone stopped it. A grab that closed on nothing
leaves the gripper closed at the grasp point: the next grab opens first
and climbs to its pre-grasp point. Each grab leaves one log line: the
orientation that planned and the two openings it judged.

A drop opens the gripper where it is. A place carries the held item to the
pose, through its pre-grasp point, in the pose's orientation alone when it
has one, else from above at each roll, and opens there.

The manipulator keeps nothing between goals: ids and held flags live in
`State`.
"""

from __future__ import annotations

import logging
from typing import Optional

from ..limb_state import LimbSnapshot
from ..ports import Cancelled, CancelToken, Grasp, GraspProposer, Gripper, Item, Outcome, Pose, Quat, Vec3
from ..robot import MoveResult
from ..waiting import until
from .grasp import lifted, orientations_for, pre_grasp

logger = logging.getLogger(__name__)

CLEARANCE_M = 0.10
LIFT_M = 0.10
HOLD_OPENING_MIN = 0.05
SNAPSHOT_WAIT_S = 1.0
OPEN = 1.0
CLOSED = 0.0
NOT_STARTED = "the scripted manipulator has not been started"


class _Failed(Exception):
    """A step of a sequence that did not go through: `message` ends the
    sequence's outcome."""

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


class ScriptedManipulator:
    name = "scripted"

    def __init__(
        self,
        proposer: GraspProposer,
        *,
        clearance_m: float = CLEARANCE_M,
        lift_m: float = LIFT_M,
        hold_opening_min: float = HOLD_OPENING_MIN,
        snapshot_wait_s: float = SNAPSHOT_WAIT_S,
    ) -> None:
        self.proposer = proposer
        self.clearance_m = clearance_m
        self.lift_m = lift_m
        self.hold_opening_min = hold_opening_min
        self.snapshot_wait_s = snapshot_wait_s
        self._robot = None
        self._limb_states = None

    def why_unavailable(self) -> str:
        if self._robot is None or self._limb_states is None:
            return NOT_STARTED
        return self._limb_states.why_unavailable()

    async def start(self, robot, limb_states) -> None:
        self._robot = robot
        self._limb_states = limb_states

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
        grasp = self._propose(item)
        orientation, (pre, point, lift) = await self._first_reach(
            gripper, grasp, f"item '{item.item_id}'", cancel, notes, with_lift=True
        )
        await self._grip(gripper.name, OPEN, 0.0, cancel)
        await self._move(gripper.arm, pre, orientation, cancel)
        await self._move(gripper.arm, point, orientation, cancel)
        closed = await self._grip(gripper.name, CLOSED, max_effort, cancel)
        notes.append(f"closed at {closed:.3f}")
        if closed < self.hold_opening_min:
            raise _Failed(f"gripper '{gripper.name}' closed on nothing")
        await self._move(gripper.arm, lift, orientation, cancel)
        after = await self._snapshot_after(self._limb_states.latest, cancel)
        opening = _opening_of(after, gripper.name)
        notes.append(f"{opening:.3f} after the lift")
        if opening < self.hold_opening_min:
            raise _Failed(f"item '{item.item_id}' slipped while lifting")
        notes.append("held")
        return Outcome(True)

    async def _place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome:
        self._require_started()
        try:
            orientations = orientations_for(pose.orientation)
        except ValueError as error:
            raise _Failed(str(error)) from None
        target = "the pose [{:.3f}, {:.3f}, {:.3f}]".format(*pose.position)
        orientation, (pre, point) = await self._first_reach(gripper, Grasp(pose.position, orientations), target, cancel, [])
        await self._move(gripper.arm, pre, orientation, cancel)
        placed = await self._move(gripper.arm, point, orientation, cancel)
        await self._grip(gripper.name, OPEN, 0.0, cancel)
        return Outcome(True, "", placed.final_position)

    def _require_started(self) -> None:
        reason = self.why_unavailable()
        if reason:
            raise _Failed(reason)

    def _propose(self, item: Item) -> Grasp:
        try:
            return self.proposer.propose(item)
        except ValueError as error:
            raise _Failed(f"no grasp for item '{item.item_id}': {error}") from None

    async def _first_reach(
        self, gripper: Gripper, grasp: Grasp, target: str, cancel: CancelToken, notes: list[str], *, with_lift: bool = False
    ) -> tuple[Quat, tuple[Vec3, ...]]:
        """The first orientation of `grasp` whose moves all plan, with the
        points of those moves in order: the pre-grasp point, the grasp
        point, and the lifted point when `with_lift`."""
        last_refusal = ""
        for index, orientation in enumerate(grasp.orientations):
            points = (pre_grasp(grasp.point, orientation, self.clearance_m), grasp.point)
            if with_lift:
                points += (lifted(grasp.point, self.lift_m),)
            refusal = await self._first_refusal(gripper.arm, points, orientation, cancel)
            if not refusal:
                notes.append(
                    f"orientation {index + 1} of {len(grasp.orientations)} planned, "
                    f"[{', '.join(f'{value:.3f}' for value in orientation)}]"
                )
                return orientation, points
            last_refusal = refusal
        raise _Failed(f"no plan reaches {target} with gripper '{gripper.name}': {last_refusal}")

    async def _first_refusal(self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken) -> str:
        """The backbone's words for the first of `points` it has no plan
        for, or "" when it has a plan for each."""
        for point in points:
            check = await until(cancel.wait(), self._robot.check_arm_move(arm, point, orientation))
            cancel.check()
            if not check.success:
                return check.message or "check_arm_move found no plan"
        return ""

    async def _move(self, arm: str, point: Vec3, orientation: Quat, cancel: CancelToken) -> MoveResult:
        moved = await self._robot.move_arm(arm, point, orientation)
        cancel.check()
        if not moved.success:
            raise _Failed(moved.message or "move_arm failed")
        return moved

    async def _grip(self, gripper: str, opening: float, max_effort: float, cancel: CancelToken) -> float:
        """Drives `gripper` to `opening` and returns its final opening."""
        gripped = await self._robot.set_gripper(gripper, opening, max_effort=max_effort)
        cancel.check()
        if not gripped.success:
            raise _Failed(gripped.message or "move_gripper failed")
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


def _opening_of(snapshot: Optional[LimbSnapshot], gripper: str) -> float:
    opening = snapshot.opening(gripper) if snapshot is not None else None
    if opening is None:
        raise _Failed(f"limb_states names no gripper '{gripper}'")
    return opening
