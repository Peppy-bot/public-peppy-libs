"""grab_item: close a gripper on an item named by id or by pose. A goal that
names no item_id takes its target, which its decision parses and places in
the robot frame (`Brain.place_target`); a goal that names one sends a target
that is not read. A target the decision could not place for want of the
robot frame or of the lookup's answer is refused in the result. The
manipulator publishes the stages of the grab through the reports of the
goal, which name the item by its label (`stages.GrabReports`)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..brain import RESULT_ENDS, GoalTarget, placed_target
from ..ports import ExpectedFailure, FailureKind, Role
from ..stages import GrabReports, reporter_of

ZERO = dict(gripper_name="", item_id="")


@dataclass(frozen=True)
class Goal:
    """A grab_item goal as its decision admitted it: the item by its id, or,
    with `item_id` empty, its target as the decision found it."""

    gripper_name: str
    item_id: str
    target: Optional[GoalTarget]
    max_effort: float


async def admit(brain, goal) -> Goal:
    """The goal, its target placed when it names no item_id; a target that
    `Brain.place_target` refuses refuses the goal."""
    target = None if goal.item_id else await brain.place_target(goal.target)
    return Goal(goal.gripper_name, goal.item_id, target, goal.max_effort)


async def run(brain, ctx, goal: Goal) -> None:
    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        state = brain.state
        # The item first: its position decides which free gripper suits it.
        known = state.item_to_grab(goal.item_id) if goal.item_id else None
        target = placed_target(goal.target) if known is None else None
        near = known.position if known is not None else target.position
        gripper = state.gripper_for_grab(goal.gripper_name, near)
        brain.manipulator_or_refuse(manipulator)
        item = known
        if item is None:
            item = state.mint_from_pose(target.position, target.orientation, brain.now())
        # The scene equal to the held flags before the manipulator plans: a
        # brain or a planner that restarted, a failed scene call or a forget
        # left it otherwise.
        await brain.make_scene_follow_holds()
        reports = GrabReports(reporter_of(ctx), item.label)
        outcome = await manipulator.grab(item, gripper, goal.max_effort, job.cancel, reports)
        job.cancel.check()
        if not outcome.success:
            raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, outcome.message or "the grasp failed")
        await brain.hold(gripper, item.item_id, outcome.closed_at)
        return dict(gripper_name=gripper.name, item_id=item.item_id)

    await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", body, ZERO, ends=RESULT_ENDS)
