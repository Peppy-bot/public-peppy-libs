"""place_item: carry the held item to a pose, or into or onto a target item,
and open there. With `target_item_id` the brain reads the target item from
what it knows of that item, and the goal's target is not read; an absent or
empty id names no target item, and the goal's target, which its decision
parses and places in the robot frame (`Brain.place_target`), is where the
item goes. A target the decision could not place for want of the robot
frame or of the lookup's answer is refused in the result. The result names
the item the gripper held when the place started, also when a forget ran
during the place, and where the measured grasp point stood when the jaws
opened, in the robot frame at the time of that measurement, absent when
the manipulator measured none."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..brain import RESULT_ENDS, GoalTarget, placed_target
from ..ports import ExpectedFailure, FailureKind, Role

ZERO = dict(gripper_name="", item_id="", final_point=None)


@dataclass(frozen=True)
class Goal:
    """A place_item goal as its decision admitted it: the target item by its
    id, or, with `target_item_id` absent or empty, its target as the
    decision found it."""

    gripper_name: str
    target: Optional[GoalTarget]
    target_item_id: Optional[str]


async def admit(brain, goal) -> Goal:
    """The goal, its target placed when it names no target_item_id; a target
    that `Brain.place_target` refuses refuses the goal."""
    target = None if goal.target_item_id else await brain.place_target(goal.target)
    return Goal(goal.gripper_name, target, goal.target_item_id)


async def run(brain, ctx, goal: Goal) -> None:
    module = brain.bindings.place_item

    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        gripper = brain.state.holder(goal.gripper_name)
        item_id = gripper.held_item_id
        into = brain.state.target_item(goal.target_item_id, gripper) if goal.target_item_id else None
        pose = placed_target(goal.target) if into is None else None
        brain.manipulator_or_refuse(manipulator)
        # The scene equal to the held flags before the manipulator plans, so
        # the held item rides attached to the arm.
        await brain.make_scene_follow_holds()
        outcome = await manipulator.place(gripper, into if into is not None else pose, job.cancel)
        job.cancel.check()
        if not outcome.success:
            raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, outcome.message or "the placement failed")
        final = outcome.final_position
        if final is None:
            final = pose.position if into is None else (into.centre[0], into.centre[1], into.top_z)
            final_point = None
        else:
            final_point = module.ResultFinalPoint(frame=brain.placer.robot_frame, timestamp=outcome.final_timestamp, position=list(final))
        await brain.release_placed(gripper, item_id, final, pose.orientation if into is None else None)
        return dict(gripper_name=gripper.name, item_id=item_id, final_point=final_point)

    await brain.run_guarded(ctx, Role.MANIPULATION, "place_item", body, ZERO, ends=RESULT_ENDS)
