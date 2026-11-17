"""place_item: carry the held item to a pose, or into or onto a target item,
and open there. With `target_item_id` the brain reads the target from what
it knows of that item, and `position` and `orientation` are not read; an
absent or empty id names no target. The result names the item the gripper
held when the place started, also when a forget ran during the place."""

from __future__ import annotations

from ..brain import RESULT_ENDS
from ..ports import ExpectedFailure, FailureKind, Pose, Role

ZERO = dict(gripper_name="", item_id="", final_position=[0.0, 0.0, 0.0])


async def run(brain, ctx) -> None:
    goal = ctx.request().data

    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        gripper = brain.state.holder(goal.gripper_name)
        item_id = gripper.held_item_id
        target = brain.state.target_item(goal.target_item_id, gripper) if goal.target_item_id else None
        brain.manipulator_or_refuse(manipulator)
        orientation = tuple(goal.orientation) if goal.orientation is not None and target is None else None
        pose = Pose(tuple(goal.position), orientation)
        # The scene equal to the held flags before the manipulator plans, so
        # the held item rides attached to the arm.
        await brain.make_scene_follow_holds()
        outcome = await manipulator.place(gripper, target if target is not None else pose, job.cancel)
        job.cancel.check()
        if not outcome.success:
            raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, outcome.message or "the placement failed")
        final = outcome.final_position
        if final is None:
            final = pose.position if target is None else (target.centre[0], target.centre[1], target.top_z)
        await brain.release_placed(gripper, item_id, final, orientation)
        return dict(gripper_name=gripper.name, item_id=item_id, final_position=list(final))

    await brain.run_guarded(ctx, Role.MANIPULATION, "place_item", body, ZERO, ends=RESULT_ENDS)
