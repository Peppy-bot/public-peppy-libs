"""drop_item: open a gripper and release what it holds, where the arm
stands or, with `target_item_id`, above that item: the brain carries the
held item over it and opens there with no descent. An absent or empty id
names no target. The brain forgets the dropped item
(`Brain.release_dropped`). The result names the item the gripper held when
the drop started, also when a forget ran during the drop."""

from __future__ import annotations

from ..brain import RESULT_ENDS
from ..ports import ExpectedFailure, FailureKind, Role

ZERO = dict(gripper_name="", item_id="")


async def run(brain, ctx, goal) -> None:
    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        gripper = brain.state.holder(goal.gripper_name)
        item_id = gripper.held_item_id
        target = brain.state.target_item(goal.target_item_id, gripper) if goal.target_item_id else None
        brain.manipulator_or_refuse(manipulator)
        if target is not None:
            # A drop above a target plans a carry: the scene equal to the
            # held flags first, so the held item rides attached to the arm.
            await brain.make_scene_follow_holds()
        outcome = await manipulator.drop(gripper, job.cancel, above=target)
        job.cancel.check()
        if not outcome.success:
            raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, outcome.message or "the release failed")
        await brain.release_dropped(gripper, item_id)
        return dict(gripper_name=gripper.name, item_id=item_id)

    await brain.run_guarded(ctx, Role.MANIPULATION, "drop_item", body, ZERO, ends=RESULT_ENDS)
