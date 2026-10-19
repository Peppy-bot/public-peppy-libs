"""place_item: carry the held item to a pose and open there."""

from __future__ import annotations

from ..ports import Pose, Refusal, Role

ZERO = dict(gripper_name="", item_id="", final_position=[0.0, 0.0, 0.0])


async def run(brain, ctx) -> None:
    goal = ctx.request().data

    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        state = brain.state
        gripper = state.holder(goal.gripper_name)
        brain.manipulator_or_refuse(manipulator)
        orientation = tuple(goal.orientation) if goal.orientation is not None else None
        pose = Pose(tuple(goal.position), orientation)
        outcome = await manipulator.place(gripper, pose, job.cancel)
        job.cancel.check()
        if not outcome.success:
            raise Refusal(outcome.message or "the placement failed")
        final = outcome.final_position if outcome.final_position is not None else pose.position
        item_id = state.clear_held_placed(gripper, final, pose.orientation)
        return dict(gripper_name=gripper.name, item_id=item_id, final_position=list(final))

    await brain.run_guarded(ctx, Role.MANIPULATION, "place_item", body, ZERO)
