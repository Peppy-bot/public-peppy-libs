"""drop_item: open a gripper where it is and release what it holds. The
brain forgets the dropped item (`State.forget_dropped`)."""

from __future__ import annotations

from ..brain import RESULT_ENDS
from ..ports import ExpectedFailure, FailureKind, Role

ZERO = dict(gripper_name="", item_id="")


async def run(brain, ctx) -> None:
    goal = ctx.request().data

    async def body(job) -> dict:
        manipulator = brain.models.current(Role.MANIPULATION)
        state = brain.state
        gripper = state.holder(goal.gripper_name)
        outcome = await brain.manipulator_or_refuse(manipulator).drop(gripper, job.cancel)
        job.cancel.check()
        if not outcome.success:
            raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, outcome.message or "the release failed")
        item_id = state.forget_dropped(gripper)
        return dict(gripper_name=gripper.name, item_id=item_id)

    await brain.run_guarded(ctx, Role.MANIPULATION, "drop_item", body, ZERO, ends=RESULT_ENDS)
