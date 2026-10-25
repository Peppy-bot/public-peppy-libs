"""switch_model: make a model the active one of a role while the brain
runs. A lane-less job: it holds neither lane, and checks the lane of its
own role itself (`Models.switch`). The role is read against the roles
this brain has, at the decision and again when the switch starts."""

from __future__ import annotations

from ..brain import STATUS_ENDS
from ..ports import parse_role

ZERO = dict(role="", model="")


def decide(brain, goal) -> None:
    """Refuses a switch that cannot start now: `precondition_failed` once
    the node shuts down, then the refusals of the role and of the switch
    (`parse_role`, `Models.check_switch`)."""
    brain.sequencer.check(None)
    role = parse_role(goal.role, brain.models.roles())
    brain.models.check_switch(role, goal.model)


async def run(brain, ctx) -> None:
    goal = ctx.request().data

    async def body(job) -> dict:
        role = parse_role(goal.role, brain.models.roles())
        model = await brain.models.switch(role, goal.model, job)
        return dict(role=role.value, model=model)

    await brain.run_guarded(ctx, None, "switch_model", body, ZERO, ends=STATUS_ENDS, asked=f"{goal.role} '{goal.model}'")
