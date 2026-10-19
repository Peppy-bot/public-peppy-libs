"""switch_model: make a model the active one of a role while the brain
runs. A lane-less job: it holds neither lane, and checks the lane of its
own role itself (`Models.switch`). The role is read once, against the
roles this brain has."""

from __future__ import annotations

from ..ports import parse_role

ZERO = dict(role="", model="")


async def run(brain, ctx) -> None:
    goal = ctx.request().data

    async def body(job) -> dict:
        role = parse_role(goal.role, brain.models.roles())
        model = await brain.models.switch(role, goal.model, job)
        return dict(role=role.value, model=model)

    await brain.run_guarded(ctx, None, "switch_model", body, ZERO, asked=f"{goal.role} '{goal.model}'")
