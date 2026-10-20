"""list_models: every model switch_model accepts, grouped by role in
get_model's order, a role repeated once per model."""

from __future__ import annotations


def handle(brain, _request):
    """The node's generated list_models Response."""
    roles: list[str] = []
    models: list[str] = []
    for role in brain.models.roles():
        for name in brain.models.names(role):
            roles.append(role.value)
            models.append(name)
    return brain.bindings.list_models.Response(roles=roles, models=models)
