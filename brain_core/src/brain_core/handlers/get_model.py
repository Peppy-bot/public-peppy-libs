"""get_model: the active model of each role the brain has, in a stable
order, and why each role cannot serve a goal yet while the brain starts it."""

from __future__ import annotations


def handle(brain, _request):
    """The node's generated get_model Response."""
    roles = list(brain.models.roles())
    return brain.bindings.get_model.Response(
        roles=[role.value for role in roles],
        models=[brain.models.current_name(role) for role in roles],
        starting=[brain.starting(role) for role in roles],
    )
