"""get_model: the active model of each role the brain has, in a stable
order."""

from __future__ import annotations


def handle(brain, _request):
    """The node's generated get_model Response."""
    roles = list(brain.models.roles())
    return brain.bindings.get_model.Response(
        roles=[role.value for role in roles],
        models=[brain.models.current_name(role) for role in roles],
    )
