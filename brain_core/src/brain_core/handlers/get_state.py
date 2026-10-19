"""get_state: what the brain does now and what it holds, from memory.

`current_action` is the manipulation lane only, as the item_manipulation
contract defines it; a running search does not appear here.
"""

from __future__ import annotations

from ..ports import Role


def handle(brain, _request):
    """The node's generated get_state Response."""
    job = brain.sequencer.running(Role.MANIPULATION)
    names, holding, held = brain.state.snapshot()
    return brain.bindings.get_state.Response(
        current_action=job.action if job is not None else "",
        current_action_elapsed_s=job.elapsed_s(brain.now()) if job is not None else 0.0,
        gripper_names=names,
        holding=holding,
        held_item_ids=held,
    )
