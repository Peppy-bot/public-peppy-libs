"""forget: the brain forgets every item it knows and every hold, for a
world that changed while it did not look (`State.forget`). The robot does
not move, and a goal that runs keeps running: its result changes the
memory as each result does."""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def handle(brain, _request):
    """The node's generated forget Response."""
    forgotten = brain.state.forget()
    logger.info("forget: the brain forgot %s", ", ".join(forgotten) if forgotten else "no item")
    return brain.bindings.forget.Response(item_ids=forgotten)
