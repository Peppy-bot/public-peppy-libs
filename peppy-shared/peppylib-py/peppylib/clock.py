"""Clock helpers: align a node's notion of time with the core node.

This module is the Python face of `peppylib::clock`. It exposes the one-shot
NTP-style `synchronize`, the long-lived `subscribe` to the periodic ``clock``
topic, `for_node` (which builds a pre-bound `PeppyClock` that reads the
daemon-resolved time without caring whether the node runs in wall or sim mode),
and `SimTimePublisher` (the launch's one simulated-time source, publishing each
tick to every machine of the launch), plus the clock wire/value types.
``SimTimePublisher.for_node`` returns ``None`` on a node the launch did not
declare the source, so holding a publisher is the same fact as being it.
"""

from __future__ import annotations

from ._peppylib.core_node import (  # type: ignore[import-not-found]
    ClockRequest,
    ClockResponse,
    ClockSubscription,
    ClockSync,
    ClockTick,
    PeppyClock,
    SimTimePublisher,
    clock_for_node as for_node,
    subscribe_clock as subscribe,
    synchronize,
)

__all__ = [
    "subscribe",
    "synchronize",
    "for_node",
    "PeppyClock",
    "SimTimePublisher",
    "ClockSync",
    "ClockSubscription",
    "ClockRequest",
    "ClockResponse",
    "ClockTick",
]
