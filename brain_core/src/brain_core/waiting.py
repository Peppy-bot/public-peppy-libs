"""The one way a part of the brain waits for something unless a stop comes
first: the node's cancellation token for a loop, a job's cancel token for
a step a cancel must cut short."""

from __future__ import annotations

import asyncio
from typing import Awaitable, Optional, TypeVar

T = TypeVar("T")


async def until(stop: Awaitable[object], awaitable: Awaitable[T]) -> Optional[T]:
    """Awaits `awaitable` and returns its result, or None when `stop`
    completes first, the awaitable cancelled. What the awaitable raises is
    raised here."""
    pending = asyncio.ensure_future(awaitable)
    stopped = asyncio.ensure_future(stop)
    try:
        await asyncio.wait([stopped, pending], return_when=asyncio.FIRST_COMPLETED)
    except asyncio.CancelledError:
        pending.cancel()
        raise
    finally:
        stopped.cancel()
    if not pending.done():
        pending.cancel()
        return None
    return pending.result()


async def unless_cancelled(token, awaitable: Awaitable[T]) -> Optional[T]:
    """Awaits `awaitable` and returns its result, or None when the node's
    `token` fires first, the awaitable cancelled."""
    return await until(token.cancelled(), awaitable)
