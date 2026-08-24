"""Asyncio plumbing shared by Python peppy nodes' stream tasks: logging,
pacing, subscription receive, gated wire-input consumption, service serving,
and wire timestamps."""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import AsyncIterator
from typing import Any, Protocol


class SetpointError(ValueError):
    """A wire message that must not become a motion target."""


_RECEIVE_RETRY_S = 0.1
_SERVICE_RETRY_S = 1.0

_node_tag = "node"


def configure(node_tag: str) -> None:
    """Name the node in every log line; called once at process start."""
    global _node_tag
    _node_tag = node_tag


class CancellationToken(Protocol):
    def is_cancelled(self) -> bool: ...
    async def cancelled(self) -> None: ...


def log(message: str) -> None:
    print(f"[{_node_tag}] {message}", flush=True)


class Latch:
    """Log a recurring condition once, re-arming once it clears."""

    def __init__(self) -> None:
        self._tripped = False

    def trip(self, message: str) -> None:
        if not self._tripped:
            self._tripped = True
            log(f"{message}; suppressing repeats")

    def clear(self) -> None:
        self._tripped = False


async def ticks(period_s: float, token: CancellationToken) -> AsyncIterator[None]:
    """Yield once per period until the token cancels. Deadline-paced so the
    cadence does not drift; a slipped tick resyncs instead of bursting."""
    deadline = time.monotonic()
    while not token.is_cancelled():
        deadline += period_s
        delay = deadline - time.monotonic()
        if delay > 0.0:
            await asyncio.sleep(delay)
        else:
            deadline = time.monotonic()
            await asyncio.sleep(0)
        if token.is_cancelled():
            return
        yield


async def _next_message(subscription, token: CancellationToken) -> Any:
    cancelled = asyncio.ensure_future(token.cancelled())
    receive = None
    try:
        receive = asyncio.ensure_future(subscription.next())
        await asyncio.wait([cancelled, receive], return_when=asyncio.FIRST_COMPLETED)
        if not receive.done():
            return None
        try:
            return receive.result()
        except asyncio.CancelledError:
            return None
    finally:
        cancelled.cancel()
        if receive is not None:
            receive.cancel()


async def messages(
    subscription, token: CancellationToken, label: str
) -> AsyncIterator[Any]:
    """Every (producer, message) until the token cancels or the subscription
    closes; a failed receive logs and backs off instead of ending the stream."""
    failing = Latch()
    while not token.is_cancelled():
        try:
            received = await _next_message(subscription, token)
        except Exception as e:
            failing.trip(f"{label} receive error: {e!r}")
            await asyncio.sleep(_RECEIVE_RETRY_S)
            continue
        failing.clear()
        if received is None:
            return
        yield received


async def consume_gated(
    subscription,
    token: CancellationToken,
    label: str,
    now_s,
    stale_timeout_s: float,
    handle,
    on_reject=None,
    on_conform=None,
) -> None:
    """One wire-input stream: age-gate on the wire timestamp, then
    handle(message). The gate is symmetric and what arrival-side freshness
    cannot give: a backlog replayed after a stall, a future-stamped message,
    or an unstampable one must not be followed as fresh commands.

    A SetpointError rejects the message (on_reject may raise an alert; a
    later conforming message calls on_conform); any other handler surprise,
    e.g. a sim clock before its first tick, logs and continues, never
    killing the stream task.
    """
    rejected = Latch()
    stale = Latch()
    async for _producer, message in messages(subscription, token, label):
        try:
            age_s = now_s() - message.timestamp
            if not math.isfinite(age_s) or abs(age_s) > stale_timeout_s:
                stale.trip(f"{label} stale, future-stamped, or unstampable on arrival; dropping")
                continue
            stale.clear()
            handle(message)
        except SetpointError as e:
            rejected.trip(f"{label} rejected: {e}")
            if on_reject is not None:
                await on_reject(str(e))
            continue
        except Exception as e:
            rejected.trip(f"{label} handling failed: {e!r}")
            continue
        rejected.clear()
        if on_conform is not None:
            await on_conform()


async def serve(node_runner, service_module, handler, label: str) -> None:
    """Answer one exposed service until shutdown; a raising handler must not
    end the loop or the service would never answer again."""
    token = node_runner.cancellation_token()
    while not token.is_cancelled():
        try:
            await service_module.handle_next_request(node_runner, handler)
        except Exception as e:
            log(f"{label} service error: {e!r}")
            await asyncio.sleep(_SERVICE_RETRY_S)


def wire_timestamp_s(clock_now_ns: int, captured_monotonic: float) -> float:
    """Back-date the daemon clock by the snapshot's age, so wire stamps name
    capture time without the device thread touching the peppygen clock."""
    age_s = time.monotonic() - captured_monotonic
    return clock_now_ns / 1e9 - age_s
