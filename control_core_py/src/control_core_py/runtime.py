"""Asyncio plumbing shared by Python peppy nodes' stream tasks: logging,
pacing, subscription receive, gated wire-input consumption, service serving,
asking another node until it answers, and wire timestamps."""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import AsyncIterator, Awaitable, Callable
from typing import Any, Protocol, TypeVar

from control_core_py.params import require_positive


class SetpointError(ValueError):
    """A wire message that must not become a motion target."""


_RECEIVE_RETRY_S = 0.1
_SERVICE_RETRY_S = 1.0
_ASK_RETRY_S = 1.0

_Answer = TypeVar("_Answer")

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


class RateMeter:
    """Measured cadence of a loop, logged periodically so a launch shows
    whether the loop actually runs at its configured rate (control_core's
    Pacer keeps the same accounting for the Rust nodes). One line per report
    window; silent between reports."""

    def __init__(self, label: str, target_hz: float, report_period_s: float = 10.0):
        self._label = label
        self._target_hz = target_hz
        self._report_period_s = report_period_s
        self._window_start = time.monotonic()
        self._ticks = 0

    def tick(self) -> None:
        self._ticks += 1
        now = time.monotonic()
        elapsed = now - self._window_start
        if elapsed < self._report_period_s:
            return
        measured = self._ticks / elapsed
        log(
            f"{self._label}: {measured:.2f} Hz measured over {elapsed:.0f}s "
            f"(target {self._target_hz:g})"
        )
        self._window_start = now
        self._ticks = 0


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
    require_positive("period_s", period_s)
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
        # Both can finish in the same event loop turn; cancellation wins so a
        # message never becomes work after shutdown began.
        if cancelled.done() or not receive.done():
            return None
        try:
            return receive.result()
        except asyncio.CancelledError:
            return None
    finally:
        cancelled.cancel()
        if receive is not None:
            receive.cancel()
        # Await both before returning: a receive that failed in the same
        # turn cancellation won would otherwise hold an unretrieved
        # exception and log at task teardown.
        await asyncio.gather(
            *(task for task in (cancelled, receive) if task is not None),
            return_exceptions=True,
        )


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
    # A NaN timeout would make the age comparison vacuous and disarm the gate.
    require_positive("stale_timeout_s", stale_timeout_s)
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
    """Answer one exposed service from one endpoint until shutdown. When the
    service stops, its request stream ended or it could not listen, the log
    says why and the loop listens again _SERVICE_RETRY_S later, so a
    service that stops at once cannot hot-spin the loop."""
    token = node_runner.cancellation_token()
    while not token.is_cancelled():
        try:
            await service_module.handle_requests(node_runner, handler)
        except Exception as e:
            log(f"{label} service error: {e!r}; it listens again in {_SERVICE_RETRY_S:g} s")
        else:
            log(f"{label}: its request stream ended; it listens again in {_SERVICE_RETRY_S:g} s")
        await asyncio.sleep(_SERVICE_RETRY_S)


async def _unless_cancelled(awaitable: Awaitable[_Answer], token: CancellationToken) -> tuple[bool, _Answer | None]:
    """Await `awaitable` unless the token cancels first: (True, its result), or
    (False, None) on a cancel, which wins a tie. An error of `awaitable`
    propagates."""
    cancelled = asyncio.ensure_future(token.cancelled())
    work = asyncio.ensure_future(awaitable)
    try:
        await asyncio.wait([cancelled, work], return_when=asyncio.FIRST_COMPLETED)
        if cancelled.done() or not work.done():
            return False, None
        return True, work.result()
    finally:
        cancelled.cancel()
        work.cancel()
        await asyncio.gather(cancelled, work, return_exceptions=True)


async def ask(
    question: str, token: CancellationToken, ask_once: Callable[[], Awaitable[_Answer]]
) -> _Answer | None:
    """Ask `ask_once` until it answers, and give the answer, or None once the
    token cancels. A cancel ends the loop at once, while it asks or waits.
    The first failure of a run is logged as "{question} failing: {error};
    suppressing repeats", and the loop asks again _ASK_RETRY_S after each
    failure."""
    failing = Latch()
    while True:
        try:
            answered, answer = await _unless_cancelled(ask_once(), token)
        except Exception as e:
            failing.trip(f"{question} failing: {e!r}")
        else:
            return answer if answered else None
        waited, _ = await _unless_cancelled(asyncio.sleep(_ASK_RETRY_S), token)
        if not waited:
            return None


def wire_timestamp_s(clock_now_ns: int, captured_monotonic: float) -> float:
    """Back-date the daemon clock by the snapshot's age, so wire stamps name
    capture time without the device thread touching the peppygen clock."""
    age_s = time.monotonic() - captured_monotonic
    return clock_now_ns / 1e9 - age_s
