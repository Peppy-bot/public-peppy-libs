"""The shared stream plumbing: latching, pacing, the gated consume, and
capture-time wire stamps."""

import asyncio
import gc
import time

import pytest

from control_core_py import runtime
from control_core_py.runtime import SetpointError


class FakeToken:
    def __init__(self):
        self._cancelled = asyncio.Event()

    def cancel(self):
        self._cancelled.set()

    def is_cancelled(self):
        return self._cancelled.is_set()

    async def cancelled(self):
        await self._cancelled.wait()


class FakeSubscription:
    """Serves a scripted message list, then blocks until cancelled."""

    def __init__(self, messages):
        self._messages = list(messages)
        self._drained = asyncio.Event()

    async def next(self):
        if self._messages:
            return ("producer", self._messages.pop(0))
        self._drained.set()
        await asyncio.Event().wait()

    async def drained(self):
        await self._drained.wait()


class Message:
    def __init__(self, timestamp, value=0):
        self.timestamp = timestamp
        self.value = value


class StoppingService:
    """An exposed service whose `handle_requests` stops at once, as each of
    `stops` says in turn: None for a request stream that ends, or the
    exception it raises. The token fires once they run out."""

    def __init__(self, token, stops):
        self.token = token
        self.stops = list(stops)
        self.listens = 0

    async def handle_requests(self, node_runner, handler):
        self.listens += 1
        stop = self.stops.pop(0)
        if not self.stops:
            self.token.cancel()
        if stop is not None:
            raise stop


async def test_a_service_that_stops_is_logged_and_waited_out_before_its_loop_listens_again(monkeypatch, capsys):
    pauses = []

    async def pause(seconds):
        pauses.append(seconds)

    monkeypatch.setattr(runtime.asyncio, "sleep", pause)
    token = FakeToken()
    node_runner = type("NodeRunner", (), {"cancellation_token": lambda self: token})()
    service = StoppingService(token, [None, RuntimeError("the session is closed")])
    await asyncio.wait_for(runtime.serve(node_runner, service, None, "is_ready"), 1.0)
    assert service.listens == 2
    assert pauses == [1.0, 1.0]
    out = capsys.readouterr().out
    assert "is_ready: its request stream ended; it listens again in 1 s" in out
    assert "is_ready service error: RuntimeError('the session is closed'); it listens again in 1 s" in out


def test_latch_logs_once_and_rearms(capsys):
    latch = runtime.Latch()
    latch.trip("condition")
    latch.trip("condition")
    assert capsys.readouterr().out.count("condition") == 1
    latch.clear()
    latch.trip("condition")
    assert capsys.readouterr().out.count("condition") == 1


async def test_ticks_pace_and_stop_on_cancel():
    token = FakeToken()
    seen = 0
    start = time.monotonic()
    async for _ in runtime.ticks(0.01, token):
        seen += 1
        if seen == 5:
            token.cancel()
    # Five deadline-paced ticks take at least four full periods.
    assert time.monotonic() - start >= 0.04
    assert seen == 5


@pytest.mark.parametrize("period_s", [0.0, -0.01, float("nan")])
async def test_ticks_reject_an_invalid_period(period_s):
    with pytest.raises(ValueError):
        async for _ in runtime.ticks(period_s, FakeToken()):
            pass


async def test_next_message_prefers_cancellation_over_a_ready_message():
    # Both futures complete in the same event loop turn: the token is already
    # cancelled and the subscription has a message queued.
    token = FakeToken()
    token.cancel()
    subscription = FakeSubscription([Message(time.time(), "late")])
    assert await runtime._next_message(subscription, token) is None


async def run_gated(messages, handle, **kwargs):
    token = FakeToken()
    subscription = FakeSubscription(messages)
    task = asyncio.create_task(
        runtime.consume_gated(
            subscription, token, "test", time.time, 0.25, handle, **kwargs
        )
    )
    await asyncio.wait_for(subscription.drained(), 2.0)
    token.cancel()
    await asyncio.wait_for(task, 2.0)


@pytest.mark.parametrize("stale_timeout_s", [0.0, -1.0, float("nan")])
async def test_consume_gated_rejects_an_invalid_stale_timeout(stale_timeout_s):
    with pytest.raises(ValueError):
        await runtime.consume_gated(
            FakeSubscription([]), FakeToken(), "test", time.time,
            stale_timeout_s, lambda m: None,
        )


async def test_consume_gated_drops_stale_future_and_unstampable():
    handled = []
    now = time.time()
    await run_gated(
        [
            Message(now - 60.0, "stale"),
            Message(now + 60.0, "future"),
            Message(float("nan"), "unstampable"),
            Message(time.time(), "fresh"),
        ],
        lambda m: handled.append(m.value),
    )
    assert handled == ["fresh"]


async def test_consume_gated_reject_and_conform_lifecycle():
    events = []

    def handle(m):
        if m.value == "bad":
            raise SetpointError("bad value")
        events.append(("handled", m.value))

    async def on_reject(reason):
        events.append(("rejected", reason))

    async def on_conform():
        events.append(("conforming",))

    await run_gated(
        [Message(time.time(), "bad"), Message(time.time(), "good")],
        handle,
        on_reject=on_reject,
        on_conform=on_conform,
    )
    assert events == [("rejected", "bad value"), ("handled", "good"), ("conforming",)]


async def test_consume_gated_survives_handler_surprises():
    handled = []

    def handle(m):
        if m.value == "boom":
            raise RuntimeError("surprise")
        handled.append(m.value)

    await run_gated(
        [Message(time.time(), "boom"), Message(time.time(), "after")], handle
    )
    assert handled == ["after"]


def test_wire_timestamp_backdates_by_snapshot_age():
    captured = time.monotonic() - 0.5
    clock_now_ns = 1_000_000_000_000
    stamp = runtime.wire_timestamp_s(clock_now_ns, captured)
    age = clock_now_ns / 1e9 - stamp
    assert 0.5 <= age < 0.6


def test_rate_meter_reports_measured_rate(monkeypatch, capsys):
    from control_core_py.runtime import RateMeter

    clock = {"now": 100.0}
    monkeypatch.setattr("control_core_py.runtime.time.monotonic", lambda: clock["now"])
    meter = RateMeter("test loop", target_hz=60, report_period_s=1.0)
    for _ in range(61):
        clock["now"] += 1.0 / 60.0
        meter.tick()
    out = capsys.readouterr().out
    assert "test loop: 60.00 Hz measured" in out
    assert "(target 60)" in out
    # The window resets: another sub-window of ticks stays silent.
    meter.tick()
    assert capsys.readouterr().out == ""


async def test_tied_cancellation_and_receive_failure_leaves_no_orphan_exception():
    class FailingSubscription:
        async def next(self):
            raise RuntimeError("wire failed")

    class CancelledToken:
        def is_cancelled(self):
            return True

        async def cancelled(self):
            return None

    captured = []
    loop = asyncio.get_running_loop()
    loop.set_exception_handler(lambda _loop, context: captured.append(context))
    try:
        # Both futures complete in the same event loop turn: cancellation
        # wins, and the failed receive must still be awaited so its
        # exception never surfaces as an orphan at garbage collection.
        assert await runtime._next_message(FailingSubscription(), CancelledToken()) is None
        for _ in range(3):
            await asyncio.sleep(0)
        gc.collect()
        await asyncio.sleep(0)
    finally:
        loop.set_exception_handler(None)
    assert captured == []


async def test_ask_gives_the_first_answer():
    async def ask_once():
        return "so101"

    assert await runtime.ask("get_identity", FakeToken(), ask_once) == "so101"


async def test_ask_waits_out_each_failure_and_logs_the_first_of_a_run(monkeypatch, capsys):
    pauses = []
    real_sleep = asyncio.sleep

    async def pause(seconds):
        pauses.append(seconds)
        await real_sleep(0)

    monkeypatch.setattr(runtime.asyncio, "sleep", pause)
    answers = [RuntimeError("no reply"), RuntimeError("no reply"), "so101"]

    async def ask_once():
        answer = answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    assert await runtime.ask("robot_identity", FakeToken(), ask_once) == "so101"
    assert pauses == [1.0, 1.0]
    out = capsys.readouterr().out
    assert out.count("robot_identity failing: RuntimeError('no reply'); suppressing repeats") == 1


async def test_ask_ends_with_none_once_cancelled_while_it_asks():
    token = FakeToken()
    asked = asyncio.Event()

    async def ask_once():
        asked.set()
        await asyncio.Event().wait()

    asking = asyncio.ensure_future(runtime.ask("get_identity", token, ask_once))
    await asked.wait()
    token.cancel()
    assert await asyncio.wait_for(asking, 1.0) is None


async def test_ask_ends_with_none_once_cancelled_while_it_waits_out_a_failure():
    token = FakeToken()

    async def ask_once():
        token.cancel()
        raise RuntimeError("no reply")

    assert await asyncio.wait_for(runtime.ask("get_identity", token, ask_once), 1.0) is None
