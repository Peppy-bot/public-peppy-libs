"""The loops' one way of waiting on the node's token, an action loop that
decides each goal before it runs one, and what a handler that raises
leaves in the log."""

import asyncio
import logging
from types import SimpleNamespace

import fake_peppygen
import pytest

from conftest import FakeDetector, FakeManipulator, FakeSource, FakeToken, make_brain, records_of
from brain_core.brain import decider
from brain_core.handlers import switch_model
from brain_core.ports import Role
from brain_core.serve import report_failure, serve_action
from brain_core.waiting import unless_cancelled


async def test_the_awaitable_wins_when_it_finishes_first():
    async def answer():
        return "goal"

    assert await unless_cancelled(FakeToken(), answer()) == "goal"


async def test_the_token_wins_and_the_awaitable_is_cancelled():
    token = FakeToken()
    reached = asyncio.Event()
    cancelled = asyncio.Event()

    async def wait_forever():
        reached.set()
        try:
            await asyncio.Event().wait()
        except asyncio.CancelledError:
            cancelled.set()
            raise

    waiting = asyncio.create_task(unless_cancelled(token, wait_forever()))
    await reached.wait()
    token.cancel()
    assert await asyncio.wait_for(waiting, 1.0) is None
    await asyncio.wait_for(cancelled.wait(), 1.0)


async def test_what_the_awaitable_raises_is_raised():
    async def fail():
        raise ValueError("no")

    with pytest.raises(ValueError, match="no"):
        await unless_cancelled(FakeToken(), fail())


@pytest.fixture
def serve_log(caplog):
    """The serve module's log records."""
    with records_of(caplog, "brain_core.serve") as records:
        yield records


async def test_a_handler_that_raises_is_reported_with_its_traceback(serve_log):
    async def handler():
        raise RuntimeError("the goal context is gone")

    task = asyncio.create_task(handler())
    with pytest.raises(RuntimeError):
        await task
    report_failure(task)
    [record] = serve_log.records
    assert record.levelno == logging.ERROR
    assert record.getMessage() == "a goal handler failed and its goal was never completed"
    assert "RuntimeError: the goal context is gone" in serve_log.text


async def test_a_cancelled_or_clean_handler_is_not_reported(serve_log):
    async def clean():
        return None

    async def hang():
        await asyncio.Event().wait()

    done = asyncio.create_task(clean())
    await done
    cancelled = asyncio.create_task(hang())
    cancelled.cancel()
    with pytest.raises(asyncio.CancelledError):
        await cancelled
    report_failure(done)
    report_failure(cancelled)
    assert serve_log.records == []


class ScriptedAction:
    """An exposed action whose goals the test scripts: each request goes
    through the decider; a refused goal is kept with its refusal, and an
    accepted one comes out as its goal context. Once the requests run out,
    it waits for the next goal until the loop stops."""

    def __init__(self, requests) -> None:
        self.requests = list(requests)
        self.refused: list = []
        self.drained = asyncio.Event()

    async def handle_goal_next_request(self, decide):
        while self.requests:
            request = self.requests.pop(0)
            decision = decide(request)
            if decision.refusal is not None:
                self.refused.append((request.data.model, decision.refusal))
                continue
            return SimpleNamespace(request=request)
        self.drained.set()
        await asyncio.Event().wait()


def exposing(action_module, scripted: ScriptedAction):
    """`action_module` whose ActionHandle exposes `scripted`."""

    async def expose(node_runner):
        return scripted

    return SimpleNamespace(
        ACTION_NAME=action_module.ACTION_NAME,
        GoalDecision=action_module.GoalDecision,
        ActionHandle=SimpleNamespace(expose=expose),
    )


def switch_request(role: str, model: str):
    return SimpleNamespace(data=SimpleNamespace(role=role, model=model))


async def test_an_action_loop_refuses_at_once_what_its_check_refuses_and_runs_the_rest():
    source = FakeSource({Role.PERCEPTION: {"fast": FakeDetector()}, Role.MANIPULATION: {"fake": FakeManipulator()}})
    brain = await make_brain(source=source, perception_model="fast", manipulation_model="fake")
    busy = brain.sequencer.start(Role.MANIPULATION, "grab_item", 0)
    scripted = ScriptedAction([
        switch_request("arm", "fast"),
        switch_request("perception", "slow"),
        switch_request("manipulation", "fake"),
        switch_request("perception", "fast"),
    ])
    module = exposing(fake_peppygen.switch_model, scripted)
    token, tasks, ran = FakeToken(), set(), []

    async def run(ctx) -> None:
        ran.append(ctx.request.data.model)

    loop = asyncio.create_task(serve_action(module, None, token, tasks, decider(module, brain, switch_model.decide), run))
    await asyncio.wait_for(scripted.drained.wait(), 1.0)
    await asyncio.gather(*tasks)
    failure = fake_peppygen.switch_model.Failure
    assert scripted.refused == [
        ("fast", failure("rejected", "this brain runs no model for the role 'arm'")),
        ("slow", failure("rejected", "'slow' is not a perception model here; see list_models")),
        ("fake", failure("precondition_failed", "grab_item is running")),
    ]
    assert ran == ["fast"]
    assert source.prepared == []
    token.cancel()
    await asyncio.wait_for(loop, 1.0)
    brain.sequencer.finish(busy)


async def test_an_action_loop_without_a_check_accepts_every_goal():
    # Even a closed brain: item_manipulation reports refusals in its result.
    brain = await make_brain()
    brain.sequencer.close()
    scripted = ScriptedAction([switch_request("arm", "one"), switch_request("arm", "two")])
    module = exposing(fake_peppygen.grab_item, scripted)
    token, tasks, ran = FakeToken(), set(), []

    async def run(ctx) -> None:
        ran.append(ctx.request.data.model)

    loop = asyncio.create_task(serve_action(module, None, token, tasks, decider(module, brain, None), run))
    await asyncio.wait_for(scripted.drained.wait(), 1.0)
    await asyncio.gather(*tasks)
    assert ran == ["one", "two"] and scripted.refused == []
    token.cancel()
    await asyncio.wait_for(loop, 1.0)
