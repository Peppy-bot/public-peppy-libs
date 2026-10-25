"""One sequence per lane, and stopping the one that runs."""

import asyncio

import pytest

from brain_core.ports import Cancelled, FailureKind, Refusal, Role
from brain_core.sequencer import Sequencer

PERCEPTION, MANIPULATION = Role.PERCEPTION, Role.MANIPULATION


async def test_a_second_sequence_in_a_lane_is_refused_by_name():
    sequencer = Sequencer()
    job = sequencer.start(MANIPULATION, "grab_item", now_ns=0)
    with pytest.raises(Refusal, match="grab_item is running") as raised:
        sequencer.start(MANIPULATION, "drop_item", now_ns=1)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    # The other lane is free: a search may run beside a manipulation.
    search = sequencer.start(PERCEPTION, "scan_items", now_ns=1)
    assert sequencer.running(PERCEPTION) is search
    sequencer.finish(job)
    assert sequencer.running(MANIPULATION) is None
    sequencer.start(MANIPULATION, "drop_item", now_ns=2)


async def test_a_closed_sequencer_refuses_every_sequence_and_lets_the_running_one_end():
    sequencer = Sequencer()
    job = sequencer.start(MANIPULATION, "grab_item", now_ns=0)
    sequencer.close()
    for lane, action in ((MANIPULATION, "drop_item"), (PERCEPTION, "scan_items")):
        with pytest.raises(Refusal, match="^the node is shutting down$") as raised:
            sequencer.start(lane, action, now_ns=1)
        assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    assert sequencer.running(MANIPULATION) is job
    sequencer.finish(job)
    with pytest.raises(Refusal, match="^the node is shutting down$"):
        sequencer.start(MANIPULATION, "grab_item", now_ns=2)


async def test_stopping_an_idle_lane_reports_nothing():
    assert await Sequencer().stop(MANIPULATION, "aborted") is None


async def test_stopping_a_running_job_sets_its_token_calls_the_stopper_and_waits():
    stopped_lanes: list[str] = []

    async def stopper(lane: str) -> None:
        stopped_lanes.append(lane)

    sequencer = Sequencer(stopper=stopper)
    job = sequencer.start(MANIPULATION, "grab_item", now_ns=0)

    async def worker():
        await job.cancel.wait()
        with pytest.raises(Cancelled) as raised:
            job.cancel.check()
        assert raised.value.signalled is False
        assert raised.value.reason == "aborted: operator"
        sequencer.finish(job)

    task = asyncio.create_task(worker())
    stopped = await asyncio.wait_for(sequencer.stop(MANIPULATION, "aborted: operator"), 1.0)
    await task
    assert stopped == "grab_item"
    assert stopped_lanes == [MANIPULATION]
    assert sequencer.running(MANIPULATION) is None


async def test_a_cancel_by_the_goals_own_signal_is_marked_as_such():
    sequencer = Sequencer()
    job = sequencer.start(PERCEPTION, "scan_items", now_ns=0)
    await sequencer.cancel(job, "cancelled by its caller", signalled=True)
    assert job.cancel.cancelled and job.cancel.signalled
    assert job.elapsed_s(now_ns=2_500_000_000) == 2.5


async def test_a_check_raises_the_refusal_of_a_start_and_admits_nothing():
    sequencer = Sequencer()
    sequencer.check(PERCEPTION)
    sequencer.check(None)
    assert sequencer.running(PERCEPTION) is None
    job = sequencer.start(PERCEPTION, "identify_item", now_ns=0)
    with pytest.raises(Refusal, match="^identify_item is running$") as raised:
        sequencer.check(PERCEPTION)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    sequencer.check(MANIPULATION)
    sequencer.check(None)
    assert sequencer.running(PERCEPTION) is job
    sequencer.close()
    for lane in (MANIPULATION, None):
        with pytest.raises(Refusal, match="^the node is shutting down$"):
            sequencer.check(lane)


async def test_a_lane_less_job_holds_no_lane_and_is_refused_once_closed():
    sequencer = Sequencer()
    switch = sequencer.start(None, "switch_model", now_ns=0)
    assert switch.lane is None
    # Both lanes stay free beside it.
    sequencer.finish(sequencer.start(MANIPULATION, "grab_item", now_ns=1))
    sequencer.finish(sequencer.start(PERCEPTION, "scan_items", now_ns=1))
    sequencer.close()
    with pytest.raises(Refusal, match="^the node is shutting down$"):
        sequencer.start(None, "switch_model", now_ns=2)
    sequencer.finish(switch)
    assert switch.done.is_set()


async def test_stopping_the_lane_less_jobs_sets_each_token_and_waits_for_each():
    stopped_lanes: list = []

    async def stopper(lane) -> None:
        stopped_lanes.append(lane)

    sequencer = Sequencer(stopper=stopper)
    jobs = [sequencer.start(None, "switch_model", now_ns=0) for _ in range(2)]

    async def worker(job):
        await job.cancel.wait()
        assert (job.cancel.reason, job.cancel.signalled) == ("aborted: the node is shutting down", False)
        sequencer.finish(job)

    workers = [asyncio.create_task(worker(job)) for job in jobs]
    await asyncio.wait_for(sequencer.stop_laneless("aborted: the node is shutting down"), 1.0)
    await asyncio.gather(*workers)
    assert all(job.done.is_set() for job in jobs)
    # A lane-less job has no lane to halt.
    assert stopped_lanes == []
    await asyncio.wait_for(sequencer.stop_laneless("again"), 1.0)
