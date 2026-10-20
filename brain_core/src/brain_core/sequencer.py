"""One sequence at a time per lane, and the way to stop the one running.

The lanes are the roles (`Role`), because the contracts allow a search
while a manipulation runs: `Role.PERCEPTION` for scan_items and
identify_item, `Role.MANIPULATION` for grab_item, drop_item and
place_item. get_state reports the manipulation lane only; a running search
never appears there, so a caller that reads "identify_item is running" is
never talked out of a grab.

A lane-less job runs beside both lanes and holds neither: switch_model,
which checks the lane of its role itself (`Models.switch`). It gets what a
lane's job gets: the refusal once the sequencer is closed, a token a
cancel sets, and the stop of a shutdown.

A stop is the same whether the caller cancelled its own goal or anyone
sent abort: the job's token is set, the lane's stopper halts the backend
and the move in flight, and the job finishes on its own.

`close` is the first thing a shutdown does: from then on every sequence
is refused with "the node is shutting down", so a goal that arrives while
the lanes stop never finds a free lane.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Optional

from .ports import CancelToken, Refusal, Role

Stopper = Callable[[Role], Awaitable[None]]

SHUTTING_DOWN = "the node is shutting down"


@dataclass(eq=False)
class Job:
    # The lane the job runs in, or none for a lane-less job.
    lane: Optional[Role]
    action: str
    started_ns: int
    cancel: CancelToken = field(default_factory=CancelToken)
    done: asyncio.Event = field(default_factory=asyncio.Event)
    # What the sequence found out on its way, in a few words each, for the
    # line its goal leaves in the log (`Brain.run_guarded`).
    notes: list[str] = field(default_factory=list)

    def elapsed_s(self, now_ns: int) -> float:
        return max(0.0, (now_ns - self.started_ns) / 1e9)


class Sequencer:
    def __init__(self, stopper: Optional[Stopper] = None) -> None:
        self._running: dict[Role, Job] = {}
        self._laneless: set[Job] = set()
        self._stopper = stopper
        self._closed = False

    def running(self, lane: Role) -> Optional[Job]:
        return self._running.get(lane)

    def start(self, lane: Optional[Role], action: str, now_ns: int) -> Job:
        """Admits a sequence into its lane, or refuses it naming the one
        that runs, as the contracts ask; a lane-less one when `lane` is
        none. Once closed, refuses every one."""
        if self._closed:
            raise Refusal(SHUTTING_DOWN)
        if lane is None:
            job = Job(lane=None, action=action, started_ns=now_ns)
            self._laneless.add(job)
            return job
        running = self._running.get(lane)
        if running is not None:
            raise Refusal(f"{running.action} is running")
        job = Job(lane=lane, action=action, started_ns=now_ns)
        self._running[lane] = job
        return job

    def close(self) -> None:
        """Refuses every sequence from now on. The ones that run go on
        until they are stopped or end."""
        self._closed = True

    def finish(self, job: Job) -> None:
        if job.lane is None:
            self._laneless.discard(job)
        elif self._running.get(job.lane) is job:
            del self._running[job.lane]
        job.done.set()

    async def cancel(self, job: Job, reason: str, *, by_caller: bool) -> None:
        """Stops one job: sets its token and halts its lane's backend."""
        if job.done.is_set():
            return
        job.cancel.cancel(reason, by_caller=by_caller)
        if job.lane is not None and self._stopper is not None:
            await self._stopper(job.lane)

    async def stop(self, lane: Role, reason: str) -> Optional[str]:
        """abort: stops whatever runs in the lane and waits until it has
        finished, which for manipulation means the robot has come to rest.
        Returns the stopped action's name, or none when the lane was idle."""
        job = self._running.get(lane)
        if job is None:
            return None
        await self.cancel(job, reason, by_caller=False)
        await job.done.wait()
        return job.action

    async def stop_laneless(self, reason: str) -> None:
        """Stops every lane-less job and waits until each has finished."""
        jobs = list(self._laneless)
        for job in jobs:
            await self.cancel(job, reason, by_caller=False)
        for job in jobs:
            await job.done.wait()
