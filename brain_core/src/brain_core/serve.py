"""The loops every exposed member runs, written once.

An action loop waits for a goal, decides it, hands an accepted one to its
own task and goes back to waiting, until the node's cancellation token
fires. The per-goal tasks are kept in a set the caller owns: asyncio holds
tasks weakly, and a task collected mid-await drops its goal context
without ending it, which the caller sees as abandoned.

A service loop listens once and answers one request at a time, until the
token fires; it listens again when its service stops.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable

from .waiting import unless_cancelled

logger = logging.getLogger(__name__)


async def serve_action(
    action_module, node_runner, token, tasks: set, decide: Callable[[object], object], run: Callable[[object], Awaitable[None]]
) -> None:
    """Serves one exposed action: `decide` gives the generated decision of
    each goal at once, as `brain.decider` makes it, and `run` carries out
    each accepted goal in a task of its own."""
    action = await action_module.ActionHandle.expose(node_runner)
    while not token.is_cancelled():
        ctx = await unless_cancelled(token, action.handle_goal_next_request(decide))
        if ctx is None:
            return
        task = asyncio.create_task(run(ctx))
        tasks.add(task)
        task.add_done_callback(tasks.discard)
        task.add_done_callback(report_failure)


def report_failure(task: asyncio.Task) -> None:
    """A handler that raises leaves its goal without an end, which the
    caller sees as abandoned; the reason must at least reach the log."""
    if task.cancelled():
        return
    error = task.exception()
    if error is not None:
        logger.error("a goal handler failed and its goal was never completed", exc_info=error)


async def serve_service(service_module, node_runner, token, handler) -> None:
    """Serves one exposed service from one endpoint, one request after
    another, and listens again when the service stops: its request stream
    ends, or it cannot listen or send a reply, which the log says."""
    while not token.is_cancelled():
        try:
            await unless_cancelled(token, service_module.handle_requests(node_runner, handler))
        except Exception as error:
            logger.error("%s service failed: %r", service_module.SERVICE_NAME, error)
