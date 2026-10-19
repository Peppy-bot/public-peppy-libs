"""The entry every brain node shares. Wiring only: build the brain with the
node's generated modules and the backends the node selected, and start one
loop per exposed member plus the camera follower. The rules live in the
core, the moves and the model behind the two ports."""

from __future__ import annotations

import asyncio
import logging
import sys
from functools import partial

from .bindings import Bindings
from .brain import Brain
from .handlers import abort as abort_handler
from .handlers import drop_item as drop_item_handler
from .handlers import get_state as get_state_handler
from .handlers import grab_item as grab_item_handler
from .handlers import identify_item as identify_item_handler
from .handlers import place_item as place_item_handler
from .handlers import scan_items as scan_items_handler
from .ports import Detector, Manipulator
from .serve import serve_action, serve_service

logger = logging.getLogger(__name__)


async def setup(
    params,
    node_runner,
    bindings: Bindings,
    detector: Detector,
    manipulator: Manipulator,
) -> list[asyncio.Task]:
    """A brain node's setup: `params` are the node's parameters, `bindings`
    its generated modules, `detector` and `manipulator` the backends the
    node selected."""
    await bindings.clock.init(node_runner)
    token = node_runner.cancellation_token()
    brain = Brain(params, node_runner, bindings, detector, manipulator)
    await brain.start()
    node_runner.on_shutdown(brain.shutdown)
    logger.info(
        "grippers %s, perception '%s', manipulation '%s'",
        brain.state.gripper_names(), brain.detector.name, brain.manipulator.name,
    )
    # Per-goal tasks live here so a goal in flight is never collected
    # before it completes.
    goal_tasks: set[asyncio.Task] = set()
    actions = [
        (bindings.scan_items, scan_items_handler.run),
        (bindings.identify_item, identify_item_handler.run),
        (bindings.grab_item, grab_item_handler.run),
        (bindings.drop_item, drop_item_handler.run),
        (bindings.place_item, place_item_handler.run),
        (bindings.abort, abort_handler.run),
    ]
    loops = [
        asyncio.create_task(serve_action(module, node_runner, token, goal_tasks, partial(run, brain)))
        for module, run in actions
    ]
    loops.append(
        asyncio.create_task(serve_service(bindings.get_state, node_runner, token, partial(get_state_handler.handle, brain)))
    )
    loops.extend(brain.background(token))
    return loops


def configure_logging(node_package: str, level: int = logging.INFO) -> None:
    """The log lines of brain_core and of the node's own package
    `node_package` reach the daemon's log, each opened with "[brain]".
    Python drops INFO by default; only these two loggers are raised, so the
    model libraries stay quiet. One handler each however many times it is
    called."""
    for name in ("brain_core", node_package):
        package_logger = logging.getLogger(name)
        if not package_logger.handlers:
            handler = logging.StreamHandler(sys.stderr)
            handler.setFormatter(logging.Formatter("[brain] %(message)s"))
            package_logger.addHandler(handler)
        package_logger.setLevel(level)
        package_logger.propagate = False
