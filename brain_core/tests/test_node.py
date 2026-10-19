"""The shared entry against fake bindings: a brain node boots, serves every
member, registers its shutdown and stops on its token; and its log lines
reach stderr."""

import asyncio
import logging
from types import SimpleNamespace

from conftest import PARAMS, FakeDetector, FakeManipulator, FakeSource, FakeToken, records_of
from fake_peppygen import fake_bindings

from brain_core.node import configure_logging, setup


class FakeRunner:
    """The node runner as setup uses it: the token and the shutdown hooks."""

    def __init__(self) -> None:
        self.token = FakeToken()
        self.shutdown_hooks: list = []

    def cancellation_token(self) -> FakeToken:
        return self.token

    def on_shutdown(self, hook) -> None:
        self.shutdown_hooks.append(hook)


async def test_a_brain_node_boots_and_stops_on_its_token(caplog):
    runner = FakeRunner()
    bindings = fake_bindings()
    detector, manipulator = FakeDetector(), FakeManipulator()
    source = FakeSource.of(detector, manipulator)
    params = SimpleNamespace(**{**PARAMS, "perception_model": "fake", "manipulation_model": "fake"})
    with records_of(caplog, "brain_core.node") as log:
        loops = await setup(params, runner, bindings, source)
    assert bindings.clock.initialised_with is runner
    assert manipulator.robot is not None
    assert [record.getMessage() for record in log.records] == [
        "grippers ['left_gripper', 'right_gripper'], perception 'fake', manipulation 'fake'"
    ]
    [shutdown] = runner.shutdown_hooks
    # Six actions, get_state, and the camera follower and the two geometry
    # learners, which return at once on vacant slots.
    assert len(loops) == 10
    runner.token.cancel()
    await asyncio.wait_for(asyncio.gather(*loops), 1.0)
    await asyncio.wait_for(shutdown(), 1.0)


def test_the_brains_info_lines_reach_stderr(capsys):
    configure_logging("some_brain")
    configure_logging("some_brain")  # idempotent: one handler each, however many times it is called
    logging.getLogger("brain_core.perception.perceiver").info("perceiver: %d boxes", 3)
    logging.getLogger("some_brain.perception.detector").info("detector: %d items", 120)
    logging.getLogger("transformers").info("stays quiet")
    err = capsys.readouterr().err
    assert err.count("[brain] perceiver: 3 boxes") == 1
    assert err.count("[brain] detector: 120 items") == 1
    assert "stays quiet" not in err
