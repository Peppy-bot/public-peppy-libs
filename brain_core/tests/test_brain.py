"""The brain assembled: what it hands its parts, where the camera model
lives, and how run_guarded completes a goal and logs it, with a lane or
without one."""

import asyncio
import logging
from types import SimpleNamespace

import fake_peppygen
import pytest
from conftest import FakeCtx, FakeDetector, FakeManipulator, FakeToken, Ticks, camera_poses, make_brain, records_of
from fake_peppygen import FakeTopic, fake_bindings, limb_states_message
from brain_core.handlers import abort
from brain_core.perception.frames import FrameStore
from brain_core.ports import ExpectedFailure, Refusal, Role


def answering(data):
    """A consumed service bound to one producer that answers `data`."""

    async def poll(node_runner, producer, timeout):
        return SimpleNamespace(data=data)

    return SimpleNamespace(bound_producer=lambda node_runner: "producer", poll=poll)


async def test_the_geometry_learners_give_the_perceiver_its_camera_and_the_brain_keeps_none():
    colour = SimpleNamespace(
        success=True, message="", width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[]
    )
    depth = SimpleNamespace(success=True, message="", depth_model="z")
    bindings = fake_bindings(
        get_color_intrinsics=answering(colour),
        get_depth_intrinsics=answering(depth),
        get_camera_poses=answering(camera_poses()),
    )
    brain = await make_brain(bindings=bindings)
    assert not brain.perceiver.camera.ready
    # The camera slot is vacant, so the frame follower returns at once; the
    # two learners return once they have their answer.
    await asyncio.wait_for(asyncio.gather(*brain.background(FakeToken())), 1.0)
    camera = brain.perceiver.camera
    assert camera.ready
    assert (camera.intrinsics.fx, camera.intrinsics.cx) == (6.0, 8.0)
    assert camera.pose.position == (0.0, 0.0, 0.0)
    assert not hasattr(brain, "camera") and not hasattr(brain, "frames")


async def test_the_limb_follower_runs_in_the_background_and_fills_the_brains_store():
    topic = FakeTopic()
    brain = await make_brain(bindings=fake_bindings(limb_states=topic))
    token = FakeToken()
    loops = brain.background(token)
    topic.push(limb_states_message(3.0, left_gripper=0.5))
    snapshot = await asyncio.wait_for(brain.limb_states.newer_than(0.0, 1.0), 1.0)
    assert snapshot.opening("left_gripper") == 0.5
    token.cancel()
    await asyncio.wait_for(asyncio.gather(*loops), 1.0)


GRAB_ZERO = dict(gripper_name="", item_id="")


def grab_ctx() -> FakeCtx:
    return FakeCtx(fake_peppygen.grab_item)


async def test_action_time_is_the_elapsed_time_whatever_the_outcome():
    ticks = Ticks()
    brain = await make_brain(ticks=ticks)

    def taking(seconds: float, then):
        async def body(job) -> dict:
            ticks.now_ns += int(seconds * 1e9)
            return then(job)

        return body

    def refuse(job):
        raise Refusal("gripper 'claw' is unknown")

    def fail_as_expected(job):
        raise ExpectedFailure("model-x failed: RateLimitError: slow down")

    def break_down(job):
        raise RuntimeError("the GPU is gone")

    def succeed(job):
        return dict(gripper_name="left_gripper", item_id="cup_1-t0")

    for seconds, then in ((1.5, refuse), (2.0, fail_as_expected), (2.5, break_down), (3.0, succeed)):
        ctx = grab_ctx()
        await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", taking(seconds, then), GRAB_ZERO)
        assert ctx.completed["action_time"] == seconds, then.__name__

    # A refusal at admission: the clock moves a quarter second at each
    # reading, one when the goal arrives and one when it completes.
    ticks.step_ns = 250_000_000
    busy = brain.sequencer.start(Role.MANIPULATION, "drop_item", ticks.now_ns)
    ctx = grab_ctx()
    await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", taking(0.0, succeed), GRAB_ZERO)
    assert (ctx.completed["message"], ctx.completed["action_time"]) == ("drop_item is running", 0.25)
    brain.sequencer.finish(busy)
    ticks.step_ns = 0


async def test_a_cancelled_or_aborted_goal_reports_its_elapsed_time():
    ticks = Ticks()
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(ticks=ticks, manipulator=manipulator)

    async def grab(job) -> dict:
        outcome = await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel)
        return dict(gripper_name="left_gripper", item_id="cup") if outcome.success else GRAB_ZERO

    cancelled = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(cancelled, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO))
    await manipulator.begun()
    ticks.now_ns += 1_250_000_000
    cancelled.cancel()
    await asyncio.wait_for(running, 1.0)
    assert cancelled.cancelled["action_time"] == 1.25

    aborted = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(aborted, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO))
    await manipulator.begun()
    ticks.now_ns += 4_000_000_000
    stop = FakeCtx(fake_peppygen.abort, reason="operator")
    await abort.run(brain, stop)
    await asyncio.wait_for(running, 1.0)
    assert (aborted.completed["message"], aborted.completed["action_time"]) == ("aborted: operator", 4.0)
    assert (stop.completed["aborted_action"], stop.completed["action_time"]) == ("grab_item", 0.0)


async def test_an_expected_failure_completes_as_failed_with_one_line_and_no_traceback(caplog):
    brain = await make_brain()

    async def body(job) -> dict:
        job.notes.append("frame 3, no box")
        raise ExpectedFailure("model-x failed: RateLimitError: slow down")

    ctx = grab_ctx()
    with records_of(caplog, "brain_core.brain") as log:
        await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", body, GRAB_ZERO)
    assert ctx.completed == dict(success=False, message="model-x failed: RateLimitError: slow down", action_time=0.0, **GRAB_ZERO)
    [record] = log.records
    assert record.levelno == logging.WARNING
    assert record.getMessage() == "grab_item failed after 0.00 s: model-x failed: RateLimitError: slow down; frame 3, no box"
    assert record.exc_info is None and "Traceback" not in log.text


class CountingFrames(FrameStore):
    """A frame store that counts how often a frame is read from it."""

    def __init__(self) -> None:
        super().__init__()
        self.reads = 0

    def latest(self):
        self.reads += 1
        return super().latest()


async def test_the_detectors_reason_comes_before_any_frame_is_read_then_the_perceivers():
    detector = FakeDetector(unavailable="perception_model 'sam3_siglip' is still loading")
    brain = await make_brain(detector=detector)
    frames = CountingFrames()
    brain.perceiver.frames = frames
    job = brain.sequencer.start(Role.PERCEPTION, "scan_items", 0)
    with pytest.raises(Refusal, match="^no perception source: perception_model 'sam3_siglip' is still loading$"):
        await brain.look(job, detector, [], 0.0)
    assert frames.reads == 0 and detector.phrases == []
    detector.unavailable = ""
    with pytest.raises(Refusal, match="^no perception source: no camera frame received$"):
        await brain.look(job, detector, [], 0.0)
    assert frames.reads == 1 and detector.phrases == []


async def test_a_lane_less_job_holds_no_lane_and_a_shutdown_aborts_it():
    brain = await make_brain()
    began = asyncio.Event()

    async def wait_for_a_stop(job) -> dict:
        began.set()
        await job.cancel.wait()
        job.cancel.check()
        return dict(GRAB_ZERO)

    ctx = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(ctx, None, "grab_item", wait_for_a_stop, GRAB_ZERO))
    await began.wait()
    # Both lanes are free beside it, and get_state reports no action.
    assert brain.sequencer.running(Role.MANIPULATION) is None and brain.sequencer.running(Role.PERCEPTION) is None
    brain.sequencer.finish(brain.sequencer.start(Role.MANIPULATION, "drop_item", 0))
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await asyncio.wait_for(running, 1.0)
    assert ctx.cancelled is None
    assert (ctx.completed["success"], ctx.completed["message"]) == (False, "aborted: the node is shutting down")
    # Once the shutdown began, a lane-less goal is refused too.
    late = grab_ctx()
    await brain.run_guarded(late, None, "grab_item", wait_for_a_stop, GRAB_ZERO)
    assert late.completed["message"] == "the node is shutting down"


async def test_a_lane_less_job_cancelled_by_its_caller_completes_as_cancelled():
    brain = await make_brain()
    began = asyncio.Event()

    async def wait_for_a_stop(job) -> dict:
        began.set()
        await job.cancel.wait()
        job.cancel.check()
        return dict(GRAB_ZERO)

    ctx = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(ctx, None, "grab_item", wait_for_a_stop, GRAB_ZERO))
    await began.wait()
    ctx.cancel()
    await asyncio.wait_for(running, 1.0)
    assert ctx.completed is None
    assert (ctx.cancelled["success"], ctx.cancelled["message"]) == (False, "cancelled by the caller")
