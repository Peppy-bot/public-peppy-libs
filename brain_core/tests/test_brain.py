"""The brain assembled: what it hands its parts, where the camera model
lives, how a goal is decided before it is accepted, and how run_guarded
ends a goal and logs it, by its status or in its result, with a lane or
without one."""

import asyncio
import logging
from types import SimpleNamespace

import fake_peppygen
import pytest
from conftest import (
    CAMERA_16X12, FakeCtx, FakeDetector, FakeManipulator, FakeToken, Ticks, camera_poses, depth_frame, failed_with, make_brain,
    records_of, rgb_frame,
)
from fake_peppygen import answering, fake_bindings
from brain_core.brain import RESULT_ENDS, STATUS_ENDS, decider
from brain_core.handlers import abort
from brain_core.perception.frames import FrameStore
from brain_core.ports import ExpectedFailure, FailureKind, Refusal, Role


async def test_the_geometry_learners_give_the_perceiver_its_camera_and_the_brain_keeps_none():
    colour = SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])
    depth = SimpleNamespace(depth_model="z")
    bindings = fake_bindings(
        get_color_intrinsics=answering("get_color_intrinsics", colour),
        get_depth_intrinsics=answering("get_depth_intrinsics", depth),
        get_camera_poses=answering("get_camera_poses", camera_poses()),
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


GRAB_ZERO = dict(gripper_name="", item_id="")
SWITCH_ZERO = dict(role="", model="")


def grab_ctx() -> FakeCtx:
    return FakeCtx(fake_peppygen.grab_item)


def switch_ctx() -> FakeCtx:
    return FakeCtx(fake_peppygen.switch_model, role="perception", model="fast")


def ready_to_search(brain) -> None:
    """The camera's frames, its intrinsics and its pose, as the slots give
    them to a running brain."""
    brain.perceiver.frames.add_color(rgb_frame(16, 12))
    brain.perceiver.frames.add_depth(depth_frame(1.0, 16, 12))
    brain.perceiver.frames.depth_unit = 0.001
    brain.perceiver.set_camera(brain.perceiver.camera.with_intrinsics(CAMERA_16X12).with_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


async def test_action_time_is_the_elapsed_time_whatever_the_outcome():
    ticks = Ticks()
    brain = await make_brain(ticks=ticks)

    def taking(seconds: float, then):
        async def body(job) -> dict:
            ticks.now_ns += int(seconds * 1e9)
            return then(job)

        return body

    def refuse(job):
        raise Refusal(FailureKind.REJECTED, "gripper 'claw' is unknown")

    def fail_as_expected(job):
        raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, "model-x failed: RateLimitError: slow down")

    def break_down(job):
        raise RuntimeError("the GPU is gone")

    def succeed(job):
        return dict(gripper_name="left_gripper", item_id="cup_1-t0")

    for seconds, then in ((1.5, refuse), (2.0, fail_as_expected), (2.5, break_down), (3.0, succeed)):
        ctx = grab_ctx()
        await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", taking(seconds, then), GRAB_ZERO, ends=RESULT_ENDS)
        assert ctx.completed["action_time"] == seconds, then.__name__

    # A refusal at admission: the clock moves a quarter second at each
    # reading, one when the goal arrives and one when it ends.
    ticks.step_ns = 250_000_000
    busy = brain.sequencer.start(Role.MANIPULATION, "drop_item", ticks.now_ns)
    ctx = grab_ctx()
    await brain.run_guarded(ctx, Role.MANIPULATION, "grab_item", taking(0.0, succeed), GRAB_ZERO, ends=RESULT_ENDS)
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
    running = asyncio.create_task(brain.run_guarded(cancelled, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO, ends=RESULT_ENDS))
    await manipulator.begun()
    ticks.now_ns += 1_250_000_000
    cancelled.cancel()
    await asyncio.wait_for(running, 1.0)
    assert cancelled.cancelled["action_time"] == 1.25

    aborted = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(aborted, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO, ends=RESULT_ENDS))
    await manipulator.begun()
    ticks.now_ns += 4_000_000_000
    stop = FakeCtx(fake_peppygen.abort, reason="operator")
    await abort.run(brain, stop)
    await asyncio.wait_for(running, 1.0)
    assert (aborted.completed["message"], aborted.completed["action_time"]) == ("aborted: operator", 4.0)
    assert (stop.completed["aborted_action"], stop.completed["action_time"]) == ("grab_item", 0.0)


async def test_an_expected_failure_ends_with_one_line_and_no_traceback_by_status_or_in_the_result(caplog):
    brain = await make_brain()

    async def body(job) -> dict:
        job.notes.append("frame 3, no box")
        raise ExpectedFailure(FailureKind.ATTEMPT_FAILED, "model-x failed: RateLimitError: slow down")

    in_result = grab_ctx()
    by_status = switch_ctx()
    with records_of(caplog, "brain_core.brain") as log:
        await brain.run_guarded(in_result, Role.MANIPULATION, "grab_item", body, GRAB_ZERO, ends=RESULT_ENDS)
        await brain.run_guarded(by_status, None, "switch_model", body, SWITCH_ZERO, ends=STATUS_ENDS)
    assert in_result.completed == dict(success=False, message="model-x failed: RateLimitError: slow down", action_time=0.0, **GRAB_ZERO)
    # By status: a failed goal, its kind and no result.
    assert failed_with(by_status, FailureKind.ATTEMPT_FAILED, "model-x failed: RateLimitError: slow down")
    assert [(record.levelno, record.getMessage()) for record in log.records] == [
        (logging.WARNING, "grab_item failed after 0.00 s: model-x failed: RateLimitError: slow down; frame 3, no box"),
        (logging.WARNING, "switch_model failed after 0.00 s: model-x failed: RateLimitError: slow down; frame 3, no box"),
    ]
    assert all(record.exc_info is None for record in log.records) and "Traceback" not in log.text


async def test_by_status_a_refusal_or_a_fault_fails_the_goal_with_its_kind_and_no_result(caplog):
    brain = await make_brain()

    async def refuse(job) -> dict:
        raise Refusal(FailureKind.REJECTED, "this brain runs no model for the role 'arm'")

    async def break_down(job) -> dict:
        raise RuntimeError("the probe broke")

    async def succeed(job) -> dict:
        return dict(role="perception", model="fast")

    refused, broken, succeeded = switch_ctx(), switch_ctx(), switch_ctx()
    with records_of(caplog, "brain_core.brain") as log:
        await brain.run_guarded(refused, None, "switch_model", refuse, SWITCH_ZERO, ends=STATUS_ENDS)
        await brain.run_guarded(broken, None, "switch_model", break_down, SWITCH_ZERO, ends=STATUS_ENDS)
        await brain.run_guarded(succeeded, None, "switch_model", succeed, SWITCH_ZERO, ends=STATUS_ENDS)
    assert failed_with(refused, FailureKind.REJECTED, "this brain runs no model for the role 'arm'")
    assert failed_with(broken, FailureKind.ATTEMPT_FAILED, "switch_model failed: RuntimeError('the probe broke')")
    # A success carries no success field: the status says it.
    assert succeeded.completed == dict(role="perception", model="fast", action_time=0.0)
    assert [record.levelno for record in log.records] == [logging.INFO, logging.ERROR, logging.INFO]
    assert log.records[1].exc_info is not None


async def test_by_status_a_goal_that_loses_its_lane_after_the_decision_fails_with_the_refusals_kind():
    brain = await make_brain()
    busy = brain.sequencer.start(Role.PERCEPTION, "scan_items", 0)

    async def never(job) -> dict:
        raise AssertionError("a refused goal ran its body")

    ctx = FakeCtx(fake_peppygen.identify_item, description="cup", timeout_s=0.0)
    await brain.run_guarded(ctx, Role.PERCEPTION, "identify_item", never, {}, ends=STATUS_ENDS)
    assert failed_with(ctx, FailureKind.PRECONDITION_FAILED, "scan_items is running")
    brain.sequencer.finish(busy)


@pytest.mark.parametrize("cause, reason", [("caller", "cancelled by its caller"), ("sender_gone", "its sender is gone")])
async def test_a_goals_own_cancel_ends_it_cancelled_with_the_reason_of_its_cause(cause, reason):
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(manipulator=manipulator)

    async def grab(job) -> dict:
        await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel)
        return dict(GRAB_ZERO)

    async def wait_for_a_stop(job) -> dict:
        await job.cancel.wait()
        job.cancel.check()
        return dict(role="perception", model="fast")

    in_result = grab_ctx()
    running = asyncio.create_task(brain.run_guarded(in_result, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO, ends=RESULT_ENDS))
    await manipulator.begun()
    in_result.cancel(cause)
    await asyncio.wait_for(running, 1.0)
    assert in_result.cancelled_reason == reason
    assert in_result.cancelled == dict(success=False, message=reason, action_time=0.0, **GRAB_ZERO)
    assert manipulator.stopped == 1

    by_status = switch_ctx()
    running = asyncio.create_task(brain.run_guarded(by_status, None, "switch_model", wait_for_a_stop, SWITCH_ZERO, ends=STATUS_ENDS))
    by_status.cancel(cause)
    await asyncio.wait_for(running, 1.0)
    assert (by_status.cancelled_reason, by_status.cancelled) == (reason, dict(action_time=0.0, **SWITCH_ZERO))


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
    with pytest.raises(Refusal, match="^no perception source: perception_model 'sam3_siglip' is still loading$") as raised:
        await brain.look(job, detector, [], 0.0)
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert frames.reads == 0 and detector.phrases == []
    detector.unavailable = ""
    with pytest.raises(Refusal, match="^no perception source: no camera frame received$") as raised:
        await brain.look(job, detector, [], 0.0)
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert frames.reads == 1 and detector.phrases == []


async def test_a_search_is_checked_at_once_for_the_shutdown_the_lane_the_model_and_the_source():
    detector = FakeDetector(unavailable="perception_model 'sam3_siglip' is still loading")
    brain = await make_brain(detector=detector)

    def refusal() -> Refusal:
        with pytest.raises(Refusal) as raised:
            brain.check_search()
        return raised.value

    assert (refusal().kind, refusal().message) == (
        FailureKind.RESOURCE_UNAVAILABLE, "no perception source: perception_model 'sam3_siglip' is still loading"
    )
    detector.unavailable = ""
    assert (refusal().kind, refusal().message) == (FailureKind.RESOURCE_UNAVAILABLE, "no perception source: no camera frame received")
    ready_to_search(brain)
    brain.check_search()
    # A running search refuses the next one, before its source is asked.
    busy = brain.sequencer.start(Role.PERCEPTION, "identify_item", 0)
    detector.unavailable = "perception_model 'sam3_siglip' could not load"
    assert (refusal().kind, refusal().message) == (FailureKind.PRECONDITION_FAILED, "identify_item is running")
    brain.sequencer.finish(busy)
    brain.sequencer.close()
    assert (refusal().kind, refusal().message) == (FailureKind.PRECONDITION_FAILED, "the node is shutting down")


async def test_a_perception_model_none_refuses_a_search_with_precondition_failed():
    brain = await make_brain()
    ready_to_search(brain)
    with pytest.raises(Refusal, match="^no perception source: perception_model is 'none'$") as raised:
        brain.check_search()
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED


class ScriptedModule:
    """A generated action module for the decider: its name, and the
    GoalDecision of a member whose kinds are the kinds of peppy."""

    def __init__(self) -> None:
        action = fake_peppygen.FakeAction("probe_action", ("action_time",))
        self.ACTION_NAME = action.ACTION_NAME
        self.GoalDecision = action.GoalDecision
        self.Failure = action.Failure


async def test_the_decider_accepts_refuses_with_the_kind_and_refuses_a_broken_check(caplog):
    brain = await make_brain()
    module = ScriptedModule()
    request = SimpleNamespace(data=SimpleNamespace(role="arm"))
    seen = []

    def passing(brain_, goal) -> None:
        seen.append((brain_, goal.role))

    def refusing(brain_, goal) -> None:
        raise Refusal(FailureKind.REJECTED, f"this brain runs no model for the role '{goal.role}'")

    def broken(brain_, goal) -> None:
        raise KeyError("role")

    with records_of(caplog, "brain_core.brain") as log:
        accepted = decider(module, brain, passing)(request)
        refused = decider(module, brain, refusing)(request)
        failed = decider(module, brain, broken)(request)
        every = decider(module, brain, None)(request)
    assert accepted.refusal is None and every.refusal is None
    assert seen == [(brain, "arm")]
    assert refused.refusal == module.Failure("rejected", "this brain runs no model for the role 'arm'")
    assert failed.refusal == module.Failure("attempt_failed", "probe_action could not be decided: KeyError('role')")
    assert [(record.levelno, record.getMessage()) for record in log.records] == [
        (logging.INFO, "probe_action refused: this brain runs no model for the role 'arm'"),
        (logging.ERROR, "probe_action could not be decided: KeyError('role')"),
    ]
    assert log.records[1].exc_info is not None


async def test_a_shutdown_ends_a_lane_less_job_cancelled_and_refuses_every_goal_after():
    brain = await make_brain()
    began = asyncio.Event()

    async def wait_for_a_stop(job) -> dict:
        began.set()
        await job.cancel.wait()
        job.cancel.check()
        return dict(role="perception", model="fast")

    ctx = switch_ctx()
    running = asyncio.create_task(brain.run_guarded(ctx, None, "switch_model", wait_for_a_stop, SWITCH_ZERO, ends=STATUS_ENDS))
    await began.wait()
    # Both lanes are free beside it, and get_state reports no action.
    assert brain.sequencer.running(Role.MANIPULATION) is None and brain.sequencer.running(Role.PERCEPTION) is None
    brain.sequencer.finish(brain.sequencer.start(Role.MANIPULATION, "drop_item", 0))
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await asyncio.wait_for(running, 1.0)
    # A switch ends cancelled, the shutdown its reason.
    assert (ctx.cancelled_reason, ctx.cancelled) == ("the node is shutting down", dict(action_time=0.0, **SWITCH_ZERO))
    # Once the shutdown began, a lane-less goal is refused too.
    late = switch_ctx()
    await brain.run_guarded(late, None, "switch_model", wait_for_a_stop, SWITCH_ZERO, ends=STATUS_ENDS)
    assert failed_with(late, FailureKind.PRECONDITION_FAILED, "the node is shutting down")


async def test_a_shutdown_ends_a_search_cancelled_and_a_manipulation_aborted():
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(manipulator=manipulator)
    began = asyncio.Event()

    async def search(job) -> dict:
        began.set()
        await job.cancel.wait()
        job.cancel.check()
        return {}

    async def grab(job) -> dict:
        await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel)
        return dict(GRAB_ZERO)

    scan = FakeCtx(fake_peppygen.scan_items, timeout_s=0.0)
    scan_zero = dict(item_ids=[], labels=[], positions=[], confidences=[], regions=[], camera="", image_width=0, image_height=0, frame_timestamp=0.0)
    grabbing = grab_ctx()
    searching = asyncio.create_task(brain.run_guarded(scan, Role.PERCEPTION, "scan_items", search, scan_zero, ends=STATUS_ENDS))
    moving = asyncio.create_task(brain.run_guarded(grabbing, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO, ends=RESULT_ENDS))
    await began.wait()
    await manipulator.begun()
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await asyncio.wait_for(asyncio.gather(searching, moving), 1.0)
    assert (scan.cancelled_reason, scan.cancelled) == ("the node is shutting down", dict(action_time=0.0, **scan_zero))
    assert grabbing.cancelled is None
    assert grabbing.completed == dict(success=False, message="aborted: the node is shutting down", action_time=0.0, **GRAB_ZERO)
