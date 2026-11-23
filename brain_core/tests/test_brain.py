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
    LOOKUP_TIMEOUT_S, PARAMS, QUIET_GRAB, FakeCtx, FakeDetector, Reports, FakeManipulator, FakeSource, FakeToken, Ticks, failed_with,
    lookup_answer, make_brain, records_of, sensors_described, with_frames, run_goal,
)
from fake_peppygen import (
    ROBOT_FRAME, FakeConsumedService, FakeTopic, LookupTransformRequest, answering, fake_bindings, limb_states_message,
)
from brain_core.brain import RESULT_ENDS, STATUS_ENDS, Brain, Decider
from brain_core.manipulators.none import NoneManipulator
from control_core_py.geometry import Placement

from brain_core.perception.geometry import NOT_DESCRIBED
from brain_core.perception.none import NoneDetector
from brain_core.handlers import abort, scan_items
from brain_core.perception.frames import FrameStore
from brain_core.ports import ExpectedFailure, FailureKind, Refusal, Role
from brain_core.stages import scan_look


async def test_the_learners_give_the_perceiver_its_intrinsics_and_the_placer_the_robot_frame():
    colour = SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])
    depth = SimpleNamespace(depth_model="z")
    lookup = FakeConsumedService("lookup_transform", LookupTransformRequest, producer="robot")
    lookup.enqueue(lookup_answer((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0)))
    bindings = fake_bindings(
        get_color_intrinsics=answering("get_color_intrinsics", colour),
        get_depth_intrinsics=answering("get_depth_intrinsics", depth),
        describe_sensors=answering("describe_sensors", sensors_described(("chest", "wrist_left"))),
        lookup_transform=lookup,
    )
    brain = Brain(SimpleNamespace(**PARAMS), None, bindings, FakeSource.of(NoneDetector(), NoneManipulator()), now=Ticks(), run_token="t0")
    await brain.start()
    assert brain.perceiver.camera.intrinsics is None
    with pytest.raises(Refusal, match=f"^{NOT_DESCRIBED}$"):
        brain.placer.robot_frame
    # The camera slot is vacant, so the frame follower returns at once; the
    # two learners return once they have their answer.
    await asyncio.wait_for(asyncio.gather(*brain.background(FakeToken())), 1.0)
    camera = brain.perceiver.camera
    assert (camera.intrinsics.fx, camera.intrinsics.cx) == (6.0, 8.0)
    assert brain.perceiver.placer is brain.placer and brain.placer.robot_frame == "base_link"
    # A manipulator asks where the frame a picture names stood when the
    # picture was taken.
    header = SimpleNamespace(frame="wrist_left_color_optical_frame", timestamp=3.25, capture_index=7)
    assert await brain.senses.camera_pose(header) == Placement((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    assert lookup.polled == [
        LookupTransformRequest("base_link", "wrist_left_color_optical_frame", 3.25, 3.25, LOOKUP_TIMEOUT_S)
    ]
    assert not hasattr(brain, "camera") and not hasattr(brain, "frames")


async def test_a_lookup_timeout_transform_lookup_does_not_take_stops_the_node_at_its_start():
    source = FakeSource.of(NoneDetector(), NoneManipulator())
    for timeout_s in (-1.0, 2.0):
        params = SimpleNamespace(**{**PARAMS, "lookup_timeout_s": timeout_s})
        with pytest.raises(ValueError, match=f"^lookup_timeout_s is {timeout_s}: set it from 0 to 1 s"):
            Brain(params, None, fake_bindings(), source, now=Ticks(), run_token="t0")


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
SWITCH_ZERO = dict(role="", model="")


def grab_ctx() -> FakeCtx:
    return FakeCtx(fake_peppygen.grab_item)


def switch_ctx() -> FakeCtx:
    return FakeCtx(fake_peppygen.switch_model, role="perception", model="fast")


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
        outcome = await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel, QUIET_GRAB)
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
    await run_goal(abort, brain, stop)
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
        await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel, QUIET_GRAB)
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


async def test_the_detectors_reason_comes_before_any_frame_is_read_then_the_perceivers_and_a_refused_look_reports_nothing():
    detector = FakeDetector(unavailable="perception_model 'sam3_siglip' is still loading")
    brain = await make_brain(detector=detector)
    frames = CountingFrames()
    brain.perceiver.frames = frames
    job = brain.sequencer.start(Role.PERCEPTION, "scan_items", 0)
    reports = Reports()
    with pytest.raises(Refusal, match="^no perception source: perception_model 'sam3_siglip' is still loading$") as raised:
        await brain.look(job, detector, [], 0.0, publish=reports, report=scan_look())
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert frames.reads == 0 and detector.phrases == []
    detector.unavailable = ""
    with pytest.raises(Refusal, match="^no perception source: no camera frame received$") as raised:
        await brain.look(job, detector, [], 0.0, publish=reports, report=scan_look())
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert frames.reads == 1 and detector.phrases == []
    assert reports.reports == []


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
    with_frames(brain)
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
    with_frames(brain)
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


async def test_the_decider_accepts_refuses_with_the_kind_refuses_a_broken_admit_and_gives_the_goal_it_accepted(caplog):
    brain = await make_brain()
    module = ScriptedModule()
    request = SimpleNamespace(data=SimpleNamespace(role="arm"))
    seen = []

    async def passing(brain_, goal):
        seen.append((brain_, goal.role))
        return f"the goal of {goal.role}"

    async def refusing(brain_, goal):
        raise Refusal(FailureKind.REJECTED, f"this brain runs no model for the role '{goal.role}'")

    async def broken(brain_, goal):
        raise KeyError("role")

    deciders = [Decider(module, brain, admit) for admit in (passing, refusing, broken, None)]
    with records_of(caplog, "brain_core.brain") as log:
        accepted, refused, failed, every = [await decider.decide(request) for decider in deciders]
    assert accepted.refusal is None and every.refusal is None
    assert seen == [(brain, "arm")]
    assert refused.refusal == module.Failure("rejected", "this brain runs no model for the role 'arm'")
    assert failed.refusal == module.Failure("attempt_failed", "probe_action could not be decided: KeyError('role')")
    assert [(record.levelno, record.getMessage()) for record in log.records] == [
        (logging.INFO, "probe_action refused: this brain runs no model for the role 'arm'"),
        (logging.ERROR, "probe_action could not be decided: KeyError('role')"),
    ]
    assert log.records[1].exc_info is not None
    # Each decider gives the goal of its accepted decision once: the goal
    # its admit made, or the request data without one; a refused goal none.
    taken = [decider.take() for decider in deciders]
    assert taken == ["the goal of arm", None, None, request.data]
    assert [decider.take() for decider in deciders] == [None] * 4


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
    ticks = Ticks()
    brain = await make_brain(manipulator=manipulator, ticks=ticks)
    began = asyncio.Event()

    async def search(job) -> dict:
        began.set()
        await job.cancel.wait()
        job.cancel.check()
        return {}

    async def grab(job) -> dict:
        await manipulator.grab(SimpleNamespace(item_id="cup"), SimpleNamespace(name="left_gripper"), 0.0, job.cancel, QUIET_GRAB)
        return dict(GRAB_ZERO)

    scan = FakeCtx(fake_peppygen.scan_items, timeout_s=0.0)
    grabbing = grab_ctx()
    searching = asyncio.create_task(
        brain.run_guarded(scan, Role.PERCEPTION, "scan_items", search, scan_items.ZERO, ends=STATUS_ENDS, stamped=True)
    )
    moving = asyncio.create_task(brain.run_guarded(grabbing, Role.MANIPULATION, "grab_item", grab, GRAB_ZERO, ends=RESULT_ENDS))
    await began.wait()
    await manipulator.begun()
    ticks.now_ns += 2_500_000_000
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await asyncio.wait_for(asyncio.gather(searching, moving), 1.0)
    # The empty result of the scan names the robot frame and the time of the cancel.
    assert (scan.cancelled_reason, scan.cancelled) == (
        "the node is shutting down", dict(scan_items.ZERO, frame=ROBOT_FRAME, timestamp=3.5, action_time=2.5),
    )
    assert grabbing.cancelled is None
    assert grabbing.completed == dict(success=False, message="aborted: the node is shutting down", action_time=2.5, **GRAB_ZERO)
