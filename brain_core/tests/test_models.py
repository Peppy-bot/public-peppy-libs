"""The model of each role and the switch rule: a goal uses only its own
role's model, a switch never changes the model of a goal that runs, and a
switch that does not finish leaves the role's model as it was. Every fake
prepare and every slow search waits on an event the test releases."""

import asyncio

import fake_peppygen
import pytest
from conftest import CAMERA_16X12, FakeCtx, FakeDetector, FakeManipulator, FakeSource, depth_frame, failed_with, make_brain, rgb_frame

from brain_core.handlers import abort, grab_item, place_item, scan_items, switch_model
from brain_core.manipulators.none import NoneManipulator
from brain_core.ports import Box, ExpectedFailure, FailureKind, Refusal, Role

SWITCH_ZERO = dict(role="", model="")


class GatedDetector(FakeDetector):
    """A detector whose search waits until the test opens its gate;
    `searching` resolves once a search has begun."""

    def __init__(self, boxes=None) -> None:
        super().__init__(boxes)
        self.gate = asyncio.Event()
        self.searching = asyncio.Event()

    async def detect(self, image, phrases, deadline):
        self.searching.set()
        await self.gate.wait()
        return await super().detect(image, phrases, deadline)


class TrapDetector:
    """A perception model that fails the test whenever it is asked
    anything."""

    name = "trap"

    def why_unavailable(self) -> str:
        raise AssertionError("a manipulation goal asked the perception model")

    async def detect(self, image, phrases, deadline):
        raise AssertionError("a manipulation goal asked the perception model")

    def scan_coverage(self):
        raise AssertionError("a manipulation goal asked the perception model")


def two_detectors(old, new, manipulator=None) -> FakeSource:
    """Perception offers `old` and `new`; manipulation offers `manipulator`
    under its name, the none manipulator when not given."""
    manipulator = manipulator or NoneManipulator()
    return FakeSource({
        Role.PERCEPTION: {"old": old, "new": new},
        Role.MANIPULATION: {manipulator.name: manipulator},
    })


async def brain_with(source: FakeSource, manipulation_model: str = "none"):
    brain = await make_brain(source=source, perception_model="old", manipulation_model=manipulation_model)
    brain.perceiver.frames.add_color(rgb_frame(16, 12))
    brain.perceiver.frames.add_depth(depth_frame(1.0, 16, 12))
    brain.perceiver.frames.depth_unit = 0.001
    brain.perceiver.set_camera(brain.perceiver.camera.with_intrinsics(CAMERA_16X12).with_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    return brain


def switch(brain, role: Role, name: str):
    """A switch_model goal for `role`, and its run."""
    ctx = FakeCtx(fake_peppygen.switch_model, role=role.value, model=name)
    return ctx, switch_model.run(brain, ctx)


async def switched(brain, role: Role, name: str) -> FakeCtx:
    ctx, run = switch(brain, role, name)
    await run
    return ctx


def scan_goal() -> FakeCtx:
    return FakeCtx(fake_peppygen.scan_items, timeout_s=0.0)


def grab_goal(**overrides) -> FakeCtx:
    goal = dict(gripper_name="left_gripper", item_id="", position=[0.5, 0.1, 0.7], orientation=None, max_effort=0.0)
    goal.update(overrides)
    return FakeCtx(fake_peppygen.grab_item, **goal)


async def test_the_parameters_name_each_roles_model_at_the_start():
    manipulator = FakeManipulator()
    source = FakeSource({Role.PERCEPTION: {"old": FakeDetector(), "new": FakeDetector()}, Role.MANIPULATION: {"fake": manipulator}})
    brain = await make_brain(source=source, perception_model="old", manipulation_model="fake")
    assert source.started == [(Role.PERCEPTION, "old"), (Role.MANIPULATION, "fake")]
    assert [brain.models.current_name(role) for role in brain.models.roles()] == ["old", "fake"]
    assert brain.models.current(Role.MANIPULATION) is manipulator
    # The manipulator is handed the robot and its measured limbs before it
    # runs.
    assert manipulator.robot is brain.robot and manipulator.limb_states is brain.limb_states
    assert source.prepared == []


async def test_a_starting_model_the_role_does_not_offer_stops_the_node_naming_it():
    source = two_detectors(FakeDetector(), FakeDetector())
    with pytest.raises(ValueError, match="^perception_model: 'sam9' is not a perception model here; see list_models$"):
        await make_brain(source=source, perception_model="sam9")
    with pytest.raises(ValueError, match="^manipulation_model: 'policy' is not a manipulation model here"):
        await make_brain(source=source, perception_model="old", manipulation_model="policy")


async def test_a_name_the_role_does_not_offer_is_refused_in_the_sources_words():
    brain = await brain_with(two_detectors(FakeDetector(), FakeDetector()))
    ctx = await switched(brain, Role.PERCEPTION, "sam9")
    assert failed_with(ctx, FailureKind.REJECTED, "'sam9' is not a perception model here; see list_models")
    assert brain.models.current_name(Role.PERCEPTION) == "old"


async def test_a_perception_switch_is_refused_during_a_scan_and_the_running_scan_keeps_its_model():
    old, new = GatedDetector([Box("cup", 0.9, 7, 5, 9, 7)]), FakeDetector([Box("bowl", 0.9, 7, 5, 9, 7)])
    source = two_detectors(old, new)
    brain = await brain_with(source)
    scan = scan_goal()
    running = asyncio.create_task(scan_items.run(brain, scan))
    await old.searching.wait()
    with pytest.raises(Refusal, match="^scan_items is running$") as raised:
        brain.models.check_switch(Role.PERCEPTION, "new")
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    ctx = await switched(brain, Role.PERCEPTION, "new")
    assert failed_with(ctx, FailureKind.PRECONDITION_FAILED, "scan_items is running")
    # Refused before any work: nothing was prepared.
    assert source.prepared == []
    old.gate.set()
    await asyncio.wait_for(running, 1.0)
    assert scan.completed["labels"] == ["cup"]
    assert brain.models.current_name(Role.PERCEPTION) == "old"


async def test_a_scan_that_starts_after_a_switch_uses_the_new_model():
    old, new = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]), FakeDetector([Box("bowl", 0.9, 7, 5, 9, 7)])
    brain = await brain_with(two_detectors(old, new))
    ctx = await switched(brain, Role.PERCEPTION, "new")
    assert ctx.completed == dict(role="perception", model="new", action_time=0.0)
    assert brain.models.current(Role.PERCEPTION) is new
    scan = scan_goal()
    await scan_items.run(brain, scan)
    assert scan.completed["labels"] == ["bowl"]
    assert old.phrases == [] and new.phrases == [[]]


async def test_a_grab_never_asks_the_perception_model():
    manipulator = FakeManipulator()
    source = FakeSource({Role.PERCEPTION: {"trap": TrapDetector()}, Role.MANIPULATION: {"fake": manipulator}})
    brain = await make_brain(source=source, perception_model="trap", manipulation_model="fake")
    by_pose = grab_goal()
    await grab_item.run(brain, by_pose)
    assert by_pose.completed["success"] is True
    await place_item.run(brain, FakeCtx(fake_peppygen.place_item, gripper_name="", position=[0.6, 0.2, 0.75], orientation=None))
    # The item's position comes from the brain's memory, not from a look.
    by_id = grab_goal(item_id=by_pose.completed["item_id"], position=[0.0, 0.0, 0.0])
    await grab_item.run(brain, by_id)
    assert by_id.completed["success"] is True
    assert [call[0] for call in manipulator.calls] == ["grab", "place", "grab"]


async def test_a_perception_switch_succeeds_during_a_grab():
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await brain_with(two_detectors(FakeDetector(), FakeDetector(), manipulator), manipulation_model="fake")
    grab = grab_goal()
    running = asyncio.create_task(grab_item.run(brain, grab))
    await manipulator.begun()
    ctx = await switched(brain, Role.PERCEPTION, "new")
    assert ctx.completed is not None and brain.models.current_name(Role.PERCEPTION) == "new"
    # A manipulation switch, though, waits for the grab to end.
    refused = await switched(brain, Role.MANIPULATION, "fake")
    assert failed_with(refused, FailureKind.PRECONDITION_FAILED, "grab_item is running")
    await abort.run(brain, FakeCtx(fake_peppygen.abort, reason="done"))
    await asyncio.wait_for(running, 1.0)


async def test_a_manipulation_switch_hands_the_new_manipulator_the_robot_and_the_limbs():
    first, second = FakeManipulator(), FakeManipulator()
    source = FakeSource({Role.PERCEPTION: {"none": FakeDetector()}, Role.MANIPULATION: {"first": first, "second": second}})
    brain = await make_brain(source=source, perception_model="none", manipulation_model="first")
    assert second.robot is None and second.limb_states is None
    ctx = await switched(brain, Role.MANIPULATION, "second")
    assert ctx.completed is not None
    assert second.robot is brain.robot and second.limb_states is brain.limb_states
    assert brain.models.current(Role.MANIPULATION) is second


async def test_a_second_switch_for_a_role_is_refused_while_the_first_runs():
    source = two_detectors(FakeDetector(), FakeDetector())
    source.gate = asyncio.Event()
    brain = await brain_with(source)
    first, run = switch(brain, Role.PERCEPTION, "new")
    running = asyncio.create_task(run)
    await source.preparing.wait()
    with pytest.raises(Refusal, match="^switch_model is running for perception$") as raised:
        brain.models.check_switch(Role.PERCEPTION, "old")
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    second = await switched(brain, Role.PERCEPTION, "old")
    assert failed_with(second, FailureKind.PRECONDITION_FAILED, "switch_model is running for perception")
    source.gate.set()
    await asyncio.wait_for(running, 1.0)
    assert first.completed is not None and brain.models.current_name(Role.PERCEPTION) == "new"


async def test_a_lane_that_becomes_busy_during_prepare_refuses_the_switch_at_the_set():
    old = GatedDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    source = two_detectors(old, FakeDetector())
    source.gate = asyncio.Event()
    brain = await brain_with(source)
    ctx, run = switch(brain, Role.PERCEPTION, "new")
    switching = asyncio.create_task(run)
    await source.preparing.wait()
    # A scan is admitted while the switch prepares: the switch holds no lane.
    scan = scan_goal()
    scanning = asyncio.create_task(scan_items.run(brain, scan))
    await old.searching.wait()
    source.gate.set()
    await asyncio.wait_for(switching, 1.0)
    # The check after the prepare fails the switch with the kind a refusal
    # would have given.
    assert failed_with(ctx, FailureKind.PRECONDITION_FAILED, "scan_items is running")
    assert brain.models.current(Role.PERCEPTION) is old
    old.gate.set()
    await asyncio.wait_for(scanning, 1.0)
    assert scan.completed["labels"] == ["cup"]


async def test_a_cancel_during_prepare_completes_as_cancelled_and_the_role_keeps_its_model():
    source = two_detectors(FakeDetector(), FakeDetector())
    # The prepare never ends by itself: the cancel cuts it short.
    source.gate = asyncio.Event()
    brain = await brain_with(source)
    ctx, run = switch(brain, Role.PERCEPTION, "new")
    running = asyncio.create_task(run)
    await source.preparing.wait()
    ctx.cancel()
    await asyncio.wait_for(running, 1.0)
    assert ctx.completed is None
    assert (ctx.cancelled_reason, ctx.cancelled) == ("cancelled by its caller", dict(action_time=0.0, **SWITCH_ZERO))
    assert brain.models.current_name(Role.PERCEPTION) == "old"


async def test_a_shutdown_during_prepare_ends_the_switch_cancelled_and_the_role_keeps_its_model():
    source = two_detectors(FakeDetector(), FakeDetector())
    source.gate = asyncio.Event()
    brain = await brain_with(source)
    ctx, run = switch(brain, Role.PERCEPTION, "new")
    running = asyncio.create_task(run)
    await source.preparing.wait()
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await asyncio.wait_for(running, 1.0)
    assert ctx.completed is None
    assert (ctx.cancelled_reason, ctx.cancelled) == ("the node is shutting down", dict(action_time=0.0, **SWITCH_ZERO))
    assert brain.models.current_name(Role.PERCEPTION) == "old"


@pytest.mark.parametrize("failure, kind, message", [
    (
        Refusal(FailureKind.RESOURCE_UNAVAILABLE, "no key for 'new': set GEMINI_API_KEY"),
        FailureKind.RESOURCE_UNAVAILABLE,
        "no key for 'new': set GEMINI_API_KEY",
    ),
    (
        ExpectedFailure(FailureKind.ATTEMPT_FAILED, "new failed: RateLimitError: slow down"),
        FailureKind.ATTEMPT_FAILED,
        "new failed: RateLimitError: slow down",
    ),
    (RuntimeError("the probe broke"), FailureKind.ATTEMPT_FAILED, "switch_model failed: RuntimeError('the probe broke')"),
])
async def test_a_switch_whose_prepare_fails_fails_with_its_kind_and_releases_the_switching_mark(failure, kind, message):
    source = two_detectors(FakeDetector(), FakeDetector())
    source.failure = failure
    brain = await brain_with(source)
    ctx = await switched(brain, Role.PERCEPTION, "new")
    assert failed_with(ctx, kind, message)
    assert brain.models.current_name(Role.PERCEPTION) == "old"
    source.failure = None
    again = await switched(brain, Role.PERCEPTION, "new")
    assert again.completed == dict(role="perception", model="new", action_time=0.0)


async def test_the_switching_mark_is_released_after_a_cancelled_switch():
    source = two_detectors(FakeDetector(), FakeDetector())
    source.gate = asyncio.Event()
    brain = await brain_with(source)
    ctx, run = switch(brain, Role.PERCEPTION, "new")
    running = asyncio.create_task(run)
    await source.preparing.wait()
    ctx.cancel()
    await asyncio.wait_for(running, 1.0)
    source.gate.set()
    again = await switched(brain, Role.PERCEPTION, "new")
    assert again.completed is not None and brain.models.current_name(Role.PERCEPTION) == "new"
