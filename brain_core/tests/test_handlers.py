"""Every handler against the brain's core with fake backends: the
contract rules from the decision of a goal to its end, and the line each
goal leaves in the log, with no router and no robot."""

import asyncio
import logging
import math

import pytest

from types import SimpleNamespace

from conftest import (
    ORIGIN, PLACE_MEASURED_AT, UNREAD_TARGET, FakeCtx, FakeDetector, FakeManipulator, FakeToken, FixedPlacer, Ticks, depth_frame, failed_with,
    make_brain, records_of, rgb_frame, run_goal, target, with_frames,
)
from fake_peppygen import abort as abort_action
from fake_peppygen import drop_item as drop_action
from fake_peppygen import grab_item as grab_action
from fake_peppygen import identify_item as identify_action
from fake_peppygen import place_item as place_action
from fake_peppygen import scan_items as scan_action
from fake_peppygen import LEFT_AT_REST, ROBOT_FRAME, FakeTopic, PointRecord, fake_bindings, limb_states_message

from control_core_py.geometry import Placement

from brain_core.brain import Brain, Decider
from brain_core.handlers import abort, drop_item, forget, get_state, grab_item, identify_item, place_item, scan_items
from brain_core.manipulators.scripted import CentreProposer, ScriptedManipulator
from brain_core.perception.geometry import NOT_DESCRIBED, LookupFailure
from brain_core.ports import Box, Coverage, Detection, FailureKind, ItemShape, Outcome, Refusal, Role, SearchTimeout


def scan_goal(timeout_s: float = 0.0) -> FakeCtx:
    return FakeCtx(scan_action, timeout_s=timeout_s)


def identify_goal(description: str, timeout_s: float = 0.0) -> FakeCtx:
    return FakeCtx(identify_action, description=description, timeout_s=timeout_s)


def grab_goal(position=(0.0, 0.0, 0.0), orientation=None, **overrides) -> FakeCtx:
    """A grab_item goal whose target is `position` and `orientation` in the
    robot frame at the latest time."""
    goal = dict(gripper_name="", item_id="", target=target(position, orientation), max_effort=0.0)
    goal.update(overrides)
    return FakeCtx(grab_action, **goal)


def drop_goal(gripper_name: str = "", target_item_id=None) -> FakeCtx:
    return FakeCtx(drop_action, gripper_name=gripper_name, target_item_id=target_item_id)


def place_goal(position, gripper_name: str = "", orientation=None, target_item_id=None) -> FakeCtx:
    """A place_item goal whose target is `position` and `orientation` in the
    robot frame at the latest time."""
    return FakeCtx(place_action, gripper_name=gripper_name, target=target(position, orientation), target_item_id=target_item_id)


def abort_goal(reason: str = "") -> FakeCtx:
    return FakeCtx(abort_action, reason=reason)


def state_of(brain: Brain):
    return get_state.handle(brain, None)


@pytest.fixture
def brain_log(caplog):
    """The records of the brain's core, where each goal leaves its line."""
    with records_of(caplog, "brain_core.brain") as records:
        yield records


def lines(log) -> list[str]:
    return [record.getMessage() for record in log.records]


async def test_get_state_starts_idle_with_the_configured_grippers():
    brain = await make_brain()
    response = state_of(brain)
    assert response.current_action == ""
    assert response.current_action_elapsed_s == 0.0
    assert response.gripper_names == ["left_gripper", "right_gripper"]
    assert response.holding == [False, False]
    assert response.held_item_ids == ["", ""]
    # No limb_states snapshot yet: no measured opening.
    assert response.gripper_openings == []
    brain.limb_states.add(limb_states_message(1.0, left_gripper=1.0, right_gripper=0.0))
    assert state_of(brain).gripper_openings == [1.0, 0.0]


async def test_scan_and_grab_are_refused_without_backends_but_the_rules_still_speak_first():
    brain = await make_brain()
    with pytest.raises(Refusal, match="^no perception source: perception_model is 'none'$") as raised:
        await scan_items.admit(brain, scan_goal().request().data)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    # A scan the decision let through fails with the same kind.
    ctx = scan_goal()
    await run_goal(scan_items, brain, ctx)
    assert failed_with(ctx, FailureKind.PRECONDITION_FAILED, "no perception source: perception_model is 'none'")

    ctx = grab_goal(gripper_name="claw")
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["message"] == "unknown gripper 'claw'"

    ctx = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is False
    assert ctx.completed["message"] == "no manipulation model: manipulation_model is 'none'"
    assert ctx.completed["gripper_name"] == "" and ctx.completed["item_id"] == ""

    ctx = drop_goal()
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["message"] == "no gripper holds an item"
    ctx = place_goal([0.5, 0.1, 0.7])
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["message"] == "no gripper holds an item"
    assert ctx.completed["final_point"] is None


async def test_scan_then_identify_then_grab_by_id_then_place():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7), Box("banana", 0.7, 1, 1, 3, 3)])
    manipulator = FakeManipulator()
    ticks = Ticks()
    brain = await make_brain(detector=detector, manipulator=manipulator, ticks=ticks)
    with_frames(brain, depth_m=1.0)

    ctx = scan_goal()
    await run_goal(scan_items, brain, ctx)
    assert ctx.completed["item_ids"] == ["cup_1-t0", "banana_1-t0"]
    assert ctx.completed["labels"] == ["cup", "banana"]
    # Every point in the robot frame at the capture time of the picture.
    assert (ctx.completed["frame"], ctx.completed["timestamp"]) == ("base_link", 1.0)
    assert [len(point.position) for point in ctx.completed["points"]] == [3, 3]
    assert ctx.completed["confidences"] == [0.9, 0.7]
    # Each item's box, in the picture the scan looked at.
    assert ctx.completed["regions"] == [7.0, 5.0, 9.0, 7.0, 1.0, 1.0, 3.0, 3.0]
    assert ctx.completed["camera"] == "chest"
    assert (ctx.completed["image_width"], ctx.completed["image_height"]) == (16, 12)

    ctx = identify_goal("cup")
    await run_goal(identify_item, brain, ctx)
    assert ctx.completed["item_id"] == "cup_1-t0"
    assert detector.phrases[-1] == ["cup"]
    # The item's point, and no pose: the detector resolved no turn.
    point = ctx.completed["item_point"]
    assert (point.frame, point.timestamp, len(point.position)) == ("base_link", 1.0, 3)
    assert ctx.completed["item_pose"] is None
    assert ctx.completed["region"] == [7.0, 5.0, 9.0, 7.0]
    assert ctx.completed["camera"] == "chest"
    assert (ctx.completed["image_width"], ctx.completed["image_height"]) == (16, 12)

    ctx = grab_goal(gripper_name="", item_id="cup_1-t0")
    ticks.now_ns += 3_000_000_000
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is True
    assert ctx.completed["gripper_name"] == "left_gripper"
    assert ctx.completed["item_id"] == "cup_1-t0"
    assert manipulator.calls[-1][:3] == ("grab", "cup_1-t0", "left_gripper")
    assert state_of(brain).holding == [True, False]
    assert state_of(brain).held_item_ids == ["cup_1-t0", ""]

    ctx = grab_goal(gripper_name="left_gripper", item_id="banana_1-t0")
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["message"] == "gripper 'left_gripper' already holds item 'cup_1-t0'"

    ctx = place_goal([0.6, 0.2, 0.75])
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True
    assert ctx.completed["gripper_name"] == "left_gripper" and ctx.completed["item_id"] == "cup_1-t0"
    assert ctx.completed["final_point"] == PointRecord("base_link", PLACE_MEASURED_AT, [0.6, 0.2, 0.76])
    assert state_of(brain).holding == [False, False]


async def test_a_grab_of_an_item_a_gripper_holds_is_refused_naming_the_gripper():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    manipulator = FakeManipulator()
    brain = await make_brain(detector=detector, manipulator=manipulator)
    with_frames(brain)
    await run_goal(scan_items, brain, scan_goal())
    await run_goal(grab_item, brain, grab_goal(gripper_name="left_gripper", item_id="cup_1-t0"))
    assert state_of(brain).held_item_ids == ["cup_1-t0", ""]

    # The item is in the left jaws, not on the table: neither the right
    # gripper by name nor a free gripper by choice may be sent for it.
    for gripper_name in ("right_gripper", ""):
        ctx = grab_goal(gripper_name=gripper_name, item_id="cup_1-t0")
        await run_goal(grab_item, brain, ctx)
        assert ctx.completed["success"] is False
        assert ctx.completed["message"] == "item 'cup_1-t0' is held by gripper 'left_gripper'"
    assert state_of(brain).held_item_ids == ["cup_1-t0", ""]
    assert len(manipulator.calls) == 1


async def test_a_grab_by_pose_mints_an_id_and_a_failed_grab_changes_nothing():
    manipulator = FakeManipulator(fail="nothing grasped")
    brain = await make_brain(manipulator=manipulator)
    ctx = grab_goal(gripper_name="right_gripper", position=[0.5, -0.1, 0.7])
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is False and ctx.completed["message"] == "nothing grasped"
    assert state_of(brain).holding == [False, False]
    assert "item_1-t0" in brain.state.items

    manipulator.fail = ""
    ctx = grab_goal(gripper_name="right_gripper", item_id="item_1-t0")
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is True and ctx.completed["item_id"] == "item_1-t0"

    ctx = drop_goal()
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["success"] is True
    assert ctx.completed["gripper_name"] == "right_gripper" and ctx.completed["item_id"] == "item_1-t0"
    assert state_of(brain).holding == [False, False]


# The target of a grab_item or a place_item goal: parsed and placed in the
# robot frame when the goal is decided, and refused there with `rejected`
# or the failure kind of the lookup.

# The robot's tree stands the target's frame 1 m ahead of the robot frame,
# turned a quarter turn about +Z.
QUARTER_TURN = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
AHEAD = Placement((1.0, 0.0, 0.0), QUARTER_TURN)


async def test_a_grab_by_pose_runs_at_the_target_its_decision_placed_in_the_robot_frame():
    manipulator = FakeManipulator()
    placer = FixedPlacer(AHEAD)
    brain = await make_brain(manipulator=manipulator, placer=placer)
    ctx = grab_goal(target=target((0.2, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), frame="odom", timestamp=4.5))
    goal = await grab_item.admit(brain, ctx.request().data)
    [stamped] = placer.targets
    assert (stamped.frame, stamped.timestamp, stamped.value.position.position) == ("odom", 4.5, (0.2, 0.0, 0.1))
    assert goal.target.pose().position == pytest.approx((1.0, 0.2, 0.1))
    assert goal.target.pose().orientation == pytest.approx(QUARTER_TURN)
    await grab_item.run(brain, ctx, goal)
    assert ctx.completed["success"] is True
    item = brain.state.items[ctx.completed["item_id"]]
    assert item.position == pytest.approx((1.0, 0.2, 0.1)) and item.position_is_grasp_point


async def test_a_target_with_no_frame_is_placed_from_the_robot_frame_at_its_timestamp():
    placer = FixedPlacer(AHEAD)
    brain = await make_brain(manipulator=FakeManipulator(), placer=placer)
    record = target((0.2, 0.0, 0.1), frame=None, timestamp=4.5)
    for module, admit, ctx in (
        (grab_action, grab_item.admit, grab_goal(target=record)),
        (place_action, place_item.admit, FakeCtx(place_action, gripper_name="", target=record, target_item_id=None)),
    ):
        decider = Decider(module, brain, admit)
        assert (await decider.decide(ctx.request())).refusal is None
        assert decider.take().target.pose().position == pytest.approx((1.0, 0.2, 0.1))
    assert [(stamped.frame, stamped.timestamp, stamped.value.position.position) for stamped in placer.targets] == [
        (ROBOT_FRAME, 4.5, (0.2, 0.0, 0.1)),
    ] * 2


async def test_a_place_by_pose_runs_at_the_target_its_decision_placed_and_no_orientation_leaves_the_turn():
    manipulator = FakeManipulator()
    brain = await make_brain(manipulator=manipulator, placer=FixedPlacer(AHEAD))
    [cup] = brain.state.remember([Detection("cup", (0.3, 0.2, 0.72), 0.9)], brain.now(), coverage=Coverage(every_label=True))
    brain.state.set_held(brain.state.grippers["left_gripper"], cup.item_id)
    ctx = place_goal([0.2, 0.0, 0.1])
    await run_goal(place_item, brain, ctx)
    [(_, gripper, to)] = manipulator.calls
    assert gripper == "left_gripper" and to == pytest.approx((1.0, 0.2, 0.1))
    assert ctx.completed["final_point"].position == pytest.approx([1.0, 0.2, 0.11])
    assert brain.state.items[cup.item_id].orientation is None


async def test_a_place_whose_manipulator_measured_no_grasp_point_gives_no_final_point_and_the_item_takes_the_target():
    class Unmeasured(FakeManipulator):
        async def place(self, gripper, to, cancel):
            self.calls.append(("place", gripper.name, to.position))
            return Outcome(True)

    manipulator = Unmeasured()
    brain = await make_brain(manipulator=manipulator)
    [cup] = brain.state.remember([Detection("cup", (0.3, 0.2, 0.72), 0.9)], brain.now(), coverage=Coverage(every_label=True))
    brain.state.set_held(brain.state.grippers["left_gripper"], cup.item_id)
    ctx = place_goal([0.6, 0.2, 0.75])
    await run_goal(place_item, brain, ctx)
    assert (ctx.completed["success"], ctx.completed["final_point"]) == (True, None)
    assert brain.state.items[cup.item_id].position == (0.6, 0.2, 0.75)


@pytest.mark.parametrize("record, words", [
    (target((0.5, 0.1, 0.7), frame=""), "the target: the frame is empty: name a frame of the robot's tree, such as the robot frame that "
     "sensor_mounts' describe_sensors names"),
    (
        target((0.5, 0.1, 0.7), (0.0, 0.0, 0.0, 2.0)),
        "the target: the orientation [0, 0, 0, 2] has a squared norm of 4, 0.01 or more from 1: "
        "send a unit quaternion [x, y, z, w], such as [0, 0, 0, 1]",
    ),
    (target((0.5, 0.1, 0.7), (0.0, 0.0, 1.0)), "the target: the orientation has 3 entries: send 4, a unit quaternion [x, y, z, w]"),
    (target((0.5, math.nan, 0.7)), "the target: a position must be finite: got x 0.5, y NaN, z 0.7; give a finite number of meters on each axis"),
])
async def test_a_target_no_contract_takes_refuses_the_goal_with_rejected_before_anything_moves(record, words):
    manipulator = FakeManipulator()
    brain = await make_brain(manipulator=manipulator)
    for module, request in (
        (grab_action, grab_goal(gripper_name="left_gripper", target=record).request()),
        (place_action, place_goal([0.0, 0.0, 0.0]).request()),
    ):
        if module is place_action:
            request.data.target = record
        decision = await Decider(module, brain, grab_item.admit if module is grab_action else place_item.admit).decide(request)
        assert decision.refusal == module.Failure("rejected", words)
    assert manipulator.calls == [] and brain.placer.targets == []


async def test_a_target_whose_timestamp_no_stamp_holds_is_rejected_whatever_its_frame_before_the_robot_names_its_frame():
    brain = await make_brain(manipulator=FakeManipulator(), placer=FixedPlacer(None))
    words = (
        "the target: the timestamp -1.5 s from 1970-01-01T00:00:00Z lies outside the times a stamp holds: "
        "send one from 0 s to 18446744073.709551615 s after it"
    )
    for frame in (ROBOT_FRAME, None):
        record = target((0.5, 0.1, 0.7), frame=frame, timestamp=-1.5)
        for module, admit, ctx in (
            (grab_action, grab_item.admit, grab_goal(target=record)),
            (place_action, place_item.admit, FakeCtx(place_action, gripper_name="", target=record, target_item_id=None)),
        ):
            decision = await Decider(module, brain, admit).decide(ctx.request())
            assert decision.refusal == module.Failure("rejected", words), frame
    assert brain.placer.targets == []


@pytest.mark.parametrize("kind", ["unknown_frame", "no_chain", "extrapolation", "broken_tree"])
async def test_a_target_the_robots_tree_cannot_place_refuses_the_goal_with_the_lookups_kind(kind):
    refusal = LookupFailure(FailureKind(kind), f"the target: the cause of {kind}")
    brain = await make_brain(manipulator=FakeManipulator(), placer=FixedPlacer(refusal=refusal))
    for module, admit, ctx in (
        (grab_action, grab_item.admit, grab_goal(target=target((0.5, 0.1, 0.7), frame="map"))),
        (place_action, place_item.admit, place_goal([0.5, 0.1, 0.7])),
    ):
        decision = await Decider(module, brain, admit).decide(ctx.request())
        assert decision.refusal == module.Failure(kind, f"the target: the cause of {kind}")


@pytest.mark.parametrize("frame", [ROBOT_FRAME, None])
async def test_a_target_the_brain_cannot_place_before_the_robot_names_its_frame_is_refused_in_the_result(frame):
    manipulator = FakeManipulator()
    brain = await make_brain(manipulator=manipulator, placer=FixedPlacer(None))
    [cup] = brain.state.remember([Detection("cup", (0.3, 0.2, 0.72), 0.9)], brain.now(), coverage=Coverage(every_label=True))
    brain.state.set_held(brain.state.grippers["right_gripper"], cup.item_id)
    for module, admit, run, ctx in (
        (grab_action, grab_item.admit, grab_item.run, grab_goal(gripper_name="left_gripper", target=target((0.5, 0.1, 0.7), frame=frame))),
        (place_action, place_item.admit, place_item.run, FakeCtx(place_action, gripper_name="", target=target((0.6, 0.2, 0.75), frame=frame), target_item_id=None)),
    ):
        decider = Decider(module, brain, admit)
        assert (await decider.decide(ctx.request())).refusal is None
        await run(brain, ctx, decider.take())
        assert (ctx.completed["success"], ctx.completed["message"]) == (False, NOT_DESCRIBED)
    assert manipulator.calls == [] and state_of(brain).held_item_ids == ["", cup.item_id]


async def test_a_grab_that_names_its_item_sends_a_target_that_is_neither_parsed_nor_placed():
    brain = await make_brain(manipulator=FakeManipulator())
    goal = await grab_item.admit(brain, grab_goal(item_id="cup_1-t0", target=UNREAD_TARGET).request().data)
    assert (goal.item_id, goal.target) == ("cup_1-t0", None)
    assert brain.placer.targets == []


async def test_a_dropped_item_is_forgotten_and_a_drop_that_does_not_end_changes_nothing():
    manipulator = FakeManipulator()
    brain = await make_brain(manipulator=manipulator)
    grab = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    await run_goal(grab_item, brain, grab)
    item_id = grab.completed["item_id"]

    # A drop that fails, or that an abort stops, leaves the item held and known.
    manipulator.fail = "the jaws did not open"
    failed = drop_goal()
    await run_goal(drop_item, brain, failed)
    assert failed.completed["success"] is False
    manipulator.fail, manipulator.wait_for_stop = "", True
    stopped = drop_goal()
    running = asyncio.create_task(run_goal(drop_item, brain, stopped))
    # The grab and the failed drop already began a sequence each, so wait
    # for this drop to hold the lane rather than on the manipulator.
    while brain.sequencer.running(Role.MANIPULATION) is None:
        await asyncio.sleep(0)
    await run_goal(abort, brain, abort_goal("operator"))
    await running
    assert stopped.completed["message"] == "aborted: operator"
    assert state_of(brain).held_item_ids == [item_id, ""] and item_id in brain.state.items

    manipulator.wait_for_stop = False
    dropped = drop_goal()
    await run_goal(drop_item, brain, dropped)
    assert (dropped.completed["success"], dropped.completed["item_id"]) == (True, item_id)
    assert state_of(brain).held_item_ids == ["", ""] and item_id not in brain.state.items
    again = grab_goal(gripper_name="left_gripper", item_id=item_id)
    await run_goal(grab_item, brain, again)
    assert again.completed["message"] == f"unknown item '{item_id}': scan or identify it again"
    assert [call[0] for call in manipulator.calls] == ["grab", "drop", "drop", "drop"]


async def test_a_search_under_other_words_returns_the_scans_id():
    detector = FakeDetector([Box("mustard bottle", 0.9, 7, 5, 9, 7)])
    brain = await make_brain(detector=detector)
    with_frames(brain)
    scan = scan_goal()
    await run_goal(scan_items, brain, scan)
    assert scan.completed["item_ids"] == ["mustard_bottle_1-t0"]

    # The words route names the same box with the caller's description.
    detector.boxes = [Box("yellow bottle", 0.96, 7, 5, 9, 7)]
    ctx = identify_goal("yellow bottle")
    await run_goal(identify_item, brain, ctx)
    assert ctx.completed["item_id"] == "mustard_bottle_1-t0"
    assert ctx.completed["label"] == "mustard bottle"
    assert ctx.completed["confidence"] == 0.96


async def test_a_search_for_an_item_in_another_returns_its_own_id():
    detector = FakeDetector([Box("the bowl", 0.0, 6, 4, 10, 8)])
    brain = await make_brain(detector=detector)
    with_frames(brain)
    bowl = identify_goal("the bowl")
    await run_goal(identify_item, brain, bowl)
    assert bowl.completed["item_id"] == "the_bowl_1-t0"

    # The mug stands in the bowl: its box has the same centre, so the same place.
    detector.boxes = [Box("the mug", 0.0, 7, 5, 9, 7)]
    mug = identify_goal("the mug")
    await run_goal(identify_item, brain, mug)
    assert (mug.completed["item_id"], mug.completed["label"]) == ("the_mug_1-t0", "the mug")


async def test_a_description_names_whole_words_only():
    # A scan under the vocabulary returns general names; a description
    # that shares letters with one of them, or only stopwords, names none.
    detector = FakeDetector([Box("thermos", 0.9, 7, 5, 9, 7), Box("candle", 0.8, 1, 1, 3, 3), Box("feather", 0.7, 4, 4, 6, 6)])
    brain = await make_brain(detector=detector)
    with_frames(brain)
    for description in ("the coffee can please", "can", "the mug"):
        ctx = identify_goal(description)
        await run_goal(identify_item, brain, ctx)
        assert failed_with(ctx, FailureKind.NOT_FOUND, f"no item matches '{description}'")
    ctx = identify_goal("the candle please")
    await run_goal(identify_item, brain, ctx)
    assert ctx.completed["label"] == "candle"


async def test_a_scan_keeps_an_item_only_a_description_finds():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)], coverage=Coverage(frozenset({"cup"})))
    brain = await make_brain(detector=detector, manipulator=FakeManipulator())
    with_frames(brain)
    await run_goal(scan_items, brain, scan_goal())

    detector.boxes = [Box("blue ball", 0.8, 1, 1, 3, 3)]
    ctx = identify_goal("blue ball")
    await run_goal(identify_item, brain, ctx)
    assert ctx.completed["item_id"] == "blue_ball_1-t0"

    # The scan cannot name a blue ball, so not seeing it is no news.
    detector.boxes = [Box("cup", 0.9, 7, 5, 9, 7)]
    scan = scan_goal()
    await run_goal(scan_items, brain, scan)
    assert scan.completed["item_ids"] == ["cup_1-t0"]
    grab = grab_goal(gripper_name="", item_id="blue_ball_1-t0")
    await run_goal(grab_item, brain, grab)
    assert grab.completed["success"] is True and grab.completed["item_id"] == "blue_ball_1-t0"


async def test_a_placed_item_keeps_its_id_in_the_next_scan():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    brain = await make_brain(detector=detector, manipulator=FakeManipulator())
    with_frames(brain, depth_m=1.0)
    await run_goal(scan_items, brain, scan_goal())
    await run_goal(grab_item, brain, grab_goal(gripper_name="", item_id="cup_1-t0"))

    # Put it where a box in the image's corner points, far from where it was.
    corner = Box("cup", 0.9, 1, 1, 3, 3)
    target = brain.perceiver.camera.with_pose(ORIGIN).deproject(*corner.centre, 1.0, 16, 12)
    ctx = place_goal(list(target))
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True and ctx.completed["item_id"] == "cup_1-t0"

    detector.boxes = [corner]
    scan = scan_goal()
    await run_goal(scan_items, brain, scan)
    assert scan.completed["item_ids"] == ["cup_1-t0"]


@pytest.mark.parametrize("ends", ["place", "drop"])
async def test_a_place_or_a_drop_that_a_forget_runs_during_names_the_item_it_started_with(ends):
    release = asyncio.Event()
    manipulator = FakeManipulator()
    brain = await make_brain(manipulator=manipulator)
    grab = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    await run_goal(grab_item, brain, grab)
    item_id = grab.completed["item_id"]
    # The grab reached the manipulator: take that mark, so the next one is
    # the place's or the drop's.
    await manipulator.begun()

    manipulator.release = release
    ctx = place_goal([0.6, 0.0, 0.75]) if ends == "place" else drop_goal()
    running = asyncio.create_task(run_goal(place_item if ends == "place" else drop_item, brain, ctx))
    await manipulator.begun()
    assert forget.handle(brain, None).item_ids == [item_id]
    release.set()
    await running
    assert (ctx.completed["success"], ctx.completed["gripper_name"], ctx.completed["item_id"]) == (True, "left_gripper", item_id)
    # The forget stands: no gripper holds, and the item stays unknown.
    assert state_of(brain).held_item_ids == ["", ""] and brain.state.items == {}


async def test_identify_that_finds_nothing_fails_with_not_found():
    brain = await make_brain(detector=FakeDetector([Box("bowl", 0.9, 7, 5, 9, 7)]))
    with_frames(brain)
    ctx = identify_goal("cup")
    await run_goal(identify_item, brain, ctx)
    assert failed_with(ctx, FailureKind.NOT_FOUND, "no item matches 'cup'")
    # No item at all, for an empty description, is not found either.
    brain.perceiver.frames.add_color(rgb_frame(16, 12, capture_index=2))
    brain.perceiver.frames.add_depth(depth_frame(1.0, 16, 12, capture_index=2))
    brain.models.current(Role.PERCEPTION).boxes = []
    ctx = identify_goal("")
    await run_goal(identify_item, brain, ctx)
    assert failed_with(ctx, FailureKind.NOT_FOUND, "no item in view")


async def test_a_search_that_stops_at_its_deadline_fails_with_timeout_and_frees_the_lane():
    class TimingOut(FakeDetector):
        async def detect(self, image, phrases, deadline):
            raise SearchTimeout(deadline.budget_s)

    brain = await make_brain(detector=TimingOut([Box("cup", 0.9, 7, 5, 9, 7)]))
    with_frames(brain)
    ctx = scan_goal(timeout_s=0.5)
    await run_goal(scan_items, brain, ctx)
    assert failed_with(ctx, FailureKind.TIMEOUT, "the search did not finish within 0.5 s")
    assert brain.sequencer.running("perception") is None


async def test_a_search_is_refused_at_once_while_another_runs_or_without_a_source():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    brain = await make_brain(detector=detector)
    for admit, goal in ((scan_items.admit, scan_goal()), (identify_item.admit, identify_goal("cup"))):
        with pytest.raises(Refusal, match="^no perception source: no camera frame received$") as raised:
            await admit(brain, goal.request().data)
        assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    with_frames(brain)
    await scan_items.admit(brain, scan_goal().request().data)
    await identify_item.admit(brain, identify_goal("cup").request().data)
    running = brain.sequencer.start(Role.PERCEPTION, "scan_items", 0)
    for admit, goal in ((scan_items.admit, scan_goal()), (identify_item.admit, identify_goal("cup"))):
        with pytest.raises(Refusal, match="^scan_items is running$") as raised:
            await admit(brain, goal.request().data)
        assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    brain.sequencer.finish(running)
    # A search beside a manipulation is let through.
    brain.sequencer.start(Role.MANIPULATION, "grab_item", 0)
    await scan_items.admit(brain, scan_goal().request().data)
    assert detector.phrases == []


class GatedDetector(FakeDetector):
    """Answers once the test opens its gate; `searching` resolves once a
    search has begun."""

    def __init__(self, boxes) -> None:
        super().__init__(boxes)
        self.gate = asyncio.Event()
        self.searching = asyncio.Event()

    async def detect(self, image, phrases, deadline):
        self.searching.set()
        await self.gate.wait()
        return await super().detect(image, phrases, deadline)


async def test_a_callers_cancel_ends_a_search_cancelled_with_empty_fields():
    detector = GatedDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    brain = await make_brain(detector=detector)
    with_frames(brain)
    ctx = identify_goal("cup")
    running = asyncio.create_task(run_goal(identify_item, brain, ctx))
    await detector.searching.wait()
    ctx.cancel("sender_gone")
    # The search is awaited until the detector returns; what it found is
    # not reported.
    detector.gate.set()
    await asyncio.wait_for(running, 1.0)
    assert ctx.cancelled_reason == "its sender is gone"
    assert ctx.cancelled == dict(identify_item.ZERO, action_time=0.0)
    assert ctx.cancelled["item_point"] is None
    assert brain.state.items == {} and brain.sequencer.running(Role.PERCEPTION) is None


async def test_a_callers_cancel_ends_a_scan_cancelled_with_empty_arrays_in_the_robot_frame_at_the_time_of_the_cancel():
    detector = GatedDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    ticks = Ticks()
    brain = await make_brain(detector=detector, ticks=ticks)
    with_frames(brain)
    ctx = scan_goal()
    running = asyncio.create_task(run_goal(scan_items, brain, ctx))
    await detector.searching.wait()
    ticks.now_ns += 2_500_000_000
    ctx.cancel("sender_gone")
    detector.gate.set()
    await asyncio.wait_for(running, 1.0)
    assert ctx.cancelled_reason == "its sender is gone"
    assert ctx.cancelled == dict(scan_items.ZERO, frame=ROBOT_FRAME, timestamp=3.5, action_time=2.5)
    assert brain.state.items == {}


async def test_one_manipulation_at_a_time_and_abort_stops_it_without_touching_holding():
    manipulator = FakeManipulator(wait_for_stop=True)
    ticks = Ticks()
    brain = await make_brain(manipulator=manipulator, ticks=ticks)
    first = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    running = asyncio.create_task(run_goal(grab_item, brain, first))
    await manipulator.begun()
    ticks.now_ns += 2_000_000_000
    assert state_of(brain).current_action == "grab_item"
    assert state_of(brain).current_action_elapsed_s == 2.0

    second = grab_goal(gripper_name="right_gripper", position=[0.5, -0.1, 0.7])
    await run_goal(grab_item, brain, second)
    assert second.completed["message"] == "grab_item is running"

    stop = abort_goal("operator pressed stop")
    await run_goal(abort, brain, stop)
    await running
    assert stop.completed["success"] is True
    assert stop.completed["aborted_action"] == "grab_item"
    # item_manipulation keeps its abort: the running goal completes with
    # success false, not cancelled.
    assert first.cancelled is None
    assert first.completed["success"] is False
    assert first.completed["message"] == "aborted: operator pressed stop"
    assert manipulator.stopped == 1
    assert state_of(brain).holding == [False, False]
    assert state_of(brain).current_action == ""

    idle = abort_goal()
    await run_goal(abort, brain, idle)
    assert idle.completed["success"] is True
    assert idle.completed["aborted_action"] == ""
    assert idle.completed["message"] == "no sequence was running"


async def test_a_callers_cancel_completes_the_goal_as_cancelled():
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(manipulator=manipulator)
    ctx = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    running = asyncio.create_task(run_goal(grab_item, brain, ctx))
    await manipulator.begun()
    ctx.cancel()
    await asyncio.wait_for(running, 1.0)
    assert ctx.completed is None
    assert ctx.cancelled_reason == "cancelled by its caller"
    assert ctx.cancelled["success"] is False
    assert ctx.cancelled["message"] == "cancelled by its caller"
    assert manipulator.stopped == 1
    assert state_of(brain).current_action == ""


async def test_a_search_may_run_beside_a_manipulation():
    detector = FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)])
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(detector=detector, manipulator=manipulator)
    with_frames(brain)
    grab = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    running = asyncio.create_task(run_goal(grab_item, brain, grab))
    await manipulator.begun()
    scan = scan_goal()
    await run_goal(scan_items, brain, scan)
    assert scan.completed["labels"] == ["cup"]
    assert state_of(brain).current_action == "grab_item"
    await run_goal(abort, brain, abort_goal())
    await running


async def test_shutdown_stops_the_running_sequence_as_aborted():
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(manipulator=manipulator)
    ctx = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    running = asyncio.create_task(run_goal(grab_item, brain, ctx))
    await manipulator.begun()
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await running
    assert ctx.completed["success"] is False
    assert ctx.completed["message"] == "aborted: the node is shutting down"
    assert manipulator.stopped == 1
    assert state_of(brain).current_action == ""
    # Idle lanes make a shutdown a no-op.
    await asyncio.wait_for(brain.shutdown(), 1.0)


async def test_a_goal_that_arrives_once_the_shutdown_began_is_refused():
    manipulator = FakeManipulator(wait_for_stop=True)
    brain = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]), manipulator=manipulator)
    with_frames(brain)
    running = asyncio.create_task(run_goal(grab_item, brain, grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])))
    await manipulator.begun()
    await asyncio.wait_for(brain.shutdown(), 1.0)
    await running
    # The lanes are free again, and still every goal is refused: a grab in
    # its result, a scan at its decision, or failed once it runs.
    grab = grab_goal(gripper_name="right_gripper", position=[0.5, -0.1, 0.7])
    await run_goal(grab_item, brain, grab)
    assert (grab.completed["success"], grab.completed["message"]) == (False, "the node is shutting down")
    with pytest.raises(Refusal, match="^the node is shutting down$") as raised:
        await scan_items.admit(brain, scan_goal().request().data)
    assert raised.value.kind is FailureKind.PRECONDITION_FAILED
    scan = scan_goal()
    await run_goal(scan_items, brain, scan)
    assert failed_with(scan, FailureKind.PRECONDITION_FAILED, "the node is shutting down")
    assert len(manipulator.calls) == 1


async def test_a_search_logs_the_frame_it_looked_at_each_box_and_the_item_it_returned(brain_log):
    ticks = Ticks()

    class Slow(FakeDetector):
        """Takes 0.84 s of the brain's clock to answer."""

        async def detect(self, image, phrases, deadline):
            ticks.now_ns += 840_000_000
            return await super().detect(image, phrases, deadline)

    brain = await make_brain(detector=Slow([Box("cup", 0.9, 7, 5, 9, 7), Box("cup", 0.85, 7, 5, 9, 7.2)]), ticks=ticks)
    with_frames(brain)
    # The frames were taken at 1.0 s of the brain's clock.
    ticks.now_ns = 1_250_000_000
    await run_goal(identify_item, brain, identify_goal("cup"))
    await run_goal(identify_item, brain, identify_goal("bowl"))
    await run_goal(scan_items, brain, scan_goal())
    boxes = "2 boxes: cup 0.90, cup 0.85 (duplicate)"
    assert lines(brain_log) == [
        f"identify_item 'cup' succeeded after 0.84 s; frame 1 taken 0.25 s before the search, {boxes}; item cup_1-t0",
        f"identify_item 'bowl' failed after 0.84 s: no item matches 'bowl'; frame 1 taken 1.09 s before the search, {boxes}",
        f"scan_items succeeded after 0.84 s; frame 1 taken 1.93 s before the search, {boxes}",
    ]
    # A search that found nothing failed, without a traceback.
    assert [record.levelno for record in brain_log.records] == [logging.INFO, logging.WARNING, logging.INFO]
    assert all(record.exc_info is None for record in brain_log.records)


async def test_a_search_that_found_nothing_logs_whether_the_detector_boxed_anything(brain_log):
    no_depth = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]))
    with_frames(no_depth, depth_m=0.0)
    await run_goal(identify_item, no_depth, identify_goal("cup"))
    no_box = await make_brain(detector=FakeDetector())
    with_frames(no_box)
    await run_goal(identify_item, no_box, identify_goal(""))
    assert lines(brain_log) == [
        "identify_item 'cup' failed after 0.00 s: no item matches 'cup'; frame 1 taken 0.00 s before the search, 1 box: cup 0.90 (no depth)",
        "identify_item '' failed after 0.00 s: no item in view; frame 1 taken 0.00 s before the search, no box",
    ]


async def test_a_search_refused_before_it_looked_logs_the_reason_alone(brain_log):
    brain = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]))
    await run_goal(identify_item, brain, identify_goal("  cup "))
    assert lines(brain_log) == ["identify_item 'cup' refused after 0.00 s: no perception source: no camera frame received"]


async def test_a_search_that_failed_logs_its_traceback(brain_log):
    class Broken(FakeDetector):
        async def detect(self, image, phrases, deadline):
            raise RuntimeError("the GPU is gone")

    brain = await make_brain(detector=Broken())
    with_frames(brain)
    ctx = scan_goal()
    await run_goal(scan_items, brain, ctx)
    assert failed_with(ctx, FailureKind.ATTEMPT_FAILED, "scan_items failed: RuntimeError('the GPU is gone')")
    [record] = brain_log.records
    assert record.levelno == logging.ERROR
    assert record.getMessage() == "scan_items failed after 0.00 s: RuntimeError('the GPU is gone')"
    assert "RuntimeError: the GPU is gone" in brain_log.text


async def test_a_manipulation_goal_logs_how_it_ended(brain_log):
    manipulator = FakeManipulator(wait_for_stop=True)
    ticks = Ticks()
    brain = await make_brain(manipulator=manipulator, ticks=ticks)
    first = grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7])
    running = asyncio.create_task(run_goal(grab_item, brain, first))
    await manipulator.begun()
    ticks.now_ns += 2_000_000_000
    await run_goal(grab_item, brain, grab_goal(gripper_name="right_gripper", position=[0.5, -0.1, 0.7]))
    await run_goal(abort, brain, abort_goal("operator pressed stop"))
    await running
    manipulator.wait_for_stop = False
    await run_goal(grab_item, brain, grab_goal(gripper_name="left_gripper", position=[0.5, 0.1, 0.7]))
    assert lines(brain_log) == [
        "grab_item refused after 0.00 s: grab_item is running",
        "grab_item stopped after 2.00 s: aborted: operator pressed stop",
        "grab_item succeeded after 0.00 s",
    ]


WAIT = 10.0


class LimbStream:
    """The fake backbone's limb_states stream: until `stopped`, a snapshot
    with a later stamp each turn of the loop, the left grasp point where the
    last landed move put it, the left gripper at `left_opening` and the
    right one open."""

    def __init__(self, topic: FakeTopic) -> None:
        self.topic = topic
        self.left_arm = LEFT_AT_REST
        self.left_opening = 1.0
        self.stamp = 1.0
        self.stopped = asyncio.Event()

    async def run(self) -> None:
        while not self.stopped.is_set():
            self.topic.push(limb_states_message(self.stamp, left_arm=self.left_arm, left_gripper=self.left_opening, right_gripper=1.0))
            self.stamp += 0.01
            await asyncio.sleep(0.005)


async def test_a_scripted_grab_through_the_backbone_sets_the_held_flag():
    """The real handler, the scripted manipulator and the robot proxy over
    the fake limb_motion clients, with the limb follower reading the fake
    limb_states topic: the arm is where the last landed move put it."""
    topic, token = FakeTopic(), FakeToken()
    bindings = fake_bindings(limb_states=topic)
    brain = await make_brain(manipulator=ScriptedManipulator(CentreProposer()), bindings=bindings)
    loops = brain.background(token)
    limbs = LimbStream(topic)
    streaming = asyncio.create_task(limbs.run())
    await asyncio.wait_for(brain.limb_states.newer_than(0.0, WAIT), WAIT)
    [cup] = brain.state.remember([Detection("cup", (0.5, 0.1, 0.72), 0.9)], brain.now(), coverage=Coverage(every_label=True))
    for _ in range(3):
        bindings.check_arm_move.enqueue(SimpleNamespace(reachable=True, reason="", duration_s=1.0))

    ctx = grab_goal(item_id=cup.item_id, max_effort=20.0)
    grab = asyncio.create_task(run_goal(grab_item, brain, ctx))

    def landed(goal):
        """The goal's result, the arm put on its target first."""
        target = goal.request.target
        limbs.left_arm = (tuple(target.position), tuple(target.orientation))
        final_pose = SimpleNamespace(frame="base_link", timestamp=limbs.stamp, position=target.position, orientation=target.orientation)
        return SimpleNamespace(final_pose=final_pose, ended_on_contact=False, action_time=1.0)

    opened = await asyncio.wait_for(bindings.move_gripper.next_goal(), WAIT)
    assert (opened.request.gripper_name, opened.request.opening) == ("left_gripper", 1.0)
    opened.complete(SimpleNamespace(settled=True, final_opening=1.0, action_time=0.5))
    # The cup was seen at 0.72 m: the grasp point is 2.5 cm under it, the
    # pre-grasp point 7.5 cm above that.
    for z in (0.77, 0.695):
        move = await asyncio.wait_for(bindings.move_arm.next_goal(), WAIT)
        target = move.request.target
        assert move.request.arm_name == "left_arm" and target.position == pytest.approx([0.5, 0.1, z])
        # Each move names the robot frame, with no timestamp: the latest time.
        assert (target.frame, target.timestamp, target.orientation) == ("base_link", None, [0.0, 1.0, 0.0, 0.0])
        move.complete(landed(move))
    closed = await asyncio.wait_for(bindings.move_gripper.next_goal(), WAIT)
    assert (closed.request.opening, closed.request.max_effort) == (0.0, 20.0)
    limbs.left_opening = 0.3
    closed.complete(SimpleNamespace(settled=True, final_opening=0.3, action_time=0.5))
    lift = await asyncio.wait_for(bindings.move_arm.next_goal(), WAIT)
    assert lift.request.target.position == pytest.approx([0.5, 0.1, 0.77])
    # The grab waits until the stream shows the arm at the lifted point,
    # then judges the hold from the snapshot after that one.
    lift.complete(landed(lift))
    await asyncio.wait_for(grab, WAIT)

    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert (ctx.completed["gripper_name"], ctx.completed["item_id"]) == ("left_gripper", cup.item_id)
    assert state_of(brain).holding == [True, False]
    assert state_of(brain).held_item_ids == [cup.item_id, ""]
    assert state_of(brain).gripper_openings == [0.3, 1.0]
    assert [request.arm_name for request in bindings.check_arm_move.polled] == ["left_arm"] * 3
    # The cup slips out of the jaws later: the held flag follows results and
    # stays set, the measured opening shows the jaws closed on nothing.
    limbs.left_opening = 0.0
    # Snapshots published before the change may still be on their way.
    while brain.limb_states.latest.opening("left_gripper") != 0.0:
        await asyncio.wait_for(brain.limb_states.newer_than(brain.limb_states.latest.timestamp, WAIT), WAIT)
    assert state_of(brain).holding == [True, False]
    assert state_of(brain).gripper_openings == [0.0, 1.0]
    limbs.stopped.set()
    await asyncio.wait_for(streaming, WAIT)
    token.cancel()
    await asyncio.wait_for(asyncio.gather(*loops), WAIT)


# A place into a target item: the target read from what the brain knows of
# it, and the three refusals of the target before any move.

BOWL_SHAPE = ItemShape((0.337, 0.000, 0.487), (0.192, 0.196, 0.088), 0.0)


async def holding_with_a_bowl(manipulator):
    """A brain whose left gripper holds the cup and which knows a bowl with
    its box, and the bowl."""
    brain = await make_brain(manipulator=manipulator)
    cup, bowl = brain.state.remember(
        [Detection("cup", (0.3, 0.2, 0.72), 0.9), Detection("wooden bowl", (0.38, -0.008, 0.461), 0.9, width_m=0.18, shape=BOWL_SHAPE)],
        brain.now(), coverage=Coverage(every_label=True),
    )
    brain.state.set_held(brain.state.grippers["left_gripper"], cup.item_id)
    return brain, cup, bowl


async def test_a_place_with_a_target_item_places_into_it_and_reads_no_position():
    manipulator = FakeManipulator()
    brain, cup, bowl = await holding_with_a_bowl(manipulator)
    # A goal that names a target item sends a target that is not read, and
    # the brain places none.
    ctx = FakeCtx(place_action, gripper_name="", target=UNREAD_TARGET, target_item_id=bowl.item_id)
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert manipulator.calls == [("place", "left_gripper", bowl.item_id)]
    assert brain.placer.targets == []
    # The fake opens 1 cm above the bowl's rim at its centre.
    final = ctx.completed["final_point"]
    assert (final.frame, final.timestamp) == ("base_link", PLACE_MEASURED_AT)
    assert final.position == pytest.approx([0.337, 0.0, 0.541])
    placed = brain.state.items[cup.item_id]
    assert placed.position == pytest.approx((0.337, 0.0, 0.541)) and placed.orientation is None
    # The bowl keeps its id and what the brain knew of it.
    assert brain.state.items[bowl.item_id].shape == BOWL_SHAPE
    assert state_of(brain).holding == [False, False]


async def test_an_empty_target_item_id_names_no_target():
    manipulator = FakeManipulator()
    brain, _, _ = await holding_with_a_bowl(manipulator)
    ctx = place_goal([0.5, 0.1, 0.7], target_item_id="")
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True and manipulator.calls == [("place", "left_gripper", (0.5, 0.1, 0.7))]


async def test_a_place_into_an_unknown_item_the_held_item_or_an_item_another_gripper_holds_is_refused_before_any_move():
    manipulator = FakeManipulator()
    brain, cup, bowl = await holding_with_a_bowl(manipulator)
    for target_item_id, words in [
        ("mug_1-t0", "unknown item 'mug_1-t0': scan or identify it again"),
        (cup.item_id, f"item '{cup.item_id}' is the item that gripper 'left_gripper' holds: it cannot be its own target"),
    ]:
        ctx = place_goal([0.0, 0.0, 0.0], target_item_id=target_item_id)
        await run_goal(place_item, brain, ctx)
        assert (ctx.completed["success"], ctx.completed["message"]) == (False, words)
    brain.state.set_held(brain.state.grippers["right_gripper"], bowl.item_id)
    ctx = place_goal([0.0, 0.0, 0.0], gripper_name="left_gripper", target_item_id=bowl.item_id)
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["message"] == f"target item '{bowl.item_id}' is held by gripper 'right_gripper'"
    assert manipulator.calls == []
    assert state_of(brain).holding == [True, True]


async def test_the_target_is_refused_before_a_brain_with_no_manipulation_model_says_so():
    brain, _, _ = await holding_with_a_bowl(None)
    ctx = place_goal([0.0, 0.0, 0.0], target_item_id="mug_1-t0")
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["message"] == "unknown item 'mug_1-t0': scan or identify it again"


def recording_the_order(brain, manipulator) -> list[str]:
    """The order of the scene calls and of the manipulator's drop."""
    order: list[str] = []
    follow, drop = brain.make_scene_follow_holds, manipulator.drop

    async def following(*args):
        order.append("scene")
        await follow(*args)

    async def dropping(*args, **kwargs):
        order.append("drop")
        return await drop(*args, **kwargs)

    brain.make_scene_follow_holds, manipulator.drop = following, dropping
    return order


async def test_a_drop_with_a_target_item_makes_the_scene_follow_the_holds_then_drops_above_it_and_forgets_the_item():
    manipulator = FakeManipulator()
    brain, cup, bowl = await holding_with_a_bowl(manipulator)
    order = recording_the_order(brain, manipulator)
    ctx = drop_goal(target_item_id=bowl.item_id)
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert manipulator.calls == [("drop", "left_gripper", bowl.item_id)]
    # The scene follows before the carry, and again after the release.
    assert order == ["scene", "drop", "scene"]
    assert cup.item_id not in brain.state.items and bowl.item_id in brain.state.items


async def test_a_drop_with_no_target_opens_where_the_arm_stands_and_the_scene_follows_after_it_alone():
    manipulator = FakeManipulator()
    brain, _, _ = await holding_with_a_bowl(manipulator)
    order = recording_the_order(brain, manipulator)
    ctx = drop_goal(target_item_id="")
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["success"] is True and manipulator.calls == [("drop", "left_gripper")]
    assert order == ["drop", "scene"]


async def test_a_drop_above_an_unknown_item_is_refused_and_keeps_the_held_item():
    manipulator = FakeManipulator()
    brain, cup, _ = await holding_with_a_bowl(manipulator)
    ctx = drop_goal(target_item_id="mug_1-t0")
    await run_goal(drop_item, brain, ctx)
    assert (ctx.completed["success"], ctx.completed["message"]) == (False, "unknown item 'mug_1-t0': scan or identify it again")
    assert manipulator.calls == [] and state_of(brain).held_item_ids == [cup.item_id, ""]
