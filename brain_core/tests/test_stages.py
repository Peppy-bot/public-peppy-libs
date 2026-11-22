"""The feedback of each goal: the sentence of each stage, the reporter of a
goal context, where the scripted manipulator and its aligner publish each
stage of a sequence among the robot's calls, and the stages each handler
publishes, none for a goal refused before it starts."""

import asyncio

import pytest
from conftest import UNREAD_TARGET, FakeCtx, FakeDetector, Reports, make_brain, records_of, run_goal, target, with_frames
from fake_peppygen import drop_item as drop_action
from fake_peppygen import grab_item as grab_action
from fake_peppygen import identify_item as identify_action
from fake_peppygen import limb_states_message
from fake_peppygen import place_item as place_action
from fake_peppygen import scan_items as scan_action
from test_planned_transit import AT_PRE, OPEN_FOR_THE_CUP, THE_RUN, start_refused
from test_planned_transit import started as started_with_planner
from test_scripted import (
    DOWN,
    GRASP,
    HELD,
    LEFT,
    LIFT,
    PLACE_ABOVE,
    PLACE_POINT,
    PLACE_Q,
    PRE,
    FakeClock,
    FakeRobot,
    cup,
    gripped,
    snapshot_after_the_lift,
    started,
)
from test_scripted_targets import BOWL, HIGH

from brain_core.handlers import drop_item, grab_item, identify_item, place_item, scan_items
from brain_core.limb_state import LimbStateStore
from brain_core.manipulators.scripted import CentreProposer, ScriptedManipulator
from brain_core.manipulators.scripted.aligning import Aligned, OutOfSteps
from brain_core.manipulators.scripted.grasp import lifted
from brain_core.manipulators.scripted.manipulator import LIFT_M, NOT_STARTED
from brain_core.ports import Box, CancelToken, Coverage, Detection, ItemShape, Outcome, Pose, Senses, TargetItem
from brain_core.robot import MoveCheck
from brain_core.stages import CarryReports, GrabReports, Report, Stage, identify_look, reporter_of, scan_look, the_item

# The answer of a model that finds the gripper aligned.
READY = "ready"


class ScriptedAligningModel:
    """An aligner that answers as an aligning model does, one scripted
    answer per step of `max_steps`: each step publishes its question, a move
    (dx, dy, in metres) publishes its correction and moves the gripper
    sideways through the manipulator, and READY ends Aligned where the
    gripper stands. Steps that run out end OutOfSteps."""

    def __init__(self, *answers, max_steps: int = 3) -> None:
        self.answers = answers
        self.max_steps = max_steps

    def start(self, senses) -> None:
        return None

    def why_unavailable(self, arm: str) -> str:
        return ""

    async def align(self, request, moves, cancel, reports):
        pre, point = request.pre, request.point
        for step, answer in enumerate(self.answers[: self.max_steps], start=1):
            await reports.align_look(step, self.max_steps)
            if answer == READY:
                return Aligned(pre, point, lifted(point, request.lift_m), request.orientation, steps=step)
            dx, dy = answer
            pre = (pre[0] + dx, pre[1] + dy, pre[2])
            point = (point[0] + dx, point[1] + dy, point[2])
            await reports.align_move(step, self.max_steps)
            await moves.move_to(request.gripper.arm, pre, request.orientation)
        return OutOfSteps(self.max_steps)


def shifted(point, dx: float):
    return (point[0] + dx, point[1], point[2])


def holding_after_the_lift(robot: FakeRobot, store: LimbStateStore) -> None:
    """The open, then the close at 0.3, and the snapshot after the lift, the
    move that follows the close, reads 0.3: the jaws hold the item. The
    pre-grasp point and the lifted point coincide, so the lift is told by
    the call before it."""
    robot.grips.extend([gripped(1.0), gripped(0.3)])

    def on_call(call: tuple) -> None:
        if call[0] == "move" and robot.calls[-2][:3] == ("grip", "left_gripper", 0.0):
            asyncio.get_running_loop().call_soon(store.add, limb_states_message(2.0, left_gripper=0.3))

    robot.on_call = on_call


# The sentences.


async def test_each_stage_of_a_grab_names_the_item_by_its_label():
    reports = Reports()
    grab = GrabReports(reports, "apple")
    await grab.plan()
    await grab.move_above()
    await grab.align_look(2, 5)
    await grab.align_move(2, 5)
    await grab.grasp()
    assert reports.reports == [
        Report(Stage.PLAN, "The robot opens its hand for the apple and finds a way to reach it"),
        Report(Stage.MOVE_ABOVE, "The robot moves its hand above the apple"),
        Report(Stage.ALIGN_LOOK, "The robot looks at the apple and decides how to correct its hand (2 of 5)", 2, 5),
        Report(Stage.ALIGN_MOVE, "The robot corrects the position of its hand above the apple (2 of 5)", 2, 5),
        Report(Stage.GRASP, "The robot lowers its hand, closes it on the apple and lifts it"),
    ]


async def test_each_stage_of_a_place_into_a_target_item_names_the_held_item_and_the_target():
    reports = Reports()
    place = CarryReports(reports, "apple", BOWL)
    await place.plan()
    await place.move_above()
    await place.align_look(1, 2)
    await place.align_move(1, 2)
    await place.set_down()
    assert [report.activity for report in reports.reports] == [
        "The robot finds a way to carry the apple above the wooden bowl",
        "The robot carries the apple above the wooden bowl",
        "The robot looks at the wooden bowl and decides where to put the apple (1 of 2)",
        "The robot corrects the position of the apple above the wooden bowl (1 of 2)",
        "The robot lowers the apple into the wooden bowl, opens its hand and moves its arm back up",
    ]
    assert reports.stages == [("plan", 0, 0), ("move_above", 0, 0), ("align_look", 1, 2), ("align_move", 1, 2), ("set_down", 0, 0)]


async def test_each_stage_of_a_place_at_a_pose_names_the_held_item_and_the_spot():
    reports = Reports()
    place = CarryReports(reports, "apple")
    await place.plan()
    await place.move_above()
    await place.align_look(2, 2)
    await place.align_move(2, 2)
    await place.set_down()
    assert [report.activity for report in reports.reports] == [
        "The robot finds a way to carry the apple above the spot where it puts it down",
        "The robot carries the apple above the spot where it puts it down",
        "The robot looks down and decides where to put the apple (2 of 2)",
        "The robot corrects the position of the apple above the spot where it puts it down (2 of 2)",
        "The robot puts the apple down, opens its hand and moves its arm back up",
    ]


async def test_the_release_of_a_drop_names_the_target_only_above_it():
    reports = Reports()
    await CarryReports(reports, "apple", BOWL).release(above_target=True)
    await CarryReports(reports, "apple", BOWL).release(above_target=False)
    await CarryReports(reports, "apple").release(above_target=False)
    assert reports.reports == [
        Report(Stage.RELEASE, "The robot opens its hand and lets the apple fall into the wooden bowl"),
        Report(Stage.RELEASE, "The robot opens its hand and lets go of the apple"),
        Report(Stage.RELEASE, "The robot opens its hand and lets go of the apple"),
    ]


@pytest.mark.parametrize("label, words", [
    ("apple", "the apple"),
    ("coffee_can", "the coffee can"),
    (" Red  Cup ", "the red cup"),
    # An item a goal addressed by pose, or one a forget dropped.
    ("", "the item"),
])
def test_an_item_is_its_label_in_lower_case_with_spaces_or_the_item(label, words):
    assert the_item(label) == words


async def test_a_target_item_with_no_label_is_the_item():
    reports = Reports()
    await CarryReports(reports, "", TargetItem("bowl_1-t0", "", (0.3, 0.0), 0.5, 0.4, None)).move_above()
    assert reports.reports == [Report(Stage.MOVE_ABOVE, "The robot carries the item above the item")]


@pytest.mark.parametrize("description, sentence", [
    ("apple", "The robot looks for the apple"),
    ("a red cup", "The robot looks for the red cup"),
    ("An apple", "The robot looks for the apple"),
    ("The coffee_can", "The robot looks for the coffee can"),
    ("", "The robot looks for an item that it can take"),
    ("   ", "The robot looks for an item that it can take"),
])
def test_the_look_of_a_search_names_what_it_looks_for(description, sentence):
    assert identify_look(description) == Report(Stage.LOOK, sentence)


def test_the_look_of_a_scan():
    assert scan_look() == Report(Stage.LOOK, "The robot looks for each item that it can see")


@pytest.mark.parametrize("number, maximum", [(0, 3), (4, 3), (1, 0), (-1, 0), (0, -1)])
def test_a_report_is_number_0_of_0_or_a_number_from_1_to_its_maximum(number, maximum):
    with pytest.raises(ValueError, match=f"got {number} of {maximum}"):
        Report(Stage.ALIGN_LOOK, "The robot looks", number, maximum)


# The reporter of a goal context.


async def test_the_reporter_of_a_goal_publishes_each_report_as_the_fields_of_the_feedback():
    ctx = FakeCtx(grab_action)
    await reporter_of(ctx)(Report.repeat(Stage.ALIGN_LOOK, "The robot looks at the cup and decides how to correct its hand", 1, 5))
    assert ctx.feedback == [
        dict(stage="align_look", activity="The robot looks at the cup and decides how to correct its hand (1 of 5)", number=1, maximum=5),
    ]


class UnpublishedCtx(FakeCtx):
    """A goal context whose feedback cannot be published: the messenger is
    gone."""

    async def publish_feedback(self, **fields) -> None:
        raise RuntimeError("the messenger is gone")


async def test_a_report_the_goal_cannot_publish_is_logged_and_the_goal_runs_to_its_end(caplog):
    brain = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]))
    with_frames(brain)
    ctx = UnpublishedCtx(scan_action, timeout_s=0.0)
    with records_of(caplog, "brain_core.stages") as log:
        await run_goal(scan_items, brain, ctx)
    assert ctx.completed["item_ids"] == ["cup_1-t0"]
    # The records of this module alone: another test can have set the level
    # of the package logger, so the brain's lines reach the same handler.
    assert [record.getMessage() for record in log.records if record.name == "brain_core.stages"] == [
        "the feedback 'The robot looks for each item that it can see' was not published: RuntimeError('the messenger is gone')"
    ]


# The stages of the scripted manipulator, in order among the robot's calls.


async def test_a_grab_reports_each_stage_as_it_starts_and_the_number_of_each_question_to_the_aligning_model():
    manipulator, robot, store = await started(aligner=ScriptedAligningModel((0.01, 0.0), READY))
    q = DOWN[0]
    aligned_pre, aligned_point = shifted(PRE, 0.01), shifted(GRASP, 0.01)
    aligned_lift = lifted(aligned_point, LIFT_M)
    holding_after_the_lift(robot, store)
    reports = Reports(robot.calls)
    assert await manipulator.grab(cup(), LEFT, 12.0, CancelToken(), GrabReports(reports, "cup")) == HELD
    assert robot.calls == [
        ("report", "plan", 0, 0),
        ("grip", "left_gripper", 1.0, 0.0),
        ("check", PRE, q), ("check", GRASP, q), ("check", LIFT, q),
        ("report", "move_above", 0, 0),
        ("move", PRE, q),
        # The first question, and the correction its answer gives.
        ("report", "align_look", 1, 3),
        ("report", "align_move", 1, 3),
        ("move", aligned_pre, q),
        # The second question: the model finds the gripper aligned.
        ("report", "align_look", 2, 3),
        ("report", "grasp", 0, 0),
        ("move", aligned_point, q),
        ("grip", "left_gripper", 0.0, 12.0),
        ("move", aligned_lift, q),
    ]


async def test_a_grab_with_no_aligner_goes_from_the_move_above_to_the_grasp():
    manipulator, robot, store = await started()
    holding_after_the_lift(robot, store)
    reports = Reports()
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken(), GrabReports(reports, "cup")) == HELD
    assert reports.stages == [("plan", 0, 0), ("move_above", 0, 0), ("grasp", 0, 0)]


async def test_the_second_path_of_a_grab_stays_in_the_plan_and_the_move_above():
    """The close of the jaws and the plan with the hand closed are in the
    plan; the run of the plan and the open at the pre-grasp point in the move
    above."""
    manipulator, robot, store = await started_with_planner(jaws=0.0)
    robot.plans.append(start_refused())
    robot.grips.extend([gripped(OPEN_FOR_THE_CUP), gripped(0.0), gripped(OPEN_FOR_THE_CUP), gripped(0.3)])
    snapshot_after_the_lift(robot, store, limb_states_message(2.0, left_gripper=0.3))
    reports = Reports(robot.calls)
    assert await manipulator.grab(cup(width_m=0.06), LEFT, 12.0, CancelToken(), GrabReports(reports, "cup")) == HELD
    q = DOWN[0]
    assert robot.calls[robot.calls.index(("report", "plan", 0, 0)):] == [
        ("report", "plan", 0, 0),
        ("grip", "left_gripper", OPEN_FOR_THE_CUP, 0.0),
        ("describe",),
        ("add", "brain/cup_1-t0", cup().position, "sphere", (0.05,)),
        ("allow", "brain/cup_1-t0", robot.described.arms[0].gripper_links),
        ("plan", PRE, q),
        ("grip", "left_gripper", 0.0, 0.0),
        ("plan", PRE, q),
        ("line", (GRASP, LIFT), q, False, AT_PRE),
        ("report", "move_above", 0, 0),
        THE_RUN,
        ("grip", "left_gripper", OPEN_FOR_THE_CUP, 0.0),
        ("line", (GRASP, LIFT), q, True, ()),
        ("report", "grasp", 0, 0),
        ("move", GRASP, q),
        ("grip", "left_gripper", 0.0, 12.0),
        ("move", LIFT, q),
    ]


async def test_a_grab_that_no_orientation_reaches_reports_its_plan_alone():
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, "no joint solution", 0.0)] * len(DOWN))
    reports = Reports()
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken(), GrabReports(reports, "cup"))
    assert not outcome.success and outcome.message.startswith("no plan reaches item 'cup_1-t0'")
    assert reports.stages == [("plan", 0, 0)]


async def test_a_grab_that_ends_before_its_first_stage_reports_nothing():
    manipulator = ScriptedManipulator(CentreProposer(), clock=FakeClock())
    reports = Reports()
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken(), GrabReports(reports, "cup")) == Outcome(False, NOT_STARTED)
    assert reports.reports == []


async def test_a_place_at_a_pose_reports_each_stage_as_it_starts():
    manipulator, robot, _ = await started(aligner=ScriptedAligningModel((0.01, 0.0), READY, max_steps=2))
    reports = Reports(robot.calls)
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken(), CarryReports(reports, "cup"))
    assert outcome.success
    aligned_above, aligned_point = shifted(PLACE_ABOVE, 0.01), shifted(PLACE_POINT, 0.01)
    assert robot.calls == [
        ("report", "plan", 0, 0),
        ("check", PLACE_ABOVE, PLACE_Q), ("check", PLACE_POINT, PLACE_Q),
        ("report", "move_above", 0, 0),
        ("move", PLACE_ABOVE, PLACE_Q),
        ("report", "align_look", 1, 2),
        ("report", "align_move", 1, 2),
        ("move", aligned_above, PLACE_Q),
        ("report", "align_look", 2, 2),
        ("report", "set_down", 0, 0),
        ("move", aligned_point, PLACE_Q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", aligned_above, PLACE_Q),
    ]


async def test_a_place_whose_jaws_stand_closed_on_air_reports_nothing():
    manipulator, robot, _ = await started(opening=0.0)
    reports = Reports()
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken(), CarryReports(reports, "cup"))
    assert not outcome.success and "closed on air" in outcome.message
    assert reports.reports == [] and robot.calls == []


async def test_a_drop_above_a_target_item_reports_its_carry_its_alignment_and_its_release():
    manipulator, robot, _ = await started(aligner=ScriptedAligningModel(READY, max_steps=2))
    reports = Reports(robot.calls)
    outcome = await manipulator.drop(LEFT, CancelToken(), CarryReports(reports, "cup", BOWL), above=BOWL)
    assert outcome.success
    q = DOWN[0]
    assert robot.calls == [
        ("report", "plan", 0, 0),
        ("check", HIGH, q),
        ("report", "move_above", 0, 0),
        ("move", HIGH, q),
        ("report", "align_look", 1, 2),
        ("report", "release", 0, 0),
        ("grip", "left_gripper", 1.0, 0.0),
    ]
    assert reports.reports[-1].activity == "The robot opens its hand and lets the cup fall into the wooden bowl"


@pytest.mark.parametrize("opening, above", [
    # No target item: the jaws open where the arm stands.
    (1.0, None),
    # Jaws closed on air carry nothing: they open where the arm stands.
    (0.0, BOWL),
])
async def test_a_drop_that_opens_where_the_arm_stands_reports_its_release_alone(opening, above):
    manipulator, robot, _ = await started(opening=opening)
    reports = Reports(robot.calls)
    assert (await manipulator.drop(LEFT, CancelToken(), CarryReports(reports, "cup", above), above=above)).success
    assert robot.calls == [("report", "release", 0, 0), ("grip", "left_gripper", 1.0, 0.0)]
    assert [report.activity for report in reports.reports] == ["The robot opens its hand and lets go of the cup"]


# The stages each handler publishes on the feedback of its goal.

SEEN = (0.5, 0.1, 0.72)
BOWL_SHAPE = ItemShape((0.337, 0.000, 0.487), (0.192, 0.196, 0.088), 0.0)


async def scripted_brain(aligner=None, *, opening: float = 1.0):
    """A brain whose manipulation model is the scripted manipulator with
    `aligner`, moving the fake robot of test_scripted, whose store holds a
    first snapshot with the left jaws at `opening`; it knows a cup and a
    wooden bowl with its box."""
    manipulator = ScriptedManipulator(CentreProposer(), aligner=aligner, snapshot_wait_s=0.0, clock=FakeClock())
    brain = await make_brain(manipulator=manipulator)
    robot, store = FakeRobot(), LimbStateStore()
    robot.limb_states = store
    store.add(limb_states_message(1.0, left_gripper=opening))
    await manipulator.start(Senses(robot, store, None, None, None))
    cup_item, bowl = brain.state.remember(
        [Detection("cup", SEEN, 0.9), Detection("wooden bowl", (0.38, -0.008, 0.461), 0.9, width_m=0.18, shape=BOWL_SHAPE)],
        brain.now(), coverage=Coverage(every_label=True),
    )
    return brain, robot, store, cup_item, bowl


def holding_the_cup(brain, cup_item) -> None:
    brain.state.set_held(brain.state.grippers["left_gripper"], cup_item.item_id)


async def test_grab_item_publishes_each_stage_of_the_grab_naming_the_item():
    brain, robot, store, cup_item, _ = await scripted_brain(ScriptedAligningModel((0.01, 0.0), READY))
    holding_after_the_lift(robot, store)
    ctx = FakeCtx(grab_action, gripper_name="left_gripper", item_id=cup_item.item_id, target=UNREAD_TARGET, max_effort=0.0)
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.stages == [
        ("plan", 0, 0), ("move_above", 0, 0), ("align_look", 1, 3), ("align_move", 1, 3), ("align_look", 2, 3), ("grasp", 0, 0),
    ]
    assert ctx.activities == [
        "The robot opens its hand for the cup and finds a way to reach it",
        "The robot moves its hand above the cup",
        "The robot looks at the cup and decides how to correct its hand (1 of 3)",
        "The robot corrects the position of its hand above the cup (1 of 3)",
        "The robot looks at the cup and decides how to correct its hand (2 of 3)",
        "The robot lowers its hand, closes it on the cup and lifts it",
    ]


async def test_a_grab_by_pose_names_the_item():
    brain, robot, store, _, _ = await scripted_brain()
    point = (0.5, 0.1, 0.7)
    holding_after_the_lift(robot, store)
    ctx = FakeCtx(grab_action, gripper_name="left_gripper", item_id="", target=target(point), max_effort=0.0)
    await run_goal(grab_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.activities[0] == "The robot opens its hand for the item and finds a way to reach it"


async def test_place_item_into_a_target_item_publishes_each_stage_naming_the_held_item_and_the_target():
    brain, _, _, cup_item, bowl = await scripted_brain(ScriptedAligningModel(READY, max_steps=2))
    holding_the_cup(brain, cup_item)
    ctx = FakeCtx(place_action, gripper_name="", target=UNREAD_TARGET, target_item_id=bowl.item_id)
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.stages == [("plan", 0, 0), ("move_above", 0, 0), ("align_look", 1, 2), ("set_down", 0, 0)]
    assert ctx.activities == [
        "The robot finds a way to carry the cup above the wooden bowl",
        "The robot carries the cup above the wooden bowl",
        "The robot looks at the wooden bowl and decides where to put the cup (1 of 2)",
        "The robot lowers the cup into the wooden bowl, opens its hand and moves its arm back up",
    ]


async def test_place_item_at_a_pose_publishes_each_stage_naming_the_held_item():
    brain, _, _, cup_item, _ = await scripted_brain()
    holding_the_cup(brain, cup_item)
    ctx = FakeCtx(place_action, gripper_name="", target=target(PLACE_POINT, PLACE_Q), target_item_id=None)
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.activities == [
        "The robot finds a way to carry the cup above the spot where it puts it down",
        "The robot carries the cup above the spot where it puts it down",
        "The robot puts the cup down, opens its hand and moves its arm back up",
    ]


async def test_drop_item_above_a_target_item_publishes_its_carry_and_its_release():
    brain, _, _, cup_item, bowl = await scripted_brain()
    holding_the_cup(brain, cup_item)
    ctx = FakeCtx(drop_action, gripper_name="", target_item_id=bowl.item_id)
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.activities == [
        "The robot finds a way to carry the cup above the wooden bowl",
        "The robot carries the cup above the wooden bowl",
        "The robot opens its hand and lets the cup fall into the wooden bowl",
    ]


async def test_drop_item_of_an_item_the_brain_no_longer_knows_names_the_item():
    brain, _, _, cup_item, _ = await scripted_brain()
    holding_the_cup(brain, cup_item)
    del brain.state.items[cup_item.item_id]
    ctx = FakeCtx(drop_action, gripper_name="", target_item_id=None)
    await run_goal(drop_item, brain, ctx)
    assert ctx.completed["success"] is True, ctx.completed["message"]
    assert ctx.feedback == [dict(stage="release", activity="The robot opens its hand and lets go of the item", number=0, maximum=0)]


async def test_scan_items_and_identify_item_publish_their_look():
    brain = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]))
    with_frames(brain)
    scan = FakeCtx(scan_action, timeout_s=0.0)
    await run_goal(scan_items, brain, scan)
    search = FakeCtx(identify_action, description="a cup", timeout_s=0.0)
    await run_goal(identify_item, brain, search)
    any_item = FakeCtx(identify_action, description="", timeout_s=0.0)
    await run_goal(identify_item, brain, any_item)
    assert scan.feedback == [dict(stage="look", activity="The robot looks for each item that it can see", number=0, maximum=0)]
    assert search.feedback == [dict(stage="look", activity="The robot looks for the cup", number=0, maximum=0)]
    assert any_item.activities == ["The robot looks for an item that it can take"]
    assert all(ctx.completed is not None for ctx in (scan, search, any_item))


async def test_a_search_that_finds_nothing_still_published_its_look():
    brain = await make_brain(detector=FakeDetector([]))
    with_frames(brain)
    ctx = FakeCtx(identify_action, description="cup", timeout_s=0.0)
    await run_goal(identify_item, brain, ctx)
    assert ctx.failed is not None and ctx.activities == ["The robot looks for the cup"]


async def test_a_search_refused_when_it_starts_publishes_nothing():
    # No perception model.
    brain = await make_brain()
    with_frames(brain)
    no_model = FakeCtx(identify_action, description="cup", timeout_s=0.0)
    await run_goal(identify_item, brain, no_model)
    # A model, and no camera frame yet.
    brain = await make_brain(detector=FakeDetector([Box("cup", 0.9, 7, 5, 9, 7)]))
    no_frame = FakeCtx(scan_action, timeout_s=0.0)
    await run_goal(scan_items, brain, no_frame)
    for ctx in (no_model, no_frame):
        assert ctx.failed is not None and ctx.feedback == []


@pytest.mark.parametrize("handler, action, goal, refusal", [
    (grab_item, grab_action, dict(gripper_name="", item_id="mug_1-t0", target=UNREAD_TARGET, max_effort=0.0),
     "unknown item 'mug_1-t0': scan or identify it again"),
    (grab_item, grab_action, dict(gripper_name="third_gripper", item_id="cup_1-t0", target=UNREAD_TARGET, max_effort=0.0),
     "unknown gripper 'third_gripper'"),
    (place_item, place_action, dict(gripper_name="", target=target(PLACE_POINT), target_item_id=None), "no gripper holds an item"),
    (drop_item, drop_action, dict(gripper_name="left_gripper", target_item_id=None), "gripper 'left_gripper' holds nothing"),
    (drop_item, drop_action, dict(gripper_name="", target_item_id="mug_1-t0"), "no gripper holds an item"),
])
async def test_a_manipulation_goal_refused_in_its_result_before_it_starts_publishes_nothing(handler, action, goal, refusal):
    brain, robot, _, _, _ = await scripted_brain()
    ctx = FakeCtx(action, **goal)
    await run_goal(handler, brain, ctx)
    assert ctx.completed["success"] is False and ctx.completed["message"].startswith(refusal), ctx.completed["message"]
    assert ctx.feedback == [] and robot.calls == []


async def test_a_place_whose_jaws_stand_closed_on_air_publishes_nothing():
    brain, robot, _, cup_item, _ = await scripted_brain(opening=0.0)
    holding_the_cup(brain, cup_item)
    ctx = FakeCtx(place_action, gripper_name="", target=target(PLACE_POINT), target_item_id=None)
    await run_goal(place_item, brain, ctx)
    assert ctx.completed["success"] is False and "closed on air" in ctx.completed["message"]
    assert ctx.feedback == [] and robot.calls == []
