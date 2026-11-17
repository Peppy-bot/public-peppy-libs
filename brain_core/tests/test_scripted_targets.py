"""The scripted manipulator with a target item: a place into a bowl (the
carry above its top, the alignment told the bowl, the line down to its
bottom that stops on contact, the release and the climb), and every way it
ends before and after the arm moves."""

import pytest
from conftest import records_of
from test_scripted import (
    DOWN,
    LEFT,
    arrived,
    move_failed,
    move_refused,
    started,
    touched,
)
from test_scripted_alignment import FakeAligner
from test_scripted_alignment import started as started_with_aligner

from brain_core.manipulators.scripted.aligning import Aligned, GaveUp
from brain_core.manipulators.scripted.manipulator import ABOVE_TARGET_M, CLEARANCE_M, StepFailure
from brain_core.ports import CancelToken, Outcome, TargetItem
from brain_core.robot import MoveCheck

# The wooden bowl of the box session: its box 19.2 x 19.6 cm, its rim at
# z 0.531 and its bottom on the table at z 0.443.
BOWL = TargetItem("wooden_bowl_1-t0", "wooden bowl", (0.337, 0.0), 0.531, 0.443, 0.192)
Q = DOWN[0]
BOTTOM = (0.337, 0.0, 0.443)
HIGH = (0.337, 0.0, 0.531 + ABOVE_TARGET_M)
LOW = (0.337, 0.0, 0.531 + CLEARANCE_M)
LOG = "brain_core.manipulators.scripted.manipulator"


async def place_into(manipulator, target=BOWL) -> Outcome:
    return await manipulator.place(LEFT, target, CancelToken())


async def test_a_place_into_a_bowl_carries_above_its_rim_lowers_to_its_bottom_opens_and_climbs_back(caplog):
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), arrived((0.337, 0.0, 0.4432), Q)])
    with records_of(caplog, LOG) as log:
        outcome = await place_into(manipulator)
    assert outcome == Outcome(True, "", (0.337, 0.0, 0.4432))
    # The pre-place point 12.5 cm above the rim and the bottom of the bowl,
    # checked in the first roll; then the transit, the line down, the
    # release and the climb.
    assert robot.calls == [
        ("check", HIGH, Q), ("check", BOTTOM, Q),
        ("move", HIGH, Q), ("move", BOTTOM, Q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", HIGH, Q),
    ]
    # The line down: 21.3 cm at 5 cm/s, with the contact gap of every line down.
    _, line_down = [entry for entry in robot.bars if entry[0] == "move"][1]
    assert line_down["duration_s"] == pytest.approx(0.213 / 0.05) and line_down["contact_gap_m"] == 0.02
    assert [record.step for record in outcome.steps] == ["carry", "descend", "release"]
    assert [record.getMessage() for record in log.records] == [
        "place with gripper 'left_gripper' into item 'wooden_bowl_1-t0': orientation 1 of 8 planned, "
        "[0.000, 1.000, 0.000, 0.000] (2 checks, 0 ms)",
        "place with gripper 'left_gripper' into item 'wooden_bowl_1-t0' met nothing down to the item's bottom: it releases there",
    ]


async def test_a_place_into_a_bowl_that_holds_fruit_releases_where_the_item_comes_to_rest(caplog):
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), touched((0.337, 0.0, 0.512), Q)])
    with records_of(caplog, LOG) as log:
        outcome = await place_into(manipulator)
    assert outcome == Outcome(True, "", (0.337, 0.0, 0.512))
    assert robot.calls[-2:] == [("grip", "left_gripper", 1.0, 0.0), ("move", HIGH, Q)]
    assert log.records[-1].getMessage() == (
        "place with gripper 'left_gripper' into item 'wooden_bowl_1-t0' ended its line down on contact "
        "[+0.0, +0.0, +6.9] cm from the item's bottom: it releases there"
    )


async def test_a_place_into_a_bowl_with_no_plan_12_5_cm_above_its_rim_carries_to_7_5_cm(caplog):
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, "no line plan fits the limits")] * len(DOWN))
    with records_of(caplog, LOG) as log:
        outcome = await place_into(manipulator)
    assert outcome.success
    [transit] = [call for call in robot.calls if call[0] == "move"][:1]
    assert transit[1] == pytest.approx(LOW) and transit[2] == Q
    assert log.records[0].getMessage().startswith(
        "place with gripper 'left_gripper' into item 'wooden_bowl_1-t0': no plan 12.5 cm above the top of item "
        "'wooden_bowl_1-t0': the pre-place point stands 7.5 cm above it; orientation 1 of 8 planned"
    )


async def test_a_bowl_that_no_orientation_reaches_fails_the_place_before_any_move_with_its_id():
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, "no plan")] * (2 * len(DOWN)))
    outcome = await place_into(manipulator)
    assert outcome.message.startswith("no plan reaches item 'wooden_bowl_1-t0' with gripper 'left_gripper': orientation 8 of 8")
    assert outcome.message.endswith("has no plan to its pre-place point [0.337, 0.000, 0.606]: no plan")
    assert [(record.step, record.failure) for record in outcome.steps] == [("carry", StepFailure.NO_REACH)]
    assert set(robot.kinds()) == {"check"}


@pytest.mark.parametrize("descent", [move_failed("still 32.1 mm / 0.05 rad from the target", "timeout"), move_refused("the side is busy")])
async def test_a_place_into_a_bowl_whose_line_down_does_not_complete_fails_and_still_holds_the_item(descent):
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), descent])
    outcome = await place_into(manipulator)
    assert outcome == Outcome(False, descent.message)
    assert "grip" not in robot.kinds()


async def test_a_place_into_a_bowl_with_the_jaws_closed_on_air_fails_before_any_move():
    manipulator, robot, _ = await started(opening=0.02)
    outcome = await place_into(manipulator)
    assert outcome.message.startswith("gripper 'left_gripper' holds nothing now")
    assert robot.calls == []


async def test_a_place_into_a_target_with_no_box_carries_above_its_top_and_lowers_to_its_seen_point():
    manipulator, robot, _ = await started()
    seen = TargetItem("tray_1-t0", "tray", (0.40, 0.10), 0.47, 0.45, None)
    assert (await place_into(manipulator, seen)).success
    assert robot.calls[:2] == [("check", (0.40, 0.10, 0.47 + ABOVE_TARGET_M), Q), ("check", (0.40, 0.10, 0.45), Q)]


# With an aligner: the request names the bowl, and the line down starts from
# the aligned point.

async def test_the_alignment_of_a_place_into_a_bowl_is_told_the_bowl_and_the_line_down_follows_it():
    aligned_pre, aligned_bottom = (0.35, 0.03, HIGH[2]), (0.35, 0.03, 0.443)
    aligner = FakeAligner(Aligned(aligned_pre, aligned_bottom, aligned_pre, Q, steps=1))
    manipulator, robot, _, _ = await started_with_aligner(aligner)
    outcome = await place_into(manipulator)
    assert outcome.success and [record.step for record in outcome.steps] == ["carry", "align", "descend", "release"]
    [request] = aligner.requests
    assert request.into == BOWL and request.descends and request.is_place
    assert (request.pre, request.point, request.target) == (HIGH, BOTTOM, (0.337, 0.0, 0.531))
    assert request.centre_words == "the centre of the wooden bowl"
    assert [call for call in robot.calls if call[0] != "check"][1:] == [
        ("move", aligned_bottom, Q), ("grip", "left_gripper", 1.0, 0.0), ("move", aligned_pre, Q),
    ]


async def test_a_model_that_gives_up_above_the_bowl_fails_the_place_before_the_line_down():
    manipulator, robot, _, _ = await started_with_aligner(FakeAligner(GaveUp(steps=2)))
    outcome = await place_into(manipulator)
    assert outcome == Outcome(False, "the model gave up after 2 steps")
    assert robot.kinds() == ["check", "check", "move"]
