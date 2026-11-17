"""The records of the scripted manipulator's steps, read from the
outcome's `steps`: what each step was asked, what it gave the next one,
why it failed and its numbers."""

import pytest
from test_scripted import (
    DOWN,
    GRASP,
    HELD,
    LEFT,
    PLACE_ABOVE,
    PLACE_POINT,
    PLACE_Q,
    PRE,
    SHORT,
    arrived,
    cup,
    grip_failed,
    gripped,
    holding,
    move_failed,
    move_refused,
    move_stopped,
    started,
    touched,
)

from brain_core.manipulators.scripted.manipulator import Landed, Released, StepFailure, StepRecord
from brain_core.ports import CancelToken, Outcome, Pose
from brain_core.robot import GripResult, MoveEnd

DROP_INPUTS = {"gripper": "left_gripper", "climb_to": None}
PLACE_RELEASE_INPUTS = {"gripper": "left_gripper", "climb_to": PLACE_ABOVE}


async def place(manipulator) -> Outcome:
    return await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())


async def test_a_drop_records_its_release_with_the_opening_the_jaws_stood_at():
    manipulator, robot, _ = await started()
    robot.grips.append(gripped(0.97))
    outcome = await manipulator.drop(LEFT, CancelToken())
    assert outcome == Outcome(True)
    assert outcome.steps == (StepRecord("release", DROP_INPUTS, Released(0.97), measured={"opening": 0.97}),)


async def test_a_drop_whose_jaws_do_not_open_records_a_failed_move_and_a_stopped_one_a_stop():
    manipulator, robot, _ = await started()
    robot.grips.append(grip_failed("the follower stopped reporting"))
    failed = await manipulator.drop(LEFT, CancelToken())
    assert failed.steps == (StepRecord("release", DROP_INPUTS, failure=StepFailure.MOVE, message="the follower stopped reporting"),)
    robot.grips.append(GripResult(MoveEnd.CANCELLED, "stopped: the operator asked"))
    stopped = await manipulator.drop(LEFT, CancelToken())
    assert stopped == Outcome(False, "stopped: the operator asked")
    assert stopped.steps == (StepRecord("release", DROP_INPUTS, failure=StepFailure.STOPPED, message="stopped: the operator asked"),)


async def test_a_drop_that_cannot_start_runs_no_step():
    manipulator, robot, _ = await started(opening=None)
    outcome = await manipulator.drop(LEFT, CancelToken())
    assert not outcome.success and outcome.steps == ()
    assert robot.calls == []


async def test_a_place_ends_with_its_release_and_the_climb_back_to_the_pre_place_point():
    manipulator, robot, _ = await started()
    outcome = await place(manipulator)
    assert outcome.success
    assert outcome.steps[-1] == StepRecord("release", PLACE_RELEASE_INPUTS, Released(1.0, climbed=True), measured={"opening": 1.0})


async def test_a_place_whose_climb_fails_records_a_release_that_went_through_with_no_climb():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), arrived(), move_refused("the side is busy")])
    outcome = await place(manipulator)
    assert outcome.success
    assert outcome.steps[-1] == StepRecord("release", PLACE_RELEASE_INPUTS, Released(1.0, climbed=False), measured={"opening": 1.0})


async def test_a_place_whose_jaws_do_not_open_ends_with_the_failed_release_and_does_not_climb():
    manipulator, robot, _ = await started()
    robot.grips.append(grip_failed("the follower stopped reporting"))
    outcome = await place(manipulator)
    assert outcome == Outcome(False, "the follower stopped reporting")
    assert outcome.steps[-1] == StepRecord(
        "release", PLACE_RELEASE_INPUTS, failure=StepFailure.MOVE, message="the follower stopped reporting"
    )
    assert robot.kinds()[-1] == "grip"


# The descend step: the line down of a place, 15 cm from the pre-place point
# at 5 cm/s, and of a grab, 7.5 cm from the pre-grasp point, each with a
# contact gap of 2 cm.

PLACE_DESCEND_INPUTS = {
    "start": PLACE_ABOVE, "end": PLACE_POINT, "orientation": PLACE_Q, "contact_gap_m": 0.02, "duration_s": pytest.approx(3.0),
}
GRAB_DESCEND_INPUTS = {"start": PRE, "end": GRASP, "orientation": DOWN[0], "contact_gap_m": 0.02, "duration_s": pytest.approx(1.5)}


async def test_a_place_records_its_line_down_settled_at_the_pose_before_its_release():
    manipulator, robot, _ = await started()
    outcome = await place(manipulator)
    assert [record.step for record in outcome.steps] == ["descend", "release"]
    assert outcome.steps[0] == StepRecord(
        "descend", PLACE_DESCEND_INPUTS, Landed(PLACE_POINT, on_contact=False), measured={"short_of_end_m": 0.0}
    )


async def test_a_place_whose_line_down_ends_on_contact_records_where_and_how_far_short_of_the_pose():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), touched((0.6, 0.0, 0.832), PLACE_Q)])
    outcome = await place(manipulator)
    descend = outcome.steps[0]
    assert descend.result == Landed((0.6, 0.0, 0.832), on_contact=True)
    assert descend.measured == {"short_of_end_m": pytest.approx(0.082)}


async def test_a_line_down_whose_result_gives_no_position_measures_nothing():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), arrived()])
    outcome = await place(manipulator)
    assert outcome.steps[0].result == Landed(None, on_contact=False) and outcome.steps[0].measured == {}


@pytest.mark.parametrize("descent, failure", [
    (move_failed(SHORT, "timeout"), StepFailure.MOVE),
    (move_failed("the follower stopped reporting"), StepFailure.MOVE),
    (move_refused("the side is busy"), StepFailure.MOVE),
    (move_stopped("stopped: the operator asked"), StepFailure.STOPPED),
])
async def test_a_place_whose_line_down_does_not_complete_ends_with_its_failed_descend_and_no_release(descent, failure):
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), descent])
    outcome = await place(manipulator)
    assert outcome.steps == (StepRecord("descend", PLACE_DESCEND_INPUTS, failure=failure, message=descent.message),)


async def test_a_grab_records_its_line_down_and_closes_where_it_ended():
    manipulator, robot, store = await started()
    holding(robot, store)
    robot.moves.extend([arrived(), touched((GRASP[0], GRASP[1], GRASP[2] + 0.012), DOWN[0])])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == HELD
    assert outcome.steps == (
        StepRecord(
            "descend", GRAB_DESCEND_INPUTS, Landed((GRASP[0], GRASP[1], GRASP[2] + 0.012), on_contact=True),
            measured={"short_of_end_m": pytest.approx(0.012)},
        ),
    )


async def test_a_grab_whose_line_down_times_out_ends_with_its_failed_descend_after_the_retreat():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), move_failed(SHORT, "timeout")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, SHORT)
    assert outcome.steps == (StepRecord("descend", GRAB_DESCEND_INPUTS, failure=StepFailure.MOVE, message=SHORT),)
    # The retreat is no step: the gripper opens and the arm climbs back.
    assert robot.calls[-2:] == [("grip", "left_gripper", 1.0, 0.0), ("move", PRE, DOWN[0])]
