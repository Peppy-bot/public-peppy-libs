"""The records of the scripted manipulator's steps, read from the
outcome's `steps`: what each step was asked, what it gave the next one,
why it failed and its numbers."""

from test_scripted import (
    LEFT,
    PLACE_ABOVE,
    PLACE_POINT,
    PLACE_Q,
    arrived,
    grip_failed,
    gripped,
    move_refused,
    started,
)

from brain_core.manipulators.scripted.manipulator import Released, StepFailure, StepRecord
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
