"""The records of the scripted manipulator's steps, read from the
outcome's `steps`: what each step was asked, what it gave the next one,
why it failed and its numbers."""

import pytest
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
from test_scripted_alignment import (
    ALIGNED_LIFT,
    ALIGNED_PLACE_ABOVE,
    ALIGNED_PLACE_POINT,
    ALIGNED_POINT,
    ALIGNED_PRE,
    TURNED,
    FakeAligner,
)
from test_scripted_alignment import started as started_with_aligner

from brain_core.manipulators.scripted.aligning import Aligned, GaveUp, OutOfSteps, Stopped
from brain_core.manipulators.none import NoneManipulator
from brain_core.manipulators.scripted.manipulator import Landed, Released, StepFailure, StepRecord
from brain_core.ports import Cancelled, CancelToken, Outcome, Pose, TargetItem
from brain_core.robot import ArmDescriptions, GripResult, MoveCheck, MoveEnd, PathResult

DROP_INPUTS = {"gripper": "left_gripper", "climb_to": None}
PLACE_RELEASE_INPUTS = {"gripper": "left_gripper", "climb_to": PLACE_ABOVE}


async def place(manipulator) -> Outcome:
    return await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())


def names(outcome: Outcome) -> list[str]:
    return [record.step for record in outcome.steps]


def record_of(outcome: Outcome, step: str) -> StepRecord:
    """The one record of `step` in the outcome."""
    [record] = [record for record in outcome.steps if record.step == step]
    return record


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
    assert names(outcome)[-2:] == ["descend", "release"]
    assert record_of(outcome, "descend") == StepRecord(
        "descend", PLACE_DESCEND_INPUTS, Landed(PLACE_POINT, on_contact=False), measured={"short_of_end_m": 0.0}
    )


async def test_a_place_whose_line_down_ends_on_contact_records_where_and_how_far_short_of_the_pose():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), touched((0.6, 0.0, 0.832), PLACE_Q)])
    outcome = await place(manipulator)
    descend = record_of(outcome, "descend")
    assert descend.result == Landed((0.6, 0.0, 0.832), on_contact=True)
    assert descend.measured == {"short_of_end_m": pytest.approx(0.082)}


async def test_a_line_down_whose_result_gives_no_position_measures_nothing():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), arrived()])
    outcome = await place(manipulator)
    descend = record_of(outcome, "descend")
    assert descend.result == Landed(None, on_contact=False) and descend.measured == {}


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
    assert outcome.steps[-1] == StepRecord("descend", PLACE_DESCEND_INPUTS, failure=failure, message=descent.message)
    assert "release" not in names(outcome)


async def test_a_grab_records_its_line_down_and_closes_where_it_ended():
    manipulator, robot, store = await started()
    holding(robot, store)
    robot.moves.extend([arrived(), touched((GRASP[0], GRASP[1], GRASP[2] + 0.012), DOWN[0])])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == HELD
    assert outcome.steps[-1] == StepRecord(
        "descend", GRAB_DESCEND_INPUTS, Landed((GRASP[0], GRASP[1], GRASP[2] + 0.012), on_contact=True),
        measured={"short_of_end_m": pytest.approx(0.012)},
    )


async def test_a_grab_whose_line_down_times_out_ends_with_its_failed_descend_after_the_retreat():
    manipulator, robot, _ = await started()
    robot.moves.extend([arrived(), move_failed(SHORT, "timeout")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, SHORT)
    assert outcome.steps[-1] == StepRecord("descend", GRAB_DESCEND_INPUTS, failure=StepFailure.MOVE, message=SHORT)
    # The retreat is no step: the gripper opens and the arm climbs back.
    assert robot.calls[-2:] == [("grip", "left_gripper", 1.0, 0.0), ("move", PRE, DOWN[0])]


# The carry step: the transit to the pre point and the move there. A place
# tries the pre-place point 15 cm above the pose, then 7.5 cm; a grab the
# pre-grasp point 7.5 cm back along the approach.

PLACE_CARRY_INPUTS = {"reference": PLACE_POINT, "orientations": (PLACE_Q,), "heights_m": (0.15, 0.075), "rise_m": 0.0}
LOW_ABOVE = (0.6, 0.0, 0.75 + 0.075)


def carry_measured(checks: int, height_m: float, plans: int = 0, reach_tests: int = 0) -> dict:
    # The clock of the tests stands still.
    return {"checks": checks, "plans": plans, "reach_tests": reach_tests, "seconds": 0.0, "height_above_reference_m": height_m}


async def test_a_place_first_carries_the_item_to_the_high_pre_place_point():
    manipulator, robot, _ = await started()
    outcome = await place(manipulator)
    assert names(outcome) == ["carry", "descend", "release"]
    carry = record_of(outcome, "carry")
    assert (carry.inputs, carry.failure, carry.message) == (PLACE_CARRY_INPUTS, None, "")
    assert (carry.result.orientation, carry.result.points) == (PLACE_Q, (PLACE_ABOVE, PLACE_POINT))
    # The pre-place point and the place point, each checked once.
    assert carry.measured == carry_measured(2, pytest.approx(0.15))


async def test_a_place_with_no_plan_at_the_high_point_carries_to_the_low_one_and_counts_every_check():
    manipulator, robot, _ = await started()
    robot.checks.append(MoveCheck(False, "no line plan fits the limits", 0.0))
    carry = record_of(await place(manipulator), "carry")
    assert carry.result.points == (LOW_ABOVE, PLACE_POINT)
    assert carry.measured == carry_measured(3, 0.075)


async def test_a_place_that_no_orientation_reaches_records_no_reach_and_nothing_moves():
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, "no plan")] * 2)
    outcome = await place(manipulator)
    assert outcome.steps == (StepRecord("carry", PLACE_CARRY_INPUTS, failure=StepFailure.NO_REACH, message=outcome.message),)
    assert outcome.message.startswith("no plan reaches the pose [0.600, 0.000, 0.750] with gripper 'left_gripper'")
    assert robot.kinds() == ["check", "check"]


@pytest.mark.parametrize("transit, failure", [
    (move_refused("the side is busy"), StepFailure.MOVE),
    (move_stopped("stopped: the operator asked"), StepFailure.STOPPED),
])
async def test_a_place_whose_transit_does_not_complete_ends_with_its_failed_carry(transit, failure):
    manipulator, robot, _ = await started()
    robot.moves.append(transit)
    outcome = await place(manipulator)
    assert outcome.steps == (StepRecord("carry", PLACE_CARRY_INPUTS, failure=failure, message=transit.message),)


async def test_a_grab_carries_the_gripper_to_the_pre_grasp_point_with_the_grasp_and_lifted_points_after_it():
    manipulator, robot, store = await started()
    holding(robot, store)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert names(outcome) == ["carry", "descend"]
    carry = record_of(outcome, "carry")
    assert carry.inputs == {"reference": GRASP, "orientations": DOWN, "heights_m": (0.075,), "rise_m": 0.0}
    assert (carry.result.orientation, carry.result.points) == (DOWN[0], (PRE, GRASP, LIFT))
    assert carry.measured == carry_measured(3, 0.075)


async def test_a_grab_that_no_orientation_reaches_records_no_reach():
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, f"refusal {i}") for i in range(len(DOWN))])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert [(record.step, record.failure) for record in outcome.steps] == [("carry", StepFailure.NO_REACH)]


async def test_a_planned_carry_counts_its_plans_and_reach_tests():
    manipulator, robot, _ = await started_with_planner()
    carry = record_of(await place(manipulator), "carry")
    assert carry.measured == carry_measured(0, pytest.approx(0.15), plans=1, reach_tests=1)


async def test_a_planned_grab_whose_transit_run_fails_records_the_failed_carry_and_removes_the_object():
    manipulator, robot, _ = await started_with_planner()
    robot.runs.append(PathResult(MoveEnd.FAILED, "the guard would hold the path short at waypoint 1", "unreachable"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert [(record.step, record.failure, record.message) for record in outcome.steps] == [
        ("carry", StepFailure.MOVE, "the guard would hold the path short at waypoint 1"),
    ]
    assert robot.calls[-1] == ("remove", "brain/cup_1-t0")


async def test_a_carry_the_planner_cannot_describe_fails_as_a_plan_failure_with_no_try_of_the_low_point():
    manipulator, robot, _ = await started_with_planner()
    robot.described = ArmDescriptions(reason="describe_arms gave no answer: TimeoutError('no reply')")
    outcome = await place(manipulator)
    assert [(record.step, record.failure) for record in outcome.steps] == [("carry", StepFailure.PLAN)]
    assert robot.kinds() == ["describe"]


async def test_a_cancel_during_a_planned_grabs_transit_run_removes_the_object_and_raises():
    manipulator, robot, _ = await started_with_planner()
    cancel = CancelToken()
    robot.on_call = lambda call: cancel.cancel("aborted: operator", signalled=False) if call[0] == "run" else None
    with pytest.raises(Cancelled):
        await manipulator.grab(cup(), LEFT, 0.0, cancel)
    assert [call[0] for call in robot.calls[-2:]] == ["run", "remove"]


# The align step: the aligner's correction above the point of a grab or a
# place, between the carry and the descent.


async def aligned_started(ending=None, **kwargs):
    """A started manipulator whose aligner ends as `ending` says, and the
    aligner."""
    aligner = FakeAligner(ending, **kwargs)
    manipulator, robot, store, _ = await started_with_aligner(aligner)
    return aligner, manipulator, robot, store


async def test_a_grab_with_an_aligner_runs_carry_align_descend_and_records_how_the_model_moved_the_gripper():
    aligner, manipulator, robot, store = await aligned_started(Aligned(ALIGNED_PRE, ALIGNED_POINT, ALIGNED_LIFT, TURNED, steps=2))
    holding(robot, store)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == HELD and names(outcome) == ["carry", "align", "descend"]
    align = record_of(outcome, "align")
    [request] = aligner.requests
    assert align.inputs == {"request": request}
    assert align.result == Aligned(ALIGNED_PRE, ALIGNED_POINT, ALIGNED_LIFT, TURNED, steps=2)
    # 1.2 cm along x and 0.4 cm along y; DOWN[1] closes the jaws along 45 degrees.
    assert align.measured == {"steps": 2, "shift_cm": pytest.approx(1.265, abs=1e-3), "jaw_line_deg": pytest.approx(45.0)}
    # The descent starts from the aligned pre-grasp point.
    assert record_of(outcome, "descend").inputs["start"] == ALIGNED_PRE


async def test_a_place_with_an_aligner_reads_as_its_four_steps():
    # The place point and the orientation of the alignment tests are the
    # ones of these.
    ending = Aligned(ALIGNED_PLACE_ABOVE, ALIGNED_PLACE_POINT, ALIGNED_PLACE_ABOVE, PLACE_Q, steps=2)
    aligner, manipulator, robot, _ = await aligned_started(ending)
    outcome = await place(manipulator)
    assert outcome.success and names(outcome) == ["carry", "align", "descend", "release"]
    assert record_of(outcome, "align").result == ending
    assert record_of(outcome, "descend").inputs["end"] == ALIGNED_PLACE_POINT
    assert record_of(outcome, "release").inputs["climb_to"] == ALIGNED_PLACE_ABOVE


@pytest.mark.parametrize("ending, failure, message, measured", [
    (GaveUp(steps=1), StepFailure.ALIGNMENT, "the model gave up after 1 step", {"steps": 1}),
    (OutOfSteps(5), StepFailure.ALIGNMENT, "no ready in 5 steps", {"steps": 5}),
    (Stopped("stopped: operator", steps=2), StepFailure.STOPPED, "stopped: operator", {"steps": 2}),
])
async def test_an_alignment_that_does_not_end_ready_ends_the_grab_with_its_failed_align_record(ending, failure, message, measured):
    aligner, manipulator, robot, _ = await aligned_started(ending)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, message)
    assert names(outcome) == ["carry", "align"]
    assert outcome.steps[-1] == StepRecord("align", {"request": aligner.requests[0]}, failure=failure, message=message, measured=measured)


async def test_an_alignment_that_cannot_observe_ends_the_place_with_a_failed_align_and_nothing_measured():
    aligner, manipulator, robot, _ = await aligned_started(failure="no frame from wrist camera 'wrist_left' after the move")
    outcome = await place(manipulator)
    assert outcome == Outcome(False, "the alignment failed: no frame from wrist camera 'wrist_left' after the move")
    assert names(outcome) == ["carry", "align"]
    assert (outcome.steps[-1].failure, outcome.steps[-1].measured) == (StepFailure.ALIGNMENT, {})


# The none manipulator refuses a target item as any sequence.

BOWL = TargetItem("wooden_bowl_1-t0", "wooden bowl", (0.337, 0.0), 0.531, 0.443, 0.192)


async def test_the_none_manipulator_refuses_a_target_item_as_any_sequence():
    none = NoneManipulator()
    assert await none.place(LEFT, BOWL, CancelToken()) == Outcome(False, none.why_unavailable())
    assert await none.drop(LEFT, CancelToken(), above=BOWL) == Outcome(False, none.why_unavailable())
