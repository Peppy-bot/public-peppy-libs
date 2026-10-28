"""The scripted manipulator with an aligner: where the alignment sits in
the grab, what the descent, the lift and the retreat use after it, and how
each way an alignment ends ends the grab."""

from types import SimpleNamespace

import pytest
from conftest import records_of
from fake_peppygen import limb_states_message
from test_scripted import DOWN, GRASP, LIFT, LIFT_MOVE, PRE, FakeClock, FakeRobot, cup, gripped, holding

from brain_core.limb_state import LimbStateStore
from brain_core.manipulators.scripted.aligning import Aligned, AlignmentFailed, GaveUp, OutOfSteps, Stopped
from brain_core.manipulators.scripted.centre_proposer import CentreProposer
from brain_core.manipulators.scripted.grasp import lifted
from brain_core.manipulators.scripted.manipulator import ScriptedManipulator
from brain_core.ports import CancelToken, Gripper, Outcome, Pose, Senses
from brain_core.robot import MoveCheck, MoveEnd, MoveResult

LEFT = Gripper("left_gripper", "left_arm")
ALIGNED_PRE = (PRE[0] + 0.012, PRE[1] - 0.004, PRE[2])
ALIGNED_POINT = (ALIGNED_PRE[0], ALIGNED_PRE[1], GRASP[2])
ALIGNED_LIFT = lifted(ALIGNED_POINT, LIFT[2] - GRASP[2])
TURNED = DOWN[1]


class FakeAligner:
    """Ends the way the test says, and records what it was asked."""

    def __init__(self, ending=None, *, unavailable: str = "", failure: str = "") -> None:
        self.ending = ending
        self.unavailable = unavailable
        self.failure = failure
        self.senses = None
        self.requests: list = []
        self.moves = None

    def start(self, senses) -> None:
        self.senses = senses

    def why_unavailable(self, arm: str) -> str:
        return self.unavailable

    async def align(self, request, moves, cancel):
        self.requests.append(request)
        self.moves = moves
        if self.failure:
            raise AlignmentFailed(self.failure)
        return self.ending


async def started(aligner, **kwargs):
    robot, store = FakeRobot(), LimbStateStore()
    store.add(limb_states_message(1.0, left_gripper=1.0))
    manipulator = ScriptedManipulator(CentreProposer(), aligner=aligner, clock=FakeClock(), **kwargs)
    senses = Senses(robot, store, SimpleNamespace(), SimpleNamespace(), None)
    await manipulator.start(senses)
    return manipulator, robot, store, senses


async def test_start_hands_the_aligner_the_senses():
    aligner = FakeAligner(Aligned(PRE, GRASP, LIFT, DOWN[0]))
    _, _, _, senses = await started(aligner)
    assert aligner.senses is senses


async def test_the_alignment_runs_above_the_item_and_the_descent_lift_and_hold_use_the_aligned_points(caplog):
    aligner = FakeAligner(Aligned(ALIGNED_PRE, ALIGNED_POINT, ALIGNED_LIFT, TURNED, steps=2))
    manipulator, robot, store, _ = await started(aligner)
    holding(robot, store)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(True)
    (request,) = aligner.requests
    assert (request.pre, request.point, request.orientation) == (PRE, GRASP, DOWN[0])
    assert request.item.item_id == "cup_1-t0" and request.gripper is LEFT
    assert aligner.moves is manipulator
    moves = [call for call in robot.calls if call[0] == "move"]
    # The pre-grasp move in the first orientation, then the descent and the
    # lift at the aligned points in the aligned orientation.
    assert moves == [("move", PRE, DOWN[0]), ("move", ALIGNED_POINT, TURNED), ("move", ALIGNED_LIFT, TURNED)]
    line = log.records[-1].getMessage()
    # DOWN[1] is roll 45 of the grasp frame, which turns the jaw line
    # clockwise from +y: it lies at 45 degrees from +x.
    assert "aligned in 2 steps: the grasp point moved 1.3 cm, the jaws close along 45 degrees" in line


async def test_a_failure_after_the_descent_retreats_to_the_aligned_pre_grasp_point():
    aligner = FakeAligner(Aligned(ALIGNED_PRE, ALIGNED_POINT, ALIGNED_LIFT, TURNED))
    manipulator, robot, _, _ = await started(aligner)
    robot.grips.extend([gripped(1.0), gripped(0.0)])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, _token())
    assert outcome == Outcome(False, "gripper 'left_gripper' closed on nothing")
    assert robot.calls[-1] == ("move", ALIGNED_PRE, TURNED)


@pytest.mark.parametrize("ending, message", [
    (GaveUp(steps=2), "the model gave up after 2 steps"),
    (GaveUp(steps=1), "the model gave up after 1 step"),
    (OutOfSteps(5), "no ready in 5 steps"),
])
async def test_a_model_that_gives_up_or_runs_out_ends_the_grab_before_the_descent_with_no_retreat(ending, message):
    manipulator, robot, _, _ = await started(FakeAligner(ending))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, _token())
    assert outcome == Outcome(False, message)
    # The checks, the open and the move to the pre-grasp point; nothing after.
    assert robot.kinds() == ["check", "check", "check", "grip", "move"]


async def test_a_stopped_alignment_ends_the_grab_with_the_stops_words_and_nothing_more():
    manipulator, robot, _, _ = await started(FakeAligner(Stopped("stopped: operator", steps=1)))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, _token())
    assert outcome == Outcome(False, "stopped: operator")
    assert robot.kinds() == ["check", "check", "check", "grip", "move"]


async def test_an_alignment_that_fails_names_what_it_could_not_get():
    manipulator, robot, _, _ = await started(FakeAligner(failure="no frame from wrist camera 'wrist_left' after the move"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, _token())
    assert outcome == Outcome(False, "the alignment failed: no frame from wrist camera 'wrist_left' after the move")
    assert robot.kinds() == ["check", "check", "check", "grip", "move"]


async def test_an_unavailable_alignment_fails_the_grab_before_any_move():
    manipulator, robot, _, _ = await started(FakeAligner(unavailable="no wrist camera for left_arm: the wrist_cameras parameter names none"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, _token())
    assert outcome == Outcome(False, "no alignment: no wrist camera for left_arm: the wrist_cameras parameter names none")
    assert robot.calls == []


async def test_the_aligners_moves_go_through_the_manipulator_with_its_bars():
    manipulator, robot, _, _ = await started(FakeAligner(GaveUp()))
    assert await manipulator.refusal_for("left_arm", (PRE, GRASP), DOWN[0], _token()) == ""
    robot.checks.append(MoveCheck(False, "no joint solution"))
    assert await manipulator.refusal_for("left_arm", (PRE,), DOWN[0], _token()) == "no joint solution"
    moved = await manipulator.move_to("left_arm", ALIGNED_PRE, TURNED)
    assert moved == MoveResult(MoveEnd.COMPLETED, final_position=ALIGNED_PRE, final_orientation=TURNED)
    kind, bars = robot.bars[-1]
    assert kind == "move" and set(bars) == {
        "duration_s", "plan_position_tolerance_m", "plan_orientation_tolerance_rad",
        "settle_position_tolerance_m", "settle_orientation_tolerance_rad", "settle_timeout_s",
    }


async def test_without_an_aligner_the_grab_is_the_stage_1_sequence():
    manipulator, robot, store, _ = await started(None)
    holding(robot, store)
    assert await manipulator.grab(cup(), LEFT, 0.0, _token()) == Outcome(True)
    assert robot.kinds().count("move") == LIFT_MOVE


def _token() -> CancelToken:
    return CancelToken()


# A place with an aligner: the same step above the spot, then the descent
# and the release at the aligned points.

PLACE_Q = (0.0, 1.0, 0.0, 0.0)
PLACE_POINT = (0.6, 0.0, 0.75)
PLACE_ABOVE = (0.6, 0.0, 0.75 + (0.075 + 0.075))
ALIGNED_PLACE_ABOVE = (0.61, -0.02, PLACE_ABOVE[2])
ALIGNED_PLACE_POINT = (0.61, -0.02, PLACE_POINT[2])


async def test_a_place_aligns_above_the_spot_and_releases_at_the_aligned_point(caplog):
    aligner = FakeAligner(Aligned(ALIGNED_PLACE_ABOVE, ALIGNED_PLACE_POINT, ALIGNED_PLACE_ABOVE, PLACE_Q, steps=2))
    manipulator, robot, _, _ = await started(aligner)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), _token())
    assert outcome == Outcome(True, "", ALIGNED_PLACE_POINT)
    (request,) = aligner.requests
    assert request.item is None and request.target == PLACE_POINT and request.is_place
    assert (request.pre, request.point, request.orientation) == (PLACE_ABOVE, PLACE_POINT, PLACE_Q)
    assert [call for call in robot.calls if call[0] != "check"] == [
        ("move", PLACE_ABOVE, PLACE_Q),
        ("move", ALIGNED_PLACE_POINT, PLACE_Q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", ALIGNED_PLACE_ABOVE, PLACE_Q),
    ]
    assert "place with gripper 'left_gripper': aligned in 2 steps: the grasp point moved 2.2 cm" in log.records[0].getMessage()


async def test_a_place_whose_model_gives_up_fails_before_the_descent_and_keeps_the_item():
    manipulator, robot, _, _ = await started(FakeAligner(GaveUp(steps=1)))
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), _token())
    assert outcome == Outcome(False, "the model gave up after 1 step")
    assert robot.kinds() == ["check", "check", "move"]


async def test_an_unavailable_alignment_fails_the_place_before_any_move():
    manipulator, robot, _, _ = await started(FakeAligner(unavailable="no frame from wrist camera 'wrist_left' yet"))
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), _token())
    assert outcome == Outcome(False, "no alignment: no frame from wrist camera 'wrist_left' yet")
    assert robot.calls == []
