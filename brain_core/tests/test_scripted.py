"""The scripted manipulator against a fake robot, a limb store the test
fills and a clock that stands still: the grab sequence and every way it
ends, the bars each move names, the drop, the place, and two properties
over any pattern of backbone answers."""

import asyncio
from collections import deque
from typing import Callable, Optional

import pytest
from conftest import records_of
from fake_peppygen import limb_states_message
from hypothesis import event, given, settings
from hypothesis import strategies as st

from brain_core.limb_state import NO_SNAPSHOT, LimbStateStore
from brain_core.manipulators.scripted.centre_proposer import GRASP_DEPTH_M, CentreProposer
from brain_core.manipulators.scripted.grasp import GraspDirection, candidates, lowered
from brain_core.manipulators.scripted.manipulator import (
    CLEARANCE_M,
    HELD_ITEM_DEPTH_M,
    GRIP_SETTLE_TIMEOUT_S,
    LIFT_M,
    MOVE_DURATION_S,
    NOT_STARTED,
    PLAN_ORIENTATION_RAD,
    PLAN_POSITION_M,
    SETTLE_ORIENTATION_RAD,
    SETTLE_POSITION_M,
    SETTLE_TIMEOUT_S,
    ScriptedManipulator,
)
from brain_core.ports import Cancelled, CancelToken, Gripper, Item, Outcome, Pose, Senses
from brain_core.robot import GripResult, MoveCheck, MoveEnd, MoveResult

LEFT = Gripper("left_gripper", "left_arm")
DOWN = candidates(GraspDirection.DOWN)
SEEN = (0.5, 0.1, 0.72)
GRASP = lowered(SEEN, GRASP_DEPTH_M)
# The pre-grasp and the lifted points of a grab from above coincide when
# the clearance and the lift are equal, as their defaults are.
PRE = (GRASP[0], GRASP[1], GRASP[2] + CLEARANCE_M)
LIFT = (GRASP[0], GRASP[1], GRASP[2] + LIFT_M)
# A grab's third move is its lift.
LIFT_MOVE = 3
# The bars a check, an arm move and a gripper move name.
CHECK_BARS = dict(duration_s=MOVE_DURATION_S, plan_position_tolerance_m=PLAN_POSITION_M, plan_orientation_tolerance_rad=PLAN_ORIENTATION_RAD)
MOVE_BARS = dict(
    CHECK_BARS, settle_position_tolerance_m=SETTLE_POSITION_M, settle_orientation_tolerance_rad=SETTLE_ORIENTATION_RAD,
    settle_timeout_s=SETTLE_TIMEOUT_S,
)
GRIP_BARS = dict(settle_timeout_s=GRIP_SETTLE_TIMEOUT_S)


def arrived(position=None, orientation=None) -> MoveResult:
    """An arm move that completed, its grasp point at `position`."""
    return MoveResult(MoveEnd.COMPLETED, final_position=position, final_orientation=orientation)


def move_failed(message: str, kind: str = "attempt_failed") -> MoveResult:
    """An arm move that started and failed: no result, so no position."""
    return MoveResult(MoveEnd.FAILED, message, kind)


def move_refused(message: str, kind: str = "precondition_failed") -> MoveResult:
    return MoveResult(MoveEnd.REFUSED, message, kind)


def move_stopped(reason: str) -> MoveResult:
    """An arm move that a stop ended: cancelled, with the stop's reason."""
    return MoveResult(MoveEnd.CANCELLED, reason)


def gripped(opening: float) -> GripResult:
    """A gripper move that completed, the jaws standing at `opening`."""
    return GripResult(MoveEnd.COMPLETED, settled=True, final_opening=opening)


def grip_failed(message: str, kind: str = "attempt_failed") -> GripResult:
    return GripResult(MoveEnd.FAILED, message, kind)


class FakeClock:
    """The brain's monotonic clock in a test: it stands still."""

    def __call__(self) -> float:
        return 0.0


class FakeRobot:
    """Stands in for the Robot. Records every call in order: ("check",
    position, orientation), ("move", position, orientation) or ("grip",
    gripper, opening, max_effort), and the bars each one named, beside it
    in `bars`. A check plans, a move lands on its target, a grip stops at
    its target, unless the test queued another answer for that kind.
    `on_call` runs once each call is recorded, before it answers: a test
    hooks the lift or a cancel there.

    Each call yields to the loop once before it is recorded, as a call to
    the backbone does, so what an earlier call scheduled has happened by
    then; it answers with no yield after `on_call`, so what `on_call`
    schedules happens after the caller has read the answer."""

    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.bars: list[tuple[str, dict]] = []
        self.checks: deque = deque()
        self.moves: deque = deque()
        self.grips: deque = deque()
        self.on_call: Callable[[tuple], None] = lambda call: None

    async def check_arm_move(self, arm, position, orientation, **bars) -> MoveCheck:
        assert arm == "left_arm"
        await asyncio.sleep(0)
        self.bars.append(("check", bars))
        self._record(("check", position, orientation))
        return self.checks.popleft() if self.checks else MoveCheck(True, "", 1.0)

    async def move_arm(self, arm, position, orientation, **bars) -> MoveResult:
        assert arm == "left_arm"
        await asyncio.sleep(0)
        self.bars.append(("move", bars))
        self._record(("move", position, orientation))
        return self.moves.popleft() if self.moves else arrived(position, orientation)

    async def set_gripper(self, gripper, opening, *, max_effort, **bars) -> GripResult:
        await asyncio.sleep(0)
        self.bars.append(("grip", bars))
        self._record(("grip", gripper, opening, max_effort))
        return self.grips.popleft() if self.grips else gripped(opening)

    def _record(self, call: tuple) -> None:
        self.calls.append(call)
        self.on_call(call)

    def kinds(self) -> list[str]:
        return [call[0] for call in self.calls]

    def is_lift(self, call: tuple) -> bool:
        """Whether `call`, the last one recorded, is a grab's lift."""
        return call[0] == "move" and self.kinds().count("move") == LIFT_MOVE


def snapshot_after_the_lift(robot: FakeRobot, store: LimbStateStore, message) -> None:
    """The backbone publishes `message` just after it answers the lift."""

    def on_call(call: tuple) -> None:
        if robot.is_lift(call):
            asyncio.get_running_loop().call_soon(store.add, message)

    robot.on_call = on_call


async def started(*, opening: Optional[float] = 1.0, wait_s: float = 0.0, **kwargs):
    """A started scripted manipulator on a fake robot and a still clock, the
    store holding a first snapshot with the left gripper open, unless
    `opening` is none."""
    robot, store = FakeRobot(), LimbStateStore()
    if opening is not None:
        store.add(limb_states_message(1.0, left_gripper=opening))
    manipulator = ScriptedManipulator(CentreProposer(), snapshot_wait_s=wait_s, clock=FakeClock(), **kwargs)
    await manipulator.start(Senses(robot, store, None, None, None))
    return manipulator, robot, store


def cup(position=SEEN, orientation=None, *, grasp_point=False, width_m=None) -> Item:
    return Item("cup_1-t0", "cup", position, orientation, 0.9, 1, position_is_grasp_point=grasp_point, width_m=width_m)


def holding(robot: FakeRobot, store: LimbStateStore, closed: float = 0.3, after: float = 0.3) -> None:
    """The close stops at `closed`, and the snapshot after the lift reads
    `after`."""
    robot.grips.extend([gripped(1.0), gripped(closed)])
    snapshot_after_the_lift(robot, store, limb_states_message(2.0, left_gripper=after))


async def test_a_hold_checks_the_three_moves_then_opens_reaches_closes_and_lifts(caplog):
    manipulator, robot, store = await started()
    holding(robot, store)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.grab(cup(), LEFT, 12.0, CancelToken())
    assert outcome == Outcome(True)
    q = DOWN[0]
    assert robot.calls == [
        ("check", PRE, q), ("check", GRASP, q), ("check", LIFT, q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", PRE, q), ("move", GRASP, q),
        ("grip", "left_gripper", 0.0, 12.0),
        ("move", LIFT, q),
    ]
    [line] = [record.getMessage() for record in log.records]
    # The fake descent lands on the grasp point; the fake limb_states put
    # the left grasp point at [0.4, 0.2, 0.9]; the clock stands still.
    assert line == (
        "grab of item 'cup_1-t0' with gripper 'left_gripper': orientation 1 of 8 planned, [0.000, 1.000, 0.000, 0.000] (3 checks, 0 ms); "
        "landed [+0.0, +0.0, +0.0] cm from the grasp point; limb_state at the close [-10.0, +10.0, +20.5] cm from it; "
        "closed at 0.300; 0.300 after the lift; held"
    )


async def test_every_check_arm_move_and_gripper_move_names_its_bars():
    manipulator, robot, store = await started()
    holding(robot, store)
    assert await manipulator.grab(cup(), LEFT, 12.0, CancelToken()) == Outcome(True)
    bars = {"check": CHECK_BARS, "move": MOVE_BARS, "grip": GRIP_BARS}
    assert robot.bars == [(kind, bars[kind]) for kind in robot.kinds()]
    # Plan 1 mm and 0.01 rad; settle within 2 cm and 0.15 rad, 2 s after the
    # planned end at most; the jaws get 2 s to stand still.
    assert MOVE_BARS == dict(
        duration_s=0.0, plan_position_tolerance_m=0.001, plan_orientation_tolerance_rad=0.01,
        settle_position_tolerance_m=0.02, settle_orientation_tolerance_rad=0.15, settle_timeout_s=2.0,
    )
    assert GRIP_BARS == dict(settle_timeout_s=2.0)


async def test_a_grab_opens_the_jaws_to_the_items_width_and_a_margin_each_side(caplog):
    manipulator, robot, store = await started()
    holding(robot, store)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        assert await manipulator.grab(cup(width_m=0.06), LEFT, 0.0, CancelToken()) == Outcome(True)
    # 6 cm and 1.5 cm on each side, of the jaws' 14.8 cm span.
    [opened] = [call for call in robot.calls if call[0] == "grip"][:1]
    assert opened == ("grip", "left_gripper", pytest.approx(0.09 / 0.148), 0.0)
    assert "opened to 0.61 for its 6.0 cm width" in log.records[-1].getMessage()


async def test_an_item_wider_than_the_jaws_less_their_margins_gets_them_fully_open():
    manipulator, robot, store = await started()
    holding(robot, store)
    assert await manipulator.grab(cup(width_m=0.14), LEFT, 0.0, CancelToken()) == Outcome(True)
    assert [call for call in robot.calls if call[0] == "grip"][0] == ("grip", "left_gripper", 1.0, 0.0)


def landing_log(robot: FakeRobot) -> None:
    """The descent settles 4 mm off: 3 mm toward the robot, 2 mm left,
    1 mm below, within the 2 cm it names."""
    robot.moves.extend([arrived(), arrived((GRASP[0] - 0.003, GRASP[1] + 0.002, GRASP[2] - 0.001))])


async def test_the_log_line_gives_where_the_descent_settled_against_the_grasp_point(caplog):
    manipulator, robot, store = await started()
    holding(robot, store)
    landing_log(robot)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(True)
    [line] = [record.getMessage() for record in log.records]
    assert "landed [-0.3, +0.2, -0.1] cm from the grasp point" in line


# A step that fails once the descent has begun retreats: the gripper opens
# and the arm climbs straight back to the pre-grasp point.

RETREAT = [("grip", "left_gripper", 1.0, 0.0), ("move", PRE, DOWN[0])]


async def test_a_descent_that_does_not_settle_retreats():
    manipulator, robot, store = await started()
    unsettled = "still 8.0 mm / 0.02 rad from the target, 2.0 s after the planned end"
    robot.moves.extend([arrived(), move_failed(unsettled, "timeout")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, unsettled)
    assert robot.calls[-3] == ("move", GRASP, DOWN[0]) and robot.calls[-2:] == RETREAT


async def test_a_failed_close_retreats():
    manipulator, robot, store = await started()
    robot.grips.extend([gripped(1.0), grip_failed("the follower stopped reporting")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the follower stopped reporting")
    assert robot.calls[-3] == ("grip", "left_gripper", 0.0, 0.0) and robot.calls[-2:] == RETREAT


async def test_a_close_on_nothing_retreats_with_no_lift():
    manipulator, robot, store = await started()
    holding(robot, store, closed=0.01)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "gripper 'left_gripper' closed on nothing")
    assert robot.calls[-3] == ("grip", "left_gripper", 0.0, 0.0) and robot.calls[-2:] == RETREAT
    # The only moves are the reach and the retreat: no lift.
    assert robot.kinds().count("move") == 3


async def test_a_failed_lift_retreats():
    manipulator, robot, store = await started()
    holding(robot, store)
    robot.moves.extend([arrived(), arrived(), move_failed("the follower stopped reporting")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the follower stopped reporting")
    assert robot.calls[-3] == ("move", LIFT, DOWN[0]) and robot.calls[-2:] == RETREAT


async def test_a_slip_retreats():
    manipulator, robot, store = await started()
    holding(robot, store, after=0.0)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "item 'cup_1-t0' slipped while lifting")
    assert robot.calls[-3] == ("move", LIFT, DOWN[0]) and robot.calls[-2:] == RETREAT


async def test_jaws_that_close_further_during_the_lift_lost_the_item_above_the_hold_opening():
    manipulator, robot, store = await started()
    # The apple of the bowl run of 2026-10-24: the close stopped at 0.378 and
    # the jaws stood at 0.062 after the lift, above the 0.05 of an empty close.
    holding(robot, store, closed=0.378, after=0.062)
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "item 'cup_1-t0' slipped while lifting")
    assert robot.calls[-2:] == RETREAT


async def test_jaws_that_squeeze_a_little_further_during_the_lift_still_hold():
    manipulator, robot, store = await started()
    # The lemon of the same runs fell from 0.331 to 0.327 and stayed held.
    holding(robot, store, closed=0.331, after=0.327)
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(True)


async def test_the_most_the_opening_may_fall_during_the_lift_is_a_setting():
    manipulator, robot, store = await started(hold_drop_max=0.01)
    holding(robot, store, closed=0.425, after=0.412)
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(False, "item 'cup_1-t0' slipped while lifting")


async def test_no_snapshot_after_the_lift_fails_the_grab_and_retreats():
    manipulator, robot, store = await started()
    holding(robot, store)
    # The only snapshot after the lift is no newer than the one before it.
    snapshot_after_the_lift(robot, store, limb_states_message(1.0, left_gripper=0.3))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "no limb_states snapshot after the lift")
    assert robot.calls[-2:] == RETREAT


async def test_a_retreat_that_cannot_open_adds_why_and_moves_no_further(caplog):
    manipulator, robot, store = await started()
    robot.grips.extend([gripped(1.0), gripped(0.01), grip_failed("the follower stopped reporting")])
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(
        False, "gripper 'left_gripper' closed on nothing; the retreat to the pre-grasp point failed: the follower stopped reporting"
    )
    assert robot.calls[-1] == ("grip", "left_gripper", 1.0, 0.0)
    assert "the retreat stopped short: the follower stopped reporting" in log.records[-1].getMessage()


async def test_a_retreat_whose_climb_fails_adds_why():
    manipulator, robot, store = await started()
    holding(robot, store, closed=0.01)
    robot.moves.extend([arrived(), arrived(), move_refused("the side is busy")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "gripper 'left_gripper' closed on nothing; the retreat to the pre-grasp point failed: the side is busy")
    assert robot.calls[-2:] == RETREAT


async def test_a_step_a_stop_ended_retreats_from_nothing():
    """The stop service ends a goal as cancelled whoever called it: the arm
    stays where the stop left it."""
    manipulator, robot, store = await started()
    robot.moves.extend([arrived(), move_stopped("stopped: the operator asked")])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "stopped: the operator asked")
    assert robot.calls[-1] == ("move", GRASP, DOWN[0])


async def test_a_cancel_during_the_retreat_raises_cancelled():
    manipulator, robot, store = await started()
    holding(robot, store, closed=0.01)
    cancel = CancelToken()
    robot.on_call = lambda call: cancel.cancel("aborted: operator", signalled=False) if call == RETREAT[0] else None
    with pytest.raises(Cancelled):
        await manipulator.grab(cup(), LEFT, 0.0, cancel)
    assert robot.calls[-1] == RETREAT[0]


async def test_a_step_that_fails_before_the_descent_retreats_from_nothing():
    manipulator, robot, store = await started()
    robot.grips.append(GripResult(MoveEnd.REFUSED, "a leader streams the gripper", "leader_streaming"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "a leader streams the gripper")
    assert robot.kinds() == ["check", "check", "check", "grip"]


async def test_a_snapshot_after_the_lift_is_waited_for():
    manipulator, robot, store = await started(wait_s=10.0)
    robot.grips.extend([gripped(1.0), gripped(0.3)])
    lifted = asyncio.Event()
    robot.on_call = lambda call: lifted.set() if robot.is_lift(call) else None
    grab = asyncio.create_task(manipulator.grab(cup(), LEFT, 0.0, CancelToken()))
    await asyncio.wait_for(lifted.wait(), 10.0)
    # The grab is now waiting for a snapshot newer than the first one.
    store.add(limb_states_message(1.0, left_gripper=0.0))
    store.add(limb_states_message(2.0, left_gripper=0.3))
    assert await asyncio.wait_for(grab, 10.0) == Outcome(True)


async def test_the_first_orientation_whose_three_checks_plan_is_the_one_moved_in():
    manipulator, robot, store = await started()
    holding(robot, store)
    robot.checks.extend([MoveCheck(True, ""), MoveCheck(False, "goal pose not planned within 0.010 m")])
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(True)
    checks = [call for call in robot.calls if call[0] == "check"]
    assert [call[2] for call in checks] == [DOWN[0], DOWN[0], DOWN[1], DOWN[1], DOWN[1]]
    assert {call[2] for call in robot.calls if call[0] == "move"} == {DOWN[1]}


async def test_no_orientation_that_plans_fails_with_the_backbones_last_words():
    manipulator, robot, store = await started()
    robot.checks.extend([MoveCheck(False, f"refusal {i}") for i in range(len(DOWN))])
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "no plan reaches item 'cup_1-t0' with gripper 'left_gripper': refusal 7")
    assert robot.kinds() == ["check"] * len(DOWN)


async def test_a_refused_pre_grasp_move_ends_the_grab_with_its_message():
    manipulator, robot, store = await started()
    robot.moves.append(move_refused("the side is busy"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the side is busy")
    assert robot.kinds() == ["check", "check", "check", "grip", "move"]


async def test_a_cancel_between_two_moves_raises_cancelled_out_of_the_grab(caplog):
    manipulator, robot, store = await started()
    cancel = CancelToken()
    robot.on_call = lambda call: cancel.cancel("aborted: operator", signalled=False) if call == ("move", PRE, DOWN[0]) else None
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        with pytest.raises(Cancelled) as stopped:
            await manipulator.grab(cup(), LEFT, 0.0, cancel)
    assert (stopped.value.reason, stopped.value.signalled) == ("aborted: operator", False)
    assert robot.kinds()[-1] == "move" and len([k for k in robot.kinds() if k == "move"]) == 1
    assert log.records[-1].getMessage().endswith("; stopped: aborted: operator")


async def test_a_cancel_during_a_check_raises_cancelled_before_any_move():
    manipulator, robot, _ = await started()
    cancel = CancelToken()
    robot.on_call = lambda call: cancel.cancel("cancelled by the caller", signalled=True)
    with pytest.raises(Cancelled):
        await manipulator.grab(cup(), LEFT, 0.0, cancel)
    assert robot.kinds() == ["check"]


async def test_a_grab_by_pose_is_not_lowered_and_its_orientation_is_the_only_one_tried():
    manipulator, robot, store = await started()
    point, q = (0.5, 0.1, 0.7), (0.0, 0.0, 0.0, 1.0)
    robot.checks.append(MoveCheck(False, "goal pose not planned within 0.010 m"))
    outcome = await manipulator.grab(cup(point, q, grasp_point=True), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "no plan reaches item 'cup_1-t0' with gripper 'left_gripper': goal pose not planned within 0.010 m")
    # The approach of the identity orientation is +Z: the pre-grasp point
    # stands below the grasp point.
    assert robot.calls == [("check", (0.5, 0.1, 0.7 - CLEARANCE_M), q)]


async def test_a_grab_by_pose_without_an_orientation_tries_the_down_rolls_at_the_point_itself():
    manipulator, robot, store = await started()
    robot.checks.extend([MoveCheck(False, "no")] * len(DOWN))
    await manipulator.grab(cup((0.5, 0.1, 0.7), grasp_point=True), LEFT, 0.0, CancelToken())
    assert [call[2] for call in robot.calls] == list(DOWN)
    for call in robot.calls:
        assert call[1] == pytest.approx((0.5, 0.1, 0.7 + CLEARANCE_M))


async def test_a_grab_with_a_gripper_limb_states_does_not_name_fails_before_moving():
    manipulator, robot, store = await started()
    outcome = await manipulator.grab(cup(), Gripper("third_gripper", "left_arm"), 0.0, CancelToken())
    assert outcome == Outcome(False, "limb_states names no gripper 'third_gripper'")
    assert robot.calls == []


async def test_it_is_unavailable_until_started_and_until_a_snapshot_arrives():
    manipulator = ScriptedManipulator(CentreProposer())
    assert manipulator.why_unavailable() == NOT_STARTED
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(False, NOT_STARTED)
    manipulator, robot, store = await started(opening=None)
    assert manipulator.why_unavailable() == NO_SNAPSHOT
    assert await manipulator.drop(LEFT, CancelToken()) == Outcome(False, NO_SNAPSHOT)
    store.add(limb_states_message(1.0, left_gripper=1.0))
    assert manipulator.why_unavailable() == ""
    assert robot.calls == []


async def test_a_drop_opens_the_gripper_where_it_is():
    manipulator, robot, _ = await started()
    assert await manipulator.drop(LEFT, CancelToken()) == Outcome(True)
    assert robot.calls == [("grip", "left_gripper", 1.0, 0.0)]
    robot.grips.append(grip_failed("the follower stopped reporting"))
    assert await manipulator.drop(LEFT, CancelToken()) == Outcome(False, "the follower stopped reporting")


async def test_a_place_with_the_jaws_closed_on_air_fails_before_any_move():
    manipulator, robot, _ = await started(opening=0.02)
    outcome = await manipulator.place(LEFT, Pose((0.6, 0.0, 0.75), (0.0, 1.0, 0.0, 0.0)), CancelToken())
    assert outcome == Outcome(False, "gripper 'left_gripper' holds nothing now: its jaws stand at 0.02, closed on air; drop to let it go")
    assert robot.calls == []


async def test_a_place_checks_its_orientation_alone_opens_climbs_back_and_reports_where_it_settled():
    manipulator, robot, _ = await started()
    q, point = (0.0, 1.0, 0.0, 0.0), (0.6, 0.0, 0.75)
    above = (0.6, 0.0, 0.75 + (CLEARANCE_M + HELD_ITEM_DEPTH_M))
    robot.moves.extend([arrived(), arrived((0.6, 0.0, 0.752), q)])
    outcome = await manipulator.place(LEFT, Pose(point, q), CancelToken())
    assert outcome == Outcome(True, "", (0.6, 0.0, 0.752))
    assert robot.calls == [
        ("check", above, q), ("check", point, q),
        ("move", above, q), ("move", point, q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", above, q),
    ]


# A place lowers the item from the pre-place point (0.90 m: the clearance
# and the held item's depth above the place point, 0.75 m); an arm that
# stops short on the way down has set it down. Its descent fails with
# `timeout`, and limb_states says where it stands.

PLACE_Q = (0.0, 1.0, 0.0, 0.0)
PLACE_POINT = (0.6, 0.0, 0.75)
PLACE_ABOVE = (0.6, 0.0, 0.75 + (CLEARANCE_M + HELD_ITEM_DEPTH_M))
SHORT = "still 32.1 mm / 0.05 rad from the target, 2.0 s after the planned end"


async def place_stopping_at(stopped_at, descent: MoveResult = move_failed(SHORT, "timeout")):
    """A place whose descent ends as `descent` says, with the arm stopped
    at `stopped_at` in the snapshot the backbone publishes during the
    descent, and the robot."""
    manipulator, robot, store = await started()

    def on_call(call: tuple) -> None:
        if call == ("move", PLACE_POINT, PLACE_Q):
            store.add(limb_states_message(2.0, left_arm=(stopped_at, PLACE_Q), left_gripper=0.3))

    robot.on_call = on_call
    robot.moves.extend([arrived(), descent])
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    return outcome, robot


async def test_a_place_that_stops_short_on_the_way_down_releases_there_and_climbs_back(caplog):
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome, robot = await place_stopping_at((0.6, 0.0, 0.7821))
    assert outcome == Outcome(True, "", (0.6, 0.0, 0.7821))
    assert robot.calls[-3:] == [("move", PLACE_POINT, PLACE_Q), ("grip", "left_gripper", 1.0, 0.0), ("move", PLACE_ABOVE, PLACE_Q)]
    assert [record.getMessage() for record in log.records] == [
        f"place with gripper 'left_gripper' stopped 3.2 cm short of the place point on the way down ({SHORT}): it releases there"
    ]


async def test_a_place_whose_item_meets_the_floor_right_under_the_pre_place_point_releases_there():
    # A tall item's bottom meets a bowl's floor 0.8 cm below the pre-place
    # point, and the arm tilts a little against it: the item rests there.
    outcome, robot = await place_stopping_at((0.6, 0.012, 0.892))
    assert outcome == Outcome(True, "", (0.6, 0.012, 0.892))
    assert robot.kinds()[-2:] == ["grip", "move"]


async def test_a_place_that_stops_at_the_pre_place_point_itself_releases_there():
    outcome, robot = await place_stopping_at(PLACE_ABOVE)
    assert outcome == Outcome(True, "", PLACE_ABOVE)
    assert "grip" in robot.kinds()


async def test_a_place_whose_item_slides_off_what_is_under_it_releases_where_it_rests():
    # The item's bottom meets a round item already in the container and
    # slides 4 cm off its top before it rests.
    outcome, robot = await place_stopping_at((0.6, 0.04, 0.80))
    assert outcome == Outcome(True, "", (0.6, 0.04, 0.80))
    assert robot.kinds()[-2:] == ["grip", "move"]


async def test_a_place_pushed_over_5_cm_off_the_line_down_or_above_the_pre_place_point_fails_with_no_release():
    outcome, robot = await place_stopping_at((0.6, 0.06, 0.78))
    assert outcome == Outcome(False, SHORT)
    assert "grip" not in robot.kinds()
    outcome, robot = await place_stopping_at((0.6, 0.0, 0.96))
    assert outcome == Outcome(False, SHORT)
    assert "grip" not in robot.kinds()


async def test_a_place_whose_high_pre_place_point_has_no_plan_stands_at_the_clearance_alone(caplog):
    manipulator, robot, _ = await started()
    robot.checks.append(MoveCheck(False, "no line plan fits the limits", 0.0))
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    low = (0.6, 0.0, 0.75 + CLEARANCE_M)
    assert outcome == Outcome(True, "", PLACE_POINT)
    assert robot.calls == [
        ("check", PLACE_ABOVE, PLACE_Q), ("check", low, PLACE_Q), ("check", PLACE_POINT, PLACE_Q),
        ("move", low, PLACE_Q), ("move", PLACE_POINT, PLACE_Q), ("grip", "left_gripper", 1.0, 0.0), ("move", low, PLACE_Q),
    ]
    assert [record.getMessage() for record in log.records] == [
        "place with gripper 'left_gripper': no plan 15.0 cm above the pose: the pre-place point stands 7.5 cm above it"
    ]


async def test_a_place_whose_descent_a_stop_ended_fails_with_no_release():
    outcome, robot = await place_stopping_at((0.6, 0.0, 0.78), move_stopped("stopped: the operator asked"))
    assert outcome == Outcome(False, "stopped: the operator asked")
    assert "grip" not in robot.kinds()


async def test_a_place_whose_descent_fails_with_another_kind_than_timeout_fails_with_no_release():
    """Only a move that did not settle stopped short of its target: another
    failure says nothing of where the item is."""
    outcome, robot = await place_stopping_at((0.6, 0.0, 0.78), move_failed("the follower stopped reporting"))
    assert outcome == Outcome(False, "the follower stopped reporting")
    assert "grip" not in robot.kinds()


async def test_a_place_whose_climb_fails_still_places_and_says_so_in_the_log(caplog):
    manipulator, robot, _ = await started()
    q, point = (0.0, 1.0, 0.0, 0.0), (0.6, 0.0, 0.75)
    robot.moves.extend([arrived(), arrived(), move_refused("the side is busy")])
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.place(LEFT, Pose(point, q), CancelToken())
    assert outcome == Outcome(True, "", None)
    assert [record.getMessage() for record in log.records] == [
        "place with gripper 'left_gripper' released the item, but the climb back to the pre-place point failed: the side is busy"
    ]


async def test_a_place_without_an_orientation_tries_the_down_rolls():
    # Each roll's high pre-place point, then each roll's low one.
    manipulator, robot, _ = await started()
    robot.checks.extend([MoveCheck(False, "no plan")] * (2 * len(DOWN)))
    outcome = await manipulator.place(LEFT, Pose((0.6, 0.0, 0.75)), CancelToken())
    assert outcome == Outcome(False, "no plan reaches the pose [0.600, 0.000, 0.750] with gripper 'left_gripper': no plan")
    assert [call[2] for call in robot.calls] == list(DOWN) + list(DOWN)
    assert {round(call[1][2], 6) for call in robot.calls} == {0.9, 0.825}
    zero = await manipulator.place(LEFT, Pose((0.6, 0.0, 0.75), (0.0, 0.0, 0.0, 0.0)), CancelToken())
    assert zero == Outcome(False, "the orientation [0.0, 0.0, 0.0, 0.0] is not a rotation")


# Properties over any pattern of backbone answers: every check, move and
# grip answer drawn, a move or grip that does not complete ended by a stop
# or failed with a kind, a snapshot published or not after each move, and a
# cancel at any call or none.

HOLD = 0.05
DROP = 0.05


def mostly(one_in: int):
    """True but one time in `one_in`, so a drawn sequence often runs to its
    end and every way it ends is drawn."""
    return st.integers(min_value=1, max_value=one_in).map(lambda n: n != 1)


checks = st.builds(MoveCheck, mostly(8), st.sampled_from(["", "no plan"]), st.just(1.0))
# How a failed move or grip fails: the kinds the backbone gives.
move_kinds = st.sampled_from(["timeout", "attempt_failed", "unreachable", "leader_streaming"])
# Openings at the hold opening and around it, and anywhere.
openings = st.one_of(st.sampled_from([0.0, HOLD - 1e-9, HOLD, 0.3, 1.0]), st.floats(min_value=0.0, max_value=1.0))
# A snapshot after a move, captured before the latest one, with it, or after.
snapshots = st.one_of(st.none(), st.tuples(st.sampled_from([0.5, 1.0, 2.0, 3.0]), openings))
# Whether a failed move or grip ended as cancelled, by a stop.
stops = mostly(4).map(lambda go_on: not go_on)
cancels = mostly(25).map(lambda go_on: not go_on)


def answering_drawn(robot: FakeRobot, store: LimbStateStore, data, cancel: CancelToken, record: dict) -> None:
    """Every answer of `robot` drawn from `data`, a snapshot drawn after
    each move, and a cancel drawn at each call; `record` keeps what a
    property reads afterwards, `answers` each answer in call order."""
    record["answers"] = []

    def on_call(call: tuple) -> None:
        kind = call[0]
        if kind == "check":
            robot.checks.append(data.draw(checks))
        elif kind == "move":
            if data.draw(mostly(12), label="completed"):
                robot.moves.append(arrived(call[1], call[2]))
            elif data.draw(stops, label="stopped"):
                robot.moves.append(MoveResult(MoveEnd.CANCELLED, "stopped", "", call[1], call[2]))
            else:
                robot.moves.append(move_failed("failed", data.draw(move_kinds)))
            record["latest_before_last_move"] = store.latest.timestamp
            snapshot = data.draw(snapshots)
            if snapshot is not None:
                message = limb_states_message(snapshot[0], left_gripper=snapshot[1])
                asyncio.get_running_loop().call_soon(store.add, message)
            record["after_last_move"] = snapshot
        else:
            if data.draw(mostly(12), label="completed"):
                # A completed move_gripper result always carries its opening.
                grip = gripped(data.draw(openings))
            elif data.draw(stops, label="stopped"):
                grip = GripResult(MoveEnd.CANCELLED, "stopped", "", False, data.draw(openings))
            else:
                grip = grip_failed("failed", data.draw(move_kinds))
            robot.grips.append(grip)
            record["last_grip"] = grip
        queued = {"check": robot.checks, "move": robot.moves, "grip": robot.grips}[kind]
        record["answers"].append(queued[-1])
        if not cancel.cancelled and data.draw(cancels, label="cancel here"):
            cancel.cancel("aborted: drawn", signalled=False)

    robot.on_call = on_call


@settings(max_examples=400)
@given(st.data())
def test_a_grab_returns_an_outcome_or_raises_cancelled_holds_only_when_both_openings_pass_and_retreats(data):
    async def scenario():
        manipulator, robot, store = await started(hold_opening_min=HOLD, hold_drop_max=DROP)
        cancel, record = CancelToken(), {}
        answering_drawn(robot, store, data, cancel, record)
        try:
            outcome = await manipulator.grab(cup(), LEFT, 5.0, cancel)
        except Cancelled:
            event("cancelled")
            assert cancel.cancelled
            return
        assert not cancel.cancelled
        assert isinstance(outcome, Outcome)
        if not outcome.success:
            assert outcome.message
            event(f"failed: {outcome.message}")
            assert_a_failed_grab_retreats_only_once_down_and_unstopped(robot, record["answers"], outcome)
            return
        event("held")
        # A hold: the whole sequence ran, the close stopped at or above the
        # hold opening, and so did the opening after the lift, no more than
        # the drop below the close's.
        assert robot.kinds()[-5:] == ["grip", "move", "move", "grip", "move"]
        closed = record["last_grip"].final_opening
        assert robot.calls[-2][2] == 0.0 and closed >= HOLD
        stamp, after = record["after_last_move"]
        assert stamp > record["latest_before_last_move"] and after >= HOLD and closed - after <= DROP

    asyncio.run(scenario())


def assert_a_failed_grab_retreats_only_once_down_and_unstopped(robot: FakeRobot, answers: list, outcome: Outcome) -> None:
    """A failed grab: an answer that a stop ended is the last call. Else,
    one that failed before the descent was sent sends nothing after it; one
    that failed once it was sent ends with the retreat, the gripper opened
    and the arm sent back to the pre-grasp point, and the outcome says why
    a retreat stopped short."""
    calls = robot.calls
    if any(answer.cancelled for answer in answers if not isinstance(answer, MoveCheck)):
        event("a stop ended a step")
        assert answers[-1].cancelled
        return
    moves = [index for index, call in enumerate(calls) if call[0] == "move"]
    if len(moves) < 2:
        assert robot.kinds().count("grip") <= 1
        return
    opened = max(index for index, call in enumerate(calls) if call[0] == "grip" and call[2] == 1.0)
    assert opened > moves[1]
    retreat = calls[opened:]
    pre = calls[moves[0]]
    assert retreat in ([calls[opened]], [calls[opened], pre])
    stopped_short = len(retreat) == 1 or not answers[-1].completed
    event("the retreat stopped short" if stopped_short else "retreated")
    assert ("the retreat to the pre-grasp point failed" in outcome.message) == stopped_short


@given(st.data())
def test_a_drop_and_a_place_return_an_outcome_or_raise_cancelled(data):
    async def scenario():
        manipulator, robot, store = await started()
        cancel = CancelToken()
        answering_drawn(robot, store, data, cancel, {})
        orientation = data.draw(st.one_of(st.none(), st.sampled_from(DOWN)))
        try:
            dropped = await manipulator.drop(LEFT, cancel)
            placed = await manipulator.place(LEFT, Pose((0.6, 0.0, 0.75), orientation), cancel)
        except Cancelled:
            assert cancel.cancelled
            return
        assert isinstance(dropped, Outcome) and isinstance(placed, Outcome)

    asyncio.run(scenario())
