"""The backbone proxy against a scripted limb_motion client: every way a
move ends, from the goal the brain fires to the outcome it reads."""

import asyncio
from types import SimpleNamespace

import pytest
from fake_peppygen import FakeLimbAction, MoveArmRequest, MoveGripperRequest, ResultStatus

from brain_core.robot import GripResult, MoveEnd, MoveResult, Robot

WAIT = 10.0
POSITION = (0.5, 0.1, 0.7)
ORIENTATION = (0.0, 0.0, 0.0, 1.0)
# The tolerances and settle timeout the moves below name.
BARS = {
    "plan_position_tolerance_m": 0.001,
    "plan_orientation_tolerance_rad": 0.01,
    "settle_position_tolerance_m": 0.02,
    "settle_orientation_tolerance_rad": 0.15,
    "settle_timeout_s": 2.0,
}


def backbone(**kwargs):
    """A Robot on scripted move_arm and move_gripper clients, and the two
    clients."""
    move_arm = FakeLimbAction("move_arm", MoveArmRequest)
    move_gripper = FakeLimbAction("move_gripper", MoveGripperRequest)
    return Robot("runner", move_arm, move_gripper, **kwargs), move_arm, move_gripper


async def in_flight(robot: Robot) -> None:
    while robot.moves_in_flight == 0:
        await asyncio.sleep(0)


def arm_result():
    return SimpleNamespace(final_position=[0.51, 0.1, 0.69], final_orientation=[0.0, 0.0, 0.0, 1.0], action_time=0.4)


async def test_a_completed_move_returns_where_the_limb_stands():
    robot, move_arm, move_gripper = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=1.5, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    assert goal.request.arm_name == "left_arm"
    assert goal.request.position == [0.5, 0.1, 0.7] and goal.request.duration_s == 1.5
    assert {name: getattr(goal.request, name) for name in BARS} == BARS
    # The client keeps every feedback message, up to peppylib's own bound.
    assert goal.feedback_bound is None
    goal.complete(arm_result())
    moved = await asyncio.wait_for(move, WAIT)
    assert moved == MoveResult(MoveEnd.COMPLETED, "", "", (0.51, 0.1, 0.69), (0.0, 0.0, 0.0, 1.0))
    assert moved.completed

    grip = asyncio.create_task(robot.set_gripper("left_gripper", 0.2, max_effort=5.0, settle_timeout_s=0.8))
    goal = await asyncio.wait_for(move_gripper.next_goal(), WAIT)
    assert (goal.request.gripper_name, goal.request.opening, goal.request.max_effort) == ("left_gripper", 0.2, 5.0)
    assert goal.request.settle_timeout_s == 0.8
    goal.complete(SimpleNamespace(settled=False, final_opening=0.21, action_time=0.3))
    # A gripper that reached its settle timeout still completes: `settled`
    # says so.
    assert await asyncio.wait_for(grip, WAIT) == GripResult(MoveEnd.COMPLETED, "", "", False, 0.21)


async def test_a_move_the_backbone_refuses_or_fails_says_why_with_its_kind():
    robot, move_arm, _ = backbone()
    move_arm.refuse_next("rejected", "unknown arm 'third_arm'")
    refused = await robot.move_arm("third_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS)
    assert refused == MoveResult(MoveEnd.REFUSED, "unknown arm 'third_arm'", "rejected")
    assert not refused.completed

    move_arm.refuse_next("leader_streaming", "a leader streams left_arm")
    refused = await robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS)
    assert (refused.end, refused.failure_kind) == (MoveEnd.REFUSED, "leader_streaming")

    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    goal.fail("unreachable", "no plan reaches the target")
    failed = await asyncio.wait_for(move, WAIT)
    # A failed goal gives no result: nothing says where the arm stands.
    assert failed == MoveResult(MoveEnd.FAILED, "no plan reaches the target", "unreachable")
    assert robot.moves_in_flight == 0


async def test_a_move_that_cannot_be_sent_says_why():
    robot, move_arm, _ = backbone()
    move_arm.fail_next_send(ConnectionError("no route to the backbone"))
    result = await robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS)
    assert result == MoveResult(MoveEnd.LOST, "move_arm could not be sent: ConnectionError('no route to the backbone')")
    assert robot.moves_in_flight == 0


async def test_a_stop_cancels_the_move_in_flight_and_reports_it_cancelled_with_its_reason():
    robot, move_arm, _ = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    stop = asyncio.create_task(robot.stop())
    await asyncio.wait_for(goal.cancel_asked.wait(), WAIT)
    goal.complete_cancelled("cancelled by its caller", arm_result())
    await asyncio.wait_for(stop, WAIT)
    result = await asyncio.wait_for(move, WAIT)
    assert (result.end, result.message, result.final_position) == (MoveEnd.CANCELLED, "cancelled by its caller", (0.51, 0.1, 0.69))
    # Nothing is left in flight: a second stop has nothing to cancel.
    assert robot.moves_in_flight == 0
    await asyncio.wait_for(robot.stop(), WAIT)


async def test_a_move_the_backbones_stop_ends_reports_the_stops_reason_without_a_cancel():
    """The backbone's stop service ends a move as cancelled with no cancel
    from the brain: the move ends cancelled with the stop's reason, so a
    backend sees it did not complete and does not go on."""
    robot, move_arm, move_gripper = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    goal.complete_cancelled("stopped: the operator asked", arm_result())
    result = await asyncio.wait_for(move, WAIT)
    assert (result.end, result.message) == (MoveEnd.CANCELLED, "stopped: the operator asked")
    assert not result.completed and not goal.cancel_asked.is_set()
    assert robot.moves_in_flight == 0

    grip = asyncio.create_task(robot.set_gripper("left_gripper", 1.0, max_effort=0.0, settle_timeout_s=1.0))
    goal = await asyncio.wait_for(move_gripper.next_goal(), WAIT)
    goal.complete_cancelled("", SimpleNamespace(settled=False, final_opening=0.6, action_time=0.2))
    # A cancel with no reason is named by the move.
    assert await asyncio.wait_for(grip, WAIT) == GripResult(MoveEnd.CANCELLED, "move_gripper was cancelled", "", False, 0.6)


async def test_a_move_without_a_result_in_time_is_cancelled_and_reported_lost():
    robot, move_arm, _ = backbone(result_timeout_s=0.5)
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    # The backbone never answers within the result timeout: the proxy gives
    # up on the result and cancels the goal so the arm does not keep going
    # unwatched.
    goal.lose_result(TimeoutError("no result"))
    result = await asyncio.wait_for(move, WAIT)
    assert result == MoveResult(MoveEnd.LOST, "move_arm gave no result: TimeoutError('no result')")
    assert goal.result_timeouts == [0.5]
    assert goal.cancel_asked.is_set()


@pytest.mark.parametrize("status", [ResultStatus.ABANDONED, ResultStatus.EXPIRED])
async def test_a_goal_that_ends_abandoned_or_expired_is_lost_and_names_how_it_ended(status):
    robot, move_arm, _ = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    goal.end(status)
    assert await asyncio.wait_for(move, WAIT) == MoveResult(MoveEnd.LOST, f"move_arm ended as {status.name.lower()}")
