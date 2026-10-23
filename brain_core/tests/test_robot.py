"""The backbone proxy against a scripted limb_motion client: every way a
move ends, from the goal the brain fires to the outcome it reads."""

import asyncio
from types import SimpleNamespace

from fake_peppygen import CheckArmMoveRequest, FakeLimbAction, FakeService, MoveArmRequest, MoveGripperRequest, ResultStatus

from brain_core.robot import GripResult, MoveCheck, MoveResult, Robot

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
    robot, move_arm, move_gripper, _ = backbone_with_check(**kwargs)
    return robot, move_arm, move_gripper


def backbone_with_check(**kwargs):
    """A Robot on scripted limb_motion clients, and the three clients."""
    move_arm = FakeLimbAction("move_arm", MoveArmRequest)
    move_gripper = FakeLimbAction("move_gripper", MoveGripperRequest)
    check_arm_move = FakeService("check_arm_move", CheckArmMoveRequest)
    robot = Robot("runner", move_arm, move_gripper, check_arm_move, "standard", **kwargs)
    return robot, move_arm, move_gripper, check_arm_move


async def in_flight(robot: Robot) -> None:
    while robot.moves_in_flight == 0:
        await asyncio.sleep(0)


def arm_result(success: bool, message: str):
    return SimpleNamespace(
        success=success, message=message, final_position=[0.51, 0.1, 0.69], final_orientation=[0.0, 0.0, 0.0, 1.0], action_time=0.4
    )


async def test_an_accepted_move_returns_the_backbones_result():
    robot, move_arm, move_gripper = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=1.5, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    assert goal.request.arm_name == "left_arm"
    assert goal.request.position == [0.5, 0.1, 0.7] and goal.request.duration_s == 1.5
    assert {name: getattr(goal.request, name) for name in BARS} == BARS
    assert goal.qos == "standard"
    goal.complete(arm_result(True, ""))
    assert await asyncio.wait_for(move, WAIT) == MoveResult(True, "", (0.51, 0.1, 0.69), (0.0, 0.0, 0.0, 1.0))

    grip = asyncio.create_task(robot.set_gripper("left_gripper", 0.2, max_effort=5.0, settle_timeout_s=0.8))
    goal = await asyncio.wait_for(move_gripper.next_goal(), WAIT)
    assert (goal.request.gripper_name, goal.request.opening, goal.request.max_effort) == ("left_gripper", 0.2, 5.0)
    assert goal.request.settle_timeout_s == 0.8
    goal.complete(SimpleNamespace(success=True, message="", final_opening=0.21, action_time=0.3))
    assert await asyncio.wait_for(grip, WAIT) == GripResult(True, "", 0.21)


async def test_a_move_the_backbone_refuses_or_fails_says_why():
    robot, move_arm, _ = backbone()
    move_arm.refuse_next("unknown arm 'third_arm'")
    assert await robot.move_arm("third_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS) == MoveResult(False, "unknown arm 'third_arm'")

    move_arm.refuse_next("")
    result = await robot.move_arm("third_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS)
    assert (result.success, result.message) == (False, "move_arm was refused by the backbone")

    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    goal.complete(arm_result(False, "no plan reaches the target"))
    result = await asyncio.wait_for(move, WAIT)
    assert (result.success, result.message) == (False, "no plan reaches the target")


async def test_a_move_that_cannot_be_sent_says_why():
    robot, move_arm, _ = backbone()
    move_arm.fail_next_send(ConnectionError("no route to the backbone"))
    result = await robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS)
    assert result == MoveResult(False, "move_arm could not be sent: ConnectionError('no route to the backbone')")
    assert robot.moves_in_flight == 0


async def test_a_stop_cancels_the_move_in_flight_and_reports_it_cancelled():
    robot, move_arm, _ = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    stop = asyncio.create_task(robot.stop())
    await asyncio.wait_for(goal.cancel_asked.wait(), WAIT)
    goal.complete_cancelled(arm_result(False, "stopped on request"))
    await asyncio.wait_for(stop, WAIT)
    result = await asyncio.wait_for(move, WAIT)
    assert (result.success, result.message, result.final_position) == (False, "stopped on request", (0.51, 0.1, 0.69))
    assert result.cancelled
    # Nothing is left in flight: a second stop has nothing to cancel.
    assert robot.moves_in_flight == 0
    await asyncio.wait_for(robot.stop(), WAIT)


async def test_a_move_the_backbone_stops_reports_the_stops_message_without_a_cancel():
    """The backbone's stop service ends a move as cancelled with no cancel
    from the brain: the move fails with the stop's message, so a backend
    sees it fail and does not go on."""
    robot, move_arm, _ = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    goal.complete_cancelled(arm_result(False, "stopped: the operator asked"))
    result = await asyncio.wait_for(move, WAIT)
    assert (result.success, result.message) == (False, "stopped: the operator asked")
    assert result.cancelled
    assert not goal.cancel_asked.is_set()
    assert robot.moves_in_flight == 0


async def test_a_grip_the_backbone_stops_is_reported_cancelled_and_one_that_fails_is_not():
    robot, _, move_gripper = backbone()
    grip = asyncio.create_task(robot.set_gripper("left_gripper", 0.0, max_effort=0.0, settle_timeout_s=2.0))
    goal = await asyncio.wait_for(move_gripper.next_goal(), WAIT)
    goal.complete_cancelled(SimpleNamespace(success=False, message="", final_opening=0.4, action_time=0.2))
    assert await asyncio.wait_for(grip, WAIT) == GripResult(False, "move_gripper was cancelled", 0.4, cancelled=True)

    grip = asyncio.create_task(robot.set_gripper("left_gripper", 0.0, max_effort=0.0, settle_timeout_s=2.0))
    goal = await asyncio.wait_for(move_gripper.next_goal(), WAIT)
    goal.complete(SimpleNamespace(success=False, message="overran", final_opening=0.4, action_time=0.2))
    assert await asyncio.wait_for(grip, WAIT) == GripResult(False, "overran", 0.4)


async def test_a_move_without_a_result_in_time_is_cancelled_and_reported():
    robot, move_arm, _ = backbone(result_timeout_s=0.5)
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    # The backbone never answers within the result timeout: the proxy gives
    # up on the result and cancels the goal so the arm does not keep going
    # unwatched.
    goal.lose_result(TimeoutError("no result"))
    result = await asyncio.wait_for(move, WAIT)
    assert result == MoveResult(False, "move_arm gave no result: TimeoutError('no result')")
    assert goal.result_timeouts == [0.5]
    assert goal.cancel_asked.is_set()


async def test_a_goal_that_ends_abandoned_names_how_it_ended():
    robot, move_arm, _ = backbone()
    move = asyncio.create_task(robot.move_arm("left_arm", POSITION, ORIENTATION, duration_s=0.0, **BARS))
    goal = await asyncio.wait_for(move_arm.next_goal(), WAIT)
    goal.end(ResultStatus.ABANDONED)
    assert await asyncio.wait_for(move, WAIT) == MoveResult(False, "move_arm ended as abandoned")


def check_answer(success: bool, message: str, duration_s: float):
    return SimpleNamespace(success=success, message=message, duration_s=duration_s)


async def test_a_check_asks_with_the_move_arm_fields_and_reads_the_plan():
    robot, _, _, check = backbone_with_check()
    check.enqueue(check_answer(True, "", 1.25))
    planned = await robot.check_arm_move("left_arm", POSITION, ORIENTATION, plan_position_tolerance_m=0.01)
    assert planned == MoveCheck(True, "", 1.25)
    [request] = check.polled
    assert request == CheckArmMoveRequest(
        arm_name="left_arm", position=[0.5, 0.1, 0.7], orientation=[0.0, 0.0, 0.0, 1.0], duration_s=0.0,
        plan_position_tolerance_m=0.01, plan_orientation_tolerance_rad=0.0,
    )
    # A check moves nothing: no goal is in flight for a stop to cancel.
    assert robot.moves_in_flight == 0


async def test_a_check_the_backbone_refuses_carries_its_words():
    robot, _, _, check = backbone_with_check()
    check.enqueue(check_answer(False, "goal pose not planned within 0.010 m", 0.0))
    refused = await robot.check_arm_move("left_arm", POSITION, ORIENTATION)
    assert refused == MoveCheck(False, "goal pose not planned within 0.010 m", 0.0)


async def test_a_check_with_no_answer_is_reported_as_having_none():
    robot, _, _, check = backbone_with_check()
    check.enqueue(TimeoutError("no reply"))
    lost = await robot.check_arm_move("left_arm", POSITION, ORIENTATION)
    assert lost == MoveCheck(False, "check_arm_move gave no answer: TimeoutError('no reply')", 0.0)
