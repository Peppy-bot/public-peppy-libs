"""The backbone proxy against a scripted limb_motion client: every way a
move ends, from the goal the brain fires to the outcome it reads."""

import asyncio
from dataclasses import fields
from types import SimpleNamespace

import pytest
from fake_peppygen import (
    CheckArmMoveRequest,
    ExecuteJointPathArm,
    ExecuteJointPathRequest,
    ExecuteJointPathWaypoint,
    FakeConsumedAction,
    FakeConsumedService,
    MoveArmRequest,
    MoveGripperRequest,
    PlanJointPathArm,
    PlanJointPathOrientationConstraint,
    PlanJointPathPositionConstraint,
    PlanCartesianPathRequest,
    PlanCartesianPathWaypoint,
    PlanJointPathRequest,
    ResultStatus,
    fake_bindings,
    fake_planner,
    planner_bindings,
)

from brain_core.bindings import PLANNER_MODULES
from brain_core.robot import (
    CartesianPath,
    GripResult,
    JointPlan,
    MoveCheck,
    MoveEnd,
    MoveResult,
    PathResult,
    PlannerModules,
    Robot,
    Waypoint,
)

WAIT = 10.0
POSITION = (0.5, 0.1, 0.7)
ORIENTATION = (0.0, 0.0, 0.0, 1.0)
# The tolerances and settle timeout the moves below name.
BARS = {
    "settle_position_tolerance_m": 0.02,
    "settle_orientation_tolerance_rad": 0.15,
    "settle_timeout_s": 2.0,
}
# The field a check names besides the pose.
PLAN = {"duration_s": 0.0}


def backbone(**kwargs):
    """A Robot on scripted move_arm and move_gripper clients, and the two
    clients."""
    robot, move_arm, move_gripper, _ = backbone_with_check(**kwargs)
    return robot, move_arm, move_gripper


def backbone_with_check(**kwargs):
    """A Robot on scripted limb_motion clients, and the three clients."""
    move_arm = FakeConsumedAction("move_arm", MoveArmRequest)
    move_gripper = FakeConsumedAction("move_gripper", MoveGripperRequest)
    check_arm_move = FakeConsumedService("check_arm_move", CheckArmMoveRequest)
    robot = Robot("runner", move_arm, move_gripper, check_arm_move, **kwargs)
    return robot, move_arm, move_gripper, check_arm_move


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
    assert not failed.completed and not failed.cancelled
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
    assert result.cancelled
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
    assert result.cancelled and not result.completed and not goal.cancel_asked.is_set()
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


def check_answer(reachable: bool, reason: str, duration_s: float):
    return SimpleNamespace(reachable=reachable, reason=reason, duration_s=duration_s)


async def test_a_check_asks_with_the_move_arm_fields_and_reads_the_plan():
    robot, _, _, check = backbone_with_check()
    check.enqueue(check_answer(True, "", 1.25))
    planned = await robot.check_arm_move("left_arm", POSITION, ORIENTATION, duration_s=1.5)
    assert planned == MoveCheck(True, "", 1.25)
    [request] = check.polled
    assert request == CheckArmMoveRequest(
        arm_name="left_arm", position=[0.5, 0.1, 0.7], orientation=[0.0, 0.0, 0.0, 1.0], duration_s=1.5,
    )
    # A check moves nothing: no goal is in flight for a stop to cancel.
    assert robot.moves_in_flight == 0


async def test_a_check_with_no_plan_carries_the_backbones_reason():
    robot, _, _, check = backbone_with_check()
    check.enqueue(check_answer(False, "goal pose not planned", 0.0))
    unreachable = await robot.check_arm_move("left_arm", POSITION, ORIENTATION, **PLAN)
    assert unreachable == MoveCheck(False, "goal pose not planned", 0.0)


async def test_a_check_that_fails_is_not_reachable_and_names_its_kind():
    robot, _, _, check = backbone_with_check()
    check.enqueue(check.Failure("leader_streaming", "a leader streams left_arm"))
    failed = await robot.check_arm_move("left_arm", POSITION, ORIENTATION, **PLAN)
    assert failed == MoveCheck(False, "check_arm_move failed with leader_streaming: a leader streams left_arm")


async def test_a_check_with_no_answer_is_reported_as_having_none():
    robot, _, _, check = backbone_with_check()
    check.enqueue(TimeoutError("no reply"))
    lost = await robot.check_arm_move("left_arm", POSITION, ORIENTATION, **PLAN)
    assert lost == MoveCheck(False, "check_arm_move gave no answer: TimeoutError('no reply')", 0.0)


def test_the_planner_modules_are_the_planner_fields_of_the_bindings():
    assert {field.name for field in fields(PlannerModules)} == set(PLANNER_MODULES)
    planner = fake_planner()
    assert PlannerModules.from_bindings(fake_bindings(**planner_bindings(planner))) == planner


def test_bindings_that_lack_a_planner_module_give_no_planner_modules():
    assert PlannerModules.from_bindings(fake_bindings()) is None
    lacking = {**planner_bindings(fake_planner()), "set_allowed_collision": None}
    assert PlannerModules.from_bindings(fake_bindings(**lacking)) is None


def test_a_robot_has_the_planner_only_with_its_modules_and_its_slot_bound():
    robot, _, _ = backbone()
    assert not robot.has_planner
    robot, _, _ = backbone(planner=fake_planner(producer=None))
    assert not robot.has_planner
    robot, _, _ = backbone(planner=fake_planner())
    assert robot.has_planner


# The planner's members, on a robot whose planner slot is bound. A plan is a
# goal in flight like a move: every way it ends, and a stop during it.

JOINTS = [0.1, -0.2, 0.3, 1.4, 0.5, -0.6, 0.7]
PLANNED = [0.2, -0.1, 0.35, 1.3, 0.4, -0.5, 0.8]
LINK = "openarm_left_tcp"
# What a transit plan names besides the arm, the link and the pose.
PLAN_BARS = dict(position_tolerance_m=0.001, orientation_tolerance_rad=0.01, planner="rrt_connect", refiner="none", allowed_planning_time_s=0.5)


def planned():
    """A Robot whose planner slot is bound, and the planner's modules."""
    planner = fake_planner()
    robot, _, _ = backbone(planner=planner)
    return robot, planner


def path_waypoint(joints, position=POSITION, orientation=ORIENTATION):
    return SimpleNamespace(joint_positions=list(joints), position=list(position), orientation=list(orientation))


def joint_plan_result(arm="left_arm", start=JOINTS, waypoints=(PLANNED,)):
    return SimpleNamespace(
        arms=[SimpleNamespace(arm_name=arm, start_joint_positions=list(start), waypoints=[path_waypoint(w) for w in waypoints])],
        map_stamp=3.5,
    )


async def test_a_plan_asks_for_the_link_at_the_pose_and_reads_its_start_and_waypoints():
    robot, planner = planned()
    plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS))
    goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
    assert goal.request == PlanJointPathRequest(
        arms=[PlanJointPathArm(arm_name="left_arm", start_joint_positions=[])],
        goal_joint_constraints=[],
        goal_position_constraints=[PlanJointPathPositionConstraint(link_name=LINK, position=[0.5, 0.1, 0.7], tolerance_m=0.001)],
        goal_orientation_constraints=[
            PlanJointPathOrientationConstraint(link_name=LINK, orientation=[0.0, 0.0, 0.0, 1.0], tolerance_rad=0.01, free_axis="none")
        ],
        path_orientation_constraints=[],
        planner="rrt_connect",
        refiner="none",
        allowed_planning_time_s=0.5,
    )
    goal.complete(joint_plan_result(waypoints=(JOINTS, PLANNED)))
    answer = await asyncio.wait_for(plan, WAIT)
    assert answer == JointPlan(
        MoveEnd.COMPLETED,
        start_joint_positions=tuple(JOINTS),
        waypoints=(Waypoint(tuple(JOINTS), POSITION, ORIENTATION), Waypoint(tuple(PLANNED), POSITION, ORIENTATION)),
    )
    assert answer.completed and robot.moves_in_flight == 0


async def test_a_plan_from_a_given_start_names_it():
    robot, planner = planned()
    plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, start_joint_positions=JOINTS, **PLAN_BARS))
    goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
    assert goal.request.arms == [PlanJointPathArm(arm_name="left_arm", start_joint_positions=JOINTS)]
    goal.complete(joint_plan_result())
    assert (await asyncio.wait_for(plan, WAIT)).completed


@pytest.mark.parametrize("kind, message", [
    ("start_in_collision", "openarm_left_ee_link1 stands 0.0388 m from the map, nearer than the clearance 0.0400 m"),
    ("no_path", "no path joined the goal in 0.5 s"),
    ("unreachable", "no joint positions meet the goal without a collision in 0.5 s"),
])
async def test_a_failed_plan_gives_the_planners_kind_and_words_and_no_path(kind, message):
    robot, planner = planned()
    plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS))
    goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
    goal.fail(kind, message)
    answer = await asyncio.wait_for(plan, WAIT)
    assert answer == JointPlan(MoveEnd.FAILED, message, kind)
    assert answer.words == f"{kind}: {message}" and answer.waypoints == ()


async def test_a_refused_plan_says_why_with_its_kind():
    robot, planner = planned()
    planner.plan_joint_path.refuse_next("rejected", "no link 'tcp' on arm 'left_arm'")
    answer = await robot.plan_joint_path("left_arm", "tcp", POSITION, ORIENTATION, **PLAN_BARS)
    assert answer == JointPlan(MoveEnd.REFUSED, "no link 'tcp' on arm 'left_arm'", "rejected")
    assert answer.words == "rejected: no link 'tcp' on arm 'left_arm'"


async def test_a_stop_cancels_a_plan_in_flight():
    robot, planner = planned()
    plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS))
    goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    stop = asyncio.create_task(robot.stop())
    await asyncio.wait_for(goal.cancel_asked.wait(), WAIT)
    goal.complete_cancelled("aborted: operator", None)
    await asyncio.wait_for(stop, WAIT)
    answer = await asyncio.wait_for(plan, WAIT)
    assert answer == JointPlan(MoveEnd.CANCELLED, "aborted: operator")
    assert answer.cancelled and robot.moves_in_flight == 0


async def test_a_plan_with_no_result_or_no_waypoints_for_the_arm_is_lost():
    robot, planner = planned()
    plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS))
    goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
    goal.lose_result(TimeoutError("no result"))
    assert await asyncio.wait_for(plan, WAIT) == JointPlan(MoveEnd.LOST, "plan_joint_path gave no result: TimeoutError('no result')")
    for result in (joint_plan_result(arm="right_arm"), joint_plan_result(waypoints=())):
        plan = asyncio.create_task(robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS))
        goal = await asyncio.wait_for(planner.plan_joint_path.next_goal(), WAIT)
        goal.complete(result)
        assert await asyncio.wait_for(plan, WAIT) == JointPlan(MoveEnd.LOST, "plan_joint_path answered no waypoints for arm 'left_arm'")


async def test_a_planner_member_called_without_the_planner_is_a_fault():
    robot, _, _ = backbone()
    with pytest.raises(RuntimeError, match="does not consume the motion planner"):
        await robot.plan_joint_path("left_arm", LINK, POSITION, ORIENTATION, **PLAN_BARS)


GRASP = (0.5, 0.1, 0.625)
LIFTED = (0.5, 0.1, 0.775)


def cartesian_result(waypoints=(PLANNED,), fraction=1.0, start=JOINTS):
    return SimpleNamespace(
        start_joint_positions=list(start), waypoints=[path_waypoint(w) for w in waypoints], fraction=fraction, map_stamp=None,
    )


async def test_a_cartesian_path_asks_for_the_lines_through_the_poses_and_reads_how_far_it_goes():
    robot, planner = planned()
    path = asyncio.create_task(
        robot.plan_cartesian_path("left_arm", LINK, [(GRASP, ORIENTATION), (LIFTED, ORIENTATION)], avoid_collisions=False, start_joint_positions=PLANNED)
    )
    goal = await asyncio.wait_for(planner.plan_cartesian_path.next_goal(), WAIT)
    assert goal.request == PlanCartesianPathRequest(
        arm_name="left_arm",
        start_joint_positions=PLANNED,
        link_name=LINK,
        waypoints=[
            PlanCartesianPathWaypoint(position=[0.5, 0.1, 0.625], orientation=[0.0, 0.0, 0.0, 1.0]),
            PlanCartesianPathWaypoint(position=[0.5, 0.1, 0.775], orientation=[0.0, 0.0, 0.0, 1.0]),
        ],
        avoid_collisions=False,
    )
    goal.complete(cartesian_result(waypoints=(JOINTS, PLANNED), fraction=1.0, start=PLANNED))
    answer = await asyncio.wait_for(path, WAIT)
    assert answer == CartesianPath(
        MoveEnd.COMPLETED,
        start_joint_positions=tuple(PLANNED),
        waypoints=(Waypoint(tuple(JOINTS), POSITION, ORIENTATION), Waypoint(tuple(PLANNED), POSITION, ORIENTATION)),
        fraction=1.0,
    )
    assert robot.moves_in_flight == 0


async def test_a_checked_cartesian_path_from_where_the_arm_stands_that_stops_short_gives_its_fraction():
    robot, planner = planned()
    path = asyncio.create_task(robot.plan_cartesian_path("left_arm", LINK, [(GRASP, ORIENTATION)], avoid_collisions=True))
    goal = await asyncio.wait_for(planner.plan_cartesian_path.next_goal(), WAIT)
    assert goal.request.avoid_collisions and goal.request.start_joint_positions == []
    goal.complete(cartesian_result(fraction=0.38))
    answer = await asyncio.wait_for(path, WAIT)
    assert answer.completed and answer.fraction == 0.38
    # Not even the first step valid: a completed answer with no waypoints.
    path = asyncio.create_task(robot.plan_cartesian_path("left_arm", LINK, [(GRASP, ORIENTATION)], avoid_collisions=True))
    goal = await asyncio.wait_for(planner.plan_cartesian_path.next_goal(), WAIT)
    goal.complete(cartesian_result(waypoints=(), fraction=0.0))
    assert await asyncio.wait_for(path, WAIT) == CartesianPath(MoveEnd.COMPLETED, start_joint_positions=tuple(JOINTS))


async def test_a_cartesian_path_from_a_start_in_collision_fails_with_the_planners_words():
    robot, planner = planned()
    path = asyncio.create_task(robot.plan_cartesian_path("left_arm", LINK, [(GRASP, ORIENTATION)], avoid_collisions=True))
    goal = await asyncio.wait_for(planner.plan_cartesian_path.next_goal(), WAIT)
    goal.fail("start_in_collision", "openarm_left_ee_link2 stands 0.0239 m from the map")
    answer = await asyncio.wait_for(path, WAIT)
    assert answer == CartesianPath(MoveEnd.FAILED, "openarm_left_ee_link2 stands 0.0239 m from the map", "start_in_collision")
    planner.plan_cartesian_path.refuse_next("rejected", "no waypoints")
    refused = await robot.plan_cartesian_path("left_arm", LINK, [], avoid_collisions=False)
    assert refused == CartesianPath(MoveEnd.REFUSED, "no waypoints", "rejected")


async def test_a_stop_cancels_a_cartesian_path_in_flight():
    robot, planner = planned()
    path = asyncio.create_task(robot.plan_cartesian_path("left_arm", LINK, [(GRASP, ORIENTATION)], avoid_collisions=True))
    goal = await asyncio.wait_for(planner.plan_cartesian_path.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    stop = asyncio.create_task(robot.stop())
    await asyncio.wait_for(goal.cancel_asked.wait(), WAIT)
    goal.complete_cancelled("cancelled by its caller", None)
    await asyncio.wait_for(stop, WAIT)
    assert await asyncio.wait_for(path, WAIT) == CartesianPath(MoveEnd.CANCELLED, "cancelled by its caller")


# The run of a plan through the backbone's execute_joint_path.

A_PLAN = JointPlan(
    MoveEnd.COMPLETED,
    start_joint_positions=tuple(JOINTS),
    waypoints=(Waypoint(tuple(PLANNED), POSITION, ORIENTATION), Waypoint(tuple(JOINTS), POSITION, ORIENTATION)),
)
RUN_BARS = dict(duration_s=0.0, settle_tolerance_rad=0.05, settle_timeout_s=2.0)


def path_ran(arm="left_arm", joints=JOINTS):
    return SimpleNamespace(arms=[SimpleNamespace(arm_name=arm, final_joint_positions=list(joints))], action_time=2.1)


async def test_a_run_sends_the_plans_waypoints_with_its_start_as_the_expected_start():
    robot, planner = planned()
    run = asyncio.create_task(robot.execute_joint_path("left_arm", A_PLAN, **RUN_BARS))
    goal = await asyncio.wait_for(planner.execute_joint_path.next_goal(), WAIT)
    assert goal.request == ExecuteJointPathRequest(
        arms=[
            ExecuteJointPathArm(
                arm_name="left_arm",
                waypoints=[ExecuteJointPathWaypoint(joint_positions=PLANNED), ExecuteJointPathWaypoint(joint_positions=JOINTS)],
                expected_start_joint_positions=JOINTS,
                settle_tolerances_rad=[0.05] * 7,
            )
        ],
        duration_s=0.0,
        settle_timeout_s=2.0,
    )
    goal.complete(path_ran())
    assert await asyncio.wait_for(run, WAIT) == PathResult(MoveEnd.COMPLETED, final_joint_positions=tuple(JOINTS))


@pytest.mark.parametrize("kind, message", [
    ("precondition_failed", "left_arm stands 0.08 rad from the expected start at joint 2"),
    ("unreachable", "the guard would hold the path short at waypoint 1"),
    ("timeout", "joint 4 still 0.07 rad from its target, 2.0 s after the planned end"),
])
async def test_a_run_that_fails_says_why_with_its_kind(kind, message):
    robot, planner = planned()
    run = asyncio.create_task(robot.execute_joint_path("left_arm", A_PLAN, **RUN_BARS))
    goal = await asyncio.wait_for(planner.execute_joint_path.next_goal(), WAIT)
    goal.fail(kind, message)
    assert await asyncio.wait_for(run, WAIT) == PathResult(MoveEnd.FAILED, message, kind)


async def test_a_stop_cancels_a_run_in_flight_and_gives_where_the_joints_stopped():
    robot, planner = planned()
    run = asyncio.create_task(robot.execute_joint_path("left_arm", A_PLAN, **RUN_BARS))
    goal = await asyncio.wait_for(planner.execute_joint_path.next_goal(), WAIT)
    await asyncio.wait_for(in_flight(robot), WAIT)
    stop = asyncio.create_task(robot.stop())
    await asyncio.wait_for(goal.cancel_asked.wait(), WAIT)
    goal.complete_cancelled("aborted: operator", path_ran(joints=PLANNED))
    await asyncio.wait_for(stop, WAIT)
    result = await asyncio.wait_for(run, WAIT)
    assert result == PathResult(MoveEnd.CANCELLED, "aborted: operator", "", tuple(PLANNED))
    assert result.cancelled


async def test_a_refused_run_says_why():
    robot, planner = planned()
    planner.execute_joint_path.refuse_next("leader_streaming", "a leader streams left_arm")
    assert await robot.execute_joint_path("left_arm", A_PLAN, **RUN_BARS) == PathResult(
        MoveEnd.REFUSED, "a leader streams left_arm", "leader_streaming"
    )
