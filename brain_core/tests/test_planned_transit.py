"""The scripted manipulator with the motion planner bound: the planned
transit of a grab (each orientation planned to the pre-grasp point, the
reach of the descent and the lift tested from the end of the plan, the plan
run) and of a place (the high pre-place point, then the low one, the reach
of the line down tested), and every way each ends before the arm moves on."""

import asyncio
from collections import deque

import pytest
from conftest import records_of
from fake_peppygen import limb_states_message
from test_scripted import DOWN, GRASP, LIFT, PRE, FakeClock, FakeRobot, cup, holding

from brain_core.limb_state import LimbStateStore
from brain_core.manipulators.scripted.centre_proposer import CentreProposer
from brain_core.manipulators.scripted.manipulator import (
    ALLOWED_PLANNING_TIME_S,
    PLAN_ORIENTATION_TOLERANCE_RAD,
    PLAN_POSITION_TOLERANCE_M,
    PLANNER,
    REFINER,
    SETTLE_JOINT_RAD,
    SETTLE_TIMEOUT_S,
    ScriptedManipulator,
)
from brain_core.manipulators.scripted.manipulator import CLEARANCE_M, HELD_ITEM_DEPTH_M
from brain_core.ports import Cancelled, CancelToken, Gripper, Outcome, Pose, Senses
from brain_core.robot import (
    ArmDescription,
    ArmDescriptions,
    CartesianPath,
    JointPlan,
    MoveEnd,
    PathResult,
    Waypoint,
)

LEFT = Gripper("left_gripper", "left_arm")
GRASP_LINK = "openarm_left_tcp"
GRIPPER_LINKS = ("openarm_left_ee_base_link", "openarm_left_ee_link1", "openarm_left_ee_link2")
LEFT_ARM = ArmDescription("left_arm", "left_gripper", tuple(f"openarm_left_joint{i}" for i in range(1, 8)), GRASP_LINK, GRIPPER_LINKS)
START = (0.1, -0.2, 0.3, 1.4, 0.5, -0.6, 0.7)
MIDWAY = (0.15, -0.15, 0.32, 1.35, 0.45, -0.55, 0.75)
AT_PRE = (0.2, -0.1, 0.35, 1.3, 0.4, -0.5, 0.8)
STARTS_IN_COLLISION = "openarm_left_ee_link1 stands 0.0388 m from the map, nearer than the clearance 0.0400 m"


def a_plan(position, orientation) -> JointPlan:
    """A completed plan from START through MIDWAY to the pre point."""
    return JointPlan(
        MoveEnd.COMPLETED, start_joint_positions=START,
        waypoints=(Waypoint(MIDWAY, (0.4, 0.15, 0.8), orientation), Waypoint(AT_PRE, position, orientation)),
    )


def no_plan(kind: str, message: str) -> JointPlan:
    return JointPlan(MoveEnd.FAILED, message, kind)


def reached(fraction: float = 1.0) -> CartesianPath:
    return CartesianPath(MoveEnd.COMPLETED, start_joint_positions=AT_PRE, fraction=fraction)


class FakePlannerRobot(FakeRobot):
    """The fake robot with its planner slot bound. describe_arms names the
    left arm; a plan reaches its pose, a reach test covers its lines and a
    run completes, unless the test queued another answer. Each is recorded
    in `calls` beside the backbone's: ("describe",), ("plan", position,
    orientation), ("line", positions, orientation, avoid_collisions, start)
    and ("run", start, waypoint joints), and the bars of each plan and run
    in `bars`."""

    has_planner = True

    def __init__(self) -> None:
        super().__init__()
        self.described = ArmDescriptions((LEFT_ARM,))
        self.plans: deque = deque()
        self.lines: deque = deque()
        self.runs: deque = deque()

    def is_lift(self, call: tuple) -> bool:
        """With the planner the transit is a run, so a grab's lift is its
        second move."""
        return call[0] == "move" and self.kinds().count("move") == 2

    async def describe_arms(self) -> ArmDescriptions:
        await asyncio.sleep(0)
        self._record(("describe",))
        return self.described

    async def plan_joint_path(self, arm, link, position, orientation, **bars) -> JointPlan:
        assert (arm, link) == ("left_arm", GRASP_LINK)
        await asyncio.sleep(0)
        self.bars.append(("plan", bars))
        self._record(("plan", position, orientation))
        return self.plans.popleft() if self.plans else a_plan(position, orientation)

    async def plan_cartesian_path(self, arm, link, poses, *, avoid_collisions, start_joint_positions=()) -> CartesianPath:
        assert (arm, link) == ("left_arm", GRASP_LINK)
        await asyncio.sleep(0)
        orientations = {orientation for _, orientation in poses}
        assert len(orientations) == 1
        self._record(("line", tuple(position for position, _ in poses), orientations.pop(), avoid_collisions, tuple(start_joint_positions)))
        return self.lines.popleft() if self.lines else reached()

    async def execute_joint_path(self, arm, plan, **bars) -> PathResult:
        assert arm == "left_arm"
        await asyncio.sleep(0)
        self.bars.append(("run", bars))
        self._record(("run", plan.start_joint_positions, tuple(w.joint_positions for w in plan.waypoints)))
        return self.runs.popleft() if self.runs else PathResult(MoveEnd.COMPLETED, final_joint_positions=AT_PRE)


async def started(**kwargs):
    robot, store = FakePlannerRobot(), LimbStateStore()
    store.add(limb_states_message(1.0, left_gripper=1.0))
    manipulator = ScriptedManipulator(CentreProposer(), clock=FakeClock(), **kwargs)
    await manipulator.start(Senses(robot, store, None, None, None))
    return manipulator, robot, store


THE_RUN = ("run", START, (MIDWAY, AT_PRE))


async def test_a_planned_grab_opens_plans_the_transit_tests_the_descent_and_lift_runs_the_plan_then_moves_by_lines(caplog):
    manipulator, robot, store = await started()
    holding(robot, store)
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.grab(cup(), LEFT, 12.0, CancelToken())
    assert outcome == Outcome(True)
    q = DOWN[0]
    assert robot.calls == [
        ("grip", "left_gripper", 1.0, 0.0),
        ("describe",),
        ("plan", PRE, q),
        # The reach of the descent and of the lift, from the end of the plan,
        # with no collision check.
        ("line", (GRASP, LIFT), q, False, AT_PRE),
        THE_RUN,
        ("move", GRASP, q),
        ("grip", "left_gripper", 0.0, 12.0),
        ("move", LIFT, q),
    ]
    plan_bars = dict(
        position_tolerance_m=0.001, orientation_tolerance_rad=0.01, planner="rrt_connect", refiner="none", allowed_planning_time_s=0.5,
    )
    assert (PLAN_POSITION_TOLERANCE_M, PLAN_ORIENTATION_TOLERANCE_RAD, PLANNER, REFINER, ALLOWED_PLANNING_TIME_S) == tuple(plan_bars.values())
    assert ("plan", plan_bars) in robot.bars
    assert ("run", dict(duration_s=0.0, settle_tolerance_rad=SETTLE_JOINT_RAD, settle_timeout_s=SETTLE_TIMEOUT_S)) in robot.bars
    assert SETTLE_JOINT_RAD == 0.05
    ran, grab = [record.getMessage() for record in log.records]
    assert ran == "planned transit of arm 'left_arm' ran: 2 waypoints to the grasp point [0.500, 0.100, 0.770]"
    assert grab.startswith(
        "grab of item 'cup_1-t0' with gripper 'left_gripper': orientation 1 of 8 planned, [0.000, 1.000, 0.000, 0.000] "
        "(1 plan, 1 reach test, 0 ms); landed"
    )


async def test_the_planning_time_is_a_setting():
    manipulator, robot, store = await started(allowed_planning_time_s=1.5)
    holding(robot, store)
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(True)
    [plan_bars] = [bars for kind, bars in robot.bars if kind == "plan"]
    assert plan_bars["allowed_planning_time_s"] == 1.5


async def test_an_orientation_with_no_plan_gives_way_to_the_next():
    manipulator, robot, store = await started()
    holding(robot, store)
    robot.plans.append(no_plan("unreachable", "no joint positions meet the goal without a collision in 0.5 s"))
    assert await manipulator.grab(cup(), LEFT, 0.0, CancelToken()) == Outcome(True)
    plans = [call for call in robot.calls if call[0] == "plan"]
    assert [call[2] for call in plans] == [DOWN[0], DOWN[1]]
    assert {call[2] for call in robot.calls if call[0] == "move"} == {DOWN[1]}


async def test_no_orientation_with_a_plan_fails_with_the_planners_kind_and_words_before_the_arm_moves():
    manipulator, robot, store = await started()
    robot.plans.extend([no_plan("start_in_collision", STARTS_IN_COLLISION)] * len(DOWN))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(
        False,
        "no plan reaches item 'cup_1-t0' with gripper 'left_gripper': orientation 8 of 8, "
        f"[-0.383, 0.924, 0.000, 0.000], has no plan to its pre-grasp point [0.500, 0.100, 0.770]: start_in_collision: {STARTS_IN_COLLISION}",
    )
    # No line as a fallback, and no move: the jaws stand open as they opened.
    assert robot.kinds() == ["grip", "describe"] + ["plan"] * len(DOWN)


async def test_an_orientation_whose_descent_or_lift_does_not_reach_gives_way_and_names_that_point():
    manipulator, robot, store = await started()
    robot.lines.extend([reached(0.38), reached(0.6)] * (len(DOWN) // 2))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    # Half of the first line: the grasp point; past the first line: the lifted point.
    assert outcome.message.endswith(
        "has no plan to its lifted point [0.500, 0.100, 0.770]: plan_cartesian_path reaches 0.60 of the lines from the end of the plan"
    )
    assert robot.kinds() == ["grip", "describe"] + ["plan", "line"] * len(DOWN)
    robot.calls.clear()
    robot.lines.append(CartesianPath(MoveEnd.FAILED, "the start stands in collision", "start_in_collision"))
    robot.lines.extend([reached(0.38)] * (len(DOWN) - 1))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome.message.endswith(
        "has no plan to its grasp point [0.500, 0.100, 0.695]: plan_cartesian_path reaches 0.38 of the lines from the end of the plan"
    )


async def test_a_run_of_the_plan_that_fails_ends_the_grab_with_its_words_and_nothing_more_moves():
    manipulator, robot, store = await started()
    robot.runs.append(PathResult(MoveEnd.FAILED, "left_arm stands 0.08 rad from the expected start at joint 2", "precondition_failed"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "left_arm stands 0.08 rad from the expected start at joint 2")
    assert robot.calls[-1] == THE_RUN


async def test_a_run_a_stop_ended_retreats_from_nothing():
    manipulator, robot, store = await started()
    robot.runs.append(PathResult(MoveEnd.CANCELLED, "stopped: the operator asked"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "stopped: the operator asked")
    assert robot.calls[-1] == THE_RUN


async def test_a_cancel_during_a_plan_raises_cancelled_before_the_arm_moves():
    manipulator, robot, store = await started()
    cancel = CancelToken()
    robot.on_call = lambda call: cancel.cancel("aborted: operator", signalled=False) if call[0] == "plan" else None
    robot.plans.append(JointPlan(MoveEnd.CANCELLED, "aborted: operator"))
    with pytest.raises(Cancelled):
        await manipulator.grab(cup(), LEFT, 0.0, cancel)
    assert robot.kinds() == ["grip", "describe", "plan"]


async def test_a_plan_the_brain_did_not_cancel_that_ends_cancelled_ends_the_grab_as_stopped():
    manipulator, robot, store = await started()
    robot.plans.append(JointPlan(MoveEnd.CANCELLED, "the planner shuts down"))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the planner shuts down")
    assert robot.kinds() == ["grip", "describe", "plan"]


async def test_a_planner_that_does_not_describe_the_arm_fails_the_grab_before_any_plan():
    manipulator, robot, store = await started()
    robot.described = ArmDescriptions(reason="describe_arms gave no answer: TimeoutError('no reply')")
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the motion planner does not describe arm 'left_arm': describe_arms gave no answer: TimeoutError('no reply')")
    assert robot.kinds() == ["grip", "describe"]
    robot.described = ArmDescriptions((ArmDescription("right_arm", "right_gripper", (), "openarm_right_tcp", ()),))
    outcome = await manipulator.grab(cup(), LEFT, 0.0, CancelToken())
    assert outcome == Outcome(False, "the motion planner does not describe arm 'left_arm': it names no such arm")


# The planned transit of a place: to the pre-place point 15 cm above the
# pose, else 7.5 cm above it, the reach of the line down tested from the end
# of each plan.

PLACE_Q = DOWN[0]
PLACE_POINT = (0.6, 0.0, 0.75)
HIGH = (0.6, 0.0, 0.75 + (CLEARANCE_M + HELD_ITEM_DEPTH_M))
LOW = (0.6, 0.0, 0.75 + CLEARANCE_M)


async def test_a_planned_place_plans_to_the_high_pre_place_point_tests_the_line_down_runs_the_plan_and_moves_by_lines(caplog):
    manipulator, robot, _ = await started()
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    assert outcome == Outcome(True, "", PLACE_POINT)
    assert robot.calls == [
        ("describe",),
        ("plan", HIGH, PLACE_Q),
        ("line", (PLACE_POINT,), PLACE_Q, False, AT_PRE),
        THE_RUN,
        ("move", PLACE_POINT, PLACE_Q),
        ("grip", "left_gripper", 1.0, 0.0),
        ("move", HIGH, PLACE_Q),
    ]
    assert [record.getMessage() for record in log.records] == [
        "planned transit of arm 'left_arm' ran: 2 waypoints to the grasp point [0.600, 0.000, 0.900]"
    ]


async def test_a_place_with_no_plan_to_the_high_point_plans_to_the_low_one(caplog):
    manipulator, robot, _ = await started()
    robot.plans.append(no_plan("no_path", "no path joined the goal in 0.5 s"))
    with records_of(caplog, "brain_core.manipulators.scripted.manipulator") as log:
        outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    assert outcome == Outcome(True, "", PLACE_POINT)
    assert [call for call in robot.calls if call[0] in ("plan", "move")] == [
        ("plan", HIGH, PLACE_Q), ("plan", LOW, PLACE_Q), ("move", PLACE_POINT, PLACE_Q), ("move", LOW, PLACE_Q),
    ]
    assert "place with gripper 'left_gripper': no plan 15.0 cm above the pose: the pre-place point stands 7.5 cm above it" in [
        record.getMessage() for record in log.records
    ]


async def test_a_place_whose_line_down_does_not_reach_from_the_high_point_tries_the_low_one():
    manipulator, robot, _ = await started()
    robot.lines.append(reached(0.4))
    assert await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken()) == Outcome(True, "", PLACE_POINT)
    assert [call[1] for call in robot.calls if call[0] == "plan"] == [HIGH, LOW]


async def test_a_place_with_no_plan_at_either_height_fails_with_the_planners_words_and_nothing_moves():
    manipulator, robot, _ = await started()
    robot.plans.extend([no_plan("start_in_collision", STARTS_IN_COLLISION)] * 2)
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    assert outcome == Outcome(
        False,
        "no plan reaches the pose [0.600, 0.000, 0.750] with gripper 'left_gripper': orientation 1 of 1, "
        f"[0.000, 1.000, 0.000, 0.000], has no plan to its pre-place point [0.600, 0.000, 0.825]: start_in_collision: {STARTS_IN_COLLISION}",
    )
    assert robot.kinds() == ["describe", "plan", "describe", "plan"]


async def test_a_place_without_an_orientation_plans_each_roll_high_then_each_roll_low():
    manipulator, robot, _ = await started()
    robot.plans.extend([no_plan("no_path", "no path")] * (2 * len(DOWN)))
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT), CancelToken())
    assert not outcome.success
    plans = [call for call in robot.calls if call[0] == "plan"]
    assert [call[2] for call in plans] == list(DOWN) + list(DOWN)
    assert [round(call[1][2], 6) for call in plans] == [0.9] * len(DOWN) + [0.825] * len(DOWN)


async def test_a_place_the_planner_cannot_describe_fails_at_once_with_no_try_of_the_low_point():
    manipulator, robot, _ = await started()
    robot.described = ArmDescriptions(reason="describe_arms gave no answer: TimeoutError('no reply')")
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    assert outcome == Outcome(False, "the motion planner does not describe arm 'left_arm': describe_arms gave no answer: TimeoutError('no reply')")
    assert robot.kinds() == ["describe"]


async def test_a_place_whose_run_fails_fails_with_its_words_and_the_jaws_stay_closed():
    manipulator, robot, _ = await started()
    robot.runs.append(PathResult(MoveEnd.FAILED, "the guard would hold the path short at waypoint 1", "unreachable"))
    outcome = await manipulator.place(LEFT, Pose(PLACE_POINT, PLACE_Q), CancelToken())
    assert outcome == Outcome(False, "the guard would hold the path short at waypoint 1")
    assert robot.calls[-1] == THE_RUN and "grip" not in robot.kinds()
