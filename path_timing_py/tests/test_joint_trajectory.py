"""JointTrajectory, control_core's joint trajectory on the caller's clock:
what it takes, what it refuses, and how it plays."""

import math

import pytest

from path_timing_py import JOINT_PATH_CORNER_DEVIATION, MIN_JOINT_CHANGE_RAD, JointTrajectory

# Two joints, each capped at 1 rad/s and 2 rad/s².
VELOCITY_LIMITS = (1.0, 1.0)
ACCELERATION_LIMITS = (2.0, 2.0)
# 1 rad on the first joint, then 1 rad on the second: one corner, a right
# angle.
WAYPOINTS = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))


def through(waypoints=WAYPOINTS, requested_s=0.0):
    return JointTrajectory.through(waypoints, VELOCITY_LIMITS, ACCELERATION_LIMITS, requested_s)


def test_the_constants_are_those_of_control_core():
    assert JOINT_PATH_CORNER_DEVIATION == 0.1
    assert MIN_JOINT_CHANGE_RAD == 0.001


def test_a_trajectory_plays_from_its_first_waypoint_to_its_last():
    trajectory = through()
    assert trajectory.start == (0.0, 0.0)
    assert trajectory.end == (1.0, 1.0)
    assert trajectory.sample(-1.0) == trajectory.sample(0.0) == trajectory.start
    assert trajectory.sample(trajectory.duration_s) == trajectory.end
    assert trajectory.sample(trajectory.duration_s + 1.0) == trajectory.end


def test_a_trajectory_is_done_from_its_duration_on():
    trajectory = through()
    assert trajectory.duration_s > 0.0
    assert not trajectory.done(0.0)
    assert not trajectory.done(math.nextafter(trajectory.duration_s, 0.0))
    assert trajectory.done(trajectory.duration_s)


def test_the_joints_are_tuples_of_floats():
    trajectory = JointTrajectory.through([[0, 0], [1, 2]], [1, 1], [2, 2], 0)
    for joints in (trajectory.start, trajectory.end, trajectory.sample(0.5)):
        assert type(joints) is tuple
        assert all(type(position) is float for position in joints)
    assert type(trajectory.duration_s) is float


def test_the_arguments_are_any_iterables_of_numbers():
    trajectory = JointTrajectory.through(
        (iter(waypoint) for waypoint in WAYPOINTS),
        iter(VELOCITY_LIMITS),
        iter(ACCELERATION_LIMITS),
        0.0,
    )
    assert trajectory.duration_s == through().duration_s
    assert trajectory.sample(1.0) == through().sample(1.0)


def test_each_limit_bounds_its_own_joint():
    # 1 rad on the first joint alone, under 1 rad/s and 2 rad/s²: 0.5 s up to
    # speed, 0.5 s at speed and 0.5 s down, the trapezoid time of 1.5 s to the
    # grid's three parts in a thousand. The limits of the second joint, which
    # does not move, bound nothing.
    trajectory = JointTrajectory.through(((0.0, 0.0), (1.0, 0.0)), (1.0, 0.1), (2.0, 0.1), 0.0)
    assert trajectory.duration_s == pytest.approx(1.5, rel=3e-3)


def test_a_corner_is_rounded_within_the_joint_path_deviation():
    # The right angle is rounded on an arc whose midpoint lies the deviation
    # from the corner: the nearest the trajectory comes to it.
    trajectory = through()
    steps = 20_000
    nearest = min(
        math.dist(trajectory.sample(trajectory.duration_s * k / steps), WAYPOINTS[1])
        for k in range(steps + 1)
    )
    assert nearest == pytest.approx(JOINT_PATH_CORNER_DEVIATION, abs=1e-6)


def test_a_longer_request_plays_the_same_path_slowed_by_one_factor():
    fastest = through()
    slowed = through(requested_s=2.0 * fastest.duration_s)
    assert slowed.duration_s == 2.0 * fastest.duration_s
    for k in range(11):
        t_s = fastest.duration_s * k / 10
        assert slowed.sample(2.0 * t_s) == fastest.sample(t_s)


def test_a_shorter_request_plays_the_path_in_its_own_time():
    fastest = through()
    assert through(requested_s=0.5 * fastest.duration_s).duration_s == fastest.duration_s


def test_a_single_waypoint_stands_there_throughout():
    trajectory = JointTrajectory.through([(0.5, -0.5)], VELOCITY_LIMITS, ACCELERATION_LIMITS, -0.0)
    assert math.copysign(1.0, trajectory.duration_s) == 1.0
    assert trajectory.duration_s == 0.0
    assert trajectory.done(0.0)
    assert trajectory.sample(-1.0) == trajectory.sample(1.0) == (0.5, -0.5)


def test_limits_too_small_for_a_finite_time_hold_the_joints_at_the_start():
    trajectory = JointTrajectory.through(((0.0,), (1.0,)), (1e-310,), (1e-310,), 0.0)
    assert trajectory.duration_s == math.inf
    assert trajectory.sample(1e300) == trajectory.start == (0.0,)
    assert not trajectory.done(1e300)


@pytest.mark.parametrize(
    "waypoints, limits",
    [(((0.0,), (1e200,)), (1.0,)), (((0.0,), (1.0,)), (5e-324,))],
    ids=["waypoints whose distance overflows", "limits whose speeds underflow"],
)
def test_a_path_whose_arithmetic_fails_is_refused(waypoints, limits):
    with pytest.raises(
        ValueError,
        match="^control_core times no path through these waypoints under these limits: ",
    ):
        JointTrajectory.through(waypoints, limits, limits, 0.0)


def test_a_path_without_a_waypoint_is_refused():
    with pytest.raises(ValueError, match="^a joint path has a waypoint at least: where the joints"):
        through(waypoints=[])


def test_waypoints_of_other_numbers_of_joints_are_refused():
    with pytest.raises(
        ValueError,
        match=r"^the waypoints hold one position per joint: waypoints\[0\] holds 2, "
        r"waypoints\[2\] holds 3$",
    ):
        through(waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0, 1.0)])


@pytest.mark.parametrize("position, text", [(math.nan, "NaN"), (math.inf, "inf")])
def test_a_waypoint_that_is_not_finite_is_refused(position, text):
    with pytest.raises(
        ValueError,
        match=rf"^waypoints\[1\] holds {text}, and a joint position is a finite number$",
    ):
        through(waypoints=[(0.0, 0.0), (1.0, position)])


@pytest.mark.parametrize("quantity", ["velocity", "acceleration"])
def test_limits_of_another_number_of_joints_are_refused(quantity):
    limits = {"velocity": VELOCITY_LIMITS, "acceleration": ACCELERATION_LIMITS}
    limits[quantity] = (1.0,)
    with pytest.raises(
        ValueError, match=f"^the {quantity} limits hold one limit per joint, 2, not 1$"
    ):
        JointTrajectory.through(WAYPOINTS, limits["velocity"], limits["acceleration"], 0.0)


@pytest.mark.parametrize("quantity", ["velocity", "acceleration"])
@pytest.mark.parametrize(
    "limit, text", [(0.0, "0"), (-1.0, "-1"), (math.nan, "NaN"), (math.inf, "inf")]
)
def test_a_limit_that_is_not_finite_and_positive_is_refused(quantity, limit, text):
    limits = {"velocity": VELOCITY_LIMITS, "acceleration": ACCELERATION_LIMITS}
    limits[quantity] = (1.0, limit)
    with pytest.raises(
        ValueError, match=f"^a {quantity} limit must be a positive finite number, got {text}$"
    ):
        JointTrajectory.through(WAYPOINTS, limits["velocity"], limits["acceleration"], 0.0)


@pytest.mark.parametrize("requested_s, text", [(-1.0, "-1"), (math.nan, "NaN"), (math.inf, "inf")])
def test_a_requested_duration_that_is_not_a_finite_number_of_at_least_0_is_refused(
    requested_s, text
):
    with pytest.raises(
        ValueError, match=f"^a requested duration is a finite number of at least 0 s, not {text}$"
    ):
        through(requested_s=requested_s)


@pytest.mark.parametrize("t_s", [math.nan, math.inf, -math.inf])
def test_a_sample_time_that_is_not_finite_is_refused(t_s):
    with pytest.raises(ValueError, match="^a sample time is a finite number of seconds, not"):
        through().sample(t_s)


@pytest.mark.parametrize(
    "waypoints", [[0.0, 1.0], [("a", "b")]], ids=["a number for a waypoint", "text for a position"]
)
def test_waypoints_that_are_not_iterables_of_numbers_are_refused(waypoints):
    with pytest.raises(TypeError):
        through(waypoints=waypoints)
