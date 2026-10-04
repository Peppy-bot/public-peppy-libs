import math

import pytest

from workspace_core_py import (
    ABOVE_SURFACE,
    GRASP_ANGLE_TOLERANCE,
    GRASP_DIRECTIONS,
    REACH_TOLERANCE,
    GraspDirection,
    angle_between,
)


def test_the_limits_are_those_of_workspace_core():
    assert REACH_TOLERANCE == 0.01
    assert GRASP_ANGLE_TOLERANCE == 0.05
    assert ABOVE_SURFACE == 0.04


def test_the_grasp_directions_come_down_first_then_forward():
    named = [(direction.name, direction.approach) for direction in GRASP_DIRECTIONS]
    assert named == [("down", (0.0, 0.0, -1.0)), ("forward", (1.0, 0.0, 0.0))]
    assert all(isinstance(direction, GraspDirection) for direction in GRASP_DIRECTIONS)
    assert GRASP_DIRECTIONS[0] == GRASP_DIRECTIONS[0]
    assert GRASP_DIRECTIONS[0] != GRASP_DIRECTIONS[1]
    assert repr(GRASP_DIRECTIONS[1]) == "<GraspDirection forward>"


def test_a_grasp_direction_is_not_made_in_python():
    with pytest.raises(TypeError):
        GraspDirection()


def test_the_angle_between_two_directions_runs_from_zero_to_pi():
    assert angle_between((0.0, 0.0, -1.0), [0.0, 0.0, -2.0]) == pytest.approx(0.0)
    assert angle_between((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)) == pytest.approx(math.pi / 2)
    assert angle_between((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)) == pytest.approx(math.pi)
    assert angle_between((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) == math.pi


def test_a_direction_is_any_iterable_of_three_numbers():
    tilted = (math.sin(0.04), 0.0, -math.cos(0.04))
    down = GRASP_DIRECTIONS[0].approach
    assert angle_between(iter(tilted), down) == pytest.approx(0.04)
    assert angle_between([1, 0, 0], down) == pytest.approx(math.pi / 2)


def test_a_direction_of_another_size_or_not_of_numbers_is_refused():
    with pytest.raises(ValueError, match="^a direction holds 3 numbers, not 2$"):
        angle_between((1.0, 0.0), (1.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="^a direction holds 3 numbers, not 4$"):
        angle_between((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    with pytest.raises(TypeError):
        angle_between("xyz", (1.0, 0.0, 0.0))  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        angle_between(1.0, (1.0, 0.0, 0.0))  # type: ignore[arg-type]
