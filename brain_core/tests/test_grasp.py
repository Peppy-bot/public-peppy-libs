"""The geometry of a grasp and the grasp the scripted manipulator asks
for: the orientations workspace_core tries, the points around a grasp
point, and the centre proposer's two cases, an item a look saw and one
whose position is a grasp point."""

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from brain_core.manipulators.scripted.centre_proposer import CentreProposer
from brain_core.manipulators.scripted.grasp import (
    ROLLS,
    Grasp,
    GraspDirection,
    approach,
    candidates,
    lifted,
    lowered,
    orientation,
    orientations_for,
    pre_grasp,
    unit,
)
from brain_core.ports import Item


def rotate(q, v):
    """`v` turned by the unit quaternion `q`."""
    x, y, z, w = q
    columns = (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(columns[j][i] * v[j] for j in range(3)) for i in range(3))


def close(a, b, tolerance=1e-12):
    return all(abs(p - q) <= tolerance for p, q in zip(a, b))


def angle_between(a, b):
    dot = sum(p * q for p, q in zip(a, b))
    return math.acos(max(-1.0, min(1.0, dot)))


def test_down_at_roll_0_points_the_approach_down_and_keeps_the_jaws_on_y():
    q = orientation(GraspDirection.DOWN, 0.0)
    assert close(q, (0.0, 1.0, 0.0, 0.0))
    assert close(rotate(q, (0.0, 0.0, 1.0)), (0.0, 0.0, -1.0))
    assert close(rotate(q, (0.0, 1.0, 0.0)), (0.0, 1.0, 0.0))


def test_each_down_roll_turns_the_jaw_axis_45_degrees_about_the_approach():
    rolls = candidates(GraspDirection.DOWN)
    assert len(rolls) == ROLLS == 8
    assert close(rolls[0], orientation(GraspDirection.DOWN, 0.0))
    for q in rolls:
        assert close(approach(q), (0.0, 0.0, -1.0))
    jaws = [rotate(q, (0.0, 1.0, 0.0)) for q in rolls]
    for a, b in zip(jaws, jaws[1:]):
        assert angle_between(a, b) == pytest.approx(math.pi / 4)
    # A quarter turn takes the jaw axis from +Y to +X, as workspace_core's.
    assert close(jaws[2], (1.0, 0.0, 0.0))


def test_forward_at_roll_0_approaches_along_x():
    q = orientation(GraspDirection.FORWARD, 0.0)
    assert close(rotate(q, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))
    assert close(rotate(q, (0.0, 1.0, 0.0)), (0.0, 1.0, 0.0))


def test_the_pre_grasp_point_is_clearance_back_along_the_approach_and_the_lift_straight_up():
    point = (0.5, 0.1, 0.7)
    down = orientation(GraspDirection.DOWN, math.pi / 4)
    assert close(pre_grasp(point, down, 0.1), (0.5, 0.1, 0.8))
    forward = orientation(GraspDirection.FORWARD, 0.0)
    assert close(pre_grasp(point, forward, 0.1), (0.4, 0.1, 0.7))
    assert lifted(point, 0.1) == (0.5, 0.1, 0.7 + 0.1)
    assert lowered(point, 0.02) == (0.5, 0.1, 0.7 - 0.02)


def test_a_chosen_orientation_is_tried_alone_and_none_tries_the_down_rolls():
    assert orientations_for(None) == candidates(GraspDirection.DOWN)
    assert orientations_for((0.0, 2.0, 0.0, 0.0)) == ((0.0, 1.0, 0.0, 0.0),)
    with pytest.raises(ValueError, match=r"^the orientation \[0.0, 0.0, 0.0, 0.0\] is not a rotation$"):
        orientations_for((0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="is not a rotation"):
        unit((math.nan, 0.0, 0.0, 1.0))


def test_a_grasp_needs_an_orientation_to_try():
    with pytest.raises(ValueError, match="^a grasp needs at least one orientation to try$"):
        Grasp((0.5, 0.0, 0.7), ())


def item(position, orientation=None, *, grasp_point=False) -> Item:
    return Item("cup_1-t0", "cup", position, orientation, 0.9, 1, position_is_grasp_point=grasp_point)


def test_a_seen_item_is_grasped_from_above_under_its_seen_point():
    grasp = CentreProposer(grasp_depth_m=0.03).propose(item((0.5, 0.1, 0.72), (0.0, 0.0, 0.0, 1.0)))
    assert grasp.point == (0.5, 0.1, 0.72 - 0.03)
    # The item's own orientation is not a grasp: every roll from above.
    assert grasp.orientations == candidates(GraspDirection.DOWN)


def test_a_grasp_point_is_grasped_unlowered_in_its_orientation_alone_or_from_above():
    proposer = CentreProposer()
    chosen = proposer.propose(item((0.5, 0.1, 0.7), (0.0, 0.0, 0.0, 1.0), grasp_point=True))
    assert chosen == Grasp((0.5, 0.1, 0.7), ((0.0, 0.0, 0.0, 1.0),))
    free = proposer.propose(item((0.5, 0.1, 0.7), None, grasp_point=True))
    assert free == Grasp((0.5, 0.1, 0.7), candidates(GraspDirection.DOWN))
    with pytest.raises(ValueError, match="is not a rotation"):
        proposer.propose(item((0.5, 0.1, 0.7), (0.0, 0.0, 0.0, 0.0), grasp_point=True))


# Properties over any roll and any point.

rolls = st.floats(min_value=-4 * math.pi, max_value=4 * math.pi, allow_nan=False)
coordinates = st.floats(min_value=-2.0, max_value=2.0, allow_nan=False)
points = st.tuples(coordinates, coordinates, coordinates)


@given(st.sampled_from(list(GraspDirection)), rolls)
def test_every_orientation_is_a_unit_quaternion_that_approaches_along_its_direction(direction, roll):
    q = orientation(direction, roll)
    assert math.sqrt(sum(v * v for v in q)) == pytest.approx(1.0)
    assert close(approach(q), direction.value, 1e-9)
    # The jaw axis stays square to the approach.
    jaws = rotate(q, (0.0, 1.0, 0.0))
    assert sum(a * b for a, b in zip(jaws, direction.value)) == pytest.approx(0.0, abs=1e-9)


@given(points, rolls, st.floats(min_value=0.0, max_value=0.5))
def test_the_pre_grasp_point_lies_clearance_away_against_the_approach(point, roll, clearance):
    q = orientation(GraspDirection.DOWN, roll)
    pre = pre_grasp(point, q, clearance)
    offset = tuple(p - g for p, g in zip(point, pre))
    assert math.dist(pre, point) == pytest.approx(clearance, abs=1e-9)
    assert close(offset, tuple(clearance * a for a in approach(q)), 1e-9)


def test_a_turn_about_the_vertical_turns_the_jaw_line_counter_clockwise_and_keeps_the_approach():
    from brain_core.manipulators.scripted.grasp import jaw_axis, jaw_line_deg, turned_about_z

    down_0 = orientation(GraspDirection.DOWN, 0.0)
    assert close(jaw_axis(down_0), (0.0, 1.0, 0.0)) and jaw_line_deg(down_0) == pytest.approx(90.0)
    turned = turned_about_z(down_0, math.radians(45.0))
    # +45 counter-clockwise seen from above takes the jaw line from +y
    # (90 degrees) to 135 degrees; the approach still points down.
    assert jaw_line_deg(turned) == pytest.approx(135.0)
    assert close(approach(turned), (0.0, 0.0, -1.0), 1e-9)
    assert jaw_line_deg(turned_about_z(down_0, math.radians(-45.0))) == pytest.approx(45.0)
    # A line has no sense: a half turn gives the same line.
    assert jaw_line_deg(turned_about_z(down_0, math.pi)) == pytest.approx(90.0)
    assert sum(v * v for v in turned) == pytest.approx(1.0)
