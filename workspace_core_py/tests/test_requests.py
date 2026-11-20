import math
import pickle
import re

import pytest

from workspace_core_py import ABOVE_SURFACE, NO_POINTS, Positions, SurfaceHeight

ROWS, LANES = 51, 13


def test_a_surface_height_is_parsed_to_the_millimetre():
    assert SurfaceHeight.from_wire(0.4504) == SurfaceHeight.from_wire(0.45)
    assert SurfaceHeight.from_wire(0.4506).metres == 0.451
    assert SurfaceHeight.from_wire(-0.0004).metres == 0.0
    assert SurfaceHeight.from_wire(-0.25).metres == -0.25
    assert SurfaceHeight.from_wire(1).metres == 1.0
    assert SurfaceHeight.from_wire(0.45) != SurfaceHeight.from_wire(0.451)


def test_heights_that_round_alike_are_one_key():
    heights = {SurfaceHeight.from_wire(0.4502), SurfaceHeight.from_wire(0.4498)}
    assert heights == {SurfaceHeight.from_wire(0.45)}


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_surface_height_that_is_not_a_finite_number_is_refused(bad):
    with pytest.raises(ValueError, match="^surface_height must be a finite number$"):
        SurfaceHeight.from_wire(bad)


@pytest.mark.parametrize("metres", [1000.0, -1000.0])
def test_a_surface_height_1000_m_from_the_base_is_within_the_bound(metres):
    assert SurfaceHeight.from_wire(metres).metres == metres


# 1000.0004 rounds to 1000 m, but the bound applies to the height the request
# gives.
@pytest.mark.parametrize("far", [1000.0004, 1000.001, -1000.001, 2e6, -2e6, 1e300])
def test_a_surface_height_more_than_1000_m_from_the_base_is_refused(far):
    with pytest.raises(
        ValueError,
        match="^surface_height must be within 1000 m of the robot's base point$",
    ):
        SurfaceHeight.from_wire(far)


def test_a_surface_height_survives_pickling_and_reads_as_its_parsing():
    height = SurfaceHeight.from_wire(0.45)
    assert pickle.loads(pickle.dumps(height)) == height
    assert repr(height) == "SurfaceHeight.from_wire(0.45)"


def test_the_grid_targets_stand_above_the_grid_row_by_row_from_the_nearest():
    targets = SurfaceHeight.from_wire(0.45).grid_targets()
    assert len(targets) == ROWS * LANES
    z = 0.45 + ABOVE_SURFACE
    expected = [
        (round(row * 0.02, 9), round(-0.3 + lane * 0.05, 9))
        for row in range(ROWS)
        for lane in range(LANES)
    ]
    for (x, y, target_z), (expected_x, expected_y) in zip(targets, expected):
        assert (x, y) == pytest.approx((expected_x, expected_y), abs=1e-12)
        assert target_z == pytest.approx(z)
    assert targets[0][:2] == (0.0, -0.3)
    assert targets[LANES - 1][:2] == (0.0, 0.3)
    assert targets[LANES][:2] == (0.02, -0.3)
    assert targets[-1][:2] == (1.0, 0.3)


def test_placed_points_are_taken_in_order():
    positions = Positions.parse([(0.1, 0.2, 0.3), [0.4, 0.5, 0.6]])
    assert positions.points == [(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)]
    assert len(positions) == 2
    assert Positions.parse(iter([iter((1000.0, -1000.0, 0))])).points == [
        (1000.0, -1000.0, 0.0)
    ]


def test_the_refusal_of_no_points_reads_as_the_contract_says():
    assert NO_POINTS == "points must hold at least one point"


TOO_FAR = (
    "points must lie within 1000 m of the robot's base point on each axis, placed in the robot "
    "frame"
)


@pytest.mark.parametrize(
    ("points", "refusal"),
    [
        ([], NO_POINTS),
        ([(1e200, 0.0, 0.0)], TOO_FAR),
        ([(0.3, 0.0, 0.5), (0.0, -1000.5, 0.0)], TOO_FAR),
    ],
)
def test_points_that_are_not_points_of_a_request_are_refused(points, refusal):
    with pytest.raises(ValueError, match=f"^{re.escape(refusal)}$"):
        Positions.parse(points)


@pytest.mark.parametrize(
    ("position", "refusal"),
    [
        (
            (0.1, math.nan, 0.3),
            "a position must be finite: got x 0.1, y NaN, z 0.3; give a finite number of meters "
            "on each axis",
        ),
        (
            (0.1, 0.2, -math.inf),
            "a position must be finite: got x 0.1, y 0.2, z -inf; give a finite number of meters "
            "on each axis",
        ),
        ((0.1, 0.2, 0.3, 0.4), "a point holds 3 numbers, not 4"),
    ],
)
def test_a_point_that_is_not_finite_numbers_is_refused_saying_what_to_give(position, refusal):
    with pytest.raises(ValueError, match=f"^{re.escape(refusal)}$"):
        Positions.parse([(0.0, 0.0, 0.0), position])


def test_points_of_values_that_are_not_numbers_are_refused():
    with pytest.raises(TypeError):
        Positions.parse([("0.1", "0.2", "0.3")])  # type: ignore[list-item]
    with pytest.raises(TypeError):
        Positions.parse(0.1)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        Positions.parse([0.1])  # type: ignore[list-item]
