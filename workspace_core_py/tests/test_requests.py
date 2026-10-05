import math
import pickle

import pytest

from workspace_core_py import ABOVE_SURFACE, Positions, SurfaceHeight

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


def test_positions_are_parsed_into_points_in_order():
    positions = Positions.from_wire([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert positions.points == [(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)]
    assert len(positions) == 2
    assert Positions.from_wire(iter((1000.0, -1000.0, 0))).points == [
        (1000.0, -1000.0, 0.0)
    ]


@pytest.mark.parametrize(
    ("values", "refusal"),
    [
        ([], "positions must hold at least one point"),
        ([0.1, 0.2, 0.3, 0.4], r"positions must hold 3 values \(x, y, z\) per point"),
        ([0.1, math.nan, 0.3], "positions must hold finite numbers only"),
        ([0.1, 0.2, math.inf], "positions must hold finite numbers only"),
        (
            [1e200, 0.0, 0.0],
            "positions must hold coordinates within 1000 m of the robot's base point",
        ),
        (
            [0.0, -1000.5, 0.0],
            "positions must hold coordinates within 1000 m of the robot's base point",
        ),
    ],
)
def test_positions_that_are_not_points_of_a_request_are_refused(values, refusal):
    with pytest.raises(ValueError, match=f"^{refusal}$"):
        Positions.from_wire(values)


def test_positions_of_values_that_are_not_numbers_are_refused():
    with pytest.raises(TypeError):
        Positions.from_wire(["0.1", "0.2", "0.3"])  # type: ignore[list-item]
    with pytest.raises(TypeError):
        Positions.from_wire(0.1)  # type: ignore[arg-type]
