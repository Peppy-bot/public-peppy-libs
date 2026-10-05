import math
import pickle

import pytest

from workspace_core_py import Reach, ReachMemo, SurfaceHeight, SurfaceReach

GRID_TARGETS = 51 * 13


def height(metres):
    return SurfaceHeight.from_wire(metres)


def surface(metres, reach):
    return SurfaceReach.from_grid_order(height(metres), [reach] * GRID_TARGETS)


def test_a_reached_target_names_its_arm_and_falls_short_by_nothing():
    reach = Reach.reached_by("arm")
    assert (reach.reached, reach.arm, reach.short_by) == (True, "arm", 0.0)


def test_a_target_out_of_reach_names_no_arm_and_says_how_far_short():
    reach = Reach.short(0.25)
    assert (reach.reached, reach.arm, reach.short_by) == (False, "", 0.25)
    assert Reach.short(0.0).short_by == 0.0


def test_a_reach_short_by_minus_zero_carries_zero():
    assert math.copysign(1.0, Reach.short(-0.0).short_by) == 1.0


def test_reaches_compare_by_value_and_read_as_their_making():
    assert Reach.reached_by("arm") == Reach.reached_by("arm")
    assert Reach.reached_by("arm") != Reach.reached_by("left_arm")
    assert Reach.short(0.1) != Reach.reached_by("arm")
    assert repr(Reach.reached_by("arm")) == 'Reach.reached_by("arm")'
    assert repr(Reach.short(0.25)) == "Reach.short(0.25)"


@pytest.mark.parametrize("reach", [Reach.reached_by("arm"), Reach.short(0.25)])
def test_a_reach_survives_pickling_to_another_process(reach):
    assert pickle.loads(pickle.dumps(reach)) == reach


def test_a_reach_without_an_arm_name_is_refused():
    with pytest.raises(ValueError, match="^a reached target names the arm"):
        Reach.reached_by("")


@pytest.mark.parametrize("by", [-0.001, math.nan, math.inf])
def test_a_shortfall_that_is_not_a_finite_distance_is_refused(by):
    with pytest.raises(ValueError, match="^a target out of reach is short by a finite"):
        Reach.short(by)


def test_a_surface_reach_holds_one_reach_per_grid_target_in_order():
    reaches = [
        Reach.reached_by("arm") if index % 3 == 0 else Reach.short(index / 1000)
        for index in range(GRID_TARGETS)
    ]
    measured = SurfaceReach.from_grid_order(height(0.3), iter(reaches))
    assert measured.height == height(0.3)
    assert measured.reaches == reaches


@pytest.mark.parametrize("count", [0, GRID_TARGETS - 1, GRID_TARGETS + 1])
def test_a_surface_reach_of_another_count_is_refused(count):
    expected = (
        f"^{count} reaches are given for {GRID_TARGETS} targets: "
        "give one reach per target, in order$"
    )
    with pytest.raises(ValueError, match=expected):
        SurfaceReach.from_grid_order(height(0.3), [Reach.short(0.1)] * count)


def test_a_surface_reach_of_values_that_are_not_reaches_is_refused():
    with pytest.raises(TypeError):
        SurfaceReach.from_grid_order(height(0.3), [0.1] * GRID_TARGETS)  # type: ignore[list-item]


def test_the_memo_answers_a_stored_height_and_keeps_the_reach_stored_first():
    memo = ReachMemo()
    assert len(memo) == 0
    assert memo.get(height(0.45)) is None
    first = surface(0.45, Reach.short(0.1))
    assert memo.insert(first).reaches[0] == Reach.short(0.1)
    kept = memo.insert(surface(0.4502, Reach.short(0.9)))
    assert kept.reaches[0] == Reach.short(0.1), "the stored reach stands"
    assert len(memo) == 1
    stored = memo.get(height(0.4498))
    assert stored is not None
    assert stored.reaches[0] == Reach.short(0.1)


def test_the_memo_keeps_32_heights_and_drops_the_one_stored_first():
    memo = ReachMemo()
    for millimetres in range(32):
        memo.insert(surface(millimetres / 1000, Reach.short(millimetres / 1000)))
    assert memo.get(height(0.0)) is not None, "asked again after it was stored"
    memo.insert(surface(0.032, Reach.short(0.032)))
    assert len(memo) == 32
    assert memo.get(height(0.0)) is None, "stored first, dropped first"
    assert memo.get(height(0.001)) is not None
    assert memo.get(height(0.032)) is not None
