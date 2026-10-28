"""The aligner port's request: what a grab and a place hand an aligner, and
the request's own rules."""

import pytest

from brain_core.manipulators.scripted.aligning import AlignRequest
from brain_core.manipulators.scripted.grasp import GraspDirection, orientation
from brain_core.ports import Gripper, Item

LEFT = Gripper("left_gripper", "left_arm")
SEEN = (0.50, 0.10, 0.72)
PRE = (0.50, 0.10, 0.770)
POINT = (0.50, 0.10, 0.695)
LIFT_M = 0.075
DOWN_0 = orientation(GraspDirection.DOWN, 0.0)


def cube(label: str = "cube") -> Item:
    return Item("cube_1-t0", label, SEEN, None, 0.9, 1)


def test_a_grab_request_names_the_item_and_measures_from_its_seen_centre():
    request = AlignRequest(cube(), LEFT, PRE, POINT, DOWN_0, LIFT_M)
    assert request.is_place is False
    assert request.label == "cube" and request.centre == (SEEN[0], SEEN[1])
    assert request.centre_words == "the cube's seen centre"


def test_an_item_with_no_label_is_the_item():
    request = AlignRequest(cube(""), LEFT, PRE, POINT, DOWN_0, LIFT_M)
    assert request.label == "item" and request.centre_words == "the item's seen centre"


def test_a_place_request_measures_from_the_target_point():
    target = (0.52, 0.11, 0.70)
    request = AlignRequest(None, LEFT, PRE, (0.52, 0.11, 0.70), DOWN_0, LIFT_M, target=target)
    assert request.is_place is True
    assert request.centre == (0.52, 0.11) and request.centre_words == "the target point"


def test_an_alignment_with_neither_an_item_nor_a_target_is_refused():
    with pytest.raises(ValueError, match="an item to grasp or a target point"):
        AlignRequest(None, LEFT, PRE, POINT, DOWN_0, LIFT_M)
