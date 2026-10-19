"""The limbs parameter: arm and gripper pairs, parsed once at the start."""

import pytest

from brain_core.limbs import Limb, parse_limbs


def test_two_pairs_in_their_order():
    assert parse_limbs("left_arm:left_gripper,right_arm:right_gripper") == (
        Limb("left_arm", "left_gripper"),
        Limb("right_arm", "right_gripper"),
    )


def test_one_pair_with_no_side_in_its_names():
    assert parse_limbs("arm:gripper") == (Limb("arm", "gripper"),)


def test_spaces_around_each_name_are_trimmed():
    assert parse_limbs(" left_arm : left_gripper , right_arm:right_gripper ") == (
        Limb("left_arm", "left_gripper"),
        Limb("right_arm", "right_gripper"),
    )


@pytest.mark.parametrize("text", ["", "   "])
def test_no_pair_is_refused(text):
    with pytest.raises(ValueError, match="limbs must name at least one arm:gripper pair"):
        parse_limbs(text)


@pytest.mark.parametrize("text, pair", [
    ("left_arm", "left_arm"),
    ("left_arm:left_gripper,right_arm", "right_arm"),
    ("left_arm:left_gripper,", ""),
    ("left_arm:left_gripper:extra", "left_arm:left_gripper:extra"),
])
def test_a_pair_that_is_not_one_arm_and_one_gripper_is_refused(text, pair):
    with pytest.raises(ValueError, match=f"limbs: '{pair}' is not one arm:gripper pair"):
        parse_limbs(text)


@pytest.mark.parametrize("text, pair", [(":left_gripper", ":left_gripper"), ("left_arm: ", "left_arm:"), (" : ", ":")])
def test_an_empty_side_is_refused(text, pair):
    with pytest.raises(ValueError, match=f"limbs: '{pair}' needs an arm name and a gripper name"):
        parse_limbs(text)


def test_an_arm_or_a_gripper_named_twice_is_refused():
    with pytest.raises(ValueError, match="limbs: the arm 'arm' is named twice"):
        parse_limbs("arm:left_gripper,arm:right_gripper")
    with pytest.raises(ValueError, match="limbs: the gripper 'gripper' is named twice"):
        parse_limbs("left_arm:gripper, right_arm : gripper")
