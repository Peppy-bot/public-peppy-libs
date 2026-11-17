"""The roles a brain's models serve, parsed once from a goal's text, the
kind and the words each refusal and failure carries, and the target value
of a known item."""

import pytest

from brain_core.ports import CancelToken, Cancelled, ExpectedFailure, FailureKind, Item, ItemShape, Refusal, Role, TargetItem, parse_role

BOTH = (Role.PERCEPTION, Role.MANIPULATION)


def test_a_role_parses_to_its_role_and_reads_as_its_name():
    assert parse_role("perception", BOTH) is Role.PERCEPTION
    assert parse_role("manipulation", BOTH) is Role.MANIPULATION
    assert f"{Role.PERCEPTION}" == "perception"


@pytest.mark.parametrize("text", ["arm", "", "Perception", " perception", "manipulation"])
def test_a_role_the_brain_does_not_have_is_refused_naming_it(text):
    with pytest.raises(Refusal, match=f"^this brain runs no model for the role '{text}'$") as raised:
        parse_role(text, (Role.PERCEPTION,))
    assert raised.value.kind is FailureKind.REJECTED


def test_a_refusal_and_an_expected_failure_carry_their_kind_and_message():
    failure = ExpectedFailure(FailureKind.ATTEMPT_FAILED, "model-x failed: RateLimitError: slow down")
    assert failure.message == str(failure) == "model-x failed: RateLimitError: slow down"
    assert failure.kind is FailureKind.ATTEMPT_FAILED
    refusal = Refusal(FailureKind.PRECONDITION_FAILED, "scan_items is running")
    assert (refusal.kind, refusal.message) == (FailureKind.PRECONDITION_FAILED, "scan_items is running")


def test_the_brains_kinds_are_the_kinds_of_peppy_and_not_found():
    assert [kind.value for kind in FailureKind] == [
        "attempt_failed", "timeout", "precondition_failed", "resource_unavailable", "rejected", "not_found",
    ]


def test_a_token_is_set_once_and_says_whether_the_goals_own_signal_set_it():
    token = CancelToken()
    token.check()
    token.cancel("its sender is gone", signalled=True)
    token.cancel("aborted: operator", signalled=False)
    with pytest.raises(Cancelled) as raised:
        token.check()
    assert (raised.value.reason, raised.value.signalled) == ("its sender is gone", True)


# The target value of a known item (TargetItem.of): its box, else its top,
# else its seen point.

BOWL_BOX = ItemShape((0.337, 0.000, 0.487), (0.192, 0.196, 0.088), 0.0)


def a_bowl(**fields) -> Item:
    return Item("wooden_bowl_1-t0", "wooden bowl", (0.380, -0.008, 0.461), None, 0.9, 1, **fields)


def test_a_target_with_a_box_reads_its_centre_top_bottom_and_shorter_side():
    target = TargetItem.of(a_bowl(shape=BOWL_BOX, width_m=0.18, top_z=0.52))
    assert (target.item_id, target.label) == ("wooden_bowl_1-t0", "wooden bowl")
    assert target.centre == (0.337, 0.000)
    assert target.top_z == pytest.approx(0.531) and target.bottom_z == pytest.approx(0.443)
    assert target.width_m == 0.192


def test_a_box_under_half_the_width_across_the_picture_is_not_the_items_own():
    # A bowl first seen with a fruit in it: the depth step fitted the box
    # around the fruit (the box session of 2026-11-18).
    fruit_box = ItemShape((0.334, -0.029, 0.486), (0.065, 0.053, 0.087), 1.536)
    target = TargetItem.of(a_bowl(shape=fruit_box, width_m=0.18))
    assert target.centre == (0.380, -0.008) and target.width_m == 0.18
    assert target.top_z == pytest.approx(0.5295) and target.bottom_z == pytest.approx(0.4425)


def test_a_target_with_no_box_reads_its_top_and_its_seen_point():
    target = TargetItem.of(a_bowl(top_z=0.53, width_m=0.18))
    assert (target.centre, target.top_z, target.bottom_z, target.width_m) == ((0.380, -0.008), 0.53, 0.461, 0.18)


def test_a_target_with_no_box_and_no_top_reads_its_seen_point_alone():
    target = TargetItem.of(a_bowl())
    assert (target.centre, target.top_z, target.bottom_z, target.width_m) == ((0.380, -0.008), 0.461, 0.461, None)
