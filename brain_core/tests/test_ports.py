"""The roles a brain's models serve, parsed once from a goal's text, and
the kind and the words each refusal and failure carries."""

import pytest

from brain_core.ports import CancelToken, Cancelled, ExpectedFailure, FailureKind, Refusal, Role, parse_role

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
