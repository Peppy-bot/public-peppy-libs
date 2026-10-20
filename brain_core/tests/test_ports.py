"""The roles a brain's models serve, parsed once from a goal's text."""

import pytest

from brain_core.ports import ExpectedFailure, Refusal, Role, parse_role

BOTH = (Role.PERCEPTION, Role.MANIPULATION)


def test_a_role_parses_to_its_role_and_reads_as_its_name():
    assert parse_role("perception", BOTH) is Role.PERCEPTION
    assert parse_role("manipulation", BOTH) is Role.MANIPULATION
    assert f"{Role.PERCEPTION}" == "perception"


@pytest.mark.parametrize("text", ["arm", "", "Perception", " perception", "manipulation"])
def test_a_role_the_brain_does_not_have_is_refused_naming_it(text):
    with pytest.raises(Refusal, match=f"^this brain runs no model for the role '{text}'$"):
        parse_role(text, (Role.PERCEPTION,))


def test_an_expected_failure_carries_its_message():
    failure = ExpectedFailure("model-x failed: RateLimitError: slow down")
    assert failure.message == str(failure) == "model-x failed: RateLimitError: slow down"
