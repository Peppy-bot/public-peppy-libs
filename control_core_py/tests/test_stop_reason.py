"""Tests of control_core_py.stop_reason, the cases of control_core::stop_reason's."""

from control_core_py.stop_reason import stop_reason


def test_the_stop_reason_names_the_stop_and_the_reason_of_its_request():
    assert stop_reason("the operator asked") == "stopped: the operator asked"
    assert stop_reason("") == "stopped"
