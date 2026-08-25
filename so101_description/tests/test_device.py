"""The lerobot device boundary: calibrated connect and the flat read."""

import pytest

from so101_description import device
from so101_description.units import MOTOR_NAMES, NUM_JOINTS


class FakeHardware:
    def __init__(self):
        self.positions = {name: float(i) for i, name in enumerate(MOTOR_NAMES)}

    def read_positions(self):
        return dict(self.positions)


def test_read_positions_validated_orders_and_splits():
    joints, gripper = device.read_positions_validated(FakeHardware())
    assert joints == tuple(float(i) for i in range(NUM_JOINTS))
    assert gripper == float(NUM_JOINTS)


def test_read_positions_validated_rejects_non_finite():
    hardware = FakeHardware()
    hardware.positions["elbow_flex"] = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        device.read_positions_validated(hardware)


def test_read_positions_validated_rejects_missing_motor():
    hardware = FakeHardware()
    del hardware.positions["gripper"]
    with pytest.raises(KeyError):
        device.read_positions_validated(hardware)


class FakeRobot:
    def __init__(self, calibrated):
        self.is_calibrated = calibrated
        self.connect_calls = []
        self.disconnected = False

    def connect(self, calibrate):
        self.connect_calls.append(calibrate)

    def disconnect(self):
        self.disconnected = True


def test_connect_calibrated_never_enters_the_interactive_flow():
    robot = FakeRobot(calibrated=True)
    device.connect_calibrated(robot, "arm")
    assert robot.connect_calls == [False]


def test_uncalibrated_device_disconnects_and_fails_the_launch():
    robot = FakeRobot(calibrated=False)
    with pytest.raises(RuntimeError, match="calibration is missing"):
        device.connect_calibrated(robot, "arm")
    assert robot.disconnected
