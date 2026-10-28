"""The SO-family device boundary over lerobot: calibrated connection, the
flat motor-list read (five joints then the gripper) and its stamped sample,
and the position channel names on either side of lerobot."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from so101_description.units import MOTOR_NAMES, NUM_JOINTS


@dataclass(frozen=True)
class PositionSample:
    """One full position read: the joints (deg), the gripper (percent), and
    the monotonic time the read returned."""

    positions_deg: tuple[float, ...]
    gripper_percent: float
    captured_monotonic: float


def connect_calibrated(robot, what: str) -> None:
    """Connect a lerobot device without the interactive calibration flow and
    refuse to serve uncalibrated: a headless node must never block on a
    terminal prompt, and an uncalibrated bus is a launch error."""
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise RuntimeError(
            f"{what} calibration is missing or no longer matches the motors; "
            "run lerobot-calibrate for this id"
        )


def read_positions_validated(hardware) -> tuple[tuple[float, ...], float]:
    """One full position read as (joints_deg, gripper_percent), raising on a
    missing motor or a non-finite value."""
    by_motor = hardware.read_positions()
    values = tuple(float(by_motor[name]) for name in MOTOR_NAMES)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite position read")
    return values[:NUM_JOINTS], values[NUM_JOINTS]


def read_position_sample(hardware) -> PositionSample:
    """One read_positions_validated read, stamped with the monotonic time it
    returned; raises as that read does."""
    positions_deg, gripper_percent = read_positions_validated(hardware)
    return PositionSample(
        positions_deg=positions_deg,
        gripper_percent=gripper_percent,
        captured_monotonic=time.monotonic(),
    )


# The fastest an SO-101 node may run a loop that reaches the bus. Six
# STS3215s share one 1 Mbaud half-duplex line, and a goal write plus a
# position read must both fit in every cycle. The leader, the follower and
# the backbone each cap their rates at this one ceiling.
BUS_MAX_RATE_HZ = 1000

# lerobot names a motor's position channel `<motor>.pos`; peppy's wire and
# every node-side map are keyed by the bare motor name.
_POSITION_SUFFIX = ".pos"


def motor_positions(channels: dict[str, float]) -> dict[str, float]:
    """A lerobot observation or action keyed by bare motor name.

    Stripping the suffix can collide (`wrist_flex.pos` and `wrist_flex` name
    the same motor afterwards), and a collision means two readings for one
    joint. Refused rather than resolved: nothing here can know which of them
    the bus actually reported."""
    positions: dict[str, float] = {}
    for key, value in channels.items():
        motor = key.removesuffix(_POSITION_SUFFIX)
        if motor in positions:
            raise ValueError(f"two channels name motor {motor}")
        positions[motor] = value
    return positions


def position_channels(goals_by_motor: dict[str, float]) -> dict[str, float]:
    """Goals keyed the way lerobot's action interface expects them."""
    return {f"{motor}{_POSITION_SUFFIX}": value for motor, value in goals_by_motor.items()}
