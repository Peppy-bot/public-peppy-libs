"""The SO-family device boundary over lerobot: calibrated connection and the
flat motor-list read (five joints then the gripper) both hardware nodes
share."""

from __future__ import annotations

import math

from so101_description.units import MOTOR_NAMES, NUM_JOINTS


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
