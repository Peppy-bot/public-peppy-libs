"""Joint position limits parsed from the SO-101 URDF at startup.

The backbone owns the URDF, so it owns limit enforcement too: goals more than
LIMIT_SLACK_RAD beyond a limit are rejected and closer ones run at the limit,
streamed targets and IK solutions are clamped or refused, and postures are
validated at launch. Parse, don't validate: a URDF missing a
named revolute joint or its limit fails the launch.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from so101_description.units import JOINT_NAMES

# How far outside a joint limit a target may lie and still be taken, at that
# limit (rad). A measured position sent back as a target lies that close.
LIMIT_SLACK_RAD = 0.001


@dataclass(frozen=True)
class OutsideLimits:
    """A joint position farther outside its limits than LIMIT_SLACK_RAD: the
    joint counted from 0, its position and its limits."""

    joint: int
    position: float
    lower: float
    upper: float

    def __str__(self) -> str:
        """The joint counted from 1, with every digit of its position and its
        limits, so that a position a hair outside a limit reads apart from it."""
        return (
            f"joint {self.joint + 1} at {self.position!r} rad, "
            f"outside its limits [{self.lower!r}, {self.upper!r}]"
        )


@dataclass(frozen=True)
class JointLimits:
    lower: tuple[float, ...]
    upper: tuple[float, ...]

    def contains(self, positions: tuple[float, ...]) -> bool:
        if len(positions) != len(self.lower):
            return False
        return all(
            lo <= p <= hi
            for p, lo, hi in zip(positions, self.lower, self.upper, strict=True)
        )

    def first_outside(self, positions: tuple[float, ...]) -> OutsideLimits | None:
        """The first joint of `positions` farther than LIMIT_SLACK_RAD outside
        its limits, or None when `clamp` takes every joint at most that far."""
        if len(positions) != len(self.lower):
            raise ValueError("expected one position per joint")
        for joint, (p, lo, hi) in enumerate(zip(positions, self.lower, self.upper, strict=True)):
            if not lo - LIMIT_SLACK_RAD <= p <= hi + LIMIT_SLACK_RAD:
                return OutsideLimits(joint=joint, position=p, lower=lo, upper=hi)
        return None

    def clamp(self, positions: tuple[float, ...]) -> tuple[float, ...]:
        if len(positions) != len(self.lower):
            raise ValueError("expected one position per joint")
        return tuple(
            min(max(p, lo), hi)
            for p, lo, hi in zip(positions, self.lower, self.upper, strict=True)
        )


def from_urdf(urdf_path: str) -> JointLimits:
    """The five joints' position limits (rad), in wire order."""
    # Direct children only: transmission blocks nest limitless <joint> stubs
    # under the same names, and iter() would let them shadow the real joints.
    joints = {
        joint.get("name"): joint
        for joint in ET.parse(urdf_path).getroot().findall("joint")
    }
    lower, upper = [], []
    for name in JOINT_NAMES:
        joint = joints.get(name)
        if joint is None:
            raise ValueError(f"URDF has no joint named {name}")
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"URDF joint {name} declares no position limits")
        lo, hi = float(limit.get("lower")), float(limit.get("upper"))
        if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
            raise ValueError(f"URDF joint {name} has unusable limits [{lo}, {hi}]")
        lower.append(lo)
        upper.append(hi)
    return JointLimits(lower=tuple(lower), upper=tuple(upper))
