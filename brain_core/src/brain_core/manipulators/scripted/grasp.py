"""The geometry of a grasp, pure: the orientations a gripper takes at a
point, and the points of the moves around it.

The grasp frame is limb_motion's and workspace_core's
(`public-peppy-libs/workspace_core/src/grasp.rs`): its origin is the grasp
point, its +Z the approach, out of the gripper, and its +Y the axis the
jaws close on. A `GraspDirection` points the approach along an axis of the
robot frame (+X forward, +Y left, +Z up). At roll 0 the jaws close along
the robot frame's +Y; `ROLLS` rolls, 45 degrees apart, turn them about the
approach. These are the orientations workspace_core's reach test proves
reachability with, `DOWN` before `FORWARD`.

Every orientation is a unit quaternion [x, y, z, w].

`Grasp` is where and how a gripper closes on an item, the point and the
orientations to try there; a `GraspProposer` gives the grasp of an item.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Protocol

from ...ports import Item, Quat, Vec3

ROLLS = 8


class GraspDirection(Enum):
    """The way the gripper points when it grasps, as the unit approach in
    the robot frame."""

    # Straight down, along the robot frame's -Z: a grasp from above.
    DOWN = (0.0, 0.0, -1.0)
    # Straight forward, along the robot frame's +X: a grasp from the side.
    FORWARD = (1.0, 0.0, 0.0)


def orientation(direction: GraspDirection, roll_rad: float) -> Quat:
    """The grasp frame with its approach along `direction`, rolled
    `roll_rad` about the approach from the frame whose jaws close along
    the robot frame's +Y."""
    z = direction.value
    y = (0.0, 1.0, 0.0)
    x = _cross(y, z)
    sin, cos = math.sin(roll_rad), math.cos(roll_rad)
    rolled_x = tuple(cos * x[i] + sin * y[i] for i in range(3))
    rolled_y = tuple(cos * y[i] - sin * x[i] for i in range(3))
    return _quaternion(rolled_x, rolled_y, z)


def candidates(direction: GraspDirection) -> tuple[Quat, ...]:
    """The `ROLLS` orientations of `direction`, from roll 0 up."""
    return tuple(orientation(direction, step * math.tau / ROLLS) for step in range(ROLLS))


def orientations_for(chosen: Optional[Quat]) -> tuple[Quat, ...]:
    """The orientations to try at a grasp point: `chosen` alone when the
    caller chose one, else the `DOWN` candidates. A ValueError names a
    chosen quaternion that is not a rotation."""
    if chosen is None:
        return candidates(GraspDirection.DOWN)
    return (unit(chosen),)


def unit(q: Quat) -> Quat:
    """`q` scaled to length 1, or a ValueError when it names no rotation:
    zero, or not finite."""
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError(f"the orientation {list(q)} is not a rotation")
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def approach(q: Quat) -> Vec3:
    """The approach of the grasp frame `q`, its +Z, in the robot frame."""
    x, y, z, w = unit(q)
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))


def jaw_axis(q: Quat) -> Vec3:
    """The axis the jaws close on in the grasp frame `q`, its +Y, in the
    robot frame."""
    x, y, z, w = unit(q)
    return (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x))


def jaw_line_deg(q: Quat) -> float:
    """The direction of the line the jaws of `q` close along, seen from
    above: degrees counter-clockwise from the robot's +X, in [0, 180),
    since a line has no sense."""
    ax, ay, _az = jaw_axis(q)
    return math.degrees(math.atan2(ay, ax)) % 180.0


def turned_about_z(q: Quat, angle_rad: float) -> Quat:
    """The grasp frame `q` turned `angle_rad` about the robot's +Z,
    counter-clockwise seen from above: the turn an alignment step asks for
    with its roll_deg. The approach stays as it was."""
    half = angle_rad / 2.0
    return _multiply((0.0, 0.0, math.sin(half), math.cos(half)), unit(q))


def pre_grasp(point: Vec3, q: Quat, clearance_m: float) -> Vec3:
    """Where the grasp point stands before it closes in on `point`:
    `clearance_m` back along the approach of `q`, so the gripper reaches
    the point moving along its own approach."""
    a = approach(q)
    return (point[0] - a[0] * clearance_m, point[1] - a[1] * clearance_m, point[2] - a[2] * clearance_m)


def lifted(point: Vec3, lift_m: float) -> Vec3:
    """`point` raised `lift_m` straight up."""
    return (point[0], point[1], point[2] + lift_m)


def lowered(point: Vec3, depth_m: float) -> Vec3:
    """`point` lowered `depth_m` straight down."""
    return (point[0], point[1], point[2] - depth_m)


def _multiply(a: Quat, b: Quat) -> Quat:
    """The rotation `b` then `a`, as quaternions [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _quaternion(x: Vec3, y: Vec3, z: Vec3) -> Quat:
    """The unit quaternion of the rotation whose columns are the axes `x`,
    `y` and `z`, by the branch that divides by the largest term."""
    m00, m01, m02 = x[0], y[0], z[0]
    m10, m11, m12 = x[1], y[1], z[1]
    m20, m21, m22 = x[2], y[2], z[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


@dataclass(frozen=True)
class Grasp:
    """Where and how a gripper closes on an item: the grasp point, robot
    frame, and the orientations of the gripper to try there, in order; a
    manipulator takes the first one the backbone has a plan for. Each is a
    unit quaternion [x, y, z, w] of limb_motion's grasp frame: +Z along the
    approach, out of the gripper, +Y the axis the jaws close on."""

    point: Vec3
    orientations: tuple[Quat, ...]

    def __post_init__(self) -> None:
        if not self.orientations:
            raise ValueError("a grasp needs at least one orientation to try")


class GraspProposer(Protocol):
    """Where and how a manipulator grasps an item: the piece of a grab a
    model can fill. A ValueError names why it has no grasp for the item."""

    def propose(self, item: Item) -> Grasp: ...
