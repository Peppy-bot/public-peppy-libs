"""The geometry types of the contracts, as contracts-hub's README defines them, and the rotation
arithmetic under them: control_core::geometry and frame_tree::arithmetic in Python.

A node's generated code holds its own dataclass for each field, and the node converts the fields
of each into these types through the one parse here, which refuses what no contract takes: a
number that is not finite, an orientation whose squared norm lies tf2's tolerance, _QUATERNION_NORMALIZATION_TOLERANCE,
or more from 1, a position or an orientation of another number of entries, an empty frame, and a
timestamp a stamp of control_core does not hold. A command with no timestamp (None) is placed at
the latest time, which tf2 names `TimePointZero`, and a given timestamp is a time like any other,
0.0 the Unix epoch; a target with no orientation (None) gives none. A planar member reads a pose on
the floor plane through `Pose.planar`, which refuses a pose off the plane. Every refusal says what
to send instead, each number in it printed as Rust's `Display` prints it, and a refusal
control_core::geometry also gives has its text (control_core/tests/fixtures/geometry_refusals.tsv).

A quaternion is (x, y, z, w), the order of the contracts; MuJoCo and Isaac hold (w, x, y, z),
which `xyzw_of_wxyz` and `wxyz_of_xyzw` convert. Each operation runs as frame_tree's does, which
runs it as tf2 does (geometry2 0.45.10, tf2/include/tf2/LinearMath).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from decimal import Decimal
from fractions import Fraction

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Rotation = tuple[Vector, Vector, Vector]

IDENTITY: Quaternion = (0.0, 0.0, 0.0, 1.0)

# How far the squared norm of an orientation may lie from 1: the tolerance of tf2's buffer
# (buffer_core.cpp:66), which the transforms contract names.
_QUATERNION_NORMALIZATION_TOLERANCE = 10e-3

# How far a planar pose's height and its tilt may lie from 0, in m and rad: a value composed
# through planar transforms alone gives an exact 0, and the rounding of a chain of 20 transforms
# at 100 m stays near 4.4e-13.
_PLANAR_TOLERANCE = 1e-9

# The nanoseconds from the Unix epoch past which a stamp of control_core holds no time: one past
# the last u64.
_STAMP_NANOS_END = 2**64
_NANOS_PER_SECOND = 10**9


class GeometryError(ValueError):
    """What no contract takes, refused with `rejected`; the message says what to send."""


# ---- the arithmetic, as frame_tree's ---------------------------------------------------------


def product(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """`q1 * q2` (Quaternion.hpp:427-434)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
        w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _product_with_vector(q: Quaternion, v: Vector) -> Quaternion:
    """`q * v`, the vector taken as a quaternion of zero w (Quaternion.hpp:436-443)."""
    x, y, z, w = q
    vx, vy, vz = v
    return (
        w * vx + y * vz - z * vy,
        w * vy + z * vx - x * vz,
        w * vz + x * vy - y * vx,
        -x * vx - y * vy - z * vz,
    )


def inverse(q: Quaternion) -> Quaternion:
    """The conjugate (Quaternion.hpp:282-285)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _dot(q1: Quaternion, q2: Quaternion) -> float:
    return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]


def normalized(q: Quaternion) -> Quaternion:
    """`q` times the reciprocal of its length (Quaternion.hpp:169-172)."""
    inverse_length = 1.0 / math.sqrt(_dot(q, q))
    x, y, z, w = q
    return (x * inverse_length, y * inverse_length, z * inverse_length, w * inverse_length)


def rotate(q: Quaternion, v: Vector) -> Vector:
    """`quatRotate`: `v` turned by `q`, as `q * v * inverse(q)` (Quaternion.hpp:502-507)."""
    x, y, z, _ = product(_product_with_vector(q, v), inverse(q))
    return (x, y, z)


def angle_shortest_path(q1: Quaternion, q2: Quaternion) -> float:
    """`angleShortestPath`: the angle (rad) of the turn from `q1` to the nearer of `q2` and `-q2`,
    twice the angle between them (Quaternion.hpp:239-247)."""
    s = math.sqrt(_dot(q1, q1) * _dot(q2, q2))
    d = _dot(q1, q2)
    ratio = -d / s if d < 0.0 else d / s
    return math.acos(max(-1.0, min(1.0, ratio))) * 2.0


def slerp(q1: Quaternion, q2: Quaternion, t: float) -> Quaternion:
    """`q1` turned toward `q2` by the fraction `t` of the shorter arc; `q1` where the two give one
    rotation (Quaternion.hpp:340-364)."""
    theta = angle_shortest_path(q1, q2) / 2.0
    d = _dot(q1, q2)
    if theta == 0.0:
        return q1
    reciprocal = 1.0 / math.sin(theta)
    s0 = math.sin((1.0 - t) * theta)
    s1 = math.sin(t * theta)
    toward = tuple(-c for c in q2) if d < 0.0 else q2
    return tuple((a * s0 + b * s1) * reciprocal for a, b in zip(q1, toward))  # type: ignore[return-value]


def rotation_of(q: Quaternion) -> Rotation:
    """`Matrix3x3::setRotation`: the rotation of `q` as given, its length divided out."""
    x, y, z, w = q
    s = 2.0 / (x * x + y * y + z * z + w * w)
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return (
        (1.0 - (yy + zz), xy - wz, xz + wy),
        (xy + wz, 1.0 - (xx + zz), yz - wx),
        (xz - wy, yz + wx, 1.0 - (xx + yy)),
    )


def quaternion_of(m: Rotation) -> Quaternion:
    """`Matrix3x3::getRotation`: from the trace when it is above 0, else from the largest
    diagonal entry."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        root = math.sqrt(trace + 1.0)
        s = 0.5 / root
        return ((m[2][1] - m[1][2]) * s, (m[0][2] - m[2][0]) * s, (m[1][0] - m[0][1]) * s, root * 0.5)
    if m[0][0] < m[1][1]:
        i = 2 if m[1][1] < m[2][2] else 1
    else:
        i = 2 if m[0][0] < m[2][2] else 0
    j, k = (i + 1) % 3, (i + 2) % 3
    root = math.sqrt(m[i][i] - m[j][j] - m[k][k] + 1.0)
    s = 0.5 / root
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = root * 0.5
    q[3] = (m[k][j] - m[j][k]) * s
    q[j] = (m[j][i] + m[i][j]) * s
    q[k] = (m[k][i] + m[i][k]) * s
    return (q[0], q[1], q[2], q[3])


def _rotated(m: Rotation, v: Vector) -> Vector:
    """`m * v`: each row of `m` dotted with `v`."""
    return tuple(row[0] * v[0] + row[1] * v[1] + row[2] * v[2] for row in m)  # type: ignore[return-value]


def quaternion_of_yaw(yaw: float) -> Quaternion:
    """A turn by `yaw` about +Z, as tf2's `setRPY(0, 0, yaw)` makes it (Quaternion.hpp:98-113)."""
    roll = pitch = 0.0
    half_yaw, half_pitch, half_roll = yaw * 0.5, pitch * 0.5, roll * 0.5
    cos_yaw, sin_yaw = math.cos(half_yaw), math.sin(half_yaw)
    cos_pitch, sin_pitch = math.cos(half_pitch), math.sin(half_pitch)
    cos_roll, sin_roll = math.cos(half_roll), math.sin(half_roll)
    # The roll and the pitch of 0 stay in the arithmetic, so each sign of zero comes out as tf2's.
    return (
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    )


# The sine of the pitch at or past which tf2's `getYaw` reads a heading from x and y alone
# (tf2/impl/utils.hpp:158-161).
_POLE_PITCH_SINE = 0.99999


def yaw_of_quaternion(q: Quaternion) -> float:
    """The heading of `q` as tf2's `getYaw` reads it (tf2/impl/utils.hpp:140-166)."""
    x, y, z, w = q
    sqx, sqy, sqz, sqw = x * x, y * y, z * z, w * w
    pitch_sine = -2.0 * (x * z - w * y) / (sqx + sqy + sqz + sqw)
    if pitch_sine <= -_POLE_PITCH_SINE:
        return -2.0 * math.atan2(y, x)
    if pitch_sine >= _POLE_PITCH_SINE:
        return 2.0 * math.atan2(y, x)
    return math.atan2(2.0 * (x * y + w * z), sqw + sqx - sqy - sqz)


def xyzw_of_wxyz(q: Sequence[float]) -> Quaternion:
    """A quaternion of MuJoCo or Isaac, (w, x, y, z), in the contracts' order."""
    w, x, y, z = q
    return (x, y, z, w)


def wxyz_of_xyzw(q: Sequence[float]) -> tuple[float, float, float, float]:
    """A quaternion of the contracts, (x, y, z, w), in MuJoCo's and Isaac's order."""
    x, y, z, w = q
    return (w, x, y, z)


# ---- the types and their parse ---------------------------------------------------------------


def _display(value: float) -> str:
    """`value` as Rust's `Display` prints an f64: the shortest digits that read back as it, in
    positional notation, with no fractional part when it has none; NaN, inf and -inf by name."""
    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return format(Decimal(repr(value)), "f").removesuffix(".0")


def _listed(values: Sequence[float]) -> str:
    """`values` as a refusal prints them: `[a, b, ...]`, each number as Rust's `Display` prints it."""
    return "[" + ", ".join(_display(v) for v in values) + "]"


def _entries(field: str, values: Sequence[float], count: int, send: str) -> tuple[float, ...]:
    """The `count` numbers of `field`; refused with another number of entries."""
    if len(values) != count:
        raise GeometryError(f"the {field} has {len(values)} entries: send {count}, {send}")
    return tuple(float(v) for v in values)


def _finite(what: str, names: Sequence[str], values: Sequence[float], give: str) -> None:
    if all(math.isfinite(v) for v in values):
        return
    named = ", ".join(f"{n} {_display(v)}" for n, v in zip(names, values))
    raise GeometryError(f"{what} must be finite: got {named}; give {give}")


@dataclass(frozen=True)
class Point:
    """A position (x, y, z) in m, every coordinate finite."""

    position: Vector

    @classmethod
    def parse(cls, position: Sequence[float]) -> Point:
        x, y, z = _entries("position", position, 3, "[x, y, z] in m")
        _finite("a position", ("x", "y", "z"), (x, y, z), "a finite number of meters on each axis")
        return cls((x, y, z))


@dataclass(frozen=True)
class UnitQuaternion:
    """An orientation, a unit quaternion (x, y, z, w), normalized from the one a caller gave."""

    quaternion: Quaternion

    @classmethod
    def parse(cls, orientation: Sequence[float]) -> UnitQuaternion:
        x, y, z, w = _entries("orientation", orientation, 4, "a unit quaternion [x, y, z, w]")
        _finite(
            "an orientation",
            ("x", "y", "z", "w"),
            (x, y, z, w),
            "a unit quaternion [x, y, z, w] of finite numbers, such as [0, 0, 0, 1]",
        )
        # tf2's check of a quaternion sums w first (buffer_core.cpp:250-255), as frame_tree's.
        squared_norm = w * w + x * x + y * y + z * z
        if abs(squared_norm - 1.0) >= _QUATERNION_NORMALIZATION_TOLERANCE:
            raise GeometryError(
                f"the orientation {_listed((x, y, z, w))} has a squared norm of "
                f"{_display(squared_norm)}, {_display(_QUATERNION_NORMALIZATION_TOLERANCE)} or more "
                "from 1: send a unit quaternion [x, y, z, w], such as [0, 0, 0, 1]"
            )
        return cls(normalized((x, y, z, w)))


@dataclass(frozen=True)
class PlanarPose:
    """A pose on the floor plane: x and y in m and a heading in rad, counterclockwise about +Z."""

    x: float
    y: float
    yaw: float


def _tilt_of(q: Quaternion) -> float:
    """The angle (rad) between the frame's +Z and its parent's: `2 asin(sqrt(x^2 + y^2))`."""
    x, y, _, _ = q
    return 2.0 * math.asin(min(1.0, math.sqrt(x * x + y * y)))


@dataclass(frozen=True)
class Pose:
    """A position and an orientation."""

    position: Point
    orientation: UnitQuaternion

    @classmethod
    def parse(cls, position: Sequence[float], orientation: Sequence[float]) -> Pose:
        return cls(Point.parse(position), UnitQuaternion.parse(orientation))

    def planar(self) -> PlanarPose:
        """The pose on the floor plane; refused when its height or its tilt lies farther than
        _PLANAR_TOLERANCE from 0."""
        x, y, z = self.position.position
        if abs(z) > _PLANAR_TOLERANCE:
            raise GeometryError(
                f"the position lies {_display(z)} m above the floor plane: send a position "
                "[x, y, 0] on the floor"
            )
        q = self.orientation.quaternion
        tilt = _tilt_of(q)
        if tilt > _PLANAR_TOLERANCE:
            raise GeometryError(
                f"the orientation {_listed(q)} tilts {_display(tilt)} rad off the vertical: send a "
                "turn about +z alone, [0, 0, sin(yaw/2), cos(yaw/2)]"
            )
        return PlanarPose(x, y, yaw_of_quaternion(q))


@dataclass(frozen=True)
class PoseTarget:
    """The target of a goal: a position, and an orientation or none, as a command leaves out an
    orientation it does not mean."""

    position: Point
    orientation: UnitQuaternion | None

    @classmethod
    def parse(cls, position: Sequence[float], orientation: Sequence[float] | None) -> PoseTarget:
        """The target of `position` and `orientation`, a unit quaternion or None."""
        point = Point.parse(position)
        if orientation is None:
            return cls(point, None)
        return cls(point, UnitQuaternion.parse(orientation))

    def pose(self) -> Pose:
        """The pose of a target that gives an orientation."""
        if self.orientation is None:
            raise GeometryError("the target gives no orientation: send a unit quaternion [x, y, z, w]")
        return Pose(self.position, self.orientation)


@dataclass(frozen=True)
class Stamped[T]:
    """A value in a frame at a time; a `timestamp` of None names the latest time."""

    frame: str
    timestamp: float | None
    value: T

    @classmethod
    def parse(cls, frame: str, timestamp: float | None, value: T) -> Stamped[T]:
        """`value` in `frame` at `timestamp`, seconds from the Unix epoch, or at the latest time
        for None; refused for an empty frame and for a time before the epoch or past the last
        nanosecond a stamp of control_core holds."""
        if not frame:
            raise GeometryError(
                "the frame is empty: name a frame of the robot's tree, such as the robot frame that "
                "sensor_mounts' describe_sensors names"
            )
        return cls(frame, lookup_time(timestamp), value)


def lookup_time(timestamp: float | None) -> float | None:
    """The time of a lookup that `timestamp`, seconds from the Unix epoch, names: None, the latest
    time, as tf2 names `TimePointZero`, for None, and the time given otherwise, the Unix epoch
    among them; refused for a time before the epoch or past the last nanosecond a stamp of
    control_core holds. control_core::geometry's lookup_time, which gives the latest time as
    `LookupTime::Latest`."""
    if timestamp is None:
        return None
    held = math.isfinite(timestamp) and 0.0 <= timestamp
    if not held or Fraction(timestamp) * _NANOS_PER_SECOND >= _STAMP_NANOS_END:
        last = _STAMP_NANOS_END - 1
        raise GeometryError(
            f"the timestamp {_display(timestamp)} s from 1970-01-01T00:00:00Z lies outside the "
            "times a stamp holds: send one from 0 s to "
            f"{last // _NANOS_PER_SECOND}.{last % _NANOS_PER_SECOND:09d} s after it"
        )
    return float(timestamp)
