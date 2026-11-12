"""Pose transforms between the wire's (position, quaternion) and the 4x4
matrices lerobot's kinematics speaks. Quaternions are [x, y, z, w]."""

from __future__ import annotations

import math

import numpy as np

_NORMALIZATION_TOLERANCE = 1e-3


class PoseError(ValueError):
    """A wire pose that cannot become a transform."""


# Above this dot product two unit quaternions are one orientation to slerp's
# rounding: the sine it divides by vanishes.
_SAME_ORIENTATION_DOT = 0.9995


def matrix_from_pose(position, orientation) -> np.ndarray:
    if len(position) != 3 or len(orientation) != 4:
        raise PoseError(
            f"expected 3 position and 4 orientation values, "
            f"got {len(position)} and {len(orientation)}"
        )
    values = [*position, *orientation]
    if not all(math.isfinite(v) for v in values):
        raise PoseError("non-finite pose component")
    x, y, z, w = orientation
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.0) > _NORMALIZATION_TOLERANCE:
        raise PoseError(f"orientation is not a unit quaternion (norm {norm:.4f})")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    matrix[:3, 3] = position
    return matrix


def pose_from_matrix(matrix: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """(position, quaternion [x, y, z, w]) of a rigid transform, w kept
    non-negative so equal rotations compare equal."""
    r = matrix[:3, :3]
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    position = tuple(float(v) for v in matrix[:3, 3])
    return position, (float(x), float(y), float(z), float(w))


def relative_rotation_rad(q0, q1) -> float:
    """Angle of the relative rotation between two unit quaternions; |dot|
    folds the double cover."""
    dot = abs(sum(a * b for a, b in zip(q0, q1, strict=True)))
    return 2.0 * math.acos(min(1.0, dot))


def slerp(q0, q1, share: float) -> tuple[float, float, float, float]:
    """The unit quaternion at `share` of the shorter turn from `q0` to `q1`;
    `q1`'s own at `share` 1, and a blend of the two where they are one
    orientation to the rounding slerp divides by."""
    if share >= 1.0:
        return tuple(q1)
    dot = sum(a * b for a, b in zip(q0, q1, strict=True))
    near = tuple(-v for v in q1) if dot < 0.0 else tuple(q1)
    dot = abs(dot)
    if dot > _SAME_ORIENTATION_DOT:
        blended = [a + (b - a) * share for a, b in zip(q0, near, strict=True)]
    else:
        turn = math.acos(dot)
        blended = [
            (math.sin((1.0 - share) * turn) * a + math.sin(share * turn) * b) / math.sin(turn)
            for a, b in zip(q0, near, strict=True)
        ]
    norm = math.sqrt(sum(v * v for v in blended))
    return tuple(v / norm for v in blended)
