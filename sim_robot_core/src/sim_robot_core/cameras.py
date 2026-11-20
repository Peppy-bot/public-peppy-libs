"""Camera plumbing shared by the simulation engines.

Reads how a model's entry renders each camera, builds the configs of a
robot's cameras from the optical frames of its description, with the depths
each depth camera measures, and those renders, gives where a rendered
camera's pixels point in the terms of camera_geometry:v1, converts rendered
depth to the z16 wire format, counts captures, and paces per-camera capture;
everything renderer-specific stays in the engine.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from control_core_py.geometry import (
    IDENTITY,
    Point,
    Pose,
    UnitQuaternion,
    product,
    wxyz_of_xyzw,
)

from sim_robot_core.errors import ModelEntryError

# Wire constants shared with the real camera nodes: rgb8 color frames and z16
# depth (u16 little-endian millimeters, 0 = invalid), aligned depth-to-color
# because depth renders from the color camera's own pose.
COLOR_ENCODING = "rgb8"
DEPTH_ENCODING = "z16"
DEPTH_UNIT_M_PER_LSB = 0.001
ALIGN_MODE = "depth_to_color"

# What the geometry topic says of a rendered camera, in the camera_geometry
# contract's terms: an ideal pinhole, its depth the distance to the image
# plane, and depth_to_color, the pose of its depth optical frame in the
# camera's frame, its colour optical frame: the identity, since a rendered
# depth comes from the colour camera's own viewpoint. The pose holds at every
# time, so depth_to_color holds no timestamp.
DISTORTION_MODEL = "none"
DEPTH_MODEL = "z"
DEPTH_TO_COLOR = Pose(Point((0.0, 0.0, 0.0)), UnitQuaternion(IDENTITY))

# The engines' camera frame in the optical frame, (x, y, z, w): half a turn
# about the optical frame's X, which takes its +Z view to -Z and its +Y down
# the picture to +Y up it.
_HALF_TURN_ABOUT_X = (1.0, 0.0, 0.0, 0.0)

# The largest value the z16 wire carries, and the deepest depth it can therefore
# express. Both the encoder and the range check derive from the unit declared
# above, so the unit consumers are told is the unit the bytes are in.
_MAX_DEPTH_LSB = 65535.0
_MAX_RANGE_M = _MAX_DEPTH_LSB * DEPTH_UNIT_M_PER_LSB
# stream_info carries frames_per_second as a u8.
_MAX_FPS = 255


@dataclass(frozen=True)
class DepthSpec:
    width: int
    height: int
    min_depth_m: float
    max_range_m: float

    def __post_init__(self) -> None:
        if not 0.0 < self.min_depth_m < self.max_range_m <= _MAX_RANGE_M:
            raise ValueError(
                f"depth range ({self.min_depth_m}, {self.max_range_m}) must "
                f"satisfy 0 < min < max <= {_MAX_RANGE_M} (the z16 wire ceiling)"
            )


@dataclass(frozen=True)
class DepthStream:
    """The size of the depth image the engines render for a camera that
    measures depth, whose range its description gives."""

    width: int
    height: int


@dataclass(frozen=True)
class CameraRender:
    """How the engines render one camera of a model: its vertical field of
    view, its color stream, its frame rate and, on an rgbd camera, its depth
    stream."""

    fovy_deg: float
    width: int
    height: int
    fps: int
    depth: DepthStream | None


@dataclass(frozen=True)
class CameraConfig:
    """One camera a robot carries: its frame, the link it hangs from and its
    pose in that link's frame, in the engines' camera frame, which looks along
    its own -Z with +Y up the picture and +X to its right; and how it renders."""

    name: str
    # The camera's frame in the robot's tree: its optical frame, the link
    # `<name>_optical_frame` of the description, which sensor_mounts gives for
    # the camera's name and each of its pictures names.
    frame: str
    parent_link: str
    pos: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    fovy_deg: float
    width: int
    height: int
    fps: int
    depth: DepthSpec | None


# Every key each object accepts. A key outside these is a typo, which the
# parser refuses: `dpeth` names no stream of the camera.
_CAMERA_KEYS = frozenset({"fovy_deg", "color", "fps", "depth"})
_COLOR_KEYS = frozenset({"width", "height"})
_DEPTH_KEYS = frozenset({"width", "height"})


def _reject_unknown_keys(obj: dict, allowed: frozenset, what: str, fail) -> None:
    unknown = sorted(set(obj) - allowed)
    if unknown:
        raise fail(f"{what} has unknown key(s) {unknown}; allowed: {sorted(allowed)}")


def parse_renders(source: str, entries) -> dict[str, CameraRender]:
    """How the engines render each camera of a model, by the camera's name:
    `entries`, a model entry's map of camera names to how each renders, read
    once. A bad render is refused, so a typo never distorts a stream.
    `source` names the entry in every refusal."""
    if not isinstance(entries, dict):
        raise ModelEntryError(f"{source}: 'cameras' must map camera names to entries, got {entries!r}")
    return {name: _parse_render(source, name, entry) for name, entry in entries.items()}


def camera_configs(
    source: str, description, renders: dict[str, CameraRender] | None
) -> tuple[CameraConfig, ...]:
    """The configs of a robot's cameras: each optical frame of its
    description (robot_description_py's RobotDescription), placed by the
    fixed joint that holds it to its link, measuring the depths the
    description gives it, rendered as the camera of the same name in
    `renders`, the renders of the entry at `source`, or None where no file is
    there. A camera of the description with no render and a render
    of no camera of the description are refused, so a typo never drops a
    stream: the refusal names the entry, or says to create it where no file
    is there. A camera rendered with a depth stream is one the description
    gives a depth range, and one rendered without is one it gives none; any
    other is refused naming the camera and what to write."""
    frames = description.cameras
    names = [frame.name for frame in frames]
    if renders is None and names:
        raise ModelEntryError(
            f"the cameras of the description are {sorted(names)}, and no entry renders them: "
            f"create {source} with one render per camera of the description, under its name"
        )
    if renders is None:
        return ()
    if set(names) != set(renders):
        raise ModelEntryError(
            f"{source}: the cameras of the description are {sorted(names)}, and the entry "
            f"renders {sorted(renders)}: give the entry one render per camera of the "
            "description, under its name"
        )
    placed = {transform.child: transform for transform in description.fixed_transforms()}
    return tuple(
        _config(source, frame, placed[frame.frame], renders[frame.name]) for frame in frames
    )


def _config(source: str, frame, placed, render: CameraRender) -> CameraConfig:
    """The config of the camera of `frame`, whose optical frame `placed`, the
    fixed transform of its joint, places in its link."""
    pos, quat = camera_pose(placed.position, placed.orientation)
    return CameraConfig(
        name=frame.name,
        frame=frame.frame,
        parent_link=frame.parent_link,
        pos=pos,
        quat_wxyz=quat,
        fovy_deg=render.fovy_deg,
        width=render.width,
        height=render.height,
        fps=render.fps,
        depth=_depth(source, frame, render.depth),
    )


def _depth(source: str, frame, stream: DepthStream | None) -> DepthSpec | None:
    """The depth the camera of `frame` streams: the depth range its
    description gives it, rendered at the size of `stream`, the depth stream
    of its render in the entry at `source`."""
    name = frame.name
    depth_range = frame.depth_range
    if stream is None and depth_range is None:
        return None
    if depth_range is None:
        raise ModelEntryError(
            f"{source}: camera '{name}' renders depth, and the description gives it no depth "
            f'range: give its URDF <gazebo reference="{name}_optical_frame"><sensor name="{name}" '
            'type="depth"><camera><clip><near>NEAR</near><far>FAR</far></clip></camera></sensor>'
            "</gazebo>, in meters, or render it without depth"
        )
    if stream is None:
        raise ModelEntryError(
            f"{source}: the description gives camera '{name}' a depth range of "
            f"{depth_range.near} to {depth_range.far} m, and its render streams no depth: "
            "give it depth: { width, height }"
        )
    try:
        return DepthSpec(
            width=stream.width,
            height=stream.height,
            min_depth_m=depth_range.near,
            max_range_m=depth_range.far,
        )
    except ValueError as exc:
        raise ModelEntryError(f"{source}: camera '{name}': {exc}") from exc


def camera_pose(
    position, orientation
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """The pose in the engines' camera frame of a camera whose optical frame
    stands at `position` turned by `orientation`, a unit quaternion
    (x, y, z, w), in its link: the optical frame looks along its +Z with +Y
    down the picture, and the camera frame is the optical frame turned half a
    turn about its own X, which looks along -Z with +Y up. The orientation is
    (w, x, y, z), the engines' order."""
    x, y, z = (float(coordinate) for coordinate in position)
    camera = product(tuple(float(c) for c in orientation), _HALF_TURN_ABOUT_X)
    return (x, y, z), wxyz_of_xyzw(camera)


def _parse_render(source: str, name: str, entry) -> CameraRender:
    def fail(reason: str) -> ModelEntryError:
        return ModelEntryError(f"{source}: camera '{name}': {reason}")

    if not isinstance(entry, dict):
        raise fail(f"its entry must be an object, got {entry!r}")
    _reject_unknown_keys(entry, _CAMERA_KEYS, "camera entry", fail)
    fovy_deg = entry.get("fovy_deg")
    if not _is_number(fovy_deg) or not 0.0 < float(fovy_deg) < 180.0:
        raise fail(f"fovy_deg {fovy_deg!r} must be in (0, 180)")
    color = entry.get("color")
    if not isinstance(color, dict):
        raise fail(f"color must be an object with width/height, got {color!r}")
    _reject_unknown_keys(color, _COLOR_KEYS, "color", fail)
    width = _positive_int(color.get("width"), fail, "color.width")
    height = _positive_int(color.get("height"), fail, "color.height")
    fps = _positive_int(entry.get("fps"), fail, "fps")
    if fps > _MAX_FPS:
        raise fail(f"fps {fps} does not fit the u8 wire type")

    depth = None
    if "depth" in entry:
        d = entry["depth"]
        if not isinstance(d, dict):
            raise fail(f"depth must be an object, got {d!r}")
        _reject_unknown_keys(d, _DEPTH_KEYS, "depth", fail)
        depth = DepthStream(
            width=_positive_int(d.get("width"), fail, "depth.width"),
            height=_positive_int(d.get("height"), fail, "depth.height"),
        )
        # Color and depth describe one camera, so the depth grid must decimate
        # the color grid by a single integer factor on both axes: the two grids
        # then share a principal point and an aspect ratio whichever way an
        # engine produces them.
        if (
            width % depth.width != 0
            or height % depth.height != 0
            or width // depth.width != height // depth.height
        ):
            raise fail(
                f"depth {depth.width}x{depth.height} must subsample color "
                f"{width}x{height} by one integer factor"
            )

    return CameraRender(fovy_deg=float(fovy_deg), width=width, height=height, fps=fps, depth=depth)


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _positive_int(value, fail, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise fail(f"{field} must be a positive integer, got {value!r}")
    return value


@dataclass(frozen=True)
class Pinhole:
    """The pinhole model of one published grid, in the camera_geometry
    contract's pixel convention, which is OpenCV's: the centre of the top-left
    pixel is (0, 0), so pixel centres sit on whole numbers. Focal lengths and
    principal point are in pixels."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def pinhole(fovy_deg: float, width: int, height: int) -> Pinhole:
    """The model of a grid rendered `width` x `height` under a vertical field
    of view of `fovy_deg`: square pixels and the optical axis through the image
    centre, which is what an MJCF or a USD camera renders. With pixel centres
    on whole numbers the middle of a `width` x `height` image is
    ((width - 1) / 2, (height - 1) / 2)."""
    focal = (height / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return Pinhole(width, height, focal, focal, (width - 1) / 2.0, (height - 1) / 2.0)


def depth_to_z16(depth_m: np.ndarray, spec: DepthSpec) -> bytes:
    """Metric depth (float meters, HxW) to the z16 wire format: little-endian
    u16 counts of DEPTH_UNIT_M_PER_LSB rounded to the nearest, 0 for anything
    non-finite or outside [min_depth_m, max_range_m], clipped away from 0
    otherwise so a valid depth never reads as the invalid marker."""
    if depth_m.shape != (spec.height, spec.width):
        raise ValueError(
            f"depth shape {depth_m.shape} does not match spec {(spec.height, spec.width)}"
        )
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= spec.min_depth_m)
        & (depth_m <= spec.max_range_m)
    )
    lsb = np.rint(depth_m / DEPTH_UNIT_M_PER_LSB)
    return np.where(valid, np.clip(lsb, 1.0, _MAX_DEPTH_LSB), 0.0).astype("<u2").tobytes()


class CaptureIndexCounter:
    """The capture_index of each capture: a wrapping u32 counter; the colour
    and depth pictures of one rgbd capture share one index."""

    def __init__(self) -> None:
        self._next = 0

    def next(self) -> int:
        value = self._next
        self._next = (self._next + 1) & 0xFFFFFFFF
        return value


class FramePacer:
    """Absolute-deadline pacing at a fixed fps against a monotonic clock.
    Deadlines advance by exact periods so the long-run rate holds. The grid
    may trail the clock by under one period, so ticks arriving at the paced
    rate with jitter are all taken rather than every other one; once it
    trails by a full period the schedule resyncs to now instead of bursting
    to catch up."""

    def __init__(self, fps: int) -> None:
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        self._period = 1.0 / fps
        self._deadline: float | None = None

    def seconds_until_due(self, now: float) -> float:
        """Return the remaining wait without claiming a frame or moving the deadline."""
        if self._deadline is None:
            return 0.0
        return max(0.0, self._deadline - now)

    def take_if_due(self, now: float) -> bool:
        """Claim the slot due at time now, advancing the schedule: True exactly
        once per period, and the caller owes that period a frame."""
        if self._deadline is None:
            self._deadline = now + self._period
            return True
        if now < self._deadline:
            return False
        self._deadline += self._period
        if now - self._deadline >= self._period:
            self._deadline = now + self._period
        return True
