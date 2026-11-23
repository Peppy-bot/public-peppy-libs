"""The one frame of a simulation, world, as the simulation contracts give it.

Every report of a simulation engine gives its values in world, and a command names world or
leaves its frame out: a command in another frame is refused with `unknown_frame` (UnknownFrame),
and its timestamp, given or left out, places its value in world at any time, since world does
not move. A value no contract takes is refused with `rejected` by control_core_py's parse
(GeometryError). `failure_kind_of` gives the failure kind of each such refusal, and
`attach_placement` reads where an attach of simulation_robot places its robot. A robot that
rolls stands at the placement its wheels carried it to (`rolled_placement`).
"""

from __future__ import annotations

from collections.abc import Sequence

from control_core_py.geometry import (
    WORLD,
    Point,
    Pose,
    PoseTarget,
    Stamped,
    UnitQuaternion,
    quaternion_of_yaw,
    yaw_of_quaternion,
)

# The failure kinds that refuse a command given in world, by their name on the wire: a frame
# other than world, and a value no contract takes.
UNKNOWN_FRAME = "unknown_frame"
REJECTED = "rejected"


class UnknownFrame(ValueError):
    """A command in a frame other than world, refused with `unknown_frame`; `str()` names the
    frame given and says to send world."""


def failure_kind_of(error: ValueError) -> str:
    """The failure kind that refuses a command whose parse raised `error`: `unknown_frame` for
    UnknownFrame, a frame other than world, and `rejected` for any other, a value no contract
    takes (GeometryError) among them."""
    return UNKNOWN_FRAME if isinstance(error, UnknownFrame) else REJECTED


def in_world[T](frame: str | None, timestamp: float | None, value: T) -> T:
    """`value`, given in `frame` at `timestamp`, a frame of None naming world and a timestamp of
    None the latest time: refused with UnknownFrame for a frame other than world, then with
    GeometryError for a timestamp the shared parse refuses."""
    if frame is not None and frame != WORLD:
        raise UnknownFrame(
            f"the frame {frame!r} is not {WORLD}, the one frame of the simulation: send {WORLD}"
        )
    return Stamped.parse(WORLD, timestamp, value).value


def base_target(
    frame: str | None,
    timestamp: float | None,
    position: Sequence[float],
    orientation: Sequence[float] | None,
) -> PoseTarget:
    """Where a robot's base is to stand: a position in world and, when `orientation` is not
    None, the way the base faces, an orientation that turns about +z alone. A frame other
    than world raises UnknownFrame; a position or an orientation the shared parse refuses, and
    an orientation that tilts, raise GeometryError."""
    target = in_world(frame, timestamp, PoseTarget.parse(position, orientation))
    if target.orientation is not None:
        # A pose at the floor's origin with this orientation is planar exactly when the
        # orientation turns about +z alone; Pose.planar refuses it otherwise, saying what to send.
        Pose(Point((0.0, 0.0, 0.0)), target.orientation).planar()
    return target


def attach_placement(placement) -> Pose | None:
    """Where an attach goal of simulation_robot places its robot: `placement`, the goal's record
    of frame, timestamp, position and orientation, as a pose in world whose orientation turns
    about +z alone, or None when the goal gives no record and leaves the spot to the engine. A
    frame of None names world, and a frame other than world raises UnknownFrame; a pose the
    shared parse refuses, one with no orientation among them, and an orientation that tilts
    raise GeometryError."""
    if placement is None:
        return None
    return base_target(
        placement.frame, placement.timestamp, placement.position, placement.orientation
    ).pose()


def rolled_placement(position: Sequence[float], orientation: Sequence[float]) -> Pose:
    """The placement of a robot that rolls, whose robot frame stands at `position` turned by
    `orientation`, a unit quaternion (x, y, z, w), in world: that position, facing the heading
    its wheels turned it to, an orientation that turns about +z alone. A placement is where a
    robot stands, so the tilt a step of the floor gives its body is left out."""
    heading = quaternion_of_yaw(yaw_of_quaternion(tuple(orientation)))
    return Pose(Point.parse(position), UnitQuaternion(heading))
