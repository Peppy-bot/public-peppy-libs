"""Where the camera model comes from: its intrinsics from the camera
itself, through `camera_geometry:v1` on the `geometry` slot, and its pose
at the time of each picture from the robot's tree of frames, through
`sensor_mounts:v1` on the `sensor_mounts` slot and `transform_lookup:v1` on
the `transform_lookup` slot. get_color_intrinsics gives the pinhole the
detections' pixels are in; get_depth_intrinsics says what a depth sample
measures; describe_sensors names the robot frame and the frame of each
camera the robot carries; lookup_transform places a camera's frame in the
robot frame at the time a picture was taken.

The slots are optional, like the camera's. Vacant, the source never comes
and every search is refused naming the slot. Bound, the camera's services
are asked again every `RETRY_S` until each has given its response and the
answers are usable, and until then a search is refused with the last
reason they were not. A camera that does not know what is asked gives an
error reply: `precondition_failed` before it has heard from its
simulation, `rejected` for a device without a calibration. The brain keeps
asking whatever the kind, so a camera node that starts again with what it
lacked is heard: the first answers within seconds, the second gives its
error reply for the node's whole life and the log says so once. So no
answer and the error reply `precondition_failed` are reasons that end on
their own while the source starts (`SourceStarting`), and every other
reason holds until the source changes, which get_model of brain_model
tells apart. A depth
model other than "z" is a reason too: the brain reads a sample as the
distance along the optical axis, which is what the camera model
deprojects. The intrinsics are those of the colour stream, which the
detections' pixels are in; the depth is read at the same pixels, so the
frame store refuses a pair of frames that are not aligned.

describe_sensors is asked the same way, until the robot gives its response
and names the camera of the `camera_name` parameter among its cameras. Its
answer holds for as long as the robot runs, so it is read once. A camera's
pose is then asked at the time of each picture: lookup_transform waits up
to `lookup_timeout_s`, which the launcher sets, for the transforms of that
time, and a lookup that fails refuses the search that needed it with the
lookup's words. A camera an arm carries is placed where the arm held it
when the picture was taken.

Each reason is a source that is not there, so its Refusal is
`resource_unavailable`, the kind a search refused for it gives.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Mapping, Optional

from ..ports import FailureKind, Refusal
from ..waiting import unless_cancelled
from .camera import CameraPose, Intrinsics
from .perceiver import SourceReason

logger = logging.getLogger(__name__)

# How long one answer of a service of the camera or of the robot is waited
# for, beyond the wait the request itself asks for, and how long the brain
# waits before it asks again a service whose answer it cannot use.
SERVICE_TIMEOUT_S = 5.0
RETRY_S = 2.0
VACANT = "the geometry slot is vacant: link the camera's camera_geometry to it"
MOUNTS_VACANT = "the sensor_mounts slot is vacant: link the robot's sensor_mounts to it"
LOOKUP_VACANT = "the transform_lookup slot is vacant: link the robot's transform_lookup to it"
NOT_DESCRIBED = "the robot's sensors have not been described yet"
# The longest wait (s) a request of transform_lookup's lookup_transform may
# ask for; the implementer refuses a longer one with `rejected`.
MAX_LOOKUP_TIMEOUT_S = 1.0
# The contracts a camera streams on, as describe_sensors names them: the
# frame of each is the camera's colour optical frame.
CAMERA_KINDS = ("rgb_camera", "rgbd_camera")
# The depth model the brain reads: a sample is the distance along the
# optical axis.
Z_DEPTH = "z"
# The error reply of a source that has not heard yet from what it reads.
PRECONDITION_FAILED = "precondition_failed"


def intrinsics_from(data) -> Intrinsics:
    """The camera model of a get_color_intrinsics response; a ValueError
    names what is unusable."""
    return Intrinsics(
        int(data.width), int(data.height), float(data.fx), float(data.fy), float(data.cx), float(data.cy),
        str(data.distortion_model), tuple(float(c) for c in (data.distortion or ())),
    )


async def camera_intrinsics(node_runner, producer, get_color_intrinsics, get_depth_intrinsics) -> Intrinsics:
    """One round of both questions, through the node's generated geometry
    modules: the colour intrinsics when the camera gives both responses,
    measures depth along the optical axis and reports a usable pinhole; a
    Refusal naming the reason otherwise."""
    color = await _answer(
        get_color_intrinsics, node_runner, producer,
        "the camera does not know its colour intrinsics", "the camera's geometry is not answered yet",
    )
    depth = await _answer(
        get_depth_intrinsics, node_runner, producer,
        "the camera does not know its depth intrinsics", "the camera's geometry is not answered yet",
    )
    if depth.depth_model != Z_DEPTH:
        raise _not_there(
            f"the camera's depth samples are '{depth.depth_model}' distances; the brain reads distances along the optical axis ('{Z_DEPTH}')"
        )
    try:
        return intrinsics_from(color)
    except ValueError as error:
        raise _not_there(f"the camera's intrinsics are unusable: {error}") from error


class SourceStarting(Refusal):
    """The Refusal of a source that has not given what the brain reads yet
    and may give it on its own: it has not answered, or it gave the error
    reply `precondition_failed`, which a camera gives before it has heard
    from its simulation. Any other Refusal of a source holds until the
    source changes."""


async def _answer(service, node_runner, producer, unknown: str, unanswered: str):
    """The response data of one poll of `service`, a generated consumed
    service module whose request has no fields; a Refusal that starts with
    `unknown` for its error reply, or with `unanswered` for no answer at
    all. The Refusal of no answer, and of the error reply
    `precondition_failed`, is a SourceStarting."""
    try:
        return (await service.poll(node_runner, producer, SERVICE_TIMEOUT_S)).data
    except service.Failure as failure:
        reason = f"{unknown}: {failure.message}"
        if failure.kind == PRECONDITION_FAILED:
            raise SourceStarting(FailureKind.RESOURCE_UNAVAILABLE, reason) from failure
        raise _not_there(reason) from failure
    except Exception as error:
        raise SourceStarting(FailureKind.RESOURCE_UNAVAILABLE, f"{unanswered} ({error!r})") from error


def _not_there(reason: str) -> Refusal:
    return Refusal(FailureKind.RESOURCE_UNAVAILABLE, reason)


async def _learn(token, source: SourceReason, ask, learned) -> None:
    """Asks `ask` until it answers, then hands the answer to `learned`;
    meanwhile `source` carries the last reason the answer was refused, and
    whether it is a SourceStarting."""
    reported = ""
    while not token.is_cancelled():
        try:
            answer = await ask()
        except Refusal as refusal:
            source.update(refusal.message, starting=isinstance(refusal, SourceStarting))
            if refusal.message != reported:
                logger.info("%s; asking again every %g s", refusal.message, RETRY_S)
                reported = refusal.message
            await unless_cancelled(token, asyncio.sleep(RETRY_S))
            continue
        learned(answer)
        return


async def learn_intrinsics(node_runner, token, perceiver, get_color_intrinsics, get_depth_intrinsics) -> None:
    """Asks the geometry slot for the camera's intrinsics until it has
    them, and gives the perceiver its camera; meanwhile the perceiver
    carries the reason it has none."""
    producer = get_color_intrinsics.bound_producer(node_runner)
    if producer is None:
        perceiver.intrinsics.update(VACANT, starting=False)
        return

    async def ask() -> Intrinsics:
        return await camera_intrinsics(node_runner, producer, get_color_intrinsics, get_depth_intrinsics)

    def learned(k: Intrinsics) -> None:
        perceiver.set_intrinsics(k)
        logger.info(
            "camera geometry: %dx%d fx %.1f fy %.1f cx %.1f cy %.1f %s",
            k.width, k.height, k.fx, k.fy, k.cx, k.cy, k.distortion_model,
        )

    await _learn(token, perceiver.intrinsics, ask, learned)


def parse_lookup_timeout(value) -> float:
    """The `lookup_timeout_s` parameter: how long lookup_transform waits for
    the transforms of a picture's time (s), from 0 to MAX_LOOKUP_TIMEOUT_S,
    as the contract takes it; a ValueError otherwise, so the node stops at
    its start."""
    timeout_s = float(value)
    if not 0.0 <= timeout_s <= MAX_LOOKUP_TIMEOUT_S:
        raise ValueError(
            f"lookup_timeout_s is {value}: set it from 0 to {MAX_LOOKUP_TIMEOUT_S:g} s, "
            "the wait transform_lookup's lookup_transform takes"
        )
    return timeout_s


@dataclass(frozen=True)
class Sensor:
    """One sensor of the robot as describe_sensors gives it: the contract it
    streams on and its frame in the robot's tree."""

    kind: str
    frame: str


@dataclass(frozen=True)
class SensorFrames:
    """What describe_sensors says of the robot: the robot frame, and each
    sensor by the name it runs under."""

    robot_frame: str
    sensors: Mapping[str, Sensor]

    @classmethod
    def parse(cls, data) -> "SensorFrames":
        """The frames of a describe_sensors response; a ValueError names
        what is unusable: an empty robot frame, a sensor name given twice or
        empty, an empty sensor frame, or arrays that do not run beside
        sensor_names."""
        robot_frame = str(data.robot_frame)
        if not robot_frame:
            raise ValueError("it names no robot frame")
        names = [str(name) for name in data.sensor_names]
        kinds = [str(kind) for kind in data.sensor_kinds]
        frames = [str(frame) for frame in data.sensor_frames]
        if not len(names) == len(kinds) == len(frames) == len(data.carried_by):
            raise ValueError(
                f"it names {len(names)} sensors and gives {len(kinds)} kinds, {len(frames)} frames "
                f"and {len(data.carried_by)} carriers"
            )
        if "" in names or len(set(names)) != len(names):
            raise ValueError(f"its sensor names are not distinct and not empty: {names}")
        if "" in frames:
            raise ValueError(f"a sensor of {names} has no frame")
        return cls(robot_frame, {name: Sensor(kind, frame) for name, kind, frame in zip(names, kinds, frames)})

    def camera_frame(self, name: str) -> str:
        """The frame of the camera `name`, its colour optical frame; a
        Refusal when the robot carries no camera of that name."""
        sensor = self.sensors.get(name)
        if sensor is not None and sensor.kind in CAMERA_KINDS:
            return sensor.frame
        cameras = [other for other, each in self.sensors.items() if each.kind in CAMERA_KINDS]
        carried = ", ".join(cameras) if cameras else "none"
        if sensor is None:
            raise _not_there(f"the robot carries no camera named '{name}' (its cameras: {carried})")
        raise _not_there(f"the robot's sensor '{name}' streams on {sensor.kind}, not as a camera (its cameras: {carried})")


class CameraPlacer:
    """Where the robot's cameras stand in the robot frame at a time. The
    robot frame and each camera's frame come from describe_sensors, which
    `learn` asks until the robot answers and names the brain's camera, and
    a camera's pose at a time from lookup_transform, which waits up to
    `lookup_timeout_s` for the transforms of that time. `reason` says why
    no camera can be placed yet, or is empty once one can."""

    def __init__(self, node_runner, describe_sensors, lookup_transform, lookup_timeout_s: float) -> None:
        self._node_runner = node_runner
        self._describe_sensors = describe_sensors
        self._lookup_transform = lookup_transform
        self._lookup_timeout_s = lookup_timeout_s
        self.frames: Optional[SensorFrames] = None
        self.reason = SourceReason(NOT_DESCRIBED)

    async def learn(self, token, camera_name: str) -> None:
        """Asks the sensor_mounts slot for the robot's sensors until the
        robot answers and names `camera_name` among its cameras; meanwhile
        `reason` carries why it has not."""
        producer = self._describe_sensors.bound_producer(self._node_runner)
        if producer is None:
            self.reason.update(MOUNTS_VACANT, starting=False)
            return
        if self._lookup_transform.bound_producer(self._node_runner) is None:
            self.reason.update(LOOKUP_VACANT, starting=False)
            return

        async def ask() -> SensorFrames:
            answer = await _answer(
                self._describe_sensors, self._node_runner, producer,
                "the robot cannot describe its sensors", "the robot's sensor mounts are not answered yet",
            )
            try:
                frames = SensorFrames.parse(answer)
            except ValueError as error:
                raise _not_there(f"the robot's sensor mounts are unusable: {error}") from error
            frames.camera_frame(camera_name)
            return frames

        def learned(frames: SensorFrames) -> None:
            self.frames = frames
            self.reason.update("", starting=False)
            logger.info(
                "camera '%s' is the frame '%s' of the robot, whose robot frame is '%s'",
                camera_name, frames.camera_frame(camera_name), frames.robot_frame,
            )

        await _learn(token, self.reason, ask, learned)

    async def pose_at(self, camera_name: str, timestamp: float) -> CameraPose:
        """The pose of the camera `camera_name` in the robot frame at
        `timestamp`, the time a picture of it was taken; a Refusal naming
        why it cannot be had: the robot's sensors are not known yet, the
        robot carries no such camera, or the lookup failed."""
        if self.frames is None:
            raise _not_there(self.reason.reason)
        frame = self.frames.camera_frame(camera_name)
        lookup = self._lookup_transform
        request = lookup.Request(
            target_frame=self.frames.robot_frame, source_frame=frame, time=timestamp, timeout_s=self._lookup_timeout_s
        )
        try:
            answer = await lookup.poll(
                self._node_runner, lookup.bound_producer(self._node_runner), request,
                self._lookup_timeout_s + SERVICE_TIMEOUT_S,
            )
        except lookup.Failure as failure:
            raise _not_there(f"the robot cannot place camera '{camera_name}' at {timestamp:.3f} s: {failure.message}") from failure
        except Exception as error:
            raise _not_there(f"the robot's transform lookup is not answered ({error!r})") from error
        try:
            return CameraPose.parse(answer.data.position, answer.data.orientation)
        except ValueError as error:
            raise _not_there(f"the robot's pose of camera '{camera_name}' is unusable: {error}") from error
