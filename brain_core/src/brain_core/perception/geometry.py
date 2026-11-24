"""Where the brain places what it sees and what it is asked: the camera
model's intrinsics from the camera itself, through `camera_geometry:v1` on
the `geometry` slot, and the robot's tree of frames, through
`sensor_mounts:v1` on the `sensor_mounts` slot and `transform_lookup:v1` on
the `transform_lookup` slot. get_color_intrinsics gives the pinhole the
detections' pixels are in; get_depth_intrinsics says what a depth sample
measures; describe_sensors names the robot frame, the frame the brain holds
every place in; lookup_transform places a frame in it.

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
and names its robot frame. Its answer holds for as long as the robot runs,
so it is read once. The `Placer` then places by lookup_transform, which
waits up to `lookup_timeout_s`, which the launcher sets, for the transforms
of the lookup's time:

- the frame a picture names, in the robot frame at the picture's capture
  time (`place_picture`), so a camera an arm carries is placed where the
  arm held it when the picture was taken; a picture whose frame the tree
  cannot place is dropped, and the log gives the lookup's failure kind;
- a goal's target, its frame at its timestamp in the robot frame at the
  latest time (`place_target`), as item_manipulation places a target when
  it accepts a goal: a target in the robot frame with no timestamp names
  where the robot frame stands at acceptance, and one with the timestamp of
  a report names where the robot frame stood then.

A lookup that fails gives a `LookupFailure` with the failure kind of the
lookup, its message naming the value placed and then the lookup's cause.
A lookup that gives no answer, or fails with a kind of peppy, gives a
Refusal, `resource_unavailable`, as a robot frame not known yet does.

Each reason is a source that is not there, so its Refusal is
`resource_unavailable`, the kind a search refused for it gives.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from control_core_py.geometry import Placement, Point, PoseTarget, Stamped, UnitQuaternion

from ..ports import LOOKUP_KINDS, FailureKind, Pose, Refusal
from ..waiting import unless_cancelled
from .camera import Intrinsics
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
    the transforms of a lookup's time (s), from 0 to MAX_LOOKUP_TIMEOUT_S,
    as the contract takes it; a ValueError otherwise, so the node stops at
    its start."""
    timeout_s = float(value)
    if not 0.0 <= timeout_s <= MAX_LOOKUP_TIMEOUT_S:
        raise ValueError(
            f"lookup_timeout_s is {value}: set it from 0 to {MAX_LOOKUP_TIMEOUT_S:g} s, "
            "the wait transform_lookup's lookup_transform takes"
        )
    return timeout_s


def robot_frame_of(data) -> str:
    """The robot frame a describe_sensors response names; a ValueError when
    it names none."""
    robot_frame = str(data.robot_frame)
    if not robot_frame:
        raise ValueError("it names no robot frame")
    return robot_frame


class LookupFailure(Refusal):
    """A lookup the robot's tree answered with the failure of the lookup:
    its kind, one of LOOKUP_KINDS, and its message, which names the value
    placed and then the lookup's cause."""


class Placer:
    """The robot's tree as transform_lookup answers it, and the robot frame
    describe_sensors names, which `learn` asks until the robot answers. Each
    lookup waits up to `lookup_timeout_s` for the transforms of its time.
    `reason` says why nothing can be placed yet, or is empty once the robot
    frame is known."""

    def __init__(self, node_runner, describe_sensors, lookup_transform, lookup_timeout_s: float) -> None:
        self._node_runner = node_runner
        self._describe_sensors = describe_sensors
        self._lookup_transform = lookup_transform
        self._lookup_timeout_s = lookup_timeout_s
        self._robot_frame: Optional[str] = None
        self.reason = SourceReason(NOT_DESCRIBED)

    @property
    def robot_frame(self) -> str:
        """The robot frame, or the Refusal, `resource_unavailable`, that says
        why it is not known yet."""
        if self._robot_frame is None:
            raise _not_there(self.reason.reason)
        return self._robot_frame

    async def learn(self, token) -> None:
        """Asks the sensor_mounts slot for the robot frame until the robot
        names one; meanwhile `reason` carries why it has not."""
        producer = self._describe_sensors.bound_producer(self._node_runner)
        if producer is None:
            self.reason.update(MOUNTS_VACANT, starting=False)
            return
        if self._lookup_transform.bound_producer(self._node_runner) is None:
            self.reason.update(LOOKUP_VACANT, starting=False)
            return

        async def ask() -> str:
            answer = await _answer(
                self._describe_sensors, self._node_runner, producer,
                "the robot cannot describe its sensors", "the robot's sensor mounts are not answered yet",
            )
            try:
                return robot_frame_of(answer)
            except ValueError as error:
                raise _not_there(f"the robot's sensor mounts are unusable: {error}") from error

        def learned(robot_frame: str) -> None:
            self._robot_frame = robot_frame
            self.reason.update("", starting=False)
            logger.info("the robot frame is '%s'", robot_frame)

        await _learn(token, self.reason, ask, learned)

    async def place_picture(self, header) -> Placement:
        """Where the frame a picture's `header` names stood in the robot frame
        at the picture's capture time. A picture whose frame the tree cannot
        place is dropped: the log gives its capture_index, the lookup's
        failure kind and its cause. Each refusal is `resource_unavailable`,
        the kind a search refused for its source gives: the dropped picture,
        a robot frame not known yet, or a lookup with no answer."""
        frame, timestamp = str(header.frame), float(header.timestamp)
        try:
            return await self._look_up(frame, timestamp, timestamp, f"the frame '{frame}' at {timestamp:.3f} s")
        except LookupFailure as failure:
            reason = f"picture {header.capture_index} dropped: {failure.kind.value}: {failure.message}"
            logger.warning("%s", reason)
            raise _not_there(reason) from failure

    async def place_target(self, target: Stamped[PoseTarget]) -> Pose:
        """`target` in the robot frame at the latest time, where its frame
        stood at its timestamp, the latest time when it has none, as
        item_manipulation places a goal's target when it accepts the goal; a
        LookupFailure when the tree cannot place it, and a Refusal,
        `resource_unavailable`, when the robot frame is not known or the
        lookup gives no answer; control_core_py's GeometryError for one placed
        past the finite numbers."""
        placement = await self._look_up(target.frame, target.timestamp, None, "the target")
        placed = placement.place(target.value)
        orientation = placed.orientation.xyzw if placed.orientation is not None else None
        return Pose(placed.position.position, orientation)

    async def _look_up(
        self, frame: str, source_timestamp: Optional[float], target_timestamp: Optional[float], what: str
    ) -> Placement:
        """Where `frame` stood at `source_timestamp` in the robot frame as it
        stands at `target_timestamp`, each none for the latest time."""
        robot_frame = self.robot_frame
        lookup = self._lookup_transform
        request = lookup.Request(
            target_frame=robot_frame,
            source_frame=frame,
            source_timestamp=source_timestamp,
            target_timestamp=target_timestamp,
            timeout_s=self._lookup_timeout_s,
        )
        try:
            answer = await lookup.poll(
                self._node_runner, lookup.bound_producer(self._node_runner), request,
                self._lookup_timeout_s + SERVICE_TIMEOUT_S,
            )
        except lookup.Failure as failure:
            kind = failure.kind.value
            if kind not in LOOKUP_KINDS:
                raise _not_there(f"{what}: the robot's transform lookup failed with {kind}: {failure.message}") from failure
            raise LookupFailure(FailureKind(kind), f"{what}: {failure.message}") from failure
        except Exception as error:
            raise _not_there(f"{what}: the robot's transform lookup is not answered ({error!r})") from error
        try:
            return Placement(Point.parse(answer.data.position).position, UnitQuaternion.parse(answer.data.orientation))
        except ValueError as error:
            raise _not_there(f"{what}: the robot's transform lookup gave no pose: {error}") from error
