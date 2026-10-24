"""Where the camera model comes from: its intrinsics from the camera
itself, through `camera_geometry:v1` on the `geometry` slot, and its pose
from the robot, through `camera_mounts:v1` on the `camera_mounts` slot.
get_color_intrinsics gives the pinhole the detections' pixels are in;
get_depth_intrinsics says what a depth sample measures; get_camera_poses
says where the camera named by the `camera_name` parameter stands in the
robot frame.

The slot is optional, like the camera's. Vacant, the intrinsics never come
and every search is refused naming the slot. Bound, both services are asked
again every `INTRINSICS_RETRY_S` until each has given its response and the
answers are usable, and until then a search is refused with the last
reason they were not. A camera that does not know what is asked gives an
error reply: `precondition_failed` before it has heard from its
simulation, `rejected` for a device without a calibration. The brain keeps
asking whatever the kind, so a camera node that starts again with what it
lacked is heard: the first answers within seconds, the second gives its
error reply for the node's whole life and the log says so once. A depth
model other than "z" is a reason too: the brain reads a sample as the
distance along the optical axis, which is what the camera model
deprojects. The intrinsics are those of the colour stream, which the
detections' pixels are in; the depth is read at the same pixels, so the
frame store refuses a pair of frames that are not aligned.

The pose is asked the same way: until the robot gives its response and
names the camera, every search is refused with the reason, and the log
says it once. A robot that has not measured its joints yet gives an error
reply. The pose is read once: the brain looks through a camera fixed to
the robot's base, and reports the values the robot's design gives, exact
in a simulation and nominal on hardware.

Each reason is a source that is not there, so its Refusal is
`resource_unavailable`, the kind a search refused for it gives.
"""

from __future__ import annotations

import asyncio
import logging

from ..ports import FailureKind, Refusal
from ..waiting import unless_cancelled
from .camera import CameraModel, Intrinsics

logger = logging.getLogger(__name__)

INTRINSICS_TIMEOUT_S = 5.0
INTRINSICS_RETRY_S = 2.0
VACANT = "the geometry slot is vacant: link the camera's camera_geometry to it"
MOUNTS_VACANT = "the camera_mounts slot is vacant: link the robot's camera_mounts to it"
# The depth model the brain reads: a sample is the distance along the
# optical axis.
Z_DEPTH = "z"


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


async def _answer(service, node_runner, producer, unknown: str, unanswered: str):
    """The response data of one poll of `service`, a generated consumed
    service module; a Refusal that starts with `unknown` for its error
    reply, or with `unanswered` for no answer at all."""
    try:
        return (await service.poll(node_runner, producer, INTRINSICS_TIMEOUT_S)).data
    except service.Failure as failure:
        raise _not_there(f"{unknown}: {failure.message}") from failure
    except Exception as error:
        raise _not_there(f"{unanswered} ({error!r})") from error


def _not_there(reason: str) -> Refusal:
    return Refusal(FailureKind.RESOURCE_UNAVAILABLE, reason)


async def _learn(token, perceiver, reason_attribute: str, ask, learned) -> None:
    """Asks `ask` for the camera until it answers one, then gives it to the
    perceiver and logs it through `learned`; meanwhile the perceiver's
    `reason_attribute` carries the last reason the answer was refused."""
    reported = ""
    while not token.is_cancelled():
        try:
            camera = await ask()
        except Refusal as refusal:
            setattr(perceiver, reason_attribute, refusal.message)
            if refusal.message != reported:
                logger.info("%s; asking again every %g s", refusal.message, INTRINSICS_RETRY_S)
                reported = refusal.message
            await unless_cancelled(token, asyncio.sleep(INTRINSICS_RETRY_S))
            continue
        perceiver.set_camera(camera)
        learned(camera)
        return


async def learn_intrinsics(node_runner, token, perceiver, get_color_intrinsics, get_depth_intrinsics) -> None:
    """Asks the geometry slot for the camera's intrinsics until it has
    them, and gives the perceiver its camera; meanwhile the perceiver
    carries the reason it has none."""
    producer = get_color_intrinsics.bound_producer(node_runner)
    if producer is None:
        perceiver.intrinsics_reason = VACANT
        return

    async def ask() -> CameraModel:
        # The perceiver's camera is read after the answer, so a pose the
        # other learner gave it meanwhile is kept.
        intrinsics = await camera_intrinsics(node_runner, producer, get_color_intrinsics, get_depth_intrinsics)
        return perceiver.camera.with_intrinsics(intrinsics)

    def learned(camera: CameraModel) -> None:
        k = camera.intrinsics
        logger.info(
            "camera geometry: %dx%d fx %.1f fy %.1f cx %.1f cy %.1f %s",
            k.width, k.height, k.fx, k.fy, k.cx, k.cy, k.distortion_model,
        )

    await _learn(token, perceiver, "intrinsics_reason", ask, learned)


def camera_pose_from(data, camera_name: str):
    """The pose of `camera_name` in a get_camera_poses response, as
    (position, orientation); a Refusal when the response names no such
    camera."""
    names = list(data.camera_names)
    if camera_name not in names:
        carried = ", ".join(names) if names else "none"
        raise _not_there(f"the robot carries no camera named '{camera_name}' (it carries: {carried})")
    index = names.index(camera_name)
    position = tuple(float(v) for v in data.positions[index * 3 : index * 3 + 3])
    orientation = tuple(float(v) for v in data.orientations[index * 4 : index * 4 + 4])
    return position, orientation


async def camera_pose(node_runner, producer, camera_name: str, get_camera_poses):
    """One question to the robot, through the node's generated
    camera_mounts module: where `camera_name` stands, or a Refusal naming
    why the answer is not usable."""
    answer = await _answer(
        get_camera_poses, node_runner, producer,
        "the robot does not know where its cameras stand", "the robot's camera mounts are not answered yet",
    )
    return camera_pose_from(answer, camera_name)


async def learn_camera_pose(node_runner, token, perceiver, camera_name: str, get_camera_poses) -> None:
    """Asks the camera_mounts slot where the camera stands until the robot
    answers and names it, and gives the perceiver its camera; meanwhile the
    perceiver carries the reason it is not placed."""
    producer = get_camera_poses.bound_producer(node_runner)
    if producer is None:
        perceiver.pose_reason = MOUNTS_VACANT
        return

    async def ask() -> CameraModel:
        position, orientation = await camera_pose(node_runner, producer, camera_name, get_camera_poses)
        try:
            return perceiver.camera.with_pose(position, orientation)
        except ValueError as error:
            raise _not_there(str(error)) from error

    def learned(camera: CameraModel) -> None:
        logger.info(
            "camera '%s' stands at (%.3f, %.3f, %.3f) in the robot frame, looking along %s",
            camera_name, *camera.pose.position, tuple(round(v, 3) for v in camera.forward()),
        )

    await _learn(token, perceiver, "pose_reason", ask, learned)
