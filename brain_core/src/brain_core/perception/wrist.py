"""The wrist cameras: the newest colour frame of the camera each arm
carries, kept by a small loop the node starts.

The node's `wrist_cameras` slot takes any number of rgb_camera producers,
and the `wrist_cameras` parameter names each arm's camera by the instance
id its fragment gives it ("left_arm:wrist_left,right_arm:wrist_right"),
the name camera_mounts knows it by. A frame is matched to its camera by
the producer that published it: in a stack that runs the robot as a copy,
the instance runs as `<copy>_<id>` (`alpha_wrist_left`), and the slot's
bound members say which copy each producer belongs to, so the store takes
the copy's prefix off. The launcher binds the cameras and the parameter
names them; nothing here knows how a camera is mounted. An arm the
parameter does not name has no wrist camera: an alignment of that arm is
unavailable and says so before any move.

A frame's stamp is the camera's capture time, on the camera's clock, so
the store compares stamps only with each other: `newer_than` waits for a
frame captured after another one, which is how a step gets a picture of
the arm where it stands now and not where it was.

`jpeg_of` gives a frame as JPEG bytes for a model: the frame's own bytes
when the camera streams MJPEG, else the decoded picture encoded once.
"""

from __future__ import annotations

import asyncio
import io
import logging
from typing import Any, Mapping, Optional, Sequence

from ..ports import Refusal
from ..waiting import unless_cancelled
from .frames import decode_color

logger = logging.getLogger(__name__)

JPEG_QUALITY = 95
VACANT = "no wrist camera source: this brain has no wrist_cameras slot"
NO_SOURCE = "no wrist camera for {arm}: the wrist_cameras parameter names none"

# A message of the node's generated wrist video_stream module.
ColorMessage = Any


def parse_wrist_cameras(text: str, arms: Sequence[str]) -> dict[str, str]:
    """The camera of each arm from "arm:camera,arm:camera": each arm one
    of `arms`, named at most once, each camera id non-empty and named
    once. Empty names no camera at all. A ValueError names what is wrong."""
    cameras: dict[str, str] = {}
    for pair in (piece.strip() for piece in text.split(",") if piece.strip()):
        arm, colon, camera = (part.strip() for part in pair.partition(":"))
        if not colon or not arm or not camera:
            raise ValueError(f"wrist_cameras: '{pair}' is not 'arm:camera'")
        if arm not in arms:
            raise ValueError(f"wrist_cameras: '{arm}' is not an arm of this robot (the arms are {', '.join(arms)})")
        if arm in cameras:
            raise ValueError(f"wrist_cameras: arm '{arm}' is named twice")
        if camera in cameras.values():
            raise ValueError(f"wrist_cameras: camera '{camera}' is named twice")
        cameras[arm] = camera
    return cameras


class WristFrames:
    def __init__(self, cameras_by_arm: Mapping[str, str]) -> None:
        self.cameras_by_arm = dict(cameras_by_arm)
        self._latest: dict[str, ColorMessage] = {}
        self._vacant = False
        # Set and replaced at each frame of a camera, so a waiter wakes
        # once per frame and looks again.
        self._arrived: dict[str, asyncio.Event] = {camera: asyncio.Event() for camera in self.cameras_by_arm.values()}

    def camera_of(self, arm: str) -> Optional[str]:
        return self.cameras_by_arm.get(arm)

    def why_unavailable(self, arm: str) -> str:
        """Why no picture of `arm`'s wrist camera can be had now, or the
        empty string."""
        camera = self.cameras_by_arm.get(arm)
        if camera is None:
            return NO_SOURCE.format(arm=arm)
        if self._vacant:
            return VACANT
        if camera not in self._latest:
            return f"no frame from wrist camera '{camera}' yet"
        return ""

    def latest(self, arm: str) -> Optional[ColorMessage]:
        camera = self.cameras_by_arm.get(arm)
        return self._latest.get(camera) if camera is not None else None

    def latest_stamp(self, arm: str) -> float:
        """The capture time of the newest frame of `arm`'s camera, or
        -inf when there is none, so that `newer_than` it is any frame."""
        frame = self.latest(arm)
        return float(frame.header.timestamp) if frame is not None else float("-inf")

    def add(self, camera: str, message: ColorMessage) -> None:
        if camera not in self._arrived:
            return
        self._latest[camera] = message
        arrived, self._arrived[camera] = self._arrived[camera], asyncio.Event()
        arrived.set()

    async def newer_than(self, arm: str, timestamp: float, timeout_s: float) -> Optional[ColorMessage]:
        """The first frame of `arm`'s camera captured after `timestamp`, a
        stamp of an earlier frame, or none when no such frame arrives
        within `timeout_s`. A frame already held answers at once."""
        camera = self.cameras_by_arm.get(arm)
        if camera is None:
            return None
        if self._is_newer(camera, timestamp):
            return self._latest[camera]
        try:
            return await asyncio.wait_for(self._wait_newer(camera, timestamp), timeout_s)
        except TimeoutError:
            return None

    async def run(self, node_runner, token, video_stream) -> None:
        """Follows the wrist_cameras slot until the node stops, through the
        node's generated video_stream module: one subscription over every
        camera bound to the slot, which follows the set as a join grows it.
        Nothing to do when the node has no such slot (`video_stream` none)."""
        if video_stream is None:
            self._vacant = True
            return
        await unless_cancelled(token, self._follow(node_runner, video_stream))

    def _is_newer(self, camera: str, timestamp: float) -> bool:
        frame = self._latest.get(camera)
        return frame is not None and float(frame.header.timestamp) > timestamp

    async def _wait_newer(self, camera: str, timestamp: float) -> ColorMessage:
        while not self._is_newer(camera, timestamp):
            await self._arrived[camera].wait()
        return self._latest[camera]

    async def _follow(self, node_runner, video_stream) -> None:
        subscription = await video_stream.subscribe(node_runner)
        async for producer, message in subscription:
            # The set follows the stack's joins, so it is read per frame.
            self.add(camera_id(producer, video_stream.bound_members(node_runner)), message)


def camera_id(producer, members) -> str:
    """The id a frame's `producer` runs under in its fragment: the id
    within its copy when the slot's `members` place it in one
    (`alpha_wrist_left` in copy `alpha` is `wrist_left`), else its instance
    id."""
    instance_id = getattr(producer, "instance_id", str(producer))
    core_node = getattr(producer, "core_node", None)
    for member in members:
        bound = member.producer
        if bound.instance_id != instance_id or (core_node is not None and bound.core_node != core_node):
            continue
        copy = getattr(member, "copy", None)
        if copy is None:
            return instance_id
        within = getattr(copy, "instance_id", None)
        if within:
            return within
        name = getattr(copy, "name", copy)
        if isinstance(name, str) and instance_id.startswith(f"{name}_"):
            return instance_id[len(name) + 1:]
        return instance_id
    return instance_id


def jpeg_of(message: ColorMessage) -> bytes:
    """`message`'s picture as JPEG bytes: its own bytes when the camera
    streams MJPEG, else the decoded picture encoded at JPEG_QUALITY. A
    Refusal names a frame that cannot be decoded."""
    if message.encoding.lower() in ("mjpeg", "jpeg", "jpg"):
        return bytes(message.frame)
    from PIL import Image

    image = decode_color(message)
    out = io.BytesIO()
    Image.fromarray(image).save(out, format="JPEG", quality=JPEG_QUALITY)
    return out.getvalue()


def decode_or_refuse(message: ColorMessage):
    """The frame decoded, or a Refusal; kept beside `jpeg_of` for callers
    that draw on the picture."""
    try:
        return decode_color(message)
    except Refusal:
        raise
