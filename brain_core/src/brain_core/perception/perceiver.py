"""The core Perceiver: frames in, world detections out. The detector a
search uses is the only swappable piece; the frames, the depth lookup, the
camera model and the duplicate rule live here, once, so two detectors
cannot drift apart on geometry.

A scan asks the detector for its whole vocabulary. An identify search
asks for the description's phrase and lets the core pick the best match
afterwards (`state.best_match`), so no detector implements selection.

The Perceiver checks what it holds, the frames and the camera model, and
refuses a search without them with `resource_unavailable`. Whether a
detector can search is the detector's own answer
(`Detector.why_unavailable`), which the brain asks first (`Brain.look`).
A search past its deadline fails with `timeout`.

The depth points of an item (`item_points`) are the readings inside its
box, in the robot frame, less the surface it stands on, and of the rest the
group that holds the reading nearest the box's centre. The surface is the
height of the readings on the border of the box: a box holds its item and
some of what is around it, and in doubt the rule keeps fewer points, since
a shape too big hides a neighbour from the motion planner.

A search awaits the detector's `detect`. A detector whose model blocks
runs its compute through `in_daemon_thread`, a daemon thread of its own
rather than the event loop's executor: the executor's threads are joined
when the process exits, and a call that hangs would hold the node past its
shutdown window, while a daemon thread dies with the process. A search is
awaited until the detector returns, whatever its deadline: the detector
keeps the deadline between its stages, and the lane the search runs in
stays busy until it has returned, so no second search starts on the same
models while the first still runs.
"""

from __future__ import annotations

import asyncio
import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

import numpy as np

from ..ports import Box, CancelToken, Deadline, Detection, Detector, ExpectedFailure, FailureKind, ItemShape, Refusal, SearchTimeout, iou
from .camera import CameraModel
from .frames import FrameStore, decode_color, decode_depth, depth_at, depth_pixel

# The budget of a search whose goal names a timeout_s of 0. A hosted model
# with reasoning answers a scan in 3 to 10 s and at times later, and a
# failure that may be asked again needs room for the second call.
DEFAULT_TIMEOUT_S = 30.0
DUPLICATE_IOU = 0.5
# The middle of a box whose depth readings give the top of its item: this
# share of the box's width and of its height, around its centre, read at
# TOP_SAMPLES points along each side. The highest readings give the top,
# but not the very highest: TOP_QUANTILE of them stand lower.
TOP_WINDOW = 0.6
TOP_SAMPLES = 24
TOP_QUANTILE = 0.9
# The points of an item: the depth readings inside its box, at most
# SHAPE_GRID along each side. The surface under the item is the
# SURFACE_QUANTILE quantile of the heights of the readings on the border of
# the box, and a reading belongs to the surface up to SURFACE_MARGIN_M above
# it. Two readings next to each other in the depth image are of one group
# when they stand at most GROUP_GAP_M apart.
SHAPE_GRID = 64
SURFACE_QUANTILE = 0.5
SURFACE_MARGIN_M = 0.005
GROUP_GAP_M = 0.015
# The box around an item's points tries each turn about the vertical in
# steps of YAW_STEP_DEG, and none of its sides is shorter than MIN_SIDE_M,
# so the planner gets a solid even for a single point.
YAW_STEP_DEG = 1
MIN_SIDE_M = 0.01


async def in_daemon_thread(function: Callable, *args):
    """Runs `function(*args)` in a daemon thread and returns what it
    returned, or raises what it raised."""
    loop = asyncio.get_running_loop()
    done: asyncio.Future = loop.create_future()

    def settle(result, error: Optional[Exception]) -> None:
        if done.done():
            return
        if error is None:
            done.set_result(result)
        else:
            done.set_exception(error)

    def run() -> None:
        try:
            outcome = (function(*args), None)
        except Exception as error:
            outcome = (None, error)
        try:
            loop.call_soon_threadsafe(settle, *outcome)
        except RuntimeError:
            # The loop is closed: the node stopped while the call ran, and
            # nothing awaits the outcome any more.
            pass

    threading.Thread(target=run, name=f"brain-{function.__name__}", daemon=True).start()
    return await done


@dataclass(frozen=True)
class Look:
    """What one look at the camera gave: the detections, each placed in the
    robot frame with its region in the picture, and the picture itself:
    its size in pixels, its capture time and its capture-pair counter, so a
    region can be read against the same frame. The detector's other boxes
    are kept for the log: each duplicate of a more confident box, and each
    box with no depth reading under its centre."""

    detections: list[Detection]
    image_width: int
    image_height: int
    frame_timestamp: float
    frame_id: int
    duplicates: list[Box]
    unplaced: list[Box]

    def describe(self, search_started_ns: int) -> str:
        """The look in a few words for the log: the frame, when the camera
        took it against the start of the search, and every box of the
        detector with its confidence, the placed ones first, each with the
        shape fitted around its depth points when it has one, then the
        duplicates and the boxes with no depth, each marked so."""
        age_s = search_started_ns / 1e9 - self.frame_timestamp
        if age_s >= 0.0:
            taken = f"{age_s:.2f} s before the search"
        else:
            taken = f"{-age_s:.2f} s after the search began"
        boxes = [f"{detection.label} {detection.confidence:.2f}{_shape_words(detection.shape)}" for detection in self.detections]
        boxes += [f"{box.label} {box.confidence:.2f} (duplicate)" for box in self.duplicates]
        boxes += [f"{box.label} {box.confidence:.2f} (no depth)" for box in self.unplaced]
        if not boxes:
            return f"frame {self.frame_id} taken {taken}, no box"
        count = "1 box" if len(boxes) == 1 else f"{len(boxes)} boxes"
        return f"frame {self.frame_id} taken {taken}, {count}: {', '.join(boxes)}"


def _shape_words(shape: Optional[ItemShape]) -> str:
    """An item's shape for the log: its sides in centimetres, its centre and
    its turn; nothing for an item with none."""
    if shape is None:
        return ""
    length, width, height = (100 * side for side in shape.sides)
    x, y, z = shape.centre
    return f" (box {length:.1f} x {width:.1f} x {height:.1f} cm at [{x:.3f}, {y:.3f}, {z:.3f}], {math.degrees(shape.yaw_rad):.0f} deg)"


@dataclass
class SourceReason:
    """Why a source of the camera model has not given what the brain reads,
    or the empty string once it has, and whether the reason ends on its own
    while the source starts: an answer that has not come yet, or the error
    reply a source gives before it has heard from what it reads. A vacant
    slot, an answer the brain cannot read and any other error reply hold
    until the source changes."""

    reason: str
    starting: bool = True

    def update(self, reason: str, *, starting: bool) -> None:
        self.reason = reason
        self.starting = starting


class Perceiver:
    def __init__(self, frames: FrameStore, camera: CameraModel) -> None:
        self.frames = frames
        self.camera = camera
        # Why the camera cannot place a pixel yet: its intrinsics, until
        # the camera has answered, and its pose, until the robot has.
        self.intrinsics = SourceReason("the camera's intrinsics have not been received yet")
        self.pose = SourceReason("the camera's pose has not been received from the robot yet")

    def set_camera(self, camera: CameraModel) -> None:
        self.camera = camera
        if camera.intrinsics is not None:
            self.intrinsics.update("", starting=False)
        if camera.placed:
            self.pose.update("", starting=False)

    def why_unavailable(self) -> str:
        """Why the frames or the camera model cannot serve a search yet, or
        the empty string."""
        reason = next((source.reason for source in self._sources() if source.reason), "")
        return f"no perception source: {reason}" if reason else ""

    def starting(self) -> str:
        """Why the frames or the camera model cannot serve a search yet
        while their sources start, or the empty string: once they serve,
        and when a source does not become able to serve one (a vacant slot,
        frames that are not aligned, a camera the robot does not carry, an
        error reply that holds), whatever the other sources say. A camera
        that never sends what the brain reads, such as one with no depth
        stream, reads as starting: nothing tells it from one that has not
        sent yet."""
        reasons = [source for source in self._sources() if source.reason]
        if not reasons or not all(source.starting for source in reasons):
            return ""
        return f"no perception source: {reasons[0].reason}"

    def _sources(self) -> list[SourceReason]:
        """The frames, the intrinsics and the pose, each with why it cannot
        serve a search yet, in the order a search reports them."""
        frames = self.frames.why_unavailable()
        intrinsics = self.intrinsics if self.camera.intrinsics is None else SourceReason("", starting=False)
        pose = self.pose if not self.camera.placed else SourceReason("", starting=False)
        return [SourceReason(frames, starting=frames == self.frames.starting()), intrinsics, pose]

    def refuse_unless_ready(self) -> None:
        """Raises the Refusal, `resource_unavailable`, of a search the
        frames or the camera model cannot serve yet."""
        reason = self.why_unavailable()
        if reason:
            raise Refusal(FailureKind.RESOURCE_UNAVAILABLE, reason)

    async def scan(self, detector: Detector, phrases: Sequence[str], cancel: CancelToken, timeout_s: float) -> Look:
        """Looks once at the latest frame through `detector` for `phrases`,
        or for everything the detector knows when `phrases` is empty, within
        `timeout_s`, the default budget when that is zero."""
        self.refuse_unless_ready()
        frame = self.frames.latest()
        image = decode_color(frame.color)
        depth_m = decode_depth(frame.depth, frame.depth_unit)
        deadline = Deadline.after(timeout_s if timeout_s > 0.0 else DEFAULT_TIMEOUT_S)
        try:
            boxes = await detector.detect(image, list(phrases), deadline)
        except SearchTimeout as timeout:
            raise ExpectedFailure(FailureKind.TIMEOUT, str(timeout)) from None
        cancel.check()
        kept, duplicates = merge_duplicates(boxes)
        detections: list[Detection] = []
        unplaced: list[Box] = []
        for box in kept:
            depth = depth_at(depth_m, box, frame.color.width, frame.color.height)
            if depth is None:
                unplaced.append(box)
                continue
            u, v = box.centre
            width, height = frame.color.width, frame.color.height
            position = self.camera.deproject(u, v, depth, width, height)
            # The box's left and right edges at the depth of its centre.
            width_m = math.dist(self.camera.deproject(box.x0, v, depth, width, height), self.camera.deproject(box.x1, v, depth, width, height))
            found = item_points(depth_m, box, width, height, self.camera)
            detections.append(
                Detection(
                    label=box.label,
                    position=position,
                    confidence=box.confidence,
                    region=box.xyxy,
                    width_m=width_m,
                    top_z=top_height(depth_m, box, width, height, self.camera),
                    shape=item_box(found) if found is not None else None,
                )
            )
        return Look(
            detections=detections,
            image_width=int(frame.color.width),
            image_height=int(frame.color.height),
            frame_timestamp=float(frame.color.header.timestamp),
            frame_id=int(frame.color.header.frame_id),
            duplicates=duplicates,
            unplaced=unplaced,
        )


def top_height(depth_m: np.ndarray, box: Box, color_width: int, color_height: int, camera: CameraModel) -> Optional[float]:
    """The height of the top of the item in `box`, in the robot frame: the
    TOP_QUANTILE quantile of the heights that the depth readings of the
    middle of the box give (TOP_WINDOW, TOP_SAMPLES), each the reading
    under its colour pixel (`depth_pixel`), all of them deprojected in one
    pass. None when that middle holds no reading. Under the centre of the
    box, a camera that looks into an open item, a mug or a bowl, reads its
    inside bottom; the walls and the rim around it give its top."""
    depth_height, depth_width = depth_m.shape
    u, v = box.centre
    half_width = (box.x1 - box.x0) * TOP_WINDOW / 2.0
    half_height = (box.y1 - box.y0) * TOP_WINDOW / 2.0
    rows, columns = np.meshgrid(
        np.linspace(v - half_height, v + half_height, TOP_SAMPLES),
        np.linspace(u - half_width, u + half_width, TOP_SAMPLES),
        indexing="ij",
    )
    columns, rows = columns.ravel(), rows.ravel()
    x, y = depth_pixel(depth_m, columns, rows, color_width, color_height)
    inside = (0 <= x) & (x < depth_width) & (0 <= y) & (y < depth_height)
    columns, rows, x, y = columns[inside], rows[inside], x[inside], y[inside]
    depths = depth_m[y, x]
    read = depths > 0.0
    if not read.any():
        return None
    points = camera.deproject_points(columns[read], rows[read], depths[read], color_width, color_height)
    return float(np.quantile(points[:, 2], TOP_QUANTILE))


@dataclass(frozen=True)
class ItemPoints:
    """The depth points of one item in the robot frame, one row of x, y, z
    each, and the height of the surface it stands on."""

    points: np.ndarray
    surface_z: float


def item_points(depth_m: np.ndarray, box: Box, color_width: int, color_height: int, camera: CameraModel) -> Optional[ItemPoints]:
    """The depth points of the item in `box`, a box of a colour picture of
    `color_width` x `color_height`:

    1. the readings of `depth_m` inside the box, at most SHAPE_GRID along
       each side, in the robot frame;
    2. less the points of the surface the item stands on: the surface is the
       SURFACE_QUANTILE quantile of the heights of the readings on the
       border of the box, and a point up to SURFACE_MARGIN_M above it is of
       the surface;
    3. of the rest, the group that holds the point nearest the centre of the
       box: points next to each other in the depth image at most
       GROUP_GAP_M apart are of one group, so a neighbour inside the box
       that no such chain joins is not part of the item.

    None when the box holds no reading on its border or none above the
    surface."""
    height, width = depth_m.shape
    x0, y0 = (int(value) for value in depth_pixel(depth_m, box.x0, box.y0, color_width, color_height))
    x1, y1 = (int(value) for value in depth_pixel(depth_m, box.x1, box.y1, color_width, color_height))
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(width, x1), min(height, y1)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    stride = max(1, math.ceil(max(x1 - x0, y1 - y0) / SHAPE_GRID))
    rows, columns = np.arange(y0, y1, stride), np.arange(x0, x1, stride)
    depths = depth_m[np.ix_(rows, columns)]
    read = depths > 0.0
    grid_rows, grid_columns = np.meshgrid(rows, columns, indexing="ij")
    points = np.full((*depths.shape, 3), np.nan)
    if read.any():
        # The colour pixel under the centre of each reading, pixel centres
        # scaled as Intrinsics.scaled_to scales them; depth_pixel maps it back
        # to the reading.
        us = (grid_columns[read] + 0.5) * color_width / width - 0.5
        vs = (grid_rows[read] + 0.5) * color_height / height - 0.5
        points[read] = camera.deproject_points(us, vs, depths[read], color_width, color_height)
    border = np.zeros(depths.shape, dtype=bool)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    if not (border & read).any():
        return None
    surface_z = float(np.quantile(points[border & read][:, 2], SURFACE_QUANTILE))
    above = read & (np.nan_to_num(points[..., 2], nan=-np.inf) > surface_z + SURFACE_MARGIN_M)
    if not above.any():
        return None
    candidates = np.argwhere(above)
    centre = (np.array(depths.shape) - 1) / 2.0
    seed = tuple(candidates[np.argmin(((candidates - centre) ** 2).sum(axis=1))])
    group = _group_of(points, above, seed)
    return ItemPoints(points[group], surface_z)


def item_box(found: ItemPoints) -> ItemShape:
    """The box around an item's points: upright, turned about the vertical
    to the angle, in steps of YAW_STEP_DEG from 0 up to a quarter turn, that
    gives the smallest rectangle around the points seen from above; its
    bottom at the surface under the item, its top at the highest point. No
    side is shorter than MIN_SIDE_M."""
    xy = found.points[:, :2]
    angles = np.radians(np.arange(0, 90, YAW_STEP_DEG))
    cosines, sines = np.cos(angles), np.sin(angles)
    along = xy[:, :1] * cosines + xy[:, 1:] * sines
    across = -xy[:, :1] * sines + xy[:, 1:] * cosines
    lengths = along.max(axis=0) - along.min(axis=0)
    widths = across.max(axis=0) - across.min(axis=0)
    best = int(np.argmin(lengths * widths))
    yaw = float(angles[best])
    middle_along = (along[:, best].max() + along[:, best].min()) / 2.0
    middle_across = (across[:, best].max() + across[:, best].min()) / 2.0
    top = float(found.points[:, 2].max())
    bottom = found.surface_z
    return ItemShape(
        centre=(
            float(middle_along * math.cos(yaw) - middle_across * math.sin(yaw)),
            float(middle_along * math.sin(yaw) + middle_across * math.cos(yaw)),
            (top + bottom) / 2.0,
        ),
        sides=(max(MIN_SIDE_M, float(lengths[best])), max(MIN_SIDE_M, float(widths[best])), max(MIN_SIDE_M, top - bottom)),
        yaw_rad=yaw,
    )


def _group_of(points: np.ndarray, kept: np.ndarray, seed: tuple[int, int]) -> np.ndarray:
    """The cells of the grid `kept` joined to `seed` by chains of cells next
    to each other (above, below, left, right) whose points stand at most
    GROUP_GAP_M apart."""
    down = kept[1:, :] & kept[:-1, :] & (np.linalg.norm(points[1:, :] - points[:-1, :], axis=-1) <= GROUP_GAP_M)
    right = kept[:, 1:] & kept[:, :-1] & (np.linalg.norm(points[:, 1:] - points[:, :-1], axis=-1) <= GROUP_GAP_M)
    group = np.zeros(kept.shape, dtype=bool)
    group[seed] = True
    queue = deque([seed])
    rows, columns = kept.shape
    while queue:
        row, column = queue.popleft()
        steps = (
            (row + 1, column, row < rows - 1 and down[row, column]),
            (row - 1, column, row > 0 and down[row - 1, column]),
            (row, column + 1, column < columns - 1 and right[row, column]),
            (row, column - 1, column > 0 and right[row, column - 1]),
        )
        for next_row, next_column, joined in steps:
            if joined and not group[next_row, next_column]:
                group[next_row, next_column] = True
                queue.append((next_row, next_column))
    return group


def merge_duplicates(boxes: Sequence[Box]) -> tuple[list[Box], list[Box]]:
    """Two boxes of one label that overlap by more than `DUPLICATE_IOU`
    are one item: the more confident box stays. The boxes that stay, then
    the duplicates, each list from the most confident box down."""
    kept: list[Box] = []
    duplicates: list[Box] = []
    for box in sorted(boxes, key=lambda b: b.confidence, reverse=True):
        if any(other.label == box.label and iou(other.xyxy, box.xyxy) > DUPLICATE_IOU for other in kept):
            duplicates.append(box)
            continue
        kept.append(box)
    return kept, duplicates
