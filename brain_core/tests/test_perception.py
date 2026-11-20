"""The camera model, the frame store and the core Perceiver: pixels to
world positions, frames paired by capture, the camera's answers asked
until they come, duplicate boxes, the ways a scan is refused, and the
words a look leaves in the log."""

import asyncio
import logging
import math
import threading
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import (
    CAMERA_16X12,
    CAMERA_FRAME,
    LOOKUP_TIMEOUT_S,
    NEVER,
    ORIGIN,
    FakeDetector,
    FakeToken,
    FixedPlacer,
    depth_frame,
    lookup_answer,
    records_of,
    rgb_frame,
    sensors_described,
)
from fake_peppygen import FakeConsumedService, LookupTransformRequest, VacantSlot, consumed_service
from control_core_py.geometry import Placement, PoseTarget, Stamped, product, rotate

from brain_core.perception.camera import (
    INVERSE_PLUMB_BOB,
    NONE,
    PLUMB_BOB,
    CameraModel,
    Intrinsics,
    pixel_to_ray,
    undistort,
)
from brain_core.perception import frames as frames_module
from brain_core.perception import geometry
from brain_core.perception.frames import FRAME_BUFFER, FrameStore, decode_color, decode_depth, depth_at, depth_pixel
from brain_core.perception.perceiver import (
    DEFAULT_TIMEOUT_S,
    TOP_QUANTILE,
    TOP_SAMPLES,
    TOP_WINDOW,
    Look,
    Perceiver,
    in_daemon_thread,
    merge_duplicates,
    top_height,
)
from brain_core.ports import Box, CancelToken, Coverage, Detection, ExpectedFailure, FailureKind, Refusal, SearchTimeout

# At the robot frame's origin, looking along its +Z: the identity pose.
AT_ORIGIN = CameraModel().with_pose(ORIGIN)
IDENTITY = AT_ORIGIN.with_intrinsics(CAMERA_16X12)


def close(a, b, tolerance=1e-6):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


def perceiver_of(frames: FrameStore, camera: CameraModel) -> Perceiver:
    """A perceiver of the camera "chest" over `frames`, holding the
    intrinsics of `camera`, whose placer stands the camera at the pose of
    `camera` at every time, or has not heard from the robot when `camera`
    has no pose."""
    return Perceiver(frames, CameraModel(intrinsics=camera.intrinsics), FixedPlacer(camera.pose))


def test_a_camera_model_places_a_pixel_once_it_holds_its_intrinsics_and_a_pose():
    placed = CameraModel().with_pose(Placement((1, 2, 3), (0, 0, 0, 1)))
    with pytest.raises(ValueError, match="intrinsics are not known yet"):
        placed.deproject(1.0, 1.0, 1.0, 16, 12)
    unplaced = CameraModel().with_intrinsics(CAMERA_16X12)
    with pytest.raises(ValueError, match="pose is not known yet"):
        unplaced.deproject(1.0, 1.0, 1.0, 16, 12)
    with pytest.raises(ValueError, match="pose is not known yet"):
        unplaced.forward()
    assert close(placed.with_intrinsics(CAMERA_16X12).deproject(8.0, 6.0, 1.0, 16, 12), (1.0, 2.0, 4.0))


def test_intrinsics_are_checked_and_scale_with_the_image():
    with pytest.raises(ValueError, match="positive finite focal"):
        Intrinsics(16, 12, 0.0, 6.0, 8.0, 6.0)
    with pytest.raises(ValueError, match="unknown distortion model"):
        Intrinsics(16, 12, 6.0, 6.0, 8.0, 6.0, "fisheye")
    # The ZED's colour intrinsics at 1280x720, used on a 640x360 stream:
    # focal lengths halve, the principal point follows the pixel centres.
    k = Intrinsics(1280, 720, 700.0, 700.0, 639.5, 359.5, PLUMB_BOB, (0.1, 0.0, 0.0, 0.0, 0.0))
    half = k.scaled_to(640, 360)
    assert (half.fx, half.fy) == (350.0, 350.0)
    assert close((half.cx, half.cy), (319.5, 179.5))
    assert half.distortion_model == PLUMB_BOB and half.distortion == (0.1, 0.0, 0.0, 0.0, 0.0)
    assert k.scaled_to(1280, 720) is k


def test_an_identity_camera_deprojects_along_plus_z_with_x_right_and_y_down():
    # The optical frame camera_geometry and transforms share: +X to the
    # right of the image, +Y down it, +Z along the view.
    width, height = 16, 12
    centre = IDENTITY.deproject(8.0, 6.0, 1.0, width, height)
    assert close(centre, (0.0, 0.0, 1.0))
    assert close(IDENTITY.forward(), (0.0, 0.0, 1.0))
    # 90 degree vertical field of view: the top edge is one focal length up.
    focal = IDENTITY.intrinsics.fy
    assert close((focal,), (6.0,))
    right = IDENTITY.deproject(8.0 + focal, 6.0, 2.0, width, height)
    assert close(right, (2.0, 0.0, 2.0))
    down = IDENTITY.deproject(8.0, 6.0 + focal, 2.0, width, height)
    assert close(down, (0.0, 2.0, 2.0))


# A D455-like colour lens, the magnitudes a real unit reports.
LENS = (-0.055, 0.066, -0.0007, 0.0005, -0.021)


def test_a_pixel_becomes_a_ray_through_the_intrinsics_with_no_lens():
    # Under "none" the ray is the pixel through fx, fy, cx, cy and nothing
    # else, so the principal point looks straight ahead and one focal
    # length to the right is 45 degrees.
    assert close(pixel_to_ray(639.5, 359.5, 738.1, 738.1, 639.5, 359.5, NONE, ()), (0.0, 0.0))
    assert close(pixel_to_ray(639.5 + 738.1, 359.5, 738.1, 738.1, 639.5, 359.5, NONE, ()), (1.0, 0.0))
    # deproject is the same path.
    assert close(IDENTITY.deproject(8.0 + 6.0, 6.0, 2.0, 16, 12), (2.0, 0.0, 2.0))


def test_plumb_bob_round_trips_through_distort_and_undistort():
    # The lens distorts an ideal point by the forward polynomial, written
    # out here; undistorting by iteration brings it back, out to the image
    # corners.
    k1, k2, p1, p2, k3 = LENS
    for x, y in [(0.0, 0.0), (0.3, -0.2), (-0.6, 0.35), (0.8, 0.45)]:
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        distorted = (
            x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
        )
        assert not close(distorted, (x, y), 1e-4) or (x, y) == (0.0, 0.0)
        assert close(undistort(PLUMB_BOB, LENS, *distorted), (x, y), 1e-9)


def test_the_inverse_model_applies_the_polynomial_once_from_the_distorted_point():
    # inverse_plumb_bob is one application of the polynomial to the
    # distorted point, nothing iterated: the result is the closed form.
    xd, yd = 0.3, -0.2
    k1, k2, p1, p2, k3 = LENS
    r2 = xd * xd + yd * yd
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    expected = (
        xd * radial + 2.0 * p1 * xd * yd + p2 * (r2 + 2.0 * xd * xd),
        yd * radial + p1 * (r2 + 2.0 * yd * yd) + 2.0 * p2 * xd * yd,
    )
    assert close(undistort(INVERSE_PLUMB_BOB, LENS, xd, yd), expected, 1e-12)


def test_inverse_and_forward_models_agree_for_small_coefficients():
    # To first order the inverse of the polynomial with coefficients c is
    # the polynomial with -c, so for a small lens the direct inverse model
    # with -c and the iterated forward model with c undistort a pixel to
    # the same ray, within the second-order term.
    for scale in (0.01, 0.003):
        small = tuple(c * scale for c in (-0.5, 0.6, -0.07, 0.05, -0.2))
        negated = tuple(-c for c in small)
        for xd, yd in [(0.3, -0.2), (-0.6, 0.35), (0.8, 0.45)]:
            forward = undistort(PLUMB_BOB, small, xd, yd)
            direct = undistort(INVERSE_PLUMB_BOB, negated, xd, yd)
            tolerance = 20.0 * scale * scale
            assert close(forward, direct, tolerance), (scale, xd, yd, forward, direct)
        # The agreement is second order: a lens ten times smaller agrees a
        # hundred times better.
    gaps = []
    for scale in (0.01, 0.001):
        small = tuple(c * scale for c in (-0.5, 0.6, -0.07, 0.05, -0.2))
        negated = tuple(-c for c in small)
        forward = undistort(PLUMB_BOB, small, 0.8, 0.45)
        direct = undistort(INVERSE_PLUMB_BOB, negated, 0.8, 0.45)
        gaps.append(math.hypot(forward[0] - direct[0], forward[1] - direct[1]))
    assert gaps[0] / gaps[1] > 50.0


def test_any_other_distortion_model_is_refused_by_name():
    with pytest.raises(ValueError, match="kannala_brandt"):
        pixel_to_ray(10.0, 10.0, 500.0, 500.0, 320.0, 240.0, "kannala_brandt", (0.1, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="modified_plumb_bob"):
        undistort("modified_plumb_bob", LENS, 0.1, 0.1)
    # The right name with the wrong number of coefficients is refused too.
    with pytest.raises(ValueError, match="k1, k2, p1, p2, k3"):
        undistort(PLUMB_BOB, (0.1, 0.0), 0.1, 0.1)
    with pytest.raises(ValueError, match="no coefficients"):
        undistort(NONE, (0.1,), 0.1, 0.1)
    with pytest.raises(ValueError, match="finite"):
        undistort(INVERSE_PLUMB_BOB, (math.nan, 0.0, 0.0, 0.0, 0.0), 0.1, 0.1)


# The OpenArm v2 chest camera as the robot's tree places it: the design's mount
# (looking along its own -Z with +Y up) turned half a turn about X into the
# optical frame.
CHEST_POSITION = (0.0792, 0.0315, 0.7941)
CHEST_ORIENTATION = product((0.1710647, -0.1710647, -0.6861027, 0.6861027), (1.0, 0.0, 0.0, 0.0))


def test_the_chest_pose_of_the_robots_tree_looks_ahead_and_down():
    # The OpenArm v2 chest camera faces the robot's +X, 62 degrees below the
    # horizon, and the image's +Y (down the picture) leans forward and down.
    # Its colour intrinsics: a 52 degree vertical field of view over 720
    # rows is a focal length of 360 / tan(26 degrees), and no lens.
    chest = CameraModel().with_pose(Placement(CHEST_POSITION, CHEST_ORIENTATION)).with_intrinsics(Intrinsics(1280, 720, 738.1, 738.1, 640.0, 360.0))
    forward = chest.forward()
    assert close(forward, (math.cos(math.radians(62)), 0.0, -math.sin(math.radians(62))), 1e-3)
    assert close(rotate(chest.pose.orientation, (0.0, 1.0, 0.0)), (-math.sin(math.radians(62)), 0.0, -math.cos(math.radians(62))), 1e-3)
    # One meter along the axis from the chest lands on the table in front.
    point = chest.deproject(640.0, 360.0, 1.0, 1280, 720)
    assert close(point, (0.0792 + forward[0], 0.0315, 0.7941 + forward[2]), 1e-3)


def test_frames_decode_and_depth_reads_the_median_under_a_box():
    colour = decode_color(rgb_frame(16, 12))
    assert colour.shape == (12, 16, 3)
    depth = decode_depth(depth_frame(0.75, 8, 6), 0.001)
    assert depth.shape == (6, 8) and abs(float(depth[0, 0]) - 0.75) < 1e-6
    # The box is in colour pixels; the depth frame is half the size.
    box = Box("cup", 0.9, 6.0, 4.0, 10.0, 8.0)
    assert abs(depth_at(depth, box, 16, 12) - 0.75) < 1e-6
    depth[:, :] = 0.0
    assert depth_at(depth, box, 16, 12) is None
    # A frame that does not decode fails the search that read it.
    message = rgb_frame(4, 4)
    message.encoding = "yuyv"
    with pytest.raises(ExpectedFailure, match="unsupported colour encoding") as raised:
        decode_color(message)
    assert raised.value.kind is FailureKind.ATTEMPT_FAILED
    with pytest.raises(ExpectedFailure, match="depth frame holds 3 bytes") as raised:
        decode_depth(replace_frame(depth_frame(1.0, 2, 2), b"abc"), 0.001)
    assert raised.value.kind is FailureKind.ATTEMPT_FAILED


def replace_frame(message, data: bytes):
    message.data = data
    return message


def store_with(colors=(), depths=(), depth_unit=0.001) -> FrameStore:
    store = FrameStore()
    for message in colors:
        store.add_color(message)
    for message in depths:
        store.add_depth(message)
    store.depth_unit = depth_unit
    return store


def test_the_frame_store_pairs_colour_and_depth_by_capture():
    # Colour runs a frame ahead of depth: the newest pair is capture 2.
    store = store_with([rgb_frame(capture_index=1), rgb_frame(capture_index=2), rgb_frame(capture_index=3)], [depth_frame(1.0, capture_index=1), depth_frame(2.0, capture_index=2)])
    frame = store.latest()
    assert frame.color.header.capture_index == 2 and frame.depth.header.capture_index == 2
    assert store.why_unavailable() == "" and store.frames_seen == 3
    # Only the last FRAME_BUFFER frames of a stream are kept.
    for frame_id in range(10, 10 + FRAME_BUFFER + 2):
        store.add_color(rgb_frame(capture_index=frame_id))
    assert [m.header.capture_index for m in store.colors] == list(range(12, 12 + FRAME_BUFFER))


def test_the_frame_store_says_what_is_missing():
    assert FrameStore().why_unavailable() == "no camera frame received"
    assert store_with([rgb_frame()]).why_unavailable() == "no depth frame received"
    assert store_with([rgb_frame()], [depth_frame(1.0)], depth_unit=None).why_unavailable() == "the camera has not answered depth_stream_info yet"
    disjoint = store_with([rgb_frame(capture_index=1), rgb_frame(capture_index=2)], [depth_frame(1.0, capture_index=3)])
    assert disjoint.why_unavailable() == "no colour and depth frame of one capture received (latest colour capture_index 2, depth 3)"
    with pytest.raises(Refusal, match="no colour and depth frame of one capture") as raised:
        disjoint.latest()
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE


def test_the_frame_store_starts_until_a_pair_comes_and_not_for_a_vacant_slot_or_frames_not_aligned():
    # Each frame, the depth unit and a pair of one capture come on their own.
    assert FrameStore().starting() == "no camera frame received"
    assert store_with([rgb_frame()]).starting() == "no depth frame received"
    assert store_with([rgb_frame()], [depth_frame(1.0)], depth_unit=None).starting() == "the camera has not answered depth_stream_info yet"
    disjoint = store_with([rgb_frame(capture_index=1)], [depth_frame(1.0, capture_index=3)])
    assert disjoint.starting() == disjoint.why_unavailable() != ""
    assert store_with([rgb_frame()], [depth_frame(1.0)]).starting() == ""
    # The next frames of a camera whose streams are not aligned are not
    # aligned either, and a vacant slot sends none.
    unaligned = store_with([rgb_frame(align_mode="none")], [depth_frame(1.0, align_mode="none")])
    assert unaligned.why_unavailable() != "" and unaligned.starting() == ""
    vacant = FrameStore()
    vacant.vacant = True
    assert vacant.why_unavailable() == "no camera frame received" and vacant.starting() == ""


def test_frames_that_are_not_aligned_to_each_other_are_refused():
    unaligned = store_with([rgb_frame(align_mode="none")], [depth_frame(1.0, align_mode="none")])
    assert unaligned.why_unavailable() == "the camera's depth is not aligned to its colour (align_mode 'none'); the brain reads depth at colour pixels"
    mixed = store_with([rgb_frame(align_mode="depth_to_color")], [depth_frame(1.0, align_mode="none")])
    assert mixed.why_unavailable() == "the colour and depth frames name different alignments ('depth_to_color' and 'none')"
    # The depth is read at the colour pixels, so both name the camera's frame.
    elsewhere = store_with([rgb_frame()], [depth_frame(1.0, frame="chest_depth_optical_frame")])
    assert elsewhere.why_unavailable() == (
        "the colour and depth frames name different frames ('chest_color_optical_frame' and 'chest_depth_optical_frame')"
    )
    for mode in ("depth_to_color", "color_to_depth"):
        assert store_with([rgb_frame(align_mode=mode)], [depth_frame(1.0, align_mode=mode)]).why_unavailable() == ""
    unknown = store_with([rgb_frame(align_mode="rectified")], [depth_frame(1.0, align_mode="rectified")])
    assert "align_mode 'rectified'" in unknown.why_unavailable()


class Feed:
    """A subscription that delivers its messages, then waits until it is
    cancelled; `delivered` says the messages are all out."""

    def __init__(self, messages) -> None:
        self.messages = list(messages)
        self.delivered = asyncio.Event()

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.messages:
            return ("camera", self.messages.pop(0))
        self.delivered.set()
        await asyncio.Event().wait()


async def test_the_frame_store_asks_the_depth_unit_until_answered_then_follows_both_streams(monkeypatch):
    answers = [TimeoutError("no answer yet"), SimpleNamespace(data=SimpleNamespace(depth_unit=0.001))]

    async def poll(node_runner, producer, timeout):
        answer = answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    colours = Feed([rgb_frame(capture_index=1), rgb_frame(capture_index=2)])
    depths = Feed([depth_frame(1.0, capture_index=2)])

    async def subscribe_colour(node_runner):
        return colours

    async def subscribe_depth(node_runner):
        return depths

    monkeypatch.setattr(frames_module, "DEPTH_INFO_RETRY_S", 0.0)
    video_stream = SimpleNamespace(bound_producer=lambda node_runner: "camera", subscribe=subscribe_colour)
    depth_stream = SimpleNamespace(subscribe=subscribe_depth)
    depth_stream_info = SimpleNamespace(poll=poll)
    store = FrameStore()
    token = FakeToken()
    running = asyncio.create_task(store.run(None, token, video_stream, depth_stream, depth_stream_info))
    await asyncio.wait_for(colours.delivered.wait(), 1.0)
    await asyncio.wait_for(depths.delivered.wait(), 1.0)
    assert answers == [] and store.depth_unit == 0.001 and store.frames_seen == 2
    frame = store.latest()
    assert frame.color.header.capture_index == 2 and frame.depth.header.capture_index == 2
    token.cancel()
    await asyncio.wait_for(running, 1.0)


async def test_a_vacant_camera_slot_leaves_the_store_empty():
    store = FrameStore()
    await asyncio.wait_for(store.run(None, FakeToken(), VacantSlot, VacantSlot, VacantSlot), 1.0)
    assert store.why_unavailable() == "no camera frame received"


# 1 m over the table top, looking straight down: the optical +Z is the
# robot's -Z, so a reading of depth d stands 1 - d above the top.
LOOKING_DOWN = CameraModel().with_pose(Placement((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0))).with_intrinsics(CAMERA_16X12)


def open_item_depths(width: int = 64, height: int = 48, *, rim_m: float = 0.08, bottom_m: float = 0.005):
    """A depth frame of LOOKING_DOWN over a table top with an open item, a
    mug, at the middle of the picture: the table top 1 m deep, the mug's rim
    and walls `rim_m` over it in a ring 4 to 14 px from the middle, and its
    inside bottom `bottom_m` over it within 4 px of the middle."""
    rows, columns = np.mgrid[0:height, 0:width]
    from_middle = np.hypot(columns - width / 2, rows - height / 2)
    depths = np.full((height, width), 1.0)
    depths[from_middle <= 14] = 1.0 - rim_m
    depths[from_middle <= 4] = 1.0 - bottom_m
    return depths


def test_the_top_of_an_open_item_is_its_rim_and_its_seen_point_its_inside_bottom():
    depths = open_item_depths()
    mug = Box("mug", 0.9, 16, 12, 48, 36)
    assert top_height(depths, mug, 64, 48, LOOKING_DOWN) == pytest.approx(0.08)
    assert 1.0 - depth_at(depths, mug, 64, 48) == pytest.approx(0.005)
    # A box whose middle holds no reading has no top.
    assert top_height(np.zeros((48, 64)), mug, 64, 48, LOOKING_DOWN) is None
    # The colour picture can be larger than the depth frame.
    assert top_height(depths, Box("mug", 0.9, 32, 24, 96, 72), 128, 96, LOOKING_DOWN) == pytest.approx(0.08)


# A camera with a lens, turned and placed off the origin, over a picture
# larger than the one its intrinsics were given for.
LENS_CAMERA = (
    CameraModel()
    .with_pose(Placement((0.1, -0.2, 0.9), (0.9, 0.1, -0.05, 0.4)))
    .with_intrinsics(Intrinsics(64, 48, 40.0, 41.0, 31.5, 23.5, PLUMB_BOB, LENS))
)


def test_a_grid_of_pixels_deprojects_as_each_pixel_does():
    us = np.array([0.0, 17.25, 63.0, 100.5])
    vs = np.array([0.0, 30.5, 47.0, 70.25])
    depths = np.array([0.3, 0.75, 1.2, 2.0])
    points = LENS_CAMERA.deproject_points(us, vs, depths, 128, 96)
    assert points.shape == (4, 3)
    for point, u, v, depth in zip(points, us, vs, depths):
        assert close(point, LENS_CAMERA.deproject(float(u), float(v), float(depth), 128, 96), 1e-12)


def test_a_colour_pixel_falls_on_the_depth_reading_under_it():
    depths = np.zeros((48, 64))
    # The colour picture is twice the depth frame: pixel (21, 9) is reading
    # (10.5, 4.5), which rounds to the nearest even reading, (10, 4).
    assert tuple(int(i) for i in depth_pixel(depths, 21.0, 9.0, 128, 96)) == (10, 4)
    columns, rows = depth_pixel(depths, np.array([21.0, 23.0, 127.0]), np.array([9.0, 11.0, 95.0]), 128, 96)
    assert columns.tolist() == [10, 12, 64] and rows.tolist() == [4, 6, 48]


def test_the_top_of_an_item_is_the_quantile_of_the_heights_each_reading_of_the_middle_gives():
    # Depth readings that differ from pixel to pixel, through a lens: the
    # top is the quantile of the heights that each reading, deprojected on
    # its own, gives, read in the depth frame under each colour pixel.
    depths = 0.6 + 0.4 * np.random.default_rng(7).random((48, 64))
    depths[20:24, 30:34] = 0.0
    box = Box("mug", 0.9, 20.0, 14.0, 100.0, 80.0)
    u, v = box.centre
    half_width, half_height = (box.x1 - box.x0) * TOP_WINDOW / 2.0, (box.y1 - box.y0) * TOP_WINDOW / 2.0
    heights = []
    for row in np.linspace(v - half_height, v + half_height, TOP_SAMPLES):
        for column in np.linspace(u - half_width, u + half_width, TOP_SAMPLES):
            x, y = (int(round(value)) for value in (column * 64 / 128, row * 48 / 96))
            if 0 <= x < 64 and 0 <= y < 48 and depths[y, x] > 0.0:
                heights.append(LENS_CAMERA.deproject(float(column), float(row), float(depths[y, x]), 128, 96)[2])
    assert top_height(depths, box, 128, 96, LENS_CAMERA) == pytest.approx(float(np.quantile(heights, TOP_QUANTILE)), abs=1e-12)
    # A box that stands half out of the picture reads the part inside.
    edge = Box("mug", 0.9, 100.0, 60.0, 160.0, 120.0)
    assert top_height(depths, edge, 128, 96, LENS_CAMERA) is not None


async def test_a_scan_gives_each_item_its_top():
    depths = open_item_depths()
    message = SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame=CAMERA_FRAME, capture_index=3, align_mode="depth_to_color"),
        encoding="z16",
        width=64,
        height=48,
        data=np.round(depths / 0.001).astype("<u2").tobytes(),
    )
    frames = store_with([rgb_frame(64, 48, capture_index=3)], [message])
    perceiver = perceiver_of(frames, LOOKING_DOWN)
    look = await perceiver.scan(FakeDetector([Box("mug", 0.9, 16, 12, 48, 36)]), ["mug"], CancelToken(), timeout_s=0.0)
    [mug] = look.detections
    assert mug.position[2] == pytest.approx(0.005, abs=1e-3)
    assert mug.top_z == pytest.approx(0.08, abs=1e-3)


def test_duplicate_boxes_of_one_label_keep_the_most_confident():
    a = Box("cup", 0.9, 0, 0, 10, 10)
    b = Box("cup", 0.8, 1, 1, 11, 11)
    other = Box("bowl", 0.7, 1, 1, 11, 11)
    far = Box("cup", 0.6, 50, 50, 60, 60)
    kept, duplicates = merge_duplicates([b, a, other, far])
    assert kept == [a, other, far]
    assert duplicates == [b]


async def test_a_scan_turns_boxes_into_world_detections():
    frames = store_with([rgb_frame(16, 12, capture_index=7)], [depth_frame(2.0, 16, 12, capture_index=7)])
    duplicate = Box("cup", 0.85, 6.5, 4.5, 10.5, 8.5)
    # Its centre is off the picture, where the depth frame has no reading.
    off_the_picture = Box("bowl", 0.5, 30, 30, 40, 40)
    detector = FakeDetector([Box("cup", 0.9, 6, 4, 10, 8), duplicate, off_the_picture])
    perceiver = perceiver_of(frames, IDENTITY)
    look = await perceiver.scan(detector, ["cup"], CancelToken(), timeout_s=0.0)
    assert detector.phrases == [["cup"]]
    detections = look.detections
    assert len(detections) == 1
    assert detections[0].label == "cup" and detections[0].confidence == 0.9
    assert close(detections[0].position, (0.0, 0.0, 2.0))
    # The look reports the box in the picture it used, and the picture.
    assert detections[0].region == (6, 4, 10, 8)
    # Its width: the box is 4 px wide, 2 m deep, through a 6 px focal length.
    assert detections[0].width_m == pytest.approx(4 * 2.0 / 6.0)
    assert (look.frame, look.image_width, look.image_height, look.timestamp, look.capture_index) == ("base_link", 16, 12, 1.0, 7)
    # The look places the frame the picture names, at its capture time.
    assert perceiver.placer.asked[-1] == (CAMERA_FRAME, 1.0)
    # The detector's other boxes stay in the look for the log.
    assert look.duplicates == [duplicate]
    assert look.unplaced == [off_the_picture]
    # The detector is handed the search's deadline: the default budget for
    # a zero timeout, the caller's otherwise.
    assert detector.deadlines[-1].budget_s == DEFAULT_TIMEOUT_S
    await perceiver.scan(detector, [], CancelToken(), timeout_s=2.5)
    assert detector.deadlines[-1].budget_s == 2.5
    assert detector.phrases[-1] == []


async def test_a_detector_past_its_deadline_fails_the_scan_with_timeout():
    class TimingOut(FakeDetector):
        async def detect(self, image, phrases, deadline):
            raise SearchTimeout(deadline.budget_s)

    perceiver = perceiver_of(store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)]), IDENTITY)
    with pytest.raises(ExpectedFailure, match="^the search did not finish within 2.5 s$") as raised:
        await perceiver.scan(TimingOut(), [], CancelToken(), timeout_s=2.5)
    assert raised.value.kind is FailureKind.TIMEOUT


def test_a_look_names_its_frame_its_age_and_every_box_for_the_log():
    look = Look(
        detections=[Detection("cup", (0.0, 0.0, 2.0), 0.9)],
        frame="base_link",
        image_width=16,
        image_height=12,
        timestamp=10.0,
        capture_index=7,
        duplicates=[Box("cup", 0.85, 6.5, 4.5, 10.5, 8.5)],
        unplaced=[Box("bowl", 0.5, 30, 30, 40, 40)],
    )
    boxes = "3 boxes: cup 0.90, cup 0.85 (duplicate), bowl 0.50 (no depth)"
    assert look.describe(10_250_000_000) == f"frame 7 taken 0.25 s before the search, {boxes}"
    # The latest frame may be newer than the start of the search.
    assert look.describe(9_900_000_000) == f"frame 7 taken 0.10 s after the search began, {boxes}"
    one = replace(look, duplicates=[], unplaced=[])
    assert one.describe(10_000_000_000) == "frame 7 taken 0.00 s before the search, 1 box: cup 0.90"
    nothing = replace(one, detections=[])
    assert nothing.describe(10_000_000_000) == "frame 7 taken 0.00 s before the search, no box"


async def test_a_blocking_call_runs_in_a_daemon_thread_of_its_own():
    def compute(value):
        return threading.current_thread(), value * 2

    thread, doubled = await in_daemon_thread(compute, 21)
    assert doubled == 42
    assert thread.daemon and thread is not threading.main_thread()
    assert thread.name == "brain-compute"

    def boom():
        raise ValueError("in the thread")

    with pytest.raises(ValueError, match="in the thread"):
        await in_daemon_thread(boom)


async def test_the_none_detector_is_never_ready_and_finds_nothing():
    from brain_core.perception.none import NoneDetector

    assert NoneDetector().why_unavailable() == "perception_model is 'none'"
    assert NoneDetector().scan_coverage() == Coverage()
    assert await NoneDetector().detect(None, [], NEVER) == []


async def test_a_scan_is_refused_without_frames_or_with_frames_that_are_not_aligned():
    perceiver = perceiver_of(FrameStore(), IDENTITY)
    detector = FakeDetector()
    with pytest.raises(Refusal, match="^no perception source: no camera frame received$") as raised:
        await perceiver.scan(detector, [], CancelToken(), 0.0)
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert perceiver.why_unavailable() == "no perception source: no camera frame received"
    perceiver.frames.add_color(rgb_frame(align_mode="none"))
    perceiver.frames.add_depth(depth_frame(1.0, align_mode="none"))
    perceiver.frames.depth_unit = 0.001
    with pytest.raises(Refusal, match="no perception source: the camera's depth is not aligned"):
        await perceiver.scan(detector, [], CancelToken(), 0.0)
    assert detector.phrases == []


async def test_a_scan_waits_for_the_cameras_intrinsics_and_names_the_slot():
    from brain_core.perception.geometry import VACANT, intrinsics_from

    frames = store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)])
    perceiver = perceiver_of(frames, AT_ORIGIN)
    detector = FakeDetector([Box("mug", 0.9, 4, 4, 8, 8)])
    assert perceiver.why_unavailable() != ""
    with pytest.raises(Refusal, match="intrinsics have not been received"):
        await perceiver.scan(detector, [], CancelToken(), 1.0)
    perceiver.intrinsics.update(VACANT, starting=False)
    with pytest.raises(Refusal, match="geometry slot is vacant"):
        await perceiver.scan(detector, [], CancelToken(), 1.0)
    answer = SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])
    perceiver.set_intrinsics(intrinsics_from(answer))
    assert perceiver.why_unavailable() == "" and perceiver.intrinsics.reason == ""
    found = await perceiver.scan(detector, [], CancelToken(), 1.0)
    assert [d.label for d in found.detections] == ["mug"]
    with pytest.raises(ValueError, match="unknown distortion model"):
        intrinsics_from(SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="fisheye", distortion=[]))


def colour_answer(fx: float = 6.0):
    return SimpleNamespace(width=16, height=12, fx=fx, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])


def depth_answer(depth_model: str = "z"):
    return SimpleNamespace(depth_model=depth_model)


def scripted_service(name: str, answers: list, before_each=None):
    """A consumed service bound to "camera" that answers each poll with the
    next of `answers`: a response's data, an exception to raise, or a pair
    (kind, message), its error reply. `before_each` runs before each
    answer."""
    service = consumed_service(name, None, producer="camera")

    async def poll(node_runner, producer, timeout):
        if before_each is not None:
            before_each()
        answer = answers.pop(0)
        if isinstance(answer, tuple):
            raise service.Failure(*answer)
        if isinstance(answer, Exception):
            raise answer
        return SimpleNamespace(data=answer)

    service.poll = poll
    return service


async def test_the_camera_is_asked_again_until_it_knows_its_geometry(monkeypatch):
    # The reasons a search is refused with, one per round, until the
    # camera answers both questions with something usable: not answered,
    # colour not known yet (the error reply a sim relay gives before its
    # simulation speaks), depth not known, a depth model the brain does not
    # read, an unusable pinhole, then the answer. An error reply about the
    # colour stream leaves the depth stream unasked in that round.
    perceiver = perceiver_of(FrameStore(), AT_ORIGIN)
    colour = [
        TimeoutError("no answer"),
        ("precondition_failed", "no camera geometry received from the simulation yet"),
        colour_answer(),
        colour_answer(),
        colour_answer(fx=0.0),
        colour_answer(),
    ]
    depth = [("rejected", "no depth stream"), depth_answer(depth_model="range"), depth_answer(), depth_answer()]
    reasons: list[str] = []

    monkeypatch.setattr(geometry, "RETRY_S", 0.0)
    get_color_intrinsics = scripted_service("get_color_intrinsics", colour, lambda: reasons.append(perceiver.intrinsics.reason))
    get_depth_intrinsics = scripted_service("get_depth_intrinsics", depth)
    await asyncio.wait_for(geometry.learn_intrinsics(None, FakeToken(), perceiver, get_color_intrinsics, get_depth_intrinsics), 5.0)
    assert reasons == [
        "the camera's intrinsics have not been received yet",
        "the camera's geometry is not answered yet (TimeoutError('no answer'))",
        "the camera does not know its colour intrinsics: no camera geometry received from the simulation yet",
        "the camera does not know its depth intrinsics: no depth stream",
        "the camera's depth samples are 'range' distances; the brain reads distances along the optical axis ('z')",
        "the camera's intrinsics are unusable: intrinsics need positive finite focal lengths, got fx 0.0 fy 6.0",
    ]
    assert colour == [] and depth == []
    assert perceiver.camera.intrinsics is not None and perceiver.intrinsics.reason == ""


async def test_a_camera_that_never_knows_its_geometry_is_asked_until_the_node_stops(monkeypatch):
    perceiver = perceiver_of(FrameStore(), AT_ORIGIN)
    token = FakeToken()
    asked = 0

    def count() -> None:
        nonlocal asked
        asked += 1
        if asked == 3:
            token.cancel()

    monkeypatch.setattr(geometry, "RETRY_S", 0.0)
    # A device without a calibration gives `rejected` every time; the
    # brain asks again all the same.
    get_color_intrinsics = scripted_service("get_color_intrinsics", [("rejected", "this camera has no calibration")] * 3, count)
    get_depth_intrinsics = scripted_service("get_depth_intrinsics", [])
    await asyncio.wait_for(geometry.learn_intrinsics(None, token, perceiver, get_color_intrinsics, get_depth_intrinsics), 5.0)
    assert asked == 3
    assert perceiver.camera.intrinsics is None
    assert perceiver.intrinsics.reason == "the camera does not know its colour intrinsics: this camera has no calibration"


async def test_a_vacant_geometry_slot_is_the_reason_at_once():
    perceiver = perceiver_of(FrameStore(), AT_ORIGIN)
    await asyncio.wait_for(geometry.learn_intrinsics(None, FakeToken(), perceiver, VacantSlot, VacantSlot), 1.0)
    assert perceiver.intrinsics.reason == geometry.VACANT
    assert not perceiver.intrinsics.starting


async def test_no_answer_and_precondition_failed_are_reasons_that_start_and_others_hold(monkeypatch):
    # One reason per round: no answer and the error reply of a camera that
    # has not heard from its simulation end on their own; a device without
    # a calibration and a depth model the brain does not read hold.
    perceiver = perceiver_of(FrameStore(), AT_ORIGIN)
    colour = [
        TimeoutError("no answer"),
        ("precondition_failed", "no camera geometry received from the simulation yet"),
        ("rejected", "this camera has no calibration"),
        colour_answer(),
        colour_answer(),
    ]
    depth = [depth_answer(depth_model="range"), depth_answer()]
    seen: list[tuple[str, bool]] = []

    def record() -> None:
        seen.append((perceiver.intrinsics.reason, perceiver.intrinsics.starting))

    monkeypatch.setattr(geometry, "RETRY_S", 0.0)
    get_color_intrinsics = scripted_service("get_color_intrinsics", colour, record)
    get_depth_intrinsics = scripted_service("get_depth_intrinsics", depth)
    await asyncio.wait_for(geometry.learn_intrinsics(None, FakeToken(), perceiver, get_color_intrinsics, get_depth_intrinsics), 5.0)
    assert [starting for _, starting in seen[1:]] == [True, True, False, False]
    assert seen[3][0] == "the camera does not know its colour intrinsics: this camera has no calibration"
    assert seen[4][0].startswith("the camera's depth samples are 'range' distances")
    assert perceiver.intrinsics.reason == ""


def test_the_perceiver_starts_while_every_reason_starts_and_not_once_one_holds():
    frames = store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)])
    perceiver = perceiver_of(frames, CameraModel())
    assert perceiver.starting() == "no perception source: the camera's intrinsics have not been received yet"
    # A vacant sensor_mounts slot holds: the perceiver does not become able
    # to serve, whatever the intrinsics say.
    perceiver.placer.reason.update(geometry.MOUNTS_VACANT, starting=False)
    assert perceiver.starting() == ""
    assert perceiver.why_unavailable() == "no perception source: the camera's intrinsics have not been received yet"
    # Frames that are not aligned hold as well.
    unaligned = perceiver_of(store_with([rgb_frame(align_mode="none")], [depth_frame(1.0, align_mode="none")]), CameraModel())
    assert unaligned.why_unavailable().startswith("no perception source: the camera's depth is not aligned")
    assert unaligned.starting() == ""
    ready = perceiver_of(frames, IDENTITY)
    assert ready.starting() == "" and ready.why_unavailable() == ""


def lookup_service(*answers, producer="robot"):
    """The robot's lookup_transform, answering each lookup with the next of
    `answers`: a response's data, or an exception to raise, such as an
    error reply of its `Failure`."""
    service = FakeConsumedService("lookup_transform", LookupTransformRequest, producer=producer)
    for answer in answers:
        service.enqueue(answer)
    return service


def placer_of(describe_sensors=VacantSlot, lookup_transform=VacantSlot) -> geometry.Placer:
    return geometry.Placer(None, describe_sensors, lookup_transform, LOOKUP_TIMEOUT_S)


async def learned_placer(lookup_transform) -> geometry.Placer:
    """A placer that has learned the robot frame from its first answer, and
    asks `lookup_transform` for its lookups."""
    placer = placer_of(scripted_service("describe_sensors", [sensors_described(("wrist_left", "chest"))]), lookup_transform)
    await asyncio.wait_for(placer.learn(FakeToken()), 5.0)
    return placer


def header(frame=CAMERA_FRAME, timestamp=12.5, capture_index=9) -> SimpleNamespace:
    """The header of a picture that names `frame`."""
    return SimpleNamespace(frame=frame, timestamp=timestamp, capture_index=capture_index)


async def test_the_robot_is_asked_again_until_it_names_its_robot_frame(monkeypatch):
    # The reasons a search is refused with, one per round, until the robot
    # gives its response and names its robot frame: not answered, an error
    # reply, an answer it cannot use, then the answer, whatever cameras it
    # names.
    frames = store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)])
    answers = [
        TimeoutError("no answer"),
        ("rejected", "no model yet"),
        sensors_described(robot_frame=""),
        sensors_described(names=("wrist_left", "wrist_right")),
    ]
    reasons: list[tuple[str, bool]] = []
    describe_sensors = scripted_service(
        "describe_sensors", answers, lambda: reasons.append((placer.reason.reason, placer.reason.starting))
    )
    placer = placer_of(describe_sensors, lookup_service())
    perceiver = Perceiver(frames, CameraModel().with_intrinsics(CAMERA_16X12), placer)
    with pytest.raises(Refusal, match="sensors have not been described yet"):
        await perceiver.scan(FakeDetector(), [], CancelToken(), 1.0)
    with pytest.raises(Refusal, match=f"^{geometry.NOT_DESCRIBED}$") as raised:
        placer.robot_frame
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE

    monkeypatch.setattr(geometry, "RETRY_S", 0.0)
    await asyncio.wait_for(placer.learn(FakeToken()), 5.0)

    assert reasons == [
        (geometry.NOT_DESCRIBED, True),
        ("the robot's sensor mounts are not answered yet (TimeoutError('no answer'))", True),
        ("the robot cannot describe its sensors: no model yet", False),
        ("the robot's sensor mounts are unusable: it names no robot frame", False),
    ]
    assert answers == []
    assert placer.reason.reason == "" and perceiver.why_unavailable() == ""
    assert placer.robot_frame == "base_link"


async def test_a_vacant_sensor_mounts_or_transform_lookup_slot_is_the_reason_at_once():
    without_mounts = placer_of(VacantSlot, lookup_service())
    await asyncio.wait_for(without_mounts.learn(FakeToken()), 1.0)
    assert (without_mounts.reason.reason, without_mounts.reason.starting) == (geometry.MOUNTS_VACANT, False)
    without_lookup = placer_of(scripted_service("describe_sensors", [sensors_described()]), VacantSlot)
    await asyncio.wait_for(without_lookup.learn(FakeToken()), 1.0)
    assert (without_lookup.reason.reason, without_lookup.reason.starting) == (geometry.LOOKUP_VACANT, False)
    with pytest.raises(Refusal, match=f"^{geometry.LOOKUP_VACANT}$"):
        without_lookup.robot_frame


async def test_a_picture_is_placed_by_a_lookup_of_the_frame_it_names_in_the_robot_frame_at_its_time():
    lookup = lookup_service(lookup_answer((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0004), child_frame="wrist_left_color_optical_frame"))
    placer = await learned_placer(lookup)

    pose = await placer.place_picture(header("wrist_left_color_optical_frame"))

    assert lookup.polled == [
        LookupTransformRequest(
            target_frame="base_link", source_frame="wrist_left_color_optical_frame", source_timestamp=12.5,
            target_timestamp=12.5, timeout_s=LOOKUP_TIMEOUT_S,
        )
    ]
    assert pose == Placement((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))


async def test_a_picture_whose_frame_the_tree_does_not_place_is_dropped_and_logged_with_the_lookups_kind(caplog):
    lookup = lookup_service()
    lookup.enqueue(lookup.Failure("unknown_frame", "no transform names head_color_optical_frame; it holds base_link"))
    lookup.enqueue(lookup.Failure("extrapolation", "the transform to wrist_link at 12.500 s: held 11.000 s to 12.000 s"))
    lookup.enqueue(TimeoutError("no answer"))
    lookup.enqueue(lookup_answer(orientation=(0.0, 0.0, 0.0, 0.0)))
    placer = await learned_placer(lookup)
    refusals = []
    with records_of(caplog, "brain_core.perception.geometry") as log:
        for frame in ("head_color_optical_frame", CAMERA_FRAME, CAMERA_FRAME, CAMERA_FRAME):
            with pytest.raises(Refusal) as raised:
                await placer.place_picture(header(frame))
            refusals.append((raised.value.kind, raised.value.message))

    unavailable = FailureKind.RESOURCE_UNAVAILABLE
    at = f"the frame '{CAMERA_FRAME}' at 12.500 s"
    assert refusals == [
        (
            unavailable,
            "picture 9 dropped: unknown_frame: the frame 'head_color_optical_frame' at 12.500 s: "
            "no transform names head_color_optical_frame; it holds base_link",
        ),
        (unavailable, f"picture 9 dropped: extrapolation: {at}: the transform to wrist_link at 12.500 s: held 11.000 s to 12.000 s"),
        (unavailable, f"{at}: the robot's transform lookup is not answered (TimeoutError('no answer'))"),
        (unavailable, f"{at}: the robot's transform lookup gave no pose: the orientation [0, 0, 0, 0] has a squared norm of 0, 0.01 or more from 1: send a unit quaternion [x, y, z, w], such as [0, 0, 0, 1]"),
    ]
    # Each dropped picture leaves its warning; a lookup that gave no answer
    # drops no picture.
    assert [record.getMessage() for record in log.records if record.levelno == logging.WARNING] == [refusals[0][1], refusals[1][1]]
    assert len(lookup.polled) == 4
    with pytest.raises(Refusal, match=f"^{geometry.NOT_DESCRIBED}$"):
        await placer_of(lookup_transform=lookup).place_picture(header())


async def test_a_target_is_placed_where_its_frame_stood_at_its_timestamp_or_the_latest_time_in_the_robot_frame_at_the_latest_time():
    # The target's frame stands 1 m ahead of the robot frame, turned a
    # quarter turn about +Z.
    turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    lookup = lookup_service(*[lookup_answer((1.0, 0.0, 0.0), turn, child_frame="odom")] * 2)
    placer = await learned_placer(lookup)

    pose = await placer.place_target(Stamped.parse("odom", 4.5, PoseTarget.parse((0.2, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))))
    point = await placer.place_target(Stamped.parse("odom", None, PoseTarget.parse((0.2, 0.0, 0.1), None)))

    assert lookup.polled == [
        LookupTransformRequest("base_link", "odom", 4.5, None, LOOKUP_TIMEOUT_S),
        LookupTransformRequest("base_link", "odom", None, None, LOOKUP_TIMEOUT_S),
    ]
    assert close(pose.position, (1.0, 0.2, 0.1), 1e-12) and close(pose.orientation, turn, 1e-12)
    assert close(point.position, (1.0, 0.2, 0.1), 1e-12) and point.orientation is None


async def test_a_target_the_tree_cannot_place_is_refused_with_the_lookups_kind():
    lookup = lookup_service()
    for kind in ("unknown_frame", "no_chain", "extrapolation", "broken_tree"):
        lookup.enqueue(lookup.Failure(kind, f"the cause of {kind}"))
    placer = await learned_placer(lookup)
    target = Stamped.parse("map", None, PoseTarget.parse((0.2, 0.0, 0.1), None))
    for kind in ("unknown_frame", "no_chain", "extrapolation", "broken_tree"):
        with pytest.raises(geometry.LookupFailure) as raised:
            await placer.place_target(target)
        assert (raised.value.kind, raised.value.message) == (FailureKind(kind), f"the target: the cause of {kind}")
    # A lookup that fails with a kind of peppy is no failure of the lookup:
    # the lookup gave no answer.
    lookup.enqueue(lookup.Failure("resource_unavailable", "the tree is not received yet"))
    with pytest.raises(Refusal) as raised:
        await placer.place_target(target)
    assert not isinstance(raised.value, geometry.LookupFailure)
    assert (raised.value.kind, raised.value.message) == (
        FailureKind.RESOURCE_UNAVAILABLE,
        "the target: the robot's transform lookup failed with resource_unavailable: the tree is not received yet",
    )


async def test_a_scan_places_the_frame_its_picture_names_where_it_stood_when_the_picture_was_taken():
    frames = store_with([rgb_frame(16, 12, capture_index=4)], [depth_frame(1.0, 16, 12, capture_index=4)])
    # 1 m over the robot frame's origin, looking straight down, as the robot
    # answers for the frame's time, 1.0 s.
    lookup = lookup_service(lookup_answer((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0)))
    placer = await learned_placer(lookup)
    perceiver = Perceiver(frames, CameraModel().with_intrinsics(CAMERA_16X12), placer)

    look = await perceiver.scan(FakeDetector([Box("mug", 0.9, 6, 4, 10, 8)]), [], CancelToken(), 1.0)

    [mug] = look.detections
    assert close(mug.position, (0.0, 0.0, 0.0))
    assert [(request.source_frame, request.source_timestamp, request.target_timestamp) for request in lookup.polled] == [
        (CAMERA_FRAME, 1.0, 1.0)
    ]
    assert (look.frame, look.timestamp, look.capture_index) == ("base_link", 1.0, 4)


async def test_a_scan_whose_picture_names_a_frame_the_tree_does_not_hold_is_refused_before_the_detector_runs():
    frames = store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)])
    lookup = lookup_service()
    lookup.enqueue(lookup.Failure("unknown_frame", "no transform names chest_color_optical_frame"))
    placer = await learned_placer(lookup)
    perceiver = Perceiver(frames, CameraModel().with_intrinsics(CAMERA_16X12), placer)
    detector = FakeDetector([Box("mug", 0.9, 6, 4, 10, 8)])

    with pytest.raises(Refusal, match="^picture 1 dropped: unknown_frame: the frame 'chest_color_optical_frame' at 1.000 s: no transform names") as raised:
        await perceiver.scan(detector, [], CancelToken(), 1.0)
    assert raised.value.kind is FailureKind.RESOURCE_UNAVAILABLE
    assert detector.phrases == []


def test_the_lookup_timeout_is_from_0_to_the_longest_wait_transform_lookup_takes():
    assert geometry.parse_lookup_timeout(0) == 0.0
    assert geometry.parse_lookup_timeout(0.25) == 0.25
    assert geometry.parse_lookup_timeout(geometry.MAX_LOOKUP_TIMEOUT_S) == geometry.MAX_LOOKUP_TIMEOUT_S
    past_the_bound = math.nextafter(geometry.MAX_LOOKUP_TIMEOUT_S, math.inf)
    for value in (-0.1, math.nan, math.inf, past_the_bound, 2.0):
        with pytest.raises(ValueError) as raised:
            geometry.parse_lookup_timeout(value)
        assert str(raised.value) == (
            f"lookup_timeout_s is {value}: set it from 0 to 1 s, the wait transform_lookup's lookup_transform takes"
        )
