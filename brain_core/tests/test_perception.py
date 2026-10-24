"""The camera model, the frame store and the core Perceiver: pixels to
world positions, frames paired by capture, the camera's answers asked
until they come, duplicate boxes, the ways a scan is refused, and the
words a look leaves in the log."""

import asyncio
import math
import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest

from conftest import CAMERA_16X12, NEVER, FakeDetector, FakeToken, camera_poses, depth_frame, rgb_frame
from fake_peppygen import VacantSlot
from brain_core.perception.camera import (
    INVERSE_PLUMB_BOB,
    NONE,
    PLUMB_BOB,
    CameraModel,
    Intrinsics,
    pixel_to_ray,
    rotate,
    undistort,
)
from brain_core.perception import frames as frames_module
from brain_core.perception import geometry
from brain_core.perception.frames import FRAME_BUFFER, FrameStore, decode_color, decode_depth, depth_at
from brain_core.perception.perceiver import DEFAULT_TIMEOUT_S, Look, Perceiver, in_daemon_thread, merge_duplicates
from brain_core.ports import Box, CancelToken, Coverage, Detection, Refusal

# At the robot frame's origin, looking along its +Z: the identity pose.
AT_ORIGIN = CameraModel().with_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
IDENTITY = AT_ORIGIN.with_intrinsics(CAMERA_16X12)


def close(a, b, tolerance=1e-6):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


def test_the_camera_pose_the_robot_reports_is_checked_and_normalised():
    with pytest.raises(ValueError, match="three coordinates"):
        CameraModel().with_pose((0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="finite"):
        CameraModel().with_pose((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="zero"):
        CameraModel().with_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    camera = CameraModel().with_pose((1, 2, 3), (0, 0, 0, 2))
    assert camera.pose.position == (1.0, 2.0, 3.0)
    assert close(camera.pose.orientation, (0.0, 0.0, 0.0, 1.0))
    # Without intrinsics, or without a pose, the camera cannot place a
    # pixel, and says which.
    assert camera.placed and not camera.ready
    with pytest.raises(ValueError, match="intrinsics are not known yet"):
        camera.deproject(1.0, 1.0, 1.0, 16, 12)
    unplaced = CameraModel().with_intrinsics(CAMERA_16X12)
    assert not unplaced.placed and not unplaced.ready
    with pytest.raises(ValueError, match="pose is not known yet"):
        unplaced.deproject(1.0, 1.0, 1.0, 16, 12)
    with pytest.raises(ValueError, match="pose is not known yet"):
        unplaced.forward()
    assert camera.with_intrinsics(CAMERA_16X12).ready


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
    # The optical frame camera_geometry and camera_mounts share: +X to the
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


def quaternion_product(a, b):
    """`a` then `b` as one rotation, both [x, y, z, w]: the rotation `a * b`."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# The OpenArm v2 chest camera as the backbone reports it: the design's mount
# (looking along its own -Z with +Y up) turned half a turn about X into the
# optical frame.
CHEST_POSITION = (0.0792, 0.0315, 0.7941)
CHEST_ORIENTATION = quaternion_product((0.1710647, -0.1710647, -0.6861027, 0.6861027), (1.0, 0.0, 0.0, 0.0))


def test_the_chest_pose_the_backbone_reports_looks_ahead_and_down():
    # The OpenArm v2 chest camera faces the robot's +X, 62 degrees below the
    # horizon, and the image's +Y (down the picture) leans forward and down.
    # Its colour intrinsics: a 52 degree vertical field of view over 720
    # rows is a focal length of 360 / tan(26 degrees), and no lens.
    chest = CameraModel().with_pose(CHEST_POSITION, CHEST_ORIENTATION).with_intrinsics(Intrinsics(1280, 720, 738.1, 738.1, 640.0, 360.0))
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
    with pytest.raises(Refusal, match="unsupported colour encoding"):
        message = rgb_frame(4, 4)
        message.encoding = "yuyv"
        decode_color(message)


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
    store = store_with([rgb_frame(frame_id=1), rgb_frame(frame_id=2), rgb_frame(frame_id=3)], [depth_frame(1.0, frame_id=1), depth_frame(2.0, frame_id=2)])
    frame = store.latest()
    assert frame.color.header.frame_id == 2 and frame.depth.header.frame_id == 2
    assert store.why_unavailable() == "" and store.frames_seen == 3
    # Only the last FRAME_BUFFER frames of a stream are kept.
    for frame_id in range(10, 10 + FRAME_BUFFER + 2):
        store.add_color(rgb_frame(frame_id=frame_id))
    assert [m.header.frame_id for m in store.colors] == list(range(12, 12 + FRAME_BUFFER))


def test_the_frame_store_says_what_is_missing():
    assert FrameStore().why_unavailable() == "no camera frame received"
    assert store_with([rgb_frame()]).why_unavailable() == "no depth frame received"
    assert store_with([rgb_frame()], [depth_frame(1.0)], depth_unit=None).why_unavailable() == "the camera has not answered depth_stream_info yet"
    disjoint = store_with([rgb_frame(frame_id=1), rgb_frame(frame_id=2)], [depth_frame(1.0, frame_id=3)])
    assert disjoint.why_unavailable() == "no colour and depth frame of one capture received (latest colour frame_id 2, depth 3)"
    with pytest.raises(Refusal, match="no colour and depth frame of one capture"):
        disjoint.latest()


def test_frames_that_are_not_aligned_to_each_other_are_refused():
    unaligned = store_with([rgb_frame(align_mode="none")], [depth_frame(1.0, align_mode="none")])
    assert unaligned.why_unavailable() == "the camera's depth is not aligned to its colour (align_mode 'none'); the brain reads depth at colour pixels"
    mixed = store_with([rgb_frame(align_mode="depth_to_color")], [depth_frame(1.0, align_mode="none")])
    assert mixed.why_unavailable() == "the colour and depth frames name different alignments ('depth_to_color' and 'none')"
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

    colours = Feed([rgb_frame(frame_id=1), rgb_frame(frame_id=2)])
    depths = Feed([depth_frame(1.0, frame_id=2)])

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
    assert frame.color.header.frame_id == 2 and frame.depth.header.frame_id == 2
    token.cancel()
    await asyncio.wait_for(running, 1.0)


async def test_a_vacant_camera_slot_leaves_the_store_empty():
    store = FrameStore()
    await asyncio.wait_for(store.run(None, FakeToken(), VacantSlot, VacantSlot, VacantSlot), 1.0)
    assert store.why_unavailable() == "no camera frame received"


def test_duplicate_boxes_of_one_label_keep_the_most_confident():
    a = Box("cup", 0.9, 0, 0, 10, 10)
    b = Box("cup", 0.8, 1, 1, 11, 11)
    other = Box("bowl", 0.7, 1, 1, 11, 11)
    far = Box("cup", 0.6, 50, 50, 60, 60)
    kept, duplicates = merge_duplicates([b, a, other, far])
    assert kept == [a, other, far]
    assert duplicates == [b]


async def test_a_scan_turns_boxes_into_world_detections():
    frames = store_with([rgb_frame(16, 12, frame_id=7)], [depth_frame(2.0, 16, 12, frame_id=7)])
    duplicate = Box("cup", 0.85, 6.5, 4.5, 10.5, 8.5)
    # Its centre is off the picture, where the depth frame has no reading.
    off_the_picture = Box("bowl", 0.5, 30, 30, 40, 40)
    detector = FakeDetector([Box("cup", 0.9, 6, 4, 10, 8), duplicate, off_the_picture])
    perceiver = Perceiver(frames, IDENTITY)
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
    assert (look.image_width, look.image_height, look.frame_timestamp, look.frame_id) == (16, 12, 1.0, 7)
    # The detector's other boxes stay in the look for the log.
    assert look.duplicates == [duplicate]
    assert look.unplaced == [off_the_picture]
    # The detector is handed the search's deadline: the default budget for
    # a zero timeout, the caller's otherwise.
    assert detector.deadlines[-1].budget_s == DEFAULT_TIMEOUT_S
    await perceiver.scan(detector, [], CancelToken(), timeout_s=2.5)
    assert detector.deadlines[-1].budget_s == 2.5
    assert detector.phrases[-1] == []


def test_a_look_names_its_frame_its_age_and_every_box_for_the_log():
    look = Look(
        detections=[Detection("cup", (0.0, 0.0, 2.0), 0.9)],
        image_width=16,
        image_height=12,
        frame_timestamp=10.0,
        frame_id=7,
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
    perceiver = Perceiver(FrameStore(), IDENTITY)
    detector = FakeDetector()
    with pytest.raises(Refusal, match="^no perception source: no camera frame received$"):
        await perceiver.scan(detector, [], CancelToken(), 0.0)
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
    perceiver = Perceiver(frames, AT_ORIGIN)
    detector = FakeDetector([Box("mug", 0.9, 4, 4, 8, 8)])
    assert perceiver.why_unavailable() != ""
    with pytest.raises(Refusal, match="intrinsics have not been received"):
        await perceiver.scan(detector, [], CancelToken(), 1.0)
    perceiver.intrinsics_reason = VACANT
    with pytest.raises(Refusal, match="geometry slot is vacant"):
        await perceiver.scan(detector, [], CancelToken(), 1.0)
    answer = SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])
    perceiver.set_camera(perceiver.camera.with_intrinsics(intrinsics_from(answer)))
    assert perceiver.why_unavailable() == "" and perceiver.intrinsics_reason == ""
    found = await perceiver.scan(detector, [], CancelToken(), 1.0)
    assert [d.label for d in found.detections] == ["mug"]
    with pytest.raises(ValueError, match="unknown distortion model"):
        intrinsics_from(SimpleNamespace(width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="fisheye", distortion=[]))


def colour_answer(success: bool = True, message: str = "", fx: float = 6.0):
    return SimpleNamespace(success=success, message=message, width=16, height=12, fx=fx, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[])


def depth_answer(success: bool = True, message: str = "", depth_model: str = "z"):
    return SimpleNamespace(success=success, message=message, depth_model=depth_model)


async def test_the_camera_is_asked_again_until_it_knows_its_geometry(monkeypatch):
    # The reasons a search is refused with, one per round, until the
    # camera answers both questions with something usable: not answered,
    # colour not known yet (what a sim relay says before its simulation
    # speaks), depth not known, a depth model the brain does not read, an
    # unusable pinhole, then the answer.
    perceiver = Perceiver(FrameStore(), AT_ORIGIN)
    colour = [
        TimeoutError("no answer"),
        colour_answer(False, "no camera geometry received from the simulation yet"),
        colour_answer(),
        colour_answer(),
        colour_answer(fx=0.0),
        colour_answer(),
    ]
    depth = [depth_answer(), depth_answer(False, "no depth stream"), depth_answer(depth_model="range"), depth_answer(), depth_answer()]
    reasons: list[str] = []

    async def poll_colour(node_runner, producer, timeout):
        reasons.append(perceiver.intrinsics_reason)
        answer = colour.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return SimpleNamespace(data=answer)

    async def poll_depth(node_runner, producer, timeout):
        return SimpleNamespace(data=depth.pop(0))

    monkeypatch.setattr(geometry, "INTRINSICS_RETRY_S", 0.0)
    get_color_intrinsics = SimpleNamespace(bound_producer=lambda node_runner: "camera", poll=poll_colour)
    get_depth_intrinsics = SimpleNamespace(poll=poll_depth)
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
    assert perceiver.camera.ready and perceiver.intrinsics_reason == ""


async def test_a_camera_that_never_knows_its_geometry_is_asked_until_the_node_stops(monkeypatch):
    perceiver = Perceiver(FrameStore(), AT_ORIGIN)
    token = FakeToken()
    asked = 0

    async def poll_colour(node_runner, producer, timeout):
        nonlocal asked
        asked += 1
        if asked == 3:
            token.cancel()
        return SimpleNamespace(data=colour_answer(False, "this camera has no calibration"))

    async def poll_depth(node_runner, producer, timeout):
        return SimpleNamespace(data=depth_answer())

    monkeypatch.setattr(geometry, "INTRINSICS_RETRY_S", 0.0)
    get_color_intrinsics = SimpleNamespace(bound_producer=lambda node_runner: "camera", poll=poll_colour)
    get_depth_intrinsics = SimpleNamespace(poll=poll_depth)
    await asyncio.wait_for(geometry.learn_intrinsics(None, token, perceiver, get_color_intrinsics, get_depth_intrinsics), 5.0)
    assert asked == 3
    assert not perceiver.camera.ready
    assert perceiver.intrinsics_reason == "the camera does not know its colour intrinsics: this camera has no calibration"


async def test_a_vacant_geometry_slot_is_the_reason_at_once():
    perceiver = Perceiver(FrameStore(), AT_ORIGIN)
    await asyncio.wait_for(geometry.learn_intrinsics(None, FakeToken(), perceiver, VacantSlot, VacantSlot), 1.0)
    assert perceiver.intrinsics_reason == geometry.VACANT


def poses_answer(success: bool = True, message: str = "", names=("wrist_left", "chest"), chest_position=(0.1, 0.2, 0.3)):
    """The robot's answer as conftest's camera_poses gives it, the last
    camera named, the chest, standing at `chest_position`."""
    answer = camera_poses(success, message, names)
    if success:
        answer.positions[-3:] = list(chest_position)
    return answer


async def test_the_robot_is_asked_again_until_it_places_the_camera(monkeypatch):
    # The reasons a search is refused with, one per round, until the robot
    # answers with success and names the camera: not answered, not measured
    # yet, a robot without that camera, then the answer.
    frames = store_with([rgb_frame(16, 12)], [depth_frame(1.0, 16, 12)])
    perceiver = Perceiver(frames, CameraModel().with_intrinsics(CAMERA_16X12))
    assert perceiver.why_unavailable() != ""
    with pytest.raises(Refusal, match="pose has not been received from the robot"):
        await perceiver.scan(FakeDetector(), [], CancelToken(), 1.0)
    answers = [
        TimeoutError("no answer"),
        poses_answer(False, "the robot has not measured its joints yet"),
        poses_answer(names=("wrist_left", "wrist_right")),
        poses_answer(),
    ]
    reasons: list[str] = []

    async def poll(node_runner, producer, timeout):
        reasons.append(perceiver.pose_reason)
        answer = answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return SimpleNamespace(data=answer)

    monkeypatch.setattr(geometry, "INTRINSICS_RETRY_S", 0.0)
    get_camera_poses = SimpleNamespace(bound_producer=lambda node_runner: "robot", poll=poll)
    await asyncio.wait_for(geometry.learn_camera_pose(None, FakeToken(), perceiver, "chest", get_camera_poses), 5.0)
    assert reasons == [
        "the camera's pose has not been received from the robot yet",
        "the robot's camera mounts are not answered yet (TimeoutError('no answer'))",
        "the robot does not know where its cameras stand: the robot has not measured its joints yet",
        "the robot carries no camera named 'chest' (it carries: wrist_left, wrist_right)",
    ]
    assert answers == []
    assert perceiver.camera.ready and perceiver.pose_reason == ""
    assert perceiver.camera.pose.position == (0.1, 0.2, 0.3)
    assert perceiver.why_unavailable() == ""


async def test_a_vacant_camera_mounts_slot_is_the_reason_at_once():
    perceiver = Perceiver(FrameStore(), CameraModel())
    await asyncio.wait_for(geometry.learn_camera_pose(None, FakeToken(), perceiver, "chest", VacantSlot), 1.0)
    assert perceiver.pose_reason == geometry.MOUNTS_VACANT
    assert not perceiver.camera.placed
