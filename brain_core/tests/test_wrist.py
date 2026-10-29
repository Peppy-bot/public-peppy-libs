"""The wrist cameras' frames: the parameter that names them, the follower,
the wait for a newer frame, and the JPEG a model gets."""

import asyncio
import io
from types import SimpleNamespace

import pytest
from conftest import FakeToken, rgb_frame
from fake_peppygen import FakeTopic

from brain_core.perception.wrist import NO_SOURCE, VACANT, WristFrames, jpeg_of, parse_wrist_cameras
from brain_core.ports import ExpectedFailure, FailureKind

ARMS = ["left_arm", "right_arm"]


def camera(instance_id: str):
    return SimpleNamespace(core_node="cameras", instance_id=instance_id)


def frame(timestamp: float, frame_id: int = 1):
    message = rgb_frame(frame_id=frame_id)
    message.header.timestamp = timestamp
    return message


def test_the_parameter_names_each_arms_camera():
    assert parse_wrist_cameras("left_arm:wrist_left, right_arm:wrist_right", ARMS) == {"left_arm": "wrist_left", "right_arm": "wrist_right"}
    assert parse_wrist_cameras("left_arm:wrist_left", ARMS) == {"left_arm": "wrist_left"}
    assert parse_wrist_cameras("", ARMS) == {}


@pytest.mark.parametrize("text, why", [
    ("wrist_left", "'wrist_left' is not 'arm:camera'"),
    ("left_arm:", "'left_arm:' is not 'arm:camera'"),
    ("head:cam", "'head' is not an arm of this robot"),
    ("left_arm:a,left_arm:b", "arm 'left_arm' is named twice"),
    ("left_arm:a,right_arm:a", "camera 'a' is named twice"),
])
def test_a_parameter_the_store_cannot_read_is_refused(text, why):
    with pytest.raises(ValueError, match=why):
        parse_wrist_cameras(text, ARMS)


async def test_frames_are_kept_by_the_producers_instance_id_and_a_newer_one_is_waited_for():
    topic = FakeTopic()
    wrists = WristFrames({"left_arm": "wrist_left", "right_arm": "wrist_right"})
    assert wrists.why_unavailable("left_arm") == "no frame from wrist camera 'wrist_left' yet"
    token = FakeToken()
    follower = asyncio.ensure_future(wrists.run(None, token, topic))
    topic.push(frame(1.0), camera("wrist_left"))
    topic.push(frame(5.0), camera("wrist_right"))
    topic.push(frame(2.0), camera("chest"))
    first = await asyncio.wait_for(wrists.newer_than("left_arm", float("-inf"), 1.0), 1.0)
    assert first.header.timestamp == 1.0 and wrists.why_unavailable("left_arm") == ""
    assert wrists.latest("right_arm").header.timestamp == 5.0
    assert wrists.latest_stamp("left_arm") == 1.0
    waiting = asyncio.ensure_future(wrists.newer_than("left_arm", 1.0, 1.0))
    await asyncio.sleep(0)
    assert not waiting.done()
    topic.push(frame(1.5, frame_id=2), camera("wrist_left"))
    assert (await asyncio.wait_for(waiting, 1.0)).header.frame_id == 2
    token.cancel()
    await asyncio.wait_for(follower, 1.0)


async def test_no_newer_frame_within_the_wait_gives_none():
    wrists = WristFrames({"left_arm": "wrist_left"})
    wrists.add("wrist_left", frame(1.0))
    assert await wrists.newer_than("left_arm", 1.0, 0.0) is None
    assert await wrists.newer_than("right_arm", float("-inf"), 0.0) is None


async def test_an_arm_with_no_camera_and_a_brain_with_no_slot_say_so():
    wrists = WristFrames({"left_arm": "wrist_left"})
    assert wrists.why_unavailable("right_arm") == NO_SOURCE.format(arm="right_arm")
    assert wrists.camera_of("right_arm") is None and wrists.latest_stamp("right_arm") == float("-inf")
    await wrists.run(None, FakeToken(), None)
    assert wrists.why_unavailable("left_arm") == VACANT


async def test_a_slot_with_no_camera_bound_yet_is_followed_for_the_cameras_a_join_brings():
    topic = FakeTopic()
    wrists = WristFrames({"left_arm": "wrist_left"})
    token = FakeToken()
    follower = asyncio.ensure_future(wrists.run(None, token, topic))
    await asyncio.wait_for(topic.subscribed.wait(), 1.0)
    assert wrists.why_unavailable("left_arm") == "no frame from wrist camera 'wrist_left' yet"
    token.cancel()
    await asyncio.wait_for(follower, 1.0)


def test_the_jpeg_of_an_mjpeg_frame_is_its_own_bytes_and_an_rgb8_frame_is_encoded():
    from PIL import Image

    mjpeg = SimpleNamespace(encoding="mjpeg", frame=b"\xff\xd8jpeg", width=16, height=12)
    assert jpeg_of(mjpeg) == b"\xff\xd8jpeg"
    encoded = jpeg_of(rgb_frame())
    with Image.open(io.BytesIO(encoded)) as picture:
        assert picture.format == "JPEG" and picture.size == (16, 12)


def test_a_frame_that_cannot_be_decoded_fails_the_step():
    with pytest.raises(ExpectedFailure, match="unsupported colour encoding") as failed:
        jpeg_of(SimpleNamespace(encoding="yuv", frame=b"", width=16, height=12))
    assert failed.value.kind is FailureKind.ATTEMPT_FAILED


async def test_a_camera_run_as_a_copys_instance_is_known_by_its_fragments_id():
    """In a stack, alpha's wrist camera runs as `alpha_wrist_left`; the
    slot's members say it belongs to copy `alpha`, so its frames are the
    frames of `wrist_left`."""
    from brain_core.perception.wrist import camera_id

    alpha_left = camera("alpha_wrist_left")
    members = [
        SimpleNamespace(producer=alpha_left, copy=SimpleNamespace(name="alpha", instance_id="wrist_left")),
        SimpleNamespace(producer=camera("wrist_right"), copy=None),
    ]
    assert camera_id(alpha_left, members) == "wrist_left"
    assert camera_id(camera("wrist_right"), members) == "wrist_right"
    # A copy given by its name alone: the prefix comes off.
    assert camera_id(alpha_left, [SimpleNamespace(producer=alpha_left, copy="alpha")]) == "wrist_left"
    # A producer the members do not list keeps its instance id.
    assert camera_id(camera("beta_wrist_left"), members) == "beta_wrist_left"
    topic = FakeTopic(members)
    wrists = WristFrames({"left_arm": "wrist_left"})
    token = FakeToken()
    follower = asyncio.ensure_future(wrists.run(None, token, topic))
    topic.push(frame(1.0), alpha_left)
    assert (await asyncio.wait_for(wrists.newer_than("left_arm", float("-inf"), 1.0), 1.0)).header.timestamp == 1.0
    token.cancel()
    await asyncio.wait_for(follower, 1.0)
