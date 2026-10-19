"""The brain assembled: what it hands its parts, and where the camera model
lives."""

import asyncio
from types import SimpleNamespace

from conftest import PARAMS, FakeDetector, FakeManipulator, FakeToken, camera_poses
from fake_peppygen import fake_bindings

from brain_core.brain import Brain


def answering(data):
    """A consumed service bound to one producer that answers `data`."""

    async def poll(node_runner, producer, timeout):
        return SimpleNamespace(data=data)

    return SimpleNamespace(bound_producer=lambda node_runner: "producer", poll=poll)


async def test_the_geometry_learners_give_the_perceiver_its_camera_and_the_brain_keeps_none():
    colour = SimpleNamespace(
        success=True, message="", width=16, height=12, fx=6.0, fy=6.0, cx=8.0, cy=6.0, distortion_model="none", distortion=[]
    )
    depth = SimpleNamespace(success=True, message="", depth_model="z")
    bindings = fake_bindings(
        get_color_intrinsics=answering(colour),
        get_depth_intrinsics=answering(depth),
        get_camera_poses=answering(camera_poses()),
    )
    brain = Brain(SimpleNamespace(**PARAMS), None, bindings, FakeDetector(), FakeManipulator())
    assert not brain.perceiver.camera.ready
    # The camera slot is vacant, so the frame follower returns at once; the
    # two learners return once they have their answer.
    await asyncio.wait_for(asyncio.gather(*brain.background(FakeToken())), 1.0)
    camera = brain.perceiver.camera
    assert camera.ready
    assert (camera.intrinsics.fx, camera.intrinsics.cx) == (6.0, 8.0)
    assert camera.pose.position == (0.0, 0.0, 0.0)
    assert not hasattr(brain, "camera") and not hasattr(brain, "frames")
