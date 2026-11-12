"""A description from Python: read from its two files, its arms with their
grippers, its named states, its cameras and its joint limits."""

import xml.etree.ElementTree as ET

import pytest

from robot_description_py import CLOSED, HOME, OPEN, READY, RobotDescription, model


def test_files_of_two_robots_do_not_make_one_description():
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(model("so101").urdf, model("openarm_v2").srdf)
    assert "the SRDF is of the robot" in str(refusal.value)


def test_a_file_that_does_not_read_is_refused_naming_its_file():
    so101 = model("so101")
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(so101.urdf, "<robot")
    assert str(refusal.value).startswith("the SRDF does not parse:")


def test_a_parsed_description_is_the_bundled_one():
    so101 = model("so101")
    parsed = RobotDescription.parse(so101.urdf, so101.srdf)
    assert [arm.name for arm in parsed.arms] == [arm.name for arm in so101.description().arms]
    assert (parsed.urdf, parsed.srdf) == (so101.urdf, so101.srdf)


def test_each_arm_gives_its_chain_and_its_gripper():
    description = model("openarm_v2").description()
    for arm, side in zip(description.arms, ["left", "right"], strict=True):
        assert arm.name == f"{side}_arm"
        assert arm.joints == tuple(f"openarm_{side}_joint{index}" for index in range(1, 8))
        assert arm.gripper.name == f"{side}_gripper"
        fingers = arm.gripper.fingers
        assert [finger.joint for finger in fingers] == [
            f"openarm_{side}_finger_joint1",
            f"openarm_{side}_finger_joint2",
        ]
        closed = description.state(arm.gripper.name, CLOSED)
        opened = description.state(arm.gripper.name, OPEN)
        assert closed is not None and opened is not None
        assert fingers[0].closed == closed[0] and fingers[0].open == opened[0]
        # The second finger follows the first one to one, as the URDF's <mimic> gives it.
        assert fingers[0].mimic is None
        mimic = fingers[1].mimic
        assert mimic is not None
        assert (mimic.joint, mimic.multiplier, mimic.offset) == (fingers[0].joint, 1.0, 0.0)


def test_a_state_gives_one_position_per_joint_or_none():
    description = model("so101").description()
    (arm,) = description.arms
    for posture in [READY, HOME]:
        positions = description.state(arm.name, posture)
        assert positions is not None and len(positions) == len(arm.joints)
    assert description.state(arm.name, "no_such_state") is None
    assert description.state("no_such_group", READY) is None


def test_each_camera_gives_its_optical_frame():
    description = model("openarm_v2").description()
    cameras = {camera.name: camera for camera in description.cameras}
    assert set(cameras) == {"wrist_left", "wrist_right", "chest"}
    assert cameras["chest"].parent_link == "openarm_body_link0"
    for camera in cameras.values():
        assert len(camera.xyz) == 3 and len(camera.rpy) == 3


def test_a_depth_camera_gives_the_depths_its_sensor_clips_at():
    v2 = model("openarm_v2")
    cameras = {camera.name: camera for camera in v2.description().cameras}
    clip = ET.fromstring(v2.urdf).find("gazebo[@reference='chest_optical_frame']/sensor/camera/clip")
    assert clip is not None
    depth = cameras["chest"].depth_range
    assert depth is not None
    assert (depth.near, depth.far) == (float(clip.findtext("near")), float(clip.findtext("far")))
    assert cameras["wrist_left"].depth_range is None and cameras["wrist_right"].depth_range is None


def test_a_depth_sensor_that_does_not_read_is_refused_naming_it():
    v2 = model("openarm_v2")
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(v2.urdf.replace("<far>", "<far>far", 1), v2.srdf)
    assert 'the <far> of the depth sensor on "chest_optical_frame"' in str(refusal.value)


def test_joint_limits_are_the_urdfs():
    so101 = model("so101")
    description = so101.description()
    joints = {joint.get("name"): joint for joint in ET.fromstring(so101.urdf).findall("joint")}
    for arm in description.arms:
        for joint in arm.joints:
            limit = joints[joint].find("limit")
            assert limit is not None
            limits = description.joint_limits(joint)
            assert (limits.lower, limits.upper) == (float(limit.get("lower")), float(limit.get("upper")))
            assert limits.velocity == float(limit.get("velocity"))
            assert limits.effort == float(limit.get("effort"))
    with pytest.raises(KeyError):
        description.joint_limits("no_such_joint")
