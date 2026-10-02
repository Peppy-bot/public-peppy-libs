import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from so101_description import limits, postures, simulation
from so101_description.model import KINEMATICS_URDF_PATH
from so101_description.units import GRIPPER_NAME, JOINT_NAMES


def test_a_simulated_arm_starts_in_home_with_its_jaw_closed():
    assert simulation.start_posture_name() == "home"
    assert simulation.start_gripper_opening() == 0.0
    closed, _ = simulation.gripper_range_rad()
    assert simulation.start_positions_rad() == {
        **dict(zip(JOINT_NAMES, postures.HOME_POSITIONS_RAD, strict=True)),
        GRIPPER_NAME: closed,
    }


def test_the_jaw_range_is_the_urdfs():
    assert simulation.gripper_range_rad() == (-0.174533, 1.74533)


def test_the_start_posture_is_inside_the_urdf_limits():
    start = tuple(simulation.start_positions_rad()[name] for name in JOINT_NAMES)
    assert limits.from_urdf(KINEMATICS_URDF_PATH).contains(start)


def test_the_wrist_camera_hangs_from_the_gripper_link_of_the_urdf():
    links = {link.get("name") for link in ET.parse(KINEMATICS_URDF_PATH).getroot().findall("link")}
    camera = simulation.wrist_camera()
    assert camera.parent_link == "gripper_link"
    assert camera.parent_link in links


def test_the_wrist_camera_is_the_hardware_stream():
    camera = simulation.wrist_camera()
    assert camera.name == "wrist"
    assert (camera.width, camera.height, camera.fps) == (1280, 720, 30)


def test_the_wrist_camera_spans_the_modules_horizontal_field_of_view():
    camera = simulation.wrist_camera()
    half_vertical = math.radians(camera.fovy_deg) / 2.0
    half_horizontal = math.atan(math.tan(half_vertical) * camera.width / camera.height)
    assert math.degrees(2.0 * half_horizontal) == pytest.approx(106.0, abs=0.01)


def test_the_wrist_camera_sees_the_tool_point():
    camera = simulation.wrist_camera()
    w, x, y, z = camera.quat_wxyz
    assert math.isclose(math.sqrt(w * w + x * x + y * y + z * z), 1.0, abs_tol=1e-6)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    tool = _tool_point_in_gripper_link()
    # The tool point in the camera's frame, which looks along its -Z.
    seen = rotation.T @ (tool - np.array(camera.pos))
    assert seen[2] < 0.0
    half_vertical = math.radians(camera.fovy_deg) / 2.0
    half_horizontal = math.atan(math.tan(half_vertical) * camera.width / camera.height)
    assert abs(math.atan2(seen[0], -seen[2])) < half_horizontal
    assert abs(math.atan2(seen[1], -seen[2])) < half_vertical


def _tool_point_in_gripper_link() -> np.ndarray:
    """The origin of gripper_frame_link, the tool point, in gripper_link's
    frame: the fixed joint's offset between them."""
    for joint in ET.parse(KINEMATICS_URDF_PATH).getroot().findall("joint"):
        if joint.get("name") != "gripper_frame_joint":
            continue
        assert joint.find("parent").get("link") == "gripper_link"
        return np.array([float(v) for v in joint.find("origin").get("xyz").split()])
    raise AssertionError("the URDF has no gripper_frame_joint")
