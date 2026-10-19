import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
import pytest

from so101_description import limits, postures, simulation
from so101_description.kinematics import Kinematics
from so101_description.model import KINEMATICS_URDF_PATH
from so101_description.transforms import matrix_from_pose
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


# The wrist camera's picture is upright when world-up stands within this angle
# of image-up on the picture plane.
_PICTURE_ROLL_BAR_RAD = math.radians(10.0)
_FORWARD = np.array([1.0, 0.0, 0.0])
_UP = np.array([0.0, 0.0, 1.0])


@pytest.fixture(scope="module")
def kinematics():
    return Kinematics(KINEMATICS_URDF_PATH)


def test_the_ready_posture_is_inside_the_urdf_limits():
    assert limits.from_urdf(KINEMATICS_URDF_PATH).contains(postures.READY_POSITIONS_RAD)


def test_the_wrist_camera_orientation_is_a_unit_quaternion():
    w, x, y, z = simulation.wrist_camera().quat_wxyz
    assert math.isclose(math.sqrt(w * w + x * x + y * y + z * z), 1.0, abs_tol=1e-6)


def test_the_wrist_camera_sees_the_tool_point():
    camera = simulation.wrist_camera()
    tool, _ = _gripper_frame_joint_origin()
    # The tool point in the camera's frame, which looks along its -Z.
    seen = _camera_in_gripper_link().T @ (tool - np.array(camera.pos))
    assert seen[2] < 0.0
    half_vertical = math.radians(camera.fovy_deg) / 2.0
    half_horizontal = math.atan(math.tan(half_vertical) * camera.width / camera.height)
    assert abs(math.atan2(seen[0], -seen[2])) < half_horizontal
    assert abs(math.atan2(seen[1], -seen[2])) < half_vertical


def test_at_ready_the_wrist_camera_looks_ahead_and_down_with_its_picture_upright(kinematics):
    axes = _wrist_camera_axes_in_base(kinematics, postures.READY_POSITIONS_RAD)
    assert axes.view @ _FORWARD > 0.7
    assert axes.view @ _UP < -0.4
    assert abs(axes.picture_roll_rad(_UP)) < _PICTURE_ROLL_BAR_RAD


@dataclass(frozen=True)
class _CameraAxes:
    """The wrist camera's picture axes in base_link: +X right, +Y up, and the
    direction it looks along, its -Z."""

    right: np.ndarray
    up: np.ndarray
    view: np.ndarray

    def picture_roll_rad(self, direction: np.ndarray) -> float:
        """How far the direction stands from the top of the picture, clockwise."""
        return math.atan2(direction @ self.right, direction @ self.up)


def _wrist_camera_axes_in_base(
    kinematics: Kinematics, positions_rad: tuple[float, ...]
) -> _CameraAxes:
    position, orientation = kinematics.forward_kinematics(positions_rad)
    tool_in_base = matrix_from_pose(position, orientation)[:3, :3]
    _, rpy = _gripper_frame_joint_origin()
    gripper_link_in_base = tool_in_base @ _rotation_from_rpy(rpy).T
    camera_in_base = gripper_link_in_base @ _camera_in_gripper_link()
    return _CameraAxes(
        right=camera_in_base[:, 0], up=camera_in_base[:, 1], view=-camera_in_base[:, 2]
    )


def _camera_in_gripper_link() -> np.ndarray:
    """The wrist camera's rotation in gripper_link's frame."""
    w, x, y, z = simulation.wrist_camera().quat_wxyz
    return matrix_from_pose((0.0, 0.0, 0.0), (x, y, z, w))[:3, :3]


def _rotation_from_rpy(rpy: np.ndarray) -> np.ndarray:
    """A URDF origin's rotation: roll about X, then pitch about Y, then yaw
    about Z, each about the fixed parent axes."""
    roll, pitch, yaw = rpy
    about_x = np.array(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
    )
    about_y = np.array(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]
    )
    about_z = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    )
    return about_z @ about_y @ about_x


def _gripper_frame_joint_origin() -> tuple[np.ndarray, np.ndarray]:
    """The origin of gripper_frame_link in gripper_link's frame, as the fixed
    joint between them places it: its xyz is the tool point."""
    for joint in ET.parse(KINEMATICS_URDF_PATH).getroot().findall("joint"):
        if joint.get("name") != "gripper_frame_joint":
            continue
        assert joint.find("parent").get("link") == "gripper_link"
        origin = joint.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()])
        rpy = np.array([float(v) for v in origin.get("rpy").split()])
        return xyz, rpy
    raise AssertionError("the URDF has no gripper_frame_joint")
