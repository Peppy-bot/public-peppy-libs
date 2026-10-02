"""What every simulation of the SO-101 agrees on and no URDF says: the posture
a simulated arm starts in, how far its jaw is open then, and where its wrist
camera sits.

On hardware the wrist camera is an Innomaker U20CAM-1080P, the 32x32 mm UVC
module of SO-ARM100's wrist camera mount, screwed to the gripper and looking
past the fixed jaw at the grasp. The simulated one has the pose of MuJoCo
Menagerie's wrist_cam on that mount, in the gripper link's frame: 55 mm along
its +Y and 45 mm along its -Z, the way the jaw tips point, turned 0.57 rad
about its x axis so that the tool point is in view. The camera looks along its
own -Z with +Y as image-up, the convention MJCF <camera> and UsdGeom.Camera
share. Its vertical field of view is the one a pinhole camera has with the
module's 106 degree horizontal field of view across a 16:9 image; the lens's
barrel distortion is not modelled. Its stream is the hardware camera's 720p at
30 frames per second, so a dataset recorded in a simulation has the image
shape of one recorded on hardware.

The facts live in simulation.json beside this module, so an engine written in
another language reads the same bytes; each engine's model is held to them by
a test of its own.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import xml.etree.ElementTree as ET

from so101_description import postures
from so101_description.model import KINEMATICS_URDF_PATH
from so101_description.units import GRIPPER_NAME, JOINT_NAMES

SIMULATION_FACTS_PATH = str(Path(__file__).parent / "simulation.json")

_POSTURES_RAD = {
    "home": postures.HOME_POSITIONS_RAD,
    "ready": postures.READY_POSITIONS_RAD,
}


@dataclass(frozen=True)
class WristCamera:
    """The simulated wrist camera: its name on the robot, the URDF link it
    hangs from, its pose in that link's frame, and its stream."""

    name: str
    parent_link: str
    pos: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    fovy_deg: float
    width: int
    height: int
    fps: int


def _read() -> dict:
    return json.loads(Path(SIMULATION_FACTS_PATH).read_text())


def start_posture_name() -> str:
    """The named posture a simulated SO-101 starts in."""
    name = _read()["start_posture"]
    if name not in _POSTURES_RAD:
        raise ValueError(
            f"simulation.json starts the arm in {name!r}, and the postures are "
            f"{sorted(_POSTURES_RAD)}"
        )
    return name


def start_gripper_opening() -> float:
    """How far the jaw is open when a simulated arm starts: a gripper_link
    opening fraction, 0 closed."""
    opening = float(_read()["start_gripper_opening"])
    if not 0.0 <= opening <= 1.0:
        raise ValueError(f"simulation.json opens the jaw to {opening}, outside 0..1")
    return opening


def gripper_range_rad(urdf_path: str = KINEMATICS_URDF_PATH) -> tuple[float, float]:
    """The jaw joint's limits: its closed position and its fully open one."""
    # Direct children only: transmission blocks nest limitless <joint> stubs
    # under the same names.
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        if joint.get("name") != GRIPPER_NAME:
            continue
        limit = joint.find("limit")
        return float(limit.get("lower")), float(limit.get("upper"))
    raise ValueError(f"URDF has no joint named {GRIPPER_NAME}")


def start_positions_rad() -> dict[str, float]:
    """Where every joint starts, joint name to radians: the arm in the start
    posture, in wire order, then the jaw at its start opening."""
    closed, fully_open = gripper_range_rad()
    jaw = closed + start_gripper_opening() * (fully_open - closed)
    arm = zip(JOINT_NAMES, _POSTURES_RAD[start_posture_name()], strict=True)
    return {**dict(arm), GRIPPER_NAME: jaw}


def wrist_camera() -> WristCamera:
    camera = _read()["wrist_camera"]
    return WristCamera(
        name=camera["name"],
        parent_link=camera["parent_link"],
        pos=tuple(float(v) for v in camera["pos"]),
        quat_wxyz=tuple(float(v) for v in camera["quat_wxyz"]),
        fovy_deg=float(camera["fovy_deg"]),
        width=int(camera["color"]["width"]),
        height=int(camera["color"]["height"]),
        fps=int(camera["fps"]),
    )
