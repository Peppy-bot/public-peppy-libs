# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy>=2", "scipy>=1.13"]
# ///
"""Writes the collision meshes of SO-101's URDF in robot_description, one
binary STL each under robot_description/models/so101/meshes/:

- The convex hull of each part of TheRobotStudio's SO-ARM100 (Apache-2.0) at
  COMMIT, the commit whose so101_new_calib.urdf places the parts, under the
  part's file name. The URDF's <collision> element of each part is
  upstream's, its mesh this hull.
- wrist_camera_module.stl, the wrist camera module in gripper_link's frame:
  the convex hull of the camera mount of MuJoCo Menagerie's robotstudio_so101
  (Apache-2.0) at MENAGERIE_COMMIT, where Menagerie places it on the gripper,
  together with the camera on it at the URDF's wrist_optical_frame.

Run it from any directory:

    uv run robot_description/tools/so101_collision_hulls.py
"""

import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import urlopen

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

COMMIT = "385e8d7c68e24945df6c60d9bd68837a4b7411ae"
SOURCE = (
    f"https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/{COMMIT}/Simulation/SO101/assets"
)
PARTS = [
    "base_motor_holder_so101_v1",
    "base_so101_v2",
    "motor_holder_so101_base_v1",
    "motor_holder_so101_wrist_v1",
    "moving_jaw_so101_v1",
    "rotation_pitch_so101_v1",
    "sts3215_03a_no_horn_v1",
    "sts3215_03a_v1",
    "under_arm_so101_v1",
    "upper_arm_so101_v1",
    "waveshare_mounting_plate_so101_v2",
    "wrist_roll_follower_so101_v1",
    "wrist_roll_pitch_so101_v2",
]

MENAGERIE_COMMIT = "ac6b2b09983786f3036cab1000221017fa2193b4"
CAMERA_MOUNT_SOURCE = (
    f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{MENAGERIE_COMMIT}"
    "/robotstudio_so101/assets/wrist_roll_follower_so101_camera_mount.stl"
)
# The camera mount's pose in Menagerie's gripper body, whose frame is
# gripper_link's: its geom's pos, and its quat "0 1 0 0", a half turn about X.
CAMERA_MOUNT_POSITION = (0.0, -0.000218, 0.00095)
CAMERA_MOUNT_QUAT_XYZW = (1.0, 0.0, 0.0, 0.0)

# The camera on the mount, in the wrist camera's frame, which looks along its
# -Z: a 32 x 32 mm board (SO-ARM100's wrist mounts take a 32 mm x 32 mm
# module) square to the view, reaching MODULE_BACK_M behind the camera past
# the mount's plate, and the M12 lens through the plate, a barrel within
# LENS_RADIUS_M of the view axis. Innomaker's drawing of the module in its
# case (github.com/INNO-MAKER/U20CAM-CASE, UserManual-v1.0.pdf, section 3)
# gives the outer diameter of the lens's front ring, and CASE_LENS_REACH_M,
# how far the front of the focused lens stands ahead of the back of the case,
# which holds the board.
MODULE_SIDE_M = 0.032
MODULE_BACK_M = 0.016
LENS_RADIUS_M = 0.016 / 2
CASE_LENS_REACH_M = 0.020
# The lens's front rim, a polygon of this many sides around its circle.
LENS_SIDES = 16

MODEL = Path(__file__).resolve().parents[1] / "models" / "so101"
URDF = MODEL / "so101.urdf"
COLLISION = MODEL / "meshes"
CAMERA_MODULE = "wrist_camera_module"


def vertices(stl: bytes) -> np.ndarray:
    """The vertices of a binary or an ASCII STL, one row each."""
    if stl.lstrip().startswith(b"solid") and b"facet" in stl[:512]:
        rows = [
            line.split()[1:]
            for line in stl.decode().splitlines()
            if line.strip().startswith("vertex")
        ]
        return np.array(rows, dtype=float)
    (count,) = struct.unpack_from("<I", stl, 80)
    facet = np.dtype([("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")])
    facets = np.frombuffer(stl, dtype=facet, count=count, offset=84)
    return facets["corners"].reshape(-1, 3).astype(float)


def binary_stl(points: np.ndarray, triangles: np.ndarray) -> bytes:
    """A binary STL of `triangles`, rows of indices into `points`, each facet
    wound outward."""
    center = points.mean(axis=0)
    facets = []
    for a, b, c in triangles:
        p, q, r = points[a], points[b], points[c]
        normal = np.cross(q - p, r - p)
        if np.dot(normal, p - center) < 0:
            q, r, normal = r, q, -normal
        normal = normal / np.linalg.norm(normal)
        facets.append(struct.pack("<12fH", *normal, *p, *q, *r, 0))
    return b"collision hull".ljust(80, b"\0") + struct.pack("<I", len(facets)) + b"".join(facets)


def fetched_vertices(url: str) -> np.ndarray:
    with urlopen(url) as response:
        return vertices(response.read())


def camera_on_its_mount(mount: np.ndarray) -> np.ndarray:
    """The corners of the camera's board and the front rim of its lens, in
    gripper_link's frame, from `mount`, the mount's vertices in that frame.

    The board's front seats against the mount's face farthest behind the
    camera within the board's square. The lens, focused as in the case, stands
    ahead of the board's back by CASE_LENS_REACH_M, so ahead of that seat by
    less."""
    (joint,) = [
        joint
        for joint in ET.parse(URDF).getroot().findall("joint")
        if joint.find("child").get("link") == "wrist_optical_frame"
    ]
    origin = joint.find("origin")
    # The optical frame looks along its +Z with +Y down; the camera frame,
    # which looks along -Z with +Y up, is it turned half a turn about X.
    optical = Rotation.from_euler("xyz", [float(v) for v in origin.get("rpy").split()])
    to_gripper = optical * Rotation.from_euler("x", np.pi)
    position = np.array([float(v) for v in origin.get("xyz").split()])
    mount_in_camera = to_gripper.inv().apply(mount - position)
    half = MODULE_SIDE_M / 2
    within_the_board = np.all(np.abs(mount_in_camera[:, :2]) <= half, axis=1)
    seat = mount_in_camera[within_the_board, 2].max()
    lens_front = CASE_LENS_REACH_M - seat
    print(f"board seat {seat:.4f} m behind the camera, lens front {lens_front:.4f} m ahead of it")
    board = [(x, y, z) for x in (-half, half) for y in (-half, half) for z in (0.0, MODULE_BACK_M)]
    rim = LENS_RADIUS_M / np.cos(np.pi / LENS_SIDES)
    angles = np.arange(LENS_SIDES) * 2 * np.pi / LENS_SIDES
    lens = np.column_stack(
        [rim * np.cos(angles), rim * np.sin(angles), np.full(LENS_SIDES, -lens_front)]
    )
    return to_gripper.apply(np.vstack([np.array(board), lens])) + position


def write_hull(name: str, points: np.ndarray) -> None:
    hull = ConvexHull(points)
    path = COLLISION / f"{name}.stl"
    path.write_bytes(binary_stl(points, hull.simplices))
    extent = points.max(axis=0) - points.min(axis=0)
    print(f"{path.name}: {len(hull.vertices)} hull vertices, extent {np.round(extent, 4)} m")


def main():
    COLLISION.mkdir(parents=True, exist_ok=True)
    for part in PARTS:
        write_hull(part, fetched_vertices(f"{SOURCE}/{part}.stl"))
    mount = Rotation.from_quat(CAMERA_MOUNT_QUAT_XYZW).apply(
        fetched_vertices(CAMERA_MOUNT_SOURCE)
    ) + np.array(CAMERA_MOUNT_POSITION)
    write_hull(CAMERA_MODULE, np.vstack([mount, camera_on_its_mount(mount)]))


if __name__ == "__main__":
    main()
