# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyyaml>=6",
#     "xacro @ git+https://github.com/ros/xacro.git@390772abfe1e068f54aed674ce43873229a7db4e",
# ]
# ///
"""Writes the model botshare_base of robot_description, its URDF and its SRDF,
under robot_description/models/botshare_base/, from botshare.ai's OpenAMRobot
2.0 base: the description package openamrobot_description of
openAMRobot/openamr-platform-sw (MIT) at COMMIT, expanded by xacro 2.1.1
(ros/xacro, BSD-3-Clause) at its defaults.

The URDF is upstream's expansion, with the changes URDF_HEADER lists and the
repository's licence after that header. The SRDF
disables every pair of links with collision geometry, as MoveIt's setup
assistant names the reasons: Adjacent for two links one joint joins, or two
neighbors of a link with no collision geometry, and Never for two whose bounds
in the robot frame stand apart. The base is rigid but for its two wheels, each a cylinder
about its joint's axis, so the bounds of every link hold wherever the wheels
turn. Any other pair stops the tool, naming it.

Run it from any directory:

    uv run robot_description/tools/botshare_base.py          # writes the model
    uv run robot_description/tools/botshare_base.py --check  # fails unless the model is what it writes
"""

import argparse
import itertools
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import urlopen

import xacro
import yaml

COMMIT = "69847ab7c9c6cd49b3022dc1a62f86c044ba10ff"
REPOSITORY = f"https://raw.githubusercontent.com/openAMRobot/openamr-platform-sw/{COMMIT}"
SOURCE = f"{REPOSITORY}/ros2/src/openamrobot_description"
# The files of the package the expansion reads, by their paths in it.
PACKAGE_FILES = [
    "urdf/robo_urdf.urdf.xacro",
    "urdf/sensors.xacro",
    "urdf/gazebo_control.xacro",
    "config/roadmap_mass_export.yaml",
    "config/sensor_profiles.yaml",
]
TOP = "urdf/robo_urdf.urdf.xacro"

MODEL = "botshare_base"
MODEL_DIR = Path(__file__).resolve().parent.parent / "models" / MODEL

# Upstream's frames of the legacy IMX708 camera, which the 2.0 base does not carry.
LEGACY_CAMERA = {"links": ["camera_link", "camera_optical_frame"], "joints": ["camera_joint", "camera_optical_joint"]}
# The Gemini 336L: upstream's two optical frames at one place, and the one this model keeps.
CAMERA = "base_camera"
CAMERA_PROFILE = "base_camera_link"
COLOR_FRAME = ("base_camera_color_optical_frame", "base_camera_color_optical_joint")
DEPTH_FRAME = ("base_camera_depth_optical_frame", "base_camera_depth_optical_joint")
OPTICAL_FRAME = (f"{CAMERA}_optical_frame", f"{CAMERA}_optical_joint")

URDF_HEADER = f"""<!--
  botshare.ai's OpenAMRobot 2.0 base: a chassis on two driven hub-motor wheels
  and four casters, with an Orbbec Gemini 336L depth camera at its front and an
  RPLIDAR S3 2-D lidar. robot_description/tools/botshare_base.py writes this
  file from openamrobot_description of openAMRobot/openamr-platform-sw (MIT),
  expanded by xacro at its defaults, at commit
  {COMMIT}.
  That commit's files give the source of every value.

  base_footprint, the root, stands below the midpoint of the drive axle, +x
  forward, +y left and +z up (REP-103, REP-105), and a positive turn of either
  wheel rolls the base forward.

  What it changes in upstream's expansion:
  - the robot's name, {MODEL};
  - upstream's comments and <gazebo> elements are left out;
  - camera_link and camera_optical_frame are left out: the legacy IMX708
    camera, which the 2.0 base does not carry;
  - the Gemini 336L has one optical frame, {OPTICAL_FRAME[0]}, upstream's
    {COLOR_FRAME[0]}; upstream's {DEPTH_FRAME[0]}
    stands at the same place and is left out. Its depth sensor measures from the
    range_min to the range_max of upstream's config/sensor_profiles.yaml.
-->"""

SRDF_HEADER = """<!--
  robot_description/tools/botshare_base.py writes this file. A base has no arm
  and no gripper, so it has no group and no named state. Each pair of links
  with collision geometry is disabled: Adjacent for two links one joint joins,
  or two neighbors of a link with no collision geometry, and Never for two
  whose bounds in the robot frame stand apart, which they do wherever the
  wheels turn.
-->"""


def fetch_package(dest: Path) -> None:
    """Writes PACKAGE_FILES under `dest`, each from SOURCE at COMMIT."""
    for path in PACKAGE_FILES:
        target = dest / path
        target.parent.mkdir(parents=True, exist_ok=True)
        with urlopen(f"{SOURCE}/{path}") as response:
            target.write_bytes(response.read())


def upstream_license() -> str:
    """The licence of the repository at COMMIT, its LICENSE file."""
    with urlopen(f"{REPOSITORY}/LICENSE") as response:
        return response.read().decode()


def license_comment(license_text: str) -> str:
    """`license_text` as an XML comment, each line indented."""
    if "--" in license_text:
        sys.exit("the upstream licence holds '--', which an XML comment cannot carry")
    lines = [f"  {line}".rstrip() for line in license_text.strip().splitlines()]
    return "<!--\n  The licence of openamr-platform-sw, which covers this file:\n\n" + "\n".join(lines) + "\n-->"


def expanded(package: Path) -> ET.Element:
    """Upstream's URDF: TOP expanded by xacro at its defaults, without comments."""
    os.chdir(package / Path(TOP).parent)
    document = xacro.process_file(Path(TOP).name, mappings={})
    return ET.fromstring(document.toxml())


def one(root: ET.Element, tag: str, name: str) -> ET.Element:
    """The one element `<tag name="name">` under `root`; stops the tool unless upstream has it."""
    found = [element for element in root.findall(tag) if element.get("name") == name]
    if len(found) != 1:
        sys.exit(f"upstream's URDF at {COMMIT} has {len(found)} {tag} named {name}, not one")
    return found[0]


def remove(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = one(root, tag, name)
    root.remove(element)
    return element


def placement(joint: ET.Element) -> tuple:
    origin = joint.find("origin")
    return (joint.find("parent").get("link"), origin.get("xyz"), origin.get("rpy"))


def depth_range(package: Path) -> tuple[float, float]:
    """The Gemini 336L's depth range (m) in upstream's sensor profiles."""
    profiles = yaml.safe_load((package / "config/sensor_profiles.yaml").read_text())
    depth = profiles["frames"][CAMERA_PROFILE]["channels"]["depth"]
    values = []
    for key in ["range_min", "range_max"]:
        entry = depth[key]
        if entry["unit"] != "m" or entry["status"] != "datasheet":
            sys.exit(f"upstream's {CAMERA_PROFILE} depth {key} is {entry}, not a datasheet value in m")
        values.append(float(entry["value"]))
    return values[0], values[1]


def adapted(root: ET.Element, near: float, far: float) -> ET.Element:
    """Upstream's URDF with the changes URDF_HEADER lists."""
    root.set("name", MODEL)
    for gazebo in root.findall("gazebo"):
        root.remove(gazebo)
    for link in LEGACY_CAMERA["links"]:
        remove(root, "link", link)
    for joint in LEGACY_CAMERA["joints"]:
        remove(root, "joint", joint)

    color_joint = one(root, "joint", COLOR_FRAME[1])
    depth_joint = remove(root, "joint", DEPTH_FRAME[1])
    remove(root, "link", DEPTH_FRAME[0])
    if placement(depth_joint) != placement(color_joint):
        sys.exit(f"upstream's {DEPTH_FRAME[0]} stands apart from {COLOR_FRAME[0]}")
    one(root, "link", COLOR_FRAME[0]).set("name", OPTICAL_FRAME[0])
    color_joint.set("name", OPTICAL_FRAME[1])
    color_joint.find("child").set("link", OPTICAL_FRAME[0])
    gazebo = ET.SubElement(root, "gazebo", reference=OPTICAL_FRAME[0])
    sensor = ET.SubElement(gazebo, "sensor", name=CAMERA, type="depth")
    clip = ET.SubElement(ET.SubElement(sensor, "camera"), "clip")
    ET.SubElement(clip, "near").text = repr(near)
    ET.SubElement(clip, "far").text = repr(far)
    return root


def vector(text: str | None) -> list[float]:
    return [float(value) for value in (text or "0 0 0").split()]


def rotation(rpy: list[float]) -> list[list[float]]:
    """The matrix of a URDF origin's roll, pitch and yaw, each about the parent's fixed axes."""
    (sr, cr), (sp, cp), (sy, cy) = ((math.sin(a), math.cos(a)) for a in rpy)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def times(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def frame(origin: ET.Element | None) -> tuple[list[list[float]], list[float]]:
    if origin is None:
        return rotation([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0]
    return rotation(vector(origin.get("rpy"))), vector(origin.get("xyz"))


def compose(outer, inner):
    (r_outer, p_outer), (r_inner, p_inner) = outer, inner
    moved = apply(r_outer, p_inner)
    return times(r_outer, r_inner), [p_outer[i] + moved[i] for i in range(3)]


def link_frames(root: ET.Element) -> dict[str, tuple]:
    """Each link's frame in the root link's, every joint at zero."""
    joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    frames: dict[str, tuple] = {}

    def of(link: str):
        if link not in frames:
            joint = joints.get(link)
            frames[link] = (
                frame(None) if joint is None else compose(of(joint.find("parent").get("link")), frame(joint.find("origin")))
            )
        return frames[link]

    for link in root.findall("link"):
        of(link.get("name"))
    return frames


def half_extent(element: ET.Element, turn: list[list[float]]) -> list[float]:
    """The half size, along each axis of the root frame, of a collision element's geometry turned by `turn`."""
    (geometry,) = list(element.find("geometry"))
    if geometry.tag == "box":
        half = [size / 2 for size in vector(geometry.get("size"))]
        return [sum(abs(turn[i][k]) * half[k] for k in range(3)) for i in range(3)]
    if geometry.tag == "sphere":
        return [float(geometry.get("radius"))] * 3
    if geometry.tag == "cylinder":
        radius, length = float(geometry.get("radius")), float(geometry.get("length"))
        axis = [turn[i][2] for i in range(3)]
        return [radius * math.sqrt(max(0.0, 1 - a * a)) + length / 2 * abs(a) for a in axis]
    sys.exit(f"a collision element of upstream's URDF is a {geometry.tag}, which the tool does not bound")


def bounds(root: ET.Element) -> dict[str, tuple[list[float], list[float]]]:
    """The box, in the root link's frame, that holds the collision geometry of each link that has some."""
    frames = link_frames(root)
    boxes = {}
    for link in root.findall("link"):
        elements = link.findall("collision")
        if not elements:
            continue
        lows, highs = [], []
        for element in elements:
            turn, center = compose(frames[link.get("name")], frame(element.find("origin")))
            half = half_extent(element, turn)
            lows.append([center[i] - half[i] for i in range(3)])
            highs.append([center[i] + half[i] for i in range(3)])
        boxes[link.get("name")] = ([min(c) for c in zip(*lows)], [max(c) for c in zip(*highs)])
    return boxes


def is_wheel(joint: ET.Element, links: dict[str, ET.Element]) -> bool:
    """Whether `joint` is continuous and its link's one collision element a cylinder about the
    joint's axis, a shape that stays where it is wherever the joint turns."""
    if joint.get("type") != "continuous":
        return False
    elements = links[joint.find("child").get("link")].findall("collision")
    if len(elements) != 1 or elements[0].find("geometry/cylinder") is None:
        return False
    cylinder_axis = [row[2] for row in rotation(vector(elements[0].find("origin").get("rpy")))]
    joint_axis = vector(joint.find("axis").get("xyz"))
    alignment = sum(c * j for c, j in zip(cylinder_axis, joint_axis))
    return abs(abs(alignment) - 1) < 1e-6


def hold_rigid_but_wheels(root: ET.Element) -> None:
    """Stops the tool unless every joint is fixed or a wheel."""
    links = {link.get("name"): link for link in root.findall("link")}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed" and not is_wheel(joint, links):
            sys.exit(f"joint {joint.get('name')} moves a link that is no wheel; the tool derives the pairs from one pose")


def adjacent_pairs(root: ET.Element) -> set[frozenset[str]]:
    """The pairs of links MoveIt's setup assistant calls adjacent: two links one joint joins,
    and any two neighbors of a link with no collision geometry."""
    neighbors: dict[str, set[str]] = {link.get("name"): set() for link in root.findall("link")}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        neighbors[parent].add(child)
        neighbors[child].add(parent)
    pairs = {frozenset((link, other)) for link, others in neighbors.items() for other in others}
    for link in root.findall("link"):
        if not link.findall("collision"):
            around = sorted(neighbors[link.get("name")])
            pairs |= {frozenset(pair) for pair in itertools.combinations(around, 2)}
    return pairs


def disabled_pairs(root: ET.Element) -> list[tuple[str, str, str]]:
    """Every pair of links with collision geometry, each (link1, link2, reason), link1 before link2."""
    hold_rigid_but_wheels(root)
    boxes = bounds(root)
    adjacent = adjacent_pairs(root)
    pairs = []
    for first, second in itertools.combinations(sorted(boxes), 2):
        if frozenset((first, second)) in adjacent:
            pairs.append((first, second, "Adjacent"))
            continue
        (low_a, high_a), (low_b, high_b) = boxes[first], boxes[second]
        apart = any(high_a[i] < low_b[i] or high_b[i] < low_a[i] for i in range(3))
        if not apart:
            sys.exit(f"links {first} and {second} meet and no joint joins them: the SRDF needs a reason for the pair")
        pairs.append((first, second, "Never"))
    return pairs


def urdf_text(root: ET.Element, license_text: str) -> str:
    ET.indent(root, space="  ")
    return (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        + URDF_HEADER
        + "\n"
        + license_comment(license_text)
        + "\n"
        + ET.tostring(root, encoding="unicode")
        + "\n"
    )


def srdf_text(pairs: list[tuple[str, str, str]]) -> str:
    lines = [f'    <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>' for a, b, reason in pairs]
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        + SRDF_HEADER
        + f'\n<robot name="{MODEL}">\n'
        + "\n".join(lines)
        + "\n</robot>\n"
    )


def model_files() -> dict[str, str]:
    with tempfile.TemporaryDirectory() as scratch:
        package = Path(scratch)
        fetch_package(package)
        near, far = depth_range(package)
        root = adapted(expanded(package), near, far)
        pairs = disabled_pairs(root)
        return {f"{MODEL}.urdf": urdf_text(root, upstream_license()), f"{MODEL}.srdf": srdf_text(pairs)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail unless the model's files are what the tool writes")
    check = parser.parse_args().check
    files = model_files()
    if check:
        stale = [name for name, text in files.items() if (MODEL_DIR / name).read_text() != text]
        if stale:
            sys.exit(f"{', '.join(stale)} under {MODEL_DIR} differ from what the tool writes: run it without --check")
        return
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    for name, text in files.items():
        (MODEL_DIR / name).write_text(text)


if __name__ == "__main__":
    main()
