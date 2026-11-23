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
repository's licence after that header. The SRDF disables every pair of links
with collision geometry, as MoveIt's setup assistant names the reasons:
Adjacent for two links that one joint joins, or that links with no collision
geometry alone join between them, and Never for every other pair.
collision_geometry's test of the bundled base holds the bounds of each Never
pair in the robot frame apart wherever the wheels turn.

Run it from any directory:

    uv run robot_description/tools/botshare_base.py          # writes the model
    uv run robot_description/tools/botshare_base.py --check  # fails unless the model is what it writes
"""

import argparse
import itertools
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
  with collision geometry is disabled: Adjacent for two links that one joint
  joins, or that links with no collision geometry alone join between them, and
  Never for two whose bounds in the robot frame stand apart, which they do
  wherever the wheels turn.
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


def adjacent_pairs(root: ET.Element) -> set[frozenset[str]]:
    """The pairs of links MoveIt's setup assistant calls adjacent: two links that one joint joins,
    or that links with no collision geometry alone join between them, as its connection graph
    joins the neighbors of each link with no geometry until no pair is new."""
    neighbors: dict[str, set[str]] = {link.get("name"): set() for link in root.findall("link")}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        neighbors[parent].add(child)
        neighbors[child].add(parent)
    bare = {link.get("name") for link in root.findall("link") if not link.findall("collision")}
    pairs: set[frozenset[str]] = set()
    for start in neighbors:
        reached, through = {start}, [start]
        while through:
            for other in neighbors[through.pop()] - reached:
                reached.add(other)
                pairs.add(frozenset((start, other)))
                if other in bare:
                    through.append(other)
    return pairs


def disabled_pairs(root: ET.Element) -> list[tuple[str, str, str]]:
    """Every pair of links with collision geometry, each (link1, link2, reason), link1 before link2:
    Adjacent for an adjacent pair, Never for any other."""
    colliding = sorted(link.get("name") for link in root.findall("link") if link.findall("collision"))
    adjacent = adjacent_pairs(root)
    return [
        (first, second, "Adjacent" if frozenset((first, second)) in adjacent else "Never")
        for first, second in itertools.combinations(colliding, 2)
    ]


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
