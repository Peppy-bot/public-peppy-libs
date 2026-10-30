"""Writes the forward kinematics scenarios of the ROS reference harness.

The configurations of each chain are its zero configuration, its lower limits, its upper
limits, and configurations drawn uniformly inside its limits, five of them by default.
Run it from any directory:

    python3 ros_reference/tools/draw_forward_kinematics.py
    python3 ros_reference/tools/draw_forward_kinematics.py --drawn 500

The second draws a large set for a local run, which the README describes.
"""

import argparse
import xml.etree.ElementTree as ElementTree

from drawing import REPOSITORY, configurations, generator, joint_limits, write

DRAWN_CONFIGURATIONS = 5


def openarm_arm(side):
    return (side, [f"openarm_{side}_joint{i}" for i in range(1, 8)], f"openarm_{side}_tcp")


ROBOTS = {
    "forward_kinematics_openarm_v2": (
        "openarm_description/assets/openarm_v20.urdf",
        [openarm_arm("left"), openarm_arm("right")],
    ),
    "forward_kinematics_so101": (
        "so101_description/src/so101_description/urdf/so101_kinematics.urdf",
        [
            (
                "arm",
                ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
                "gripper_frame_link",
            )
        ],
    ),
}


def scenario(rng, urdf, chains, drawn_count):
    robot = ElementTree.parse(REPOSITORY / urdf).getroot()
    return {
        "kind": "forward_kinematics",
        "urdf": urdf,
        "chains": [
            {
                "name": name,
                "joints": joints,
                "tip_link": tip_link,
                "configurations": configurations(
                    rng, [joint_limits(robot, joint) for joint in joints], drawn_count
                ),
            }
            for name, joints, tip_link in chains
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--drawn",
        type=int,
        default=DRAWN_CONFIGURATIONS,
        help="configurations drawn inside the limits of each chain",
    )
    drawn_count = parser.parse_args().drawn
    for name, (urdf, chains) in ROBOTS.items():
        write(name, scenario(generator(name), urdf, chains, drawn_count))


if __name__ == "__main__":
    main()
