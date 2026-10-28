"""Writes the forward kinematics scenarios of the ROS reference harness.

The configurations of each chain are its zero configuration, its lower limits, its upper
limits, and configurations drawn uniformly inside its limits. Each scenario draws from a
generator seeded with its own name, so every run writes the same scenarios, and a change
to one scenario leaves the draws of the others as they are. Run it from any directory:

    python3 ros_reference/tools/draw_forward_kinematics.py
"""

import json
import random
import xml.etree.ElementTree as ElementTree
from pathlib import Path

SEED = 20261028
DRAWN_CONFIGURATIONS = 50
REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIOS = REPOSITORY / "ros_reference" / "scenarios"


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


def joint_limits(robot, joint):
    element = robot.find(f"joint[@name='{joint}']/limit")
    if element is None:
        raise SystemExit(f"the URDF gives no limits for {joint}")
    # A URDF limit without a lower or an upper bound bounds it at 0.
    return float(element.get("lower", 0)), float(element.get("upper", 0))


def configurations(rng, limits):
    lower = [low for low, _ in limits]
    upper = [high for _, high in limits]
    drawn = [[rng.uniform(low, high) for low, high in limits] for _ in range(DRAWN_CONFIGURATIONS)]
    return [[0.0] * len(limits), lower, upper, *drawn]


def scenario(rng, urdf, chains):
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
                    rng, [joint_limits(robot, joint) for joint in joints]
                ),
            }
            for name, joints, tip_link in chains
        ],
    }


def main():
    for name, (urdf, chains) in ROBOTS.items():
        rng = random.Random(f"{SEED}:{name}")
        path = SCENARIOS / f"{name}.json"
        path.write_text(json.dumps(scenario(rng, urdf, chains), indent=2) + "\n")
        print(f"wrote {path.relative_to(REPOSITORY)}")


if __name__ == "__main__":
    main()
