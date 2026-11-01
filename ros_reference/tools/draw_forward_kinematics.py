"""Writes the forward kinematics scenarios of the ROS reference harness.

The configurations of each chain are its zero configuration, its lower limits, its upper
limits, and configurations drawn uniformly inside its limits, five of them by default.
Run it from any directory:

    python3 ros_reference/tools/draw_forward_kinematics.py
    python3 ros_reference/tools/draw_forward_kinematics.py --drawn 500

The second draws a large set for a local run, which the README describes.
"""

from drawing import (
    CONFIGURATIONS_DRAWN,
    configurations,
    generator,
    joint_limits,
    openarm_joints,
    options,
    robot,
    write,
)

DRAWN_CONFIGURATIONS = 5


def openarm_arm(side):
    return (side, openarm_joints(side), f"openarm_{side}_tcp")


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


def scenario(rng, urdf, chains, drawn):
    parsed = robot(urdf)
    return {
        "kind": "forward_kinematics",
        "urdf": urdf,
        "chains": [
            {
                "name": name,
                "joints": joints,
                "tip_link": tip_link,
                "configurations": configurations(
                    rng, [joint_limits(parsed, joint) for joint in joints], drawn
                ),
            }
            for name, joints, tip_link in chains
        ],
    }


def main():
    drawn = options(__doc__, DRAWN_CONFIGURATIONS, CONFIGURATIONS_DRAWN).parse_args().drawn
    for name, (urdf, chains) in ROBOTS.items():
        write(name, scenario(generator(name), urdf, chains, drawn))


if __name__ == "__main__":
    main()
