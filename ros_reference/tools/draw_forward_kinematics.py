# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the forward kinematics scenarios of the ROS reference harness.

The configurations of each arm of each robot of ROBOTS are its zero configuration, its
lower limits, its upper limits, and configurations drawn uniformly inside its limits, five
of them by default. Run it from any directory:

    uv run ros_reference/tools/draw_forward_kinematics.py
    uv run ros_reference/tools/draw_forward_kinematics.py --drawn 500

The second draws a large set for a local run, which the README describes.
"""

from drawing import (
    CONFIGURATIONS_DRAWN,
    OPENARM_V2,
    SO101,
    arms,
    configurations,
    generator,
    joint_limits,
    options,
    write,
)

DRAWN_CONFIGURATIONS = 5

# Each scenario, by its name: the model directory of the robot it poses.
ROBOTS = {
    "forward_kinematics_openarm_v2": OPENARM_V2,
    "forward_kinematics_so101": SO101,
}


def scenario(rng, description, drawn):
    return {
        "kind": "forward_kinematics",
        "description": description,
        "arms": [
            {
                "name": arm.name,
                "configurations": configurations(
                    rng, [joint_limits(description, joint) for joint in arm.joints], drawn
                ),
            }
            for arm in arms(description)
        ],
    }


def main():
    drawn = options(__doc__, DRAWN_CONFIGURATIONS, CONFIGURATIONS_DRAWN).parse_args().drawn
    for name, description in ROBOTS.items():
        write(name, scenario(generator(name), description, drawn))


if __name__ == "__main__":
    main()
