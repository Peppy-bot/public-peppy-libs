# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes a self distance scenario of the ROS reference harness: OpenArm v2 with both
arms and both grippers posed at the KEPT configurations of its candidates, or with
--robot so101, SO-101 with its arm and its jaw posed at SO101_KEPT. A gripper is posed by
its finger joint that moves of its own, which any other finger joint mimics. Run it from
any directory:

    uv run ros_reference/tools/draw_self_distance.py
    uv run ros_reference/tools/draw_self_distance.py --robot so101
    uv run ros_reference/tools/draw_self_distance.py --drawn 500

The candidates are the zero configuration, the lower limits, the upper limits, and
DRAWN_CONFIGURATIONS configurations drawn uniformly inside the limits. KEPT holds the
fewest candidates that put each pair of links that FCL measures in each state that some
candidate puts it in, apart and in contact. The third command writes every candidate,
with the count of drawn ones it names, for a local run, which the README describes. To
choose KEPT again, write the candidates, run their references locally, and select from
the output:

    uv run ros_reference/tools/draw_self_distance.py --drawn 30
    ros_reference/run.sh
    uv run ros_reference/tools/draw_self_distance.py --select \\
        ros_reference/reference/self_distance_openarm_v2.json

and the same with --robot so101 and its output, self_distance_so101.json.
"""

import json

from drawing import (
    CONFIGURATIONS_DRAWN,
    OPENARM_V2,
    SO101,
    arms,
    configurations,
    generator,
    gripper_joint,
    joint_limits,
    options,
    write,
)

NAME = "self_distance_openarm_v2"
DRAWN_CONFIGURATIONS = 30
# The candidates the scenario keeps, by index: what --select prints for the output of
# every candidate.
KEPT = [0, 2, 10]

SO101_NAME = "self_distance_so101"
SO101_KEPT = [0, 2, 12]


def scenario(robot_name):
    """The name, the model directory, the joints and the kept candidates of the scenario of
    `robot_name`: the joints of each arm of the robot and the finger joint of its gripper."""
    name, description, kept = {
        "so101": (SO101_NAME, SO101, SO101_KEPT),
        "openarm_v2": (NAME, OPENARM_V2, KEPT),
    }[robot_name]
    joints = [
        joint
        for arm in arms(description)
        for joint in [*arm.joints, gripper_joint(description, arm)]
    ]
    return name, description, joints, kept


def candidates(name, description, joints, drawn):
    """Each candidate configuration of the scenario `name` of the model directory
    `description`, with `drawn` drawn inside the limits, a position per joint of `joints`."""
    limits = [joint_limits(description, joint) for joint in joints]
    poses = configurations(generator(name), limits, drawn)
    return [dict(zip(joints, positions)) for positions in poses]


def select(output):
    """The fewest candidates, by index, that put each pair of the reference output of
    every candidate in each state it reaches: greedily, the candidate that adds the most
    pairs in a state not yet held, the lowest index first on a tie."""
    with open(output) as file:
        reference = json.load(file)
    pairs = [tuple(pair) for pair in reference["pairs"]]
    states = [
        {
            (pair, distance > 0)
            for pair, distance in zip(pairs, configuration["distances"])
        }
        for configuration in reference["configurations"]
    ]
    needed = set().union(*states)
    kept = []
    while needed:
        best = max(range(len(states)), key=lambda i: (len(states[i] & needed), -i))
        kept.append(best)
        needed -= states[best]
    return sorted(kept)


def main():
    parser = options(__doc__, None, CONFIGURATIONS_DRAWN)
    parser.add_argument("--select", metavar="OUTPUT", help="print KEPT for this output")
    parser.add_argument(
        "--robot",
        choices=("openarm_v2", "so101"),
        default="openarm_v2",
        help=f"so101 writes {SO101_NAME}, SO-101's arm and jaw",
    )
    arguments = parser.parse_args()
    if arguments.select:
        print(select(arguments.select))
        return
    name, description, joints, kept = scenario(arguments.robot)
    if arguments.drawn is None:
        every = candidates(name, description, joints, DRAWN_CONFIGURATIONS)
        chosen = [every[index] for index in kept]
    else:
        chosen = candidates(name, description, joints, arguments.drawn)
    write(
        name,
        {
            "kind": "self_distance",
            "description": description,
            "configurations": chosen,
        },
    )


if __name__ == "__main__":
    main()
