"""Writes the self distance scenario of the ROS reference harness: OpenArm v2 with both
arms and both grippers posed at the KEPT configurations of its candidates. A gripper is
posed by its first finger joint, which the second mimics. Run it from any directory:

    python3 ros_reference/tools/draw_self_distance.py
    python3 ros_reference/tools/draw_self_distance.py --drawn 500

The candidates are the zero configuration, the lower limits, the upper limits, and
DRAWN_CONFIGURATIONS configurations drawn uniformly inside the limits. KEPT holds the
fewest candidates that put each pair of links that FCL measures in each state that some
candidate puts it in, apart and in contact. The second command writes every candidate,
with the count of drawn ones it names, for a local run, which the README describes. To
choose KEPT again, write the candidates, run their references locally, and select from
the output:

    python3 ros_reference/tools/draw_self_distance.py --drawn 30
    ros_reference/run.sh
    python3 ros_reference/tools/draw_self_distance.py --select \\
        ros_reference/reference/self_distance_openarm_v2.json
"""

import json
import struct

from drawing import configurations, generator, joint_limits, openarm_joints, options, robot, write

NAME = "self_distance_openarm_v2"
URDF = "openarm_description/assets/openarm_v20.urdf"
MESHES = "openarm_description/assets/meshes_v20"
DRAWN_CONFIGURATIONS = 30
# The candidates the scenario keeps, by index: what --select prints for the output of
# every candidate.
KEPT = [0, 2, 10]


def candidates(drawn):
    """Each candidate configuration, with `drawn` drawn inside the limits, a position per
    joint name."""
    parsed = robot(URDF)
    joints = [
        joint
        for side in ["left", "right"]
        for joint in openarm_joints(side) + [f"openarm_{side}_finger_joint1"]
    ]
    limits = [joint_limits(parsed, joint) for joint in joints]
    poses = configurations(generator(NAME), limits, drawn)
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
            (pair, struct.unpack(">d", bytes.fromhex(distance[2:]))[0] > 0)
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
    parser = options(__doc__, None)
    parser.add_argument("--select", metavar="OUTPUT", help="print KEPT for this output")
    arguments = parser.parse_args()
    if arguments.select:
        print(select(arguments.select))
        return
    if arguments.drawn is None:
        every = candidates(DRAWN_CONFIGURATIONS)
        chosen = [every[index] for index in KEPT]
    else:
        chosen = candidates(arguments.drawn)
    write(
        NAME,
        {
            "kind": "self_distance",
            "urdf": URDF,
            "meshes": MESHES,
            "configurations": chosen,
        },
    )


if __name__ == "__main__":
    main()
