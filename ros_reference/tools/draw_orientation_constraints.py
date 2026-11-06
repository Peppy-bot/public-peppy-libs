"""Writes the orientation constraints scenario of the ROS reference harness.

OpenArm v2's left arm stands at a configuration drawn inside its limits, and its grasp link
holds an angle from a target that is a share of the target's tolerance, at each tolerance,
for each value of free_axis. A target with no free axis is the link's orientation turned by
that angle about a drawn axis. One with a free axis is the link's orientation turned by a
drawn angle about the free axis, then by that angle about a drawn axis square to it, so the
free axis stands exactly that angle from its direction in the target. Run it from any
directory:

    python3 ros_reference/tools/draw_orientation_constraints.py
    python3 ros_reference/tools/draw_orientation_constraints.py --drawn 500

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import (
    about_axis,
    chain,
    configuration,
    generator,
    joint_limits,
    link_pose,
    options,
    product,
    quaternion,
    robot,
    write,
)

NAME = "orientation_constraints_openarm_v2"
URDF = "openarm_description/assets/openarm_v20.urdf"
DRAWN_CASES = 1
# A tight tolerance and a wide one, which tells the sine of the free axis's bound from its
# angle.
TOLERANCES = [0.05, 0.5]
FREE_AXES = {"none": None, "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]}
# The angles from the target, as shares of the tolerance: inside what MoveIt accepts at the
# tolerance over the square root of 3 about each axis, on either side of the tolerance, and
# past what MoveIt accepts at the tolerance about each axis.
SHARES = [0.3, 0.7, 0.95, 1.05, 1.5, 3.0]


def drawn_axis(rng):
    """A unit axis drawn uniformly."""
    z = rng.uniform(-1.0, 1.0)
    turn = rng.uniform(0.0, 2.0 * math.pi)
    side = math.sqrt(1.0 - z * z)
    return [side * math.cos(turn), side * math.sin(turn), z]


def drawn_square_to(rng, free):
    """A unit axis drawn uniformly among those square to the unit axis `free`."""
    axis = drawn_axis(rng)
    along = sum(a * f for a, f in zip(axis, free))
    square = [a - along * f for a, f in zip(axis, free)]
    norm = math.sqrt(sum(s * s for s in square))
    return [s / norm for s in square]


def target_from(rng, turn, angle, free):
    """The rotation `angle` from the link's rotation `turn`, as the module says."""
    if free is None:
        return product(turn, about_axis(drawn_axis(rng), angle))
    twist = about_axis(free, rng.uniform(-math.pi, math.pi))
    return product(product(turn, twist), about_axis(drawn_square_to(rng, free), angle))


def main():
    parsed_arguments = options(__doc__, DRAWN_CASES, "cases per tolerance, axis and share").parse_args()
    rng = generator(NAME)
    parsed = robot(URDF)
    arm = chain("left")
    joints = arm["joints"]
    limits = [joint_limits(parsed, joint) for joint in joints]
    cases = []
    for tolerance in TOLERANCES:
        for free_axis, free in FREE_AXES.items():
            for share in SHARES:
                for _ in range(parsed_arguments.drawn):
                    positions = configuration(rng, limits)
                    turn, _at = link_pose(parsed, arm["tip_link"], dict(zip(joints, positions)))
                    cases.append(
                        {
                            "positions": positions,
                            "orientation": quaternion(
                                target_from(rng, turn, share * tolerance, free)
                            ),
                            "tolerance_rad": tolerance,
                            "free_axis": free_axis,
                        }
                    )
    write(
        NAME,
        {
            "kind": "orientation_constraints",
            "urdf": URDF,
            "joints": joints,
            "link_name": arm["tip_link"],
            "cases": cases,
        },
    )


if __name__ == "__main__":
    main()
