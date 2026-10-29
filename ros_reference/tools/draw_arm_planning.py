"""Writes the arm planning scenario of the ROS reference harness.

Scenes of OpenArm v2's left arm among boxes of voxels: OUTSIDE_LIMITS scenes start one
joint outside its limits, below its lower limit and above its upper one, for the refusal
of such a start; every other scene holds one to three boxes in front of the arm, a start
and a goal configuration drawn inside the joint limits, and a seed. A box is written as
the keys of its first and last voxel on each axis, the voxels whose centers the drawn
box holds. motion_planning_core's example plan_arm_scenes plans every scene and gives its
answer and the spacing at which the reference probes a path, which the script writes
into the scenario. Of the first DRAWN scenes, the scenario keeps those at COMMITTED, one
of each outcome the parity test asks for, and the one whose path has the most corners.
It runs cargo in motion_planning_core, so the host builds that crate. Run it from any
directory:

    python3 ros_reference/tools/draw_arm_planning.py
    python3 ros_reference/tools/draw_arm_planning.py --drawn 200

The second keeps the first 200 drawn scenes in place of those it picks, a large set for a
local run, which the README describes.
"""

import json
import math
import subprocess

from drawing import REPOSITORY, generator, joint_limits, openarm_joints, options, robot, write

NAME = "arm_planning_openarm_v2"
URDF = "openarm_description/assets/openarm_v20.urdf"
VOXEL_SIZE = 0.04
MAP_HALF_SIDE = 1.6
OUTSIDE_LIMITS = 2
MIN_CLEARANCE = 0.0
CORNER_TOLERANCE = 0.1
GOAL_TOLERANCE = 0.01
TIME_LIMIT = 600.0
ITERATIONS = 20000
# The drawn scenes the scenario picks from.
DRAWN = 200
# The drawn scenes the scenario keeps, by their place among the drawn scenes, each named
# by the outcome of the plans that it shows.
COMMITTED = {
    "a straight path": 0,
    "a path with corners": 39,
    "MoveIt's path invalid": 183,
    "a start in a box": 11,
    "a goal in a box": 38,
}
# Where the boxes stand: in front of the arm, between its hand's reach low and high.
BOX_CORNERS = [(0.15, 0.5), (-0.1, 0.4), (0.2, 0.7)]
BOX_SIZES = (0.04, 0.2)
EXAMPLE = [
    "cargo",
    "run",
    "--release",
    "--quiet",
    "--manifest-path",
    str(REPOSITORY / "motion_planning_core" / "Cargo.toml"),
    "--example",
    "plan_arm_scenes",
]


def voxel_box(rng):
    """A box drawn in meters, as the keys of the first and the last voxel on each axis
    whose center it holds."""
    low = [rng.uniform(low, high) for low, high in BOX_CORNERS]
    high = [corner + rng.uniform(*BOX_SIZES) for corner in low]
    first = [math.ceil(c / VOXEL_SIZE - 0.5) for c in low]
    last = [math.floor(c / VOXEL_SIZE - 0.5) for c in high]
    assert all(f <= l for f, l in zip(first, last)), "a box as wide as a voxel holds a center"
    return {"first": first, "last": last}


def configuration(rng, limits):
    return [rng.uniform(low, high) for low, high in limits]


def drawn_scene(rng, limits):
    return {
        "boxes": [voxel_box(rng) for _ in range(rng.randint(1, 3))],
        "start": configuration(rng, limits),
        "goal": configuration(rng, limits),
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def outside_limits(rng, limits, k):
    """A scene whose start lies 0.02 below the lower limit of joint k, for an even k, or
    above the upper one, for an odd k."""
    start = configuration(rng, limits)
    low, high = limits[k]
    start[k] = high + 0.02 if k % 2 else low - 0.02
    return {
        "boxes": [],
        "start": start,
        "goal": configuration(rng, limits),
        "seed": k + 1,
        "ours": None,
    }


def corners(ours):
    """The corners of our answer to a scene: its waypoints but the first and the last."""
    return max(len(ours.get("waypoints", [])) - 2, 0)


def main():
    drawn = options(__doc__, 0, "drawn scenes kept in place of those it picks; 0 picks")
    drawn = drawn.parse_args().drawn
    rng = generator(NAME)
    model = robot(URDF)
    limits = [joint_limits(model, joint) for joint in openarm_joints("left")]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    drawn_scenes = [drawn_scene(rng, limits) for _ in range(drawn or DRAWN)]
    header = {
        "kind": "arm_planning",
        "urdf": URDF,
        "meshes": "openarm_description/assets/meshes_v20",
        "base_link": "openarm_left_base_link",
        "tip_link": "openarm_left_tcp",
        "joints": openarm_joints("left"),
        "voxel_size": VOXEL_SIZE,
        "map_half_side": MAP_HALF_SIDE,
        "min_clearance": MIN_CLEARANCE,
        "corner_tolerance": CORNER_TOLERANCE,
        "goal_tolerance": GOAL_TOLERANCE,
        "time_limit": TIME_LIMIT,
        "iterations": ITERATIONS,
    }
    scenes = [*drawn_scenes, *outside]
    planned = subprocess.run(
        EXAMPLE,
        input=json.dumps({**header, "scenes": scenes}),
        stdout=subprocess.PIPE,
        text=True,
        check=True,
    )
    answers = json.loads(planned.stdout)
    answered = [
        {**scene, "ours": ours} for scene, ours in zip(scenes, answers["ours"], strict=True)
    ]
    kept_drawn = answered[: len(drawn_scenes)]
    if not drawn:
        most = max(range(DRAWN), key=lambda k: corners(kept_drawn[k]["ours"]))
        kept_drawn = [kept_drawn[k] for k in dict.fromkeys([*COMMITTED.values(), most])]
    kept = [*kept_drawn, *answered[len(drawn_scenes) :]]
    write(NAME, {**header, "probe_spacing": answers["probe_spacing"], "scenes": kept})


if __name__ == "__main__":
    main()
