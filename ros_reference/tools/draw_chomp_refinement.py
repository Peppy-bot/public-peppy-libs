"""Writes the CHOMP refinement scenario of the ROS reference harness.

Scenes of OpenArm v2's left arm among one to three boxes of voxels, drawn as
draw_arm_planning.py draws its scenes among boxes, each with the input path that CHOMP refines:
the path motion_planning_core's RRTConnect plans to the scene's drawn goal under its seed, ended
on draw_arm_planning.py's ITERATIONS, and motion_planning_core's CHOMP refinement of it, the
same plan refined by `chomp`. The example plan_arm_scenes plans each scene and gives the
spacing at which the reference probes a path. A scene holds its start, its path, our
refinement and its seed, which seeds CHOMP's draws too; a drawn scene that motion_planning_core
plans no path of is dropped. Of the first DRAWN scenes, the scenario keeps the first DETOURS
whose path turns at a waypoint and the first whose path is straight. It runs cargo in
motion_planning_core, so the host builds that crate. Run it from any directory:

    python3 ros_reference/tools/draw_chomp_refinement.py
    python3 ros_reference/tools/draw_chomp_refinement.py --drawn 200 --min-clearance 0.04

The second keeps every scene with a path of the first 200 drawn, in place of those it picks, a
large set for a local run, which the README describes, planned at a clearance of 4 cm.
`--min-clearance` sets the clearance, draw_arm_planning.py's MIN_CLEARANCE by default: the
min_clearance_m of motion_planning_core's plans, and the padding of each link of MoveIt's robot
against the octomap and the objects in the reference's probes; it goes with a drawn set.
"""

import math

from draw_arm_planning import (
    CORNER_TOLERANCE,
    GOAL_TOLERANCE,
    ITERATIONS,
    MESHES,
    MIN_CLEARANCE,
    TIME_LIMIT,
    URDF,
    VOXEL_MAP,
    corners,
    drawn_scene,
    header,
    planned,
    plans,
)
from drawing import chain, generator, joint_limits, openarm_joints, options, robot, write

NAME = "chomp_refinement_openarm_v2"
# The drawn scenes the scenario picks from, and the scenes it keeps of them whose path turns.
DRAWN = 40
DETOURS = 4


def refinement_scene(scene, refined):
    """The scene of `scene`, which plan_arm_scenes answered with a path, with that path, its
    input path, and `refined`, plan_arm_scenes's answer to it refined by `chomp`, our
    refinement."""
    return {
        "boxes": scene["boxes"],
        "objects": scene["objects"],
        "attached": scene["attached"],
        "allowed": scene["allowed"],
        "start": scene["start"],
        "goal": scene["goal"],
        "path": scene["ours"]["waypoints"],
        "refined": refined["ours"]["waypoints"],
        "seed": scene["seed"],
    }


def picked(scenes):
    """Of `scenes`, which plan_arm_scenes answered, in their order: the first DETOURS whose
    path turns at a waypoint and the first whose path is straight."""
    detours = [k for k, scene in enumerate(scenes) if corners(scene["ours"]) > 0][:DETOURS]
    straight = [k for k, scene in enumerate(scenes) if corners(scene["ours"]) == 0][:1]
    if len(detours) < DETOURS or not straight:
        raise SystemExit(
            f"the first {DRAWN} drawn scenes hold {len(detours)} paths that turn and "
            f"{len(straight)} straight; raise DRAWN"
        )
    return [scenes[k] for k in sorted([*detours, *straight])]


def refinement_scenario(drawn, min_clearance):
    """The scenario of the drawn scenes with a path of motion_planning_core, planned at
    `min_clearance`: those it picks, or with `drawn` every one of the first `drawn`."""
    rng = generator(NAME)
    model = robot(URDF)
    limits = [joint_limits(model, joint) for joint in openarm_joints("left")]
    arms = {"chains": [chain("left")]}
    scenes = [drawn_scene(rng, limits) for _ in range(drawn or DRAWN)]
    planning = plans("rrt_connect", "none", None)
    answered = planned(header(arms, VOXEL_MAP, min_clearance, planning), scenes)
    with_path = [scene for scene in answered["scenes"] if "waypoints" in scene["ours"]]
    print(f"{len(with_path)} of {len(scenes)} drawn scenes with a path of ours")
    kept = with_path if drawn else picked(with_path)
    refining = plans("rrt_connect", "chomp", None)
    refined = planned(
        header(arms, VOXEL_MAP, min_clearance, refining),
        [{**scene, "ours": None} for scene in kept],
    )
    return {
        "kind": "chomp_refinement",
        "urdf": URDF,
        "meshes": MESHES,
        **arms,
        **VOXEL_MAP,
        "min_clearance": min_clearance,
        "corner_tolerance": CORNER_TOLERANCE,
        "goal_tolerance": GOAL_TOLERANCE,
        "iterations": ITERATIONS,
        "time_limit": TIME_LIMIT,
        "probe_spacing": answered["probe_spacing"],
        "scenes": [
            refinement_scene(scene, ours)
            for scene, ours in zip(kept, refined["scenes"], strict=True)
        ],
    }


def main():
    parser = options(__doc__, 0, "drawn scenes whose paths are kept in place of those it picks")
    parser.add_argument(
        "--min-clearance",
        type=float,
        default=MIN_CLEARANCE,
        metavar="METERS",
        help="the clearance of the input paths, min_clearance_m, and MoveIt's link padding",
    )
    args = parser.parse_args()
    if not 0 <= args.min_clearance < math.inf:
        parser.error(f"--min-clearance is {args.min_clearance}; give the meters, 0 or more")
    if not args.drawn and args.min_clearance != MIN_CLEARANCE:
        parser.error(
            f"the committed scenario plans at a clearance of {MIN_CLEARANCE}, the plans its "
            f"scenes are picked by: give --drawn 200 with --min-clearance"
        )
    write(NAME, refinement_scenario(args.drawn, args.min_clearance))


if __name__ == "__main__":
    main()
