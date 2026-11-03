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
    python3 ros_reference/tools/draw_arm_planning.py --episodes <episodes> \
        --launcher <launchers-hub>/openarm/fragments/motion_planner.json5
    python3 ros_reference/tools/draw_arm_planning.py --drawn 200 --objective shortest \
        --planning-time 5

The second keeps the first 200 drawn scenes in place of those it picks, a large set for a
local run, which the README describes. The third writes, in place of the drawn scenes,
the scenes of episodes of OpenArm v2 recorded in simulation, each a subdirectory of
<episodes> in the format the README gives, in the map of the motion planner whose
launcher fragment `--launcher` names, also a large set for a local run:
motion_planning_core's example scenes_from_episodes turns each episode into a scene, and
the script prints the episodes it drops and why. Each plan optimizes `--objective`:
`any`, the planner RRTConnect, by default, as plan_joint_path names it, or `shortest`,
RRTstar, a mode the harness measures. Each plan ends on ITERATIONS, TIME_LIMIT far beyond
them, so it repeats under its seed. The fourth ends each plan on `--planning-time` alone,
as a robot's plan ends; that option and `--objective shortest` go with a drawn set or the
episodes' scenes.
"""

import json
import math
import subprocess
from collections import Counter
from pathlib import Path

from drawing import REPOSITORY, generator, joint_limits, openarm_joints, options, robot, write

NAME = "arm_planning_openarm_v2"
URDF = "openarm_description/assets/openarm_v20.urdf"
MESHES = "openarm_description/assets/meshes_v20"
VOXEL_SIZE = 0.04
MAP_HALF_SIDE = 1.6
OUTSIDE_LIMITS = 2
MIN_CLEARANCE = 0.0
CORNER_TOLERANCE = 0.1
GOAL_TOLERANCE = 0.01
OBJECTIVES = ("any", "shortest")
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


def example(name, *arguments):
    """The command that runs motion_planning_core's example `name` with `arguments`."""
    manifest = str(REPOSITORY / "motion_planning_core" / "Cargo.toml")
    return [
        "cargo",
        "run",
        "--release",
        "--quiet",
        "--manifest-path",
        manifest,
        "--example",
        name,
        "--",
        *arguments,
    ]


def run_example(command, given):
    """What `command` writes on stdout, as JSON, given `given` as JSON on stdin."""
    done = subprocess.run(
        command, input=json.dumps(given), stdout=subprocess.PIPE, text=True, check=True
    )
    return json.loads(done.stdout)


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


def plans(objective, planning_time):
    """The plans' objective and termination: ended on ITERATIONS, or with `planning_time`
    on that time alone."""
    if planning_time is None:
        return {"objective": objective, "termination": "iterations", "time_limit": TIME_LIMIT}
    return {"objective": objective, "termination": "time", "time_limit": planning_time}


def header(arm, voxel_map, planning):
    """The scenario's header: the robot, the arm `arm` (its base_link, tip_link and
    joints), the map `voxel_map` (its voxel_size and map_half_side) and the plans'
    parameters, `planning` (their objective and termination) among them."""
    return {
        "kind": "arm_planning",
        "urdf": URDF,
        "meshes": MESHES,
        **arm,
        **voxel_map,
        "min_clearance": MIN_CLEARANCE,
        "corner_tolerance": CORNER_TOLERANCE,
        "goal_tolerance": GOAL_TOLERANCE,
        **planning,
        "iterations": ITERATIONS,
    }


def planned(scenario_header, scenes):
    """The scenario of `scenes`, each with motion_planning_core's answer, and the spacing
    at which the reference probes a path, from the example plan_arm_scenes."""
    answers = run_example(example("plan_arm_scenes"), {**scenario_header, "scenes": scenes})
    answered = [
        {**scene, "ours": ours} for scene, ours in zip(scenes, answers["ours"], strict=True)
    ]
    return {**scenario_header, "probe_spacing": answers["probe_spacing"], "scenes": answered}


def drawn_scenario(drawn, planning):
    """The scenario of the drawn scenes: those it picks, or with `drawn` the first `drawn`,
    and the starts outside the limits, planned as `planning` says."""
    rng = generator(NAME)
    model = robot(URDF)
    limits = [joint_limits(model, joint) for joint in openarm_joints("left")]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    drawn_scenes = [drawn_scene(rng, limits) for _ in range(drawn or DRAWN)]
    left = {
        "base_link": "openarm_left_base_link",
        "tip_link": "openarm_left_tcp",
        "joints": openarm_joints("left"),
    }
    voxel_map = {"voxel_size": VOXEL_SIZE, "map_half_side": MAP_HALF_SIDE}
    scenario = planned(header(left, voxel_map, planning), [*drawn_scenes, *outside])
    answered = scenario["scenes"]
    kept_drawn = answered[: len(drawn_scenes)]
    if not drawn:
        most = max(range(DRAWN), key=lambda k: corners(kept_drawn[k]["ours"]))
        kept_drawn = [kept_drawn[k] for k in dict.fromkeys([*COMMITTED.values(), most])]
    return {**scenario, "scenes": [*kept_drawn, *answered[len(drawn_scenes) :]]}


def episodes_scenario(episodes, launcher, planning):
    """The scenario of the episodes under `episodes`, in the map of the motion planner of
    the launcher fragment `launcher`, planned as `planning` says; prints the episodes the
    example drops. The example draws each goal within TIME_LIMIT, so the scenes are the
    same under every `planning`."""
    command = example("scenes_from_episodes", str(episodes), str(launcher))
    converted = run_example(command, header({}, {}, plans("any", None)))
    scenes, dropped = converted["scenes"], converted["dropped"]
    print(f"{len(scenes)} scenes of {len(scenes) + len(dropped)} episodes")
    for reason, count in Counter(d["reason"] for d in dropped).items():
        print(f"dropped {count}: {reason}")
    for d in dropped:
        print(f"  {d['episode']}: {d['reason']}: {d['detail']}")
    return planned(header(converted["arm"], converted["map"], planning), scenes)


def main():
    parser = options(__doc__, 0, "drawn scenes kept in place of those it picks; 0 picks")
    parser.add_argument(
        "--episodes", type=Path, help="a directory of episodes, in the format the README gives"
    )
    parser.add_argument(
        "--launcher", type=Path, help="the launcher fragment of the motion planner, with --episodes"
    )
    parser.add_argument(
        "--objective",
        choices=OBJECTIVES,
        default="any",
        help="what each plan optimizes: any (RRTConnect) or shortest (RRTstar)",
    )
    parser.add_argument(
        "--planning-time",
        type=float,
        metavar="SECONDS",
        help="the time each plan ends on alone, as a robot's plan ends",
    )
    args = parser.parse_args()
    if (args.episodes is None) != (args.launcher is None):
        parser.error(
            "--episodes and --launcher go together: give the episodes and the launcher fragment "
            "of the motion planner whose map they fill"
        )
    if args.episodes is not None and args.drawn:
        parser.error("--episodes writes the episodes' scenes in place of drawn ones: drop --drawn")
    if args.planning_time is not None and not 0 < args.planning_time < math.inf:
        parser.error(f"--planning-time is {args.planning_time}; give the seconds, more than 0")
    committed = args.episodes is None and not args.drawn
    if committed and (args.objective != "any" or args.planning_time is not None):
        parser.error(
            "the committed scenario plans `any` on its iterations, the plans its scenes are "
            "picked by: give --drawn 200, or --episodes and --launcher, with --objective or "
            "--planning-time"
        )
    planning = plans(args.objective, args.planning_time)
    if args.episodes is None:
        write(NAME, drawn_scenario(args.drawn, planning))
        return
    write(NAME, episodes_scenario(args.episodes, args.launcher, planning))


if __name__ == "__main__":
    main()
