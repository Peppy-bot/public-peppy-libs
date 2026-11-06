"""Writes the arm planning scenarios of the ROS reference harness.

Scenes of OpenArm v2's left arm among boxes of voxels: OUTSIDE_LIMITS scenes start one
joint outside its limits, below its lower limit and above its upper one, for the refusal
of such a start; every other scene holds one to three boxes in front of the arm, a start
and a goal configuration drawn inside the joint limits, and a seed. A box is written as
the keys of its first and last voxel on each axis, the voxels whose centers the drawn
box holds. motion_planning_core's example plan_arm_scenes plans every scene and gives its
answer, the pairs of the robot's own links its plans check and the spacing at which the
reference probes a path, which the script writes into the scenario. Of the first DRAWN
scenes, the scenario keeps those at COMMITTED, one of each outcome the parity test asks
for, and the one whose path has the most corners.
It runs cargo in motion_planning_core, so the host builds that crate. With --objects it
writes OBJECTS_NAME, scenes among one to three collision objects in place of the boxes,
each one of OBJECT_KINDS turned any way, the hand holding a cube or a cylinder past its
grasp point in every other scene, the gripper its touch links, the other hand holding a
cube in every third, and in every eighth the held object set down on a table at the
goal, the table allowed to touch the object and the gripper; the scenario keeps the
drawn scenes at COMMITTED_OBJECTS. With --poses it writes POSES_NAME, scenes among one to
three boxes whose goal is a pose of the grasp point, which MoveIt reaches through KDL's
inverse kinematics: a short move from the start's grasp pose in every other scene, and the
grasp pose of a configuration drawn inside the limits in the rest; the scenario keeps the
drawn scenes at COMMITTED_POSES.

With --both-arms it writes BOTH_NAME, with --objects BOTH_OBJECTS_NAME, or with --poses
BOTH_POSES_NAME, scenes of both arms planned as one group, each start and goal of the
drawn scenes one configuration of each arm, which the example self_distances, this
crate's self model of a plan of both arms, holds apart, the goal by at least
min_clearance_m: boxes stand in front of either
arm, mirrored for the right one; among objects, the left hand holds an object in every
other scene, the right hand a cube in every third, and in every eighth the left hand sets
its object down on a table; to poses, each arm's goal is a pose of its grasp point, short
moves in every other scene and the grasp poses of the drawn goal in the rest. Two scenes,
one starting and one ending with the robot's own bodies meeting, take them more than
MEETING_DEPTH into each other in the self model. The scenario keeps the drawn scenes at
COMMITTED_BOTH, COMMITTED_BOTH_OBJECTS or COMMITTED_BOTH_POSES. Run it from any
directory:

    python3 ros_reference/tools/draw_arm_planning.py
    python3 ros_reference/tools/draw_arm_planning.py --drawn 200
    python3 ros_reference/tools/draw_arm_planning.py --episodes <episodes> \
        --launcher <launchers-hub>/openarm/fragments/motion_planner.json5
    python3 ros_reference/tools/draw_arm_planning.py --drawn 200 --objective shortest \
        --planning-time 5
    python3 ros_reference/tools/draw_arm_planning.py --objects --drawn 200 --planning-time 1 \
        --min-clearance 0.04
    python3 ros_reference/tools/draw_arm_planning.py --poses --drawn 200 --planning-time 5
    python3 ros_reference/tools/draw_arm_planning.py --both-arms --objects

The second keeps the first 200 drawn scenes in place of those it picks, a large set for a
local run, which the README describes. The third writes, in place of the drawn scenes,
the scenes of episodes of OpenArm v2 recorded in simulation, each a subdirectory of
<episodes> in the format the README gives, in the map of the motion planner whose
launcher fragment `--launcher` names, also a large set for a local run:
motion_planning_core's example scenes_from_episodes turns each episode into a scene, and
the script prints the episodes it drops and why. Each plan optimizes `--objective`:
`any`, the planner RRTConnect, by default, as plan_joint_path names it, or `shortest`,
RRTstar, a mode the harness measures. Each plan ends on ITERATIONS, TIME_LIMIT far beyond
them, so it repeats under its seed. The fourth ends MoveIt's plans on `--planning-time`
alone and ours on ITERATIONS or that time, whichever comes first, as the robot runs (its
motion planner's launcher fragment bounds a plan by max_iterations); that option and
`--objective shortest` go with a drawn set or the episodes' scenes. The fifth draws 200
scenes among collision objects, planned as the fourth plans them at 1 s, at a clearance
of 4 cm, and the sixth 200 scenes whose goal is a pose, planned so at 5 s; `--objects` and
`--poses` each go with every option but `--episodes` and the other. The seventh writes the
committed scenario of both arms among collision objects; `--both-arms` goes with every
option but `--episodes`, whose scenes plan the arm their episodes record.
`--min-clearance` sets the clearance of the plans, MIN_CLEARANCE by default: the
min_clearance_m of motion_planning_core's plans, and the padding of each link of MoveIt's
robot against the octomap and the objects, which meets itself unpadded; it goes with a
drawn set or the episodes' scenes.
"""

import math
from collections import Counter
from pathlib import Path

from drawing import (
    IDENTITY,
    about_axis,
    box_mesh,
    chain,
    configuration,
    example,
    generator,
    grasp_link,
    joint_limits,
    l_mesh,
    link_pose,
    openarm_joints,
    options,
    pose_by_link,
    product,
    quaternion,
    robot,
    run_example,
    shape,
    unit_quaternion,
    write,
)

NAME = "arm_planning_openarm_v2"
# The scenario of scenes among collision objects, written with --objects.
OBJECTS_NAME = "arm_planning_objects_openarm_v2"
# The scenario of scenes whose goal is a pose of the grasp point, written with --poses.
POSES_NAME = "arm_planning_poses_openarm_v2"
# The scenarios of both arms, among boxes and among collision objects, written with
# --both-arms.
BOTH_NAME = "arm_planning_both_arms_openarm_v2"
BOTH_OBJECTS_NAME = "arm_planning_both_arms_objects_openarm_v2"
# The scenario of both arms whose goal is a pose of each grasp point, written with
# --both-arms --poses.
BOTH_POSES_NAME = "arm_planning_both_arms_poses_openarm_v2"
# The arms, in the order a scene of both arms holds their joints.
SIDES = ("left", "right")
URDF = "openarm_description/assets/openarm_v20.urdf"
MESHES = "openarm_description/assets/meshes_v20"
VOXEL_SIZE = 0.04
MAP_HALF_SIDE = 1.6
VOXEL_MAP = {"voxel_size": VOXEL_SIZE, "map_half_side": MAP_HALF_SIDE}
OUTSIDE_LIMITS = 2
# The clearance of the committed scenarios' plans: MoveIt's links unpadded, so the parity
# compares its verdicts at contact.
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
# The drawn scenes of the objects scenario it keeps, as COMMITTED names them.
COMMITTED_OBJECTS = {
    "a straight path": 0,
    "a path with corners": 4,
    "MoveIt's path invalid": 8,
    "a start in an object": 10,
    "a goal in an object": 23,
}
# The short pose moves: the goal's grasp point at most 10 cm from the start's, a sixth of
# the arm's 0.6 m reach from its shoulder (openarm_left_link2's origin) to its grasp point
# with the arm straight, and turned at most 0.3 rad, which turns the grasp point about 5
# cm, half that, about the wrist 16 cm from it (openarm_left_link6's origin). Each pose
# goal holds within 1 mm and 0.01 rad, the defaults of MoveIt's CartesianPrecision.
SHORT_REACH_M = 0.1
SHORT_TURN_RAD = 0.3
POSE_TOLERANCE_M = 0.001
POSE_TOLERANCE_RAD = 0.01
# The drawn scenes of the pose scenario it keeps, as COMMITTED names them.
COMMITTED_POSES = {
    "a straight path": 0,
    "a path with corners": 9,
    "MoveIt's path invalid": 103,
    "a start in a box": 17,
    "a goal in a box": 2,
}
# The kinds of a drawn object: a box, a sphere, a cylinder, a closed box mesh, an L-shaped
# closed mesh, and an object of two shapes, a box with a cylinder standing on it.
OBJECT_KINDS = ("box", "sphere", "cylinder", "mesh", "L", "two shapes")
# The object the hand holds in every other scene of the objects scenario, a cube and a
# cylinder in turn, and the one the other hand holds in every third: 3 cm across, under
# the 3.6 cm between the finger joints' origins (the URDF's finger joints, 1.8 cm either
# side of the hand's axis), centered 4 cm along the grasp point's z axis, past its half
# side, so it lies out of the gripper, which it may touch.
HELD_SIDE = 0.03
HELD_AHEAD = 0.04
# The table a held object is set down on in every eighth scene: 20 cm square and 2 cm
# thick, under the object where the goal holds it, which reaches 5 mm into it, so it
# rests on the table; the scene allows the object and the gripper to touch the table, as
# a caller setting an object down allows them.
SET_DOWN_TABLE = [0.2, 0.2, 0.02]
SET_DOWN_REACH = 0.005
# The drawn scenes of both arms the scenarios keep, as COMMITTED names them, and the one
# whose straight joint line runs through the robot's own body, which both plan around.
COMMITTED_BOTH = {
    "a straight path": 0,
    "a path with corners": 6,
    "MoveIt's path invalid": 66,
    "a start in a box": 14,
    "a goal in a box": 50,
    "a straight joint line through the robot's own body": 36,
}
# Among objects, the scenes of COMMITTED_BOTH's outcomes, and a path with both hands
# holding an object and one setting the left hand's object down on a table.
COMMITTED_BOTH_OBJECTS = {
    "a straight path": 0,
    "a path with corners": 16,
    "MoveIt's path invalid": 138,
    "a start in an object": 11,
    "a goal in an object": 14,
    "a straight joint line through the robot's own body": 58,
    "a path with both hands holding": 3,
    "a path setting a held object down on a table": 7,
}
# To poses, the scenes of COMMITTED_BOTH's outcomes but MoveIt's path invalid, which none
# of the first DRAWN gives at the committed settings, and the straight joint line, which a
# goal of poses has none of.
COMMITTED_BOTH_POSES = {
    "a straight path": 1,
    "a path with corners": 10,
    "a start in a box": 13,
    "a goal in a box": 0,
}
# How deep the two scenes of both arms whose start, and whose goal, holds the robot's own
# bodies meeting take them into each other in the self model, whose hulls enclose the meshes
# MoveIt checks, the other end of each apart.
MEETING_DEPTH = 0.02
# The start and goal pairs of both arms drawn at once for the self model to measure.
PAIRS_BATCH = 200
EMPTY_BATCHES = 10


def gripper(side):
    """The links of the gripper of the arm on `side`, which a held object may touch."""
    return [f"openarm_{side}_ee_base_link", *(f"openarm_{side}_ee_link{k}" for k in (1, 2))]


def voxel_box(rng, side="left"):
    """A box drawn in meters in front of the arm on `side`, as the keys of the first and the
    last voxel on each axis whose center it holds."""
    low = [rng.uniform(low, high) for low, high in BOX_CORNERS]
    high = [corner + rng.uniform(*BOX_SIZES) for corner in low]
    if side == "right":
        low[1], high[1] = -high[1], -low[1]
    first = [math.ceil(c / VOXEL_SIZE - 0.5) for c in low]
    last = [math.floor(c / VOXEL_SIZE - 0.5) for c in high]
    assert all(f <= l for f, l in zip(first, last)), "a box as wide as a voxel holds a center"
    return {"first": first, "last": last}


def drawn_scene(rng, limits):
    return {
        "boxes": [voxel_box(rng) for _ in range(rng.randint(1, 3))],
        "objects": [],
        "attached": [],
        "allowed": [],
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
    return plain_scene(start, configuration(rng, limits), k + 1)


def plain_scene(start, goal, seed):
    """The scene from `start` to `goal` under `seed`, among no box and no object."""
    return {
        "boxes": [],
        "objects": [],
        "attached": [],
        "allowed": [],
        "start": start,
        "goal": goal,
        "seed": seed,
        "ours": None,
    }


def drawn_object(rng, k, side="left"):
    """The object `object_<k>`: one of OBJECT_KINDS, its sizes from BOX_SIZES, turned any
    way, its origin where a box's corner stands in front of the arm on `side`."""
    size = lambda: rng.uniform(*BOX_SIZES)
    kind = rng.choice(OBJECT_KINDS)
    if kind == "box":
        shapes = [shape("box", [size(), size(), size()])]
    elif kind == "sphere":
        shapes = [shape("sphere", [size() / 2])]
    elif kind == "cylinder":
        shapes = [shape("cylinder", [size(), size() / 2])]
    elif kind == "mesh":
        shapes = [box_mesh([size(), size(), size()], (0.0, 0.0, 0.0))]
    elif kind == "L":
        side = size()
        shapes = [l_mesh(side, rng.uniform(BOX_SIZES[0] / 2, side / 2), size())]
    else:
        base = [size(), size(), size() / 2]
        height, radius = size(), size() / 4
        shapes = [
            shape("box", base),
            shape("cylinder", [height, radius], at=(0.0, 0.0, (base[2] + height) / 2)),
        ]
    position = [rng.uniform(low, high) for low, high in BOX_CORNERS]
    if side == "right":
        position[1] = -position[1]
    return {
        "id": f"object_{k}",
        "operation": "add",
        "position": position,
        "orientation": unit_quaternion(rng),
        "shapes": shapes,
    }


def held(object_id, k, side):
    """The object `object_id` the hand on `side` holds, in the frame of its grasp point, a
    cube or, for an odd `k`, a cylinder, its gripper the links it may touch."""
    held_shape = (
        shape("cylinder", [HELD_SIDE, HELD_SIDE / 2]) if k % 2 else shape("box", [HELD_SIDE] * 3)
    )
    return {
        "object": {
            "id": object_id,
            "operation": "add",
            "position": [0.0, 0.0, HELD_AHEAD],
            "orientation": IDENTITY,
            "shapes": [held_shape],
        },
        "link": grasp_link(side),
        "touch_links": gripper(side),
    }


def set_down_table(model, goal):
    """The table the object the left hand holds is set down on at the goal configuration
    `goal`, which holds the left arm's joints first: under the object's center, its top
    SET_DOWN_REACH above the object's lowest point, which a cube or a cylinder of HELD_SIDE
    turned any way puts at least half of HELD_SIDE below its center."""
    joints = dict(zip(openarm_joints("left"), goal))
    center = pose_by_link(model, grasp_link("left"), joints, [0.0, 0.0, HELD_AHEAD])["position"]
    top = center[2] - HELD_SIDE / 2 + SET_DOWN_REACH
    return {
        "id": "table",
        "operation": "add",
        "position": [center[0], center[1], top - SET_DOWN_TABLE[2] / 2],
        "orientation": IDENTITY,
        "shapes": [shape("box", SET_DOWN_TABLE)],
    }


def object_scene(rng, model, limits, k):
    """A scene among one to three drawn objects, from the `k`th: the hand holding a cube
    or a cylinder in every other scene, the other hand holding a cube in every third, and
    in every eighth the held object set down on a table at the goal."""
    objects = [drawn_object(rng, i) for i in range(rng.randint(1, 3))]
    start, goal = configuration(rng, limits), configuration(rng, limits)
    holding = k % 2 == 1
    allowed = []
    if holding and k % 8 == 7:
        objects.append(set_down_table(model, goal))
        allowed = [["held", "table"], *[["table", link] for link in gripper("left")]]
    attached = [held("held", k // 2, "left")] if holding else []
    if k % 3 == 0:
        attached.append(held("other", 0, "right"))
    return {
        "boxes": [],
        "objects": objects,
        "attached": attached,
        "allowed": allowed,
        "start": start,
        "goal": goal,
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def grasp_pose_goal(rng, model, side, start, goal):
    """The pose goal of the grasp point of the arm on `side`, whose joints stand at `start`:
    with no `goal`, a short move, the start's grasp pose moved within SHORT_REACH_M and
    turned within SHORT_TURN_RAD, and else the grasp pose of the arm's joints `goal`; within
    POSE_TOLERANCE_M and POSE_TOLERANCE_RAD."""
    joints = openarm_joints(side)
    if goal is not None:
        pose = pose_by_link(model, grasp_link(side), dict(zip(joints, goal)), [0, 0, 0])
    else:
        turn, at = link_pose(model, grasp_link(side), dict(zip(joints, start)))
        direction = [rng.gauss(0, 1) for _ in range(3)]
        norm = math.sqrt(sum(d * d for d in direction))
        reach = SHORT_REACH_M * rng.random() ** (1 / 3)
        axis = [rng.gauss(0, 1) for _ in range(3)]
        pose = {
            "position": [a + reach * d / norm for a, d in zip(at, direction)],
            "orientation": quaternion(
                product(about_axis(axis, SHORT_TURN_RAD * rng.random()), turn)
            ),
        }
    return {
        **pose,
        "position_tolerance": POSE_TOLERANCE_M,
        "orientation_tolerance": POSE_TOLERANCE_RAD,
    }


def pose_scene(rng, model, limits, k):
    """A scene among one to three boxes whose goal is a pose of the grasp point: for an even
    `k` a short move, and for an odd `k` the grasp pose of a configuration drawn inside the
    limits (`grasp_pose_goal`)."""
    scene = drawn_scene(rng, limits)
    goal = configuration(rng, limits) if k % 2 else None
    return {**scene, "goal": [grasp_pose_goal(rng, model, "left", scene["start"], goal)]}


def corners(ours):
    """The corners of our answer to a scene: its waypoints but the first and the last."""
    return max(len(ours.get("waypoints", [])) - 2, 0)


def plans(objective, planning_time):
    """The plans' objective and termination: ended on ITERATIONS, or with `planning_time`,
    MoveIt's on that time alone and ours on ITERATIONS or that time, whichever comes
    first."""
    if planning_time is None:
        return {"objective": objective, "termination": "iterations", "time_limit": TIME_LIMIT}
    return {"objective": objective, "termination": "time", "time_limit": planning_time}


def header(arms, voxel_map, min_clearance, planning):
    """The scenario's header: the robot, the arms `arms` (their chains), the map
    `voxel_map` (its voxel_size and map_half_side), the plans' clearance `min_clearance`,
    and the plans' parameters, `planning` (their objective and termination) among them."""
    return {
        "kind": "arm_planning",
        "urdf": URDF,
        "meshes": MESHES,
        **arms,
        **voxel_map,
        "min_clearance": min_clearance,
        "corner_tolerance": CORNER_TOLERANCE,
        "goal_tolerance": GOAL_TOLERANCE,
        **planning,
        "iterations": ITERATIONS,
    }


def planned(scenario_header, scenes):
    """The scenario of `scenes`, each with motion_planning_core's answer, the pairs of the
    robot's own links its plans check, after the chains, and the spacing at which the
    reference probes a path, from the example plan_arm_scenes."""
    answers = run_example(example("plan_arm_scenes"), {**scenario_header, "scenes": scenes})
    answered = [
        {**scene, "ours": ours} for scene, ours in zip(scenes, answers["ours"], strict=True)
    ]
    checked = {}
    for key, value in scenario_header.items():
        checked[key] = value
        if key == "chains":
            checked["checked_pairs"] = answers["checked_pairs"]
    return {**checked, "probe_spacing": answers["probe_spacing"], "scenes": answered}


def drawn_scenario(drawn, planning, min_clearance, name):
    """The scenario `name` of the drawn scenes, among boxes of voxels (NAME), among
    collision objects (OBJECTS_NAME) or among boxes to a pose (POSES_NAME): those it picks,
    or with `drawn` the first `drawn`, and the starts outside the limits, planned as
    `planning` says at `min_clearance`."""
    rng = generator(name)
    model = robot(URDF)
    limits = [joint_limits(model, joint) for joint in openarm_joints("left")]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    scene_of, committed = {
        NAME: (lambda k: drawn_scene(rng, limits), COMMITTED),
        OBJECTS_NAME: (lambda k: object_scene(rng, model, limits, k), COMMITTED_OBJECTS),
        POSES_NAME: (lambda k: pose_scene(rng, model, limits, k), COMMITTED_POSES),
    }[name]
    drawn_scenes = [scene_of(k) for k in range(drawn or DRAWN)]
    scenario_header = header({"chains": [chain("left")]}, VOXEL_MAP, min_clearance, planning)
    return kept(planned(scenario_header, [*drawn_scenes, *outside]), drawn, committed)


def kept(scenario, drawn, committed):
    """`scenario`, its drawn scenes first and every other scene after them, keeping of
    those drawn the first `drawn`, or with no `drawn` those at `committed` and the one whose
    path has the most corners among the first DRAWN."""
    answered = scenario["scenes"]
    count = drawn or DRAWN
    kept_drawn = answered[:count]
    if not drawn:
        most = max(range(DRAWN), key=lambda k: corners(kept_drawn[k]["ours"]))
        kept_drawn = [kept_drawn[k] for k in dict.fromkeys([*committed.values(), most])]
    return {**scenario, "scenes": [*kept_drawn, *answered[count:]]}


def self_distances(scenario_header, pairs):
    """The self model's least distance between the robot's own bodies at the start and at
    the goal of each of `pairs`, and along the straight joint line between them, from the
    example self_distances."""
    scenes = [plain_scene(start, goal, 1) for start, goal in pairs]
    return run_example(example("self_distances"), {**scenario_header, "scenes": scenes})


def both_arms_pairs(rng, scenario_header, limits, count):
    """The first `count` starts and goals of both arms drawn inside `limits`, in batches of
    PAIRS_BATCH, whose start the self model holds apart and whose goal it holds at least
    min_clearance_m apart, a goal nearer than that being one this crate's plans refuse and
    MoveIt's, which check the robot's own bodies unpadded, reach; and the scenes of the first
    drawn whose start holds the arms meeting, by more than MEETING_DEPTH, its goal apart, and
    of the first whose goal holds them so, its start apart. Stops the script when
    EMPTY_BATCHES batches hold no such pair."""
    apart, start_meeting, goal_meeting = [], None, None
    clearance = scenario_header["min_clearance"]
    batches = 0
    while len(apart) < count or start_meeting is None or goal_meeting is None:
        batches += 1
        if batches > EMPTY_BATCHES and not apart:
            raise SystemExit(
                f"--min-clearance {clearance}: no drawn goal of both arms keeps it in the self "
                f"model; give less"
            )
        batch = [
            (configuration(rng, limits), configuration(rng, limits)) for _ in range(PAIRS_BATCH)
        ]
        for pair, measured in zip(batch, self_distances(scenario_header, batch), strict=True):
            start, goal = measured["start"], measured["goal"]
            if start > 0 and goal > clearance:
                apart.append(pair)
            elif start_meeting is None and start < -MEETING_DEPTH and goal > 0:
                start_meeting = plain_scene(*pair, 1)
            elif goal_meeting is None and goal < -MEETING_DEPTH and start > 0:
                goal_meeting = plain_scene(*pair, 2)
    return apart[:count], [start_meeting, goal_meeting]


def both_arms_scene(rng, model, k, start, goal, among_objects):
    """The `k`th drawn scene of both arms from `start` to `goal`: among one to three boxes,
    each in front of either arm, or `among_objects`, among one to three drawn objects, the
    left hand holding a cube or a cylinder in every other scene, the right hand a cube in
    every third, and in every eighth the left hand's object set down on a table at the
    goal."""
    scene = plain_scene(start, goal, None)
    if not among_objects:
        boxes = [voxel_box(rng, rng.choice(SIDES)) for _ in range(rng.randint(1, 3))]
        return {**scene, "boxes": boxes, "seed": rng.randint(1, 2**31 - 1)}
    objects = [drawn_object(rng, i, rng.choice(SIDES)) for i in range(rng.randint(1, 3))]
    holding = k % 2 == 1
    allowed = []
    if holding and k % 8 == 7:
        objects.append(set_down_table(model, goal))
        allowed = [["held", "table"], *[["table", link] for link in gripper("left")]]
    attached = [held("held", k // 2, "left")] if holding else []
    if k % 3 == 0:
        attached.append(held("other", 0, "right"))
    return {
        **scene,
        "objects": objects,
        "attached": attached,
        "allowed": allowed,
        "seed": rng.randint(1, 2**31 - 1),
    }


def both_arms_pose_scene(rng, model, k, start, goal):
    """The `k`th drawn scene of both arms among one to three boxes, each in front of either
    arm, whose goal is a pose of each arm's grasp point: for an even `k` each a short move,
    and for an odd `k` the grasp pose of each arm's joints of `goal` (`grasp_pose_goal`)."""
    scene = both_arms_scene(rng, model, k, start, goal, False)
    joints = len(openarm_joints("left"))
    arms = zip(SIDES, (start[:joints], start[joints:]), (goal[:joints], goal[joints:]))
    poses = [
        grasp_pose_goal(rng, model, side, arm_start, arm_goal if k % 2 else None)
        for side, arm_start, arm_goal in arms
    ]
    return {**scene, "goal": poses}


def both_arms_scenario(drawn, planning, min_clearance, name):
    """The scenario `name` of the drawn scenes of both arms, among boxes of voxels
    (BOTH_NAME), among collision objects (BOTH_OBJECTS_NAME) or among boxes to a pose of each
    grasp point (BOTH_POSES_NAME): those it picks, or with `drawn` the first `drawn`, the
    scenes whose start and whose goal hold the robot's own bodies meeting, and the starts
    outside the limits, planned as `planning` says at `min_clearance`."""
    rng = generator(name)
    model = robot(URDF)
    limits = [joint_limits(model, joint) for side in SIDES for joint in openarm_joints(side)]
    chains = {"chains": [chain(side) for side in SIDES]}
    scenario_header = header(chains, VOXEL_MAP, min_clearance, planning)
    pairs, meeting = both_arms_pairs(rng, scenario_header, limits, drawn or DRAWN)
    scene_of, committed = {
        BOTH_NAME: (
            lambda k, start, goal: both_arms_scene(rng, model, k, start, goal, False),
            COMMITTED_BOTH,
        ),
        BOTH_OBJECTS_NAME: (
            lambda k, start, goal: both_arms_scene(rng, model, k, start, goal, True),
            COMMITTED_BOTH_OBJECTS,
        ),
        BOTH_POSES_NAME: (
            lambda k, start, goal: both_arms_pose_scene(rng, model, k, start, goal),
            COMMITTED_BOTH_POSES,
        ),
    }[name]
    drawn_scenes = [scene_of(k, start, goal) for k, (start, goal) in enumerate(pairs)]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    scenario = planned(scenario_header, [*drawn_scenes, *meeting, *outside])
    return kept(scenario, drawn, committed)


def episodes_scenario(episodes, launcher, planning, min_clearance):
    """The scenario of the episodes under `episodes`, in the map of the motion planner of
    the launcher fragment `launcher`, planned as `planning` says at `min_clearance`;
    prints the episodes the example drops. The example draws each goal within
    TIME_LIMIT, at the plans' clearance, so the scenes are the same under every objective
    and termination."""
    command = example("scenes_from_episodes", str(episodes), str(launcher))
    converted = run_example(command, header({}, {}, min_clearance, plans("any", None)))
    scenes, dropped = converted["scenes"], converted["dropped"]
    print(f"{len(scenes)} scenes of {len(scenes) + len(dropped)} episodes")
    for reason, count in Counter(d["reason"] for d in dropped).items():
        print(f"dropped {count}: {reason}")
    for d in dropped:
        print(f"  {d['episode']}: {d['reason']}: {d['detail']}")
    return planned(header(converted["arms"], converted["map"], min_clearance, planning), scenes)


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
        "--objects",
        action="store_true",
        help=f"write {OBJECTS_NAME}, scenes among collision objects, the hand holding a cube "
        "in every other one",
    )
    parser.add_argument(
        "--poses",
        action="store_true",
        help=f"write {POSES_NAME}, scenes among boxes whose goal is a pose of the grasp point",
    )
    parser.add_argument(
        "--both-arms",
        action="store_true",
        help=f"write {BOTH_NAME}, with --objects {BOTH_OBJECTS_NAME} or with --poses "
        f"{BOTH_POSES_NAME}, scenes of both arms planned as one group",
    )
    parser.add_argument(
        "--planning-time",
        type=float,
        metavar="SECONDS",
        help="the planning time: MoveIt's plans end on it alone, ours on it or ITERATIONS",
    )
    parser.add_argument(
        "--min-clearance",
        type=float,
        default=MIN_CLEARANCE,
        metavar="METERS",
        help="the clearance each plan keeps: min_clearance_m, and MoveIt's link padding",
    )
    args = parser.parse_args()
    if (args.episodes is None) != (args.launcher is None):
        parser.error(
            "--episodes and --launcher go together: give the episodes and the launcher fragment "
            "of the motion planner whose map they fill"
        )
    if args.episodes is not None and args.drawn:
        parser.error("--episodes writes the episodes' scenes in place of drawn ones: drop --drawn")
    if args.episodes is not None and args.objects:
        parser.error("--episodes writes the episodes' scenes among their voxels: drop --objects")
    if args.episodes is not None and args.both_arms:
        parser.error("--episodes writes scenes of the arm the episodes record: drop --both-arms")
    if args.objects and args.poses:
        parser.error("--objects and --poses each write a scenario of their own: give one")
    if args.episodes is not None and args.poses:
        parser.error("--episodes writes the episodes' scenes to their configurations: drop --poses")
    if args.planning_time is not None and not 0 < args.planning_time < math.inf:
        parser.error(f"--planning-time is {args.planning_time}; give the seconds, more than 0")
    if not 0 <= args.min_clearance < math.inf:
        parser.error(f"--min-clearance is {args.min_clearance}; give the meters, 0 or more")
    committed = args.episodes is None and not args.drawn
    if committed and (
        args.objective != "any"
        or args.planning_time is not None
        or args.min_clearance != MIN_CLEARANCE
    ):
        parser.error(
            "the committed scenario plans `any` on its iterations at a clearance of 0, the "
            "plans its scenes are picked by: give --drawn 200, or --episodes and --launcher, "
            "with --objective, --planning-time or --min-clearance"
        )
    planning = plans(args.objective, args.planning_time)
    if args.both_arms:
        name = BOTH_OBJECTS_NAME if args.objects else BOTH_POSES_NAME if args.poses else BOTH_NAME
        write(name, both_arms_scenario(args.drawn, planning, args.min_clearance, name))
        return
    if args.episodes is None:
        name = OBJECTS_NAME if args.objects else POSES_NAME if args.poses else NAME
        write(name, drawn_scenario(args.drawn, planning, args.min_clearance, name))
        return
    write(NAME, episodes_scenario(args.episodes, args.launcher, planning, args.min_clearance))


if __name__ == "__main__":
    main()
