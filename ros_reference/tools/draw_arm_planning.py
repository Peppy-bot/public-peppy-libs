# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the arm planning scenarios of the ROS reference harness.

Scenes of OpenArm v2's left arm among boxes of voxels: OUTSIDE_LIMITS scenes start one
joint outside its limits, below its lower limit and above its upper one, for the refusal
of such a start; every other scene holds one to three boxes in front of the arm, a start
and a goal configuration drawn inside the joint limits, and a seed. A box is written as
the keys of its first and last voxel on each axis, the voxels whose centers the drawn
box holds. motion_planning_core's example plan_arm_scenes plans every scene and gives its
answer and the spacing at which the reference probes a path, which the script writes into
the scenario. Of the first DRAWN
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
COMMITTED_BOTH, COMMITTED_BOTH_OBJECTS or COMMITTED_BOTH_POSES.

With --path-orientation it writes the scenes of a path that holds orientations, each among one
to three boxes, whose goal is a pose of each grasp point that keeps its turn at the start:
`whole`, the grasp point's whole orientation at the start; `free`, its approach axis within the
tolerance of straight down, free about it, from a start within FREE_START_RAD of that, where
MoveIt's check and its projection read the free axis alike, for MoveIt's check reads its tolerances
about the robot frame's axes and its projection about the link's; `two_links`, `whole` and the
elbow's link within a wide tolerance. With --both-arms each arm holds its grasp point: MoveIt's
reference ends in a segmentation fault on one orientation of a group of two chains. --path-tolerance
sets the tolerance, and --moveit-box the rotation-vector tolerances MoveIt holds: `inner`, a box inside the tolerance, each component at it
over the square root of 3, or `outer`, one around it, each at it, or with a free axis at pi times
the sine of half of it; a free axis takes pi about it. These scenes run locally, with --drawn.

With --robot so101 it writes SO101_NAME, scenes of SO-101's arm among one to three collision
objects within its reach, each start and goal configuration of the drawn scenes one that the
example self_distances, this crate's self model of a plan of the arm, holds apart, the goal by
at least min_clearance_m: its goal, by --goal, the position of the grasp point at the goal
configuration, that configuration, or that pose tilted and held within a tolerance, the hand
holding an object in every other scene with --holding. Two scenes, one starting and one ending
with the arm's own bodies meeting, take them more than MEETING_DEPTH into each other in the
self model. The scenario keeps the drawn scenes at COMMITTED_SO101. Run it from any
directory:

    uv run ros_reference/tools/draw_arm_planning.py
    uv run ros_reference/tools/draw_arm_planning.py --drawn 200
    uv run ros_reference/tools/draw_arm_planning.py --episodes <episodes> \
        --launcher <launchers-hub>/openarm/fragments/motion_planner.json5
    uv run ros_reference/tools/draw_arm_planning.py --drawn 200 --planner rrt_star \
        --planning-time 5
    uv run ros_reference/tools/draw_arm_planning.py --drawn 200 --refiner chomp \
        --planning-time 5
    uv run ros_reference/tools/draw_arm_planning.py --objects --drawn 200 --planning-time 1 \
        --min-clearance 0.04
    uv run ros_reference/tools/draw_arm_planning.py --poses --drawn 200 --planning-time 5
    uv run ros_reference/tools/draw_arm_planning.py --both-arms --objects
    uv run ros_reference/tools/draw_arm_planning.py --path-orientation free --drawn 200 \
        --planning-time 5 --moveit-box inner

The second keeps the first 200 drawn scenes in place of those it picks, a large set for a
local run, which the README describes. The third writes, in place of the drawn scenes,
the scenes of episodes of OpenArm v2 recorded in simulation, each a subdirectory of
<episodes> in the format the README gives, in the map of the motion planner whose
launcher fragment `--launcher` names, also a large set for a local run:
motion_planning_core's example scenes_from_episodes turns each episode into a scene, and
the script prints the episodes it drops and why. Each plan runs `--planner`, as
plan_joint_path names it: `rrt_connect`, RRTConnect, by default, or `rrt_star`, RRTstar; and
its path is refined by `--refiner`, as plan_joint_path names it: `none`, by default, or
`chomp`, MoveIt's CHOMP for MoveIt's plans and motion_planning_core's for ours. Each plan
ends on ITERATIONS, TIME_LIMIT far beyond them, so it repeats under its seed. The fourth
ends MoveIt's plans on `--planning-time` alone and ours on ITERATIONS or that time,
whichever comes first, as the robot runs (its motion planner's launcher fragment bounds a
plan by max_iterations); that option, `--planner rrt_star` and `--refiner chomp` go with a
drawn set or the episodes' scenes. The fifth draws 200
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
    OPENARM_V2,
    SO101,
    about_axis,
    arm,
    box_mesh,
    configuration,
    example,
    generator,
    gripper_links,
    joint_limits,
    l_mesh,
    link_pose,
    moved_link,
    options,
    pose_by_link,
    product,
    quaternion,
    robot,
    run_example,
    shape,
    turned,
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
# OpenArm v2's arms, by their SRDF names: a scene of one arm plans the left one, and a scene
# of both arms plans OPENARM_ARMS, in the order the SRDF's group BOTH_ARMS holds them, which
# is the order a scene holds their joints in. The boxes and the objects of a scene stand in
# front of the left arm, or mirrored across the robot's xz plane in front of the right one.
LEFT_ARM = arm(OPENARM_V2, "left_arm")
RIGHT_ARM = arm(OPENARM_V2, "right_arm")
OPENARM_ARMS = (LEFT_ARM, RIGHT_ARM)
BOTH_ARMS = "both_arms"
VOXEL_SIZE = 0.04
MAP_HALF_SIDE = 1.6
VOXEL_MAP = {"voxel_size": VOXEL_SIZE, "map_half_side": MAP_HALF_SIDE}
OUTSIDE_LIMITS = 2
# The clearance of the committed scenarios' plans: MoveIt's links unpadded, so the parity
# compares its verdicts at contact.
MIN_CLEARANCE = 0.0
CORNER_TOLERANCE = 0.1
GOAL_TOLERANCE = 0.01
PLANNERS = ("rrt_connect", "rrt_star")
REFINERS = ("none", "chomp")
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
    "MoveIt's path invalid": 9,
    "a path with corners": 103,
    "a start in a box": 17,
    "a goal in a box": 2,
}
# SO-101's scenes among collision objects, written with --robot so101.
SO101_NAME = "arm_planning_objects_so101"
SO101_ARM = arm(SO101, "arm")
SO101_GRIPPER = gripper_links(SO101, SO101_ARM)
# No scene of SO-101 holds a voxel: the map is empty, as on the robot, which has no depth
# camera.
SO101_VOXEL_MAP = {"voxel_size": 0.02, "map_half_side": 0.8}
# Where an object's origin stands: wrist_link's origin between its 5th and 95th percentile
# over 20000 configurations drawn inside the limits, rounded out to 1 cm.
SO101_OBJECT_CORNERS = [(-0.06, 0.26), (-0.21, 0.21), (-0.02, 0.36)]
# BOX_SIZES scaled by the 0.39 m of SO-101's grasp point from its base at all joints 0, over
# the 0.6 m of OpenArm v2's reach.
SO101_OBJECT_SIZES = (0.026, 0.13)
# A goal that names an orientation: the configuration's, tilted within the tolerance's reach
# of the orientations the arm holds at the position.
SO101_TILT_RAD = 0.05
SO101_TOLERANCE_RAD = 0.1
# The drawn scenes of SO-101's scenario it keeps, as COMMITTED names them, none of them in
# SO101_ON_KDL_TIMEOUT.
COMMITTED_SO101 = {
    "a straight path": 0,
    "a path with corners": 67,
    "MoveIt's path invalid": 112,
    "a start in an object": 52,
    "a goal in an object": 80,
}
# SO-101's drawn scenes in which a search of MoveIt's goal sampler reaches KDL's timeout, a
# clock, so MoveIt's goal states follow the machine's speed: measured with KDL's debug lines
# in the reference image on 2026-11-11, a search ending on the timeout in 27 and 137, and one
# solving after it in 65 and 102. The scenario keeps none of them.
SO101_ON_KDL_TIMEOUT = (27, 65, 102, 137)
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
    "a straight joint line through the robot's own body": 142,
}
# Among objects, the scenes of COMMITTED_BOTH's outcomes but MoveIt's path invalid, which
# none of the first DRAWN gives at the committed settings, and a path with both hands holding
# an object and one setting the left hand's object down on a table.
COMMITTED_BOTH_OBJECTS = {
    "a straight path": 0,
    "a path with corners": 16,
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
# What a path of the path orientation scenes holds, as the module says.
PATH_MODES = ("whole", "free", "two_links")
MOVEIT_BOXES = ("inner", "outer")
# The tolerance of a held orientation: the order of a carried cup's, about 5.7 degrees.
PATH_TOLERANCE = 0.1
# A start of `free` stands this near straight down, under any tolerance drawn here.
FREE_START_RAD = 0.04
# The grasp frame with its approach axis, +z, along the robot's -z, as a quaternion [x, y, z, w].
TOP_DOWN = [1.0, 0.0, 0.0, 0.0]
# The elbow's link of `two_links`, the link the arm's joint at ELBOW_JOINT moves, its third,
# and its tolerance: wide, for the hand moves.
ELBOW_JOINT = 2
ELBOW_TOLERANCE = 1.0
# How far a goal's grasp point stands from the start's in the path orientation scenes: from a
# short move to half the arm's 0.6 m reach from its shoulder to its grasp point.
HELD_REACH_M = (0.05, 0.3)
# How deep the two scenes whose start, and whose goal, holds the robot's own bodies meeting
# take them into each other in the self model, whose hulls enclose the meshes MoveIt checks,
# the other end of each apart.
MEETING_DEPTH = 0.02
# The start and goal pairs of the arms drawn at once for the self model to measure.
PAIRS_BATCH = 200
EMPTY_BATCHES = 10


def gripper(arm):
    """The links of the gripper of `arm`, an arm of OpenArm v2, which a held object may
    touch."""
    return gripper_links(OPENARM_V2, arm)


def group_of(planned_arms):
    """The SRDF's group of `planned_arms`: the arm itself, or both arms."""
    return {"group": planned_arms[0].name if len(planned_arms) == 1 else BOTH_ARMS}


def voxel_box(rng, arm=LEFT_ARM):
    """A box drawn in meters in front of `arm`, as the keys of the first and the last voxel on
    each axis whose center it holds."""
    low = [rng.uniform(low, high) for low, high in BOX_CORNERS]
    high = [corner + rng.uniform(*BOX_SIZES) for corner in low]
    if arm.name == RIGHT_ARM.name:
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
        "path_orientations": [],
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
        "path_orientations": [],
        "start": start,
        "goal": goal,
        "seed": seed,
        "ours": None,
    }


def drawn_object(rng, k, arm=LEFT_ARM, sizes=BOX_SIZES, corners=BOX_CORNERS):
    """The object `object_<k>`: one of OBJECT_KINDS, its sizes from `sizes`, turned any
    way, its origin where a corner of `corners` stands in front of `arm`."""
    size = lambda: rng.uniform(*sizes)
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
        length = size()
        shapes = [l_mesh(length, rng.uniform(sizes[0] / 2, length / 2), size())]
    else:
        base = [size(), size(), size() / 2]
        height, radius = size(), size() / 4
        shapes = [
            shape("box", base),
            shape("cylinder", [height, radius], at=(0.0, 0.0, (base[2] + height) / 2)),
        ]
    position = [rng.uniform(low, high) for low, high in corners]
    if arm.name == RIGHT_ARM.name:
        position[1] = -position[1]
    return {
        "id": f"object_{k}",
        "operation": "add",
        "position": position,
        "orientation": unit_quaternion(rng),
        "shapes": shapes,
    }


def held(object_id, k, arm):
    """The object `object_id` the hand of `arm` holds, in the frame of its grasp point, a
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
        "link": arm.grasp_link,
        "touch_links": gripper(arm),
    }


def set_down_table(model, goal):
    """The table the object the left hand holds is set down on at the goal configuration
    `goal`, which holds the left arm's joints first: under the object's center, its top
    SET_DOWN_REACH above the object's lowest point, which a cube or a cylinder of HELD_SIDE
    turned any way puts at least half of HELD_SIDE below its center."""
    joints = dict(zip(LEFT_ARM.joints, goal))
    center = pose_by_link(model, LEFT_ARM.grasp_link, joints, [0.0, 0.0, HELD_AHEAD])["position"]
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
        allowed = [["held", "table"], *[["table", link] for link in gripper(LEFT_ARM)]]
    attached = [held("held", k // 2, LEFT_ARM)] if holding else []
    if k % 3 == 0:
        attached.append(held("other", 0, RIGHT_ARM))
    return {
        "boxes": [],
        "objects": objects,
        "attached": attached,
        "allowed": allowed,
        "path_orientations": [],
        "start": start,
        "goal": goal,
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def grasp_pose_goal(rng, model, arm, start, goal):
    """The pose goal of the grasp point of `arm`, whose joints stand at `start`: with no
    `goal`, a short move, the start's grasp pose moved within SHORT_REACH_M and turned within
    SHORT_TURN_RAD, and else the grasp pose of the arm's joints `goal`; within
    POSE_TOLERANCE_M and POSE_TOLERANCE_RAD."""
    if goal is not None:
        pose = pose_by_link(model, arm.grasp_link, dict(zip(arm.joints, goal)), [0, 0, 0])
    else:
        turn, at = link_pose(model, arm.grasp_link, dict(zip(arm.joints, start)))
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
    return {**scene, "goal": [grasp_pose_goal(rng, model, LEFT_ARM, scene["start"], goal)]}


def held_orientation(arm_index, link, orientation, tolerance, free_axis, box):
    """An orientation of a path, with the tolerances MoveIt holds of `box`, as the module says."""
    if box == "inner":
        bound = tolerance / math.sqrt(3.0)
    elif free_axis == "none":
        bound = tolerance
    else:
        bound = math.pi * math.sin(tolerance / 2.0)
    return {
        "arm": arm_index,
        "link_name": link,
        "orientation": orientation,
        "tolerance_rad": tolerance,
        "free_axis": free_axis,
        "moveit_tolerances": [bound, bound, math.pi if free_axis == "z" else bound],
    }


def held_start(rng, model, limits, arm, free):
    """A configuration of `arm` drawn inside `limits`; with `free`, drawn until its approach
    axis stands within FREE_START_RAD of straight down."""
    while True:
        start = configuration(rng, limits)
        if not free:
            return start
        turn, _ = link_pose(model, arm.grasp_link, dict(zip(arm.joints, start)))
        if turned(turn, [0.0, 0.0, 1.0])[2] <= -math.cos(FREE_START_RAD):
            return start


def held_goal(rng, model, arm, start):
    """A pose goal of the grasp point of `arm`, whose joints stand at `start`: its turn there,
    at a position within HELD_REACH_M of its position there along a drawn direction."""
    turn, at = link_pose(model, arm.grasp_link, dict(zip(arm.joints, start)))
    direction = [rng.gauss(0.0, 1.0) for _ in range(3)]
    norm = math.sqrt(sum(d * d for d in direction))
    reach = rng.uniform(*HELD_REACH_M)
    return {
        "position": [a + d / norm * reach for a, d in zip(at, direction)],
        "orientation": quaternion(turn),
        "position_tolerance": POSE_TOLERANCE_M,
        "orientation_tolerance": POSE_TOLERANCE_RAD,
    }


def path_scene(rng, model, planned_arms, mode, tolerance, box):
    """A drawn scene of `planned_arms` that holds the orientations of `mode`."""
    free = mode == "free"
    starts = []
    goals = []
    held = []
    for k, posed in enumerate(planned_arms):
        limits = [joint_limits(OPENARM_V2, joint) for joint in posed.joints]
        start = held_start(rng, model, limits, posed, free)
        starts.extend(start)
        goals.append(held_goal(rng, model, posed, start))
        joints = dict(zip(posed.joints, start))
        turn, _ = link_pose(model, posed.grasp_link, joints)
        orientation = TOP_DOWN if free else quaternion(turn)
        free_axis = "z" if free else "none"
        held.append(held_orientation(k, posed.grasp_link, orientation, tolerance, free_axis, box))
        if mode == "two_links":
            elbow_link = moved_link(OPENARM_V2, posed.joints[ELBOW_JOINT])
            elbow, _ = link_pose(model, elbow_link, joints)
            held.append(
                held_orientation(k, elbow_link, quaternion(elbow), ELBOW_TOLERANCE, "none", box)
            )
    return {
        "boxes": [
            voxel_box(rng, posed) for posed in planned_arms for _ in range(rng.randint(1, 2))
        ],
        "objects": [],
        "attached": [],
        "allowed": [],
        "path_orientations": held,
        "start": starts,
        "goal": goals,
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def path_scenario(drawn, planning, min_clearance, mode, both_arms, tolerance, box):
    """The name of the scenario of `drawn` path orientation scenes of `mode` and the scenario."""
    name = f"arm_planning{'_both_arms' if both_arms else ''}_path_{mode}_openarm_v2"
    rng = generator(name)
    model = robot(OPENARM_V2)
    planned_arms = OPENARM_ARMS if both_arms else (LEFT_ARM,)
    scenes = [path_scene(rng, model, planned_arms, mode, tolerance, box) for _ in range(drawn)]
    scenario_header = header(group_of(planned_arms), VOXEL_MAP, min_clearance, planning)
    return name, planned(scenario_header, scenes)


def corners(ours):
    """The corners of our answer to a scene: its waypoints but the first and the last."""
    return max(len(ours.get("waypoints", [])) - 2, 0)


def plans(planner, refiner, planning_time):
    """The plans' planner, refiner and termination: ended on ITERATIONS, or with
    `planning_time`, MoveIt's on that time alone and ours on ITERATIONS or that time,
    whichever comes first."""
    if planning_time is None:
        termination = {"termination": "iterations", "time_limit": TIME_LIMIT}
    else:
        termination = {"termination": "time", "time_limit": planning_time}
    return {"planner": planner, "refiner": refiner, **termination}


def header(group, voxel_map, min_clearance, planning, description=OPENARM_V2):
    """The scenario's header: the robot, by its model directory `description`, the arms
    `group` (the SRDF's group of them), the map `voxel_map` (its voxel_size and
    map_half_side), the plans' clearance `min_clearance`, and the plans' parameters,
    `planning` (their planner, refiner and termination) among them."""
    return {
        "kind": "arm_planning",
        "description": description,
        **group,
        **voxel_map,
        "min_clearance": min_clearance,
        "corner_tolerance": CORNER_TOLERANCE,
        "goal_tolerance": GOAL_TOLERANCE,
        **planning,
        "iterations": ITERATIONS,
    }


def planned(scenario_header, scenes):
    """The scenario of `scenes`, each with motion_planning_core's answer, and the spacing at
    which the reference probes a path, from the example plan_arm_scenes."""
    answers = run_example(example("plan_arm_scenes"), {**scenario_header, "scenes": scenes})
    answered = [
        {**scene, "ours": ours} for scene, ours in zip(scenes, answers["ours"], strict=True)
    ]
    return {**scenario_header, "probe_spacing": answers["probe_spacing"], "scenes": answered}


def drawn_scenario(drawn, planning, min_clearance, name):
    """The scenario `name` of the drawn scenes, among boxes of voxels (NAME), among
    collision objects (OBJECTS_NAME) or among boxes to a pose (POSES_NAME): those it picks,
    or with `drawn` the first `drawn`, and the starts outside the limits, planned as
    `planning` says at `min_clearance`."""
    rng = generator(name)
    model = robot(OPENARM_V2)
    limits = [joint_limits(OPENARM_V2, joint) for joint in LEFT_ARM.joints]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    scene_of, committed = {
        NAME: (lambda k: drawn_scene(rng, limits), COMMITTED),
        OBJECTS_NAME: (lambda k: object_scene(rng, model, limits, k), COMMITTED_OBJECTS),
        POSES_NAME: (lambda k: pose_scene(rng, model, limits, k), COMMITTED_POSES),
    }[name]
    drawn_scenes = [scene_of(k) for k in range(drawn or DRAWN)]
    scenario_header = header(group_of((LEFT_ARM,)), VOXEL_MAP, min_clearance, planning)
    return kept(planned(scenario_header, [*drawn_scenes, *outside]), drawn, committed)


def so101_goal(rng, model, joints, goal):
    """The goal of a scene of SO-101, of the grasp point at the configuration `joints`: `goal`
    names it, the configuration, its position, or its pose tilted by SO101_TILT_RAD within
    SO101_TOLERANCE_RAD."""
    if goal == "joints":
        return joints
    turn, at = link_pose(model, SO101_ARM.grasp_link, dict(zip(SO101_ARM.joints, joints)))
    if goal == "position":
        return [{"position": at, "position_tolerance": POSE_TOLERANCE_M}]
    axis = [rng.gauss(0, 1) for _ in range(3)]
    return [
        {
            "position": at,
            "orientation": quaternion(product(turn, about_axis(axis, SO101_TILT_RAD))),
            "position_tolerance": POSE_TOLERANCE_M,
            "orientation_tolerance": SO101_TOLERANCE_RAD,
        }
    ]


def so101_held(k):
    """The object SO-101's hand holds in its grasp point's frame, a cube or, for an odd `k`,
    a cylinder, its gripper the links it may touch."""
    held_shape = (
        shape("cylinder", [HELD_SIDE, HELD_SIDE / 2]) if k % 2 else shape("box", [HELD_SIDE] * 3)
    )
    return {
        "object": {
            "id": "held",
            "operation": "add",
            "position": [0.0, 0.0, HELD_AHEAD],
            "orientation": IDENTITY,
            "shapes": [held_shape],
        },
        "link": SO101_ARM.grasp_link,
        "touch_links": SO101_GRIPPER,
    }


def so101_scene(rng, model, k, start, joints, goal, holding):
    """The `k`th drawn scene of SO-101 among one to three drawn objects, from `start` to the
    goal `goal` names of the configuration `joints`, and with `holding`, the hand holding an
    object in every other scene."""
    objects = [
        drawn_object(rng, i, sizes=SO101_OBJECT_SIZES, corners=SO101_OBJECT_CORNERS)
        for i in range(rng.randint(1, 3))
    ]
    return {
        "boxes": [],
        "objects": objects,
        "attached": [so101_held(k // 2)] if holding and k % 2 else [],
        "allowed": [],
        "path_orientations": [],
        "start": start,
        "goal": so101_goal(rng, model, joints, goal),
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def so101_scenario(drawn, planning, min_clearance, goal, holding):
    """The scenario SO101_NAME of SO-101 among collision objects: the drawn scenes it picks,
    or with `drawn` the first `drawn`, the scenes whose start and whose goal hold the arm's own
    bodies meeting, and the starts outside the limits, planned as `planning` says at
    `min_clearance`."""
    rng = generator(SO101_NAME)
    model = robot(SO101)
    limits = [joint_limits(SO101, joint) for joint in SO101_ARM.joints]
    scenario_header = header(
        {"group": SO101_ARM.name},
        SO101_VOXEL_MAP,
        min_clearance,
        planning,
        description=SO101,
    )
    pairs, meeting = self_clear_pairs(rng, scenario_header, limits, drawn or DRAWN)
    scenes = [
        so101_scene(rng, model, k, start, joints, goal, holding)
        for k, (start, joints) in enumerate(pairs)
    ]
    outside = [outside_limits(rng, limits, k) for k in range(OUTSIDE_LIMITS)]
    scenario = planned(scenario_header, [*scenes, *meeting, *outside])
    return kept(scenario, drawn, COMMITTED_SO101, SO101_ON_KDL_TIMEOUT)


def kept(scenario, drawn, committed, excluded=()):
    """`scenario`, its drawn scenes first and every other scene after them, keeping of
    those drawn the first `drawn`, or with no `drawn` those at `committed` and the one whose
    path has the most corners among the first DRAWN but those `excluded`."""
    answered = scenario["scenes"]
    count = drawn or DRAWN
    kept_drawn = answered[:count]
    if not drawn:
        candidates = [k for k in range(DRAWN) if k not in excluded]
        most = max(candidates, key=lambda k: corners(kept_drawn[k]["ours"]))
        kept_drawn = [kept_drawn[k] for k in dict.fromkeys([*committed.values(), most])]
    return {**scenario, "scenes": [*kept_drawn, *answered[count:]]}


def self_distances(scenario_header, pairs):
    """The self model's least distance between the robot's own bodies at the start and at
    the goal of each of `pairs`, and along the straight joint line between them, from the
    example self_distances."""
    scenes = [plain_scene(start, goal, 1) for start, goal in pairs]
    return run_example(example("self_distances"), {**scenario_header, "scenes": scenes})


def self_clear_pairs(rng, scenario_header, limits, count):
    """The first `count` starts and goals of the arms of `scenario_header` drawn inside
    `limits`, in batches of PAIRS_BATCH, whose start the self model holds apart and whose goal
    it holds at least min_clearance_m apart, a goal nearer than that being one this crate's
    plans refuse and MoveIt's, which check the robot's own bodies unpadded, reach; and the
    scenes of the first drawn whose start holds the robot's own bodies meeting, by more than
    MEETING_DEPTH, its goal apart, and of the first whose goal holds them so, its start
    apart. Stops the script when EMPTY_BATCHES batches hold no such pair."""
    apart, start_meeting, goal_meeting = [], None, None
    clearance = scenario_header["min_clearance"]
    batches = 0
    while len(apart) < count or start_meeting is None or goal_meeting is None:
        batches += 1
        if batches > EMPTY_BATCHES and not apart:
            raise SystemExit(
                f"--min-clearance {clearance}: no drawn goal of the arms keeps it in the self "
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
        boxes = [voxel_box(rng, rng.choice(OPENARM_ARMS)) for _ in range(rng.randint(1, 3))]
        return {**scene, "boxes": boxes, "seed": rng.randint(1, 2**31 - 1)}
    objects = [
        drawn_object(rng, i, rng.choice(OPENARM_ARMS)) for i in range(rng.randint(1, 3))
    ]
    holding = k % 2 == 1
    allowed = []
    if holding and k % 8 == 7:
        objects.append(set_down_table(model, goal))
        allowed = [["held", "table"], *[["table", link] for link in gripper(LEFT_ARM)]]
    attached = [held("held", k // 2, LEFT_ARM)] if holding else []
    if k % 3 == 0:
        attached.append(held("other", 0, RIGHT_ARM))
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
    joints = len(LEFT_ARM.joints)
    each_arm = zip(OPENARM_ARMS, (start[:joints], start[joints:]), (goal[:joints], goal[joints:]))
    poses = [
        grasp_pose_goal(rng, model, arm, arm_start, arm_goal if k % 2 else None)
        for arm, arm_start, arm_goal in each_arm
    ]
    return {**scene, "goal": poses}


def both_arms_scenario(drawn, planning, min_clearance, name):
    """The scenario `name` of the drawn scenes of both arms, among boxes of voxels
    (BOTH_NAME), among collision objects (BOTH_OBJECTS_NAME) or among boxes to a pose of each
    grasp point (BOTH_POSES_NAME): those it picks, or with `drawn` the first `drawn`, the
    scenes whose start and whose goal hold the robot's own bodies meeting, and the starts
    outside the limits, planned as `planning` says at `min_clearance`."""
    rng = generator(name)
    model = robot(OPENARM_V2)
    limits = [joint_limits(OPENARM_V2, joint) for arm in OPENARM_ARMS for joint in arm.joints]
    scenario_header = header(group_of(OPENARM_ARMS), VOXEL_MAP, min_clearance, planning)
    pairs, meeting = self_clear_pairs(rng, scenario_header, limits, drawn or DRAWN)
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
    TIME_LIMIT, at the plans' clearance, so the scenes are the same under every planner,
    refiner and termination."""
    command = example("scenes_from_episodes", str(episodes), str(launcher))
    planning = plans("rrt_connect", "none", None)
    converted = run_example(command, header({}, {}, min_clearance, planning))
    scenes, dropped = converted["scenes"], converted["dropped"]
    print(f"{len(scenes)} scenes of {len(scenes) + len(dropped)} episodes")
    for reason, count in Counter(d["reason"] for d in dropped).items():
        print(f"dropped {count}: {reason}")
    for d in dropped:
        print(f"  {d['episode']}: {d['reason']}: {d['detail']}")
    return planned(
        header({"group": converted["group"]}, converted["map"], min_clearance, planning), scenes
    )


def main():
    parser = options(__doc__, 0, "drawn scenes kept in place of those it picks; 0 picks")
    parser.add_argument(
        "--episodes", type=Path, help="a directory of episodes, in the format the README gives"
    )
    parser.add_argument(
        "--launcher", type=Path, help="the launcher fragment of the motion planner, with --episodes"
    )
    parser.add_argument(
        "--planner",
        choices=PLANNERS,
        default="rrt_connect",
        help="the planner of each plan: rrt_connect (RRTConnect) or rrt_star (RRTstar)",
    )
    parser.add_argument(
        "--refiner",
        choices=REFINERS,
        default="none",
        help="what refines each plan's path: none, or chomp (CHOMP, MoveIt's and ours)",
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
        "--robot",
        choices=("openarm_v2", "so101"),
        default="openarm_v2",
        help=f"so101 writes {SO101_NAME}, scenes of SO-101 among collision objects",
    )
    parser.add_argument(
        "--goal",
        choices=("position", "joints", "tolerance"),
        default="position",
        help="with --robot so101, what each scene's goal names: the grasp point's position, a "
        "configuration, or a pose held within a tolerance",
    )
    parser.add_argument(
        "--holding",
        action="store_true",
        help="with --robot so101, the hand holds an object in every other scene",
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
    parser.add_argument(
        "--path-orientation",
        choices=PATH_MODES,
        help="write scenes of a path that holds orientations, as the module says, with --drawn",
    )
    parser.add_argument(
        "--path-tolerance",
        type=float,
        default=PATH_TOLERANCE,
        help="the tolerance of the orientations a path holds, in radians, with --path-orientation",
    )
    parser.add_argument(
        "--moveit-box",
        choices=MOVEIT_BOXES,
        default="inner",
        help="the rotation-vector tolerances MoveIt holds, with --path-orientation",
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
    if args.robot == "so101" and (
        args.episodes is not None
        or args.objects
        or args.poses
        or args.both_arms
        or args.path_orientation is not None
    ):
        parser.error(
            "--robot so101 writes its scenes among collision objects: drop --episodes, "
            "--objects, --poses, --both-arms and --path-orientation, and name the goal with --goal"
        )
    if args.planning_time is not None and not 0 < args.planning_time < math.inf:
        parser.error(f"--planning-time is {args.planning_time}; give the seconds, more than 0")
    if not 0 <= args.min_clearance < math.inf:
        parser.error(f"--min-clearance is {args.min_clearance}; give the meters, 0 or more")
    if args.robot != "so101" and (args.goal != "position" or args.holding):
        parser.error("--goal and --holding name what SO-101's scenes hold: give --robot so101")
    if args.path_orientation is not None:
        if not args.drawn or args.episodes is not None or args.objects or args.poses:
            parser.error(
                "--path-orientation draws a set for a local run: give --drawn, and drop --episodes, "
                "--objects and --poses"
            )
        if args.path_orientation == "two_links" and args.both_arms:
            parser.error("--path-orientation two_links plans one arm: drop --both-arms")
        write(
            *path_scenario(
                args.drawn,
                plans(args.planner, args.refiner, args.planning_time),
                args.min_clearance,
                args.path_orientation,
                args.both_arms,
                args.path_tolerance,
                args.moveit_box,
            )
        )
        return
    committed = args.episodes is None and not args.drawn
    if committed and (
        args.planner != "rrt_connect"
        or args.refiner != "none"
        or args.planning_time is not None
        or args.min_clearance != MIN_CLEARANCE
    ):
        parser.error(
            "the committed scenario plans `rrt_connect` refined by `none` on its iterations at a "
            "clearance of 0, the plans its scenes are picked by: give --drawn 200, or --episodes "
            "and --launcher, with --planner, --refiner, --planning-time or --min-clearance"
        )
    if committed and (args.goal != "position" or args.holding):
        parser.error(
            "SO-101's committed scenario goes to positions with nothing held: give --drawn 200 "
            "with --goal or --holding"
        )
    planning = plans(args.planner, args.refiner, args.planning_time)
    if args.robot == "so101":
        write(
            SO101_NAME,
            so101_scenario(args.drawn, planning, args.min_clearance, args.goal, args.holding),
        )
        return
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
