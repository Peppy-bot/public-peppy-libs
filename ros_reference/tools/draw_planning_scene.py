# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the planning scene scenario of the ROS reference harness: calls of
motion_planning's scene members on OpenArm v2, applied in order, and configurations of
its left arm to measure the scene at.

The calls add, in one batch, an object of each kind of shape: a box, a sphere, a
cylinder, a table, an L-shaped closed mesh and a closed box mesh; a caddy of three
shapes, a box, a cylinder and a closed box mesh; and a tote of a closed box mesh and then
a box, its mesh before its primitive, where the caddy stands mirrored to the right of the
robot. They allow pairs of
objects with links, an object and the voxel map, and stop one; move the sphere, whose
pair stays, and remove the cylinder, which drops its pair, in one batch; replace the
box, whose pair stays; allow a pair of an item, which stays through its attach to the
left grasp point with the fingers as its touch links, and allow another once it is
attached; attach and detach a lid; and attach a tool to the right grasp point, with the
right fingers as its touch links; and attach a cap to the left grasp point, allowed to
touch the voxel map. Each attach and detach names the joint positions of its arm.

The paths hold the configurations in their order, each two in turn and all of them in
one path, and CLUTTER_PATHS drawn among the objects. The run of the first, from the zero
configuration to the lower limits, passes a post: a sphere at the left grasp point
halfway along the run, which the calls add with the table. The reference checks each
path again with MoveIt's links padded by PATH_PADDING.

The cut-out probes are points drawn about each object the calls leave standing but the
lid, PROBES_PER_OBJECT of them within PROBE_REACH of its origin on each axis, which the
reference tests against MoveIt's shape mask of the objects grown by each of PADDINGS.

The scene holds a voxel map from the start: voxels that the table, the item and the cap
fill where they are added, which the scene clears and MoveIt's octomap keeps; a voxel
beside CONTACT_LINK's box at its placed configuration, MAP_STANDOFF off the box or more;
and voxels drawn in REGION. The calls mark two more: where the sphere moves to, before the
move, and where the item stands, before its attach, which the move and the attach clear.

The configurations are the zero configuration, the lower limits, the upper limits, and
configurations drawn inside the limits: the one the item and the lid are attached at,
the one the lid is detached at, and three that each place objects around a link, in
the link's frame: the sphere holding the box of CONTACT_LINK's collision mesh and the
replacing box filling ALLOWED_LINK's, whose pair with it is allowed; INSIDE_LINK
wholly inside the box mesh; and CORNER_LINK in the open corner of the L. The objects are
drawn before the configurations, so --drawn adds configurations and leaves the objects
as they are. Run it from any directory:

    uv run ros_reference/tools/draw_planning_scene.py
    uv run ros_reference/tools/draw_planning_scene.py --drawn 60

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import (
    IDENTITY,
    OPENARM_V2,
    arm,
    box_mesh,
    configurations,
    finger_links,
    generator,
    joint_limits,
    l_mesh,
    link_bounds,
    link_pose,
    meshes,
    options,
    pose_by_link,
    robot,
    shape,
    turned,
    unit_quaternion,
    write,
)

NAME = "planning_scene_openarm_v2"
MESHES = meshes(OPENARM_V2)
# OpenArm v2's arms, by their SRDF names: the scenario places the left one, and the right
# one holds a tool.
LEFT_ARM = arm(OPENARM_V2, "left_arm")
RIGHT_ARM = arm(OPENARM_V2, "right_arm")
LEFT_FINGERS = finger_links(LEFT_ARM)
RIGHT_FINGERS = finger_links(RIGHT_ARM)
# The links the placed configurations put in their cases.
CONTACT_LINK = "openarm_left_link6"
ALLOWED_LINK = "openarm_left_link5"
INSIDE_LINK = "openarm_left_link6"
CORNER_LINK = "openarm_left_link4"
# How far the placed objects stand off the bounds of the links they place: more than the
# 16.6 mm a link's pieces read nearer an object than FCL, as the parity test prints it.
STANDOFF = 0.02
# The side of the map's voxels, as motion_planning's launch of OpenArm gives it, and how
# many voxels are drawn in REGION.
VOXEL_SIZE = 0.04
DRAWN_VOXELS = 12
# How far the voxel beside CONTACT_LINK stands off the link's box at the least: short of
# contact.
MAP_STANDOFF = 0.03
# The box of the robot frame the drawn objects stand in: in front of the left arm, at
# its height.
REGION = ((0.05, 0.6), (-0.15, 0.55), (0.3, 1.1))
TABLE = [0.6, 1.0, 0.05]
ITEM = [0.04, 0.04, 0.08]
# A cylinder's height then its radius.
LID = [0.02, 0.06]
TOOL = [0.1, 0.02]
CAP = [0.03, 0.03, 0.03]
CAP_AHEAD = 0.05
POST_RADIUS = 0.03
# The paths drawn among the objects, from their own generator: from the zero
# configuration, the lower limits and a configuration drawn inside the limits in turn,
# each to a configuration within CLUTTER_REACH_RAD of it in each joint, inside the limits;
# some pass among the objects, some run into them, and some keep clear of them.
CLUTTER_PATHS = 12
CLUTTER_REACH_RAD = 0.3
# The paddings the cut-out probes are tested at: none; the motion planner's launch
# padding_offset_m of 0.04 at padding_scale 1; and a scale with an offset, which MoveIt
# applies to each dimension about the shape's center, then the offset (geometric_shapes'
# scaleAndPadd): at 1.2 and 0.02, the scale moves each side face of the 4 cm item 4 mm
# out, and the offset 2 cm more.
PADDINGS = [
    {"scale": 1.0, "offset": 0.0},
    {"scale": 1.0, "offset": 0.04},
    {"scale": 1.2, "offset": 0.02},
]
# The padding of MoveIt's second check of each path, and the clearance of this crate's: the
# motion planner's launch min_clearance_m. MoveIt pads each link with collision geometry,
# moving each vertex of its mesh along the vertex's normal, so a face beside a right-angled
# edge moves out the padding over the square root of 2; it pads no object the hand holds,
# which takes the padding of the grasp point, a link with no collision geometry.
PATH_PADDING = 0.04
PROBES_PER_OBJECT = 12
# 15 cm on each axis from an object's origin: past the grown faces of the caddy, the
# farthest of the small objects' (its can's top, 6 + 4 = 10 cm from its origin, 14 cm
# grown by the launch offset), so their probes fall inside them, between their paddings
# and beyond them; the table's and the L's probes lie about their origins.
PROBE_REACH = 0.15
# The caddy's shapes: a box, a cylinder standing on it, and a cube mesh beside it.
CADDY_BOX = [0.1, 0.06, 0.04]
CADDY_CAN = [0.08, 0.02]
CADDY_CUBE = [0.04, 0.04, 0.04]
# The tote's shapes, in this order: a bin, a closed box mesh, and a lid, a box resting on
# the bin. MoveIt's planning scene message lists an object's meshes after its primitives.
TOTE_BIN = [0.08, 0.06, 0.04]
TOTE_LID = [0.08, 0.06, 0.01]
# The configurations drawn inside the left arm's limits for the calls and the cases, in
# their order after the zero configuration and the limits.
PLACED = ["attach", "detach", "contact", "inside", "corner"]
DRAWN = "configurations drawn inside the left arm's limits after the placed ones"


def drawn_pose(rng):
    return {
        "position": [rng.uniform(low, high) for low, high in REGION],
        "orientation": unit_quaternion(rng),
    }


def mark(at):
    """The voxel that holds the position of `at`, marked occupied."""
    return {"op": "mark", "keys": [[math.floor(c / VOXEL_SIZE) for c in at["position"]]]}


def near(rng, at, limits):
    """A configuration drawn within CLUTTER_REACH_RAD of `at` in each joint, inside
    `limits`, which lists each joint's in the order of `at`."""
    return {
        joint: rng.uniform(max(low, q - CLUTTER_REACH_RAD), min(high, q + CLUTTER_REACH_RAD))
        for (joint, q), (low, high) in zip(at.items(), limits)
    }


def entry(operation, object_id, at=None, shapes=()):
    """One entry of apply_collision_objects; a removal carries the fields it does not
    read as an identity pose and no shape, and a move no shape."""
    at = at or {"position": [0.0, 0.0, 0.0], "orientation": IDENTITY}
    return {"id": object_id, "operation": operation, **at, "shapes": list(shapes)}


def objects(*entries):
    return {"op": "objects", "objects": list(entries)}


def allow(name, other_names, allowed=True):
    return {"op": "allow", "name": name, "other_names": other_names, "allowed": allowed}


def attached(object_id, operation, link_name, positions, touch_links=()):
    return {
        "op": "attached",
        "object_id": object_id,
        "link_name": link_name,
        "touch_links": list(touch_links),
        "operation": operation,
        "positions": positions,
    }


def center_and_size(bounds):
    low, high = bounds
    return [(lo + hi) / 2 for lo, hi in zip(low, high)], [hi - lo for lo, hi in zip(low, high)]


def main():
    drawn = options(__doc__, 0, DRAWN).parse_args().drawn
    rng = generator(NAME)
    parsed = robot(OPENARM_V2)

    box_at = drawn_pose(rng)
    box_size = [rng.uniform(0.05, 0.3) for _ in range(3)]
    ball_at = drawn_pose(rng)
    can_at = drawn_pose(rng)
    can_size = [rng.uniform(0.1, 0.3), rng.uniform(0.03, 0.08)]
    table_at = drawn_pose(rng)
    corner_side = rng.uniform(0.22, 0.3)
    corner_width = rng.uniform(0.05, 0.1)
    lid_at = drawn_pose(rng)
    caddy_at = drawn_pose(rng)
    drawn_voxels = [
        [rng.uniform(low, high) for low, high in REGION] for _ in range(DRAWN_VOXELS)
    ]

    right_limits = [joint_limits(OPENARM_V2, joint) for joint in RIGHT_ARM.joints]
    right_at = dict(zip(RIGHT_ARM.joints, configurations(rng, right_limits, 1)[-1]))
    left_limits = [joint_limits(OPENARM_V2, joint) for joint in LEFT_ARM.joints]
    left_configurations = [
        dict(zip(LEFT_ARM.joints, q))
        for q in configurations(rng, left_limits, len(PLACED) + drawn)
    ]
    placed_at = dict(zip(PLACED, left_configurations[3:]))

    # The sphere holds the box of CONTACT_LINK's collision mesh, and the replacing box
    # is ALLOWED_LINK's.
    contact_center, contact_size = center_and_size(link_bounds(parsed, CONTACT_LINK, MESHES))
    ball_radius = math.sqrt(sum(s * s for s in contact_size)) / 2 + STANDOFF
    allowed_center, allowed_size = center_and_size(link_bounds(parsed, ALLOWED_LINK, MESHES))
    # The box mesh holds INSIDE_LINK's box, STANDOFF inside each of its faces.
    inside_center, inside_size = center_and_size(link_bounds(parsed, INSIDE_LINK, MESHES))
    crate_size = [s + 2 * STANDOFF for s in inside_size]
    # The L's bars stand STANDOFF off CORNER_LINK's box, which lies in the L's open
    # corner, over the box's height. Each bar is 22 to 30 cm long: past the box, 7.8 cm
    # across, its STANDOFF on both sides and the other bar's width of up to 10 cm, 21.8
    # cm, and short of the shoulder, so the clutter paths that keep clear of the objects
    # pass 2 cm or more from them.
    (x_low, y_low, z_low), (_, _, z_high) = link_bounds(parsed, CORNER_LINK, MESHES)
    corner_origin = [x_low - STANDOFF - corner_width, y_low - STANDOFF - corner_width, z_low]

    # The cap hangs 5 cm past the left grasp point where the lid is detached, 5 mm into
    # the item, which reaches half its height, 4 cm, past the grasp point that holds it.
    cap_at = pose_by_link(parsed, LEFT_ARM.grasp_link, placed_at["detach"], [0.0, 0.0, CAP_AHEAD])
    item_at = pose_by_link(parsed, LEFT_ARM.grasp_link, placed_at["attach"], [0.0, 0.0, 0.0])
    # The post stands at the left grasp point halfway along the run from the zero
    # configuration to the lower limits.
    zero, lower = left_configurations[:2]
    halfway = {joint: (zero[joint] + lower[joint]) / 2 for joint in LEFT_ARM.joints}
    post_at = pose_by_link(parsed, LEFT_ARM.grasp_link, halfway, [0.0, 0.0, 0.0])
    clutter_rng = generator(f"{NAME}:clutter")
    drawn_starts = [
        dict(zip(LEFT_ARM.joints, q))
        for q in configurations(clutter_rng, left_limits, CLUTTER_PATHS // 3)[3:]
    ]
    starts = [
        left_configurations[k % 3] if k % 3 < 2 else drawn_starts[k // 3]
        for k in range(CLUTTER_PATHS)
    ]
    clutter = [[start, near(clutter_rng, start, left_limits)] for start in starts]
    paths = (
        [
            [left_configurations[i], left_configurations[i + 1]]
            for i in range(len(left_configurations) - 1)
        ]
        + [left_configurations]
        + clutter
    )
    contact_turn, contact_origin = link_pose(parsed, CONTACT_LINK, placed_at["contact"])
    (_, _, _), (x_high, _, _) = link_bounds(parsed, CONTACT_LINK, MESHES)
    # The voxel beside CONTACT_LINK holds the point a voxel's diagonal past MAP_STANDOFF
    # off the box's face, so each of its points lies MAP_STANDOFF past the face or more.
    beside = x_high + MAP_STANDOFF + VOXEL_SIZE * math.sqrt(3)
    beside_contact = [
        p + q
        for p, q in zip(
            contact_origin,
            turned(contact_turn, [beside, contact_center[1], contact_center[2]]),
        )
    ]
    voxel_points = (
        [table_at["position"], item_at["position"], cap_at["position"], beside_contact]
        + drawn_voxels
    )
    octomap = {
        "voxel_size": VOXEL_SIZE,
        "keys": sorted({tuple(math.floor(c / VOXEL_SIZE) for c in p) for p in voxel_points}),
    }
    # The tote stands where the caddy does, mirrored across the robot's sagittal plane, in
    # front of the right arm.
    tote_at = {
        "position": [caddy_at["position"][0], -caddy_at["position"][1], caddy_at["position"][2]],
        "orientation": caddy_at["orientation"],
    }
    probe_rng = generator(f"{NAME}:cut_out")
    probes = [
        [c + probe_rng.uniform(-PROBE_REACH, PROBE_REACH) for c in at["position"]]
        for at in [
            table_at,
            post_at,
            caddy_at,
            pose_by_link(parsed, CORNER_LINK, placed_at["corner"], corner_origin),
            pose_by_link(parsed, INSIDE_LINK, placed_at["inside"], inside_center),
            pose_by_link(parsed, CONTACT_LINK, placed_at["contact"], contact_center),
            pose_by_link(parsed, ALLOWED_LINK, placed_at["contact"], allowed_center),
            tote_at,
            # The attached objects where the calls leave them: the item and the cap on the
            # left grasp point, at the configuration the cap is attached at, and the tool
            # on the right one.
            pose_by_link(parsed, LEFT_ARM.grasp_link, placed_at["detach"], [0.0, 0.0, 0.0]),
            cap_at,
            pose_by_link(parsed, RIGHT_ARM.grasp_link, right_at, [0.0, 0.0, 0.0]),
        ]
        for _ in range(PROBES_PER_OBJECT)
    ]
    operations = [
        objects(
            entry("add", "box", box_at, [shape("box", box_size)]),
            entry("add", "ball", ball_at, [shape("sphere", [ball_radius])]),
            entry("add", "can", can_at, [shape("cylinder", can_size)]),
            entry("add", "table", table_at, [shape("box", TABLE)]),
            entry("add", "post", post_at, [shape("sphere", [POST_RADIUS])]),
            entry(
                "add",
                "corner",
                pose_by_link(parsed, CORNER_LINK, placed_at["corner"], corner_origin),
                [l_mesh(corner_side, corner_width, z_high - z_low)],
            ),
            entry(
                "add",
                "crate",
                pose_by_link(parsed, INSIDE_LINK, placed_at["inside"], [0.0, 0.0, 0.0]),
                [box_mesh(crate_size, inside_center)],
            ),
            entry(
                "add",
                "caddy",
                caddy_at,
                [
                    shape("box", CADDY_BOX),
                    shape("cylinder", CADDY_CAN, at=(0.0, 0.0, 0.06)),
                    box_mesh(CADDY_CUBE, (0.08, 0.0, 0.0)),
                ],
            ),
            entry(
                "add",
                "tote",
                tote_at,
                [
                    box_mesh(TOTE_BIN, (0.0, 0.0, 0.0)),
                    shape("box", TOTE_LID, at=(0.0, 0.0, (TOTE_BIN[2] + TOTE_LID[2]) / 2)),
                ],
            ),
        ),
        allow("box", [ALLOWED_LINK]),
        allow("corner", ["openarm_left_link6", "table"]),
        allow("corner", ["openarm_left_link6"], allowed=False),
        allow("crate", ["occupancy_map"]),
        allow("ball", ["openarm_left_link1"]),
        allow("can", ["openarm_left_link3"]),
        mark(pose_by_link(parsed, CONTACT_LINK, placed_at["contact"], contact_center)),
        objects(
            entry(
                "move",
                "ball",
                pose_by_link(parsed, CONTACT_LINK, placed_at["contact"], contact_center),
            ),
            entry("remove", "can"),
        ),
        objects(
            entry(
                "add",
                "box",
                pose_by_link(parsed, ALLOWED_LINK, placed_at["contact"], allowed_center),
                [shape("box", allowed_size)],
            )
        ),
        objects(
            entry(
                "add",
                "item",
                item_at,
                [shape("box", ITEM)],
            ),
            entry("add", "lid", lid_at, [shape("cylinder", LID)]),
            entry(
                "add",
                "tool",
                pose_by_link(parsed, RIGHT_ARM.grasp_link, right_at, [0.0, 0.0, 0.0]),
                [shape("cylinder", TOOL)],
            ),
            entry("add", "cap", cap_at, [shape("box", CAP)]),
        ),
        allow("item", ["table"]),
        mark(item_at),
        attached("item", "attach", LEFT_ARM.grasp_link, placed_at["attach"], LEFT_FINGERS),
        allow("item", ["openarm_left_link3"]),
        attached("lid", "attach", LEFT_ARM.grasp_link, placed_at["attach"], LEFT_FINGERS),
        attached("lid", "detach", LEFT_ARM.grasp_link, placed_at["detach"]),
        attached("tool", "attach", RIGHT_ARM.grasp_link, right_at, RIGHT_FINGERS),
        allow("cap", ["occupancy_map"]),
        attached("cap", "attach", LEFT_ARM.grasp_link, placed_at["detach"], LEFT_FINGERS),
    ]
    write(
        NAME,
        {
            "kind": "planning_scene",
            "description": OPENARM_V2,
            "arm": LEFT_ARM.name,
            "octomap": {
                "voxel_size": octomap["voxel_size"],
                "keys": [list(k) for k in octomap["keys"]],
            },
            "operations": operations,
            "configurations": left_configurations,
            "paths": paths,
            "path_padding": PATH_PADDING,
            "cut_out": {"paddings": PADDINGS, "probes": probes},
        },
    )


if __name__ == "__main__":
    main()
