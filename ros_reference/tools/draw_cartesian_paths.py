# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the Cartesian paths scenario of the ROS reference harness.

Cases of OpenArm v2's left arm, each a start drawn inside the joint limits off the seams of
the arm's branches, the link that follows the case's lines, the lines, whether the case
avoids collisions, and a seed. The end of each line is an offset from the link's pose at
the start, a translation in the robot frame and a rotation vector in the link's frame, so
the reference and the parity test place it from their own forward kinematics. The cases
come in rounds, each one case of each draw of `round_of_cases`: a line of the grasp point; a
line straight away from the shoulder, past the arm's reach, collisions unchecked; a line
that runs into a box and avoids collisions, and the same line unchecked; a line into a
collision object; a turn in place; two lines; a line that stands still, then a line; a line of
the last joint's link; and a line
from a start inside a box. A line, a turn and two lines avoid collisions in every other
round. motion_planning_core's example cartesian_scenes answers every case and gives the
step of its lines, which the script writes into the scenario. Of the first DRAWN cases, the
scenario keeps the first that shows each behavior of BEHAVIORS, and the first line a box
ends with the same line unchecked, whole. It runs cargo in motion_planning_core, so the
host builds that crate. Run it from any directory:

    uv run ros_reference/tools/draw_cartesian_paths.py
    uv run ros_reference/tools/draw_cartesian_paths.py --drawn 200
    uv run ros_reference/tools/draw_cartesian_paths.py --drawn 200 --min-clearance 0.04

The second keeps the first 200 cases in place of those it picks, a large set for a local
run, which the README describes. The third keeps them at a clearance of 4 cm:
`--min-clearance` sets the clearance of the paths, MIN_CLEARANCE by default, the
min_clearance_m of motion_planning_core's paths and the padding of each link of MoveIt's
robot against the octomap and the objects, which meets itself unpadded; it goes with a drawn
set.
"""

import itertools
import math

from drawing import (
    IDENTITY,
    OPENARM_V2,
    arm,
    configuration,
    example,
    generator,
    joint_limits,
    link_pose,
    moved_link,
    options,
    robot,
    run_example,
    shape,
    write,
)

NAME = "cartesian_paths_openarm_v2"
# The arm the cases move, by its SRDF name.
ARM = arm(OPENARM_V2, "left_arm")
VOXEL_SIZE = 0.04
MAP_HALF_SIDE = 1.6
# The clearance of the committed scenario's paths: MoveIt's links unpadded, so the parity
# compares its verdicts at contact.
MIN_CLEARANCE = 0.0
# The drawn cases the scenario picks from.
DRAWN = 200
# The joints at the middle of each spherical limb, whose two branches meet at a quarter turn
# (motion_planning_core tests/common off_the_branch_seams): a start stands at least a quarter
# of pi from it.
SEAM_JOINTS = (1, 5)
# The link the arm's last joint moves, to which the grasp link is fixed.
LAST_JOINT_LINK = moved_link(OPENARM_V2, ARM.joints[-1])
# Where the arm's reach starts: the origin of its second link, the link its second joint
# moves, 0.6 m from the grasp point with the arm straight (draw_arm_planning.py's
# SHORT_REACH_M).
SHOULDER_LINK = moved_link(OPENARM_V2, ARM.joints[1])
# How long a line is, and how far it turns.
LINE_M = (0.05, 0.3)
# A line that neither moves nor turns.
STANDING_STILL = {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}
# How long a line past the arm's reach is: half a step off a whole number of steps. A line of a
# whole number of steps takes one step more or fewer by the last bit of each side's forward
# kinematics, so its fractions differ with no difference of reach.
PAST_THE_REACH_M = 0.305
MAX_TURN_RAD = 0.5
# The side of a box and of a collision object on a line, or about a start, in voxels.
BLOCK_VOXELS = 3
# A line into a block: at least 20 cm long, the block about its point at BLOCK_SHARE of it, at
# least 15 cm from the start. The block's near face stands at most 2 voxels, 8 cm, nearer on
# each axis, 7 cm from the start.
BLOCKED_LINE_M = (0.2, 0.3)
BLOCK_SHARE = 0.75


def off_the_branch_seams(rng, limits):
    """A configuration drawn inside `limits`, each joint of SEAM_JOINTS at least a quarter of
    pi from a quarter turn."""
    while True:
        q = configuration(rng, limits)
        if all(abs(math.cos(q[k])) >= math.sqrt(0.5) for k in SEAM_JOINTS):
            return q


def direction(rng):
    """A unit vector drawn uniformly."""
    drawn = [rng.gauss(0, 1) for _ in range(3)]
    norm = math.sqrt(sum(d * d for d in drawn))
    return [d / norm for d in drawn]


def rotation_vector(rng, most):
    """A rotation vector about an axis drawn uniformly, turning at most `most`."""
    turn = rng.uniform(0, most)
    return [turn * axis for axis in direction(rng)]


def line(rng, lengths=LINE_M, most=MAX_TURN_RAD):
    """An offset of a length drawn in `lengths`, in a direction drawn uniformly, turning at
    most `most`."""
    length = rng.uniform(*lengths)
    return {
        "translation": [length * d for d in direction(rng)],
        "rotation": rotation_vector(rng, most),
    }


def position_at(model, link, start):
    """The position of `link` with the arm at `start`."""
    return link_pose(model, link, dict(zip(ARM.joints, start)))[1]


def line_to_a_block(rng, model, limits):
    """A start, a line of the grasp point from it, and the line's point at BLOCK_SHARE."""
    start = off_the_branch_seams(rng, limits)
    offset = line(rng, BLOCKED_LINE_M)
    at = position_at(model, ARM.grasp_link, start)
    return start, offset, [p + BLOCK_SHARE * d for p, d in zip(at, offset["translation"])]


def block_of_voxels(point):
    """The box of BLOCK_VOXELS on a side whose middle voxel holds `point`, as the keys of its
    first and its last voxel on each axis."""
    keys = [math.floor(c / VOXEL_SIZE) for c in point]
    reach = BLOCK_VOXELS // 2
    return {"first": [k - reach for k in keys], "last": [k + reach for k in keys]}


def block_object(point):
    """The collision object `block`, a cube of BLOCK_VOXELS voxels a side centered at
    `point`."""
    return {
        "id": "block",
        "operation": "add",
        "position": point,
        "orientation": IDENTITY,
        "shapes": [shape("box", [BLOCK_VOXELS * VOXEL_SIZE] * 3)],
    }


def case(rng, start, offsets, avoid_collisions, boxes=(), objects=(), link=ARM.grasp_link):
    return {
        "start": start,
        "link": link,
        "offsets": offsets,
        "boxes": list(boxes),
        "objects": list(objects),
        "attached": [],
        "allowed": [],
        "avoid_collisions": avoid_collisions,
        "seed": rng.randint(1, 2**31 - 1),
        "ours": None,
    }


def away_from_the_shoulder(rng, model, start):
    """The longest line from the grasp point's pose at `start` straight away from the
    shoulder, turning as a drawn line turns."""
    at = position_at(model, ARM.grasp_link, start)
    shoulder = position_at(model, SHOULDER_LINK, start)
    away = [a - s for a, s in zip(at, shoulder)]
    norm = math.sqrt(sum(a * a for a in away))
    return {
        "translation": [PAST_THE_REACH_M * a / norm for a in away],
        "rotation": rotation_vector(rng, MAX_TURN_RAD),
    }


def two_lines(rng):
    """Two lines in turn, the second from the end of the first, each rotation vector turning
    at most half of MAX_TURN_RAD from the start, so a line turns at most MAX_TURN_RAD."""
    first = line(rng, most=MAX_TURN_RAD / 2)
    second = line(rng, most=MAX_TURN_RAD / 2)
    return [
        first,
        {
            "translation": [a + b for a, b in zip(first["translation"], second["translation"])],
            "rotation": second["rotation"],
        },
    ]


def round_of_cases(rng, model, limits, k):
    """One case of each draw, the `k`th round: as the module says."""
    start = lambda: off_the_branch_seams(rng, limits)
    checked = k % 2 == 0
    free = case(rng, start(), [line(rng)], checked)
    past_start = start()
    past_the_reach = case(rng, past_start, [away_from_the_shoulder(rng, model, past_start)], False)
    box_start, box_line, box_point = line_to_a_block(rng, model, limits)
    into_a_box = case(rng, box_start, [box_line], True, boxes=[block_of_voxels(box_point)])
    object_start, object_line, object_point = line_to_a_block(rng, model, limits)
    into_an_object = case(
        rng, object_start, [object_line], True, objects=[block_object(object_point)]
    )
    turn = {"translation": [0.0, 0.0, 0.0], "rotation": rotation_vector(rng, MAX_TURN_RAD)}
    contact_start = start()
    contact_point = position_at(model, ARM.grasp_link, contact_start)
    return [
        free,
        past_the_reach,
        into_a_box,
        {**into_a_box, "avoid_collisions": False},
        into_an_object,
        case(rng, start(), [turn], checked),
        case(rng, start(), two_lines(rng), checked),
        case(rng, start(), [STANDING_STILL, line(rng)], checked),
        case(rng, start(), [line(rng)], checked, link=LAST_JOINT_LINK),
        case(rng, contact_start, [line(rng)], True, boxes=[block_of_voxels(contact_point)]),
    ]


def drawn_cases(rng, model, limits, count):
    """The first `count` cases of the rounds in turn."""
    cases = []
    k = 0
    while len(cases) < count:
        cases.extend(round_of_cases(rng, model, limits, k))
        k += 1
    return cases[:count]


def fraction(answered):
    """Our fraction of the answered case, none where we refuse its start."""
    return answered["ours"].get("fraction")


def whole(answered):
    return fraction(answered) == 1.0


def ended_along(answered):
    """Whether the path of the answered case leaves its start and ends before its last pose."""
    share = fraction(answered)
    return share is not None and 0.0 < share < 1.0


def in_place(answered):
    return all(offset["translation"] == [0.0, 0.0, 0.0] for offset in answered["offsets"])


def follows_the_grasp_point(answered):
    return answered["link"] == ARM.grasp_link


def one_free_line(answered):
    """Whether the answered case is one line of the grasp point that moves, among nothing."""
    return (
        len(answered["offsets"]) == 1
        and follows_the_grasp_point(answered)
        and not in_place(answered)
        and not answered["boxes"]
        and not answered["objects"]
    )


# Each behavior the scenario keeps a case of, by the answer that shows it, as the parity test
# names them; the first line a box ends and the same line unchecked go together (`box_pair`).
def two_lines_that_move(c):
    """Whether `c` holds two lines and its first leaves the start."""
    return len(c["offsets"]) == 2 and c["offsets"][0] != STANDING_STILL


BEHAVIORS = {
    "a free line whole": lambda c: c["avoid_collisions"] and one_free_line(c) and whole(c),
    "a line past the reach": lambda c: not c["avoid_collisions"]
    and one_free_line(c)
    and fraction(c) < 1.0,
    "a line that a collision object ends": lambda c: bool(c["objects"])
    and c["avoid_collisions"]
    and ended_along(c),
    "a turn in place": lambda c: in_place(c) and whole(c),
    "two lines whole": lambda c: two_lines_that_move(c) and whole(c),
    "a line that stands still, then a line": lambda c: c["offsets"][0] == STANDING_STILL
    and whole(c),
    "two lines, the second ended": lambda c: two_lines_that_move(c)
    and ended_along(c)
    and fraction(c) >= 0.5,
    "a line of the last joint's link": lambda c: not follows_the_grasp_point(c) and whole(c),
    "a start in contact": lambda c: c["ours"].get("failure") == "start_in_collision",
}


def box_pair(answered):
    """The place of the first of `answered` that a box ends and whose next is the same line
    unchecked, whole."""
    for k, (blocked, unchecked) in enumerate(itertools.pairwise(answered)):
        if (
            blocked["boxes"]
            and blocked["avoid_collisions"]
            and ended_along(blocked)
            and not unchecked["avoid_collisions"]
            and unchecked["offsets"] == blocked["offsets"]
            and whole(unchecked)
        ):
            return k
    raise SystemExit(f"none of the first {DRAWN} cases shows a line a box ends, whole unchecked")


def first_of(answered, behavior, shows):
    """The place of the first of `answered` that shows `behavior`."""
    for k, one in enumerate(answered):
        if shows(one):
            return k
    raise SystemExit(f"none of the first {DRAWN} cases shows {behavior}")


def picked(answered):
    """The first of `answered` to show each behavior, in their order among them."""
    pair = box_pair(answered)
    places = {pair, pair + 1} | {
        first_of(answered, behavior, shows) for behavior, shows in BEHAVIORS.items()
    }
    return [answered[k] for k in sorted(places)]


def scenario(drawn, min_clearance):
    """The scenario of the cases it picks, or with `drawn` of the first `drawn`, at
    `min_clearance`."""
    rng = generator(NAME)
    model = robot(OPENARM_V2)
    limits = [joint_limits(OPENARM_V2, joint) for joint in ARM.joints]
    cases = drawn_cases(rng, model, limits, drawn or DRAWN)
    voxels = {"voxel_size": VOXEL_SIZE, "map_half_side": MAP_HALF_SIDE}
    header = {
        "kind": "cartesian_paths",
        "description": OPENARM_V2,
        "group": ARM.name,
    }
    answers = run_example(
        example("cartesian_scenes"),
        {**header, **voxels, "min_clearance": min_clearance, "cases": cases},
    )
    answered = [{**one, "ours": ours} for one, ours in zip(cases, answers["ours"], strict=True)]
    return {
        **header,
        **voxels,
        "min_clearance": min_clearance,
        "step": answers["step"],
        "cases": answered if drawn else picked(answered),
    }


def main():
    parser = options(__doc__, 0, "drawn cases kept in place of those it picks; 0 picks")
    parser.add_argument(
        "--min-clearance",
        type=float,
        default=MIN_CLEARANCE,
        metavar="METERS",
        help="the clearance each path keeps: min_clearance_m, and MoveIt's link padding",
    )
    args = parser.parse_args()
    if not 0 <= args.min_clearance < math.inf:
        parser.error(f"--min-clearance is {args.min_clearance}; give the meters, 0 or more")
    if not args.drawn and args.min_clearance != MIN_CLEARANCE:
        parser.error(
            "the committed scenario keeps a clearance of 0, the paths its cases are picked by: "
            "give --drawn 200 with --min-clearance"
        )
    write(NAME, scenario(args.drawn, args.min_clearance))


if __name__ == "__main__":
    main()
