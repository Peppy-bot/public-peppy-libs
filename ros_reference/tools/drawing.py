"""What the scripts that write the scenarios of the ROS reference harness share: the
paths, the generator of each scenario's draws, the --drawn option, the model directory of
each robot the scenarios pose, its parsed URDF, its arms and the limits of its joints as
robot_description_py reads its URDF and SRDF, the links of an arm's chain and of its
gripper, the finger joint of a gripper that moves of its own,
the configurations of a chain, the origin of a URDF element, the bounds of a link's
collision elements, a rotation drawn uniformly, a shape of a collision object as
motion_planning writes it, a closed box mesh and an L-shaped one, the pose of a link at
joint positions, the constants of a Rust file, the running of an example of
motion_planning_core, and the writing of a scenario.

Each scenario draws from a generator seeded with its own name, so every run writes the
same scenarios, and a change to one scenario leaves the draws of the others as they are.

A scenario is written with a value on one line when the value is a scalar, an array of
scalars, or a record, an object whose members are each a scalar or an array of scalars.
Every other array or object spreads over lines, one member a line, indented by two
spaces. The write_json of the reference program lays out the reference outputs by the
same rule.
"""

import argparse
import functools
import json
import math
import operator
import random
import re
import struct
import subprocess
from pathlib import Path
from xml.etree import ElementTree

from robot_description_py import RobotDescription

SEED = 20261028
REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIOS = REPOSITORY / "ros_reference" / "scenarios"
# What --drawn counts in a script that draws the configurations of a chain.
CONFIGURATIONS_DRAWN = "configurations drawn inside the limits of each chain"


def generator(name):
    return random.Random(f"{SEED}:{name}")


def options(doc, drawn, what):
    """The options of a script whose docstring is `doc`: --drawn, how many of `what` it
    draws, `drawn` by default. A larger count draws a large set for a local run, which
    the README describes."""
    parser = argparse.ArgumentParser(description=doc.splitlines()[0])
    parser.add_argument("--drawn", type=int, default=drawn, help=what)
    return parser


# The model directories of the robots the scenarios pose, relative to the repository root,
# each holding `<name>.urdf`, `<name>.srdf` and the collision meshes under `meshes/`, `<name>`
# the directory's last part: robot_description's layout.
OPENARM_V2 = "robot_description/models/openarm_v2"
SO101 = "robot_description/models/so101"


def model_file(description, extension):
    """The file `<name>.<extension>` of the model directory `description`."""
    return f"{description}/{Path(description).name}.{extension}"


def meshes(description):
    """The directory of the collision meshes of the model directory `description`."""
    return f"{description}/meshes"


@functools.cache
def robot(description):
    """The parsed URDF of the model directory `description`."""
    return ElementTree.parse(REPOSITORY / model_file(description, "urdf")).getroot()


@functools.cache
def described(description):
    """The robot the model directory `description` describes, its URDF and its SRDF as
    robot_description_py reads them."""
    return RobotDescription.parse(
        (REPOSITORY / model_file(description, "urdf")).read_text(),
        (REPOSITORY / model_file(description, "srdf")).read_text(),
    )


def arms(description):
    """The arms of the model directory `description`, in its SRDF's order: each the SRDF's
    group of one chain, with its base link, its grasp link, the chain's tip, the joints
    between them that move of their own, base to tip, and its gripper, with the joint and
    the link of each finger."""
    return described(description).arms


def arm(description, name):
    """The arm `name` of the model directory `description`."""
    by_name = {each.name: each for each in arms(description)}
    if name not in by_name:
        raise SystemExit(f"{description} has no arm {name}: its arms are {sorted(by_name)}")
    return by_name[name]


def joint_limits(description, joint):
    """The position limits of the joint `joint` of the model directory `description`, lower
    then upper."""
    limits = described(description).joint_limits(joint)
    return limits.lower, limits.upper


def joint_named(description, name):
    """The URDF's joint `name` of the model directory `description`."""
    return next(j for j in robot(description).findall("joint") if j.get("name") == name)


def joint_above(description, link):
    """The URDF's joint of the model directory `description` whose child is `link`."""
    return next(
        j for j in robot(description).findall("joint") if j.find("child").get("link") == link
    )


def moved_link(description, joint):
    """The link the URDF's joint `joint` of the model directory `description` moves: its
    child."""
    return joint_named(description, joint).find("child").get("link")


def chain_links(description, arm):
    """The links of the chain of `arm`, an arm of the model directory `description`, past
    its base link, each the child of a joint of the URDF, base to grasp link."""
    links = []
    link = arm.grasp_link
    while link != arm.base_link:
        links.insert(0, link)
        link = joint_above(description, link).find("parent").get("link")
    return links


def finger_links(arm):
    """The links the fingers of the gripper of `arm` move, in its group's order."""
    return [finger.link for finger in arm.gripper.fingers]


def gripper_links(description, arm):
    """The links of the gripper of `arm`, an arm of the model directory `description`: the
    link its fingers hang from, then the links of its fingers."""
    fingers = finger_links(arm)
    return [joint_above(description, fingers[0]).find("parent").get("link"), *fingers]


def gripper_joint(description, arm):
    """The finger joint of the gripper of `arm`, an arm of the model directory
    `description`, that moves of its own: the one the URDF gives no mimic."""
    return next(
        finger.joint
        for finger in arm.gripper.fingers
        if joint_named(description, finger.joint).find("mimic") is None
    )


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


def configuration(rng, limits):
    """A configuration drawn uniformly inside the limits."""
    return [rng.uniform(low, high) for low, high in limits]


def configurations(rng, limits, drawn):
    """The zero configuration, the lower limits, the upper limits, and `drawn`
    configurations drawn uniformly inside the limits."""
    lower = [low for low, _ in limits]
    upper = [high for _, high in limits]
    inside = [configuration(rng, limits) for _ in range(drawn)]
    return [[0.0] * len(limits), lower, upper, *inside]


IDENTITY = [0.0, 0.0, 0.0, 1.0]


def shape(kind, dimensions=(), vertices=(), triangles=(), at=(0.0, 0.0, 0.0)):
    """A shape `at` from its object's origin, turned as its object."""
    return {
        "kind": kind,
        "dimensions": list(dimensions),
        "mesh_vertices": [c for vertex in vertices for c in vertex],
        "mesh_triangles": [i for triangle in triangles for i in triangle],
        "position": list(at),
        "orientation": IDENTITY,
    }


def box_mesh(size, at):
    """A closed box of `size` centered `at` from its object's origin, as twelve
    triangles."""
    h = [s / 2 for s in size]
    vertices = [
        (h[0] if i & 1 else -h[0], h[1] if i & 2 else -h[1], h[2] if i & 4 else -h[2])
        for i in range(8)
    ]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    triangles = [t for a, b, c, d in faces for t in ((a, b, c), (a, c, d))]
    return shape("mesh", vertices=vertices, triangles=triangles, at=at)


def l_mesh(side, width, height):
    """An L of two bars, each `side` long and `width` wide, meeting at the origin along x
    and y, from z = 0 to `height`: a closed solid, whose open corner is the square from
    `width` to `side` on x and on y."""
    corners = [(0, 0), (side, 0), (side, width), (width, width), (width, side), (0, side)]
    vertices = [(x, y, z) for z in (0.0, height) for x, y in corners]
    fan = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5)]
    caps = [t for a, b, c in fan for t in ((a, c, b), (a + 6, b + 6, c + 6))]
    sides = [
        t
        for i in range(6)
        for t in ((i, (i + 1) % 6, (i + 1) % 6 + 6), (i, (i + 1) % 6 + 6, i + 6))
    ]
    return shape("mesh", vertices=vertices, triangles=caps + sides)


IDENTITY_ROWS = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def product(a, b):
    """The product of the rotations `a` and `b`, as rows."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def turned(turn, point):
    """`point` turned by the rotation `turn`, as rows."""
    return [sum(turn[row][k] * point[k] for k in range(3)) for row in range(3)]


def about_axis(axis, angle):
    """The rotation by `angle` about `axis`, as rows (Rodrigues)."""
    norm = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / norm for a in axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def quaternion(turn):
    """The unit quaternion [x, y, z, w] of the rotation `turn`, as rows (Shepperd)."""
    (a, b, c), (d, e, f), (g, h, i) = turn
    trace = a + e + i
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        return [(h - f) / s, (c - g) / s, (d - b) / s, s / 4]
    if a >= e and a >= i:
        s = 2 * math.sqrt(1 + a - e - i)
        return [s / 4, (b + d) / s, (c + g) / s, (h - f) / s]
    if e >= i:
        s = 2 * math.sqrt(1 + e - a - i)
        return [(b + d) / s, s / 4, (f + h) / s, (c - g) / s]
    s = 2 * math.sqrt(1 + i - a - e)
    return [(c + g) / s, (f + h) / s, s / 4, (d - b) / s]


def link_pose(parsed, link, positions):
    """The rotation, as rows, and the translation of `link` in the frame of the URDF
    root, each joint that `positions` names at its position and every other at 0."""
    joint = next(
        (j for j in parsed.findall("joint") if j.find("child").get("link") == link), None
    )
    if joint is None:
        return IDENTITY_ROWS, [0.0, 0.0, 0.0]
    parent_turn, parent_at = link_pose(parsed, joint.find("parent").get("link"), positions)
    xyz, turn = origin(joint)
    kind = joint.get("type")
    if kind == "revolute":
        axis = [float(v) for v in joint.find("axis").get("xyz").split()]
        turn = product(turn, about_axis(axis, positions.get(joint.get("name"), 0.0)))
    elif kind != "fixed":
        raise SystemExit(f"{joint.get('name')} is {kind}; the scenario poses revolute joints")
    at = [p + q for p, q in zip(parent_at, turned(parent_turn, xyz))]
    return product(parent_turn, turn), at


def pose_by_link(parsed, link, positions, offset):
    """The pose of a frame `offset` from the origin of `link` along its axes, turned as
    it, with the joints at `positions`."""
    turn, at = link_pose(parsed, link, positions)
    return {
        "position": [p + q for p, q in zip(at, turned(turn, offset))],
        "orientation": quaternion(turn),
    }


def unit_quaternion(rng):
    """A rotation drawn uniformly, as the unit quaternion [x, y, z, w] (Shoemake)."""
    u1, u2, u3 = rng.random(), rng.random(), rng.random()
    a, b = math.sqrt(1 - u1), math.sqrt(u1)
    return [
        a * math.sin(2 * math.pi * u2),
        a * math.cos(2 * math.pi * u2),
        b * math.sin(2 * math.pi * u3),
        b * math.cos(2 * math.pi * u3),
    ]


def stl_vertices(path):
    data = path.read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    return [
        struct.unpack_from("<3f", data, 84 + facet * 50 + 12 + corner * 12)
        for facet in range(count)
        for corner in range(3)
    ]


def rotation(roll, pitch, yaw):
    """The rotation of a URDF origin's rpy, as rows."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def origin(element):
    """The translation and the rotation, as rows, of the origin of the URDF element
    `element`, a joint or a collision element: the identity where it has none."""
    found = element.find("origin")

    def values(key):
        text = found.get(key, "0 0 0") if found is not None else "0 0 0"
        return [float(v) for v in text.split()]

    return values("xyz"), rotation(*values("rpy"))


def element_to_link(collision):
    """The map of a point from the frame of the collision element `collision` to the frame
    of its link."""
    xyz, turn = origin(collision)
    return lambda point: [
        sum(turn[row][k] * point[k] for k in range(3)) + xyz[row] for row in range(3)
    ]


def element_corners(geometry, meshes):
    """The points that bound a collision element's geometry in its own frame: a mesh's
    vertices, scaled, or the corners of a primitive's box."""
    mesh = geometry.find("mesh")
    if mesh is not None:
        scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
        basename = mesh.get("filename").rsplit("/", 1)[-1]
        return [
            [vertex[axis] * scale[axis] for axis in range(3)]
            for vertex in stl_vertices(REPOSITORY / meshes / basename)
        ]
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        half = [float(v) / 2 for v in box.get("size").split()]
    elif cylinder is not None:
        radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
        half = [radius, radius, length / 2]
    elif sphere is not None:
        radius = float(sphere.get("radius"))
        half = [radius, radius, radius]
    else:
        raise SystemExit("a collision geometry is a mesh, a box, a cylinder or a sphere")
    return [
        [sign_x * half[0], sign_y * half[1], sign_z * half[2]]
        for sign_x in (-1, 1)
        for sign_y in (-1, 1)
        for sign_z in (-1, 1)
    ]


def link_bounds(parsed, link, meshes):
    """The bounds, in the link's frame, of every collision element of the link."""
    points = [
        element_to_link(collision)(corner)
        for collision in parsed.findall(f"link[@name='{link}']/collision")
        for corner in element_corners(collision.find("geometry"), meshes)
    ]
    if not points:
        raise SystemExit(f"{link} has no collision element")
    return [min(p[axis] for p in points) for axis in range(3)], [
        max(p[axis] for p in points) for axis in range(3)
    ]


def in_order(values):
    """The sum of `values` added one by one in their order, as Rust adds the terms of a
    constant and the items of an iterator of f64."""
    return functools.reduce(operator.add, values, -0.0)


def rust_constants(path):
    """The f64 and usize constants that the Rust file at `path` declares at the top level,
    by name: each one a number literal, or number literals joined by " + " and added in
    their order."""
    declared = re.findall(
        r"^(?:pub )?const (\w+): (?:f64|usize) = (.+);$", path.read_text(), flags=re.MULTILINE
    )
    return {name: in_order(map(float, value.split(" + "))) for name, value in declared}


def _is_flat(value):
    if isinstance(value, list):
        return not any(isinstance(member, (dict, list)) for member in value)
    return not isinstance(value, dict)


def _is_one_line(value):
    if isinstance(value, dict):
        return all(_is_flat(member) for member in value.values())
    return _is_flat(value)


def _layout(value, indent):
    """The text of `value`, by the rule of the module's docstring, with its later lines
    indented by `indent`."""
    if _is_one_line(value):
        return json.dumps(value, separators=(",", ":"))
    inner = indent + "  "
    if isinstance(value, dict):
        members = [
            f"{inner}{json.dumps(key)}: {_layout(member, inner)}" for key, member in value.items()
        ]
        return "{\n" + ",\n".join(members) + "\n" + indent + "}"
    members = [f"{inner}{_layout(member, inner)}" for member in value]
    return "[\n" + ",\n".join(members) + "\n" + indent + "]"


def write(name, scenario):
    path = SCENARIOS / f"{name}.json"
    path.write_text(_layout(scenario, "") + "\n")
    print(f"wrote {path.relative_to(REPOSITORY)}")
