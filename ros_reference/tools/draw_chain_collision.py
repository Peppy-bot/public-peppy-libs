# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the chain collision scenarios of the ROS reference harness, and the model
directory of the robot dressed in primitives that one of them names: its URDF, its SRDF and
the mesh of its wedge.

Each arm of OpenArm v2, dressed in its collision meshes, and the arm of PRIMITIVE_ARM,
dressed in boxes, cylinders and spheres drawn about any origin and in a wedge mesh
turned about its origin, is posed at its zero configuration, its lower limits, its
upper limits, and configurations drawn uniformly inside its limits, one by default.
At each configuration, POINTS_PER_LINK probe points are drawn uniformly in the frame of
each link the arm carries, inside the bounds of that link's collision elements grown by
MARGIN on each side, so they fall inside the link, near it and outside it; and at the
first, for each padding and each primitive, a point SURFACE_STEP inside and one
SURFACE_STEP outside a drawn point of the surface of the primitive grown by the padding.
A probe lies in its link's frame, so it tests the same body at every configuration. The
robot filter grows the links by each of PADDINGS: none, an offset alone, and a scale
with an offset. CHOMP's collision checker places each carried link's collision spheres
at each of SPHERE_PADDINGS: none, and the clearance of OpenArm v2's launch. Run it from
any directory:

    uv run ros_reference/tools/draw_chain_collision.py
    uv run ros_reference/tools/draw_chain_collision.py --drawn 50

The second draws a large set for a local run, which the README describes.
"""

import math
import struct

from robot_description_py import CLOSED, OPEN

from drawing import (
    CONFIGURATIONS_DRAWN,
    OPENARM_V2,
    REPOSITORY,
    arms,
    chain_links,
    configurations,
    element_to_link,
    generator,
    joint_limits,
    link_bounds,
    meshes,
    model_file,
    options,
    robot,
    write,
)

DRAWN_CONFIGURATIONS = 1
POINTS_PER_LINK = 4
MARGIN = 0.05
PADDINGS = [
    {"scale": 1.0, "offset": 0.0},
    {"scale": 1.0, "offset": 0.03},
    {"scale": 1.1, "offset": 0.01},
]
# The paddings of the collision spheres, in meters: none, and the min_clearance_m of
# OpenArm v2's motion planner (launchers-hub openarm/fragments/motion_planner.json5).
SPHERE_PADDINGS = [0.0, 0.04]

OPENARM = "chain_collision_openarm_v2"

PRIMITIVES = "chain_collision_primitive_arm"
# The robot dressed in primitives, by its name, its model directory, and the joints of its
# arm.
PRIMITIVE_ROBOT = "primitive_arm"
PRIMITIVE_ARM = f"ros_reference/robots/{PRIMITIVE_ROBOT}"
PRIMITIVE_JOINTS = 7
# Its arm, by its group, the chain from BASE_LINK to TOOL_LINK, its grasp point, and its
# gripper, by its group, the finger that slides off the arm's last link on FINGER_JOINT,
# closed at the start of FINGER_TRAVEL and open at its end.
ARM_GROUP = "arm"
BASE_LINK = "base"
TOOL_LINK = "tool"
GRIPPER_GROUP = "gripper"
FINGER_JOINT = "finger_joint"
FINGER_TRAVEL = (0.0, 0.05)
# The link of PRIMITIVE_ARM that carries the wedge mesh, and the mesh: a square of side
# 0.1 at x = 0 and a point at x = 1, which the URDF scales by WEDGE_SCALE.
WEDGE_LINK = "link_4"
WEDGE = "wedge.stl"
WEDGE_SCALE = 0.1
# How far inside and outside the surface of a grown primitive a surface probe lies, in
# meters.
SURFACE_STEP = 1e-5
# The least and the largest size of a drawn primitive, and the reach of its origin from
# its link's frame, in meters.
SIZES = (0.02, 0.15)
REACH = 0.08


def carried_links(parsed, description, arm):
    """The collision-bearing links that `arm`, an arm of the model directory `description`
    whose URDF is `parsed`, carries: each link of its chain past its base link, in the
    chain's order, that holds a collision element."""
    return [
        link
        for link in chain_links(description, arm)
        if parsed.find(f"link[@name='{link}']/collision") is not None
    ]


def probe_points(rng, bounds):
    return [
        {
            "link": link,
            "position": [rng.uniform(lo - MARGIN, hi + MARGIN) for lo, hi in zip(*box)],
        }
        for link, box in bounds.items()
        for _ in range(POINTS_PER_LINK)
    ]


def grown_surface_point(rng, geometry, padding):
    """A drawn point of the surface of the primitive `geometry` grown by `padding`, in
    the primitive's frame, with the surface's outward unit normal there; None for a mesh.
    A point of a face of a box, of the side or a cap of a cylinder, lies within 90% of
    the face, the length or the radius."""
    scale, offset = padding["scale"], padding["offset"]
    inset = lambda extent: 0.9 * extent * rng.uniform(-1, 1)
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        half = [float(v) / 2 for v in box.get("size").split()]
        axis = rng.randrange(3)
        sign = rng.choice([-1.0, 1.0])
        point = [
            sign * (half[i] * scale + offset) if i == axis else inset(half[i]) for i in range(3)
        ]
        return point, [sign if i == axis else 0.0 for i in range(3)]
    if cylinder is not None:
        radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
        angle = rng.uniform(-math.pi, math.pi)
        if rng.random() < 0.5:
            radial = [math.cos(angle), math.sin(angle), 0.0]
            reach = radius * scale + offset
            return [radial[0] * reach, radial[1] * reach, inset(length / 2)], radial
        sign = rng.choice([-1.0, 1.0])
        across = 0.9 * radius * rng.random()
        cap = sign * (length * scale / 2 + offset)
        return [across * math.cos(angle), across * math.sin(angle), cap], [0.0, 0.0, sign]
    if sphere is not None:
        direction = [rng.gauss(0, 1) for _ in range(3)]
        norm = math.sqrt(sum(d * d for d in direction))
        direction = [d / norm for d in direction]
        reach = float(sphere.get("radius")) * scale + offset
        return [d * reach for d in direction], direction
    return None


def surface_probes(rng, parsed, links):
    """For each padding and each primitive of `links`, the probe points SURFACE_STEP
    inside and outside a drawn point of the surface of the grown primitive."""
    probes = []
    for padding in PADDINGS:
        for link in links:
            for collision in parsed.findall(f"link[@name='{link}']/collision"):
                drawn = grown_surface_point(rng, collision.find("geometry"), padding)
                if drawn is None:
                    continue
                point, normal = drawn
                to_link = element_to_link(collision)
                probes += [
                    {
                        "link": link,
                        "position": to_link([point[k] + normal[k] * step for k in range(3)]),
                    }
                    for step in (-SURFACE_STEP, SURFACE_STEP)
                ]
    return probes


def posed_arm(rng, description, arm, drawn):
    """`arm`, an arm of the model directory `description`, as a scenario poses it, with its
    configurations, the probe points about each link it carries at each, and the surface
    probes of their primitives at the first."""
    parsed = robot(description)
    links = carried_links(parsed, description, arm)
    bounds = {link: link_bounds(parsed, link, meshes(description)) for link in links}
    limits = [joint_limits(description, joint) for joint in arm.joints]
    return {
        "name": arm.name,
        "configurations": [
            {
                "positions": positions,
                "points": probe_points(rng, bounds)
                + (surface_probes(rng, parsed, links) if index == 0 else []),
            }
            for index, positions in enumerate(
                configurations(rng, limits, drawn)
            )
        ],
    }


def draw_primitive(rng):
    """The <collision> element of a drawn primitive: a box, a cylinder or a sphere, about
    any origin within REACH of its link's frame, in any orientation."""
    def size():
        return rng.uniform(*SIZES)

    shape = rng.choice(["box", "cylinder", "sphere"])
    if shape == "box":
        geometry = f'<box size="{size()!r} {size()!r} {size()!r}"/>'
    elif shape == "cylinder":
        geometry = f'<cylinder radius="{size() / 2!r}" length="{size()!r}"/>'
    else:
        geometry = f'<sphere radius="{size() / 2!r}"/>'
    xyz = " ".join(repr(rng.uniform(-REACH, REACH)) for _ in range(3))
    rpy = " ".join(repr(rng.uniform(-math.pi, math.pi)) for _ in range(3))
    return (
        f'    <collision><origin xyz="{xyz}" rpy="{rpy}"/>'
        f"<geometry>{geometry}</geometry></collision>\n"
    )


def draw_link(rng, name):
    """A link dressed in one or two drawn primitives, and WEDGE_LINK in the wedge too,
    turned anyhow about its origin."""
    elements = "".join(draw_primitive(rng) for _ in range(rng.choice([1, 2])))
    if name == WEDGE_LINK:
        rpy = " ".join(repr(rng.uniform(-math.pi, math.pi)) for _ in range(3))
        elements += (
            f'    <collision><origin xyz="0 0 0" rpy="{rpy}"/><geometry>'
            f'<mesh filename="package://{meshes(PRIMITIVE_ARM)}/{WEDGE}" '
            f'scale="{WEDGE_SCALE} {WEDGE_SCALE} {WEDGE_SCALE}"/></geometry></collision>\n'
        )
    return f'  <link name="{name}">\n{elements}  </link>\n'


def draw_joint(rng, name, kind, parent, child, limits):
    """A joint whose origin lies 0.2 m along z from its parent's frame, give or take
    REACH on each axis, turned anyhow, about or along a drawn unit axis."""
    xyz = " ".join(
        repr(rng.uniform(-REACH, REACH) + (0.2 if axis == 2 else 0.0)) for axis in range(3)
    )
    rpy = " ".join(repr(rng.uniform(-math.pi, math.pi)) for _ in range(3))
    axis = [rng.gauss(0, 1) for _ in range(3)]
    norm = math.sqrt(sum(a * a for a in axis))
    axis_text = " ".join(repr(a / norm) for a in axis)
    limit = (
        f'<limit lower="{limits[0]!r}" upper="{limits[1]!r}" effort="1" velocity="1"/>'
        if limits
        else ""
    )
    return (
        f'  <joint name="{name}" type="{kind}">'
        f'<parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis_text}"/>{limit}</joint>\n'
    )


def draw_primitive_arm(rng):
    """Writes the URDF of an arm of PRIMITIVE_JOINTS revolute joints from BASE_LINK, its
    links dressed in drawn primitives: `link_1` to `link_7`, TOOL_LINK fixed to `link_7`,
    a `finger` on the prismatic FINGER_JOINT off `link_7`, and a `stand` fixed to
    BASE_LINK; and its SRDF and the mesh of its wedge."""
    links = [BASE_LINK] + [f"link_{i}" for i in range(1, PRIMITIVE_JOINTS + 1)]
    text = (
        f'<?xml version="1.0"?>\n<robot name="{PRIMITIVE_ROBOT}">\n'
        f'  <link name="{BASE_LINK}"/>\n'
    )
    text += "".join(
        draw_link(rng, link) for link in links[1:] + [TOOL_LINK, "finger", "stand"]
    )
    for i in range(1, PRIMITIVE_JOINTS + 1):
        reach = rng.uniform(0.5, math.pi)
        text += draw_joint(rng, f"joint_{i}", "revolute", links[i - 1], links[i], (-reach, reach))
    text += draw_joint(rng, "tool_joint", "fixed", links[-1], TOOL_LINK, None)
    text += draw_joint(rng, FINGER_JOINT, "prismatic", links[-1], "finger", FINGER_TRAVEL)
    text += draw_joint(rng, "stand_joint", "fixed", BASE_LINK, "stand", None)
    text += "</robot>\n"
    write_model_file("urdf", text)
    write_srdf()
    write_wedge()


def write_model_file(extension, text):
    """Writes `text` into PRIMITIVE_ARM's file of `extension`."""
    path = REPOSITORY / model_file(PRIMITIVE_ARM, extension)
    path.parent.mkdir(exist_ok=True)
    path.write_text(text)
    print(f"wrote {model_file(PRIMITIVE_ARM, extension)}")


def srdf_number(value):
    """`value` as an SRDF of the repository writes a number: the shortest text that reads
    back as it, a whole number with no fraction."""
    return repr(value).removesuffix(".0")


def write_srdf():
    """Writes the SRDF of PRIMITIVE_ARM: its one arm, the chain from BASE_LINK to TOOL_LINK,
    and its gripper, FINGER_JOINT, closed and open at the ends of FINGER_TRAVEL."""
    closed, opened = (srdf_number(value) for value in FINGER_TRAVEL)
    write_model_file(
        "srdf",
        f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  The arm of primitives and a turned wedge that the chain collision scenarios dress: its one
  arm, the chain from the base to the tool, and its gripper, the one sliding finger, shut and
  open at the ends of its travel.
-->
<robot name="{PRIMITIVE_ROBOT}">
    <group name="{ARM_GROUP}">
        <chain base_link="{BASE_LINK}" tip_link="{TOOL_LINK}"/>
    </group>
    <group name="{GRIPPER_GROUP}">
        <joint name="{FINGER_JOINT}"/>
    </group>
    <group_state name="{CLOSED}" group="{GRIPPER_GROUP}">
        <joint name="{FINGER_JOINT}" value="{closed}"/>
    </group_state>
    <group_state name="{OPEN}" group="{GRIPPER_GROUP}">
        <joint name="{FINGER_JOINT}" value="{opened}"/>
    </group_state>
    <end_effector name="hand" parent_link="{TOOL_LINK}" group="{GRIPPER_GROUP}" parent_group="{ARM_GROUP}"/>
</robot>
""",
    )


def write_wedge():
    """Writes the binary STL of the wedge: its square base, two triangles, and four
    triangles to its point."""
    base = [(0.0, -0.05, -0.05), (0.0, 0.05, -0.05), (0.0, 0.05, 0.05), (0.0, -0.05, 0.05)]
    point = (1.0, 0.0, 0.0)
    triangles = [(base[0], base[2], base[1]), (base[0], base[3], base[2])] + [
        (base[i], base[(i + 1) % 4], point) for i in range(4)
    ]
    data = bytes(80) + struct.pack("<I", len(triangles))
    for triangle in triangles:
        data += struct.pack("<3f", 0.0, 0.0, 0.0)
        for corner in triangle:
            data += struct.pack("<3f", *corner)
        data += bytes(2)
    path = REPOSITORY / meshes(PRIMITIVE_ARM) / WEDGE
    path.parent.mkdir(exist_ok=True)
    path.write_bytes(data)
    print(f"wrote {meshes(PRIMITIVE_ARM)}/{WEDGE}")


def main():
    drawn = options(__doc__, DRAWN_CONFIGURATIONS, CONFIGURATIONS_DRAWN).parse_args().drawn
    rng = generator(OPENARM)
    write(
        OPENARM,
        {
            "kind": "chain_collision",
            "description": OPENARM_V2,
            "paddings": PADDINGS,
            "sphere_paddings": SPHERE_PADDINGS,
            "arms": [posed_arm(rng, OPENARM_V2, arm, drawn) for arm in arms(OPENARM_V2)],
        },
    )

    rng = generator(PRIMITIVES)
    draw_primitive_arm(rng)
    write(
        PRIMITIVES,
        {
            "kind": "chain_collision",
            "description": PRIMITIVE_ARM,
            "paddings": PADDINGS,
            "sphere_paddings": SPHERE_PADDINGS,
            "arms": [posed_arm(rng, PRIMITIVE_ARM, arm, drawn) for arm in arms(PRIMITIVE_ARM)],
        },
    )


if __name__ == "__main__":
    main()
