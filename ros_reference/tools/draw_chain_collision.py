"""Writes the chain collision scenarios of the ROS reference harness, and the URDF of the
arm dressed in primitives that one of them names, with the mesh of its wedge.

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
with an offset. Run it from any directory:

    python3 ros_reference/tools/draw_chain_collision.py
    python3 ros_reference/tools/draw_chain_collision.py --drawn 50

The second draws a large set for a local run, which the README describes.
"""

import math
import struct

from drawing import (
    REPOSITORY,
    configurations,
    generator,
    joint_limits,
    openarm_joints,
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

OPENARM = "chain_collision_openarm_v2"
OPENARM_URDF = "openarm_description/assets/openarm_v20.urdf"
OPENARM_MESHES = "openarm_description/assets/meshes_v20"

PRIMITIVES = "chain_collision_primitive_arm"
PRIMITIVE_ARM = "ros_reference/robots/primitive_arm.urdf"
PRIMITIVE_MESHES = "ros_reference/robots/meshes"
PRIMITIVE_JOINTS = 7
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


def openarm_carried_links(side):
    """The collision-bearing links an arm of OpenArm v2 carries: the child of each joint,
    the end effector's base being the child of the last."""
    return [f"openarm_{side}_link{i}" for i in range(1, 7)] + [
        f"openarm_{side}_ee_base_link"
    ]


def primitive_carried_links():
    """The collision-bearing links the arm of PRIMITIVE_ARM carries: the child of each
    joint, and the tool fixed to the last."""
    return [f"link_{i}" for i in range(1, PRIMITIVE_JOINTS + 1)] + ["tool"]


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


def element_to_link(collision):
    """The map of a point from the frame of the collision element `collision` to the frame
    of its link."""
    origin = collision.find("origin")

    def values(key):
        text = origin.get(key, "0 0 0") if origin is not None else "0 0 0"
        return [float(v) for v in text.split()]

    xyz, turn = values("xyz"), rotation(*values("rpy"))
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
    if box is not None:
        half = [float(v) / 2 for v in box.get("size").split()]
    else:
        shape = geometry.find("cylinder")
        if shape is None:
            shape = geometry.find("sphere")
        radius = float(shape.get("radius"))
        half = [radius, radius, float(shape.get("length", 2 * radius)) / 2]
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


def chain(rng, parsed, meshes, name, joints, tip_link, links, drawn):
    """A chain of a scenario, with its configurations, the probe points about `links` at
    each, and the surface probes of its primitives at the first."""
    bounds = {link: link_bounds(parsed, link, meshes) for link in links}
    limits = [joint_limits(parsed, joint) for joint in joints]
    return {
        "name": name,
        "joints": joints,
        "tip_link": tip_link,
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
            f'<mesh filename="package://ros_reference/robots/meshes/{WEDGE}" '
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
    """Writes the URDF of an arm of PRIMITIVE_JOINTS revolute joints from `base`, its
    links dressed in drawn primitives: `link_1` to `link_7`, a `tool` fixed to `link_7`,
    a `finger` on a prismatic joint off `link_7`, and a `stand` fixed to `base`."""
    links = ["base"] + [f"link_{i}" for i in range(1, PRIMITIVE_JOINTS + 1)]
    text = '<?xml version="1.0"?>\n<robot name="primitive_arm">\n  <link name="base"/>\n'
    text += "".join(draw_link(rng, link) for link in links[1:] + ["tool", "finger", "stand"])
    for i in range(1, PRIMITIVE_JOINTS + 1):
        reach = rng.uniform(0.5, math.pi)
        text += draw_joint(rng, f"joint_{i}", "revolute", links[i - 1], links[i], (-reach, reach))
    text += draw_joint(rng, "tool_joint", "fixed", links[-1], "tool", None)
    text += draw_joint(rng, "finger_joint", "prismatic", links[-1], "finger", (0.0, 0.05))
    text += draw_joint(rng, "stand_joint", "fixed", "base", "stand", None)
    text += "</robot>\n"
    path = REPOSITORY / PRIMITIVE_ARM
    path.parent.mkdir(exist_ok=True)
    path.write_text(text)
    print(f"wrote {PRIMITIVE_ARM}")
    write_wedge()


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
    path = REPOSITORY / PRIMITIVE_MESHES / WEDGE
    path.parent.mkdir(exist_ok=True)
    path.write_bytes(data)
    print(f"wrote {PRIMITIVE_MESHES}/{WEDGE}")


def main():
    drawn = options(__doc__, DRAWN_CONFIGURATIONS).parse_args().drawn
    rng = generator(OPENARM)
    parsed = robot(OPENARM_URDF)
    write(
        OPENARM,
        {
            "kind": "chain_collision",
            "urdf": OPENARM_URDF,
            "meshes": OPENARM_MESHES,
            "paddings": PADDINGS,
            "chains": [
                chain(
                    rng,
                    parsed,
                    OPENARM_MESHES,
                    side,
                    openarm_joints(side),
                    f"openarm_{side}_tcp",
                    openarm_carried_links(side),
                    drawn,
                )
                for side in ["left", "right"]
            ],
        },
    )

    rng = generator(PRIMITIVES)
    draw_primitive_arm(rng)
    parsed = robot(PRIMITIVE_ARM)
    write(
        PRIMITIVES,
        {
            "kind": "chain_collision",
            "urdf": PRIMITIVE_ARM,
            "meshes": PRIMITIVE_MESHES,
            "paddings": PADDINGS,
            "chains": [
                chain(
                    rng,
                    parsed,
                    PRIMITIVE_MESHES,
                    "arm",
                    [f"joint_{i}" for i in range(1, PRIMITIVE_JOINTS + 1)],
                    "tool",
                    primitive_carried_links(),
                    drawn,
                )
            ],
        },
    )


if __name__ == "__main__":
    main()
