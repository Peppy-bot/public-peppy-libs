# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the occupancy scenario of the ROS reference harness.

A turned sensor reads a still scene REPEATS times, which takes a voxel to each clamp of
its log-odds, and then one later cloud. The scene holds points of these cases, in the map
frame:

- scattered: points drawn within the range, outside the sphere that stands for the
  robot's own body, whose rays clear what they cross;
- occluded: a near point on the ray of a far one, so one insert both hits and crosses the
  near point's voxel, and the hit wins;
- crossed later: a point on the ray of the later cloud's point, so the later cloud
  crosses a voxel that the scene took to the upper clamp, which one miss leaves occupied;
- behind the body: a point behind the sphere and a point inside it on the first's ray,
  so one insert both cuts out and crosses the inner point's voxel, and the cut-out wins;
- straddling: two points in one voxel that the sphere's surface crosses, one inside the
  sphere and one outside it, so one insert both cuts out and hits that voxel, and the
  cut-out wins;
- inside: points drawn inside the sphere, which the updater cuts out;
- beyond: points drawn beyond the range, which clear to the range and mark nothing.

The scene holds DRAWN_POINTS points of each drawn case, scattered, inside and beyond. The
later cloud holds the point that crosses the scene's voxel and DRAWN_POINTS scattered
points. A point that lies on another's ray, and that other point, lie at least
FACE_MARGIN from every face of their voxels, and no point lies within MARGIN of the
sphere's surface, so neither the voxel that a ray crosses nor whether a point lies inside
the sphere turns on the rounding of the frame it is judged in. A last cloud, from a level
sensor, holds points beyond the range straight along +X and +Y at FACE_DISTANCE, where
FACE_DISTANCE * (1 / FACE_DISTANCE) falls short of 1: tf2's normalize, which multiplies a
point by the reciprocal of its length, then puts the point at the range one rounding
short of the voxel face at MAX_RANGE, and a division by the length puts it on the face.
Run it from any directory:

    uv run ros_reference/tools/draw_occupancy.py
    uv run ros_reference/tools/draw_occupancy.py --drawn 30

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import generator, options, write

NAME = "occupancy_room"
VOXEL_SIZE = 0.05
MAX_RANGE = 2.0
CUT_OUT = {"center": [0.4, -0.3, 0.2], "radius": 0.25}
REPEATS = 5
DRAWN_POINTS = 1
MARGIN = 1e-3
FACE_MARGIN = 5e-3
# How far each point of a straddling pair lies from the sphere's surface.
STRADDLE = 0.01
# The distances (m) from the sensor of a scattered point, of a far point whose ray
# another point lies on, and of a point beyond the range.
SCATTERED = (0.4, MAX_RANGE * 0.95)
FAR = (1.2, MAX_RANGE * 0.95)
BEYOND = (MAX_RANGE * 1.05, MAX_RANGE * 1.6)
# How far behind the sphere's far side the point behind the body lies.
BEHIND = 0.2
LEVEL = {"position": [0.0, 0.0, 0.5], "orientation": [0.0, 0.0, 0.0, 1.0]}
FACE_DISTANCE = 49.0


def rotate(q, v):
    """`v` turned by the unit quaternion `q`, [x, y, z, w]."""
    x, y, z, w = q
    t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])]
    return [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]


def unit(v):
    norm = math.sqrt(sum(c * c for c in v))
    return [c / norm for c in v]


def in_sensor(sensor, p):
    """The point `p` of the map frame in the sensor's frame."""
    x, y, z, w = sensor["orientation"]
    offset = [p[i] - sensor["position"][i] for i in range(3)]
    return rotate([-x, -y, -z, w], offset)


def past_surface(p):
    """How far `p` lies outside the sphere's surface, negative inside it."""
    return math.dist(p, CUT_OUT["center"]) - CUT_OUT["radius"]


def voxel_center(p):
    return [(math.floor(c / VOXEL_SIZE) + 0.5) * VOXEL_SIZE for c in p]


def within_voxel(p, margin):
    """Whether `p` lies at least `margin` from every face of its voxel."""
    return all(
        margin < c - math.floor(c / VOXEL_SIZE) * VOXEL_SIZE < VOXEL_SIZE - margin for c in p
    )


def along(start, end, share):
    """The point `share` of the way from `start` to `end`."""
    return [s + share * (e - s) for s, e in zip(start, end)]


def random_unit(rng):
    return unit([rng.uniform(-1, 1) for _ in range(3)])


def scattered(rng, origin, distances):
    """A point drawn over `distances` from `origin` in a direction drawn uniformly, outside
    the sphere by MARGIN."""
    while True:
        distance = rng.uniform(*distances)
        p = [o + distance * d for o, d in zip(origin, random_unit(rng))]
        if past_surface(p) > MARGIN:
            return p


def on_its_ray(rng, origin, share):
    """A far point, and the point `share` of the way from `origin` to its voxel's center,
    on the ray that the updater walks to that voxel, outside the sphere by MARGIN."""
    while True:
        far = scattered(rng, origin, FAR)
        near = along(origin, voxel_center(far), share)
        clear = all(within_voxel(p, FACE_MARGIN) for p in (far, near))
        if clear and past_surface(near) > MARGIN:
            return far, near


def behind_the_body(rng, origin):
    """A point behind the sphere as seen from `origin`, and a point inside the sphere on
    the ray that the updater walks to its voxel's center."""
    center = CUT_OUT["center"]
    while True:
        direction = unit([c - o + 0.1 * u for c, o, u in zip(center, origin, random_unit(rng))])
        distance = math.dist(origin, center) + CUT_OUT["radius"] + BEHIND
        behind = [o + distance * d for o, d in zip(origin, direction)]
        end = voxel_center(behind)
        inner = along(origin, end, math.dist(origin, center) / math.dist(origin, end))
        clear = all(within_voxel(p, FACE_MARGIN) for p in (behind, inner))
        if clear and past_surface(behind) > MARGIN and past_surface(inner) < -MARGIN:
            return behind, inner


def straddling(rng):
    """Two points in one voxel, STRADDLE inside the sphere's surface and STRADDLE outside
    it."""
    while True:
        normal = random_unit(rng)
        surface = [c + CUT_OUT["radius"] * n for c, n in zip(CUT_OUT["center"], normal)]
        if within_voxel(surface, STRADDLE + FACE_MARGIN):
            return [[s + side * STRADDLE * n for s, n in zip(surface, normal)] for side in (-1, 1)]


def inside(rng):
    """A point drawn inside the sphere, deeper than 2 * MARGIN."""
    while True:
        p = [c + rng.uniform(-1, 1) * CUT_OUT["radius"] for c in CUT_OUT["center"]]
        if past_surface(p) < -2 * MARGIN:
            return p


def beyond(rng):
    """A point beyond the range, in the sensor's frame."""
    distance = rng.uniform(*BEYOND)
    return [distance * c for c in random_unit(rng)]


def sensor_clouds(rng, sensor, drawn):
    """The scene of `sensor`, REPEATS times, and its later cloud."""
    origin = sensor["position"]
    later, crossed_later = on_its_ray(rng, origin, 0.6)
    far, near = on_its_ray(rng, origin, 0.5)
    behind, inner = behind_the_body(rng, origin)
    scene = [
        *(scattered(rng, origin, SCATTERED) for _ in range(drawn)),
        far,
        near,
        crossed_later,
        behind,
        inner,
        *straddling(rng),
        *(inside(rng) for _ in range(drawn)),
    ]
    later_cloud = [later, *(scattered(rng, origin, SCATTERED) for _ in range(drawn))]
    scene_points = [in_sensor(sensor, p) for p in scene] + [beyond(rng) for _ in range(drawn)]
    return [{"sensor": sensor, "points": scene_points}] * REPEATS + [
        {"sensor": sensor, "points": [in_sensor(sensor, p) for p in later_cloud]}
    ]


def straddles_a_face(distance):
    """Whether the point at the range of a point `distance` along an axis from the level
    sensor lies in another voxel when it is scaled by the reciprocal of `distance` than when
    it is divided by `distance`."""
    factor = 1.0 / VOXEL_SIZE
    by_reciprocal = distance * (1.0 / distance) * MAX_RANGE
    by_division = distance / distance * MAX_RANGE
    return math.floor(factor * by_reciprocal) != math.floor(factor * by_division)


def face_cloud():
    if not straddles_a_face(FACE_DISTANCE):
        raise SystemExit(f"a point {FACE_DISTANCE} m out lands in one voxel by either rounding")
    return [[FACE_DISTANCE, 0.0, 0.0], [0.0, FACE_DISTANCE, 0.0]]


def sensor(rng, position):
    return {"position": position, "orientation": unit([rng.uniform(-1, 1) for _ in range(4)])}


def main():
    parser = options(__doc__, DRAWN_POINTS, "points drawn in each scene of each drawn case")
    drawn = parser.parse_args().drawn
    rng = generator(NAME)
    clouds = sensor_clouds(rng, sensor(rng, [0.0, 0.0, 0.5]), drawn)
    clouds.append({"sensor": LEVEL, "points": face_cloud()})
    write(
        NAME,
        {
            "kind": "occupancy",
            "voxel_size": VOXEL_SIZE,
            "max_range": MAX_RANGE,
            "padding_offset": 0.0,
            "padding_scale": 1.0,
            "cut_out": CUT_OUT,
            "clouds": clouds,
        },
    )


if __name__ == "__main__":
    main()
