# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the AMCL global localization scenario of the ROS reference harness.

Each room is drawn as walls, with the scans of its map and a drive through it from a pose
that its searches do not know. The map's scans stand at drawn poses along the room, and
the reference builds the map's grid from them as karto builds one from its scans. The
drive's scans come with the base's odometry in a frame of its own, the odometry moving as
the base moves: still, a turn in place, then on ahead and around a corner, and in the
L-shaped room back again, so each motion model moves its particles backward too. Every
ray of a scan stands at the scan's stamp and meets a wall of the room within the lidar's
range, its range the distance to the wall in single precision. Each search of a room runs
AMCL's global localization over the drive, with its own seed:

- l_room: an L-shaped room with a box, turned off the map's axes, searched with AMCL's
  differential model, and with its omnidirectional model and a resampling every second
  update;
- box_room: a rectangular room with two boxes, turned the other way, searched with the
  differential model and 30 beams a scan.

Run it from any directory:

    uv run ros_reference/tools/draw_amcl_global_localization.py
"""

import math
import struct

from drawing import generator, point_to_segment, write

NAME = "amcl_global_localization_drawn"
# The lidar: a 270-degree sweep of RAYS rays, each angle in single precision.
RAYS = 61
RANGE_MIN = 0.15
RANGE_MAX = 12.0
# The grid of the map, as the navigator's launch sets slam_toolbox's.
GRID = {"resolution": 0.05, "min_pass_through": 2, "occupancy_threshold": 0.1}
# The scan period (ns) of the drive.
PERIOD_NS = 100_000_000
# The odometry frame of the drive in the map frame.
ODOMETRY_FRAME = (1.3, -0.8, 0.6)
# Nav2 1.5.2's AMCL parameters of the navigator's launch, but for the particle counts.
AMCL = {
    "alpha1": 0.2,
    "alpha2": 0.2,
    "alpha3": 0.2,
    "alpha4": 0.2,
    "alpha5": 0.2,
    "beam_skip_distance": 0.5,
    "beam_skip_error_threshold": 0.9,
    "beam_skip_threshold": 0.3,
    "do_beamskip": False,
    "lambda_short": 0.1,
    "laser_likelihood_max_dist": 2.0,
    "laser_max_range": 100.0,
    "laser_min_range": -1.0,
    "laser_model_type": "likelihood_field",
    "max_beams": 60,
    "max_particles": 400,
    "min_particles": 100,
    "pf_err": 0.05,
    "pf_z": 0.99,
    "recovery_alpha_fast": 0.0,
    "recovery_alpha_slow": 0.0,
    "resample_interval": 1,
    "robot_model_type": "nav2_amcl::DifferentialMotionModel",
    "sigma_hit": 0.2,
    "transform_tolerance": 1.0,
    "update_min_a": 0.2,
    "update_min_d": 0.25,
    "z_hit": 0.5,
    "z_max": 0.05,
    "z_rand": 0.5,
    "z_short": 0.05,
}


def single(value):
    """`value` rounded to single precision."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


ANGLE_MIN = single(-0.75 * math.pi)
ANGLE_INCREMENT = single(1.5 * math.pi / (RAYS - 1))


def placed(point, turn, shift):
    """`point` turned by `turn` about the origin, then moved by `shift`."""
    c, s = math.cos(turn), math.sin(turn)
    return (shift[0] + c * point[0] - s * point[1], shift[1] + s * point[0] + c * point[1])


def outline(points, turn, shift):
    """The walls of the closed polygon `points`, turned and moved."""
    corners = [placed(p, turn, shift) for p in points]
    return [(corners[i], corners[(i + 1) % len(corners)]) for i in range(len(corners))]


def box(center, size):
    (cx, cy), (w, h) = center, size
    return [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2), (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]


class Room:
    """A drawn room: its outline and its boxes in room coordinates, and the frame, a turn
    about the origin and a shift, that carries the room into the map frame."""

    def __init__(self, outline_points, boxes, turn, shift):
        self.outline_points = outline_points
        self.boxes = boxes
        self.turn = turn
        self.shift = shift

    def walls(self):
        return [
            wall
            for polygon in [self.outline_points, *self.boxes]
            for wall in outline(polygon, self.turn, self.shift)
        ]

    def pose(self, x, y, yaw):
        """The pose (x, y, yaw) of room coordinates in the map frame, checked to stand in
        the room's free space, CLEARANCE from every wall."""
        inside = contains(self.outline_points, x, y) and not any(contains(b, x, y) for b in self.boxes)
        clear = all(
            point_to_segment((x, y), a, b) >= CLEARANCE
            for polygon in [self.outline_points, *self.boxes]
            for a, b in zip(polygon, polygon[1:] + polygon[:1])
        )
        assert inside and clear, f"({x}, {y}) stands off the room's free space"
        return (*placed((x, y), self.turn, self.shift), yaw + self.turn)


# How far (m) every pose of a case stands from every wall.
CLEARANCE = 0.3


def contains(polygon, x, y):
    """Whether (x, y) lies inside `polygon`, by the crossings of a ray along +x."""
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


L_ROOM = Room(
    [(0.0, 0.0), (6.0, 0.0), (6.0, 2.0), (3.0, 2.0), (3.0, 4.0), (0.0, 4.0)],
    [box((1.5, 1.0), (0.6, 0.4))],
    0.3,
    (0.5, 0.2),
)
BOX_ROOM = Room(
    [(0.0, 0.0), (5.0, 0.0), (5.0, 3.0), (0.0, 3.0)],
    [box((1.2, 2.1), (0.5, 0.5)), box((3.8, 0.8), (0.8, 0.3))],
    -0.2,
    (0.3, 1.4),
)


def hit(walls, x, y, angle):
    """The distance from (x, y) along `angle` to the nearest wall of `walls`."""
    dx, dy = math.cos(angle), math.sin(angle)
    nearest = math.inf
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        denominator = dx * ey - dy * ex
        if denominator == 0.0:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / denominator
        u = ((x1 - x) * dy - (y1 - y) * dx) / denominator
        if t > 0.0 and 0.0 <= u <= 1.0:
            nearest = min(nearest, t)
    return nearest


def ranges(walls, pose):
    """The ranges of a scan of `walls` from `pose`, each in single precision."""
    x, y, yaw = pose
    values = []
    for i in range(RAYS):
        distance = hit(walls, x, y, yaw + ANGLE_MIN + i * ANGLE_INCREMENT)
        assert RANGE_MIN < distance < RANGE_MAX, f"a ray from {pose} meets no wall within range"
        values.append(single(distance))
    return values


def drive_poses(room, start, steps):
    """The base's pose in the map frame at each scan of a drive in `room` from `start`
    (room coordinates), each step `(ahead, turn)` the motion since the scan before."""
    x, y, yaw = start
    poses = [room.pose(x, y, yaw)]
    for ahead, turn in steps:
        yaw += turn
        x += ahead * math.cos(yaw)
        y += ahead * math.sin(yaw)
        poses.append(room.pose(x, y, yaw))
    return poses


def in_odometry(pose):
    """`pose` of the map frame in the odometry frame."""
    ox, oy, oyaw = ODOMETRY_FRAME
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = pose[0] - ox, pose[1] - oy
    return [c * dx + s * dy, -s * dx + c * dy, pose[2] - oyaw]


# The map's scans of each room, in room coordinates, with a heading.
L_MAP = (
    [(x, 0.5, 0.0) for x in (0.6, 1.8, 3.0, 4.2, 5.4)]
    + [(0.6, y, math.pi / 2) for y in (1.6, 2.6, 3.6)]
    + [(2.4, 3.4, math.pi), (2.2, 1.6, 0.0), (4.6, 1.6, math.pi), (1.5, 2.4, -math.pi / 2)]
)
BOX_MAP = (
    [(x, 1.5, 0.0) for x in (0.5, 1.8, 3.1, 4.5)]
    + [(x, 0.4, math.pi) for x in (4.6, 2.5, 0.5)]
    + [(x, 2.6, 0.0) for x in (0.4, 2.5, 4.6)]
)
# A drive: still, a turn in place, then on ahead and around, and in the L-shaped room back
# along its last heading.
TURN_IN_PLACE = [(0.0, 0.0)] * 4 + [(0.0, 0.3)] * 5
L_DRIVE = (
    (5.2, 1.0, math.pi / 2),
    TURN_IN_PLACE + [(0.1, 0.0)] * 24 + [(0.1, -0.2)] * 8 + [(0.1, 0.0)] * 2 + [(-0.1, 0.0)] * 10,
)
BOX_DRIVE = ((2.5, 1.5, -math.pi / 2), TURN_IN_PLACE + [(0.1, 0.0)] * 10 + [(0.1, 0.18)] * 8 + [(0.1, 0.0)] * 2)


def room_case(name, room, map_points, drive, searches):
    """A room's map scans and drive, and the searches of the drive, each `(name, AMCL's
    parameters)`, each search with a seed of its own."""
    walls = room.walls()
    return {
        "name": name,
        "map_scans": [{"pose": list(pose), "ranges": ranges(walls, pose)} for pose in (room.pose(*p) for p in map_points)],
        "scans": [
            {"stamp_ns": PERIOD_NS * (k + 1), "odometry": in_odometry(pose), "ranges": ranges(walls, pose)}
            for k, pose in enumerate(drive_poses(room, *drive))
        ],
        "searches": searches,
    }


def main():
    rng = generator(NAME)

    def search(name, amcl):
        return {"name": name, "seed": rng.randrange(2**31), "amcl": amcl}

    omnidirectional = {**AMCL, "robot_model_type": "nav2_amcl::OmniMotionModel", "resample_interval": 2}
    rooms = [
        room_case(
            "l_room",
            L_ROOM,
            L_MAP,
            L_DRIVE,
            [search("differential", AMCL), search("omnidirectional", omnidirectional)],
        ),
        room_case("box_room", BOX_ROOM, BOX_MAP, BOX_DRIVE, [search("thirty_beams", {**AMCL, "max_beams": 30})]),
    ]
    write(
        NAME,
        {
            "kind": "amcl_global_localization",
            "laser": {
                "angle_min": ANGLE_MIN,
                "angle_increment": ANGLE_INCREMENT,
                "rays": RAYS,
                "range_min": RANGE_MIN,
                "range_max": RANGE_MAX,
            },
            "grid": GRID,
            "rooms": rooms,
        },
    )


if __name__ == "__main__":
    main()
