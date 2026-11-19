# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the laser projection scenario of the ROS reference harness.

pointcloud_to_laserscan projects each cloud with the parameters with which the room
dataset scenarios project their depth frames (LASER_SCAN of draw_room_dataset.py). Each
cloud holds the points of one of these cases, in the scan's frame, each coordinate a
single precision value:

- scattered: points drawn inside the height band, the range limits and the fan;
- nearest: points of one bearing at several ranges, far to near and near to far, so the
  nearest one gives the range of its bin whatever their order;
- band: points at the single precision value nearest each end of the height band, and at
  the next value beyond it;
- range: points straight ahead at the single precision value nearest each range limit,
  and at the next value beyond it;
- fan: points a hair inside and a hair outside each end of the fan;
- bins: points a hair either side of the boundary between two bins, which the
  truncation of the callback's bin index splits;
- behind: points behind and beside the scan's origin, outside the fan;
- empty: no point, so every bin holds +inf.

Run it from any directory:

    uv run ros_reference/tools/draw_laser_projection.py
    uv run ros_reference/tools/draw_laser_projection.py --drawn 5000

The second draws a large set for a local run, which the README describes.
"""

import math
import struct

from draw_room_dataset import LASER_SCAN
from drawing import generator, options, write

NAME = "laser_projection_room_dataset"
DRAWN_POINTS = 40
# The height of a point of a case that does not test the band (m).
HEIGHT = 0.5
# The range of a point of a case that tests a bearing (m).
RANGE = 3.0
# How far a point of the fan and bins cases lies from the bearing it tests (rad).
HAIR = 2e-6
NEAREST_BEARING = 0.1
NEAREST_RANGES = (5.0, 3.0, 4.0, 1.0, 2.0)
BOUNDARY_BINS = (1, 57, 110, 218)


def single(x):
    """`x` rounded to single precision."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def next_single(x, direction):
    """The single precision value next to the positive single precision value `x`, above
    it for a positive `direction` and below it for a negative one."""
    if x <= 0 or single(x) != x:
        raise SystemExit(f"{x} is not a positive single precision value")
    pattern = struct.unpack("<I", struct.pack("<f", x))[0] + (1 if direction > 0 else -1)
    return struct.unpack("<f", struct.pack("<I", pattern))[0]


def at_bearing(bearing, distance, height=HEIGHT):
    """The point at `distance` along `bearing` on the floor plane, at `height`."""
    return [
        single(distance * math.cos(bearing)),
        single(distance * math.sin(bearing)),
        single(height),
    ]


def scattered(rng, drawn):
    fan = (LASER_SCAN["angle_min"], LASER_SCAN["angle_max"])
    ranges = (LASER_SCAN["range_min"] + 0.05, LASER_SCAN["range_max"] - 0.5)
    heights = (LASER_SCAN["min_height"] + 0.05, LASER_SCAN["max_height"] - 0.05)
    return [
        at_bearing(rng.uniform(*fan), rng.uniform(*ranges), rng.uniform(*heights))
        for _ in range(drawn)
    ]


def nearest():
    return [at_bearing(NEAREST_BEARING, distance) for distance in NEAREST_RANGES] + [
        at_bearing(NEAREST_BEARING, distance) for distance in reversed(NEAREST_RANGES)
    ]


def band():
    low, high = single(LASER_SCAN["min_height"]), single(LASER_SCAN["max_height"])
    heights = [
        low,
        next_single(low, -1),
        next_single(low, 1),
        high,
        next_single(high, 1),
        next_single(high, -1),
    ]
    return [[single(RANGE), single(0.1 * index), height] for index, height in enumerate(heights)]


def range_limits():
    low, high = single(LASER_SCAN["range_min"]), single(LASER_SCAN["range_max"])
    distances = [
        low,
        next_single(low, -1),
        next_single(low, 1),
        high,
        next_single(high, 1),
        next_single(high, -1),
    ]
    return [[distance, 0.0, single(HEIGHT)] for distance in distances]


def fan():
    ends = (LASER_SCAN["angle_min"], LASER_SCAN["angle_max"])
    return [at_bearing(end + offset, RANGE) for end in ends for offset in (-HAIR, 0.0, HAIR)]


def bins():
    boundaries = [
        LASER_SCAN["angle_min"] + index * LASER_SCAN["angle_increment"] for index in BOUNDARY_BINS
    ]
    return [
        at_bearing(boundary + offset, RANGE) for boundary in boundaries for offset in (-HAIR, HAIR)
    ]


def behind():
    bearings = (math.pi, math.pi - 0.1, -math.pi + 0.1, math.pi / 2, -math.pi / 2)
    return [at_bearing(bearing, RANGE) for bearing in bearings]


def main():
    parser = options(__doc__, DRAWN_POINTS, "points drawn in the scattered cloud")
    drawn = parser.parse_args().drawn
    rng = generator(NAME)
    clouds = [
        scattered(rng, drawn),
        nearest(),
        band(),
        range_limits(),
        fan(),
        bins(),
        behind(),
        [],
    ]
    write(NAME, {"kind": "laser_projection", "laser_scan": LASER_SCAN, "clouds": clouds})


if __name__ == "__main__":
    main()
