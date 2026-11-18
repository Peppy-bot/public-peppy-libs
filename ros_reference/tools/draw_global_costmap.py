# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the global costmap scenarios of the ROS reference harness.

A scenario is a saved map, as its image and YAML file give it to map_io, the parameters
of a static, an obstacle and an inflation layer, the base's footprint with Nav2's default
padding and its pose, and one cloud of a depth frame. The map's image draws a room: walls of black pixels around a floor of white
ones, a pillar, and behind the camera, out of its view, gray pixels on either side of each
threshold of the trinary map (89 and 90 by the occupied one, 191 and 192 by the free one)
and a patch of unknown gray, so the cloud's rays clear no cell the map leaves unknown. The
camera renders, as draw_obstacle_projection.py renders it, the floor, a box in the height
band near a wall, and a box that rises through the band's top; the scene's table top above
the band is met by none of its rays.
Run it from any directory:

    uv run ros_reference/tools/draw_global_costmap.py
"""

from draw_obstacle_projection import frame, parameters
from drawing import write

NAME = "global_costmap_room"
RESOLUTION = 0.05
WIDTH = 60
HEIGHT = 50
ORIGIN = [-0.75, -1.25]
GRID = {"size_x": WIDTH, "size_y": HEIGHT, "resolution": RESOLUTION, "origin": ORIGIN}
BAND = (0.05, 0.5)
BLACK = 0
WHITE = 254
UNKNOWN_GRAY = 128
# Each pixel value beside a threshold: 89 occupied and 90 unknown, 191 unknown and 192
# free.
THRESHOLD_PIXELS = [89, 90, 191, 192]
BOXES = [
    [0.9, 1.1, -1.0, -0.8, 0.0, 0.3],
    [1.3, 1.45, 0.3, 0.45, 0.0, 0.9],
    [0.4, 0.9, 0.2, 0.6, 0.7, 0.74],
]
CAMERA = ([-0.2, -0.1, 0.55], 0.1, 0.4)
# Where the base stands, x, y and yaw: its padded footprint over the patch of unknown gray
# and the wall beside it, so its clearing frees unknown cells and keeps the wall's.
BASE = [-0.45, 0.45, 0.2]
INFLATION = {
    "enabled": True,
    "inflation_radius": 0.55,
    "custom_inscribed_radius": -1.0,
    "cost_scaling_factor": 4.0,
    "inflate_unknown": False,
    "inflate_around_unknown": False,
    "num_threads": 1,
}


def cell_of(x, y):
    """The image's column and row, from the top, of the point (x, y) of the map."""
    column = int((x - ORIGIN[0]) / RESOLUTION)
    row_from_bottom = int((y - ORIGIN[1]) / RESOLUTION)
    return column, HEIGHT - 1 - row_from_bottom


def image():
    rows = [[WHITE] * WIDTH for _ in range(HEIGHT)]
    for row in range(HEIGHT):
        rows[row][0] = rows[row][WIDTH - 1] = BLACK
    rows[0] = [BLACK] * WIDTH
    rows[HEIGHT - 1] = [BLACK] * WIDTH
    pillar = cell_of(1.6, -0.5)
    for dy in range(3):
        for dx in range(3):
            rows[pillar[1] + dy][pillar[0] + dx] = BLACK
    for i, pixel in enumerate(THRESHOLD_PIXELS):
        column, row = cell_of(-0.6, -1.0 + 0.25 * i)
        rows[row][column] = pixel
    behind = cell_of(-0.7, 0.5)
    for dy in range(5):
        for dx in range(4):
            rows[behind[1] + dy][behind[0] + dx] = UNKNOWN_GRAY
    return rows


def main():
    cloud = frame(*CAMERA, boxes=BOXES, grid=GRID, band=BAND)
    write(NAME, {
        "kind": "global_costmap",
        "map": {
            "image": image(),
            "resolution": RESOLUTION,
            "origin": [*ORIGIN, 0.0],
            "free_thresh": 0.25,
            "occupied_thresh": 0.65,
            "mode": "trinary",
            "negate": False,
        },
        "static": {
            "enabled": True,
            "subscribe_to_updates": False,
            "footprint_clearing_enabled": False,
            "restore_cleared_footprint": True,
            "map_topic": "map",
            "map_subscribe_transient_local": True,
        },
        "obstacle": parameters(BAND),
        "inflation": INFLATION,
        "globals": {
            "track_unknown_space": True,
            "use_maximum": False,
            "lethal_cost_threshold": 100,
            "inscribed_obstacle_cost_value": 99,
            "unknown_cost_value": 255,
            "trinary_costmap": True,
            "transform_tolerance": 0.1,
            "allow_parameter_qos_overrides": True,
        },
        "footprint": [[-0.22, -0.2], [0.22, -0.2], [0.22, 0.2], [-0.22, 0.2]],
        "footprint_padding": 0.01,
        "base": BASE,
        "cloud": cloud,
    })


if __name__ == "__main__":
    main()
