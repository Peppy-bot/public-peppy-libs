# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the map export scenario of the ROS reference harness.

Nav2's map saver writes each grid as a trinary map, at the thresholds that slam_toolbox's
`save_map` leaves it, its defaults. Each grid holds the cells a map's grid holds, unknown
(-1), free (0) and occupied (100), and one of these cases:

- each: one cell of each state;
- drawn: a grid of odd width and height whose cells are drawn;
- rounding: a cell side and an origin whose decimals the description rounds to three,
  among them a coordinate that rounds to -0.000;
- far: an origin far from the map frame's, with four digits before the point;
- unknown: a grid that no ray reached.

Run it from any directory:

    uv run ros_reference/tools/draw_map_export.py
    uv run ros_reference/tools/draw_map_export.py --drawn 5000

The second draws a large grid for a local run, which the README describes.
"""

import struct

from drawing import generator, options, write

NAME = "map_export_grids"
# The thresholds of nav2_map_server's map saver when its caller sets none.
FREE_THRESH = 0.25
OCCUPIED_THRESH = 0.65
STATES = (-1, 0, 100)
DRAWN_CELLS = 91
DRAWN_WIDTH = 13


def single(x):
    """`x` rounded to single precision, as a map's message holds its cell side."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def grid(width, height, resolution, origin, cells):
    if len(cells) != width * height:
        raise SystemExit(f"a grid of {width} by {height} cells holds {len(cells)}")
    return {
        "width": width,
        "height": height,
        "resolution": single(resolution),
        "origin": origin,
        "cells": cells,
    }


def main():
    parser = options(__doc__, DRAWN_CELLS, "cells drawn in the drawn grid")
    drawn = parser.parse_args().drawn
    rng = generator(NAME)
    height = max(1, drawn // DRAWN_WIDTH)
    grids = [
        grid(3, 1, 0.05, [0.0, 0.0, 0.0], list(STATES)),
        grid(
            DRAWN_WIDTH,
            height,
            0.05,
            [-3.2617, 1.0004, 0.0],
            [rng.choice(STATES) for _ in range(DRAWN_WIDTH * height)],
        ),
        grid(2, 3, 0.025, [-0.0004, 12.3456, 0.0], [0, 100, -1, 100, 0, -1]),
        grid(2, 2, 0.1, [1234.5678, -0.5, 0.0], [100, 0, 0, 100]),
        grid(4, 2, 0.05, [-1.0, -1.0, 0.0], [-1] * 8),
    ]
    write(
        NAME,
        {
            "kind": "map_export",
            "free_thresh": FREE_THRESH,
            "occupied_thresh": OCCUPIED_THRESH,
            "grids": grids,
        },
    )


if __name__ == "__main__":
    main()
