# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the inflation scenarios of the ROS reference harness.

Each scenario is a footprint, the padding Costmap2DROS pads it by, the inflation layer's
parameters and grids to inflate. A
grid holds free cells and, drawn over them, lethal cells alone and in blocks, lethal cells
on its edges, patches of unknown cells, some next to a lethal cell, and cells of a cost
between free and lethal, which keep the higher of their cost and the inflated one:

- inflation_rectangle: the base's 0.44 m by 0.40 m footprint padded by Nav2's default of
  1 cm, Nav2's default radius of 0.55 m and scaling factor of 10, on cells of 5 cm, and a
  grid with no lethal cell;
- inflation_triangle: a triangle about the base's center, a vertex on its x axis, padded
  by 2 cm, a radius of 1 m on cells of 7 cm, which no whole number of cells spans, and a
  scaling factor of 3;
- inflation_inside_inscribed: no padding, a radius of 0.15 m, inside the footprint's
  inscribed radius, and a scaling factor of 0, so a cell past the inscribed radius takes
  252.

Each scenario pins every parameter the layer reads: enabled, inflate_unknown and
inflate_around_unknown false, no custom inscribed radius, and one thread. Run it from any
directory:

    uv run ros_reference/tools/draw_inflation.py
    uv run ros_reference/tools/draw_inflation.py --drawn 40

The second draws a large set for a local run, which the README describes.
"""

from drawing import generator, options, write

FREE = 0
LETHAL = 254
UNKNOWN = 255
# The grids each scenario draws, besides its named cases.
DRAWN_GRIDS = 2


def parameters(radius, scaling):
    return {
        "enabled": True,
        "inflation_radius": radius,
        "custom_inscribed_radius": -1.0,
        "cost_scaling_factor": scaling,
        "inflate_unknown": False,
        "inflate_around_unknown": False,
        "num_threads": 1,
    }


def drawn_grid(rng, size_x, size_y, resolution):
    """A grid of free cells with lethal cells alone, in blocks and on its edges, unknown
    patches, one beside a lethal cell, and cells of costs between."""
    costs = [[FREE] * size_x for _ in range(size_y)]

    def fill(x0, y0, x1, y1, cost):
        for y in range(max(0, y0), min(size_y, y1 + 1)):
            for x in range(max(0, x0), min(size_x, x1 + 1)):
                costs[y][x] = cost

    for _ in range(rng.randint(10, 20)):
        x, y = rng.randrange(size_x), rng.randrange(size_y)
        costs[y][x] = rng.randint(1, 252)
    for _ in range(2):
        w, h = rng.randint(2, 6), rng.randint(2, 6)
        x, y = rng.randrange(size_x), rng.randrange(size_y)
        fill(x, y, x + w - 1, y + h - 1, UNKNOWN)
    for _ in range(rng.randint(3, 6)):
        costs[rng.randrange(size_y)][rng.randrange(size_x)] = LETHAL
    w, h = rng.randint(1, 4), rng.randint(1, 4)
    x, y = rng.randrange(size_x - w), rng.randrange(size_y - h)
    fill(x, y, x + w - 1, y + h - 1, LETHAL)
    fill(x + w, y, x + w + 2, y + h - 1, UNKNOWN)
    costs[0][rng.randrange(size_x)] = LETHAL
    costs[rng.randrange(size_y)][size_x - 1] = LETHAL
    origin = [rng.randint(-40, 40) * resolution, rng.randint(-40, 40) * resolution]
    grid = {"size_x": size_x, "size_y": size_y, "resolution": resolution, "origin": origin}
    return {"grid": grid, "costs": costs}


def scenario(name, footprint, padding, inflation, resolution, drawn, extra=()):
    rng = generator(name)
    grids = [*extra, *(drawn_grid(rng, rng.randint(20, 32), rng.randint(16, 28), resolution)
                       for _ in range(drawn))]
    write(name, {"kind": "inflation", "footprint": footprint, "footprint_padding": padding,
                 "inflation": inflation, "grids": grids})


def main():
    parser = options(__doc__, DRAWN_GRIDS, "grids drawn in each scenario")
    drawn = parser.parse_args().drawn
    rectangle = [[-0.22, -0.2], [0.22, -0.2], [0.22, 0.2], [-0.22, 0.2]]
    empty = {"grid": {"size_x": 12, "size_y": 9, "resolution": 0.05, "origin": [0.0, 0.0]},
             "costs": [[FREE] * 12 for _ in range(9)]}
    scenario("inflation_rectangle", rectangle, 0.01, parameters(0.55, 10.0), 0.05, drawn + 1,
             [empty])
    triangle = [[0.3, 0.0], [-0.15, 0.25], [-0.15, -0.25]]
    scenario("inflation_triangle", triangle, 0.02, parameters(1.0, 3.0), 0.07, drawn)
    scenario("inflation_inside_inscribed", rectangle, 0.0, parameters(0.15, 0.0), 0.05, drawn)


if __name__ == "__main__":
    main()
