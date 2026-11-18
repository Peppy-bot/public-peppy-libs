# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the Smac 2D scenarios of the ROS reference harness.

Each scenario is a cost grid, the footprint of a base, Smac 2D's planner and smoother
parameters, and the starts and goals it plans between, each x and y in meters and a
heading in radians. Every planner plans with a goal tolerance of 0, to the goal's cell
itself:

- smac_2d_open_floor: every cell free, so every step onto a cell costs the same and the
  open set holds entries of equal cost all the way, and the order in which Smac's heap
  gives them up decides the path; starts and goals drawn over the floor, a goal on a
  cell's corner, one along a row, one along a diagonal and one in the start's own cell;
- smac_2d_cluttered: lethal blocks, each ringed by the inscribed cost out to the
  footprint's inscribed radius and by a cost that falls with distance beyond it, and
  patches of unknown cells, with allow_unknown true and a cost multiplier of 2; starts and
  goals drawn on passable cells, one goal past a patch of unknown cells, one goal that a
  wall closes off, a goal deep inside a block and a start deep inside one;
- smac_2d_known_only: a grid drawn the same way with allow_unknown false, the last heading
  along the final approach, and a smoother whose iteration bound ends it; starts and goals
  drawn on known passable cells, and a goal that a wall closes off;
- smac_2d_ties: a free floor with no cost multiplier, with two short
  walls, each with a start and a goal on the grid points of its axis of symmetry, both
  ways round, so the two ways round a wall cost the same at every step and the order in
  which Smac's heap gives up entries of equal cost picks the way; and four goals on grid
  points at a knight's move or more off a row or a diagonal from their starts, where many
  paths cost the same.

Each grid's origin is a multiple of 1/8 m, which single precision holds, so the corner of
the grid's first column, where Smac puts a path's poses, lies on the grid. Run it from any
directory:

    uv run ros_reference/tools/draw_smac_2d.py
    uv run ros_reference/tools/draw_smac_2d.py --drawn 40

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import generator, options, write

FREE = 0
INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255
RESOLUTION = 0.1
# The cell of smac_2d_ties: a power of two, so a grid point, a whole number of cells from
# the origin, has exact coordinates.
TIE_RESOLUTION = 0.125
# The footprint of the base, 0.44 m by 0.40 m about its center: its inscribed radius is
# 0.2 m.
FOOTPRINT = [[-0.22, -0.2], [0.22, -0.2], [0.22, 0.2], [-0.22, 0.2]]
INSCRIBED_RADIUS = 0.2
# Beyond the inscribed radius, a cell's cost falls as 252 * exp(-DECAY * (d - 0.2)) out to
# INFLATION_RADIUS from the nearest lethal cell.
INFLATION_RADIUS = 0.55
DECAY = 5.0
# The starts and goals each scenario draws beyond its named cases.
DRAWN_PLANS = 4
# How far a drawn start or goal lies from the grid's edge, in meters.
EDGE_CLEARANCE = 0.3
# How far apart a drawn start and goal lie at least, in meters.
LEAST_LENGTH = 0.6
# The half side, in cells, of the square of free cells a wall closes off.
ENCLOSED = 4
# The half side, in cells, of a block a goal and a start lie deep inside.
DEEP = 3

SMOOTHER = {
    "tolerance": 1e-10,
    "max_iterations": 1000,
    "w_data": 0.2,
    "w_smooth": 0.3,
    "do_refinement": True,
    "refinement_num": 2,
}


def planner(**changes):
    """Smac 2D's planner parameters, with `changes`: a goal tolerance of 0, under which
    max_on_approach_iterations ends no search, and max_planning_time so large that no clock
    ends a plan."""
    return {
        "tolerance": 0.0,
        "cost_travel_multiplier": 2.0,
        "allow_unknown": True,
        "max_iterations": 1000000,
        "max_on_approach_iterations": 1000,
        "terminal_checking_interval": 5000,
        "max_planning_time": 1e9,
        "use_final_approach_orientation": False,
        **changes,
    }


class Grid:
    def __init__(self, size_x, size_y, origin, resolution=RESOLUTION):
        self.size_x = size_x
        self.size_y = size_y
        self.origin = origin
        self.resolution = resolution
        self.costs = [[FREE] * size_x for _ in range(size_y)]

    def fill(self, x0, y0, x1, y1, cost):
        """Sets the cells from (x0, y0) to (x1, y1), both included, to `cost`."""
        for y in range(max(0, y0), min(self.size_y, y1 + 1)):
            for x in range(max(0, x0), min(self.size_x, x1 + 1)):
                self.costs[y][x] = cost

    def ring(self, cx, cy, half):
        """A lethal square outline `half` + 1 cells from (cx, cy) on each side."""
        side = half + 1
        self.fill(cx - side, cy - side, cx + side, cy - side, LETHAL)
        self.fill(cx - side, cy + side, cx + side, cy + side, LETHAL)
        self.fill(cx - side, cy - side, cx - side, cy + side, LETHAL)
        self.fill(cx + side, cy - side, cx + side, cy + side, LETHAL)

    def inflate(self):
        """Raises each known cell near a lethal cell: to the inscribed cost within the
        inscribed radius, and by the falling cost out to INFLATION_RADIUS."""
        lethal = [
            (x, y)
            for y in range(self.size_y)
            for x in range(self.size_x)
            if self.costs[y][x] == LETHAL
        ]
        for y in range(self.size_y):
            for x in range(self.size_x):
                if self.costs[y][x] in (LETHAL, UNKNOWN) or not lethal:
                    continue
                d = RESOLUTION * min(math.hypot(x - lx, y - ly) for lx, ly in lethal)
                if d <= INSCRIBED_RADIUS:
                    self.costs[y][x] = INSCRIBED
                elif d <= INFLATION_RADIUS:
                    self.costs[y][x] = int(252 * math.exp(-DECAY * (d - INSCRIBED_RADIUS)))

    def cell_of(self, point):
        return (
            int((point[0] - self.origin[0]) / self.resolution),
            int((point[1] - self.origin[1]) / self.resolution),
        )

    def corner(self, x, y):
        """The grid point (x, y), the corner of the cell (x, y) at the origin's side."""
        return [self.origin[0] + x * self.resolution, self.origin[1] + y * self.resolution]

    def center(self, x, y):
        return [
            self.origin[0] + (x + 0.5) * self.resolution,
            self.origin[1] + (y + 0.5) * self.resolution,
        ]

    def nearest_passable(self, x, y, allow_unknown):
        """The center of the known passable cell nearest the cell (x, y)."""
        cells = sorted(
            ((cx, cy) for cy in range(self.size_y) for cx in range(self.size_x)),
            key=lambda cell: (math.hypot(cell[0] - x, cell[1] - y), cell),
        )
        return next(
            self.center(cx, cy)
            for cx, cy in cells
            if self.costs[cy][cx] < INSCRIBED or (allow_unknown and self.costs[cy][cx] == UNKNOWN)
        )

    def passable(self, point, allow_unknown):
        x, y = self.cell_of(point)
        cost = self.costs[y][x]
        return (cost == UNKNOWN and allow_unknown) or cost < INSCRIBED

    def scenario(self, name, planner_parameters, smoother, plans, blocked=()):
        """The scenario `name`, whose plans start and end on passable cells but for the
        points `blocked`."""
        allow_unknown = planner_parameters["allow_unknown"]
        for plan in plans:
            for end in (plan["start"], plan["goal"]):
                if end[:2] not in blocked and not self.passable(end, allow_unknown):
                    raise SystemExit(f"{name}: {end} lies on no passable cell")
        return {
            "kind": "smac_2d",
            "grid": {
                "size_x": self.size_x,
                "size_y": self.size_y,
                "resolution": self.resolution,
                "origin": self.origin,
            },
            "costs": self.costs,
            "footprint": FOOTPRINT,
            "planner": planner_parameters,
            "smoother": smoother,
            "plans": plans,
        }


def heading(rng):
    return rng.uniform(-math.pi, math.pi)


def point(rng, grid, allow_unknown):
    """A point drawn at least EDGE_CLEARANCE inside the grid, on a passable cell."""
    while True:
        p = [
            grid.origin[0] + rng.uniform(EDGE_CLEARANCE, grid.size_x * RESOLUTION - EDGE_CLEARANCE),
            grid.origin[1] + rng.uniform(EDGE_CLEARANCE, grid.size_y * RESOLUTION - EDGE_CLEARANCE),
        ]
        if grid.passable(p, allow_unknown):
            return p


def drawn_plans(rng, grid, allow_unknown, count):
    """`count` starts and goals drawn on passable cells, at least LEAST_LENGTH apart."""
    plans = []
    while len(plans) < count:
        start, goal = point(rng, grid, allow_unknown), point(rng, grid, allow_unknown)
        if math.dist(start, goal) >= LEAST_LENGTH:
            plans.append({"start": [*start, heading(rng)], "goal": [*goal, heading(rng)]})
    return plans


def blocks(rng, grid, count):
    """`count` lethal blocks of 2 to 6 cells a side, drawn anywhere."""
    for _ in range(count):
        w, h = rng.randint(2, 6), rng.randint(2, 6)
        x, y = rng.randint(0, grid.size_x - w), rng.randint(0, grid.size_y - h)
        grid.fill(x, y, x + w - 1, y + h - 1, LETHAL)


def unknown_patches(rng, grid, count):
    for _ in range(count):
        w, h = rng.randint(3, 8), rng.randint(3, 8)
        x, y = rng.randint(0, grid.size_x - w), rng.randint(0, grid.size_y - h)
        grid.fill(x, y, x + w - 1, y + h - 1, UNKNOWN)


def open_floor(drawn):
    name = "smac_2d_open_floor"
    rng = generator(name)
    grid = Grid(40, 30, [-2.0, -1.5])
    corner = [grid.origin[0] + 31 * RESOLUTION, grid.origin[1] + 22 * RESOLUTION]
    start = grid.center(5, 6)
    plans = [
        {"start": [*grid.center(4, 4), 0.0], "goal": [*corner, heading(rng)]},
        {"start": [*grid.center(3, 15), 0.0], "goal": [*grid.center(36, 15), 0.0]},
        {"start": [*grid.center(4, 3), 0.5], "goal": [*grid.center(28, 27), -0.5]},
        {"start": [*start, 0.3], "goal": [start[0] + 0.01, start[1] - 0.015, 1.2]},
        *drawn_plans(rng, grid, True, drawn),
    ]
    write(name, grid.scenario(name, planner(), SMOOTHER, plans))


def cluttered(drawn):
    name = "smac_2d_cluttered"
    rng = generator(name)
    grid = Grid(48, 36, [-2.5, -1.75])
    unknown_patches(rng, grid, 2)
    grid.fill(30, 4, 37, 9, UNKNOWN)
    blocks(rng, grid, 4)
    enclosed = (38, 26)
    grid.ring(*enclosed, ENCLOSED)
    deep = (10, 26)
    grid.fill(deep[0] - DEEP, deep[1] - DEEP, deep[0] + DEEP, deep[1] + DEEP, LETHAL)
    grid.inflate()
    west = grid.nearest_passable(27, 6, True)
    east = grid.nearest_passable(41, 7, True)
    corner = grid.nearest_passable(2, 2, True)
    plans = [
        {"start": [*west, 0.0], "goal": [*east, 0.0]},
        {"start": [*corner, 0.0], "goal": [*grid.center(*enclosed), 0.0]},
        {"start": [*corner, 0.0], "goal": [*grid.center(*deep), 0.0]},
        {"start": [*grid.center(*deep), 0.0], "goal": [*east, 0.0]},
        *drawn_plans(rng, grid, True, drawn),
    ]
    blocked = [grid.center(*deep)]
    write(name, grid.scenario(name, planner(), SMOOTHER, plans, blocked))


def known_only(drawn):
    name = "smac_2d_known_only"
    rng = generator(name)
    grid = Grid(48, 36, [-2.5, -1.75])
    unknown_patches(rng, grid, 3)
    blocks(rng, grid, 4)
    enclosed = (36, 25)
    grid.ring(*enclosed, ENCLOSED)
    grid.inflate()
    parameters = planner(allow_unknown=False, use_final_approach_orientation=True)
    smoother = {
        **SMOOTHER,
        "max_iterations": 50,
        "w_data": 0.1,
        "w_smooth": 0.4,
        "do_refinement": False,
    }
    plans = [
        {"start": [*grid.nearest_passable(3, 3, False), 0.0], "goal": [*grid.center(*enclosed), 1.0]},
        *drawn_plans(rng, grid, False, drawn + 3),
    ]
    write(name, grid.scenario(name, parameters, smoother, plans))


def ties():
    """A free grid with two walls, each with its start and goal on the grid points of its
    axis of symmetry, so the two ways round it cost the same at every step, and the order
    in which Smac's heap gives up entries of equal cost picks the way."""
    name = "smac_2d_ties"
    grid = Grid(40, 30, [-2.0, -2.0], TIE_RESOLUTION)
    grid.fill(15, 9, 15, 11, LETHAL)
    grid.fill(29, 20, 31, 20, LETHAL)
    ends = [
        ((5, 10), (25, 10)),
        ((25, 10), (5, 10)),
        ((30, 26), (30, 14)),
        ((30, 14), (30, 26)),
        ((3, 22), (13, 26)),
        ((20, 3), (36, 9)),
        ((36, 3), (22, 7)),
        ((8, 27), (2, 15)),
    ]
    plans = [
        {"start": [*grid.center(*start), 0.0], "goal": [*grid.corner(*goal), 0.0]}
        for start, goal in ends
    ]
    write(name, grid.scenario(name, planner(cost_travel_multiplier=0.0), SMOOTHER, plans))


def main():
    parser = options(__doc__, DRAWN_PLANS, "starts and goals drawn in each scenario")
    drawn = parser.parse_args().drawn
    open_floor(drawn)
    cluttered(drawn)
    known_only(drawn)
    ties()


if __name__ == "__main__":
    main()
