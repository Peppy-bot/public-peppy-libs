"""Writes the planning scenario of the ROS reference harness.

Three worlds of box obstacles in the unit cube: a wall with a block before it, in two
and in seven dimensions, and the two-dimensional wall's goal walled in by two slabs and
the cube's sides, where no path reaches it. Every planner plans the two-dimensional wall
under seed 1, the seven-dimensional one under seed 2, and the walled-in goal under seed
1, where it finds no path; RRTstar also plans the two-dimensional wall under seed 2,
where its simplification gives back its path. Each plan is bounded at ITERATIONS, at
which PRM and PRM* grow their roadmap, expand it and grow it again, and PRM's path, which
the reference does not answer, is not simplified. A box is open, so its surface is free.
Run it from any directory:

    python3 ros_reference/tools/draw_planning.py
    python3 ros_reference/tools/draw_planning.py --drawn 200

The second also has every planner plan each wall under each seed from 1 to 200, a large
set for a local run, which the README describes.
"""

from drawing import options, write

NAME = "planning_boxes"
MOTION_STEP = 0.01
ITERATIONS = 2000
SIMPLIFICATION = 100
DRAWN_SEEDS = 0
PLANNERS = [
    "RRTConnect",
    "RRT",
    "RRTstar",
    "BiTRRT",
    "KPIECE1",
    "BKPIECE1",
    "LBKPIECE1",
    "EST",
    "SBL",
    "PRM",
    "PRMstar",
]
# The slabs that wall in the two-dimensional wall's goal at (0.9, 0.1): one across x from
# 0.8 to 0.85 and one across y from 0.2 to 0.25, each thicker than MOTION_STEP, overlapping
# where they meet.
WALLED_IN = [((0.8, 0.85), (-1.0, 0.25)), ((0.8, 2.0), (0.2, 0.25))]


def slab(dimension, first, second):
    """A box spanning `first` in the first dimension, `second` in the second, and the
    whole cube in every other."""
    low, high = [-1.0] * dimension, [2.0] * dimension
    (low[0], high[0]), (low[1], high[1]) = first, second
    return {"min": low, "max": high}


def ends(dimension):
    """The start and the goal of a world of `dimension` dimensions: on either side of the
    middle of the first dimension, low in the second, and at the middle of every other."""
    start, goal = [0.5] * dimension, [0.5] * dimension
    start[0], start[1] = 0.1, 0.1
    goal[0], goal[1] = 0.9, 0.1
    return start, goal


def wall(dimension):
    start, goal = ends(dimension)
    return {
        "obstacles": [
            slab(dimension, (0.45, 0.55), (-1.0, 0.8)),
            slab(dimension, (0.2, 0.3), (-1.0, 0.6)),
        ],
        "start": start,
        "goal": goal,
    }


def walled_in(dimension):
    start, goal = ends(dimension)
    return {
        "obstacles": [slab(dimension, *sides) for sides in WALLED_IN],
        "start": start,
        "goal": goal,
    }


def plan(world, planner, seed):
    return {
        "world": world,
        "planner": planner,
        "seed": seed,
        "iterations": ITERATIONS,
        "simplification": 0 if planner == "PRM" else SIMPLIFICATION,
    }


def main():
    seeds = options(__doc__, DRAWN_SEEDS, "seeds each planner plans each wall under")
    seeds = seeds.parse_args().drawn
    worlds = [wall(2), wall(7), walled_in(2)]
    plans = [
        *(plan(0, planner, 1) for planner in PLANNERS),
        *(plan(1, planner, 2) for planner in PLANNERS),
        plan(0, "RRTstar", 2),
        *(plan(2, planner, 1) for planner in PLANNERS),
        *(
            plan(world, p, seed)
            for world in (0, 1)
            for p in PLANNERS
            for seed in range(1, seeds + 1)
        ),
    ]
    write(
        NAME,
        {"kind": "planning", "motion_step": MOTION_STEP, "worlds": worlds, "plans": plans},
    )


if __name__ == "__main__":
    main()
