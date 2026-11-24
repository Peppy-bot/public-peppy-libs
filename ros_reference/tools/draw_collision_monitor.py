# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the collision monitor scenarios of the ROS reference harness.

Each scenario is the collision monitor's parameters, with one polygon, the base's
footprint in approach mode, a contact at one point (min_points 1), and one point cloud
source, the largest step down of navigation_core's LiveObstacles under test, what its
points are, and cases, each a time in nanoseconds, the age of the newest cloud then, its
arrival, or null when none came, the cloud's points in the base's frame, z up from the
floor, and the velocity, linear x, linear y and angular z. The footprint is 0.5 m by
0.375 m about the base's center, its vertices whole in single precision:

- collision_monitor_approach: the points lie at the centers of a grid's cells, each at the
  height of an obstacle in the band, and the stop under test reads them as a source's
  newest points: no cloud, a cloud older than the source timeout and one just as old, a
  wall ahead within what the time before collision covers and one beyond it, cells already
  inside the footprint, a turn towards a wall at the side, a turn in place that sweeps a
  corner of the footprint over a cell, a sideways drive at a wall, a short wall of three
  cells, one cell ahead, and drawn cells at drawn velocities;
- collision_monitor_step_down: the points are a depth frame's, which the stop under test
  reads through navigation_core's LiveObstacles, from a sensor 1 m over the base's center.
  Each step lies over the columns of the test's 5 cm voxels that the footprint crosses,
  its edge on a column's side: in each such column, a floor point in the column before the
  edge, the floor the frame sees before the edge, and three points of the lower floor
  whose rays cross the floor's plane in the column past the edge, the three rays
  navigation_core's drop rule needs to mark that column a drop. The cases: a floor that
  drops 0.3 m, more than the largest step down, within reach ahead, where the collision
  monitor drives on, so the stop under test brings the base to rest before the drop; the
  same drop beyond reach, which leaves the stop driving on as the monitor does; the drop
  with a wall past its edge, which the monitor approaches; the same step no deeper than
  the largest step down, which marks no drop; and a turn towards a drop at the side, where
  the monitor drives on and the stop brings the base to rest.

Run it from any directory:

    uv run ros_reference/tools/draw_collision_monitor.py
    uv run ros_reference/tools/draw_collision_monitor.py --drawn 200

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import generator, options, write

# The footprint's front and half width, in meters.
FRONT = 0.25
HALF_WIDTH = 0.1875
FOOTPRINT = [[-FRONT, -HALF_WIDTH], [FRONT, -HALF_WIDTH], [FRONT, HALF_WIDTH],
             [-FRONT, HALF_WIDTH]]
# The time of each case, in nanoseconds, and the source timeout, in seconds and in
# nanoseconds.
NOW_NS = 5_000_000_000
SOURCE_TIMEOUT_S = 0.5
SOURCE_TIMEOUT_NS = 500_000_000
# The age of a fresh cloud, in nanoseconds.
FRESH_NS = 40_000_000
# The height of an obstacle point and of the floor, the depth of a drop, and the depth of
# a step down no deeper than the largest, in meters.
OBSTACLE = 0.2
FLOOR = 0.0
DROP = -0.3
STEP_DOWN = -0.05
# The height of the parity test's depth sensor over the base's center, in meters
# (SENSOR_HEIGHT_M of navigation_core/tests/ros_reference_collision_monitor.rs).
SENSOR_HEIGHT_M = 1.0
# The side of the voxels of the parity test's LiveObstacles, in meters (VOXEL_M of
# navigation_core/tests/ros_reference_collision_monitor.rs); its columns' sides lie on
# multiples of it.
VOXEL_M = 0.05
# Where the rays of a step's lower floor cross the floor's plane in the column past the
# edge, as fractions of the column's side from the edge: three rays, as many as
# navigation_core's drop rule needs to mark a column a drop, each 1.2 cm or more inside
# the column after the rounding to DECIMALS.
RAY_FRACTIONS = (0.25, 0.5, 0.75)
# The decimals of each coordinate of a step's points.
DECIMALS = 4
# The axes a step's edge lies across: x, ahead of the base, and y, to its left.
AHEAD = 0
LEFT = 1
# The clouds each scenario draws beyond its named cases.
DRAWN_CASES = 3


def monitor():
    points = ", ".join(f"[{x}, {y}]" for x, y in FOOTPRINT)
    return {
        "cmd_vel_in_topic": "cmd_vel_smoothed",
        "cmd_vel_out_topic": "cmd_vel",
        "state_topic": "",
        "base_frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "transform_tolerance": 0.1,
        "source_timeout": SOURCE_TIMEOUT_S,
        "base_shift_correction": False,
        "collision_points_marker_3d": False,
        "stop_pub_timeout": 1.0,
        "polygons": ["FootprintApproach"],
        "observation_sources": ["depth"],
        "enabled": True,
        "use_realtime_priority": False,
        "enable_stamped_cmd_vel": True,
        "allow_parameter_qos_overrides": True,
        "introspection_mode": "disabled",
        "FootprintApproach": {
            "type": "polygon",
            "points": f"[{points}]",
            "action_type": "approach",
            "enabled": True,
            "min_points": 1,
            "trigger_consecutive_points": 1,
            "release_consecutive_points": 1,
            "time_before_collision": 2.0,
            "simulation_time_step": 0.1,
            "visualize": False,
            "polygon_subscribe_transient_local": False,
            "sources_names": ["depth"],
        },
        "depth": {
            "type": "pointcloud",
            "topic": "depth_points",
            "enabled": True,
            "source_timeout": SOURCE_TIMEOUT_S,
            "min_height": 0.05,
            "max_height": 0.5,
            "min_range": 0.0,
            "use_global_height": False,
            "transport_type": "raw",
            "exclusion_zones": [],
        },
    }


def case(points, velocity, age_ns=FRESH_NS):
    return {"now_ns": NOW_NS, "cloud_age_ns": age_ns, "points": points if age_ns is not None
            else [], "velocity": velocity}


def column(x, z, half=0.16, count=9):
    """Points across the footprint's width at `x` ahead, at height `z`."""
    return [[x, -half + 2 * half * i / (count - 1), z] for i in range(count)]


def row(y, z, low=0.0, high=0.6, count=7):
    """Points along x at `y` beside the base, at height `z`."""
    return [[low + (high - low) * i / (count - 1), y, z] for i in range(count)]


def patch(x0, x1, z, y0=-0.3, y1=0.3, step=0.15, across=0.12):
    """Points over the floor from `x0` to `x1` ahead, `step` apart, and `y0` to `y1` across,
    `across` apart, at height `z`."""
    xs = [x0 + step * i for i in range(int(round((x1 - x0) / step)) + 1)]
    ys = [y0 + across * i for i in range(int(round((y1 - y0) / across)) + 1)]
    return [[x, y, z] for x in xs for y in ys]


def drawn_case(rng):
    points = [[rng.uniform(-0.6, 2.0), rng.uniform(-0.8, 0.8), OBSTACLE]
              for _ in range(rng.randint(4, 24))]
    velocity = [rng.uniform(0.0, 0.6), rng.uniform(-0.2, 0.2), rng.uniform(-1.0, 1.0)]
    return case(points, velocity)


def approach(drawn):
    name = "collision_monitor_approach"
    rng = generator(name)
    cases = [
        case([], [0.4, 0.0, 0.0], age_ns=None),
        case(column(1.0, OBSTACLE), [0.4, 0.0, 0.0], age_ns=SOURCE_TIMEOUT_NS + 100_000_000),
        case(patch(0.3, 1.2, OBSTACLE), [0.4, 0.0, 0.0], age_ns=SOURCE_TIMEOUT_NS),
        case(column(FRONT + 0.3, OBSTACLE), [0.4, 0.0, 0.0]),
        case(column(FRONT + 1.2, OBSTACLE), [0.4, 0.0, 0.0]),
        case(column(0.1, OBSTACLE), [0.4, 0.0, 0.0]),
        case(row(HALF_WIDTH + 0.25, OBSTACLE, 0.2, 1.0, count=13), [0.3, 0.0, 0.8]),
        case([[0.27, 0.0, OBSTACLE], [0.27, 0.01, OBSTACLE], [0.27, -0.01, OBSTACLE],
              [0.28, 0.0, OBSTACLE]], [0.0, 0.0, 1.0]),
        case(row(HALF_WIDTH + 0.2, OBSTACLE, -0.2, 0.2), [0.0, 0.3, 0.0]),
        case(column(FRONT + 0.3, OBSTACLE, count=3), [0.4, 0.0, 0.0]),
        case([[FRONT + 0.5, 0.05, OBSTACLE]], [0.4, 0.0, 0.0]),
        *(drawn_case(rng) for _ in range(drawn)),
    ]
    write(name, {"kind": "collision_monitor", "monitor": monitor(), "max_step_down": 0.1,
                 "points_are": "cell_centers", "cases": cases})


def column_centers(low, high):
    """The centers of the test's columns that hold the stretch from `low` to `high` along
    one axis."""
    first = math.floor(round(low / VOXEL_M, DECIMALS))
    last = math.ceil(round(high / VOXEL_M, DECIMALS))
    return [round((i + 0.5) * VOXEL_M, DECIMALS) for i in range(first, last)]


def through_floor(crossing, depth):
    """The point at the height `depth`, below the floor, whose ray from the test's sensor
    crosses the floor's plane at `crossing`, [x, y]."""
    scale = (SENSOR_HEIGHT_M - depth) / SENSOR_HEIGHT_M
    return [round(c * scale, DECIMALS) for c in crossing] + [depth]


def step(edge, depth, centers, axis):
    """A floor that steps down to the height `depth` past `edge` along `axis`, AHEAD or
    LEFT, over the columns whose centers on the other axis are `centers`: in each, a floor
    point at the center of the column before the edge, the floor the frame sees before
    the edge, and the points of the lower floor whose rays cross the floor's plane in the
    column past the edge at RAY_FRACTIONS of its side, at its center across."""
    assert round(edge / VOXEL_M, DECIMALS).is_integer(), \
        f"the edge {edge} lies on a column's side, a multiple of VOXEL_M"
    def placed(along, across):
        return [along, across] if axis == AHEAD else [across, along]
    before = round(edge - VOXEL_M / 2, DECIMALS)
    floor = [placed(before, center) + [FLOOR] for center in centers]
    lower = [through_floor(placed(edge + fraction * VOXEL_M, center), depth)
             for center in centers for fraction in RAY_FRACTIONS]
    return floor + lower


def step_down():
    name = "collision_monitor_step_down"
    edge = FRONT + 0.25
    far_edge = 1.55
    wall = edge + 0.3
    side_edge = 0.3
    # The columns a straight drive's footprint crosses, and those past the side edge whose
    # centers the footprint covers within reach turning at [0.3, 0.0, 0.9].
    ahead = column_centers(-HALF_WIDTH, HALF_WIDTH)
    beside = column_centers(0.15, 0.6)
    cases = [
        case(step(edge, DROP, ahead, AHEAD), [0.4, 0.0, 0.0]),
        case(step(far_edge, DROP, ahead, AHEAD), [0.4, 0.0, 0.0]),
        case(step(edge, DROP, ahead, AHEAD) + column(wall, OBSTACLE), [0.4, 0.0, 0.0]),
        case(step(edge, STEP_DOWN, ahead, AHEAD), [0.4, 0.0, 0.0]),
        case(step(side_edge, DROP, beside, LEFT), [0.3, 0.0, 0.9]),
    ]
    write(name, {"kind": "collision_monitor", "monitor": monitor(), "max_step_down": 0.1,
                 "points_are": "depth", "cases": cases})


def main():
    parser = options(__doc__, DRAWN_CASES, "clouds drawn in the approach scenario")
    drawn = parser.parse_args().drawn
    approach(drawn)
    step_down()


if __name__ == "__main__":
    main()
