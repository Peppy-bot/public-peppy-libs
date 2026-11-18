# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the pure pursuit scenarios of the ROS reference harness.

Each scenario is a cost grid in the odometry frame and whether it tracks unknown cells, the
footprint of a base, Nav2's regulated pure pursuit, simple goal checker, simple progress
checker and feasible path handler parameters, a plan of poses every 5 cm, each x and y in
meters and a heading in radians, and the steps the controller server runs, each at a time
in nanoseconds with the base's pose and its speed.
The base stands at a distance along the plan, beside it and turned off its heading by
drawn amounts:

- pure_pursuit_open_floor: a free floor and a plan that runs straight, turns left on an arc
  of 0.6 m, inside the radius below which the speed falls, and runs straight to a goal
  heading 0.7 rad off the last step's; a base turned far off the plan, which turns in
  place towards it, steps along the straight run at full speed, on the arc, on the
  approach to the goal, within the goal's tolerance, where the base turns to the goal's
  heading, and at the goal's heading, where the goal is reached, and a step that comes
  more than movement_time_allowance after the last one at nearly the same place;
- pure_pursuit_cluttered: inflated blocks beside the plan, which slow the base by the cost
  under it, a patch of unknown cells on it, which a grid that tracks unknown cells lets
  the footprint cross, and a lethal wall across it, which fails the command once the arc
  ahead reaches it;
- pure_pursuit_scaled_lookahead: a lookahead scaled by the base's speed and a minimum
  distance to obstacles on a straight plan that leaves the grid; a block of inflated cost
  beside the plan, a base slowing to rest, its lookahead at its lower bound, a base off the
  grid, and a base whose nearest plan pose lies off the grid.

Run it from any directory:

    uv run ros_reference/tools/draw_pure_pursuit.py
    uv run ros_reference/tools/draw_pure_pursuit.py --drawn 40

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import generator, options, write

FREE = 0
INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255
RESOLUTION = 0.05
# The grid's cells a side: 4 m.
CELLS = 80
# The footprint of the base, 0.4 m by 0.3 m about its center: its inscribed radius is
# 0.15 m.
FOOTPRINT = [[-0.2, -0.15], [0.2, -0.15], [0.2, 0.15], [-0.2, 0.15]]
INSCRIBED_RADIUS = 0.15
# Beyond the inscribed radius, a cell's cost falls as 252 * exp(-SCALING * (d - 0.15)) out to
# INFLATION_RADIUS from the nearest lethal cell, with the controller's
# inflation_cost_scaling_factor.
INFLATION_RADIUS = 0.55
SCALING = 3.0
# The distance between two poses of a plan, in meters.
SPACING = 0.05
# The first step's time and the time between two steps, in nanoseconds.
START_NS = 1_000_000_000
STEP_NS = 50_000_000
# The steps each scenario draws beyond its named ones.
DRAWN_STEPS = 4
# The largest drawn offset of the base beside the plan, in meters, and off its heading,
# in radians.
LATERAL = 0.04
TURNED = 0.2


def controller(**changes):
    """The controller's parameters, Nav2's defaults with a lookahead of 0.4 m, and
    `changes`."""
    return {
        "max_linear_vel": 0.5,
        "min_linear_vel": -0.5,
        "max_angular_vel": 2.5,
        "min_angular_vel": -2.5,
        "lookahead_dist": 0.4,
        "min_lookahead_dist": 0.3,
        "max_lookahead_dist": 0.9,
        "lookahead_time": 1.5,
        "rotate_to_heading_angular_vel": 1.8,
        "use_velocity_scaled_lookahead_dist": False,
        "min_approach_linear_velocity": 0.05,
        "approach_velocity_scaling_dist": 0.6,
        "max_allowed_time_to_collision_up_to_carrot": 1.0,
        "min_distance_to_obstacle": -1.0,
        "use_regulated_linear_velocity_scaling": True,
        "use_cost_regulated_linear_velocity_scaling": True,
        "cost_scaling_dist": 0.6,
        "cost_scaling_gain": 1.0,
        "inflation_cost_scaling_factor": SCALING,
        "regulated_linear_scaling_min_radius": 0.9,
        "regulated_linear_scaling_min_speed": 0.25,
        "use_fixed_curvature_lookahead": False,
        "curvature_lookahead_dist": 0.6,
        "use_rotate_to_heading": True,
        "rotate_to_heading_min_angle": 0.785,
        "max_linear_accel": 2.5,
        "max_linear_decel": -2.5,
        "max_angular_accel": 3.2,
        "max_angular_decel": -3.2,
        "use_cancel_deceleration": False,
        "cancel_deceleration": 3.2,
        "allow_reversing": False,
        "interpolate_curvature_after_goal": False,
        "use_collision_detection": True,
        "use_dynamic_window": False,
        "allow_obstacle_checking_beyond_goal": False,
        **changes,
    }


GOAL_CHECKER = {
    "xy_goal_tolerance": 0.25,
    "xy_goal_tolerance_buffer": 0.0,
    "yaw_goal_tolerance": 0.25,
    "path_length_tolerance": 1.0,
    "stateful": True,
    "symmetric_yaw_tolerance": False,
}
PROGRESS_CHECKER = {"required_movement_radius": 0.5, "movement_time_allowance": 10.0}
PATH_HANDLER = {
    "reject_unit_path": False,
    "max_robot_pose_search_dist": 2.0,
    "prune_distance": 1.5,
    "enforce_path_inversion": False,
    "enforce_path_rotation": False,
    "inversion_xy_tolerance": 0.2,
    "inversion_yaw_tolerance": 0.4,
    "minimum_rotation_angle": 0.785,
}


class Plan:
    """A plan of poses every SPACING along straight runs and arcs, each heading along
    the plan."""

    def __init__(self, x, y, heading):
        self.poses = [[x, y, heading]]

    def straight(self, length):
        x, y, heading = self.poses[-1]
        for i in range(1, round(length / SPACING) + 1):
            self.poses.append([x + i * SPACING * math.cos(heading),
                               y + i * SPACING * math.sin(heading), heading])
        return self

    def arc(self, radius, angle):
        """A left turn by `angle`, a right one when it is negative."""
        x, y, heading = self.poses[-1]
        side = 1.0 if angle > 0 else -1.0
        cx, cy = x - side * radius * math.sin(heading), y + side * radius * math.cos(heading)
        steps = round(abs(angle) * radius / SPACING)
        for i in range(1, steps + 1):
            turn = heading + angle * i / steps
            self.poses.append([cx + side * radius * math.sin(turn),
                               cy - side * radius * math.cos(turn), turn])
        return self

    def ending(self, turn):
        """The plan with its last pose turned by `turn` off the last step's heading."""
        self.poses[-1][2] += turn
        return self.poses

    def at(self, distance):
        """The pose at `distance` along the plan."""
        index = min(int(distance / SPACING), len(self.poses) - 1)
        return self.poses[index]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Steps:
    def __init__(self, rng):
        self.rng = rng
        self.at_ns = START_NS
        self.steps = []

    def add(self, pose, speed, gap_ns=STEP_NS):
        self.steps.append({"at_ns": self.at_ns, "pose": pose, "speed": speed})
        self.at_ns += gap_ns

    def beside(self, plan, distance, turn=None, speed=None, **kwargs):
        """The base `distance` along `plan`, a drawn offset beside it, turned off its
        heading by `turn` or by a drawn amount, at `speed` or a drawn one."""
        x, y, heading = plan.at(distance)
        lateral = self.rng.uniform(-LATERAL, LATERAL)
        turn = self.rng.uniform(-TURNED, TURNED) if turn is None else turn
        pose = [x - lateral * math.sin(heading), y + lateral * math.cos(heading),
                wrap(heading + turn)]
        if speed is None:
            speed = [self.rng.uniform(0.0, 0.5), 0.0, self.rng.uniform(-0.5, 0.5)]
        self.add(pose, speed, **kwargs)

    def along(self, plan, named, low, high, count, **kwargs):
        """The base at each of the `named` distances along `plan` and at `count` drawn
        between `low` and `high`, in their order along the plan."""
        drawn = [self.rng.uniform(low, high) for _ in range(count)]
        for distance in sorted([*named, *drawn]):
            self.beside(plan, distance, **kwargs)


class Grid:
    def __init__(self):
        self.costs = [[FREE] * CELLS for _ in range(CELLS)]

    def cell(self, x, y):
        return int(x / RESOLUTION), int(y / RESOLUTION)

    def fill(self, low, high, cost):
        """Sets the cells whose centers lie between the corners `low` and `high`."""
        for cy in range(CELLS):
            for cx in range(CELLS):
                x, y = (cx + 0.5) * RESOLUTION, (cy + 0.5) * RESOLUTION
                if low[0] <= x <= high[0] and low[1] <= y <= high[1]:
                    self.costs[cy][cx] = cost

    def inflate(self):
        """Raises each known cell near a lethal cell: to the inscribed cost within the
        inscribed radius, and by the falling cost out to INFLATION_RADIUS."""
        lethal = [(cx, cy) for cy in range(CELLS) for cx in range(CELLS)
                  if self.costs[cy][cx] == LETHAL]
        for cy in range(CELLS):
            for cx in range(CELLS):
                if self.costs[cy][cx] in (LETHAL, UNKNOWN):
                    continue
                d = RESOLUTION * min(math.hypot(cx - lx, cy - ly) for lx, ly in lethal)
                if d <= INSCRIBED_RADIUS:
                    self.costs[cy][cx] = INSCRIBED
                elif d <= INFLATION_RADIUS:
                    cost = int(252 * math.exp(-SCALING * (d - INSCRIBED_RADIUS)))
                    self.costs[cy][cx] = max(self.costs[cy][cx], cost)


def scenario(name, grid, plan, steps, track_unknown_space=True, **changes):
    write(name, {
        "kind": "pure_pursuit",
        "grid": {"size_x": CELLS, "size_y": CELLS, "resolution": RESOLUTION,
                 "origin": [0.0, 0.0]},
        "costs": grid.costs,
        "track_unknown_space": track_unknown_space,
        "footprint": FOOTPRINT,
        "controller_frequency": 20.0,
        "controller": controller(**changes),
        "goal_checker": GOAL_CHECKER,
        "progress_checker": PROGRESS_CHECKER,
        "path_handler": PATH_HANDLER,
        "plan": plan,
        "steps": steps.steps,
    })


def open_floor(drawn):
    name = "pure_pursuit_open_floor"
    route = Plan(0.6, 0.6, 0.0).straight(1.0).arc(0.6, math.pi / 2).straight(1.2)
    length = (len(route.poses) - 1) * SPACING
    plan = route.ending(0.7)
    steps = Steps(generator(name))
    steps.beside(route, 0.0, turn=1.8, speed=[0.0, 0.0, 0.0])
    steps.beside(route, 0.0, turn=0.9, speed=[0.0, 0.0, -0.16])
    for distance in (0.1, 0.3):
        steps.beside(route, distance)
    steps.beside(route, 0.6, gap_ns=10_500_000_000)
    steps.beside(route, 0.61)
    steps.along(route, (1.2, 1.45, 2.6), 0.7, 2.4, drawn)
    end = route.poses[-1]
    steps.add([end[0] + 0.05, end[1] - 0.12, end[2] - 0.7], [0.0, 0.0, 0.0])
    steps.add([end[0] + 0.02, end[1] - 0.18, end[2] - 0.3], [0.0, 0.0, 1.0])
    steps.add([end[0] + 0.01, end[1] - 0.05, end[2] - 0.1], [0.0, 0.0, 0.4])
    scenario(name, Grid(), plan, steps)
    return length


def cluttered(drawn):
    name = "pure_pursuit_cluttered"
    route = Plan(0.5, 1.2, 0.0).straight(1.2).arc(1.2, 0.8).straight(1.0)
    grid = Grid()
    grid.fill([0.6, 0.65], [0.9, 0.85], LETHAL)
    grid.fill([1.3, 1.6], [1.5, 1.75], LETHAL)
    grid.inflate()
    x, y, heading = route.at(2.5)
    wall = [x + 0.15 * math.cos(heading), y + 0.15 * math.sin(heading)]
    grid.fill([wall[0] - 0.05, wall[1] - 0.6], [wall[0] + 0.05, wall[1] + 0.6], LETHAL)
    grid.fill([1.0, 1.1], [1.15, 1.3], UNKNOWN)
    steps = Steps(generator(name))
    steps.along(route, (0.2, 0.45, 0.6, 0.9, 1.8, 2.25), 1.0, 1.7, drawn,
                speed=[0.4, 0.0, 0.0])
    scenario(name, grid, route.ending(0.0), steps)


def scaled_lookahead(drawn):
    name = "pure_pursuit_scaled_lookahead"
    route = Plan(0.3, 2.0, 0.0).straight(4.5)
    grid = Grid()
    grid.fill([1.4, 2.35], [1.6, 2.45], LETHAL)
    grid.inflate()
    steps = Steps(generator(name))
    steps.add([-0.1, 2.0, 0.0], [0.2, 0.0, 0.0])
    for distance, speed in ((0.2, 0.1), (0.5, 0.35), (0.8, 0.6), (1.1, 0.5)):
        steps.beside(route, distance, speed=[speed, 0.0, 0.0])
    steps.along(route, (), 1.2, 2.6, drawn)
    steps.beside(route, 2.8, speed=[0.3, 0.0, 0.0])
    steps.beside(route, 2.85, speed=[0.14, 0.0, 0.0])
    steps.beside(route, 2.9, speed=[0.05, 0.0, 0.0])
    steps.beside(route, 2.92, speed=[0.0, 0.0, 0.0])
    steps.add([4.1, 2.0, 0.0], [0.0, 0.0, 0.0])
    scenario(name, grid, route.ending(0.0), steps, use_velocity_scaled_lookahead_dist=True,
             min_distance_to_obstacle=0.4)


def main():
    parser = options(__doc__, DRAWN_STEPS, "steps drawn in each scenario")
    drawn = parser.parse_args().drawn
    open_floor(drawn)
    cluttered(drawn)
    scaled_lookahead(drawn)


if __name__ == "__main__":
    main()
