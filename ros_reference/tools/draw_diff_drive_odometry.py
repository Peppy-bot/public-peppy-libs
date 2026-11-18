# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the diff drive odometry scenarios of the ROS reference harness.

Each scenario is the wheels of a differential base, their separation, each wheel's radius
and the rolling window of the velocity, the period of the updates, and the left and the
right wheel's angle, in radians, at each update, from 0. The angles follow drive segments,
each a number of updates at a pair of wheel rates in rad/s:

- diff_drive_odometry_equal_wheels: wheels of one radius; a straight run, an arc, a spin
  in place, a turn so slight that each update turns the base less than 1e-6 rad, the
  integration's straight branch, a run backwards, and drawn segments;
- diff_drive_odometry_unequal_wheels: wheels of radii 2% apart, a longer window and a
  shorter period, and drawn segments only, the first longer than the window.

Run it from any directory:

    uv run ros_reference/tools/draw_diff_drive_odometry.py
    uv run ros_reference/tools/draw_diff_drive_odometry.py --drawn 40

The second draws a large set for a local run, which the README describes.
"""

from drawing import generator, options, write

# The segments each scenario draws beyond its named ones.
DRAWN_SEGMENTS = 4
# The fastest wheel rate a drawn segment takes, in rad/s.
MAX_RATE = 10.0


def positions_of(segments, period):
    """The wheel angles at each update of `segments`, each a count of updates and the left
    and the right wheel's rate."""
    left = right = 0.0
    positions = []
    for count, left_rate, right_rate in segments:
        for _ in range(count):
            left += left_rate * period
            right += right_rate * period
            positions.append([left, right])
    return positions


def drawn_segments(rng, drawn, least):
    return [(rng.randint(least, least + 8), rng.uniform(-MAX_RATE, MAX_RATE),
             rng.uniform(-MAX_RATE, MAX_RATE)) for _ in range(drawn)]


def scenario(name, wheels, period, segments):
    write(name, {"kind": "diff_drive_odometry", "wheels": wheels, "period": period,
                 "positions": positions_of(segments, period)})


def main():
    parser = options(__doc__, DRAWN_SEGMENTS, "segments drawn in each scenario")
    drawn = parser.parse_args().drawn
    name = "diff_drive_odometry_equal_wheels"
    rng = generator(name)
    named = [
        (6, 4.0, 4.0),
        (6, 3.0, 5.0),
        (6, -4.0, 4.0),
        (6, 4.0, 4.0001),
        (6, -3.0, -3.0),
    ]
    wheels = {"wheel_separation": 0.4, "left_wheel_radius": 0.05, "right_wheel_radius": 0.05,
              "velocity_rolling_window_size": 4}
    scenario(name, wheels, 0.02, named + drawn_segments(rng, drawn, 2))
    name = "diff_drive_odometry_unequal_wheels"
    rng = generator(name)
    wheels = {"wheel_separation": 0.35, "left_wheel_radius": 0.049,
              "right_wheel_radius": 0.051, "velocity_rolling_window_size": 10}
    scenario(name, wheels, 0.01, drawn_segments(rng, drawn, 12))


if __name__ == "__main__":
    main()
