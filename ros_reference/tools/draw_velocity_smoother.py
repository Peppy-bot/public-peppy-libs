# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the velocity smoother scenarios of the ROS reference harness.

Each scenario is the smoother's parameters, open loop with velocity_timeout 0.5 s and the
axes scaled together, and a sequence of events, each at a time in nanoseconds: a setpoint
that arrives, linear x, linear y and angular z, or a cycle of the smoother's timer, one
every 50 ms at 20 Hz:

- velocity_smoother_differential: a differential base, its linear y of range 0 to 0 with
  0 acceleration, deceleration and deadband, as base_state's describe_base gives it, a
  deadband on the turn rate; cycles before the first setpoint, which publish nothing; a
  start from rest, a turn that scales the forward speed with it, a reversal through 0, a
  setpoint past every limit, setpoints drawn between cycles, the last setpoint followed
  by a cycle exactly velocity_timeout after it and the ramp to rest after that, cycles at
  rest that publish nothing, and a setpoint inside the deadband after the rest;
- velocity_smoother_holonomic: a base that drives sideways too, with decelerations
  unequal to its accelerations, and setpoints drawn between cycles, the first half a cycle
  after one and each later one a drawn gap of 1, 3 or 11 cycles, or 7/3 of a cycle, after
  the one before, so a gap of 11 cycles is a pause past the timeout.

Run it from any directory:

    uv run ros_reference/tools/draw_velocity_smoother.py
    uv run ros_reference/tools/draw_velocity_smoother.py --drawn 200

The second draws a large set for a local run, which the README describes.
"""

from drawing import generator, options, write

# The first event of each scenario, in nanoseconds on the node's clock.
START_NS = 1_000_000_000
# One cycle of the timer at 20 Hz, in nanoseconds.
CYCLE_NS = 50_000_000
# How long a setpoint holds, in seconds and in nanoseconds.
VELOCITY_TIMEOUT_S = 0.5
VELOCITY_TIMEOUT_NS = 500_000_000
# The setpoints each scenario draws beyond its named cases.
DRAWN_SETPOINTS = 6


def smoother(max_velocity, min_velocity, max_accel, max_decel, deadband):
    return {
        "smoothing_frequency": 20.0,
        "feedback": "OPEN_LOOP",
        "scale_velocities": True,
        "max_velocity": max_velocity,
        "min_velocity": min_velocity,
        "max_accel": max_accel,
        "max_decel": max_decel,
        "deadband_velocity": deadband,
        "velocity_timeout": VELOCITY_TIMEOUT_S,
        "odom_topic": "odom",
        "odom_duration": 0.1,
        "enable_stamped_cmd_vel": True,
        "use_realtime_priority": False,
        "allow_parameter_qos_overrides": True,
    }


def setpoint(at_ns, twist):
    return {"event": "setpoint", "at_ns": at_ns, "twist": twist}


def tick(at_ns):
    return {"event": "tick", "at_ns": at_ns}


def events_of(setpoints, end_ns):
    """The setpoints and a cycle every CYCLE_NS from START_NS to `end_ns`, in time order;
    a setpoint at the time of a cycle comes before it, as one that arrived first."""
    cycles = [tick(at_ns) for at_ns in range(START_NS, end_ns + 1, CYCLE_NS)]
    timed = [(event["at_ns"], 0, event) for event in setpoints]
    timed += [(event["at_ns"], 1, event) for event in cycles]
    return [event for _, _, event in sorted(timed, key=lambda item: item[:2])]


def drawn_twist(rng, limits_max, limits_min):
    """A setpoint up to half past each axis's limits."""
    return [rng.uniform(1.5 * low, 1.5 * high) for low, high in zip(limits_min, limits_max)]


def differential(drawn):
    name = "velocity_smoother_differential"
    rng = generator(name)
    max_velocity, min_velocity = [0.5, 0.0, 2.5], [-0.5, 0.0, -2.5]
    params = smoother(max_velocity, min_velocity, [2.5, 0.0, 3.2], [-2.5, 0.0, -3.2],
                      [0.0, 0.0, 0.05])
    first_ns = START_NS + 3 * CYCLE_NS + 10_000_000
    named = [
        (first_ns, [0.4, 0.0, 0.0]),
        (START_NS + 20 * CYCLE_NS, [0.3, 0.0, 1.5]),
        (START_NS + 32 * CYCLE_NS + 25_000_000, [-0.4, 0.0, -2.0]),
        (START_NS + 44 * CYCLE_NS, [0.9, 0.0, 3.0]),
    ]
    drawn_ns = sorted(rng.randrange(START_NS + 46 * CYCLE_NS, START_NS + 60 * CYCLE_NS)
                      for _ in range(drawn))
    # The last setpoint comes at the time of a cycle, so a cycle comes exactly
    # velocity_timeout after it.
    last_ns = START_NS + 61 * CYCLE_NS
    setpoints = [setpoint(at, twist) for at, twist in named]
    setpoints += [setpoint(at, drawn_twist(rng, max_velocity, min_velocity)) for at in drawn_ns]
    setpoints.append(setpoint(last_ns, [0.2, 0.0, 0.0]))
    restart_ns = last_ns + VELOCITY_TIMEOUT_NS + 30 * CYCLE_NS
    setpoints.append(setpoint(restart_ns, [0.1, 0.0, 0.02]))
    events = events_of(setpoints, restart_ns + 10 * CYCLE_NS)
    write(name, {"kind": "velocity_smoother", "smoother": params, "events": events})


def holonomic(drawn):
    name = "velocity_smoother_holonomic"
    rng = generator(name)
    max_velocity, min_velocity = [0.6, 0.4, 1.5], [-0.3, -0.4, -1.5]
    params = smoother(max_velocity, min_velocity, [1.0, 0.8, 2.0], [-1.6, -1.2, -3.0],
                      [0.01, 0.01, 0.0])
    at_ns = START_NS + CYCLE_NS // 2
    setpoints = []
    for index in range(2 * drawn):
        gap = rng.choice([CYCLE_NS, 3 * CYCLE_NS, 7 * CYCLE_NS // 3, 11 * CYCLE_NS])
        at_ns += gap if index else 0
        setpoints.append(setpoint(at_ns, drawn_twist(rng, max_velocity, min_velocity)))
    events = events_of(setpoints, at_ns + VELOCITY_TIMEOUT_NS + 30 * CYCLE_NS)
    write(name, {"kind": "velocity_smoother", "smoother": params, "events": events})


def main():
    parser = options(__doc__, DRAWN_SETPOINTS, "setpoints drawn in each scenario")
    drawn = parser.parse_args().drawn
    differential(drawn)
    holonomic(drawn)


if __name__ == "__main__":
    main()
