"""Writes the path timing scenario of the ROS reference harness.

Each path runs through waypoints of OpenArm v2's left arm inside its joint limits, under
the arm's URDF velocity limits and an acceleration limit of 5 rad/s^2 on every joint. The
scenario holds DRAWN_PATHS paths of each drawn case and one path of each other case:

- cornered: waypoints drawn uniformly inside the limits, so the legs are long and the
  deviation bounds the arc at each corner, blended at each of MAX_DEVIATIONS;
- short legs: legs that alternate between LONG_LEGS and SHORT_LEGS, so the half-leg
  bound binds at each corner and the arcs of a short leg's two corners meet at its
  midpoint;
- near reversal: corners that turn back by 180 degrees less a shortfall drawn
  log-uniformly over NEAR_REVERSAL_SHORTFALLS, so its median turn is about 178.27
  degrees;
- straight: a run between two waypoints drawn inside the limits, which never reaches the
  velocity limits, and a cruising one, under CRUISING_SCALE of them, which reaches them;
- turning back: a path that runs out along a leg and back along it, which TOTG refuses;
- reversal boundary: a corner by each of REVERSAL_BOUNDARY_TURNS, either side of TOTG's
  turning-back threshold of about 179.74 degrees;
- near straight: a corner by each of NEAR_STRAIGHT_TURNS, either side of TOTG's
  straight-on threshold of about 0.081 degrees.

The corners of the cornered, short-leg and near-reversal paths turn by more than TOTG's
straight-on threshold and less than its turning-back one, so TOTG blends each. Every path
but a cornered one blends at execute_joint_path's deviation, JOINT_PATH_DEVIATION. TOTG
gives each path's point at the fractions of its length that sample each of its pieces
(`piece_fractions`). Run it from any directory:

    python3 ros_reference/tools/draw_path_timing.py
    python3 ros_reference/tools/draw_path_timing.py --drawn 5

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import (
    REPOSITORY,
    generator,
    joint_limits,
    openarm_joints,
    options,
    robot,
    rust_constants,
    write,
)

NAME = "path_timing_openarm_v2"
URDF = "openarm_description/assets/openarm_v20.urdf"
DRAWN_PATHS = 1
MAX_DEVIATIONS = [0.01, 0.05, 0.1, 0.2]
# control_core's JOINT_PATH_CORNER_DEVIATION, the deviation of execute_joint_path.
JOINT_PATH_DEVIATION = rust_constants(REPOSITORY / "control_core" / "src" / "path_timing.rs")[
    "JOINT_PATH_CORNER_DEVIATION"
]
# The waypoints of a cornered path.
CORNERED_WAYPOINTS = 4
# The lengths (rad, Euclidean over the joints) the legs of a short-leg path alternate
# between, from a long one, over SHORT_LEG_LEGS legs.
LONG_LEGS = (0.2, 0.3)
SHORT_LEGS = (0.05, 0.1)
SHORT_LEG_LEGS = 5
# How far short of 180 degrees a near-reversal path's corners turn, from 0.3, a turn of
# 179.7 degrees, below TOTG's turning-back threshold of about 179.74, to 10; and the
# lengths (rad) of its NEAR_REVERSAL_LEGS legs.
NEAR_REVERSAL_SHORTFALLS = (0.3, 10.0)
NEAR_REVERSAL_LENGTHS = (0.05, 0.6)
NEAR_REVERSAL_LEGS = 3
# The share of the URDF velocity limits under which the cruising straight run reaches
# them.
CRUISING_SCALE = 0.1
# The turns (degrees) of the reversal-boundary paths, by name: the inside one turns by
# less than TOTG's turning-back threshold of about 179.74 degrees, and the outside one by
# more.
REVERSAL_BOUNDARY_TURNS = {"reversal_inside": 179.735, "reversal_outside": 179.75}
# The turns (degrees) of the near-straight paths, by name: TOTG blends the corner of the
# first, which turns by more than its straight-on threshold of about 0.081 degrees, and
# runs the corner of the second straight on.
NEAR_STRAIGHT_TURNS = {"near_straight_above": 0.1, "near_straight_below": 0.05}
# Where along each arc, and along each straight piece, TOTG gives the path's point, as
# shares of the piece's length.
ARC_SHARES = (0.25, 0.5, 0.75)
STRAIGHT_SHARES = (0.5,)
MAX_ACCELERATION = 5.0
# TOTG blends a corner whose cosine lies above TURNING_BACK_COS and at most
# STRAIGHT_ON_COS (moveit2 2.15.2): above the second it runs straight on, and at or below
# the first it refuses the path.
STRAIGHT_ON_COS = 0.999999
# TOTG's turning-back threshold: control_core's REVERSAL_COS, -0.99999, a turn of about
# 179.74 degrees.
TURNING_BACK_COS = rust_constants(REPOSITORY / "control_core" / "src" / "path_timing.rs")[
    "REVERSAL_COS"
]


def velocity_limits(parsed, joints):
    return [float(parsed.find(f"joint[@name='{joint}']/limit").get("velocity")) for joint in joints]


def unit(v):
    norm = math.sqrt(sum(x * x for x in v))
    return [x / norm for x in v]


def random_direction(rng, dimension):
    """A unit direction drawn uniformly over the sphere."""
    return unit([rng.gauss(0.0, 1.0) for _ in range(dimension)])


def turned(rng, direction, turn):
    """A unit direction `turn` radians from the unit `direction`, toward a direction
    across it drawn uniformly."""
    drawn = random_direction(rng, len(direction))
    along = sum(x * d for x, d in zip(drawn, direction))
    across = unit([x - along * d for x, d in zip(drawn, direction)])
    return [math.cos(turn) * d + math.sin(turn) * a for d, a in zip(direction, across)]


def corner_cos(before, corner, after):
    """The cosine of the turn at `corner` between the leg from `before` and the leg to
    `after`."""
    leg_in = unit([q - p for p, q in zip(before, corner)])
    leg_out = unit([q - p for p, q in zip(corner, after)])
    return sum(x * y for x, y in zip(leg_in, leg_out))


def turns_cleanly(waypoints):
    return all(
        TURNING_BACK_COS < corner_cos(a, b, c) <= STRAIGHT_ON_COS
        for a, b, c in zip(waypoints, waypoints[1:], waypoints[2:])
    )


def drawn_waypoints(rng, limits, count, accepted=turns_cleanly):
    """`count` waypoints drawn uniformly inside `limits`, drawn again until `accepted`
    takes them; by default, until every corner turns cleanly."""
    while True:
        waypoints = [[rng.uniform(low, high) for low, high in limits] for _ in range(count)]
        if accepted(waypoints):
            return waypoints


def cruises(waypoints, max_velocity):
    """Whether the straight run between the two `waypoints` reaches its velocity limit
    under `max_velocity` and MAX_ACCELERATION: whether it is longer than the distance in
    which it reaches that speed from rest and stops from it again."""
    start, end = waypoints
    length = math.dist(start, end)
    shares = [abs(b - a) / length for a, b in zip(start, end)]
    speed = min(v / share for v, share in zip(max_velocity, shares) if share)
    acceleration = MAX_ACCELERATION / max(shares)
    return speed * speed / acceleration < length


def walked_waypoints(rng, limits, legs, next_direction, accepted=turns_cleanly):
    """Waypoints from a point drawn inside `limits` along legs of the lengths `legs`, the
    first in a direction drawn uniformly and each next one `next_direction(direction)`,
    drawn again until every waypoint lies inside the limits and `accepted` takes them;
    by default, until every corner turns cleanly."""
    while True:
        point = [rng.uniform(low, high) for low, high in limits]
        direction = random_direction(rng, len(limits))
        waypoints = [point]
        for k, leg in enumerate(legs):
            direction = next_direction(direction) if k else direction
            point = [p + leg * d for p, d in zip(point, direction)]
            waypoints.append(point)
        inside = all(low <= x <= high for q in waypoints for x, (low, high) in zip(q, limits))
        if inside and accepted(waypoints):
            return waypoints


def short_leg_waypoints(rng, limits):
    legs = [rng.uniform(*(SHORT_LEGS if k % 2 else LONG_LEGS)) for k in range(SHORT_LEG_LEGS)]
    return walked_waypoints(rng, limits, legs, lambda _: random_direction(rng, len(limits)))


def near_reversal_turn(rng):
    """A turn (rad) of 180 degrees less a shortfall drawn log-uniformly over
    NEAR_REVERSAL_SHORTFALLS."""
    low, high = (math.log(degrees) for degrees in NEAR_REVERSAL_SHORTFALLS)
    return math.radians(180.0 - math.exp(rng.uniform(low, high)))


def near_reversal_waypoints(rng, limits):
    legs = [rng.uniform(*NEAR_REVERSAL_LENGTHS) for _ in range(NEAR_REVERSAL_LEGS)]
    return walked_waypoints(
        rng, limits, legs, lambda direction: turned(rng, direction, near_reversal_turn(rng))
    )


def one_corner_waypoints(rng, limits, degrees):
    """Waypoints along two legs of one length drawn over NEAR_REVERSAL_LENGTHS, the second
    turned by `degrees` from the first, whichever side of TOTG's thresholds that turn
    lies."""
    leg = rng.uniform(*NEAR_REVERSAL_LENGTHS)
    turn = math.radians(degrees)
    return walked_waypoints(
        rng, limits, [leg, leg], lambda direction: turned(rng, direction, turn), lambda _: True
    )


def turn(before, corner, after):
    """The turn (rad) at `corner` between the leg from `before` and the leg to `after`."""
    return math.acos(max(-1.0, min(1.0, corner_cos(before, corner, after))))


def tangent_length(turn_rad, leg_in, leg_out, max_deviation):
    """How far from a corner that turns by `turn_rad` between legs of the lengths `leg_in`
    and `leg_out` TOTG's arc meets each leg: the lesser of half each leg and the distance
    at which the arc's midpoint lies `max_deviation` from the corner (CircularPathSegment,
    moveit2 2.15.2); 0 at a corner that does not turn."""
    half_turn = 0.5 * turn_rad
    if half_turn == 0.0:
        return 0.0
    at_deviation = max_deviation * math.sin(half_turn) / (1.0 - math.cos(half_turn))
    return min(leg_in / 2, leg_out / 2, at_deviation)


def pieces(waypoints, max_deviation):
    """The pieces of the path through `waypoints` blended at `max_deviation`, in their
    order, each its length and the shares of it at which TOTG gives its point: a straight
    piece along each leg, and at each corner an arc that turns with the corner, of radius
    its tangent length over tan(turn / 2)."""
    turns = [turn(*corner) for corner in zip(waypoints, waypoints[1:], waypoints[2:])]
    legs = [math.dist(a, b) for a, b in zip(waypoints, waypoints[1:])]
    tangents = [
        tangent_length(turn_rad, leg_in, leg_out, max_deviation)
        for turn_rad, leg_in, leg_out in zip(turns, legs, legs[1:])
    ]
    cuts = [0.0, *tangents, 0.0]
    straights = [leg - before - after for leg, before, after in zip(legs, cuts, cuts[1:])]
    arcs = [
        turn_rad * tangent / math.tan(0.5 * turn_rad) if tangent else 0.0
        for turn_rad, tangent in zip(turns, tangents)
    ]
    laid = [(straights[0], STRAIGHT_SHARES)]
    for arc, straight in zip(arcs, straights[1:]):
        laid += [(arc, ARC_SHARES), (straight, STRAIGHT_SHARES)]
    return laid


def piece_fractions(waypoints, max_deviation):
    """The fractions of the path's length at which TOTG gives its point: its start, its
    end, and the shares of each piece of positive length, so every arc holds three
    points, the corner TOTG runs straight on among them."""
    laid = pieces(waypoints, max_deviation)
    total = sum(length for length, _ in laid)
    starts = [sum(length for length, _ in laid[:k]) for k in range(len(laid))]
    inner = [
        (start + share * length) / total
        for start, (length, shares) in zip(starts, laid)
        if length > 0.0
        for share in shares
    ]
    return [0.0, *inner, 1.0]


def path(name, waypoints, max_deviation, max_velocity):
    return {
        "name": name,
        "waypoints": waypoints,
        "max_deviation": max_deviation,
        "max_velocity": max_velocity,
        "max_acceleration": [MAX_ACCELERATION] * len(max_velocity),
        "fractions": piece_fractions(waypoints, max_deviation),
    }


def drawn_paths(rng, limits, max_velocity, k):
    """The paths of the drawn cases, numbered `k`."""
    cruising = [CRUISING_SCALE * v for v in max_velocity]
    cornered = drawn_waypoints(rng, limits, CORNERED_WAYPOINTS)
    return [
        *(
            path(f"cornered_{k}_deviation_{deviation}", cornered, deviation, max_velocity)
            for deviation in MAX_DEVIATIONS
        ),
        path(
            f"short_legs_{k}", short_leg_waypoints(rng, limits), JOINT_PATH_DEVIATION, max_velocity
        ),
        path(
            f"near_reversal_{k}",
            near_reversal_waypoints(rng, limits),
            JOINT_PATH_DEVIATION,
            max_velocity,
        ),
        path(f"straight_{k}", drawn_waypoints(rng, limits, 2), JOINT_PATH_DEVIATION, max_velocity),
        path(
            f"straight_cruising_{k}",
            drawn_waypoints(rng, limits, 2, lambda run: cruises(run, cruising)),
            JOINT_PATH_DEVIATION,
            cruising,
        ),
    ]


def main():
    drawn = options(__doc__, DRAWN_PATHS, "paths drawn of each drawn case").parse_args().drawn
    rng = generator(NAME)
    parsed = robot(URDF)
    joints = openarm_joints("left")
    limits = [joint_limits(parsed, joint) for joint in joints]
    max_velocity = velocity_limits(parsed, joints)
    paths = [p for k in range(drawn) for p in drawn_paths(rng, limits, max_velocity, k)]
    out, back = drawn_waypoints(rng, limits, 2)
    paths.append(path("turning_back", [out, back, out], JOINT_PATH_DEVIATION, max_velocity))
    for name, degrees in {**REVERSAL_BOUNDARY_TURNS, **NEAR_STRAIGHT_TURNS}.items():
        waypoints = one_corner_waypoints(rng, limits, degrees)
        paths.append(path(name, waypoints, JOINT_PATH_DEVIATION, max_velocity))
    write(NAME, {"kind": "path_timing", "paths": paths})


if __name__ == "__main__":
    main()
