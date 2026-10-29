"""Parity with MoveIt's time-optimal trajectory generation (TOTG) on the paths
of every path timing scenario of the ROS reference harness, which the reader
ros_reference reads as ros_reference/src/path_timing.rs does, blended at
deviations from 0.01 to 0.2, the deviation of execute_joint_path among them:
where TOTG blends every corner, the rounded path is TOTG's blended path, point
for point at equal arc length; where TOTG runs a corner straight on, a turn of
under about 0.081 degrees, the rounded path rounds it and lies within the
path's deviation of TOTG's; and its timing is no faster than TOTG's, to a part
in 1e9, and within the grid's slack of it on a straight run. A path that TOTG refuses because it
turns back on itself comes to rest at that waypoint and turns back. TOTG's
turning-back threshold, and REVERSAL_COS, lie between the turns of the
scenario's reversal-boundary paths. The scenarios hold no repeated waypoint:
MoveIt's computeTimeStamps removes repeated points before it builds TOTG's path
(moveit2 2.15.2), and the rest at a repeated waypoint is joint_path's own rule,
which test_joint_path checks."""

import collections
import itertools
import math

import ros_reference
from ros_reference import PathCase, PathTimingScenario, Refused, Timed

from control_core_py.joint_path import (
    JOINT_PATH_CORNER_DEVIATION,
    REVERSAL_COS,
    Arc,
    RoundedPath,
    TimedPath,
)

# The arc and the straight pieces are computed by different formulas here and
# in TOTG (a turn from the start of the arc against a center and two axes), so
# the points agree to the rounding of those formulas over paths a few radians
# long.
POINT_TOLERANCE = 1e-12

# The deviations a scenario blends its cornered paths at.
SCENARIO_DEVIATIONS = {0.01, 0.05, 0.1, 0.2}

# The slack of the timing grid on a straight run: a straight run's time lies
# within three parts in a thousand of its trapezoid (CELLS_PER_PATH), and
# TOTG's lies on its trapezoid to its millisecond integration step.
STRAIGHT_RUN_SLACK = 1.003

# The rounding of a duration computed by different formulas here and in TOTG,
# relative to the duration: on a straight run both land on the run's trapezoid
# time.
DURATION_ROUNDING = 1e-9

# How near an arc's tangent length or its midpoint's distance from its corner lies to a
# bound, relative to the bound, to meet it: the rounding of the arc's formulas.
BOUND_TOLERANCE = 1e-9

# The cosine of a turn above which TOTG runs a corner straight on through its
# waypoint, a turn of under about 0.081 degrees (CircularPathSegment, moveit2
# 2.15.2).
TOTG_STRAIGHT_ON_COS = 0.999999

# The widest a scenario's turns may bracket TOTG's turning-back threshold, in
# degrees: the sharpest turn TOTG blends and the mildest it refuses lie within
# this of each other.
TURNING_BACK_BRACKET_DEG = 0.02


def scenarios() -> list[PathTimingScenario]:
    """Every path timing scenario of the harness."""
    found = ros_reference.path_timing_scenarios()
    assert found, "the harness holds a path timing scenario"
    return found


def rounded(case: PathCase) -> RoundedPath:
    return RoundedPath.through(case.waypoints, case.max_deviation)


def timed(case: PathCase) -> TimedPath:
    """The path of `case`, rounded at its deviation and timed under its
    limits."""
    return TimedPath.along(rounded(case), tuple(case.max_velocity), tuple(case.max_acceleration))


def corner_cos(before: list[float], corner: list[float], after: list[float]) -> float:
    """The cosine of the turn of the corner at `corner` between the legs from
    `before` and to `after`."""
    leg_in = [(c - b) / math.dist(before, corner) for b, c in zip(before, corner, strict=True)]
    leg_out = [(a - c) / math.dist(corner, after) for c, a in zip(corner, after, strict=True)]
    return sum(x * y for x, y in zip(leg_in, leg_out, strict=True))


def turn_deg(before: list[float], corner: list[float], after: list[float]) -> float:
    """The turn, in degrees, of the corner at `corner` between the legs from
    `before` and to `after`."""
    return math.degrees(math.acos(min(max(corner_cos(before, corner, after), -1.0), 1.0)))


def has_a_straight_on_corner(case: PathCase) -> bool:
    """Whether TOTG runs a corner of `case` straight on."""
    return any(
        corner_cos(*case.waypoints[k - 1 : k + 2]) > TOTG_STRAIGHT_ON_COS
        for k in range(1, len(case.waypoints) - 1)
    )


def sharpest_corner(case: PathCase) -> tuple[int, float] | None:
    """The waypoint of `case`'s sharpest corner, and its turn in degrees; None
    for a path with no corner."""
    turns = [
        (k, turn_deg(*case.waypoints[k - 1 : k + 2])) for k in range(1, len(case.waypoints) - 1)
    ]
    return max(turns, key=lambda turn: turn[1], default=None)


# On every path TOTG blends each corner of, the rounded path has TOTG's length,
# and its point at each fraction of that length is TOTG's. The cornered paths
# compared span the scenario's deviations.
def test_the_rounded_path_is_totgs_blended_path():
    for scenario in scenarios():
        deviations: set[float] = set()
        for case in scenario.paths:
            if not isinstance(case.totg, Timed) or has_a_straight_on_corner(case):
                continue
            length = case.totg.length
            path = rounded(case)
            at = f"{scenario.name} {case.name}"
            assert abs(path.length - length) <= POINT_TOLERANCE * max(length, 1.0), (
                f"{at}: length {path.length} against TOTG's {length}"
            )
            for fraction, config in zip(case.fractions, case.totg.configs, strict=True):
                off = math.dist(path.at(fraction * length), config)
                assert off <= POINT_TOLERANCE, f"{at}: {off} off TOTG at {fraction}"
            if len(case.waypoints) > 2:
                deviations.add(case.max_deviation)
        assert deviations == SCENARIO_DEVIATIONS, scenario.name


# TOTG runs a corner that turns by less than its straight-on threshold straight
# on through its waypoint, so its path is longer than the rounded path, which
# rounds the corner on an arc within the path's deviation; at each fraction of
# TOTG's length, the rounded path lies within that deviation of TOTG's point.
def test_a_corner_totg_runs_straight_on_is_rounded_within_the_deviation():
    for scenario in scenarios():
        cases = [case for case in scenario.paths if has_a_straight_on_corner(case)]
        assert cases, f"{scenario.name}: no path with a corner TOTG runs straight on"
        for case in cases:
            at = f"{scenario.name} {case.name}"
            assert isinstance(case.totg, Timed), f"{at}: TOTG refuses a path it runs straight on"
            length = case.totg.length
            path = rounded(case)
            assert length - path.length > POINT_TOLERANCE, (
                f"{at}: TOTG's length {length} against the rounded path's {path.length}"
            )
            for fraction, config in zip(case.fractions, case.totg.configs, strict=True):
                off = math.dist(path.at(fraction * length), config)
                assert off <= case.max_deviation, f"{at}: {off} off TOTG at {fraction}"


# The timing is no faster than TOTG's on any path, to a part in 1e9, and on a
# straight run no slower than the grid's slack over it.
def test_the_timing_is_no_faster_than_totg_and_matches_it_on_a_straight_run():
    for scenario in scenarios():
        straight_runs = 0
        for case in scenario.paths:
            if not isinstance(case.totg, Timed):
                continue
            totg = case.totg.duration_s
            ours = timed(case).duration_s
            at = f"{scenario.name} {case.name}"
            assert ours >= totg * (1.0 - DURATION_ROUNDING), f"{at}: {ours} s, TOTG {totg} s"
            if len(case.waypoints) == 2:
                assert ours <= totg * STRAIGHT_RUN_SLACK, f"{at}: {ours} s, TOTG {totg} s"
                straight_runs += 1
        assert straight_runs > 0, f"{scenario.name}: no straight run compared"


# A path TOTG refuses because it turns back on itself comes to rest at the
# waypoint of its sharpest corner: its timing passes the cell boundary at that
# waypoint at speed 0. It runs no farther from the waypoint than where it
# starts.
def test_a_path_totg_refuses_for_turning_back_comes_to_rest_and_turns_back():
    for scenario in scenarios():
        refused = [case for case in scenario.paths if isinstance(case.totg, Refused)]
        assert refused, f"{scenario.name}: the scenario holds a path that turns back"
        for case in refused:
            k, _ = sharpest_corner(case)
            start, turn = case.waypoints[0], case.waypoints[k]
            path = timed(case)
            at = f"{scenario.name} {case.name}"
            boundary = min(
                path.cells, key=lambda cell: math.dist(path.path.at(cell.start_at), turn)
            )
            off = math.dist(path.path.at(boundary.start_at), turn)
            assert off <= POINT_TOLERANCE, f"{at}: no cell boundary at its turn, {off} off"
            assert boundary.speed_in == 0.0, f"{at}: reaches its turn at {boundary.speed_in}"
            samples = [path.at_fraction(j / 1000) for j in range(1001)]
            assert all(math.dist(q, turn) <= math.dist(start, turn) + 1e-9 for q in samples), (
                f"{at}: the path runs past the waypoint it turns back at"
            )


# TOTG refuses a path whose sharpest corner turns farther than that of every
# path it blends, and the scenario brackets that turning-back threshold within
# TURNING_BACK_BRACKET_DEG. REVERSAL_COS lies in the bracket, so the path
# rounds each corner TOTG blends and stops at each TOTG refuses.
def test_totgs_turning_back_threshold_lies_between_the_scenarios_turns():
    for scenario in scenarios():

        def sharpest(kind: type) -> list[float]:
            corners = (
                sharpest_corner(case) for case in scenario.paths if isinstance(case.totg, kind)
            )
            return [turn for _, turn in filter(None, corners)]

        blended, refused = max(sharpest(Timed)), min(sharpest(Refused))
        assert blended < refused, (
            f"{scenario.name}: TOTG blends a turn of {blended} degrees and refuses one of {refused}"
        )
        assert refused - blended <= TURNING_BACK_BRACKET_DEG, (scenario.name, blended, refused)
        assert blended < math.degrees(math.acos(REVERSAL_COS)) <= refused, scenario.name


def binding_corners(scenario: PathTimingScenario, max_deviation: float) -> collections.Counter:
    """Of the corners TOTG blends at `max_deviation` in `scenario`: how many
    meet each bound, "half leg" where the arc's tangent points lie half the
    shorter leg from the corner and "deviation" where its midpoint lies the
    deviation from it, and "meeting", how many pairs of neighboring arcs meet
    at the midpoint of their leg."""
    counts: collections.Counter = collections.Counter()
    for case in scenario.paths:
        if case.max_deviation != max_deviation or not isinstance(case.totg, Timed):
            continue
        waypoints = case.waypoints
        arcs = [
            placed.piece
            for placed in RoundedPath.through(waypoints, max_deviation).pieces
            if isinstance(placed.piece, Arc)
        ]
        assert len(arcs) == len(waypoints) - 2, f"{case.name}: an arc at each corner"
        for before, corner, after, arc in zip(
            waypoints, waypoints[1:], waypoints[2:], arcs, strict=False
        ):
            half_leg = min(math.dist(before, corner), math.dist(corner, after)) / 2.0
            tangent = math.dist(arc.start, corner)
            deviation = math.dist(arc.at(arc.length / 2.0), corner)
            counts["half leg"] += abs(tangent - half_leg) <= BOUND_TOLERANCE * half_leg
            counts["deviation"] += abs(deviation - max_deviation) <= BOUND_TOLERANCE * max_deviation
        counts["meeting"] += sum(
            math.dist(first.at(first.length), second.start) <= POINT_TOLERANCE
            for first, second in itertools.pairwise(arcs)
        )
    return counts


# At the deviation of execute_joint_path, the corners TOTG blends in each
# scenario meet each bound, and neighboring arcs meet at the midpoint of a
# leg, so the comparison above covers each.
def test_the_scenario_binds_each_bound_at_the_joint_path_deviation():
    for scenario in scenarios():
        counts = binding_corners(scenario, JOINT_PATH_CORNER_DEVIATION)
        assert all(counts[bound] > 0 for bound in ("half leg", "deviation", "meeting")), (
            scenario.name,
            counts,
        )
