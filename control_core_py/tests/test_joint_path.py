"""Joint paths: the rounded geometry of execute_joint_path, where it rests,
and the time-optimal timing that plays it."""

import importlib.util
import itertools
import math
from pathlib import Path

import pytest
from hypothesis import assume, example, given
from hypothesis import strategies as st

from control_core_py import joint_path
from control_core_py.joint_path import (
    CELLS_PER_PATH,
    JOINT_PATH_CORNER_DEVIATION,
    MIN_JOINT_CHANGE_RAD,
    REVERSAL_COS,
    STRAIGHT_SIN,
    Arc,
    RoundedPath,
    Straight,
)

REPOSITORY = Path(__file__).resolve().parents[2]
# control_core's path timing, whose constants joint_path holds.
CONTROL_CORE_PATH_TIMING = REPOSITORY / "control_core" / "src" / "path_timing.rs"


def module_at(path: Path):
    """The Python module in the file at `path`, loaded."""
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ros_reference's drawing helpers, whose parser of a Rust file's constants and
# whose sum in Rust's order the scenario scripts share.
drawing = module_at(REPOSITORY / "ros_reference" / "tools" / "drawing.py")

JOINTS = 4
ORIGIN = (0.0,) * JOINTS
VELOCITY_CAPS = (1.0,) * JOINTS
ACCELERATION_CAPS = (1.0,) * JOINTS

# Rounding slack on a bound the geometry or the timing meets exactly.
SLACK = 1.0 + 1e-9
# Rounding slack on a rate measured as a finite difference of samples.
RATE_SLACK = 1.0 + 1e-6


def steps(least: int):
    """Steps of up to 0.5 in each joint on a grid of thousandths, at least
    `least` thousandths in some joint."""
    return st.tuples(*[st.integers(-500, 500)] * JOINTS).filter(
        lambda step: max(map(abs, step)) >= least
    )


def walk(steps_taken) -> tuple[tuple[float, ...], ...]:
    """The origin and each waypoint `steps_taken`, in thousandths, reach from
    it in turn."""
    return tuple(
        itertools.accumulate(
            (tuple(v / 1000.0 for v in step) for step in steps_taken),
            lambda q, step: tuple(a + b for a, b in zip(q, step, strict=True)),
            initial=ORIGIN,
        )
    )


@st.composite
def paths(draw, least_step: int = 1):
    """Up to twelve waypoints from the origin, each a step of at least
    `least_step` thousandths in some joint from the last, so no two
    consecutive are equal; the origin alone when no step is drawn."""
    return walk(draw(st.lists(steps(least_step), max_size=11)))


@st.composite
def joint_paths(draw):
    """Up to twelve waypoints of a joint path from the origin, each a step
    from the last, with whether it is a near step: of at most 0.0009 in every
    joint, which repeats the waypoint given before it, or of at least 0.002 in
    some joint, which does not."""
    near = st.tuples(*[st.integers(-9, 9)] * JOINTS).map(lambda step: (step, True))
    far = steps(least=2).map(lambda step: (tuple(10 * v for v in step), False))
    drawn = draw(st.lists(st.one_of(near, far), min_size=1, max_size=11))
    # Each step in ten-thousandths.
    waypoints = itertools.accumulate(
        (tuple(v / 10_000.0 for v in step) for step, _ in drawn),
        lambda q, step: tuple(a + b for a, b in zip(q, step, strict=True)),
        initial=ORIGIN,
    )
    return tuple(zip(waypoints, (False, *(is_near for _, is_near in drawn)), strict=True))


def velocity_caps():
    return st.tuples(*[st.floats(0.1, 5.0)] * JOINTS)


def acceleration_caps():
    return st.tuples(*[st.floats(0.1, 20.0)] * JOINTS)


def distance_to_segment(point, a, b) -> float:
    """The distance from `point` to the segment from `a` to `b`."""
    run = tuple(y - x for x, y in zip(a, b, strict=True))
    length_sq = sum(v * v for v in run)
    offset = tuple(p - x for p, x in zip(point, a, strict=True))
    t = min(max(sum(o * r for o, r in zip(offset, run, strict=True)) / length_sq, 0.0), 1.0)
    return math.dist(point, tuple(x + r * t for x, r in zip(a, run, strict=True)))


def rests(profile: joint_path.Profile) -> list[tuple[float, ...]]:
    """Where `profile` comes to rest: its start, and the end of each cell its
    timing leaves at no speed, its end among them."""
    return [
        profile.start,
        *(
            profile.path.at(cell.start_at + cell.length)
            for cell in profile.timed.cells
            if cell.speed_out == 0.0
        ),
    ]


def among(point, points) -> bool:
    """Whether `point` lies on one of `points`, to the rounding of a path's
    pieces."""
    return any(math.dist(point, p) <= 1e-12 for p in points)


def trace(profile: joint_path.Profile, samples: int) -> tuple[float, list[tuple[float, ...]]]:
    """The time step of `samples` steps through `profile`, and its
    configuration at each of them, its start and its end included."""
    dt = profile.duration_s / samples
    return dt, [profile.sample(k * dt) for k in range(samples + 1)]


def past_the_caps(samples, dt, velocity, acceleration) -> str | None:
    """The first sample, of configurations sampled `dt` apart, where a joint
    moves or accelerates past its cap: None when the motion keeps its caps."""
    for k, (a, b, c) in enumerate(zip(samples, samples[1:], samples[2:], strict=False)):
        for i in range(len(velocity)):
            speed = abs(b[i] - a[i]) / dt
            if speed > velocity[i] * RATE_SLACK:
                return f"joint {i} at {speed} rad/s past its {velocity[i]} at sample {k}"
            rate = abs(c[i] - 2.0 * b[i] + a[i]) / dt**2
            if rate > acceleration[i] * RATE_SLACK:
                return f"joint {i} at {rate} rad/s^2 past its {acceleration[i]} at sample {k}"
    return None


def arcs_with_corners(path: RoundedPath) -> list[tuple[Arc, int]]:
    """Each arc of `path`, through waypoints no two consecutive of which are
    the same point, with the index of the waypoint whose corner it rounds:
    the inner corners in order, each one an arc or the end of a straight run
    that no arc follows."""
    arcs = []
    corner = 1
    for placed, after in itertools.zip_longest(path.pieces, path.pieces[1:]):
        if isinstance(placed.piece, Arc):
            arcs.append((placed.piece, corner))
            corner += 1
        elif after is None or not isinstance(after.piece, Arc):
            corner += 1
    return arcs


def rounded_within_the_joint_deviation(points) -> tuple[RoundedPath, RoundedPath]:
    """The joint path through `points`, and the path through them rounded
    within its deviation that rests at equal waypoints alone."""
    return (
        RoundedPath.joint_path(points),
        RoundedPath.through(points, JOINT_PATH_CORNER_DEVIATION),
    )


# On any path rounded within JOINT_PATH_CORNER_DEVIATION, the joint path or
# the one resting at equal waypoints alone, each arc meets both legs of its
# corner at one distance from it, its midpoint lies at most the deviation
# from the corner, its tangent points lie within half of each leg, and one of
# the two bounds holds with equality. The example comes back through its own
# waypoints, around a square of thousandths and on along its first side, and
# pairs each arc with the corner it rounds at the later visit of the point.
@given(paths())
@example(walk([(1, 0, 0, 0), (0, 1, 0, 0), (-1, 0, 0, 0), (0, -1, 0, 0), (1, 0, 0, 0)]))
def test_each_corner_arc_is_the_largest_within_both_bounds(points):
    for path in rounded_within_the_joint_deviation(points):
        for arc, k in arcs_with_corners(path):
            before, corner, after = points[k - 1], points[k], points[k + 1]
            arc_end = arc.at(arc.length)
            assert distance_to_segment(arc.start, before, corner) <= 1e-9
            assert distance_to_segment(arc_end, corner, after) <= 1e-9
            tangent = math.dist(arc.start, corner)
            assert math.dist(arc_end, corner) == pytest.approx(tangent, rel=1e-9)
            deviation = math.dist(arc.at(arc.length / 2.0), corner)
            half_leg = min(math.dist(corner, before), math.dist(after, corner)) / 2.0
            assert deviation <= JOINT_PATH_CORNER_DEVIATION * SLACK
            assert tangent <= half_leg * SLACK
            assert (
                abs(deviation - JOINT_PATH_CORNER_DEVIATION) <= 1e-9
                or abs(tangent - half_leg) <= 1e-9 * (1.0 + half_leg)
            ), f"an arc {deviation} from its corner, meeting its legs {tangent} from it"


# A waypoint that repeats the one before it is a rest: the path stops on it.
@given(paths(least_step=2), st.data())
def test_a_repeated_waypoint_is_a_rest(points, data):
    assume(len(points) >= 3)
    k = data.draw(st.integers(1, len(points) - 2), label="repeated")
    repeated = (*points[: k + 1], *points[k:])
    profile = joint_path.plan(repeated, 0.0, VELOCITY_CAPS, ACCELERATION_CAPS)
    assert among(points[k], rests(profile))


# A waypoint where the path turns back on itself is a rest: the path stops on
# it and goes back along the leg it came by.
@given(paths(least_step=2), st.floats(1.0, 2.0))
def test_a_reversal_is_a_rest(points, back):
    assume(len(points) >= 2)
    before, turn = points[-2], points[-1]
    reversed_to = tuple(t + (b - t) * back for t, b in zip(turn, before, strict=True))
    profile = joint_path.plan((*points, reversed_to), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS)
    assert among(turn, rests(profile))


def turning(cos: float, sin: float):
    """A path from the origin to a corner half a unit along the first joint,
    and on from it by half of (`cos`, `sin`) in the first two joints: a turn
    of that cosine and sine when the two make a unit vector."""
    return (ORIGIN, (0.5, 0.0, 0.0, 0.0), (0.5 + 0.5 * cos, 0.5 * sin, 0.0, 0.0))


def laid(path: RoundedPath) -> list[tuple[type, bool]]:
    """The kind of each piece of `path`, and whether the path rests after it."""
    return [(type(placed.piece), placed.rests_after) for placed in path.pieces]


ROUNDED_CORNER = [(Straight, False), (Arc, False), (Straight, True)]


# REVERSAL_COS against TOTG's turning-back tolerance, from both sides: a corner
# turning by 179.735 degrees is rounded, as TOTG blends it, and one turning by
# 179.75 keeps its point and the path rests there, where TOTG refuses the path.
# A corner whose cosine is exactly REVERSAL_COS rests too, and one whose cosine
# lies the least float above it is rounded.
def test_the_reversal_cosine_is_totgs_turning_back_tolerance():
    def turning_by(degrees):
        turn = math.radians(degrees)
        return RoundedPath.joint_path(turning(math.cos(turn), math.sin(turn)))

    rests_at_its_corner = [(Straight, True), (Straight, True)]
    assert laid(turning_by(179.735)) == ROUNDED_CORNER
    assert laid(turning_by(179.75)) == rests_at_its_corner

    # From the origin back along (cos, sin) after a unit run along the first
    # joint, where cos^2 + sin^2 rounds to 1, so the turn's cosine is cos.
    def back(cos, sin):
        assert cos * cos + sin * sin == 1.0
        return RoundedPath.joint_path(((-1.0, 0.0, 0.0, 0.0), ORIGIN, (cos, sin, 0.0, 0.0)))

    assert laid(back(REVERSAL_COS, 0.004472124774634615)) == rests_at_its_corner
    above = math.nextafter(REVERSAL_COS, 0.0)
    assert laid(back(above, 0.00447212477465944)) == ROUNDED_CORNER


# RoundedPath.joint_path rests at a waypoint within MIN_JOINT_CHANGE_RAD of the
# one before it, which the path through the same waypoints at the joint path's
# deviation rounds; plan times the joint path.
def test_the_joint_path_rests_at_a_near_repeat_that_the_rounding_alone_rounds():
    near = (0.0005, 0.0)
    waypoints = ((0.0, 1.0), (0.0, 0.0), near, (0.0005, 1.0))
    joint, through = rounded_within_the_joint_deviation(waypoints)

    def rests_of(path):
        return [p.piece.at(p.piece.length) for p in path.pieces if p.rests_after]

    assert among(near, rests_of(joint))
    assert not among(near, rests_of(through))
    assert joint_path.plan(waypoints, 0.0, (1.0, 1.0), (1.0, 1.0)).path == joint


# STRAIGHT_SIN from both sides: a corner whose turn has a sine of 1e-10 runs
# straight on through its point, and one of 1e-8 is rounded.
def test_a_corner_within_the_straight_sine_runs_straight_on_and_one_past_it_is_rounded():
    def path(sin):
        return RoundedPath.through(turning(1.0, sin), JOINT_PATH_CORNER_DEVIATION)

    assert laid(path(1e-10)) == [(Straight, False), (Straight, True)]
    assert laid(path(1e-8)) == ROUNDED_CORNER


# A joint path rests at each waypoint within MIN_JOINT_CHANGE_RAD in every
# joint of the waypoint given before it, and at its last, each where it is
# given; and every point it rests at is a waypoint it is given.
@given(joint_paths())
def test_a_joint_path_rests_at_each_waypoint_that_repeats_the_one_given_before_it(drawn):
    points = tuple(q for q, _ in drawn)
    found = rests(joint_path.plan(points, 0.0, VELOCITY_CAPS, ACCELERATION_CAPS))
    for q in [*(q for q, near in drawn if near), points[-1]]:
        assert among(q, found), f"no rest at {q}, only at {found}"
    for rest in found:
        assert among(rest, points), f"a rest at {rest}, no waypoint"


def joint_path_rests(waypoints):
    """Where the joint path through `waypoints`, of two joints, rests."""
    return rests(joint_path.plan(waypoints, 0.0, (1.0, 1.0), (1.0, 1.0)))


# MIN_JOINT_CHANGE_RAD is inclusive: a waypoint exactly that far from the one
# given before it repeats it, and the path rests there; one a hair farther
# does not, and the path rounds it.
def test_a_waypoint_exactly_the_least_change_from_the_one_before_it_repeats_it():
    def through(x):
        return joint_path_rests(((0.0, 1.0), (0.0, 0.0), (x, 0.0), (x, 1.0)))

    assert among((MIN_JOINT_CHANGE_RAD, 0.0), through(MIN_JOINT_CHANGE_RAD))
    past = math.nextafter(MIN_JOINT_CHANGE_RAD, math.inf)
    assert not among((past, 0.0), through(past))


# Each waypoint is judged against the waypoint given before it: through near
# steps that add up past MIN_JOINT_CHANGE_RAD, the path rests at each of them.
def test_a_joint_path_rests_at_each_of_a_run_of_near_steps():
    found = joint_path_rests(((0.0, 0.0), (0.0006, 0.0), (0.0012, 0.0), (0.0012, 1.0)))
    assert len(found) == 4, f"rests at {found}"
    assert among((0.0006, 0.0), found) and among((0.0012, 0.0), found)


# A profile through any waypoints, near repeats among them, under any caps and
# request, keeps every joint within its velocity and acceleration caps:
# sampled densely through its duration, every joint's velocity and
# acceleration, as finite differences of the samples, stay within its caps.
@given(joint_paths(), velocity_caps(), acceleration_caps(), st.floats(0.0, 10.0))
def test_a_profile_keeps_every_joint_within_its_caps(drawn, velocity, acceleration, requested):
    profile = joint_path.plan(tuple(q for q, _ in drawn), requested, velocity, acceleration)
    assume(profile.duration_s > 0.0)
    dt, samples = trace(profile, 2000)
    past = past_the_caps(samples, dt, velocity, acceleration)
    assert past is None, past


# Through the same two legs, a corner that turns by 170 degrees runs slower
# than one that turns by 90, its arc's curvature taking the acceleration
# caps, and each keeps its caps through its arc.
def test_a_tight_corner_runs_slower_than_a_wide_one():
    def profile(degrees):
        return joint_path.plan(turning_by(degrees), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS)

    tight, wide = profile(170.0), profile(90.0)
    assert tight.duration_s > wide.duration_s
    for corner in (tight, wide):
        dt, samples = trace(corner, 20_000)
        past = past_the_caps(samples, dt, VELOCITY_CAPS, ACCELERATION_CAPS)
        assert past is None, past


# A profile starts on its first waypoint and holds exactly its last from its
# duration on.
@given(paths(), velocity_caps(), acceleration_caps(), st.floats(0.0, 10.0))
def test_a_profile_runs_from_its_first_waypoint_to_exactly_its_last(
    points, velocity, acceleration, requested
):
    profile = joint_path.plan(points, requested, velocity, acceleration)
    assert (profile.start, profile.end) == (points[0], points[-1])
    assert profile.sample(-1.0) == points[0]
    assert profile.sample(0.0) == points[0]
    assert profile.sample(profile.duration_s) == points[-1]
    assert profile.sample(profile.duration_s + 5.0) == points[-1]
    assert profile.done(profile.duration_s)
    if profile.duration_s > 0.0:
        assert not profile.done(profile.duration_s * (1.0 - 1e-9))


# A requested duration above the path's own time is the profile's duration,
# and the profile plays the fastest one slowed by one factor: at each share of
# its duration, where the fastest is at that share of its own. A request below
# the path's own time plays the fastest.
@given(paths(), velocity_caps(), acceleration_caps(), st.floats(0.0, 60.0))
def test_a_requested_duration_slows_the_fastest_profile_by_one_factor(
    points, velocity, acceleration, requested
):
    fastest = joint_path.plan(points, 0.0, velocity, acceleration)
    profile = joint_path.plan(points, requested, velocity, acceleration)
    assert profile.duration_s == max(requested, fastest.duration_s)
    assert profile.timed == fastest.timed
    for k in range(21):
        share = k / 20.0
        off = math.dist(
            profile.sample(share * profile.duration_s), fastest.sample(share * fastest.duration_s)
        )
        assert off <= 1e-12, f"{off} off the fastest profile at {k}/20 of its duration"


# A straight run between two waypoints takes the time of the trapezoid under
# the path's speed and acceleration caps, no less, and no more than the
# grid's three parts in CELLS_PER_PATH.
@given(steps(least=1), velocity_caps(), acceleration_caps())
def test_a_straight_run_takes_its_trapezoid_time(step, velocity, acceleration):
    end = tuple(v / 1000.0 for v in step)
    length = math.hypot(*end)
    direction = tuple(v / length for v in end)

    def cap(caps):
        return min(c / abs(d) for c, d in zip(caps, direction, strict=True) if d != 0.0)

    v, a = cap(velocity), cap(acceleration)
    trapezoid = length / v + v / a if length >= v * v / a else 2.0 * math.sqrt(length / a)
    duration = joint_path.plan((ORIGIN, end), 0.0, velocity, acceleration).duration_s
    assert duration >= trapezoid / RATE_SLACK, f"{duration} s beats the {trapezoid} s trapezoid"
    grid = 1.0 + 3.0 / CELLS_PER_PATH
    assert duration <= trapezoid * grid, f"{duration} s past the grid's slack on {trapezoid} s"


def turning_by(degrees: float):
    """A path from the origin a unit along the first joint, and a unit on
    from there in a direction turned by `degrees` toward the second joint."""
    turn = math.radians(degrees)
    return (ORIGIN, (1.0, 0.0, 0.0, 0.0), (1.0 + math.cos(turn), math.sin(turn), 0.0, 0.0))


# control_core times these joint paths in these durations (s), recorded from
# its JointTrajectory's timed path: a straight run, corners turning by 90 and
# by 170 degrees, and a path through a near repeat and a reversal. joint_path
# times each alike, to the rounding of its sums.
@pytest.mark.parametrize(
    ("waypoints", "velocity", "acceleration", "rust_duration_s"),
    [
        ((ORIGIN, (0.5, -0.3, 0.2, 0.1)), (1.0, 2.0, 0.5, 1.0), (2.0, 1.0, 3.0, 1.0), 1.095445115010333),
        (turning_by(90.0), VELOCITY_CAPS, ACCELERATION_CAPS, 3.520895894988214),
        (turning_by(170.0), VELOCITY_CAPS, ACCELERATION_CAPS, 3.8627311178851618),
        (turning_by(170.0), VELOCITY_CAPS, (5.0,) * JOINTS, 2.2186623550155398),
        (
            (
                ORIGIN,
                (0.4, 0.1, 0.0, 0.0),
                (0.4, 0.5, 0.2, 0.0),
                (0.4005, 0.5003, 0.2, 0.0),
                (0.1, 0.2, 0.3, -0.2),
                (0.3, 0.1, 0.4, -0.3),
                (0.1, 0.2, 0.3, -0.2),
            ),
            (1.0, 1.5, 0.8, 2.0),
            (3.0, 2.0, 4.0, 1.5),
            2.8865787714059836,
        ),
    ],
)
def test_the_ports_time_a_joint_path_alike(waypoints, velocity, acceleration, rust_duration_s):
    duration = joint_path.plan(waypoints, 0.0, velocity, acceleration).duration_s
    assert duration == pytest.approx(rust_duration_s, rel=1e-12)


def test_a_path_through_one_point_takes_the_requested_duration_and_holds_it():
    profile = joint_path.plan((ORIGIN, ORIGIN), 2.0, VELOCITY_CAPS, ACCELERATION_CAPS)
    assert (profile.duration_s, profile.timed.cells) == (2.0, ())
    assert profile.sample(1.0) == ORIGIN
    assert joint_path.plan((ORIGIN,), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS).duration_s == 0.0


@pytest.mark.parametrize(
    ("waypoints", "requested", "velocity", "acceleration", "refusal"),
    [
        ((), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS, "a path has a waypoint"),
        ((ORIGIN, (1.0, 0.0, 0.0)), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS, "one dimension"),
        (((1.0, 0.0, 0.0),), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS, "one dimension"),
        ((ORIGIN,), 0.0, VELOCITY_CAPS, (1.0, 1.0, 1.0), "one dimension"),
        ((ORIGIN, (math.nan, 0.0, 0.0, 0.0)), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS, "finite"),
        ((ORIGIN, (math.inf, 0.0, 0.0, 0.0)), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS, "finite"),
        ((ORIGIN,), math.inf, VELOCITY_CAPS, ACCELERATION_CAPS, "finite and non-negative"),
        ((ORIGIN,), math.nan, VELOCITY_CAPS, ACCELERATION_CAPS, "finite and non-negative"),
        ((ORIGIN,), -1.0, VELOCITY_CAPS, ACCELERATION_CAPS, "finite and non-negative"),
        ((ORIGIN,), 0.0, (1.0, 1.0, 0.0, 1.0), ACCELERATION_CAPS, "velocity caps must be finite"),
        ((ORIGIN,), 0.0, (1.0, 1.0, math.nan, 1.0), ACCELERATION_CAPS, "velocity caps"),
        ((ORIGIN,), 0.0, (1.0, 1.0, math.inf, 1.0), ACCELERATION_CAPS, "velocity caps"),
        ((ORIGIN,), 0.0, VELOCITY_CAPS, (1.0, 0.0, 1.0, 1.0), "acceleration caps must be finite"),
        ((ORIGIN,), 0.0, VELOCITY_CAPS, (1.0, math.nan, 1.0, 1.0), "acceleration caps"),
        ((ORIGIN,), 0.0, VELOCITY_CAPS, (1.0, math.inf, 1.0, 1.0), "acceleration caps"),
    ],
)
def test_a_plan_that_no_profile_honors_is_refused(
    waypoints, requested, velocity, acceleration, refusal
):
    with pytest.raises(ValueError, match=refusal):
        joint_path.plan(waypoints, requested, velocity, acceleration)


@pytest.mark.parametrize(
    ("waypoints", "deviation", "refusal"),
    [
        ((), 0.1, "a path has a waypoint"),
        ((ORIGIN, (1.0, 0.0)), 0.1, "one dimension"),
        ((ORIGIN, (0.0, math.nan, 0.0, 0.0)), 0.1, "finite"),
        ((ORIGIN, (0.0, 0.0, -math.inf, 0.0)), 0.1, "finite"),
        ((ORIGIN,), 0.0, "positive and finite"),
        ((ORIGIN,), math.nan, "positive and finite"),
        ((ORIGIN,), math.inf, "positive and finite"),
    ],
)
def test_a_path_that_no_rounding_makes_is_refused(waypoints, deviation, refusal):
    with pytest.raises(ValueError, match=refusal):
        RoundedPath.through(waypoints, deviation)


@pytest.mark.parametrize("shorter", [0.5, 0.0, math.nan])
def test_a_profile_shorter_than_its_paths_own_time_is_refused(shorter):
    timed = joint_path.plan(
        (ORIGIN, (0.5, 0.0, 0.0, 0.0)), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS
    ).timed
    joint_path.Profile(timed=timed, duration_s=timed.duration_s)
    with pytest.raises(ValueError, match="no faster than its own time"):
        joint_path.Profile(timed=timed, duration_s=shorter * timed.duration_s)


@pytest.mark.parametrize("t", [math.nan, math.inf, -math.inf])
def test_nonfinite_sample_times_are_refused(t):
    profile = joint_path.plan(
        (ORIGIN, (0.5, 0.0, 0.0, 0.0)), 0.0, VELOCITY_CAPS, ACCELERATION_CAPS
    )
    with pytest.raises(ValueError, match="finite"):
        profile.sample(t)


# The constants of joint_path hold control_core's values.
def test_the_constants_are_control_cores():
    held = {
        "JOINT_PATH_CORNER_DEVIATION": JOINT_PATH_CORNER_DEVIATION,
        "REVERSAL_COS": REVERSAL_COS,
        "STRAIGHT_SIN": STRAIGHT_SIN,
        "MIN_JOINT_CHANGE_RAD": MIN_JOINT_CHANGE_RAD,
        "CELLS_PER_PATH": CELLS_PER_PATH,
    }
    declared = drawing.rust_constants(CONTROL_CORE_PATH_TIMING)
    assert {name: declared.get(name) for name in held} == held


def control_core_turn(before, corner, after) -> tuple[float, float]:
    """The cosine and the sine of the turn at `corner`, between the runs from
    `before` and to `after`, as control_core's path timing computes them: each
    run's unit direction its difference times the reciprocal of its length,
    and each sum added in its order."""

    def unit(start, end):
        run = tuple(y - x for x, y in zip(start, end, strict=True))
        length = math.sqrt(drawing.in_order(v * v for v in run))
        return tuple(v * (1.0 / length) for v in run)

    direction_in, direction_out = unit(before, corner), unit(corner, after)
    cos_turn = drawing.in_order(a * b for a, b in zip(direction_in, direction_out, strict=True))
    across = tuple(o - i * cos_turn for i, o in zip(direction_in, direction_out, strict=True))
    return cos_turn, math.sqrt(drawing.in_order(v * v for v in across))


def control_core_corner(cos_turn: float, sin_turn: float) -> list[tuple[type, bool]]:
    """The pieces control_core lays through a corner of that turn, by its own
    constants: a rest at a reversal, a straight run on through a corner in
    line, and an arc otherwise."""
    constants = drawing.rust_constants(CONTROL_CORE_PATH_TIMING)
    if cos_turn <= constants["REVERSAL_COS"]:
        return [(Straight, True), (Straight, True)]
    if sin_turn <= constants["STRAIGHT_SIN"]:
        return [(Straight, False), (Straight, True)]
    return ROUNDED_CORNER


# The two ports lay a corner alike: on drawn corners on a grid of thousandths,
# run straight on, turned back or turned by a step, joint_path rests, runs
# straight on or rounds where control_core does, as control_core's constants
# and its sums in their order judge the corner. The two sums round apart, so a
# corner whose cosine lies within their rounding of REVERSAL_COS is left out.
# The example is a corner in line whose sine control_core's sums round to
# 1.8e-16 and joint_path's to 0.
@given(
    st.tuples(*[st.integers(-500, 500)] * JOINTS),
    steps(least=1),
    st.integers(1, 5),
    st.integers(1, 5),
    st.sampled_from(("ahead", "back", "turned")),
    steps(least=1),
)
@example((0, 0, 0, 0), (-5, -5, -4, -3), 1, 1, "ahead", (1, 0, 0, 0))
def test_the_ports_lay_a_corner_alike(before, step, run_in, run_out, way, turn_step):
    before = tuple(v / 1000.0 for v in before)
    corner = tuple(b + run_in * v / 1000.0 for b, v in zip(before, step, strict=True))
    out = {
        "ahead": step,
        "back": tuple(-v for v in step),
        "turned": tuple(v + w for v, w in zip(step, turn_step, strict=True)),
    }[way]
    assume(any(out))
    after = tuple(c + run_out * v / 1000.0 for c, v in zip(corner, out, strict=True))
    cos_turn, sin_turn = control_core_turn(before, corner, after)
    assume(abs(cos_turn - REVERSAL_COS) > 1e-12)
    path = RoundedPath.through((before, corner, after), JOINT_PATH_CORNER_DEVIATION)
    assert laid(path) == control_core_corner(cos_turn, sin_turn)
