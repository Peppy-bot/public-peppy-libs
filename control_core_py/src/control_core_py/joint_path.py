"""Joint paths of limb_motion's execute_joint_path, and the time-optimal
timing that plays them under per-joint velocity and acceleration caps.

This is control_core's path_timing, ported. The path runs straight between its
waypoints, and each inner corner whose turn has a sine above STRAIGHT_SIN is
cut by the largest circular arc tangent to both legs whose midpoint lies at
most a deviation from the corner and whose tangent points lie at most half of
each leg from it. MoveIt's time-optimal trajectory generation (TOTG) blends a
corner so where it turns by about 0.081 degrees or more (a cosine of at most
0.999999), and runs a milder one straight on through its waypoint. The path
rests at its last waypoint; where it turns back on itself, where the cosine of
the angle between one leg and the next is REVERSAL_COS, -0.99999, or less, a
turn of about 179.74 degrees; and at a waypoint that repeats the waypoint given
before it: at no distance from it, or, on a joint path, within
MIN_JOINT_CHANGE_RAD of it in every joint.

The timing follows Kunz and Stilman ("Time-Optimal Trajectory Generation for
Path Following with Bounded Acceleration and Velocity", RSS 2012), as TOTG
does. The speed along the path is integrated over a grid of cells: a backward
pass from rest at the end finds the fastest each cell boundary can be passed
at and still stop by the end, and a forward pass from rest at the start then
runs each cell as hard as the caps allow within that. A boundary where the
path rests is passed at rest. The path acceleration is constant over a cell,
and a cell's bounds take each joint's share of the tangent and its curvature
at their largest over the cell, so the caps hold at every point of it: the
curvature of an arc slows the path there alone. A requested duration above
the path's own time plays the same timing slowed by one factor.
"""

from __future__ import annotations

import bisect
import dataclasses
import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass

from control_core_py import minimum_jerk

_Vector = tuple[float, ...]

# The farthest the midpoint of a corner's arc lies from the corner on a path
# of execute_joint_path, Euclidean over the joints (rad, or m for a prismatic
# joint).
JOINT_PATH_CORNER_DEVIATION = 0.1

# The cosine of the angle between one leg and the next at or below which the
# path turns back on itself at their corner: -0.99999, a turn of about 179.74
# degrees. The path rests there and turns back. TOTG refuses a path with such
# a turn (its angle_tolerance, moveit2 2.15.2 Path::create).
REVERSAL_COS = -1.0 + 1e-5

# The sine of a turn at or below which a corner runs straight on: the runs
# meet at its point with no arc. It lies a hundred times above the rounding of
# the sine between runs in line a ten-thousandth long, so this port, whose
# sums round otherwise, and control_core judge such a corner alike.
STRAIGHT_SIN = 1e-9

# The most a waypoint of a joint path differs, in every joint, from the
# waypoint given before it and still repeats it, so the path rests there.
MIN_JOINT_CHANGE_RAD = 0.001

# The grid the speed along the path is integrated on: each piece of the path
# is split evenly into cells of at most this share of the path's length, two
# at least, so a piece the path rests at both ends of has a boundary that
# carries speed. The speed switches between accelerating, cruising and
# braking only at cell boundaries, so each switch costs at most one cell's
# time over the optimum; a straight run has two and takes its trapezoid time
# to within three parts in this many.
CELLS_PER_PATH = 1000


def _dot(a: _Vector, b: _Vector) -> float:
    return sum(x * y for x, y in zip(a, b, strict=True))


def _norm(a: _Vector) -> float:
    return math.sqrt(_dot(a, a))


def _scaled(a: _Vector, k: float) -> _Vector:
    return tuple(v * k for v in a)


def _sum(a: _Vector, b: _Vector) -> _Vector:
    return tuple(x + y for x, y in zip(a, b, strict=True))


def _difference(a: _Vector, b: _Vector) -> _Vector:
    return tuple(x - y for x, y in zip(a, b, strict=True))


def _max_abs_sinusoid(a: float, b: float, start: float, stop: float) -> float:
    """The largest |a cos(angle) + b sin(angle)| over the angles `start` to
    `stop`: the amplitude where a peak lies inside the span, the larger end
    otherwise."""
    # a cos(x) + b sin(x) = amplitude cos(x + phase)
    amplitude = math.hypot(a, b)
    phase = math.atan2(-b, a)
    # A peak of |cos| sits at every multiple of pi from -phase.
    first_peak = -phase + math.pi * math.ceil((start + phase) / math.pi)
    if first_peak <= stop:
        return amplitude
    return max(
        abs(a * math.cos(angle) + b * math.sin(angle)) for angle in (start, stop)
    )


@dataclass(frozen=True)
class Straight:
    """A straight run of the path: from `start` along the unit `direction`
    for `length`."""

    start: _Vector
    direction: _Vector
    length: float

    def at(self, distance: float) -> _Vector:
        """The configuration `distance` along the run."""
        return _sum(self.start, _scaled(self.direction, distance))

    def max_abs_tangent(self, start: float, stop: float) -> _Vector:
        """The largest share of the unit tangent each joint has anywhere
        between `start` and `stop` along the run: its share of the run's
        direction."""
        return tuple(abs(d) for d in self.direction)

    def max_abs_curvature(self, start: float, stop: float) -> _Vector:
        """The largest curvature each joint has anywhere between `start` and
        `stop` along the run: none."""
        return (0.0,) * len(self.direction)


@dataclass(frozen=True)
class Arc:
    """The arc that rounds a corner: from `start` for `turn` radians at
    `radius`. `tangent` is the unit tangent at `start` and `normal` the unit
    vector from `start` toward the center."""

    start: _Vector
    tangent: _Vector
    normal: _Vector
    radius: float
    turn: float

    @property
    def length(self) -> float:
        return self.radius * self.turn

    def at(self, distance: float) -> _Vector:
        """The configuration `distance` along the arc."""
        angle = distance / self.radius
        # radius (1 - cos(angle)), in the form that keeps its precision when
        # the angle is small.
        rise = 2.0 * self.radius * math.sin(0.5 * angle) ** 2
        return _sum(
            self.start,
            _sum(_scaled(self.tangent, self.radius * math.sin(angle)), _scaled(self.normal, rise)),
        )

    def max_abs_tangent(self, start: float, stop: float) -> _Vector:
        """The largest share of the unit tangent each joint has anywhere
        between `start` and `stop` along the arc."""
        # tangent_i(angle) = tangent_i cos(angle) + normal_i sin(angle)
        return tuple(
            _max_abs_sinusoid(t, n, start / self.radius, stop / self.radius)
            for t, n in zip(self.tangent, self.normal, strict=True)
        )

    def max_abs_curvature(self, start: float, stop: float) -> _Vector:
        """The largest curvature each joint has anywhere between `start` and
        `stop` along the arc: the rate of its share of the tangent along the
        path."""
        # curvature_i(angle) = (normal_i cos(angle) - tangent_i sin(angle)) / radius
        return tuple(
            _max_abs_sinusoid(
                n / self.radius, -t / self.radius, start / self.radius, stop / self.radius
            )
            for t, n in zip(self.tangent, self.normal, strict=True)
        )


_Piece = Straight | Arc


@dataclass(frozen=True)
class _Placed:
    """A piece of the path, the path position (arc length from 0) it starts
    at, and whether the path rests where the piece ends."""

    start_at: float
    piece: _Piece
    rests_after: bool


def _lay(placed: list[_Placed], piece: _Piece, rests_after: bool) -> None:
    """Lay `piece` where the last of `placed` ends, the path resting at its
    end when `rests_after`. A piece of no length goes, and a rest at its end
    moves to the piece before it, which ends at the same point."""
    if piece.length > 0.0:
        start_at = placed[-1].start_at + placed[-1].piece.length if placed else 0.0
        placed.append(_Placed(start_at, piece, rests_after))
    elif placed:
        placed[-1] = dataclasses.replace(
            placed[-1], rests_after=placed[-1].rests_after or rests_after
        )


def _tangent_length(turn: float, run_in: float, run_out: float, max_deviation: float) -> float:
    """How far from a corner turning by `turn` radians, between runs of
    `run_in` and `run_out`, the arc meets each run: TOTG's blend distance
    (CircularPathSegment, moveit2 2.15.2), at most half the shorter run."""
    half = 0.5 * turn
    versine = 1.0 - math.cos(half)
    # A turn too small for its cosine to leave 1 sets no bound by the
    # deviation: the runs alone bound its arc.
    by_deviation = max_deviation * math.sin(half) / versine if versine > 0.0 else math.inf
    return min(min(run_in, run_out) / 2.0, by_deviation)


def _pieces(
    points: Sequence[_Vector], stops: Sequence[bool], max_deviation: float
) -> tuple[_Placed, ...]:
    """The pieces of the path through `points`, no two consecutive ones the
    same point: a straight run between each pair, and at each corner an arc
    tangent to both runs, meeting each at its tangent length. A corner the
    path reverses at, or one `stops` marks, keeps its point, where the path
    rests, and a corner within STRAIGHT_SIN of straight on keeps its point,
    which the path runs through."""
    placed: list[_Placed] = []
    start = points[0]
    for k in range(1, len(points)):
        corner = points[k]
        direction_in = _difference(corner, start)
        length_in = _norm(direction_in)
        direction_in = _scaled(direction_in, 1.0 / length_in)
        if k + 1 == len(points):
            _lay(placed, Straight(start, direction_in, length_in), True)
            break
        run_in = _norm(_difference(corner, points[k - 1]))
        direction_out = _difference(points[k + 1], corner)
        length_out = _norm(direction_out)
        direction_out = _scaled(direction_out, 1.0 / length_out)
        cos_turn = _dot(direction_in, direction_out)
        # The part of the outgoing direction across the incoming one: its
        # length is the sine of the turn, and it points toward the center.
        across = _difference(direction_out, _scaled(direction_in, cos_turn))
        sin_turn = _norm(across)
        rests = stops[k] or cos_turn <= REVERSAL_COS
        if rests or sin_turn <= STRAIGHT_SIN:
            _lay(placed, Straight(start, direction_in, length_in), rests)
            start = corner
            continue
        turn = math.atan2(sin_turn, cos_turn)
        tangent_length = _tangent_length(turn, run_in, length_out, max_deviation)
        radius = tangent_length / math.tan(turn / 2.0)
        _lay(placed, Straight(start, direction_in, length_in - tangent_length), False)
        arc_start = _sum(corner, _scaled(direction_in, -tangent_length))
        normal = _scaled(across, 1.0 / sin_turn)
        _lay(placed, Arc(arc_start, direction_in, normal, radius, turn), False)
        start = _sum(corner, _scaled(direction_out, tangent_length))
    return tuple(placed)


def _lies_within(waypoint: _Vector, before: _Vector, within: float) -> bool:
    """Whether `waypoint` lies within `within` of `before` in every joint."""
    return all(abs(x - y) <= within for x, y in zip(waypoint, before, strict=True))


def _resting_points(
    waypoints: Sequence[_Vector], repeat_within: float
) -> tuple[list[_Vector], list[bool]]:
    """Each point the path through `waypoints` runs through, and whether it
    rests there: at each waypoint within `repeat_within` in every joint of the
    waypoint given before it. A waypoint at no distance from the one before it
    is the same point, where the path rests."""
    points = [waypoints[0]]
    stops = [False]
    for before, waypoint in itertools.pairwise(waypoints):
        if _norm(_difference(waypoint, before)) == 0.0:
            stops[-1] = True
        else:
            points.append(waypoint)
            stops.append(_lies_within(waypoint, before, repeat_within))
    return points, stops


@dataclass(frozen=True)
class RoundedPath:
    """The geometric path through waypoints: straight runs, each corner
    rounded on an arc within a deviation, and at a corner the path reverses
    at, a rest and a turn back. A waypoint at no distance from the one before
    it repeats it, and the path rests there."""

    pieces: tuple[_Placed, ...]
    start: _Vector
    end: _Vector

    @classmethod
    def through(cls, waypoints: Sequence[Sequence[float]], max_deviation: float) -> RoundedPath:
        """The path through `waypoints` (one or more, finite, of one
        dimension), each corner's arc midpoint at most `max_deviation`
        (positive) from it."""
        return cls._resting_at_repeats(waypoints, max_deviation, 0.0)

    @classmethod
    def joint_path(cls, waypoints: Sequence[Sequence[float]]) -> RoundedPath:
        """The path of limb_motion's execute_joint_path through `waypoints`
        (one or more, finite, of one dimension): each corner rounded within
        JOINT_PATH_CORNER_DEVIATION, resting where it turns back on itself and
        at each waypoint within MIN_JOINT_CHANGE_RAD in every joint of the
        waypoint given before it. plan times this path."""
        return cls._resting_at_repeats(
            waypoints, JOINT_PATH_CORNER_DEVIATION, MIN_JOINT_CHANGE_RAD
        )

    @classmethod
    def _resting_at_repeats(
        cls, waypoints: Sequence[Sequence[float]], max_deviation: float, repeat_within: float
    ) -> RoundedPath:
        """The path through `waypoints` (one or more, finite, of one
        dimension), each corner's arc midpoint at most `max_deviation`
        (positive) from it, resting at each waypoint within `repeat_within` in
        every joint of the waypoint given before it."""
        if not waypoints:
            raise ValueError("a path has a waypoint")
        if len({len(w) for w in waypoints}) != 1:
            raise ValueError("the waypoints have one dimension")
        if not all(math.isfinite(v) for w in waypoints for v in w):
            raise ValueError("the waypoints must be finite")
        if not (math.isfinite(max_deviation) and max_deviation > 0.0):
            raise ValueError(f"the deviation must be positive and finite, got {max_deviation}")
        given = [tuple(float(v) for v in w) for w in waypoints]
        points, stops = _resting_points(given, repeat_within)
        return cls(
            pieces=_pieces(points, stops, max_deviation), start=given[0], end=given[-1]
        )

    @property
    def length(self) -> float:
        """The distance along the path from its first waypoint to its last."""
        if not self.pieces:
            return 0.0
        last = self.pieces[-1]
        return last.start_at + last.piece.length

    def at(self, s: float) -> _Vector:
        """The point at `s` along the path, held to the path's ends: its first
        waypoint at or before 0 and its last at or after its length."""
        placed = self._piece_at(s)
        if placed is None:
            return self.end
        return placed.piece.at(min(max(s - placed.start_at, 0.0), placed.piece.length))

    def _piece_at(self, s: float) -> _Placed | None:
        """The piece the point at `s` lies on, or None at or after the path's
        length."""
        if s >= self.length or not self.pieces:
            return None
        after = bisect.bisect_right(self.pieces, s, key=lambda placed: placed.start_at)
        return self.pieces[min(max(after, 1), len(self.pieces)) - 1]


@dataclass(frozen=True)
class _Cell:
    """One cell of the integration grid: its length along the path and, for
    each joint, the largest share of the tangent and the largest curvature it
    has anywhere in the cell."""

    length: float
    tangent_share: _Vector
    curvature: _Vector

    def ceiling(self, velocity_caps: _Vector, acceleration_caps: _Vector) -> float:
        """The fastest the path may run anywhere in the cell: each joint's
        velocity cap over its share of the tangent, and, where the path
        curves, the speed at which the curvature alone takes a joint's whole
        acceleration cap."""
        return min(
            (
                min(
                    v_max / share if share > 0.0 else math.inf,
                    math.sqrt(a_max / curvature) if curvature > 0.0 else math.inf,
                )
                for share, curvature, v_max, a_max in zip(
                    self.tangent_share, self.curvature, velocity_caps, acceleration_caps, strict=True
                )
            ),
            default=math.inf,
        )

    def reach(self, speed: float, acceleration_caps: _Vector) -> float:
        """The speed at the far end of the cell entered at `speed`,
        accelerating along the path as hard as the caps allow at every point
        of the cell: each joint's acceleration cap, less the curvature's share
        at the speed reached, over its share of the tangent."""
        acceleration = min(
            (
                (a_max - curvature * speed * speed) / (share + 2.0 * curvature * self.length)
                for share, curvature, a_max in zip(
                    self.tangent_share, self.curvature, acceleration_caps, strict=True
                )
                if share > 0.0 or curvature > 0.0
            ),
            default=math.inf,
        )
        return math.sqrt(speed * speed + 2.0 * acceleration * self.length)


@dataclass(frozen=True)
class _TimedCell:
    """One cell of the grid as the path plays it: the path position it starts
    at and the time it starts at, its length, and the path speed at each of
    its ends, 0 where the path rests."""

    start_at: float
    length: float
    time_s: float
    speed_in: float
    speed_out: float


def _cells(path: RoundedPath) -> tuple[list[_Cell], list[bool]]:
    """The cells of the grid along `path`, and whether the path rests at each
    cell boundary: at its ends and at each rest between them."""
    cell_length = path.length / CELLS_PER_PATH
    cells: list[_Cell] = []
    rests = [True]
    for placed in path.pieces:
        piece = placed.piece
        count = max(math.ceil(piece.length / cell_length), 2)
        step = piece.length / count
        for c in range(count):
            start, stop = step * c, min(step * (c + 1), piece.length)
            cells.append(
                _Cell(
                    stop - start,
                    piece.max_abs_tangent(start, stop),
                    piece.max_abs_curvature(start, stop),
                )
            )
            rests.append(False)
        rests[-1] = placed.rests_after
    return cells, rests


@dataclass(frozen=True)
class TimedPath:
    """A path timed from rest to rest under per-joint velocity and
    acceleration caps, as fast as the caps allow on its grid: `duration_s` is
    its own time."""

    path: RoundedPath
    cells: tuple[_TimedCell, ...]
    duration_s: float

    @classmethod
    def along(
        cls, path: RoundedPath, velocity_caps: _Vector, acceleration_caps: _Vector
    ) -> TimedPath:
        """`path` timed under `velocity_caps` and `acceleration_caps`, one of
        each per joint, every one finite and positive."""
        if not len(velocity_caps) == len(acceleration_caps) == len(path.end):
            raise ValueError("the path and both caps must have one dimension")
        minimum_jerk.require_caps(velocity_caps, "velocity")
        minimum_jerk.require_caps(acceleration_caps, "acceleration")
        cells, rests = _cells(path)
        positions = tuple(itertools.accumulate((cell.length for cell in cells), initial=0.0))
        ceilings = [cell.ceiling(velocity_caps, acceleration_caps) for cell in cells]

        def boundary_ceiling(k: int) -> float:
            return 0.0 if rests[k] else min(ceilings[k - 1], ceilings[k])

        # Backward: the fastest each boundary can be passed at and still stop
        # by the end of the path.
        stoppable = [0.0] * (len(cells) + 1)
        for k in reversed(range(len(cells))):
            stoppable[k] = min(
                cells[k].reach(stoppable[k + 1], acceleration_caps), boundary_ceiling(k)
            )
        # Forward: the fastest each boundary is reached at from rest at the
        # start, within what is stoppable from it.
        speeds = [0.0] * (len(cells) + 1)
        for k, cell in enumerate(cells):
            speeds[k + 1] = min(cell.reach(speeds[k], acceleration_caps), stoppable[k + 1])
        # Each cell's time at the mean of its end speeds, the path accelerating
        # evenly over it; a cell is passed at speed, so the mean is positive.
        times = tuple(
            itertools.accumulate(
                (
                    2.0 * cell.length / (speeds[k] + speeds[k + 1])
                    for k, cell in enumerate(cells)
                ),
                initial=0.0,
            )
        )
        timed = tuple(
            _TimedCell(positions[k], cell.length, times[k], speeds[k], speeds[k + 1])
            for k, cell in enumerate(cells)
        )
        return cls(path=path, cells=timed, duration_s=times[-1])

    def at_fraction(self, fraction: float) -> _Vector:
        """The configuration at `fraction` of the duration: the first waypoint
        at or before 0, the last at or after 1, and between them the path at
        the position the speed has reached by then."""
        if fraction >= 1.0 or not self.cells:
            return self.path.end
        t_s = max(fraction, 0.0) * self.duration_s
        after = bisect.bisect_right(self.cells, t_s, key=lambda cell: cell.time_s)
        cell = self.cells[min(max(after, 1), len(self.cells)) - 1]
        acceleration = (cell.speed_out**2 - cell.speed_in**2) / (2.0 * cell.length)
        tau = t_s - cell.time_s
        distance = min(max(cell.speed_in * tau + 0.5 * acceleration * tau * tau, 0.0), cell.length)
        return self.path.at(cell.start_at + distance)


@dataclass(frozen=True)
class Profile:
    """A joint path's timing played over `duration_s`, no shorter than its
    own: a shorter or NaN `duration_s` is refused. Plays as minimum_jerk's
    Profile does: the first waypoint at or before 0, the last from
    `duration_s` on."""

    timed: TimedPath
    duration_s: float

    def __post_init__(self) -> None:
        if not self.duration_s >= self.timed.duration_s:
            raise ValueError(
                f"a path plays no faster than its own time: {self.duration_s} s is under "
                f"its {self.timed.duration_s} s"
            )

    @property
    def path(self) -> RoundedPath:
        return self.timed.path

    @property
    def start(self) -> _Vector:
        return self.path.start

    @property
    def end(self) -> _Vector:
        return self.path.end

    def sample(self, t_s: float) -> _Vector:
        if not math.isfinite(t_s):
            raise ValueError("sample time must be finite")
        if t_s >= self.duration_s or not self.timed.cells:
            return self.end
        return self.timed.at_fraction(max(t_s, 0.0) / self.duration_s)

    def done(self, t_s: float) -> bool:
        return t_s >= self.duration_s


def plan(
    waypoints: Sequence[Sequence[float]],
    requested_duration_s: float,
    velocity_caps: tuple[float, ...],
    acceleration_caps: tuple[float, ...],
) -> Profile:
    """The joint path through `waypoints` (one or more: where the joints
    stand, then the waypoints to pass), as RoundedPath.joint_path rounds it,
    timed under the velocity and acceleration caps and played over the longer
    of the requested duration and its own time; a requested 0 means fastest
    allowed. Refuses what minimum_jerk.require_requested_duration,
    RoundedPath.joint_path and TimedPath.along refuse."""
    minimum_jerk.require_requested_duration(requested_duration_s)
    timed = TimedPath.along(RoundedPath.joint_path(waypoints), velocity_caps, acceleration_caps)
    return Profile(timed=timed, duration_s=max(requested_duration_s, timed.duration_s))
