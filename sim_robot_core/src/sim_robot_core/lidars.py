"""The 2-D lidars of a robot, shared by the simulation engines.

Reads how a model's entry sweeps each lidar, builds the configs of a robot's
lidars from the lidars of its description, with the ranges each measures, and
those sweeps, gives when each ray of a scan is due, the way it points in the
lidar's frame and its range in lidar_2d's marks, and sweeps a lidar on the
engine's clock (`LidarSweeper`): which rays each step casts, from where the
lidar's frame stands at each ray's time, and the scans they complete. Casting
a ray, and where the lidar's frame stands at each step, stay in the engine.

A lidar turns once per scan, counterclockwise about its frame's +Z: scan `k`
starts at `k / scan_rate_hz` seconds of the engine's clock, and ray `i` points
`min_angle_rad + i x angle_increment_rad` from the frame's +X in its x-y plane
and is due `round(i x time_increment_s x 1e9)` nanoseconds after its scan
starts, as Waldo sweeps its lidars.

After each step of the engine every ray due since the step before is cast
from the frame's pose at the ray's own time, between the frame's poses at the
two steps: linear in position, spherical in orientation. A scan starts only
while a pair views the lidar. A step that does not move the engine's clock
forward drops the scan in progress, and so does an interruption, which an
engine makes at each rebuild or edit of its scene: the sweep starts again at
the next scan. The capture index counts the scans a sweep completes, across
its interruptions.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from control_core_py.geometry import Placement, UnitQuaternion, normalized, slerp

from sim_robot_core.cameras import CaptureIndexCounter
from sim_robot_core.entry_values import is_number, positive_int, reject_unknown_keys
from sim_robot_core.errors import ModelEntryError

# lidar_2d's two marks, as camera_core's laser_scan gives them: the range of a
# ray not measured and of a ray that met nothing within the lidar's range.
UNMEASURED_MARK_M = 0.0
NO_RETURN_MARK_M = 10000.0
# The most scans a second a sweep takes, as Waldo bounds it.
MAX_SCAN_RATE_HZ = 100

_SWEEP_KEYS = frozenset({"min_angle_rad", "angle_increment_rad", "rays", "scan_rate_hz"})


@dataclass(frozen=True)
class LidarSweep:
    """How the engines sweep one lidar of a model, in lidar_2d's terms: the
    angle of its first ray, the angle from one ray to the next, above 0, the
    rays of one scan, at least 1, and its scans a second. The fan spans at
    most one turn, `(rays - 1) x angle_increment_rad <= 2 pi`: at one turn its
    last ray points where its first does, one scan later."""

    min_angle_rad: float
    angle_increment_rad: float
    rays: int
    scan_rate_hz: int


@dataclass(frozen=True)
class LidarConfig:
    """One 2-D lidar a robot carries: its name, its frame and the link that
    frame is fixed to, with its pose in that link's frame, the orientation in
    the engines' order (w, x, y, z); how it sweeps, and the ranges it
    measures, its description's."""

    name: str
    # The lidar's frame in the robot's tree, the link its sensor references,
    # which sensor_mounts gives for the lidar's name and each scan names.
    frame: str
    parent_link: str
    pos: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    sweep: LidarSweep
    min_range_m: float
    max_range_m: float

    @property
    def time_increment_s(self) -> float:
        """The time from one ray to the next (s): the time the lidar takes to
        turn by one angle increment at one turn a scan."""
        return self.sweep.angle_increment_rad / (2.0 * math.pi * self.sweep.scan_rate_hz)

    @property
    def period_ns(self) -> int:
        """The time from the start of one scan to the start of the next (ns)."""
        return round(1e9 / self.sweep.scan_rate_hz)

    def ray_angle(self, ray: int) -> float:
        """The angle of ray `ray` from the frame's +X, counterclockwise (rad)."""
        return self.sweep.min_angle_rad + ray * self.sweep.angle_increment_rad

    def ray_direction(self, ray: int) -> tuple[float, float, float]:
        """The unit vector ray `ray` points along, in the lidar's frame."""
        angle = self.ray_angle(ray)
        return (math.cos(angle), math.sin(angle), 0.0)

    def ray_due_ns(self, ray: int) -> int:
        """When ray `ray` is due after its scan starts (ns)."""
        return round(ray * self.time_increment_s * 1e9)

    def range_of(self, distance: float | None) -> float:
        """The range of a ray whose first surface lies `distance` meters along
        it, None for a ray that meets nothing, in lidar_2d's marks: the
        distance within the lidar's range, NO_RETURN_MARK_M past it or for
        nothing met, and UNMEASURED_MARK_M for a surface nearer than its
        nearest range."""
        if distance is None or distance > self.max_range_m:
            return NO_RETURN_MARK_M
        if distance < self.min_range_m:
            return UNMEASURED_MARK_M
        return distance


Vector = tuple[float, float, float]
# How an engine casts a ray: from an origin along a unit direction, both in
# world, as far as a range (m), to the distance of the first surface the ray
# meets (m), or None where it meets none.
Cast = Callable[[Vector, Vector, float], float | None]


@dataclass(frozen=True)
class Scan:
    """One scan a sweep completes: when its ray 0 was due on the engine's
    clock (ns), its capture index, and the range of each ray in ray order,
    in lidar_2d's marks (m)."""

    start_ns: int
    capture_index: int
    ranges: tuple[float, ...]

    @property
    def start_s(self) -> float:
        """When its ray 0 was due on the engine's clock (s)."""
        return self.start_ns / 1e9


@dataclass(frozen=True)
class _FramePose:
    """Where a lidar's frame stands in world at an instant of the engine's
    clock (ns)."""

    at_ns: int
    placement: Placement

    def toward(self, later: _FramePose, at_ns: int) -> Placement:
        """The frame's placement at `at_ns`, between this pose and `later`:
        linear in position, spherical in orientation."""
        share = (at_ns - self.at_ns) / (later.at_ns - self.at_ns)
        start, end = self.placement, later.placement
        position = tuple(
            a + share * (b - a) for a, b in zip(start.position, end.position, strict=True)
        )
        turn = slerp(start.orientation.xyzw, end.orientation.xyzw, share)
        return Placement(position, UnitQuaternion(normalized(turn)))


@dataclass
class _ScanInProgress:
    """A scan the sweep is measuring: when its ray 0 is due on the engine's
    clock (ns), and the range of each ray cast so far, in ray order."""

    start_ns: int
    ranges: list[float] = field(default_factory=list)


class LidarSweeper:
    """Sweeps one lidar of one robot on the engine's clock: `step` after
    each step of the engine, with the step's engine time, where the lidar's
    frame stands then and how the engine casts a ray, gives the scans the
    step completes, and `interrupt` drops the scan in progress. `viewed`
    says whether a pair views the lidar, which a scan starts only while it
    does."""

    def __init__(self, lidar: LidarConfig, viewed: Callable[[], bool]) -> None:
        self._lidar = lidar
        self._viewed = viewed
        # Each ray: when it is due after its scan starts (ns), and the unit
        # vector it points along in the lidar's frame.
        self._rays = tuple(
            (lidar.ray_due_ns(ray), lidar.ray_direction(ray)) for ray in range(lidar.sweep.rays)
        )
        self._capture_indices = CaptureIndexCounter()
        self._previous: _FramePose | None = None
        self._scan: _ScanInProgress | None = None

    def step(self, now_s: float, frame: Placement, cast: Cast) -> list[Scan]:
        """Casts every ray due since the step before with `cast`, from the
        frame's pose at the ray's time between where it stood then and
        `frame`, where it stands in world at `now_s`, the engine time of this
        step, and gives each scan that completes, in order. The first step
        after the sweeper is made or interrupted casts nothing: it takes
        where the frame stands."""
        now = _FramePose(round(now_s * 1e9), frame)
        previous, self._previous = self._previous, now
        if previous is None:
            return []
        if now.at_ns <= previous.at_ns:
            self._scan = None
            return []
        scans: list[Scan] = []
        scan = self._scan
        start_ns = self._first_start_after(previous.at_ns) if scan is None else scan.start_ns
        while start_ns <= now.at_ns:
            if scan is None and not self._viewed():
                start_ns += self._lidar.period_ns
                continue
            scan = scan or _ScanInProgress(start_ns)
            if not self._cast_due(scan, previous, now, cast):
                break
            scans.append(Scan(scan.start_ns, self._capture_indices.next(), tuple(scan.ranges)))
            scan = None
            start_ns += self._lidar.period_ns
        self._scan = scan
        return scans

    def interrupt(self) -> None:
        """Drops the scan in progress and where the frame stood: the next
        step only takes where it stands."""
        self._previous = None
        self._scan = None

    def _first_start_after(self, at_ns: int) -> int:
        """The first scan start after `at_ns`: a whole number of periods of
        the engine's clock."""
        period_ns = self._lidar.period_ns
        return (at_ns // period_ns + 1) * period_ns

    def _cast_due(
        self, scan: _ScanInProgress, previous: _FramePose, now: _FramePose, cast: Cast
    ) -> bool:
        """Casts each ray of `scan` due by `now`, from the frame's pose at the
        ray's time. True once every ray of the scan is cast."""
        lidar = self._lidar
        for due_ns, direction in self._rays[len(scan.ranges) :]:
            at_ns = scan.start_ns + due_ns
            if at_ns > now.at_ns:
                return False
            frame = previous.toward(now, at_ns)
            distance = cast(frame.position, frame.orientation.rotate(direction), lidar.max_range_m)
            scan.ranges.append(lidar.range_of(distance))
        return True


def parse_sweeps(source: str, entries) -> dict[str, LidarSweep]:
    """How the engines sweep each lidar of a model, by the lidar's name:
    `entries`, a model entry's map of lidar names to how each sweeps, read
    once. A bad sweep is refused, so a typo never distorts a scan. `source`
    names the entry in every refusal."""
    if not isinstance(entries, dict):
        raise ModelEntryError(f"{source}: 'lidars' must map lidar names to entries, got {entries!r}")
    return {name: _parse_sweep(source, name, entry) for name, entry in entries.items()}


def lidar_configs(
    source: str, description, sweeps: dict[str, LidarSweep] | None
) -> tuple[LidarConfig, ...]:
    """The configs of a robot's lidars: each lidar of its description
    (robot_description_py's RobotDescription), placed by the fixed joint that
    holds its frame to its link, measuring the ranges the description gives
    it, swept as the lidar of the same name in `sweeps`, the sweeps of the
    entry at `source`, or None where no file is there. A lidar of the
    description with no sweep and a sweep of no lidar of the description are
    refused naming the entry, or saying to create it where no file is there,
    so a typo never drops a scan."""
    lidars = description.lidars
    names = [lidar.name for lidar in lidars]
    if sweeps is None and names:
        raise ModelEntryError(
            f"the lidars of the description are {sorted(names)}, and no entry sweeps them: "
            f"create {source} with `lidars`, one sweep per lidar of the description, under its name"
        )
    swept = sweeps or {}
    if set(names) != set(swept):
        raise ModelEntryError(
            f"{source}: the lidars of the description are {sorted(names)}, and the entry "
            f"sweeps {sorted(swept)}: give the entry's `lidars` one sweep per lidar of the "
            "description, under its name"
        )
    placed = {transform.child: transform for transform in description.fixed_transforms()}
    return tuple(_config(source, lidar, placed, swept[lidar.name]) for lidar in lidars)


def _config(source: str, lidar, placed, sweep: LidarSweep) -> LidarConfig:
    """The config of `lidar`, whose frame the fixed transform in `placed`
    places in its link."""
    if lidar.range.max >= NO_RETURN_MARK_M:
        raise ModelEntryError(
            f"{source}: lidar '{lidar.name}' measures to {lidar.range.max} m, and lidar_2d "
            f"marks {NO_RETURN_MARK_M} m as no return: give its URDF sensor a <max> below it"
        )
    mount = placed[lidar.frame]
    x, y, z = (float(coordinate) for coordinate in mount.position)
    qx, qy, qz, qw = (float(coordinate) for coordinate in mount.orientation)
    return LidarConfig(
        name=lidar.name,
        frame=lidar.frame,
        parent_link=mount.parent,
        pos=(x, y, z),
        quat_wxyz=(qw, qx, qy, qz),
        sweep=sweep,
        min_range_m=lidar.range.min,
        max_range_m=lidar.range.max,
    )


def _parse_sweep(source: str, name: str, entry) -> LidarSweep:
    def fail(reason: str) -> ModelEntryError:
        return ModelEntryError(f"{source}: lidar '{name}': {reason}")

    if not isinstance(entry, dict):
        raise fail(f"its entry must be an object, got {entry!r}")
    reject_unknown_keys(entry, _SWEEP_KEYS, "lidar entry", fail)
    min_angle = entry.get("min_angle_rad")
    if not is_number(min_angle) or not math.isfinite(min_angle):
        raise fail(f"min_angle_rad {min_angle!r} must be a finite number")
    increment = entry.get("angle_increment_rad")
    if not is_number(increment) or not (math.isfinite(increment) and increment > 0.0):
        raise fail(f"angle_increment_rad {increment!r} must be a finite number above 0")
    rays = positive_int(entry.get("rays"), fail, "rays")
    if (rays - 1) * increment > 2.0 * math.pi:
        raise fail(
            f"its fan of {rays} rays {increment} rad apart spans more than one turn: "
            "(rays - 1) x angle_increment_rad must be at most 2 pi"
        )
    rate = positive_int(entry.get("scan_rate_hz"), fail, "scan_rate_hz")
    if rate > MAX_SCAN_RATE_HZ:
        raise fail(f"scan_rate_hz {rate} must be at most {MAX_SCAN_RATE_HZ}")
    return LidarSweep(
        min_angle_rad=float(min_angle),
        angle_increment_rad=float(increment),
        rays=rays,
        scan_rate_hz=rate,
    )
