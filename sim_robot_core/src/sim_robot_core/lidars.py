"""The 2-D lidars of a robot, shared by the simulation engines.

Reads how a model's entry sweeps each lidar, builds the configs of a robot's
lidars from the lidars of its description, with the ranges each measures, and
those sweeps, and gives when each ray of a scan is due, the way it points in
the lidar's frame and its range in lidar_2d's marks; casting a ray stays in
the engine.

A lidar turns once per scan, counterclockwise about its frame's +Z: scan `k`
starts at `k / scan_rate_hz` seconds of the engine's clock, and ray `i` points
`min_angle_rad + i x angle_increment_rad` from the frame's +X in its x-y plane
and is due `round(i x time_increment_s x 1e9)` nanoseconds after its scan
starts, as Waldo sweeps its lidars.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

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
