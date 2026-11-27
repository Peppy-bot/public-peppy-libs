"""Tests for the lidar plumbing: a robot's lidars from its description's
lidars and a model entry's sweeps, when each ray of a scan is due and where
it points, a ray's range in lidar_2d's marks, and the sweep on the engine's
clock: which rays each step casts and from where, the scans it completes and
the ones it drops, their capture index, and the scans no pair views."""

import math
from dataclasses import dataclass, field

import pytest
from control_core_py.geometry import Placement, UnitQuaternion, quaternion_of_yaw
from robot_description_py import model

from sim_robot_core.errors import ModelEntryError
from sim_robot_core.lidars import (
    NO_RETURN_MARK_M,
    UNMEASURED_MARK_M,
    LidarConfig,
    LidarSweep,
    LidarSweeper,
    Scan,
    lidar_configs,
    parse_sweeps,
)
from sim_robot_core.models import shipped_entry


@dataclass(frozen=True)
class Range:
    """The ranges a lidar measures as robot_description_py's LidarRange
    gives them."""

    min: float = 0.1
    max: float = 12.0


@dataclass(frozen=True)
class Lidar:
    """A lidar as robot_description_py's LidarFrame gives one."""

    name: str = "scanner"
    frame: str = "scanner_frame"
    range: Range = Range()


@dataclass(frozen=True)
class Transform:
    """A fixed transform as robot_description_py's LinkTransform gives one."""

    parent: str
    child: str
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Description:
    """A description as robot_description_py's RobotDescription gives its
    lidars and its fixed transforms: each lidar's frame held to the link
    `mount` with `placements`' pose for it, the identity by default."""

    lidars: tuple[Lidar, ...] = ()
    placements: dict[str, Transform] = field(default_factory=dict)

    def fixed_transforms(self) -> tuple[Transform, ...]:
        return tuple(
            self.placements.get(lidar.name, Transform("mount", lidar.frame))
            for lidar in self.lidars
        )


def sweep(**overrides) -> dict:
    fields = {"min_angle_rad": -1.0, "angle_increment_rad": 0.5, "rays": 5, "scan_rate_hz": 10}
    fields.update(overrides)
    return fields


def one_lidar(lidar: Lidar = Lidar(), **placements: Transform):
    """The config of `lidar`, swept as `sweep()`."""
    (config,) = lidar_configs(
        "entry",
        Description(lidars=(lidar,), placements=placements),
        parse_sweeps("entry", {lidar.name: sweep()}),
    )
    return config


class TestSweeps:
    def test_a_sweep_reads_its_four_fields(self):
        assert parse_sweeps("entry", {"scanner": sweep()}) == {
            "scanner": LidarSweep(min_angle_rad=-1.0, angle_increment_rad=0.5, rays=5, scan_rate_hz=10)
        }

    # A fan of one whole turn is taken, its last ray where its first points; one ray more is
    # past the turn.
    def test_a_fan_spans_at_most_one_turn(self):
        turn = sweep(min_angle_rad=-math.pi, angle_increment_rad=2 * math.pi / 8, rays=9)
        assert parse_sweeps("entry", {"scanner": turn})["scanner"].rays == 9
        with pytest.raises(ModelEntryError, match="spans more than one turn"):
            parse_sweeps("entry", {"scanner": {**turn, "rays": 10}})

    @pytest.mark.parametrize(
        ("field", "value", "says"),
        [
            ("min_angle_rad", math.inf, "min_angle_rad inf must be a finite number"),
            ("min_angle_rad", "0", "min_angle_rad '0' must be a finite number"),
            ("angle_increment_rad", 0.0, "angle_increment_rad 0.0 must be a finite number above 0"),
            ("angle_increment_rad", math.nan, "angle_increment_rad nan must be a finite number above 0"),
            ("rays", 0, "rays must be a positive integer, got 0"),
            ("rays", True, "rays must be a positive integer, got True"),
            ("scan_rate_hz", 2.5, "scan_rate_hz must be a positive integer, got 2.5"),
            ("scan_rate_hz", 101, "scan_rate_hz 101 must be at most 100"),
        ],
    )
    def test_a_malformed_field_is_refused_naming_the_lidar_and_the_field(self, field, value, says):
        with pytest.raises(ModelEntryError) as refused:
            parse_sweeps("entry", {"scanner": sweep(**{field: value})})
        assert str(refused.value) == f"entry: lidar 'scanner': {says}"

    def test_an_unknown_key_is_refused(self):
        with pytest.raises(ModelEntryError, match="lidar entry has unknown key\\(s\\) \\['rate'\\]"):
            parse_sweeps("entry", {"scanner": sweep(rate=10)})

    def test_lidars_that_are_not_a_map_are_refused(self):
        with pytest.raises(ModelEntryError, match="'lidars' must map lidar names"):
            parse_sweeps("entry", [sweep()])


class TestConfigs:
    def test_a_lidar_stands_where_its_frames_fixed_joint_places_it(self):
        half = math.sin(0.4), math.cos(0.4)
        config = one_lidar(
            scanner=Transform("chassis", "scanner_frame", (0.3, -0.1, 0.2), (0.0, 0.0, half[0], half[1]))
        )
        assert (config.name, config.frame, config.parent_link) == ("scanner", "scanner_frame", "chassis")
        assert config.pos == (0.3, -0.1, 0.2)
        assert config.quat_wxyz == (half[1], 0.0, 0.0, half[0])
        assert (config.min_range_m, config.max_range_m) == (0.1, 12.0)

    def test_a_lidar_of_the_description_with_no_sweep_is_refused(self):
        with pytest.raises(
            ModelEntryError,
            match="entry: the lidars of the description are \\['scanner'\\], and the entry sweeps "
            "\\[\\]: give the entry's `lidars` one sweep per lidar of the description",
        ):
            lidar_configs("entry", Description(lidars=(Lidar(),)), {})

    def test_a_sweep_of_no_lidar_of_the_description_is_refused(self):
        with pytest.raises(ModelEntryError, match="the entry sweeps \\['other', 'scanner'\\]"):
            lidar_configs(
                "entry",
                Description(lidars=(Lidar(),)),
                parse_sweeps("entry", {"scanner": sweep(), "other": sweep()}),
            )

    def test_a_lidar_of_the_description_with_no_file_says_to_create_it(self):
        with pytest.raises(
            ModelEntryError,
            match="the lidars of the description are \\['scanner'\\], and no entry sweeps them: "
            "create models/aloha.json5 with `lidars`",
        ):
            lidar_configs("models/aloha.json5", Description(lidars=(Lidar(),)), None)

    def test_a_model_with_no_lidar_needs_no_sweep(self):
        assert lidar_configs("models/aloha.json5", Description(), None) == ()
        assert lidar_configs("entry", Description(), {}) == ()

    def test_a_range_to_the_mark_of_no_return_is_refused(self):
        with pytest.raises(ModelEntryError, match="lidar 'scanner' measures to 10000.0 m"):
            one_lidar(Lidar(range=Range(max=NO_RETURN_MARK_M)))


class TestRays:
    # Ray i points min_angle + i x increment from +X, and is due after its scan starts by the
    # time the lidar takes to turn that far at one turn a scan.
    def test_each_ray_points_and_is_due_as_the_lidar_turns(self):
        config = one_lidar()
        assert config.period_ns == 100_000_000
        assert config.time_increment_s == pytest.approx(0.5 / (2 * math.pi * 10), abs=1e-18)
        for ray in range(5):
            angle = -1.0 + 0.5 * ray
            assert config.ray_angle(ray) == angle
            assert config.ray_direction(ray) == (math.cos(angle), math.sin(angle), 0.0)
            assert config.ray_due_ns(ray) == round(ray * 0.5 / (2 * math.pi * 10) * 1e9)

    @pytest.mark.parametrize(
        ("distance", "range_m"),
        [
            (None, NO_RETURN_MARK_M),
            (12.5, NO_RETURN_MARK_M),
            (12.0, 12.0),
            (3.25, 3.25),
            (0.1, 0.1),
            (0.05, UNMEASURED_MARK_M),
        ],
    )
    def test_a_rays_range_is_in_lidar_2ds_marks(self, distance, range_m):
        assert one_lidar().range_of(distance) == range_m


# The bundled botshare base's lidar, the RPLIDAR S3 on its scan frame, swept as the sllidar_ros2
# driver gives its scans: 3240 rays over one whole turn from -pi, ten scans a second.
def test_the_botshare_bases_lidar_is_the_s3_on_its_scan_frame():
    (lidar,) = shipped_entry("botshare_base", model("botshare_base").description()).lidars
    assert (lidar.name, lidar.frame, lidar.parent_link) == ("lidar", "lidar_frame", "lidar_link")
    assert (lidar.pos, lidar.quat_wxyz) == ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert lidar.sweep == LidarSweep(
        min_angle_rad=-math.pi, angle_increment_rad=2 * math.pi / 3239, rays=3240, scan_rate_hz=10
    )
    assert lidar.ray_angle(3239) == math.pi
    assert (lidar.min_range_m, lidar.max_range_m) == (0.05, 40.0)


# Four rays a quarter turn apart at 10 Hz: one ray every 25 ms, one scan every 100 ms.
_RAYS = 4


def quarter_turns(rays=_RAYS, increment=math.pi / 2, min_angle=-math.pi) -> LidarConfig:
    """A lidar sweeping from -pi, as the botshare base's does, unless the test says otherwise,
    measuring from 0.05 m to 40 m."""
    return LidarConfig(
        name="lidar",
        frame="lidar_frame",
        parent_link="lidar_link",
        pos=(0.0, 0.0, 0.0),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        sweep=LidarSweep(min_angle_rad=min_angle, angle_increment_rad=increment, rays=rays, scan_rate_hz=10),
        min_range_m=0.05,
        max_range_m=40.0,
    )


def at(x=0.0, y=0.0, z=0.0, yaw=0.0) -> Placement:
    return Placement((x, y, z), UnitQuaternion(quaternion_of_yaw(yaw)))


class Caster:
    """An engine's cast: records each ray, which meets the distance `meets` gives its index."""

    def __init__(self, meets=lambda ray: None):
        self.casts = []
        self._meets = meets

    def __call__(self, origin, direction, max_range_m):
        self.casts.append((origin, direction, max_range_m))
        return self._meets(len(self.casts) - 1)

    def times(self) -> list:
        """Each ray's origin's x, which is when it was cast where the frame stands at x = t."""
        return [origin[0] for origin, _, _ in self.casts]


class Pairs:
    """Whether a pair views the lidar, and how often the sweeper asked."""

    def __init__(self, viewed=True):
        self.viewed = viewed
        self.asked = 0

    def __call__(self) -> bool:
        self.asked += 1
        return self.viewed


def moving(sweeper, cast, *times_s) -> list[Scan]:
    """Steps `sweeper` at each of `times_s`, its frame standing at x = the step's time, and gives
    every scan the steps complete."""
    return [scan for now_s in times_s for scan in sweeper.step(now_s, at(x=now_s), cast)]


def starts(scans) -> list:
    return [scan.start_s for scan in scans]


class TestWhenRaysAreDue:
    def test_a_scan_starts_on_the_first_whole_period_after_the_first_step(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()

        moving(sweeper, cast, 0.01)
        assert cast.casts == [], "the first step only takes where the frame stands"

        moving(sweeper, cast, 0.13)
        assert cast.times() == pytest.approx([0.1, 0.125])

    def test_each_ray_is_cast_at_its_own_time_one_time_increment_after_the_last(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()

        moving(sweeper, cast, 0.01, 0.13, 0.2)

        # A ray due on a step's own instant is cast on that step.
        assert cast.times() == pytest.approx([0.1, 0.125, 0.15, 0.175, 0.2])
        assert quarter_turns().time_increment_s == pytest.approx(0.025)

    def test_a_scan_completes_once_its_last_ray_is_cast_and_starts_at_its_ray_0(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()

        assert moving(sweeper, cast, 0.01, 0.13) == []
        assert moving(sweeper, cast, 0.2) == [
            Scan(start_ns=100_000_000, capture_index=0, ranges=(NO_RETURN_MARK_M,) * _RAYS)
        ]

    def test_scans_start_one_period_apart_on_the_engine_clock_whatever_the_steps(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()

        # Sixty steps a second, then one step that spans two scans.
        scans = moving(sweeper, cast, *(step / 60 for step in range(31)), 0.85)

        assert starts(scans) == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        assert [scan.capture_index for scan in scans] == [0, 1, 2, 3, 4, 5, 6]


class TestWhereRaysPoint:
    def test_each_ray_leaves_the_frame_along_its_angle_turned_into_world(self):
        # The frame stands at (1, 2.3, 0.1) facing +y.
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()
        frame = at(1.0, 2.3, 0.1, yaw=math.pi / 2)

        sweeper.step(0.0, frame, cast)
        sweeper.step(0.2, frame, cast)

        # The first ray points at -pi in the frame, the ones after it a quarter turn
        # counterclockwise each.
        assert [origin for origin, _, _ in cast.casts[:_RAYS]] == [pytest.approx((1.0, 2.3, 0.1))] * _RAYS
        assert [direction for _, direction, _ in cast.casts[:_RAYS]] == [
            pytest.approx((0.0, -1.0, 0.0), abs=1e-12),
            pytest.approx((1.0, 0.0, 0.0), abs=1e-12),
            pytest.approx((0.0, 1.0, 0.0), abs=1e-12),
            pytest.approx((-1.0, 0.0, 0.0), abs=1e-12),
        ]

    def test_a_ray_takes_the_turn_of_the_frame_at_its_own_time(self):
        # One ray a scan, along the frame's +x, while the frame turns a quarter turn
        # counterclockwise from 0.0 s to 0.2 s.
        sweeper, cast = LidarSweeper(quarter_turns(rays=1, min_angle=0.0), Pairs()), Caster()

        sweeper.step(0.0, at(), cast)
        sweeper.step(0.2, at(yaw=math.pi / 2), cast)

        # Rays due at 0.1 s and 0.2 s: halfway through the turn and at its end.
        half = math.sqrt(0.5)
        assert [direction for _, direction, _ in cast.casts] == [
            pytest.approx((half, half, 0.0)),
            pytest.approx((0.0, 1.0, 0.0), abs=1e-12),
        ]

    def test_each_ray_reaches_as_far_as_the_lidar_measures(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()

        moving(sweeper, cast, 0.0, 0.2)

        assert {max_range_m for _, _, max_range_m in cast.casts} == {40.0}


def test_each_range_is_in_lidar_2ds_marks():
    # Six rays a sixth of a turn apart, each meeting what the list gives it: nearer than 0.05 m,
    # at 0.05 m, within reach, at 40 m, nothing, and past 40 m.
    meets = [0.01, 0.05, 5.0, 40.0, None, 50.0]
    sweeper = LidarSweeper(quarter_turns(rays=6, increment=math.pi / 3), Pairs())

    (scan,) = moving(sweeper, Caster(lambda ray: meets[ray % 6]), 0.0, 0.2)

    assert scan.ranges == (UNMEASURED_MARK_M, 0.05, 5.0, 40.0, NO_RETURN_MARK_M, NO_RETURN_MARK_M)


class TestAScanIsDropped:
    @pytest.mark.parametrize("back_s", [0.13, 0.12], ids=["standing_still", "going_back"])
    def test_by_a_step_that_does_not_move_the_engine_clock_forward(self, back_s):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()
        moving(sweeper, cast, 0.01, 0.13)
        assert len(cast.casts) == 2

        # The scan of 0.1 s is dropped, and the sweep starts again at the next scan.
        scans = moving(sweeper, cast, back_s, 0.31)

        assert starts(scans) == pytest.approx([0.2])
        assert cast.times()[2:] == pytest.approx([0.2, 0.225, 0.25, 0.275, 0.3])

    def test_by_an_interruption(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()
        moving(sweeper, cast, 0.01, 0.13)

        sweeper.interrupt()
        scans = moving(sweeper, cast, 0.24, 0.41)

        # The first step after the interruption only takes where the frame stands, so no ray is
        # cast across it: the scan of 0.2 s is never begun.
        assert starts(scans) == pytest.approx([0.3])
        assert cast.times()[2:] == pytest.approx([0.3, 0.325, 0.35, 0.375, 0.4])

    def test_and_the_capture_index_counts_on_across_it(self):
        sweeper, cast = LidarSweeper(quarter_turns(), Pairs()), Caster()
        before = moving(sweeper, cast, 0.0, 0.29)

        sweeper.interrupt()
        after = moving(sweeper, cast, 0.33, 0.49)

        assert [(scan.start_s, scan.capture_index) for scan in before + after] == [
            pytest.approx((0.1, 0)),
            pytest.approx((0.2, 1)),
            pytest.approx((0.4, 2)),
        ]


class TestAScanNoPairViews:
    def test_is_never_started(self):
        pairs = Pairs(viewed=False)
        sweeper, cast = LidarSweeper(quarter_turns(), pairs), Caster()

        assert moving(sweeper, cast, 0.0, 0.29) == []
        assert cast.casts == []

        # A pair forms: the next scan starts.
        pairs.viewed = True
        scans = moving(sweeper, cast, 0.39)

        assert starts(scans) == pytest.approx([0.3])
        assert [scan.capture_index for scan in scans] == [0]

    def test_is_asked_at_each_scan_start_and_one_begun_completes(self):
        pairs = Pairs()
        sweeper, cast = LidarSweeper(quarter_turns(), pairs), Caster()
        moving(sweeper, cast, 0.0, 0.13)
        assert pairs.asked == 1

        # The pair goes while the scan of 0.1 s is measured: that scan completes, and the next
        # one does not start.
        pairs.viewed = False
        scans = moving(sweeper, cast, 0.21, 0.31)

        assert starts(scans) == pytest.approx([0.1])
        assert cast.times() == pytest.approx([0.1, 0.125, 0.15, 0.175])
        assert pairs.asked == 3
