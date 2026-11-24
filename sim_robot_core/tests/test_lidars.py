"""Tests for the lidar plumbing: a robot's lidars from its description's
lidars and a model entry's sweeps, when each ray of a scan is due and where
it points, and a ray's range in lidar_2d's marks."""

import math
from dataclasses import dataclass, field

import pytest
from robot_description_py import model

from sim_robot_core.errors import ModelEntryError
from sim_robot_core.lidars import (
    NO_RETURN_MARK_M,
    UNMEASURED_MARK_M,
    LidarSweep,
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
