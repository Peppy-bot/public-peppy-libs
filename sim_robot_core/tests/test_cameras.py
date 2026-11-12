"""Tests for the camera plumbing: a robot's cameras from its description's
optical frames and a model entry's renders, z16 conversion, and pacing."""

import math
import struct
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
import pytest
from robot_description_py import model

from sim_robot_core.cameras import (
    CameraRender,
    DepthSpec,
    DepthStream,
    FrameIdCounter,
    FramePacer,
    camera_configs,
    camera_pose,
    depth_to_z16,
    parse_renders,
)
from sim_robot_core.errors import ModelEntryError
from sim_robot_core.models import shipped_entry


@dataclass(frozen=True)
class Depths:
    """The depths a camera measures as robot_description_py's DepthRange
    gives them."""

    near: float
    far: float


@dataclass(frozen=True)
class Frame:
    """An optical frame as robot_description_py's CameraFrame gives one."""

    name: str = "cam"
    parent_link: str = "link"
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    depth_range: Depths | None = None


def render(**overrides) -> dict:
    fields = {"fovy_deg": 60, "color": {"width": 8, "height": 4}, "fps": 15}
    fields.update(overrides)
    return fields


def one_camera(depth_range: Depths | None = None, **overrides):
    """The config of one camera whose description gives it `depth_range`,
    rendered as `render(**overrides)`."""
    frame = Frame(depth_range=depth_range)
    (config,) = camera_configs("entry", [frame], parse_renders("entry", {"cam": render(**overrides)}))
    return config


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rotate(quat_wxyz, vector):
    """`vector` turned by the unit quaternion `quat_wxyz`."""
    w, x, y, z = quat_wxyz
    axis = (x, y, z)
    twice = tuple(2.0 * c for c in cross(axis, vector))
    turned = cross(axis, twice)
    return tuple(v + w * t + u for v, t, u in zip(vector, twice, turned, strict=True))


class TestCameraPose:
    def test_an_unturned_optical_frame_is_the_camera_frame_turned_half_about_x(self):
        pos, quat = camera_pose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0))
        assert pos == (0.1, 0.2, 0.3)
        assert quat == pytest.approx((0.0, 1.0, 0.0, 0.0))

    @pytest.mark.parametrize(
        "rpy", [(0.3, -0.2, 1.1), (2.5715925867, 0.0, 0.0), (-1.0, 0.7, -2.9)]
    )
    def test_the_camera_looks_along_the_optical_frames_view_with_its_up_against_the_optical_down(
        self, rpy
    ):
        _, quat = camera_pose((0.0, 0.0, 0.0), rpy)
        roll, pitch, yaw = rpy
        # The optical frame's +z and +y in the parent frame, built from the
        # URDF's fixed-axis angles: Rz(yaw) Ry(pitch) Rx(roll).
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw))
        view = (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)
        down = (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr)
        assert rotate(quat, (0.0, 0.0, -1.0)) == pytest.approx(view)
        assert rotate(quat, (0.0, 1.0, 0.0)) == pytest.approx(tuple(-v for v in down))
        assert quat[0] >= 0.0 and math.isclose(sum(v * v for v in quat), 1.0)


class TestCameras:
    def test_the_openarm_v2_rig_parses(self):
        configs = shipped_entry("openarm_v2", model("openarm_v2").description()).cameras
        assert [c.name for c in configs] == ["wrist_left", "wrist_right", "chest"]
        by_name = {c.name: c for c in configs}
        for wrist in ("wrist_left", "wrist_right"):
            assert (by_name[wrist].width, by_name[wrist].height) == (960, 600)
            assert by_name[wrist].depth is None
            assert by_name[wrist].fps == 15
        chest = by_name["chest"]
        assert (chest.width, chest.height) == (1280, 720)
        assert chest.parent_link == "openarm_body_link0"
        # The chest camera looks at the robot's front, 62 degrees below the
        # horizon, as Waldo's head camera derivation reads it off the CAD.
        view = rotate(chest.quat_wxyz, (0.0, 0.0, -1.0))
        assert view[1] == pytest.approx(0.0, abs=1e-6) and view[0] > 0.0
        assert math.degrees(math.atan2(-view[2], view[0])) == pytest.approx(62.0, abs=0.05)
        assert chest.depth is not None
        assert (chest.depth.width, chest.depth.height) == (640, 360)
        # The depths the description's depth sensor on the chest camera gives.
        (described,) = (
            frame.depth_range
            for frame in model("openarm_v2").description().cameras
            if frame.name == "chest"
        )
        assert (chest.depth.min_depth_m, chest.depth.max_range_m) == (described.near, described.far)

    def test_a_camera_takes_its_pose_from_its_frame(self):
        frame = Frame(xyz=(0.0, 0.055, -0.045), rpy=(2.5715925867, 0.0, 0.0))
        (config,) = camera_configs("entry", [frame], parse_renders("entry", {"cam": render()}))
        assert (config.pos, config.quat_wxyz) == camera_pose(frame.xyz, frame.rpy)
        assert config.parent_link == "link"

    def test_a_render_reads_once_into_typed_values(self):
        assert parse_renders("entry", {"cam": render(depth={"width": 4, "height": 2})}) == {
            "cam": CameraRender(
                fovy_deg=60.0, width=8, height=4, fps=15, depth=DepthStream(width=4, height=2)
            )
        }

    def test_a_depth_camera_streams_the_depths_its_description_gives_at_its_renders_size(self):
        config = one_camera(Depths(near=0.1, far=10.0), depth={"width": 4, "height": 2})
        assert config.depth == DepthSpec(width=4, height=2, min_depth_m=0.1, max_range_m=10.0)

    def test_a_camera_rendering_depth_its_description_gives_no_range_says_what_to_write(self):
        with pytest.raises(ModelEntryError) as refused:
            one_camera(depth={"width": 4, "height": 2})
        assert str(refused.value) == (
            "entry: camera 'cam' renders depth, and the description gives it no depth range: "
            'give its URDF <gazebo reference="cam_optical_frame"><sensor name="cam" '
            'type="depth"><camera><clip><near>NEAR</near><far>FAR</far></clip></camera></sensor>'
            "</gazebo>, in meters, or render it without depth"
        )

    def test_a_camera_its_description_gives_a_depth_range_streams_depth(self):
        with pytest.raises(
            ModelEntryError,
            match=r"entry: the description gives camera 'cam' a depth range of 0\.1 to 10\.0 m, and "
            r"its render streams no depth: give it depth: \{ width, height \}",
        ):
            one_camera(Depths(near=0.1, far=10.0))

    def test_a_description_range_beyond_the_wire_ceiling_is_refused_naming_the_camera(self):
        with pytest.raises(ModelEntryError, match="entry: camera 'cam': .*wire ceiling"):
            one_camera(Depths(near=0.1, far=100.0), depth={"width": 4, "height": 2})

    def test_a_camera_of_the_description_with_no_entry_is_rejected(self):
        with pytest.raises(
            ModelEntryError,
            match="the cameras of the description are \\['cam'\\], and the entry renders \\[\\]: "
            "give the entry one render per camera of the description",
        ):
            camera_configs("entry", [Frame()], {})

    def test_an_entry_of_no_camera_of_the_description_is_rejected(self):
        with pytest.raises(ModelEntryError, match="the entry renders \\['cam', 'other'\\]"):
            camera_configs("entry", [Frame()], parse_renders("entry", {"cam": render(), "other": render()}))

    def test_a_model_with_no_camera_lists_none(self):
        assert camera_configs("entry", [], {}) == ()

    def test_a_camera_of_the_description_with_no_file_says_to_create_it(self):
        with pytest.raises(
            ModelEntryError,
            match="the cameras of the description are \\['cam'\\], and no entry renders them: "
            "create models/aloha.json5 with one render per camera of the description",
        ):
            camera_configs("models/aloha.json5", [Frame()], None)

    def test_a_model_with_no_camera_needs_no_file(self):
        assert camera_configs("models/aloha.json5", [], None) == ()

    def test_cameras_that_are_not_a_map_are_rejected(self):
        with pytest.raises(ModelEntryError, match="'cameras' must map camera names"):
            parse_renders("entry", [])

    def test_unknown_camera_key_rejected(self):
        """A misspelled optional key fails: `dpeth` names no stream."""
        with pytest.raises(ModelEntryError, match="unknown key"):
            one_camera(dpeth={"width": 4, "height": 2})

    def test_unknown_color_key_rejected(self):
        with pytest.raises(ModelEntryError, match="unknown key"):
            one_camera(color={"width": 8, "height": 4, "fps": 15})

    def test_unknown_depth_key_rejected(self):
        """The depths a camera measures are its description's, not its render's."""
        with pytest.raises(ModelEntryError, match=r"unknown key\(s\) \['max_range_m', 'min_depth_m'\]"):
            one_camera(depth={"width": 4, "height": 2, "min_depth_m": 0.1, "max_range_m": 10})

    @pytest.mark.parametrize("fovy", [0, 180, -10, True])
    def test_fovy_out_of_range_rejected(self, fovy):
        with pytest.raises(ModelEntryError, match="fovy"):
            one_camera(fovy_deg=fovy)

    @pytest.mark.parametrize("fps", [0, -1, 1.5, 256, True])
    def test_bad_fps_rejected(self, fps):
        with pytest.raises(ModelEntryError, match="fps"):
            one_camera(fps=fps)

    def test_non_object_entry_rejected(self):
        with pytest.raises(ModelEntryError, match="object"):
            parse_renders("entry", {"cam": 7})

    def test_non_object_depth_rejected(self):
        with pytest.raises(ModelEntryError, match="depth must be an object"):
            one_camera(depth=7)

    def test_fps_upper_bound_accepted(self):
        assert one_camera(fps=255).fps == 255

    @pytest.mark.parametrize("dims", [(3, 2), (4, 1)])
    def test_depth_grid_must_subsample_color_grid(self, dims):
        width, height = dims
        with pytest.raises(ModelEntryError, match="subsample"):
            one_camera(depth={"width": width, "height": height})


class TestDepthSpec:
    def test_range_beyond_wire_ceiling_rejected(self):
        with pytest.raises(ValueError, match="wire ceiling"):
            DepthSpec(width=4, height=2, min_depth_m=0.1, max_range_m=65.536)

    def test_nonpositive_min_rejected(self):
        with pytest.raises(ValueError, match="wire ceiling"):
            DepthSpec(width=4, height=2, min_depth_m=0.0, max_range_m=1.0)

    def test_inverted_range_rejected(self):
        with pytest.raises(ValueError, match="wire ceiling"):
            DepthSpec(width=4, height=2, min_depth_m=2.0, max_range_m=1.0)


class TestDepthToZ16:
    _SPEC = DepthSpec(width=3, height=1, min_depth_m=0.4, max_range_m=65.535)

    def _decode(self, payload: bytes) -> list[int]:
        return list(struct.unpack(f"<{len(payload) // 2}H", payload))

    def test_in_range_converts_to_little_endian_mm(self):
        payload = depth_to_z16(np.array([[0.5, 1.0, 2.5]], dtype=np.float32), self._SPEC)
        assert self._decode(payload) == [500, 1000, 2500]

    def test_out_of_range_and_nonfinite_become_zero(self):
        depth = np.array([[0.1, np.nan, np.inf]], dtype=np.float32)
        assert self._decode(depth_to_z16(depth, self._SPEC)) == [0, 0, 0]

    def test_rounds_to_nearest_millimeter(self):
        depth = np.array([[1.2349, 1.2351, 0.4004]], dtype=np.float64)
        assert self._decode(depth_to_z16(depth, self._SPEC)) == [1235, 1235, 400]

    def test_exact_bounds_encode(self):
        depth = np.array([[0.4, 65.535, 65.0]], dtype=np.float64)
        assert self._decode(depth_to_z16(depth, self._SPEC)) == [400, 65535, 65000]

    def test_sub_millimeter_clips_to_one(self):
        spec = DepthSpec(width=1, height=1, min_depth_m=0.0002, max_range_m=1.0)
        depth = np.array([[0.0004]], dtype=np.float64)
        assert self._decode(depth_to_z16(depth, spec)) == [1]

    def test_shape_mismatch_rejected(self):
        with pytest.raises(ValueError, match="shape"):
            depth_to_z16(np.zeros((2, 3), dtype=np.float32), self._SPEC)


class TestFrameIdCounter:
    def test_wraps_at_u32(self):
        counter = FrameIdCounter()
        counter._next = 0xFFFFFFFF
        assert counter.next() == 0xFFFFFFFF
        assert counter.next() == 0


class TestFramePacer:
    def test_remaining_wait_does_not_initialize_the_schedule(self):
        pacer = FramePacer(10)
        assert pacer.seconds_until_due(0.0) == 0.0
        assert pacer.seconds_until_due(100.0) == 0.0
        assert pacer.take_if_due(100.0)
        assert pacer.seconds_until_due(100.0) == pytest.approx(0.1)

    def test_remaining_wait_does_not_claim_or_advance_a_deadline(self):
        pacer = FramePacer(10)
        assert pacer.take_if_due(100.0)
        for _ in range(2):
            assert pacer.seconds_until_due(100.04) == pytest.approx(0.06)
            assert pacer.seconds_until_due(100.1) == 0.0
            assert pacer.seconds_until_due(105.0) == 0.0
        assert pacer.take_if_due(100.1)
        assert pacer.seconds_until_due(100.1) == pytest.approx(0.1)

    def test_first_call_is_due(self):
        pacer = FramePacer(10)
        assert pacer.take_if_due(100.0)
        assert not pacer.take_if_due(100.05)
        assert pacer.take_if_due(100.1)

    def test_stall_resyncs_instead_of_bursting(self):
        pacer = FramePacer(10)
        assert pacer.take_if_due(100.0)
        assert pacer.take_if_due(105.0)
        # After the resync the next frame is one period out, not a burst of
        # 50 missed frames.
        assert not pacer.take_if_due(105.05)
        assert pacer.take_if_due(105.1)

    def test_ticks_jittered_around_the_period_are_all_taken(self):
        # A 60 Hz request over a 60 fps loop whose frames alternate 16.2 and
        # 17.2 ms: one early tick skips, and the grid must absorb the late
        # one that follows rather than resync onto it, or the pair locks
        # into taking every other tick.
        pacer = FramePacer(60)
        ticks = [100.0 + (i // 2) * 0.0334 + (i % 2) * 0.0162 for i in range(600)]
        taken = [pacer.take_if_due(t) for t in ticks]
        assert sum(taken) >= 598

    def test_ticks_slightly_faster_than_the_period_skip_at_most_one_in_a_row(self):
        pacer = FramePacer(60)
        taken = [pacer.take_if_due(100.0 + i * 0.0165) for i in range(600)]
        assert sum(taken) >= 594
        assert all(a or b for a, b in pairwise(taken))
