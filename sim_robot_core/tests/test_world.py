"""Tests for world, the one frame of a simulation: a command in another frame is refused with
unknown_frame and a value no contract takes with rejected, a robot base's target turns about +z
alone, an attach places its robot at a pose in world or leaves the spot to the engine, and a
robot that rolls stands where its wheels carried it."""

import math
from dataclasses import dataclass

import pytest
from control_core_py.geometry import (
    WORLD,
    GeometryError,
    Point,
    Pose,
    PoseTarget,
    UnitQuaternion,
    product,
    quaternion_of_yaw,
)

from sim_robot_core.world import (
    UnknownFrame,
    attach_placement,
    base_target,
    failure_kind_of,
    in_world,
    rolled_placement,
)

# A turn of 90 degrees about +z, as a caller types it: its squared norm lies within the
# shared parse's tolerance of 1, and the parse normalizes it.
QUARTER_TURN = (0.0, 0.0, 0.7071, 0.7071)


@dataclass(frozen=True)
class Placement:
    """An attach goal's placement record, as a node's generated code holds it."""

    frame: str | None
    timestamp: float | None
    position: list[float]
    orientation: list[float] | None


class TestInWorld:
    @pytest.mark.parametrize("timestamp", [None, 0.0, 12.5])
    def test_a_value_in_world_is_taken_at_any_time(self, timestamp):
        assert in_world(WORLD, timestamp, "value") == "value"

    @pytest.mark.parametrize("timestamp", [None, 12.5])
    def test_a_value_with_no_frame_is_in_world(self, timestamp):
        assert in_world(None, timestamp, "value") == "value"

    @pytest.mark.parametrize("frame", ["odom", "base_footprint", "map", "", "World"])
    def test_another_frame_is_refused_naming_it_and_saying_to_send_world(self, frame):
        with pytest.raises(UnknownFrame) as refused:
            in_world(frame, 0.0, "value")
        assert str(refused.value) == (
            f"the frame {frame!r} is not world, the one frame of the simulation: send world"
        )

    @pytest.mark.parametrize("timestamp", [-1.0, math.nan, math.inf])
    def test_a_timestamp_no_stamp_holds_is_rejected(self, timestamp):
        with pytest.raises(GeometryError, match="lies outside the times a stamp holds"):
            in_world(WORLD, timestamp, "value")

    def test_the_frame_is_checked_before_the_timestamp(self):
        with pytest.raises(UnknownFrame):
            in_world("odom", -1.0, "value")


class TestBaseTarget:
    def test_a_position_and_a_turn_about_z_are_taken_normalized(self):
        target = base_target(WORLD, 0.0, [1.0, -2.0, 0.25], QUARTER_TURN)
        assert target.position == Point((1.0, -2.0, 0.25))
        assert target.orientation == UnitQuaternion.parse(QUARTER_TURN)
        assert math.isclose(sum(c * c for c in target.orientation.xyzw), 1.0)

    def test_a_target_with_no_orientation_has_none(self):
        assert base_target(None, None, [1.0, 2.0, 0.0], None) == PoseTarget(
            Point((1.0, 2.0, 0.0)), None
        )

    def test_a_target_that_needs_an_orientation_refuses_one_with_none(self):
        with pytest.raises(GeometryError, match="gives no orientation"):
            base_target(WORLD, 0.0, [1.0, 2.0, 0.0], None).pose()

    def test_an_empty_orientation_is_refused_saying_what_to_send(self):
        with pytest.raises(GeometryError, match=r"has 0 entries: send 4"):
            base_target(WORLD, 0.0, [1.0, 2.0, 0.0], [])

    @pytest.mark.parametrize(
        "orientation",
        [
            (math.sin(0.05), 0.0, 0.0, math.cos(0.05)),
            (0.0, math.sin(0.05), 0.0, math.cos(0.05)),
            (1.0, 0.0, 0.0, 0.0),
        ],
    )
    def test_an_orientation_that_tilts_is_rejected_saying_what_to_send(
        self, orientation
    ):
        with pytest.raises(GeometryError, match=r"send a turn about \+z alone"):
            base_target(WORLD, 0.0, [0.0, 0.0, 0.0], orientation)

    def test_an_orientation_far_from_unit_is_rejected(self):
        with pytest.raises(GeometryError, match="squared norm"):
            base_target(WORLD, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 2.0))

    def test_a_target_in_another_frame_is_refused_with_unknown_frame(self):
        with pytest.raises(UnknownFrame, match="'odom'"):
            base_target("odom", 0.0, [0.0, 0.0, 0.0], QUARTER_TURN)


class TestFailureKindOf:
    def test_a_frame_other_than_world_is_unknown_frame(self):
        with pytest.raises(UnknownFrame) as refused:
            in_world("odom", None, "value")
        assert failure_kind_of(refused.value) == "unknown_frame"

    def test_a_value_no_contract_takes_is_rejected(self):
        with pytest.raises(GeometryError) as refused:
            base_target(WORLD, None, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 2.0))
        assert failure_kind_of(refused.value) == "rejected"

    def test_any_other_refusal_of_a_parse_is_rejected(self):
        assert failure_kind_of(ValueError("a name the engine cannot stand")) == "rejected"


class TestAttachPlacement:
    def test_a_goal_without_a_placement_leaves_the_spot_to_the_engine(self):
        assert attach_placement(None) is None

    @pytest.mark.parametrize("frame", [WORLD, None])
    def test_a_placement_in_world_is_a_pose_turned_about_z_and_normalized(self, frame):
        placed = attach_placement(Placement(frame, 3.0, [1.0, -2.0, 0.0], list(QUARTER_TURN)))
        assert placed == Pose(Point((1.0, -2.0, 0.0)), UnitQuaternion.parse(QUARTER_TURN))

    def test_a_placement_without_an_orientation_is_rejected(self):
        with pytest.raises(GeometryError, match="gives no orientation"):
            attach_placement(Placement(WORLD, None, [1.0, 2.0, 0.0], None))

    def test_a_placement_that_tilts_is_rejected(self):
        with pytest.raises(GeometryError, match=r"send a turn about \+z alone"):
            attach_placement(Placement(WORLD, None, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]))

    def test_a_placement_in_another_frame_is_refused_with_unknown_frame(self):
        with pytest.raises(UnknownFrame, match="'map'"):
            attach_placement(Placement("map", None, [0.0, 0.0, 0.0], list(QUARTER_TURN)))


def test_a_rolled_placement_faces_the_heading_its_wheels_turned_it_to():
    # A base turned 0.6 rad and pitched 0.05 rad by a step of the floor, half a centimetre up.
    pitched = (0.0, math.sin(0.025), 0.0, math.cos(0.025))
    tilted = product(quaternion_of_yaw(0.6), pitched)
    placement = rolled_placement((1.25, -0.5, 0.005), tilted)
    assert placement.position.position == (1.25, -0.5, 0.005)
    assert placement.orientation.yaw() == pytest.approx(0.6)
    assert placement.orientation.xyzw[:2] == (0.0, 0.0), "it turns about +z alone"
