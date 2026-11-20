"""Tests of control_core_py.geometry: the parse and the arithmetic, held to frame_tree's."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from control_core_py import geometry as g

# The refusals control_core::geometry gives in the same words: a call, its arguments and its
# message on each line, tab-separated.
REFUSALS = Path(__file__).resolve().parents[2] / "control_core" / "tests" / "fixtures" / "geometry_refusals.tsv"


def _numbers(field: str) -> list[float]:
    return [float(number) for number in field.split()]


def _optional_numbers(field: str) -> list[float] | None:
    """The numbers of a field, or None for an empty field."""
    return _numbers(field) if field else None


def _optional_seconds(field: str) -> float | None:
    """The seconds of a field, or None for an empty field."""
    return float(field) if field else None


# Each call of the table on its arguments, as control_core's suite makes it.
CALLS = {
    "lookup_time": lambda seconds: g.lookup_time(_optional_seconds(seconds)),
    "Point.parse": lambda position: g.Point.parse(_numbers(position)),
    "UnitQuaternion.parse": lambda orientation: g.UnitQuaternion.parse(_numbers(orientation)),
    "PoseTarget.parse": lambda position, orientation: g.PoseTarget.parse(
        _numbers(position), _optional_numbers(orientation)
    ),
    "PoseTarget.pose": lambda position, orientation: g.PoseTarget.parse(
        _numbers(position), _optional_numbers(orientation)
    ).pose(),
    "Stamped.parse": lambda frame, seconds: g.Stamped.parse(frame, _optional_seconds(seconds), None),
}


def _shared_refusals() -> list[list[str]]:
    lines = REFUSALS.read_text().splitlines()
    return [line.split("\t") for line in lines if line and not line.startswith("#")]


@pytest.mark.parametrize("fields", _shared_refusals(), ids=lambda fields: " ".join(fields[:-1]))
def test_each_refusal_of_the_shared_table_gives_its_message(fields: list[str]):
    call, *arguments, message = fields
    with pytest.raises(g.GeometryError) as refused:
        CALLS[call](*arguments)
    assert str(refused.value) == message


def about_z(yaw: float) -> g.Quaternion:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def close(a, b, tolerance: float = 1e-12) -> bool:
    return all(abs(x - y) < tolerance for x, y in zip(a, b))


def test_a_quarter_turn_about_z_takes_x_to_y_on_both_paths():
    q = about_z(math.pi / 2.0)
    assert close(g.rotate(q, (1.0, 0.0, 2.0)), (0.0, 1.0, 2.0))
    assert close(g._rotated(g.rotation_of(q), (1.0, 0.0, 2.0)), (0.0, 1.0, 2.0))
    assert close(g.product(about_z(0.3), about_z(0.4)), about_z(0.7))
    assert close(g.product(about_z(0.3), g.inverse(about_z(0.3))), g.IDENTITY)


def test_a_rotation_gives_back_its_quaternion():
    norm = math.sqrt(0.1**2 + 0.2**2 + 0.3**2 + 0.9**2)
    q = tuple(c / norm for c in (0.1, 0.2, 0.3, 0.9))
    assert close(g.quaternion_of(g.rotation_of(q)), q)
    # Past 120 degrees the trace is below 0 and the largest diagonal entry gives the quaternion.
    turned = (math.sin(1.2), 0.0, 0.0, math.cos(1.2))
    assert close(g.quaternion_of(g.rotation_of(turned)), turned)


def test_the_shortest_path_takes_the_nearer_of_q2_and_its_negation():
    assert abs(g.angle_shortest_path(about_z(0.2), about_z(1.0)) - 0.8) < 1e-12
    far = tuple(-c for c in about_z(1.0))
    assert abs(g.angle_shortest_path(about_z(0.2), far) - 0.8) < 1e-12
    assert g.angle_shortest_path(about_z(0.7), about_z(0.7)) == 0.0


def test_slerp_turns_along_the_shorter_arc_and_keeps_q1_for_one_rotation():
    assert close(g.slerp(about_z(0.2), about_z(1.0), 0.25), about_z(0.4))
    far = tuple(-c for c in about_z(1.0))
    assert close(g.slerp(about_z(0.2), far, 0.25), about_z(0.4))
    q = about_z(0.7)
    assert g.slerp(q, q, 0.5) == q


@pytest.mark.parametrize("yaw", [i * 0.25 for i in range(-12, 13)])
def test_a_heading_reads_back_from_its_quaternion(yaw: float):
    assert abs(g.yaw_of_quaternion(g.quaternion_of_yaw(yaw)) - yaw) < 1e-12


def test_the_two_orders_convert_one_into_the_other():
    assert g.xyzw_of_wxyz((4.0, 1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0, 4.0)
    assert g.wxyz_of_xyzw((1.0, 2.0, 3.0, 4.0)) == (4.0, 1.0, 2.0, 3.0)


def test_an_orientation_parses_within_tf2s_tolerance_and_comes_out_normalized():
    typed = g.UnitQuaternion.parse([0.0, 0.0, 0.7071, 0.7071])
    assert abs(sum(c * c for c in typed.quaternion) - 1.0) < 1e-15
    for refused in ([0.0, 0.0, 0.0, 1.006], [0.0, 0.0, 0.0, 0.0], [math.nan, 0.0, 0.0, 1.0]):
        with pytest.raises(g.GeometryError):
            g.UnitQuaternion.parse(refused)


def test_a_target_takes_an_orientation_or_none():
    assert g.PoseTarget.parse([1.0, 2.0, 0.0], None).orientation is None
    with pytest.raises(g.GeometryError, match="gives no orientation"):
        g.PoseTarget.parse([1.0, 2.0, 0.0], None).pose()
    assert g.PoseTarget.parse([1.0, 2.0, 0.0], [0, 0, 0, 1]).orientation == g.UnitQuaternion(g.IDENTITY)
    # A given orientation has four entries, and one of another count, an empty one among them,
    # is refused.
    for entries in ([], [0.0, 0.0, 1.0]):
        with pytest.raises(g.GeometryError) as refused:
            g.PoseTarget.parse([1.0, 2.0, 0.0], entries)
        assert str(refused.value) == (
            f"the orientation has {len(entries)} entries: send 4, a unit quaternion [x, y, z, w]"
        )
    with pytest.raises(g.GeometryError, match="must be finite"):
        g.PoseTarget.parse([math.inf, 0.0, 0.0], None)


def test_no_timestamp_is_the_latest_time_and_the_epoch_is_a_time():
    assert g.Stamped.parse("odom", None, None).timestamp is None
    assert g.Stamped.parse("odom", 0.0, None).timestamp == 0.0
    assert g.Stamped.parse("base_footprint", 12.0, None).timestamp == 12.0
    with pytest.raises(g.GeometryError, match="frame is empty"):
        g.Stamped.parse("", 12.0, None)
    for refused in (math.nan, math.inf, -1e-9):
        with pytest.raises(g.GeometryError, match="lies outside the times a stamp holds"):
            g.Stamped.parse("odom", refused, None)
    # The last time a stamp holds is 18446744073.709551615 s: the double below it is a time, the
    # double above it, 18446744073.709553, is refused in the shared table.
    assert g.Stamped.parse("odom", math.nextafter(18446744073.709553, 0.0), None).timestamp > 0.0


def test_a_timestamp_alone_is_looked_up_as_a_stamped_value_looks_it_up():
    assert g.lookup_time(None) is None
    assert g.lookup_time(0.0) == 0.0
    assert g.lookup_time(12) == 12.0 and isinstance(g.lookup_time(12), float)
    for refused in (math.nan, math.inf, -1e-9):
        with pytest.raises(g.GeometryError, match="lies outside the times a stamp holds"):
            g.lookup_time(refused)


def test_a_value_of_another_number_of_entries_is_refused_with_what_to_send():
    cases = [
        (lambda: g.Point.parse([1.0, 2.0]), "the position has 2 entries: send 3, [x, y, z] in m"),
        (
            lambda: g.UnitQuaternion.parse([0.0, 0.0, 1.0]),
            "the orientation has 3 entries: send 4, a unit quaternion [x, y, z, w]",
        ),
    ]
    for call, message in cases:
        with pytest.raises(g.GeometryError) as refused:
            call()
        assert str(refused.value) == message


def test_a_planar_pose_is_refused_off_the_plane_saying_what_to_send():
    planar = g.Pose.parse([1.0, -2.0, 0.0], about_z(2.5)).planar()
    assert (planar.x, planar.y) == (1.0, -2.0) and abs(planar.yaw - 2.5) < 1e-12
    cases = [
        (
            ([1.0, 0.0, 1e-7], g.IDENTITY),
            "the position lies 0.0000001 m above the floor plane: send a position [x, y, 0] on "
            "the floor",
        ),
        (
            ([1.0, 0.0, 0.0], (1.0, 0.0, 0.0, 0.0)),
            "the orientation [1, 0, 0, 0] tilts 3.141592653589793 rad off the vertical: send a "
            "turn about +z alone, [0, 0, sin(yaw/2), cos(yaw/2)]",
        ),
    ]
    for (position, orientation), message in cases:
        with pytest.raises(g.GeometryError) as refused:
            g.Pose.parse(position, orientation).planar()
        assert str(refused.value) == message
    with pytest.raises(g.GeometryError, match="tilts"):
        g.Pose.parse([1.0, 0.0, 0.0], [math.sin(0.05), 0.0, 0.0, math.cos(0.05)]).planar()


def test_a_value_placed_past_the_finite_numbers_is_refused_as_its_parse_refuses_it():
    eighth = math.pi / 8.0
    placement = g.Placement((1.7976931348623157e308, 0.0, 0.0), (0.0, 0.0, math.sin(eighth), math.cos(eighth)))
    with pytest.raises(g.GeometryError) as refused:
        placement.place(g.Point.parse([1.7976931348623157e308, 0.0, 0.0]))
    assert str(refused.value).startswith("a position must be finite: got x inf")
    assert placement.place(g.Point.parse([0.0, 1.0, 0.0])).position[0] == 1.7976931348623157e308


def test_values_of_one_frame_and_one_time_are_placed_together_and_refused_whole():
    quarter = about_z(math.pi / 2.0)
    placement = g.Placement((1.0, 0.0, 0.0), quarter)
    near, ahead = g.Point.parse([0.0, 0.0, 0.0]), g.Point.parse([1.0, 0.0, 0.0])
    placed = placement.place((near, ahead))
    assert close(placed[0].position, (1.0, 0.0, 0.0)) and close(placed[1].position, (1.0, 1.0, 0.0))
    target = placement.place(g.PoseTarget.parse([1.0, 0.0, 0.0], None))
    assert close(target.position.position, (1.0, 1.0, 0.0)) and target.orientation is None
    far = g.Placement((1.7976931348623157e308, 0.0, 0.0), g.IDENTITY)
    with pytest.raises(g.GeometryError, match="got x inf"):
        far.place((near, g.Point.parse([1.7976931348623157e308, 0.0, 0.0])))


def test_a_placement_carries_a_pose():
    quarter = about_z(math.pi / 2.0)
    placement = g.Placement((1.0, 0.0, 0.0), quarter)
    pose = placement.place(g.Pose.parse([1.0, 0.0, 0.0], quarter))
    assert close(pose.position.position, (1.0, 1.0, 0.0))
    assert close(pose.orientation.quaternion, (0.0, 0.0, 1.0, 0.0))
