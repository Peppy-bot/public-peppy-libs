import pytest

from workspace_core_py import (
    Positions,
    Reach,
    SurfaceHeight,
    SurfaceReach,
    check_without_perception_camera,
    describe_without_perception_camera,
)

UNCHECKED = "The view is not checked: the robot has no perception camera."


def surface(metres, reaches):
    """The reach of the surface at `metres` where the arm reaches the target
    above each grid point `(x, y)` that `reaches` holds, and stops 0.2 m short
    of every other one."""
    height = SurfaceHeight.from_wire(metres)
    return SurfaceReach.from_grid_order(
        height,
        [
            Reach.reached_by("arm") if reaches(x, y) else Reach.short(0.2)
            for x, y, _ in height.grid_targets()
        ],
    )


def test_a_surface_the_arm_reaches_is_workable_and_says_where_to_put_items():
    reach = surface(0.45, lambda x, y: 0.2 <= x <= 0.4 and abs(y) <= 0.1)
    answer = describe_without_perception_camera(reach)
    assert answer.workable
    assert answer.area == pytest.approx(0.055)
    assert answer.rectangle == pytest.approx((0.2, 0.4, -0.1, 0.1))
    assert answer.reach == pytest.approx((0.2, 0.4, -0.1, 0.1))
    assert answer.view is None
    assert answer.message == (
        "Workable: 0.055 m². Put items between x 0.20 and 0.40 m and between "
        f"y -0.10 and 0.10 m in the robot frame. {UNCHECKED}"
    )


def test_a_surface_no_arm_reaches_is_not_reachable_and_gives_no_placement():
    answer = describe_without_perception_camera(surface(0.45, lambda x, y: False))
    assert not answer.workable
    assert answer.area == 0.0
    assert (answer.rectangle, answer.reach, answer.view) == (None, None, None)
    assert answer.message == (
        "Not reachable: no arm reaches any point of it with its gripper pointing "
        f"down or forward. {UNCHECKED}"
    )


def test_a_surface_with_too_little_room_gives_its_rectangle_but_no_placement():
    reach = surface(0.45, lambda x, y: x == 0.3 and y in (0.0, 0.05))
    answer = describe_without_perception_camera(reach)
    assert not answer.workable
    assert answer.area == pytest.approx(0.002)
    assert answer.rectangle == pytest.approx((0.3, 0.3, 0.0, 0.05))
    assert answer.message == (
        "Too little room: 0.002 m² is workable, under the 0.030 m² that holds a "
        f"few objects. {UNCHECKED}"
    )


def test_each_checked_point_carries_its_reach_and_message_in_the_requests_order():
    positions = Positions.from_wire([0.3, 0.0, 0.5, 0.9, 0.0, 0.5, 0.4, 0.1, 0.3])
    reaches = [Reach.reached_by("arm"), Reach.short(0.6), Reach.short(0.004)]
    answer = check_without_perception_camera(positions, reaches)
    assert [point.position for point in answer.points] == positions.points
    assert [point.reach for point in answer.points] == reaches
    assert [point.workable for point in answer.points] == [True, False, False]
    assert all(point.view == "no_camera" for point in answer.points)
    assert not any(point.in_view for point in answer.points)
    assert [point.message for point in answer.points] == [
        "Workable: arm reaches it; its view is not checked.",
        "Not workable: it is out of reach by 0.60 m; its view is not checked.",
        (
            "Not workable: no arm reaches it with its gripper pointing down or "
            "forward; its view is not checked."
        ),
    ]
    assert not answer.all_workable
    assert answer.message == f"1 of the 3 points is workable. {UNCHECKED}"


@pytest.mark.parametrize(
    ("reach", "workable", "message"),
    [
        (Reach.reached_by("arm"), True, "The point is workable."),
        (Reach.short(0.25), False, "The point is not workable."),
    ],
)
def test_the_answer_of_one_point_says_whether_it_is_workable_then_why_no_view_is_checked(
    reach, workable, message
):
    answer = check_without_perception_camera(
        Positions.from_wire([0.3, 0.0, 0.5]), [reach]
    )
    assert answer.all_workable is workable
    assert answer.message == f"{message} {UNCHECKED}"


def test_a_check_with_another_count_of_reaches_is_refused():
    positions = Positions.from_wire([0.3, 0.0, 0.5, 0.9, 0.0, 0.5])
    with pytest.raises(
        ValueError,
        match="^1 reaches are given for 2 targets: give one reach per target, in order$",
    ):
        check_without_perception_camera(positions, [Reach.reached_by("arm")])
    with pytest.raises(TypeError):
        check_without_perception_camera(positions, [Reach.short(0.1), None])  # type: ignore[list-item]
