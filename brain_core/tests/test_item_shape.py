"""The shape of an item from the chest camera's depth readings, on
synthetic depth images (synthetic_depth.py): which readings are the
item's (`item_points`) and the box around them (`item_box`), for each case
of the plan: a pen at 0, 45 and 90 degrees, a cube, a ball, an item beside
a neighbour, an item in a bowl, a box with no depth."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import synthetic_depth as sd
from conftest import FakeDetector, rgb_frame

from brain_core.limbs import parse_limbs
from brain_core.perception.frames import FrameStore, decode_depth
from brain_core.perception.perceiver import MIN_SIDE_M, SURFACE_MARGIN_M, ItemPoints, Look, Perceiver, item_box, item_points
from brain_core.ports import Box, CancelToken, Coverage, Detection, ItemShape
from brain_core.state import State

LIME = sd.Sphere((0.30, -0.20, 0.413), 0.043)
CUBE = sd.UprightBox((0.32, 0.05, 0.395), (0.05, 0.05, 0.05), 30.0)
BALL = sd.Sphere((0.30, 0.10, 0.405), 0.035)
BOWL = sd.Bowl((0.27, 0.07), 0.085, 0.455)
APPLE_IN_THE_BOWL = sd.Sphere((0.27, 0.07, 0.409), 0.039)
# How far a point may stand from the surface of the solid it lies on.
ON_M = 0.003


def points_of(scene: sd.Scene, index: int, loose_px: int = 2):
    return item_points(scene.depth_m, scene.box(index, loose_px=loose_px), sd.WIDTH, sd.HEIGHT, sd.CHEST)


def on_sphere(points: np.ndarray, sphere: sd.Sphere) -> np.ndarray:
    return np.abs(np.linalg.norm(points - np.array(sphere.centre), axis=1) - sphere.radius) <= ON_M


def inside(points: np.ndarray, solid: sd.Solid) -> np.ndarray:
    low, high = solid.bounds()
    return np.all((points >= low - ON_M) & (points <= high + ON_M), axis=1)


def test_the_points_of_an_item_on_the_table_are_its_own_and_none_of_the_table():
    found = points_of(sd.render([LIME]), 0)
    assert found.surface_z == pytest.approx(sd.TABLE_Z, abs=1e-6)
    assert len(found.points) > 1000 and on_sphere(found.points, LIME).all()
    assert (found.points[:, 2] > sd.TABLE_Z + SURFACE_MARGIN_M).all()


@pytest.mark.parametrize("solid", [CUBE, BALL], ids=["cube", "ball"])
def test_a_cube_and_a_ball_keep_their_own_points(solid):
    found = points_of(sd.render([solid]), 0)
    assert len(found.points) > 500 and inside(found.points, solid).all()


@pytest.mark.parametrize("yaw_deg", [0.0, 45.0, 90.0])
def test_a_pen_lying_at_any_turn_keeps_the_points_of_its_upper_half(yaw_deg):
    """A pen 1 cm thick: only its points more than the surface margin above
    the table are kept, and each lies on the pen."""
    pen = sd.UprightBox((0.30, 0.0, 0.375), (0.14, 0.01, 0.01), yaw_deg)
    found = points_of(sd.render([pen]), 0)
    assert len(found.points) > 150 and inside(found.points, pen).all()


def test_a_neighbour_inside_the_box_that_the_table_parts_from_the_item_is_left_out():
    neighbour = sd.UprightBox((0.30, -0.117, 0.39), (0.04, 0.04, 0.04), 0.0)
    scene = sd.render([LIME, neighbour])
    box = scene.box(0, loose_px=12)
    # The loose box holds part of the neighbour.
    assert scene.masks[1][int(box.y0):int(box.y1), int(box.x0):int(box.x1)].any()
    found = points_of(scene, 0, loose_px=12)
    assert on_sphere(found.points, LIME).all() and not inside(found.points, neighbour).any()


def test_a_tall_neighbour_behind_the_item_is_left_out():
    behind = sd.UprightBox((0.40, -0.20, 0.45), (0.04, 0.06, 0.16), 0.0)
    found = points_of(sd.render([LIME, behind]), 0, loose_px=6)
    assert on_sphere(found.points, LIME).all()


def test_a_neighbour_that_touches_the_item_joins_it():
    """Two items that touch are one group: the rule cannot part them, and
    the box around both is the larger shape."""
    touching = sd.UprightBox((0.30, -0.137, 0.39), (0.04, 0.04, 0.04), 0.0)
    found = points_of(sd.render([LIME, touching]), 0, loose_px=12)
    assert inside(found.points, touching).any() and on_sphere(found.points, LIME).any()


def test_an_item_in_a_bowl_keeps_its_own_points_above_the_bowl_wall_around_it():
    """The border of the apple's box is the inside of the bowl, so the
    surface is the height of that wall, not the bottom: the apple keeps
    fewer points, and none of the bowl."""
    found = points_of(sd.render([BOWL, APPLE_IN_THE_BOWL]), 1)
    assert sd.TABLE_Z + 0.01 < found.surface_z < APPLE_IN_THE_BOWL.centre[2]
    assert len(found.points) > 500 and on_sphere(found.points, APPLE_IN_THE_BOWL).all()


def test_noisy_readings_keep_mostly_the_items_points():
    """A depth error of 2 mm, the same on every run: a few table readings
    rise above the margin next to the item, and join it."""
    pen = sd.UprightBox((0.30, 0.0, 0.375), (0.14, 0.01, 0.01), 45.0)
    found = points_of(sd.render([pen], noise_m=0.002), 0)
    assert inside(found.points, pen).mean() >= 0.95
    found = points_of(sd.render([LIME], noise_m=0.002), 0)
    assert np.mean(np.abs(np.linalg.norm(found.points - np.array(LIME.centre), axis=1) - LIME.radius) <= 0.008) >= 0.99


def test_a_box_with_no_depth_or_no_reading_on_its_border_has_no_points():
    scene = sd.render([LIME])
    box = scene.box(0)
    assert item_points(np.zeros_like(scene.depth_m), box, sd.WIDTH, sd.HEIGHT, sd.CHEST) is None
    border_blank = scene.depth_m.copy()
    x0, y0, x1, y1 = (int(value) for value in (box.x0, box.y0, box.x1, box.y1))
    border_blank[y0:y1, x0] = border_blank[y0:y1, x1 - 1] = 0.0
    border_blank[y0, x0:x1] = border_blank[y1 - 1, x0:x1] = 0.0
    assert item_points(border_blank, box, sd.WIDTH, sd.HEIGHT, sd.CHEST) is None


def test_a_box_of_the_bare_table_or_too_small_to_have_a_border_has_no_points():
    scene = sd.render([])
    assert item_points(scene.depth_m, Box("table", 0.9, 150, 100, 170, 120), sd.WIDTH, sd.HEIGHT, sd.CHEST) is None
    assert item_points(scene.depth_m, Box("speck", 0.9, 150, 100, 152, 102), sd.WIDTH, sd.HEIGHT, sd.CHEST) is None


def test_a_box_of_a_colour_picture_larger_than_the_depth_image_reads_the_same_points():
    """A box of the 1280 x 720 colour picture over the 320 x 180 depth
    image: the readings under it are the same."""
    scene = sd.render([LIME])
    box = scene.box(0)
    found = points_of(scene, 0)
    scaled = Box("lime", 0.9, 4 * box.x0, 4 * box.y0, 4 * box.x1, 4 * box.y1)
    camera = sd.CHEST.with_intrinsics(sd.CHEST.intrinsics.scaled_to(4 * sd.WIDTH, 4 * sd.HEIGHT))
    big = item_points(scene.depth_m, scaled, 4 * sd.WIDTH, 4 * sd.HEIGHT, camera)
    assert big.points.shape == found.points.shape and np.allclose(big.points, found.points, atol=1e-6)


# The box around the points: upright, turned to the smallest rectangle from
# above, from the surface to the highest point.


def box_of(solids, index=0, **render):
    scene = sd.render(solids, **render)
    found = points_of(scene, index)
    return found, item_box(found)


def long_axis_deg(shape: ItemShape) -> float:
    """The direction of the box's longer side from above, 0 to 180 degrees."""
    yaw = math.degrees(shape.yaw_rad)
    return (yaw if shape.sides[0] >= shape.sides[1] else yaw + 90.0) % 180.0


def holds(shape: ItemShape, points: np.ndarray) -> bool:
    """Whether every point lies inside the box, to a micrometre."""
    c, s = math.cos(shape.yaw_rad), math.sin(shape.yaw_rad)
    offset = points - np.array(shape.centre)
    local = np.stack([c * offset[:, 0] + s * offset[:, 1], -s * offset[:, 0] + c * offset[:, 1], offset[:, 2]], axis=1)
    return bool(np.all(np.abs(local) <= np.array(shape.sides) / 2.0 + 1e-6))


@pytest.mark.parametrize("yaw_deg", [0.0, 45.0, 90.0])
def test_the_box_of_a_pen_lies_along_it_at_any_turn(yaw_deg):
    pen = sd.UprightBox((0.30, 0.0, 0.375), (0.14, 0.01, 0.01), yaw_deg)
    found, shape = box_of([pen])
    assert max(shape.sides[:2]) == pytest.approx(0.14, abs=0.005)
    assert min(shape.sides[:2]) == pytest.approx(0.01, abs=0.004)
    assert shape.sides[2] == pytest.approx(0.01, abs=1e-6)
    assert abs((long_axis_deg(shape) - yaw_deg + 90.0) % 180.0 - 90.0) <= 1.0
    assert shape.centre == pytest.approx((0.30, 0.0, 0.375), abs=0.004)
    assert holds(shape, found.points)


def test_the_box_of_a_turned_cube_is_the_cube():
    found, shape = box_of([CUBE])
    assert shape.sides == pytest.approx((0.05, 0.05, 0.05), abs=0.002)
    assert math.degrees(shape.yaw_rad) % 90.0 == pytest.approx(30.0, abs=1.0)
    assert shape.centre == pytest.approx(CUBE.centre, abs=0.002)
    assert holds(shape, found.points)


@pytest.mark.parametrize("sphere", [LIME, BALL], ids=["lime", "ball"])
def test_the_box_of_a_round_item_stands_on_the_table_and_reaches_its_top_and_no_further_than_its_side(sphere):
    """The camera sees the near half of a round item: its box reaches the
    top and the sides, and is shorter than the item towards the camera."""
    found, shape = box_of([sphere])
    top = sphere.centre[2] + sphere.radius
    assert shape.centre[2] - shape.sides[2] / 2 == pytest.approx(sd.TABLE_Z, abs=1e-6)
    assert shape.centre[2] + shape.sides[2] / 2 == pytest.approx(top, abs=1e-3)
    assert max(shape.sides[:2]) <= 2 * sphere.radius + 1e-3
    assert holds(shape, found.points)


def test_the_box_of_an_item_in_a_bowl_starts_at_the_bowl_wall_around_it():
    found, shape = box_of([BOWL, APPLE_IN_THE_BOWL], index=1)
    assert shape.centre[2] - shape.sides[2] / 2 == pytest.approx(found.surface_z)
    assert shape.centre[2] + shape.sides[2] / 2 == pytest.approx(APPLE_IN_THE_BOWL.centre[2] + APPLE_IN_THE_BOWL.radius, abs=1e-3)


def test_the_box_of_a_single_point_has_the_least_side():
    shape = item_box(ItemPoints(np.array([[0.3, 0.1, 0.4]]), 0.37))
    assert shape.sides == pytest.approx((MIN_SIDE_M, MIN_SIDE_M, 0.03))
    assert shape.centre == pytest.approx((0.3, 0.1, 0.385)) and shape.yaw_rad == 0.0


# The shape on the detections of a scan, on the items, and in the log.


def z16(depth_m: np.ndarray, frame_id: int = 3):
    """The depth_stream message of `depth_m`, in millimetres."""
    return SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame_id=frame_id, align_mode="depth_to_color"),
        encoding="z16", width=depth_m.shape[1], height=depth_m.shape[0],
        frame=np.round(depth_m / 0.001).astype("<u2").tobytes(),
    )


async def test_a_scan_gives_each_detection_the_box_around_its_depth_points():
    scene = sd.render([LIME])
    frames = FrameStore()
    frames.add_color(rgb_frame(sd.WIDTH, sd.HEIGHT, frame_id=3))
    frames.add_depth(z16(scene.depth_m))
    frames.depth_unit = 0.001
    perceiver = Perceiver(frames, sd.CHEST)
    box = scene.box(0, label="lime")
    look = await perceiver.scan(FakeDetector([box]), ["lime"], CancelToken(), timeout_s=0.0)
    [lime] = look.detections
    read = decode_depth(z16(scene.depth_m), 0.001)
    assert lime.shape == item_box(item_points(read, box, sd.WIDTH, sd.HEIGHT, sd.CHEST))
    assert lime.shape.centre[2] + lime.shape.sides[2] / 2 == pytest.approx(0.456, abs=0.002)
    assert "lime 0.90 (box " in look.describe(1_000_000_000)


def test_the_log_of_a_look_names_the_shape_of_each_item_that_has_one():
    shape = ItemShape((0.3, -0.2, 0.413), (0.082, 0.071, 0.086), math.radians(67.0))
    look = Look([Detection("lime", (0.3, -0.2, 0.45), 0.9, shape=shape), Detection("bowl", (0.27, 0.07, 0.4), 0.8)], 320, 180, 1.0, 3, [], [])
    assert look.describe(1_000_000_000) == (
        "frame 3 taken 0.00 s before the search, 2 boxes: lime 0.90 (box 8.2 x 7.1 x 8.6 cm at [0.300, -0.200, 0.413], 67 deg), bowl 0.80"
    )


def test_an_item_keeps_the_shape_of_the_look_that_last_saw_it_and_a_place_clears_it():
    state = State(parse_limbs("left_arm:left_gripper"), "t0")
    first = ItemShape((0.3, -0.2, 0.413), (0.08, 0.07, 0.086), 0.0)
    second = ItemShape((0.31, -0.2, 0.413), (0.081, 0.07, 0.086), 0.1)
    [item] = state.remember([Detection("lime", (0.3, -0.2, 0.45), 0.9, shape=first)], 1, coverage=Coverage(every_label=True))
    assert item.shape == first
    [again] = state.remember([Detection("lime", (0.31, -0.2, 0.45), 0.9, shape=second)], 2, coverage=Coverage(every_label=True))
    assert again is item and item.shape == second
    gripper = state.grippers["left_gripper"]
    state.set_held(gripper, item.item_id)
    state.clear_held_placed(gripper, item.item_id, (0.4, 0.1, 0.5), None)
    assert item.shape is None
    assert state.mint_from_pose((0.5, 0.1, 0.7), None, 3).shape is None
