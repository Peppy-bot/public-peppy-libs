# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the obstacle projection scenarios of the ROS reference harness.

A scenario is a grid, the base's footprint with Nav2's default padding, the obstacle
layer's parameters with one observation source of a point cloud and footprint clearing
on, and the clouds of depth frames a pinhole camera renders of a scene, each from its own
pose with the base at BASE, whose footprint covers a box: a floor, a box inside the height band, a box that rises through the
band's top, a slab on the floor below the band, and a table top above the band with the
floor in view under it. Each frame is FRAME_WIDTH by FRAME_HEIGHT pixels, read at depths
of whole millimeters, and its points lie in the camera's optical frame (+X right, +Y down,
+Z along the view). The source's band is the layer's, both ranges lie past every point,
and the band's edges lie on multiples of the cell size, which the voxel map under test
takes as its voxel size. A point within MARGIN of a cell's edge or of the band's, in the
grid's frame, is left out, so the cell and the voxel a point lands in, and whether it lies
in the band, turn on no rounding of the frame they are judged in. Run it from any
directory:

    uv run ros_reference/tools/draw_obstacle_projection.py
    uv run ros_reference/tools/draw_obstacle_projection.py --drawn 6

The second draws a large set for a local run, which the README describes.
"""

import math

from drawing import generator, options, write

NAME = "obstacle_projection_room"
RESOLUTION = 0.05
GRID = {"size_x": 50, "size_y": 40, "resolution": RESOLUTION, "origin": [-0.25, -1.0]}
BAND = (0.05, 0.5)
FRAME_WIDTH = 24
FRAME_HEIGHT = 14
FOCAL = 15.0
DEPTH_UNIT = 0.001
MAX_DEPTH = 6.0
MARGIN = 2e-4
RANGE = 50.0
# The boxes of the scene, each [x min, x max, y min, y max, z min, z max] in meters.
BOXES = [
    [0.6, 0.85, -0.35, -0.05, 0.0, 0.4],
    [1.0, 1.2, -0.6, -0.4, 0.0, 0.9],
    [0.4, 0.6, 0.4, 0.6, 0.0, 0.03],
    [1.2, 1.8, 0.15, 0.75, 0.7, 0.74],
]
# The named cameras, each a position and a heading and a pitch down, in radians.
CAMERAS = [([0.0, 0.0, 0.6], 0.0, 0.45), ([0.2, -0.5, 0.8], 0.5, 0.6), ([0.1, 0.6, 0.45], -0.35, 0.25)]
# The base's footprint, 0.44 m by 0.40 m about its center, and Nav2's default padding.
FOOTPRINT = [[-0.22, -0.2], [0.22, -0.2], [0.22, 0.2], [-0.22, 0.2]]
FOOTPRINT_PADDING = 0.01
# Where the base stands, x, y and yaw, its padded footprint over the box at x from 1 to
# 1.2 m, which its clearing takes off the grid, turned so its edges cross cells
# diagonally.
BASE = [1.1, -0.5, 0.3]
# The cameras drawn beyond the named ones.
DRAWN_CAMERAS = 0


def parameters(band=BAND):
    """The obstacle layer's parameters for `band`, every one it reads pinned."""
    return {
        "enabled": True,
        "footprint_clearing_enabled": True,
        "min_obstacle_height": band[0],
        "max_obstacle_height": band[1],
        "combination_method": 1,
        "observation_sources": "depth",
        "tf_filter_tolerance": 0.05,
        "depth": {
            "topic": "points",
            "sensor_frame": "",
            "observation_persistence": 0.0,
            "expected_update_rate": 0.0,
            "data_type": "PointCloud2",
            "min_obstacle_height": band[0],
            "max_obstacle_height": band[1],
            "inf_is_valid": False,
            "marking": True,
            "clearing": False,
            "transport_type": "raw",
            "obstacle_max_range": RANGE,
            "obstacle_min_range": 0.0,
            "raytrace_max_range": RANGE,
            "raytrace_min_range": 0.0,
        },
    }


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def axes(heading, pitch):
    """The optical frame's +X, +Y and +Z in the grid's frame."""
    forward = [math.cos(pitch) * math.cos(heading), math.cos(pitch) * math.sin(heading), -math.sin(pitch)]
    right = [math.sin(heading), -math.cos(heading), 0.0]
    return right, cross(forward, right), forward


def quaternion(x_axis, y_axis, z_axis):
    """The unit quaternion [x, y, z, w] of the rotation whose columns are the axes."""
    m = [[x_axis[i], y_axis[i], z_axis[i]] for i in range(3)]
    w = math.sqrt(max(0.0, 1.0 + m[0][0] + m[1][1] + m[2][2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0][0] - m[1][1] - m[2][2])) / 2, m[2][1] - m[1][2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0][0] + m[1][1] - m[2][2])) / 2, m[0][2] - m[2][0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0][0] - m[1][1] + m[2][2])) / 2, m[1][0] - m[0][1])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / norm, y / norm, z / norm, w / norm]


def hit(origin, direction, boxes):
    """The least ray parameter at which the ray meets the floor or one of `boxes`, none
    past MAX_DEPTH."""
    hits = []
    if direction[2] < 0:
        hits.append(-origin[2] / direction[2])
    for box in boxes:
        near, far = -math.inf, math.inf
        for axis in range(3):
            low, high = box[2 * axis], box[2 * axis + 1]
            if direction[axis] == 0:
                if not low <= origin[axis] <= high:
                    break
                continue
            t0, t1 = sorted(((low - origin[axis]) / direction[axis], (high - origin[axis]) / direction[axis]))
            near, far = max(near, t0), min(far, t1)
        else:
            if near <= far and near > 0:
                hits.append(near)
    reached = [t for t in hits if t <= MAX_DEPTH]
    return min(reached) if reached else None


def clear_of_edges(point, grid, band):
    """Whether `point`, in the grid's frame, lies MARGIN or more from every edge of the
    cells of `grid` and from both ends of `band`."""
    for axis, origin in enumerate(grid["origin"]):
        offset = (point[axis] - origin) / grid["resolution"]
        if abs(offset - round(offset)) * grid["resolution"] < MARGIN:
            return False
    return all(abs(point[2] - edge) >= MARGIN for edge in band)


def frame(position, heading, pitch, boxes=BOXES, grid=GRID, band=BAND):
    """The sensor's pose and the points, in its optical frame, of the depth frame it
    renders of the floor and `boxes`, but for those `clear_of_edges` leaves out."""
    x_axis, y_axis, z_axis = axes(heading, pitch)
    points = []
    for v in range(FRAME_HEIGHT):
        for u in range(FRAME_WIDTH):
            ray = [(u - (FRAME_WIDTH - 1) / 2) / FOCAL, (v - (FRAME_HEIGHT - 1) / 2) / FOCAL, 1.0]
            direction = [x_axis[i] * ray[0] + y_axis[i] * ray[1] + z_axis[i] * ray[2] for i in range(3)]
            t = hit(position, direction, boxes)
            if t is None:
                continue
            depth = round(t / DEPTH_UNIT) * DEPTH_UNIT
            local = [ray[0] * depth, ray[1] * depth, depth]
            world = [position[i] + x_axis[i] * local[0] + y_axis[i] * local[1] + z_axis[i] * local[2] for i in range(3)]
            if clear_of_edges(world, grid, band):
                points.append(local)
    sensor = {"position": position, "orientation": quaternion(x_axis, y_axis, z_axis)}
    return {"sensor": sensor, "points": points}


def main():
    parser = options(__doc__, DRAWN_CAMERAS, "cameras drawn beyond the named ones")
    drawn = parser.parse_args().drawn
    rng = generator(NAME)
    cameras = CAMERAS + [
        ([rng.uniform(0.0, 0.4), rng.uniform(-0.6, 0.6), rng.uniform(0.4, 0.9)],
         rng.uniform(-0.6, 0.6), rng.uniform(0.2, 0.7))
        for _ in range(drawn)
    ]
    write(NAME, {
        "kind": "obstacle_projection",
        "grid": GRID,
        "footprint": FOOTPRINT,
        "footprint_padding": FOOTPRINT_PADDING,
        "obstacle": parameters(),
        "globals": {"track_unknown_space": True, "transform_tolerance": 0.1,
                    "allow_parameter_qos_overrides": True},
        "clouds": [{**frame(*camera), "base": BASE} for camera in cameras],
    })


if __name__ == "__main__":
    main()
