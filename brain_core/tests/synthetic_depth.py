"""Synthetic depth images for the tests of an item's shape: a ray caster
that renders what the OpenArm v2 chest camera sees of a table and of simple
solids on it, with the true bounds of each solid and the picture box a
detector gives it.

The camera is the chest camera of the backbone (`CHEST`): 62 degrees below
the horizon, at 320 x 180 pixels, a quarter of its 1280 x 720 with the same
field of view. The table is the coffee table of the bowl scene, its top at
z 0.370. A depth reading is the distance along the optical axis, 0 where a
ray meets nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from brain_core.perception.camera import CameraModel, Intrinsics
from brain_core.ports import Box

WIDTH, HEIGHT = 320, 180
TABLE_Z = 0.370


def _product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# The chest camera's optical frame as the backbone reports it (see
# test_perception.py), and its intrinsics at a quarter of their resolution.
CHEST_POSITION = (0.0792, 0.0315, 0.7941)
CHEST_ORIENTATION = _product((0.1710647, -0.1710647, -0.6861027, 0.6861027), (1.0, 0.0, 0.0, 0.0))
CHEST = CameraModel().with_pose(CHEST_POSITION, CHEST_ORIENTATION).with_intrinsics(
    Intrinsics(WIDTH, HEIGHT, 738.1 / 4, 738.1 / 4, WIDTH / 2, HEIGHT / 2)
)


def _rays(camera: CameraModel) -> tuple[np.ndarray, np.ndarray]:
    """The origin of every ray and, per pixel centre, the robot-frame
    vector a depth of 1 m along the optical axis reaches: a point at depth
    t is origin + t * vector."""
    columns, rows = np.meshgrid(np.arange(WIDTH, dtype=float), np.arange(HEIGHT, dtype=float))
    at_one = camera.deproject_points(columns.ravel(), rows.ravel(), np.ones(WIDTH * HEIGHT), WIDTH, HEIGHT)
    origin = np.array(camera.pose.position)
    return origin, (at_one - origin).reshape(HEIGHT, WIDTH, 3)


class Solid:
    """A solid that a ray can meet: `depth` gives, per ray, the depth of the
    first point where the ray meets it, or inf."""

    def depth(self, origin: np.ndarray, rays: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """The lowest and the highest corner of its box aligned to the axes."""
        raise NotImplementedError


@dataclass
class Plane(Solid):
    """The horizontal plane z = `z`, seen from above."""

    z: float

    def depth(self, origin, rays):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.z - origin[2]) / rays[..., 2]
        return np.where(t > 0, t, np.inf)


@dataclass
class Sphere(Solid):
    centre: tuple[float, float, float]
    radius: float

    def depth(self, origin, rays):
        offset = origin - np.array(self.centre)
        a = np.einsum("...i,...i", rays, rays)
        b = 2.0 * np.einsum("...i,i", rays, offset)
        c = offset @ offset - self.radius**2
        disc = b * b - 4 * a * c
        root = np.sqrt(np.where(disc >= 0, disc, 0.0))
        t = (-b - root) / (2 * a)
        return np.where((disc >= 0) & (t > 0), t, np.inf)

    def bounds(self):
        centre = np.array(self.centre)
        return centre - self.radius, centre + self.radius


@dataclass
class UprightBox(Solid):
    """A box standing on its bottom: its centre, its full sides along its own
    x, y and z, and its turn `yaw_deg` about the vertical."""

    centre: tuple[float, float, float]
    sides: tuple[float, float, float]
    yaw_deg: float = 0.0

    def _turn(self, vectors: np.ndarray, sign: float) -> np.ndarray:
        c, s = math.cos(math.radians(sign * self.yaw_deg)), math.sin(math.radians(sign * self.yaw_deg))
        x, y, z = vectors[..., 0], vectors[..., 1], vectors[..., 2]
        return np.stack([c * x - s * y, s * x + c * y, z], axis=-1)

    def depth(self, origin, rays):
        local_origin = self._turn(origin - np.array(self.centre), -1.0)
        local_rays = self._turn(rays, -1.0)
        half = np.array(self.sides) / 2.0
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half - local_origin) / local_rays
            t2 = (half - local_origin) / local_rays
        near = np.nanmax(np.minimum(t1, t2), axis=-1)
        far = np.nanmin(np.maximum(t1, t2), axis=-1)
        return np.where((near <= far) & (near > 0), near, np.inf)

    def bounds(self):
        half = np.array(self.sides) / 2.0
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
        turned = self._turn(corners, 1.0) + np.array(self.centre)
        return turned.min(axis=0), turned.max(axis=0)


@dataclass
class Bowl(Solid):
    """A thin hemispherical bowl standing on the table: its rim a circle of
    `radius` at the height `rim_z`, its bottom `radius` below the rim. A ray
    meets its outside wall, or, through the opening, its inside."""

    centre_xy: tuple[float, float]
    radius: float
    rim_z: float

    def depth(self, origin, rays):
        sphere = np.array([self.centre_xy[0], self.centre_xy[1], self.rim_z])
        offset = origin - sphere
        a = np.einsum("...i,...i", rays, rays)
        b = 2.0 * np.einsum("...i,i", rays, offset)
        c = offset @ offset - self.radius**2
        disc = b * b - 4 * a * c
        root = np.sqrt(np.where(disc >= 0, disc, 0.0))
        best = np.full(a.shape, np.inf)
        for t in ((-b + root) / (2 * a), (-b - root) / (2 * a)):
            z = origin[2] + t * rays[..., 2]
            meets = (disc >= 0) & (t > 0) & (z <= self.rim_z)
            best = np.where(meets & (t < best), t, best)
        return best

    def bounds(self):
        low = np.array([self.centre_xy[0] - self.radius, self.centre_xy[1] - self.radius, self.rim_z - self.radius])
        high = np.array([self.centre_xy[0] + self.radius, self.centre_xy[1] + self.radius, self.rim_z])
        return low, high


@dataclass
class Scene:
    """A rendered scene: the depth image, and per solid the mask of the
    pixels where it is the first thing a ray meets."""

    depth_m: np.ndarray
    masks: list[np.ndarray]
    solids: list[Solid]

    def box(self, index: int, label: str = "item", loose_px: int = 2) -> Box:
        """The picture box a detector gives solid `index`: the box around
        its pixels, `loose_px` wider on each side."""
        rows, columns = np.nonzero(self.masks[index])
        return Box(
            label, 0.9,
            float(max(0, columns.min() - loose_px)), float(max(0, rows.min() - loose_px)),
            float(min(WIDTH, columns.max() + 1 + loose_px)), float(min(HEIGHT, rows.max() + 1 + loose_px)),
        )


def render(solids: Sequence[Solid], *, camera: CameraModel = CHEST, table_z: Optional[float] = TABLE_Z, noise_m: float = 0.0, seed: int = 7) -> Scene:
    """What `camera` sees of `solids` on the table: the depth image and the
    mask of each solid. `noise_m` adds a Gaussian error of that size to
    each reading, from a generator of a fixed `seed`, so every run gives
    the same image."""
    origin, rays = _rays(camera)
    depths = [solid.depth(origin, rays) for solid in solids]
    nearest = np.full((HEIGHT, WIDTH), np.inf)
    for depth in depths:
        nearest = np.minimum(nearest, depth)
    background = Plane(table_z).depth(origin, rays) if table_z is not None else np.full((HEIGHT, WIDTH), np.inf)
    masks = [(depth == nearest) & (depth < background) & np.isfinite(depth) for depth in depths]
    image = np.minimum(nearest, background)
    image = np.where(np.isfinite(image), image, 0.0)
    if noise_m > 0.0:
        image = np.where(image > 0.0, image + np.random.default_rng(seed).normal(0.0, noise_m, image.shape), 0.0)
    return Scene(image, masks, list(solids))
