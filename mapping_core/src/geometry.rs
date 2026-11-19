// Copyright 2010 SRI International
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This file translates into Rust karto's planar geometry as slam_toolbox 2.10.0 carries it:
// lib/karto_sdk/include/karto_sdk/Math.h and Karto.h (Copyright 2010 SRI International).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! The planar geometry of karto, as slam_toolbox 2.10.0 carries it
//! (lib/karto_sdk/include/karto_sdk): the rounding, the tolerance and the angle
//! normalization of Math.h, and Karto.h's `Vector2`, `Pose2`, `Matrix3`, `Transform` and
//! `BoundingBox2`, with their arithmetic in its order.

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// `KT_TOLERANCE`: how near two values lie for karto to take them as equal.
pub(crate) const TOLERANCE: f64 = 1e-6;

/// `math::Round`: half away from zero.
pub(crate) fn round(value: f64) -> f64 {
    if value >= 0.0 {
        (value + 0.5).floor()
    } else {
        (value - 0.5).ceil()
    }
}

/// `math::DoubleEqual`: whether `a` and `b` lie within [`TOLERANCE`].
pub(crate) fn double_equal(a: f64, b: f64) -> bool {
    let delta = a - b;
    if delta < 0.0 {
        delta >= -TOLERANCE
    } else {
        delta <= TOLERANCE
    }
}

/// `math::NormalizeAngle`: `angle` moved by whole turns into [-pi, pi]. A non-finite angle
/// is returned as it is.
pub(crate) fn normalize_angle(angle: f64) -> f64 {
    if !angle.is_finite() {
        return angle;
    }
    let mut angle = angle;
    while angle < -PI {
        if angle < -TAU {
            angle += f64::from((angle / -TAU) as u32) * TAU;
        } else {
            angle += TAU;
        }
    }
    while angle > PI {
        if angle > TAU {
            angle -= f64::from((angle / TAU) as u32) * TAU;
        } else {
            angle -= TAU;
        }
    }
    angle
}

/// `math::NormalizeAngleDifference`: `minuend` moved by whole turns to within pi of
/// `subtrahend`. A non-finite difference leaves `minuend` as it is.
pub(crate) fn normalize_angle_difference(minuend: f64, subtrahend: f64) -> f64 {
    if !(minuend - subtrahend).is_finite() {
        return minuend;
    }
    let mut minuend = minuend;
    while minuend - subtrahend < -PI {
        minuend += TAU;
    }
    while minuend - subtrahend > PI {
        minuend -= TAU;
    }
    minuend
}

/// A double converted to an int as the amd64 build converts it (`cvttsd2si`): truncated
/// toward zero, and `i32::MIN`, the integer indefinite, for NaN and for a value whose
/// truncation lies outside the ints.
pub(crate) fn truncate_to_i32(value: f64) -> i32 {
    if value > -2_147_483_649.0 && value < 2_147_483_648.0 {
        value as i32
    } else {
        i32::MIN
    }
}

/// `Vector2<kt_double>`: a point or a displacement on the floor (m).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Vector2 {
    pub x: f64,
    pub y: f64,
}

impl Vector2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn plus(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }

    pub fn minus(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }

    pub fn scaled(self, factor: f64) -> Self {
        Self::new(self.x * factor, self.y * factor)
    }

    pub fn divided(self, divisor: f64) -> Self {
        Self::new(self.x / divisor, self.y / divisor)
    }

    pub fn squared_length(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn squared_distance(self, other: Self) -> f64 {
        self.minus(other).squared_length()
    }
}

/// A pose on the floor: a position (m) and a heading (rad), karto's `Pose2`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub(crate) fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub(crate) fn at(position: Vector2, yaw: f64) -> Self {
        Self::new(position.x, position.y, yaw)
    }

    pub(crate) fn position(self) -> Vector2 {
        Vector2::new(self.x, self.y)
    }

    /// `Pose2::SquaredDistance`: of the positions alone.
    pub(crate) fn squared_distance(self, other: Self) -> f64 {
        self.position().squared_distance(other.position())
    }

    /// `Pose2::operator+`: the sum of the positions, and of the headings normalized.
    pub(crate) fn plus(self, other: Self) -> Self {
        Self::at(
            self.position().plus(other.position()),
            normalize_angle(self.yaw + other.yaw),
        )
    }

    /// `Pose2::operator-`: the difference of the positions, and of the headings
    /// normalized.
    pub(crate) fn minus(self, other: Self) -> Self {
        Self::at(
            self.position().minus(other.position()),
            normalize_angle(self.yaw - other.yaw),
        )
    }
}

impl From<camera_core::pose::PlanarPose> for Pose2 {
    fn from(pose: camera_core::pose::PlanarPose) -> Self {
        Self::new(pose.x(), pose.y(), pose.yaw())
    }
}

impl From<Pose2> for camera_core::pose::PlanarPose {
    /// A pose the crate computed from finite poses, which the type of every pose a caller
    /// gives holds finite.
    fn from(pose: Pose2) -> Self {
        Self::new(pose.x, pose.y, pose.yaw).expect("a pose computed from finite poses is finite")
    }
}

/// `Matrix3`: a 3x3 matrix, row by row.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Matrix3(pub [[f64; 3]; 3]);

impl Matrix3 {
    pub fn zero() -> Self {
        Self([[0.0; 3]; 3])
    }

    pub fn identity() -> Self {
        Self([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    }

    /// `FromAxisAngle(0, 0, 1, radians)`: the turn by `radians` about the z axis, with
    /// karto's arithmetic of a general axis on the axis (0, 0, 1).
    pub fn turn(radians: f64) -> Self {
        let (x, y, z) = (0.0f64, 0.0f64, 1.0f64);
        let cos = radians.cos();
        let sin = radians.sin();
        let one_minus_cos = 1.0 - cos;
        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy_m_cos = x * y * one_minus_cos;
        let xz_m_cos = x * z * one_minus_cos;
        let yz_m_cos = y * z * one_minus_cos;
        let x_sin = x * sin;
        let y_sin = y * sin;
        let z_sin = z * sin;
        Self([
            [xx * one_minus_cos + cos, xy_m_cos - z_sin, xz_m_cos + y_sin],
            [xy_m_cos + z_sin, yy * one_minus_cos + cos, yz_m_cos - x_sin],
            [xz_m_cos - y_sin, yz_m_cos + x_sin, zz * one_minus_cos + cos],
        ])
    }

    pub fn transpose(self) -> Self {
        let m = self.0;
        Self([0, 1, 2].map(|row| [0, 1, 2].map(|column| m[column][row])))
    }

    /// `Matrix3::Inverse`: `InverseFast` with a tolerance of 1e-14 on the determinant.
    /// Of a matrix whose determinant lies within it, the cofactors unscaled, as karto
    /// leaves them.
    pub fn inverse(self) -> Self {
        let m = self.0;
        let mut inverse = [
            [
                m[1][1] * m[2][2] - m[1][2] * m[2][1],
                m[0][2] * m[2][1] - m[0][1] * m[2][2],
                m[0][1] * m[1][2] - m[0][2] * m[1][1],
            ],
            [
                m[1][2] * m[2][0] - m[1][0] * m[2][2],
                m[0][0] * m[2][2] - m[0][2] * m[2][0],
                m[0][2] * m[1][0] - m[0][0] * m[1][2],
            ],
            [
                m[1][0] * m[2][1] - m[1][1] * m[2][0],
                m[0][1] * m[2][0] - m[0][0] * m[2][1],
                m[0][0] * m[1][1] - m[0][1] * m[1][0],
            ],
        ];
        let determinant =
            m[0][0] * inverse[0][0] + m[0][1] * inverse[1][0] + m[0][2] * inverse[2][0];
        if determinant.abs() <= 1e-14 {
            return Self(inverse);
        }
        let inverse_determinant = 1.0 / determinant;
        for row in &mut inverse {
            for value in row.iter_mut() {
                *value *= inverse_determinant;
            }
        }
        Self(inverse)
    }

    pub fn times(self, other: Self) -> Self {
        let (a, b) = (self.0, other.0);
        Self([0, 1, 2].map(|row| {
            [0, 1, 2].map(|column| {
                a[row][0] * b[0][column] + a[row][1] * b[1][column] + a[row][2] * b[2][column]
            })
        }))
    }

    /// `Matrix3::operator*(Pose2)`: the matrix times (x, y, heading).
    pub fn times_pose(self, pose: Pose2) -> Pose2 {
        let m = self.0;
        Pose2::new(
            m[0][0] * pose.x + m[0][1] * pose.y + m[0][2] * pose.yaw,
            m[1][0] * pose.x + m[1][1] * pose.y + m[1][2] * pose.yaw,
            m[2][0] * pose.x + m[2][1] * pose.y + m[2][2] * pose.yaw,
        )
    }

    pub fn plus(self, other: Self) -> Self {
        let (a, b) = (self.0, other.0);
        Self([0, 1, 2].map(|row| [0, 1, 2].map(|column| a[row][column] + b[row][column])))
    }
}

/// `Transform`: the rigid motion that takes one pose to another.
pub(crate) struct Transform {
    transform: Pose2,
    rotation: Matrix3,
    inverse_rotation: Matrix3,
}

impl Transform {
    /// `Transform(pose)`: the motion from the origin to `pose`.
    pub fn to(pose: Pose2) -> Self {
        Self::between(Pose2::default(), pose)
    }

    /// `Transform(pose1, pose2)`: the motion that takes `from` to `to`.
    pub fn between(from: Pose2, to: Pose2) -> Self {
        if from == to {
            return Self {
                transform: Pose2::default(),
                rotation: Matrix3::identity(),
                inverse_rotation: Matrix3::identity(),
            };
        }
        let rotation = Matrix3::turn(to.yaw - from.yaw);
        let inverse_rotation = Matrix3::turn(from.yaw - to.yaw);
        let position = if from.x != 0.0 || from.y != 0.0 {
            to.minus(rotation.times_pose(from)).position()
        } else {
            to.position()
        };
        Self {
            transform: Pose2::at(position, to.yaw - from.yaw),
            rotation,
            inverse_rotation,
        }
    }

    /// `TransformPose`: `pose` moved by the motion.
    pub fn transform_pose(&self, pose: Pose2) -> Pose2 {
        let moved = self.transform.plus(self.rotation.times_pose(pose));
        Pose2::at(
            moved.position(),
            normalize_angle(pose.yaw + self.transform.yaw),
        )
    }

    /// `InverseTransformPose`: `pose` moved by the inverse of the motion.
    pub fn inverse_transform_pose(&self, pose: Pose2) -> Pose2 {
        let moved = self.inverse_rotation.times_pose(pose.minus(self.transform));
        Pose2::at(
            moved.position(),
            normalize_angle(pose.yaw - self.transform.yaw),
        )
    }
}

/// `BoundingBox2`: the box that holds the points added to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BoundingBox {
    pub minimum: Vector2,
    pub maximum: Vector2,
}

impl Default for BoundingBox {
    /// The empty box, which karto starts from 1e18 on each side.
    fn default() -> Self {
        Self {
            minimum: Vector2::new(1e18, 1e18),
            maximum: Vector2::new(-1e18, -1e18),
        }
    }
}

impl BoundingBox {
    /// `Add(point)`: the box grown to hold `point`.
    pub fn add(&mut self, point: Vector2) {
        if point.x < self.minimum.x {
            self.minimum.x = point.x;
        }
        if point.y < self.minimum.y {
            self.minimum.y = point.y;
        }
        if point.x > self.maximum.x {
            self.maximum.x = point.x;
        }
        if point.y > self.maximum.y {
            self.maximum.y = point.y;
        }
    }

    /// `Add(box)`: the box grown to hold the corners of `other`.
    pub fn add_box(&mut self, other: &BoundingBox) {
        self.add(other.minimum);
        self.add(other.maximum);
    }

    pub fn size(&self) -> Vector2 {
        self.maximum.minus(self.minimum)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_takes_halves_away_from_zero() {
        assert_eq!(round(2.5), 3.0);
        assert_eq!(round(-2.5), -3.0);
        assert_eq!(round(2.4999), 2.0);
    }

    #[test]
    fn an_angle_normalizes_by_whole_turns() {
        assert_eq!(normalize_angle(3.0 * PI), 3.0 * PI - TAU);
        assert!((normalize_angle(-7.0) - (-7.0 + TAU)).abs() < 1e-15);
        assert_eq!(normalize_angle(1.0), 1.0);
        assert!(normalize_angle(f64::NAN).is_nan());
        assert_eq!(normalize_angle(f64::INFINITY), f64::INFINITY);
    }

    #[test]
    fn an_angle_difference_moves_the_minuend_within_pi_of_the_subtrahend() {
        let moved = normalize_angle_difference(3.0, -3.0);
        assert!((moved - (3.0 - TAU)).abs() < 1e-15);
        assert_eq!(normalize_angle_difference(0.5, 0.0), 0.5);
    }

    #[test]
    fn a_double_out_of_the_ints_converts_to_the_integer_indefinite() {
        assert_eq!(truncate_to_i32(-2.7), -2);
        assert_eq!(truncate_to_i32(f64::NAN), i32::MIN);
        assert_eq!(truncate_to_i32(f64::INFINITY), i32::MIN);
        assert_eq!(truncate_to_i32(3e9), i32::MIN);
        assert_eq!(truncate_to_i32(-2_147_483_647.5), -2_147_483_647);
    }

    #[test]
    fn a_transform_takes_its_first_pose_to_its_second() {
        let from = Pose2::new(1.0, -2.0, 0.3);
        let to = Pose2::new(-0.5, 4.0, -2.0);
        let moved = Transform::between(from, to).transform_pose(from);
        assert!((moved.x - to.x).abs() < 1e-12 && (moved.y - to.y).abs() < 1e-12);
        assert!((moved.yaw - to.yaw).abs() < 1e-12);
        let back = Transform::between(from, to).inverse_transform_pose(moved);
        assert!((back.x - from.x).abs() < 1e-12 && (back.y - from.y).abs() < 1e-12);
    }

    #[test]
    fn the_inverse_of_a_matrix_times_it_is_the_identity() {
        let m = Matrix3([[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]]);
        let product = m.times(m.inverse()).0;
        for (row, values) in product.iter().enumerate() {
            for (column, value) in values.iter().enumerate() {
                let expected = if row == column { 1.0 } else { 0.0 };
                assert!((value - expected).abs() < 1e-12);
            }
        }
    }
}
