// Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
// Altered by Peppy Inc: this file translates into Rust the quaternion, vector and matrix
// arithmetic of tf2's LinearMath (geometry2 0.45.10), tf2/include/tf2/LinearMath/Quaternion.hpp,
// Vector3.hpp, Matrix3x3.hpp and Scalar.hpp, which tf2 takes from Bullet, tf2's getYaw
// (tf2/include/tf2/impl/utils.hpp), and KDL's rotation arithmetic (orocos_kdl 1.5.2
// src/frames.cpp), which tf2_geometry_msgs runs a pose's turn through.

//! The rotation arithmetic of the libraries, operation for operation as tf2 and KDL run it, so a
//! lookup composes the bits tf2's `TransformAccum` composes and a point lands on the bits tf2 gives
//! it. A quaternion is [x, y, z, w]; a rotation is a 3x3 matrix, row by row.

pub type Vector = [f64; 3];
pub type Quaternion = [f64; 4];
/// A rotation matrix, row by row, as tf2's `Matrix3x3` holds it.
pub type Rotation = [[f64; 3]; 3];

pub const IDENTITY: Quaternion = [0.0, 0.0, 0.0, 1.0];

/// `q1 * q2` (Quaternion.hpp:427-434).
pub fn product(q1: Quaternion, q2: Quaternion) -> Quaternion {
    let [x1, y1, z1, w1] = q1;
    let [x2, y2, z2, w2] = q2;
    [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
        w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
}

/// `q * v`, the vector taken as a quaternion of zero w (Quaternion.hpp:436-443).
fn product_with_vector(q: Quaternion, v: Vector) -> Quaternion {
    let [x, y, z, w] = q;
    let [vx, vy, vz] = v;
    [
        w * vx + y * vz - z * vy,
        w * vy + z * vx - x * vz,
        w * vz + x * vy - y * vx,
        -x * vx - y * vy - z * vz,
    ]
}

/// tf2's `Quaternion::inverse`, the conjugate (Quaternion.hpp:282-285).
pub fn inverse(q: Quaternion) -> Quaternion {
    let [x, y, z, w] = q;
    [-x, -y, -z, w]
}

/// `normalized`: `q` multiplied by the reciprocal of its length (Quaternion.hpp:169-172,
/// 181-184, 207-219, 134-138).
pub fn normalized(q: Quaternion) -> Quaternion {
    let inverse_length = 1.0 / dot(q, q).sqrt();
    q.map(|c| c * inverse_length)
}

/// `quatRotate`: `v` turned by `q`, as `q * v * inverse(q)` (Quaternion.hpp:502-507).
pub fn rotate(q: Quaternion, v: Vector) -> Vector {
    let [x, y, z, _] = product(product_with_vector(q, v), inverse(q));
    [x, y, z]
}

/// `v1 + v2` (Vector3.hpp:362).
pub(crate) fn sum(v1: Vector, v2: Vector) -> Vector {
    [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]
}

/// `-v` (Vector3.hpp:382-385).
pub(crate) fn negated(v: Vector) -> Vector {
    [-v[0], -v[1], -v[2]]
}

/// `setInterpolate3`: `v0` blended toward `v1` by `ratio` (Vector3.hpp:222-230).
pub(crate) fn interpolated(v0: [f64; 3], v1: [f64; 3], ratio: f64) -> [f64; 3] {
    let s = 1.0 - ratio;
    [
        s * v0[0] + ratio * v1[0],
        s * v0[1] + ratio * v1[1],
        s * v0[2] + ratio * v1[2],
    ]
}

/// The dot product of two quaternions (Quaternion.hpp:155-159).
fn dot(q1: Quaternion, q2: Quaternion) -> f64 {
    q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
}

/// `tf2Acos`, which clamps its argument into [-1, 1] (Scalar.hpp:182).
fn clamped_acos(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).acos()
}

/// `angleShortestPath`: twice the angle between `q1` and the nearer of `q2` and `-q2`
/// (Quaternion.hpp:239-247).
fn angle_shortest_path(q1: Quaternion, q2: Quaternion) -> f64 {
    let s = (dot(q1, q1) * dot(q2, q2)).sqrt();
    let d = dot(q1, q2);
    if d < 0.0 {
        clamped_acos(-d / s) * 2.0
    } else {
        clamped_acos(d / s) * 2.0
    }
}

/// `slerp`: `q1` turned toward `q2` by the fraction `t` of the shorter arc between them;
/// `q1` itself where the two give one rotation (Quaternion.hpp:340-364).
pub(crate) fn slerp(q1: [f64; 4], q2: [f64; 4], t: f64) -> [f64; 4] {
    let theta = angle_shortest_path(q1, q2) / 2.0;
    if theta == 0.0 {
        return q1;
    }
    let d = 1.0 / theta.sin();
    let s0 = ((1.0 - t) * theta).sin();
    let s1 = (t * theta).sin();
    let toward = if dot(q1, q2) < 0.0 {
        q2.map(|c| -c)
    } else {
        q2
    };
    std::array::from_fn(|i| (q1[i] * s0 + toward[i] * s1) * d)
}

/// `Matrix3x3::setRotation`: the rotation of the quaternion `q`, built from `q` as given, its
/// length divided out by `2 / |q|^2`.
pub fn rotation_of(q: Quaternion) -> Rotation {
    let [x, y, z, w] = q;
    let s = 2.0 / (x * x + y * y + z * z + w * w);
    let (xs, ys, zs) = (x * s, y * s, z * s);
    let (wx, wy, wz) = (w * xs, w * ys, w * zs);
    let (xx, xy, xz) = (x * xs, x * ys, x * zs);
    let (yy, yz, zz) = (y * ys, y * zs, z * zs);
    [
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ]
}

/// `Matrix3x3::getRotation`: the quaternion of the rotation `m`, from its trace when the trace
/// is above 0, else from its largest diagonal entry.
pub fn quaternion_of(m: &Rotation) -> Quaternion {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let root = (trace + 1.0).sqrt();
        let s = 0.5 / root;
        return [
            (m[2][1] - m[1][2]) * s,
            (m[0][2] - m[2][0]) * s,
            (m[1][0] - m[0][1]) * s,
            root * 0.5,
        ];
    }
    let i = if m[0][0] < m[1][1] {
        if m[1][1] < m[2][2] { 2 } else { 1 }
    } else if m[0][0] < m[2][2] {
        2
    } else {
        0
    };
    let (j, k) = ((i + 1) % 3, (i + 2) % 3);
    let root = (m[i][i] - m[j][j] - m[k][k] + 1.0).sqrt();
    let s = 0.5 / root;
    let mut q = [0.0; 4];
    q[i] = root * 0.5;
    q[3] = (m[k][j] - m[j][k]) * s;
    q[j] = (m[j][i] + m[i][j]) * s;
    q[k] = (m[k][i] + m[i][k]) * s;
    q
}

/// `m * v`: each row of `m` dotted with `v`.
pub fn rotated(m: &Rotation, v: Vector) -> Vector {
    std::array::from_fn(|i| row_dot(m[i], v))
}

/// `outer * inner`, the rotation of `inner` then `outer`, each entry summed in tf2's order.
pub fn rotation_product(outer: &Rotation, inner: &Rotation) -> Rotation {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            inner[0][j] * outer[i][0] + inner[1][j] * outer[i][1] + inner[2][j] * outer[i][2]
        })
    })
}

/// The transpose of `m`, its inverse.
pub(crate) fn transposed(m: &Rotation) -> Rotation {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// One row of a rotation dotted with a vector.
fn row_dot(row: Vector, v: Vector) -> f64 {
    row[0] * v[0] + row[1] * v[1] + row[2] * v[2]
}

/// The sine of the pitch at or past which tf2's `getYaw` reads a quaternion's heading from its
/// x and y alone, as for a pitch of a quarter turn (tf2 include/tf2/impl/utils.hpp:158-161).
const POLE_PITCH_SINE: f64 = 0.99999;

/// The quaternion of a turn by `yaw` about +Z, as tf2's `Quaternion::setRPY(0, 0, yaw)` makes
/// it (Quaternion.hpp:98-113), the roll and the pitch of 0 kept in its arithmetic so each sign
/// of zero comes out as tf2's.
pub fn quaternion_of_yaw(yaw: f64) -> Quaternion {
    let (roll, pitch) = (0.0_f64, 0.0_f64);
    let half_yaw = yaw * 0.5;
    let half_pitch = pitch * 0.5;
    let half_roll = roll * 0.5;
    let (cos_yaw, sin_yaw) = (half_yaw.cos(), half_yaw.sin());
    let (cos_pitch, sin_pitch) = (half_pitch.cos(), half_pitch.sin());
    let (cos_roll, sin_roll) = (half_roll.cos(), half_roll.sin());
    [
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    ]
}

/// The heading of the quaternion `q`, as tf2's `getYaw` reads it (tf2
/// include/tf2/impl/utils.hpp:140-166): the yaw of its yaw, pitch and roll, in [-pi, pi]; for a
/// pitch whose sine reaches [`POLE_PITCH_SINE`] either way, twice the angle of its x and y,
/// signed as the pitch, in [-2 pi, 2 pi].
pub fn yaw_of_quaternion(q: Quaternion) -> f64 {
    let [x, y, z, w] = q;
    let (sqx, sqy, sqz, sqw) = (x * x, y * y, z * z, w * w);
    let pitch_sine = -2.0 * (x * z - w * y) / (sqx + sqy + sqz + sqw);
    if pitch_sine <= -POLE_PITCH_SINE {
        return -2.0 * y.atan2(x);
    }
    if pitch_sine >= POLE_PITCH_SINE {
        return 2.0 * y.atan2(x);
    }
    (2.0 * (x * y + w * z)).atan2(sqw + sqx - sqy - sqz)
}

/// KDL's rotation arithmetic (orocos_kdl 1.5.2 src/frames.cpp), which tf2_geometry_msgs's
/// `doTransform` runs a pose's turn through. A rotation here is KDL's 3x3 matrix, row by row in
/// nine entries.
pub mod kdl {
    use super::Quaternion;

    /// The trace above which KDL's `GetQuaternion` takes a rotation's quaternion from its trace
    /// (frames.cpp:207-208).
    const TRACE_EPSILON: f64 = 1e-12;

    /// KDL's rotation matrix of a quaternion (`Rotation::Quaternion`, frames.cpp:190-197).
    pub fn rotation([x, y, z, w]: Quaternion) -> [f64; 9] {
        let (x2, y2, z2, w2) = (x * x, y * y, z * z, w * w);
        [
            w2 + x2 - y2 - z2,
            2.0 * x * y - 2.0 * w * z,
            2.0 * x * z + 2.0 * w * y,
            2.0 * x * y + 2.0 * w * z,
            w2 - x2 + y2 - z2,
            2.0 * y * z - 2.0 * w * x,
            2.0 * x * z - 2.0 * w * y,
            2.0 * y * z + 2.0 * w * x,
            w2 - x2 - y2 + z2,
        ]
    }

    /// KDL's product of two rotation matrices (frames.cpp:173-187).
    pub fn product(lhs: [f64; 9], rhs: [f64; 9]) -> [f64; 9] {
        std::array::from_fn(|index| {
            let (row, column) = (index / 3 * 3, index % 3);
            lhs[row] * rhs[column] + lhs[row + 1] * rhs[column + 3] + lhs[row + 2] * rhs[column + 6]
        })
    }

    /// KDL's `Rotation::GetQuaternion` of a turn about +Z (frames.cpp:204-235). Below its trace
    /// epsilon KDL picks the branch of the largest diagonal entry; for a turn about +Z the third
    /// entry is 1 to rounding and the first two are the turn's cosine, at most about -0.5 there,
    /// so the third entry's branch runs and KDL's other two are left out.
    pub fn quaternion_of_turn_about_z(m: [f64; 9]) -> Quaternion {
        let trace = m[0] + m[4] + m[8];
        if trace > TRACE_EPSILON {
            let s = 0.5 / (trace + 1.0).sqrt();
            return [
                (m[7] - m[5]) * s,
                (m[2] - m[6]) * s,
                (m[3] - m[1]) * s,
                0.25 / s,
            ];
        }
        let first_largest = m[0] > m[4] && m[0] > m[8];
        let second_largest = m[4] > m[8];
        debug_assert!(
            !first_largest && !second_largest,
            "a turn about +Z whose third diagonal entry is not the largest: {m:?}"
        );
        let s = 2.0 * (1.0 + m[8] - m[0] - m[4]).sqrt();
        [
            (m[2] + m[6]) / s,
            (m[5] + m[7]) / s,
            0.25 * s,
            (m[3] - m[1]) / s,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn about_z(angle: f64) -> Quaternion {
        [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()]
    }

    fn close(a: &[f64], b: &[f64]) -> bool {
        a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-12)
    }

    #[test]
    fn a_quarter_turn_about_z_takes_x_to_y() {
        let turned = rotate(about_z(std::f64::consts::FRAC_PI_2), [1.0, 0.0, 2.0]);
        assert!(close(&turned, &[0.0, 1.0, 2.0]), "{turned:?}");
        assert!(close(&product(about_z(0.3), about_z(0.4)), &about_z(0.7)));
        assert!(close(
            &product(about_z(0.3), inverse(about_z(0.3))),
            &IDENTITY
        ));
    }

    #[test]
    fn slerp_turns_along_the_shorter_arc_and_keeps_q1_for_one_rotation() {
        assert!(close(
            &slerp(about_z(0.2), about_z(1.0), 0.25),
            &about_z(0.4)
        ));
        // -q2 is q2's rotation: the dot product is negative, and the arc is the same.
        let far = about_z(1.0).map(|c| -c);
        assert!(close(&slerp(about_z(0.2), far, 0.25), &about_z(0.4)));
        let q = about_z(0.7);
        assert_eq!(slerp(q, q, 0.5), q);
    }

    #[test]
    fn interpolation_blends_each_coordinate() {
        assert_eq!(
            interpolated([0.0, 2.0, -4.0], [4.0, 2.0, 0.0], 0.25),
            [1.0, 2.0, -3.0]
        );
        assert_eq!(sum([1.0, 2.0, 3.0], negated([1.0, 2.0, 3.0])), [0.0; 3]);
    }
}
