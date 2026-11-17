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
// Altered by Peppy Inc: this file translates into Rust the quaternion and vector arithmetic
// of tf2's LinearMath (geometry2 0.45.10), tf2/include/tf2/LinearMath/Quaternion.hpp,
// Vector3.hpp and Scalar.hpp, which tf2 takes from Bullet.

//! The quaternion and vector arithmetic of tf2's LinearMath, operation for operation, so a
//! lookup composes the bits tf2's `TransformAccum` composes. A quaternion is [x, y, z, w].

pub(crate) type Vector = [f64; 3];
pub(crate) type Quaternion = [f64; 4];

pub(crate) const IDENTITY: Quaternion = [0.0, 0.0, 0.0, 1.0];

/// `q1 * q2` (Quaternion.hpp:427-434).
pub(crate) fn product(q1: Quaternion, q2: Quaternion) -> Quaternion {
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
pub(crate) fn inverse(q: Quaternion) -> Quaternion {
    let [x, y, z, w] = q;
    [-x, -y, -z, w]
}

/// `normalized`: `q` multiplied by the reciprocal of its length (Quaternion.hpp:169-172,
/// 181-184, 207-219, 134-138).
pub(crate) fn normalized(q: Quaternion) -> Quaternion {
    let inverse_length = 1.0 / dot(q, q).sqrt();
    q.map(|c| c * inverse_length)
}

/// `quatRotate`: `v` turned by `q`, as `q * v * inverse(q)` (Quaternion.hpp:502-507).
pub(crate) fn rotate(q: Quaternion, v: Vector) -> Vector {
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
pub(crate) fn interpolated(v0: Vector, v1: Vector, ratio: f64) -> Vector {
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
pub(crate) fn slerp(q1: Quaternion, q2: Quaternion, t: f64) -> Quaternion {
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
