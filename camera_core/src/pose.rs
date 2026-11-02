//! Where one frame sits in another, as camera_geometry:v1 carries a transform:
//! a position in meters and a unit quaternion [x, y, z, w]. A pose carries a
//! point into the frame it is placed in, composes with another and inverts,
//! each in tf2's arithmetic, operation for operation, so a point lands on the
//! bits tf2 gives it. occupancy_map's parity with MoveIt's point cloud updater,
//! which places each point by tf2, compares those bits.

use std::fmt;

use crate::geometry::pose_is_valid;

/// Where one frame, the placed one, sits in another: the rotation of the
/// placed frame as tf2 builds the matrix of a quaternion, and its origin. A
/// point `p` of the placed frame is `R * p + origin` in the other.
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    rows: [[f64; 3]; 3],
    origin: [f64; 3],
}

/// A position and an orientation that make no pose: a coordinate that is not
/// a number, or a quaternion that is not of unit length.
#[derive(Debug)]
pub struct PoseError {
    position: [f64; 3],
    orientation: [f64; 4],
}

impl fmt::Display for PoseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "position {:?} and orientation {:?} make no pose: give a position of numbers \
             and a unit quaternion [x, y, z, w]",
            self.position, self.orientation
        )
    }
}

impl std::error::Error for PoseError {}

impl Pose {
    /// The pose whose origin is `position` and whose orientation is the unit
    /// quaternion `orientation`, [x, y, z, w], with its matrix built as tf2's
    /// `Matrix3x3::setRotation` builds it.
    pub fn new(position: [f64; 3], orientation: [f64; 4]) -> Result<Self, PoseError> {
        if !pose_is_valid(&position, &orientation) {
            return Err(PoseError {
                position,
                orientation,
            });
        }
        let [x, y, z, w] = orientation;
        let s = 2.0 / (x * x + y * y + z * z + w * w);
        let (xs, ys, zs) = (x * s, y * s, z * s);
        let (wx, wy, wz) = (w * xs, w * ys, w * zs);
        let (xx, xy, xz) = (x * xs, x * ys, x * zs);
        let (yy, yz, zz) = (y * ys, y * zs, z * zs);
        Ok(Self {
            rows: [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ],
            origin: position,
        })
    }

    /// The placed frame's origin, in the frame it is placed in.
    pub fn origin(&self) -> [f64; 3] {
        self.origin
    }

    /// A point of the placed frame in the frame it is placed in: each row of
    /// the rotation dotted with the point, plus the origin.
    pub fn apply(&self, point: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| dot(self.rows[i], point) + self.origin[i])
    }

    /// The pose of the frame `inner` places in this pose's placed frame, in
    /// the frame this pose is placed in: a point goes through `inner`, then
    /// through this pose. tf2's product of two transforms.
    pub fn compose(&self, inner: &Pose) -> Pose {
        let rows = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                inner.rows[0][j] * self.rows[i][0]
                    + inner.rows[1][j] * self.rows[i][1]
                    + inner.rows[2][j] * self.rows[i][2]
            })
        });
        Pose {
            rows,
            origin: self.apply(inner.origin),
        }
    }

    /// The pose of the frame this pose is placed in, in the placed frame:
    /// tf2's inverse, the rotation transposed and the origin carried back
    /// through it and negated.
    pub fn inverse(&self) -> Pose {
        let rows: [[f64; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| self.rows[j][i]));
        let back = self.origin.map(|c| -c);
        Pose {
            rows,
            origin: std::array::from_fn(|i| dot(rows[i], back)),
        }
    }
}

fn dot(row: [f64; 3], point: [f64; 3]) -> f64 {
    row[0] * point[0] + row[1] * point[1] + row[2] * point[2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::close;

    /// A pose with a turn about each axis and an offset on each.
    fn tilted() -> Pose {
        let norm = (0.1_f64 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9).sqrt();
        Pose::new([0.4, -0.5, 0.6], [0.1, 0.2, 0.3, 0.9].map(|c| c / norm))
            .expect("a unit quaternion")
    }

    #[test]
    fn a_pose_turns_a_point_and_moves_it_by_its_origin() {
        // A quarter turn about +Z takes +X to +Y, then the origin is added.
        let half = std::f64::consts::FRAC_PI_4;
        let pose = Pose::new([0.015, 0.0, -0.001], [0.0, 0.0, half.sin(), half.cos()])
            .expect("a unit quaternion");
        assert!(close(
            pose.apply([1.0, 0.0, 2.0]),
            [0.015, 1.0, 1.999],
            1e-12
        ));
        assert_eq!(pose.origin(), [0.015, 0.0, -0.001]);
        let identity = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("the identity");
        assert_eq!(identity.apply([0.3, -0.2, 1.5]), [0.3, -0.2, 1.5]);
    }

    // A composed pose carries a point as its two poses do one after the
    // other, inner first, and the inverse carries it back.
    #[test]
    fn composing_runs_the_inner_pose_first_and_the_inverse_undoes_a_pose() {
        let outer = tilted();
        let half = 0.35_f64;
        let inner = Pose::new([-0.06, 0.01, 0.002], [0.0, half.sin(), 0.0, half.cos()])
            .expect("a unit quaternion");
        let point = [0.7, -0.3, 1.1];
        let composed = outer.compose(&inner);
        assert!(close(
            composed.apply(point),
            outer.apply(inner.apply(point)),
            1e-12
        ));
        assert!(!close(
            composed.apply(point),
            inner.apply(outer.apply(point)),
            1e-12
        ));
        assert!(close(
            outer.inverse().apply(outer.apply(point)),
            point,
            1e-12
        ));
        assert!(close(
            outer.apply(outer.inverse().apply(point)),
            point,
            1e-12
        ));
    }

    #[test]
    fn a_pose_wants_numbers_and_a_unit_quaternion() {
        for (position, orientation) in [
            ([0.0; 3], [0.0; 4]),
            ([0.0; 3], [0.0, 0.0, 0.0, 2.0]),
            ([0.0; 3], [f64::NAN, 0.0, 0.0, 1.0]),
            ([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ([f64::INFINITY, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ] {
            assert!(
                Pose::new(position, orientation).is_err(),
                "{position:?} {orientation:?}"
            );
        }
        assert_eq!(
            Pose::new([0.0; 3], [0.0; 4]).unwrap_err().to_string(),
            "position [0.0, 0.0, 0.0] and orientation [0.0, 0.0, 0.0, 0.0] make no pose: \
             give a position of numbers and a unit quaternion [x, y, z, w]"
        );
    }
}
