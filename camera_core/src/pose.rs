//! Where one frame sits in another, as camera_geometry:v1 carries a transform:
//! a position in meters and a unit quaternion [x, y, z, w]. A pose carries a
//! point into the frame it is placed in, composes with another and inverts,
//! each in tf2's arithmetic, operation for operation, so a point lands on the
//! bits tf2 gives it. occupancy_map's parity with MoveIt's point cloud updater,
//! which places each point by tf2, compares those bits. A pose also gives its
//! orientation back as a quaternion and runs a share of the way to another, for
//! a sensor that moves while it measures. The quaternion of a turn about +Z
//! and the heading of a quaternion come out as tf2's `setRPY` makes the one and
//! its `getYaw` reads the other.
//!
//! A [`PlanarPose`] is where one frame sits in another on the floor plane: a
//! position and a heading, the pose of a mobile base and of the frames it
//! drives in.

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

    /// The placed frame's orientation, a unit quaternion [x, y, z, w], from the
    /// rotation as tf2's `Matrix3x3::getRotation` takes it: from the trace when
    /// it is above 0, else from the largest diagonal entry.
    pub fn orientation(&self) -> [f64; 4] {
        let m = &self.rows;
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

    /// The pose `ratio` of the way from this pose to `to`, 0 this pose and 1
    /// `to`: the origins interpolated as tf2's `Vector3::setInterpolate3`, and
    /// the orientations by tf2's `Quaternion::slerp`, the shorter way round.
    pub fn interpolated(&self, to: &Pose, ratio: f64) -> Pose {
        let rest = 1.0 - ratio;
        let origin = std::array::from_fn(|i| rest * self.origin[i] + ratio * to.origin[i]);
        let orientation = slerp(self.orientation(), to.orientation(), ratio);
        Pose::new(origin, orientation).expect("a slerp of two unit quaternions is one")
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

fn dot4(a: [f64; 4], b: [f64; 4]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// tf2's `Quaternion::slerp` of `from` towards `to` by `ratio`: half the
/// shortest angle between them (`angleShortestPath`, its arccosine clamped as
/// `tf2Acos` clamps it), `to` negated when it lies the long way round, and
/// `from` itself when the angle is 0.
fn slerp(from: [f64; 4], to: [f64; 4], ratio: f64) -> [f64; 4] {
    let dot = dot4(from, to);
    let length = (dot4(from, from) * dot4(to, to)).sqrt();
    let cosine = (dot.abs() / length).clamp(-1.0, 1.0);
    let theta = cosine.acos();
    if theta == 0.0 {
        return from;
    }
    let d = 1.0 / theta.sin();
    let s0 = ((1.0 - ratio) * theta).sin();
    let s1 = (ratio * theta).sin();
    let toward = if dot < 0.0 { to.map(|c| -c) } else { to };
    std::array::from_fn(|i| (from[i] * s0 + toward[i] * s1) * d)
}

/// The sine of the pitch at or past which tf2's `getYaw` reads a quaternion's
/// heading from its x and y alone, as for a pitch of a quarter turn (tf2
/// include/tf2/impl/utils.hpp:158-161, geometry2 0.45.10).
const POLE_PITCH_SINE: f64 = 0.99999;

/// The quaternion [x, y, z, w] of a turn by `yaw` about +Z, as tf2's
/// `Quaternion::setRPY(0, 0, yaw)` makes it (tf2
/// include/tf2/LinearMath/Quaternion.hpp:98-113, geometry2 0.45.10), the
/// roll and the pitch of 0 kept in its arithmetic so each sign of zero comes
/// out as tf2's.
pub fn quaternion_of_yaw(yaw: f64) -> [f64; 4] {
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

/// The heading of the quaternion [x, y, z, w], as tf2's `getYaw` reads it
/// (tf2 include/tf2/impl/utils.hpp:140-166, geometry2 0.45.10): the yaw of
/// its yaw, pitch and roll, in [-pi, pi]; for a pitch whose sine reaches
/// [`POLE_PITCH_SINE`] either way, twice the angle of its x and y, signed as
/// the pitch, in [-2 pi, 2 pi].
pub fn yaw_of_quaternion([x, y, z, w]: [f64; 4]) -> f64 {
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

/// Where one frame sits in another on the floor plane: a position in meters,
/// and a heading in radians, counterclockwise about +Z from +X.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl PlanarPose {
    /// The pose that places a frame on the frame it is placed in.
    pub const IDENTITY: PlanarPose = PlanarPose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    /// A point of the placed frame in the frame the pose places it in: turned
    /// by the heading, then moved by the position.
    pub fn apply(&self, point: [f64; 2]) -> [f64; 2] {
        let (sin, cos) = self.yaw.sin_cos();
        [
            self.x + cos * point[0] - sin * point[1],
            self.y + sin * point[0] + cos * point[1],
        ]
    }
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

    /// The pose turned by `angle_rad` about the unit axis `axis`.
    fn turned_about(axis: [f64; 3], angle_rad: f64) -> Pose {
        let (sin, cos) = (angle_rad / 2.0).sin_cos();
        Pose::new(
            [0.4, -0.5, 0.6],
            [axis[0] * sin, axis[1] * sin, axis[2] * sin, cos],
        )
        .expect("a unit quaternion")
    }

    // The quaternion a pose gives back makes the same pose, by either branch of
    // tf2's extraction: a turn whose trace lies above 0, and a half turn, whose
    // trace lies below.
    #[test]
    fn a_pose_gives_back_its_orientation() {
        let half_turn = Pose::new([0.0; 3], [1.0, 0.0, 0.0, 0.0]).expect("a half turn about +X");
        for pose in [tilted(), half_turn] {
            let again = Pose::new(pose.origin(), pose.orientation()).expect("a unit quaternion");
            let point = [0.3, -1.2, 0.8];
            assert!(close(again.apply(point), pose.apply(point), 1e-12));
        }
        assert_eq!(half_turn.orientation(), [1.0, 0.0, 0.0, 0.0]);
    }

    // A turn of 120 degrees or more has a trace of at most 0, so tf2 reads its
    // quaternion from its largest diagonal entry: the entry of +Z for a heading
    // past 120 degrees either way, of +X and +Y for turns about them, and of
    // +Z, reached past the other two, for a turn about a general axis. Each
    // quaternion makes the same pose again.
    #[test]
    fn a_turn_past_120_degrees_gives_back_its_orientation() {
        let general = {
            let norm = (0.2_f64 * 0.2 + 0.5 * 0.5 + 0.84 * 0.84).sqrt();
            [0.2 / norm, 0.5 / norm, 0.84 / norm]
        };
        let turns = [
            ([0.0, 0.0, 1.0], 2.2),
            ([0.0, 0.0, 1.0], 2.6),
            ([0.0, 0.0, 1.0], 3.0),
            ([0.0, 0.0, 1.0], -2.5),
            ([1.0, 0.0, 0.0], 2.6),
            ([0.0, 1.0, 0.0], 2.6),
            (general, 2.6),
        ];
        let point = [0.3, -1.2, 0.8];
        for (axis, angle_rad) in turns {
            let pose = turned_about(axis, angle_rad);
            let again = Pose::new(pose.origin(), pose.orientation()).expect("a unit quaternion");
            assert!(
                close(again.apply(point), pose.apply(point), 1e-12),
                "{angle_rad} rad about {axis:?}"
            );
        }
    }

    // An interpolated pose runs from the first pose at 0 to the second at 1,
    // its origin along the line between them and its heading turning by the
    // share of the turn between them, the shorter way round whichever sign the
    // second quaternion carries.
    #[test]
    fn an_interpolated_pose_runs_from_one_pose_to_the_other() {
        let start = Pose::new([1.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let quarter = std::f64::consts::FRAC_PI_4;
        let turned = [0.0, 0.0, quarter.sin(), quarter.cos()];
        for orientation in [turned, turned.map(|c| -c)] {
            let end = Pose::new([3.0, -2.0, 0.5], orientation).expect("a quarter turn about +Z");
            let point = [1.0, 0.0, 0.0];
            assert!(close(
                start.interpolated(&end, 0.0).apply(point),
                start.apply(point),
                1e-12
            ));
            assert!(close(
                start.interpolated(&end, 1.0).apply(point),
                end.apply(point),
                1e-12
            ));
            let middle = start.interpolated(&end, 0.5);
            let half_way = std::f64::consts::FRAC_PI_4;
            assert!(close(
                middle.apply(point),
                [2.0 + half_way.cos(), -1.0 + half_way.sin(), 0.5],
                1e-12
            ));
        }
    }

    // A heading comes back from its quaternion to rounding on the whole
    // circle; a heading of -0 comes back as +0, as the x and y that tf2's
    // roll and pitch of 0 make +0 add +0 to the -0 of w z; and a quaternion
    // of a pitch of a quarter turn, either way, reads its heading from its x
    // and y.
    #[test]
    fn a_heading_reads_back_from_its_quaternion_as_tf2_reads_it() {
        use std::f64::consts::{FRAC_PI_4, PI};
        for step in -100..=100 {
            let yaw = f64::from(step) * PI / 100.0;
            let back = yaw_of_quaternion(quaternion_of_yaw(yaw));
            assert!((back - yaw).abs() < 1e-14, "{yaw}: {back}");
        }
        let minus_zero = quaternion_of_yaw(-0.0);
        assert_eq!(minus_zero[2].to_bits(), (-0.0_f64).to_bits());
        assert_eq!(yaw_of_quaternion(minus_zero).to_bits(), 0.0_f64.to_bits());
        let (sin_pitch, cos_pitch) = FRAC_PI_4.sin_cos();
        let (sin_yaw, cos_yaw) = 0.2_f64.sin_cos();
        let pitched = |sign: f64| {
            [
                -sign * sin_pitch * sin_yaw,
                sign * sin_pitch * cos_yaw,
                cos_pitch * sin_yaw,
                cos_pitch * cos_yaw,
            ]
        };
        let (up, down) = (pitched(1.0), pitched(-1.0));
        assert_eq!(yaw_of_quaternion(up), 2.0 * up[1].atan2(up[0]));
        assert_eq!(yaw_of_quaternion(down), -2.0 * down[1].atan2(down[0]));
    }

    // A planar pose turns a point by its heading, then moves it.
    #[test]
    fn a_planar_pose_turns_a_point_then_moves_it() {
        let quarter = PlanarPose {
            x: 1.0,
            y: -2.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let [x, y] = quarter.apply([3.0, 0.5]);
        assert!(
            (x - 0.5).abs() < 1e-12 && (y - 1.0).abs() < 1e-12,
            "{x} {y}"
        );
        assert_eq!(PlanarPose::IDENTITY.apply([0.3, -0.7]), [0.3, -0.7]);
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
