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
//! drives in, seen from another planar frame with its heading read as Nav2's
//! controller reads a plan's pose in its base's frame, through tf2's lookup,
//! KDL's rotation and tf2's `getYaw`, in [-pi, pi].

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
    /// the orientations by tf2's `Quaternion::slerp`, the shorter way round. A
    /// ratio that is not finite, or one that carries the origin past the
    /// largest double, makes no pose: the error gives the position and the
    /// orientation.
    pub fn interpolated(&self, to: &Pose, ratio: f64) -> Result<Pose, PoseError> {
        let origin = frame_tree::interpolated(self.origin, to.origin, ratio);
        let orientation = frame_tree::slerp(self.orientation(), to.orientation(), ratio);
        Pose::new(origin, orientation)
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
/// [`PlanarPose::new`] refuses a value that is not finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPose {
    x: f64,
    y: f64,
    yaw: f64,
}

/// A position and a heading that make no planar pose: a value that is not
/// finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPoseError {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl fmt::Display for PlanarPoseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "x {} m, y {} m and yaw {} rad make no planar pose: give finite numbers",
            self.x, self.y, self.yaw
        )
    }
}

impl std::error::Error for PlanarPoseError {}

impl PlanarPose {
    /// The pose that places a frame on the frame it is placed in.
    pub const IDENTITY: PlanarPose = PlanarPose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    /// The pose at `x` and `y`, in meters, heading `yaw`, in radians; a value
    /// that is not finite makes no pose.
    pub fn new(x: f64, y: f64, yaw: f64) -> Result<Self, PlanarPoseError> {
        if !(x.is_finite() && y.is_finite() && yaw.is_finite()) {
            return Err(PlanarPoseError { x, y, yaw });
        }
        Ok(Self { x, y, yaw })
    }

    /// The position along the x axis, in meters.
    pub fn x(self) -> f64 {
        self.x
    }

    /// The position along the y axis, in meters.
    pub fn y(self) -> f64 {
        self.y
    }

    /// The heading, in radians, counterclockwise about +Z from +X.
    pub fn yaw(self) -> f64 {
        self.yaw
    }

    /// The pose in space: the position on the floor plane, at height 0, and
    /// the heading as a turn about +Z, its quaternion made as tf2's
    /// `setRPY(0, 0, yaw)` makes it ([`quaternion_of_yaw`]).
    pub fn to_pose(self) -> Pose {
        Pose::new([self.x, self.y, 0.0], quaternion_of_yaw(self.yaw))
            .expect("a finite position and the quaternion of a finite heading make a pose")
    }

    /// A point of the placed frame in the frame the pose places it in: turned
    /// by the heading, then moved by the position.
    pub fn apply(&self, point: [f64; 2]) -> [f64; 2] {
        let (sin, cos) = self.yaw.sin_cos();
        [
            self.x + cos * point[0] - sin * point[1],
            self.y + sin * point[0] + cos * point[1],
        ]
    }

    /// A point of the frame the pose places a frame in, in the placed frame:
    /// moved back by the position, then turned back by the heading, as
    /// [`PlanarPose::apply`] undone.
    pub fn apply_inverse(&self, point: [f64; 2]) -> [f64; 2] {
        let (sin, cos) = self.yaw.sin_cos();
        let dx = point[0] - self.x;
        let dy = point[1] - self.y;
        [cos * dx + sin * dy, -sin * dx + cos * dy]
    }

    /// This pose in the frame that `frame` places: its position turned back
    /// by `frame`'s heading about `frame`'s position
    /// ([`PlanarPose::apply_inverse`]), and its heading as Nav2's controller
    /// reads a plan's pose in its base's frame, through tf2's lookup of the
    /// base's frame, KDL's rotation of the pose's quaternion by it and tf2's
    /// `getYaw`, in [-pi, pi]. A pose so far from `frame` that its position
    /// there is not finite makes no pose.
    pub fn seen_from(self, frame: PlanarPose) -> Result<PlanarPose, PlanarPoseError> {
        let [x, y] = frame.apply_inverse([self.x, self.y]);
        PlanarPose::new(x, y, heading_seen_from(self.yaw, frame.yaw))
    }
}

/// The identity quaternion [x, y, z, w].
const IDENTITY_QUATERNION: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

/// The trace above which KDL's `GetQuaternion` takes a rotation's quaternion
/// from its trace (orocos_kdl 1.5.2 src/frames.cpp:207-208).
const KDL_TRACE_EPSILON: f64 = 1e-12;

/// The heading `yaw` reads as from a frame of heading `frame_yaw`, as Nav2's
/// controller reads a plan's pose in its base's frame: nav2_util's
/// `transformPathInTargetFrame` looks up the base's frame from the plan's and
/// turns each pose by it with `tf2::doTransform` (nav2_util
/// src/path_utils.cpp:108-128, nav2_regulated_pure_pursuit_controller
/// src/regulated_pure_pursuit_controller.cpp:175-177), and the controller
/// reads the heading with `tf2::getYaw` (:227). Each step runs in its
/// library's arithmetic, each quaternion tf2's `setRPY(0, 0, yaw)`:
///
/// - tf2 looks up a base's frame that hangs from the plan's frame as the base
///   quaternion times the identity, inverted, times the identity, where the
///   plan's frame is the root of the tree and tf2's walk ends `FullPath`
///   (tf2 src/buffer_core.cpp:495, 547-556, 575-583;
///   include/tf2/LinearMath/Quaternion.hpp:282-285, 429-433). Where the
///   plan's frame has a parent, as odom under map in Nav2's usual tree, the
///   walk ends `SourceParentOfTarget` at the plan's frame and leaves out the
///   last product by the identity (buffer_core.cpp:456-458, 567-573). That
///   product changes only the signs of the lookup's zero components, and the
///   two lookups give one heading for each of the 2,300 bases and goals of
///   the test `a_plan_frame_with_a_parent_reads_the_heading_of_one_at_the_root`:
///   20 bases, every eighth of a half turn and 2, -2 and 1 rad, each with the
///   goals within 3 doubles of a half turn either way and every fiftieth of a
///   half turn;
/// - `doTransform` multiplies KDL's rotation matrices of that quaternion and
///   of the pose's, and reads the product back as a quaternion
///   (tf2_geometry_msgs include/tf2_geometry_msgs/tf2_geometry_msgs.hpp:72-81,
///   408-427; orocos_kdl 1.5.2 src/frames.cpp:173-235);
/// - `getYaw` reads its heading.
///
/// The rounding of these steps, not the difference of the headings, decides
/// the sign of a half turn.
fn heading_seen_from(yaw: f64, frame_yaw: f64) -> f64 {
    let frame_to_top = tf2_product(quaternion_of_yaw(frame_yaw), IDENTITY_QUATERNION);
    let lookup = tf2_product(tf2_inverse(frame_to_top), IDENTITY_QUATERNION);
    heading_turned_by(lookup, yaw)
}

/// The heading `yaw` reads once `doTransform` turns it by tf2's `lookup`
/// quaternion through KDL's rotation matrices, as `getYaw` reads it.
fn heading_turned_by(lookup: [f64; 4], yaw: f64) -> f64 {
    let turned = kdl_product(kdl_rotation(lookup), kdl_rotation(quaternion_of_yaw(yaw)));
    yaw_of_quaternion(kdl_quaternion(turned))
}

/// tf2's product of two quaternions [x, y, z, w] (tf2
/// include/tf2/LinearMath/Quaternion.hpp:429-433).
fn tf2_product([ax, ay, az, aw]: [f64; 4], [bx, by, bz, bw]: [f64; 4]) -> [f64; 4] {
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by + ay * bw + az * bx - ax * bz,
        aw * bz + az * bw + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// tf2's inverse of a quaternion [x, y, z, w] (tf2
/// include/tf2/LinearMath/Quaternion.hpp:282-285).
fn tf2_inverse([x, y, z, w]: [f64; 4]) -> [f64; 4] {
    [-x, -y, -z, w]
}

/// KDL's rotation matrix of a quaternion [x, y, z, w], row by row
/// (`Rotation::Quaternion`, orocos_kdl 1.5.2 src/frames.cpp:190-197).
fn kdl_rotation([x, y, z, w]: [f64; 4]) -> [f64; 9] {
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

/// KDL's product of two rotation matrices, row by row (orocos_kdl 1.5.2
/// src/frames.cpp:173-187).
fn kdl_product(lhs: [f64; 9], rhs: [f64; 9]) -> [f64; 9] {
    std::array::from_fn(|index| {
        let (row, column) = (index / 3 * 3, index % 3);
        lhs[row] * rhs[column] + lhs[row + 1] * rhs[column + 3] + lhs[row + 2] * rhs[column + 6]
    })
}

/// KDL's `Rotation::GetQuaternion` of a turn about +Z, [x, y, z, w]
/// (orocos_kdl 1.5.2 src/frames.cpp:204-235). Below its trace epsilon KDL
/// picks the branch of the largest diagonal entry; for a turn about +Z the
/// third entry is 1 to rounding and the first two are the turn's cosine, at
/// most about -0.5 there, so the third entry's branch runs and KDL's other
/// two are left out.
fn kdl_quaternion(m: [f64; 9]) -> [f64; 4] {
    let trace = m[0] + m[4] + m[8];
    if trace > KDL_TRACE_EPSILON {
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

/// `angle` brought into (-pi, pi], as the angles library's `normalize_angle`
/// brings it (angles/angles.h).
pub fn normalize_angle(angle: f64) -> f64 {
    let result = (angle + std::f64::consts::PI) % std::f64::consts::TAU;
    if result <= 0.0 {
        result + std::f64::consts::PI
    } else {
        result - std::f64::consts::PI
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
                start.interpolated(&end, 0.0).unwrap().apply(point),
                start.apply(point),
                1e-12
            ));
            assert!(close(
                start.interpolated(&end, 1.0).unwrap().apply(point),
                end.apply(point),
                1e-12
            ));
            let middle = start.interpolated(&end, 0.5).unwrap();
            let half_way = std::f64::consts::FRAC_PI_4;
            assert!(close(
                middle.apply(point),
                [2.0 + half_way.cos(), -1.0 + half_way.sin(), 0.5],
                1e-12
            ));
        }
    }

    // A ratio that is not finite, and one that carries the origin past the
    // largest double, interpolate to no pose.
    #[test]
    fn an_interpolation_to_no_pose_is_refused() {
        let start = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let far = Pose::new([f64::MAX / 1.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        for ratio in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(start.interpolated(&far, ratio).is_err(), "{ratio}");
        }
        assert!(start.interpolated(&far, 2.0).is_err());
        assert!(start.interpolated(&far, 1.0).is_ok());
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

    fn planar(x: f64, y: f64, yaw: f64) -> PlanarPose {
        PlanarPose::new(x, y, yaw).expect("a finite pose")
    }

    // A planar pose takes finite numbers and gives them back; a value that is
    // not finite makes none, the refusal naming each.
    #[test]
    fn a_planar_pose_wants_finite_numbers() {
        let pose = planar(1.0, -2.0, 0.5);
        assert_eq!((pose.x(), pose.y(), pose.yaw()), (1.0, -2.0, 0.5));
        for (x, y, yaw) in [
            (f64::NAN, 0.0, 0.0),
            (0.0, f64::INFINITY, 0.0),
            (0.0, 0.0, f64::NEG_INFINITY),
        ] {
            let refused = PlanarPose::new(x, y, yaw).unwrap_err();
            assert_eq!(
                (
                    refused.x.to_bits(),
                    refused.y.to_bits(),
                    refused.yaw.to_bits()
                ),
                (x.to_bits(), y.to_bits(), yaw.to_bits())
            );
        }
        assert_eq!(
            PlanarPose::new(f64::NAN, 0.0, 0.0).unwrap_err().to_string(),
            "x NaN m, y 0 m and yaw 0 rad make no planar pose: give finite numbers"
        );
    }

    // A planar pose in space stands on the floor plane, turned about +Z by
    // its heading as tf2's setRPY makes the turn.
    #[test]
    fn a_planar_pose_in_space_stands_on_the_floor_turned_about_z() {
        let pose = planar(1.0, -2.0, 2.5).to_pose();
        assert_eq!(pose.origin(), [1.0, -2.0, 0.0]);
        assert_eq!(pose.orientation(), quaternion_of_yaw(2.5));
        let [x, y] = planar(1.0, -2.0, 2.5).apply([0.3, 0.4]);
        assert!(close(pose.apply([0.3, 0.4, 0.7]), [x, y, 0.7], 1e-12));
    }

    // A planar pose turns a point by its heading, then moves it.
    #[test]
    fn a_planar_pose_turns_a_point_then_moves_it() {
        let quarter = planar(1.0, -2.0, std::f64::consts::FRAC_PI_2);
        let [x, y] = quarter.apply([3.0, 0.5]);
        assert!(
            (x - 0.5).abs() < 1e-12 && (y - 1.0).abs() < 1e-12,
            "{x} {y}"
        );
        assert_eq!(PlanarPose::IDENTITY.apply([0.3, -0.7]), [0.3, -0.7]);
    }

    // A point and a pose seen from the frame that places them come back to
    // where the frame took them from; a pose whose position seen from a frame
    // passes the largest double makes no pose, naming that position.
    #[test]
    fn a_pose_seen_from_a_frame_undoes_the_frame() {
        let far = planar(1e308, 0.0, 0.0).seen_from(planar(-1e308, 0.0, 0.0));
        assert!(
            matches!(far, Err(PlanarPoseError { x, .. }) if x == f64::INFINITY),
            "{far:?}"
        );
        let frame = planar(1.0, -2.0, 2.5);
        let [x, y] = frame.apply([0.3, 0.4]);
        let [back_x, back_y] = frame.apply_inverse([x, y]);
        assert!(
            (back_x - 0.3).abs() < 1e-12 && (back_y - 0.4).abs() < 1e-12,
            "{back_x} {back_y}"
        );
        let seen = planar(x, y, 3.0).seen_from(frame).unwrap();
        assert!(
            (seen.x() - 0.3).abs() < 1e-12 && (seen.y() - 0.4).abs() < 1e-12,
            "{seen:?}"
        );
        assert!((seen.yaw() - 0.5).abs() < 1e-15, "{seen:?}");
        let behind = planar(x, y, -3.0).seen_from(frame).unwrap();
        assert!(
            (behind.yaw() - (2.0 * std::f64::consts::PI - 5.5)).abs() < 1e-15,
            "{behind:?}"
        );
    }

    // A heading seen from another is the one Nav2's controller reads through
    // tf2 and KDL, whose rounding decides the sign of a half turn: a goal at
    // -pi seen from a base at 0 reads -pi, one at pi reads pi, and one at the
    // double above pi reads -pi; seen from a base at 2, a goal at 2 - pi reads
    // -pi and one at the double below it pi. Off a half turn it is the
    // difference wrapped into [-pi, pi], to rounding.
    #[test]
    fn a_heading_seen_from_another_is_the_one_tf2_and_kdl_give() {
        use std::f64::consts::PI;
        let seen = |goal: f64, base: f64| {
            let at = |yaw: f64| planar(0.0, 0.0, yaw);
            at(goal).seen_from(at(base)).unwrap().yaw()
        };
        assert_eq!(seen(-PI, 0.0), -PI);
        assert_eq!(seen(PI, 0.0), PI);
        assert_eq!(seen(0.0, PI), -PI);
        assert_eq!(seen(PI.next_up(), 0.0), -PI);
        assert_eq!(seen(2.0 - PI, 2.0), -PI);
        assert_eq!(seen((2.0 - PI).next_down(), 2.0), PI);
        let wrapped = seen(3.0, -0.5);
        assert!((wrapped - (3.5 - 2.0 * PI)).abs() < 1e-15, "{wrapped}");
    }

    // tf2's lookup for a plan's frame that has a parent leaves out the last
    // product by the identity, and gives the heading of the lookup for a plan's
    // frame at the root: for goals a half turn from a base, within 3 doubles,
    // and for goals around the circle, from bases around the circle.
    #[test]
    fn a_plan_frame_with_a_parent_reads_the_heading_of_one_at_the_root() {
        use std::f64::consts::PI;
        let with_a_parent = |yaw: f64, frame_yaw: f64| {
            let frame_to_parent = tf2_product(quaternion_of_yaw(frame_yaw), IDENTITY_QUATERNION);
            heading_turned_by(tf2_inverse(frame_to_parent), yaw)
        };
        let within_3_doubles = |value: f64| {
            let lowest = (0..3).fold(value, |below, _| below.next_down());
            std::iter::successors(Some(lowest), |above| Some(above.next_up())).take(7)
        };
        let bases = (-8..=8)
            .map(|k| f64::from(k) * PI / 8.0)
            .chain([2.0, -2.0, 1.0]);
        for base in bases {
            let goals = within_3_doubles(base + PI)
                .chain(within_3_doubles(base - PI))
                .chain((-50..=50).map(|k| f64::from(k) * PI / 50.0));
            for goal in goals {
                assert_eq!(
                    with_a_parent(goal, base),
                    heading_seen_from(goal, base),
                    "goal {goal}, base {base}"
                );
            }
        }
    }

    // An angle comes into (-pi, pi], pi itself staying.
    #[test]
    fn an_angle_normalizes_into_the_half_open_circle() {
        use std::f64::consts::PI;
        assert_eq!(normalize_angle(PI), PI);
        assert_eq!(normalize_angle(-PI), PI);
        assert!((normalize_angle(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-15);
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
