//! Where one frame sits in another on the floor plane: a position and a heading, the pose of a
//! mobile base and of the frames it drives in, composed with another as tf2 composes their
//! transforms, and seen from another planar frame with its heading read as Nav2's controller
//! reads a plan's pose in its base's frame, through tf2's lookup, KDL's rotation and tf2's
//! `getYaw`, in [-pi, pi], each in frame_tree's arithmetic.

use std::fmt;

use frame_tree::arithmetic::{self, kdl, normalize_angle, quaternion_of_yaw, yaw_of_quaternion};
use frame_tree::{Transform, UnitQuaternion};

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

    /// The distance, in meters, from this pose's position to `target`'s, as
    /// nav2_util's `euclidean_distance` gives it (include/nav2_util/geometry_utils.hpp).
    pub fn distance_to(self, target: PlanarPose) -> f64 {
        (self.x - target.x).hypot(self.y - target.y)
    }

    /// The turn, in radians, of least magnitude from this pose's heading to
    /// `target`'s, in (-pi, pi]: the angles library's `shortest_angular_distance`
    /// ([`normalize_angle`] of the difference).
    pub fn yaw_to(self, target: PlanarPose) -> f64 {
        normalize_angle(target.yaw - self.yaw)
    }

    /// The transform in space: the position on the floor plane, at height 0, and
    /// the heading as a turn about +Z, its quaternion made as tf2's
    /// `setRPY(0, 0, yaw)` makes it ([`UnitQuaternion::from_yaw`]).
    pub fn to_transform(self) -> Transform {
        let heading =
            UnitQuaternion::from_yaw(self.yaw).expect("a planar pose's heading is finite");
        Transform::new([self.x, self.y, 0.0], heading)
            .expect("a planar pose's position is finite, so it makes a transform")
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

    /// `inner`, a pose in the frame this pose places, in the frame this pose is placed in:
    /// the two transforms composed in tf2's arithmetic ([`Transform::compose`]), read back
    /// on the floor plane, its heading as tf2's `getYaw` reads it. A composition whose
    /// position is not finite makes no pose. [`PlanarPose::IDENTITY`] seen from a pose
    /// ([`PlanarPose::seen_from`]) undoes the pose.
    pub fn compose(self, inner: PlanarPose) -> Result<PlanarPose, PlanarPoseError> {
        let composed = self.to_transform().compose(&inner.to_transform());
        let [x, y, _] = composed.position();
        PlanarPose::new(x, y, composed.orientation().yaw())
    }
}

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
    let frame_to_top = arithmetic::product(quaternion_of_yaw(frame_yaw), arithmetic::IDENTITY);
    let lookup = arithmetic::product(arithmetic::inverse(frame_to_top), arithmetic::IDENTITY);
    heading_turned_by(lookup, yaw)
}

/// The heading `yaw` reads once `doTransform` turns it by tf2's `lookup`
/// quaternion through KDL's rotation matrices, as `getYaw` reads it.
fn heading_turned_by(lookup: [f64; 4], yaw: f64) -> f64 {
    let turned = kdl::product(kdl::rotation(lookup), kdl::rotation(quaternion_of_yaw(yaw)));
    yaw_of_quaternion(kdl::quaternion_of_turn_about_z(turned))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whether every coordinate of `a` lies within `tolerance` of `b`'s.
    fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < tolerance)
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
        let pose = planar(1.0, -2.0, 2.5).to_transform();
        assert_eq!(pose.position(), [1.0, -2.0, 0.0]);
        assert_eq!(pose.orientation().to_xyzw(), quaternion_of_yaw(2.5));
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
            let frame_to_parent =
                arithmetic::product(quaternion_of_yaw(frame_yaw), arithmetic::IDENTITY);
            heading_turned_by(arithmetic::inverse(frame_to_parent), yaw)
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

    // A pose composed with another and then with the first one undone, the identity seen
    // from it, is the other again.
    #[test]
    fn a_composed_pose_undone_is_the_inner_pose() {
        let outer = planar(1.0, -2.0, 0.7);
        let inner = planar(0.3, 0.4, -1.2);
        let undone = PlanarPose::IDENTITY.seen_from(outer).expect("a pose");
        let back = undone
            .compose(outer.compose(inner).expect("a pose"))
            .expect("a pose");
        assert!(
            close(
                [back.x, back.y, back.yaw],
                [inner.x, inner.y, inner.yaw],
                1e-12
            ),
            "{back:?}"
        );
    }

    // Two poses whose sum leaves the finite numbers compose to none, the refusal naming the
    // position; a far pose composed with the identity is itself.
    #[test]
    fn poses_composed_past_the_finite_numbers_give_none() {
        let far = planar(f64::MAX, 0.0, 0.0);
        let refused = far.compose(far).unwrap_err();
        assert_eq!((refused.x, refused.y), (f64::INFINITY, 0.0));
        assert_eq!(far.compose(PlanarPose::IDENTITY), Ok(far));
    }

    // The distance between two poses is that of their positions, and the
    // turn from one heading to another is the shorter one, crossing the cut at
    // pi when that is shorter.
    #[test]
    fn a_pose_gives_the_distance_and_the_shorter_turn_to_another() {
        use std::f64::consts::PI;
        let (from, to) = (planar(1.0, 1.0, 3.0), planar(4.0, 5.0, -3.0));
        assert_eq!(from.distance_to(to), 5.0);
        assert_eq!(to.distance_to(from), 5.0);
        assert!((from.yaw_to(to) - (2.0 * PI - 6.0)).abs() < 1e-15);
        assert_eq!(planar(0.0, 0.0, 0.25).yaw_to(planar(0.0, 0.0, 0.75)), 0.5);
    }
}
