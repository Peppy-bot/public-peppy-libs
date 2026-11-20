//! The geometry types of the contracts, as contracts-hub's README defines them: a point, a
//! pose, whose orientation is frame_tree's [`UnitQuaternion`], and a twist, and each stamped in a
//! frame at a time. A node's generated code holds its own struct for each field, and the node
//! converts the fields of each into these types through the one parse here, which refuses what no
//! contract takes: a number that is not finite, an orientation whose squared norm lies
//! [`frame_tree::QUATERNION_NORMALIZATION_TOLERANCE`] or more from 1, an empty frame, and a
//! timestamp a stamp does not hold. A command with no timestamp is placed
//! at the latest time, which tf2 names `TimePointZero`, and a given timestamp is a time like any
//! other, 0 the Unix epoch; a target with no orientation gives none ([`PoseTarget`]). A
//! planar member reads a pose or a twist on the floor plane through [`Pose::planar`] and
//! [`Twist::planar`], which refuse a part off the plane by more than [`PLANAR_TOLERANCE`]. Every
//! refusal says what to send instead, each number in it printed as Rust's `Display` prints it,
//! and control_core_py's geometry gives the same text for each refusal it also gives
//! (`tests/fixtures/geometry_refusals.tsv`).
//!
//! Behind the `nalgebra` feature a point and a pose go into nalgebra's types with `From`, as
//! frame_tree's feature takes an orientation to and from nalgebra's, and `pose_arrays` gives an
//! isometry back as the contracts' arrays.

use std::time::SystemTime;

use frame_tree::arithmetic::Vector;
use frame_tree::{
    FrameName, LookupTime, NotFinite, OrientationError, Placement, Stamp, Transform,
    UnitQuaternion, finite_position,
};

use crate::planar_pose::PlanarPose;
use crate::twist::PlanarTwist;

/// The one frame of a simulation: every report of a simulation contract gives its values in it,
/// and a command names it or leaves its frame out.
pub const WORLD: &str = "world";

/// How far a planar pose's height and tilt, and a planar twist's vertical and tilting parts,
/// may lie from 0, in m, rad, m/s and rad/s: base_motion's bound on a planar target and a
/// planar velocity.
pub const PLANAR_TOLERANCE: f64 = 1e-9;

/// A position [x, y, z] in m, every coordinate finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point(Vector);

impl Point {
    pub fn parse(position: [f64; 3]) -> Result<Self, GeometryError> {
        Ok(Self(finite_position(position)?))
    }

    pub fn position(self) -> [f64; 3] {
        self.0
    }
}

/// A pose: a position and an orientation, as a report gives one and a goal that needs an
/// orientation takes one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    position: Point,
    orientation: UnitQuaternion,
}

impl Pose {
    pub fn new(position: Point, orientation: UnitQuaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn parse(position: [f64; 3], orientation: [f64; 4]) -> Result<Self, GeometryError> {
        Ok(Self::new(
            Point::parse(position)?,
            UnitQuaternion::parse(orientation)?,
        ))
    }

    pub fn position(self) -> Point {
        self.position
    }

    pub fn orientation(self) -> UnitQuaternion {
        self.orientation
    }

    /// The pose as tf2's transform, its matrix built as tf2's `setRotation` builds it.
    pub fn transform(self) -> Transform {
        Transform::new(self.position.position(), self.orientation)
            .expect("a parsed point is finite, so it makes a transform")
    }

    /// The pose on the floor plane: its position's x and y and its heading as tf2's `getYaw`
    /// reads it; refused when its height or its tilt lies farther than 1e-9 m or rad from 0.
    pub fn planar(self) -> Result<PlanarPose, GeometryError> {
        let [x, y, z] = self.position.position();
        if z.abs() > PLANAR_TOLERANCE {
            return Err(GeometryError::OffFloor { z });
        }
        let tilt_rad = self.orientation.tilt();
        if tilt_rad > PLANAR_TOLERANCE {
            return Err(GeometryError::Tilted {
                orientation: self.orientation,
                tilt_rad,
            });
        }
        Ok(PlanarPose::new(x, y, self.orientation.yaw())
            .expect("the finite x and y of a parsed point and the heading of a unit quaternion"))
    }
}

/// The target of a goal: a position, and an orientation or none, as a command leaves out an
/// orientation it does not mean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseTarget {
    position: Point,
    orientation: Option<UnitQuaternion>,
}

impl PoseTarget {
    /// The target of `position` and `orientation`, a unit quaternion or none.
    pub fn parse(position: [f64; 3], orientation: Option<[f64; 4]>) -> Result<Self, GeometryError> {
        Ok(Self {
            position: Point::parse(position)?,
            orientation: orientation.map(UnitQuaternion::parse).transpose()?,
        })
    }

    pub fn position(self) -> Point {
        self.position
    }

    pub fn orientation(self) -> Option<UnitQuaternion> {
        self.orientation
    }

    /// The pose of a target that gives an orientation; a member that needs one refuses the
    /// target without one.
    pub fn pose(self) -> Result<Pose, GeometryError> {
        let orientation = self.orientation.ok_or(GeometryError::NoOrientation)?;
        Ok(Pose::new(self.position, orientation))
    }
}

/// A twist: the velocity of a moving frame's origin, linear_velocity [x, y, z] in m/s and
/// angular_velocity about [x, y, z] in rad/s, every number finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist {
    linear_velocity: Vector,
    angular_velocity: Vector,
}

impl Twist {
    pub fn parse(
        linear_velocity: [f64; 3],
        angular_velocity: [f64; 3],
    ) -> Result<Self, GeometryError> {
        let ([lx, ly, lz], [ax, ay, az]) = (linear_velocity, angular_velocity);
        NotFinite::check(
            "a twist",
            [
                ("linear_velocity x", lx),
                ("linear_velocity y", ly),
                ("linear_velocity z", lz),
                ("angular_velocity x", ax),
                ("angular_velocity y", ay),
                ("angular_velocity z", az),
            ],
            "finite numbers, m/s along x, y and z and rad/s about them",
        )?;
        Ok(Self {
            linear_velocity,
            angular_velocity,
        })
    }

    pub fn linear_velocity(self) -> [f64; 3] {
        self.linear_velocity
    }

    pub fn angular_velocity(self) -> [f64; 3] {
        self.angular_velocity
    }

    /// The twist on the floor plane: linear_velocity's x and y and angular_velocity's z;
    /// refused when linear_velocity's z or angular_velocity's x or y lies farther than 1e-9 m/s
    /// or rad/s from 0.
    pub fn planar(self) -> Result<PlanarTwist, GeometryError> {
        let ([lx, ly, lz], [ax, ay, az]) = (self.linear_velocity, self.angular_velocity);
        if lz.abs() > PLANAR_TOLERANCE {
            return Err(GeometryError::Vertical { z: lz });
        }
        if ax.abs() > PLANAR_TOLERANCE || ay.abs() > PLANAR_TOLERANCE {
            return Err(GeometryError::TiltingTurn { x: ax, y: ay });
        }
        Ok(PlanarTwist::new(lx, ly, az).expect("the finite numbers of a parsed twist"))
    }
}

/// A value of the geometry types that the placement of its frame in another carries into that
/// other frame: a position through tf2's transform, an orientation turned by the placement's,
/// and a twist's two vectors turned alone.
pub trait Placed: Sized {
    /// The value in the frame `placement` places its frame in; refused, as the value's parse
    /// refuses a number that is not finite, where a coordinate it reaches leaves the finite
    /// numbers.
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError>;
}

impl Placed for Point {
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError> {
        Point::parse(placement.transform().apply(self.0))
    }
}

impl Placed for Pose {
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError> {
        Ok(Pose::new(
            self.position.placed_by(placement)?,
            placement.orientation().compose(self.orientation),
        ))
    }
}

impl Placed for PoseTarget {
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError> {
        Ok(Self {
            position: self.position.placed_by(placement)?,
            orientation: self
                .orientation
                .map(|orientation| placement.orientation().compose(orientation)),
        })
    }
}

impl Placed for Twist {
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError> {
        let orientation = placement.orientation();
        Twist::parse(
            orientation.rotate(self.linear_velocity),
            orientation.rotate(self.angular_velocity),
        )
    }
}

/// Values of one frame at one time, each placed by the one placement, so one answer of a tree
/// places all of them; refused whole where one is refused.
impl<T: Placed> Placed for Vec<T> {
    fn placed_by(&self, placement: &Placement) -> Result<Self, GeometryError> {
        self.iter()
            .map(|value| value.placed_by(placement))
            .collect()
    }
}

/// A value of the geometry types in a frame at a time: a stamped type of the contracts.
#[derive(Debug, Clone, PartialEq)]
pub struct Stamped<T> {
    frame: FrameName,
    time: LookupTime,
    value: T,
}

impl<T> Stamped<T> {
    /// `value` in the frame named `frame` at `timestamp`, or at the latest time for none.
    pub fn parse(
        frame: &str,
        timestamp: Option<SystemTime>,
        value: T,
    ) -> Result<Self, GeometryError> {
        let frame = FrameName::parse(frame).map_err(|_| GeometryError::NoFrame)?;
        Ok(Self {
            frame,
            time: lookup_time(timestamp)?,
            value,
        })
    }

    /// `value` in `frame` at the latest time, as a lookup places a value in a frame that holds
    /// still.
    pub(crate) fn at_latest(frame: FrameName, value: T) -> Self {
        Self {
            frame,
            time: LookupTime::Latest,
            value,
        }
    }

    pub fn frame(&self) -> &FrameName {
        &self.frame
    }

    pub fn time(&self) -> LookupTime {
        self.time
    }

    pub fn value(&self) -> &T {
        &self.value
    }
}

/// The time of a lookup that `timestamp` names: the latest time for none, as tf2 names it
/// `TimePointZero`, and the time given otherwise, the Unix epoch among them; refused for a time a
/// [`Stamp`] does not hold.
pub fn lookup_time(timestamp: Option<SystemTime>) -> Result<LookupTime, GeometryError> {
    let Some(time) = timestamp else {
        return Ok(LookupTime::Latest);
    };
    let stamp = Stamp::from_time(time).map_err(|_| GeometryError::Time {
        seconds: seconds_from_epoch(time),
    })?;
    Ok(LookupTime::At(stamp))
}

/// The seconds from 1970-01-01T00:00:00Z to `time`, below 0 for a time before it.
fn seconds_from_epoch(time: SystemTime) -> f64 {
    match time.duration_since(SystemTime::UNIX_EPOCH) {
        Ok(after) => after.as_secs_f64(),
        Err(before) => -before.duration().as_secs_f64(),
    }
}

/// The timestamp a report gives for a value at `time`, where the report leaves it out for a value
/// that holds at every time, as the answer of `lookup_transform` and a result of `check_positions`
/// do: none for such a value.
pub fn report_timestamp(time: Option<Stamp>) -> Option<SystemTime> {
    time.map(Stamp::time)
}

/// What no contract takes, each refused with `rejected`, the message saying what to send.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum GeometryError {
    /// A position or a velocity that is not finite, the refusal naming each value and what to
    /// give.
    #[error("{0}")]
    NotFinite(String),
    #[error(transparent)]
    Orientation(#[from] OrientationError),
    #[error("the target gives no orientation: send a unit quaternion [x, y, z, w]")]
    NoOrientation,
    #[error(
        "the frame is empty: name a frame of the robot's tree, such as the robot frame that \
         sensor_mounts' describe_sensors names"
    )]
    NoFrame,
    /// A timestamp `seconds` from the Unix epoch, before it or past the last time a [`Stamp`]
    /// holds.
    #[error(
        "the timestamp {seconds} s from 1970-01-01T00:00:00Z lies outside the times a stamp \
         holds: send one from 0 s to {} after it",
        Stamp::from_nanos(u64::MAX)
    )]
    Time { seconds: f64 },
    #[error(
        "the position lies {z} m above the floor plane: send a position [x, y, 0] on the floor"
    )]
    OffFloor { z: f64 },
    #[error(
        "the orientation {orientation} tilts {tilt_rad} rad off the vertical: send a turn about \
         +z alone, [0, 0, sin(yaw/2), cos(yaw/2)]"
    )]
    Tilted {
        orientation: UnitQuaternion,
        tilt_rad: f64,
    },
    #[error("the linear velocity moves {z} m/s along z: send [vx, vy, 0]")]
    Vertical { z: f64 },
    #[error("the angular velocity turns {x} rad/s about x and {y} rad/s about y: send [0, 0, wz]")]
    TiltingTurn { x: f64, y: f64 },
}

impl<const N: usize> From<NotFinite<N>> for GeometryError {
    fn from(refusal: NotFinite<N>) -> Self {
        Self::NotFinite(refusal.to_string())
    }
}

#[cfg(feature = "nalgebra")]
impl From<Point> for nalgebra::Point3<f64> {
    fn from(point: Point) -> Self {
        Self::from(point.0)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Point> for nalgebra::Vector3<f64> {
    fn from(point: Point) -> Self {
        Self::from(point.0)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Point> for nalgebra::Translation3<f64> {
    fn from(point: Point) -> Self {
        Self::from(point.0)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Pose> for nalgebra::Isometry3<f64> {
    fn from(pose: Pose) -> Self {
        Self::from_parts(pose.position.into(), pose.orientation.into())
    }
}

/// The position of `pose` and its orientation [x, y, z, w], as the contracts give them.
#[cfg(feature = "nalgebra")]
pub fn pose_arrays(pose: &nalgebra::Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    (
        pose.translation.vector.into(),
        pose.rotation.quaternion().coords.into(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;
    use std::time::Duration;

    fn about_z(yaw: f64) -> [f64; 4] {
        [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()]
    }

    /// The refusals control_core_py's geometry gives in the same words: a call, its arguments
    /// and its message on each line, tab-separated.
    const REFUSALS: &str = include_str!("../tests/fixtures/geometry_refusals.tsv");

    /// The numbers of a field of the table, space-separated.
    fn numbers(field: &str) -> Vec<f64> {
        field
            .split_whitespace()
            .map(|number| number.parse().expect("a number"))
            .collect()
    }

    fn array<const N: usize>(field: &str) -> [f64; N] {
        numbers(field)
            .try_into()
            .expect("as many numbers as the call takes")
    }

    /// The numbers of a field of the table, or none for an empty field.
    fn optional_array<const N: usize>(field: &str) -> Option<[f64; N]> {
        (!field.is_empty()).then(|| array(field))
    }

    /// The time `seconds` from the Unix epoch, before it for a negative count, or none for an
    /// empty field.
    fn time_of(seconds: &str) -> Option<SystemTime> {
        if seconds.is_empty() {
            return None;
        }
        let seconds: f64 = seconds.parse().expect("a number of seconds");
        let span = Duration::from_secs_f64(seconds.abs());
        Some(if seconds < 0.0 {
            SystemTime::UNIX_EPOCH - span
        } else {
            SystemTime::UNIX_EPOCH + span
        })
    }

    /// The refusal of the call a line of the table names, on its arguments.
    fn refusal(call: &str, arguments: &[&str]) -> GeometryError {
        match (call, arguments) {
            ("lookup_time", [seconds]) => lookup_time(time_of(seconds)).unwrap_err(),
            ("Point.parse", [position]) => Point::parse(array(position)).unwrap_err(),
            ("UnitQuaternion.parse", [orientation]) => {
                GeometryError::from(UnitQuaternion::parse(array(orientation)).unwrap_err())
            }
            ("PoseTarget.parse", [position, orientation]) => {
                PoseTarget::parse(array(position), optional_array(orientation)).unwrap_err()
            }
            ("PoseTarget.pose", [position, orientation]) => {
                PoseTarget::parse(array(position), optional_array(orientation))
                    .unwrap()
                    .pose()
                    .unwrap_err()
            }
            ("Pose.planar", [position, orientation]) => {
                Pose::parse(array(position), array(orientation))
                    .unwrap()
                    .planar()
                    .unwrap_err()
            }
            ("Stamped.parse", [frame, seconds]) => {
                Stamped::parse(frame, time_of(seconds), ()).unwrap_err()
            }
            _ => panic!(
                "the table names no call {call} of {} arguments",
                arguments.len()
            ),
        }
    }

    // Each refusal of the shared table gives its message byte for byte, as control_core_py's
    // geometry gives it.
    #[test]
    fn each_refusal_of_the_shared_table_gives_its_message() {
        let cases = REFUSALS
            .lines()
            .filter(|line| !line.is_empty() && !line.starts_with('#'));
        for line in cases {
            let fields: Vec<&str> = line.split('\t').collect();
            let [call, arguments @ .., message] = fields.as_slice() else {
                panic!("a call and a message: {line}");
            };
            assert_eq!(refusal(call, arguments).to_string(), *message, "{line}");
        }
    }

    // A target with no orientation gives none, and a member that needs one refuses it; a given
    // orientation is parsed as a unit quaternion.
    #[test]
    fn a_target_takes_an_orientation_or_none() {
        let none = PoseTarget::parse([1.0, 2.0, 0.0], None).unwrap();
        assert_eq!(none.orientation(), None);
        assert_eq!(none.pose(), Err(GeometryError::NoOrientation));
        let some = PoseTarget::parse([1.0, 2.0, 0.0], Some([0.0, 0.0, 0.0, 1.0])).unwrap();
        assert_eq!(some.orientation(), Some(UnitQuaternion::IDENTITY));
        assert!(matches!(
            PoseTarget::parse([1.0, 2.0, 0.0], Some([0.0; 4])),
            Err(GeometryError::Orientation(OrientationError::NotUnit { .. }))
        ));
        assert!(matches!(
            PoseTarget::parse([f64::INFINITY, 0.0, 0.0], None),
            Err(GeometryError::NotFinite(_))
        ));
    }

    // No timestamp is the latest time, the Unix epoch is a time like any other, and a frame is
    // not empty; a report that leaves its timestamp out for a value that holds at every time
    // gives none for it.
    #[test]
    fn no_timestamp_is_the_latest_time_and_the_epoch_is_a_time() {
        let latest = Stamped::parse("odom", None, ()).unwrap();
        assert_eq!(latest.time(), LookupTime::Latest);
        assert_eq!(latest.frame().as_str(), "odom");
        let epoch = Stamped::parse("odom", Some(SystemTime::UNIX_EPOCH), ()).unwrap();
        assert_eq!(epoch.time(), LookupTime::At(Stamp::from_nanos(0)));
        let at = SystemTime::UNIX_EPOCH + Duration::from_secs(12);
        let stamped = Stamped::parse("base_footprint", Some(at), ()).unwrap();
        assert_eq!(
            stamped.time(),
            LookupTime::At(Stamp::from_time(at).unwrap())
        );
        assert_eq!(
            Stamped::parse("", Some(at), ()),
            Err(GeometryError::NoFrame)
        );
        assert_eq!(report_timestamp(None), None);
        assert_eq!(
            report_timestamp(Some(Stamp::from_time(at).unwrap())),
            Some(at)
        );
    }

    // Every time a stamp holds, the Unix epoch among them, is looked up at that time.
    #[hegel::test]
    fn each_time_a_stamp_holds_is_looked_up_at_that_time(tc: TestCase) {
        let nanos: u64 = tc.draw(gs::integers::<u64>());
        let time = SystemTime::UNIX_EPOCH + Duration::from_nanos(nanos);
        assert_eq!(
            lookup_time(Some(time)),
            Ok(LookupTime::At(Stamp::from_nanos(nanos)))
        );
    }

    #[test]
    fn a_planar_pose_is_refused_off_the_floor_or_tilted() {
        let off = Pose::parse([1.0, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0]).unwrap();
        assert_eq!(off.planar(), Err(GeometryError::OffFloor { z: 0.3 }));
        let half = 0.05_f64;
        let rolled = Pose::parse([1.0, 0.0, 0.0], [half.sin(), 0.0, 0.0, half.cos()]).unwrap();
        assert!(matches!(rolled.planar(), Err(GeometryError::Tilted { .. })));
    }

    #[test]
    fn a_planar_twist_is_refused_with_a_vertical_or_tilting_part() {
        let twist = Twist::parse([0.3, 0.1, 0.0], [0.0, 0.0, 0.5]).unwrap();
        assert_eq!(twist.planar().unwrap().axes(), [0.3, 0.1, 0.5]);
        assert_eq!(
            Twist::parse([0.3, 0.0, 0.2], [0.0; 3]).unwrap().planar(),
            Err(GeometryError::Vertical { z: 0.2 })
        );
        assert_eq!(
            Twist::parse([0.3, 0.0, 0.0], [0.1, 0.0, 0.0])
                .unwrap()
                .planar(),
            Err(GeometryError::TiltingTurn { x: 0.1, y: 0.0 })
        );
    }

    // The floor plane's band is base_motion's 1e-9 on each part off the plane: a height, a tilt,
    // a vertical velocity and a tilting turn of 1e-9 are taken, and of 2e-9 refused.
    #[test]
    fn a_part_off_the_floor_plane_is_taken_within_1e_9_and_refused_past_it() {
        let rolled = |tilt: f64| [(tilt / 2.0).sin(), 0.0, 0.0, (tilt / 2.0).cos()];
        let pose = |z: f64, tilt: f64| Pose::parse([1.0, 0.0, z], rolled(tilt)).unwrap().planar();
        let twist = |vz: f64, wx: f64, wy: f64| {
            Twist::parse([0.3, 0.0, vz], [wx, wy, 0.5])
                .unwrap()
                .planar()
        };
        for taken in [1e-9, -1e-9] {
            assert!(pose(taken, 0.0).is_ok(), "z {taken}");
            assert!(twist(taken, 0.0, 0.0).is_ok(), "linear z {taken}");
            assert!(twist(0.0, taken, 0.0).is_ok(), "angular x {taken}");
            assert!(twist(0.0, 0.0, taken).is_ok(), "angular y {taken}");
        }
        assert!(pose(0.0, 1e-9).is_ok(), "a tilt of 1e-9");
        for refused in [2e-9, -2e-9] {
            assert_eq!(
                pose(refused, 0.0),
                Err(GeometryError::OffFloor { z: refused })
            );
            assert_eq!(
                twist(refused, 0.0, 0.0),
                Err(GeometryError::Vertical { z: refused })
            );
            assert_eq!(
                twist(0.0, refused, 0.0),
                Err(GeometryError::TiltingTurn { x: refused, y: 0.0 })
            );
            assert_eq!(
                twist(0.0, 0.0, refused),
                Err(GeometryError::TiltingTurn { x: 0.0, y: refused })
            );
        }
        assert!(matches!(pose(0.0, 2e-9), Err(GeometryError::Tilted { .. })));
    }

    /// control_core_py's geometry holds no twist, so the twist's refusals are Rust's alone and
    /// stay out of the shared table.
    #[test]
    fn each_refusal_of_a_twist_says_what_to_send() {
        let cases = [
            (
                Twist::parse([0.3, 0.0, 0.0], [0.0, f64::NAN, 0.0]).unwrap_err(),
                "a twist must be finite: got linear_velocity x 0.3, linear_velocity y 0, \
                 linear_velocity z 0, angular_velocity x 0, angular_velocity y NaN, angular_velocity \
                 z 0; give finite numbers, m/s along x, y and z and rad/s about them",
            ),
            (
                Twist::parse([0.3, 0.0, -1e16], [0.0; 3])
                    .unwrap()
                    .planar()
                    .unwrap_err(),
                "the linear velocity moves -10000000000000000 m/s along z: send [vx, vy, 0]",
            ),
            (
                Twist::parse([0.3, 0.0, 0.0], [0.30000000000000004, 0.0, 0.0])
                    .unwrap()
                    .planar()
                    .unwrap_err(),
                "the angular velocity turns 0.30000000000000004 rad/s about x and 0 rad/s about y: \
                 send [0, 0, wz]",
            ),
        ];
        for (refusal, message) in cases {
            assert_eq!(refusal.to_string(), message);
        }
    }

    // A twist turned past the finite numbers is refused as its parse refuses one; a point and a
    // pose are refused likewise where their position leaves them.
    #[test]
    fn a_value_placed_past_the_finite_numbers_is_refused() {
        let eighth = std::f64::consts::FRAC_PI_8;
        let turn = frame_tree::FrameTransform::parse(
            "base",
            "turned",
            [f64::MAX, 0.0, 0.0],
            [0.0, 0.0, eighth.sin(), eighth.cos()],
        )
        .unwrap()
        .placement();
        let twist = Twist::parse([f64::MAX, f64::MAX, 0.0], [0.0; 3]).unwrap();
        assert!(matches!(
            twist.placed_by(&turn),
            Err(GeometryError::NotFinite(_))
        ));
        let pose = Pose::parse([f64::MAX, 0.0, 0.0], UnitQuaternion::IDENTITY.to_xyzw()).unwrap();
        assert!(matches!(
            pose.placed_by(&turn),
            Err(GeometryError::NotFinite(_))
        ));
        let near = Point::parse([0.0, 1.0, 0.0]).unwrap();
        assert!(near.placed_by(&turn).is_ok());
    }

    // A pose goes to nalgebra and back to the arrays it parsed from, and nalgebra's isometry
    // carries a point where tf2's transform of the pose carries it: the two read the
    // quaternion's coordinates in one order.
    #[cfg(feature = "nalgebra")]
    #[hegel::test]
    fn a_pose_in_nalgebra_holds_its_arrays_and_places_a_point_as_tf2(tc: TestCase) {
        let coordinate = || gs::floats().min_value(-100.0).max_value(100.0);
        let unit = || gs::floats().min_value(-1.0).max_value(1.0);
        let position = [
            tc.draw(coordinate()),
            tc.draw(coordinate()),
            tc.draw(coordinate()),
        ];
        let drawn: [f64; 4] = [
            tc.draw(unit()),
            tc.draw(unit()),
            tc.draw(unit()),
            tc.draw(unit()),
        ];
        let norm = drawn.iter().map(|c| c * c).sum::<f64>().sqrt();
        tc.assume(norm > 0.1);
        let pose = Pose::parse(position, drawn.map(|c| c / norm)).unwrap();
        let isometry = nalgebra::Isometry3::from(pose);
        assert_eq!(
            pose_arrays(&isometry),
            (pose.position().position(), pose.orientation().to_xyzw())
        );
        assert_eq!(
            nalgebra::Point3::from(pose.position()),
            nalgebra::Point3::from(position)
        );
        assert_eq!(
            nalgebra::Vector3::from(pose.position()),
            nalgebra::Vector3::from(position)
        );
        let point = [
            tc.draw(coordinate()),
            tc.draw(coordinate()),
            tc.draw(coordinate()),
        ];
        let by_nalgebra = isometry * nalgebra::Point3::from(point);
        let by_tf2 = pose.transform().apply(point);
        for (a, b) in by_nalgebra.coords.iter().zip(by_tf2) {
            assert!((a - b).abs() < 1e-9, "{by_nalgebra} vs {by_tf2:?}");
        }
    }

    // A pose on the floor, turned about +z alone, reads back its position and its heading,
    // within the rounding of a quaternion's half angles.
    #[hegel::test]
    fn a_floor_pose_reads_back_its_heading(tc: TestCase) {
        let x: f64 = tc.draw(gs::floats().min_value(-100.0).max_value(100.0));
        let y: f64 = tc.draw(gs::floats().min_value(-100.0).max_value(100.0));
        let yaw: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
        let planar = Pose::parse([x, y, 0.0], about_z(yaw))
            .unwrap()
            .planar()
            .unwrap();
        assert_eq!([planar.x(), planar.y()], [x, y]);
        assert!(
            (planar.yaw() - yaw).abs() < 1e-12,
            "{} vs {yaw}",
            planar.yaw()
        );
    }
}
