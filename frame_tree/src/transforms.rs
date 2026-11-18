//! The transforms of the transforms contract: a transform, the place of a child frame in its
//! parent, and a message of `transforms` or of `static_transforms` field for field, as a
//! consumer receives it and as a publisher sends it. A message's arrays are index-aligned:
//! `child_frames`, `parent_frames`, `positions` (3 per transform) and `orientations` (4 per
//! transform, [x, y, z, w]). A message of `transforms` also carries `timestamp`, when its
//! publisher measured all its transforms, and `start_timestamp`, when they last started
//! over, each a `SystemTime` as the message carries it. A transform is made by parsing,
//! from a message's arrays or from a publisher's values.
//!
//! A message whose times a stamp does not hold, whose arrays do not line up, or that names
//! a child frame twice, is refused whole. Otherwise each transform stands alone, as tf2's
//! listener hands each transform of a message to `setTransform` and that call refuses an
//! invalid one by itself (buffer_core.cpp:210-266): an invalid transform is refused, with
//! the reason, and the message's other transforms are kept, each with its quaternion
//! normalized.

use std::collections::BTreeSet;
use std::time::SystemTime;

use crate::arithmetic;
use crate::frame::{FrameName, FrameNameError, Stamp, Start, TimeError};

/// tf2's tolerance on a quaternion's squared norm: an orientation whose squared norm
/// differs from 1 by this much or more is refused (buffer_core.cpp:66, 250-255).
const QUATERNION_NORMALIZATION_TOLERANCE: f64 = 10e-3;

/// Where a child frame sits in its parent: a point `p` of the child frame is
/// `R(orientation) * p + position` of the parent frame, the position in meters and the
/// orientation a unit quaternion [x, y, z, w].
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Placement {
    position: [f64; 3],
    orientation: [f64; 4],
}

impl Placement {
    /// A frame in itself.
    pub const IDENTITY: Self = Self {
        position: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
    };

    pub(crate) const fn new(position: [f64; 3], orientation: [f64; 4]) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    pub fn orientation(&self) -> [f64; 4] {
        self.orientation
    }
}

/// A transform: the place of `child` in `parent`.
#[derive(Clone, PartialEq, Debug)]
pub struct FrameTransform {
    parent: FrameName,
    child: FrameName,
    placement: Placement,
}

impl FrameTransform {
    /// The transform that places `child` in `parent` at `position`, in meters, turned by
    /// `orientation`, a quaternion [x, y, z, w], checked as tf2's `setTransformImpl` checks
    /// it (buffer_core.cpp:210-266), with its quaternion normalized. tf2 keeps a quaternion
    /// within its tolerance as given.
    pub fn parse(
        parent: &str,
        child: &str,
        position: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<Self, TransformErrorReason> {
        let child = FrameName::parse(child).map_err(TransformErrorReason::Child)?;
        let parent = FrameName::parse(parent).map_err(TransformErrorReason::Parent)?;
        if child == parent {
            return Err(TransformErrorReason::SelfTransform);
        }
        if !position.iter().all(|c| c.is_finite()) {
            return Err(TransformErrorReason::Position(position));
        }
        if !is_unit(orientation) {
            return Err(TransformErrorReason::Orientation(orientation));
        }
        Ok(Self {
            parent,
            child,
            placement: Placement::new(position, arithmetic::normalized(orientation)),
        })
    }

    pub fn parent(&self) -> &FrameName {
        &self.parent
    }

    pub fn child(&self) -> &FrameName {
        &self.child
    }

    pub fn placement(&self) -> Placement {
        self.placement
    }
}

/// A message of `transforms`, field for field: the moving transforms one publisher measured
/// at `timestamp`, which last started over at `start_timestamp`.
#[derive(Clone, PartialEq, Debug)]
pub struct MovingMessage {
    pub timestamp: SystemTime,
    pub child_frames: Vec<String>,
    pub parent_frames: Vec<String>,
    pub positions: Vec<f64>,
    pub orientations: Vec<f64>,
    pub start_timestamp: SystemTime,
}

/// A message of `static_transforms`, field for field: every fixed transform of one
/// publisher.
#[derive(Clone, PartialEq, Debug)]
pub struct StaticMessage {
    pub child_frames: Vec<String>,
    pub parent_frames: Vec<String>,
    pub positions: Vec<f64>,
    pub orientations: Vec<f64>,
}

impl MovingMessage {
    /// The message that sends `transforms`, measured at `timestamp` and last started over
    /// at `start_timestamp`, refused on the times that [`FrameTree::take_moving`] refuses.
    ///
    /// [`FrameTree::take_moving`]: crate::FrameTree::take_moving
    pub fn from_transforms(
        timestamp: SystemTime,
        transforms: &[FrameTransform],
        start_timestamp: SystemTime,
    ) -> Result<Self, MessageError> {
        stamp_and_start(timestamp, start_timestamp)?;
        let StaticMessage {
            child_frames,
            parent_frames,
            positions,
            orientations,
        } = StaticMessage::from_transforms(transforms)?;
        Ok(Self {
            timestamp,
            child_frames,
            parent_frames,
            positions,
            orientations,
            start_timestamp,
        })
    }

    /// The message's stamp, its start and its transforms; a time that a stamp or a start
    /// does not hold refuses the message whole.
    pub(crate) fn parse(&self) -> Result<(Stamp, Start, Parsed), MessageError> {
        let (stamp, start) = stamp_and_start(self.timestamp, self.start_timestamp)?;
        let parsed = Arrays {
            child_frames: &self.child_frames,
            parent_frames: &self.parent_frames,
            positions: &self.positions,
            orientations: &self.orientations,
        }
        .parse()?;
        Ok((stamp, start, parsed))
    }
}

/// The stamp of `timestamp` and the start of `start_timestamp`, each refused with the
/// field it came from.
fn stamp_and_start(
    timestamp: SystemTime,
    start_timestamp: SystemTime,
) -> Result<(Stamp, Start), MessageError> {
    Ok((
        Stamp::from_time(timestamp).map_err(MessageError::Timestamp)?,
        Start::from_time(start_timestamp).map_err(MessageError::StartTimestamp)?,
    ))
}

impl StaticMessage {
    /// The message that sends `transforms` as a publisher's whole static set.
    pub fn from_transforms(transforms: &[FrameTransform]) -> Result<Self, MessageError> {
        if let Some(child) = repeated_child(transforms.iter().map(|t| t.child.as_str())) {
            return Err(MessageError::RepeatedChild {
                child: child.to_owned(),
            });
        }
        Ok(Self {
            child_frames: transforms.iter().map(|t| t.child.to_string()).collect(),
            parent_frames: transforms.iter().map(|t| t.parent.to_string()).collect(),
            positions: transforms
                .iter()
                .flat_map(|t| t.placement.position)
                .collect(),
            orientations: transforms
                .iter()
                .flat_map(|t| t.placement.orientation)
                .collect(),
        })
    }

    pub(crate) fn parse(&self) -> Result<Parsed, MessageError> {
        Arrays {
            child_frames: &self.child_frames,
            parent_frames: &self.parent_frames,
            positions: &self.positions,
            orientations: &self.orientations,
        }
        .parse()
    }
}

/// The transforms of a message, and the ones it held that were refused.
#[derive(Debug)]
pub(crate) struct Parsed {
    pub transforms: Vec<FrameTransform>,
    pub refused: Vec<TransformError>,
}

/// A message refused whole: no transform of it is taken.
#[derive(Clone, PartialEq, Eq, Debug, thiserror::Error)]
pub enum MessageError {
    #[error("the message's timestamp is refused: {0}")]
    Timestamp(TimeError),
    #[error("the message's start_timestamp is refused: {0}")]
    StartTimestamp(TimeError),
    /// Its arrays do not hold one entry of each per child frame.
    #[error(
        "the message holds {children} child frames, {parents} parent frames, {positions} \
         position entries and {orientations} orientation entries: give one parent frame, 3 \
         position entries and 4 orientation entries per child frame"
    )]
    Shape {
        children: usize,
        parents: usize,
        positions: usize,
        orientations: usize,
    },
    #[error(
        "the message names the child frame {child:?} more than once: name each child frame once"
    )]
    RepeatedChild { child: String },
}

/// A transform of a message that was refused: the index of its child frame in
/// `child_frames`, its child frame as the message names it, and why.
#[derive(Clone, PartialEq, Debug, thiserror::Error)]
#[error("transform {index} (child frame {child:?}) is refused: {reason}")]
pub struct TransformError {
    pub index: usize,
    pub child: String,
    pub reason: TransformErrorReason,
}

#[derive(Clone, PartialEq, Debug, thiserror::Error)]
pub enum TransformErrorReason {
    #[error("its child frame: {0}")]
    Child(FrameNameError),
    #[error("its parent frame: {0}")]
    Parent(FrameNameError),
    /// tf2's TF_SELF_TRANSFORM (buffer_core.cpp:216-222).
    #[error("its parent frame is its child frame: a transform joins two frames")]
    SelfTransform,
    /// tf2 refuses a NaN (TF_NAN_INPUT, buffer_core.cpp:238-248) and stores an infinite
    /// coordinate; a position of the contract is in meters, so every coordinate is finite.
    #[error("its position {0:?} is not three finite numbers of meters")]
    Position([f64; 3]),
    /// tf2's TF_NAN_INPUT and TF_DENORMALIZED_QUATERNION (buffer_core.cpp:238-266), and an
    /// infinite coordinate, whose squared norm is no number near 1.
    #[error(
        "its orientation {0:?} is not a unit quaternion [x, y, z, w]: give four finite \
         numbers whose squared norm differs from 1 by less than \
         {QUATERNION_NORMALIZATION_TOLERANCE}"
    )]
    Orientation([f64; 4]),
    /// A stamp older than the newest stamp of its child frame by more than the cache
    /// window, as tf2's `insertData` refuses it (cache.cpp:260-264).
    #[error(
        "its timestamp {stamp} is more than the cache window before {newest}, the newest \
         stamp held of its child frame: send it within the cache window of the newest, or set a \
         wider cache window"
    )]
    BeforeWindow { stamp: Stamp, newest: Stamp },
}

/// The arrays every message of the contract carries.
struct Arrays<'a> {
    child_frames: &'a [String],
    parent_frames: &'a [String],
    positions: &'a [f64],
    orientations: &'a [f64],
}

impl Arrays<'_> {
    /// Each valid transform, and each invalid one refused; arrays that do not line up or
    /// name a child frame twice refused whole.
    fn parse(&self) -> Result<Parsed, MessageError> {
        let count = self.child_frames.len();
        if self.parent_frames.len() != count
            || self.positions.len() != 3 * count
            || self.orientations.len() != 4 * count
        {
            return Err(MessageError::Shape {
                children: count,
                parents: self.parent_frames.len(),
                positions: self.positions.len(),
                orientations: self.orientations.len(),
            });
        }
        if let Some(child) = repeated_child(self.child_frames.iter().map(String::as_str)) {
            return Err(MessageError::RepeatedChild {
                child: child.to_owned(),
            });
        }
        let (transforms, refused): (Vec<_>, Vec<_>) = (0..count)
            .map(|index| {
                FrameTransform::parse(
                    &self.parent_frames[index],
                    &self.child_frames[index],
                    std::array::from_fn(|i| self.positions[3 * index + i]),
                    std::array::from_fn(|i| self.orientations[4 * index + i]),
                )
            })
            .enumerate()
            .partition(|(_, transform)| transform.is_ok());
        Ok(Parsed {
            transforms: transforms
                .into_iter()
                .map(|(_, transform)| transform.expect("partitioned on Ok"))
                .collect(),
            refused: refused
                .into_iter()
                .map(|(index, transform)| TransformError {
                    index,
                    child: self.child_frames[index].clone(),
                    reason: transform.expect_err("partitioned on Err"),
                })
                .collect(),
        })
    }
}

/// The first child frame that `children` names a second time.
fn repeated_child<'a>(children: impl IntoIterator<Item = &'a str>) -> Option<&'a str> {
    let mut named = BTreeSet::new();
    children.into_iter().find(|child| !named.insert(*child))
}

/// tf2's check of a quaternion, its squared norm summed w first within the tolerance of 1
/// (buffer_core.cpp:250-255), each coordinate finite.
fn is_unit(orientation: [f64; 4]) -> bool {
    let [x, y, z, w] = orientation;
    orientation.iter().all(|c| c.is_finite())
        && ((w * w + x * x + y * y + z * z) - 1.0).abs() < QUATERNION_NORMALIZATION_TOLERANCE
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    /// The difference allowed where the arithmetic is exact but for its rounding.
    const TOLERANCE: f64 = 1e-12;

    const UNIT: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

    fn names(names: &[&str]) -> Vec<String> {
        names.iter().map(|name| (*name).to_owned()).collect()
    }

    fn time(nanos: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_nanos(nanos)
    }

    fn fixed(
        children: &[&str],
        parents: &[&str],
        positions: &[f64],
        orientations: &[f64],
    ) -> StaticMessage {
        StaticMessage {
            child_frames: names(children),
            parent_frames: names(parents),
            positions: positions.to_vec(),
            orientations: orientations.to_vec(),
        }
    }

    fn moving() -> MovingMessage {
        MovingMessage {
            timestamp: time(7),
            child_frames: names(&["base_footprint", "lidar_link"]),
            parent_frames: names(&["odom", "base_footprint"]),
            positions: vec![1.0, 2.0, 0.0, 0.1, 0.0, 0.2],
            orientations: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
            start_timestamp: time(3_000_000_000),
        }
    }

    #[test]
    fn a_message_parses_into_its_stamp_its_start_and_its_transforms_in_order() {
        let (stamp, start, parsed) = moving().parse().unwrap();
        assert_eq!(stamp, Stamp::from_nanos(7));
        assert_eq!(Ok(start), Start::from_time(time(3_000_000_000)));
        assert!(parsed.refused.is_empty());
        let [first, second] = parsed.transforms.as_slice() else {
            panic!("two transforms");
        };
        assert_eq!(first.child().as_str(), "base_footprint");
        assert_eq!(first.parent().as_str(), "odom");
        assert_eq!(first.placement().position(), [1.0, 2.0, 0.0]);
        assert_eq!(second.placement().orientation(), [0.0, 0.0, 1.0, 0.0]);
    }

    #[test]
    fn a_message_from_its_transforms_parses_back_into_them() {
        let message = moving();
        let (_, _, Parsed { transforms, .. }) = message.parse().unwrap();
        let sent =
            MovingMessage::from_transforms(message.timestamp, &transforms, message.start_timestamp)
                .unwrap();
        assert_eq!(sent, message);
        let fixed = StaticMessage::from_transforms(&transforms).unwrap();
        assert_eq!(fixed.parse().unwrap().transforms, transforms);
        let twice = [transforms[0].clone(), transforms[0].clone()];
        assert_eq!(
            StaticMessage::from_transforms(&twice),
            Err(MessageError::RepeatedChild {
                child: "base_footprint".into()
            })
        );
    }

    #[test]
    fn a_transform_parses_from_a_publishers_values() {
        let transform = FrameTransform::parse(
            "odom",
            "base_footprint",
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 2.0],
        );
        assert_eq!(
            transform,
            Err(TransformErrorReason::Orientation([0.0, 0.0, 0.0, 2.0]))
        );
        let transform =
            FrameTransform::parse("odom", "base_footprint", [1.0, 0.0, 0.0], UNIT).unwrap();
        assert_eq!(transform.parent().as_str(), "odom");
        assert_eq!(transform.child().as_str(), "base_footprint");
        assert_eq!(transform.placement().position(), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn arrays_that_do_not_line_up_refuse_the_message() {
        let error = fixed(&["a"], &["b"], &[0.0; 3], &[0.0; 3])
            .parse()
            .unwrap_err();
        assert_eq!(
            error,
            MessageError::Shape {
                children: 1,
                parents: 1,
                positions: 3,
                orientations: 3,
            }
        );
        assert_eq!(
            error.to_string(),
            "the message holds 1 child frames, 1 parent frames, 3 position entries and 3 \
             orientation entries: give one parent frame, 3 position entries and 4 orientation \
             entries per child frame"
        );
        let error = MovingMessage {
            orientations: UNIT.to_vec(),
            ..moving()
        }
        .parse()
        .unwrap_err();
        assert_eq!(
            error,
            MessageError::Shape {
                children: 2,
                parents: 2,
                positions: 6,
                orientations: 4,
            }
        );
    }

    #[test]
    fn a_time_before_the_epoch_refuses_the_message_naming_its_field() {
        let before = SystemTime::UNIX_EPOCH - Duration::from_nanos(1);
        let early = TimeError::BeforeEpoch {
            before: Duration::from_nanos(1),
        };
        let error = MovingMessage {
            timestamp: before,
            ..moving()
        }
        .parse()
        .unwrap_err();
        assert_eq!(error, MessageError::Timestamp(early));
        assert!(
            error
                .to_string()
                .starts_with("the message's timestamp is refused: the time is 1ns before"),
            "{error}"
        );
        let error = MovingMessage {
            start_timestamp: before,
            ..moving()
        }
        .parse()
        .unwrap_err();
        assert_eq!(error, MessageError::StartTimestamp(early));
        assert!(
            error
                .to_string()
                .starts_with("the message's start_timestamp is refused: the time is 1ns before"),
            "{error}"
        );
    }

    #[test]
    fn a_publishers_message_is_refused_on_the_times_a_consumer_refuses() {
        let before = SystemTime::UNIX_EPOCH - Duration::from_nanos(1);
        let early = TimeError::BeforeEpoch {
            before: Duration::from_nanos(1),
        };
        let message = moving();
        let (_, _, Parsed { transforms, .. }) = message.parse().unwrap();
        assert_eq!(
            MovingMessage::from_transforms(before, &transforms, message.start_timestamp),
            Err(MessageError::Timestamp(early))
        );
        assert_eq!(
            MovingMessage::from_transforms(message.timestamp, &transforms, before),
            Err(MessageError::StartTimestamp(early))
        );
    }

    #[test]
    fn a_child_named_twice_refuses_the_message() {
        let error = fixed(
            &["c", "d", "c"],
            &["a", "a", "b"],
            &[0.0; 9],
            &UNIT.repeat(3),
        )
        .parse()
        .unwrap_err();
        assert_eq!(error, MessageError::RepeatedChild { child: "c".into() });
    }

    #[test]
    fn each_invalid_transform_is_refused_with_its_reason_and_the_others_kept() {
        // A child frame, a parent frame, a position, an orientation and the reason given.
        type Case = (
            &'static str,
            &'static str,
            [f64; 3],
            [f64; 4],
            TransformErrorReason,
        );
        let cases: [Case; 7] = [
            (
                "",
                "p",
                [0.0; 3],
                UNIT,
                TransformErrorReason::Child(FrameNameError),
            ),
            (
                "c",
                "",
                [0.0; 3],
                UNIT,
                TransformErrorReason::Parent(FrameNameError),
            ),
            (
                "c",
                "c",
                [0.0; 3],
                UNIT,
                TransformErrorReason::SelfTransform,
            ),
            (
                "c",
                "p",
                [f64::INFINITY, 0.0, 0.0],
                UNIT,
                TransformErrorReason::Position([f64::INFINITY, 0.0, 0.0]),
            ),
            (
                "c",
                "p",
                [0.0; 3],
                [0.0, 0.0, 0.0, 1.01],
                TransformErrorReason::Orientation([0.0, 0.0, 0.0, 1.01]),
            ),
            (
                "c",
                "p",
                [0.0; 3],
                [0.0, 0.0, 0.0, f64::INFINITY],
                TransformErrorReason::Orientation([0.0, 0.0, 0.0, f64::INFINITY]),
            ),
            (
                "c",
                "p",
                [0.0; 3],
                [f64::NAN, 0.0, 0.0, 1.0],
                TransformErrorReason::Orientation([f64::NAN, 0.0, 0.0, 1.0]),
            ),
        ];
        for (child, parent, position, orientation, reason) in cases {
            let parsed = fixed(
                &["kept", child],
                &["root", parent],
                &[[0.0; 3], position].concat(),
                &[UNIT, orientation].concat(),
            )
            .parse()
            .unwrap();
            let [kept] = parsed.transforms.as_slice() else {
                panic!("one transform kept: {reason}");
            };
            assert_eq!(kept.child().as_str(), "kept");
            let [refused] = parsed.refused.as_slice() else {
                panic!("one transform refused: {reason}");
            };
            assert_eq!((refused.index, refused.child.as_str()), (1, child));
            // NaN equals nothing, so the reason is compared by its text.
            assert_eq!(refused.reason.to_string(), reason.to_string());
        }
    }

    #[test]
    fn a_quaternion_within_tf2s_tolerance_is_taken_and_normalized() {
        // A squared norm of 1.009 is within 10e-3 of 1.
        assert!(is_unit([0.0, 0.0, 0.0, 1.009_f64.sqrt()]));
        assert!(!is_unit([0.0, 0.0, 0.0, 1.0101_f64.sqrt()]));
        let parsed = fixed(
            &["c"],
            &["p"],
            &[0.0; 3],
            &[0.0, 0.0, 0.6 * 1.004, 0.8 * 1.004],
        )
        .parse()
        .unwrap();
        let [x, y, z, w] = parsed.transforms[0].placement().orientation();
        assert_eq!([x, y], [0.0, 0.0]);
        assert!(
            (z - 0.6).abs() < TOLERANCE && (w - 0.8).abs() < TOLERANCE,
            "{z} {w}"
        );
    }
}
