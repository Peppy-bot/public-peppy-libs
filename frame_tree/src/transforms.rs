//! The transforms of the transforms contract: a transform, the place of a child frame in its
//! parent, and a message of `transforms` or of `static_transforms` field for field, as a
//! consumer receives it and as a publisher sends it. A message holds its transforms as
//! records: each a parent frame (`frame`), a `child_frame`, a `position` and an
//! `orientation` ([x, y, z, w]). A message of `transforms` also carries `timestamp`, when its
//! publisher measured all its transforms, and `start_timestamp`, when they last started
//! over, each a `SystemTime` as the message carries it. A transform is made by parsing,
//! from a message's records or from a publisher's values.
//!
//! A message whose times a stamp does not hold, or that names a child frame twice, is
//! refused whole. Otherwise each transform stands alone, as tf2's
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

/// A transform of a message, field for field: `child_frame` placed in its parent frame,
/// `frame`, at `position` (x, y, z, m) and `orientation` (a unit quaternion [x, y, z, w]).
#[derive(Clone, PartialEq, Debug)]
pub struct TransformRecord {
    pub frame: String,
    pub child_frame: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

impl TransformRecord {
    /// The record of `child_frame` placed in `frame` at `position`, turned by `orientation`.
    pub fn new(frame: &str, child_frame: &str, position: [f64; 3], orientation: [f64; 4]) -> Self {
        Self {
            frame: frame.to_owned(),
            child_frame: child_frame.to_owned(),
            position,
            orientation,
        }
    }
}

/// A message of `transforms`, field for field: the moving transforms one publisher measured
/// at `timestamp`, which last started over at `start_timestamp`.
#[derive(Clone, PartialEq, Debug)]
pub struct MovingMessage {
    pub timestamp: SystemTime,
    pub transforms: Vec<TransformRecord>,
    pub start_timestamp: SystemTime,
}

/// The transforms of a message of `static_transforms`: every fixed transform of one
/// publisher, each holding at every time, so the message holds no timestamp.
#[derive(Clone, PartialEq, Debug)]
pub struct StaticMessage {
    pub transforms: Vec<TransformRecord>,
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
        let StaticMessage { transforms } = StaticMessage::from_transforms(transforms)?;
        Ok(Self {
            timestamp,
            transforms,
            start_timestamp,
        })
    }

    /// The message's stamp, its start and its transforms; a time that a stamp or a start
    /// does not hold refuses the message whole.
    pub(crate) fn parse(&self) -> Result<(Stamp, Start, Parsed), MessageError> {
        let (stamp, start) = stamp_and_start(self.timestamp, self.start_timestamp)?;
        Ok((stamp, start, parse(&self.transforms)?))
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
            transforms: transforms
                .iter()
                .map(|t| {
                    TransformRecord::new(
                        t.parent.as_str(),
                        t.child.as_str(),
                        t.placement.position,
                        t.placement.orientation,
                    )
                })
                .collect(),
        })
    }

    pub(crate) fn parse(&self) -> Result<Parsed, MessageError> {
        parse(&self.transforms)
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
    #[error(
        "the message names the child frame {child:?} more than once: name each child frame once"
    )]
    RepeatedChild { child: String },
}

/// A transform of a message that was refused: its index among the message's transforms, its
/// child frame as the message names it, and why.
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

/// Each valid transform of `records`, and each invalid one refused; records that name a child
/// frame twice refused whole.
fn parse(records: &[TransformRecord]) -> Result<Parsed, MessageError> {
    if let Some(child) = repeated_child(records.iter().map(|r| r.child_frame.as_str())) {
        return Err(MessageError::RepeatedChild {
            child: child.to_owned(),
        });
    }
    let (transforms, refused): (Vec<_>, Vec<_>) = records
        .iter()
        .map(|r| FrameTransform::parse(&r.frame, &r.child_frame, r.position, r.orientation))
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
                child: records[index].child_frame.clone(),
                reason: transform.expect_err("partitioned on Err"),
            })
            .collect(),
    })
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

    fn time(nanos: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_nanos(nanos)
    }

    fn fixed(transforms: Vec<TransformRecord>) -> StaticMessage {
        StaticMessage { transforms }
    }

    fn moving() -> MovingMessage {
        MovingMessage {
            timestamp: time(7),
            transforms: vec![
                TransformRecord::new("odom", "base_footprint", [1.0, 2.0, 0.0], UNIT),
                TransformRecord::new(
                    "base_footprint",
                    "lidar_link",
                    [0.1, 0.0, 0.2],
                    [0.0, 0.0, 1.0, 0.0],
                ),
            ],
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
        let error = fixed(vec![
            TransformRecord::new("a", "c", [0.0; 3], UNIT),
            TransformRecord::new("a", "d", [0.0; 3], UNIT),
            TransformRecord::new("b", "c", [0.0; 3], UNIT),
        ])
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
            let parsed = fixed(vec![
                TransformRecord::new("root", "kept", [0.0; 3], UNIT),
                TransformRecord::new(parent, child, position, orientation),
            ])
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
        let parsed = fixed(vec![TransformRecord::new(
            "p",
            "c",
            [0.0; 3],
            [0.0, 0.0, 0.6 * 1.004, 0.8 * 1.004],
        )])
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
