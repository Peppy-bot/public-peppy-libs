//! The name of a frame, a time of the tree, when a transform last started over, and who
//! publishes it.

use std::fmt;
use std::time::{Duration, SystemTime};

/// The name of a frame of the tree: a link of the robot's URDF, byte for byte, or `odom` or
/// `map`. It is not empty. tf2 strips a leading `/` from the frames of a transform it
/// stores (buffer_core.cpp:78-85, 212-213) and refuses one in a lookup
/// (buffer_core.cpp:142-146); here a name is its URDF name byte for byte, `/` and all.
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct FrameName(String);

/// An empty name, which names no frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug, thiserror::Error)]
#[error("a frame name is empty: name a link of the robot's URDF, odom or map")]
pub struct FrameNameError;

impl FrameName {
    /// The frame named `name`.
    pub fn parse(name: &str) -> Result<Self, FrameNameError> {
        if name.is_empty() {
            return Err(FrameNameError);
        }
        Ok(Self(name.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for FrameName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// A time on the clock of the tree's publishers, in nanoseconds since the Unix epoch, as
/// tf2 counts a time point in nanoseconds (buffer_core.cpp:198-201).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Stamp(u64);

impl Stamp {
    pub const fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }

    /// The stamp of `time`, as a message carries one.
    pub fn from_time(time: SystemTime) -> Result<Self, TimeError> {
        nanos_since_epoch(time).map(Self)
    }

    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// The stamp as a time, as a message carries one.
    pub fn time(self) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_nanos(self.0)
    }
}

impl fmt::Display for Stamp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        const NANOS_PER_SECOND: u64 = 1_000_000_000;
        write!(
            f,
            "{}.{:09} s",
            self.0 / NANOS_PER_SECOND,
            self.0 % NANOS_PER_SECOND
        )
    }
}

/// When a transform last started over, on the wall clock of the machine that runs its
/// publisher, in nanoseconds since the Unix epoch. Two starts are compared for equality
/// alone.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct Start(u64);

impl Start {
    /// The start at `time`, as a message carries one.
    pub(crate) fn from_time(time: SystemTime) -> Result<Self, TimeError> {
        nanos_since_epoch(time).map(Self)
    }
}

/// A time that a stamp or a start does not hold: one before the Unix epoch, or one more
/// than `u64::MAX` nanoseconds after it.
#[derive(Clone, Copy, PartialEq, Eq, Debug, thiserror::Error)]
pub enum TimeError {
    #[error(
        "the time is {before:?} before the Unix epoch: give a time at or after \
         1970-01-01T00:00:00Z"
    )]
    BeforeEpoch { before: Duration },
    #[error(
        "the time is {after:?} after the Unix epoch, past the last time a stamp holds, \
         {:?} after it: give an earlier time",
        Duration::from_nanos(u64::MAX)
    )]
    PastLast { after: Duration },
}

/// The nanoseconds from the Unix epoch to `time`.
fn nanos_since_epoch(time: SystemTime) -> Result<u64, TimeError> {
    let after = time
        .duration_since(SystemTime::UNIX_EPOCH)
        .map_err(|error| TimeError::BeforeEpoch {
            before: error.duration(),
        })?;
    u64::try_from(after.as_nanos()).map_err(|_| TimeError::PastLast { after })
}

/// A publisher of transforms: the instance that publishes, named by its core node and its
/// instance id, as the transforms contract names one.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct Publisher {
    pub core_node: String,
    pub instance_id: String,
}

impl fmt::Display for Publisher {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "instance {} of {}", self.instance_id, self.core_node)
    }
}

/// The time of a lookup.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LookupTime {
    /// The latest time at which every moving transform of the chain has a value.
    Latest,
    At(Stamp),
}

impl fmt::Display for LookupTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LookupTime::Latest => f.write_str("the latest time"),
            LookupTime::At(stamp) => write!(f, "{stamp}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_name_is_any_name_but_the_empty_one() {
        for name in ["odom", "a/b", "/odom"] {
            assert_eq!(FrameName::parse(name).unwrap().as_str(), name);
        }
        assert_eq!(FrameName::parse(""), Err(FrameNameError));
    }

    #[test]
    fn a_stamp_and_a_start_go_to_a_time_and_back() {
        let time = SystemTime::UNIX_EPOCH + Duration::new(1_763_400_000, 123_456_789);
        let stamp = Stamp::from_time(time).unwrap();
        assert_eq!(stamp.as_nanos(), 1_763_400_000_123_456_789);
        assert_eq!(stamp.time(), time);
        assert!(Start::from_time(time).is_ok());
        let last = SystemTime::UNIX_EPOCH + Duration::from_nanos(u64::MAX);
        assert_eq!(Stamp::from_time(last).unwrap().time(), last);
        assert_eq!(
            Stamp::from_time(SystemTime::UNIX_EPOCH),
            Ok(Stamp::from_nanos(0))
        );
    }

    #[test]
    fn a_time_before_the_epoch_or_past_the_last_stamp_is_refused() {
        let before = SystemTime::UNIX_EPOCH - Duration::from_nanos(1);
        let error = TimeError::BeforeEpoch {
            before: Duration::from_nanos(1),
        };
        assert_eq!(Stamp::from_time(before), Err(error));
        assert_eq!(Start::from_time(before), Err(error));
        assert_eq!(
            error.to_string(),
            "the time is 1ns before the Unix epoch: give a time at or after 1970-01-01T00:00:00Z"
        );
        let after = Duration::from_nanos(u64::MAX) + Duration::from_nanos(1);
        let past = SystemTime::UNIX_EPOCH + after;
        let error = TimeError::PastLast { after };
        assert_eq!(Stamp::from_time(past), Err(error));
        assert_eq!(Start::from_time(past), Err(error));
        assert_eq!(
            error.to_string(),
            "the time is 18446744073.709551616s after the Unix epoch, past the last time a \
             stamp holds, 18446744073.709551615s after it: give an earlier time"
        );
    }

    #[test]
    fn a_stamp_prints_in_seconds() {
        assert_eq!(
            Stamp::from_nanos(12_000_000_005).to_string(),
            "12.000000005 s"
        );
        assert_eq!(LookupTime::Latest.to_string(), "the latest time");
    }

    #[test]
    fn a_publisher_prints_its_instance_and_its_core_node() {
        let publisher = Publisher {
            core_node: "base_controller".into(),
            instance_id: "wheels".into(),
        };
        assert_eq!(publisher.to_string(), "instance wheels of base_controller");
    }
}
