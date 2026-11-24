//! drive_link's terms that its leaders and its follower share: the window of
//! the leader's stream, and the follow_state each drive_states reports.

use thiserror::Error;

use crate::stream_window::StreamWindow;

/// drive_link's window (ns): the leader streams from each setpoint that
/// arrives until more than 0.5 s pass with no setpoint, and every hold that
/// waits on the leader's silence counts it.
pub const WINDOW_NS: i64 = 500_000_000;

/// The leader's stream, on the clock its follower counts arrivals by: live
/// from each setpoint until more than [`WINDOW_NS`] pass with no setpoint.
pub const LEADER_STREAM: StreamWindow<i64> = StreamWindow::new(WINDOW_NS);

/// drive_states' follow_state: the first hold of the base that holds, or
/// `Following` when none does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FollowState {
    /// The base follows the leader, or would follow its next setpoint.
    Following,
    /// A mode other than standing and moving holds the base.
    BaseMode,
    /// A silence of the base's motion sensing holds it.
    BaseSilence,
    /// A robot stop holds it.
    RobotStop,
    /// A goal of the follower's node holds it.
    FollowerGoal,
}

/// A follow_state that names none of drive_link's values.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
#[error(
    "follow_state '{0}' is none of drive_link's: {values}",
    values = FollowState::ALL.map(FollowState::as_str).join(", ")
)]
pub struct UnknownFollowState(pub String);

impl FollowState {
    /// Every value, in drive_link's order.
    const ALL: [Self; 5] = [
        Self::Following,
        Self::BaseMode,
        Self::BaseSilence,
        Self::RobotStop,
        Self::FollowerGoal,
    ];

    /// The value drive_link names it by.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Following => "following",
            Self::BaseMode => "base_mode",
            Self::BaseSilence => "base_silence",
            Self::RobotStop => "robot_stop",
            Self::FollowerGoal => "follower_goal",
        }
    }

    /// The state drive_states names `value`.
    pub fn parse(value: &str) -> Result<Self, UnknownFollowState> {
        Self::ALL
            .into_iter()
            .find(|state| state.as_str() == value)
            .ok_or_else(|| UnknownFollowState(value.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_values_are_drive_links_in_its_order() {
        assert_eq!(
            FollowState::ALL.map(FollowState::as_str),
            [
                "following",
                "base_mode",
                "base_silence",
                "robot_stop",
                "follower_goal"
            ]
        );
    }

    #[test]
    fn each_value_of_drive_link_parses_and_no_other() {
        for state in FollowState::ALL {
            assert_eq!(FollowState::parse(state.as_str()), Ok(state));
        }
        assert_eq!(
            FollowState::parse("resting").unwrap_err().to_string(),
            "follow_state 'resting' is none of drive_link's: following, base_mode, \
             base_silence, robot_stop, follower_goal"
        );
    }

    #[test]
    fn the_leaders_stream_ends_once_more_than_half_a_second_passes() {
        assert!(LEADER_STREAM.streams(Some(0_i64), 500_000_000));
        assert!(!LEADER_STREAM.streams(Some(0_i64), 500_000_001));
    }
}
