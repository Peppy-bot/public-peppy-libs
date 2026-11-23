//! Whether a stream of commands is live: its newest command arrived within a
//! window, the window the contract of its link states, on whatever clock the
//! node counts arrivals by.

use std::ops::Sub;

/// The window, of the duration type `D` of the node's clock, within which a
/// leader's newest command keeps its stream live.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamWindow<D>(D);

impl<D: Copy + PartialOrd> StreamWindow<D> {
    pub const fn new(window: D) -> Self {
        Self(window)
    }

    /// Whether a stream whose newest command arrived at `last` is live at
    /// `now`: until more than the window passes with no command. A command
    /// that arrived after `now` keeps it live, as does one at `now`.
    pub fn streams<T: Copy + Sub<Output = D>>(&self, last: Option<T>, now: T) -> bool {
        last.is_some_and(|last| now - last <= self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::{Duration, Instant};

    // A stream lives through the window from its newest command and ends past
    // it; one with no command never streams.
    #[test]
    fn a_stream_lives_through_its_window_and_ends_past_it() {
        let window = StreamWindow::new(500_i64);
        assert!(!window.streams(None, 1_000_i64));
        assert!(window.streams(Some(1_000_i64), 1_500));
        assert!(!window.streams(Some(1_000_i64), 1_501));
        assert!(
            window.streams(Some(2_000_i64), 1_000),
            "a command after now"
        );
    }

    // On a monotonic clock, a command after now counts as one at now.
    #[test]
    fn a_window_on_a_monotonic_clock_counts_the_same() {
        let window = StreamWindow::new(Duration::from_millis(250));
        let now = Instant::now();
        assert!(window.streams(Some(now), now + Duration::from_millis(250)));
        assert!(!window.streams(Some(now), now + Duration::from_millis(251)));
        assert!(window.streams(Some(now + Duration::from_secs(1)), now));
    }
}
