//! The clock a node reads: the wall clock, or the simulation clock of the domain the
//! node is bound to. It gives the time now and a wait until a deadline on it, each in
//! nanoseconds since the Unix epoch.
//!
//! A simulation clock reads the ticks of its domain, which the node receives and passes
//! in: [`NodeClock::simulation`] gives the clock and the [`TickFeed`] that sets it, and
//! [`follow_ticks`] feeds it each tick a node's receive gives. Before its first tick a
//! simulation clock reads 0. [`BackSteps`] tells whether a clock went back between two
//! looks.

use std::fmt::Display;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use tokio::sync::watch;
use tracing::warn;

/// A node's clock: the wall clock or a simulation clock. Clones read one clock.
#[derive(Clone, Debug)]
pub struct NodeClock(Source);

#[derive(Clone, Debug)]
enum Source {
    Wall,
    Simulation {
        /// The time of the newest tick, 0 before the first.
        ticks: watch::Receiver<u64>,
        /// The ticks so far that came before the tick before them.
        steps_back: Arc<AtomicU64>,
    },
}

/// What sets a simulation clock: the time of each of its ticks.
#[derive(Debug)]
pub struct TickFeed {
    ticks: watch::Sender<u64>,
    steps_back: Arc<AtomicU64>,
}

impl TickFeed {
    /// Sets the clock to the tick at `time_ns`. A tick before the one before it is a step
    /// back, counted before the clock reads it, so a reader that sees the tick sees the
    /// step back too.
    pub fn tick(&self, time_ns: u64) {
        if time_ns < *self.ticks.borrow() {
            self.steps_back.fetch_add(1, Ordering::Release);
        }
        self.ticks.send_replace(time_ns);
    }
}

impl NodeClock {
    /// The wall clock of the machine.
    pub fn wall() -> Self {
        Self(Source::Wall)
    }

    /// A simulation clock that reads 0, and the feed that sets it at each tick. Once the
    /// feed is dropped, the clock keeps its last tick and ends no wait past it.
    pub fn simulation() -> (Self, TickFeed) {
        let (sender, ticks) = watch::channel(0);
        let steps_back = Arc::new(AtomicU64::new(0));
        let feed = TickFeed {
            ticks: sender,
            steps_back: Arc::clone(&steps_back),
        };
        (Self(Source::Simulation { ticks, steps_back }), feed)
    }

    /// A watch of this clock's steps back from now on.
    pub fn back_steps(&self) -> BackSteps {
        BackSteps(match &self.0 {
            Source::Wall => Watched::Wall {
                reading: wall_now_ns(),
            },
            Source::Simulation { steps_back, .. } => Watched::Simulation {
                seen: steps_back.load(Ordering::Acquire),
                steps_back: Arc::clone(steps_back),
            },
        })
    }

    /// The time now on this clock, in nanoseconds since the Unix epoch.
    pub fn now_ns(&self) -> u64 {
        match &self.0 {
            Source::Wall => wall_now_ns(),
            Source::Simulation { ticks, .. } => *ticks.borrow(),
        }
    }

    /// Resolves once this clock reads `deadline_ns` or later, at once for a deadline it
    /// has reached. A simulation clock whose feed was dropped ends no wait past its last
    /// tick.
    pub async fn until(&self, deadline_ns: u64) {
        match &self.0 {
            Source::Wall => {
                let left = deadline_ns.saturating_sub(wall_now_ns());
                tokio::time::sleep(Duration::from_nanos(left)).await;
            }
            Source::Simulation { ticks, .. } => {
                if ticks
                    .clone()
                    .wait_for(|now| *now >= deadline_ns)
                    .await
                    .is_err()
                {
                    std::future::pending::<()>().await;
                }
            }
        }
    }

    /// The ticks of a simulation clock, which change at each tick and close when its feed
    /// is dropped; none for the wall clock.
    pub fn ticks(&self) -> Option<watch::Receiver<u64>> {
        match &self.0 {
            Source::Wall => None,
            Source::Simulation { ticks, .. } => Some(ticks.clone()),
        }
    }
}

/// Whether a node's clock went back between one look and the next, as tf2_ros's buffer
/// watches its clock for a jump back in time (tf2_ros/src/buffer.cpp:70-80). A simulation
/// clock's feed counts each tick before the tick before it, so a step back is seen even
/// where the clock went forward again before the next look. The wall clock is read at each
/// look, and a reading before the one at the look before is a step back.
#[derive(Debug)]
pub struct BackSteps(Watched);

#[derive(Debug)]
enum Watched {
    Wall {
        /// The reading at the last look.
        reading: u64,
    },
    Simulation {
        /// The steps back the feed counted.
        steps_back: Arc<AtomicU64>,
        /// The count at the last look.
        seen: u64,
    },
}

impl BackSteps {
    /// Whether the clock went back since the last look, or since the watch began.
    pub fn went_back(&mut self) -> bool {
        match &mut self.0 {
            Watched::Wall { reading } => {
                let now = wall_now_ns();
                let went_back = now < *reading;
                *reading = now;
                went_back
            }
            Watched::Simulation { steps_back, seen } => {
                let count = steps_back.load(Ordering::Acquire);
                let went_back = count != *seen;
                *seen = count;
                went_back
            }
        }
    }
}

/// Now on the wall clock of the machine, in nanoseconds since the Unix epoch; 0 for a
/// clock set before the epoch.
fn wall_now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| {
            u64::try_from(since.as_nanos()).unwrap_or(u64::MAX)
        })
}

/// Feeds `feed` the time of each tick `next_tick` receives, in nanoseconds since the
/// Unix epoch, until it gives `Ok(None)`, the end of the clock's ticks, and then calls
/// `closed`. A tick that does not decode is left out, with a warning, and the ticks after
/// it go on.
pub async fn follow_ticks<E: Display>(
    mut next_tick: impl AsyncFnMut() -> Result<Option<u64>, E>,
    feed: TickFeed,
    closed: impl FnOnce(),
) {
    loop {
        match next_tick().await {
            Ok(Some(time_ns)) => feed.tick(time_ns),
            Ok(None) => break,
            Err(error) => warn!("a tick of the simulation clock did not decode: {error}"),
        }
    }
    drop(feed);
    closed();
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::Cell;

    use crate::test_support::Logged;

    #[test]
    fn a_simulation_clock_reads_0_then_its_newest_tick() {
        let (clock, feed) = NodeClock::simulation();
        assert_eq!(clock.now_ns(), 0);
        feed.tick(30);
        feed.tick(45);
        assert_eq!(clock.now_ns(), 45);
        assert_eq!(clock.clone().now_ns(), 45, "a clone reads the same clock");
    }

    #[test]
    fn a_step_back_of_a_simulation_clock_is_seen_at_the_next_look_even_once_it_went_forward() {
        let (clock, feed) = NodeClock::simulation();
        feed.tick(100);
        let mut back_steps = clock.back_steps();
        feed.tick(100);
        feed.tick(200);
        assert!(!back_steps.went_back(), "a tick on or after the last one");
        feed.tick(150);
        feed.tick(300);
        assert!(
            back_steps.went_back(),
            "back to 150 and on to 300 between two looks"
        );
        assert!(!back_steps.went_back(), "each step back is seen once");
        feed.tick(10);
        assert!(back_steps.went_back());
    }

    #[test]
    fn the_wall_clock_goes_on_between_two_looks() {
        let mut back_steps = BackSteps(Watched::Wall { reading: 0 });
        assert!(!back_steps.went_back());
        let mut ahead = BackSteps(Watched::Wall { reading: u64::MAX });
        assert!(
            ahead.went_back(),
            "a reading before the one at the last look"
        );
    }

    #[tokio::test]
    async fn a_wait_on_a_simulation_clock_ends_at_the_first_tick_at_its_deadline() {
        let (clock, feed) = NodeClock::simulation();
        let waiting = tokio::spawn({
            let clock = clock.clone();
            async move { clock.until(30).await }
        });
        tokio::task::yield_now().await;
        feed.tick(29);
        tokio::task::yield_now().await;
        assert!(!waiting.is_finished(), "29 ns is before the deadline");
        feed.tick(31);
        waiting.await.expect("the wait ends at the tick of 31 ns");
        clock.until(31).await;
        clock.until(5).await;
    }

    #[tokio::test]
    async fn a_closed_simulation_clock_ends_the_waits_it_reached_and_no_other() {
        let (clock, feed) = NodeClock::simulation();
        feed.tick(5);
        drop(feed);
        clock.until(5).await;
        let waiting = tokio::spawn({
            let clock = clock.clone();
            async move { clock.until(9).await }
        });
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        assert!(!waiting.is_finished(), "a closed clock never reaches 9 ns");
        waiting.abort();
        let ticks = clock.ticks().expect("a simulation clock has ticks");
        assert!(
            ticks.has_changed().is_err(),
            "the ticks of a dropped feed are closed"
        );
    }

    #[tokio::test]
    async fn the_wall_clock_reads_the_machine_and_ends_a_wait_whose_deadline_passed_at_once() {
        let clock = NodeClock::wall();
        let before = wall_now_ns();
        let now = clock.now_ns();
        assert!(now >= before && now > 0);
        clock.until(now).await;
        assert!(clock.ticks().is_none());
    }

    #[tokio::test]
    async fn the_ticks_followed_set_the_clock_a_tick_that_does_not_decode_is_left_out_and_the_end_closes_it()
     {
        let (logged, _guard) = Logged::capture();
        let (clock, feed) = NodeClock::simulation();
        let mut received = vec![
            Ok(Some(1)),
            Err("a tick that does not decode"),
            Ok(Some(2)),
            Ok(None),
            Ok(Some(3)),
        ]
        .into_iter();
        let closes = Cell::new(0);
        follow_ticks(
            async || received.next().expect("a receive past the end"),
            feed,
            || closes.set(closes.get() + 1),
        )
        .await;
        assert_eq!(
            clock.now_ns(),
            2,
            "the tick after the bad one came, and none after the end"
        );
        assert_eq!(closes.get(), 1);
        assert_eq!(logged.lines_holding("did not decode"), 1);
        let ticks = clock.ticks().expect("a simulation clock has ticks");
        assert!(
            ticks.has_changed().is_err(),
            "the end of the ticks closes the clock"
        );
    }
}
