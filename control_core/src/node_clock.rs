//! The clock a node reads: the wall clock, or the simulation clock of the domain the
//! node is bound to. It gives the time now and a wait until a deadline on it, each in
//! nanoseconds since the Unix epoch.
//!
//! A simulation clock reads the ticks of its domain, which the node receives and passes
//! in: [`NodeClock::simulation`] gives the clock and the [`TickFeed`] that sets it, and
//! [`follow_ticks`] feeds it each tick a node's receive gives. Before its first tick a
//! simulation clock reads 0.

use std::fmt::Display;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use tokio::sync::watch;
use tracing::warn;

/// A node's clock: the wall clock or a simulation clock. Clones read one clock.
#[derive(Clone, Debug)]
pub struct NodeClock(Source);

#[derive(Clone, Debug)]
enum Source {
    Wall,
    /// The time of the newest tick of the simulation clock, 0 before the first.
    Simulation(watch::Receiver<u64>),
}

/// What sets a simulation clock: the time of each of its ticks.
#[derive(Debug)]
pub struct TickFeed(watch::Sender<u64>);

impl TickFeed {
    /// Sets the clock to the tick at `time_ns`.
    pub fn tick(&self, time_ns: u64) {
        self.0.send_replace(time_ns);
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
        let (feed, ticks) = watch::channel(0);
        (Self(Source::Simulation(ticks)), TickFeed(feed))
    }

    /// The time now on this clock, in nanoseconds since the Unix epoch.
    pub fn now_ns(&self) -> u64 {
        match &self.0 {
            Source::Wall => wall_now_ns(),
            Source::Simulation(ticks) => *ticks.borrow(),
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
            Source::Simulation(ticks) => {
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
            Source::Simulation(ticks) => Some(ticks.clone()),
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
