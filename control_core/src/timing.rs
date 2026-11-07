//! The time a motion runs on: the clock it plays against and the duration a
//! caller may ask of it.

use std::time::{Duration, Instant};

/// `requested`, or the motion's own `floor_secs` when that is longer: the
/// duration a caller asks a motion to take, held to the shortest it can.
pub fn duration_at_least(floor_secs: f64, requested: Duration) -> Duration {
    requested.max(Duration::from_secs_f64(floor_secs))
}

/// The share of the wall clock a motion plays at over one tick: 0 holds it
/// where it is, 1 plays it in step with the wall clock.
#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub struct Pace(f64);

impl Pace {
    /// In step with the wall clock.
    pub const FULL: Self = Self(1.0);

    /// `fraction`, clamped into `[0, 1]`; a NaN, a share nothing decided,
    /// holds the motion.
    pub fn clamped(fraction: f64) -> Self {
        if fraction.is_nan() {
            return Self(0.0);
        }
        Self(fraction.clamp(0.0, 1.0))
    }

    pub fn get(self) -> f64 {
        self.0
    }
}

/// When a motion started, how long it lasts and how much of it has played:
/// the clock a joint trajectory plays on. Its player may advance its clock each tick
/// ([`JointTrajectory::advance`](crate::path_timing::JointTrajectory::advance)):
/// the motion then plays at the [`Pace`] the player sets, so a motion held
/// back plays on from where it was held, and the wall clock never carries it
/// past a point it did not reach. A clock no player advances plays on the
/// wall clock: the fraction played counts from the motion's start, and
/// [`MotionClock::is_complete`] holds only for a motion of no time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionClock {
    motion_start: Instant,
    duration: Duration,
    /// The time of the motion played by `advanced_at`.
    played: Duration,
    advanced_at: Instant,
}

impl MotionClock {
    /// A motion of `duration` starting at `now`. A motion of no time has
    /// played to its end at its start.
    pub fn starting_at(now: Instant, duration: Duration) -> Self {
        Self {
            motion_start: now,
            duration,
            played: Duration::ZERO,
            advanced_at: now,
        }
    }

    pub fn motion_start(&self) -> Instant {
        self.motion_start
    }

    pub fn duration(&self) -> Duration {
        self.duration
    }

    /// Wall-clock seconds since the motion started at `now`, none before it
    /// starts.
    pub fn elapsed_secs(&self, now: Instant) -> f64 {
        now.duration_since(self.motion_start).as_secs_f64()
    }

    /// The fraction of the motion played at `now` if it plays on in step with
    /// the wall clock from its last advance: what a player proposes for
    /// `now`. At most 1, and 1 throughout a motion that lasts no time.
    pub fn fraction(&self, now: Instant) -> f64 {
        if self.duration.is_zero() {
            return 1.0;
        }
        let proposed = self.played + now.duration_since(self.advanced_at);
        (proposed.as_secs_f64() / self.duration.as_secs_f64()).min(1.0)
    }

    /// Play the motion on to `now` at `pace`: the wall time since the last
    /// advance, scaled by the pace, joins the time played.
    pub fn advance(&mut self, now: Instant, pace: Pace) {
        let step = now.duration_since(self.advanced_at).mul_f64(pace.get());
        self.played = (self.played + step).min(self.duration);
        self.advanced_at = now;
    }

    /// Whether the motion has played to its end.
    pub fn is_complete(&self) -> bool {
        self.played >= self.duration
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    // A duration held to a floor is the longer of the two.
    #[hegel::test]
    fn a_duration_is_held_to_its_floor(tc: TestCase) {
        let floor: f64 = tc.draw(gs::floats().min_value(0.0).max_value(100.0));
        let requested =
            Duration::from_secs_f64(tc.draw(gs::floats().min_value(0.0).max_value(100.0)));
        assert_eq!(
            duration_at_least(floor, requested),
            requested.max(Duration::from_secs_f64(floor))
        );
    }

    // A clock of any duration reads no time elapsed before its start, the
    // fraction of its duration elapsed until its end, 1 and complete exactly
    // from its end on; a clock of no time reads 1 and complete throughout.
    #[hegel::test]
    fn a_motion_clock_reads_the_fraction_of_its_duration_elapsed(tc: TestCase) {
        let duration_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let at_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
        let duration = Duration::from_secs_f64(duration_s);
        let now = Instant::now();
        let clock = MotionClock::starting_at(now, duration);
        let at = now + Duration::from_secs_f64(at_s);
        assert_eq!(clock.elapsed_secs(now - Duration::from_secs(1)), 0.0);
        let mut played = clock;
        played.advance(at, Pace::FULL);
        assert_eq!(played.is_complete(), at >= now + duration);
        if duration.is_zero() {
            assert_eq!(clock.fraction(at), 1.0);
            return;
        }
        let elapsed = at.duration_since(now).as_secs_f64();
        let fraction = clock.fraction(at);
        assert!((fraction - (elapsed / duration.as_secs_f64()).min(1.0)).abs() <= 1e-12);
    }

    // A clock played at full pace reads the fraction of its duration the wall
    // clock has covered, and completes on the first tick at or past its
    // duration; a clock of no time reads 1 and is complete at its start.
    #[hegel::test]
    fn a_clock_at_full_pace_keeps_the_wall_clock(tc: TestCase) {
        let duration_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let ticks_s: Vec<f64> = tc.draw(
            gs::vecs(gs::floats().min_value(0.0).max_value(0.1))
                .min_size(1)
                .max_size(200),
        );
        let duration = Duration::from_secs_f64(duration_s);
        let start = Instant::now();
        let mut clock = MotionClock::starting_at(start, duration);
        assert_eq!(clock.elapsed_secs(start - Duration::from_secs(1)), 0.0);
        if duration.is_zero() {
            assert_eq!(clock.fraction(start), 1.0);
            assert!(clock.is_complete());
            return;
        }
        let mut now = start;
        for tick_s in ticks_s {
            now += Duration::from_secs_f64(tick_s);
            let elapsed = now.duration_since(start).as_secs_f64();
            let fraction = clock.fraction(now);
            assert!((fraction - (elapsed / duration.as_secs_f64()).min(1.0)).abs() <= 1e-12);
            clock.advance(now, Pace::FULL);
            assert_eq!(clock.is_complete(), now >= start + duration);
        }
    }

    // A clock held at pace 0 plays nothing, and one played at a pace plays
    // that share of each tick's wall time: it completes on the first tick
    // whose shares add up to its duration, and its proposal for the next tick
    // runs on from where it was held.
    #[hegel::test]
    fn a_clock_plays_the_share_of_the_wall_clock_its_pace_sets(tc: TestCase) {
        let duration_s: f64 = tc.draw(gs::floats().min_value(0.1).max_value(10.0));
        let pace: f64 = tc.draw(gs::floats().min_value(0.05).max_value(1.0));
        let tick_s: f64 = tc.draw(gs::floats().min_value(0.001).max_value(0.05));
        let start = Instant::now();
        let duration = Duration::from_secs_f64(duration_s);
        let mut clock = MotionClock::starting_at(start, duration);
        let tick = Duration::from_secs_f64(tick_s);
        clock.advance(start + tick, Pace::clamped(0.0));
        assert_eq!(
            clock.fraction(start + tick),
            0.0,
            "a held tick plays nothing"
        );
        let share = tick.mul_f64(Pace::clamped(pace).get());
        let ticks_to_end = duration.as_nanos().div_ceil(share.as_nanos());
        let mut now = start + tick;
        for k in 1..=ticks_to_end {
            assert!(!clock.is_complete(), "complete before tick {k}");
            now += tick;
            clock.advance(now, Pace::clamped(pace));
        }
        assert!(clock.is_complete(), "not complete on tick {ticks_to_end}");
        assert_eq!(clock.fraction(now + tick), 1.0);
    }

    // A pace is a share of the wall clock: clamped into [0, 1], and a NaN
    // holds.
    #[test]
    fn a_pace_is_clamped_into_a_share_and_nan_holds() {
        assert_eq!(Pace::clamped(f64::NAN).get(), 0.0);
        assert_eq!(Pace::clamped(-0.5).get(), 0.0);
        assert_eq!(Pace::clamped(1.5).get(), 1.0);
        assert_eq!(Pace::clamped(0.25).get(), 0.25);
        assert_eq!(Pace::FULL.get(), 1.0);
    }
}
