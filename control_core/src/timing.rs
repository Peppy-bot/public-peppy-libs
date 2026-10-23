//! The time a motion runs on: the clock it plays against, the duration a
//! caller may ask of it, the control ticks that duration is commanded at, and
//! the check that what those ticks command moves and accelerates within the
//! limits.

use std::time::{Duration, Instant};

/// `requested`, or the motion's own `floor_secs` when that is longer: the
/// duration a caller asks a motion to take, held to the shortest it can.
pub fn duration_at_least(floor_secs: f64, requested: Duration) -> Duration {
    requested.max(Duration::from_secs_f64(floor_secs))
}

fn sane_secs(secs: f64) -> bool {
    secs.is_finite() && secs >= 0.0
}

/// The largest of `ratios`, or NaN when any of them is: `f64::max` drops a
/// NaN operand, and a NaN ratio is a budget to refuse.
fn largest(ratios: impl Iterator<Item = f64>) -> f64 {
    ratios.fold(0.0, |worst, ratio| {
        if worst.is_nan() || ratio.is_nan() {
            f64::NAN
        } else {
            worst.max(ratio)
        }
    })
}

/// The fraction of a motion of `duration_secs` elapsed at each control tick,
/// one `period_secs` apart from its start: every `k * period / duration`
/// under 1, then 1 for the first tick at or past the end, which is taken as
/// the end itself. At least the two fractions 0 and 1, so a motion of no time
/// is its start and its end.
pub fn tick_fractions(duration_secs: f64, period_secs: f64) -> impl Iterator<Item = f64> {
    assert!(period_secs > 0.0, "a control period is positive");
    assert!(
        sane_secs(duration_secs),
        "a duration is finite and non-negative"
    );
    let fraction = move |k: usize| {
        if duration_secs > 0.0 {
            k as f64 * period_secs / duration_secs
        } else {
            k as f64
        }
    };
    (0..)
        .map(fraction)
        .take_while(|&fraction| fraction < 1.0)
        .chain(std::iter::once(1.0))
}

/// The largest per-component ratio, over `ticks`, the configurations a motion
/// commands one control `period_secs` apart, of the velocity from one tick to
/// the next to that component's velocity limit and of the velocity change from
/// one tick to the next to its acceleration limit. At most 1 means every
/// component moves and accelerates within its limits as the motion is
/// commanded; NaN means a tick or a limit is.
fn peak_tick_ratio<const N: usize>(
    ticks: impl IntoIterator<Item = [f64; N]>,
    period_secs: f64,
    max_velocity: &[f64; N],
    max_acceleration: &[f64; N],
) -> f64 {
    let ticks: Vec<[f64; N]> = ticks.into_iter().collect();
    let velocity = ticks.windows(2).flat_map(|w| {
        (0..N).map(move |i| (w[1][i] - w[0][i]).abs() / period_secs / max_velocity[i])
    });
    let acceleration = ticks.windows(3).flat_map(|w| {
        (0..N).map(move |i| {
            (w[2][i] - 2.0 * w[1][i] + w[0][i]).abs()
                / (period_secs * period_secs)
                / max_acceleration[i]
        })
    });
    largest(velocity.chain(acceleration))
}

/// The shortest duration, at least `floor_secs`, at most `ceiling_secs` and
/// to within one control `period_secs`, over which a motion moves and
/// accelerates every component within its `max_velocity` and
/// `max_acceleration` tick by tick, or `None` when it does not at the ceiling.
/// `ticks_at(duration)` is the configurations the motion commands one period
/// apart when it lasts `duration`, from its start to its end (see
/// [`tick_fractions`]), or `None` when the motion cannot be commanded over that
/// duration, which fails it; the motion rests before it starts and after it
/// ends, so its first and last configurations are each taken twice.
///
/// The peak ratio of velocity and acceleration to their limits over the ticks
/// falls as the motion slows, so the duration doubles from the floor, up to
/// the ceiling, until a motion passes, then halves the gap to the last one
/// that failed until the two lie within a period. The answer is always a
/// duration that passed.
pub fn tick_checked_duration<const N: usize, I: IntoIterator<Item = [f64; N]>>(
    floor_secs: f64,
    ceiling_secs: f64,
    period_secs: f64,
    max_velocity: &[f64; N],
    max_acceleration: &[f64; N],
    ticks_at: impl Fn(f64) -> Option<I>,
) -> Option<f64> {
    assert!(period_secs > 0.0, "a control period is positive");
    assert!(
        sane_secs(floor_secs) && floor_secs <= ceiling_secs,
        "the floor is finite, non-negative and at most the ceiling"
    );
    let passes = |secs: f64| {
        let Some(ticks) = ticks_at(secs) else {
            return false;
        };
        let ticks: Vec<[f64; N]> = ticks.into_iter().collect();
        assert!(
            ticks.len() >= 2,
            "a motion's ticks run from its start to its end"
        );
        let (first, last) = (ticks[0], ticks[ticks.len() - 1]);
        let at_rest = std::iter::once(first)
            .chain(ticks)
            .chain(std::iter::once(last));
        peak_tick_ratio(at_rest, period_secs, max_velocity, max_acceleration) <= 1.0
    };
    if passes(floor_secs) {
        return Some(floor_secs);
    }
    let (mut failed, mut passed) = (
        floor_secs,
        (floor_secs.max(period_secs) * 2.0).min(ceiling_secs),
    );
    while !passes(passed) {
        if passed >= ceiling_secs {
            return None;
        }
        (failed, passed) = (passed, (passed * 2.0).min(ceiling_secs));
    }
    while passed - failed > period_secs {
        let middle = (failed + passed) / 2.0;
        if passes(middle) {
            passed = middle;
        } else {
            failed = middle;
        }
    }
    Some(passed)
}

/// When a motion started and how long it lasts: the clock a joint move, a
/// line and a servo path all play on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionClock {
    motion_start: Instant,
    duration: Duration,
}

impl MotionClock {
    /// A motion of `duration` starting at `now`.
    pub fn starting_at(now: Instant, duration: Duration) -> Self {
        Self {
            motion_start: now,
            duration,
        }
    }

    pub fn motion_start(&self) -> Instant {
        self.motion_start
    }

    pub fn duration(&self) -> Duration {
        self.duration
    }

    /// Seconds since the motion started at `now`, none before it starts.
    pub fn elapsed_secs(&self, now: Instant) -> f64 {
        now.duration_since(self.motion_start).as_secs_f64()
    }

    /// The fraction of the motion elapsed at `now`: 0 before it starts, 1
    /// once it is complete, and 1 throughout a motion that lasts no time.
    pub fn fraction(&self, now: Instant) -> f64 {
        if self.duration.is_zero() {
            return 1.0;
        }
        (self.elapsed_secs(now) / self.duration.as_secs_f64()).min(1.0)
    }

    pub fn is_complete(&self, now: Instant) -> bool {
        now.duration_since(self.motion_start) >= self.duration
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path_timing::TimedPath;
    use hegel::TestCase;
    use hegel::generators as gs;
    use std::f64::consts::PI;

    const JOINTS: usize = 7;

    fn joint_vectors() -> gs::ArrayGenerator<gs::FloatGenerator<f64>, f64, JOINTS> {
        gs::arrays(gs::floats().min_value(-PI).max_value(PI))
    }

    fn limits() -> gs::ArrayGenerator<gs::FloatGenerator<f64>, f64, JOINTS> {
        gs::arrays(gs::floats().min_value(0.1).max_value(20.0))
    }

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

    // The tick fractions of any motion start at 0, end at 1, rise by at most
    // one period's share of the duration, reach the end within one share of
    // the last fraction before it, and are two for a motion of no time.
    #[hegel::test]
    fn the_tick_fractions_run_from_the_start_to_the_end(tc: TestCase) {
        let duration: f64 = tc.draw(gs::floats().min_value(0.0).max_value(100.0));
        let period: f64 = tc.draw(gs::floats().min_value(0.001).max_value(0.05));
        let fractions: Vec<f64> = tick_fractions(duration, period).collect();
        assert_eq!(fractions[0], 0.0);
        assert_eq!(fractions[fractions.len() - 1], 1.0);
        let share = if duration > 0.0 {
            period / duration
        } else {
            1.0
        };
        // Each fraction is its own quotient, rounded within an ulp of 1.
        for (k, w) in fractions.windows(2).enumerate() {
            assert!(w[1] > w[0], "fractions {k} and {} do not rise", k + 1);
            assert!(
                w[1] - w[0] <= share + 4.0 * f64::EPSILON,
                "fractions {k} and {} are {} apart, over a period's {share}",
                k + 1,
                w[1] - w[0]
            );
        }
        assert!(fractions[fractions.len() - 2] + share + 4.0 * f64::EPSILON >= 1.0);
        if duration == 0.0 {
            assert_eq!(fractions, [0.0, 1.0]);
        }
    }

    /// A timed straight move played over `secs`, as the configurations it
    /// commands one `period` apart from its start to its end.
    fn straight_ticks(path: &TimedPath, secs: f64, period: f64) -> Vec<[f64; JOINTS]> {
        tick_fractions(secs, period)
            .map(|fraction| {
                path.at_fraction(fraction)
                    .try_into()
                    .expect("a joint path has a configuration's components")
            })
            .collect()
    }

    // A straight move timed at its velocity and acceleration limits passes
    // the tick check at its own duration, since a difference averages the
    // velocity it spans and a second difference the acceleration, to the
    // rounding the check magnifies; and the tick-checked duration of the same
    // move, from any floor, passes, is at least the floor, lies no more than a
    // period above the path's own, and fails a period shorter unless it is the
    // floor; refused only when even the ceiling fails. A move of no time is a
    // jump: it passes when it moves nothing and fails when it moves a joint
    // further than a limit covers in one period, or one period squared.
    #[hegel::test]
    fn a_tick_checked_duration_is_the_shortest_that_passes_to_a_period(tc: TestCase) {
        let start: [f64; JOINTS] = tc.draw(joint_vectors());
        let end: [f64; JOINTS] = tc.draw(joint_vectors());
        let velocity: [f64; JOINTS] = tc.draw(limits());
        let accel: [f64; JOINTS] = tc.draw(limits());
        let floor: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let period: f64 = tc.draw(gs::floats().min_value(0.001).max_value(0.05));
        let path = TimedPath::new(&[start.to_vec(), end.to_vec()], &velocity, &accel);
        let closed = path.duration_s();
        let padded = |secs: f64| {
            let ticks = straight_ticks(&path, secs, period);
            let (first, last) = (ticks[0], ticks[ticks.len() - 1]);
            std::iter::once(first)
                .chain(ticks)
                .chain(std::iter::once(last))
                .collect::<Vec<_>>()
        };
        // A second difference over a millisecond magnifies the rounding of the
        // positions it is taken from a millionfold, so the closed form, which
        // sits exactly at the limit, passes to a part in a million.
        let ratio = |secs: f64| peak_tick_ratio(padded(secs), period, &velocity, &accel);
        let passes = |secs: f64| ratio(secs) <= 1.0;
        assert!(
            ratio(closed) <= 1.0 + 1e-6,
            "the closed form's {closed}s reads {} of the limit tick by tick",
            ratio(closed)
        );
        if start == end {
            assert!(passes(0.0), "a jump that moves nothing fails");
        }
        let travel_over = |limits: &[f64; JOINTS]| {
            largest((0..JOINTS).map(|i| (end[i] - start[i]).abs() / limits[i]))
        };
        if travel_over(&velocity) > period || travel_over(&accel) > period * period {
            assert!(
                !passes(0.0),
                "a jump further than a limit covers in one period, or one period squared, passes"
            );
        }
        let ceiling: f64 = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
        let checked =
            tick_checked_duration(floor, floor + ceiling, period, &velocity, &accel, |secs| {
                Some(straight_ticks(&path, secs, period))
            });
        let Some(checked) = checked else {
            assert!(
                !passes(floor + ceiling),
                "refused although the ceiling passes"
            );
            return;
        };
        assert!(checked >= floor, "{checked}s under the {floor}s floor");
        assert!(passes(checked), "{checked}s fails its own check");
        // The path's own duration passes or fails its tick check by rounding,
        // so the shortest passing duration lies within a period of it either
        // way, to that rounding.
        assert!(
            checked <= floor.max(closed) * (1.0 + 1e-9) + period,
            "{checked}s against the path's own {closed}s and the {floor}s floor"
        );
        if checked > floor + period {
            assert!(
                !passes(checked - period),
                "{checked}s is not the shortest to a period"
            );
        }
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
        assert_eq!(clock.is_complete(at), at >= now + duration);
        if duration.is_zero() {
            assert_eq!(clock.fraction(at), 1.0);
            return;
        }
        let elapsed = at.duration_since(now).as_secs_f64();
        let fraction = clock.fraction(at);
        assert!((fraction - (elapsed / duration.as_secs_f64()).min(1.0)).abs() <= 1e-12);
    }
}
