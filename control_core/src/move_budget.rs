//! How long a move is allowed to take, and how that changes underneath it.
//!
//! A limb budgets a move once, when the goal is admitted, and then executes it
//! against a speed the operator can retune every tick. The budget is what says
//! a move is stuck and not slow, so it moves with the speed that drives the
//! motion; these three pieces are that one rule.

/// Grace multiple over a move's nominal duration before the runtime declares it
/// stuck and fails the goal. The nominal proves the unobstructed motion's
/// length; the governor can hold motion off that path, so allow this multiple
/// before aborting.
pub const MOTION_TIMEOUT_FACTOR: f64 = 2.0;

/// Whether a move that has run `elapsed_s` has overrun its nominal `budget_s`
/// by more than [`MOTION_TIMEOUT_FACTOR`], the runtime abort condition.
fn motion_timed_out(elapsed_s: f64, budget_s: f64) -> bool {
    elapsed_s > budget_s * MOTION_TIMEOUT_FACTOR
}

/// A move's deadline together with the speed it was computed against: a budget
/// is read against the speed it was written at, so a return to that speed is
/// no slowdown.
#[derive(Debug, Clone, Copy)]
pub struct MoveBudget {
    /// Nominal seconds the move should take; [`MOTION_TIMEOUT_FACTOR`] times
    /// this is the abort threshold.
    seconds: f64,
    /// The speed `seconds` was computed against, in that limb's own unit.
    budgeted_at: f64,
}

impl MoveBudget {
    /// A budget of `seconds`, computed against `rate`.
    pub fn new(seconds: f64, rate: f64) -> Self {
        Self {
            seconds,
            budgeted_at: rate,
        }
    }

    /// The nominal duration, for the operator-facing overrun message.
    pub fn seconds(self) -> f64 {
        self.seconds
    }

    /// Whether a move that has run `elapsed_s` has overrun this budget.
    pub fn timed_out(self, elapsed_s: f64) -> bool {
        motion_timed_out(elapsed_s, self.seconds)
    }

    /// This budget under a retuned `rate`. A limb budgets a move once, at
    /// admission, and then executes it under a speed the operator can change
    /// every tick, such as a gripper's opening rate. Lowering it drives the
    /// motion slower than the deadline it is judged against, which this
    /// extends.
    ///
    /// Only a slowdown moves anything. Speeding up returns `self` untouched,
    /// reference speed included: the move lands sooner anyway, and shrinking a
    /// deadline under a move that has already legitimately spent time at the
    /// old speed would abort it for going fast. The extension scales what is
    /// *left* of the budget, never what is spent, so a move that stops making
    /// progress still runs out of it.
    #[must_use]
    pub fn after_rate_change(self, elapsed_s: f64, rate: f64) -> Self {
        if !(rate.is_finite() && rate > 0.0) || rate >= self.budgeted_at {
            return self;
        }
        Self {
            seconds: elapsed_s + (self.seconds - elapsed_s).max(0.0) * self.budgeted_at / rate,
            budgeted_at: rate,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    // A move times out exactly once it has run past MOTION_TIMEOUT_FACTOR times
    // its nominal budget, whatever that budget is.
    #[hegel::test]
    fn a_move_times_out_exactly_past_its_factor_of_the_budget(tc: TestCase) {
        let budget_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(100.0));
        let elapsed_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(300.0));
        assert_eq!(
            motion_timed_out(elapsed_s, budget_s),
            elapsed_s > budget_s * MOTION_TIMEOUT_FACTOR,
            "{elapsed_s}s into a {budget_s}s budget"
        );
    }

    // A rate change moves a deadline only for a slowdown to a usable rate, and
    // then scales only what is left of it by how much slower the new rate is:
    // what is spent stays spent, so the deadline never shortens, and a move that
    // stops progressing at a steady rate still runs out. A rise, a return to
    // the budgeted rate, and a zero, negative or non-finite rate leave it alone.
    #[hegel::test]
    fn a_rate_change_extends_only_the_rest_of_a_slowed_budget(tc: TestCase) {
        let seconds: f64 = tc.draw(gs::floats().min_value(0.0).max_value(60.0));
        let budgeted_at: f64 = tc.draw(gs::floats().min_value(1e-3).max_value(10.0));
        let elapsed_s: f64 = tc.draw(gs::floats().min_value(0.0).max_value(2.0 * seconds));
        let rate: f64 = tc.draw(gs::floats());
        let budget = MoveBudget::new(seconds, budgeted_at);
        let after = budget.after_rate_change(elapsed_s, rate);
        if rate.is_finite() && rate > 0.0 && rate < budgeted_at {
            let want = elapsed_s + (seconds - elapsed_s).max(0.0) * budgeted_at / rate;
            assert!(
                after.seconds() == want || (after.seconds() - want).abs() <= 1e-9 * want.max(1.0),
                "slowed from {budgeted_at} to {rate} at {elapsed_s}s of {seconds}s: {} s",
                after.seconds()
            );
            assert!(after.seconds() >= seconds - 1e-12, "the deadline shortened");
            assert_eq!(after.budgeted_at, rate);
        } else {
            assert_eq!(after.seconds(), seconds, "rate {rate} moved the deadline");
            assert_eq!(after.budgeted_at, budgeted_at);
        }
    }
}
