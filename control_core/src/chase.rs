//! Setpoint shaping: the rate-limited chase that absorbs target jumps, and
//! the guard that reads a step against the rate it was allowed.
//!
//! Every chase is built on [`rate_limited`], so all of them round and bound
//! alike.

/// Advance `from` one tick toward `target`, moving at most `rate * dt`, so a
/// target within reach is reached.
///
/// `rate` and `dt` must be finite and non-negative: the clamp bounds are
/// `[-rate * dt, rate * dt]` and `f64::clamp` panics if they invert, so an
/// unvalidated rate fails loudly at the call site.
pub fn rate_limited(from: f64, target: f64, rate: f64, dt: f64) -> f64 {
    let max_step = rate * dt;
    from + (target - from).clamp(-max_step, max_step)
}

/// Advance `setpoint` one tick toward `target`, each component stepping at
/// most its own `max_rate * dt`: the rate-limited chase of a whole vector.
pub fn chase_step<const N: usize>(
    setpoint: &[f64; N],
    target: &[f64; N],
    max_rate: &[f64; N],
    dt: f64,
) -> [f64; N] {
    std::array::from_fn(|i| rate_limited(setpoint[i], target[i], max_rate[i], dt))
}

/// Whether stepping from `from` to `to` over `dt` moves any component faster
/// than its `max_rate`.
pub fn exceeds_rate<const N: usize>(
    from: &[f64; N],
    to: &[f64; N],
    max_rate: &[f64; N],
    dt: f64,
) -> bool {
    (0..N).any(|i| (to[i] - from[i]).abs() > max_rate[i] * dt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    // A step exceeds its rates exactly when some component moved further than
    // its rate over the tick allows.
    #[hegel::test]
    fn a_step_exceeds_its_rates_exactly_when_a_component_outruns_its_own(tc: TestCase) {
        let span = || gs::floats().min_value(-10.0).max_value(10.0);
        let from: [f64; 3] = tc.draw(gs::arrays(span()));
        let to: [f64; 3] = tc.draw(gs::arrays(span()));
        let max_rate: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(0.0).max_value(20.0)));
        let dt: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.1));
        let outrun = (0..3).any(|i| (to[i] - from[i]).abs() > max_rate[i] * dt);
        assert_eq!(exceeds_rate(&from, &to, &max_rate, dt), outrun);
    }

    // One tick moves at most `rate * dt`, toward the target, reaches a target
    // within reach to the rounding of `from + (target - from)`, and holds
    // bit-exact on a target it is already at.
    #[hegel::test]
    fn a_rate_limited_step_moves_at_most_its_rate_toward_the_target(tc: TestCase) {
        let span = || gs::floats().min_value(-10.0).max_value(10.0);
        let from: f64 = tc.draw(span());
        let target: f64 = tc.draw(span());
        let rate: f64 = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
        let dt: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.1));
        let next = rate_limited(from, target, rate, dt);
        let reach = rate * dt;
        if (target - from).abs() <= reach {
            assert!(
                (next - target).abs() <= 2.0 * f64::EPSILON * from.abs().max(target.abs()),
                "{from} -> {target} within {reach} reaches {next}"
            );
        } else {
            assert!((next - from).abs() <= reach * (1.0 + 1e-12) + f64::EPSILON * from.abs());
            assert!(
                from.min(target) <= next && next <= from.max(target),
                "{from} -> {next} leaves the way to {target}"
            );
        }
        assert_eq!(rate_limited(target, target, rate, dt), target);
    }

    // The vector chase applies that limit component by component, each at its
    // own rate.
    #[hegel::test]
    fn a_chase_step_rate_limits_every_component_at_its_own_rate(tc: TestCase) {
        let span = || gs::floats().min_value(-10.0).max_value(10.0);
        let setpoint: [f64; 7] = tc.draw(gs::arrays(span()));
        let target: [f64; 7] = tc.draw(gs::arrays(span()));
        let max_rate: [f64; 7] = tc.draw(gs::arrays(gs::floats().min_value(0.0).max_value(20.0)));
        let dt: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.1));
        let next = chase_step(&setpoint, &target, &max_rate, dt);
        for i in 0..7 {
            assert_eq!(
                next[i],
                rate_limited(setpoint[i], target[i], max_rate[i], dt),
                "component {i}"
            );
        }
    }
}
