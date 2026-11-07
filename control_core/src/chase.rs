//! Setpoint shaping: the rate-limited chase that absorbs target jumps, and the
//! chase along a path that keeps a setpoint on its way to the target.
//!
//! Every component-wise chase is built on [`rate_limited`], so all of them
//! round and bound alike.

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

/// Advance `setpoint` one tick toward `target` along the straight line between
/// them: the whole step when every component's share fits its `max_rate * dt`,
/// else the step scaled down until the most loaded component steps exactly
/// its own, so the setpoint stays on the line.
pub fn chase_along<const N: usize>(
    setpoint: &[f64; N],
    target: &[f64; N],
    max_rate: &[f64; N],
    dt: f64,
) -> [f64; N] {
    let load = (0..N)
        .map(|i| (target[i] - setpoint[i]).abs() / (max_rate[i] * dt))
        .fold(0.0, f64::max);
    if load <= 1.0 {
        return *target;
    }
    std::array::from_fn(|i| setpoint[i] + (target[i] - setpoint[i]) / load)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

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

    // A chase along the line reaches a target within every component's reach,
    // and otherwise stops on the line from the setpoint to the target with its
    // most loaded component stepping its own reach and none past it.
    #[hegel::test]
    fn a_chase_along_the_line_stays_on_it_within_every_rate(tc: TestCase) {
        let span = || gs::floats().min_value(-10.0).max_value(10.0);
        let setpoint: [f64; 7] = tc.draw(gs::arrays(span()));
        let target: [f64; 7] = tc.draw(gs::arrays(span()));
        let max_rate: [f64; 7] = tc.draw(gs::arrays(gs::floats().min_value(0.1).max_value(20.0)));
        let dt: f64 = tc.draw(gs::floats().min_value(0.001).max_value(0.1));
        let next = chase_along(&setpoint, &target, &max_rate, dt);
        if (0..7).all(|i| (target[i] - setpoint[i]).abs() <= max_rate[i] * dt) {
            assert_eq!(next, target);
            return;
        }
        // The share read off the component that moves the most, the one it
        // rounds least.
        let widest = (0..7)
            .max_by(|&a, &b| {
                (target[a] - setpoint[a])
                    .abs()
                    .total_cmp(&(target[b] - setpoint[b]).abs())
            })
            .expect("seven components");
        let share = (next[widest] - setpoint[widest]) / (target[widest] - setpoint[widest]);
        assert!(share > 0.0 && share < 1.0, "a share of {share}");
        for i in 0..7 {
            let on_line = setpoint[i] + share * (target[i] - setpoint[i]);
            assert!(
                (next[i] - on_line).abs() <= 1e-12 * (1.0 + setpoint[i].abs() + target[i].abs())
            );
            // The step, and the rounding of adding it to the setpoint.
            let rounding = f64::EPSILON * (setpoint[i].abs() + target[i].abs());
            assert!((next[i] - setpoint[i]).abs() <= max_rate[i] * dt * (1.0 + 1e-12) + rounding);
        }
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
