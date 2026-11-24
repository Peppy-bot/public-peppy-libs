//! The refusal of values of which one or more is not finite, each named.

/// Values of which one or more is not finite: what they make, each value by
/// its name, and what to give.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error("{what} must be finite: got {}; give {give}", named(.values))]
pub struct NotFinite<const N: usize> {
    pub what: &'static str,
    pub values: [(&'static str, f64); N],
    pub give: &'static str,
}

impl<const N: usize> NotFinite<N> {
    /// Nothing when every one of `values` is finite; else the refusal of
    /// `what`, naming each value and saying to give `give`.
    pub fn check(
        what: &'static str,
        values: [(&'static str, f64); N],
        give: &'static str,
    ) -> Result<(), Self> {
        if values.iter().all(|(_, value)| value.is_finite()) {
            return Ok(());
        }
        Err(Self { what, values, give })
    }
}

/// Each value as its name and its number, as a refusal lists them: `x 1, y NaN`.
fn named(values: &[(&'static str, f64)]) -> String {
    let named: Vec<String> = values
        .iter()
        .map(|(name, value)| format!("{name} {value}"))
        .collect();
    named.join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    // Finite values pass; a value that is not finite, on any one of them, is refused,
    // naming each value and what to give.
    #[test]
    fn a_value_that_is_not_finite_is_refused_naming_each() {
        assert_eq!(
            NotFinite::check("a pair", [("a", 1.0), ("b", -2.0)], "two"),
            Ok(())
        );
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let refused = NotFinite::check("a pair", [("a", 1.0), ("b", bad)], "two").unwrap_err();
            assert_eq!(refused.values[1].1.to_bits(), bad.to_bits());
            assert_eq!(
                refused.to_string(),
                format!("a pair must be finite: got a 1, b {bad}; give two")
            );
        }
    }
}
