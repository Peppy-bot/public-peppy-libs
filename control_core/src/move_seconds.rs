//! The seconds a move goal asks for: a duration or a settle timeout, from 0
//! to [`MAX_MOVE_DURATION_S`], the bound the goal's contract states, parsed off
//! the wire, each refusal naming the field, the bound and the value.

use std::time::Duration;

use crate::non_negative_finite::NonNegativeFinite;
use crate::positive_finite::PositiveFinite;

/// The longest a move goal may ask a move to take, and to settle after its
/// planned end (s): the bound limb_motion and base_motion state for every
/// duration and timeout.
pub const MAX_MOVE_DURATION_S: f64 = 60.0;

/// A number of seconds a goal asks of a move, at most [`MAX_MOVE_DURATION_S`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoveSeconds(f64);

impl MoveSeconds {
    /// The seconds of the field `field` off the wire, such as `duration_s` or
    /// `settle_timeout_s`: from 0 to [`MAX_MOVE_DURATION_S`], or the refusal
    /// naming the field, the bound and the value.
    pub fn from_wire(field: &str, value: f64) -> Result<Self, String> {
        NonNegativeFinite::parse(value)
            .ok()
            .map(NonNegativeFinite::get)
            .filter(|seconds| *seconds <= MAX_MOVE_DURATION_S)
            .map(Self)
            .ok_or_else(|| {
                format!(
                    "{field} must be a number of seconds from 0 to {MAX_MOVE_DURATION_S:.0}, got {value}"
                )
            })
    }

    /// The seconds of the field `field` off the wire, such as a hold's
    /// `hold_duration_s`: above 0 and at most [`MAX_MOVE_DURATION_S`], or the
    /// refusal naming the field, the bound and the value.
    pub fn positive_from_wire(field: &str, value: f64) -> Result<Self, String> {
        PositiveFinite::parse(value)
            .ok()
            .map(PositiveFinite::get)
            .filter(|seconds| *seconds <= MAX_MOVE_DURATION_S)
            .map(Self)
            .ok_or_else(|| {
                format!(
                    "{field} must be a number of seconds above 0 and at most {MAX_MOVE_DURATION_S:.0}, got {value}"
                )
            })
    }

    /// The seconds, as they came.
    pub fn get(self) -> f64 {
        self.0
    }

    /// The seconds as a duration.
    pub fn duration(self) -> Duration {
        Duration::from_secs_f64(self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use hegel::TestCase;
    use hegel::generators as gs;

    // Seconds off the wire are taken exactly when finite from 0 to the bound,
    // and above 0 for a positive field; each is kept as it came.
    #[hegel::test]
    fn seconds_are_taken_exactly_in_their_range(tc: TestCase) {
        let value: f64 = tc.draw(gs::floats());
        let in_seconds = value.is_finite() && (0.0..=MAX_MOVE_DURATION_S).contains(&value);
        let kept =
            |parsed: Result<MoveSeconds, String>| parsed.is_ok_and(|taken| taken.get() == value);
        assert_eq!(
            kept(MoveSeconds::from_wire("settle_timeout_s", value)),
            in_seconds,
            "{value}"
        );
        assert_eq!(
            kept(MoveSeconds::positive_from_wire("hold_duration_s", value)),
            in_seconds && value > 0.0,
            "{value}"
        );
    }

    // A refusal names the field, the range and the value it got, the bound the 60 s that
    // limb_motion and base_motion state.
    #[test]
    fn a_refusal_names_the_field_the_range_and_the_value() {
        assert_eq!(
            MoveSeconds::from_wire("settle_timeout_s", 61.0),
            Err("settle_timeout_s must be a number of seconds from 0 to 60, got 61".to_string())
        );
        assert_eq!(
            MoveSeconds::positive_from_wire("hold_duration_s", 0.0),
            Err(
                "hold_duration_s must be a number of seconds above 0 and at most 60, got 0"
                    .to_string()
            )
        );
        assert_eq!(
            MoveSeconds::from_wire("duration_s", 1.5).map(MoveSeconds::duration),
            Ok(Duration::from_millis(1500))
        );
    }
}
