//! The finite numbers the suites build their parameters from.

use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;
use navigation_core::planner::CostTravelMultiplier;

/// `value`, a finite number of at least 0.
pub fn non_negative(value: f64) -> NonNegativeFinite {
    NonNegativeFinite::parse(value).expect("a finite number of at least 0")
}

/// `value`, a finite number above 0.
pub fn positive(value: f64) -> PositiveFinite {
    PositiveFinite::parse(value).expect("a finite number above 0")
}

/// `value`, a `cost_travel_multiplier` finite in single precision.
pub fn multiplier(value: f64) -> CostTravelMultiplier {
    CostTravelMultiplier::parse(value).expect("a multiplier finite in single precision")
}
