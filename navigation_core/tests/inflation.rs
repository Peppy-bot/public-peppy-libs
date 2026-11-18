//! Properties of the inflation on drawn grids: each cell's inflated cost is the inflation
//! formula at the distance to the nearest lethal cell, rounded to the cost table's step, and
//! falls with that distance.

mod common;

use common::footprints::padded;
use common::properties::grids;
use control_core::non_negative_finite::NonNegativeFinite;
use hegel::TestCase;
use hegel::generators as gs;
use navigation_core::grid::{CostGrid, INSCRIBED, LETHAL, UNKNOWN};
use navigation_core::inflation::{Inflation, InflationParams};

/// The cost table's entries a cell of distance, Nav2's `COST_LUT_PRECISION`.
const TABLE_STEPS_PER_CELL: f64 = 100.0;

/// How near a step of the table a distance may lie for either neighboring step to give
/// its cost: the transform takes a cell's distance in single precision.
const STEP_SLACK: f64 = 1e-4;

/// The cost the inflation formula gives a cell `distance` cells from the nearest lethal
/// one: lethal on it, inscribed within the inscribed radius, and beyond it
/// `252 * exp(-k * (d - r))` truncated, at `d` meters.
fn formula(distance: f64, resolution: f64, inscribed: f64, scaling: f64) -> u8 {
    if distance == 0.0 {
        return LETHAL;
    }
    let meters = distance * resolution;
    if meters <= inscribed {
        return INSCRIBED;
    }
    (252.0 * (-scaling * (meters - inscribed)).exp()) as u8
}

/// The distance in cells from `index` to the nearest lethal cell of `grid`, by checking
/// every cell; none when the grid holds no lethal cell.
fn nearest_lethal(grid: &CostGrid, index: usize) -> Option<f64> {
    let size_x = grid.geometry().size_x() as usize;
    let (x, y) = ((index % size_x) as f64, (index / size_x) as f64);
    grid.costs()
        .iter()
        .enumerate()
        .filter(|(_, cost)| **cost == LETHAL)
        .map(|(i, _)| ((i % size_x) as f64 - x).hypot((i / size_x) as f64 - y))
        .min_by(f64::total_cmp)
}

// Every cell within the inflation radius of a lethal cell takes the formula's cost at its
// distance rounded to the table's step, or keeps a higher cost of its own, an unknown cell
// taking it only at the inscribed cost or above; every other cell keeps its cost; and the
// cost falls with the distance.
#[hegel::test(test_cases = 200)]
fn inflation_cost_falls_with_distance_by_the_formula(tc: TestCase) {
    let given = tc.draw(grids(12));
    let half_width = tc.draw(gs::floats().min_value(0.05).max_value(0.6));
    let footprint = padded(
        &[
            [-half_width, -half_width],
            [half_width, -half_width],
            [half_width, half_width],
            [-half_width, half_width],
        ],
        0.0,
    );
    let radius =
        footprint.inscribed_radius_m() + tc.draw(gs::floats().min_value(0.0).max_value(3.0));
    let scaling = tc.draw(gs::floats().min_value(0.0).max_value(20.0));
    let inflation = Inflation::parse(
        InflationParams {
            inflation_radius_m: NonNegativeFinite::parse(radius).expect("drawn at least 0"),
            cost_scaling_factor: NonNegativeFinite::parse(scaling).expect("drawn at least 0"),
        },
        &footprint,
    )
    .expect("a radius drawn from the inscribed radius on");
    let mut inflated = given.clone();
    inflation
        .inflate(&mut inflated)
        .expect("a drawn radius spans cells the table indexes");
    let resolution = given.geometry().resolution_m();
    let cell_radius = (radius / resolution).ceil();
    let inscribed = footprint.inscribed_radius_m();
    let mut by_distance: Vec<(f64, u8)> = Vec::new();
    for (index, (&old, &new)) in given.costs().iter().zip(inflated.costs()).enumerate() {
        let Some(distance) =
            nearest_lethal(&given, index).filter(|&d| cell_radius > 0.0 && d <= cell_radius)
        else {
            assert_eq!(new, old, "cell {index}, beyond the radius, keeps its cost");
            continue;
        };
        let steps = distance * TABLE_STEPS_PER_CELL + 0.5;
        let allowed: Vec<u8> = [steps - STEP_SLACK, steps + STEP_SLACK]
            .iter()
            .map(|s| {
                formula(
                    s.floor() / TABLE_STEPS_PER_CELL,
                    resolution,
                    inscribed,
                    scaling,
                )
            })
            .map(|cost| match old {
                UNKNOWN if cost < INSCRIBED => UNKNOWN,
                UNKNOWN => cost,
                _ => old.max(cost),
            })
            .collect();
        assert!(
            allowed.contains(&new),
            "cell {index}, {distance} cells from a lethal cell, of cost {old}: inflated to \
             {new}, the formula gives {allowed:?}"
        );
        if old == 0 {
            by_distance.push((distance, new));
        }
    }
    by_distance.sort_by(|a, b| a.0.total_cmp(&b.0));
    for pair in by_distance.windows(2) {
        let ((near, near_cost), (far, far_cost)) = (pair[0], pair[1]);
        assert!(
            near_cost >= far_cost || far - near < STEP_SLACK,
            "a free cell {near} cells away inflates to {near_cost}, one {far} away to {far_cost}"
        );
    }
}
