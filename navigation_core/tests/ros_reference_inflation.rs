//! Inflation against Nav2's inflation layer (nav2_costmap_2d 1.5.2): `Inflation::inflate`
//! ports `InflationLayer::updateCosts`, its single precision distance transform and its
//! cost table, and `PaddedFootprint` ports `padFootprint` and `calculateMinAndMaxDistances`.
//! For each grid of each inflation scenario of the ROS reference harness, whose committed
//! outputs hold the bits of Nav2's amd64 build, the test compares, for the same costs,
//! footprint, padding and parameters: the padded footprint with Nav2's; its inscribed
//! radius with the one Nav2 takes, bit for bit; and, for a scenario whose inflation radius
//! is at least that inscribed radius, the inflated grid with Nav2's, bit for bit. A
//! scenario whose radius lies below Nav2's inscribed radius is refused where the inflation
//! is made, where Nav2 warns and inflates: the test checks that exactly those scenarios,
//! `inflation_inside_inscribed` alone, are refused, and that every other one inflates.
#![cfg(target_arch = "x86_64")]

mod common;

use common::clouds::padded_as_nav2;
use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;
use navigation_core::grid::{CostGrid, GridGeometry};
use navigation_core::inflation::{Inflation, InflationBelowInscribed, InflationParams};
use ros_reference::inflation::Cases;

// The padded footprint is Nav2's, its inscribed radius is Nav2's, and each grid of a radius
// that covers it inflates to Nav2's costs; a radius below it is refused, in exactly the
// scenarios where Nav2 warns, `inflation_inside_inscribed` alone.
#[test]
fn the_inflated_grid_is_nav2s() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds an inflation scenario"
    );
    let mut inflated_grids = 0;
    let mut refused_scenarios = Vec::new();
    for cases in &scenarios {
        let parameters = cases.inflation;
        assert!(
            parameters.enabled
                && !parameters.inflate_unknown
                && !parameters.inflate_around_unknown
                && parameters.custom_inscribed_radius < 0.0,
            "{}: the scenario runs the layer as the port does: enabled, no custom inscribed \
             radius, unknown cells neither inflated nor obstacles",
            cases.name
        );
        let footprint = padded_as_nav2(
            &cases.footprint,
            cases.footprint_padding,
            &cases.padded_footprint,
            &cases.name,
        );
        let inflation = Inflation::parse(
            InflationParams {
                inflation_radius_m: NonNegativeFinite::parse(parameters.inflation_radius)
                    .expect("a radius of at least 0"),
                cost_scaling_factor: NonNegativeFinite::parse(parameters.cost_scaling_factor)
                    .expect("a factor of at least 0"),
            },
            &footprint,
        );
        if inflation.is_err() {
            refused_scenarios.push(cases.name.as_str());
        }
        for (index, expected) in cases.grids.iter().enumerate() {
            let what = format!("{} grid {index}", cases.name);
            assert_eq!(
                footprint.inscribed_radius_m().to_bits(),
                expected.inscribed_radius.to_bits(),
                "{what}: our inscribed radius {}, Nav2's {}",
                footprint.inscribed_radius_m(),
                expected.inscribed_radius
            );
            assert_eq!(
                inflation.is_err(),
                parameters.inflation_radius < expected.inscribed_radius,
                "{what}: the radius {} is refused exactly where it lies below Nav2's inscribed \
                 radius {}, where Nav2 warns",
                parameters.inflation_radius,
                expected.inscribed_radius
            );
            let inflation = match &inflation {
                Ok(inflation) => inflation,
                Err(refused) => {
                    assert_eq!(
                        *refused,
                        InflationBelowInscribed {
                            inflation_radius_m: parameters.inflation_radius,
                            inscribed_radius_m: footprint.inscribed_radius_m(),
                        },
                        "{what}: a radius below the inscribed radius is refused, where Nav2 \
                         warns"
                    );
                    continue;
                }
            };
            let grid = expected.grid;
            let geometry = GridGeometry::new(
                grid.size_x,
                grid.size_y,
                PositiveFinite::parse(grid.resolution).expect("a positive resolution"),
                grid.origin,
            )
            .expect("the scenario's grid");
            let mut ours =
                CostGrid::from_costs(geometry, expected.given.clone()).expect("a cost a cell");
            inflation
                .inflate(&mut ours)
                .expect("the scenario's radius spans cells the table indexes");
            inflated_grids += 1;
            let differing: Vec<(usize, u8, u8)> = ours
                .costs()
                .iter()
                .zip(&expected.inflated)
                .enumerate()
                .filter(|(_, (a, b))| a != b)
                .map(|(i, (&a, &b))| (i, a, b))
                .collect();
            assert!(
                differing.is_empty(),
                "{what}: {} cells differ from Nav2's, the first (index, ours, Nav2's): {:?}",
                differing.len(),
                &differing[..differing.len().min(5)]
            );
        }
    }
    assert_eq!(
        refused_scenarios,
        ["inflation_inside_inscribed"],
        "the scenario whose radius lies inside the inscribed radius is refused, and every \
         other inflates"
    );
    assert!(inflated_grids > 0, "a scenario's grid inflates");
}
