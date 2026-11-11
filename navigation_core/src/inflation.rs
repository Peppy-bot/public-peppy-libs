//! Inflation as Nav2 1.5.2's inflation layer inflates a grid
//! (nav2_costmap_2d plugins/inflation_layer.cpp `InflationLayer::updateCosts`,
//! `applyInflation`, `computeCaches`, and `computeCost` of
//! include/nav2_costmap_2d/inflation_layer.hpp), over the whole grid: the
//! distance from each cell to the nearest lethal cell by the single
//! precision Felzenszwalb-Huttenlocher transform of
//! include/nav2_costmap_2d/distance_transform.hpp
//! (`DistanceTransform::distanceTransform2D`), columns then rows, and the
//! cost of each cell within the inflation radius read from a table of
//! `COST_LUT_PRECISION` entries a cell. A cell's cost is lethal on an
//! obstacle, inscribed within the footprint's inscribed radius of one, and
//! beyond it `252 * exp(-cost_scaling_factor * (d - inscribed radius))` at a
//! distance `d` in meters, the table's distance, truncated. A cell keeps the
//! higher of its cost and the inflated one, and an unknown cell takes the
//! inflated one only at the inscribed cost or above (`inflate_unknown`
//! false); only lethal cells are obstacles (`inflate_around_unknown` false).
//!
//! [`Inflation::parse`] makes the inflation of a base's padded footprint and
//! refuses an inflation radius below the footprint's inscribed radius, 0
//! among them: a cell within the inscribed radius of an obstacle and beyond
//! the inflation radius keeps a cost below the inscribed cost, which the
//! planner reads as clear for the base's center though the base stands on the
//! obstacle there. Nav2 warns on that radius and inflates
//! (plugins/inflation_layer.cpp:220-228); the library refuses it, by the same
//! comparison, `inflation_radius_ < inscribed_radius_`, so a radius equal to
//! the inscribed radius inflates. A grid that takes no inflation is composed
//! with none (`crate::costmap::compose`).

use control_core::non_negative_finite::NonNegativeFinite;

use crate::footprint::PaddedFootprint;
use crate::grid::{CostGrid, INSCRIBED, LETHAL, UNKNOWN};

/// The table's entries a cell of distance (`COST_LUT_PRECISION`).
const COST_LUT_PRECISION: u32 = 100;

/// The most cells an inflation radius spans: the table's last index, the
/// radius in cells times `COST_LUT_PRECISION` plus one, is an unsigned
/// 32-bit integer, as in Nav2 (`computeCaches`).
const MAX_INFLATION_RADIUS_CELLS: u32 = (u32::MAX - 1) / COST_LUT_PRECISION;

/// Why a grid takes no inflation: the radius spans more cells than the table
/// indexes.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "inflation_radius {inflation_radius_m} m spans more than {} cells of {resolution_m} m, \
     beyond which Nav2's cost table overflows its index: give at most {} m",
    MAX_INFLATION_RADIUS_CELLS,
    f64::from(MAX_INFLATION_RADIUS_CELLS) * .resolution_m
)]
pub struct InflationError {
    pub inflation_radius_m: f64,
    pub resolution_m: f64,
}

/// The distance a free cell starts the transform at (`DistanceTransform::DT_INF`).
const DT_INF: f32 = f32::MAX;

/// An inflation radius below the padded footprint's inscribed radius.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "inflation_radius_m {inflation_radius_m} m lies below the padded footprint's inscribed \
     radius, {inscribed_radius_m} m, so a cell within that radius of an obstacle keeps a cost \
     below the inscribed cost: give inflation_radius_m of at least {inscribed_radius_m} m"
)]
pub struct InflationBelowInscribed {
    pub inflation_radius_m: f64,
    pub inscribed_radius_m: f64,
}

/// The inflation layer's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InflationParams {
    /// How far from an obstacle cells take a cost, in meters
    /// (`inflation_radius`).
    pub inflation_radius_m: NonNegativeFinite,
    /// How fast the cost falls beyond the inscribed radius, per meter
    /// (`cost_scaling_factor`).
    pub cost_scaling_factor: NonNegativeFinite,
}

/// The inflation layer of a base's padded footprint: its settings, the radius
/// at least the footprint's inscribed radius, and that footprint.
#[derive(Debug, Clone, PartialEq)]
pub struct Inflation {
    params: InflationParams,
    footprint: PaddedFootprint,
}

impl Inflation {
    /// The inflation of `params` for a base of `footprint`; a radius below the
    /// footprint's inscribed radius is refused, naming the least radius to
    /// give.
    pub fn parse(
        params: InflationParams,
        footprint: &PaddedFootprint,
    ) -> Result<Self, InflationBelowInscribed> {
        let inscribed_radius_m = footprint.inscribed_radius_m();
        let inflation_radius_m = params.inflation_radius_m.get();
        if inflation_radius_m < inscribed_radius_m {
            return Err(InflationBelowInscribed {
                inflation_radius_m,
                inscribed_radius_m,
            });
        }
        Ok(Self {
            params,
            footprint: footprint.clone(),
        })
    }

    /// Inflates `grid` around its lethal cells.
    pub fn inflate(&self, grid: &mut CostGrid) -> Result<(), InflationError> {
        let geometry = grid.geometry();
        let resolution = geometry.resolution_m();
        let inflation_radius_m = self.params.inflation_radius_m.get();
        let cell_inflation_radius = cell_distance(inflation_radius_m, resolution);
        if cell_inflation_radius > MAX_INFLATION_RADIUS_CELLS {
            return Err(InflationError {
                inflation_radius_m,
                resolution_m: resolution,
            });
        }
        if cell_inflation_radius == 0 {
            return Ok(());
        }
        let lut = self.cost_table(cell_inflation_radius, resolution);
        let (width, height) = (geometry.size_x() as usize, geometry.size_y() as usize);
        let mut distance_map: Vec<f32> = grid
            .costs()
            .iter()
            .map(|&cost| if cost != LETHAL { DT_INF } else { 0.0 })
            .collect();
        distance_transform_2d(&mut distance_map, height, width);
        let cell_inflation_radius_f = cell_inflation_radius as f32;
        let lut_max = (lut.len() - 1) as u32;
        let lut_precision = COST_LUT_PRECISION as f32;
        for (old_cost, &distance_cells) in grid.costs_mut().iter_mut().zip(&distance_map) {
            if distance_cells > cell_inflation_radius_f {
                continue;
            }
            let d_scaled = lut_max.min((distance_cells * lut_precision + 0.5) as u32);
            let cost = lut[d_scaled as usize];
            *old_cost = if *old_cost == UNKNOWN && cost >= INSCRIBED {
                cost
            } else {
                (*old_cost).max(cost)
            };
        }
        Ok(())
    }

    /// `computeCaches`: the cost of each distance from 0 to the inflation
    /// radius and one entry beyond, in steps of `1 / COST_LUT_PRECISION`
    /// cells.
    fn cost_table(&self, cell_inflation_radius: u32, resolution: f64) -> Vec<u8> {
        let max_dist_scaled = cell_inflation_radius * COST_LUT_PRECISION + 1;
        (0..=max_dist_scaled)
            .map(|d_scaled| {
                let distance = f64::from(d_scaled) / f64::from(COST_LUT_PRECISION);
                self.cost(distance, resolution)
            })
            .collect()
    }

    /// `computeCost`: the cost of a cell `distance` cells from the nearest
    /// obstacle.
    fn cost(&self, distance: f64, resolution: f64) -> u8 {
        let inscribed_radius = self.footprint.inscribed_radius_m();
        if distance == 0.0 {
            return LETHAL;
        }
        if distance * resolution <= inscribed_radius {
            return INSCRIBED;
        }
        let factor = (-self.params.cost_scaling_factor.get()
            * (distance * resolution - inscribed_radius))
            .exp();
        (f64::from(INSCRIBED - 1) * factor) as u8
    }
}

/// `Costmap2D::cellDistance`: `world_dist` in cells, rounded up.
fn cell_distance(world_dist: f64, resolution: f64) -> u32 {
    (world_dist / resolution).ceil().max(0.0) as u32
}

/// `DistanceTransform::distanceTransform2D` on `img`, `height` rows of
/// `width` cells: the 1-D transform down each column, then along each row,
/// then the square root.
fn distance_transform_2d(img: &mut [f32], height: usize, width: usize) {
    let mut f = vec![0.0; height.max(width)];
    let mut d = vec![0.0; height.max(width)];
    let mut v = vec![0; height.max(width)];
    let mut z = vec![0.0; height.max(width) + 1];
    for x in 0..width {
        for (y, cell) in f[..height].iter_mut().enumerate() {
            *cell = img[y * width + x];
        }
        distance_transform_1d(&f[..height], &mut d[..height], &mut v, &mut z);
        for (y, &cell) in d[..height].iter().enumerate() {
            img[y * width + x] = cell;
        }
    }
    for y in 0..height {
        let row = &mut img[y * width..(y + 1) * width];
        f[..width].copy_from_slice(row);
        distance_transform_1d(&f[..width], &mut d[..width], &mut v, &mut z);
        row.copy_from_slice(&d[..width]);
    }
    for cell in img.iter_mut() {
        *cell = cell.sqrt();
    }
}

/// `DistanceTransform::distanceTransform1D`: the lower envelope of the
/// parabolas rooted at `f`, sampled into `d`, in single precision with the
/// squares of the indices in integers. `v` and `z` are scratch of at least
/// `f.len()` and `f.len() + 1`.
fn distance_transform_1d(f: &[f32], d: &mut [f32], v: &mut [i32], z: &mut [f32]) {
    let n = f.len() as i32;
    let at = |i: i32| i as usize;
    let intersection = |f: &[f32], q: i32, vk: i32| {
        (f[at(q)] - f[at(vk)] + (q * q - vk * vk) as f32) / (2.0 * (q - vk) as f32)
    };
    let mut k = 0_i32;
    v[0] = 0;
    z[0] = -DT_INF;
    z[1] = DT_INF;
    for q in 1..n {
        let mut s = intersection(f, q, v[at(k)]);
        while s <= z[at(k)] {
            k -= 1;
            s = intersection(f, q, v[at(k)]);
        }
        k += 1;
        v[at(k)] = q;
        z[at(k)] = s;
        z[at(k + 1)] = DT_INF;
    }
    k = 0;
    for q in 0..n {
        while z[at(k + 1)] < q as f32 {
            k += 1;
        }
        let diff = q - v[at(k)];
        d[at(q)] = (diff * diff) as f32 + f[at(v[at(k)])];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{FREE, GridGeometry};
    use control_core::footprint::Footprint;
    use control_core::positive_finite::PositiveFinite;

    fn transformed(cells: &[f32]) -> Vec<f32> {
        let mut d = vec![0.0; cells.len()];
        let mut v = vec![0; cells.len()];
        let mut z = vec![0.0; cells.len() + 1];
        distance_transform_1d(cells, &mut d, &mut v, &mut z);
        d
    }

    // Down a line, the transform gives the square of the distance to the
    // nearest root, and a line with none stays at the transform's infinity.
    #[test]
    fn the_transform_squares_the_distance_to_the_nearest_obstacle() {
        assert_eq!(
            transformed(&[DT_INF, 0.0, DT_INF, DT_INF, DT_INF, 0.0]),
            [1.0, 0.0, 1.0, 4.0, 1.0, 0.0]
        );
        assert!(transformed(&[DT_INF; 4]).iter().all(|&d| d == DT_INF));
    }

    // A radius of more cells than the table indexes is refused, naming the
    // radius it may take on the grid; one within it inflates.
    #[test]
    fn a_radius_beyond_the_tables_index_is_refused() {
        let geometry =
            GridGeometry::new(3, 3, PositiveFinite::parse(1e-6).unwrap(), [0.0; 2]).unwrap();
        let mut grid = CostGrid::filled(geometry, FREE);
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-1e-6, -1e-6], [1e-6, -1e-6], [1e-6, 1e-6], [-1e-6, 1e-6]])
                .unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let inflation = |radius_m: f64| {
            Inflation::parse(
                InflationParams {
                    inflation_radius_m: NonNegativeFinite::parse(radius_m).unwrap(),
                    cost_scaling_factor: NonNegativeFinite::parse(3.0).unwrap(),
                },
                &footprint,
            )
            .unwrap()
        };
        let refused = inflation(100.0).inflate(&mut grid).unwrap_err();
        assert_eq!(
            refused,
            InflationError {
                inflation_radius_m: 100.0,
                resolution_m: 1e-6,
            }
        );
        assert!(
            refused.to_string().contains("give at most 42.9"),
            "{refused}"
        );
        assert!(inflation(1e-5).inflate(&mut grid).is_ok());
    }

    // A radius below the padded footprint's inscribed radius is refused, 0
    // among them, naming the least radius to give; a radius of the inscribed
    // radius makes an inflation, as Nav2 compares the two.
    #[test]
    fn a_radius_below_the_inscribed_radius_is_refused() {
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.3, -0.3], [0.3, -0.3], [0.3, 0.3], [-0.3, 0.3]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let inflation = |radius_m: f64| {
            Inflation::parse(
                InflationParams {
                    inflation_radius_m: NonNegativeFinite::parse(radius_m).unwrap(),
                    cost_scaling_factor: NonNegativeFinite::parse(3.0).unwrap(),
                },
                &footprint,
            )
        };
        let inscribed_radius_m = footprint.inscribed_radius_m();
        for radius_m in [0.0, 0.1, inscribed_radius_m.next_down()] {
            let refused = inflation(radius_m).unwrap_err();
            assert_eq!(
                refused,
                InflationBelowInscribed {
                    inflation_radius_m: radius_m,
                    inscribed_radius_m,
                }
            );
            assert!(
                refused
                    .to_string()
                    .ends_with("give inflation_radius_m of at least 0.3 m"),
                "{refused}"
            );
        }
        assert!(inflation(inscribed_radius_m).is_ok());
    }

    // One lethal cell in the middle of a free grid: it stays lethal, its
    // neighbors within the inscribed radius take the inscribed cost, and the
    // cost falls with distance to the inflation radius.
    #[test]
    fn a_lethal_cell_inflates_into_a_falling_ring() {
        let geometry =
            GridGeometry::new(9, 9, PositiveFinite::parse(0.1).unwrap(), [0.0; 2]).unwrap();
        let mut costs = vec![FREE; 81];
        costs[4 * 9 + 4] = LETHAL;
        let mut grid = CostGrid::from_costs(geometry, costs).unwrap();
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let inflation = Inflation::parse(
            InflationParams {
                inflation_radius_m: NonNegativeFinite::parse(0.3).unwrap(),
                cost_scaling_factor: NonNegativeFinite::parse(3.0).unwrap(),
            },
            &footprint,
        )
        .unwrap();
        inflation.inflate(&mut grid).unwrap();
        let row: Vec<u8> = (0..9).map(|x| grid.costs()[4 * 9 + x]).collect();
        let at = |cells: f64| (252.0 * (-3.0 * (cells * 0.1 - 0.1_f64)).exp()) as u8;
        assert_eq!(
            row,
            [
                FREE,
                at(3.0),
                at(2.0),
                INSCRIBED,
                LETHAL,
                INSCRIBED,
                at(2.0),
                at(3.0),
                FREE
            ]
        );
    }
}
