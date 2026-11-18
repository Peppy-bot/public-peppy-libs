// Copyright 2010 SRI International
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This file translates into Rust karto's correlative scan matcher as slam_toolbox 2.10.0
// carries it: lib/karto_sdk/src/Mapper.cpp, lib/karto_sdk/include/karto_sdk/Mapper.h and
// Karto.h (Copyright 2010 SRI International).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! karto's correlative scan matcher, as slam_toolbox 2.10.0 carries it
//! (lib/karto_sdk/src/Mapper.cpp: `ScanMatcher::Create`, `MatchScan`, `CorrelateScan` and
//! its `operator()`, `ComputePositionalCovariance`, `ComputeAngularCovariance`,
//! `AddScans`, `AddScan`, `FindValidPoints`, `GetResponse`; Mapper.h: `CorrelationGrid`;
//! Karto.h: `GridIndexLookup`). The scans to match against mark the cells their points
//! hit in a correlation grid centered on the scan, each hit smeared by a Gaussian kernel;
//! the scan's points are looked up at each pose of a coarse search, then of a fine one
//! about the best coarse pose, and the best response gives the pose, with a covariance
//! from the responses about it. The search runs on one thread, in the order of karto's
//! loops.

use crate::geometry::{
    Matrix3, Pose2, TOLERANCE, Transform, Vector2, double_equal, normalize_angle,
    normalize_angle_difference, round,
};
use crate::grid::{Cell, Grid};
use crate::localized_scan::LocalizedScan;
use thiserror::Error;

/// `MAX_VARIANCE`: the variance of a position that no match supports (m^2).
const MAX_VARIANCE: f64 = 500.0;
/// `DISTANCE_PENALTY_GAIN` and `ANGLE_PENALTY_GAIN`.
const DISTANCE_PENALTY_GAIN: f64 = 0.2;
const ANGLE_PENALTY_GAIN: f64 = 0.2;
/// `GridStates_Occupied`: the value of a hit cell.
const OCCUPIED: u8 = 100;
/// `INVALID_SCAN`: the lookup offset of a point with no range.
const INVALID_SCAN: i32 = i32::MAX;
/// `MIN_MAPPER_VALUE` and `MAX_MAPEER_VALUE`: the bounds of a matcher's sizes.
const MIN_MAPPER_VALUE: f64 = 1e-6;
const MAX_MAPPER_VALUE: f64 = 1e6;
/// The points of a scan nearer each other than this (m) are not compared for the side of
/// the view point they lie on.
const VIEW_POINT_SPACING: f64 = 0.1;
/// The responses within this of the best one count toward a covariance.
const NEAR_BEST_RESPONSE: f64 = 0.1;
/// `KT_PI_180`, by which karto turns degrees to radians.
const RADIANS_PER_DEGREE: f64 = 0.017453292519943295;
/// How far each widening of a coarse search that finds no response turns it (deg), and how
/// many times it widens.
const EXPANSION_DEGREES: f64 = 20.0;
const EXPANSIONS: usize = 3;

/// The mapper's parameters that a match reads, by slam_toolbox's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MatchParameters {
    pub coarse_angle_resolution: f64,
    pub coarse_search_angle_offset: f64,
    pub fine_search_angle_offset: f64,
    pub distance_variance_penalty: f64,
    pub angle_variance_penalty: f64,
    pub minimum_distance_penalty: f64,
    pub minimum_angle_penalty: f64,
    pub use_response_expansion: bool,
}

/// Why sizes make no matcher.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum MatcherError {
    #[error("the scan matcher's {name} is {value}, and it must lie between 1e-6 and 1e6")]
    OutOfRange { name: &'static str, value: f64 },
    #[error(
        "the smear deviation {deviation} m lies outside half a cell and ten cells of the \
         resolution, [{minimum}, {maximum}] m"
    )]
    SmearDeviation {
        deviation: f64,
        minimum: f64,
        maximum: f64,
    },
    #[error("a pose of the search lies outside the search space's grid, at cell {x}, {y}")]
    OutsideSearchSpace { x: i32, y: i32 },
}

/// The pose of a scan that matches best, its covariance, and the response there, from 0
/// to 1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Match {
    pub mean: Pose2,
    pub covariance: Matrix3,
    pub response: f64,
}

/// `Rectangle2<kt_int32s>`: the region of a correlation grid inside its border.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Roi {
    x: i32,
    y: i32,
    width: i32,
    height: i32,
}

/// `CorrelationGrid`: a grid of hits with a border that holds the smear of a hit at the
/// edge of its region.
#[derive(Debug, Clone, PartialEq)]
struct CorrelationGrid {
    grid: Grid<u8>,
    roi: Roi,
    kernel_size: i32,
    kernel: Vec<u8>,
}

/// `GetHalfKernelSize`.
fn half_kernel_size(smear_deviation: f64, resolution: f64) -> i32 {
    round(2.0 * smear_deviation / resolution) as i32
}

impl CorrelationGrid {
    /// `CorrelationGrid::CreateGrid` and `CalculateKernel`, of a resolution and a smear
    /// deviation that [`MatcherSizes::parse`] accepted.
    fn new(width: i32, height: i32, resolution: f64, smear_deviation: f64) -> Self {
        let border = half_kernel_size(smear_deviation, resolution) + 1;
        let grid = Grid::new(width + border * 2, height + border * 2, resolution);
        let resolution = grid.resolution();
        let kernel_size = 2 * half_kernel_size(smear_deviation, resolution) + 1;
        let half = kernel_size / 2;
        let mut kernel = vec![0u8; (kernel_size * kernel_size) as usize];
        for i in -half..=half {
            for j in -half..=half {
                let distance = (f64::from(i) * resolution).hypot(f64::from(j) * resolution);
                let z = (-0.5 * (distance / smear_deviation).powi(2)).exp();
                let value = round(z * f64::from(OCCUPIED)) as u32;
                kernel[((i + half) + kernel_size * (j + half)) as usize] = value as u8;
            }
        }
        Self {
            grid,
            roi: Roi {
                x: border,
                y: border,
                width,
                height,
            },
            kernel_size,
            kernel,
        }
    }

    /// `CorrelationGrid::GridIndex`: the index of `cell`, counted from the region's corner.
    fn index(&self, cell: Cell) -> i32 {
        self.grid.index(Cell {
            x: cell.x + self.roi.x,
            y: cell.y + self.roi.y,
        })
    }

    /// `SmearPoint`: the kernel about the hit at `cell`, where it rises above the grid.
    fn smear_point(&mut self, cell: Cell) {
        let half = self.kernel_size / 2;
        for j in -half..=half {
            let row = self.index(Cell {
                x: cell.x,
                y: cell.y + j,
            });
            let kernel_constant = half + self.kernel_size * (j + half);
            for i in -half..=half {
                let kernel_value = self.kernel[(i + kernel_constant) as usize];
                if let Some(value) = self.grid.at_index_mut(row + i)
                    && kernel_value > *value
                {
                    *value = kernel_value;
                }
            }
        }
    }

    /// `ScanMatcher::AddScan` with the smear: marks the cells that the points of `scan`
    /// on the side of `view_point` hit.
    fn add_scan(&mut self, scan: &LocalizedScan, view_point: Vector2) {
        for point in find_valid_points(scan, view_point) {
            let cell = self.grid.world_to_grid(point);
            if !(0..self.roi.width).contains(&cell.x) || !(0..self.roi.height).contains(&cell.y) {
                continue;
            }
            let index = self.index(cell);
            let Some(value) = self.grid.at_index_mut(index) else {
                continue;
            };
            if *value == OCCUPIED {
                continue;
            }
            *value = OCCUPIED;
            self.smear_point(cell);
        }
    }
}

/// `FindValidPoints`: the points of `scan` that face `view_point`. A point is compared
/// with the last compared point once the two lie more than [`VIEW_POINT_SPACING`] apart,
/// and the points since the last kept stretch are kept when it turns counterclockwise as
/// seen from `view_point`.
fn find_valid_points(scan: &LocalizedScan, view_point: Vector2) -> Vec<Vector2> {
    let points = scan.unfiltered_point_readings();
    let min_square_distance = VIEW_POINT_SPACING * VIEW_POINT_SPACING;
    let mut valid = Vec::new();
    let mut trailing = 0;
    let mut first_point = Vector2::default();
    let mut first_time = true;
    for (index, &current) in points.iter().enumerate() {
        if first_time && !current.x.is_nan() && !current.y.is_nan() {
            first_point = current;
            first_time = false;
        }
        let delta = first_point.minus(current);
        if delta.squared_length() <= min_square_distance || delta.squared_length().is_nan() {
            continue;
        }
        let a = view_point.y - first_point.y;
        let b = first_point.x - view_point.x;
        let c = first_point.y * view_point.x - first_point.x * view_point.y;
        let ss = current.x * a + current.y * b + c;
        first_point = current;
        if ss < 0.0 {
            trailing = index;
        } else {
            valid.extend_from_slice(&points[trailing..index]);
            trailing = index;
        }
    }
    valid
}

/// `GridIndexLookup`: for each angle of a search, the offset in the correlation grid of
/// each point of the scan turned by that angle about the scan's origin.
#[derive(Debug, Clone, PartialEq, Default)]
struct GridIndexLookup {
    offsets: Vec<Vec<i32>>,
}

impl GridIndexLookup {
    /// `ComputeOffsets`: the offsets at each angle from `angle_center - angle_offset` in
    /// steps of `angle_resolution`.
    fn compute_offsets(
        &mut self,
        grid: &CorrelationGrid,
        scan: &LocalizedScan,
        angle_center: f64,
        angle_offset: f64,
        angle_resolution: f64,
    ) {
        let angles = (round(angle_offset * 2.0 / angle_resolution) + 1.0) as u32;
        let transform = Transform::to(scan.sensor_pose());
        let local_points: Vec<Vector2> = scan
            .unfiltered_point_readings()
            .iter()
            .map(|&point| {
                transform
                    .inverse_transform_pose(Pose2::at(point, 0.0))
                    .position()
            })
            .collect();
        let grid_offset = grid.grid.converter.offset;
        let start_angle = angle_center - angle_offset;
        self.offsets = (0..angles)
            .map(|angle_index| {
                let angle = start_angle + f64::from(angle_index) * angle_resolution;
                let (cosine, sine) = (angle.cos(), angle.sin());
                local_points
                    .iter()
                    .zip(&scan.beams)
                    .map(|(position, beam)| {
                        let range = beam.reading;
                        if range.is_nan() || range.is_infinite() {
                            return INVALID_SCAN;
                        }
                        let offset = Vector2::new(
                            cosine * position.x - sine * position.y,
                            sine * position.x + cosine * position.y,
                        );
                        grid.grid
                            .index(grid.grid.world_to_grid(offset.plus(grid_offset)))
                    })
                    .collect()
            })
            .collect();
    }

    /// `ScanMatcher::GetResponse`: the mean value of the cells that the scan's points hit
    /// at the angle `angle_index` from the cell at `grid_index`, over 100, from 0 to 1. A
    /// point off the grid adds nothing and still counts.
    fn response(&self, grid: &CorrelationGrid, angle_index: usize, grid_index: i32) -> f64 {
        let offsets = &self.offsets[angle_index];
        if offsets.is_empty() {
            return 0.0;
        }
        let data_size = grid.grid.data_size();
        let sum = offsets
            .iter()
            .filter(|&&offset| offset != INVALID_SCAN)
            .map(|&offset| grid_index.wrapping_add(offset))
            .filter(|index| (0..data_size).contains(index))
            .fold(0.0, |sum, index| {
                sum + f64::from(grid.grid.at_index(index).unwrap_or_default())
            });
        sum / f64::from(offsets.len() as u32 * u32::from(OCCUPIED))
    }
}

/// A pose of a search and its response.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PoseResponse {
    response: f64,
    pose: Pose2,
}

/// One search of `CorrelateScan`: about `center`, the positions `offset` either way in
/// steps of `resolution`, and the angles `angle_offset` either way in steps of
/// `angle_resolution`.
#[derive(Debug, Clone, Copy)]
struct Search {
    center: Pose2,
    offset: Vector2,
    resolution: Vector2,
    angle_offset: f64,
    angle_resolution: f64,
    penalize: bool,
    fine: bool,
    /// The largest distance (m) from the center of a position the search tries, when it
    /// tries the positions of a disc only.
    within: Option<f64>,
}

/// The steps `-offset, -offset + resolution, ..., offset` of one axis of a search.
fn steps(offset: f64, resolution: f64) -> Vec<f64> {
    let count = (round(offset * 2.0 / resolution) + 1.0) as u32;
    let start = -offset;
    (0..count)
        .map(|index| start + f64::from(index) * resolution)
        .collect()
}

/// `ScanMatcher`: the correlation grid, the grid of the best response of each position
/// of a coarse search, and the lookup of the scan's points.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ScanMatcher {
    parameters: MatchParameters,
    correlation_grid: CorrelationGrid,
    search_space_probabilities: Grid<f64>,
    lookup: GridIndexLookup,
}

/// The sizes of a matcher: its search (m), the cells of its grids (m) and the smear of a
/// hit (m), each between 1e-6 and 1e6, and the smear between half a cell and ten cells.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MatcherSizes {
    search_size: f64,
    resolution: f64,
    smear_deviation: f64,
}

/// `ScanMatcher::Create`'s check of a size.
fn check_size(name: &'static str, value: f64) -> Result<(), MatcherError> {
    if (MIN_MAPPER_VALUE..=MAX_MAPPER_VALUE).contains(&value) {
        Ok(())
    } else {
        Err(MatcherError::OutOfRange { name, value })
    }
}

impl MatcherSizes {
    /// The checks of `ScanMatcher::Create` and of `CalculateKernel`, with the resolution
    /// at which the correlation grid holds it, one over its scale.
    pub fn parse(
        search_size: f64,
        resolution: f64,
        smear_deviation: f64,
    ) -> Result<Self, MatcherError> {
        check_size("resolution", resolution)?;
        check_size("search size", search_size)?;
        check_size("smear deviation", smear_deviation)?;
        let held_resolution = 1.0 / (1.0 / resolution);
        let (minimum, maximum) = (0.5 * held_resolution, 10.0 * held_resolution);
        if !(minimum..=maximum).contains(&smear_deviation) {
            return Err(MatcherError::SmearDeviation {
                deviation: smear_deviation,
                minimum,
                maximum,
            });
        }
        Ok(Self {
            search_size,
            resolution,
            smear_deviation,
        })
    }
}

impl MatcherSizes {
    /// These sizes with the smallest search that holds a disc of `radius` (m) about its
    /// center and whose half side is a whole number of the coarse search's steps of two
    /// cells, so that the coarse search's positions run from edge to edge through the
    /// center; checked as [`MatcherSizes::parse`] checks it.
    pub fn holding_disc(self, radius: f64) -> Result<Self, MatcherError> {
        let coarse_step = 2.0 * self.resolution;
        let half_side_steps = (radius / coarse_step).ceil();
        Self::parse(
            2.0 * half_side_steps * coarse_step,
            self.resolution,
            self.smear_deviation,
        )
    }
}

impl ScanMatcher {
    /// `ScanMatcher::Create`: a matcher of `sizes` that holds the points of scans within
    /// `range_threshold` (m) of the scan.
    pub fn new(
        parameters: MatchParameters,
        sizes: MatcherSizes,
        range_threshold: f64,
    ) -> Result<Self, MatcherError> {
        check_size("range threshold", range_threshold)?;
        let MatcherSizes {
            search_size,
            resolution,
            smear_deviation,
        } = sizes;
        let search_space_side = (round(search_size / resolution) + 1.0) as u32 as i32;
        let point_reading_margin = (range_threshold / resolution).ceil() as u32 as i32;
        let grid_size = search_space_side + 2 * point_reading_margin;
        Ok(Self {
            parameters,
            correlation_grid: CorrelationGrid::new(
                grid_size,
                grid_size,
                resolution,
                smear_deviation,
            ),
            search_space_probabilities: Grid::new(search_space_side, search_space_side, resolution),
            lookup: GridIndexLookup::default(),
        })
    }

    /// `MatchScan`: the pose at which `scan` matches `base_scans` best, from a coarse
    /// search about its sensor pose, widened by 20 deg up to three times while no pose
    /// responds when the parameters expand the response, and then, with `refine`, a fine
    /// search about the coarse pose. With `penalize`, a response falls with the distance
    /// and the turn from the scan's pose.
    pub fn match_scan(
        &mut self,
        scan: &LocalizedScan,
        base_scans: &[&LocalizedScan],
        penalize: bool,
        refine: bool,
    ) -> Result<Match, MatcherError> {
        let scan_pose = scan.sensor_pose();
        let resolution = self.correlation_grid.grid.resolution();
        let roi = self.correlation_grid.roi;
        self.correlation_grid.grid.converter.offset = Vector2::new(
            scan_pose.x - 0.5 * f64::from(roi.width - 1) * resolution,
            scan_pose.y - 0.5 * f64::from(roi.height - 1) * resolution,
        );
        self.add_scans(base_scans, scan_pose.position());
        let search_dimensions = Vector2::new(
            f64::from(self.search_space_probabilities.width()),
            f64::from(self.search_space_probabilities.height()),
        );
        let coarse_offset = Vector2::new(
            0.5 * (search_dimensions.x - 1.0) * resolution,
            0.5 * (search_dimensions.y - 1.0) * resolution,
        );
        let coarse_resolution = Vector2::new(2.0 * resolution, 2.0 * resolution);
        let MatchParameters {
            coarse_angle_resolution,
            coarse_search_angle_offset,
            fine_search_angle_offset,
            use_response_expansion,
            ..
        } = self.parameters;
        let coarse = |angle_offset: f64| Search {
            center: scan_pose,
            offset: coarse_offset,
            resolution: coarse_resolution,
            angle_offset,
            angle_resolution: coarse_angle_resolution,
            penalize,
            fine: false,
            within: None,
        };
        let mut covariance = Matrix3::zero();
        let mut best =
            self.correlate_scan(scan, coarse(coarse_search_angle_offset), &mut covariance)?;
        if use_response_expansion && double_equal(best.response, 0.0) {
            let mut angle_offset = coarse_search_angle_offset;
            for _ in 0..EXPANSIONS {
                angle_offset += EXPANSION_DEGREES * RADIANS_PER_DEGREE;
                best = self.correlate_scan(scan, coarse(angle_offset), &mut covariance)?;
                if !double_equal(best.response, 0.0) {
                    break;
                }
            }
        }
        if refine {
            let fine = Search {
                center: best.mean,
                offset: coarse_resolution.scaled(0.5),
                resolution: Vector2::new(resolution, resolution),
                angle_offset: 0.5 * coarse_angle_resolution,
                angle_resolution: fine_search_angle_offset,
                penalize,
                fine: true,
                within: None,
            };
            best = self.correlate_scan(scan, fine, &mut covariance)?;
        }
        Ok(best)
    }

    /// The coarse search of `MatchScan` over the positions within `radius` (m) of the
    /// scan's sensor pose and the headings within `angle_offset` (rad) of it, a whole turn
    /// at pi or more, unpenalized and unrefined, as karto's loop matcher searches a loop
    /// before its fine match. The matcher's search space holds the disc.
    pub fn match_scan_within(
        &mut self,
        scan: &LocalizedScan,
        base_scans: &[&LocalizedScan],
        radius: f64,
        angle_offset: f64,
    ) -> Result<Match, MatcherError> {
        let scan_pose = scan.sensor_pose();
        let resolution = self.correlation_grid.grid.resolution();
        let roi = self.correlation_grid.roi;
        self.correlation_grid.grid.converter.offset = Vector2::new(
            scan_pose.x - 0.5 * f64::from(roi.width - 1) * resolution,
            scan_pose.y - 0.5 * f64::from(roi.height - 1) * resolution,
        );
        self.add_scans(base_scans, scan_pose.position());
        let side = f64::from(self.search_space_probabilities.width());
        let offset = 0.5 * (side - 1.0) * resolution;
        let search = Search {
            center: scan_pose,
            offset: Vector2::new(offset, offset),
            resolution: Vector2::new(2.0 * resolution, 2.0 * resolution),
            angle_offset: angle_offset.min(std::f64::consts::PI),
            angle_resolution: self.parameters.coarse_angle_resolution,
            penalize: false,
            fine: false,
            within: Some(radius),
        };
        let mut covariance = Matrix3::zero();
        self.correlate_scan(scan, search, &mut covariance)
    }

    /// `AddScans`: the correlation grid cleared, and marked with the points of `scans`.
    fn add_scans(&mut self, scans: &[&LocalizedScan], view_point: Vector2) {
        self.correlation_grid.grid.clear();
        for scan in scans {
            self.correlation_grid.add_scan(scan, view_point);
        }
    }

    /// `CorrelateScan`: the best pose of `search`, the mean of the poses whose response
    /// equals the best one, with the covariance that the coarse search gives the position
    /// or the fine search gives the angle, written into `covariance`.
    fn correlate_scan(
        &mut self,
        scan: &LocalizedScan,
        search: Search,
        covariance: &mut Matrix3,
    ) -> Result<Match, MatcherError> {
        self.lookup.compute_offsets(
            &self.correlation_grid,
            scan,
            search.center.yaw,
            search.angle_offset,
            search.angle_resolution,
        );
        if !search.fine {
            self.search_space_probabilities.clear();
            self.search_space_probabilities.converter.offset =
                search.center.position().minus(search.offset);
        }
        let x_poses = steps(search.offset.x, search.resolution.x);
        let y_poses = steps(search.offset.y, search.resolution.y);
        let angles = (round(search.angle_offset * 2.0 / search.angle_resolution) + 1.0) as u32;
        let start_angle = search.center.yaw - search.angle_offset;
        let this: &Self = self;
        let responses: Vec<PoseResponse> = y_poses
            .iter()
            .flat_map(|&y| x_poses.iter().map(move |&x| (x, y)))
            .filter(|&(x, y)| search.within.is_none_or(|radius| x.hypot(y) <= radius))
            .flat_map(|(x, y)| {
                let position = Vector2::new(search.center.x + x, search.center.y + y);
                let cell = this.correlation_grid.grid.world_to_grid(position);
                let grid_index = this.correlation_grid.index(cell);
                (0..angles).map(move |angle_index| {
                    let angle = start_angle + f64::from(angle_index) * search.angle_resolution;
                    let response = this.penalized(
                        this.lookup.response(
                            &this.correlation_grid,
                            angle_index as usize,
                            grid_index,
                        ),
                        &search,
                        x * x + y * y,
                        angle,
                    );
                    PoseResponse {
                        response,
                        pose: Pose2::at(position, normalize_angle(angle)),
                    }
                })
            })
            .collect();
        let best_response = responses.iter().fold(-1.0, |best, candidate| {
            if best > candidate.response {
                best
            } else {
                candidate.response
            }
        });
        if !search.fine {
            for candidate in &responses {
                let cell = self
                    .search_space_probabilities
                    .world_to_grid(candidate.pose.position());
                if !self.search_space_probabilities.is_valid(cell) {
                    return Err(MatcherError::OutsideSearchSpace {
                        x: cell.x,
                        y: cell.y,
                    });
                }
                let index = self.search_space_probabilities.index(cell);
                if let Some(probability) = self.search_space_probabilities.at_index_mut(index)
                    && candidate.response > *probability
                {
                    *probability = candidate.response;
                }
            }
        }
        let best_poses: Vec<&PoseResponse> = responses
            .iter()
            .filter(|candidate| double_equal(candidate.response, best_response))
            .collect();
        let count = best_poses.len() as f64;
        let (position_sum, theta_x, theta_y) = best_poses.iter().fold(
            (Vector2::default(), 0.0, 0.0),
            |(sum, theta_x, theta_y), candidate| {
                (
                    sum.plus(candidate.pose.position()),
                    theta_x + candidate.pose.yaw.cos(),
                    theta_y + candidate.pose.yaw.sin(),
                )
            },
        );
        let average_pose = Pose2::at(
            position_sum.divided(count),
            (theta_y / count).atan2(theta_x / count),
        );
        if search.fine {
            covariance.0[2][2] = self.angular_variance(average_pose, best_response, &search);
        } else {
            *covariance = self.positional_covariance(average_pose, best_response, &search)?;
        }
        Ok(Match {
            mean: average_pose,
            covariance: *covariance,
            response: best_response.min(1.0),
        })
    }

    /// The response of a pose `squared_distance` (m^2) from the search's center and at
    /// `angle`, cut by the distance and the turn from the center when the search
    /// penalizes.
    fn penalized(&self, response: f64, search: &Search, squared_distance: f64, angle: f64) -> f64 {
        if !search.penalize || double_equal(response, 0.0) {
            return response;
        }
        let at_least = |value: f64, minimum: f64| if value > minimum { value } else { minimum };
        let distance_penalty = at_least(
            1.0 - (DISTANCE_PENALTY_GAIN * squared_distance
                / self.parameters.distance_variance_penalty),
            self.parameters.minimum_distance_penalty,
        );
        let squared_angle_distance = (angle - search.center.yaw).powi(2);
        let angle_penalty = at_least(
            1.0 - (ANGLE_PENALTY_GAIN * squared_angle_distance
                / self.parameters.angle_variance_penalty),
            self.parameters.minimum_angle_penalty,
        );
        response * (distance_penalty * angle_penalty)
    }

    /// `ComputePositionalCovariance`: the spread of the positions whose best response
    /// lies within [`NEAR_BEST_RESPONSE`] of the best, about the best pose, weighted by
    /// their responses and over the best response, bounded below by a tenth of a cell.
    fn positional_covariance(
        &self,
        best_pose: Pose2,
        best_response: f64,
        search: &Search,
    ) -> Result<Matrix3, MatcherError> {
        let mut covariance = Matrix3::identity();
        if best_response < TOLERANCE {
            covariance.0[0][0] = MAX_VARIANCE;
            covariance.0[1][1] = MAX_VARIANCE;
            covariance.0[2][2] = 4.0 * search.angle_resolution.powi(2);
            return Ok(covariance);
        }
        let dx = best_pose.x - search.center.x;
        let dy = best_pose.y - search.center.y;
        let mut accumulated_xx = 0.0;
        let mut accumulated_xy = 0.0;
        let mut accumulated_yy = 0.0;
        let mut norm = 0.0;
        for y in steps(search.offset.y, search.resolution.y) {
            for x in steps(search.offset.x, search.resolution.x) {
                let cell = self
                    .search_space_probabilities
                    .world_to_grid(Vector2::new(search.center.x + x, search.center.y + y));
                let response = self.search_space_probabilities.value(cell).ok_or(
                    MatcherError::OutsideSearchSpace {
                        x: cell.x,
                        y: cell.y,
                    },
                )?;
                if response >= best_response - NEAR_BEST_RESPONSE {
                    norm += response;
                    accumulated_xx += (x - dx).powi(2) * response;
                    accumulated_xy += (x - dx) * (y - dy) * response;
                    accumulated_yy += (y - dy).powi(2) * response;
                }
            }
        }
        if norm > TOLERANCE {
            let at_least = |value: f64, minimum: f64| if value > minimum { value } else { minimum };
            let variance_xx = at_least(accumulated_xx / norm, 0.1 * search.resolution.x.powi(2));
            let variance_xy = accumulated_xy / norm;
            let variance_yy = at_least(accumulated_yy / norm, 0.1 * search.resolution.y.powi(2));
            let multiplier = 1.0 / best_response;
            covariance.0[0][0] = variance_xx * multiplier;
            covariance.0[0][1] = variance_xy * multiplier;
            covariance.0[1][0] = variance_xy * multiplier;
            covariance.0[1][1] = variance_yy * multiplier;
            covariance.0[2][2] = 4.0 * search.angle_resolution.powi(2);
        }
        if double_equal(covariance.0[0][0], 0.0) {
            covariance.0[0][0] = MAX_VARIANCE;
        }
        if double_equal(covariance.0[1][1], 0.0) {
            covariance.0[1][1] = MAX_VARIANCE;
        }
        Ok(covariance)
    }

    /// `ComputeAngularCovariance`: the spread of the angles at the best position whose
    /// response lies within [`NEAR_BEST_RESPONSE`] of the best, about the best heading
    /// moved to within pi of the search's center, weighted by their responses.
    fn angular_variance(&self, best_pose: Pose2, best_response: f64, search: &Search) -> f64 {
        let best_angle = normalize_angle_difference(best_pose.yaw, search.center.yaw);
        let cell = self
            .correlation_grid
            .grid
            .world_to_grid(best_pose.position());
        let grid_index = self.correlation_grid.index(cell);
        let angles = (round(search.angle_offset * 2.0 / search.angle_resolution) + 1.0) as u32;
        let start_angle = search.center.yaw - search.angle_offset;
        let (norm, accumulated) = (0..angles).fold((0.0, 0.0), |(norm, accumulated), index| {
            let angle = start_angle + f64::from(index) * search.angle_resolution;
            let response = self
                .lookup
                .response(&self.correlation_grid, index as usize, grid_index);
            if response >= best_response - NEAR_BEST_RESPONSE {
                (
                    norm + response,
                    accumulated + (angle - best_angle).powi(2) * response,
                )
            } else {
                (norm, accumulated)
            }
        });
        if norm > TOLERANCE {
            let accumulated = if accumulated < TOLERANCE {
                search.angle_resolution.powi(2)
            } else {
                accumulated
            };
            accumulated / norm
        } else {
            1000.0 * search.angle_resolution.powi(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localized_scan::Laser;
    use crate::placed_scan::{Beam, ScanGeometry};
    use hegel::TestCase;
    use hegel::generators as gs;

    #[test]
    fn the_kernel_peaks_at_its_center_and_falls_with_distance() {
        let grid = CorrelationGrid::new(11, 11, 0.01, 0.03);
        let half = grid.kernel_size / 2;
        let at =
            |i: i32, j: i32| grid.kernel[((i + half) + grid.kernel_size * (j + half)) as usize];
        assert_eq!(at(0, 0), OCCUPIED);
        assert!(at(1, 0) < at(0, 0) && at(2, 0) < at(1, 0));
        assert_eq!(at(1, 2), at(2, 1));
    }

    #[test]
    fn sizes_outside_karto_s_bounds_are_refused() {
        assert!(matches!(
            MatcherSizes::parse(0.5, 0.01, 0.5),
            Err(MatcherError::SmearDeviation { .. })
        ));
        assert!(matches!(
            MatcherSizes::parse(0.5, 0.0, 0.1),
            Err(MatcherError::OutOfRange {
                name: "resolution",
                ..
            })
        ));
    }

    /// A scan of a corridor's end ahead of a base at `pose`: a wall 2 m ahead and a wall
    /// 1 m to each side.
    fn corridor_scan(laser: &Laser, pose: Pose2) -> LocalizedScan {
        let beams = (0..laser.number_of_readings)
            .map(|beam| {
                let angle = laser.minimum_angle + beam as f64 * laser.angular_resolution;
                Beam {
                    reading: (2.0 / angle.cos()).min(1.0 / angle.sin().abs()),
                    origin: Vector2::new(0.0, 0.0),
                    bearing: angle,
                }
            })
            .collect();
        LocalizedScan::new(laser, beams, 0.0, pose)
    }

    /// slam_toolbox's matching parameters of `mapper_params_online_sync.yaml`.
    const PARAMETERS: MatchParameters = MatchParameters {
        coarse_angle_resolution: 0.0349,
        coarse_search_angle_offset: 0.349,
        fine_search_angle_offset: 0.00349,
        distance_variance_penalty: 0.5,
        angle_variance_penalty: 1.0,
        minimum_distance_penalty: 0.5,
        minimum_angle_penalty: 0.9,
        use_response_expansion: true,
    };

    /// A laser of 121 rays over 1.2 rad that reads from 0.1 m to 6 m.
    fn laser() -> Laser {
        let geometry = ScanGeometry {
            angle_min: -0.6,
            angle_increment: 0.01,
            rays: 121,
            range_min: 0.1,
            range_max: 6.0,
        };
        Laser::of(&geometry, 0.1, 6.0)
    }

    #[test]
    fn a_scan_matches_itself_at_its_own_pose() {
        let laser = laser();
        let sizes = MatcherSizes::parse(0.5, 0.01, 0.1).expect("sizes");
        let mut matcher =
            ScanMatcher::new(PARAMETERS, sizes, laser.range_threshold).expect("a matcher");
        let pose = Pose2::new(0.3, -0.2, 0.1);
        let base = corridor_scan(&laser, pose);
        let found = matcher
            .match_scan(&corridor_scan(&laser, pose), &[&base], true, true)
            .expect("a match");
        assert!(found.response > 0.5, "the response is {}", found.response);
        assert!(
            found.mean.squared_distance(pose).sqrt() < 0.02,
            "{:?}",
            found.mean
        );
    }

    #[hegel::test]
    fn a_matcher_holding_a_disc_searches_all_of_it_and_finds_the_scan_at_its_pose(tc: TestCase) {
        let radius: f64 = tc.draw(gs::floats().min_value(1e-3).max_value(1.0));
        let resolution: f64 = tc.draw(gs::floats().min_value(0.02).max_value(0.1));
        let laser = laser();
        let sizes = MatcherSizes::parse(1.0, resolution, resolution)
            .and_then(|sizes| sizes.holding_disc(radius))
            .expect("sizes holding the disc");
        let mut matcher =
            ScanMatcher::new(PARAMETERS, sizes, laser.range_threshold).expect("a matcher");
        let pose = Pose2::new(0.3, -0.2, 0.1);
        let scan = corridor_scan(&laser, pose);
        let found = matcher
            .match_scan_within(&scan, &[&scan], radius, 0.1)
            .expect("a match within the disc");
        // The coarse search tries the pose's own position, and positions two cells apart
        // about it.
        assert!(
            found.mean.squared_distance(pose).sqrt() <= 2.0 * resolution,
            "{:?} from {pose:?}",
            found.mean
        );
    }
}
