// slam_mapper
// Copyright (c) 2018, Simbe Robotics
// Copyright (c) 2018, Steve Macenski
// Copyright (c) 2019, Samsung Research America
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
// COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
// COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
// AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
//
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//
// visualization_utils
// Copyright (c) 2019, Samsung Research America
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
// COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
// COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
// AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
//
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//
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
// Copyright 2019 Rover Robotics
// Copyright 2010 Brian Gerkey
// Copyright (c) 2008, Willow Garage, Inc.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <ORGANIZATION> nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This file translates into Rust slam_toolbox 2.10.0's grid of a map: src/slam_mapper.cpp
// (Copyright (c) 2018, Simbe Robotics; (c) 2018, Steve Macenski; (c) 2019, Samsung Research
// America), include/slam_toolbox/visualization_utils.hpp (Copyright (c) 2019, Samsung Research
// America) and karto's lib/karto_sdk/include/karto_sdk/Karto.h (Copyright 2010 SRI
// International); and Nav2 1.5.2's map saver, nav2_map_server/src/map_io.cpp (Copyright 2019
// Rover Robotics, Copyright 2010 Brian Gerkey, Copyright (c) 2008, Willow Garage, Inc.).
//
// This file conveys karto's code under the GNU General Public License, version 3 or later.
//
// This translation takes slam_toolbox's own code under the GNU Lesser General Public License,
// version 2.1, of slam_toolbox's LICENSE; as section 3 of that license allows, the GNU General
// Public License, version 3 or later, applies in its place.
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

//! The grid of a map, as slam_toolbox 2.10.0 makes it of the map's scans (src/slam_mapper.cpp:
//! `getOccupancyGrid`; lib/karto_sdk/include/karto_sdk/Karto.h: `OccupancyGrid::CreateFromScans`,
//! `ComputeDimensions`, `AddScan`, `RayTrace`, `UpdateCell`, `Update`, and `Grid::TraceLine`)
//! and exports it (include/slam_toolbox/visualization_utils.hpp: `toNavMap`), and as Nav2
//! 1.5.2's map saver writes a trinary map (nav2_map_server/src/map_io.cpp,
//! `tryWriteMapToFile`): each ray of a scan passes through the cells it crosses and hits the
//! cell of its end within range, and a cell that more than `min_pass_through` rays passed is
//! occupied when more than `occupancy_threshold` of them hit it, and free otherwise.

use crate::geometry::{BoundingBox, TOLERANCE, Vector2, round};
use crate::grid::{Cell, Grid};
use crate::localized_scan::{Laser, LocalizedScan, point_of};
use crate::placed_scan::Beam;
use thiserror::Error;

/// The state of a cell, as karto's grid holds it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CellState {
    #[default]
    Unknown,
    Free,
    Occupied,
}

impl CellState {
    /// The cell as `toNavMap` writes it into a `nav_msgs/OccupancyGrid`: -1, 0 or 100.
    pub fn occupancy(self) -> i8 {
        match self {
            CellState::Unknown => -1,
            CellState::Free => 0,
            CellState::Occupied => 100,
        }
    }
}

/// The grid of a map: `width` by `height` cells of `resolution` (m), row by row from the
/// lowest y. As karto keys a point, a cell holds the points whose distance from `origin`
/// (m) rounds to its column and row in cells; the map saver writes `origin` as the corner
/// of the image.
#[derive(Debug, Clone, PartialEq)]
pub struct OccupancyGrid {
    width: u32,
    height: u32,
    origin: [f64; 2],
    resolution: f64,
    cells: Vec<CellState>,
}

/// The files that Nav2's map saver writes of a grid as a trinary map
/// (nav2_map_server/src/map_io.cpp, `tryWriteMapToFile`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MapSaverFiles {
    /// The image, a pgm: `P5`, the width and the height, 255, and a byte a pixel, top row
    /// first.
    pub image: Vec<u8>,
    /// The description, the map's yaml, with no final newline.
    pub description: String,
}

/// Why cells make no grid.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GridError {
    #[error("a grid of {width} by {height} cells holds {cells} cells")]
    CellCount {
        width: u32,
        height: u32,
        cells: usize,
    },
    #[error("the side of a cell is {0} m, and it must be finite and above 0")]
    Resolution(f64),
    #[error("the origin {0:?} is not finite")]
    Origin([f64; 2]),
    #[error(
        "the scans make a grid of {width} by {height} cells, more than karto counts: give a \
         larger resolution"
    )]
    TooManyCells { width: f64, height: f64 },
}

/// The most cells a grid holds: karto counts a grid's cells, its width times its height,
/// in a 32-bit int (`Grid::m_Width * m_Height`).
const MOST_CELLS: f64 = i32::MAX as f64;

/// The parameters of a map's grid, by slam_toolbox's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GridParameters {
    pub resolution: f64,
    pub min_pass_through: u32,
    pub occupancy_threshold: f64,
}

/// The thresholds of Nav2's map saver as slam_toolbox's maps are saved, in percent: a
/// cell at or below `FREE_THRESHOLD` is free, and one at or above `OCCUPIED_THRESHOLD`
/// occupied.
const FREE_THRESHOLD: i8 = 25;
const OCCUPIED_THRESHOLD: i8 = 65;

/// Where a map's grid lies in its map frame: the side of a cell, the corner of least x and
/// least y, and the extent along x and y (m), as `room_map`'s map object carries them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridExtent {
    pub resolution_m: f64,
    pub min_x_m: f64,
    pub min_y_m: f64,
    pub width_m: f64,
    pub height_m: f64,
}

/// `OccupancyGrid::ComputeDimensions` of `scans` at `resolution` (m): the corner of the box
/// that holds every scan's box, and the width and height in cells, Round(size x (1 /
/// resolution)): a point whose distance from the corner rounds to that count, as one on the
/// box's far x or far y edge does, keys no cell. None for no scan; refused for more than
/// [`MOST_CELLS`] cells.
fn dimensions<'a>(
    scans: impl Iterator<Item = &'a LocalizedScan>,
    resolution: f64,
) -> Result<Option<(Vector2, i32, i32)>, GridError> {
    let mut bounding_box = BoundingBox::default();
    let mut any = false;
    for scan in scans {
        bounding_box.add_box(scan.bounding_box());
        any = true;
    }
    if !any {
        return Ok(None);
    }
    let scale = 1.0 / resolution;
    let size = bounding_box.size();
    let (width, height) = (round(size.x * scale), round(size.y * scale));
    let cells = width * height;
    if cells.is_nan() || cells > MOST_CELLS {
        return Err(GridError::TooManyCells { width, height });
    }
    Ok(Some((bounding_box.minimum, width as i32, height as i32)))
}

impl GridExtent {
    /// The extent of the grid that karto makes of `scans` at `resolution` (m).
    pub(crate) fn of_scans<'a>(
        scans: impl Iterator<Item = &'a LocalizedScan>,
        resolution: f64,
    ) -> Result<Option<Self>, GridError> {
        let Some((offset, width, height)) = dimensions(scans, resolution)? else {
            return Ok(None);
        };
        Ok(Some(Self {
            resolution_m: resolution,
            min_x_m: offset.x,
            min_y_m: offset.y,
            width_m: f64::from(width.max(0)) * resolution,
            height_m: f64::from(height.max(0)) * resolution,
        }))
    }
}

impl OccupancyGrid {
    /// `OccupancyGrid::CreateFromScans` of `scans`, in their order; none for no scan.
    pub(crate) fn from_scans<'a>(
        scans: impl Iterator<Item = &'a LocalizedScan> + Clone,
        laser: &Laser,
        parameters: GridParameters,
    ) -> Result<Option<Self>, GridError> {
        let Some((offset, width, height)) = dimensions(scans.clone(), parameters.resolution)?
        else {
            return Ok(None);
        };
        let mut passes: Grid<u32> = Grid::new(width, height, parameters.resolution);
        let mut hits: Grid<u32> = Grid::new(width, height, parameters.resolution);
        passes.converter.offset = offset;
        hits.converter.offset = offset;
        for scan in scans {
            add_scan(&mut passes, &mut hits, scan, laser);
        }
        let cells = (0..height)
            .flat_map(|y| (0..width).map(move |x| Cell { x, y }))
            .map(|cell| {
                let index = passes.index(cell);
                let pass = passes.at_index(index).unwrap_or_default();
                let hit = hits.at_index(index).unwrap_or_default();
                cell_state(pass, hit, parameters)
            })
            .collect();
        Ok(Some(Self {
            width: width.max(0) as u32,
            height: height.max(0) as u32,
            origin: [offset.x, offset.y],
            resolution: parameters.resolution,
            cells,
        }))
    }

    /// The grid of `cells`, `width` by `height` of side `resolution` (m), row by row from
    /// the lowest y, counted from `origin` (m).
    pub fn from_cells(
        width: u32,
        height: u32,
        resolution: f64,
        origin: [f64; 2],
        cells: Vec<CellState>,
    ) -> Result<Self, GridError> {
        if cells.len() != width as usize * height as usize {
            return Err(GridError::CellCount {
                width,
                height,
                cells: cells.len(),
            });
        }
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(GridError::Resolution(resolution));
        }
        if !origin.iter().all(|coordinate| coordinate.is_finite()) {
            return Err(GridError::Origin(origin));
        }
        Ok(Self {
            width,
            height,
            origin,
            resolution,
            cells,
        })
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    /// The position (m) from which the cells are counted.
    pub fn origin(&self) -> [f64; 2] {
        self.origin
    }

    /// The side of a cell (m).
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Where the grid lies in its map frame.
    pub fn extent(&self) -> GridExtent {
        GridExtent {
            resolution_m: self.resolution,
            min_x_m: self.origin[0],
            min_y_m: self.origin[1],
            width_m: f64::from(self.width) * self.resolution,
            height_m: f64::from(self.height) * self.resolution,
        }
    }

    /// The state of the cell at column `x` and row `y`, counted from the origin.
    pub fn cell(&self, x: u32, y: u32) -> Option<CellState> {
        (x < self.width && y < self.height).then(|| self.cells[(y * self.width + x) as usize])
    }

    /// The pixels of the trinary image that Nav2's map saver writes of the grid, top row
    /// first: an occupied cell 0, a free one 254 and an unknown one 205.
    pub fn trinary_pixels(&self) -> Vec<u8> {
        (0..self.height)
            .rev()
            .flat_map(|y| (0..self.width).map(move |x| (x, y)))
            .map(|(x, y)| trinary_pixel(self.cells[(y * self.width + x) as usize].occupancy()))
            .collect()
    }
}

impl OccupancyGrid {
    /// The files that Nav2's map saver writes of the grid as a trinary map whose image is
    /// `image_file`, at the thresholds with which slam_toolbox's maps are saved: the cell
    /// side of the map's message, in single precision, and the origin, each to three
    /// decimals, a zero heading, and the thresholds of the trinary pixels.
    pub fn map_saver_files(&self, image_file: &str) -> MapSaverFiles {
        let header = format!("P5\n{} {}\n255\n", self.width, self.height);
        let [x, y] = self.origin;
        MapSaverFiles {
            image: header
                .into_bytes()
                .into_iter()
                .chain(self.trinary_pixels())
                .collect(),
            description: format!(
                "image: {image_file}\nmode: trinary\nresolution: {:.3}\norigin: [{x:.3}, {y:.3}, \
                 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196",
                f64::from(self.resolution as f32)
            ),
        }
    }
}

/// `tryWriteMapToFile`'s trinary pixel of a cell: free at or below 25%, occupied at or
/// above 65%, unknown otherwise and outside 0 to 100.
fn trinary_pixel(occupancy: i8) -> u8 {
    if !(0..=100).contains(&occupancy) {
        205
    } else if occupancy <= FREE_THRESHOLD {
        254
    } else if OCCUPIED_THRESHOLD <= occupancy {
        0
    } else {
        205
    }
}

/// `UpdateCell`.
fn cell_state(pass: u32, hit: u32, parameters: GridParameters) -> CellState {
    if pass <= parameters.min_pass_through {
        return CellState::Unknown;
    }
    let hit_ratio = f64::from(hit) / f64::from(pass);
    if hit_ratio > parameters.occupancy_threshold {
        CellState::Occupied
    } else {
        CellState::Free
    }
}

/// `OccupancyGrid::AddScan` with no update: each ray of the scan within the laser's
/// limits traced from where the laser stood at the ray's time, cut at the range threshold,
/// and its end hit when it lies within it.
fn add_scan(passes: &mut Grid<u32>, hits: &mut Grid<u32>, scan: &LocalizedScan, laser: &Laser) {
    let scan_pose = scan.sensor_pose();
    for (beam, &point) in scan.beams.iter().zip(scan.unfiltered_point_readings()) {
        let range = beam.reading;
        let is_end_point_valid = range < laser.range_threshold - TOLERANCE;
        if range <= laser.minimum_range || range >= laser.maximum_range || range.is_nan() {
            continue;
        }
        let origin = point_of(
            scan_pose,
            &Beam {
                reading: 0.0,
                ..*beam
            },
        );
        let end = if range >= laser.range_threshold {
            let ratio = laser.range_threshold / range;
            let delta = point.minus(origin);
            Vector2::new(origin.x + ratio * delta.x, origin.y + ratio * delta.y)
        } else {
            point
        };
        ray_trace(passes, hits, origin, end, is_end_point_valid);
    }
}

/// `OccupancyGrid::RayTrace`: the cells from `from` to `to` passed, and the cell of `to`
/// passed again and hit when the end is valid.
fn ray_trace(
    passes: &mut Grid<u32>,
    hits: &mut Grid<u32>,
    from: Vector2,
    to: Vector2,
    is_end_point_valid: bool,
) {
    let grid_from = passes.world_to_grid(from);
    let grid_to = passes.world_to_grid(to);
    trace_line(passes, grid_from, grid_to);
    if is_end_point_valid && passes.is_valid(grid_to) {
        let index = passes.index(grid_to);
        for grid in [&mut *passes, &mut *hits] {
            if let Some(count) = grid.at_index_mut(index) {
                *count += 1;
            }
        }
    }
}

/// `Grid::TraceLine`: Bresenham's line from `from` to `to`, each cell of it inside the
/// grid counted once.
fn trace_line(grid: &mut Grid<u32>, from: Cell, to: Cell) {
    let (mut x0, mut y0, mut x1, mut y1) = (from.x, from.y, to.x, to.y);
    let steep = (y1 - y0).abs() > (x1 - x0).abs();
    if steep {
        std::mem::swap(&mut x0, &mut y0);
        std::mem::swap(&mut x1, &mut y1);
    }
    if x0 > x1 {
        std::mem::swap(&mut x0, &mut x1);
        std::mem::swap(&mut y0, &mut y1);
    }
    let delta_x = x1 - x0;
    let delta_y = (y1 - y0).abs();
    let mut error = 0;
    let y_step = if y0 < y1 { 1 } else { -1 };
    let mut y = y0;
    for x in x0..=x1 {
        let cell = if steep {
            Cell { x: y, y: x }
        } else {
            Cell { x, y }
        };
        error += delta_y;
        if 2 * error >= delta_x {
            y += y_step;
            error -= delta_x;
        }
        if grid.is_valid(cell) {
            let index = grid.index(cell);
            if let Some(count) = grid.at_index_mut(index) {
                *count += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_line_counts_each_cell_from_its_start_to_its_end_once() {
        let mut grid: Grid<u32> = Grid::new(10, 10, 1.0);
        trace_line(&mut grid, Cell { x: 1, y: 1 }, Cell { x: 7, y: 4 });
        let counted: u32 = (0..10)
            .flat_map(|y| (0..10).map(move |x| Cell { x, y }))
            .map(|cell| grid.value(cell).unwrap_or_default())
            .sum();
        assert_eq!(counted, 7);
        assert_eq!(grid.value(Cell { x: 1, y: 1 }), Some(1));
        assert_eq!(grid.value(Cell { x: 7, y: 4 }), Some(1));
    }

    #[test]
    fn the_map_saver_writes_free_occupied_and_unknown_cells() {
        assert_eq!(trinary_pixel(CellState::Free.occupancy()), 254);
        assert_eq!(trinary_pixel(CellState::Occupied.occupancy()), 0);
        assert_eq!(trinary_pixel(CellState::Unknown.occupancy()), 205);
        assert_eq!(trinary_pixel(50), 205);
    }

    #[test]
    fn a_cell_is_known_past_the_passes_and_occupied_past_the_threshold() {
        let parameters = GridParameters {
            resolution: 0.05,
            min_pass_through: 2,
            occupancy_threshold: 0.1,
        };
        assert_eq!(cell_state(2, 2, parameters), CellState::Unknown);
        assert_eq!(cell_state(3, 1, parameters), CellState::Occupied);
        assert_eq!(cell_state(20, 2, parameters), CellState::Free);
    }
}
