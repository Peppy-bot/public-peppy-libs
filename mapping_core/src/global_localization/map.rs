//  Player - One Hell of a Robot Server
//  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
//                      gerkey@usc.edu    kaspers@robotics.usc.edu
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//  Copyright (c) 2008, Willow Garage, Inc.
//  All rights reserved.
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// This file translates into Rust Nav2 1.5.2's map of AMCL:
// nav2_amcl/include/nav2_amcl/map/map.hpp (Copyright (C) 2000 Brian Gerkey & Kasper Stoy) and
// nav2_amcl/src/amcl_node.cpp (Copyright (c) 2008, Willow Garage, Inc.).
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

//! The map as AMCL holds it (nav2_amcl include/nav2_amcl/map/map.hpp: `map_t` and the
//! macros `MAP_WXGX`, `MAP_GXWX`, `MAP_VALID` and `MAP_INDEX`; src/amcl_node.cpp:
//! `convertMap`, `createFreeSpaceVector` and `uniformPoseGenerator`). AMCL reads the
//! occupancy grid of a map as a `nav_msgs/OccupancyGrid` carries it, the resolution in
//! single precision, and counts its cells from the middle cell: the map's origin is the
//! position of cell `size / 2` (an integer division), and a position lies in the cell whose
//! center it rounds to. Each cell holds the distance to the nearest occupied cell that
//! [`super::cspace`] computes from the cells' states, and the free cells are listed for
//! global localization to spread its particles over.

use super::cspace;
use super::drand48::Drand48;
use crate::geometry::{Pose2, truncate_to_i32};
use crate::occupancy_grid::{CellState, OccupancyGrid};
use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Map {
    size_x: i32,
    size_y: i32,
    /// The side of a cell (m).
    scale: f64,
    /// The position (m) of the cell `(size_x / 2, size_y / 2)`.
    origin_x: f64,
    origin_y: f64,
    /// `occ_dist` of each cell (m), `MAP_INDEX` by `MAP_INDEX`.
    occupied_distances: Vec<f32>,
    /// `max_occ_dist` (m): the distance of a cell that no occupied cell lies near, and of a
    /// position off the map.
    max_occupied_distance: f64,
    /// `free_space_indices`: the free cells, column by column.
    free_cells: Vec<[i32; 2]>,
}

impl Map {
    /// `convertMap` of `grid`, as its `nav_msgs/OccupancyGrid` carries it, with the
    /// distances that `map_update_cspace` computes up to `max_occupied_distance` (m), and
    /// `createFreeSpaceVector` with no downsampling.
    pub fn of_grid(grid: &OccupancyGrid, max_occupied_distance: f64) -> Self {
        let size_x =
            i32::try_from(grid.width()).expect("a grid's width is an int, as karto sizes it");
        let size_y =
            i32::try_from(grid.height()).expect("a grid's height is an int, as karto sizes it");
        let scale = f64::from(grid.resolution() as f32);
        let [x, y] = grid.origin();
        let states: Vec<CellState> = (0..grid.height())
            .flat_map(|j| (0..grid.width()).map(move |i| (i, j)))
            .map(|(i, j)| grid.cell(i, j).expect("a cell of the grid"))
            .collect();
        let occupied_distances =
            cspace::occupied_distances(size_x, size_y, scale, &states, max_occupied_distance);
        let free_cells = (0..size_x)
            .flat_map(|i| (0..size_y).map(move |j| [i, j]))
            .filter(|&[i, j]| states[index(size_x, i, j)] == CellState::Free)
            .collect();
        Self {
            size_x,
            size_y,
            scale,
            origin_x: x + f64::from(size_x / 2) * scale,
            origin_y: y + f64::from(size_y / 2) * scale,
            occupied_distances,
            max_occupied_distance,
            free_cells,
        }
    }

    pub fn free_cells(&self) -> &[[i32; 2]] {
        &self.free_cells
    }

    /// `uniformPoseGenerator`: the center of a free cell drawn uniformly, at a heading
    /// drawn uniformly in [-pi, pi). The map holds a free cell.
    pub fn uniform_pose(&self, generator: &mut Drand48) -> Pose2 {
        let rand_index = (generator.draw() * self.free_cells.len() as f64) as u32;
        let [i, j] = self.free_cells[rand_index as usize];
        let x = self.origin_x + f64::from(i - self.size_x / 2) * self.scale;
        let y = self.origin_y + f64::from(j - self.size_y / 2) * self.scale;
        Pose2::new(x, y, generator.draw() * 2.0 * PI - PI)
    }

    /// The distance (m) from the position `(x, y)` to the nearest occupied cell, as the
    /// likelihood field reads it: its cell's, or the largest off the map.
    pub fn occupied_distance_at(&self, x: f64, y: f64) -> f64 {
        let i = truncate_to_i32(
            ((x - self.origin_x) / self.scale + 0.5).floor() + f64::from(self.size_x / 2),
        );
        let j = truncate_to_i32(
            ((y - self.origin_y) / self.scale + 0.5).floor() + f64::from(self.size_y / 2),
        );
        if (0..self.size_x).contains(&i) && (0..self.size_y).contains(&j) {
            f64::from(self.occupied_distances[index(self.size_x, i, j)])
        } else {
            self.max_occupied_distance
        }
    }
}

/// `MAP_INDEX`: the index of the cell `(i, j)` of a map `size_x` cells wide.
pub(crate) fn index(size_x: i32, i: i32, j: i32) -> usize {
    usize::try_from(i + j * size_x).expect("a cell of the map")
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    #[hegel::test]
    fn the_poses_of_global_localization_lie_at_free_cells_alone(tc: TestCase) {
        let width: u32 = tc.draw(gs::integers().min_value(1).max_value(30));
        let height: u32 = tc.draw(gs::integers().min_value(1).max_value(30));
        let states = [CellState::Free, CellState::Occupied, CellState::Unknown];
        let mut cells: Vec<CellState> = (0..width * height)
            .map(|_| states[tc.draw(gs::integers::<usize>().min_value(0).max_value(2))])
            .collect();
        let free: usize = tc.draw(gs::integers().min_value(0).max_value(cells.len() - 1));
        cells[free] = CellState::Free;
        let origin = [
            tc.draw(gs::floats::<f64>().min_value(-20.0).max_value(20.0)),
            tc.draw(gs::floats::<f64>().min_value(-20.0).max_value(20.0)),
        ];
        let resolution = 0.05;
        let grid =
            OccupancyGrid::from_cells(width, height, resolution, origin, cells).expect("a grid");
        let map = Map::of_grid(&grid, 2.0);
        let mut generator = Drand48::seeded(tc.draw(gs::integers()));
        for _ in 0..200 {
            let pose = map.uniform_pose(&mut generator);
            // AMCL places cell (i, j) at the origin plus i and j cells.
            let cell = |position: f64, origin: f64| ((position - origin) / resolution).round();
            let (i, j) = (cell(pose.x, origin[0]), cell(pose.y, origin[1]));
            assert_eq!(
                grid.cell(i as u32, j as u32),
                Some(CellState::Free),
                "{pose:?} lies at the cell ({i}, {j})"
            );
            assert!((-PI..PI).contains(&pose.yaw), "{pose:?}");
        }
    }
}
