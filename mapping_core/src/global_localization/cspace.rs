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
// This file translates into Rust Nav2 1.5.2's distance map of AMCL,
// nav2_amcl/src/map/map_cspace.cpp (Copyright (C) 2000 Brian Gerkey & Kasper Stoy), with each
// occupied cell entering the queue at distance 0.
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

//! The distance from each cell of AMCL's map to its nearest occupied cell, as nav2_amcl
//! computes it (src/map/map_cspace.cpp: `CachedDistanceMap`, `enqueue`,
//! `map_update_cspace`), with each occupied cell entering the queue at distance 0, as the
//! ROS reference harness's copy of the function sets it
//! (ros_reference/reference_programs/amcl_map_cspace.cpp). The occupied cells enter a
//! `std::priority_queue` of the least distance first, column by column; the cell on top
//! offers each of its four neighbors that no cell claimed yet its own nearest occupied
//! cell, a neighbor within the reach taking the distance to it, and only then leaves the
//! queue, so a neighbor it offered nearer than itself takes the top and leaves in its
//! place. The queue gives the order of libstdc++'s `std::priority_queue` (`libstdcxx_heap`),
//! since the order in which cells of equal distance leave it decides which occupied cell
//! claims a cell.

use crate::occupancy_grid::CellState;
use libstdcxx_heap::OpenSet;

/// A cell in the queue and the occupied cell whose distance it took, of `CellData`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CellData {
    i: i32,
    j: i32,
    source_i: i32,
    source_j: i32,
}

/// `CachedDistanceMap`: the distance (cells) between two cells `di` and `dj` apart, for
/// each up to the reach and one.
struct CachedDistances {
    /// The reach (cells): the largest distance over the side of a cell, truncated.
    cell_radius: i32,
    /// `distances_[di][dj]`, row by row.
    distances: Vec<f64>,
}

impl CachedDistances {
    fn new(scale: f64, max_distance: f64) -> Self {
        let cell_radius = (max_distance / scale) as i32;
        let side = cell_radius + 2;
        let distances = (0..side)
            .flat_map(|i| (0..side).map(move |j| f64::from(i * i + j * j).sqrt()))
            .collect();
        Self {
            cell_radius,
            distances,
        }
    }

    fn distance(&self, di: i32, dj: i32) -> f64 {
        self.distances
            [usize::try_from(di * (self.cell_radius + 2) + dj).expect("an offset within the reach")]
    }
}

/// `map_update_cspace`: the distance (m), in single precision, from each cell of a map
/// `size_x` by `size_y` cells of side `scale` (m), whose states `states` holds by
/// `MAP_INDEX`, to its nearest occupied cell within `max_distance` (m), and `max_distance`
/// for a cell that none lies so near.
pub(crate) fn occupied_distances(
    size_x: i32,
    size_y: i32,
    scale: f64,
    states: &[CellState],
    max_distance: f64,
) -> Vec<f32> {
    let index = |i: i32, j: i32| super::map::index(size_x, i, j);
    let cached = CachedDistances::new(scale, max_distance);
    let mut distances = vec![0.0f32; states.len()];
    let mut marked = vec![false; states.len()];
    let mut queue = OpenSet::new();
    for i in 0..size_x {
        for j in 0..size_y {
            let cell = index(i, j);
            if states[cell] == CellState::Occupied {
                distances[cell] = 0.0;
                marked[cell] = true;
                queue.push(
                    0.0,
                    CellData {
                        i,
                        j,
                        source_i: i,
                        source_j: j,
                    },
                );
            } else {
                distances[cell] = max_distance as f32;
            }
        }
    }
    // `enqueue`: the cell (i, j) takes the distance to the occupied cell of `from` and
    // enters the queue, unless a cell claimed it or the occupied cell lies beyond the reach.
    let mut enqueue = |queue: &mut OpenSet<CellData>, i: i32, j: i32, from: CellData| {
        let cell = index(i, j);
        if marked[cell] {
            return;
        }
        let distance = cached.distance((i - from.source_i).abs(), (j - from.source_j).abs());
        if distance > f64::from(cached.cell_radius) {
            return;
        }
        distances[cell] = (distance * scale) as f32;
        queue.push(
            distances[cell],
            CellData {
                i,
                j,
                source_i: from.source_i,
                source_j: from.source_j,
            },
        );
        marked[cell] = true;
    };
    while let Some((_, current)) = queue.peek() {
        if current.i > 0 {
            enqueue(&mut queue, current.i - 1, current.j, current);
        }
        if current.j > 0 {
            enqueue(&mut queue, current.i, current.j - 1, current);
        }
        if current.i < size_x - 1 {
            enqueue(&mut queue, current.i + 1, current.j, current);
        }
        if current.j < size_y - 1 {
            enqueue(&mut queue, current.i, current.j + 1, current);
        }
        queue.pop();
    }
    distances
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The distances of a map `size` cells a side, of side 0.1 m, whose occupied cells
    /// are `occupied`, within 0.35 m.
    fn distances_of(size: i32, occupied: &[(i32, i32)]) -> Vec<f32> {
        let states: Vec<CellState> = (0..size)
            .flat_map(|j| (0..size).map(move |i| (i, j)))
            .map(|cell| {
                if occupied.contains(&cell) {
                    CellState::Occupied
                } else {
                    CellState::Free
                }
            })
            .collect();
        occupied_distances(size, size, 0.1, &states, 0.35)
    }

    #[test]
    fn about_one_occupied_cell_each_cell_within_the_reach_takes_its_distance() {
        let distances = distances_of(9, &[(4, 4)]);
        for j in 0..9 {
            for i in 0..9 {
                let cells = f64::from((i - 4) * (i - 4) + (j - 4) * (j - 4)).sqrt();
                let expected = if cells <= 3.0 {
                    (cells * 0.1) as f32
                } else {
                    0.35
                };
                assert_eq!(distances[(i + j * 9) as usize], expected, "cell ({i}, {j})");
            }
        }
    }

    #[test]
    fn each_cell_takes_the_distance_to_an_occupied_cell_no_nearer_than_its_nearest() {
        let occupied = [(1, 1), (6, 2), (3, 7)];
        let distances = distances_of(9, &occupied);
        for j in 0..9 {
            for i in 0..9 {
                let nearest = occupied
                    .iter()
                    .map(|&(oi, oj)| {
                        f64::from((i - oi) * (i - oi) + (j - oj) * (j - oj)).sqrt() * 0.1
                    })
                    .fold(f64::INFINITY, f64::min);
                let distance = f64::from(distances[(i + j * 9) as usize]);
                assert!(
                    distance + 1e-6 >= nearest.min(0.35),
                    "cell ({i}, {j}): {distance} m"
                );
            }
        }
    }
}
