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
// This file translates into Rust karto's grid as slam_toolbox 2.10.0 carries it:
// lib/karto_sdk/include/karto_sdk/Karto.h (Copyright 2010 SRI International).
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

//! karto's grid (slam_toolbox 2.10.0, lib/karto_sdk/include/karto_sdk/Karto.h:
//! `CoordinateConverter` and `Grid<T>`): cells in rows whose width karto rounds up to a
//! multiple of 8, indexed from a world offset at a scale of one over the resolution.

use crate::geometry::{Vector2, round, truncate_to_i32};

/// A cell's column and row, which may lie outside the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Cell {
    pub x: i32,
    pub y: i32,
}

/// `CoordinateConverter`: world positions to cells.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CoordinateConverter {
    /// Cells per meter: one over the resolution.
    pub scale: f64,
    /// The world position (m) of cell (0, 0).
    pub offset: Vector2,
}

impl CoordinateConverter {
    /// `WorldToGrid` with no flip: the cell nearest `world`, rounded half away from zero,
    /// and converted to ints as the amd64 build converts them.
    pub fn world_to_grid(&self, world: Vector2) -> Cell {
        let x = (world.x - self.offset.x) * self.scale;
        let y = (world.y - self.offset.y) * self.scale;
        Cell {
            x: truncate_to_i32(round(x)),
            y: truncate_to_i32(round(y)),
        }
    }

    /// `GetResolution`: one over the scale (m).
    pub fn resolution(&self) -> f64 {
        1.0 / self.scale
    }
}

/// `Grid<T>`: `width` by `height` cells in rows of `width_step` cells.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Grid<T> {
    width: i32,
    height: i32,
    width_step: i32,
    data: Vec<T>,
    pub converter: CoordinateConverter,
}

/// `math::AlignValue<kt_int32s>(width, 8)`.
fn width_step(width: i32) -> i32 {
    (width + 7) & !7
}

impl<T: Copy + Default> Grid<T> {
    /// `Grid::CreateGrid`: every cell at the default value, with no offset.
    pub fn new(width: i32, height: i32, resolution: f64) -> Self {
        let step = width_step(width);
        Self {
            width,
            height,
            width_step: step,
            data: vec![T::default(); (step.max(0) as usize) * (height.max(0) as usize)],
            converter: CoordinateConverter {
                scale: 1.0 / resolution,
                offset: Vector2::default(),
            },
        }
    }

    /// `Clear`: every cell back to the default value.
    pub fn clear(&mut self) {
        self.data.fill(T::default());
    }

    pub fn width(&self) -> i32 {
        self.width
    }

    pub fn height(&self) -> i32 {
        self.height
    }

    /// `GetDataSize`: the cells of the rows, at their aligned width.
    pub fn data_size(&self) -> i32 {
        self.width_step * self.height
    }

    /// `IsValidGridIndex`.
    pub fn is_valid(&self, cell: Cell) -> bool {
        (0..self.width).contains(&cell.x) && (0..self.height).contains(&cell.y)
    }

    /// `GridIndex(cell, false)`: the index of `cell` in the rows, which may lie outside
    /// them, with the int arithmetic of the amd64 build.
    pub fn index(&self, cell: Cell) -> i32 {
        cell.x.wrapping_add(cell.y.wrapping_mul(self.width_step))
    }

    /// The cell at `index`, when the index lies inside the rows.
    pub fn at_index(&self, index: i32) -> Option<T> {
        usize::try_from(index)
            .ok()
            .and_then(|index| self.data.get(index))
            .copied()
    }

    /// The value of `cell`, which lies inside the grid.
    pub fn value(&self, cell: Cell) -> Option<T> {
        self.is_valid(cell)
            .then(|| self.data[self.index(cell) as usize])
    }

    /// A mutable reference to the value at `index`, which lies inside the rows.
    pub fn at_index_mut(&mut self, index: i32) -> Option<&mut T> {
        usize::try_from(index)
            .ok()
            .and_then(|index| self.data.get_mut(index))
    }

    pub fn world_to_grid(&self, world: Vector2) -> Cell {
        self.converter.world_to_grid(world)
    }

    pub fn resolution(&self) -> f64 {
        self.converter.resolution()
    }
}
