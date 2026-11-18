//! A saved occupancy grid as Nav2 1.5.2 takes one into its static layer: the
//! image read as nav2_map_server's `loadMapFromFile` reads a trinary map that
//! its YAML file describes with `negate` 0 (src/map_io.cpp), each pixel's
//! occupancy `1 - p / 255` in single precision, occupied at or above the
//! map's `occupied_thresh` and free at or below its `free_thresh`, each
//! threshold in single precision as Eigen compares a float array with it,
//! and unknown between; the image's top row the grid's last, and the
//! resolution in the single precision of the occupancy grid's metadata,
//! which the static layer gives the cost map; and each cell's cost as
//! `StaticLayer::interpretValue` gives it (nav2_costmap_2d
//! plugins/static_layer.cpp) with `track_unknown_space` and `trinary_costmap`
//! true and a lethal threshold of 100: lethal, free or unknown.

use control_core::positive_finite::PositiveFinite;

use crate::grid::{CostGrid, FREE, GridError, GridGeometry, LETHAL, UNKNOWN};

/// The thresholds of a trinary map, as its YAML file gives them: the
/// occupancy at or above which a pixel is occupied (`occupied_thresh`) and
/// the one at or below which it is free (`free_thresh`), each from 0 to 1,
/// the free one below the occupied one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapThresholds {
    occupied: f64,
    free: f64,
}

/// Why two numbers make no thresholds of a trinary map.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum MapThresholdsError {
    #[error("{field} {value} is no occupancy: give a number from 0 to 1")]
    OutOfRange { field: &'static str, value: f64 },
    #[error(
        "free_thresh {free} is not below occupied_thresh {occupied}: give a free threshold \
         below the occupied one"
    )]
    FreeNotBelowOccupied { free: f64, occupied: f64 },
}

impl MapThresholds {
    /// The thresholds `occupied_thresh` and `free_thresh` of a map's YAML
    /// file.
    pub fn parse(occupied: f64, free: f64) -> Result<Self, MapThresholdsError> {
        for (field, value) in [("occupied_thresh", occupied), ("free_thresh", free)] {
            if !(0.0..=1.0).contains(&value) {
                return Err(MapThresholdsError::OutOfRange { field, value });
            }
        }
        if free >= occupied {
            return Err(MapThresholdsError::FreeNotBelowOccupied { free, occupied });
        }
        Ok(Self { occupied, free })
    }

    /// A pixel's cost: its occupancy as map_io reads it, against the
    /// thresholds in single precision, as the static layer interprets the
    /// occupancy grid's value.
    fn cost_of(&self, pixel: u8) -> u8 {
        let occupancy = 1.0_f32 - f32::from(pixel) / 255.0_f32;
        if occupancy <= self.free as f32 {
            return FREE;
        }
        if occupancy >= self.occupied as f32 {
            return LETHAL;
        }
        UNKNOWN
    }
}

/// A saved grid's costs: [`LETHAL`], [`FREE`] or [`UNKNOWN`] a cell.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedGrid {
    grid: CostGrid,
}

/// Why an image makes no saved grid.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SavedGridError {
    #[error(transparent)]
    Grid(#[from] GridError),
    #[error(
        "{pixels} pixels fill no image of {width} by {height}: give one gray pixel a cell, row \
         by row from the top"
    )]
    PixelCount {
        pixels: usize,
        width: u32,
        height: u32,
    },
    #[error(
        "the resolution {0} m is no single precision number above 0, as an occupancy grid's \
         metadata carries it: give a resolution from 1e-38 m to 1e38 m"
    )]
    Resolution(f64),
}

impl SavedGrid {
    /// The grid of the 8-bit gray image `pixels`, `width` by `height`, row by
    /// row from the top, dark occupied, read at `thresholds`, of cells of side
    /// `resolution_m` in single precision whose first, the bottom left pixel,
    /// has its corner at `origin`: the image and the description of a map's
    /// YAML file.
    pub fn from_image(
        width: u32,
        height: u32,
        pixels: &[u8],
        resolution_m: PositiveFinite,
        origin: [f64; 2],
        thresholds: MapThresholds,
    ) -> Result<Self, SavedGridError> {
        let single = f64::from(resolution_m.get() as f32);
        let resolution = PositiveFinite::parse(single)
            .map_err(|_| SavedGridError::Resolution(resolution_m.get()))?;
        let geometry = GridGeometry::new(width, height, resolution, origin)?;
        if pixels.len() != geometry.cell_count() {
            return Err(SavedGridError::PixelCount {
                pixels: pixels.len(),
                width,
                height,
            });
        }
        let costs: Vec<u8> = pixels
            .chunks(width as usize)
            .rev()
            .flatten()
            .map(|&pixel| thresholds.cost_of(pixel))
            .collect();
        Ok(Self {
            grid: CostGrid::from_costs(geometry, costs)?,
        })
    }

    pub fn grid(&self) -> &CostGrid {
        &self.grid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tenth() -> PositiveFinite {
        PositiveFinite::parse(0.1).unwrap()
    }

    /// The thresholds 0.65 and 0.25.
    fn ticket() -> MapThresholds {
        MapThresholds::parse(0.65, 0.25).unwrap()
    }

    #[test]
    fn thresholds_are_occupancies_with_the_free_one_below() {
        assert_eq!(
            MapThresholds::parse(1.2, 0.25),
            Err(MapThresholdsError::OutOfRange {
                field: "occupied_thresh",
                value: 1.2
            })
        );
        assert!(matches!(
            MapThresholds::parse(0.65, f64::NAN),
            Err(MapThresholdsError::OutOfRange {
                field: "free_thresh",
                ..
            })
        ));
        assert_eq!(
            MapThresholds::parse(0.4, 0.4),
            Err(MapThresholdsError::FreeNotBelowOccupied {
                free: 0.4,
                occupied: 0.4
            })
        );
        assert_eq!(
            MapThresholds::parse(0.4, 0.5).unwrap_err().to_string(),
            "free_thresh 0.5 is not below occupied_thresh 0.4: give a free threshold below the \
             occupied one"
        );
    }

    // At 0.65 and 0.25 no 8-bit pixel's occupancy equals a threshold in single
    // precision, so map_io's "at or above" and "at or below" pick the pixels
    // above and below them.
    #[test]
    fn no_pixel_lies_on_the_thresholds_of_0_65_and_0_25() {
        let thresholds = ticket();
        for pixel in 0..=u8::MAX {
            let occupancy = 1.0_f32 - f32::from(pixel) / 255.0_f32;
            assert_ne!(occupancy, thresholds.occupied as f32, "pixel {pixel}");
            assert_ne!(occupancy, thresholds.free as f32, "pixel {pixel}");
        }
    }

    // Black is lethal, white free and the gray between unknown, the image's
    // top row is the grid's last, and the resolution is single precision.
    #[test]
    fn dark_is_lethal_light_free_and_the_top_row_last() {
        let saved = SavedGrid::from_image(
            3,
            2,
            &[0, 89, 90, 254, 192, 191],
            tenth(),
            [0.0, 0.0],
            ticket(),
        )
        .unwrap();
        assert_eq!(
            saved.grid().costs(),
            [FREE, FREE, UNKNOWN, LETHAL, LETHAL, UNKNOWN]
        );
        assert!(matches!(
            SavedGrid::from_image(3, 2, &[0; 5], tenth(), [0.0; 2], ticket()),
            Err(SavedGridError::PixelCount { pixels: 5, .. })
        ));
        assert_eq!(saved.grid().geometry().resolution_m(), f64::from(0.1_f32));
        assert!(matches!(
            SavedGrid::from_image(
                3,
                2,
                &[0; 6],
                PositiveFinite::parse(1e-50).unwrap(),
                [0.0; 2],
                ticket()
            ),
            Err(SavedGridError::Resolution(_))
        ));
    }

    // map_io compares a pixel's single precision occupancy with each threshold
    // made single precision: a pixel whose occupancy lies a hair below
    // occupied_thresh in double precision, and on it in single, is occupied.
    #[test]
    fn a_threshold_compares_in_single_precision() {
        let occupancy = f64::from(1.0_f32 - f32::from(89_u8) / 255.0_f32);
        let thresholds = MapThresholds::parse(occupancy + 1e-12, 0.25).unwrap();
        let saved = SavedGrid::from_image(1, 1, &[89], tenth(), [0.0; 2], thresholds).unwrap();
        assert_eq!(saved.grid().costs(), [LETHAL]);
    }

    // A map as map_saver writes it in trinary mode, its occupied pixels 0, its
    // free ones 254 and its unknown ones 205, with occupied_thresh 0.65 and
    // free_thresh 0.196 in its YAML file, reads back lethal, free and unknown.
    #[test]
    fn a_map_saver_map_reads_back_lethal_free_and_unknown() {
        let thresholds = MapThresholds::parse(0.65, 0.196).unwrap();
        let saved =
            SavedGrid::from_image(3, 1, &[0, 254, 205], tenth(), [0.0; 2], thresholds).unwrap();
        assert_eq!(saved.grid().costs(), [LETHAL, FREE, UNKNOWN]);
    }
}
