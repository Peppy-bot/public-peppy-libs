//! Parity with Nav2 1.5.2's map saver. For each grid of the map export scenarios,
//! `OccupancyGrid::map_saver_files` gives the bytes of the image and the text of the
//! description that `nav2_map_server::saveMapToFile` wrote as a trinary map, byte for byte.
//! The map saver writes text and whole pixels, with no arithmetic that rounds by
//! architecture, so the test compares on every architecture.

use mapping_core::{CellState, OccupancyGrid};
use ros_reference::map_export::{Cases, Grid};

/// The image file that the reference's description names.
const IMAGE_FILE: &str = "map.pgm";

fn occupancy_grid(name: &str, grid: &Grid) -> OccupancyGrid {
    let [x, y, yaw] = grid.origin;
    assert_eq!(yaw, 0.0, "{name}: a map's grid lies along its map frame");
    let cells = grid
        .cells
        .iter()
        .map(|&cell| match cell {
            -1 => CellState::Unknown,
            0 => CellState::Free,
            100 => CellState::Occupied,
            other => panic!("{name}: a map's grid holds no cell {other}"),
        })
        .collect();
    OccupancyGrid::from_cells(
        grid.width,
        grid.height,
        f64::from(grid.resolution),
        [x, y],
        cells,
    )
    .unwrap_or_else(|e| panic!("{name}: {e}"))
}

#[test]
fn each_grid_exports_as_the_map_saver_writes_it() {
    let all = Cases::load_all();
    assert!(!all.is_empty(), "the harness holds a map export scenario");
    for cases in all {
        assert_eq!(
            (cases.free_thresh, cases.occupied_thresh),
            (0.25, 0.65),
            "{}: the map saver's thresholds when its caller sets none",
            cases.name
        );
        for (index, (grid, export)) in cases.maps.iter().enumerate() {
            let files = occupancy_grid(&cases.name, grid).map_saver_files(IMAGE_FILE);
            assert_eq!(
                files.description, export.description,
                "{} grid {index}: the description",
                cases.name
            );
            assert_eq!(
                files.image, export.image,
                "{} grid {index}: the image",
                cases.name
            );
        }
    }
}
