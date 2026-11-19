//! The search against Nav2's AMCL (nav2_amcl 1.5.2) on drawn rooms: `GlobalLocalization`
//! ports AMCL's global localization, its particle filter, its likelihood field model and
//! its motion models, so in a map of the room's map scans, whose grid is karto's grid of the
//! same scans cell for cell, with the same odometry, scans and seed, the search answers
//! after the scans that AMCL publishes an `amcl_pose` after and after no other, each
//! answer's pose and deviations within 1e-12 of AMCL's. The two sides compute the same
//! operations on the same draws, and their sines, cosines, arc tangents, exponentials and
//! logarithms come from the libm variant that each process runs. The evidence: every room
//! and search of every AMCL global localization scenario of the ROS reference harness. The
//! test runs on amd64, whose image the committed outputs come from: resampling and the KLD
//! sampling's bins turn on comparisons that another build's rounding can flip.

#![cfg(target_arch = "x86_64")]

mod common;

use camera_core::laser_scan::LaserScan;
use common::properties::planar;
use common::{
    MAPPING_START_UNIX_NS, amcl_parameters, localization_parameters, placed, scratch_directory,
};
use control_core::positive_finite::PositiveFinite;
use mapping_core::{
    CellState, GlobalLocalization, GlobalLocalizationParameters, LocalizationLimits, MapPose,
    OccupancyGrid, RoomMap, SearchStep,
};
use ros_reference::amcl_global_localization::{
    Cases, Estimate, Grid, GridParameters, Laser, MapScan,
};
use serde_json::json;
use std::path::Path;

/// How far a coordinate (m, rad) or a deviation (m, rad) of an answer may lie from AMCL's.
const BOUND: f64 = 1e-12;

/// The search limits (m, rad), below every deviation that the drives leave the filter at,
/// so each search runs over its whole drive.
const NEVER_ENDS: f64 = 1e-6;

fn limit(value: f64) -> PositiveFinite {
    PositiveFinite::parse(value).expect("a positive limit")
}

/// The angle (rad) of ray `index` of `laser`, as `PlacedScan` places a ray of a sensor at
/// the base's origin facing ahead.
fn ray_angle(laser: Laser, index: usize) -> f64 {
    f64::from(laser.angle_min) + index as f64 * f64::from(laser.angle_increment)
}

/// The lidar's last angle, as a `sensor_msgs/LaserScan` carries it.
fn angle_max(laser: Laser) -> f32 {
    laser.angle_min + (laser.rays - 1) as f32 * laser.angle_increment
}

/// Writes into `directory` a saved map whose pose graph holds `map_scans`, each at its pose,
/// with no link, as slam_toolbox's laser of `laser` reads them, and whose description and
/// image are those of karto's `grid` of them: the map loads when its grid is karto's, cell
/// for cell.
fn save_map_of_scans(
    directory: &Path,
    laser: Laser,
    parameters: GridParameters,
    map_scans: &[MapScan],
    grid: &Grid,
) {
    let pose = |[x, y, yaw]: [f64; 3]| json!({"x": x, "y": y, "yaw": yaw});
    let nodes: Vec<_> = map_scans
        .iter()
        .enumerate()
        .map(|(id, scan)| {
            let beams: Vec<_> = scan
                .ranges
                .iter()
                .enumerate()
                .map(|(index, &range)| {
                    json!({
                        "reading": f64::from(range),
                        "origin": [0.0, 0.0],
                        "bearing": ray_angle(laser, index),
                    })
                })
                .collect();
            json!({
                "id": id,
                "time_s": 0.0,
                "odometric_pose": pose(scan.pose),
                "corrected_pose": pose(scan.pose),
                "beams": beams,
            })
        })
        .collect();
    let pose_graph = json!({
        "laser": {
            "minimum_angle": f64::from(laser.angle_min),
            "maximum_angle": f64::from(angle_max(laser)),
            "angular_resolution": f64::from(laser.angle_increment),
            "minimum_range": f64::from(laser.range_min),
            "maximum_range": f64::from(laser.range_max),
            "range_threshold": f64::from(laser.range_max),
            "readings": laser.rays,
        },
        "nodes": nodes,
        "edges": [],
        "running_scans": (0..map_scans.len()).collect::<Vec<_>>(),
        "last_scan": map_scans.len() - 1,
        "next_id": map_scans.len(),
    });
    let meta = json!({
        "resolution": parameters.resolution,
        "min_pass_through": parameters.min_pass_through,
        "occupancy_threshold": parameters.occupancy_threshold,
        "origin": grid.origin,
        "width": grid.width,
        "height": grid.height,
        "scans": map_scans.len(),
        "mapping_start_unix_ns": MAPPING_START_UNIX_NS,
    });
    let cell = |character: char| match character {
        '#' => CellState::Occupied,
        '.' => CellState::Free,
        '?' => CellState::Unknown,
        other => panic!("karto's grid holds a cell {other:?}"),
    };
    let karto_grid = OccupancyGrid::from_cells(
        grid.width,
        grid.height,
        parameters.resolution,
        grid.origin,
        grid.rows
            .iter()
            .flat_map(|row| row.chars().map(cell))
            .collect(),
    )
    .expect("karto's grid");
    let files = karto_grid.map_saver_files("map.pgm");
    std::fs::create_dir_all(directory).expect("the map's directory");
    let write = |file: &str, bytes: &[u8]| {
        std::fs::write(directory.join(file), bytes).expect("the map's file");
    };
    write("pose_graph.json", pose_graph.to_string().as_bytes());
    write("meta.json", meta.to_string().as_bytes());
    write("map.pgm", &files.image);
    write("map.yaml", files.description.as_bytes());
}

/// The largest difference between `answer` and AMCL's `estimate`: of the positions' and the
/// headings' coordinates, the heading's wrapped, and of the deviations.
fn difference(answer: &MapPose, estimate: &Estimate) -> f64 {
    let turn = answer.pose.yaw() - estimate.pose.yaw;
    [
        answer.pose.x() - estimate.pose.x,
        answer.pose.y() - estimate.pose.y,
        turn.sin().atan2(turn.cos()),
        answer.deviations.position_std_m - estimate.deviations.position_std_m,
        answer.deviations.yaw_std_rad - estimate.deviations.yaw_std_rad,
    ]
    .into_iter()
    .map(f64::abs)
    .fold(0.0, f64::max)
}

#[test]
fn each_search_answers_as_amcl_publishes() {
    let all = Cases::load_all();
    assert!(
        !all.is_empty(),
        "the harness holds an AMCL global localization scenario"
    );
    for cases in all {
        let laser = cases.laser;
        for room in &cases.rooms {
            let label = format!("{} {}", cases.name, room.name);
            let directory = scratch_directory(&room.name);
            save_map_of_scans(&directory, laser, cases.grid, &room.map_scans, &room.grid);
            let map = RoomMap::load(&directory).unwrap_or_else(|e| {
                panic!("{label}: the map's grid is karto's, cell for cell: {e}")
            });
            let scans: Vec<LaserScan> = room
                .scans
                .iter()
                .map(|scan| {
                    LaserScan::parse(
                        laser.angle_min,
                        laser.angle_increment,
                        0.0,
                        laser.range_min,
                        laser.range_max,
                        &scan.ranges,
                    )
                    .expect("a drawn scan")
                })
                .collect();
            for (search, estimates) in &room.searches {
                let label = format!("{label} {}", search.name);
                let parameters = GlobalLocalizationParameters {
                    amcl: amcl_parameters(&label, &search.amcl),
                    search_limits: LocalizationLimits {
                        position_std_m: limit(NEVER_ENDS),
                        yaw_std_rad: limit(NEVER_ENDS),
                    },
                    localization: localization_parameters(),
                };
                let mut search_state = Some(
                    GlobalLocalization::start(&map, parameters, search.seed)
                        .unwrap_or_else(|e| panic!("{label}: {e}")),
                );
                let mut answers: Vec<(usize, MapPose)> = Vec::new();
                for (index, (scan, drive)) in scans.iter().zip(&room.scans).enumerate() {
                    let [x, y, yaw] = drive.odometry;
                    let placed_scan = placed(drive.stamp_ns, planar(x, y, yaw), scan);
                    match search_state.take().expect("a search").add(&placed_scan) {
                        SearchStep::Searching(going_on, answer) => {
                            search_state = Some(going_on);
                            answers.extend(answer.map(|answer| (index, answer)));
                        }
                        SearchStep::Found(..) => {
                            panic!("{label}: the search ends at scan {index}, within {NEVER_ENDS}")
                        }
                    }
                }
                let answered: Vec<usize> = answers.iter().map(|(index, _)| *index).collect();
                let published: Vec<usize> =
                    estimates.iter().map(|estimate| estimate.scan).collect();
                assert_eq!(
                    answered, published,
                    "{label}: the search answers after the scans that AMCL publishes after"
                );
                let largest = answers
                    .iter()
                    .zip(estimates)
                    .map(|((_, answer), estimate)| difference(answer, estimate))
                    .fold(0.0, f64::max);
                println!(
                    "{label}: {} answers, the largest difference from AMCL's {largest:e}",
                    answers.len()
                );
                assert!(
                    largest <= BOUND,
                    "{label}: an answer lies {largest} from AMCL's"
                );
            }
        }
    }
}
