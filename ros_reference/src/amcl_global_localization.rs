//! AMCL's global localization. The reference is Nav2's AMCL: in the map that karto builds of
//! each room's map scans at their poses, as slam_toolbox publishes the grid of its map, each
//! search of the room starts AMCL's global localization with `drand48` seeded by the search
//! and takes the room's drive scan by scan, and gives the `amcl_pose` it publishes after
//! each scan that it publishes one after: the mean of the filter's heaviest cluster
//! (`getMaxWeightHyp`) and the covariance of the whole filter.

use crate::metrics::{Deviations, PlanarPose};
use crate::room_dataset::{PoseCovariance, RosParameters};
use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of an AMCL global localization scenario and of its reference output, which
/// refuses every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "amcl_global_localization")]
    AmclGlobalLocalization,
}

/// The lidar of every scan: its first angle and the angle between two rays (rad), its rays
/// a scan, and its range limits (m), each angle and limit in single precision. Every ray
/// stands at its scan's stamp.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Laser {
    pub angle_min: f32,
    pub angle_increment: f32,
    pub rays: usize,
    pub range_min: f32,
    pub range_max: f32,
}

/// The parameters of the grid that karto builds of a room's map scans: the side of a cell
/// (m), and the passes and the share of hits that make a cell occupied.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GridParameters {
    pub resolution: f64,
    pub min_pass_through: u32,
    pub occupancy_threshold: f64,
}

/// A scan of a room's map: the lidar's pose in the map frame `[x, y, yaw]` (m, rad), and
/// its ranges (m).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapScan {
    pub pose: [f64; 3],
    pub ranges: Vec<f32>,
}

/// A scan of a room's drive: its stamp (ns), the base's pose in the odometry frame at the
/// stamp `[x, y, yaw]` (m, rad), and its ranges (m).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveScan {
    pub stamp_ns: i64,
    pub odometry: [f64; 3],
    pub ranges: Vec<f32>,
}

/// A search of a room's drive: its name, the seed of `srand48`, and AMCL's parameters.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Search {
    pub name: String,
    pub seed: u32,
    pub amcl: RosParameters,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScenarioRoom {
    name: String,
    map_scans: Vec<MapScan>,
    scans: Vec<DriveScan>,
    searches: Vec<Search>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    laser: Laser,
    grid: GridParameters,
    rooms: Vec<ScenarioRoom>,
}

/// karto's grid of a room's map scans: the corner of its first cell (m), its size in cells,
/// and its cells, a row a string from the lowest y, `#` occupied, `.` free and `?` unknown.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Grid {
    pub origin: [f64; 2],
    pub width: u32,
    pub height: u32,
    pub rows: Vec<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceEstimate {
    scan: usize,
    pose: [f64; 3],
    covariance: PoseCovariance,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceSearch {
    name: String,
    estimates: Vec<ReferenceEstimate>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceRoom {
    name: String,
    grid: Grid,
    searches: Vec<ReferenceSearch>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    rooms: Vec<ReferenceRoom>,
}

/// An `amcl_pose` that AMCL published: the index of its scan in the drive, the pose in the
/// map frame, and the deviations of the whole filter's covariance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    pub scan: usize,
    pub pose: PlanarPose,
    pub deviations: Deviations,
}

/// A room of a scenario with what AMCL gave: its map scans, its drive, karto's grid of the
/// map, and each search with the estimates AMCL published, in the drive's order.
#[derive(Debug, Clone, PartialEq)]
pub struct Room {
    pub name: String,
    pub map_scans: Vec<MapScan>,
    pub scans: Vec<DriveScan>,
    pub grid: Grid,
    pub searches: Vec<(Search, Vec<Estimate>)>,
}

/// An AMCL global localization scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub laser: Laser,
    pub grid: GridParameters,
    pub rooms: Vec<Room>,
}

impl Cases {
    /// Every AMCL global localization scenario under `scenarios/`, in the order of their
    /// names, each room paired with karto's grid and each search with AMCL's estimates.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::AmclGlobalLocalization)
            .into_iter()
            .map(|name| {
                let Scenario {
                    kind: ThisKind::AmclGlobalLocalization,
                    laser,
                    grid,
                    rooms,
                } = read_scenario(&name);
                let Reference {
                    kind: ThisKind::AmclGlobalLocalization,
                    rooms: references,
                } = read_reference(&name);
                assert_eq!(
                    rooms.len(),
                    references.len(),
                    "{name}: AMCL searched every room"
                );
                let rooms = rooms
                    .into_iter()
                    .zip(references)
                    .map(|(room, reference)| paired(&name, laser, room, reference))
                    .collect();
                Cases {
                    name,
                    laser,
                    grid,
                    rooms,
                }
            })
            .collect()
    }
}

/// `room` with its reference, each search with its estimates, checked to name the same
/// room and searches, every scan to hold the lidar's rays, and every estimate to name a
/// scan of the drive, in order.
fn paired(name: &str, laser: Laser, room: ScenarioRoom, reference: ReferenceRoom) -> Room {
    let room_name = &room.name;
    assert_eq!(
        room_name, &reference.name,
        "{name}: the reference names the scenario's rooms in order"
    );
    let rays = room
        .map_scans
        .iter()
        .map(|scan| scan.ranges.len())
        .chain(room.scans.iter().map(|scan| scan.ranges.len()));
    assert!(
        rays.into_iter().all(|count| count == laser.rays),
        "{name} {room_name}: every scan holds the lidar's {} rays",
        laser.rays
    );
    assert_eq!(
        room.searches.len(),
        reference.searches.len(),
        "{name} {room_name}: AMCL ran every search"
    );
    let searches = room
        .searches
        .into_iter()
        .zip(reference.searches)
        .map(|(search, searched)| {
            assert_eq!(
                search.name, searched.name,
                "{name} {room_name}: the reference names the room's searches in order"
            );
            let estimates: Vec<Estimate> = searched
                .estimates
                .iter()
                .map(|estimate| Estimate {
                    scan: estimate.scan,
                    pose: PlanarPose {
                        x: estimate.pose[0],
                        y: estimate.pose[1],
                        yaw: estimate.pose[2],
                    },
                    deviations: estimate.covariance.deviations(),
                })
                .collect();
            assert!(
                estimates.windows(2).all(|pair| pair[0].scan < pair[1].scan)
                    && estimates
                        .last()
                        .is_none_or(|last| last.scan < room.scans.len()),
                "{name} {room_name} {}: each estimate names a later scan of the drive",
                search.name
            );
            (search, estimates)
        })
        .collect();
    Room {
        name: room.name,
        map_scans: room.map_scans,
        scans: room.scans,
        grid: reference.grid,
        searches,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "occupancy", "laser": {}, "grid": {}, "rooms": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    // Each search of the L-shaped room publishes an estimate after a move back: a motion of
    // at least AMCL's smallest move with a heading, 1 cm, since its estimate before, whose
    // bearing turns more than a quarter turn from the base's heading there, so its parity
    // covers the motion models' turns of a move back.
    #[test]
    fn each_search_of_the_l_shaped_room_publishes_after_a_move_back() {
        for cases in Cases::load_all() {
            let room = cases
                .rooms
                .iter()
                .find(|room| room.name == "l_room")
                .expect("the L-shaped room");
            for (search, estimates) in &room.searches {
                let moved_back = estimates.windows(2).any(|pair| {
                    let [x0, y0, yaw0] = room.scans[pair[0].scan].odometry;
                    let [x1, y1, _] = room.scans[pair[1].scan].odometry;
                    let bearing = (y1 - y0).atan2(x1 - x0) - yaw0;
                    (x1 - x0).hypot(y1 - y0) >= 0.01 && bearing.cos() < 0.0
                });
                assert!(
                    moved_back,
                    "{} {}: {} publishes no estimate after a move back",
                    cases.name, room.name, search.name
                );
            }
        }
    }

    #[test]
    fn every_amcl_global_localization_scenario_pairs_with_its_reference() {
        let all = Cases::load_all();
        assert!(
            !all.is_empty(),
            "scenarios/ holds an AMCL global localization scenario"
        );
        for cases in all {
            for room in &cases.rooms {
                assert_eq!(
                    room.grid.rows.len(),
                    room.grid.height as usize,
                    "{} {}: the grid holds a row of cells a row",
                    cases.name,
                    room.name
                );
                assert!(
                    room.grid
                        .rows
                        .iter()
                        .all(|row| row.len() == room.grid.width as usize),
                    "{} {}: each row holds the grid's width of cells",
                    cases.name,
                    room.name
                );
                for (search, estimates) in &room.searches {
                    assert!(
                        !estimates.is_empty(),
                        "{} {} {}: AMCL published a pose",
                        cases.name,
                        room.name,
                        search.name
                    );
                }
            }
        }
    }
}
