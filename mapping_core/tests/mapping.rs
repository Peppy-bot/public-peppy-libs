//! Properties of rooms mapped from drawn scans: the matcher recovers a scan's pose from
//! odometry off by a drawn offset; a drive around a drawn room closes its loop, so the
//! walls of its last scan lie where its first scan put them, within the map's resolution;
//! each scan added to the graph answers once and counts once; the map frame is the odometry
//! frame where mapping started; and parameters that keep no localization leave the map in the
//! error.

use control_core::planar_pose::PlanarPose;
use hegel::TestCase;
use hegel::generators as gs;
use mapping_core::{CellState, FinishError, GridError, LocalizationError, Mapping, ParameterError};

mod common;
use common::properties::{
    Room, circle_drive, compose, drifting_odometry, heading_error, largest_point_displacement,
    loop_drive, planar, position_error, room,
};
use common::{
    CREATED_AT_UNIX_NS, SCAN_PERIOD_NS, localization_parameters, map_of, placed, room_map_of,
    sync_parameters,
};

/// How far the matcher may leave a scan from its true pose, in position (m) and heading
/// (rad): two cells of the fine search, and three of its angle steps.
const MATCHED_POSITION: f64 = 0.02;
const MATCHED_HEADING: f64 = 0.0105;

#[hegel::test(test_cases = 25)]
fn the_matcher_recovers_the_pose_of_a_scan_from_odometry_off_by_a_drawn_offset(tc: TestCase) {
    let room = room(&tc);
    let x: f64 = tc.draw(gs::floats().min_value(-0.5).max_value(0.5));
    let yaw: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
    let start = planar(x, 0.0, yaw);
    // The second pose lies 0.8 m from the first, toward the room's center line.
    let toward = if x > 0.0 { std::f64::consts::PI } else { 0.0 };
    let truth = planar(x + 0.8 * toward.cos(), 0.0, yaw + 0.3);
    let offset = planar(
        tc.draw(gs::floats().min_value(-0.15).max_value(0.15)),
        tc.draw(gs::floats().min_value(-0.15).max_value(0.15)),
        tc.draw(gs::floats().min_value(-0.2).max_value(0.2)),
    );
    let odometry = compose(truth, offset);
    let mut mapping = Mapping::start(sync_parameters()).expect("valid parameters");
    // slam_toolbox's gate passes the first scan and holds the second to the fourth.
    for index in 0..4 {
        mapping
            .add(&placed(index * SCAN_PERIOD_NS, start, &room.scan(start)))
            .expect("a scan");
    }
    let matched = mapping
        .add(&placed(4 * SCAN_PERIOD_NS, odometry, &room.scan(truth)))
        .expect("a scan")
        .expect("the fifth scan passes the gate")
        .pose;
    assert!(
        position_error(matched, truth) <= MATCHED_POSITION
            && heading_error(matched, truth) <= MATCHED_HEADING,
        "the scan at {truth:?}, by odometry at {odometry:?}, matched at {matched:?}"
    );
}

#[hegel::test(test_cases = 10)]
fn a_drive_around_a_drawn_room_closes_its_loop(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let odometry = drifting_odometry(&tc, &drive);
    let parameters = sync_parameters();
    let mut mapping = Mapping::start(parameters).expect("valid parameters");
    let added: Vec<usize> = drive
        .iter()
        .zip(&odometry)
        .enumerate()
        .filter_map(|(index, (truth, odometry))| {
            mapping
                .add(&placed(
                    index as i64 * SCAN_PERIOD_NS,
                    *odometry,
                    &room.scan(*truth),
                ))
                .expect("a scan")
                .map(|_| index)
        })
        .collect();
    let &last_index = added.last().expect("the drive adds scans");
    let poses = mapping.poses();
    let (_, last_pose) = poses.last().expect("the last scan's pose");
    let displacement =
        largest_point_displacement(&room.scan(drive[last_index]), *last_pose, drive[last_index]);
    assert!(
        displacement <= parameters.resolution,
        "the walls of the last scan lie {displacement} m from where the first scan's frame \
         puts them"
    );
}

#[hegel::test(test_cases = 5)]
fn the_grid_of_a_drawn_room_holds_the_walls_along_its_origin_occupied_and_the_drive_free(
    tc: TestCase,
) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let grid = map.grid();
    let cell_of = |x: f64, y: f64| {
        let [origin_x, origin_y] = grid.origin();
        let column = ((x - origin_x) / grid.resolution()).round();
        let row = ((y - origin_y) / grid.resolution()).round();
        (column >= 0.0 && row >= 0.0)
            .then(|| grid.cell(column as u32, row as u32))
            .flatten()
    };
    let near = |x: f64, y: f64, state: CellState| {
        let step = grid.resolution();
        [-step, 0.0, step].iter().any(|dx| {
            [-step, 0.0, step]
                .iter()
                .any(|dy| cell_of(x + dx, y + dy) == Some(state))
        })
    };
    // The middle of each wall along the grid's origin: karto's grid holds Round(extent /
    // resolution) cells from the lowest corner of the scans' box, so the hits on the far
    // walls fall in the cells past its last column and row.
    let walls = [(-room.half_width, 0.0), (0.0, -room.half_height)];
    for (x, y) in walls {
        assert!(
            near(x, y, CellState::Occupied),
            "the wall at {x}, {y} is occupied"
        );
    }
    for pose in &drive {
        assert_eq!(
            cell_of(pose.x(), pose.y()),
            Some(CellState::Free),
            "the floor at {pose:?} is free"
        );
    }
}

#[hegel::test(test_cases = 5)]
fn each_scan_added_to_the_graph_answers_once_and_counts_once(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let odometry = drifting_odometry(&tc, &drive);
    let mut mapping = Mapping::start(sync_parameters()).expect("valid parameters");
    let mut answered = 0;
    for (index, (truth, odometry)) in drive.iter().zip(&odometry).enumerate() {
        let before = mapping.scans();
        let added = mapping
            .add(&placed(
                index as i64 * SCAN_PERIOD_NS,
                *odometry,
                &room.scan(*truth),
            ))
            .expect("a scan");
        if let Some(pose) = added {
            assert!(pose.localized, "the base is localized while mapping runs");
            answered += 1;
        }
        assert_eq!(
            mapping.scans(),
            before + u32::from(added.is_some()),
            "scan {index}"
        );
    }
    assert_eq!(mapping.scans(), answered);
    assert_eq!(mapping.poses().len(), answered as usize);
    let map = room_map_of(&room, &drive, &odometry, sync_parameters());
    assert_eq!(map.summary().scans, answered);
}

#[hegel::test(test_cases = 5)]
fn the_map_frame_is_the_odometry_frame_where_mapping_started(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    // The odometry frame lies anywhere in the room, so the first scan's odometry is any
    // pose.
    let frame = planar(
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
    );
    let odometry: Vec<PlanarPose> = drive.iter().map(|pose| compose(frame, *pose)).collect();
    let mut mapping = Mapping::start(sync_parameters()).expect("valid parameters");
    for (index, (truth, odometry)) in drive.iter().zip(&odometry).enumerate() {
        let Some(added) = mapping
            .add(&placed(
                index as i64 * SCAN_PERIOD_NS,
                *odometry,
                &room.scan(*truth),
            ))
            .expect("a scan")
        else {
            continue;
        };
        let expected = compose(frame, *truth);
        assert!(
            position_error(added.pose, expected) <= MATCHED_POSITION
                && heading_error(added.pose, expected) <= MATCHED_HEADING,
            "scan {index} at {:?} in the map, where the odometry frame puts it at {expected:?}",
            added.pose
        );
    }
}

#[test]
fn localization_parameters_that_keep_no_pose_leave_the_map_in_the_error() {
    let room = Room::new(3.0, 2.0, 0.25, 1);
    let drive = circle_drive(0.6);
    let finished = map_of(&room, &drive, &drive, sync_parameters())
        .finish(CREATED_AT_UNIX_NS, localization_parameters())
        .expect("valid parameters")
        .expect("the drive maps the room")
        .0;
    let mut no_buffer = localization_parameters();
    no_buffer.slam_toolbox.scan_buffer_size = 0;
    let Err(FinishError::Localization { map, error }) =
        map_of(&room, &drive, &drive, sync_parameters()).finish(CREATED_AT_UNIX_NS, no_buffer)
    else {
        panic!("a scan buffer of 0 keeps no pose");
    };
    assert_eq!(*map, finished);
    assert_eq!(
        error,
        LocalizationError::Parameters(ParameterError::ZeroCount {
            name: "scan_buffer_size"
        })
    );
}

#[test]
fn a_resolution_so_small_that_karto_cannot_count_the_cells_is_refused_at_the_end() {
    let room = Room::new(3.0, 2.0, 0.25, 1);
    let drive = circle_drive(0.6);
    let mut parameters = sync_parameters();
    parameters.resolution = 1e-9;
    let finished = map_of(&room, &drive, &drive, parameters)
        .finish(CREATED_AT_UNIX_NS, localization_parameters());
    assert!(
        matches!(
            finished,
            Err(FinishError::Grid(GridError::TooManyCells { .. }))
        ),
        "{finished:?}"
    );
}
