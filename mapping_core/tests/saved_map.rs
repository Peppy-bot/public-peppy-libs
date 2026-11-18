//! A saved map loads as the pose graph, the grid and the summary that it saved; a
//! directory whose files disagree with its pose graph is refused, by the file and the
//! field; and a resolution that makes no grid is refused, by the file and the field.

use hegel::TestCase;
use mapping_core::{LoadError, RoomMap};
use serde_json::Value;
use std::path::Path;

mod common;
use common::properties::{Room, circle_drive, drifting_odometry, loop_drive, room};
use common::{CREATED_AT_UNIX_NS, room_map_of, scratch_directory, sync_parameters};

#[hegel::test(test_cases = 5)]
fn a_saved_map_loads_as_the_same_pose_graph_and_grid(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let odometry = drifting_odometry(&tc, &drive);
    let map = room_map_of(&room, &drive, &odometry, sync_parameters());
    let directory = scratch_directory("saved_map");
    map.save(&directory).expect("the map saves");
    let loaded = RoomMap::load(&directory).expect("the map loads");
    assert_eq!(loaded, map);
    assert_eq!(loaded.summary(), map.summary());
    assert_eq!(loaded.summary().created_at_unix_ns, CREATED_AT_UNIX_NS);
}

/// A map of a drive around a room 6 by 4 m, saved in a directory of its own.
fn saved_room_map() -> std::path::PathBuf {
    let room = Room::new(3.0, 2.0, 0.25, 1);
    let drive = circle_drive(0.6);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let directory = scratch_directory("disagreeing");
    map.save(&directory).expect("the map saves");
    directory
}

fn edit_json(path: &Path, edit: impl FnOnce(&mut Value)) {
    let mut value: Value =
        serde_json::from_slice(&std::fs::read(path).expect("the file reads")).expect("JSON");
    edit(&mut value);
    std::fs::write(path, serde_json::to_vec_pretty(&value).expect("JSON")).expect("writes");
}

fn disagreement(directory: &Path) -> Option<(&'static str, String)> {
    match RoomMap::load(directory) {
        Err(LoadError::Disagree { file, field }) => Some((file, field)),
        _ => None,
    }
}

#[test]
fn a_directory_whose_files_disagree_is_refused_by_the_file_and_the_field() {
    let directory = saved_room_map();
    edit_json(&directory.join("meta.json"), |meta| {
        meta["width"] = Value::from(meta["width"].as_u64().expect("a width") + 1);
    });
    assert_eq!(
        disagreement(&directory),
        Some(("meta.json", "width".into()))
    );

    let directory = saved_room_map();
    edit_json(&directory.join("meta.json"), |meta| {
        meta["scans"] = Value::from(0);
    });
    assert_eq!(
        disagreement(&directory),
        Some(("meta.json", "scans".into()))
    );

    let directory = saved_room_map();
    let yaml = std::fs::read_to_string(directory.join("map.yaml")).expect("the description");
    std::fs::write(
        directory.join("map.yaml"),
        yaml.replace("resolution: 0.050", "resolution: 0.100"),
    )
    .expect("writes");
    assert_eq!(
        disagreement(&directory),
        Some(("map.yaml", "resolution".into()))
    );

    let directory = saved_room_map();
    let mut image = std::fs::read(directory.join("map.pgm")).expect("the image");
    let last = image.len() - 1;
    image[last] = if image[last] == 0 { 254 } else { 0 };
    std::fs::write(directory.join("map.pgm"), image).expect("writes");
    assert_eq!(disagreement(&directory), Some(("map.pgm", "pixels".into())));

    let directory = saved_room_map();
    edit_json(&directory.join("pose_graph.json"), |graph| {
        graph["nodes"][0]["beams"]
            .as_array_mut()
            .expect("beams")
            .pop();
    });
    assert!(matches!(
        RoomMap::load(&directory),
        Err(LoadError::Invalid {
            file: "pose_graph.json",
            ..
        })
    ));
}

#[test]
fn a_resolution_that_makes_no_grid_is_refused_by_the_file_and_the_field() {
    let invalid = |resolution: f64| {
        let directory = saved_room_map();
        edit_json(&directory.join("meta.json"), |meta| {
            meta["resolution"] = Value::from(resolution);
        });
        match RoomMap::load(&directory) {
            Err(LoadError::Invalid { file, field, .. }) => Some((file, field)),
            _ => None,
        }
    };
    let refused = Some(("meta.json", "resolution".to_string()));
    // Not above 0, and so small that the scans' box holds more cells than karto counts.
    assert_eq!(invalid(0.0), refused);
    assert_eq!(invalid(-0.05), refused);
    assert_eq!(invalid(1e-9), refused);
}
