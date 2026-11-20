//! Maps saved by name: a map name is a lowercase letter, then lowercase letters, digits and
//! `_`, at most 64 characters, as room_map:v1 states it; a store lists the maps whose
//! description reads, sorted by name, each with the summary of the map it loads as; a save
//! of a name a saved map has is refused, and of racing saves of one name one saves; a
//! removed map lists and loads no more, and its name is free again, a delete that fails
//! after the rename included; and what a crashed save left is no saved map.

use hegel::TestCase;
use hegel::generators::{self as gs, Generator};
use mapping_core::{MapName, MapNameError, MapStore, RoomMap, SavedMap, StoreError};
use std::sync::LazyLock;

mod common;
use common::properties::{Room, circle_drive};
use common::{room_map_of, scratch_directory, sync_parameters};

/// The characters of a map name after its first.
const NAME_CHARACTERS: &str = "abcdefghijklmnopqrstuvwxyz0123456789_";
const LOWERCASE: &str = "abcdefghijklmnopqrstuvwxyz";
const LONGEST_NAME: usize = 64;

/// A map of a drawn room, saved by every test.
static MAP: LazyLock<RoomMap> = LazyLock::new(|| {
    let room = Room::new(3.0, 2.0, 0.25, 1);
    let drive = circle_drive(0.6);
    room_map_of(&room, &drive, &drive, sync_parameters())
});

fn name(text: &str) -> MapName {
    MapName::parse(text).expect("a map name")
}

fn names(maps: &[SavedMap]) -> Vec<&str> {
    maps.iter().map(|saved| saved.name.as_str()).collect()
}

/// An empty store in a directory of its own.
fn empty_store(label: &str) -> MapStore {
    MapStore::open(scratch_directory(label)).expect("the store opens")
}

#[hegel::test(test_cases = 100)]
fn a_lowercase_letter_then_up_to_63_name_characters_is_a_name(tc: TestCase) {
    let first = tc.draw(gs::text().alphabet(LOWERCASE).min_size(1).max_size(1));
    let rest = tc.draw(
        gs::text()
            .alphabet(NAME_CHARACTERS)
            .max_size(LONGEST_NAME - 1),
    );
    let text = format!("{first}{rest}");
    assert_eq!(name(&text).as_str(), text);
}

#[hegel::test(test_cases = 100)]
fn a_name_that_holds_any_other_character_is_refused_by_the_character(tc: TestCase) {
    let before = tc.draw(gs::text().alphabet(NAME_CHARACTERS).max_size(10));
    let after = tc.draw(gs::text().alphabet(NAME_CHARACTERS).max_size(10));
    let other: char = tc.draw(
        gs::characters()
            .filter(|c: &char| !(c.is_ascii_lowercase() || c.is_ascii_digit() || *c == '_')),
    );
    let text = format!("a{before}{other}{after}");
    assert_eq!(
        MapName::parse(&text),
        Err(MapNameError::Character {
            name: text.clone(),
            character: other
        })
    );
}

#[test]
fn a_name_starts_with_a_lowercase_letter_and_holds_at_most_64_characters() {
    assert_eq!(MapName::parse(""), Err(MapNameError::Empty));
    for refused in ["Lab", "1lab", "_lab"] {
        let first = refused.chars().next().expect("a first character");
        assert_eq!(
            MapName::parse(refused),
            Err(MapNameError::First {
                name: refused.into(),
                first
            })
        );
    }
    let longest = "l".repeat(LONGEST_NAME);
    assert_eq!(name(&longest).as_str(), longest);
    let too_long = "l".repeat(LONGEST_NAME + 1);
    assert_eq!(
        MapName::parse(&too_long),
        Err(MapNameError::Length {
            name: too_long.clone(),
            length: LONGEST_NAME + 1
        })
    );
    let refusal = MapName::parse("Lab").expect_err("refused").to_string();
    assert!(
        refusal.contains("name the map with a lowercase letter"),
        "{refusal}"
    );
}

#[test]
fn a_saved_map_lists_with_the_summary_of_the_map_it_loads_as() {
    let directory = scratch_directory("store_round_trip").join("maps");
    let store = MapStore::open(&directory).expect("the store opens");
    assert!(directory.is_dir(), "the store makes its directory");
    store.save(&name("kitchen"), &MAP).expect("the map saves");
    let listed = store.list().expect("the maps list");
    assert_eq!(
        listed,
        [SavedMap {
            name: name("kitchen"),
            summary: MAP.summary()
        }]
    );
    let loaded = store.load(&name("kitchen")).expect("the map loads");
    assert_eq!(loaded, *MAP);
    assert!(store.contains(&name("kitchen")).expect("the names read"));
    assert!(!store.contains(&name("garage")).expect("the names read"));
}

#[test]
fn the_saved_maps_list_sorted_by_name() {
    let store = empty_store("store_order");
    for saved in ["lab_2", "kitchen", "lab", "a1"] {
        store.save(&name(saved), &MAP).expect("the map saves");
    }
    assert_eq!(
        names(&store.list().expect("the maps list")),
        ["a1", "kitchen", "lab", "lab_2"]
    );
}

#[test]
fn a_save_of_a_name_that_a_saved_map_has_is_refused() {
    let store = empty_store("store_same_name");
    store.save(&name("lab"), &MAP).expect("the map saves");
    let error = store.save(&name("lab"), &MAP).expect_err("refused");
    assert!(
        matches!(&error, StoreError::Exists { name } if name.as_str() == "lab"),
        "{error}"
    );
    assert_eq!(names(&store.list().expect("the maps list")), ["lab"]);
}

#[test]
fn a_removed_map_lists_and_loads_no_more_and_its_name_is_free_again() {
    let store = empty_store("store_remove");
    store.save(&name("lab"), &MAP).expect("the map saves");
    store.save(&name("kitchen"), &MAP).expect("the map saves");
    store.remove(&name("lab")).expect("the map is removed");
    assert_eq!(names(&store.list().expect("the maps list")), ["kitchen"]);
    assert!(matches!(
        store.load(&name("lab")),
        Err(StoreError::NotFound { .. })
    ));
    store
        .save(&name("lab"), &MAP)
        .expect("the name is free again");
    assert_eq!(
        names(&store.list().expect("the maps list")),
        ["kitchen", "lab"]
    );
}

// A removal whose delete fails after its rename has removed the map: the map lists and
// loads no more and its name is free; the leftover directory is the temporary one.
#[cfg(unix)]
#[test]
fn a_removal_whose_delete_fails_has_still_removed_the_map() {
    use std::os::unix::fs::PermissionsExt;
    let directory = scratch_directory("store_remove_undeleted");
    let store = MapStore::open(&directory).expect("the store opens");
    store.save(&name("lab"), &MAP).expect("the map saves");
    // A directory inside the map that the delete cannot empty.
    let locked = directory.join("lab").join("locked");
    std::fs::create_dir(&locked).expect("a directory in the map");
    std::fs::write(locked.join("file"), "").expect("a file in it");
    std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o555))
        .expect("the directory is read-only");
    let removed = store.remove(&name("lab"));
    let leftovers: Vec<_> = std::fs::read_dir(&directory)
        .expect("the store's directory")
        .map(|entry| entry.expect("an entry").path())
        .collect();
    for leftover in &leftovers {
        let locked = leftover.join("locked");
        if locked.exists() {
            std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o755))
                .expect("the directory is writable again");
        }
    }
    removed.expect("the map is removed");
    assert!(!leftovers.is_empty(), "the delete failed and left the map's directory");
    assert_eq!(store.list().expect("the maps list"), []);
    assert!(matches!(
        store.load(&name("lab")),
        Err(StoreError::NotFound { .. })
    ));
    assert!(
        leftovers.iter().all(|path| path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.starts_with('.'))),
        "{leftovers:?}"
    );
    store
        .save(&name("lab"), &MAP)
        .expect("the name is free again");
}

#[test]
fn a_missing_map_is_not_found_by_load_or_remove() {
    let store = empty_store("store_missing");
    let missing = name("garage");
    let not_found =
        |error: StoreError| matches!(error, StoreError::NotFound { name } if name == missing);
    assert!(not_found(store.load(&missing).expect_err("no map")));
    assert!(not_found(store.remove(&missing).expect_err("no map")));
}

#[test]
fn what_a_crashed_save_left_is_no_saved_map_and_a_removal_frees_its_name() {
    let directory = scratch_directory("store_leftover");
    let store = MapStore::open(&directory).expect("the store opens");
    // A crashed writer's temporary directory, with a torn file.
    let leftover = directory.join(".4242-0.tmp");
    std::fs::create_dir_all(&leftover).expect("a leftover");
    std::fs::write(leftover.join("meta.json"), "{").expect("a torn file");
    // Neither a directory whose name is no map's name, nor a file, is a saved map.
    std::fs::create_dir_all(directory.join("Living room")).expect("a directory");
    std::fs::write(directory.join("notes"), "").expect("a file");
    assert_eq!(store.list().expect("the maps list"), []);
    store.save(&name("lab"), &MAP).expect("the map saves");
    assert_eq!(names(&store.list().expect("the maps list")), ["lab"]);
    assert_eq!(store.load(&name("lab")).expect("the map loads"), *MAP);
    // The empty directory of a name that a crashed save claimed lists no map and keeps the
    // name, does not load, and a removal frees the name.
    std::fs::create_dir(directory.join("kitchen")).expect("a claimed name");
    assert_eq!(names(&store.list().expect("the maps list")), ["lab"]);
    assert!(matches!(
        store.save(&name("kitchen"), &MAP),
        Err(StoreError::Exists { .. })
    ));
    assert!(matches!(
        store.load(&name("kitchen")),
        Err(StoreError::Load { .. })
    ));
    store.remove(&name("kitchen")).expect("the name is freed");
    store.save(&name("kitchen"), &MAP).expect("the map saves");
    assert_eq!(
        names(&store.list().expect("the maps list")),
        ["kitchen", "lab"]
    );
}

#[test]
fn a_map_whose_description_does_not_read_is_left_out_of_the_list() {
    let directory = scratch_directory("store_unreadable");
    let store = MapStore::open(&directory).expect("the store opens");
    store.save(&name("kitchen"), &MAP).expect("the map saves");
    store.save(&name("lab"), &MAP).expect("the map saves");
    std::fs::write(directory.join("kitchen").join("meta.json"), "{").expect("a torn file");
    assert_eq!(
        store.list().expect("the maps list"),
        [SavedMap {
            name: name("lab"),
            summary: MAP.summary()
        }]
    );
}

/// The rounds of the racing saves, and the writers of each.
const RACE_ROUNDS: usize = 20;
const RACE_WRITERS: usize = 4;

// Writers saving the same name at once: one saves, the others are refused as the name
// exists, and the store holds one map of the name and no temporary directory.
#[test]
fn of_racing_saves_of_one_name_one_saves() {
    for round in 0..RACE_ROUNDS {
        let directory = scratch_directory(&format!("store_race_{round}"));
        let store = MapStore::open(&directory).expect("the store opens");
        let outcomes: Vec<Result<(), StoreError>> = std::thread::scope(|scope| {
            let writers: Vec<_> = (0..RACE_WRITERS)
                .map(|_| {
                    let store = &store;
                    scope.spawn(move || store.save(&name("lab"), &MAP))
                })
                .collect();
            writers
                .into_iter()
                .map(|writer| writer.join().expect("a writer"))
                .collect()
        });
        let saved = outcomes.iter().filter(|outcome| outcome.is_ok()).count();
        assert_eq!(saved, 1, "round {round}: {outcomes:?}");
        assert!(
            outcomes
                .iter()
                .all(|outcome| matches!(outcome, Ok(()) | Err(StoreError::Exists { .. }))),
            "round {round}: a writer failed otherwise: {outcomes:?}"
        );
        assert_eq!(names(&store.list().expect("the maps list")), ["lab"]);
        let leftovers: Vec<_> = std::fs::read_dir(&directory)
            .expect("the store reads")
            .map(|entry| entry.expect("an entry").file_name())
            .filter(|file| file.to_string_lossy().starts_with('.'))
            .collect();
        assert_eq!(leftovers, Vec::<std::ffi::OsString>::new(), "round {round}");
    }
}

#[test]
fn a_store_opened_again_on_its_directory_lists_the_same_maps() {
    let directory = scratch_directory("store_reopened");
    let store = MapStore::open(&directory).expect("the store opens");
    store.save(&name("kitchen"), &MAP).expect("the map saves");
    store.save(&name("lab"), &MAP).expect("the map saves");
    let listed = store.list().expect("the maps list");
    drop(store);
    let reopened = MapStore::open(&directory).expect("the store opens again");
    assert_eq!(reopened.list().expect("the maps list"), listed);
    assert_eq!(reopened.load(&name("lab")).expect("the map loads"), *MAP);
}
