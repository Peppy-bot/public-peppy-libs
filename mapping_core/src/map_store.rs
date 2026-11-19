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

//! Maps saved by name in one directory, as room_map:v1 keeps them on the machine that runs
//! its implementer: each map is the directory of its [`MapName`], which [`RoomMap::save`]
//! writes.
//!
//! A save claims its name by creating the name's directory, which no other writer of the
//! directory can then create, writes the map into a temporary directory of its own, flushes
//! it to the disk and renames it over the claimed directory, so a reader finds no map or a
//! whole one. A removal renames the map's directory to a temporary directory before it
//! deletes it. A temporary directory's name starts with `.`, so it is no map's name.

use crate::saved_map::{LoadError, MapSummary, RoomMap, SaveError};
use std::fmt;
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use thiserror::Error;
use tracing::warn;

/// The longest map name, in characters.
const LONGEST_NAME: usize = 64;

/// What a map name is, as a refusal of one says it.
const NAME_RULE: &str = "name the map with a lowercase letter, then lowercase letters, \
     digits and `_`, at most 64 characters, such as `lab` or `second_floor`";

/// The name of a saved map, as room_map:v1 states it: a lowercase letter, then lowercase
/// letters, digits and `_`, at most [`LONGEST_NAME`] characters. Two names are the same name
/// when their characters are.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MapName(String);

/// Why a text is no map name.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MapNameError {
    #[error("the map name is empty: {NAME_RULE}")]
    Empty,
    #[error("the map name {name:?} starts with {first:?}: {NAME_RULE}")]
    First { name: String, first: char },
    #[error("the map name {name:?} holds {character:?}: {NAME_RULE}")]
    Character { name: String, character: char },
    #[error("the map name {name:?} holds {length} characters: {NAME_RULE}")]
    Length { name: String, length: usize },
}

impl MapName {
    pub fn parse(name: &str) -> Result<MapName, MapNameError> {
        let mut characters = name.chars();
        let first = characters.next().ok_or(MapNameError::Empty)?;
        if !first.is_ascii_lowercase() {
            return Err(MapNameError::First {
                name: name.to_owned(),
                first,
            });
        }
        if let Some(character) = characters.find(|&character| !is_name_character(character)) {
            return Err(MapNameError::Character {
                name: name.to_owned(),
                character,
            });
        }
        if name.len() > LONGEST_NAME {
            return Err(MapNameError::Length {
                name: name.to_owned(),
                length: name.len(),
            });
        }
        Ok(Self(name.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for MapName {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

fn is_name_character(character: char) -> bool {
    character.is_ascii_lowercase() || character.is_ascii_digit() || character == '_'
}

/// The maps saved in one directory.
#[derive(Debug)]
pub struct MapStore {
    directory: PathBuf,
}

/// A saved map as a listing gives it: its name and its summary.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedMap {
    pub name: MapName,
    pub summary: MapSummary,
}

/// Why the store did not do what it was asked.
#[derive(Debug, Error)]
pub enum StoreError {
    #[error("cannot read {}: {error}", path.display())]
    Read { path: PathBuf, error: io::Error },
    #[error("cannot write {}: {error}", path.display())]
    Write { path: PathBuf, error: io::Error },
    #[error("the saved map {name} does not load: {error}; remove it and map the room again")]
    Load { name: MapName, error: LoadError },
    #[error(transparent)]
    Save(#[from] SaveError),
    #[error("a map named {name} is saved: name the map otherwise, or remove {name} first")]
    Exists { name: MapName },
    #[error("no map named {name} is saved: list the saved maps for their names")]
    NotFound { name: MapName },
}

impl MapStore {
    /// The store of the maps in `directory`, which it creates when it is missing.
    pub fn open(directory: impl Into<PathBuf>) -> Result<MapStore, StoreError> {
        let directory = directory.into();
        fs::create_dir_all(&directory).map_err(|error| StoreError::Write {
            path: directory.clone(),
            error,
        })?;
        Ok(Self { directory })
    }

    /// The saved maps whose description reads, sorted by name, each summary read from the
    /// map's description alone. A map whose description does not read is logged by its
    /// name and left out.
    pub fn list(&self) -> Result<Vec<SavedMap>, StoreError> {
        Ok(self
            .saved_names()?
            .into_iter()
            .filter_map(|name| match RoomMap::load_summary(&self.path_of(&name)) {
                Ok(summary) => Some(SavedMap { name, summary }),
                Err(error) => {
                    warn!("the saved map {name} is left out of the list: {error}");
                    None
                }
            })
            .collect())
    }

    /// Whether a map named `name` is saved, whether or not it loads.
    pub fn contains(&self, name: &MapName) -> Result<bool, StoreError> {
        Ok(self.saved_names()?.contains(name))
    }

    /// Saves `map` as `name`; refused while a saved map, or another writer's save, has the
    /// name. Once the map's directory is renamed into place the map is saved: a failed
    /// flush of the store's directory after it is logged.
    pub fn save(&self, name: &MapName, map: &RoomMap) -> Result<(), StoreError> {
        let path = self.path_of(name);
        match fs::create_dir(&path) {
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
                return Err(StoreError::Exists { name: name.clone() });
            }
            Err(error) => return Err(StoreError::Write { path, error }),
            Ok(()) => {}
        }
        if let Err(error) = self.write_into(&path, map) {
            if let Err(left) = fs::remove_dir(&path) {
                warn!(
                    "the directory {} of a refused save stays: {left}",
                    path.display()
                );
            }
            return Err(error);
        }
        log_unflushed(name, sync(&self.directory));
        Ok(())
    }

    /// Writes `map` into a temporary directory of this writer, flushes it, and renames it
    /// over the empty directory `path`.
    fn write_into(&self, path: &Path, map: &RoomMap) -> Result<(), StoreError> {
        let temporary = self.temporary_path();
        let written = map
            .save(&temporary)
            .map_err(StoreError::from)
            .and_then(|()| sync_directory(&temporary))
            .and_then(|()| rename(&temporary, path));
        if written.is_err()
            && let Err(left) = fs::remove_dir_all(&temporary)
            && left.kind() != io::ErrorKind::NotFound
        {
            warn!(
                "the temporary directory {} of a refused save stays: {left}",
                temporary.display()
            );
        }
        written
    }

    /// The saved map named `name`.
    pub fn load(&self, name: &MapName) -> Result<RoomMap, StoreError> {
        self.found(name)?;
        RoomMap::load(&self.path_of(name)).map_err(|error| StoreError::Load {
            name: name.clone(),
            error,
        })
    }

    /// Removes the saved map named `name`, whether or not it loads.
    pub fn remove(&self, name: &MapName) -> Result<(), StoreError> {
        self.found(name)?;
        let temporary = self.temporary_path();
        rename(&self.path_of(name), &temporary)?;
        sync(&self.directory)?;
        fs::remove_dir_all(&temporary).map_err(|error| StoreError::Write {
            path: temporary,
            error,
        })
    }

    /// Refuses `name` unless a map of that name is saved.
    fn found(&self, name: &MapName) -> Result<(), StoreError> {
        if self.contains(name)? {
            Ok(())
        } else {
            Err(StoreError::NotFound { name: name.clone() })
        }
    }

    /// The names of the saved maps, sorted: each directory of the store whose name is a
    /// map's name.
    fn saved_names(&self) -> Result<Vec<MapName>, StoreError> {
        let unreadable = |error| StoreError::Read {
            path: self.directory.clone(),
            error,
        };
        let mut names = fs::read_dir(&self.directory)
            .map_err(unreadable)?
            .map(|entry| {
                let entry = entry.map_err(unreadable)?;
                let is_directory = entry.file_type().map_err(unreadable)?.is_dir();
                Ok(is_directory
                    .then(|| entry.file_name())
                    .and_then(|name| MapName::parse(name.to_str()?).ok()))
            })
            .filter_map(Result::transpose)
            .collect::<Result<Vec<_>, StoreError>>()?;
        names.sort();
        Ok(names)
    }

    fn path_of(&self, name: &MapName) -> PathBuf {
        self.directory.join(name.as_str())
    }

    /// A temporary directory that no other writer of the store writes: its name starts with
    /// `.` and holds the process's id and a count of the process's temporary directories.
    fn temporary_path(&self) -> PathBuf {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let index = NEXT.fetch_add(1, Ordering::Relaxed);
        self.directory
            .join(format!(".{}-{index}.tmp", std::process::id()))
    }
}

/// Logs a saved map whose store's directory `flushed` did not flush.
fn log_unflushed(name: &MapName, flushed: Result<(), StoreError>) {
    if let Err(error) = flushed {
        warn!("the map {name} is saved, and the store's directory is not flushed: {error}");
    }
}

fn rename(from: &Path, to: &Path) -> Result<(), StoreError> {
    fs::rename(from, to).map_err(|error| StoreError::Write {
        path: to.to_owned(),
        error,
    })
}

fn sync(path: &Path) -> Result<(), StoreError> {
    File::open(path)
        .and_then(|file| file.sync_all())
        .map_err(|error| StoreError::Write {
            path: path.to_owned(),
            error,
        })
}

/// Flushes each file of `directory`, then the directory.
fn sync_directory(directory: &Path) -> Result<(), StoreError> {
    let unreadable = |error| StoreError::Read {
        path: directory.to_owned(),
        error,
    };
    fs::read_dir(directory)
        .map_err(unreadable)?
        .try_for_each(|entry| sync(&entry.map_err(unreadable)?.path()))?;
    sync(directory)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// The text logged on this thread while the capture's guard lives. Each test that
    /// reaches a log site of the store runs under a capture: a site first reached on a
    /// thread with no subscriber can cache itself as never logged.
    #[derive(Clone, Default)]
    struct Logged(Arc<Mutex<Vec<u8>>>);

    impl io::Write for Logged {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.0.lock().expect("the log").extend_from_slice(bytes);
            Ok(bytes.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Logged {
        fn capture() -> (Self, tracing::subscriber::DefaultGuard) {
            let logged = Self::default();
            let writer = logged.clone();
            let guard = tracing::subscriber::set_default(
                tracing_subscriber::fmt()
                    .with_writer(move || writer.clone())
                    .with_ansi(false)
                    .finish(),
            );
            (logged, guard)
        }

        fn text(&self) -> String {
            String::from_utf8_lossy(&self.0.lock().expect("the log")).into_owned()
        }
    }

    fn scratch(name: &str) -> PathBuf {
        let directory =
            std::env::temp_dir().join(format!("mapping_core_store_{name}_{}", std::process::id()));
        let _ = fs::remove_dir_all(&directory);
        directory
    }

    #[test]
    fn a_map_left_out_of_the_list_is_logged_by_its_name() {
        let (logged, _guard) = Logged::capture();
        let directory = scratch("unlisted");
        let store = MapStore::open(&directory).expect("the store opens");
        fs::create_dir(directory.join("garage")).expect("a map's directory");
        assert_eq!(store.list().expect("the maps list"), []);
        assert!(
            logged
                .text()
                .contains("the saved map garage is left out of the list"),
            "{}",
            logged.text()
        );
    }

    #[test]
    fn a_save_whose_store_directory_is_not_flushed_is_logged_as_saved() {
        let (logged, _guard) = Logged::capture();
        let name = MapName::parse("lab").expect("a name");
        log_unflushed(&name, Ok(()));
        assert_eq!(logged.text(), "");
        let unflushed = StoreError::Write {
            path: PathBuf::from("/maps"),
            error: io::Error::other("no flush"),
        };
        log_unflushed(&name, Err(unflushed));
        assert!(
            logged
                .text()
                .contains("the map lab is saved, and the store's directory is not flushed"),
            "{}",
            logged.text()
        );
    }

    #[test]
    fn each_temporary_directory_of_a_store_is_its_own() {
        let store = MapStore {
            directory: PathBuf::from("/maps"),
        };
        let (first, second) = (store.temporary_path(), store.temporary_path());
        assert_ne!(first, second);
        for path in [first, second] {
            let file = path
                .file_name()
                .expect("a name")
                .to_string_lossy()
                .into_owned();
            assert!(MapName::parse(&file).is_err(), "{file} is no map's name");
            assert!(file.contains(&std::process::id().to_string()), "{file}");
        }
    }
}
