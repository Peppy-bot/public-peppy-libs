//! One running instance per device.
//!
//! A node claims its device with [`claim`] and holds the returned
//! [`InstanceLock`] for its whole run. The claim is an exclusive advisory lock
//! on a file under [`LOCK_DIR`], taken in one syscall, so of two starts that
//! race for one device exactly one wins. The kernel releases the lock when the
//! holding process exits, by any route including SIGKILL and the OOM killer, so
//! the next start of that device finds the lock free.
//!
//! The lock file stays on disk once a lock is released, and every claim of a
//! device locks the same inode behind the same path. Its content is the
//! `instance_id` of the holder, which [`Error::Held`] names so an operator can
//! stop the holder.
//!
//! The directory is shared ground: a node that runs in a container reaches the
//! one the other instances use by binding [`LOCK_DIR`] into the container, with
//! a `mount_paths` entry in the node manifest. The directory must already
//! exist, and a claim naming one that does not is refused.

use std::fs::{File, TryLockError};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

/// The directory holding every device's lock file. One directory for the whole
/// machine: the devices a lock stands for (a CAN bus, a USB port) are reached
/// by every process on the host, whichever daemon started it.
pub const LOCK_DIR: &str = "/tmp/peppy_instance_locks";

/// The most of a lock file read back to name its holder. An `instance_id` is far
/// shorter; the cap bounds the read of a file any process on the host can write.
const MAX_HOLDER_BYTES: u64 = 256;

/// Who holds a device, read from its lock file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Holder {
    /// The `instance_id` the holder wrote when it took the lock.
    Instance(String),
    /// The holder took the lock and wrote no name, which a start does between
    /// its own lock and its write.
    Unnamed,
}

/// Why a device could not be claimed.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{}", held_message(path, holder))]
    Held { path: PathBuf, holder: Holder },
    #[error(
        "instance lock name `{name}` is not a device name of ASCII letters, digits, `_` and `-`"
    )]
    Name { name: String },
    #[error(
        "instance lock directory {dir} does not exist; add `{dir}` to the node manifest's container `mount_paths`",
        dir = dir.display()
    )]
    MissingDir { dir: PathBuf },
    #[error("instance lock {path} could not be taken: {source}", path = path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
}

fn held_message(path: &Path, holder: &Holder) -> String {
    let path = path.display();
    match holder {
        Holder::Instance(id) => format!(
            "instance lock {path} is held by instance `{id}`; stop it with `peppy node stop {id}`"
        ),
        Holder::Unnamed => format!(
            "instance lock {path} is held by a process that wrote no instance id; find it with `peppy stack list`"
        ),
    }
}

/// An exclusive claim on one device, held until it is dropped.
#[derive(Debug)]
pub struct InstanceLock {
    path: PathBuf,
    file: File,
}

impl InstanceLock {
    /// The lock file this claim holds.
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for InstanceLock {
    fn drop(&mut self) {
        let _ = self.file.unlock();
    }
}

/// Claims the device `name` for `holder` under [`LOCK_DIR`]. `name` is the
/// device, such as `openarm_arm_0`; `holder` is the claiming node's
/// `instance_id`.
pub fn claim(name: &str, holder: &str) -> Result<InstanceLock, Error> {
    claim_in(Path::new(LOCK_DIR), name, holder)
}

/// Claims `name` under `dir`, which [`claim`] fills with [`LOCK_DIR`].
pub fn claim_in(dir: &Path, name: &str, holder: &str) -> Result<InstanceLock, Error> {
    let file_name = lock_file_name(name)?;
    if !dir.is_dir() {
        return Err(Error::MissingDir {
            dir: dir.to_path_buf(),
        });
    }
    let path = dir.join(file_name);
    let file = File::options()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&path)
        .map_err(|source| Error::Io {
            path: path.clone(),
            source,
        })?;

    match file.try_lock() {
        Ok(()) => {}
        Err(TryLockError::WouldBlock) => {
            let holder = read_holder(&path);
            return Err(Error::Held { path, holder });
        }
        Err(TryLockError::Error(source)) => return Err(Error::Io { path, source }),
    }

    write_holder(&file, holder).map_err(|source| Error::Io {
        path: path.clone(),
        source,
    })?;
    Ok(InstanceLock { path, file })
}

/// Parses `name` as the lock file of a device: the name of the file under the
/// lock directory, carried as proof it names a file in that directory and
/// nothing else.
fn lock_file_name(name: &str) -> Result<String, Error> {
    let is_device_name = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
    if !is_device_name {
        return Err(Error::Name {
            name: name.to_owned(),
        });
    }
    Ok(format!("{name}.lock"))
}

/// Replaces the lock file's content with `holder`, so a later claim that loses
/// the race names it.
fn write_holder(file: &File, holder: &str) -> std::io::Result<()> {
    file.set_len(0)?;
    let mut file = file;
    file.write_all(holder.as_bytes())?;
    file.flush()
}

/// Reads the holder a locked file names. Every unreadable shape of file, from an
/// empty one to bytes that are not text, is a holder that did not name itself.
fn read_holder(path: &Path) -> Holder {
    let Ok(file) = File::open(path) else {
        return Holder::Unnamed;
    };
    let mut text = String::new();
    if file
        .take(MAX_HOLDER_BYTES)
        .read_to_string(&mut text)
        .is_err()
    {
        return Holder::Unnamed;
    }
    match text.trim() {
        "" => Holder::Unnamed,
        id => Holder::Instance(id.to_owned()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOLDER: &str = "left_arm_inst";

    #[test]
    fn a_claim_names_its_holder_in_the_lock_file() {
        let dir = tempfile::tempdir().unwrap();
        let lock = claim_in(dir.path(), "openarm_arm_0", HOLDER).unwrap();

        assert_eq!(lock.path(), dir.path().join("openarm_arm_0.lock"));
        assert_eq!(std::fs::read_to_string(lock.path()).unwrap(), HOLDER);
    }

    #[test]
    fn a_second_claim_of_one_device_is_refused_naming_the_holder() {
        let dir = tempfile::tempdir().unwrap();
        let _held = claim_in(dir.path(), "openarm_arm_0", HOLDER).unwrap();

        let refused = claim_in(dir.path(), "openarm_arm_0", "right_arm_inst").unwrap_err();

        let Error::Held { holder, .. } = &refused else {
            panic!("expected Held, got {refused:?}");
        };
        assert_eq!(holder, &Holder::Instance(HOLDER.to_owned()));
        assert!(
            refused
                .to_string()
                .contains("peppy node stop left_arm_inst"),
            "the refusal says how to stop the holder: {refused}"
        );
    }

    #[test]
    fn a_dropped_claim_frees_the_device() {
        let dir = tempfile::tempdir().unwrap();
        let held = claim_in(dir.path(), "openarm_ker", HOLDER).unwrap();
        drop(held);

        let next = claim_in(dir.path(), "openarm_ker", "ker_inst").unwrap();
        assert_eq!(std::fs::read_to_string(next.path()).unwrap(), "ker_inst");
    }

    #[test]
    fn two_devices_do_not_contend() {
        let dir = tempfile::tempdir().unwrap();
        let _left = claim_in(dir.path(), "openarm_arm_0", "left_arm_inst").unwrap();
        claim_in(dir.path(), "openarm_arm_1", "right_arm_inst").unwrap();
    }

    #[test]
    fn a_holder_that_wrote_no_name_is_reported_as_unnamed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("openarm_arm_0.lock");
        let held = File::options()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)
            .unwrap();
        held.try_lock().unwrap();

        let refused = claim_in(dir.path(), "openarm_arm_0", HOLDER).unwrap_err();

        let Error::Held { holder, .. } = &refused else {
            panic!("expected Held, got {refused:?}");
        };
        assert_eq!(holder, &Holder::Unnamed);
        assert!(
            refused.to_string().contains("peppy stack list"),
            "the refusal says how to find the holder: {refused}"
        );
    }

    #[test]
    fn a_missing_directory_is_refused_naming_the_mount() {
        let dir = tempfile::tempdir().unwrap();
        let absent = dir.path().join("never_bound");

        let refused = claim_in(&absent, "openarm_arm_0", HOLDER).unwrap_err();

        assert!(matches!(refused, Error::MissingDir { .. }), "{refused:?}");
        assert!(
            refused.to_string().contains("mount_paths"),
            "the refusal says what the manifest needs: {refused}"
        );
    }

    #[test]
    fn a_name_that_is_not_a_device_name_is_refused() {
        let dir = tempfile::tempdir().unwrap();

        for name in ["", "../escaped", "openarm/arm", "openarm arm"] {
            let refused = claim_in(dir.path(), name, HOLDER).unwrap_err();
            assert!(matches!(refused, Error::Name { .. }), "{name}: {refused:?}");
        }
    }

    #[test]
    fn a_claim_replaces_the_name_a_previous_holder_left() {
        let dir = tempfile::tempdir().unwrap();
        drop(claim_in(dir.path(), "openarm_arm_0", "a_long_previous_instance").unwrap());

        let next = claim_in(dir.path(), "openarm_arm_0", "short").unwrap();

        assert_eq!(std::fs::read_to_string(next.path()).unwrap(), "short");
    }
}
