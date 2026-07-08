//! Same-directory temp write + fsync + rename, so every on-disk file is either
//! the old complete version or the new complete version.

use std::fs::File;
use std::io::Write;
use std::path::Path;

use crate::error::Error;

/// Atomically replaces `dest` with `bytes`.
pub fn write_atomic(dest: &Path, bytes: &[u8]) -> Result<(), Error> {
    replace_via_temp(dest, |file, _| {
        file.write_all(bytes).map_err(Error::io(dest))
    })
}

/// Atomically replaces `dest` with whatever `write` produces.
///
/// `write` receives an open temp file in `dest`'s directory plus the temp path
/// (for writers that reopen by path). The temp file is fsynced and renamed over
/// `dest` only if `write` succeeds; on error the temp file is removed.
pub fn replace_via_temp<T>(
    dest: &Path,
    write: impl FnOnce(&mut File, &Path) -> Result<T, Error>,
) -> Result<T, Error> {
    let dir = dest.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir).map_err(Error::io(dir))?;
    let mut temp = tempfile::Builder::new()
        .prefix(".tmp-")
        .tempfile_in(dir)
        .map_err(Error::io(dir))?;
    let temp_path = temp.path().to_path_buf();
    let value = write(temp.as_file_mut(), &temp_path)?;
    temp.as_file().sync_all().map_err(Error::io(&temp_path))?;
    temp.persist(dest).map_err(|e| Error::Io {
        path: dest.to_path_buf(),
        source: e.error,
    })?;
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_and_replaces() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("nested/a.json");
        write_atomic(&dest, b"one").unwrap();
        assert_eq!(std::fs::read(&dest).unwrap(), b"one");
        write_atomic(&dest, b"two").unwrap();
        assert_eq!(std::fs::read(&dest).unwrap(), b"two");
    }

    #[test]
    fn failed_write_leaves_dest_untouched_and_no_temp() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("a.json");
        write_atomic(&dest, b"keep").unwrap();
        let failed: Result<(), Error> = replace_via_temp(&dest, |_, path| {
            Err(Error::Io {
                path: path.to_path_buf(),
                source: std::io::Error::other("boom"),
            })
        });
        assert!(failed.is_err());
        assert_eq!(std::fs::read(&dest).unwrap(), b"keep");
        let leftovers: Vec<_> = std::fs::read_dir(dir.path())
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        assert_eq!(leftovers, vec![std::ffi::OsString::from("a.json")]);
    }
}
