//! A module of `build.rs`: the download cache of a peppy container build, the directory
//! that `PEPPY_DOWNLOAD_CACHE` names, which the builds of every node share. It holds
//! files that builds downloaded, each named by its SHA-256 digest. A file is read only
//! when its content has the digest of its name, and stored under a temporary name and
//! then renamed, so a build never reads a file that another build is still writing.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Read as _, Write as _};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use sha2::{Digest as _, Sha256};

/// The SHA-256 digest of `bytes`, as 64 lowercase hexadecimal digits.
pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// The file named `digest` in the cache `directory`, when it holds at most `size` bytes
/// and its content has that digest. A missing file, one that cannot be read, a larger one
/// and one of another digest are all a miss; the read stops past `size` bytes.
pub fn cached(directory: &Path, digest: &str, size: u64) -> Option<Vec<u8>> {
    let mut bytes = Vec::new();
    File::open(directory.join(digest))
        .and_then(|file| file.take(size.saturating_add(1)).read_to_end(&mut bytes))
        .ok()?;
    let fits = u64::try_from(bytes.len()).is_ok_and(|read| read <= size);
    (fits && sha256_hex(&bytes) == digest).then_some(bytes)
}

/// Stores `bytes`, whose SHA-256 digest is `digest`, as the file named `digest` in the
/// cache `directory`, in place of any file of that name: written under a temporary name
/// of this process, then renamed.
pub fn store(directory: &Path, digest: &str, bytes: &[u8]) -> io::Result<()> {
    assert_eq!(sha256_hex(bytes), digest, "the digest of the stored bytes");
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_nanos());
    let part = directory.join(format!(".{digest}.{}.{nanos}", std::process::id()));
    let written = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&part)
        .and_then(|mut file| file.write_all(bytes))
        .and_then(|()| fs::rename(&part, directory.join(digest)));
    if written.is_err() {
        let _ = fs::remove_file(&part);
    }
    written
}
