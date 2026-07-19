//! SHA-256 hashing and verification of downloaded artifacts.

use std::io::Read;
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

fn digest_hex(hash: impl AsRef<[u8]>) -> String {
    let hash = hash.as_ref();
    let mut hex = String::with_capacity(hash.len() * 2);
    for byte in hash {
        use std::fmt::Write;
        write!(hex, "{byte:02x}").unwrap();
    }
    hex
}

/// Computes the SHA-256 hash of a file using the `sha2` crate. Returns `None` on I/O error.
fn sha256_file(path: &Path) -> Option<String> {
    let mut file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            println!(
                "cargo:warning=Failed to open {:?} for SHA-256 verification: {}",
                path, e
            );
            return None;
        }
    };

    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let n = match file.read(&mut buffer) {
            Ok(n) => n,
            Err(e) => {
                println!(
                    "cargo:warning=Failed to read {:?} for SHA-256 verification: {}",
                    path, e
                );
                return None;
            }
        };
        if n == 0 {
            break;
        }
        hasher.update(&buffer[..n]);
    }

    Some(digest_hex(hasher.finalize()))
}

/// Hashes every regular file below `root`, plus a caller-owned policy tag.
///
/// Relative path names, file lengths, and bytes are framed independently and
/// processed in sorted path order, so renames and boundary changes cannot
/// collide through simple concatenation. Symlinks are rejected: build inputs
/// must be a self-contained source tree whose cache key does not depend on an
/// external target changing behind it.
pub(crate) fn sha256_source_tree_with_tag(root: &Path, policy_tag: &str) -> Option<String> {
    fn collect_files(dir: &Path, files: &mut Vec<PathBuf>) -> std::io::Result<()> {
        for entry in std::fs::read_dir(dir)? {
            let entry = entry?;
            let file_type = entry.file_type()?;
            let path = entry.path();
            if file_type.is_dir() {
                collect_files(&path, files)?;
            } else if file_type.is_file() {
                files.push(path);
            } else {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("source tree contains unsupported entry {}", path.display()),
                ));
            }
        }
        Ok(())
    }

    let mut files = Vec::new();
    if let Err(error) = collect_files(root, &mut files) {
        println!(
            "cargo:warning=Failed to enumerate patched source tree {}: {error}",
            root.display()
        );
        return None;
    }
    files.sort();

    let mut hasher = Sha256::new();
    hasher.update(b"peppy-source-policy-v1\0");
    hasher.update((policy_tag.len() as u64).to_le_bytes());
    hasher.update(policy_tag.as_bytes());
    for path in files {
        let relative = match path.strip_prefix(root) {
            Ok(relative) => relative.to_string_lossy(),
            Err(error) => {
                println!(
                    "cargo:warning=Failed to relativize patched source {}: {error}",
                    path.display()
                );
                return None;
            }
        };
        let metadata = match std::fs::metadata(&path) {
            Ok(metadata) => metadata,
            Err(error) => {
                println!(
                    "cargo:warning=Failed to stat patched source {}: {error}",
                    path.display()
                );
                return None;
            }
        };
        hasher.update((relative.len() as u64).to_le_bytes());
        hasher.update(relative.as_bytes());
        hasher.update(metadata.len().to_le_bytes());

        let mut file = match std::fs::File::open(&path) {
            Ok(file) => file,
            Err(error) => {
                println!(
                    "cargo:warning=Failed to open patched source {}: {error}",
                    path.display()
                );
                return None;
            }
        };
        let mut buffer = [0_u8; 64 * 1024];
        loop {
            let read = match file.read(&mut buffer) {
                Ok(read) => read,
                Err(error) => {
                    println!(
                        "cargo:warning=Failed to hash patched source {}: {error}",
                        path.display()
                    );
                    return None;
                }
            };
            if read == 0 {
                break;
            }
            hasher.update(&buffer[..read]);
        }
    }

    Some(digest_hex(hasher.finalize()))
}

/// Verifies the SHA-256 hash of a file against an expected value.
/// Returns `true` if the hash matches.
pub fn verify_sha256(path: &Path, expected: &str, label: &str) -> bool {
    let Some(actual) = sha256_file(path) else {
        return false;
    };

    if actual.eq_ignore_ascii_case(expected) {
        true
    } else {
        println!(
            "cargo:warning={} SHA-256 mismatch for {:?}: expected {}, got {}",
            label, path, expected, actual
        );
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SHA-256 of b"abc" (FIPS 180-2 test vector).
    const ABC_SHA256: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

    fn temp_file_with(contents: &[u8]) -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().expect("temp dir");
        let path = dir.path().join("artifact.bin");
        std::fs::write(&path, contents).expect("write artifact");
        (dir, path)
    }

    #[test]
    fn verify_sha256_accepts_matching_hash() {
        let (_dir, path) = temp_file_with(b"abc");
        assert!(verify_sha256(&path, ABC_SHA256, "test"));
    }

    #[test]
    fn verify_sha256_is_case_insensitive() {
        let (_dir, path) = temp_file_with(b"abc");
        assert!(verify_sha256(&path, &ABC_SHA256.to_uppercase(), "test"));
    }

    #[test]
    fn verify_sha256_rejects_mismatching_hash() {
        let (_dir, path) = temp_file_with(b"abc");
        assert!(!verify_sha256(&path, &"0".repeat(64), "test"));
    }

    #[test]
    fn verify_sha256_rejects_missing_file() {
        let dir = tempfile::tempdir().expect("temp dir");
        assert!(!verify_sha256(
            &dir.path().join("no-such-file"),
            ABC_SHA256,
            "test"
        ));
    }

    #[test]
    fn verify_sha256_hashes_files_larger_than_the_read_buffer() {
        // 200,000 bytes exceeds the 64 KiB read buffer, forcing the read
        // loop through multiple iterations.
        let (_dir, path) = temp_file_with(&vec![b'a'; 200_000]);
        assert!(verify_sha256(
            &path,
            "2287d207f24a941ff3b56c04c8a25ad56b63e3023207b3bb5b4ac0c9869d74be",
            "test"
        ));
    }

    #[test]
    fn source_tree_hash_covers_policy_paths_and_contents_deterministically() {
        let dir = tempfile::tempdir().expect("temp dir");
        std::fs::create_dir(dir.path().join("nested")).expect("create nested dir");
        std::fs::write(dir.path().join("a"), b"alpha").expect("write a");
        std::fs::write(dir.path().join("nested/b"), b"beta").expect("write b");

        let initial = sha256_source_tree_with_tag(dir.path(), "policy-v1").expect("hash tree");
        assert_eq!(
            initial,
            sha256_source_tree_with_tag(dir.path(), "policy-v1").expect("rehash tree")
        );
        assert_ne!(
            initial,
            sha256_source_tree_with_tag(dir.path(), "policy-v2").expect("hash changed policy")
        );

        std::fs::write(dir.path().join("nested/b"), b"changed").expect("change b");
        assert_ne!(
            initial,
            sha256_source_tree_with_tag(dir.path(), "policy-v1").expect("hash changed tree")
        );
    }
}
