//! The download cache that the build reads the OMPL source archive from, and stores it in
//! once downloaded: a file is read only when it holds at most the archive's size and its
//! content has the digest of its name, and a store replaces a file of another digest and
//! leaves no temporary file behind.

#[path = "../download_cache.rs"]
mod download_cache;

use std::fs;
use std::path::{Path, PathBuf};

use download_cache::{cached, sha256_hex, store};

const ARCHIVE: &[u8] = b"the bytes of an archive";

/// The size of `ARCHIVE` in bytes.
const ARCHIVE_BYTES: u64 = ARCHIVE.len() as u64;

/// An empty directory of this test's own, `name`, under cargo's scratch directory for
/// tests.
fn empty_directory(name: &str) -> PathBuf {
    let directory = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    if directory.exists() {
        fs::remove_dir_all(&directory).expect("the removal of an earlier run's directory");
    }
    fs::create_dir_all(&directory).expect("the test's directory");
    directory
}

/// The names of the files of `directory`, in order.
fn file_names(directory: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(directory)
        .expect("the directory lists")
        .map(|entry| {
            entry
                .expect("an entry reads")
                .file_name()
                .into_string()
                .expect("a UTF-8 name")
        })
        .collect();
    names.sort();
    names
}

#[test]
fn the_digest_is_the_sha256_of_the_bytes_in_lowercase_hexadecimal() {
    assert_eq!(
        sha256_hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
}

#[test]
fn an_empty_cache_holds_no_archive() {
    let cache = empty_directory("an_empty_cache_holds_no_archive");
    assert_eq!(cached(&cache, &sha256_hex(ARCHIVE), ARCHIVE_BYTES), None);
}

#[test]
fn a_stored_archive_reads_back_under_its_digest_alone() {
    let cache = empty_directory("a_stored_archive_reads_back_under_its_digest_alone");
    let digest = sha256_hex(ARCHIVE);
    store(&cache, &digest, ARCHIVE).expect("the store");
    assert_eq!(
        cached(&cache, &digest, ARCHIVE_BYTES).as_deref(),
        Some(ARCHIVE)
    );
    assert_eq!(file_names(&cache), [digest]);
}

#[test]
fn a_file_larger_than_the_size_is_a_miss() {
    let cache = empty_directory("a_file_larger_than_the_size_is_a_miss");
    let digest = sha256_hex(ARCHIVE);
    store(&cache, &digest, ARCHIVE).expect("the store");
    assert_eq!(cached(&cache, &digest, ARCHIVE_BYTES - 1), None);
}

#[test]
fn a_file_of_another_digest_is_a_miss_and_a_store_replaces_it() {
    let cache = empty_directory("a_file_of_another_digest_is_a_miss_and_a_store_replaces_it");
    let digest = sha256_hex(ARCHIVE);
    fs::write(cache.join(&digest), b"a truncated archive").expect("the corrupt file");
    assert_eq!(cached(&cache, &digest, ARCHIVE_BYTES), None);
    store(&cache, &digest, ARCHIVE).expect("the store");
    assert_eq!(
        cached(&cache, &digest, ARCHIVE_BYTES).as_deref(),
        Some(ARCHIVE)
    );
    assert_eq!(file_names(&cache), [digest]);
}

#[test]
fn a_store_into_a_missing_directory_fails_and_leaves_nothing() {
    let parent = empty_directory("a_store_into_a_missing_directory_fails_and_leaves_nothing");
    let cache = parent.join("missing");
    assert!(store(&cache, &sha256_hex(ARCHIVE), ARCHIVE).is_err());
    assert!(file_names(&parent).is_empty());
}

#[test]
#[should_panic(expected = "the digest of the stored bytes")]
fn a_store_under_another_digest_is_refused() {
    let cache = empty_directory("a_store_under_another_digest_is_refused");
    store(&cache, &sha256_hex(b"other bytes"), ARCHIVE).ok();
}
