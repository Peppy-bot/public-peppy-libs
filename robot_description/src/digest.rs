//! The digest of a description's files. The build script computes it for each bundled
//! model, and a test computes it again from the embedded files.

use sha2::{Digest, Sha256};

/// The SHA-256 digest, in lowercase hex, of a description: its URDF, its SRDF, and each
/// collision mesh by its file name, the meshes in the order of their names. Each part
/// goes in behind its length, so no two descriptions share a digest by where one part
/// ends and the next begins.
pub fn digest_of(urdf: &[u8], srdf: &[u8], meshes: &[(String, Vec<u8>)]) -> String {
    let mut sorted: Vec<&(String, Vec<u8>)> = meshes.iter().collect();
    sorted.sort_by(|a, b| a.0.cmp(&b.0));
    let mut hasher = Sha256::new();
    let mut part = |bytes: &[u8]| {
        hasher.update((bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);
    };
    part(urdf);
    part(srdf);
    for (name, bytes) in sorted {
        part(name.as_bytes());
        part(bytes);
    }
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}
