use sha2::{Digest, Sha256};
use std::fs;
use std::path::Path;

const NODE_CONFIG_FINGERPRINT_FILE: &str = "peppy.json5.sha256";

#[cfg(feature = "test_helpers")]
pub fn fingerprint_for_bytes(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    digest
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>()
}

/// Reads the codegen fingerprint from the generated output directory.
///
/// The fingerprint file is located at `{peppy_config_dir}/{output_path}/{fingerprint_file}`.
pub fn read_codegen_fingerprint(
    peppy_config: impl AsRef<Path>,
    output_path: impl AsRef<Path>,
) -> crate::error::Result<String> {
    let peppy_config_dir = peppy_config
        .as_ref()
        .parent()
        .unwrap_or_else(|| Path::new("."));
    let fingerprint_path = peppy_config_dir
        .join(output_path.as_ref())
        .join(NODE_CONFIG_FINGERPRINT_FILE);

    fs::read_to_string(&fingerprint_path)
        .map(|s| s.trim().to_string())
        .map_err(Into::into)
}

/// Creates the fingerprint files at the expected location for runtime checks.
///
/// This creates both:
/// 1. The config fingerprint (`peppy.json5.sha256`) in the peppygen output directory
#[cfg(feature = "test_helpers")]
pub fn create_codegen_fingerprint(peppy_config_path: &Path, output_path: &Path) {
    let peppy_config_dir = peppy_config_path.parent().unwrap_or(Path::new("."));
    let fingerprint_dir = peppy_config_dir.join(output_path);
    fs::create_dir_all(&fingerprint_dir).expect("fingerprint dir should be created");

    // Create config fingerprint in peppygen output directory
    let fingerprint_path = fingerprint_dir.join(NODE_CONFIG_FINGERPRINT_FILE);
    let fingerprint = fingerprint_for_bytes(
        &fs::read(peppy_config_path).expect("peppy config should be readable"),
    );
    fs::write(&fingerprint_path, format!("{fingerprint}\n"))
        .expect("fingerprint should be written");
}

/// Creates a config fingerprint file with incorrect content to test mismatch errors.
#[cfg(feature = "test_helpers")]
pub fn create_wrong_codegen_fingerprint(peppy_config_path: &Path, output_path: &Path) {
    let peppy_config_dir = peppy_config_path.parent().unwrap_or(Path::new("."));
    let fingerprint_dir = peppy_config_dir.join(output_path);
    fs::create_dir_all(&fingerprint_dir).expect("fingerprint dir should be created");
    let fingerprint_path = fingerprint_dir.join(NODE_CONFIG_FINGERPRINT_FILE);
    fs::write(&fingerprint_path, "wrong_fingerprint_value\n")
        .expect("fingerprint should be written");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn read_codegen_fingerprint_errors_when_file_missing() {
        let tmp = TempDir::new().expect("failed to create temp dir");
        let config_path = tmp.path().join(crate::consts::NODE_CONFIG_FILE);
        fs::write(&config_path, "contents").expect("failed to write config");

        // No fingerprint was ever generated, so the read must fail rather than
        // silently returning an empty/garbage digest.
        let rel_output = std::path::Path::new(crate::consts::PEPPYGEN_OUTPUT_PATH);
        assert!(read_codegen_fingerprint(&config_path, rel_output).is_err());
    }
}
