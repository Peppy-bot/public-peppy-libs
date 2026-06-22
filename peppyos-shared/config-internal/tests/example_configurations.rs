//! Integration test that parses every example node config to ensure the schema
//! types stay in sync with the ground-truth files.
//!
//! If this test fails, it means the node config types in `config-internal` have
//! drifted from the example files — either the code or the examples need updating.

use config::consts::NODE_CONFIG_FILE;
use config::node::NodeConfigParser;
use std::path::{Path, PathBuf};

/// Walk `root` recursively and collect every file named `peppy.json5`.
fn find_peppy_configs(root: &Path) -> Vec<PathBuf> {
    walkdir::WalkDir::new(root)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_file())
        .filter(|e| {
            e.file_name()
                .to_str()
                .is_some_and(|name| name == NODE_CONFIG_FILE)
        })
        .map(|e| e.into_path())
        .collect()
}

#[test]
fn test_example_node_configs_parse() {
    let examples_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("nodes_example_1");

    assert!(
        examples_root.is_dir(),
        "examples directory not found: {}",
        examples_root.display()
    );

    let configs = find_peppy_configs(&examples_root);

    assert!(
        configs.len() >= 5,
        "expected at least 5 node configs under {}, found {}",
        examples_root.display(),
        configs.len()
    );

    for path in &configs {
        let result = NodeConfigParser::from_path(path);
        assert!(
            result.is_ok(),
            "failed to parse {}: {:?}",
            path.display(),
            result.unwrap_err()
        );
    }
}
