use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=schemas/");

    // Canonicalize the manifest directory to handle symlinked source trees.
    // When peppylib is deployed via symlink (e.g. node/.peppy/libs/peppylib → shared cache),
    // Cargo sets CARGO_MANIFEST_DIR to the symlink path but CWD to the resolved path.
    // The capnp compiler resolves --src-prefix relative to CWD, so schema file paths
    // must also be canonical to ensure the prefix is stripped correctly.
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .canonicalize()
        .expect("Failed to canonicalize CARGO_MANIFEST_DIR");

    // Single source of truth: the capnp binary bundled with `build-helpers`
    // (`peppy-shared/peppy-config-model/tools`). Resolving it through
    // build-helpers means every consumer shares one copy and works whether this
    // crate is built in-tree or from a cargo git checkout. The in-place sibling
    // (`../peppy-config-model/tools`) stays as a fallback for deployed flat-cache
    // layouts where build-helpers' own copy may not be reachable.
    let capnp_path = build_helpers::bundled_capnp_path()
        .or_else(|| {
            let sibling_tools = manifest_dir
                .parent()
                .unwrap()
                .join("peppy-config-model")
                .join("tools");
            build_helpers::find_bundled_capnp(&sibling_tools)
        })
        .expect(
            "Could not find capnp binary. Please install Cap'n Proto: https://capnproto.org/install.html",
        );

    let schemas_dir = manifest_dir.join("schemas");
    for entry in std::fs::read_dir(&schemas_dir).expect("Failed to read schemas directory") {
        let entry = entry.expect("Failed to read schema directory entry");
        let path = entry.path();
        if path.extension().is_some_and(|ext| ext == "capnp") {
            capnpc::CompilerCommand::new()
                .capnp_executable(capnp_path.clone())
                .src_prefix("schemas")
                .file(&path)
                .run()
                .unwrap_or_else(|e| panic!("Failed to compile {}: {}", path.display(), e));
        }
    }
}
