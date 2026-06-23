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

    // config/tools holds the bundled capnp binaries. It lives in two
    // places depending on how peppylib is being built:
    //   1. Deployed flat cache (`.peppy/libs/<hash>/peppylib`): config is
    //      a flat sibling, so `../config/tools`.
    //   2. Superproject dev checkout (`nodes_shared_code/peppyos-shared/peppylib-rs`):
    //      config stays in the peppyos submodule, reached via the reverse
    //      path `../../../peppyos/crates/config/tools`.
    // manifest_dir is canonicalized above, so the deployed crate dir resolves to
    // the real shared-cache path (leaf "peppylib"); the dev crate dir leaf is
    // "peppylib-rs". Both candidates are evaluated against that canonical base.
    let sibling_tools = manifest_dir.parent().unwrap().join("config").join("tools");
    let reverse_tools = manifest_dir.join("../../../peppyos/crates/config/tools");
    let tools_dir = [sibling_tools, reverse_tools]
        .into_iter()
        .find(|candidate| candidate.exists())
        .expect(
            "Could not locate config/tools (capnp binaries) as a flat \
             sibling or via the peppyos reverse path",
        );
    let capnp_path = build_helpers::find_bundled_capnp(&tools_dir).expect(
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
