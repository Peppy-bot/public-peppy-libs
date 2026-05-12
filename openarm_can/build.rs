fn main() {
    use std::env;
    use std::path::PathBuf;

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bindings_path = out_path.join("bindings.rs");

    // libopenarm-can-dev is only available on Linux.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "linux" {
        std::fs::write(&bindings_path, "").expect("Failed to write empty bindings.rs");
        return;
    }

    cc::Build::new()
        .file("wrapper.cpp")
        .cpp(true)
        .flag("-std=c++17")
        .compile("openarm_can_wrapper");

    println!("cargo:rustc-link-lib=openarm_can");
    println!("cargo:rustc-link-lib=stdc++");

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=wrapper.cpp");

    bindgen::Builder::default()
        .header("wrapper.h")
        .allowlist_function("openarm_.*")
        .allowlist_type("OpenArmHandle")
        .rust_edition(bindgen::RustEdition::Edition2024)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Failed to generate bindings from wrapper.h")
        .write_to_file(&bindings_path)
        .expect("Failed to write bindings.rs");
}
