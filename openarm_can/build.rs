fn main() {
    use std::env;

    // `openarm_sdk` is set when the C++ SDK is present and the wrapper is built;
    // lib.rs gates the FFI on it. Declare it so `#[cfg(openarm_sdk)]` never warns.
    println!("cargo:rustc-check-cfg=cfg(openarm_sdk)");
    println!("cargo:rerun-if-env-changed=OPENARM_CAN_INCLUDE_DIR");

    // The openarm_can C++ SDK (libopenarm-can-dev) only exists on Linux, and even
    // there only on machines provisioned for the hardware. When it's absent (a dev
    // workstation, CI, or any non-Linux host) we skip the C++ wrapper entirely so
    // the pure-Rust parts still build and `cargo test` can run; lib.rs falls back to
    // stub FFI under `cfg(not(openarm_sdk))`.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let explicit_include = env::var("OPENARM_CAN_INCLUDE_DIR").ok();

    let have_sdk = target_os == "linux" && sdk_header_present(explicit_include.as_deref());

    if !have_sdk {
        if target_os == "linux" {
            println!(
                "cargo:warning=openarm_can: C++ SDK headers not found; building without hardware \
                 support (ArmCan/GripperCan::new will return CanError::OpenFailed). Install \
                 libopenarm-can-dev or set OPENARM_CAN_INCLUDE_DIR to enable hardware."
            );
        }
        // The FFI extern block in lib.rs is only compiled under cfg(openarm_sdk).
        return;
    }

    let mut build = cc::Build::new();
    build.file("wrapper.cpp").cpp(true).flag("-std=c++17");
    if let Some(dir) = &explicit_include {
        build.include(dir);
    }
    build.compile("openarm_can_wrapper");

    println!("cargo:rustc-link-lib=openarm_can");
    println!("cargo:rustc-link-lib=stdc++");
    println!("cargo:rustc-cfg=openarm_sdk");

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=wrapper.cpp");
}

/// True if the openarm_can SDK header can be found, so the C++ wrapper will compile.
/// Checks `$OPENARM_CAN_INCLUDE_DIR` (if set) then the standard system prefixes.
fn sdk_header_present(explicit: Option<&str>) -> bool {
    use std::path::Path;

    const HEADER: &str = "openarm/can/socket/openarm.hpp";
    let mut roots: Vec<&Path> = Vec::new();
    if let Some(dir) = explicit {
        let root = Path::new(dir);
        if !root.join(HEADER).exists() {
            println!(
                "cargo:warning=OPENARM_CAN_INCLUDE_DIR is set to {dir} but {HEADER} was not \
                 found there; falling back to system include directories"
            );
        }
        roots.push(root);
    }
    roots.push(Path::new("/usr/include"));
    roots.push(Path::new("/usr/local/include"));
    roots.iter().any(|root| root.join(HEADER).exists())
}
