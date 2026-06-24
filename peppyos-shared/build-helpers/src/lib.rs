//! Generic build-script helpers shared across peppy crates.
//!
//! This crate lives in `nodes_shared_code/peppyos-shared` so it sits at the
//! bottom of the dependency graph: both the `peppyos` workspace crates and the
//! `peppyos-shared` crates depend on it, never the other way around. Helpers
//! that are specific to a single `peppyos` crate (for example the peppylib
//! native-extension rebuild policy) live in their own crate next to that
//! consumer instead, so this crate stays free of reverse paths into `peppyos`.
//!
//! Functionality is grouped into focused submodules and re-exported flat so
//! build scripts can keep calling `build_helpers::<fn>`.

#![forbid(unsafe_code)]

mod cargo;
mod command;
mod fs;
mod hash;

pub use cargo::{
    build_target_triple, bundled_capnp_path, cargo_install_binary, embed_git_tag,
    find_bundled_capnp,
};
pub use command::{CommandOutput, run_command, run_command_streaming, run_command_with_timeout};
pub use fs::{acquire_file_lock, cache_dir, copy_if_changed, set_executable, write_if_changed};
pub use hash::verify_sha256;
