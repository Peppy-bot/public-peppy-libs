//! Build-script helpers shared across peppy crates.
//!
//! Functionality is grouped into focused submodules and re-exported flat so
//! build scripts can keep calling `build_helpers::<fn>`.

#![forbid(unsafe_code)]

mod cargo;
mod command;
mod fs;

pub use cargo::{cargo_install_binary, embed_git_tag, find_bundled_capnp};
pub use fs::{acquire_file_lock, cache_dir, copy_if_changed, set_executable};
