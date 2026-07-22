//! Cargo/build-environment helpers: target triples, env embedding, and
//! locating or compiling tool binaries.

use std::path::{Path, PathBuf};
use std::process::Command;

use crate::command::run_command_streaming;
use crate::fs::{CleanupDir, acquire_file_lock};

/// Returns the Rust target triple for the current build.
///
/// Must be called from a build script. It reads the `TARGET` environment
/// variable, which cargo only sets while running `build.rs`. The read
/// `expect()`s on purpose: the variable's absence means the function was
/// called outside that context, which is a programming error rather than a
/// recoverable runtime condition.
pub fn build_target_triple() -> String {
    std::env::var("TARGET")
        .expect("TARGET not set; build_target_triple must be called from a build script")
}

/// Embed the `PEPPY_GIT_TAG` environment variable into the binary at compile time.
///
/// If `PEPPY_GIT_TAG` is set and non-empty (by build_release.sh), emits a
/// `cargo:rustc-env` directive so the crate can read it via `env!()`.
/// Also registers `cargo:rerun-if-env-changed` so cargo rebuilds when the
/// variable changes.
pub fn embed_git_tag() {
    let tag = std::env::var("PEPPY_GIT_TAG").ok();
    for line in git_tag_directives(tag.as_deref()) {
        println!("{line}");
    }
}

/// Cargo directives emitted by [`embed_git_tag`], in emission order.
fn git_tag_directives(tag: Option<&str>) -> Vec<String> {
    let mut directives = Vec::new();
    if let Some(tag) = tag
        && !tag.is_empty()
    {
        directives.push(format!("cargo:rustc-env=PEPPY_GIT_TAG={tag}"));
    }
    directives.push("cargo:rerun-if-env-changed=PEPPY_GIT_TAG".to_string());
    directives
}

/// Find the bundled capnp binary for the build target in `tools_dir`.
///
/// The filename is chosen by `capnp_binary_name`, which selects the binary for
/// the build **target** (read from cargo's `CARGO_CFG_TARGET_OS` /
/// `CARGO_CFG_TARGET_ARCH` inside a build script). When those variables are
/// absent, for example when this helper runs outside a build script, it falls
/// back to the host platform.
///
/// Returns `Some(path)` if a binary matching that platform exists in
/// `tools_dir`, `None` otherwise. The `tools_dir` should point to the directory
/// containing platform-specific capnp binaries (e.g. `peppy-config-model/tools/`).
pub fn find_bundled_capnp(tools_dir: &Path) -> Option<PathBuf> {
    let binary_name = capnp_binary_name();
    let binary_path = tools_dir.join(binary_name);
    if binary_path.exists() {
        Some(binary_path)
    } else {
        None
    }
}

/// Locate the bundled capnp binary that ships next to this crate, in
/// `peppy-shared/peppy-config-model/tools/`, for the current host platform.
///
/// The lookup is resolved relative to *this crate's own* source directory,
/// baked in at compile time via `CARGO_MANIFEST_DIR`. That makes it the single
/// source of truth for every consumer, regardless of how `build-helpers` is
/// pulled in:
///   - As a path dependency inside the `peppy-shared` workspace, the tools
///     dir is the real sibling on disk.
///   - As a cargo **git** dependency (for example from the `peppy` workspace),
///     cargo checks out the whole `public-peppy-libs` repo, so the sibling tools
///     dir rides along in that checkout — no superproject sibling or duplicated
///     copy required.
///
/// This deliberately reads `build-helpers`'s own manifest dir, not the calling
/// build script's, so the binary is found in the one place it lives rather than
/// via fragile `../../../` paths from each consumer. Returns `Some(path)` if a
/// binary matching the host platform exists.
pub fn bundled_capnp_path() -> Option<PathBuf> {
    let tools_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("peppy-config-model")
        .join("tools");
    find_bundled_capnp(&tools_dir)
}

/// Locate the `peppy-shared` directory that this crate lives inside.
///
/// `build-helpers` always sits at `peppy-shared/build-helpers`, so the parent
/// of its own manifest dir is `peppy-shared` — the directory that holds every
/// sibling crate (`peppylib-rs`, `peppy-config-model`, `core-node-api`,
/// `peppy-messaging-interface`, `peppylib-py`, …). The path is baked in at
/// compile time via `CARGO_MANIFEST_DIR`, the same single-source approach as
/// [`bundled_capnp_path`], so it resolves correctly regardless of how
/// `build-helpers` is pulled in:
///   - As a path dependency inside `peppy-shared`, it is the real dir on disk.
///   - As a cargo **git** dependency (for example from the `peppy` workspace),
///     cargo checks out the whole `public-peppy-libs` repo, so every sibling
///     rides along in that checkout — no superproject sibling and no fragile
///     `../../../` reaches from each consumer.
///
/// Consumers such as `generator`'s build script use this to find the shared
/// crate source trees they embed, giving one source of truth instead of a
/// relative path duplicated at every call site.
pub fn peppy_shared_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("build-helpers' manifest dir always has a peppy-shared parent")
        .to_path_buf()
}

/// Filename of the bundled capnp binary to embed, selected for the build
/// **target**.
///
/// This function runs inside a build script, where cargo exports the target
/// platform as `CARGO_CFG_TARGET_OS` / `CARGO_CFG_TARGET_ARCH`. Selection must
/// key on the target, not the host: `build-helpers` is compiled as a build
/// dependency (for the host), so a compile-time `#[cfg(target_arch)]` here would
/// resolve to the *host* arch and embed the wrong binary into a cross-compiled
/// release (an aarch64 capnp inside an x86_64 build, which then ENOEXECs).
///
/// When those env vars are absent (unit tests and other non-build-script
/// callers) we fall back to the host platform, which is correct for a native
/// build and keeps direct callers working.
fn capnp_binary_name() -> &'static str {
    match (target_cfg("CARGO_CFG_TARGET_OS"), target_cfg("CARGO_CFG_TARGET_ARCH")) {
        (Some(os), Some(arch)) => capnp_binary_name_for(&os, &arch),
        _ => host_capnp_binary_name(),
    }
}

/// Reads a `CARGO_CFG_*` build-script env var, treating an empty value as unset.
fn target_cfg(var: &str) -> Option<String> {
    std::env::var(var).ok().filter(|value| !value.is_empty())
}

/// Maps an `(os, arch)` pair to the bundled capnp filename, or
/// `"capnp_unsupported"` for platforms we do not ship a binary for.
fn capnp_binary_name_for(os: &str, arch: &str) -> &'static str {
    match (os, arch) {
        ("linux", "x86_64") => "capnp_linux_x86_64",
        ("linux", "aarch64") => "capnp_linux_aarch64",
        ("macos", "aarch64") => "capnp_macos_aarch64",
        _ => "capnp_unsupported",
    }
}

fn host_capnp_binary_name() -> &'static str {
    #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
    {
        "capnp_linux_x86_64"
    }
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    {
        "capnp_linux_aarch64"
    }
    #[cfg(all(target_os = "macos", target_arch = "aarch64"))]
    {
        "capnp_macos_aarch64"
    }
    #[cfg(not(any(
        all(target_os = "linux", target_arch = "x86_64"),
        all(target_os = "linux", target_arch = "aarch64"),
        all(target_os = "macos", target_arch = "aarch64")
    )))]
    {
        "capnp_unsupported"
    }
}

/// Compile a Rust binary from crates.io using `cargo install` with cross-compilation support.
///
/// Returns `Some(path)` to the cached binary on success, `None` on failure.
/// Uses a separate `CARGO_TARGET_DIR` to avoid lock conflicts with the outer
/// cargo build. Concurrent installs of the same tool sharing a `cache_dir`
/// (for example, two worktrees building at once) are serialized with a file
/// lock — acquisition blocks until the lock is free and panics if the lock
/// cannot be taken — and the cached binary is published with an atomic
/// rename so a concurrent reader can never observe a partially written file.
pub fn cargo_install_binary(
    name: &str,
    version: &str,
    target: &str,
    cache_dir: &Path,
) -> Option<PathBuf> {
    // Cargo sets $CARGO to the exact cargo that launched this build script. Run
    // the nested install with that same binary so it matches the outer build's
    // toolchain instead of whatever cargo happens to come first on PATH.
    let cargo_program = std::env::var_os("CARGO").map(PathBuf::from);
    let cargo_program = cargo_program.as_deref().unwrap_or(Path::new("cargo"));
    cargo_install_binary_with(cargo_program, name, version, target, cache_dir)
}

/// Implementation of [`cargo_install_binary`] with the cargo executable made
/// explicit, so tests can substitute a fixture script.
fn cargo_install_binary_with(
    cargo_program: &Path,
    name: &str,
    version: &str,
    target: &str,
    cache_dir: &Path,
) -> Option<PathBuf> {
    fn use_cached(name: &str, cached_binary: PathBuf) -> Option<PathBuf> {
        println!("cargo:warning=Using cached {name} binary from {cached_binary:?}");
        Some(cached_binary)
    }

    let cached_binary = cache_dir.join(format!("{name}-{version}-{target}"));

    if cached_binary.exists() {
        return use_cached(name, cached_binary);
    }

    // Serialize concurrent installs sharing this cache dir, then re-check:
    // another process may have populated the cache while we waited. The lock
    // is keyed by name alone because the install/build temp dirs below are
    // shared by all versions and targets of the tool.
    let _lock = acquire_file_lock(&cache_dir.join(format!("{name}.lock")));
    if cached_binary.exists() {
        return use_cached(name, cached_binary);
    }

    println!(
        "cargo:warning=Compiling {name} {version} from source for {target} (this may take several minutes)..."
    );

    let install_root = cache_dir.join(format!("{name}-install-tmp"));
    let cargo_target_dir = cache_dir.join(format!("cargo-build-{name}"));

    // Clean up any previous partial install
    std::fs::remove_dir_all(&install_root).ok();
    std::fs::create_dir_all(&install_root).ok();
    std::fs::create_dir_all(&cargo_target_dir).ok();

    // Guards ensure temp directories are cleaned up on all exit paths.
    let _install_guard = CleanupDir(install_root.clone());
    let _target_guard = CleanupDir(cargo_target_dir.clone());

    let crate_spec = format!("{name}@{version}");
    let mut cmd = Command::new(cargo_program);
    cmd.args(["install", &crate_spec, "--target", target, "--root"])
        .arg(&install_root)
        .env("CARGO_TARGET_DIR", &cargo_target_dir);

    let label = format!("cargo-install-{name}");
    let output = run_command_streaming(&mut cmd, &label);
    if !output.success {
        return None;
    }

    let built_binary = install_root.join("bin").join(name);
    if !built_binary.exists() {
        println!(
            "cargo:warning=cargo install succeeded but binary not found at {:?}",
            built_binary
        );
        return None;
    }

    // Publish atomically: stage next to the cache key, then rename onto it,
    // so the lock-free fast path above never observes a torn binary. The
    // fixed staging name cannot collide — staging only happens under the
    // lock — and a leftover from a killed build is truncated by the copy.
    let staged = cache_dir.join(format!("{name}-{version}-{target}.tmp"));
    let published = std::fs::copy(&built_binary, &staged)
        .and_then(|_| std::fs::rename(&staged, &cached_binary));
    if let Err(e) = published {
        std::fs::remove_file(&staged).ok();
        println!("cargo:warning=Failed to cache compiled {name} binary: {e}");
        return None;
    }

    println!("cargo:warning=Successfully compiled and cached {name} {version} for {target}");
    Some(cached_binary)
}

#[cfg(test)]
mod tests {
    use super::*;

    const RERUN_DIRECTIVE: &str = "cargo:rerun-if-env-changed=PEPPY_GIT_TAG";

    #[test]
    fn git_tag_directives_emits_rustc_env_then_rerun_for_nonempty_tag() {
        assert_eq!(
            git_tag_directives(Some("v1.2.3")),
            ["cargo:rustc-env=PEPPY_GIT_TAG=v1.2.3", RERUN_DIRECTIVE]
        );
    }

    #[test]
    fn git_tag_directives_emits_only_rerun_when_tag_unset() {
        assert_eq!(git_tag_directives(None), [RERUN_DIRECTIVE]);
    }

    #[test]
    fn git_tag_directives_emits_only_rerun_when_tag_empty() {
        assert_eq!(git_tag_directives(Some("")), [RERUN_DIRECTIVE]);
    }

    #[test]
    fn find_bundled_capnp_returns_none_for_empty_dir() {
        let dir = tempfile::tempdir().expect("temp dir");
        assert_eq!(find_bundled_capnp(dir.path()), None);
    }

    #[test]
    fn find_bundled_capnp_finds_host_binary() {
        let dir = tempfile::tempdir().expect("temp dir");
        let expected = dir.path().join(host_capnp_binary_name());
        std::fs::write(&expected, b"").expect("create fake capnp");
        assert_eq!(find_bundled_capnp(dir.path()), Some(expected));
    }

    #[test]
    fn find_bundled_capnp_ignores_wrongly_named_binary() {
        let dir = tempfile::tempdir().expect("temp dir");
        std::fs::write(dir.path().join("capnp_wrong_name"), b"").expect("create file");
        assert_eq!(find_bundled_capnp(dir.path()), None);
    }

    #[test]
    fn capnp_binary_name_for_selects_by_target_not_host() {
        // The mapping is driven purely by the (os, arch) pair cargo reports for
        // the build target, so a cross-compiled release embeds the right binary
        // regardless of the host it was built on.
        assert_eq!(capnp_binary_name_for("linux", "x86_64"), "capnp_linux_x86_64");
        assert_eq!(capnp_binary_name_for("linux", "aarch64"), "capnp_linux_aarch64");
        assert_eq!(capnp_binary_name_for("macos", "aarch64"), "capnp_macos_aarch64");
    }

    #[test]
    fn capnp_binary_name_for_reports_unsupported_platforms() {
        assert_eq!(capnp_binary_name_for("windows", "x86_64"), "capnp_unsupported");
        assert_eq!(capnp_binary_name_for("macos", "x86_64"), "capnp_unsupported");
        assert_eq!(capnp_binary_name_for("linux", "arm"), "capnp_unsupported");
    }

    #[test]
    fn bundled_capnp_path_resolves_for_supported_host() {
        // On the platforms we bundle binaries for, `bundled_capnp_path` must
        // locate one relative to this crate's own source dir — the single source
        // of truth in `peppy-config-model/tools/`. Hosts we don't bundle for
        // legitimately return `None`, so only assert on supported hosts.
        if host_capnp_binary_name() == "capnp_unsupported" {
            return;
        }
        assert!(
            bundled_capnp_path().is_some(),
            "expected a bundled capnp binary for host {}",
            host_capnp_binary_name()
        );
    }

    #[test]
    fn peppy_shared_dir_contains_sibling_crates() {
        // The locator must point at the real `peppy-shared` dir: the place that
        // holds this crate alongside its siblings. Assert via crates that always
        // exist so consumers (e.g. generator) can rely on joining a sibling name.
        let shared = peppy_shared_dir();
        for sibling in ["build-helpers", "peppy-config-model", "peppylib-rs"] {
            assert!(
                shared.join(sibling).is_dir(),
                "peppy_shared_dir() should contain {sibling}, got {}",
                shared.display()
            );
        }
    }

    #[test]
    fn cargo_install_binary_returns_cached_binary_without_installing() {
        let dir = tempfile::tempdir().expect("temp dir");
        // Pre-populating the `{name}-{version}-{target}` cache key must
        // short-circuit the install; this pins the filename contract that
        // peppy-messaging-interface and generator-internal build scripts rely on. The
        // missing cargo program makes a fast-path regression fail fast
        // instead of invoking the real cargo against the network.
        let cached = dir.path().join("mytool-1.0.0-x86_64-unknown-linux-gnu");
        std::fs::write(&cached, b"cached").expect("pre-populate cache");
        assert_eq!(
            cargo_install_binary_with(
                &dir.path().join("no-such-cargo"),
                "mytool",
                "1.0.0",
                "x86_64-unknown-linux-gnu",
                dir.path()
            ),
            Some(cached)
        );
    }

    /// Writes an executable shell script that stands in for `cargo` so the
    /// install paths can be tested without network access or PATH mutation.
    ///
    /// The script is written by a child shell rather than `std::fs::write`:
    /// a write fd opened in this multithreaded test process leaks into
    /// children forked concurrently by other tests, and exec'ing a file
    /// somebody still holds open for writing fails with ETXTBSY.
    #[cfg(unix)]
    fn write_fake_cargo(dir: &Path, body: &str) -> PathBuf {
        let path = dir.join("fake-cargo");
        let status = Command::new("sh")
            .args(["-c", r#"printf '%s' "$1" > "$2" && chmod 755 "$2""#, "sh"])
            .arg(format!("#!/bin/sh\n{body}\n"))
            .arg(&path)
            .status()
            .expect("write fixture script");
        assert!(status.success(), "fixture script write failed");
        path
    }

    #[cfg(unix)]
    fn temp_cache_dir(dir: &Path) -> PathBuf {
        let cache = dir.join("cache");
        std::fs::create_dir_all(&cache).expect("create cache dir");
        cache
    }

    #[cfg(unix)]
    #[test]
    fn cargo_install_binary_with_caches_built_binary_and_cleans_temp_dirs() {
        let dir = tempfile::tempdir().expect("temp dir");
        // The script fakes a successful `cargo install` by writing
        // bin/<name> under the --root it is given.
        let script = write_fake_cargo(
            dir.path(),
            r#"root=""
while [ $# -gt 0 ]; do
  if [ "$1" = "--root" ]; then root="$2"; shift; fi
  shift
done
mkdir -p "$root/bin"
printf fake-binary > "$root/bin/mytool""#,
        );
        let cache = temp_cache_dir(dir.path());

        let result = cargo_install_binary_with(&script, "mytool", "1.0.0", "test-target", &cache);

        let cached = cache.join("mytool-1.0.0-test-target");
        assert_eq!(result, Some(cached.clone()));
        assert_eq!(std::fs::read(&cached).expect("read cached"), b"fake-binary");
        assert!(!cache.join("mytool-install-tmp").exists());
        assert!(!cache.join("cargo-build-mytool").exists());
    }

    #[cfg(unix)]
    #[test]
    fn cargo_install_binary_with_returns_none_on_install_failure() {
        let dir = tempfile::tempdir().expect("temp dir");
        let script = write_fake_cargo(dir.path(), "exit 1");
        let cache = temp_cache_dir(dir.path());

        assert_eq!(
            cargo_install_binary_with(&script, "mytool", "1.0.0", "test-target", &cache),
            None
        );
        assert!(!cache.join("mytool-install-tmp").exists());
        assert!(!cache.join("cargo-build-mytool").exists());
    }
}
