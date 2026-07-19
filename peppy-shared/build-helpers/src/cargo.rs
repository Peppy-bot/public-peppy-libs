//! Cargo/build-environment helpers: target triples, env embedding, and
//! locating or compiling tool binaries.

use std::path::{Path, PathBuf};
use std::process::Command;

use crate::command::run_command_streaming;
use crate::fs::{CleanupDir, acquire_file_lock};
use crate::hash::sha256_source_tree_with_tag;

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

/// Find the bundled capnp binary for the current host platform in `tools_dir`.
///
/// Returns `Some(path)` if a binary matching the host OS/arch exists,
/// `None` otherwise. The `tools_dir` should point to the directory containing
/// platform-specific capnp binaries (e.g. `peppy-config-model/tools/`).
pub fn find_bundled_capnp(tools_dir: &Path) -> Option<PathBuf> {
    let binary_name = host_capnp_binary_name();
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

/// Compile a crates.io binary while replacing one dependency with a local
/// source tree.
///
/// The cached filename contains both `policy_tag` and a SHA-256 digest of the
/// complete patch tree. Consequently a stock binary, a binary built under an
/// older policy, or one built from older patch contents can never satisfy this
/// request. The nested Cargo invocation receives an explicit crates.io patch;
/// callers do not have to rely on a parent workspace's Cargo configuration.
pub fn cargo_install_binary_with_source_patch(
    name: &str,
    version: &str,
    target: &str,
    cache_dir: &Path,
    patch_crate_name: &str,
    patch_source_dir: &Path,
    policy_tag: &str,
) -> Option<PathBuf> {
    let cargo_program = std::env::var_os("CARGO").map(PathBuf::from);
    let cargo_program = cargo_program.as_deref().unwrap_or(Path::new("cargo"));
    cargo_install_binary_with_source_patch_using(
        cargo_program,
        name,
        version,
        target,
        cache_dir,
        patch_crate_name,
        patch_source_dir,
        policy_tag,
    )
}

struct CargoSourcePatch<'a> {
    crate_name: &'a str,
    source_dir: PathBuf,
    cache_discriminator: String,
}

fn cache_component(value: &str) -> String {
    let value: String = value
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || matches!(character, '-' | '_') {
                character
            } else {
                '-'
            }
        })
        .take(64)
        .collect();
    if value.is_empty() {
        "policy".to_string()
    } else {
        value
    }
}

#[allow(clippy::too_many_arguments)]
fn cargo_install_binary_with_source_patch_using(
    cargo_program: &Path,
    name: &str,
    version: &str,
    target: &str,
    cache_dir: &Path,
    patch_crate_name: &str,
    patch_source_dir: &Path,
    policy_tag: &str,
) -> Option<PathBuf> {
    if patch_crate_name.is_empty()
        || !patch_crate_name
            .chars()
            .all(|character| character.is_ascii_alphanumeric() || matches!(character, '-' | '_'))
    {
        println!("cargo:warning=Invalid crates.io patch name `{patch_crate_name}`");
        return None;
    }
    let source_dir = match std::fs::canonicalize(patch_source_dir) {
        Ok(source_dir) if source_dir.is_dir() => source_dir,
        Ok(source_dir) => {
            println!(
                "cargo:warning=Patched source is not a directory: {}",
                source_dir.display()
            );
            return None;
        }
        Err(error) => {
            println!(
                "cargo:warning=Failed to resolve patched source {}: {error}",
                patch_source_dir.display()
            );
            return None;
        }
    };
    let source_hash = sha256_source_tree_with_tag(&source_dir, policy_tag)?;
    let patch = CargoSourcePatch {
        crate_name: patch_crate_name,
        source_dir,
        cache_discriminator: format!("{}-{source_hash}", cache_component(policy_tag)),
    };
    cargo_install_binary_with_options(
        cargo_program,
        name,
        version,
        target,
        cache_dir,
        Some(&patch),
    )
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
    cargo_install_binary_with_options(cargo_program, name, version, target, cache_dir, None)
}

fn cargo_install_binary_with_options(
    cargo_program: &Path,
    name: &str,
    version: &str,
    target: &str,
    cache_dir: &Path,
    source_patch: Option<&CargoSourcePatch<'_>>,
) -> Option<PathBuf> {
    fn use_cached(name: &str, cached_binary: PathBuf) -> Option<PathBuf> {
        println!("cargo:warning=Using cached {name} binary from {cached_binary:?}");
        Some(cached_binary)
    }

    let cache_name = source_patch.map_or_else(
        || format!("{name}-{version}-{target}"),
        |patch| format!("{name}-{version}-{target}-{}", patch.cache_discriminator),
    );
    let cached_binary = cache_dir.join(&cache_name);

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
    if let Some(patch) = source_patch {
        // Cargo CLI config values use TOML syntax. Canonical paths cannot be
        // relative to Cargo's changing install scratch directory; quote and
        // escape the absolute path explicitly.
        let source_dir = patch
            .source_dir
            .to_string_lossy()
            .replace('\\', "\\\\")
            .replace('"', "\\\"");
        cmd.arg("--config").arg(format!(
            "patch.crates-io.{}.path=\"{source_dir}\"",
            patch.crate_name
        ));
    }

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
    let staged = cache_dir.join(format!("{cache_name}.tmp"));
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

    #[cfg(unix)]
    #[test]
    fn source_patch_uses_content_tagged_cache_and_explicit_cargo_config() {
        let dir = tempfile::tempdir().expect("temp dir");
        let script = write_fake_cargo(
            dir.path(),
            "printf '%s\\n' \"$@\" > \"$(dirname \"$0\")/args\"\nroot=\"\"\nwhile [ $# -gt 0 ]; do\n  if [ \"$1\" = \"--root\" ]; then root=\"$2\"; shift; fi\n  shift\ndone\nmkdir -p \"$root/bin\"\nprintf patched-binary > \"$root/bin/mytool\"",
        );
        let cache = temp_cache_dir(dir.path());
        // A stock cache entry must never satisfy a source-patched request.
        std::fs::write(cache.join("mytool-1.0.0-test-target"), b"stock")
            .expect("write stock cache");
        let patch = dir.path().join("patch");
        std::fs::create_dir(&patch).expect("create patch source");
        std::fs::write(patch.join("Cargo.toml"), b"[package]\nname='patched'\n")
            .expect("write patch manifest");

        let result = cargo_install_binary_with_source_patch_using(
            &script,
            "mytool",
            "1.0.0",
            "test-target",
            &cache,
            "zenoh-link-tls",
            &patch,
            "exclusive-platform-v1",
        )
        .expect("patched install succeeds");

        assert_ne!(result, cache.join("mytool-1.0.0-test-target"));
        let file_name = result.file_name().unwrap().to_string_lossy();
        assert!(file_name.contains("exclusive-platform-v1"));
        assert_eq!(
            std::fs::read(&result).expect("read cache"),
            b"patched-binary"
        );

        let args = std::fs::read_to_string(dir.path().join("args")).expect("read cargo args");
        let canonical_patch = std::fs::canonicalize(&patch).expect("canonical patch");
        assert!(args.contains("--config\n"));
        assert!(args.contains(&format!(
            "patch.crates-io.zenoh-link-tls.path=\"{}\"",
            canonical_patch.display()
        )));
    }

    #[cfg(unix)]
    #[test]
    fn source_patch_content_change_selects_a_new_cache_entry() {
        let dir = tempfile::tempdir().expect("temp dir");
        let script = write_fake_cargo(
            dir.path(),
            "root=\"\"\nwhile [ $# -gt 0 ]; do\n  if [ \"$1\" = \"--root\" ]; then root=\"$2\"; shift; fi\n  shift\ndone\nmkdir -p \"$root/bin\"\nprintf patched-binary > \"$root/bin/mytool\"",
        );
        let cache = temp_cache_dir(dir.path());
        let patch = dir.path().join("patch");
        std::fs::create_dir(&patch).expect("create patch source");
        let source = patch.join("source.rs");
        std::fs::write(&source, b"version one").expect("write source");

        let first = cargo_install_binary_with_source_patch_using(
            &script,
            "mytool",
            "1.0.0",
            "test-target",
            &cache,
            "zenoh-link-tls",
            &patch,
            "exclusive-platform-v1",
        )
        .expect("first patched install");
        std::fs::write(&source, b"version two").expect("change source");
        let second = cargo_install_binary_with_source_patch_using(
            &script,
            "mytool",
            "1.0.0",
            "test-target",
            &cache,
            "zenoh-link-tls",
            &patch,
            "exclusive-platform-v1",
        )
        .expect("second patched install");

        assert_ne!(first, second, "source changes must invalidate the cache");
        assert!(
            first.exists(),
            "the prior immutable cache entry is retained"
        );
        assert!(second.exists(), "the new cache entry is published");
    }
}
