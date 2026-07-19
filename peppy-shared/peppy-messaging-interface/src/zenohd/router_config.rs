//! Generation of the zenohd router's own config file. This is the daemon's
//! deployment config (how to run the router), distinct from the client session
//! configs that live with the messaging adapter. Both are produced by the
//! shared [`crate::zenoh_config`] builder.

use super::ZenohNetProtocol;
use crate::error::{Error, Result};
use crate::zenoh_config::{RouterLinks, render_router_config};
use serde_json::json;
use std::path::{Path, PathBuf};

/// Returns the identity a Peppy-managed router must keep across process
/// restarts.
///
/// Zenoh assigns a random ZID when a config omits `id`. That is unsafe for the
/// managed-router reload path: zenohd is necessarily killed to reload rotated
/// TLS files, and a replacement with a different ZID can leave the upstream
/// routing graph holding declarations from the old router until it converges.
/// A core-node presence token dropped through the replacement cannot withdraw
/// that old-ZID declaration. Persisting the generated ZID in the same managed
/// config makes every subsequent rewrite/restart the same logical router.
fn managed_router_id(config_path: &Path) -> String {
    std::fs::read_to_string(config_path)
        .ok()
        .and_then(|contents| serde_json::from_str::<serde_json::Value>(&contents).ok())
        .and_then(|config| config.get("id")?.as_str().map(str::to_owned))
        // Refuse to preserve arbitrary text from a damaged/stale file. The
        // freshly generated replacement below is always a valid Zenoh ID.
        .filter(|id| id.parse::<zenoh::config::ZenohId>().is_ok())
        .unwrap_or_else(|| zenoh::config::ZenohId::default().to_string())
}

/// Resolves the zenohd router config path. Honors a `ZENOH_CONFIG` override;
/// otherwise renders a router config to a temp file keyed by messaging port and
/// returns its path. `gossip` sets the router's gossip scouting (on for the
/// logged-out peer mesh, off when sessions relay through the router or the
/// router is federated). `links` carries the platform upstream and TLS
/// material; see [`RouterLinks`]. `RouterLinks::default()` renders a standalone
/// plaintext listener unchanged.
pub(crate) fn router_config_path(
    protocol: ZenohNetProtocol,
    host: &str,
    messaging_port: u16,
    gossip: bool,
    links: RouterLinks,
) -> Result<PathBuf> {
    if let Some(config_path) = config_override() {
        return Ok(config_path);
    }

    let config_path = std::env::temp_dir().join(format!("zenohd_config_{}.json5", messaging_port));
    render_router_config_to_path(&config_path, protocol, host, messaging_port, gossip, links)?;
    Ok(config_path)
}

/// Renders the router config and writes it to `config_path`, *bypassing* the
/// `ZENOH_CONFIG` override resolution that [`router_config_path`] does. The
/// refederation path ([`crate::ZenohAdapter::refederate`]) uses this to rewrite
/// the file captured by [`ZenohdFacade::managed`](super::ZenohdFacade::managed) in place:
/// going back through `router_config_path` would re-read the process-global
/// `ZENOH_CONFIG`, which — if it changed after startup — could redirect the write
/// to a different path or skip it entirely (the override early-return), leaving
/// the running router's actual config file stale. (The operator-pinned case is
/// already filtered out by `facade.is_pinned()` before this is reached.)
pub(crate) fn render_router_config_to_path(
    config_path: &Path,
    protocol: ZenohNetProtocol,
    host: &str,
    messaging_port: u16,
    gossip: bool,
    links: RouterLinks,
) -> Result<()> {
    // `gossip` follows the session topology bit: on only for the logged-out
    // peer mesh the router seeds; off when sessions are router-relayed or the
    // router holds a platform upstream (nothing consumes gossip then, and it
    // would only advertise locators over the federation link). Multicast is
    // off everywhere (see `crate::zenoh_config`). The router listens on `host`
    // as given (typically `0.0.0.0`) so nodes can reach it.
    //
    // `render_router_config` (shared with the out-of-process render path) also
    // validates the rendered config, which matters most right here: the
    // refederation path rewrites the running router's only config file in
    // place, and a malformed locator must fail before it clobbers the
    // known-good file and surfaces at the next restart.
    let rendered = render_router_config(protocol, host, messaging_port, gossip, links)?;
    let mut config: serde_json::Value = serde_json::from_str(&rendered).map_err(|error| {
        Error::ConfigurationError(format!(
            "Failed to parse Peppy's rendered zenohd config: {error}"
        ))
    })?;
    config["id"] = json!(managed_router_id(config_path));
    let config_content = serde_json::to_string(&config).map_err(|error| {
        Error::ConfigurationError(format!(
            "Failed to serialize Peppy's managed zenohd config: {error}"
        ))
    })?;
    // Validate after adding the persistent ID as well. This guards both the
    // generated ID and the preservation path before replacing a runnable file.
    zenoh::config::Config::from_json5(&config_content).map_err(|error| {
        Error::ConfigurationError(format!(
            "rendered managed zenohd config is invalid: {error}"
        ))
    })?;

    std::fs::write(config_path, config_content)
        .map_err(|e| Error::ConfigurationError(format!("Failed to write zenohd config: {}", e)))?;

    Ok(())
}

/// The operator-pinned router config path from `ZENOH_CONFIG`, if set. When
/// present, [`router_config_path`] returns it untouched — we never render our own
/// config over an operator-owned one. Callers that *re-render* to apply a change
/// (e.g. [`crate::ZenohAdapter::refederate`]) consult this to detect that the
/// re-render would be a no-op, so they can skip the work it would otherwise
/// trigger (a pointless zenohd restart). This is the single source of truth for
/// the override so the two call sites cannot drift.
pub(crate) fn config_override() -> Option<PathBuf> {
    // A blank or whitespace-only `ZENOH_CONFIG` is treated as unset (not an empty
    // path), so startup falls back to the rendered temp config instead of trying
    // to read `""`. Mirrors `ZenohAdapter::resolve_session_config_override`.
    std::env::var("ZENOH_CONFIG")
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
}
