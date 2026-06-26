//! Generation of the zenohd router's own config file. This is the daemon's
//! deployment config (how to run the router), distinct from the client session
//! configs that live with the messaging adapter. Both are produced by the
//! shared [`crate::zenoh_config`] builder.

use super::ZenohNetProtocol;
use crate::error::{Error, Result};
use crate::zenoh_config::{TlsConfig, render_config_string, router_spec};
use std::path::PathBuf;

/// Resolves the zenohd router config path. Honors a `ZENOH_CONFIG` override;
/// otherwise renders a router config to a temp file keyed by messaging port and
/// returns its path. `tls` (when the protocol is `Tls`) carries the listener's
/// certificate/key; `None` renders a plaintext listener unchanged.
pub(crate) fn router_config_path(
    protocol: ZenohNetProtocol,
    host: &str,
    messaging_port: u16,
    tls: Option<TlsConfig>,
) -> Result<PathBuf> {
    if let Ok(config_path) = std::env::var("ZENOH_CONFIG") {
        return Ok(PathBuf::from(config_path));
    }

    let config_path = std::env::temp_dir().join(format!("zenohd_config_{}.json5", messaging_port));

    // The router seeds gossip discovery for the peer mesh, so gossip stays on;
    // multicast is off everywhere (see `crate::zenoh_config`). The router listens
    // on `host` as given (typically `0.0.0.0`) so nodes can reach it. Shares
    // `router_spec` with the out-of-process render path (`render_router_config`).
    let config_content =
        render_config_string(&router_spec(protocol, host, messaging_port, true, tls));

    std::fs::write(&config_path, config_content)
        .map_err(|e| Error::ConfigurationError(format!("Failed to write zenohd config: {}", e)))?;

    Ok(config_path)
}
