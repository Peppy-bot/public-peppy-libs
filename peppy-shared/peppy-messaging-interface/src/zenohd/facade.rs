use super::super::error::{Error, Result};
use super::{ZenohEndpoint, ZenohNetProtocol};
use std::env;
use std::fs::File;
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use zenoh::config::Config;

enum RouterOwnership {
    /// A router spawned and supervised by peppy from a rendered (or
    /// operator-pinned) config file.
    Managed {
        zenohd_path: Option<String>,
        zenohd_config_path: PathBuf,
        pinned: bool,
        zenohd_log_path: PathBuf,
    },
    /// An already-running router that peppy only probes and uses. No binary or
    /// router config belongs to peppy in this mode.
    External,
}

/// Checks whether a child process has exited prematurely.
/// Returns the process back if still alive, or an error carrying the tail of
/// the router log if it exited. zenohd's stdout/stderr are redirected to
/// `log_path`, so the diagnostic comes from the file rather than from a pipe.
fn check_process_alive(mut child: Child, log_path: &Path) -> std::result::Result<Child, Error> {
    match child.try_wait() {
        Ok(Some(status)) => Err(Error::BackendError(format!(
            "zenohd exited unexpectedly with status: {}{}",
            status,
            zenohd_log_excerpt(log_path),
        ))),
        Ok(None) => Ok(child),
        Err(e) => Err(Error::BackendError(format!(
            "Failed to check zenohd status: {}",
            e
        ))),
    }
}

/// Builds a short suffix describing why zenohd exited by reading the tail of
/// its redirected log file. Used only on the error path; falls back to the log
/// path when the file is empty or unreadable.
fn zenohd_log_excerpt(log_path: &Path) -> String {
    match std::fs::read_to_string(log_path) {
        Ok(contents) if !contents.trim().is_empty() => {
            let tail = contents
                .lines()
                .rev()
                .take(20)
                .collect::<Vec<_>>()
                .into_iter()
                .rev()
                .collect::<Vec<_>>()
                .join("\n");
            format!(" – last zenohd log lines:\n{tail}")
        }
        _ => format!(" (see zenohd log at {})", log_path.display()),
    }
}

/// The Zenoh daemon binary facade. Zenohd is not accessible via the Rust API (or in a very limited fashion).
/// This facade allows calling the binary in the background.
pub struct ZenohdFacade {
    ownership: RouterOwnership,
    adopted: bool,
    pub router_process: Option<Child>,
    pub zenoh_endpoint: ZenohEndpoint,
}

impl ZenohdFacade {
    /// Creates a facade for a peppy-managed zenohd process. The router endpoint
    /// is extracted from the exact config that will be passed to zenohd.
    pub fn managed(zenohd_config_path: impl AsRef<Path>) -> Result<Self> {
        let zenoh_endpoint = ZenohdFacade::get_endpoint_from_config(&zenohd_config_path)?;
        let zenohd_config_path = zenohd_config_path.as_ref().to_path_buf();
        // The router is operator-pinned when it runs the `ZENOH_CONFIG` file
        // verbatim (`router_config_path` returns that path unchanged). Capture it
        // once, here, when the router is built — refederate consults this instead
        // of re-reading the process-global env on every call.
        let pinned =
            crate::zenohd::config_override().as_deref() == Some(zenohd_config_path.as_path());
        // Keep the log next to the generated config (same directory), one file
        // per router port.
        let zenohd_log_path = zenohd_config_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(format!("zenohd_{}.log", zenoh_endpoint.port()));
        Ok(Self {
            ownership: RouterOwnership::Managed {
                zenohd_path: Self::get_zenohd_binary(),
                zenohd_config_path,
                pinned,
                zenohd_log_path,
            },
            adopted: false,
            router_process: None,
            zenoh_endpoint,
        })
    }

    /// Creates a facade for a router whose process and configuration are owned
    /// by the operator. The endpoint has already been validated as a dialable
    /// TCP locator by [`ZenohAdapter::with_external_router`](crate::ZenohAdapter::with_external_router).
    pub fn external(zenoh_endpoint: ZenohEndpoint) -> Self {
        Self {
            ownership: RouterOwnership::External,
            adopted: false,
            router_process: None,
            zenoh_endpoint,
        }
    }

    fn get_zenohd_binary() -> Option<String> {
        // 1) Prefer a `zenohd` binary placed next to the current executable.
        // This is important for `sudo peppy ...` where PATH may be restricted by `secure_path`.
        if let Ok(exe_path) = env::current_exe()
            && let Some(exe_dir) = exe_path.parent()
        {
            let candidate = exe_dir.join("zenohd");
            if candidate.is_file() {
                return Some(candidate.to_string_lossy().into_owned());
            }
        }

        // 2) Compile-time path injected by build script (may be stale for packaged/cargo-installed binaries).
        if let Some(path) = option_env!("ZENOHD_BINARY_PATH") {
            // If this looks like a path, verify it exists to avoid "os error 2" at spawn-time.
            if (path.contains('/') || path.contains('\\')) && !Path::new(path).is_file() {
                // Continue to other discovery mechanisms.
            } else {
                return Some(path.to_string());
            }
        }

        // 3) Fallback to searching PATH.
        if let Some(path_var) = env::var_os("PATH") {
            for dir in env::split_paths(&path_var) {
                let candidate = dir.join("zenohd");
                if candidate.is_file() {
                    return Some(candidate.to_string_lossy().into_owned());
                }
            }
        }

        None
    }

    fn connect_addr(&self) -> String {
        let connect_host = if self.zenoh_endpoint.host() == "0.0.0.0" {
            "127.0.0.1"
        } else {
            self.zenoh_endpoint.host()
        };
        format!("{connect_host}:{}", self.zenoh_endpoint.port())
    }

    fn tcp_based(&self) -> bool {
        matches!(
            self.zenoh_endpoint.protocol(),
            ZenohNetProtocol::Tcp | ZenohNetProtocol::Tls
        )
    }

    pub(crate) fn router_endpoint_in_use(&self) -> bool {
        self.tcp_based() && TcpStream::connect(self.connect_addr()).is_ok()
    }

    /// Asynchronously checks whether the configured TCP endpoint accepts a
    /// connection within `timeout`.
    ///
    /// External endpoints may require DNS resolution or route to a remote
    /// network. Using Tokio's connect future keeps that work off the async
    /// startup thread, while the outer timeout prevents an unreachable endpoint
    /// from stalling adoption indefinitely. This is intentionally only a socket
    /// reachability check; the caller performs a separate Zenoh handshake so it
    /// can distinguish an absent endpoint from a non-Zenoh service.
    pub(crate) async fn router_endpoint_reachable(&self, timeout: std::time::Duration) -> bool {
        if !self.tcp_based() {
            return false;
        }

        matches!(
            tokio::time::timeout(timeout, tokio::net::TcpStream::connect(self.connect_addr()))
                .await,
            Ok(Ok(_))
        )
    }

    pub fn is_adopted(&self) -> bool {
        self.adopted
    }

    pub(crate) fn is_external(&self) -> bool {
        matches!(&self.ownership, RouterOwnership::External)
    }

    pub(crate) fn is_pinned(&self) -> bool {
        matches!(
            &self.ownership,
            RouterOwnership::Managed { pinned: true, .. }
        )
    }

    pub(crate) fn managed_config_path(&self) -> Option<&Path> {
        match &self.ownership {
            RouterOwnership::Managed {
                zenohd_config_path, ..
            } => Some(zenohd_config_path),
            RouterOwnership::External => None,
        }
    }

    pub(crate) fn adopt_external_router(&mut self) {
        debug_assert!(self.is_external());
        self.adopted = true;
        tracing::info!(
            "Adopted external zenoh router at {}; peppy will not manage its lifecycle",
            self.zenoh_endpoint
        );
    }

    fn get_endpoint_from_config(zenohd_config_path: impl AsRef<Path>) -> Result<ZenohEndpoint> {
        let config = Config::from_file(zenohd_config_path).map_err(|e| {
            Error::ConfigurationError(format!("Failed to load zenoh config file: {}", e))
        })?;
        let listen_json = config.get_json("listen").map_err(|e| {
            Error::ConfigurationError(format!("Failed to get listen config: {}", e))
        })?;

        let listen: serde_json::Value = serde_json::from_str(&listen_json).map_err(|e| {
            Error::ConfigurationError(format!("Failed to parse listen config: {}", e))
        })?;

        let endpoint_str = listen["endpoints"]["router"][0].as_str().ok_or_else(|| {
            Error::ConfigurationError("No router endpoint found in config".to_string())
        })?;

        endpoint_str.parse()
    }

    /// Starts a zenohd process, using std::process::Command is the recommended way as using the
    /// rust crate directly prevents the user from using plugins/adminspace
    pub fn start_router(&mut self) -> Result<()> {
        let RouterOwnership::Managed {
            zenohd_path,
            zenohd_config_path,
            zenohd_log_path,
            ..
        } = &self.ownership
        else {
            return Err(Error::BackendError(
                "cannot spawn an operator-managed external Zenoh router".to_string(),
            ));
        };
        let zenohd_path = zenohd_path.as_ref().ok_or_else(|| {
            Error::ZenohdError(
                "Zenohd binary not found. Install `zenohd` next to the `peppy` binary or make it available on PATH."
                    .to_string(),
            )
        })?;

        // `Tls` is TLS-over-TCP, so the plain TCP probes below apply to it too:
        // the listening socket accepts TCP before the TLS handshake, which is all
        // a "port already bound / accepting yet?" check needs.
        let tcp_based = self.tcp_based();
        let connect_addr = self.connect_addr();

        if self.router_endpoint_in_use() {
            return Err(Error::BackendError(format!(
                "Zenoh router port already in use: {}",
                connect_addr
            )));
        }

        // Redirect stdout+stderr to a log file instead of unread pipes: a full
        // pipe buffer blocks a zenohd thread in `write` and deadlocks the whole
        // router. Pin the log level too, so a verbose inherited `RUST_LOG`
        // can't flood the file (override with `PEPPY_ZENOHD_LOG`).
        let log_file = File::create(zenohd_log_path).map_err(|e| {
            Error::BackendError(format!(
                "Failed to create zenohd log file {}: {}",
                zenohd_log_path.display(),
                e
            ))
        })?;
        let stderr_file = log_file
            .try_clone()
            .map_err(|e| Error::BackendError(format!("Failed to set up zenohd log file: {}", e)))?;
        let zenohd_log_level =
            env::var("PEPPY_ZENOHD_LOG").unwrap_or_else(|_| "zenoh=warn".to_string());

        let mut child = Command::new(zenohd_path)
            .env("ZENOH_CONFIG", zenohd_config_path.as_os_str())
            .env("RUST_LOG", zenohd_log_level)
            .arg("-c")
            .arg(zenohd_config_path)
            .stdout(Stdio::from(log_file))
            .stderr(Stdio::from(stderr_file))
            .spawn()
            .map_err(|e| Error::BackendError(format!("Failed to start zenohd: {}", e)))?;

        if tcp_based {
            tracing::info!(
                "Waiting for Zenoh router to accept connections at {}://{}",
                self.zenoh_endpoint.protocol(),
                connect_addr
            );
            let start = std::time::Instant::now();
            let timeout = std::time::Duration::from_secs(30);
            let mut backoff = std::time::Duration::from_millis(10);
            let max_backoff = std::time::Duration::from_millis(500);

            loop {
                child = check_process_alive(child, zenohd_log_path)?;

                match TcpStream::connect(&connect_addr) {
                    Ok(_) => break,
                    Err(_) if start.elapsed() >= timeout => {
                        return Err(Error::BackendError(format!(
                            "zenohd readiness timeout after {}s (TCP {})",
                            timeout.as_secs(),
                            connect_addr
                        )));
                    }
                    Err(_) => {
                        std::thread::sleep(backoff);
                        backoff = (backoff * 2).min(max_backoff);
                    }
                }
            }
        } else {
            child = check_process_alive(child, zenohd_log_path)?;
        }

        tracing::info!(
            "Zenoh router started (config {}, logs {})",
            zenohd_config_path.display(),
            zenohd_log_path.display()
        );

        // Store the child process handle
        self.router_process = Some(child);

        Ok(())
    }

    pub fn stop_router(&mut self) -> Result<()> {
        if self.is_external() {
            tracing::info!("leaving operator-managed external zenoh router running");
            return Ok(());
        }

        // Terminate the zenohd router process if it's running
        if let Some(mut child) = self.router_process.take() {
            // Try to kill the process gracefully
            if let Err(e) = child.kill() {
                tracing::warn!("Failed to terminate zenohd router process: {}", e);
            } else {
                tracing::info!("Zenohd router process terminated");
            }

            // Wait for the process to actually exit and log any error
            if let Err(e) = child.wait() {
                tracing::warn!("Error waiting for zenohd process to exit: {}", e);
            }
        }

        Ok(())
    }
}

impl Drop for ZenohdFacade {
    fn drop(&mut self) {
        if let Err(e) = self.stop_router() {
            tracing::warn!("Failed to stop router during drop: {}", e);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zenohd_facade_creation_with_config() {
        use std::io::Write;
        use tempfile::Builder;

        let expected_host = "127.0.0.1";
        let expected_port = 7447u16;

        // Create a minimal zenoh config file with .json5 extension
        let mut config_file = Builder::new()
            .suffix(".json5")
            .tempfile()
            .expect("Failed to create temp file");
        writeln!(
            config_file,
            r#"{{
                "listen": {{
                    "endpoints": {{
                        "router": ["tcp/{expected_host}:{expected_port}"]
                    }}
                }}
            }}"#
        )
        .expect("Failed to write config");

        let facade = ZenohdFacade::managed(config_file.path());
        assert!(facade.is_ok(), "Error creating facade: {:?}", facade.err());

        let facade = facade.unwrap();
        assert_eq!(facade.zenoh_endpoint.host(), expected_host);
        assert_eq!(facade.zenoh_endpoint.port(), expected_port);
        assert_eq!(facade.zenoh_endpoint.protocol(), ZenohNetProtocol::Tcp);

        // The log file sits next to the config, named per router port.
        let RouterOwnership::Managed {
            zenohd_log_path, ..
        } = &facade.ownership
        else {
            panic!("expected a managed router facade");
        };
        let expected_log_path = config_file
            .path()
            .parent()
            .unwrap()
            .join(format!("zenohd_{expected_port}.log"));
        assert_eq!(zenohd_log_path, &expected_log_path);
    }

    #[test]
    fn test_stop_router_multiple_times() {
        use std::io::Write;
        use tempfile::Builder;

        // Create a config file with .json5 extension
        let mut config_file = Builder::new()
            .suffix(".json5")
            .tempfile()
            .expect("Failed to create temp file");
        writeln!(
            config_file,
            r#"{{
                "listen": {{
                    "endpoints": {{
                        "router": ["tcp/127.0.0.1:7447"]
                    }}
                }}
            }}"#
        )
        .expect("Failed to write config");

        let mut facade =
            ZenohdFacade::managed(config_file.path()).expect("Failed to create facade");

        // First stop should succeed (no process to stop)
        assert!(facade.stop_router().is_ok());

        // Second stop should also succeed (idempotent)
        assert!(facade.stop_router().is_ok());

        // Process should remain None
        assert!(facade.router_process.is_none());
    }

    #[test]
    fn external_router_has_no_binary_or_config_path() {
        let endpoint = "tcp/zenoh-router.internal:17447"
            .parse()
            .expect("parse external endpoint");
        let facade = ZenohdFacade::external(endpoint);

        assert!(facade.is_external());
        assert_eq!(
            facade.zenoh_endpoint.to_string(),
            "tcp/zenoh-router.internal:17447"
        );
        assert!(facade.managed_config_path().is_none());
        assert!(!facade.is_adopted());
        assert!(facade.router_process.is_none());
    }

    #[test]
    fn stop_router_is_an_idempotent_no_op_for_an_adopted_router() {
        let endpoint = "tcp/127.0.0.1:7447"
            .parse()
            .expect("parse external endpoint");
        let mut facade = ZenohdFacade::external(endpoint);
        facade.adopt_external_router();

        assert!(facade.is_adopted());
        assert!(facade.stop_router().is_ok());
        assert!(facade.stop_router().is_ok());
        assert!(facade.router_process.is_none());
    }

    #[tokio::test]
    async fn external_reachability_probe_distinguishes_listening_and_closed_endpoints() {
        let listener =
            std::net::TcpListener::bind(("127.0.0.1", 0)).expect("bind external endpoint");
        let port = listener.local_addr().expect("read listener address").port();
        let endpoint = format!("tcp/127.0.0.1:{port}")
            .parse()
            .expect("parse external endpoint");
        let facade = ZenohdFacade::external(endpoint);

        assert!(
            facade
                .router_endpoint_reachable(std::time::Duration::from_secs(1))
                .await,
            "a listening TCP endpoint must be reachable"
        );

        drop(listener);
        assert!(
            !facade
                .router_endpoint_reachable(std::time::Duration::from_secs(1))
                .await,
            "a closed TCP endpoint must not be reachable"
        );
    }

    #[test]
    fn test_zenohd_log_excerpt() {
        use std::io::Write;
        use tempfile::Builder;

        // Missing file: fall back to pointing at the path. Build it inside a
        // temp dir (without creating the file) so the test is portable.
        let dir = tempfile::tempdir().expect("Failed to create temp dir");
        let missing = dir.path().join("zenohd_0.log");
        assert!(zenohd_log_excerpt(&missing).contains("see zenohd log"));

        // Present file: include the tail of its contents.
        let mut log = Builder::new()
            .suffix(".log")
            .tempfile()
            .expect("Failed to create temp log");
        writeln!(log, "starting up\nerror: address already in use").expect("write");
        let excerpt = zenohd_log_excerpt(log.path());
        assert!(excerpt.contains("address already in use"));
    }
}
