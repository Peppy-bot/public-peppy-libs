//! Generation of Zenoh session and router configs.
//!
//! This is the single source of truth that replaced the former set of per-shape
//! Askama templates (one each for the fail-fast client, the reconnecting client,
//! the watchdog probe, and the router). A typed [`ZenohConfigSpec`] keeps the
//! mode / connect / listen / scouting / timestamping settings in one place so
//! adding a knob touches one struct instead of four string templates.
//!
//! ## Discovery model: gossip-only, multicast off
//!
//! Every generated config disables multicast scouting and relies on gossip
//! seeded by the configured connect endpoints (the router). Nodes open a `peer`
//! session that connects to the router, learn each other's locators via gossip,
//! and then form direct peer-to-peer links so data no longer relays through the
//! router. Multicast is left off on purpose: on a shared host it bridges
//! otherwise-independent peer groups (and would cross-link unrelated test runs),
//! and with a known seed it adds nothing gossip does not already cover.

use crate::zenohd::ZenohNetProtocol;
use config::namespace::Namespace;
use serde_json::json;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use tokio_rustls::rustls::SignatureScheme;
use tokio_rustls::rustls::client::ResolvesClientCert;
use tokio_rustls::rustls::sign::CertifiedKey;

/// TLS material for a `tls/` (or `quic/`) session, rendered into the zenoh
/// `transport.link.tls` block. One type serves both roles: a router/listener
/// sets `listen_certificate`/`listen_private_key` (its server identity); a
/// client sets `root_ca_certificate` (to verify that server) and
/// `verify_name_on_connect`. mTLS additionally uses `connect_certificate`/
/// `connect_private_key` (client identity) with `enable_mtls`. The keys map
/// 1:1 to zenoh 1.9's `transport.link.tls.*` (verified against its
/// `DEFAULT_CONFIG.json5`); unset path fields are omitted so a non-TLS config
/// renders byte-identical to before.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TlsConfig {
    /// CA used to validate the peer's certificate. Required on the client side
    /// for a private CA (dev/internal); omit to fall back to system WebPKI.
    pub root_ca_certificate: Option<PathBuf>,
    /// Listener (router/server) certificate chain.
    pub listen_certificate: Option<PathBuf>,
    /// Listener (router/server) private key.
    pub listen_private_key: Option<PathBuf>,
    /// Connecting (client) certificate — only for mTLS.
    pub connect_certificate: Option<PathBuf>,
    /// Connecting (client) private key — only for mTLS.
    pub connect_private_key: Option<PathBuf>,
    /// Require/verify a client certificate (mutual TLS).
    pub enable_mtls: bool,
    /// Verify the server cert's name matches the dialed host (client side).
    /// zenoh defaults this to `true`; keep it on unless a test needs otherwise.
    pub verify_name_on_connect: bool,
}

impl Default for TlsConfig {
    fn default() -> Self {
        Self {
            root_ca_certificate: None,
            listen_certificate: None,
            listen_private_key: None,
            connect_certificate: None,
            connect_private_key: None,
            enable_mtls: false,
            verify_name_on_connect: true,
        }
    }
}

/// Reads a PEM file fully, labeling any error with `what` and the path.
fn read_pem_file(path: &Path, what: &str) -> Result<Vec<u8>, String> {
    std::fs::read(path).map_err(|e| format!("read {what} `{}` failed: {e}", path.display()))
}

/// Parses every certificate from the PEM file at `path`. Errors if the file is
/// unreadable, any block fails to parse, or no certificate is present.
fn read_pem_certs(
    path: &Path,
    what: &str,
) -> Result<Vec<tokio_rustls::rustls::pki_types::CertificateDer<'static>>, String> {
    let bytes = read_pem_file(path, what)?;
    let certs = rustls_pemfile::certs(&mut &bytes[..])
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| format!("parse {what} `{}` failed: {e}", path.display()))?;
    if certs.is_empty() {
        return Err(format!(
            "{what} `{}` contained no certificates",
            path.display()
        ));
    }
    Ok(certs)
}

/// A [`ResolvesClientCert`] wrapper that records whether the server requested a
/// client certificate. rustls consults the resolver exactly when a
/// `CertificateRequest` arrives — whether or not an identity is configured — so
/// after the handshake the flag tells [`probe_tls_reachable`] whether a
/// post-handshake client-auth verdict can still be in flight.
#[derive(Debug)]
struct ClientAuthObserver {
    inner: Arc<dyn ResolvesClientCert>,
    client_auth_requested: Arc<AtomicBool>,
}

impl ResolvesClientCert for ClientAuthObserver {
    fn resolve(
        &self,
        root_hint_subjects: &[&[u8]],
        sigschemes: &[SignatureScheme],
    ) -> Option<Arc<CertifiedKey>> {
        self.client_auth_requested.store(true, Ordering::Relaxed);
        self.inner.resolve(root_hint_subjects, sigschemes)
    }

    fn only_raw_public_keys(&self) -> bool {
        self.inner.only_raw_public_keys()
    }

    fn has_certs(&self) -> bool {
        self.inner.has_certs()
    }
}

/// Builds the rustls client config for [`probe_tls_reachable`], plus the flag
/// its [`ClientAuthObserver`] sets when the server engages client-cert auth.
fn build_probe_client_config(
    tls: &TlsConfig,
) -> Result<(tokio_rustls::rustls::ClientConfig, Arc<AtomicBool>), String> {
    use tokio_rustls::rustls::{ClientConfig, RootCertStore};

    // Build the trust anchors: either an explicit private CA, or the OS roots.
    let mut roots = RootCertStore::empty();
    match &tls.root_ca_certificate {
        Some(path) => {
            for cert in read_pem_certs(path, "root CA")? {
                roots
                    .add(cert)
                    .map_err(|e| format!("add root CA `{}` failed: {e}", path.display()))?;
            }
        }
        None => {
            // rustls-native-certs 0.8 returns a `CertificateResult`; take all of
            // its `.certs` (any per-cert load errors are non-fatal as long as at
            // least one root ends up trusted).
            let native = rustls_native_certs::load_native_certs();
            for cert in native.certs {
                let _ = roots.add(cert);
            }
            if roots.is_empty() {
                return Err(
                    "system trust store is empty (no native roots could be loaded)".to_string(),
                );
            }
        }
    }

    // Select the crypto backend explicitly rather than relying on rustls's
    // process-default provider. In this dependency tree rustls is built with BOTH
    // its `ring` and `aws-lc-rs` features enabled (zenoh's stack pulls `aws-lc-rs`
    // via tokio-rustls's default, while quinn/tls-listener pull `ring`), so rustls
    // cannot auto-determine a single default provider and a bare
    // `ClientConfig::builder()` would panic. tokio-rustls's default features
    // guarantee `ring` is available, so we name it directly.
    let config_builder = ClientConfig::builder_with_provider(Arc::new(
        tokio_rustls::rustls::crypto::ring::default_provider(),
    ))
    .with_safe_default_protocol_versions()
    .map_err(|e| format!("TLS provider setup failed: {e}"))?
    .with_root_certificates(roots);

    let mut config = if tls.enable_mtls {
        let certificate_path = tls.connect_certificate.as_ref().ok_or_else(|| {
            "mTLS probe requires a connect certificate (`connect_certificate`)".to_string()
        })?;
        let private_key_path = tls.connect_private_key.as_ref().ok_or_else(|| {
            "mTLS probe requires a connect private key (`connect_private_key`)".to_string()
        })?;

        let certificate_chain = read_pem_certs(certificate_path, "connect certificate")?;

        let private_key_bytes = read_pem_file(private_key_path, "connect private key")?;
        let private_key = rustls_pemfile::private_key(&mut &private_key_bytes[..])
            .map_err(|e| {
                format!(
                    "parse connect private key `{}` failed: {e}",
                    private_key_path.display()
                )
            })?
            .ok_or_else(|| {
                format!(
                    "connect private key `{}` contained no private key",
                    private_key_path.display()
                )
            })?;

        config_builder
            .with_client_auth_cert(certificate_chain, private_key)
            .map_err(|e| format!("configure mTLS client identity failed: {e}"))?
    } else {
        config_builder.with_no_client_auth()
    };

    let client_auth_requested = Arc::new(AtomicBool::new(false));
    config.client_auth_cert_resolver = Arc::new(ClientAuthObserver {
        inner: config.client_auth_cert_resolver.clone(),
        client_auth_requested: client_auth_requested.clone(),
    });
    Ok((config, client_auth_requested))
}

/// Verify that a TLS endpoint at `host:port` is reachable AND that its
/// certificate actually validates against the trust configured in `tls`,
/// completing a real TLS handshake within `timeout` total. The TCP connect, TLS
/// handshake, and short post-handshake rejection check share one deadline, so
/// the whole call is bounded by `timeout` (not `timeout` per phase). The caller
/// relies on this single bound to keep the probe inside its federation ack
/// budget. The rejection check runs only when the server engaged
/// client-certificate auth; a plain-TLS handshake returns as soon as it
/// validates.
///
/// ## Why a raw handshake and not `zenoh::open`
///
/// In zenoh *client* mode `zenoh::open` returns `Ok` as soon as the local
/// session is created, even if the configured connect endpoint cannot be
/// reached or its certificate cannot be validated. The link failure is
/// asynchronous and silent (no data ever flows, but nothing errors), so a
/// successful `open` is NOT a usable "the federation link validates" signal.
/// The only deterministic way to know the link is good is to perform the TLS
/// handshake ourselves and observe the result. This probe does exactly that
/// with rustls directly (no zenoh session involved): a TCP connect followed by
/// a full TLS handshake that trusts the configured roots and verifies the
/// server name. `Ok(())` is returned iff the chain is trusted and the server
/// name matches; any other outcome returns a human-readable `Err` naming
/// `host:port` (it is surfaced into a user-facing CLI error).
///
/// Name verification is intentionally left ON (rustls's default) regardless of
/// `tls.verify_name_on_connect`: this probe is the authoritative "does the link
/// validate" check, so it always validates fully.
pub async fn probe_tls_reachable(
    host: &str,
    port: u16,
    tls: &TlsConfig,
    timeout: std::time::Duration,
) -> Result<(), String> {
    use tokio::io::AsyncReadExt;
    use tokio_rustls::TlsConnector;
    use tokio_rustls::rustls::pki_types::ServerName;

    let host = crate::zenohd::unbracket(host);

    // Certificate files and native-root discovery are synchronous operations.
    // Keep them off async workers and include them in the same total deadline as
    // the network phases, so a slow mounted identity cannot wedge federation.
    // The material load and the TCP dial are independent, so they run
    // concurrently: the deadline pays max(load, connect), not their sum.
    let deadline = tokio::time::Instant::now() + timeout;
    let tls = tls.clone();
    let load_material = async {
        tokio::time::timeout_at(
            deadline,
            tokio::task::spawn_blocking(move || build_probe_client_config(&tls)),
        )
        .await
        .map_err(|_| format!("loading TLS material for {host}:{port} timed out after {timeout:?}"))?
        .map_err(|error| format!("TLS material task failed: {error}"))?
    };
    let dial = async {
        match tokio::time::timeout_at(deadline, tokio::net::TcpStream::connect((host, port))).await
        {
            Err(_) => Err(format!(
                "connect to {host}:{port} timed out after {timeout:?}"
            )),
            Ok(Err(e)) => Err(format!("connect to {host}:{port} failed: {e}")),
            Ok(Ok(tcp)) => Ok(tcp),
        }
    };
    let ((config, client_auth_requested), tcp) = tokio::try_join!(load_material, dial)?;

    let server_name = ServerName::try_from(host.to_string())
        .map_err(|e| format!("invalid server name `{host}`: {e}"))?;

    // Finish the handshake under the same deadline. tokio-rustls surfaces
    // validation failures (UnknownIssuer / unknown CA / bad server name) as an
    // io::Error whose message contains the rustls reason, so `{e}` carries the
    // cause.
    let connector = TlsConnector::from(Arc::new(config));
    let mut stream =
        match tokio::time::timeout_at(deadline, connector.connect(server_name, tcp)).await {
            Err(_) => {
                return Err(format!(
                    "TLS handshake to {host}:{port} timed out after {timeout:?}"
                ));
            }
            Ok(Err(e)) => return Err(format!("TLS handshake to {host}:{port} failed: {e}")),
            Ok(Ok(stream)) => stream,
        };

    // In TLS 1.3 the server's client-certificate verdict lands only after the
    // client has already finished its side of the handshake: a missing identity
    // draws `certificate_required` and an untrusted one `bad certificate`, both
    // as post-Finished alerts. Only a server that engaged client auth (it sent a
    // CertificateRequest, observed via the wrapped cert resolver) can still have
    // such a verdict in flight; for any other server the handshake outcome above
    // is final, so return without taxing every healthy probe with the wait.
    if !client_auth_requested.load(Ordering::Relaxed) {
        return Ok(());
    }

    // Give the pending verdict a short bounded window to arrive. No application
    // data is expected from a healthy Zenoh listener, so reaching the grace
    // deadline is success.
    const POST_HANDSHAKE_GRACE: std::time::Duration = std::time::Duration::from_millis(250);
    let full_grace_deadline = tokio::time::Instant::now() + POST_HANDSHAKE_GRACE;
    let grace_was_clipped = full_grace_deadline > deadline;
    let grace_deadline = std::cmp::min(deadline, full_grace_deadline);
    let mut byte = [0u8; 1];
    match tokio::time::timeout_at(grace_deadline, stream.read(&mut byte)).await {
        Err(_) if grace_was_clipped => Err(format!(
            "TLS post-handshake check to {host}:{port} timed out after {timeout:?}"
        )),
        Err(_) => Ok(()),
        Ok(Ok(0)) => Err(format!(
            "TLS peer {host}:{port} closed the connection immediately after the handshake"
        )),
        Ok(Ok(_)) => Ok(()),
        Ok(Err(e)) => Err(format!(
            "TLS peer {host}:{port} rejected the connection after the handshake: {e}"
        )),
    }
}

impl TlsConfig {
    /// Server (router/listener) identity: a leaf certificate chain + its key.
    pub fn server(certificate: PathBuf, private_key: PathBuf) -> Self {
        Self {
            listen_certificate: Some(certificate),
            listen_private_key: Some(private_key),
            ..Self::default()
        }
    }

    /// Client trust: the CA that signed the router's certificate. Name
    /// verification stays on.
    pub fn client(root_ca_certificate: PathBuf) -> Self {
        Self {
            root_ca_certificate: Some(root_ca_certificate),
            ..Self::default()
        }
    }

    /// Mutual-TLS client: [`client`](Self::client) trust in `root_ca_certificate`
    /// plus a `certificate`/`private_key` identity presented to a listener that
    /// requires client certificates.
    pub fn mtls_client(
        root_ca_certificate: PathBuf,
        certificate: PathBuf,
        private_key: PathBuf,
    ) -> Self {
        Self {
            connect_certificate: Some(certificate),
            connect_private_key: Some(private_key),
            enable_mtls: true,
            ..Self::client(root_ca_certificate)
        }
    }
}

/// How a router wires into the platform federation beyond its primary
/// listener: at most one upstream router it dials, plus the TLS material for
/// that link and/or its own listener. One value carried through every
/// router-config entry point ([`crate::ZenohAdapter::with_router`],
/// [`refederate`](crate::ZenohAdapter::refederate), [`render_router_config`]).
/// `Default` is the standalone plaintext router.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RouterLinks {
    /// The single upstream router this router *federates* to (dials),
    /// `<proto>/<host>:<port>`, optionally carrying per-endpoint `#key=val;...`
    /// config fragments. `None` is a standalone router. `Some` turns on
    /// reconnect/keep-alive so an unreachable or restarted upstream is
    /// recovered transparently and never stops the router serving its own
    /// nodes.
    pub upstream: Option<String>,
    /// TLS material for the links: the listener certificate/key
    /// ([`TlsConfig::server`]) when the primary listener speaks `tls/`, and/or
    /// the connect-side trust root ([`TlsConfig::client`]) for a `tls/`
    /// upstream. Ignored by plaintext endpoints.
    pub tls: Option<TlsConfig>,
}

/// The Zenoh roles this codebase generates configs for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SessionMode {
    /// A node/daemon session: connects to a seed (the router), then forms direct
    /// peer-to-peer links with peers discovered via gossip.
    Peer,
    /// A short-lived liveness probe: connects to exactly one router endpoint and
    /// never discovers peers, so "is *our* router up?" stays deterministic.
    Client,
    /// The zenohd router itself.
    Router,
}

/// Inputs for [`build_zenoh_config`].
pub(crate) struct ZenohConfigSpec {
    pub mode: SessionMode,
    /// Seed endpoints to dial (`<proto>/<host>:<port>`). Empty for the router.
    pub connect_endpoints: Vec<String>,
    /// Endpoints to listen on. Empty leaves Zenoh's default (clients do not
    /// listen). Peers listen on a loopback ephemeral port by default.
    pub listen_endpoints: Vec<String>,
    /// Retry the connection forever (`timeout_ms: -1`, `exit_on_failure: false`)
    /// instead of failing fast. Used by long-lived sessions so a router restart
    /// is recovered transparently.
    pub reconnect: bool,
    /// Enable gossip scouting so peers sharing a seed discover each other and
    /// form direct links. Multicast scouting is always off (see module docs).
    pub gossip: bool,
    /// TLS material for a `tls/` endpoint. `None` for plaintext transports
    /// (`tcp/`, …), in which case no `transport.link.tls` block is rendered and
    /// the output is byte-identical to a pre-TLS config.
    pub tls: Option<TlsConfig>,
    /// Workspace namespace for an application session (routing isolation).
    /// Rendered into the session-level `namespace` field for `Peer`/`Client`
    /// sessions only; `None` for the router and for liveness probes. Zenoh
    /// prepends `<ns>/` to every declared key on egress and strips it on
    /// ingress, so two sessions interoperate iff their namespaces match.
    pub namespace: Option<Namespace>,
}

/// Builds the JSON5-equivalent config value for a session or the router.
pub(crate) fn build_zenoh_config(spec: &ZenohConfigSpec) -> serde_json::Value {
    let mode = match spec.mode {
        SessionMode::Peer => "peer",
        SessionMode::Client => "client",
        SessionMode::Router => "router",
    };

    let mut config = json!({
        "mode": mode,
        "scouting": {
            "multicast": { "enabled": false },
            "gossip": { "enabled": spec.gossip }
        }
    });

    if !spec.connect_endpoints.is_empty() {
        let mut connect = json!({ "endpoints": spec.connect_endpoints });
        if spec.reconnect {
            connect["timeout_ms"] = json!(-1);
            connect["exit_on_failure"] = json!(false);
            connect["retry"] = json!({
                "period_init_ms": 1000,
                "period_max_ms": 4000,
                "period_increase_factor": 2.0
            });
        }
        config["connect"] = connect;
    }

    if !spec.listen_endpoints.is_empty() {
        // Zenoh reads `listen.endpoints` against the session's OWN mode, so the
        // key must match `mode` above — the same per-role-map rule as
        // `timestamping` below. Deriving it exhaustively (rather than
        // `else => "peer"`) keeps a client's endpoints from being silently
        // misfiled under `peer` if one ever listens.
        let role = match spec.mode {
            SessionMode::Router => "router",
            SessionMode::Peer => "peer",
            SessionMode::Client => "client",
        };
        config["listen"] = json!({ "endpoints": { role: spec.listen_endpoints } });
    }

    // Stamp data at the producer so consumers can measure real delivery latency.
    // Zenoh matches `enabled` against the session's OWN mode, so the key must
    // match `mode` above — a `peer` session ignores an `enabled.client` entry.
    // Stamping at the source keeps peer mode (no router in the direct path) on
    // par with router mode, where the client/router already stamps.
    config["timestamping"] = match spec.mode {
        SessionMode::Router => {
            json!({ "enabled": { "router": true }, "drop_future_timestamp": false })
        }
        SessionMode::Peer => json!({ "enabled": { "peer": true }, "drop_future_timestamp": false }),
        SessionMode::Client => {
            json!({ "enabled": { "client": true }, "drop_future_timestamp": false })
        }
    };

    // Session namespace (workspace routing isolation). zenoh's `namespace` is a
    // session-level field: it is applied to the application session opened
    // against a router, where egress prepends `<ns>/` to every declared key and
    // ingress strips it — so two sessions interoperate iff their namespaces
    // match. It is rendered only for `Peer`/`Client` application sessions, never
    // for the router: a router only forwards between transport faces and never
    // opens an application session, so a router-level `namespace` would NOT
    // prefix forwarded/federated traffic. `router_spec` and `render_probe_config`
    // therefore pass `namespace: None` — readiness probes are deliberately
    // namespace-free (they only check "is our router up?").
    if let Some(namespace) = &spec.namespace
        && !matches!(spec.mode, SessionMode::Router)
    {
        config["namespace"] = json!(namespace.as_str());
    }

    // TLS material maps to zenoh's `transport.link.tls`. Rendered only when
    // present so a plaintext config is byte-identical to before. Path fields are
    // omitted when unset (zenoh treats a missing key as `null`); the two booleans
    // always render (they have no "absent" meaning).
    if let Some(tls) = &spec.tls {
        let mut tls_json = json!({
            "enable_mtls": tls.enable_mtls,
            "verify_name_on_connect": tls.verify_name_on_connect,
        });
        let mut put_path = |key: &str, value: &Option<PathBuf>| {
            if let Some(path) = value {
                tls_json[key] = json!(path.to_string_lossy());
            }
        };
        put_path("root_ca_certificate", &tls.root_ca_certificate);
        put_path("listen_certificate", &tls.listen_certificate);
        put_path("listen_private_key", &tls.listen_private_key);
        put_path("connect_certificate", &tls.connect_certificate);
        put_path("connect_private_key", &tls.connect_private_key);
        config["transport"] = json!({ "link": { "tls": tls_json } });
    }

    config
}

/// Serializes a spec to a JSON string (valid JSON5) for the zenohd config file.
pub(crate) fn render_config_string(spec: &ZenohConfigSpec) -> String {
    serde_json::to_string(&build_zenoh_config(spec)).expect("zenoh config value serializes to JSON")
}

/// Renders a spec into a parsed [`zenoh::config::Config`] for opening a session.
pub(crate) fn render_config(spec: &ZenohConfigSpec) -> zenoh::config::Config {
    zenoh::config::Config::from_json5(&render_config_string(spec))
        .expect("generated zenoh config parses")
}

/// Rewrites the unroutable wildcard bind address to a connectable loopback host.
pub(crate) fn connectable_host(host: &str) -> String {
    if host == "0.0.0.0" {
        "127.0.0.1".to_string()
    } else {
        host.to_string()
    }
}

/// The liveness-probe config shared by the router watchdog and the
/// ephemeral-router readiness check: a plain client targeting one router
/// endpoint, no peer discovery. Only the `router` paths probe, so this is
/// `router`-gated (a `zenoh`-without-`router` consumer — e.g. the backend, which
/// only renders configs — would otherwise see it as dead code).
#[cfg(feature = "router")]
pub(crate) fn render_probe_config(
    protocol: ZenohNetProtocol,
    host: &str,
    port: u16,
    tls: Option<TlsConfig>,
) -> zenoh::config::Config {
    render_config(&ZenohConfigSpec {
        mode: SessionMode::Client,
        connect_endpoints: vec![format!("{protocol}/{}:{port}", connectable_host(host))],
        listen_endpoints: Vec::new(),
        reconnect: false,
        gossip: false,
        // A `tls/` router needs the probe to speak TLS too; plaintext callers pass
        // `None` (no `transport.link.tls` block, byte-identical to before).
        tls,
        // Readiness probes are deliberately namespace-free: a probe only checks
        // "is our router up?", which a namespace would only get in the way of.
        namespace: None,
    })
}

/// The config spec for a zenohd router listening on `protocol/host:port`. Shared
/// by the in-process spawn path ([`crate::zenohd::router_config_path`]) and the
/// out-of-process render path ([`render_router_config`]) so both produce an
/// identical router config. `gossip` seeds the peer mesh (a logged-out daemon in
/// peer topology wants it on; a federated or hub router wants it off so nothing
/// gossips locators over the federation link). `links` carries the platform
/// upstream and TLS material — see [`RouterLinks`].
pub(crate) fn router_spec(
    protocol: ZenohNetProtocol,
    host: &str,
    port: u16,
    gossip: bool,
    links: RouterLinks,
) -> ZenohConfigSpec {
    let RouterLinks { upstream, tls } = links;

    ZenohConfigSpec {
        mode: SessionMode::Router,
        reconnect: upstream.is_some(),
        connect_endpoints: upstream.into_iter().collect(),
        listen_endpoints: vec![format!("{protocol}/{host}:{port}")],
        gossip,
        tls,
        // A router is never namespaced: it only forwards between transport faces
        // and never opens an application session, so a router-level namespace
        // would not prefix forwarded/federated traffic. Isolation is enforced on
        // the application sessions instead.
        namespace: None,
    }
}

/// Renders a zenohd router config to a JSON5 string, for callers that run the
/// router out of process (e.g. a container) rather than spawning it via
/// [`crate::ZenohAdapter::with_router`]. With `protocol = Tls` and a server
/// [`TlsConfig`] in `links` this emits the `transport.link.tls` listener block;
/// a `links.upstream` emits a `connect` block so the router federates to that
/// upstream (see [`RouterLinks`]). Available under the base `zenoh` feature
/// (no `router`/zenohd binary needed) because rendering a config is
/// independent of spawning a process.
///
/// The `links` endpoints arrive as raw locator strings (possibly carrying
/// `#key=val;...` fragments), so a malformed one renders into a config zenohd
/// cannot parse. Every render therefore validates the output here, at the
/// single boundary both the in-process spawn path and the out-of-process
/// callers share, so a bad locator fails at render time instead of when
/// zenohd next boots.
pub fn render_router_config(
    protocol: ZenohNetProtocol,
    host: &str,
    port: u16,
    gossip: bool,
    links: RouterLinks,
) -> Result<String, crate::error::Error> {
    let config_content = render_config_string(&router_spec(protocol, host, port, gossip, links));
    zenoh::config::Config::from_json5(&config_content).map_err(|e| {
        crate::error::Error::ConfigurationError(format!("rendered zenohd config is invalid: {e}"))
    })?;
    Ok(config_content)
}

/// The loopback ephemeral listen endpoint a peer binds. Loopback-only by design:
/// it keeps the new inbound socket off the network (co-located peering only).
pub(crate) fn loopback_listen_endpoint(protocol: ZenohNetProtocol) -> String {
    format!("{protocol}/127.0.0.1:0")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn peer_spec(reconnect: bool, gossip: bool) -> ZenohConfigSpec {
        ZenohConfigSpec {
            mode: SessionMode::Peer,
            connect_endpoints: vec!["tcp/127.0.0.1:7448".to_string()],
            listen_endpoints: vec![loopback_listen_endpoint(ZenohNetProtocol::Tcp)],
            reconnect,
            gossip,
            tls: None,
            namespace: None,
        }
    }

    #[test]
    fn peer_config_has_peer_mode_listen_and_gossip_only_scouting() {
        let cfg = build_zenoh_config(&peer_spec(false, true));

        assert_eq!(cfg["mode"], "peer");
        assert_eq!(cfg["connect"]["endpoints"][0], "tcp/127.0.0.1:7448");
        // Peers listen on a loopback ephemeral port under the per-mode key.
        assert_eq!(cfg["listen"]["endpoints"]["peer"][0], "tcp/127.0.0.1:0");
        // Discovery is gossip-only.
        assert_eq!(cfg["scouting"]["multicast"]["enabled"], false);
        assert_eq!(cfg["scouting"]["gossip"]["enabled"], true);
        // A peer session stamps under its own role, not `client` — Zenoh reads
        // `enabled` against the session's mode, so an `enabled.client` entry here
        // would be silently ignored and leave samples unstamped.
        assert_eq!(cfg["timestamping"]["enabled"]["peer"], true);
        assert_eq!(cfg["timestamping"]["drop_future_timestamp"], false);
        assert!(cfg["timestamping"]["enabled"].get("client").is_none());
        // Fail-fast: no infinite-retry connect block.
        assert!(cfg["connect"].get("timeout_ms").is_none());
    }

    #[test]
    fn reconnecting_peer_config_retries_forever() {
        let cfg = build_zenoh_config(&peer_spec(true, true));

        assert_eq!(cfg["mode"], "peer");
        assert_eq!(cfg["connect"]["timeout_ms"], -1);
        assert_eq!(cfg["connect"]["exit_on_failure"], false);
        assert_eq!(cfg["connect"]["retry"]["period_init_ms"], 1000);
        assert_eq!(cfg["connect"]["retry"]["period_max_ms"], 4000);
    }

    #[test]
    fn gossip_can_be_disabled_to_force_router_relay() {
        let cfg = build_zenoh_config(&peer_spec(false, false));
        assert_eq!(cfg["scouting"]["gossip"]["enabled"], false);
    }

    #[test]
    fn probe_config_is_a_multicast_free_client() {
        let cfg = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Client,
            connect_endpoints: vec!["tcp/127.0.0.1:7448".to_string()],
            listen_endpoints: Vec::new(),
            reconnect: false,
            gossip: false,
            tls: None,
            namespace: None,
        });

        assert_eq!(cfg["mode"], "client");
        assert_eq!(cfg["scouting"]["multicast"]["enabled"], false);
        // A client stamps under its own role so its outgoing data carries a
        // source timestamp without depending on the router to add one.
        assert_eq!(cfg["timestamping"]["enabled"]["client"], true);
        // A client never listens for inbound peers.
        assert!(cfg.get("listen").is_none());
    }

    #[test]
    fn client_listen_endpoints_land_under_the_client_key() {
        // Clients do not listen today, so this guards the per-role-map rule
        // directly: were a client ever given a listen endpoint, it must land
        // under its own `client` key, not be silently misfiled under `peer`
        // (the same mismatch that left peer-mode samples unstamped).
        let cfg = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Client,
            connect_endpoints: vec!["tcp/127.0.0.1:7448".to_string()],
            listen_endpoints: vec!["tcp/127.0.0.1:0".to_string()],
            reconnect: false,
            gossip: false,
            tls: None,
            namespace: None,
        });

        assert_eq!(cfg["mode"], "client");
        assert_eq!(cfg["listen"]["endpoints"]["client"][0], "tcp/127.0.0.1:0");
        assert!(cfg["listen"]["endpoints"].get("peer").is_none());
    }

    #[test]
    fn router_config_listens_under_router_key_with_router_timestamping() {
        let cfg = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Router,
            connect_endpoints: Vec::new(),
            listen_endpoints: vec!["tcp/0.0.0.0:7448".to_string()],
            reconnect: false,
            gossip: true,
            tls: None,
            namespace: None,
        });

        assert_eq!(cfg["mode"], "router");
        assert_eq!(cfg["listen"]["endpoints"]["router"][0], "tcp/0.0.0.0:7448");
        assert_eq!(cfg["timestamping"]["enabled"]["router"], true);
        assert_eq!(cfg["timestamping"]["drop_future_timestamp"], false);
        // Routers do not dial out.
        assert!(cfg.get("connect").is_none());
    }

    #[test]
    fn generated_session_configs_parse_as_zenoh_config() {
        // Guards the JSON5 schema: a malformed block would otherwise only
        // surface as a panic at session/router open.
        render_config(&peer_spec(true, true));
        render_config(&peer_spec(false, true));
        // Gated like `render_probe_config` itself: only `router` paths probe.
        #[cfg(feature = "router")]
        render_probe_config(ZenohNetProtocol::Tcp, "0.0.0.0", 7448, None);
    }

    // ---- TLS rendering (the Phase-A breaking change) ----

    /// Regression guard: with `tls: None` the TLS feature is inert — no
    /// `transport` block is emitted, so a plaintext config is unchanged.
    /// (We assert structurally rather than byte-for-byte because serde_json's
    /// object key order depends on whether the `preserve_order` feature is
    /// unified in by some dependency, which would make an exact-string golden
    /// flaky across builds.)
    #[test]
    fn non_tls_spec_emits_no_transport_block() {
        let value = build_zenoh_config(&peer_spec(false, true));
        assert!(
            value.get("transport").is_none(),
            "a non-TLS spec must not render a transport block: {value}"
        );
        render_config(&peer_spec(false, true)); // and still parses
    }

    #[test]
    fn tls_router_config_emits_listen_cert_and_key() {
        let spec = router_spec(
            ZenohNetProtocol::Tls,
            "0.0.0.0",
            7447,
            false,
            RouterLinks {
                upstream: None,
                tls: Some(TlsConfig::server(
                    PathBuf::from("/certs/leaf.pem"),
                    PathBuf::from("/certs/leaf.key"),
                )),
            },
        );
        let cfg = build_zenoh_config(&spec);

        // The endpoint scheme is `tls/`, driven by the protocol's Display.
        assert_eq!(cfg["listen"]["endpoints"]["router"][0], "tls/0.0.0.0:7447");
        // A standalone (non-federated) router never dials out.
        assert!(cfg.get("connect").is_none());
        let tls = &cfg["transport"]["link"]["tls"];
        assert_eq!(tls["listen_certificate"], "/certs/leaf.pem");
        assert_eq!(tls["listen_private_key"], "/certs/leaf.key");
        assert_eq!(tls["enable_mtls"], false);
        assert_eq!(tls["verify_name_on_connect"], true);
        // Server identity only: no connect-side material leaks in.
        assert!(tls.get("connect_certificate").is_none());
        assert!(tls.get("root_ca_certificate").is_none());
    }

    #[test]
    fn tls_client_config_emits_root_ca_and_verify_name() {
        let cfg = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Client,
            connect_endpoints: vec!["tls/router.example:7443".to_string()],
            listen_endpoints: Vec::new(),
            reconnect: false,
            gossip: false,
            tls: Some(TlsConfig::client(PathBuf::from("/certs/ca.pem"))),
            namespace: None,
        });
        let tls = &cfg["transport"]["link"]["tls"];
        assert_eq!(tls["root_ca_certificate"], "/certs/ca.pem");
        assert_eq!(tls["verify_name_on_connect"], true);
        // Client trust only: no listener identity.
        assert!(tls.get("listen_certificate").is_none());
        assert!(tls.get("listen_private_key").is_none());
    }

    #[test]
    fn rendered_tls_router_config_parses_as_zenoh_config() {
        // The schema check: `render_router_config` validates its own output, so
        // an accepted render proves zenoh takes the `transport.link.tls` block
        // (a wrong key would be silently dropped, so the real validation is the
        // handshake integration test in tests/zenoh.rs).
        let s = render_router_config(
            ZenohNetProtocol::Tls,
            "0.0.0.0",
            7447,
            false,
            RouterLinks {
                upstream: None,
                tls: Some(TlsConfig::server(
                    PathBuf::from("/certs/leaf.pem"),
                    PathBuf::from("/certs/leaf.key"),
                )),
            },
        )
        .expect("rendered tls router config parses");
        assert!(s.contains("tls/0.0.0.0:7447"));
    }

    #[test]
    fn router_config_renders_a_single_tls_upstream() {
        // The daemon's local router: a plaintext `tcp/` listener for its own
        // nodes, PLUS a single `tls/` connect endpoint federating it to the
        // platform hub, trusting that hub via a client CA. This is the
        // peppy-side shape of the platform-only federation design.
        let s = render_router_config(
            ZenohNetProtocol::Tcp,
            "0.0.0.0",
            7448,
            false,
            RouterLinks {
                upstream: Some("tls/cap.zenoh.localhost:7443".to_string()),
                tls: Some(TlsConfig::client(PathBuf::from("/certs/ca.pem"))),
            },
        )
        .expect("federated router config renders and parses");
        let cfg: serde_json::Value =
            serde_json::from_str(&s).expect("rendered federated router config is JSON");

        assert_eq!(cfg["mode"], "router");
        // Local nodes still reach the router over plaintext loopback/LAN.
        assert_eq!(cfg["listen"]["endpoints"]["router"][0], "tcp/0.0.0.0:7448");
        // It federates out to the remote router over TLS, and keeps retrying so a
        // remote restart/reprovision is recovered (reconnect ⇒ `timeout_ms: -1`).
        assert_eq!(
            cfg["connect"]["endpoints"][0],
            "tls/cap.zenoh.localhost:7443"
        );
        assert_eq!(cfg["connect"]["timeout_ms"], -1);
        assert_eq!(cfg["connect"]["exit_on_failure"], false);
        // Connect-side trust only (verify the remote router's cert); no listener
        // identity, since the local listener is plaintext.
        let tls = &cfg["transport"]["link"]["tls"];
        assert_eq!(tls["root_ca_certificate"], "/certs/ca.pem");
        assert_eq!(tls["verify_name_on_connect"], true);
        assert!(tls.get("listen_certificate").is_none());
    }

    #[test]
    fn router_config_upstream_may_carry_endpoint_fragments() {
        // The upstream locator may carry per-endpoint `#key=val;...` config
        // fragments (how the daemon attaches the platform link's mTLS material)
        // without emitting a global transport block.
        let fragment_upstream = concat!(
            "tls/hub.example:7447#",
            "root_ca_certificate_file=/certs/ca.pem;",
            "connect_certificate_file=/certs/client.pem;",
            "connect_private_key_file=/certs/client.key;",
            "enable_mtls=true"
        )
        .to_string();
        let s = render_router_config(
            ZenohNetProtocol::Tcp,
            "0.0.0.0",
            7448,
            false,
            RouterLinks {
                upstream: Some(fragment_upstream.clone()),
                tls: None,
            },
        )
        .expect("fragment upstream config renders and parses");
        let rendered: serde_json::Value =
            serde_json::from_str(&s).expect("rendered router config is JSON");

        assert_eq!(rendered["connect"]["endpoints"], json!([fragment_upstream]));
        assert!(
            rendered.get("transport").is_none(),
            "endpoint-local TLS fragments must not emit a global transport block"
        );
    }

    // ---- Workspace session namespace rendering ----

    /// The `namespace` key is rendered for application sessions (`Peer`/`Client`)
    /// and only for them: it is a session-level field, so a router (which only
    /// forwards between faces) must never carry one. A namespaced spec must also
    /// still parse as a real zenoh config.
    #[test]
    fn namespace_rendered_for_sessions_never_for_router() {
        let ns = Namespace::parse("550e8400-e29b-41d4-a716-446655440000").unwrap();

        // Peer session: namespace present with the workspace value.
        let peer = build_zenoh_config(&ZenohConfigSpec {
            namespace: Some(ns.clone()),
            ..peer_spec(true, true)
        });
        assert_eq!(peer["namespace"], ns.as_str());
        render_config(&ZenohConfigSpec {
            namespace: Some(ns.clone()),
            ..peer_spec(true, true)
        });

        // Client session: namespace present.
        let client = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Client,
            connect_endpoints: vec!["tcp/127.0.0.1:7448".to_string()],
            listen_endpoints: Vec::new(),
            reconnect: false,
            gossip: false,
            tls: None,
            namespace: Some(ns.clone()),
        });
        assert_eq!(client["namespace"], ns.as_str());

        // Router: even when a namespace is supplied it is NOT rendered, because a
        // router never opens an application session.
        let router = build_zenoh_config(&ZenohConfigSpec {
            mode: SessionMode::Router,
            connect_endpoints: Vec::new(),
            listen_endpoints: vec!["tcp/0.0.0.0:7448".to_string()],
            reconnect: false,
            gossip: true,
            tls: None,
            namespace: Some(ns.clone()),
        });
        assert!(
            router.get("namespace").is_none(),
            "a router must never render a namespace: {router}"
        );

        // A namespace-free session omits the key entirely.
        let bare = build_zenoh_config(&peer_spec(false, true));
        assert!(bare.get("namespace").is_none());
    }
}
