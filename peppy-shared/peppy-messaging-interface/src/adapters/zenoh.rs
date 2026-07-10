//! Zenoh-backed implementation of [`crate::MessengerBackend`].
//!
//! ## Why callback handlers, not FIFO
//!
//! Every receive-side zenoh API call in this module (`declare_subscriber`,
//! `declare_queryable`, `session.get`) uses `.callback(...)` rather than the
//! default FIFO reception handler. Zenoh's FIFO handler holds an internal
//! `flume::bounded` channel and logs
//! `zenoh::api::handlers::fifo: error=sending on a closed channel` at ERROR
//! whenever zenoh tries to deliver a sample/query/reply after the
//! receiver-side has been dropped — a routine event in this codebase (e.g. a
//! `QueryTarget::All` `call_service` keeps the query open until its
//! `NO_TIMEOUT_SENTINEL`, and sibling producers' late replies hit a
//! `ReplyStream` the consumer dropped after the first valid response).
//!
//! Callback handlers have no intermediate channel: each callback invocation
//! either forwards into our own `flume::bounded` channel (subscriber /
//! queryable, where blocking `send` preserves backpressure) or our own tokio
//! mpsc (`call_service`, where `try_send` silently drops on a closed/full
//! receiver because the caller only needs the first valid reply).
//!
//! The `tests/fifo_noise.rs` integration test pins this invariant: it
//! asserts zero `zenoh::api::handlers::fifo` ERROR events during a wildcard
//! service call with a late-replying sibling producer.

use crate::error::{Error, Result};
use crate::types::{
    ActionLivelinessEvent, ActionLivelinessProbe, ActionLivelinessToken, ActionLivelinessWatch,
    IncomingRequest, NO_TIMEOUT_SENTINEL, Payload, PublisherQoS, ReplyStream, ResponseToken,
    ServiceQueryable, ServiceReply, SubscriberBufferSizes, SubscriberQoS, TopicMessage,
    ZenohResponseToken,
};
use crate::wire::zenoh_format::{ServiceReplyAttachment, TopicAttachment, ZenohWireFormat};
use crate::wire::{
    ActionWireReceiver, ActionWireSender, ServiceQueryKind, ServiceWireReceiver, ServiceWireSender,
    TopicWireReceiver, TopicWireSender,
};
use crate::zenoh_config::{
    SessionMode, TlsConfig, ZenohConfigSpec, connectable_host, loopback_listen_endpoint,
    render_config,
};
use config::org::{LOCAL_NAMESPACE, OrgNamespace};
// `render_probe_config` and the `zenohd` module (facade/health/config-path) are
// only used by the router-management paths; a `zenoh`-without-`router` build (the
// backend, which only renders configs and opens client sessions) does not see them.
#[cfg(feature = "router")]
use crate::zenoh_config::render_probe_config;
#[cfg(feature = "router")]
use crate::zenohd;
use crate::zenohd::ZenohNetProtocol;
#[cfg(feature = "router")]
use crate::{Messenger, MessengerAdapter};
use crate::{MessengerBackend, Subscription};

use std::net::SocketAddr;
#[cfg(feature = "router")]
use std::net::TcpListener;
use std::sync::Arc;
use tracing::info;

/// Zenoh-specific QoS settings derived from a `PublisherQoS` level.
struct ZenohQoS {
    priority: Priority,
    congestion_control: CongestionControl,
    express: bool,
}

impl From<PublisherQoS> for ZenohQoS {
    fn from(qos: PublisherQoS) -> Self {
        match qos {
            PublisherQoS::BestEffort => Self {
                priority: Priority::DataLow,
                congestion_control: CongestionControl::Drop,
                express: true,
            },
            PublisherQoS::Standard => Self {
                priority: Priority::Data,
                congestion_control: CongestionControl::Drop,
                express: false,
            },
            PublisherQoS::Important => Self {
                priority: Priority::DataHigh,
                congestion_control: CongestionControl::Block,
                express: false,
            },
            PublisherQoS::Critical => Self {
                priority: Priority::RealTime,
                congestion_control: CongestionControl::Block,
                express: true,
            },
        }
    }
}

/// Reserves an ephemeral port by binding to port 0 and returning the assigned port.
/// The returned `TcpListener` holds the port until dropped.
#[cfg(feature = "router")]
fn reserve_ephemeral_port() -> std::io::Result<(u16, TcpListener)> {
    let listener = TcpListener::bind(("127.0.0.1", 0))?;
    let port = listener.local_addr()?.port();
    Ok((port, listener))
}

/// Result of starting a zenohd router process.
///
/// The router is automatically stopped when this instance is dropped.
#[cfg(feature = "router")]
pub struct ZenohdInstance {
    messenger: Option<Messenger>,
    pub host: String,
    pub port: u16,
}

#[cfg(feature = "router")]
impl ZenohdInstance {
    /// Returns a mutable reference to the messenger.
    pub fn messenger(&mut self) -> &mut Messenger {
        self.messenger
            .as_mut()
            .expect("messenger was already taken")
    }

    /// Takes ownership of the messenger, preventing automatic cleanup on drop.
    pub fn take_messenger(&mut self) -> Messenger {
        self.messenger.take().expect("messenger was already taken")
    }
}

#[cfg(feature = "router")]
impl Drop for ZenohdInstance {
    fn drop(&mut self) {
        let Some(mut messenger) = self.messenger.take() else {
            return;
        };
        let _ = std::thread::spawn(move || {
            if let Ok(rt) = tokio::runtime::Runtime::new() {
                let _ = rt.block_on(async move { messenger.stop_router().await });
            }
        })
        .join();
    }
}

use zenoh::qos::{CongestionControl, Priority};
use zenoh::sample::{SampleFields, SampleKind};

/// Resolved config for a node/daemon peer session, plus the inputs needed to
/// rebuild it (the reconnecting session is re-derived on every
/// [`start_session`](MessengerBackend::start_session)).
pub struct ZenohClientConfig {
    zenoh_config: zenoh::config::Config,
    host: String,
    port: u16,
    protocol: ZenohNetProtocol,
    /// Resolved gossip seed endpoints (defaults to the router endpoint).
    seed_peers: Vec<String>,
    /// Whether gossip-based direct peer linking is enabled for this session.
    gossip: bool,
    /// Per-QoS subscriber channel buffer sizes for this session.
    buffer_sizes: SubscriberBufferSizes,
    /// Operator override (`ZENOH_SESSION_CONFIG`), resolved once at the
    /// constructor boundary. `Some` replaces the rendered config wholesale,
    /// including on the reconnecting-session rebuild in `start_session`, so the
    /// operator file wins regardless of the routing model. `None` renders from
    /// the fields above.
    override_config: Option<zenoh::config::Config>,
    /// Client TLS material for a `tls/` endpoint (`None` for plaintext). Retained
    /// (not just baked into `zenoh_config`) so the reconnecting-session rebuild
    /// in `start_session` re-renders with the same TLS settings.
    tls: Option<TlsConfig>,
    /// Organization namespace for this session (org-id routing isolation).
    /// Retained (like `tls`) so the reconnecting-session rebuild in
    /// `start_session` re-applies it; lost otherwise on every router-restart
    /// reconnect. Applied via [`ZenohAdapter::with_namespace`]; `None` leaves the
    /// session namespace-free (probes, tests). zenoh captures the namespace once
    /// at session build, so there is no in-process swap -- a change needs a fresh
    /// session.
    namespace: Option<OrgNamespace>,
}

pub struct ZenohAdapter {
    #[cfg(feature = "router")]
    zenohd: Option<zenohd::ZenohdFacade>,
    client_config: ZenohClientConfig,
    session: Option<Arc<zenoh::Session>>,
    /// When true, [`start_session`](MessengerBackend::start_session) opens a
    /// reconnecting session (see [`ZenohReconnectingClientConfigTemplate`]).
    reconnect_session: bool,
}

impl ZenohAdapter {
    /// Creates a ZenohAdapter that owns and manages its own zenohd router.
    /// Use this when you need to start a new router instance. `gossip` selects
    /// the daemon session's own routing model (peer vs router-relay) and
    /// `buffer_sizes` its subscriber channel capacities, both resolved from
    /// `peppy_config.json5`.
    ///
    /// `connect_endpoints` *federates* the spawned router to upstream routers it
    /// dials (`<proto>/<host>:<port>` each) — e.g. the daemon's plaintext loopback
    /// router dialing a remote `tls/` router so the two zenohd routers join one
    /// network. Empty is a standalone router (today's behavior). The connect-side
    /// trust for a `tls/` upstream rides in `tls` (a [`TlsConfig::client`]); it is
    /// written into the zenohd config only and is inert for the daemon's own
    /// plaintext loopback session.
    #[cfg(feature = "router")]
    pub fn with_router(
        protocol: ZenohNetProtocol,
        host: &str,
        port: u16,
        gossip: bool,
        buffer_sizes: SubscriberBufferSizes,
        connect_endpoints: Vec<String>,
        tls: Option<TlsConfig>,
    ) -> Result<Self> {
        let zenohd_config_path =
            zenohd::router_config_path(protocol, host, port, connect_endpoints, tls.clone())?;
        let facade = zenohd::ZenohdFacade::new(zenohd_config_path)?;
        let client_config =
            Self::derive_client_config_from_zenohd(&facade, gossip, buffer_sizes, tls)?;

        Ok(Self {
            zenohd: Some(facade),
            client_config,
            session: None,
            reconnect_session: false,
        })
    }

    /// Re-renders the owned router's zenohd config file *in place* with new
    /// federation `connect_endpoints` (+ connect-side `tls`) — same protocol /
    /// host / port as it was spawned with, only the upstream connect block (and
    /// its TLS trust) change. The new config takes effect on the next
    /// `stop_router` / `start_router`; this call does not itself restart zenohd.
    ///
    /// Used by the daemon to (de)federate its local router to the user's per-user
    /// cloud router when they log in / out, without a full daemon restart.
    ///
    /// Returns whether the config was actually rewritten: `Ok(true)` when a new
    /// config was rendered (the caller must restart zenohd to apply it),
    /// `Ok(false)` when a `ZENOH_CONFIG`-overridden config is in effect — that
    /// file is operator-owned and left untouched, so re-rendering is a no-op and
    /// there is nothing for the caller to restart for. Errors if the adapter owns
    /// no router.
    #[cfg(feature = "router")]
    pub fn refederate(
        &mut self,
        connect_endpoints: Vec<String>,
        tls: Option<TlsConfig>,
    ) -> Result<bool> {
        let facade = self.zenohd.as_ref().ok_or_else(|| {
            Error::BackendError("refederate called on an adapter that owns no router".to_string())
        })?;
        // An operator-pinned `ZENOH_CONFIG` (captured when the router was built) is
        // never rendered over, so re-rendering would change nothing. Report the
        // no-op so the caller skips the restart it would otherwise do to apply a
        // change that cannot take effect here.
        if facade.pinned {
            return Ok(false);
        }
        let ep = &facade.zenoh_endpoint;
        // Rewrite the exact config file captured when the router was built, *not*
        // via `router_config_path` — that re-reads `ZENOH_CONFIG`, which (if it
        // changed after startup) could redirect this write elsewhere or skip it
        // via the override early-return, leaving the running router's file stale.
        zenohd::render_router_config_to_path(
            &facade.zenohd_config_path,
            ep.protocol,
            &ep.host,
            ep.port,
            connect_endpoints,
            tls,
        )?;
        Ok(true)
    }

    /// Creates a ZenohAdapter that joins the mesh seeded by an existing zenohd
    /// router, using the default discovery (gossip on, seed = `host:port`) and
    /// default subscriber buffers. The session runs in `peer` mode so data forms
    /// direct links instead of relaying through the router.
    pub fn connect_to(protocol: ZenohNetProtocol, host: &str, port: u16) -> Result<Self> {
        Self::connect_to_with_discovery(
            protocol,
            host,
            port,
            Vec::new(),
            true,
            SubscriberBufferSizes::default(),
            None,
        )
    }

    /// Like [`connect_to`](Self::connect_to) but over TLS: opens a `tls/`
    /// **client** session to `host:port`, verifying the router against `tls`'s
    /// `root_ca_certificate` (with `verify_name_on_connect`). Unlike `connect_to`,
    /// this is **client** mode (`gossip = false`): all traffic routes through the
    /// router and the session binds no loopback peer listener — which is what we
    /// want for a remote router (a peer listener would also need its own server
    /// cert, which a pure client has no reason to hold). Use
    /// [`connect_to_with_discovery`](Self::connect_to_with_discovery) for control.
    ///
    /// This is the low-level TLS-client primitive: it is what the `tls/` transport
    /// tests dial a router with, and is available for a direct client→router
    /// session. The peppy daemon does **not** use it to reach the per-user cloud
    /// router — that is router-to-router federation (the local zenohd dials the
    /// remote over a `tls/` `connect` endpoint; see [`Self::with_router`]).
    pub fn connect_to_tls(host: &str, port: u16, tls: TlsConfig) -> Result<Self> {
        Self::connect_to_with_discovery(
            ZenohNetProtocol::Tls,
            host,
            port,
            Vec::new(),
            false,
            SubscriberBufferSizes::default(),
            Some(tls),
        )
    }

    /// Like [`connect_to`](Self::connect_to) but with an explicit gossip seed
    /// list, gossip toggle, and subscriber buffer sizes. The node runtime passes
    /// its `DiscoveryConfig` here. An empty `seed_peers` falls back to the single
    /// `host:port` seed.
    #[allow(clippy::too_many_arguments)]
    pub fn connect_to_with_discovery(
        protocol: ZenohNetProtocol,
        host: &str,
        port: u16,
        seed_peers: Vec<String>,
        gossip: bool,
        buffer_sizes: SubscriberBufferSizes,
        tls: Option<TlsConfig>,
    ) -> Result<Self> {
        let override_config = Self::resolve_session_config_override()?;
        let client_config = Self::create_client_config(
            protocol,
            host,
            port,
            false,
            seed_peers,
            gossip,
            buffer_sizes,
            override_config,
            tls,
            // Namespace-free by default; callers apply org-id isolation with
            // [`Self::with_namespace`] (e.g. peppylib's `MessengerHandle::connect`
            // builder, which defaults the namespace to `local`).
            None,
        );

        Ok(Self {
            #[cfg(feature = "router")]
            zenohd: None,
            client_config,
            session: None,
            reconnect_session: false,
        })
    }

    /// Marks this adapter's long-lived session as reconnecting: on
    /// [`start_session`](MessengerBackend::start_session) it uses a config that
    /// retries the connection (and re-declares its subscriptions/queryables) if
    /// the router is restarted under it. Used by the daemon so the router
    /// watchdog can respawn zenohd without leaving the daemon's own session
    /// dead. CLI and short-lived adapters leave this off (fail-fast default).
    pub fn with_session_reconnect(mut self) -> Self {
        self.reconnect_session = true;
        self
    }

    /// Starts a zenohd router with an ephemeral port, retrying on bind failures.
    /// The hosted session is a peer with default subscriber buffers; use
    /// [`start_router_ephemeral_in_mode`](Self::start_router_ephemeral_in_mode)
    /// to pick the session's gossip mode and buffer sizes.
    #[cfg(feature = "router")]
    pub async fn start_router_ephemeral(host: &str, port: Option<u16>) -> Result<ZenohdInstance> {
        Self::start_router_ephemeral_in_mode(
            host,
            port,
            true,
            SubscriberBufferSizes::default(),
            None,
        )
        .await
    }

    /// Like [`start_router_ephemeral`](Self::start_router_ephemeral) but the
    /// hosted session's `gossip` (peer vs router-relay) and subscriber buffer
    /// sizes are explicit. Used by tests to exercise both messaging modes.
    ///
    /// `namespace` stamps an organization namespace onto the hosted session
    /// (the same `with_router(...).with_namespace(...)` pairing the daemon uses),
    /// so a test that runs a core node off this session and spawns nodes under
    /// that org id stays routing-consistent with them. `None` leaves the hosted
    /// session namespace-free (the default for client-vs-client tests).
    ///
    /// When `port` is `None`, automatically selects an available port and retries
    /// up to 32 times if the port becomes unavailable. When `port` is `Some`,
    /// attempts exactly once with that port.
    ///
    /// Returns a [`ZenohdInstance`] that automatically stops the router when dropped.
    #[cfg(feature = "router")]
    pub async fn start_router_ephemeral_in_mode(
        host: &str,
        port: Option<u16>,
        gossip: bool,
        buffer_sizes: SubscriberBufferSizes,
        namespace: Option<OrgNamespace>,
    ) -> Result<ZenohdInstance> {
        let max_attempts = if port.is_some() { 1 } else { 32 };

        for attempt in 0..max_attempts {
            let (port, _reservation) = match port {
                Some(p) => (p, None),
                None => {
                    let (p, listener) =
                        reserve_ephemeral_port().map_err(|e| Error::BackendError(e.to_string()))?;
                    (p, Some(listener))
                }
            };

            let adapter = Self::with_router(
                ZenohNetProtocol::Tcp,
                host,
                port,
                gossip,
                buffer_sizes,
                Vec::new(),
                None,
            )?
            .with_namespace(namespace.clone());
            // A lightweight client probe (no listener, no peer discovery) is the
            // cheapest reliable "router accepts sessions yet?" check.
            let probe_config = render_probe_config(ZenohNetProtocol::Tcp, host, port, None);
            let mut messenger = Messenger::new(MessengerAdapter::Zenoh(adapter));

            // Drop the port reservation before starting the router so zenohd can bind to it
            drop(_reservation);

            match messenger.start_router().await {
                Ok(()) => {
                    // Readiness signal: zenohd's TCP listener can accept before the
                    // protocol handshake is settled, so a real zenoh::open is the only
                    // reliable signal that subsequent sessions will succeed. The probe
                    // session is dropped immediately; the caller opens their own.
                    match zenoh::open(probe_config).await {
                        Ok(probe) => {
                            drop(probe);
                            return Ok(ZenohdInstance {
                                messenger: Some(messenger),
                                host: host.to_string(),
                                port,
                            });
                        }
                        Err(_) if attempt + 1 < max_attempts => {
                            // Drop messenger to stop the router, then retry on a fresh port.
                            drop(messenger);
                            continue;
                        }
                        Err(e) => {
                            return Err(Error::BackendError(format!(
                                "Zenoh readiness probe failed: {}",
                                e
                            )));
                        }
                    }
                }
                Err(Error::BackendError(_)) if attempt + 1 < max_attempts => {
                    continue;
                }
                Err(err) => return Err(err),
            }
        }

        Err(Error::BackendError(format!(
            "Failed to start zenoh router after {max_attempts} attempts"
        )))
    }

    pub fn client_endpoint(&self) -> (&str, u16) {
        (self.client_config.host.as_str(), self.client_config.port)
    }

    /// Builds a lock-free [`RouterHealthChecker`] bound to this adapter's router
    /// endpoint, for the router watchdog to probe liveness without holding the
    /// central messenger lock.
    #[cfg(feature = "router")]
    pub fn router_health_checker(&self) -> zenohd::RouterHealthChecker {
        let probe_config = render_probe_config(
            self.client_config.protocol,
            &self.client_config.host,
            self.client_config.port,
            // Probe a `tls/` router over TLS using the same trust the adapter
            // holds; `None` for a plaintext router renders an unchanged config.
            self.client_config.tls.clone(),
        );
        zenohd::RouterHealthChecker::new(probe_config)
    }

    /// Resolves the `ZENOH_SESSION_CONFIG` operator override into an optional
    /// parsed config. When the env var names a non-empty path the file is
    /// loaded, and an unreadable or invalid file becomes an
    /// [`Error::ConfigurationError`] rather than a panic. `Ok(None)` when the
    /// var is unset or blank, in which case the session config is rendered from
    /// the explicit constructor arguments. Resolved once at the constructor
    /// boundary so [`create_client_config`](Self::create_client_config) stays a
    /// pure, env-free builder; mirrors the router's `ZENOH_CONFIG` resolution in
    /// `zenohd::router_config_path`.
    fn resolve_session_config_override() -> Result<Option<zenoh::config::Config>> {
        match std::env::var("ZENOH_SESSION_CONFIG") {
            Ok(path) if !path.trim().is_empty() => {
                let path = path.trim();
                zenoh::config::Config::from_file(path)
                    .map(Some)
                    .map_err(|e| {
                        Error::ConfigurationError(format!(
                            "ZENOH_SESSION_CONFIG ({path}) is not a readable zenoh config: {e}"
                        ))
                    })
            }
            _ => Ok(None),
        }
    }

    /// Builds a peer-session config seeded by `host:port` (or `seed_peers` when
    /// non-empty). Pure: the operator override is resolved by the caller via
    /// [`Self::resolve_session_config_override`] and passed in as
    /// `override_config`. When present it replaces the rendered config wholesale
    /// (including the `gossip`/mode rendering below), so the operator file wins
    /// on routing model. The override does NOT affect `buffer_sizes`, which are
    /// applied later at the flume/mpsc layer (subscribe / listen / call), so
    /// peer-mode buffer tuning from `peppy_config.json5` still takes effect.
    #[allow(clippy::too_many_arguments)]
    fn create_client_config(
        protocol: ZenohNetProtocol,
        host: &str,
        port: u16,
        reconnect: bool,
        seed_peers: Vec<String>,
        gossip: bool,
        buffer_sizes: SubscriberBufferSizes,
        override_config: Option<zenoh::config::Config>,
        tls: Option<TlsConfig>,
        namespace: Option<OrgNamespace>,
    ) -> ZenohClientConfig {
        let connect_host = connectable_host(host);
        let seeds = if seed_peers.is_empty() {
            vec![format!("{protocol}/{connect_host}:{port}")]
        } else {
            seed_peers
        };

        let zenoh_config = match &override_config {
            Some(config) => config.clone(),
            // `gossip` selects the routing model. Enabled: a `peer` that binds a
            // loopback listener and forms direct peer-to-peer links via gossip.
            // Disabled: a plain `client` that routes only through the router (no
            // listener, no peer discovery). Nodes that cannot form direct
            // loopback links with their peers (e.g. container nodes in a
            // separate network namespace) use the client path so they reach the
            // rest of the system through the router instead of advertising an
            // unreachable loopback locator and churning on failed autoconnects.
            None if gossip => render_config(&ZenohConfigSpec {
                mode: SessionMode::Peer,
                connect_endpoints: seeds.clone(),
                listen_endpoints: vec![loopback_listen_endpoint(protocol)],
                reconnect,
                gossip: true,
                tls: tls.clone(),
                namespace: namespace.clone(),
            }),
            None => render_config(&ZenohConfigSpec {
                mode: SessionMode::Client,
                connect_endpoints: seeds.clone(),
                listen_endpoints: Vec::new(),
                reconnect,
                gossip: false,
                tls: tls.clone(),
                namespace: namespace.clone(),
            }),
        };

        ZenohClientConfig {
            zenoh_config,
            host: connect_host,
            port,
            protocol,
            seed_peers: seeds,
            gossip,
            buffer_sizes,
            override_config,
            tls,
            namespace,
        }
    }

    #[cfg(feature = "router")]
    fn derive_client_config_from_zenohd(
        zenohd: &zenohd::ZenohdFacade,
        gossip: bool,
        buffer_sizes: SubscriberBufferSizes,
        tls: Option<TlsConfig>,
    ) -> Result<ZenohClientConfig> {
        // The daemon joins the mesh it hosts, seeded by its own router, so peers
        // can reach its core-node/daemon services. `gossip` (from
        // `peppy_config.json5`) selects whether that session is a peer (direct
        // links) or a client (router relay); router mode makes the daemon a
        // plain client of its own loopback router. Its long-lived session is
        // rebuilt as reconnecting in `start_session` when `reconnect_session` is
        // set; the readiness probe in `start_router_ephemeral` builds its own
        // client probe config.
        let override_config = Self::resolve_session_config_override()?;
        Ok(Self::create_client_config(
            zenohd.zenoh_endpoint.protocol,
            &zenohd.zenoh_endpoint.host,
            zenohd.zenoh_endpoint.port,
            false,
            Vec::new(),
            gossip,
            buffer_sizes,
            override_config,
            tls,
            // The daemon applies its org namespace via [`Self::with_namespace`]
            // after `with_router`, so the initial derive is namespace-free.
            None,
        ))
    }

    /// Applies an organization namespace to this adapter's session (org-id
    /// routing isolation), re-rendering the stored session config so a
    /// non-reconnecting session -- which opens `client_config.zenoh_config`
    /// directly -- carries it, and `start_session`'s reconnecting rebuild
    /// re-applies it the same way it does `tls`. `None` leaves the session
    /// namespace-free.
    ///
    /// There is intentionally no in-process namespace *swap* once a session is
    /// open: zenoh captures the namespace once at session build, so a change
    /// requires a fresh session (the daemon rebuilds its whole generation). The
    /// fail-closed check against an operator override runs at `start_session`.
    pub fn with_namespace(mut self, namespace: Option<OrgNamespace>) -> Self {
        let protocol = self.client_config.protocol;
        let host = self.client_config.host.clone();
        let port = self.client_config.port;
        let seed_peers = self.client_config.seed_peers.clone();
        let gossip = self.client_config.gossip;
        let buffer_sizes = self.client_config.buffer_sizes;
        let override_config = self.client_config.override_config.clone();
        let tls = self.client_config.tls.clone();
        self.client_config = Self::create_client_config(
            protocol,
            &host,
            port,
            false,
            seed_peers,
            gossip,
            buffer_sizes,
            override_config,
            tls,
            namespace,
        );
        self
    }

    /// Fail closed if an operator `ZENOH_SESSION_CONFIG` override would drop or
    /// change the org namespace on a session that federates. The override
    /// replaces the rendered config wholesale, so a federating session whose
    /// override carries no (or a different) namespace would emit unprefixed keys
    /// and accept everything -- a cross-tenant leak. A `local` (logged-out)
    /// session does not federate, so a missing/divergent namespace there is only
    /// warned about, not refused.
    fn ensure_override_preserves_namespace(&self) -> Result<()> {
        let (Some(expected), Some(override_cfg)) = (
            self.client_config.namespace.as_ref(),
            self.client_config.override_config.as_ref(),
        ) else {
            return Ok(());
        };
        // Read the override's session-level `namespace` by serializing it (the
        // public `Config` is a transparent newtype over the inner validated
        // config, so its `namespace` field surfaces as a top-level JSON key).
        // This avoids depending on the visibility of the macro-generated getter.
        let override_json = serde_json::to_value(override_cfg).unwrap_or(serde_json::Value::Null);
        let override_ns = override_json.get("namespace").and_then(|v| v.as_str());
        if override_ns == Some(expected.as_str()) {
            return Ok(());
        }
        // The override does not declare the expected namespace.
        if expected.as_str() == LOCAL_NAMESPACE {
            tracing::warn!(
                expected = %expected.as_str(),
                override_namespace = ?override_ns,
                "ZENOH_SESSION_CONFIG overrides the local-namespace session config; \
                 proceeding because a logged-out session does not federate"
            );
            return Ok(());
        }
        Err(Error::ConfigurationError(format!(
            "ZENOH_SESSION_CONFIG drops or changes the organization namespace \
             (expected `{}`, override has `{}`); refusing to open a federating session \
             that would route across tenants",
            expected.as_str(),
            override_ns.unwrap_or("<none>"),
        )))
    }
}

impl MessengerBackend for ZenohAdapter {
    async fn start_session(&mut self) -> Result<()> {
        // Fail closed before opening: an operator `ZENOH_SESSION_CONFIG` override
        // replaces the rendered config wholesale (namespace included), so a
        // federating session whose override carries no org namespace would leak
        // across tenants. Refuse rather than silently downgrade.
        self.ensure_override_preserves_namespace()?;

        // The daemon's long-lived session uses a reconnecting config so it
        // re-establishes itself (and re-declares its subscriptions/queryables)
        // if the router is restarted under it — e.g. by the router watchdog.
        // Short-lived / CLI sessions keep the fail-fast default.
        let config = if self.reconnect_session {
            Self::create_client_config(
                self.client_config.protocol,
                &self.client_config.host,
                self.client_config.port,
                true,
                self.client_config.seed_peers.clone(),
                self.client_config.gossip,
                self.client_config.buffer_sizes,
                self.client_config.override_config.clone(),
                self.client_config.tls.clone(),
                self.client_config.namespace.clone(),
            )
            .zenoh_config
        } else {
            self.client_config.zenoh_config.clone()
        };

        let session = zenoh::open(config)
            .await
            .map_err(|e| Error::BackendError(format!("Failed to create Zenoh session: {}", e)))?;

        info!(
            "Zenoh session started on: {}://{}:{}",
            &self.client_config.protocol, &self.client_config.host, &self.client_config.port
        );
        self.session = Some(Arc::new(session));
        Ok(())
    }

    async fn stop_session(&mut self) -> Result<()> {
        if let Some(session) = self.session.take() {
            // Close while zenohd is still alive so the undeclare-face
            // messages reach the router. Drop's later close becomes a
            // no-op (primitives already taken), which is what keeps the
            // session's other Arc clones — e.g. ZenohPublisher — from
            // spamming "Undefined face context" when they finally drop.
            if let Err(err) = session.close().await {
                tracing::warn!("Zenoh session close returned an error: {err}");
            }
        }
        Ok(())
    }

    async fn subscribe_topic(
        &self,
        recv: &TopicWireReceiver,
        qos: SubscriberQoS,
    ) -> Result<Subscription> {
        let drop_secondary = recv.drops_secondary_publishes();
        self.subscribe_keyexpr(ZenohWireFormat::topic_subscribe(recv), qos, drop_secondary)
            .await
    }

    async fn publish_topic(
        &mut self,
        sender: &TopicWireSender,
        payload: Payload,
        qos: PublisherQoS,
        is_primary: bool,
    ) -> Result<()> {
        self.publish_keyexpr(
            &ZenohWireFormat::topic_publish(sender),
            payload,
            qos,
            is_primary,
        )
        .await
    }

    async fn listen_service(&self, recv: &ServiceWireReceiver) -> Result<ServiceQueryable> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;

        let (tx, rx) = flume::bounded::<IncomingRequest>(
            self.client_config
                .buffer_sizes
                .size_for(SubscriberQoS::Standard),
        );

        // One queryable per listen call. The declared keyexpr has `*` at the
        // link_id slot so a single queryable absorbs every bound link_id —
        // `process_inbound_query` does the dispatch by parsing the selector.
        // Two queryables for one process would let a caller's wildcard
        // `*` selector double-deliver via `QueryTarget::All`.
        let declare_keyexpr = ZenohWireFormat::service_queryable_declare(recv);
        let recv_clone = recv.clone();
        // Probe replies are spawned onto the listener's runtime: the zenoh
        // callback runs on a zenoh worker thread that must not block, and
        // `listen_service` always executes inside the consumer's tokio
        // runtime, so the handle is available to capture here.
        let rt = tokio::runtime::Handle::current();
        let queryable = session
            .declare_queryable(&declare_keyexpr)
            .complete(true)
            .callback(move |query| {
                process_inbound_query(query, &recv_clone, &tx, &rt);
            })
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;

        Ok(ServiceQueryable::new(rx, vec![Box::new(queryable)]))
    }

    async fn call_service(
        &self,
        sender: &ServiceWireSender,
        payload: Payload,
        kind: ServiceQueryKind,
        timeout: Option<std::time::Duration>,
    ) -> Result<ReplyStream> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        let selector = ZenohWireFormat::service_get_selector(sender);
        // Mandatory query attachment: carries the request kind (UserRequest
        // vs Probe) plus the consumer's sibling-exclusion set. The producer
        // refuses queries with no attachment, which is what makes the
        // mid-rollout failure mode loud (consumer sees ServiceUnreachable
        // instead of misclassifying the request as a default).
        let attachment = ZenohWireFormat::service_get_selector_attachment(sender, kind);

        let timeout = timeout.unwrap_or(NO_TIMEOUT_SENTINEL);

        let (tx, rx) = tokio::sync::mpsc::channel::<ServiceReply>(
            self.client_config
                .buffer_sizes
                .size_for(SubscriberQoS::Standard),
        );

        // `try_send` (not `send`) because the callback runs synchronously on
        // a zenoh worker thread that we must not block. Two drop conditions
        // are tolerated here:
        //   1. receiver dropped — caller has the first valid reply and has
        //      released the `ReplyStream`; sibling producers' late replies
        //      go nowhere, which is intentional;
        //   2. channel full (capacity = the Standard subscriber buffer size)
        //      — would only happen if the consumer's `poll_service` loop
        //      stalls for thousands of replies; in practice the consumer
        //      drains the channel as fast as zenoh fills it, so this branch
        //      is effectively unreachable. If it ever fires, the lost reply
        //      is acceptable: `QueryTarget::All` is best-effort fan-in, not
        //      a guaranteed-delivery API.
        // See the module-level "Why callback handlers, not FIFO" doc.
        session
            .get(&selector)
            .payload(payload.into_zbytes())
            .attachment(attachment.to_vec())
            .target(zenoh::query::QueryTarget::All)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .accept_replies(zenoh::query::ReplyKeyExpr::Any)
            .timeout(timeout)
            .callback(move |reply| {
                let sample = match reply.result() {
                    Ok(sample) => sample,
                    Err(err) => {
                        tracing::warn!(?err, "service reply contained an error");
                        return;
                    }
                };
                let key_expr = sample.key_expr().as_str();
                let zbytes = sample.payload().clone();
                let attachment_bytes = sample
                    .attachment()
                    .map(|z| z.to_bytes())
                    .unwrap_or_default();
                let reply_kind = match ServiceReplyAttachment::decode(attachment_bytes.as_ref()) {
                    Ok(a) => a.kind,
                    Err(err) => {
                        tracing::error!(%key_expr, %err, "dropping service reply with malformed attachment");
                        return;
                    }
                };
                match TopicMessage::from_zbytes(key_expr, zbytes) {
                    Ok(message) => {
                        let _ = tx.try_send(ServiceReply::new(message, reply_kind));
                    }
                    Err(err) => {
                        tracing::error!(%key_expr, %err, "failed to parse service reply keyexpr");
                    }
                }
            })
            .await
            .map_err(|e| Error::BackendError(e.to_string()))?;

        Ok(ReplyStream::new(rx, None))
    }

    async fn subscribe_action_feedback(
        &self,
        sender: &ActionWireSender,
        goal_id: &str,
        qos: SubscriberQoS,
    ) -> Result<Subscription> {
        // Action feedback shares the wildcard-link_id keyexpr shape with
        // topic subscribe but doesn't multi-publish per goal — feedback is
        // emitted under the single link_id chosen at goal time (see the
        // `action_feedback_publish` comment in `wire::zenoh_format`). So
        // there are no secondaries to drop; pass `false`.
        self.subscribe_keyexpr(
            ZenohWireFormat::action_feedback_subscribe(sender, goal_id),
            qos,
            false,
        )
        .await
    }

    async fn declare_action_liveliness(
        &self,
        recv: &ActionWireReceiver,
    ) -> Result<ActionLivelinessToken> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        let keyexpr = ZenohWireFormat::action_liveliness_token(recv);
        let token = session
            .liveliness()
            .declare_token(keyexpr)
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;
        Ok(ActionLivelinessToken::new(Box::new(token)))
    }

    async fn watch_action_producer(
        &self,
        sender: &ActionWireSender,
    ) -> Result<ActionLivelinessWatch> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        // Unbounded: liveliness transitions are rare (producer restarts,
        // router flaps) and the callback runs on a zenoh worker thread that
        // must never block. See the module-level "Why callback handlers,
        // not FIFO" doc.
        let (tx, rx) = flume::unbounded::<ActionLivelinessEvent>();
        let keyexpr = ZenohWireFormat::action_liveliness_watch(sender);
        // `history(true)` replays a token that was declared before this
        // watch existed as an initial PUT, so "producer already alive" and
        // "producer came alive" are observed identically.
        let subscriber = session
            .liveliness()
            .declare_subscriber(&keyexpr)
            .history(true)
            .callback(move |sample| {
                let event = match sample.kind() {
                    SampleKind::Put => ActionLivelinessEvent::Alive,
                    SampleKind::Delete => ActionLivelinessEvent::Gone,
                };
                let _ = tx.send(event);
            })
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;
        Ok(ActionLivelinessWatch::new(rx, Box::new(subscriber)))
    }

    async fn probe_action_producer(
        &self,
        sender: &ActionWireSender,
        timeout: std::time::Duration,
    ) -> Result<ActionLivelinessProbe> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        let keyexpr = ZenohWireFormat::action_liveliness_watch(sender);
        // The callback closure owns `tx`; zenoh drops it when the query
        // finalizes (at the latest after `timeout`), so the probe's
        // `resolve` observes `Disconnected` exactly when the query
        // completed with no matching token. Only issuance is awaited here.
        let (tx, rx) = flume::bounded::<()>(1);
        session
            .liveliness()
            .get(&keyexpr)
            .timeout(timeout)
            .callback(move |reply| {
                if reply.result().is_ok() {
                    let _ = tx.try_send(());
                }
            })
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;
        Ok(ActionLivelinessProbe::new(rx))
    }

    async fn start_router(&mut self) -> Result<()> {
        #[cfg(feature = "router")]
        {
            let zenohd = self
                .zenohd
                .as_mut()
                .ok_or(Error::ZenohDConfigurationNotFound)?;
            zenohd.start_router()?;
            Ok(())
        }
        // Client-only build: router management was not compiled in.
        #[cfg(not(feature = "router"))]
        {
            Err(Error::ZenohDConfigurationNotFound)
        }
    }

    async fn stop_router(&mut self) -> Result<()> {
        #[cfg(feature = "router")]
        {
            let zenohd = self
                .zenohd
                .as_mut()
                .ok_or(Error::ZenohDConfigurationNotFound)?;
            zenohd.stop_router()?;
            Ok(())
        }
        // Client-only build: router management was not compiled in.
        #[cfg(not(feature = "router"))]
        {
            Err(Error::ZenohDConfigurationNotFound)
        }
    }

    fn get_host(&self) -> SocketAddr {
        let host = &self.client_config.host;
        let port = self.client_config.port;
        let ip = host
            .parse()
            .unwrap_or(std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST));
        SocketAddr::new(ip, port)
    }
}

impl ZenohAdapter {
    /// Pre-bind a per-topic publisher for `sender`. The returned publisher
    /// holds an `Arc<Session>` clone so its `publish` is independent of the
    /// `Arc<Mutex<Messenger>>` global lock.
    pub fn declare_topic_publisher(
        &self,
        sender: &TopicWireSender,
        qos: PublisherQoS,
    ) -> Result<ZenohPublisher> {
        self.declare_publisher_keyexpr(ZenohWireFormat::topic_publish(sender), qos)
    }

    /// Pre-bind a per-goal action-feedback publisher.
    pub fn declare_action_feedback_publisher(
        &self,
        recv: &ActionWireReceiver,
        link_id: &str,
        goal_id: &str,
        qos: PublisherQoS,
    ) -> Result<ZenohPublisher> {
        self.declare_publisher_keyexpr(
            ZenohWireFormat::action_feedback_publish(recv, link_id, goal_id),
            qos,
        )
    }

    fn declare_publisher_keyexpr(
        &self,
        topic: String,
        qos: PublisherQoS,
    ) -> Result<ZenohPublisher> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        Ok(ZenohPublisher {
            session: Arc::clone(session),
            topic,
            qos: ZenohQoS::from(qos),
        })
    }

    /// Waits until a subscriber whose key expression matches `keyexpr` is known
    /// to this session, or `timeout` elapses; returns whether a match was seen.
    ///
    /// Peer-mode gossip discovery is not instantaneous, so a freshly-connected
    /// publisher may not yet know about an existing subscriber and would drop
    /// its first send. Awaiting Zenoh's matching status (event-driven via the
    /// matching listener) makes that first reliable send deterministic. A
    /// short-lived publisher is declared purely to observe matching; the publish
    /// path itself is unchanged.
    pub async fn wait_for_matching_subscriber(
        &self,
        keyexpr: &str,
        timeout: std::time::Duration,
    ) -> Result<bool> {
        self.subscriber_match_wait(keyexpr.to_string())?
            .resolve(timeout)
            .await
    }

    /// Snapshot a detached [`SubscriberMatchWait`] for `keyexpr`. Cheap and
    /// non-blocking (clones the session handle); the wait itself happens in
    /// [`SubscriberMatchWait::resolve`], so callers sharing the adapter behind
    /// a lock can release it before waiting out the timeout (mirrors
    /// [`ActionLivelinessProbe`]).
    fn subscriber_match_wait(&self, keyexpr: String) -> Result<SubscriberMatchWait> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        Ok(SubscriberMatchWait {
            session: Arc::clone(session),
            keyexpr,
        })
    }

    /// [`wait_for_matching_subscriber`](Self::wait_for_matching_subscriber) for a
    /// topic, building the publish key expression from `sender`.
    pub async fn wait_for_topic_subscriber(
        &self,
        sender: &TopicWireSender,
        timeout: std::time::Duration,
    ) -> Result<bool> {
        self.topic_subscriber_wait(sender)?.resolve(timeout).await
    }

    /// [`subscriber_match_wait`](Self::subscriber_match_wait) for a topic,
    /// building the publish key expression from `sender`.
    pub fn topic_subscriber_wait(&self, sender: &TopicWireSender) -> Result<SubscriberMatchWait> {
        self.subscriber_match_wait(ZenohWireFormat::topic_publish(sender))
    }

    async fn publish_keyexpr(
        &self,
        keyexpr: &str,
        payload: Payload,
        qos: PublisherQoS,
        is_primary: bool,
    ) -> Result<()> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;
        let zenoh_qos = ZenohQoS::from(qos);

        // session.put() directly rather than declare_publisher() + put() + drop.
        // This avoids the publisher declaration/undeclare lifecycle that causes
        // routing interference between successive service polls with different
        // targeting.
        session
            .put(keyexpr, payload.as_bytes().as_ref())
            .attachment(TopicAttachment { is_primary }.encode().to_vec())
            .congestion_control(zenoh_qos.congestion_control)
            .priority(zenoh_qos.priority)
            .express(zenoh_qos.express)
            .await
            .map_err(|e| Error::PublishError {
                topic: e.to_string(),
            })?;
        Ok(())
    }

    async fn subscribe_keyexpr(
        &self,
        keyexpr: String,
        qos: SubscriberQoS,
        drop_secondary: bool,
    ) -> Result<Subscription> {
        let (tx, rx) = flume::bounded(self.client_config.buffer_sizes.size_for(qos));

        let session = self
            .session
            .as_ref()
            .ok_or_else(|| Error::MessagingSessionError("Session not initialized".to_string()))?;

        // Blocking `flume::Sender::send` (not `try_send`) so Reliable QoS
        // topics get end-to-end backpressure: if the consumer's buffer is
        // full, zenoh's reception thread blocks here, propagating the stall
        // back to the publisher. `Err` only fires once the receiver is
        // dropped — silently discard, the subscription is going away. See
        // the module-level "Why callback handlers, not FIFO" doc.
        let subscriber = session
            .declare_subscriber(&keyexpr)
            .callback(move |sample| {
                let SampleFields {
                    key_expr,
                    payload,
                    attachment,
                    timestamp,
                    ..
                } = sample.into();
                if drop_secondary {
                    let raw = attachment
                        .as_ref()
                        .map(|z| z.to_bytes())
                        .unwrap_or_default();
                    if !TopicAttachment::decode(raw.as_ref()).is_primary {
                        return;
                    }
                }
                // Producer-stamped send time (NTP64 → ns since the Unix epoch),
                // present when session/router timestamping is enabled. Surfaced
                // so consumers can measure real delivery latency.
                let source_timestamp_nanos = timestamp.as_ref().map(|ts| ts.get_time().as_nanos());
                let key_expr = key_expr.as_str();
                match TopicMessage::from_zbytes(key_expr, payload) {
                    Ok(message) => {
                        let _ =
                            tx.send(message.with_source_timestamp_nanos(source_timestamp_nanos));
                    }
                    Err(err) => {
                        tracing::error!(
                            %key_expr,
                            %err,
                            "Failed to build ResponseMessage from sample"
                        );
                    }
                }
            })
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;

        Ok(Subscription::new(rx, Box::new(subscriber)))
    }
}

/// Per-query inbound handler. Parses the selector, verifies the caller's
/// link_id slot resolves to the producer's default `_` segment via
/// [`ParsedInboundQuery::claim`], builds an [`IncomingRequest`] with a
/// [`ResponseToken::Zenoh`] (carrying the concrete reply keyexpr) and pushes
/// it onto `tx`.
///
/// The parser also re-validates concrete target core / instance slots against
/// this receiver. That is defensive: Zenoh's matcher should already have
/// filtered them out, but a stale peer-routing view must not let a pinned
/// action goal run on a sibling instance. Queries whose link_id slot is neither
/// `*` nor `_` are dropped silently for the same reason.
///
/// This runs inside zenoh's reception callback, so the function is sync —
/// `flume::Sender::send` blocks the zenoh worker thread when the buffer is
/// full so peppylib applies backpressure rather than losing requests, and
/// returns `Err` (silently ignored) only when the consumer has dropped the
/// `ServiceQueryable`.
fn process_inbound_query(
    query: zenoh::query::Query,
    recv: &ServiceWireReceiver,
    tx: &flume::Sender<IncomingRequest>,
    rt: &tokio::runtime::Handle,
) {
    let attachment_bytes = query.attachment().map(|z| z.to_bytes()).unwrap_or_default();
    let parsed = match ZenohWireFormat::parse_inbound_query(
        recv,
        query.key_expr().as_str(),
        attachment_bytes.as_ref(),
    ) {
        Ok(p) => p,
        Err(err) => {
            tracing::warn!(
                query_keyexpr = %query.key_expr().as_str(),
                %err,
                "failed to parse inbound service query selector",
            );
            return;
        }
    };

    let chosen_link_id = match parsed.claim() {
        Some(l) => l.to_string(),
        None => {
            tracing::trace!(
                query_keyexpr = %query.key_expr().as_str(),
                parsed_link_id = %parsed.link_id,
                "dropping inbound query: link_id slot is neither '*' nor '_'",
            );
            return;
        }
    };

    let reply_keyexpr = ZenohWireFormat::service_reply_keyexpr(
        recv,
        &chosen_link_id,
        &parsed.caller_core,
        &parsed.caller_inst,
    );

    let payload = match query.payload() {
        Some(zb) => Payload::from_zbytes(zb.clone()),
        None => Payload::from_bytes(bytes::Bytes::new()),
    };

    let token = ResponseToken::Zenoh(ZenohResponseToken::new(query, reply_keyexpr));

    // Probes (liveness, discovery, benchmark sized-probes) are answered
    // right here in the dispatch path and never reach the endpoint channel:
    // the endpoint's recv loop only runs while the producer task is parked
    // in it, so answering there would starve discovery whenever user code
    // is executing a handler or goal. The reply MUST be Response-kind
    // (never Ack) — the consumer's discover-then-pin loop pins identity
    // off the first non-Ack reply. The reply is spawned (not awaited):
    // this callback runs on a zenoh worker thread that must not block.
    if parsed.kind == ServiceQueryKind::Probe {
        let response = crate::probe::probe_response_body(payload.as_bytes().as_ref());
        rt.spawn(async move {
            if let Err(err) = token.respond_response(Payload::from_bytes(response)).await {
                tracing::warn!(%err, "failed to publish probe response");
            }
        });
        return;
    }

    let request = IncomingRequest {
        payload,
        kind: parsed.kind,
        link_id: chosen_link_id,
        caller_core: parsed.caller_core,
        caller_inst: parsed.caller_inst,
        token,
    };

    let _ = tx.send(request);
}

/// Zenoh-side per-topic publisher returned by [`ZenohAdapter::declare_publisher`].
///
/// Mirrors [`ZenohAdapter::publish`]'s `session.put()` path (NOT a long-lived
/// `zenoh::pubsub::Publisher`); see the comment there about routing
/// interference between successive service polls. The win here is bypassing
/// the central `Messenger` mutex; zenoh's session itself is lock-free for
/// `put`.
pub struct ZenohPublisher {
    session: Arc<zenoh::Session>,
    topic: String,
    qos: ZenohQoS,
}

/// In-flight wait for a matching subscriber, issued by
/// [`ZenohAdapter::topic_subscriber_wait`]. Owns its session handle, so
/// [`resolve`](Self::resolve) runs detached from the adapter and callers can
/// release any shared adapter lock before waiting out the timeout — the same
/// issue-then-resolve split as
/// [`ActionLivelinessProbe`](crate::types::ActionLivelinessProbe).
pub struct SubscriberMatchWait {
    session: Arc<zenoh::Session>,
    keyexpr: String,
}

impl SubscriberMatchWait {
    /// Waits until a subscriber whose key expression matches is known to the
    /// session, or `timeout` elapses; returns whether a match was seen.
    pub async fn resolve(self, timeout: std::time::Duration) -> Result<bool> {
        let publisher = self
            .session
            .declare_publisher(self.keyexpr)
            .await
            .map_err(|e| Error::MessagingSessionError(e.to_string()))?;

        if publisher
            .matching_status()
            .await
            .map_err(|e| Error::BackendError(e.to_string()))?
            .matching()
        {
            return Ok(true);
        }
        // Subscribe to changes, then re-check once: this closes the race where a
        // matching subscriber appears between the first query and the listener
        // being installed.
        let listener = publisher
            .matching_listener()
            .await
            .map_err(|e| Error::BackendError(e.to_string()))?;
        if publisher
            .matching_status()
            .await
            .map_err(|e| Error::BackendError(e.to_string()))?
            .matching()
        {
            return Ok(true);
        }

        let matched = tokio::time::timeout(timeout, async {
            loop {
                match listener.recv_async().await {
                    Ok(status) if status.matching() => return true,
                    Ok(_) => continue,
                    Err(_) => return false,
                }
            }
        })
        .await
        .unwrap_or(false);
        Ok(matched)
    }
}

impl ZenohPublisher {
    pub async fn publish(&self, payload: bytes::Bytes) -> Result<()> {
        // Pre-bound publishers are single-link (one keyexpr per declare),
        // so from a wildcard subscriber's view this publish is the only
        // one for its emit and must be marked primary. Topic publishers
        // that need multi-link fan-out should go through `emit`, not
        // `declare_publisher` — see the rustdoc on
        // `TopicMessenger::declare_publisher`.
        self.session
            .put(&self.topic, payload.as_ref())
            .attachment(TopicAttachment { is_primary: true }.encode().to_vec())
            .congestion_control(self.qos.congestion_control)
            .priority(self.qos.priority)
            .express(self.qos.express)
            .await
            .map_err(|e| Error::PublishError {
                topic: e.to_string(),
            })?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_client_config_rewrites_wildcard_host_and_defaults_the_seed() {
        // `0.0.0.0` must be rewritten to a connectable loopback host, and an
        // empty seed list falls back to the single `host:port` endpoint. A
        // malformed config would panic here (parse is `.expect()`).
        let reconnecting = ZenohAdapter::create_client_config(
            ZenohNetProtocol::Tcp,
            "0.0.0.0",
            7448,
            true,
            Vec::new(),
            true,
            SubscriberBufferSizes::default(),
            None,
            None,
            None,
        );
        assert_eq!(reconnecting.host, "127.0.0.1");
        assert_eq!(
            reconnecting.seed_peers,
            vec!["tcp/127.0.0.1:7448".to_string()]
        );
        assert!(reconnecting.gossip);
    }

    #[test]
    fn create_client_config_honors_an_explicit_seed_list_and_buffer_sizes() {
        let cfg = ZenohAdapter::create_client_config(
            ZenohNetProtocol::Tcp,
            "127.0.0.1",
            7448,
            false,
            vec!["tcp/10.0.0.2:7448".to_string()],
            false,
            SubscriberBufferSizes {
                standard: 64,
                high_throughput: 4096,
            },
            None,
            None,
            None,
        );
        assert_eq!(cfg.seed_peers, vec!["tcp/10.0.0.2:7448".to_string()]);
        assert!(!cfg.gossip);
        assert_eq!(cfg.buffer_sizes.standard, 64);
        assert_eq!(cfg.buffer_sizes.high_throughput, 4096);
    }

    /// `refederate` re-renders the router's config in place with the upstream
    /// connect endpoint + connect-side trust — the live (de)federation the daemon
    /// drives on login/logout. `with_router` only renders + reads config (no
    /// zenohd process), so this is a pure file check.
    #[cfg(feature = "router")]
    #[test]
    fn refederate_rewrites_the_router_config_with_the_upstream_and_trust() {
        // A port unlikely to collide with other config-rendering tests (the
        // rendered config path is keyed by port).
        let port = 59247;
        let mut adapter = ZenohAdapter::with_router(
            ZenohNetProtocol::Tcp,
            "127.0.0.1",
            port,
            false,
            SubscriberBufferSizes::default(),
            Vec::new(),
            None,
        )
        .expect("build standalone router adapter");

        let cfg_path = adapter
            .zenohd
            .as_ref()
            .expect("router adapter owns a facade")
            .zenohd_config_path
            .clone();
        let before = std::fs::read_to_string(&cfg_path).expect("read rendered config");
        assert!(
            !before.contains("tls/cap.zenoh.localhost:7443"),
            "a standalone router has no upstream connect endpoint"
        );

        let rewrote = adapter
            .refederate(
                vec!["tls/cap.zenoh.localhost:7443".to_string()],
                Some(TlsConfig::client(std::path::PathBuf::from("/certs/ca.pem"))),
            )
            .expect("refederate rewrites the config in place");
        assert!(rewrote, "a rendered config reports it was rewritten");

        let after = std::fs::read_to_string(&cfg_path).expect("read refederated config");
        assert!(
            after.contains("tls/cap.zenoh.localhost:7443"),
            "upstream connect endpoint is now present"
        );
        assert!(
            after.contains("/certs/ca.pem"),
            "connect-side CA trust is now present"
        );

        // refederate on an adapter that owns no router (a client) is an error.
        let mut clientish = ZenohAdapter::connect_to(ZenohNetProtocol::Tcp, "127.0.0.1", port)
            .expect("build client adapter");
        assert!(clientish.refederate(Vec::new(), None).is_err());

        let _ = std::fs::remove_file(&cfg_path);
    }
}
