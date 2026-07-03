use crate::common::AnyType;
use crate::consts::ALLOWED_CONFIG_CHARS;
use crate::error::{ParsingError, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::{
    fs,
    path::{Path, PathBuf},
};

/// Validated identifier for node names, tags, instance ids, and core node
/// names in runtime configs: non-empty and restricted to
/// [`ALLOWED_CONFIG_CHARS`](crate::consts::ALLOWED_CONFIG_CHARS).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize)]
#[serde(into = "String")]
pub struct Name(String);

impl Name {
    pub fn new<S: Into<String>>(s: S) -> std::result::Result<Self, ParsingError> {
        Self::try_from(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    fn is_valid_char(c: char) -> bool {
        ALLOWED_CONFIG_CHARS.contains(c)
    }
}

impl TryFrom<String> for Name {
    type Error = ParsingError;

    fn try_from(value: String) -> std::result::Result<Self, Self::Error> {
        if value.is_empty() {
            return Err(ParsingError::EmptyName);
        }
        if value.chars().all(Name::is_valid_char) {
            return Ok(Name(value));
        }
        Err(ParsingError::InvalidName(
            value,
            ALLOWED_CONFIG_CHARS.to_string(),
        ))
    }
}

impl<'de> Deserialize<'de> for Name {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::de::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Name::try_from(s).map_err(|err| serde::de::Error::custom(err.to_string()))
    }
}

impl From<Name> for String {
    fn from(v: Name) -> Self {
        v.0
    }
}

impl std::fmt::Display for Name {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl AsRef<str> for Name {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl PartialEq<&str> for Name {
    fn eq(&self, other: &&str) -> bool {
        self.0 == *other
    }
}

impl PartialEq<Name> for &str {
    fn eq(&self, other: &Name) -> bool {
        *self == other.0
    }
}

impl PartialEq<String> for Name {
    fn eq(&self, other: &String) -> bool {
        self.0 == *other
    }
}

impl PartialEq<Name> for String {
    fn eq(&self, other: &Name) -> bool {
        *self == other.0
    }
}

impl PartialOrd for Name {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Name {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

/// Fully-qualified producer address. The wire addresses a producer by the
/// `(core_node, instance_id)` pair — `instance_id` alone is only unique
/// within one stack, while the pair is unique across the whole mesh — so
/// every reference to a producer below the validator carries both halves.
/// The validator stamps `core_node` when it materializes bindings (the
/// `validate_bindings` pass in the peppyos `daemon-config` crate); after
/// that point a half-address is unrepresentable.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub struct ProducerRef {
    pub core_node: String,
    pub instance_id: String,
}

impl ProducerRef {
    pub fn new(core_node: impl Into<String>, instance_id: impl Into<String>) -> Self {
        Self {
            core_node: core_node.into(),
            instance_id: instance_id.into(),
        }
    }
}

/// Resolved per-slot binding for one of this consumer instance's declared
/// `depends_on` entries. The validator translates a launcher / CLI `(KEY,
/// VALUE)` binding map into this slot-keyed view — stamping each producer
/// with the launching daemon's `core_node` — before serializing into
/// `NodeInstanceConfig`, so the spawned node does no re-resolution work
/// and always holds wire-complete producer addresses.
///
/// `Pinned` corresponds to a `depends_on` entry with `from_any: false`;
/// it must be bound (the validator rejects pinned-unbound). `FromAnyBound`
/// is a `from_any: true` slot for which the user supplied one or more
/// bindings via free-form keys. `FromAnyUnbound` is a `from_any: true`
/// slot the user left bindless — the wildcard fallback for producers no
/// sibling slot has claimed.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SlotBinding {
    Pinned { producer: ProducerRef },
    FromAnyBound { producers: Vec<ProducerRef> },
    FromAnyUnbound,
}

/// State of one pairing slot (a `depends_on.pairings` entry) of a node
/// instance. Deliberately NOT a [`SlotBinding`] variant: slot bindings feed
/// the immutable consumer-filter cache, while a pairing slot is live-mutable
/// over the node's lifetime (the daemon delivers pins via the `peer_update`
/// service). In boot configs every declared slot is `Unpaired` — all pairs,
/// including those requested at `node run`, arrive over the live channel
/// after the instance commits to Running.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PairingSlotBinding {
    Paired {
        peer: ProducerRef,
        peer_link_id: String,
    },
    Unpaired,
}

/// Represents a node instance at runtime. Used by RuntimeConfig to identify the running node and its configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeInstanceConfig {
    pub instance_id: Name,
    #[serde(default)]
    pub arguments: BTreeMap<String, AnyType>,
    #[serde(default)]
    pub framework: ResolvedFramework,
    /// Pre-resolved per-slot bindings for every `link_id` declared in the
    /// consumer manifest's `depends_on`. Built by the validator from the
    /// launcher / CLI raw binding map plus the manifest depends_on (which
    /// distinguishes pinned vs `from_any` slots). Empty when the manifest
    /// has no `depends_on` entries. Read by the generated subscribe /
    /// poll / send_goal call sites via
    /// [`crate::runtime::ConsumerFilter`].
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub slot_bindings: BTreeMap<String, SlotBinding>,
    /// Boot-time state of every pairing slot declared in
    /// `depends_on.pairings`, keyed by slot link_id. Always maps each
    /// declared slot to [`PairingSlotBinding::Unpaired`] — pairs requested
    /// via `--pair` / launcher `pairings:` are delivered live over the
    /// `peer_update` service after the instance commits to Running, so
    /// there is exactly one delivery mechanism. Empty when the manifest
    /// declares no pairings.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub pairing_slots: BTreeMap<String, PairingSlotBinding>,
}

impl NodeInstanceConfig {
    /// Builds a config with everything except `instance_id` defaulted:
    /// empty arguments, default framework, empty slot bindings. Use with
    /// struct-update syntax to override a field:
    /// `NodeInstanceConfig { arguments, ..NodeInstanceConfig::new(id) }`.
    pub fn new(instance_id: Name) -> Self {
        Self {
            instance_id,
            arguments: BTreeMap::new(),
            framework: ResolvedFramework::default(),
            slot_bindings: BTreeMap::new(),
            pairing_slots: BTreeMap::new(),
        }
    }
}

/// Framework knobs already resolved by the daemon. Distinct from
/// the launcher-file `FrameworkOverrides` (peppyos `daemon-config`) so the type system enforces "resolution
/// happens once": the launcher form carries optional overrides; this form
/// carries concrete values the spawned node reads without further fallback.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResolvedFramework {
    #[serde(default)]
    pub use_sim_time: bool,
}

fn default_true() -> bool {
    true
}

fn default_standard_buffer_size() -> usize {
    crate::peppy_config::DEFAULT_STANDARD_BUFFER_SIZE
}

fn default_high_throughput_buffer_size() -> usize {
    crate::peppy_config::DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE
}

fn default_daemon_grace_secs() -> u64 {
    crate::peppy_config::DEFAULT_DAEMON_GRACE_SECS
}

fn default_shutdown_grace_secs() -> u64 {
    crate::peppy_config::DEFAULT_SHUTDOWN_GRACE_SECS
}

/// Node lifecycle settings the daemon resolves once (from `peppy_config.json5`)
/// and ships to each spawned node. `daemon_grace_secs` is the grace period the
/// node's daemon-liveness watchdog waits, after the daemon's heartbeat goes
/// silent, before shutting itself down — the uncatchable-death safety net.
/// `shutdown_grace_secs` is the cooperative-shutdown window: the daemon waits
/// this long for a stopping node to exit before SIGKILL, and the node runtime
/// bounds its registered shutdown hooks by the same window so cleanup can never
/// hang a stop (or outlive a dead daemon) indefinitely.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LifecycleRuntimeConfig {
    #[serde(default = "default_daemon_grace_secs")]
    pub daemon_grace_secs: u64,
    #[serde(default = "default_shutdown_grace_secs")]
    pub shutdown_grace_secs: u64,
}

impl Default for LifecycleRuntimeConfig {
    fn default() -> Self {
        Self {
            daemon_grace_secs: default_daemon_grace_secs(),
            shutdown_grace_secs: default_shutdown_grace_secs(),
        }
    }
}

impl LifecycleRuntimeConfig {
    fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// Messaging-session settings the daemon resolves once and ships to a node.
///
/// Nodes open a `peer` session that connects to a seed (the router) and then
/// forms direct peer-to-peer links with peers discovered via gossip, so data
/// stops relaying through the router. Discovery is gossip-only; there is no
/// multicast (it would bridge otherwise-independent peer groups on a shared
/// host, and a known seed already covers discovery).
///
/// The subscriber buffer sizes live here too. They are a subscriber-channel
/// concern rather than a discovery one, but co-locating them keeps a single
/// struct (and a single serialized block) travelling the daemon-to-node path,
/// since this is already the value threaded into the node's session at startup.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct DiscoveryConfig {
    /// Gossip seed endpoints (full Zenoh endpoints, e.g. `"tcp/127.0.0.1:7448"`).
    /// Empty means "derive the router endpoint from `messaging_host:messaging_port`".
    #[serde(default)]
    pub seed_peers: Vec<String>,
    /// Enable gossip so peers form direct links. Setting this to `false` forces
    /// all traffic through the router (a rollback switch without a rebuild).
    #[serde(default = "default_true")]
    pub gossip: bool,
    /// Subscriber channel buffer for the `Standard` QoS tier (in-flight messages).
    #[serde(default = "default_standard_buffer_size")]
    pub standard_buffer_size: usize,
    /// Subscriber channel buffer for the `HighThroughput` QoS tier (e.g. sensor data).
    #[serde(default = "default_high_throughput_buffer_size")]
    pub high_throughput_buffer_size: usize,
    /// Organization id stamped by the daemon so each spawned node opens its
    /// session under the same namespace as the daemon (routing isolation across
    /// the federation). `None` means "logged out" and resolves to the constant
    /// `local` namespace at session open. Stored as a plain string here
    /// (serde-simple); parsed into an `org::OrgNamespace` at the session-open
    /// boundary. Omitted from serialized configs when absent so configs written
    /// before this field existed still parse byte-identically.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub organization_id: Option<String>,
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            seed_peers: Vec::new(),
            gossip: true,
            standard_buffer_size: crate::peppy_config::DEFAULT_STANDARD_BUFFER_SIZE,
            high_throughput_buffer_size: crate::peppy_config::DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE,
            organization_id: None,
        }
    }
}

impl DiscoveryConfig {
    fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// This class is generated by the peppy daemon and then passed to each respective peppy node instances spawned by it
/// through `PEPPY_RUNTIME_CONFIG` env var. It's then deserialized in the process runtime to understand
/// how to communicate with the rest of the system.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeConfig {
    pub messaging_host: String,
    pub messaging_port: u16,
    pub node_name: Name,
    pub node_tag: Name,
    pub bound_core_node: Name,
    pub node_instance: NodeInstanceConfig,
    /// Peer-discovery settings. Defaulted (and omitted from serialized configs)
    /// for the common case, so launch configs written before this field existed
    /// still parse.
    #[serde(default, skip_serializing_if = "DiscoveryConfig::is_default")]
    pub discovery: DiscoveryConfig,
    /// Node lifecycle settings (daemon-liveness grace period). Defaulted and
    /// omitted from serialized configs for the common case, so launch configs
    /// written before this field existed still parse byte-identically.
    #[serde(default, skip_serializing_if = "LifecycleRuntimeConfig::is_default")]
    pub lifecycle: LifecycleRuntimeConfig,
}

impl RuntimeConfig {
    pub fn new(
        messaging_host: &str,
        messaging_port: u16,
        node_instance: NodeInstanceConfig,
        node_name: impl Into<String>,
        node_tag: impl Into<String>,
        bound_core_node: impl Into<String>,
    ) -> Result<Self> {
        Ok(Self {
            messaging_host: messaging_host.to_owned(),
            messaging_port,
            node_instance,
            node_name: Name::new(node_name.into())?,
            node_tag: Name::new(node_tag.into())?,
            bound_core_node: Name::new(bound_core_node.into())?,
            discovery: DiscoveryConfig::default(),
            lifecycle: LifecycleRuntimeConfig::default(),
        })
    }

    /// This function is typically invoked by the `peppy` program
    /// to persist its launch configuration for `peppylib` or `peppygen` to pick it up.
    pub fn save_json5_launch_config(&self, to_path: impl AsRef<Path>) -> Result<PathBuf> {
        let path = to_path.as_ref();
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let serialized = serde_json5::to_string(self)
            .map_err(|err| crate::error::Error::Serialize(err.to_string()))?;
        fs::write(path, serialized)?;
        Ok(path.to_path_buf())
    }

    pub fn generate_peppy_config_fingerprint(peppy_config: impl AsRef<Path>) -> Result<String> {
        use sha2::{Digest, Sha256};
        let config_path = peppy_config.as_ref();
        let content = std::fs::read(config_path)?;
        let hash = Sha256::digest(&content);
        Ok(hash
            .iter()
            .fold(String::with_capacity(hash.len() * 2), |mut acc, b| {
                use std::fmt::Write;
                let _ = write!(acc, "{:02x}", b);
                acc
            }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::Error;
    use tempfile::TempDir;

    #[test]
    fn name_validation() {
        assert!(Name::new("robot").is_ok());
        assert!(Name::new("camera_v1").is_ok());

        assert!(Name::new("").is_err()); // empty not permitted
        assert!(Name::new("/").is_err()); // slash not permitted
        assert!(Name::new("/robot").is_err()); // slash not permitted
        assert!(Name::new("Robot").is_ok()); // capital now allowed
        assert!(Name::new("robot$cam").is_err()); // special
    }

    #[test]
    fn name_error_message() {
        let err = Name::new("Invalid!").unwrap_err();
        if let ParsingError::InvalidName(_, msg) = err {
            assert_eq!(msg, crate::consts::ALLOWED_CONFIG_CHARS);
        } else {
            panic!("Expected InvalidName error");
        }
    }

    fn runtime_config_from_json(instance_id: &str) -> Result<RuntimeConfig> {
        let json = r#"{
            messaging_host: "$MESSAGING_HOST",
            messaging_port: $MESSAGING_PORT,
            node_instance: {
                instance_id: "$INSTANCE_ID"
            },
            node_name: "camera",
            node_tag: "v1",
            bound_core_node: "core_node"
        }"#;

        let populated = json
            .replace("$INSTANCE_ID", instance_id)
            .replace("$MESSAGING_HOST", "127.0.0.1")
            .replace("$MESSAGING_PORT", "7448");
        serde_json5::from_str(&populated).map_err(|err| Error::Parsing(err.into()))
    }

    /// `use_sim_time` round-trips through serialize/deserialize, and a
    /// runtime config written before this field existed (no `framework` key)
    /// still parses cleanly with `use_sim_time = false`.
    #[test]
    fn resolved_framework_round_trip_and_back_compat() {
        let with_sim: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: {
                    instance_id: "camera_front",
                    framework: { use_sim_time: true }
                },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node"
            }"#,
        )
        .unwrap();
        assert!(with_sim.node_instance.framework.use_sim_time);

        let serialized = serde_json5::to_string(&with_sim).unwrap();
        let reparsed: RuntimeConfig = serde_json5::from_str(&serialized).unwrap();
        assert!(reparsed.node_instance.framework.use_sim_time);

        let legacy = runtime_config_from_json("camera_front").unwrap();
        assert!(!legacy.node_instance.framework.use_sim_time);
    }

    /// A launch config written before `lifecycle` existed parses with the
    /// default grace period, an explicit block round-trips, and a default
    /// lifecycle is omitted from the serialized form so existing configs stay
    /// byte-identical.
    #[test]
    fn lifecycle_config_default_and_round_trip() {
        let legacy = runtime_config_from_json("camera_front").unwrap();
        assert_eq!(legacy.lifecycle, LifecycleRuntimeConfig::default());
        assert_eq!(
            legacy.lifecycle.daemon_grace_secs,
            crate::peppy_config::DEFAULT_DAEMON_GRACE_SECS
        );

        // Default lifecycle is skipped on serialize.
        let serialized = serde_json5::to_string(&legacy).unwrap();
        assert!(
            !serialized.contains("lifecycle"),
            "default lifecycle should not be serialized: {serialized}"
        );

        // A partial lifecycle block fills the missing field from its default
        // and an explicit block round-trips.
        let custom: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                lifecycle: { daemon_grace_secs: 42 }
            }"#,
        )
        .unwrap();
        assert_eq!(custom.lifecycle.daemon_grace_secs, 42);
        assert_eq!(
            custom.lifecycle.shutdown_grace_secs,
            crate::peppy_config::DEFAULT_SHUTDOWN_GRACE_SECS
        );
        let reparsed: RuntimeConfig =
            serde_json5::from_str(&serde_json5::to_string(&custom).unwrap()).unwrap();
        assert_eq!(reparsed.lifecycle, custom.lifecycle);

        let custom_shutdown: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                lifecycle: { shutdown_grace_secs: 7 }
            }"#,
        )
        .unwrap();
        assert_eq!(custom_shutdown.lifecycle.shutdown_grace_secs, 7);
        let reparsed: RuntimeConfig =
            serde_json5::from_str(&serde_json5::to_string(&custom_shutdown).unwrap()).unwrap();
        assert_eq!(reparsed.lifecycle, custom_shutdown.lifecycle);
    }

    /// A launch config written before `discovery` existed (no `discovery` key)
    /// parses with the gossip-on default, an explicit discovery block
    /// round-trips, and a default discovery is omitted from the serialized form
    /// so existing configs stay byte-identical.
    #[test]
    fn discovery_config_default_and_round_trip() {
        let legacy = runtime_config_from_json("camera_front").unwrap();
        assert_eq!(legacy.discovery, DiscoveryConfig::default());
        assert!(legacy.discovery.gossip);
        assert!(legacy.discovery.seed_peers.is_empty());
        // A launch config written before the buffer fields existed still parses
        // and gets the built-in defaults.
        assert_eq!(legacy.discovery.standard_buffer_size, 128);
        assert_eq!(legacy.discovery.high_throughput_buffer_size, 1024);

        // Default discovery is skipped on serialize.
        let serialized = serde_json5::to_string(&legacy).unwrap();
        assert!(
            !serialized.contains("discovery"),
            "default discovery should not be serialized: {serialized}"
        );

        // A discovery block that omits the buffer keys still parses (defaults).
        let no_buffers: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                discovery: { seed_peers: ["tcp/10.0.0.2:7448"], gossip: false }
            }"#,
        )
        .unwrap();
        assert_eq!(
            no_buffers.discovery.seed_peers,
            vec!["tcp/10.0.0.2:7448".to_string()]
        );
        assert!(!no_buffers.discovery.gossip);
        assert_eq!(no_buffers.discovery.standard_buffer_size, 128);
        assert_eq!(no_buffers.discovery.high_throughput_buffer_size, 1024);

        // Explicit buffer sizes round-trip.
        let custom: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                discovery: {
                    seed_peers: ["tcp/10.0.0.2:7448"],
                    gossip: false,
                    standard_buffer_size: 64,
                    high_throughput_buffer_size: 4096
                }
            }"#,
        )
        .unwrap();
        assert_eq!(custom.discovery.standard_buffer_size, 64);
        assert_eq!(custom.discovery.high_throughput_buffer_size, 4096);

        let reparsed: RuntimeConfig =
            serde_json5::from_str(&serde_json5::to_string(&custom).unwrap()).unwrap();
        assert_eq!(reparsed.discovery, custom.discovery);

        // A default discovery has no organization_id and omits it on serialize.
        assert!(DiscoveryConfig::default().organization_id.is_none());
        assert!(
            !serialized.contains("organization_id"),
            "absent organization_id should not be serialized: {serialized}"
        );

        // An explicit organization_id round-trips and is emitted on serialize.
        let with_org: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                discovery: { organization_id: "550e8400-e29b-41d4-a716-446655440000" }
            }"#,
        )
        .unwrap();
        assert_eq!(
            with_org.discovery.organization_id.as_deref(),
            Some("550e8400-e29b-41d4-a716-446655440000")
        );
        let org_serialized = serde_json5::to_string(&with_org).unwrap();
        assert!(
            org_serialized.contains("organization_id"),
            "an explicit organization_id should be serialized: {org_serialized}"
        );
        let reparsed: RuntimeConfig = serde_json5::from_str(&org_serialized).unwrap();
        assert_eq!(reparsed.discovery, with_org.discovery);
    }

    #[test]
    fn writes_launch_config_and_creates_parent_dir() {
        let dir = TempDir::new().unwrap();
        let path = dir.path().join("nested").join("peppy_launcher.json5");

        let config = runtime_config_from_json("camera_front").expect("runtime config should parse");
        let returned = config
            .save_json5_launch_config(&path)
            .expect("runtime config should write");

        let written = fs::read_to_string(&path).expect("launch config should be written to disk");
        let parsed: RuntimeConfig =
            serde_json5::from_str(&written).expect("launch config should parse");

        assert_eq!(returned, path);
        assert_eq!(parsed.node_name, "camera");
        assert_eq!(parsed.node_instance.instance_id, "camera_front");
        assert_eq!(parsed.bound_core_node, "core_node");
        assert_eq!(
            parsed.node_instance.instance_id,
            config.node_instance.instance_id
        );
        assert!(parsed.node_instance.arguments.is_empty());
    }

    /// Pin the wire contract of `SlotBinding`: it is internally tagged on
    /// `kind` with snake_case variants and field names, and every producer
    /// reference is the full `(core_node, instance_id)` pair. A rename or
    /// tag change here is a `graph_json` / launch-config wire break, so
    /// assert the exact JSON shape and that each variant round-trips back
    /// to itself.
    #[test]
    fn slot_binding_serde_contract() {
        use serde_json::json;

        let cases = [
            (
                SlotBinding::Pinned {
                    producer: ProducerRef::new("core_a", "p1"),
                },
                json!({
                    "kind": "pinned",
                    "producer": { "core_node": "core_a", "instance_id": "p1" }
                }),
            ),
            (
                SlotBinding::FromAnyBound {
                    producers: vec![
                        ProducerRef::new("core_a", "p3"),
                        ProducerRef::new("core_a", "p4"),
                    ],
                },
                json!({
                    "kind": "from_any_bound",
                    "producers": [
                        { "core_node": "core_a", "instance_id": "p3" },
                        { "core_node": "core_a", "instance_id": "p4" }
                    ]
                }),
            ),
            (
                SlotBinding::FromAnyUnbound,
                json!({ "kind": "from_any_unbound" }),
            ),
        ];
        for (value, expected) in cases {
            let encoded = serde_json::to_value(&value).expect("serialize SlotBinding");
            assert_eq!(encoded, expected, "SlotBinding JSON shape changed");
            let decoded: SlotBinding =
                serde_json::from_value(expected).expect("deserialize SlotBinding");
            assert_eq!(decoded, value, "SlotBinding did not round-trip");
        }
    }

    /// Half-addresses must be unrepresentable at the parse boundary: the
    /// pre-`ProducerRef` serialized shapes (instance_id-only) and a
    /// `producer` object missing `core_node` are hard parse errors, not
    /// defaulted values. No compatibility shims.
    #[test]
    fn slot_binding_rejects_half_address_payloads() {
        use serde_json::json;

        let rejected = [
            // Old pinned shape: instance_id without a core_node.
            json!({ "kind": "pinned", "producer_instance_id": "p1" }),
            // Old from_any_bound shape.
            json!({ "kind": "from_any_bound", "producer_instance_ids": ["p3", "p4"] }),
            // New field name but half an address.
            json!({ "kind": "pinned", "producer": { "instance_id": "p1" } }),
            json!({ "kind": "pinned", "producer": { "core_node": "core_a" } }),
            // Unknown extra field on the pair.
            json!({
                "kind": "pinned",
                "producer": { "core_node": "core_a", "instance_id": "p1", "extra": 1 }
            }),
        ];
        for payload in rejected {
            let result: std::result::Result<SlotBinding, _> =
                serde_json::from_value(payload.clone());
            assert!(
                result.is_err(),
                "half-address payload must fail to parse, but parsed: {payload}"
            );
        }
    }

    /// Pin the wire contract of `PairingSlotBinding`, mirroring the
    /// `SlotBinding` contract test: internally tagged on `kind`, snake_case,
    /// full `(core_node, instance_id)` peer address plus the peer's slot
    /// link_id. This shape travels boot configs and `stack list` output.
    #[test]
    fn pairing_slot_binding_serde_contract() {
        use serde_json::json;

        let cases = [
            (
                PairingSlotBinding::Paired {
                    peer: ProducerRef::new("core_a", "arm_1"),
                    peer_link_id: "controller".to_string(),
                },
                json!({
                    "kind": "paired",
                    "peer": { "core_node": "core_a", "instance_id": "arm_1" },
                    "peer_link_id": "controller"
                }),
            ),
            (PairingSlotBinding::Unpaired, json!({ "kind": "unpaired" })),
        ];
        for (value, expected) in cases {
            let encoded = serde_json::to_value(&value).expect("serialize PairingSlotBinding");
            assert_eq!(encoded, expected, "PairingSlotBinding JSON shape changed");
            let decoded: PairingSlotBinding =
                serde_json::from_value(expected).expect("deserialize PairingSlotBinding");
            assert_eq!(decoded, value, "PairingSlotBinding did not round-trip");
        }
    }

    /// A runtime config written before `pairing_slots` existed parses with an
    /// empty map, and an empty map is omitted on serialize so existing
    /// configs stay byte-identical.
    #[test]
    fn pairing_slots_default_and_round_trip() {
        let legacy = runtime_config_from_json("camera_front").unwrap();
        assert!(legacy.node_instance.pairing_slots.is_empty());
        let serialized = serde_json5::to_string(&legacy).unwrap();
        assert!(
            !serialized.contains("pairing_slots"),
            "empty pairing_slots should not be serialized: {serialized}"
        );

        let with_slots: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: {
                    instance_id: "ctrl_1",
                    pairing_slots: { arm: { kind: "unpaired" } }
                },
                node_name: "arm_controller",
                node_tag: "v1",
                bound_core_node: "core_node"
            }"#,
        )
        .unwrap();
        assert_eq!(
            with_slots.node_instance.pairing_slots.get("arm"),
            Some(&PairingSlotBinding::Unpaired)
        );
        let reparsed: RuntimeConfig =
            serde_json5::from_str(&serde_json5::to_string(&with_slots).unwrap()).unwrap();
        assert_eq!(
            reparsed.node_instance.pairing_slots,
            with_slots.node_instance.pairing_slots
        );
    }

    #[test]
    fn rejects_invalid_instance_id() {
        let dir = TempDir::new().unwrap();
        let path = dir.path().join("peppy_launcher.json5");

        let err = runtime_config_from_json("bad id!")
            .and_then(|config| config.save_json5_launch_config(&path))
            .unwrap_err();
        assert!(
            matches!(err, Error::Parsing(ParsingError::CannotParseConfig(ref msg)) if msg.contains("Invalid name"))
                || matches!(err, Error::Parsing(ParsingError::InvalidName(_, _))),
            "expected parsing error about invalid name, got: {err}"
        );
    }
}
