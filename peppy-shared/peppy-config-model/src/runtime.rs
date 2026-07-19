use crate::common::AnyType;
use crate::consts::ALLOWED_CONFIG_CHARS;
use crate::error::{ParsingError, Result};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
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
/// `validate_bindings` pass in the peppy `daemon-config` crate); after
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

/// The runtime-resolved, immutable, ordered producer set bound to one
/// consumer slot. Order is the application declaration order (launcher
/// array order / CLI flag occurrence order), preserved verbatim from the
/// validator through boot configs to the generated bound-producer
/// accessors, so selecting the first member is deterministic. Duplicates
/// are rejected rather than removed or
/// reordered. The set's validated size is the slot's declared
/// `cardinality`: exactly one for `one` (the default), one or more for
/// `one_or_more`, zero or more for `zero_or_more`; an empty set is a
/// valid value only for a `zero_or_more` slot and simply has no bound
/// edge. The set is fixed when the node starts; producers disconnecting
/// at runtime never shrink it.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
#[serde(transparent)]
pub struct BoundProducers(Vec<ProducerRef>);

impl BoundProducers {
    pub fn as_slice(&self) -> &[ProducerRef] {
        &self.0
    }

    pub fn iter(&self) -> std::slice::Iter<'_, ProducerRef> {
        self.0.iter()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn first(&self) -> Option<&ProducerRef> {
        self.0.first()
    }
}

/// A one-producer set, for `cardinality: "one"` slots and tests.
impl From<ProducerRef> for BoundProducers {
    fn from(producer: ProducerRef) -> Self {
        Self(vec![producer])
    }
}

/// Ordered construction from an already-collected target list, rejecting
/// duplicates. The single construction gate: the deserializer delegates
/// here, and the launcher validator calls it when it materializes a
/// slot's set, so every boundary rejects the same sets with the same
/// error.
impl TryFrom<Vec<ProducerRef>> for BoundProducers {
    type Error = ParsingError;

    fn try_from(producers: Vec<ProducerRef>) -> std::result::Result<Self, Self::Error> {
        // The first duplicated producer in declaration order names the error.
        let duplicate = {
            let mut seen = HashSet::with_capacity(producers.len());
            producers
                .iter()
                .find(|producer| !seen.insert(*producer))
                .cloned()
        };
        if let Some(duplicate) = duplicate {
            return Err(ParsingError::DuplicateBoundProducer {
                core_node: duplicate.core_node,
                instance_id: duplicate.instance_id,
            });
        }
        Ok(Self(producers))
    }
}

impl<'a> IntoIterator for &'a BoundProducers {
    type Item = &'a ProducerRef;
    type IntoIter = std::slice::Iter<'a, ProducerRef>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

/// Custom deserializer so the two failure shapes give actionable errors
/// instead of generic serde type mismatches: a duplicate producer names the
/// duplicated instance, and an object payload (the removed pre-cardinality
/// single-producer shape) is called out as component version skew, since
/// the daemon, CLI, generated bindings, and node runtime must be released
/// together across the cardinality break.
impl<'de> Deserialize<'de> for BoundProducers {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::de::Deserializer<'de>,
    {
        struct BoundProducersVisitor;

        impl<'de> serde::de::Visitor<'de> for BoundProducersVisitor {
            type Value = BoundProducers;

            fn expecting(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str("an ordered array of {core_node, instance_id} producers")
            }

            fn visit_seq<A>(self, mut seq: A) -> std::result::Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mut producers: Vec<ProducerRef> =
                    Vec::with_capacity(seq.size_hint().unwrap_or(0));
                while let Some(producer) = seq.next_element::<ProducerRef>()? {
                    producers.push(producer);
                }
                BoundProducers::try_from(producers).map_err(serde::de::Error::custom)
            }

            fn visit_map<A>(self, _map: A) -> std::result::Result<Self::Value, A::Error>
            where
                A: serde::de::MapAccess<'de>,
            {
                Err(serde::de::Error::custom(
                    "slot binding uses the removed single-producer object shape; since the \
                     cardinality release a slot binds an ordered ARRAY of producers (a \
                     `cardinality: \"one\"` slot binds a one-element array). The daemon, CLI, \
                     generated bindings, and node runtime must be upgraded together",
                ))
            }
        }

        deserializer.deserialize_any(BoundProducersVisitor)
    }
}

/// The slot-binding map that travels boot configs, `node_info` responses,
/// and the daemon graph: consumer slot `link_id` → the ordered producer
/// set explicitly bound to that slot. The launcher validator materializes
/// one entry per declared `depends_on.{nodes,contracts}` slot at plan
/// time, sized per the slot's `cardinality`; an empty set is valid only
/// for `zero_or_more` slots. Every member is a full wire address; there
/// is no wildcard, no unbound state, and no discovery fallback.
pub type SlotBindings = BTreeMap<String, BoundProducers>;

/// State of one pairing slot (a `depends_on.pairings` entry) of a node
/// instance. Deliberately NOT part of `slot_bindings`: slot bindings feed
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
    /// The pre-resolved producer for every `link_id` declared in the
    /// consumer manifest's `depends_on.{nodes,contracts}`. Built by the
    /// validator from the launcher / CLI binding map — each target stamped
    /// with the launching daemon's `core_node` — so the spawned node does
    /// no re-resolution work and always holds a wire-complete producer
    /// address. Empty when the manifest has no `depends_on` entries.
    /// Read by the generated subscribe / poll / send_goal call sites via
    /// the runtime's per-slot bound-producer cache.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub slot_bindings: SlotBindings,
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
/// the launcher-file `FrameworkOverrides` (peppy `daemon-config`) so the type system enforces "resolution
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
    /// Workspace namespace stamped by the daemon so each spawned node opens
    /// its session under the same namespace as the daemon (routing isolation
    /// across the platform federation). `None` means "logged out" and resolves
    /// to the constant `local` namespace at session open. Typed: an invalid
    /// value fails runtime-config parsing instead of leaking toward a live
    /// session. Omitted from serialized configs when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub namespace: Option<crate::internal::namespace::Namespace>,
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            seed_peers: Vec::new(),
            gossip: true,
            standard_buffer_size: crate::peppy_config::DEFAULT_STANDARD_BUFFER_SIZE,
            high_throughput_buffer_size: crate::peppy_config::DEFAULT_HIGH_THROUGHPUT_BUFFER_SIZE,
            namespace: None,
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

        // A default discovery has no namespace and omits it on serialize.
        assert!(DiscoveryConfig::default().namespace.is_none());
        assert!(
            !serialized.contains("namespace"),
            "absent namespace should not be serialized: {serialized}"
        );

        // An explicit namespace round-trips and is emitted on serialize.
        let with_namespace: RuntimeConfig = serde_json5::from_str(
            r#"{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: { instance_id: "camera_front" },
                node_name: "camera",
                node_tag: "v1",
                bound_core_node: "core_node",
                discovery: { namespace: "550e8400-e29b-41d4-a716-446655440000" }
            }"#,
        )
        .unwrap();
        assert_eq!(
            with_namespace
                .discovery
                .namespace
                .as_ref()
                .map(|n| n.as_str()),
            Some("550e8400-e29b-41d4-a716-446655440000")
        );
        let namespace_serialized = serde_json5::to_string(&with_namespace).unwrap();
        assert!(
            namespace_serialized.contains("namespace"),
            "an explicit namespace should be serialized: {namespace_serialized}"
        );
        let reparsed: RuntimeConfig = serde_json5::from_str(&namespace_serialized).unwrap();
        assert_eq!(reparsed.discovery, with_namespace.discovery);
    }

    #[test]
    fn runtime_config_rejects_the_legacy_organization_id_field() {
        let err = serde_json5::from_str::<RuntimeConfig>(
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
        .expect_err("the removed organization_id field must fail parsing");
        assert!(
            err.to_string().contains("organization_id"),
            "the parse error should name the rejected field: {err}"
        );
    }

    #[test]
    fn runtime_config_rejects_an_invalid_namespace() {
        assert!(
            serde_json5::from_str::<RuntimeConfig>(
                r#"{
                    messaging_host: "127.0.0.1",
                    messaging_port: 7448,
                    node_instance: { instance_id: "camera_front" },
                    node_name: "camera",
                    node_tag: "v1",
                    bound_core_node: "core_node",
                    discovery: { namespace: "**" }
                }"#,
            )
            .is_err(),
            "an invalid namespace must fail runtime-config parsing"
        );
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

    /// Pin the wire contract of `slot_bindings`: each slot maps its
    /// `link_id` to the ORDERED ARRAY of full `(core_node, instance_id)`
    /// producer pairs bound to it — a one-element array for a
    /// `cardinality: "one"` slot, an empty array for an unbound
    /// `zero_or_more` slot. A shape change here is a `graph_json` /
    /// launch-config wire break, so assert the exact JSON and that it
    /// round-trips with member order preserved.
    #[test]
    fn slot_bindings_serde_contract() {
        use serde_json::json;

        let bindings: SlotBindings = [
            (
                "main".to_string(),
                BoundProducers::from(ProducerRef::new("core_a", "p1")),
            ),
            (
                "camera".to_string(),
                BoundProducers::try_from(vec![
                    ProducerRef::new("core_a", "front_camera"),
                    ProducerRef::new("core_a", "rear_camera"),
                ])
                .expect("distinct producers"),
            ),
            ("spare".to_string(), BoundProducers::default()),
        ]
        .into_iter()
        .collect();

        let expected = json!({
            "camera": [
                { "core_node": "core_a", "instance_id": "front_camera" },
                { "core_node": "core_a", "instance_id": "rear_camera" }
            ],
            "main": [ { "core_node": "core_a", "instance_id": "p1" } ],
            "spare": []
        });

        let encoded = serde_json::to_value(&bindings).expect("serialize slot_bindings");
        assert_eq!(encoded, expected, "slot_bindings JSON shape changed");
        let decoded: SlotBindings =
            serde_json::from_value(expected).expect("deserialize slot_bindings");
        assert_eq!(decoded, bindings, "slot_bindings did not round-trip");
        assert_eq!(
            decoded
                .get("camera")
                .expect("camera slot")
                .iter()
                .map(|p| p.instance_id.as_str())
                .collect::<Vec<_>>(),
            ["front_camera", "rear_camera"],
            "member order must survive the round-trip"
        );
    }

    /// The removed pre-cardinality single-producer object shape must fail
    /// with a message that names the break as component version skew, not a
    /// generic serde type error: the daemon, CLI, generated bindings, and
    /// node runtime ship together across this wire change.
    #[test]
    fn slot_bindings_reject_pre_cardinality_object_shape_with_clear_error() {
        use serde_json::json;

        let legacy_shape = json!({
            "main": { "core_node": "core_a", "instance_id": "p1" }
        });
        let err = serde_json::from_value::<SlotBindings>(legacy_shape)
            .expect_err("object-shaped slot binding must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("single-producer object shape"),
            "error must name the removed shape: {msg}"
        );
        assert!(
            msg.contains("upgraded together"),
            "error must call out the version-skew fix: {msg}"
        );
    }

    /// Malformed members and duplicate producers are hard parse errors:
    /// half-addresses, unknown fields on a pair, non-object members, and a
    /// producer appearing twice within one slot's set.
    #[test]
    fn slot_bindings_reject_malformed_members_and_duplicates() {
        use serde_json::json;

        let rejected = [
            // Half an address.
            json!([{ "instance_id": "p1" }]),
            json!([{ "core_node": "core_a" }]),
            // Unknown extra field on a pair.
            json!([{ "core_node": "core_a", "instance_id": "p1", "extra": 1 }]),
            // A bare string is not a producer pair.
            json!(["p1"]),
            // Duplicate producer within one slot.
            json!([
                { "core_node": "core_a", "instance_id": "p1" },
                { "core_node": "core_a", "instance_id": "p1" }
            ]),
        ];
        for payload in rejected {
            let result: std::result::Result<BoundProducers, _> =
                serde_json::from_value(payload.clone());
            assert!(
                result.is_err(),
                "payload must fail to parse as a slot's bound set, but parsed: {payload}"
            );
            let map_payload = json!({ "slot": payload });
            let map_result: std::result::Result<SlotBindings, _> =
                serde_json::from_value(map_payload.clone());
            assert!(
                map_result.is_err(),
                "payload must fail to parse inside slot_bindings, but parsed: {map_payload}"
            );
        }

        // The duplicate error names the duplicated producer.
        let dup = json!([
            { "core_node": "core_a", "instance_id": "front_camera" },
            { "core_node": "core_a", "instance_id": "front_camera" }
        ]);
        let msg = serde_json::from_value::<BoundProducers>(dup)
            .expect_err("duplicate must be rejected")
            .to_string();
        assert!(
            msg.contains("front_camera@core_a"),
            "duplicate error must name the producer: {msg}"
        );

        // Same-instance producers on different core nodes are distinct, not
        // duplicates.
        let cross_core = json!([
            { "core_node": "core_a", "instance_id": "cam" },
            { "core_node": "core_b", "instance_id": "cam" }
        ]);
        let parsed: BoundProducers =
            serde_json::from_value(cross_core).expect("distinct core nodes must parse");
        assert_eq!(parsed.len(), 2);
    }

    /// `BoundProducers::try_from` mirrors the deserializer: declaration
    /// order is preserved and duplicates are rejected (not deduplicated).
    #[test]
    fn bound_producers_try_from_preserves_order_and_rejects_duplicates() {
        let ordered = BoundProducers::try_from(vec![
            ProducerRef::new("core_a", "rear_camera"),
            ProducerRef::new("core_a", "front_camera"),
        ])
        .expect("distinct producers");
        assert_eq!(
            ordered
                .iter()
                .map(|p| p.instance_id.as_str())
                .collect::<Vec<_>>(),
            ["rear_camera", "front_camera"],
            "declaration order must be preserved, not sorted"
        );
        assert_eq!(
            ordered.first().map(|p| p.instance_id.as_str()),
            Some("rear_camera")
        );

        let err = BoundProducers::try_from(vec![
            ProducerRef::new("core_a", "cam"),
            ProducerRef::new("core_a", "cam"),
        ])
        .expect_err("duplicates must be rejected");
        let ParsingError::DuplicateBoundProducer {
            core_node,
            instance_id,
        } = err
        else {
            panic!("expected DuplicateBoundProducer, got {err:?}");
        };
        assert_eq!(core_node, "core_a");
        assert_eq!(instance_id, "cam");
    }

    /// Pin the wire contract of `PairingSlotBinding` (contrast with the
    /// plain-array `slot_bindings` shape pinned above): internally tagged on
    /// `kind`, snake_case, full `(core_node, instance_id)` peer address plus
    /// the peer's slot link_id. This shape travels boot configs and
    /// `stack list` output.
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
                    pairing_slots: {
                        arm: { kind: "unpaired" },
                        gripper: {
                            kind: "paired",
                            peer: { core_node: "core_a", instance_id: "grip_1" },
                            peer_link_id: "controller"
                        }
                    }
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
        assert_eq!(
            with_slots.node_instance.pairing_slots.get("gripper"),
            Some(&PairingSlotBinding::Paired {
                peer: ProducerRef::new("core_a", "grip_1"),
                peer_link_id: "controller".to_string(),
            })
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
