use std::path::Path;

use crate::error::{Error, ParameterDeserializationError, Result};
use crate::messaging::{PeerPin, PeerPinState};
use config::{
    AnyType, NodeArguments,
    consts::{PEPPYGEN_OUTPUT_PATH, RUNTIME_CONFIG_VAR_NAME},
    node::{NodeConfig, load_standalone_node_config},
    runtime::{Name, NodeInstanceConfig, PairingSlotBinding, RuntimeConfig},
    validate_node_arguments,
};
use std::collections::BTreeMap;
use std::sync::Arc;
use tokio::sync::watch;

use super::builder::StandaloneConfig;

/// Runtime processor that holds configuration for the node.
#[derive(Clone)]
pub struct Processor {
    runtime_config: RuntimeConfig,
    validated_arguments: NodeArguments,
    /// The ordered bound producer set per declared `link_id`, sized per the
    /// slot's declared cardinality. Computed once at startup from the
    /// daemon-supplied `slot_bindings` plus the manifest's `depends_on`;
    /// immutable for the node's lifetime (a producer disconnecting never
    /// shrinks it), and cached so subscribe / poll / send_goal call sites
    /// return a borrowed slice cheaply.
    bound_producers: BTreeMap<String, Vec<crate::messaging::ProducerRef>>,
    /// One live pairing-slot channel per `depends_on.pairings` entry, keyed
    /// by the slot's link_id. The map's key set is fixed at startup (slots
    /// are declared in the manifest); only the channel values move — the
    /// daemon mutates them over the `peer_update` service, and per-slot
    /// `PeerSubscription`s / `PeerSlot`s observe them. Behind an `Arc` so
    /// `Processor::clone` shares the live channels instead of forking them.
    pairing_slots: Arc<BTreeMap<String, watch::Sender<PeerPinState>>>,
}

impl Processor {
    /// Create processor for daemon mode.
    ///
    /// Resolves the launch-config path from the `PEPPY_RUNTIME_CONFIG` env var
    /// (set by the CLI when it launches the node) and delegates to
    /// [`new_daemon_from_path`](Self::new_daemon_from_path). Validates the
    /// fingerprint matches compiled code.
    pub fn new_daemon(peppy_config: impl AsRef<Path>) -> Result<Self> {
        let launch_config_path = std::env::var(RUNTIME_CONFIG_VAR_NAME).map_err(|source| {
            Error::MissingInstanceIdEnvVar {
                var: RUNTIME_CONFIG_VAR_NAME,
                source,
            }
        })?;

        Self::new_daemon_from_path(peppy_config, &launch_config_path)
    }

    /// Create processor for daemon mode from an explicit launch-config path.
    ///
    /// This is the env-free core that [`new_daemon`](Self::new_daemon) wraps:
    /// the launch-config path is passed in rather than read from the
    /// environment, so callers that already know the path (embedders, tests)
    /// can construct a daemon-mode processor without touching process state.
    /// Validates the fingerprint matches compiled code.
    pub fn new_daemon_from_path(
        peppy_config: impl AsRef<Path>,
        launch_config_path: &str,
    ) -> Result<Self> {
        let runtime_config = Self::load_runtime_config(launch_config_path)?;

        let codegen_fingerprint = config::fingerprint::read_codegen_fingerprint(
            peppy_config.as_ref(),
            PEPPYGEN_OUTPUT_PATH,
        )
        .map_err(|source| Error::CodegenFingerprintRead {
            path: peppy_config.as_ref().display().to_string(),
            source: std::io::Error::other(source.to_string()),
        })?;
        Self::validate_fingerprint(peppy_config.as_ref(), &codegen_fingerprint)?;

        let node_config: NodeConfig =
            serde_json5::from_str(&std::fs::read_to_string(peppy_config.as_ref())?)?;
        let validated_arguments = validate_node_arguments(
            runtime_config.node_instance.arguments.clone(),
            &node_config.execution.parameters,
        )?;

        let bound_producers = build_bound_producers(&runtime_config, &node_config)?;
        let pairing_slots = build_pairing_slots(&runtime_config, &node_config);

        Ok(Self {
            runtime_config,
            validated_arguments,
            bound_producers,
            pairing_slots,
        })
    }

    /// Create processor for standalone mode.
    ///
    /// Uses provided configuration or defaults:
    /// - messaging_host: DEFAULT_ZENOH_HOST or user-provided
    /// - messaging_port: DEFAULT_ZENOH_PORT or user-provided
    /// - instance_id: "standalone" or user-provided
    /// - node_name: from peppy.json5 or user-provided
    ///
    /// Skips fingerprint validation for development flexibility.
    pub fn new_standalone(
        peppy_config: impl AsRef<Path>,
        config: &StandaloneConfig,
    ) -> Result<Self> {
        let node_config: NodeConfig = load_standalone_node_config(peppy_config.as_ref())?;

        let arguments: BTreeMap<String, AnyType> = match &config.parameters {
            Some(params) => serde_json::from_value(params.clone()).map_err(|e| {
                ParameterDeserializationError::single(format!("failed to parse parameters: {}", e))
            })?,
            None => BTreeMap::new(),
        };

        let validated_arguments =
            validate_node_arguments(arguments, &node_config.execution.parameters)?;

        let node_name: String = config
            .node_name
            .clone()
            .unwrap_or_else(|| node_config.manifest.name.clone().into());

        let instance_id = config
            .instance_id
            .clone()
            .unwrap_or_else(|| "standalone".to_string());

        let messaging_host = config.messaging_host_or_default();
        let messaging_port = config.messaging_port_or_default();

        let instance_id_name =
            Name::new(instance_id.clone()).map_err(|e| Error::InvalidNodeName {
                node_name: instance_id,
                reason: e.to_string(),
            })?;

        // Daemon-less development: `StandaloneConfig::with_bound_producer`
        // seeds a consumer slot's producer set, standing in for the
        // launcher's validated binding map. Undeclared link_ids are ignored
        // with a warning (mirroring `peer_pins` below); a declared slot
        // whose seeded set violates its cardinality fails
        // `build_bound_producers`, exactly like a daemon boot config with a
        // bad binding. Duplicate seeded producers are rejected here, where
        // `BoundProducers::try_from` mirrors the boot-config parse rule.
        let mut slot_bindings = config::runtime::SlotBindings::new();
        let declared_slots: std::collections::BTreeSet<&str> = node_config
            .manifest
            .depends_on
            .as_ref()
            .map(|deps| {
                deps.nodes
                    .iter()
                    .map(|dep| dep.link_id.as_str())
                    .chain(deps.contracts.iter().map(|dep| dep.link_id.as_str()))
                    .collect()
            })
            .unwrap_or_default();
        for (link_id, producers) in &config.bound_producers {
            if !declared_slots.contains(link_id.as_str()) {
                tracing::warn!(
                    link_id = %link_id,
                    "StandaloneConfig bound producer names an undeclared consumer slot; ignoring"
                );
                continue;
            }
            let bound = config::runtime::BoundProducers::try_from(producers.clone())
                .map_err(config::ConfigError::from)?;
            slot_bindings.insert(link_id.clone(), bound);
        }

        let runtime_config = RuntimeConfig::new(
            &messaging_host,
            messaging_port,
            NodeInstanceConfig {
                slot_bindings,
                ..NodeInstanceConfig::new(instance_id_name)
            },
            &node_name,
            node_config.manifest.tag.as_str(),
            "standalone-core",
        )?;

        let bound_producers = build_bound_producers(&runtime_config, &node_config)?;
        let pairing_slots = build_pairing_slots(&runtime_config, &node_config);

        // Daemon-less development: `StandaloneConfig::with_peer_pin` seeds a
        // slot as already-paired, standing in for the daemon's live
        // `peer_update` delivery. `send_replace`, not `send`: no receiver
        // exists yet (the first `peer(...)` subscribes later), and `send`
        // discards the value on a receiver-less channel.
        for (link_id, pin) in &config.peer_pins {
            if let Some(sender) = pairing_slots.get(link_id) {
                sender.send_replace(PeerPinState {
                    sequence: 1,
                    pin: Some(pin.clone()),
                });
            } else {
                tracing::warn!(
                    link_id = %link_id,
                    "StandaloneConfig peer pin names an undeclared pairing slot; ignoring"
                );
            }
        }

        Ok(Self {
            runtime_config,
            validated_arguments,
            bound_producers,
            pairing_slots,
        })
    }

    fn load_runtime_config(path: &str) -> Result<RuntimeConfig> {
        let content = std::fs::read_to_string(path).map_err(|source| Error::LaunchConfigRead {
            path: path.to_string(),
            source,
        })?;
        serde_json5::from_str(&content).map_err(|source| Error::LaunchConfigParse {
            path: path.to_string(),
            source,
        })
    }

    fn validate_fingerprint(peppy_config: &Path, expected: &str) -> Result<()> {
        let actual = RuntimeConfig::generate_peppy_config_fingerprint(peppy_config)?;
        if actual == expected {
            Ok(())
        } else {
            Err(Error::PeppyConfigFingerprintMismatch {
                path: peppy_config.display().to_string(),
                expected: expected.to_string(),
                actual,
            })
        }
    }

    pub fn bound_instance_id(&self) -> &str {
        self.runtime_config.node_instance.instance_id.as_str()
    }

    pub fn bound_core_node(&self) -> &str {
        self.runtime_config.bound_core_node.as_str()
    }

    pub(crate) fn input_arguments(&self) -> &NodeArguments {
        &self.validated_arguments
    }

    pub fn node_name(&self) -> &str {
        self.runtime_config.node_name.as_str()
    }

    pub fn node_tag(&self) -> &str {
        self.runtime_config.node_tag.as_str()
    }

    pub fn messaging_host(&self) -> &str {
        &self.runtime_config.messaging_host
    }

    pub fn messaging_port(&self) -> u16 {
        self.runtime_config.messaging_port
    }

    /// Peer-discovery settings for this node's messaging session.
    pub(crate) fn discovery(&self) -> &config::runtime::DiscoveryConfig {
        &self.runtime_config.discovery
    }

    /// Grace period this node's daemon-liveness watchdog waits, after the
    /// daemon's heartbeat goes silent, before shutting the node down. Resolved
    /// by the daemon from `peppy_config.json5` and shipped in the runtime
    /// config. Read by [`crate::services::daemon_watchdog`].
    pub fn daemon_grace(&self) -> std::time::Duration {
        std::time::Duration::from_secs(self.runtime_config.lifecycle.daemon_grace_secs)
    }

    /// Cooperative-shutdown grace window. The daemon waits this long for a
    /// stopping node to exit before force-killing it, and the node runtime
    /// bounds its registered shutdown hooks
    /// ([`crate::runtime::NodeRunner::on_shutdown`]) by the same window.
    /// Resolved by the daemon from `peppy_config.json5` and shipped in the
    /// runtime config; standalone nodes use the built-in default.
    pub fn shutdown_grace(&self) -> std::time::Duration {
        std::time::Duration::from_secs(self.runtime_config.lifecycle.shutdown_grace_secs)
    }

    /// Daemon-resolved framework `use_sim_time` flag for this instance.
    /// Read by [`crate::clock::for_node`] to pick between
    /// the wall-time and sim-time `PeppyClock` implementations.
    pub fn use_sim_time(&self) -> bool {
        self.runtime_config.node_instance.framework.use_sim_time
    }

    /// The runtime-resolved, immutable, ordered producer set bound to the
    /// consumer slot declared at `link_id`, from the daemon-supplied
    /// `slot_bindings`, cached once at startup for the lifetime of the
    /// node. Serves topic subscribes and service / action calls alike:
    /// every interface kind sharing the slot's `link_id` sees the same set
    /// in the same declaration order, so `.first()` is deterministic. The
    /// set's validated size is the slot's declared cardinality (exactly one
    /// for `one`, at least one for `one_or_more`, possibly empty only for
    /// `zero_or_more`); a producer disconnecting at runtime never shrinks
    /// it. Startup validates every declared slot into the cache, so a cache
    /// miss means the generated code and the manifest disagree (version
    /// skew / stale codegen) — a bug, not a user error — and panics.
    ///
    /// The generated accessors are cardinality-typed: only `zero_or_more`
    /// slots' `bound_producers()` splice this plain, possibly empty slice
    /// directly. `one` slots go through [`Self::sole_bound_producer`] and
    /// `one_or_more` slots through [`Self::non_empty_bound_producers`];
    /// every generated `subscribe()` passes this slice to
    /// `subscribe_bound_set` regardless of cardinality.
    pub fn bound_producers(&self, link_id: &str) -> &[crate::messaging::ProducerRef] {
        self.bound_producers.get(link_id).unwrap_or_else(|| {
            panic!(
                "consumer slot `{link_id}` has no cached producer set: the generated code and \
                 the manifest disagree (version skew / stale codegen) — regenerate bindings \
                 for this node"
            )
        })
    }

    /// The sole producer bound to a `cardinality: "one"` consumer slot.
    /// Startup validated every slot's set size against its declared
    /// cardinality, so exactly one member exists; any other size here means
    /// the generated code and the manifest disagree (version skew / stale
    /// codegen), a bug rather than a user error, and panics just like an
    /// unknown `link_id` in [`Self::bound_producers`].
    ///
    /// Generated `bound_producer()` module functions of `one` slots splice
    /// `node_runner.processor().sole_bound_producer(<link_id>)`.
    pub fn sole_bound_producer(&self, link_id: &str) -> &crate::messaging::ProducerRef {
        match self.bound_producers(link_id) {
            [sole] => sole,
            set => panic!(
                "consumer slot `{link_id}` is bound to {} producers but the generated accessor \
                 expects cardinality `one`: the generated code and the manifest disagree \
                 (version skew / stale codegen); regenerate bindings for this node",
                set.len()
            ),
        }
    }

    /// The producer set bound to a `cardinality: "one_or_more"` consumer
    /// slot, as a [`NonEmptyProducers`](crate::messaging::NonEmptyProducers)
    /// view whose `first()` is infallible. Startup validated the set as
    /// non-empty; an empty set here means the generated code and the
    /// manifest disagree (version skew / stale codegen), a bug rather than
    /// a user error, and panics just like an unknown `link_id` in
    /// [`Self::bound_producers`].
    ///
    /// Generated `bound_producers()` module functions of `one_or_more`
    /// slots splice
    /// `node_runner.processor().non_empty_bound_producers(<link_id>)`.
    pub fn non_empty_bound_producers(
        &self,
        link_id: &str,
    ) -> crate::messaging::NonEmptyProducers<'_> {
        crate::messaging::NonEmptyProducers::new(self.bound_producers(link_id)).unwrap_or_else(
            || {
                panic!(
                    "consumer slot `{link_id}` is bound to an empty set but the generated \
                     accessor expects cardinality `one_or_more`: the generated code and the \
                     manifest disagree (version skew / stale codegen); regenerate bindings for \
                     this node"
                )
            },
        )
    }

    /// Checks that `target` is a member of the bound set of the slot
    /// declared at `link_id`, returning
    /// [`Error::TargetNotBound`](crate::error::Error::TargetNotBound)
    /// otherwise. Generated service `poll` / action `fire_goal` wrappers
    /// call this before anything reaches the wire: `ProducerRef` is plainly
    /// constructible, and an out-of-set instance was never checked by
    /// plan-time binding validation, so letting it through would reopen
    /// undirected `from_any`-style calls. Membership is per slot — a
    /// producer bound to a different slot of the same consumer is rejected
    /// all the same.
    pub fn ensure_target_bound(
        &self,
        link_id: &str,
        target: &crate::messaging::ProducerRef,
    ) -> Result<()> {
        if self.bound_producers(link_id).contains(target) {
            return Ok(());
        }
        Err(Error::TargetNotBound {
            link_id: link_id.to_string(),
            core_node: target.core_node.clone(),
            instance_id: target.instance_id.clone(),
        })
    }

    /// The live watch channel for the pairing slot declared at `link_id`, or
    /// `None` when the manifest declares no such slot. Used by
    /// [`crate::runtime::NodeRunner::peer`] and the generated
    /// `subscribe_peer` seam to observe pins; the `peer_update` service uses
    /// the sender side via [`Self::pairing_slot_senders`].
    pub(crate) fn peer_pin_watch(&self, link_id: &str) -> Option<watch::Receiver<PeerPinState>> {
        self.pairing_slots.get(link_id).map(|tx| tx.subscribe())
    }

    /// Shared handle to all pairing-slot channels, handed to the pre-setup
    /// `peer_update` service listener.
    pub(crate) fn pairing_slot_senders(
        &self,
    ) -> Arc<BTreeMap<String, watch::Sender<PeerPinState>>> {
        Arc::clone(&self.pairing_slots)
    }
}

/// Seed one watch channel per pairing slot declared in
/// `depends_on.pairings`. The initial value comes from the boot config's
/// `pairing_slots` map when present (the daemon always ships `Unpaired` —
/// pairs arrive live over `peer_update` — but the mapping is honored so the
/// boot contract stays a plain data translation), defaulting to `Unpaired`.
fn build_pairing_slots(
    runtime_config: &RuntimeConfig,
    node_config: &NodeConfig,
) -> Arc<BTreeMap<String, watch::Sender<PeerPinState>>> {
    let mut out = BTreeMap::new();
    if let Some(deps) = node_config.manifest.depends_on.as_ref() {
        for dep in &deps.pairings {
            let initial = match runtime_config
                .node_instance
                .pairing_slots
                .get(dep.link_id.as_str())
            {
                Some(PairingSlotBinding::Paired { peer, peer_link_id }) => PeerPinState {
                    sequence: 0,
                    pin: Some(PeerPin {
                        producer: crate::messaging::ProducerRef::new(
                            peer.core_node.clone(),
                            peer.instance_id.clone(),
                        ),
                        peer_link_id: peer_link_id.clone(),
                    }),
                },
                Some(PairingSlotBinding::Unpaired) | None => PeerPinState::unpaired(),
            };
            let (tx, _rx) = watch::channel(initial);
            out.insert(dep.link_id.clone(), tx);
        }
    }
    Arc::new(out)
}

/// Pre-resolve the ordered bound producer set for every `link_id` declared
/// in the consumer manifest's `depends_on`, from the daemon-supplied
/// `slot_bindings`, enforcing each slot's declared cardinality:
///
/// - `one`: the slot's entry must hold exactly one producer; a missing
///   entry is [`Error::SlotUnbound`], a wrong-sized one is
///   [`Error::SlotCardinalityViolated`].
/// - `one_or_more`: the entry must hold at least one producer.
/// - `zero_or_more`: a missing entry and an empty entry are both the valid
///   empty set.
///
/// The launcher validator enforces the same rules at plan time, so a
/// violation here means version skew or a hand-edited boot config. Called
/// once during [`Processor::new_daemon`] / [`Processor::new_standalone`] so
/// the per-link_id accessor is a borrow into a stable cache. Member order
/// is preserved verbatim from the boot config (application declaration
/// order).
fn build_bound_producers(
    runtime_config: &RuntimeConfig,
    node_config: &NodeConfig,
) -> Result<BTreeMap<String, Vec<crate::messaging::ProducerRef>>> {
    let mut out = BTreeMap::new();
    if let Some(deps) = node_config.manifest.depends_on.as_ref() {
        let slot_bindings = &runtime_config.node_instance.slot_bindings;
        let node_slots = deps.nodes.iter().map(|dep| (&dep.link_id, dep.cardinality));
        let contract_slots = deps
            .contracts
            .iter()
            .map(|dep| (&dep.link_id, dep.cardinality));
        for (link_id, cardinality) in node_slots.chain(contract_slots) {
            let producers: Vec<crate::messaging::ProducerRef> = match slot_bindings.get(link_id) {
                Some(bound) => bound.iter().cloned().collect(),
                None if cardinality.allows_empty() => Vec::new(),
                None => {
                    return Err(Error::SlotUnbound {
                        link_id: link_id.clone(),
                        cardinality: cardinality.as_str(),
                    });
                }
            };
            let size_ok = match cardinality {
                config::node::Cardinality::One => producers.len() == 1,
                config::node::Cardinality::OneOrMore => !producers.is_empty(),
                config::node::Cardinality::ZeroOrMore => true,
            };
            if !size_ok {
                return Err(Error::SlotCardinalityViolated {
                    link_id: link_id.clone(),
                    cardinality: cardinality.as_str(),
                    bound: producers.len(),
                });
            }
            out.insert(link_id.clone(), producers);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::{PEPPYGEN_OUTPUT_PATH, Processor};
    use crate::runtime::builder::StandaloneConfig;
    use config::node::TypeToken;
    use config::{
        AnyType, ParameterSchema, ParameterSpec, runtime::RuntimeConfig, validate_node_arguments,
    };
    use std::{collections::BTreeMap, path::Path};
    use tempfile::TempDir;

    #[test]
    fn loads_runtime_config_from_path() {
        let bound_core_node = "epic-whale-6789";
        let bound_node_name = "uvc_camera";
        let bound_instance_id = "camera_front";

        let temp_dir = TempDir::new().expect("temp dir should be created");

        // Create a peppy config file with type specifications matching runtime parameters
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "uvc_camera",
                tag: "v1",
            },
            execution: {
                language: "rust",
                parameters: {
                    exposure: "f32",
                    flags: {
                        $type: "array",
                        $items: "string"
                    },
                    nested: {
                        $type: "object",
                        enabled: "bool",
                        gain: "i64"
                    },
                    mode: "string"
                },
                run_cmd: ["./target/debug/uvc_camera"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "$INSTANCE_ID",
                arguments: {
                    exposure: 0.25,
                    flags: ["hdr", "stabilized"],
                    nested: { enabled: true, gain: 10 },
                    mode: "auto"
                }
            },
            node_name: "$NODE_NAME",
            node_tag: "v1",
            bound_core_node: "$CORE_NODE"
        }"#;

        let populated_config = json5_config
            .replace("$INSTANCE_ID", bound_instance_id)
            .replace("$NODE_NAME", bound_node_name)
            .replace("$CORE_NODE", bound_core_node);

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&populated_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let runtime_processor = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path
                .to_str()
                .expect("runtime config path should be valid UTF-8"),
        )
        .expect("runtime processor should load config from path");

        let mut expected_parameters: BTreeMap<String, AnyType> = BTreeMap::new();
        expected_parameters.insert("exposure".into(), AnyType::Float(0.25));
        expected_parameters.insert(
            "flags".into(),
            AnyType::Array(vec![
                AnyType::String("hdr".into()),
                AnyType::String("stabilized".into()),
            ]),
        );
        expected_parameters.insert(
            "nested".into(),
            AnyType::Object(BTreeMap::from([
                ("enabled".to_string(), AnyType::Bool(true)),
                ("gain".to_string(), AnyType::Int(10)),
            ])),
        );
        expected_parameters.insert("mode".into(), AnyType::String("auto".into()));

        assert_eq!(runtime_processor.bound_instance_id(), bound_instance_id);
        assert_eq!(runtime_processor.bound_core_node(), bound_core_node);
        assert_eq!(runtime_processor.node_name(), bound_node_name);
        assert_eq!(
            serde_json::to_value(runtime_processor.input_arguments()).unwrap(),
            serde_json::to_value(&expected_parameters).unwrap(),
        );
    }

    #[test]
    fn fails_when_codegen_fingerprint_mismatch() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/test_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_wrong_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { value: 42 }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#;

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected fingerprint mismatch error");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("fingerprint mismatch"),
            "expected fingerprint mismatch error, got: {err_string}"
        );
    }

    #[test]
    fn fails_when_runtime_parameter_missing_in_compiled_config() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        // Compiled config only has 'value' parameter
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/test_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        // Runtime config has 'value' AND 'extra_param' - but 'extra_param' is not in compiled
        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { value: 42, extra_param: "unexpected" }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#
        .to_string();

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected missing parameter error");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("unknown parameter") && err_string.contains("extra_param"),
            "expected unknown parameter error for 'extra_param', got: {err_string}"
        );
    }

    #[test]
    fn fails_when_parameter_type_mismatch() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        // Compiled config expects 'value' to be i64
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/test_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        // Runtime config provides 'value' as a string instead of i64
        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { value: "not_an_integer" }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#
        .to_string();

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected type mismatch error");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("type mismatch") && err_string.contains("value"),
            "expected type mismatch error for 'value', got: {err_string}"
        );
    }

    #[test]
    fn fails_when_nested_parameter_type_mismatch() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        // Compiled config expects nested object with specific types
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: {
                language: "rust",
                parameters: {
                    config: {
                        $type: "object",
                        enabled: "bool",
                        threshold: "f64"
                    }
                },
                run_cmd: ["./target/debug/test_node"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        // Runtime config provides 'enabled' as string instead of bool
        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { config: { enabled: "yes", threshold: 0.5 } }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#
        .to_string();

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected type mismatch error");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("type mismatch") && err_string.contains("config.enabled"),
            "expected type mismatch error for 'config.enabled', got: {err_string}"
        );
    }

    #[test]
    fn fails_when_array_item_type_mismatch() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        // Compiled config expects array of strings
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: {
                language: "rust",
                parameters: {
                    tags: {
                        $type: "array",
                        $items: "string"
                    }
                },
                run_cmd: ["./target/debug/test_node"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        // Runtime config provides array with mixed types (string and int)
        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { tags: ["valid", 123, "also_valid"] }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#
        .to_string();

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected type mismatch error");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("type mismatch") && err_string.contains("tags[1]"),
            "expected type mismatch error for 'tags[1]', got: {err_string}"
        );
    }

    #[test]
    fn fails_when_codegen_fingerprint_missing() {
        use crate::error::Error;

        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/test_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");
        // Note: intentionally NOT creating the fingerprint file

        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "test_instance",
                arguments: { value: 42 }
            },
            node_name: "test_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#;

        let runtime_config: RuntimeConfig =
            serde_json5::from_str(json5_config).expect("runtime config should parse");

        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        let Err(err) = Processor::new_daemon_from_path(
            &peppy_config_path,
            runtime_config_path.to_str().unwrap(),
        ) else {
            panic!("expected codegen fingerprint read error");
        };
        assert!(
            matches!(err, Error::CodegenFingerprintRead { .. }),
            "expected CodegenFingerprintRead error, got: {err:?}"
        );
    }

    #[test]
    fn standalone_mode_uses_manifest_node_name() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config = StandaloneConfig::new();
        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("should create processor");

        assert_eq!(processor.node_name(), "my_node");
        assert_eq!(processor.bound_instance_id(), "standalone");
        assert_eq!(processor.bound_core_node(), "standalone-core");
        assert_eq!(processor.messaging_host(), "127.0.0.1");
        assert_eq!(processor.messaging_port(), 7448);
    }

    #[test]
    fn standalone_mode_with_custom_config() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config = StandaloneConfig::new()
            .with_node_name("custom_name")
            .with_instance_id("custom_instance")
            .with_messaging("192.168.1.100", 9999);

        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("should create processor");

        assert_eq!(processor.node_name(), "custom_name");
        assert_eq!(processor.bound_instance_id(), "custom_instance");
        assert_eq!(processor.messaging_host(), "192.168.1.100");
        assert_eq!(processor.messaging_port(), 9999);
    }

    #[test]
    fn standalone_mode_with_json5_parameters() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config =
            StandaloneConfig::new().with_parameters_json(serde_json::json!({ "value": 42 }));

        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("should create processor");

        let args_json = serde_json::to_value(processor.input_arguments()).unwrap();
        assert_eq!(args_json.get("value"), Some(&serde_json::json!(42)));
    }

    #[test]
    fn standalone_mode_with_typed_parameters() {
        use serde::Serialize;

        #[derive(Serialize)]
        struct TestParams {
            threshold: f64,
            enabled: bool,
        }

        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", parameters: { threshold: "f64", enabled: "bool" }, run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let params = TestParams {
            threshold: 0.75,
            enabled: true,
        };
        let config = StandaloneConfig::new().with_parameters(&params);

        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("should create processor");

        let args_json = serde_json::to_value(processor.input_arguments()).unwrap();
        assert_eq!(args_json.get("threshold"), Some(&serde_json::json!(0.75)));
        assert_eq!(args_json.get("enabled"), Some(&serde_json::json!(true)));
    }

    #[test]
    fn standalone_mode_peer_pin_is_retained_for_late_subscriber() {
        // `with_peer_pin` seeds the slot before any watch receiver exists
        // (the first `peer(...)` subscribes later); the seed must land in the
        // receiver-less channel rather than being discarded.
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: {
                name: "my_node",
                tag: "v1",
                depends_on: {
                    pairings: [
                        { name: "arm_link", tag: "v1", role: "controller", link_id: "arm" },
                    ],
                },
            },
            execution: { language: "rust", run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config = StandaloneConfig::new().with_peer_pin("arm", "core_a", "arm_1", "controller");
        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("should create processor");

        let watch_rx = processor
            .peer_pin_watch("arm")
            .expect("declared slot should have a channel");
        let state = watch_rx.borrow();
        let pin = state
            .pin
            .as_ref()
            .expect("seeded pin should survive until the first subscriber");
        assert_eq!(pin.producer.core_node, "core_a");
        assert_eq!(pin.producer.instance_id, "arm_1");
        assert_eq!(pin.peer_link_id, "controller");
    }

    #[test]
    fn standalone_mode_fails_when_required_parameters_missing() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", parameters: { value: "i64" }, run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        // No parameters provided — should fail immediately
        let config = StandaloneConfig::new();
        let result = Processor::new_standalone(&peppy_config_path, &config);

        let Err(err) = result else {
            panic!("expected error when required parameters are missing");
        };
        assert!(
            matches!(err, crate::error::Error::NodeArgumentsValidation(_)),
            "expected NodeArgumentsValidation error, got: {err:?}"
        );
        let err_string = err.to_string();
        assert!(
            err_string.contains("value"),
            "error should mention missing parameter 'value', got: {err_string}"
        );
    }

    #[test]
    fn standalone_mode_fails_when_some_parameters_missing() {
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: { language: "rust", parameters: { threshold: "f64", enabled: "bool", name: "string" }, run_cmd: ["./target/debug/my_node"] },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        // Only provide one of three required parameters
        let config =
            StandaloneConfig::new().with_parameters_json(serde_json::json!({ "threshold": 0.5 }));
        let result = Processor::new_standalone(&peppy_config_path, &config);

        let Err(err) = result else {
            panic!("expected error when some required parameters are missing");
        };
        let err_string = err.to_string();
        assert!(
            err_string.contains("enabled") && err_string.contains("name"),
            "error should mention missing parameters 'enabled' and 'name', got: {err_string}"
        );
        // The provided parameter should NOT be mentioned
        assert!(
            !err_string.contains("threshold"),
            "error should not mention provided parameter 'threshold', got: {err_string}"
        );
    }

    #[test]
    fn standalone_mode_fills_defaults_for_omitted_parameters() {
        // Partial config: user omits `frame_rate`, runtime fills it from $default.
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: {
                language: "rust",
                parameters: {
                    name: "string",
                    frame_rate: { $type: "u16", $default: 30 }
                },
                run_cmd: ["./target/debug/my_node"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config =
            StandaloneConfig::new().with_parameters_json(serde_json::json!({ "name": "front" }));
        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("standalone with partial parameters should succeed");

        let args_json = serde_json::to_value(processor.input_arguments()).unwrap();
        assert_eq!(args_json.get("name"), Some(&serde_json::json!("front")));
        assert_eq!(args_json.get("frame_rate"), Some(&serde_json::json!(30)));
    }

    #[test]
    fn standalone_mode_synthesizes_fully_defaulted_group() {
        // The whole `device` group is omitted; every leaf has a default,
        // so the runtime synthesizes the group from defaults.
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: {
                language: "rust",
                parameters: {
                    device: {
                        path: { $type: "string", $default: "/dev/video0" },
                        auto_detect: { $type: "bool", $default: true }
                    }
                },
                run_cmd: ["./target/debug/my_node"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config = StandaloneConfig::new();
        let processor = Processor::new_standalone(&peppy_config_path, &config)
            .expect("fully-defaulted group should be synthesized");

        let args_json = serde_json::to_value(processor.input_arguments()).unwrap();
        assert_eq!(
            args_json.get("device"),
            Some(&serde_json::json!({ "path": "/dev/video0", "auto_detect": true }))
        );
    }

    #[test]
    fn standalone_mode_partial_default_group_reports_missing_leaf() {
        // `device.serial` has no default (USB serials vary per unit); the
        // error must name that leaf by full dot-path so users know what to
        // supply.
        let temp_dir = TempDir::new().expect("temp dir should be created");

        let peppy_config_path = temp_dir.path().join("peppy.json5");
        let peppy_config_content = r#"{
            peppy_schema: "node/v1",
            manifest: { name: "my_node", tag: "v1" },
            execution: {
                language: "rust",
                parameters: {
                    device: {
                        path: { $type: "string", $default: "/dev/video0" },
                        serial: "string"
                    }
                },
                run_cmd: ["./target/debug/my_node"]
            },
        }"#;
        std::fs::write(&peppy_config_path, peppy_config_content)
            .expect("peppy config should be written");

        let config = StandaloneConfig::new();
        let result = Processor::new_standalone(&peppy_config_path, &config);
        let err = result.err().expect("missing required leaf should error");
        let msg = err.to_string();
        assert!(
            msg.contains("device.serial"),
            "error should name the missing leaf path, got: {msg}"
        );
    }

    #[test]
    fn validated_arguments_cannot_be_serialized_back_to_raw() {
        // NodeArguments derives Serialize but does not expose the inner
        // data — the only way to consume it is through
        // deserialize_parameters, which parses into a typed struct.
        let arguments = BTreeMap::from([("x".to_string(), AnyType::Int(1))]);
        let schema = ParameterSchema::from([(
            "x".to_string(),
            ParameterSpec::Primitive {
                kind: TypeToken::I64,
                default: None,
            },
        )]);
        let validated =
            validate_node_arguments(arguments, &schema).expect("validation should pass");

        // We can serialize (for deserialize_parameters) but cannot access
        // the inner map directly — this is a compile-time guarantee.
        let json = serde_json::to_value(&validated).expect("should serialize");
        assert_eq!(json.get("x"), Some(&serde_json::json!(1)));
    }

    /// Manifest fixture declaring one default-cardinality consumer slot
    /// (`main`), shared by the unbound-slot startup tests below.
    const CONSUMER_PEPPY_CONFIG: &str = r#"{
        peppy_schema: "node/v1",
        manifest: {
            name: "consumer_node",
            tag: "v1",
            depends_on: {
                nodes: [{ name: "camera", tag: "v1", link_id: "main" }]
            }
        },
        execution: { language: "rust", run_cmd: ["./target/debug/consumer_node"] },
    }"#;

    /// Manifest fixture with one slot of each cardinality: `main` (one,
    /// omitted), `arms` (one_or_more), `spare_cameras` (zero_or_more).
    const MULTI_SLOT_PEPPY_CONFIG: &str = r#"{
        peppy_schema: "node/v1",
        manifest: {
            name: "consumer_node",
            tag: "v1",
            depends_on: {
                nodes: [
                    { name: "camera", tag: "v1", link_id: "main" },
                    { name: "robot_arm", tag: "v1", link_id: "arms", cardinality: "one_or_more" }
                ],
                contracts: [
                    { name: "uvc_camera", tag: "v1", link_id: "spare_cameras", cardinality: "zero_or_more" }
                ]
            }
        },
        execution: { language: "rust", run_cmd: ["./target/debug/consumer_node"] },
    }"#;

    /// Builds a daemon-mode processor from a manifest string and the
    /// `slot_bindings` JSON5 block of the boot config (pass `None` to omit
    /// the field entirely).
    fn daemon_processor_with_bindings(
        peppy_config: &str,
        slot_bindings_json5: Option<&str>,
    ) -> Result<Processor, crate::error::Error> {
        let temp_dir = TempDir::new().expect("temp dir should be created");
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        std::fs::write(&peppy_config_path, peppy_config).expect("peppy config should be written");
        config::fingerprint::create_codegen_fingerprint(
            &peppy_config_path,
            Path::new(PEPPYGEN_OUTPUT_PATH),
        );

        let instance_block = match slot_bindings_json5 {
            Some(bindings) => {
                format!("{{ instance_id: \"consumer_1\", slot_bindings: {bindings} }}")
            }
            None => "{ instance_id: \"consumer_1\" }".to_string(),
        };
        let json5_config = format!(
            r#"{{
                messaging_host: "127.0.0.1",
                messaging_port: 7448,
                node_instance: {instance_block},
                node_name: "consumer_node",
                node_tag: "v1",
                bound_core_node: "core-1234"
            }}"#
        );
        let runtime_config: RuntimeConfig =
            serde_json5::from_str(&json5_config).expect("runtime config should parse");
        let runtime_config_path = temp_dir.path().join("peppy_runtime.json5");
        runtime_config
            .save_json5_launch_config(&runtime_config_path)
            .expect("runtime config should be saved");

        Processor::new_daemon_from_path(&peppy_config_path, runtime_config_path.to_str().unwrap())
    }

    fn instance_ids(producers: &[config::runtime::ProducerRef]) -> Vec<&str> {
        producers.iter().map(|p| p.instance_id.as_str()).collect()
    }

    /// The runtime backstop of the launch-time rule "every declared `one` /
    /// `one_or_more` slot must be bound": a boot config missing such a
    /// slot's binding entry fails processor construction, not lazily at
    /// first call.
    #[test]
    fn fails_at_startup_when_declared_slot_has_no_binding() {
        let Err(err) = daemon_processor_with_bindings(CONSUMER_PEPPY_CONFIG, None) else {
            panic!("expected unbound-slot startup error");
        };
        assert!(
            matches!(
                &err,
                crate::error::Error::SlotUnbound { link_id, cardinality }
                    if link_id == "main" && *cardinality == "one"
            ),
            "expected SlotUnbound for `main`, got: {err}"
        );
    }

    /// Daemon processor over [`MULTI_SLOT_PEPPY_CONFIG`] with the standard
    /// bindings: `main` (one) -> camera_1, `arms` (one_or_more) ->
    /// [right_arm, left_arm], `spare_cameras` (zero_or_more) -> [].
    fn multi_slot_processor() -> Processor {
        daemon_processor_with_bindings(
            MULTI_SLOT_PEPPY_CONFIG,
            Some(
                r#"{
                    main: [{ core_node: "core-1234", instance_id: "camera_1" }],
                    arms: [
                        { core_node: "core-1234", instance_id: "right_arm" },
                        { core_node: "core-1234", instance_id: "left_arm" }
                    ],
                    spare_cameras: []
                }"#,
            ),
        )
        .expect("valid bindings should construct")
    }

    /// Happy path: each slot's bound set reaches the startup cache with
    /// member order preserved, and `ensure_target_bound` enforces per-slot
    /// membership.
    #[test]
    fn daemon_boot_config_bindings_reach_bound_producer_cache() {
        let processor = multi_slot_processor();

        assert_eq!(
            instance_ids(processor.bound_producers("main")),
            ["camera_1"]
        );
        assert_eq!(
            instance_ids(processor.bound_producers("arms")),
            ["right_arm", "left_arm"],
            "binding declaration order must be preserved, not sorted"
        );
        assert!(processor.bound_producers("spare_cameras").is_empty());

        // Membership is per slot: a producer bound to a different slot of
        // the same consumer is rejected all the same.
        let right_arm = config::runtime::ProducerRef::new("core-1234", "right_arm");
        processor
            .ensure_target_bound("arms", &right_arm)
            .expect("bound member must pass the membership check");
        let err = processor
            .ensure_target_bound("main", &right_arm)
            .expect_err("cross-slot target must fail the membership check");
        assert!(
            matches!(
                &err,
                crate::error::Error::TargetNotBound { link_id, instance_id, .. }
                    if link_id == "main" && instance_id == "right_arm"
            ),
            "expected TargetNotBound, got: {err}"
        );
    }

    /// The cardinality-typed accessors expose exactly the guarantee startup
    /// validation established: `sole_bound_producer` returns a `one` slot's
    /// single member directly, and `non_empty_bound_producers` a
    /// `one_or_more` slot's ordered set with an infallible `first()`.
    #[test]
    fn cardinality_typed_accessors_return_the_validated_shapes() {
        let processor = multi_slot_processor();

        assert_eq!(
            processor.sole_bound_producer("main").instance_id,
            "camera_1"
        );

        let arms = processor.non_empty_bound_producers("arms");
        assert_eq!(
            arms.first().instance_id,
            "right_arm",
            "first() follows binding declaration order"
        );
        assert_eq!(instance_ids(arms.as_slice()), ["right_arm", "left_arm"]);
        assert_eq!(
            arms.as_slice(),
            processor.bound_producers("arms"),
            "the typed view exposes the same cached set as the plain slice"
        );
    }

    /// A typed accessor whose guarantee the cached set does not meet is
    /// codegen / runtime skew (the accessor and the startup validation come
    /// from the same manifest), so it panics like an unknown `link_id`
    /// rather than returning an error the caller could mishandle.
    #[test]
    #[should_panic(expected = "expects cardinality `one`")]
    fn sole_bound_producer_panics_on_a_multi_member_set() {
        let _ = multi_slot_processor().sole_bound_producer("arms");
    }

    /// See [`sole_bound_producer_panics_on_a_multi_member_set`]: the
    /// non-empty view refuses an empty `zero_or_more` set the same way.
    #[test]
    #[should_panic(expected = "expects cardinality `one_or_more`")]
    fn non_empty_bound_producers_panics_on_an_empty_set() {
        let _ = multi_slot_processor().non_empty_bound_producers("spare_cameras");
    }

    /// A boot config carrying the removed pre-cardinality single-producer
    /// object shape must fail to parse with the version-skew message: the
    /// daemon, CLI, generated bindings, and node runtime ship together.
    #[test]
    fn boot_config_with_pre_cardinality_object_binding_fails_to_parse() {
        let json5_config = r#"{
            messaging_host: "127.0.0.1",
            messaging_port: 7448,
            node_instance: {
                instance_id: "consumer_1",
                slot_bindings: {
                    main: { core_node: "core-1234", instance_id: "camera_1" }
                }
            },
            node_name: "consumer_node",
            node_tag: "v1",
            bound_core_node: "core-1234"
        }"#;
        let err = serde_json5::from_str::<RuntimeConfig>(json5_config)
            .expect_err("object-valued slot binding must be a hard parse error");
        assert!(
            err.to_string().contains("upgraded together"),
            "parse error should name the version skew, got: {err}"
        );
    }

    /// The cardinality size rules are re-checked at startup: a `one` slot
    /// with two producers and a `one_or_more` slot with an empty set are
    /// both rejected even though the boot config parses.
    #[test]
    fn fails_at_startup_when_bound_set_size_violates_cardinality() {
        let two_on_a_one_slot = daemon_processor_with_bindings(
            MULTI_SLOT_PEPPY_CONFIG,
            Some(
                r#"{
                    main: [
                        { core_node: "core-1234", instance_id: "camera_1" },
                        { core_node: "core-1234", instance_id: "camera_2" }
                    ],
                    arms: [{ core_node: "core-1234", instance_id: "right_arm" }],
                    spare_cameras: []
                }"#,
            ),
        );
        let Err(two_on_a_one_slot) = two_on_a_one_slot else {
            panic!("two producers on a `one` slot must fail startup");
        };
        assert!(
            matches!(
                &two_on_a_one_slot,
                crate::error::Error::SlotCardinalityViolated { link_id, cardinality, bound }
                    if link_id == "main" && *cardinality == "one" && *bound == 2
            ),
            "expected SlotCardinalityViolated for `main`, got: {two_on_a_one_slot}"
        );

        let empty_one_or_more = daemon_processor_with_bindings(
            MULTI_SLOT_PEPPY_CONFIG,
            Some(
                r#"{
                    main: [{ core_node: "core-1234", instance_id: "camera_1" }],
                    arms: [],
                    spare_cameras: []
                }"#,
            ),
        );
        let Err(empty_one_or_more) = empty_one_or_more else {
            panic!("an empty set on a `one_or_more` slot must fail startup");
        };
        assert!(
            matches!(
                &empty_one_or_more,
                crate::error::Error::SlotCardinalityViolated { link_id, cardinality, bound }
                    if link_id == "arms" && *cardinality == "one_or_more" && *bound == 0
            ),
            "expected SlotCardinalityViolated for `arms`, got: {empty_one_or_more}"
        );
    }

    /// A `zero_or_more` slot may be left out of `slot_bindings` entirely;
    /// it resolves to the same empty set as an explicit empty array.
    #[test]
    fn zero_or_more_slot_tolerates_missing_binding_entry() {
        let processor = daemon_processor_with_bindings(
            MULTI_SLOT_PEPPY_CONFIG,
            Some(
                r#"{
                    main: [{ core_node: "core-1234", instance_id: "camera_1" }],
                    arms: [{ core_node: "core-1234", instance_id: "right_arm" }]
                }"#,
            ),
        )
        .expect("missing zero_or_more entry should construct");
        assert!(processor.bound_producers("spare_cameras").is_empty());
    }

    /// Standalone runs enforce the same rules via
    /// `StandaloneConfig::with_bound_producer`: a declared `one` slot left
    /// unseeded fails startup; repeat calls accumulate a multi slot's set
    /// in call order; duplicate seeds are rejected.
    #[test]
    fn standalone_bound_producers_accumulate_and_enforce_cardinality() {
        let temp_dir = TempDir::new().expect("temp dir should be created");
        let peppy_config_path = temp_dir.path().join("peppy.json5");
        std::fs::write(&peppy_config_path, MULTI_SLOT_PEPPY_CONFIG)
            .expect("peppy config should be written");

        let unseeded = StandaloneConfig::new();
        let Err(err) = Processor::new_standalone(&peppy_config_path, &unseeded) else {
            panic!("expected unbound-slot startup error");
        };
        assert!(
            matches!(
                &err,
                crate::error::Error::SlotUnbound { link_id, .. } if link_id == "main"
            ),
            "expected SlotUnbound for `main`, got: {err}"
        );

        // `spare_cameras` (zero_or_more) is deliberately left unseeded.
        let seeded = StandaloneConfig::new()
            .with_bound_producer("main", "core_x", "camera_1")
            .with_bound_producer("arms", "core_x", "right_arm")
            .with_bound_producer("arms", "core_x", "left_arm");
        let processor = Processor::new_standalone(&peppy_config_path, &seeded)
            .expect("seeded slots should construct");
        assert_eq!(
            instance_ids(processor.bound_producers("main")),
            ["camera_1"]
        );
        assert_eq!(
            instance_ids(processor.bound_producers("arms")),
            ["right_arm", "left_arm"],
            "with_bound_producer call order must be preserved"
        );
        assert!(processor.bound_producers("spare_cameras").is_empty());

        let duplicated = StandaloneConfig::new()
            .with_bound_producer("main", "core_x", "camera_1")
            .with_bound_producer("arms", "core_x", "right_arm")
            .with_bound_producer("arms", "core_x", "right_arm");
        let Err(err) = Processor::new_standalone(&peppy_config_path, &duplicated) else {
            panic!("duplicate seeded producer must be rejected");
        };
        assert!(
            err.to_string().contains("right_arm"),
            "duplicate error should name the producer, got: {err}"
        );
    }
}
