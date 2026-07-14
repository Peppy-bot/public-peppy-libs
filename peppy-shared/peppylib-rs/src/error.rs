use std::env::VarError;
use std::fmt;
use thiserror::Error;

pub type Result<T> = core::result::Result<T, Error>;

/// Errors that can occur during parameter deserialization or validation.
#[derive(Debug)]
pub struct ParameterDeserializationError(pub Vec<String>);

impl ParameterDeserializationError {
    pub fn single(message: impl Into<String>) -> Self {
        Self(vec![message.into()])
    }

    pub fn multiple(messages: Vec<String>) -> Self {
        Self(messages)
    }
}

impl fmt::Display for ParameterDeserializationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0.as_slice() {
            [] => write!(f, "parameter deserialization error: unknown error"),
            [single] => write!(f, "parameter deserialization error: {}", single),
            multiple => {
                write!(f, "missing required parameters:")?;
                for msg in multiple {
                    write!(f, "\n  - {}", msg)?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for ParameterDeserializationError {}

#[derive(Debug, Error)]
pub enum Error {
    // -- general
    #[error(transparent)]
    Io(#[from] std::io::Error),

    // -- system clock (set before the Unix epoch); produced by `clock::wall_now_ns`
    #[error("system clock unavailable: {0}")]
    SystemTime(#[from] std::time::SystemTimeError),

    // -- config
    #[error(transparent)]
    Config(#[from] config::ConfigError),

    // -- serde
    #[error(transparent)]
    SerdeJson5(#[from] serde_json5::Error),

    // -- peppy-messaging-interface
    #[error(transparent)]
    PeppyMessagingInterface(#[from] pmi::PeppyMessagingInterfaceError),

    // -- wire-format input validation (SenderTarget construction in generated code)
    #[error(transparent)]
    InvalidSenderTarget(#[from] pmi::SenderTargetError),

    // -- core-node-api
    #[error(transparent)]
    CoreNodeApi(#[from] core_node_api::Error),

    #[error("invalid service request '{identifier}': {reason}")]
    InvalidServiceRequest { identifier: String, reason: String },

    #[error("clock not ready: no external tick observed yet on the `clock` topic (sim mode)")]
    ClockNotReady,

    #[error("internal encoding error for '{identifier}': {reason}")]
    InternalEncodingError { identifier: String, reason: String },

    #[error("service request stream closed unexpectedly")]
    ServiceRequestStreamClosed,

    #[error("action feedback channel closed unexpectedly")]
    ActionFeedbackChannelClosed,

    // -- pairing
    #[error(
        "unknown pairing slot '{link_id}': the manifest declares no depends_on.pairings entry with that link_id"
    )]
    UnknownPairingSlot { link_id: String },

    #[error("pairing slot channel closed (runtime torn down while waiting for a peer)")]
    PairingSlotClosed,

    // -- topics/services/actions errors
    #[error(
        "service '{service_name}'{instance_suffix} is unreachable",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ServiceUnreachable {
        instance_id: Option<String>,
        service_name: String,
    },
    #[error(
        "service '{service_name}'{instance_suffix} has timed out",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ServiceTimeout {
        instance_id: Option<String>,
        service_name: String,
    },
    #[error(
        "service '{service_name}'{instance_suffix} returned error: {reason}",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ServiceError {
        instance_id: Option<String>,
        service_name: String,
        reason: String,
    },
    #[error(
        "action '{action_name}'{instance_suffix} has timed out waiting for result",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ActionResultTimeout {
        instance_id: Option<String>,
        action_name: String,
    },
    #[error(
        "action '{action_name}'{instance_suffix} is unreachable for result",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ActionResultUnreachable {
        instance_id: Option<String>,
        action_name: String,
    },
    #[error(
        "action '{action_name}'{instance_suffix} producer disappeared before closing the feedback stream",
        instance_suffix = InstanceSuffix(.instance_id.as_deref())
    )]
    ActionFeedbackProducerGone {
        instance_id: Option<String>,
        action_name: String,
    },

    // -- system
    #[error("failed to read `{var}` from the environment")]
    MissingInstanceIdEnvVar {
        var: &'static str,
        #[source]
        source: VarError,
    },

    #[error("failed to read launch config at `{path}`")]
    LaunchConfigRead {
        path: String,
        #[source]
        source: std::io::Error,
    },

    #[error("failed to parse launch config at `{path}`")]
    LaunchConfigParse {
        path: String,
        #[source]
        source: serde_json5::Error,
    },

    #[error(
        "peppy config fingerprint mismatch for `{path}` (expected `{expected}`, got `{actual}`)"
    )]
    PeppyConfigFingerprintMismatch {
        path: String,
        expected: String,
        actual: String,
    },

    #[error("failed to read codegen fingerprint at `{path}`")]
    CodegenFingerprintRead {
        path: String,
        #[source]
        source: std::io::Error,
    },

    #[error(transparent)]
    ParameterTypeMismatch(#[from] config::TypeMismatch),

    #[error(transparent)]
    NodeArgumentsValidation(#[from] config::NodeArgumentsError),

    #[error(transparent)]
    ParameterDeserialization(#[from] ParameterDeserializationError),

    #[error("parameters have already been taken (take_parameters() can only be called once)")]
    ParametersAlreadyTaken,

    // --- Serialization
    #[error("serialization error: {0}")]
    Serialization(String),

    #[error("deserialization error: {0}")]
    Deserialization(String),

    #[error("invalid messenger configuration: {0}")]
    ConfigurationError(String),

    // --- Runner
    #[error("failed to build blocking runtime for `{context}`")]
    RuntimeInitialization {
        context: String,
        #[source]
        source: std::io::Error,
    },

    // --- Node/Topic
    #[error("invalid node name `{node_name}`: {reason}")]
    InvalidNodeName { node_name: String, reason: String },

    #[error("invalid core node name `{node_name}`: {reason}")]
    InvalidCoreNodeName { node_name: String, reason: String },

    #[error("failed to subscribe to topic `{topic_name}` in node `{node_name}`, {source_msg}")]
    TopicSubscribe {
        topic_name: String,
        node_name: String,
        source_msg: String,
    },

    #[error("subscription to `{topic_name}` closed without yielding a message")]
    SubscriptionClosed { topic_name: String },

    /// Startup backstop for the launch-time rule "every declared
    /// depends_on slot with cardinality `one` / `one_or_more` must be
    /// bound": a daemon that validates bindings never ships a boot config
    /// missing such a slot's entry, so hitting this means version skew or
    /// a hand-edited boot config.
    #[error(
        "consumer slot `{link_id}` (cardinality `{cardinality}`) is unbound: the boot config \
         carries no producer set for it, but only a `zero_or_more` slot may be left unbound. \
         Fix the launcher / daemon that produced the boot config (or, in standalone mode, \
         seed the slot via `StandaloneConfig::with_bound_producer`)"
    )]
    SlotUnbound {
        link_id: String,
        cardinality: config::node::Cardinality,
    },

    /// Startup backstop for the launch-time cardinality size rules: the
    /// boot config carries a bound set whose size the slot's declared
    /// cardinality does not allow (for example two producers on a `one`
    /// slot, or an empty set on a `one_or_more` slot).
    #[error(
        "consumer slot `{link_id}` declares cardinality `{cardinality}` but the boot config \
         binds {bound} producer(s) — the launcher validator enforces this at plan time, so \
         this boot config was produced by an incompatible component version or edited by hand"
    )]
    SlotCardinalityViolated {
        link_id: String,
        cardinality: config::node::Cardinality,
        bound: usize,
    },

    /// A directed service / action call named a producer outside the
    /// slot's bound set. Every `poll` / `fire_goal` target must come from
    /// the slot's own bound set, exposed by the slot's cardinality-typed
    /// accessor (`bound_producer()` for `one`, `bound_producers()` for the
    /// multi cardinalities): an out-of-set instance was never checked by
    /// plan-time binding validation, and membership is per slot, so a
    /// producer bound to a different slot of the same consumer is rejected
    /// all the same.
    #[error(
        "target `{instance_id}@{core_node}` is not in the bound set of consumer slot \
         `{link_id}`: pass a producer from this slot's own generated accessor \
         (`bound_producer()` / `bound_producers()`)"
    )]
    TargetNotBound {
        link_id: String,
        core_node: String,
        instance_id: String,
    },

    #[error("message format for `{context}` is not available in the generator")]
    MessageFormatUnavailable { context: String },
}

struct InstanceSuffix<'a>(Option<&'a str>);

impl fmt::Display for InstanceSuffix<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(instance_id) = self.0 {
            write!(f, " for instance '{instance_id}'")
        } else {
            Ok(())
        }
    }
}
