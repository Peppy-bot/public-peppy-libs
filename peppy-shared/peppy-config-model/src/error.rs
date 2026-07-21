use thiserror::Error;

pub type Result<T> = core::result::Result<T, Error>;

/// Deserializes JSON5 content with field-path tracking and an embedded
/// structured-error bridge, generic over the caller's error types.
///
/// Parsers that need to raise rich validation errors from inside
/// `Deserialize` impls encode a payload of type `S` as JSON5 into the serde
/// error message (see `StructuredError::json5_message`); this helper decodes
/// it back and hands it to `on_structured` unchanged, since such payloads
/// already carry descriptive messages. Plain serde messages get the dotted
/// field path (e.g. `execution.run_cmd`) prepended before `on_plain`.
/// Phase-1 JSON5 syntax errors (no field path exists yet) go to `on_syntax`.
///
/// Exposed so downstream document models (e.g. the daemon-side launcher
/// configs) can reuse the engine with their own structured-error enums
/// instead of duplicating the path-tracking logic.
pub fn deserialize_json5_with_structured_errors<'de, T, S, E>(
    content: &'de str,
    on_syntax: impl FnOnce(serde_json5::Error) -> E,
    on_structured: impl FnOnce(S) -> E,
    on_plain: impl FnOnce(String) -> E,
) -> core::result::Result<T, E>
where
    T: serde::de::Deserialize<'de>,
    S: serde::de::DeserializeOwned,
{
    // Phase 1: parse JSON5 syntax. If this fails, there's no field path.
    let mut deserializer = serde_json5::Deserializer::from_str(content).map_err(on_syntax)?;

    // Phase 2: deserialize with path tracking.
    serde_path_to_error::deserialize(&mut deserializer).map_err(|path_err| {
        let path = path_err.path().to_string();
        let serde_json5::Error::Message { msg, .. } = path_err.into_inner();

        // Check if it's a structured payload (custom validation).
        // These already have rich messages; don't prepend path.
        if let Ok(structured) = serde_json5::from_str::<S>(&msg) {
            return on_structured(structured);
        }

        // Standard serde error: prepend path if non-empty.
        let message = if path.is_empty() || path == "." {
            msg
        } else {
            format!("{path}: {msg}")
        };
        on_plain(message)
    })
}

/// Deserializes JSON5 content with field-path tracking.
///
/// On error, prepends the JSON path (e.g. `execution.run_cmd`) to standard
/// serde error messages. `StructuredError`s (custom validation) are propagated
/// unchanged since they already contain descriptive messages.
pub fn deserialize_json5_with_path<'de, T>(content: &'de str) -> Result<T>
where
    T: serde::de::Deserialize<'de>,
{
    deserialize_json5_with_structured_errors(
        content,
        |e| Error::Parsing(ParsingError::from(e)),
        |s: StructuredError| Error::Parsing(ParsingError::from(s)),
        |message| Error::Parsing(ParsingError::CannotParseConfig(message)),
    )
}

/// Payload for [`ParsingError::MissingInterface`]. Boxed in the variant so
/// the six `String` fields do not inflate `ParsingError` past the
/// `clippy::result_large_err` threshold.
#[derive(Debug, Clone, Error)]
#[error(
    "`{dependant}`:{dependant_tag} expects {interface_kind} `{interface_name}` from \
     `{dependency}`:{dependency_tag}, but it is not exposed"
)]
pub struct MissingInterface {
    pub dependant: String,
    pub dependant_tag: String,
    pub dependency: String,
    pub dependency_tag: String,
    pub interface_kind: String,
    pub interface_name: String,
}

/// Payload for [`ParsingError::ConsumedInterfaceOnlyContractBacked`]: a
/// node-dependency consumer asked for an interface its producer provides only
/// as part of an implemented contract. Node dependencies expose native
/// interfaces only; contract-backed interfaces are consumable solely through
/// `depends_on.contracts`.
#[derive(Debug, Clone, Error)]
#[error(
    "`{dependant}`:{dependant_tag} consumes {interface_kind} `{interface_name}` from \
     `{dependency}`:{dependency_tag} via node link_id `{link_id}`, but the producer provides \
     it only as part of contract `{contract_name}:{contract_tag}` — consume it through a \
     `depends_on.contracts` slot for `{contract_name}:{contract_tag}` instead"
)]
pub struct ConsumedInterfaceOnlyContractBacked {
    pub dependant: String,
    pub dependant_tag: String,
    pub dependency: String,
    pub dependency_tag: String,
    pub interface_kind: String,
    pub interface_name: String,
    pub link_id: String,
    pub contract_name: String,
    pub contract_tag: String,
}

/// Payload for [`ParsingError::ContractCoverageMismatch`]: the Tier B
/// set-diff between one `manifest.implements` slot and the contract-backed
/// entries referencing it. Aggregates every discrepancy for the slot in one
/// error instead of failing on the first.
#[derive(Debug, Clone)]
pub struct ContractCoverageMismatch {
    pub contract_name: String,
    pub contract_tag: String,
    pub link_id: String,
    /// Contract members with no manifest entry.
    pub missing: Vec<String>,
    /// Manifest entries naming no contract member (of any kind).
    pub unknown: Vec<String>,
    /// Contract members referenced by more than one manifest entry.
    pub duplicated: Vec<String>,
    /// Manifest entries whose name matches a contract member of a different
    /// kind (e.g. a contract topic listed under `services.exposes`),
    /// rendered as `name (declared as X, contract declares Y)`.
    pub wrong_kind: Vec<String>,
}

impl std::fmt::Display for ContractCoverageMismatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "contract `{}:{}` (implements slot `{}`) is not fully implemented: \
             every contract member needs exactly one contract-backed entry in \
             `interfaces` referencing link_id `{}`",
            self.contract_name, self.contract_tag, self.link_id, self.link_id
        )?;
        write_labeled_lists(
            f,
            &[
                ("missing", &self.missing),
                ("unknown", &self.unknown),
                ("duplicated", &self.duplicated),
                ("wrong kind", &self.wrong_kind),
            ],
        )
    }
}

impl std::error::Error for ContractCoverageMismatch {}

/// Tier B (raised at node add/sync, where pairing documents resolve): the
/// set-diff between one `depends_on.pairings` slot and the pairing-backed
/// entries referencing it. The pairing counterpart of
/// [`ContractCoverageMismatch`], aggregating every discrepancy for the slot in
/// one error instead of failing on the first.
///
/// The two directions are held apart because their policies differ: the emit
/// side must cover the role's topics exactly, while the consume side is free
/// to name any subset of the counterpart role's topics.
#[derive(Debug, Clone)]
pub struct PairingCoverageMismatch {
    pub pairing_name: String,
    pub pairing_tag: String,
    pub link_id: String,
    /// The role this node's slot plays.
    pub role: String,
    /// Topics the role emits per the document, with no `topics.emits` entry.
    pub missing_emits: Vec<String>,
    /// `topics.emits` entries naming no topic of the document.
    pub unknown_emits: Vec<String>,
    /// Topics referenced by more than one `topics.emits` entry.
    pub duplicated_emits: Vec<String>,
    /// `topics.emits` entries naming a topic the counterpart role emits,
    /// rendered as `name (emitted by X)`.
    pub wrong_role_emits: Vec<String>,
    /// `topics.consumes` entries naming no topic of the document.
    pub unknown_consumes: Vec<String>,
    /// Topics referenced by more than one `topics.consumes` entry.
    pub duplicated_consumes: Vec<String>,
    /// `topics.consumes` entries naming a topic this node's own role emits,
    /// rendered as `name (emitted by this node's role X)`.
    pub wrong_role_consumes: Vec<String>,
}

impl PairingCoverageMismatch {
    pub fn is_empty(&self) -> bool {
        self.missing_emits.is_empty()
            && self.unknown_emits.is_empty()
            && self.duplicated_emits.is_empty()
            && self.wrong_role_emits.is_empty()
            && self.unknown_consumes.is_empty()
            && self.duplicated_consumes.is_empty()
            && self.wrong_role_consumes.is_empty()
    }
}

impl std::fmt::Display for PairingCoverageMismatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "pairing `{}:{}` (slot `{}`, role `{}`) is not correctly declared: \
             every topic the role emits needs exactly one entry in `topics.emits` \
             referencing link_id `{}`, and every `topics.consumes` entry must name \
             a topic the counterpart role emits",
            self.pairing_name, self.pairing_tag, self.link_id, self.role, self.link_id
        )?;
        write_labeled_lists(
            f,
            &[
                ("missing emits", &self.missing_emits),
                ("unknown emits", &self.unknown_emits),
                ("duplicated emits", &self.duplicated_emits),
                ("wrong role emits", &self.wrong_role_emits),
                ("unknown consumes", &self.unknown_consumes),
                ("duplicated consumes", &self.duplicated_consumes),
                ("wrong role consumes", &self.wrong_role_consumes),
            ],
        )
    }
}

impl std::error::Error for PairingCoverageMismatch {}

/// Appends every non-empty `label: [a, b]` section to a coverage-diff message.
fn write_labeled_lists(
    f: &mut std::fmt::Formatter<'_>,
    sections: &[(&str, &Vec<String>)],
) -> std::fmt::Result {
    for (label, items) in sections {
        if items.is_empty() {
            continue;
        }
        write!(f, "; {label}: [")?;
        write_string_list(f, items)?;
        write!(f, "]")?;
    }
    Ok(())
}

fn write_string_list(f: &mut std::fmt::Formatter<'_>, items: &[String]) -> std::fmt::Result {
    for (idx, item) in items.iter().enumerate() {
        if idx > 0 {
            write!(f, ", ")?;
        }
        write!(f, "{item}")?;
    }
    Ok(())
}

#[derive(Debug, Error, Clone)]
pub enum ParsingError {
    // -- General yaml syntax
    #[error("Cannot read {0}: {1}")]
    CannotRead(String, std::io::ErrorKind),
    #[error("Cannot parse configuration: {0}")]
    CannotParseConfig(String),
    #[error("Empty content found in: {0}")]
    EmptyContent(String),

    // -- node_config
    #[error("Invalid name: {0}, allowed characters: {1}")]
    InvalidName(String, String),
    #[error("Empty name")]
    EmptyName,
    #[error(
        "Duplicate link_id `{0}` in manifest (link_ids share one flat namespace across depends_on.nodes, depends_on.contracts, depends_on.pairings, and manifest.implements)"
    )]
    DuplicateLinkId(String),
    #[error(
        "Pairing link_id `{0}` in manifest.depends_on.pairings is not a valid wire segment (must not contain '/' or '@', and must not collide with a reserved sentinel) — pairing slot link_ids appear on the wire as the producer-side link_id segment"
    )]
    PairingSentinelLinkId(String),
    #[error(
        "Duplicate producer `{instance_id}@{core_node}` in a slot's bound set — bound producers must be unique within a slot"
    )]
    DuplicateBoundProducer {
        core_node: String,
        instance_id: String,
    },

    // -- build system
    #[error("Invalid toolchain {0}")]
    InvalidToolchain(String),

    // -- node config: process vs container
    #[error("Node config must have exactly one of `process` or `container`, not both")]
    ProcessAndContainerConflict,
    #[error("Node config must have either `process` or `container`")]
    NoProcessOrContainer,
    #[error("Node config `execution.run_cmd` must not be empty")]
    EmptyRunCmd,

    // -- node config: execution
    #[error("Node config `execution.language` is required when an execution block is defined")]
    MissingExecutionLanguage,

    // -- container config: mount paths
    #[error(
        "Invalid mount path `{0}`: top-level system directories ({1}) cannot be used as mount sources — use a subdirectory instead (e.g., /tmp/my_app)"
    )]
    InvalidMountPath(String, String),
    #[error("Invalid parameter reference `${{parameters:{0}}}` in mount path: {1}")]
    InvalidMountPathParameterRef(String, String),

    // -- node dependency validation
    #[error(
        "`{dependant}:{dependant_tag}` depends on `{dependency}:{dependency_tag}`, but it does not exist in the stack"
    )]
    MissingDependency {
        dependant: String,
        dependant_tag: String,
        dependency: String,
        dependency_tag: String,
    },
    #[error(
        "`{dependant}:{dependant_tag}` references undeclared link_id `{link_id}` in consumed interfaces"
    )]
    UndeclaredLinkId {
        dependant: String,
        dependant_tag: String,
        link_id: String,
    },
    #[error(
        "`{dependant}:{dependant_tag}` references pairing link_id `{link_id}` in `{section}` — a pairing document declares topics only, so a pairing slot exposes no service or action to consume; only `topics.consumes` may name a pairing slot"
    )]
    ConsumedItemReferencesPairingLinkId {
        dependant: String,
        dependant_tag: String,
        link_id: String,
        section: String,
    },
    /// Boxed payload for the same reason as
    /// [`ParsingError::BindingTargetMismatch`]: keeps the variant's
    /// String-heavy struct from inflating `ParsingError`'s size past the
    /// `clippy::result_large_err` threshold.
    #[error(transparent)]
    MissingInterface(Box<MissingInterface>),

    // -- manifest.implements + produced-interface entries
    #[error(
        "Document-backed entry `{name}` (link_id `{link_id}`) in `{section}` must not carry `{field}` — shape and QoS come from the contract or pairing document the slot resolves to; a document-backed entry is exactly `{{link_id, name}}`"
    )]
    LinkedEntryWithInlineShape {
        section: String,
        link_id: String,
        name: String,
        field: String,
    },
    #[error("Entries in `{section}` require a non-empty `name`")]
    EmptyInterfaceName { section: String },
    #[error(
        "Duplicate contract `{name}:{tag}` in manifest.implements — a node implements each contract at most once; multiplicity is the job of node instances/pairings"
    )]
    DuplicateImplementsContract { name: String, tag: String },
    #[error(
        "Contracts `{name}:{tag_a}` and `{name}:{tag_b}` in manifest.implements collide after tag sanitization (`-` becomes `_` in generated code and on the wire)"
    )]
    ImplementsTagSanitizationCollision {
        name: String,
        tag_a: String,
        tag_b: String,
    },
    #[error(
        "Entry in `{section}` references link_id `{link_id}`, which is declared in `depends_on.{found_in}` — `topics.emits` may reference a `manifest.implements` or `depends_on.pairings` slot, while `services.exposes` and `actions.exposes` may reference only a `manifest.implements` slot; interfaces obtained from a dependency belong in the `consumes` lists"
    )]
    EmitsLinkIdNotImplements {
        section: String,
        link_id: String,
        found_in: String,
    },
    #[error(
        "Entry in `{section}` references link_id `{link_id}`, which matches no slot declared in `manifest.implements` or `depends_on`"
    )]
    UndeclaredEmitsLinkId { section: String, link_id: String },
    #[error(
        "Consumed interface references implements link_id `{link_id}` — `manifest.implements` slots are produced, not consumed; to consume a contract, declare it under `depends_on.contracts`"
    )]
    ConsumedItemReferencesImplementsLinkId { link_id: String },
    #[error(
        "Pairing slot `{link_id}` in depends_on.pairings carries a `cardinality` key — a pairing is strictly 1:1 between two complementary slots; use `optional: true` to express absence. `cardinality` is valid only on depends_on.nodes and depends_on.contracts entries"
    )]
    CardinalityOnPairingSlot { link_id: String },
    #[error("Duplicate entry `{key}` in `{section}`")]
    DuplicateInterfaceEntry { section: String, key: String },

    /// Tier B: aggregated per-slot coverage diff, raised where contract
    /// documents resolve (node add/sync). Boxed for the same size reason as
    /// [`ParsingError::MissingInterface`].
    #[error(transparent)]
    ContractCoverageMismatch(Box<ContractCoverageMismatch>),
    /// Tier B: a node-dependency consumer asked for a producer interface
    /// that exists only contract-backed. Boxed for the same size reason as
    /// [`ParsingError::MissingInterface`].
    #[error(transparent)]
    ConsumedInterfaceOnlyContractBacked(Box<ConsumedInterfaceOnlyContractBacked>),
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum StructuredError {
    MissingExecutionLanguage,
    LinkedEntryWithInlineShape {
        section: String,
        link_id: String,
        name: String,
        field: String,
    },
    EmptyInterfaceName {
        section: String,
    },
    CardinalityOnPairingSlot {
        link_id: String,
    },
}

impl StructuredError {
    pub(crate) fn json5_message(&self) -> String {
        serde_json5::to_string(self).unwrap_or_else(|_| "serialization error".to_string())
    }
}

impl From<StructuredError> for ParsingError {
    fn from(s: StructuredError) -> Self {
        match s {
            StructuredError::MissingExecutionLanguage => ParsingError::MissingExecutionLanguage,
            StructuredError::LinkedEntryWithInlineShape {
                section,
                link_id,
                name,
                field,
            } => ParsingError::LinkedEntryWithInlineShape {
                section,
                link_id,
                name,
                field,
            },
            StructuredError::EmptyInterfaceName { section } => {
                ParsingError::EmptyInterfaceName { section }
            }
            StructuredError::CardinalityOnPairingSlot { link_id } => {
                ParsingError::CardinalityOnPairingSlot { link_id }
            }
        }
    }
}

impl From<serde_json5::Error> for ParsingError {
    fn from(err: serde_json5::Error) -> Self {
        match err {
            serde_json5::Error::Message { msg, .. } => {
                if let Ok(structured) = serde_json5::from_str::<StructuredError>(&msg) {
                    ParsingError::from(structured)
                } else {
                    ParsingError::CannotParseConfig(msg)
                }
            }
        }
    }
}

#[derive(Debug, Error)]
pub enum Error {
    // -- general
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("Capnp error: {0}")]
    Capnp(#[from] capnp::Error),

    // -- Parsing error
    #[error(transparent)]
    Parsing(#[from] ParsingError),
    #[error("Serialize error: {0}")]
    Serialize(String),
    #[error("Encoding error: {0}")]
    Encoding(String),

    // -- Fingerprint
    #[error(
        "Node config fingerprint mismatch: expected {expected}, got {actual}. The config may have been modified after code generation. Run `node sync` to update the peppygen lib on your node."
    )]
    FingerprintMismatch { expected: String, actual: String },
    #[error(
        "Release fingerprint mismatch: node was generated with peppy version {node_version}, but current peppy version is {current_version}. Run `node sync` to regenerate with the current version."
    )]
    ReleaseFingerprintMismatch {
        node_version: String,
        current_version: String,
    },
    #[error("Release fingerprint missing: {0}")]
    ReleaseFingerprintMissing(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_structured_error_deserialization() {
        // Helper to create a serde_json5 error from a string
        fn make_err(msg: &str) -> serde_json5::Error {
            serde::de::Error::custom(msg)
        }

        // MissingExecutionLanguage round-trips through the structured bridge.
        let json = serde_json5::to_string(&StructuredError::MissingExecutionLanguage).unwrap();
        let err = ParsingError::from(make_err(&json));
        if !matches!(err, ParsingError::MissingExecutionLanguage) {
            panic!("Expected MissingExecutionLanguage, got {:?}", err);
        }
    }

    #[test]
    fn test_fallback_mechanism() {
        fn make_err(msg: &str) -> serde_json5::Error {
            serde::de::Error::custom(msg)
        }

        let raw_msg = "This is not JSON";
        let err = ParsingError::from(make_err(raw_msg));
        if let ParsingError::CannotParseConfig(msg) = err {
            assert_eq!(msg, raw_msg);
        } else {
            panic!("Expected CannotParseConfig, got {:?}", err);
        }

        let broken_json = "{ invalid json";
        let err = ParsingError::from(make_err(broken_json));
        if let ParsingError::CannotParseConfig(msg) = err {
            assert_eq!(msg, broken_json);
        } else {
            panic!("Expected CannotParseConfig, got {:?}", err);
        }
    }
}
