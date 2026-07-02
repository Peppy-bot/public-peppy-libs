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
        "Duplicate link_id `{0}` in manifest.depends_on (link_ids must be unique across nodes and interfaces)"
    )]
    DuplicateLinkId(String),
    #[error(
        "Conflicting `from_any: true` for dependency `{name}` (tag `{tag}`) in manifest.depends_on: only one entry per (name, tag) may set from_any=true"
    )]
    ConflictingFromAny { name: String, tag: String },

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
    /// Boxed payload for the same reason as
    /// [`ParsingError::BindingTargetMismatch`]: keeps the variant's
    /// String-heavy struct from inflating `ParsingError`'s size past the
    /// `clippy::result_large_err` threshold.
    #[error(transparent)]
    MissingInterface(Box<MissingInterface>),
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum StructuredError {
    MissingExecutionLanguage,
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
