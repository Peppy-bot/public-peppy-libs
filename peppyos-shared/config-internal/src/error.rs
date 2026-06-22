use thiserror::Error;

pub type Result<T> = core::result::Result<T, Error>;

/// Deserializes JSON5 content with field-path tracking.
///
/// On error, prepends the JSON path (e.g. `execution.run_cmd`) to standard
/// serde error messages. `StructuredError`s (custom validation) are propagated
/// unchanged since they already contain descriptive messages.
pub fn deserialize_json5_with_path<'de, T>(content: &'de str) -> Result<T>
where
    T: serde::de::Deserialize<'de>,
{
    // Phase 1: parse JSON5 syntax. If this fails, there's no field path.
    let mut deserializer = serde_json5::Deserializer::from_str(content)
        .map_err(|e| Error::Parsing(ParsingError::from(e)))?;

    // Phase 2: deserialize with path tracking.
    serde_path_to_error::deserialize(&mut deserializer).map_err(|path_err| {
        let path = path_err.path().to_string();
        let inner: serde_json5::Error = path_err.into_inner();

        match inner {
            serde_json5::Error::Message { ref msg, .. } => {
                // Check if it's a StructuredError (custom validation).
                // These already have rich messages; don't prepend path.
                if let Ok(structured) = serde_json5::from_str::<StructuredError>(msg) {
                    return Error::Parsing(ParsingError::from(structured));
                }

                // Standard serde error: prepend path if non-empty.
                let message = if path.is_empty() || path == "." {
                    msg.clone()
                } else {
                    format!("{path}: {msg}")
                };
                Error::Parsing(ParsingError::CannotParseConfig(message))
            }
        }
    })
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
    #[error("Duplicate name: {0}")]
    DuplicateName(String),
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

    // -- Parsing error
    #[error(transparent)]
    Parsing(#[from] ParsingError),
    #[error("Serialize error: {0}")]
    Serialize(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn structured_error_round_trips_through_serde_message() {
        fn make_err(msg: &str) -> serde_json5::Error {
            serde::de::Error::custom(msg)
        }

        // A structured error serialized into a serde message is recovered as the
        // matching ParsingError variant rather than a generic parse error.
        let json = StructuredError::MissingExecutionLanguage.json5_message();
        let err = ParsingError::from(make_err(&json));
        assert!(
            matches!(err, ParsingError::MissingExecutionLanguage),
            "expected MissingExecutionLanguage, got {err:?}"
        );
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
