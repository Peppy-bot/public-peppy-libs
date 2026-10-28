use thiserror::Error;

/// Why a text is not a world, a setup record or a list of judges.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum Error {
    #[error(transparent)]
    Document(#[from] documents_core::Error),
    /// A value that breaks a rule of the format; the parser names the path of
    /// the value.
    #[error("{0}")]
    Rule(String),
    #[error("{field}: {reason}")]
    Field { field: String, reason: String },
    #[error("the name `{0}` is not lowercase letters, digits and `_`")]
    Name(String),
    #[error(
        "the robot name `{0}` is not {grammar}",
        grammar = crate::names::peppy_name_grammar()
    )]
    RobotName(String),
    #[error(
        "the robot name `{reserved}` {reason}; give the robot another name",
        reserved = crate::names::RESERVED_PEPPY_NAME,
        reason = crate::names::RESERVED_REASON
    )]
    ReservedRobotName,
    #[error(
        "`{0}` is an object name and a scene object name; the two share one namespace, so give the object another name"
    )]
    NameInBothNamespaces(String),
    #[error(
        "the robot names `{shorter}` and `{longer}` cannot stand together: a robot name followed by `_` begins no other, so rename one of the two robots"
    )]
    RobotNamePrefix { shorter: String, longer: String },
    #[error("the world extends `{name}` revision {revision}, which is not among the bases given")]
    MissingBase { name: String, revision: u32 },
    #[error(
        "the bases of `{0}` extend one another in a cycle; remove the `extends` of one of them"
    )]
    BaseCycle(String),
    #[error(
        "the bases given hold two different documents as `{name}` revision {revision}; a revision never changes, so give one"
    )]
    BaseConflict { name: String, revision: u32 },
    #[error("the base `{name}` revision {revision}: {refusal}")]
    InBase {
        name: String,
        revision: u32,
        refusal: Box<Error>,
    },
    #[error("the world `{0}` extends a base; apply it after it is resolved against its bases")]
    Unresolved(String),
}

impl Error {
    pub(crate) fn rule(reason: impl Into<String>) -> Error {
        Error::Rule(reason.into())
    }

    /// The error of a typed read of `tree`, with the path of the value that
    /// does not read.
    pub(crate) fn fields(error: serde_path_to_error::Error<serde_json::Error>) -> Error {
        Error::Document(documents_core::Error::Fields(error.to_string()))
    }

    pub(crate) fn field(field: impl Into<String>, reason: impl Into<String>) -> Error {
        Error::Field {
            field: field.into(),
            reason: reason.into(),
        }
    }
}
