use thiserror::Error;

use crate::Kind;

/// Why a text is not a document, or not a document of the kind asked for. A
/// refusal of a value names its path from the root as `serde_path_to_error`
/// prints it, `objects.crate.mass` or `parts[1]`; a refusal of the root, and
/// of a typed body that does not serialize ([`Error::Fields`]), has no path.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum Error {
    #[error("the text is not JSON5: {0}")]
    Syntax(String),
    #[error(
        "the text nests objects and arrays more than {depth} levels deep, at line {line} column {column}",
        depth = crate::parse::MAX_DEPTH
    )]
    TooDeep { line: usize, column: usize },
    #[error(
        "{}the value nests objects and arrays more than {depth} levels deep",
        at(.path),
        depth = crate::parse::MAX_DEPTH
    )]
    NestedTooDeep { path: String },
    #[error("{}the key `{key}` appears twice in one object", at(.path))]
    DuplicateKey { path: String, key: String },
    #[error(
        "{}{}; a document holds finite numbers only",
        at(.path),
        not_finite(*.number)
    )]
    NotFinite { path: String, number: f64 },
    /// `integer` is the integer in decimal, whatever its spelling.
    #[error(
        "{}the integer {integer} is beyond 2^53, the largest magnitude of a number in a document",
        at(.path)
    )]
    IntegerBeyondRange { path: String, integer: String },
    #[error(
        "{}the number {number:e} is beyond 2^53, the largest magnitude of a number in a document",
        at(.path)
    )]
    FloatBeyondRange { path: String, number: f64 },
    #[error("the document is not an object")]
    NotAnObject,
    #[error("the name is blank; write a name that holds a character other than whitespace")]
    BlankName,
    #[error(
        "a revision counts from 1, so 0 names none; write `revision: 1` for the first revision"
    )]
    ZeroRevision,
    #[error(
        "the document has no root key `{root_key}`; add `{0}` as the first key",
        root_key = .0.root_key()
    )]
    MissingRootKey(Kind),
    #[error("the root key `{root_key}` holds {found}; this parser reads \"{expected}\"")]
    WrongKind {
        root_key: &'static str,
        found: String,
        expected: String,
    },
    /// A field of the body does not match its type, named by its path from the
    /// root; or a body that does not serialize, in the words of its refusal.
    #[error("{0}")]
    Fields(String),
}

/// What is wrong with a number that is not finite. A float spelling beyond the
/// range of a double reads as an infinity, so the two read alike.
fn not_finite(number: f64) -> &'static str {
    if number.is_nan() {
        return "the number NaN is not finite";
    }
    "the number is an infinity or beyond the range of a double"
}

/// The start of a refusal at `path`: the path and a colon, or nothing at the
/// root.
fn at(path: &str) -> String {
    if path.is_empty() {
        return String::new();
    }
    format!("{path}: ")
}
