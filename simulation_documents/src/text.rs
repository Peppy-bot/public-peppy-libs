//! The JSON5 texts the members of `simulation_world` carry: a setup record,
//! the first stands, a list of judges and their verdicts.

use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::Error;

/// The value JSON5 `text` writes, with the refusals of
/// [`documents_core::parse`] and the path of a field that does not read.
pub(crate) fn from_json5<T: DeserializeOwned>(text: &str) -> Result<T, Error> {
    let tree = documents_core::parse(text)?;
    serde_path_to_error::deserialize(tree).map_err(Error::fields)
}

/// `value` as JSON5 text, two-space indented, keys in field order. Refuses
/// what [`documents_core::tree_of`] refuses, by its path: a float that is not
/// finite, a number beyond 2^53 and a value nested more than 128 levels deep.
pub(crate) fn to_json5<T: Serialize>(value: &T) -> Result<String, Error> {
    let tree = documents_core::tree_of(value)?;
    Ok(json5::to_string(&tree).expect("a tree serializes to JSON5"))
}
