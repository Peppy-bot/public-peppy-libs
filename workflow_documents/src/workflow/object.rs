use serde_json::{Map, Value};
use std::cell::RefCell;
use std::collections::BTreeSet;

use crate::refusal::{At, Refusal};

/// An object of a document read field by field: each field taken is marked,
/// and [`Fields::unknown`] refuses each field no reader took.
pub(super) struct Fields<'a> {
    map: &'a Map<String, Value>,
    pub(super) path: String,
    taken: RefCell<BTreeSet<&'a str>>,
}

impl<'a> Fields<'a> {
    pub(super) fn of(value: &'a Value, path: &str) -> Result<Fields<'a>, Refusal> {
        let Value::Object(map) = value else {
            return Err(Refusal::new(At::field(path), "is not an object"));
        };
        Ok(Fields {
            map,
            path: path.to_owned(),
            taken: RefCell::new(BTreeSet::new()),
        })
    }

    pub(super) fn at(&self, key: &str) -> String {
        join(&self.path, key)
    }

    pub(super) fn get(&self, key: &str) -> Option<&'a Value> {
        let (key, value) = self.map.get_key_value(key)?;
        self.taken.borrow_mut().insert(key.as_str());
        Some(value)
    }

    pub(super) fn require(&self, key: &str) -> Result<&'a Value, Refusal> {
        self.get(key)
            .ok_or_else(|| Refusal::new(At::field(&self.path), format!("has no field `{key}`")))
    }

    pub(super) fn has(&self, key: &str) -> bool {
        self.map.contains_key(key)
    }

    /// Reads the field `key` with serde, naming the field in a refusal.
    pub(super) fn read<T: serde::de::DeserializeOwned>(
        &self,
        key: &str,
    ) -> Result<Option<T>, Refusal> {
        self.get(key)
            .map(|value| read_value(value, &self.at(key)))
            .transpose()
    }

    pub(super) fn read_required<T: serde::de::DeserializeOwned>(
        &self,
        key: &str,
    ) -> Result<T, Refusal> {
        read_value(self.require(key)?, &self.at(key))
    }

    /// A refusal for each field that no reader took.
    pub(super) fn unknown(&self) -> Vec<Refusal> {
        let taken = self.taken.borrow();
        self.map
            .keys()
            .filter(|key| !taken.contains(key.as_str()))
            .map(|key| {
                Refusal::new(
                    At::field(&self.path),
                    format!("has an unknown field `{key}`"),
                )
            })
            .collect()
    }
}

/// Reads `value` with serde, naming `path` in a refusal.
pub(super) fn read_value<T: serde::de::DeserializeOwned>(
    value: &Value,
    path: &str,
) -> Result<T, Refusal> {
    serde_path_to_error::deserialize(value).map_err(|error| {
        let inner = error.path().to_string();
        let at = match inner.as_str() {
            "." => path.to_owned(),
            _ => join(path, &inner),
        };
        Refusal::new(At::field(at), error.into_inner().to_string())
    })
}

pub(super) fn join(path: &str, key: &str) -> String {
    match (path.is_empty(), key.starts_with('[')) {
        (true, _) => key.to_owned(),
        (false, true) => format!("{path}{key}"),
        (false, false) => format!("{path}.{key}"),
    }
}
