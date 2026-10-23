use serde::{Deserialize, Serialize};
use std::fmt;
use std::num::NonZeroU32;

use crate::Error;

/// The identity of a document: its name and its revision. A revision never
/// changes; changed content is the next revision.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ManifestFields")]
pub struct Manifest {
    pub name: String,
    pub revision: Revision,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, expecting = "a manifest, { name, revision }")]
struct ManifestFields {
    name: String,
    revision: Revision,
}

impl TryFrom<ManifestFields> for Manifest {
    type Error = Error;

    fn try_from(fields: ManifestFields) -> Result<Manifest, Error> {
        if fields.name.trim().is_empty() {
            return Err(Error::BlankName);
        }
        Ok(Manifest {
            name: fields.name,
            revision: fields.revision,
        })
    }
}

/// A revision number, counted from 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "u32", into = "u32")]
pub struct Revision(NonZeroU32);

impl TryFrom<u32> for Revision {
    type Error = Error;

    fn try_from(number: u32) -> Result<Revision, Error> {
        Revision::new(number).ok_or(Error::ZeroRevision)
    }
}

impl From<Revision> for u32 {
    fn from(revision: Revision) -> u32 {
        revision.get()
    }
}

impl Revision {
    pub const FIRST: Revision = Revision(NonZeroU32::MIN);

    /// The revision `number`, or `None` for 0.
    pub fn new(number: u32) -> Option<Revision> {
        NonZeroU32::new(number).map(Revision)
    }

    pub fn get(self) -> u32 {
        self.0.get()
    }
}

impl fmt::Display for Revision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest(text: &str) -> Result<Manifest, String> {
        serde_json::from_str::<Manifest>(text).map_err(|error| error.to_string())
    }

    #[test]
    fn revision_0_is_refused_with_the_revision_to_write() {
        let refused = manifest(r#"{"name": "world_3f9a1c27", "revision": 0}"#).unwrap_err();
        assert!(
            refused.starts_with(
                "a revision counts from 1, so 0 names none; write `revision: 1` for the first revision"
            ),
            "{refused}"
        );
    }

    #[test]
    fn a_manifest_reads_and_writes_as_name_and_revision() {
        let manifest = manifest(r#"{"name": "world_3f9a1c27", "revision": 3}"#).unwrap();
        assert_eq!(
            manifest,
            Manifest {
                name: "world_3f9a1c27".into(),
                revision: Revision::new(3).unwrap()
            }
        );
        assert_eq!(
            serde_json::to_string(&manifest).unwrap(),
            r#"{"name":"world_3f9a1c27","revision":3}"#
        );
    }

    #[test]
    fn revision_zero_extra_fields_and_a_blank_name_are_refused() {
        assert!(manifest(r#"{"name": "w", "revision": 0}"#).is_err());
        assert!(
            manifest(r#"{"name": "w", "revision": 1, "author": "x"}"#)
                .unwrap_err()
                .contains("unknown field `author`")
        );
        for blank in ["", " ", "\t\n"] {
            assert_eq!(
                manifest(&format!(r#"{{"name": {blank:?}, "revision": 1}}"#)).unwrap_err(),
                "the name is blank; write a name that holds a character other than whitespace"
            );
        }
    }

    #[test]
    fn a_manifest_of_another_type_names_the_fields_it_takes() {
        let refusal = manifest(r#""w""#).unwrap_err();
        assert!(
            refusal.contains("expected a manifest, { name, revision }"),
            "{refusal}"
        );
        let unknown = manifest(r#"{"name": "w", "revision": 1, "zz": 1}"#).unwrap_err();
        assert!(
            unknown.contains("expected `name` or `revision`"),
            "{unknown}"
        );
    }

    #[test]
    fn revisions_count_from_one() {
        assert_eq!(Revision::FIRST.get(), 1);
        assert_eq!(Revision::new(2).unwrap().get(), 2);
        assert_eq!(Revision::new(0), None);
    }
}
