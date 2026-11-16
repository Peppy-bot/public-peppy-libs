//! The reference to an asset of a catalogue: its kind, its id, and the
//! version of its entry when the reference names one. The field of a world
//! document that names an asset.

use std::fmt;
use std::str::FromStr;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::{Error, Name};

/// The kind of an asset that a world names: its scene, or one of its objects.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AssetKind {
    Scene,
    Object,
}

impl AssetKind {
    /// The kind as an asset id writes it, before the `/`.
    pub fn as_str(self) -> &'static str {
        match self {
            AssetKind::Scene => "scene",
            AssetKind::Object => "object",
        }
    }

    fn parse(written: &str) -> Option<AssetKind> {
        match written {
            "scene" => Some(AssetKind::Scene),
            "object" => Some(AssetKind::Object),
            _ => None,
        }
    }
}

impl fmt::Display for AssetKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The version of the entry of an asset in a catalogue release: 16
/// lowercase hex digits, which the catalogue tool computes from the facts
/// of the entry.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AssetVersion(String);

impl AssetVersion {
    /// How many hex digits a version has.
    pub const DIGITS: usize = 16;

    /// The version that `text` writes, or `None` for a text that is not 16
    /// lowercase hex digits.
    pub fn parse(text: &str) -> Option<AssetVersion> {
        let lowercase_hex = text.len() == AssetVersion::DIGITS
            && text.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'));
        lowercase_hex.then(|| AssetVersion(text.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for AssetVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl Serialize for AssetVersion {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for AssetVersion {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<AssetVersion, D::Error> {
        let text = String::deserialize(deserializer)?;
        AssetVersion::parse(&text).ok_or_else(|| {
            serde::de::Error::custom(format!(
                "`{text}` is no version of an asset: write {} lowercase hex digits",
                AssetVersion::DIGITS
            ))
        })
    }
}

/// The separator between the id of an asset and its version.
const VERSION_SEPARATOR: char = '@';

/// A reference to a scene or to an object of a catalogue, as an asset id
/// writes it: `<kind>/<id>`, which names the live entry of the asset, or
/// `<kind>/<id>@<version>`, which names the entry of one version, live or
/// retired.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AssetRef {
    kind: AssetKind,
    id: String,
    version: Option<AssetVersion>,
}

impl AssetRef {
    /// The reference that `text` writes. Refuses a text of another form: a
    /// kind that is not `scene` or `object`, an id that is empty or holds
    /// `/` or `@`, and a version that is not 16 lowercase hex digits.
    pub fn parse(text: &str) -> Result<AssetRef, Error> {
        let refused = || {
            Error::rule(format!(
                "`{text}` is not an asset id; write `scene/<id>` or `object/<id>`, and `@<version>` after it for one version of the asset ({} lowercase hex digits)",
                AssetVersion::DIGITS
            ))
        };
        let (kind, named) = text.split_once('/').ok_or_else(refused)?;
        let kind = AssetKind::parse(kind).ok_or_else(refused)?;
        let (id, version) = match named.split_once(VERSION_SEPARATOR) {
            Some((id, version)) => (id, Some(AssetVersion::parse(version).ok_or_else(refused)?)),
            None => (named, None),
        };
        if id.is_empty() || id.contains(['/', VERSION_SEPARATOR]) {
            return Err(refused());
        }
        Ok(AssetRef {
            kind,
            id: id.to_owned(),
            version,
        })
    }

    pub fn kind(&self) -> AssetKind {
        self.kind
    }

    /// The id of the asset in the registry of its kind.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// The version that the reference names, `None` for the live version.
    pub fn version(&self) -> Option<&AssetVersion> {
        self.version.as_ref()
    }

    /// The reference to the live version of the asset: the same kind and
    /// id, with no version.
    pub fn without_version(&self) -> AssetRef {
        AssetRef {
            version: None,
            ..self.clone()
        }
    }

    /// The reference to `version` of the asset.
    pub fn at_version(&self, version: AssetVersion) -> AssetRef {
        AssetRef {
            version: Some(version),
            ..self.clone()
        }
    }
}

impl fmt::Display for AssetRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.kind, self.id)?;
        match &self.version {
            Some(version) => write!(f, "{VERSION_SEPARATOR}{version}"),
            None => Ok(()),
        }
    }
}

impl FromStr for AssetRef {
    type Err = Error;

    fn from_str(text: &str) -> Result<AssetRef, Error> {
        AssetRef::parse(text)
    }
}

impl Serialize for AssetRef {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for AssetRef {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<AssetRef, D::Error> {
        let text = String::deserialize(deserializer)?;
        AssetRef::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// The field of a world document, or of a world pattern, that names an
/// asset: the scene, or the object of a name. A refusal names the field as
/// the document writes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssetField<'a> {
    Scene,
    Object(&'a Name),
}

impl fmt::Display for AssetField<'_> {
    /// "scene.asset", "objects.cup.asset".
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AssetField::Scene => f.write_str("scene.asset"),
            AssetField::Object(name) => write!(f, "objects.{name}.asset"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VERSION: &str = "9f2c41d07a3be518";

    #[test]
    fn an_asset_field_is_written_as_the_document_writes_it() {
        let cup = Name::parse("cup").unwrap();
        assert_eq!(AssetField::Scene.to_string(), "scene.asset");
        assert_eq!(AssetField::Object(&cup).to_string(), "objects.cup.asset");
    }

    #[test]
    fn an_asset_id_reads_and_writes_with_and_without_a_version() {
        let live = AssetRef::parse("object/cup").unwrap();
        assert_eq!(
            (live.kind(), live.id(), live.version()),
            (AssetKind::Object, "cup", None)
        );
        assert_eq!(live.to_string(), "object/cup");

        let written = format!("scene/kitchen@{VERSION}");
        let versioned = AssetRef::parse(&written).unwrap();
        assert_eq!(
            (
                versioned.kind(),
                versioned.id(),
                versioned.version().map(AssetVersion::as_str)
            ),
            (AssetKind::Scene, "kitchen", Some(VERSION))
        );
        assert_eq!(versioned.to_string(), written);
        assert_eq!(written.parse::<AssetRef>().unwrap(), versioned);
    }

    #[test]
    fn a_reference_takes_and_leaves_a_version() {
        let live = AssetRef::parse("object/cup").unwrap();
        let version = AssetVersion::parse(VERSION).unwrap();
        let versioned = live.at_version(version.clone());
        assert_eq!(versioned.to_string(), format!("object/cup@{VERSION}"));
        assert_eq!(versioned.version(), Some(&version));
        assert_eq!(versioned.without_version(), live);
        assert_eq!(live.without_version(), live);
    }

    #[test]
    fn an_asset_id_of_another_form_is_refused() {
        for bad in [
            "",
            "cup",
            "object/",
            "/cup",
            "template/cafe",
            "robot/openarm_v2",
            "Object/cup",
            "object/cup/lid",
            "object/cup@",
            "object/@9f2c41d07a3be518",
            "object/cup@9f2c41d07a3be51",
            "object/cup@9f2c41d07a3be5189",
            "object/cup@9F2C41D07A3BE518",
            "object/cup@9f2c41d07a3be51g",
            "object/cup@9f2c41d07a3be518@9f2c41d07a3be518",
            "object@9f2c41d07a3be518/cup",
        ] {
            assert_eq!(
                AssetRef::parse(bad).unwrap_err().to_string(),
                format!(
                    "`{bad}` is not an asset id; write `scene/<id>` or `object/<id>`, and `@<version>` after it for one version of the asset (16 lowercase hex digits)"
                ),
                "{bad}"
            );
        }
    }

    #[test]
    fn a_reference_is_a_string_in_a_document() {
        let written = format!("\"object/cup@{VERSION}\"");
        let reference: AssetRef = serde_json::from_str(&written).unwrap();
        assert_eq!(serde_json::to_string(&reference).unwrap(), written);
        assert!(
            serde_json::from_str::<AssetRef>("\"cup\"")
                .unwrap_err()
                .to_string()
                .starts_with("`cup` is not an asset id")
        );
    }

    #[test]
    fn a_version_is_16_lowercase_hex_digits() {
        assert_eq!(AssetVersion::parse(VERSION).unwrap().to_string(), VERSION);
        for bad in ["", "9f2c41d0", "9F2C41D07A3BE518", "9f2c41d07a3be518 "] {
            assert_eq!(AssetVersion::parse(bad), None, "{bad}");
        }
        assert_eq!(
            serde_json::from_str::<AssetVersion>("\"9f2c\"")
                .unwrap_err()
                .to_string(),
            "`9f2c` is no version of an asset: write 16 lowercase hex digits"
        );
    }
}
