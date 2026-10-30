use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use sha2::{Digest as _, Sha256};
use std::fmt;

use crate::Error;

/// The sha256 of a document's canonical JSON (RFC 8785): the identity of its
/// content, the same for every spelling of the same tree. It serializes as
/// its 64 lowercase hex digits.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Digest([u8; 32]);

impl Digest {
    /// The digest of a tree that [`crate::parse`] or [`crate::Document`] built.
    pub(crate) fn of(tree: &Value) -> Digest {
        Digest(Sha256::digest(canonical_json(tree)).into())
    }

    /// The digest of the JSON5 text of a document or any other file, read by
    /// the refusals of [`crate::parse`] and no other rule: equal to the
    /// [`crate::Document::digest`] of the same text.
    pub fn of_text(text: &str) -> Result<Digest, Error> {
        crate::parse(text).map(|tree| Digest::of(&tree))
    }

    /// The digest written as 64 lowercase hex digits, as sha256 tools print
    /// them, or `None` for any other text.
    pub fn parse(hex: &str) -> Option<Digest> {
        let digits = hex.as_bytes();
        let lowercase_hex = digits.len() == 64
            && digits
                .iter()
                .all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'));
        if !lowercase_hex {
            return None;
        }
        let mut bytes = [0u8; 32];
        for (byte, pair) in bytes.iter_mut().zip(digits.chunks(2)) {
            *byte = u8::from_str_radix(std::str::from_utf8(pair).ok()?, 16).ok()?;
        }
        Some(Digest(bytes))
    }
}

/// The canonical JSON of a tree that [`crate::parse`] or [`crate::Document`]
/// built: keys sorted, numbers in their shortest form, no whitespace.
fn canonical_json(tree: &Value) -> Vec<u8> {
    serde_jcs::to_vec(tree).expect(
        "a document's tree holds finite numbers under string keys, which the canonical form writes",
    )
}

impl fmt::Display for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.iter().try_for_each(|byte| write!(f, "{byte:02x}"))
    }
}

impl Serialize for Digest {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for Digest {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Digest, D::Error> {
        let text = String::deserialize(deserializer)?;
        Digest::parse(&text).ok_or_else(|| {
            serde::de::Error::custom(format!(
                "`{text}` is no digest: write 64 lowercase hex digits"
            ))
        })
    }
}

impl fmt::Debug for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Digest({self})")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn the_digest_is_the_sha256_of_the_canonical_json() {
        let tree = json!({"b": [1, 2.5], "a": "x"});
        assert_eq!(canonical_json(&tree), br#"{"a":"x","b":[1,2.5]}"#);
        assert_eq!(
            Digest::of(&tree).to_string(),
            "66efddae6a97500318e4c6cdc4bc04149f340a165a7ef2d830393048b67b7a31"
        );
    }

    #[test]
    fn key_order_and_integer_spelling_do_not_change_the_digest() {
        let one = Digest::of(&json!({"a": 1, "b": {"d": 1.0, "c": 2}}));
        let other = Digest::of(&json!({"b": {"c": 2, "d": 1}, "a": 1}));
        assert_eq!(one, other);
    }

    #[test]
    fn lowercase_hex_round_trips_and_other_texts_are_refused() {
        let digest = Digest::of(&json!({}));
        assert_eq!(Digest::parse(&digest.to_string()), Some(digest));
        assert_eq!(Digest::parse("504661c4"), None);
        assert_eq!(Digest::parse(&"g".repeat(64)), None);
        assert_eq!(Digest::parse(&digest.to_string().to_uppercase()), None);
        assert_eq!(Digest::parse(&format!("{digest}0")), None);
    }

    #[test]
    fn the_digest_of_a_text_is_that_of_its_document_and_serializes_as_hex() {
        let text = r#"{ simulation_schema: "world/v1", b: [1, 2.5], a: "x" }"#;
        let kind = crate::Kind::new("simulation_schema", "world", 1);
        let document = crate::Document::parse(text, kind).unwrap();
        assert_eq!(Digest::of_text(text).unwrap(), document.digest());
        assert!(Digest::of_text("{ a: 1, a: 2 }").is_err());
        let digest = document.digest();
        let written = serde_json::to_string(&digest).unwrap();
        assert_eq!(written, format!("\"{digest}\""));
        assert_eq!(serde_json::from_str::<Digest>(&written).unwrap(), digest);
        assert!(serde_json::from_str::<Digest>("\"504661c4\"").is_err());
    }
}
