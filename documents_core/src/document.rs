use serde::Serialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Value};

use crate::parse::Raw;
use crate::{Digest, Error, Kind, tree_of};

/// A document of a kind: the value tree of its JSON5 text, whose root object
/// holds the root key of the kind's side with the kind as its value.
/// [`Document::parse`] reads one from text, [`Document::of`] builds one from
/// a typed body, and [`Document::read`] gives the typed body.
#[derive(Clone, Debug, PartialEq)]
pub struct Document {
    kind: Kind,
    tree: Value,
}

impl Document {
    /// Parses JSON5 text as a document of `kind`. Refuses a text whose root
    /// key or kind is not `kind`'s before it reads another field, then applies
    /// the refusals of [`crate::parse`].
    pub fn parse(text: &str, kind: Kind) -> Result<Document, Error> {
        let raw = Raw::parse(text)?;
        let found = raw
            .root_field(kind.root_key())?
            .ok_or(Error::MissingRootKey(kind))?;
        if !found.is_text(&kind.tag()) {
            return Err(Error::WrongKind {
                root_key: kind.root_key(),
                found: found.described(),
                expected: kind.tag(),
            });
        }
        raw.into_tree().map(|tree| Document { kind, tree })
    }

    /// The document of `kind` whose fields are those of `body`, after the root
    /// key. `body` serializes to an object that carries no root key of its own,
    /// with the numbers [`tree_of`] accepts.
    pub fn of<T: Serialize>(kind: Kind, body: &T) -> Result<Document, Error> {
        let Value::Object(fields) = tree_of(body)? else {
            return Err(Error::NotAnObject);
        };
        if fields.contains_key(kind.root_key()) {
            return Err(Error::DuplicateKey {
                path: String::new(),
                key: kind.root_key().to_owned(),
            });
        }
        let tree = Value::Object(
            std::iter::once((kind.root_key().to_owned(), Value::String(kind.tag())))
                .chain(fields)
                .collect::<Map<_, _>>(),
        );
        Ok(Document { kind, tree })
    }

    /// Whether the document holds the field `key` after its root key.
    pub fn holds(&self, key: &str) -> bool {
        key != self.kind.root_key()
            && self
                .tree
                .as_object()
                .is_some_and(|fields| fields.contains_key(key))
    }

    /// The typed body of the document: its fields after the root key.
    pub fn read<T: DeserializeOwned>(&self) -> Result<T, Error> {
        let body = self
            .tree
            .as_object()
            .expect("a document's tree is an object that holds its root key")
            .iter()
            .filter(|(key, _)| key.as_str() != self.kind.root_key())
            .map(|(key, value)| (key.clone(), value.clone()))
            .collect::<Map<_, _>>();
        serde_path_to_error::deserialize(Value::Object(body))
            .map_err(|error| Error::Fields(error.to_string()))
    }

    /// The sha256 of the canonical JSON of the document.
    pub fn digest(&self) -> Digest {
        Digest::of(&self.tree)
    }

    /// The document as JSON5 text, fields in the order of the tree.
    pub fn to_json5(&self) -> String {
        json5::to_string(&self.tree).expect("a value tree serializes to JSON5")
    }

    pub fn tree(&self) -> &Value {
        &self.tree
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const WORLD: Kind = Kind::new("simulation_schema", "world", 1);

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Body {
        name: String,
        count: u32,
    }

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Nested {
        parts: Vec<Part>,
    }

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Part {
        size: [f64; 3],
    }

    #[test]
    fn a_document_of_the_kind_reads_its_body() {
        let document = Document::parse(
            r#"{ simulation_schema: "world/v1", name: "a", count: 2 }"#,
            WORLD,
        )
        .unwrap();
        assert_eq!(
            document.read::<Body>().unwrap(),
            Body {
                name: "a".into(),
                count: 2
            }
        );
    }

    #[test]
    fn another_kind_or_root_key_is_refused_before_the_fields() {
        let broken_body = r#"{ simulation_schema: "world/v2", count: "two" }"#;
        assert_eq!(
            Document::parse(broken_body, WORLD).unwrap_err().to_string(),
            r#"the root key `simulation_schema` holds "world/v2"; this parser reads "world/v1""#
        );
        let other_side = r#"{ peppy_schema: "world/v1", count: "two" }"#;
        assert_eq!(
            Document::parse(other_side, WORLD),
            Err(Error::MissingRootKey(WORLD))
        );
        assert_eq!(
            Error::MissingRootKey(WORLD).to_string(),
            r#"the document has no root key `simulation_schema`; add `simulation_schema: "world/v1"` as the first key"#
        );
        assert_eq!(Document::parse("[1]", WORLD), Err(Error::NotAnObject));
        let document =
            Document::parse(r#"{ simulation_schema: "world/v1", a: 1 }"#, WORLD).unwrap();
        assert!(document.holds("a") && !document.holds("b"));
        assert!(
            !document.holds("simulation_schema"),
            "the root key is no field of the body"
        );
        assert_eq!(
            Document::parse("{ simulation_schema: 2 }", WORLD)
                .unwrap_err()
                .to_string(),
            r#"the root key `simulation_schema` holds a number; this parser reads "world/v1""#
        );
    }

    /// The kind is refused before the tree rules read another field.
    #[test]
    fn the_kind_is_refused_before_the_tree_rules() {
        assert_eq!(
            Document::parse(r#"{ simulation_schema: "world/v2", a: NaN }"#, WORLD)
                .unwrap_err()
                .to_string(),
            r#"the root key `simulation_schema` holds "world/v2"; this parser reads "world/v1""#
        );
        assert_eq!(
            Document::parse(r#"{ peppy_schema: "world/v1", a: 1, a: 2 }"#, WORLD),
            Err(Error::MissingRootKey(WORLD))
        );
        assert_eq!(
            Document::parse(r#"{ simulation_schema: "world/v1", a: 1, a: 2 }"#, WORLD)
                .unwrap_err()
                .to_string(),
            "the key `a` appears twice in one object"
        );
    }

    /// A field error names the field by its path from the root of the document.
    #[test]
    fn a_field_error_names_the_field_by_its_path() {
        let document = Document::parse(
            r#"{ simulation_schema: "world/v1", name: "a", count: 2, extra: 1 }"#,
            WORLD,
        )
        .unwrap();
        assert_eq!(
            document.read::<Body>(),
            Err(Error::Fields(
                "extra: unknown field `extra`, expected `name` or `count`".into()
            ))
        );
        let nested = Document::parse(
            r#"{ simulation_schema: "world/v1", parts: [{ size: [1, 2, 3] }, { size: [1, 2] }] }"#,
            WORLD,
        )
        .unwrap();
        assert_eq!(
            nested.read::<Nested>(),
            Err(Error::Fields(
                "parts[1].size: invalid length 2, expected an array of length 3".into()
            ))
        );
    }

    #[test]
    fn a_document_built_from_a_body_puts_the_root_key_first_and_round_trips() {
        let body = Body {
            name: "a".into(),
            count: 2,
        };
        let document = Document::of(WORLD, &body).unwrap();
        assert_eq!(
            document.to_json5(),
            "{\n  simulation_schema: \"world/v1\",\n  name: \"a\",\n  count: 2,\n}"
        );
        assert_eq!(
            Document::parse(&document.to_json5(), WORLD).unwrap(),
            document
        );
        assert_eq!(document.read::<Body>().unwrap(), body);
    }

    #[test]
    fn a_body_that_is_not_an_object_or_carries_the_root_key_is_refused() {
        assert_eq!(Document::of(WORLD, &[1, 2]), Err(Error::NotAnObject));
        let with_root_key = serde_json::json!({"simulation_schema": "world/v1", "name": "a"});
        assert_eq!(
            Document::of(WORLD, &with_root_key),
            Err(Error::DuplicateKey {
                path: String::new(),
                key: "simulation_schema".into()
            })
        );
        let beyond = serde_json::json!({"parts": [{"n": u64::MAX}]});
        assert_eq!(
            Document::of(WORLD, &beyond),
            Err(Error::IntegerBeyondRange {
                path: "parts[0].n".into(),
                integer: "18446744073709551615".into()
            })
        );
    }

    #[test]
    fn a_body_with_a_float_that_is_not_finite_is_refused_at_its_path() {
        let nested = Nested {
            parts: vec![Part {
                size: [1.0, f64::NAN, 3.0],
            }],
        };
        assert_eq!(
            Document::of(WORLD, &nested).unwrap_err().to_string(),
            "parts[0].size[1]: the number NaN is not finite; a document holds finite numbers only"
        );
    }
}
