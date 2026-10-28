//! A typed body to the value tree a document holds: the body serialized into
//! the tree that [`parse`](crate::parse()) reads, and read with its refusals.

use serde::Serialize;
use serde::ser::{self, SerializeSeq, SerializeStruct};
use serde_json::Value;
use std::fmt;

use crate::Error;
use crate::parse::Raw;

/// The value tree of `body` as a document holds it, with its numbers as
/// [`parse`](crate::parse()) gives them. Refuses what [`parse`](crate::parse())
/// refuses of a value, by its path from the root: a float that is not finite,
/// a number beyond 2^53, a key twice in one object and a body nested more than
/// 128 levels deep. A body that does not serialize, such as a map whose keys
/// are not text, is refused with [`Error::Fields`].
pub fn tree_of<T: Serialize>(body: &T) -> Result<Value, Error> {
    body.serialize(RawSerializer)
        .map_err(|error| Error::Fields(error.0))?
        .into_tree()
}

/// Why a body does not serialize, in the words serde or the body gives.
#[derive(Debug)]
struct SerializeError(String);

impl fmt::Display for SerializeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SerializeError {}

impl ser::Error for SerializeError {
    fn custom<T: fmt::Display>(message: T) -> Self {
        SerializeError(message.to_string())
    }
}

/// The integer `value` as the tree holds it.
fn integer(value: i128) -> Raw {
    Raw::Integer {
        negative: value < 0,
        magnitude: value.unsigned_abs(),
    }
}

/// Serializes a body into the tree [`parse`](crate::parse()) reads, in the
/// shapes JSON gives serde's data model: a unit as null, bytes as an array of
/// integers, a unit variant as its name, and a variant with data as an object
/// whose one key is its name.
struct RawSerializer;

impl ser::Serializer for RawSerializer {
    type Ok = Raw;
    type Error = SerializeError;
    type SerializeSeq = ArrayBuilder;
    type SerializeTuple = ArrayBuilder;
    type SerializeTupleStruct = ArrayBuilder;
    type SerializeTupleVariant = VariantBuilder<ArrayBuilder>;
    type SerializeMap = ObjectBuilder;
    type SerializeStruct = ObjectBuilder;
    type SerializeStructVariant = VariantBuilder<ObjectBuilder>;

    fn serialize_bool(self, value: bool) -> Result<Raw, SerializeError> {
        Ok(Raw::Bool(value))
    }

    fn serialize_i8(self, value: i8) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_i16(self, value: i16) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_i32(self, value: i32) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_i64(self, value: i64) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_i128(self, value: i128) -> Result<Raw, SerializeError> {
        Ok(integer(value))
    }

    fn serialize_u8(self, value: u8) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_u16(self, value: u16) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_u32(self, value: u32) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_u64(self, value: u64) -> Result<Raw, SerializeError> {
        Ok(integer(value.into()))
    }

    fn serialize_u128(self, value: u128) -> Result<Raw, SerializeError> {
        Ok(Raw::Integer {
            negative: false,
            magnitude: value,
        })
    }

    fn serialize_f32(self, value: f32) -> Result<Raw, SerializeError> {
        Ok(Raw::Float(value.into()))
    }

    fn serialize_f64(self, value: f64) -> Result<Raw, SerializeError> {
        Ok(Raw::Float(value))
    }

    fn serialize_char(self, value: char) -> Result<Raw, SerializeError> {
        Ok(Raw::String(value.to_string()))
    }

    fn serialize_str(self, value: &str) -> Result<Raw, SerializeError> {
        Ok(Raw::String(value.to_owned()))
    }

    fn serialize_bytes(self, value: &[u8]) -> Result<Raw, SerializeError> {
        Ok(Raw::Array(
            value.iter().map(|&byte| integer(byte.into())).collect(),
        ))
    }

    fn serialize_none(self) -> Result<Raw, SerializeError> {
        Ok(Raw::Null)
    }

    fn serialize_some<T: ?Sized + Serialize>(self, value: &T) -> Result<Raw, SerializeError> {
        value.serialize(self)
    }

    fn serialize_unit(self) -> Result<Raw, SerializeError> {
        Ok(Raw::Null)
    }

    fn serialize_unit_struct(self, _name: &'static str) -> Result<Raw, SerializeError> {
        Ok(Raw::Null)
    }

    fn serialize_unit_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
    ) -> Result<Raw, SerializeError> {
        Ok(Raw::String(variant.to_owned()))
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        value: &T,
    ) -> Result<Raw, SerializeError> {
        value.serialize(self)
    }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        value: &T,
    ) -> Result<Raw, SerializeError> {
        Ok(Raw::Object(vec![(
            variant.to_owned(),
            value.serialize(RawSerializer)?,
        )]))
    }

    fn serialize_seq(self, len: Option<usize>) -> Result<ArrayBuilder, SerializeError> {
        Ok(ArrayBuilder(Vec::with_capacity(len.unwrap_or(0))))
    }

    fn serialize_tuple(self, len: usize) -> Result<ArrayBuilder, SerializeError> {
        self.serialize_seq(Some(len))
    }

    fn serialize_tuple_struct(
        self,
        _name: &'static str,
        len: usize,
    ) -> Result<ArrayBuilder, SerializeError> {
        self.serialize_seq(Some(len))
    }

    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        len: usize,
    ) -> Result<VariantBuilder<ArrayBuilder>, SerializeError> {
        Ok(VariantBuilder {
            variant,
            inner: self.serialize_seq(Some(len))?,
        })
    }

    fn serialize_map(self, len: Option<usize>) -> Result<ObjectBuilder, SerializeError> {
        Ok(ObjectBuilder {
            pairs: Vec::with_capacity(len.unwrap_or(0)),
            key: None,
        })
    }

    fn serialize_struct(
        self,
        _name: &'static str,
        len: usize,
    ) -> Result<ObjectBuilder, SerializeError> {
        self.serialize_map(Some(len))
    }

    fn serialize_struct_variant(
        self,
        _name: &'static str,
        _index: u32,
        variant: &'static str,
        len: usize,
    ) -> Result<VariantBuilder<ObjectBuilder>, SerializeError> {
        Ok(VariantBuilder {
            variant,
            inner: self.serialize_map(Some(len))?,
        })
    }
}

/// The items of an array, in order.
struct ArrayBuilder(Vec<Raw>);

impl ser::SerializeSeq for ArrayBuilder {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_element<T: ?Sized + Serialize>(
        &mut self,
        value: &T,
    ) -> Result<(), SerializeError> {
        self.0.push(value.serialize(RawSerializer)?);
        Ok(())
    }

    fn end(self) -> Result<Raw, SerializeError> {
        Ok(Raw::Array(self.0))
    }
}

impl ser::SerializeTuple for ArrayBuilder {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_element<T: ?Sized + Serialize>(
        &mut self,
        value: &T,
    ) -> Result<(), SerializeError> {
        SerializeSeq::serialize_element(self, value)
    }

    fn end(self) -> Result<Raw, SerializeError> {
        SerializeSeq::end(self)
    }
}

impl ser::SerializeTupleStruct for ArrayBuilder {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), SerializeError> {
        SerializeSeq::serialize_element(self, value)
    }

    fn end(self) -> Result<Raw, SerializeError> {
        SerializeSeq::end(self)
    }
}

/// The pairs of an object, in order, and the key of the pair whose value
/// comes next.
struct ObjectBuilder {
    pairs: Vec<(String, Raw)>,
    key: Option<String>,
}

impl ser::SerializeMap for ObjectBuilder {
    type Ok = Raw;
    type Error = SerializeError;

    /// Takes a key that serializes as text; refuses any other.
    fn serialize_key<T: ?Sized + Serialize>(&mut self, key: &T) -> Result<(), SerializeError> {
        match key.serialize(RawSerializer)? {
            Raw::String(text) => {
                self.key = Some(text);
                Ok(())
            }
            other => Err(SerializeError(format!(
                "a key of a map is {}; the keys of a document are text",
                other.described()
            ))),
        }
    }

    fn serialize_value<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), SerializeError> {
        let key = self
            .key
            .take()
            .expect("serde serializes the key of a pair before its value");
        self.pairs.push((key, value.serialize(RawSerializer)?));
        Ok(())
    }

    fn end(self) -> Result<Raw, SerializeError> {
        Ok(Raw::Object(self.pairs))
    }
}

impl ser::SerializeStruct for ObjectBuilder {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), SerializeError> {
        self.pairs
            .push((key.to_owned(), value.serialize(RawSerializer)?));
        Ok(())
    }

    fn end(self) -> Result<Raw, SerializeError> {
        Ok(Raw::Object(self.pairs))
    }
}

/// The data of an enum variant, `inner`, under its name `variant`.
struct VariantBuilder<B> {
    variant: &'static str,
    inner: B,
}

impl ser::SerializeTupleVariant for VariantBuilder<ArrayBuilder> {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), SerializeError> {
        SerializeSeq::serialize_element(&mut self.inner, value)
    }

    fn end(self) -> Result<Raw, SerializeError> {
        let data = SerializeSeq::end(self.inner)?;
        Ok(Raw::Object(vec![(self.variant.to_owned(), data)]))
    }
}

impl ser::SerializeStructVariant for VariantBuilder<ObjectBuilder> {
    type Ok = Raw;
    type Error = SerializeError;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        key: &'static str,
        value: &T,
    ) -> Result<(), SerializeError> {
        SerializeStruct::serialize_field(&mut self.inner, key, value)
    }

    fn end(self) -> Result<Raw, SerializeError> {
        let data = SerializeStruct::end(self.inner)?;
        Ok(Raw::Object(vec![(self.variant.to_owned(), data)]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::collections::BTreeMap;

    #[derive(Serialize)]
    enum Shape {
        Unit,
        Newtype(f64),
        Tuple(i8, String),
        Struct { at: [u32; 2], label: Option<char> },
    }

    #[derive(Serialize)]
    struct Shapes {
        shapes: Vec<Shape>,
        empty: (),
        bytes: Bytes,
        missing: Option<u8>,
        wrapped: Wrapped,
        pair: (bool, i128),
        small: (i16, i32, u128),
        single: f32,
        marker: Marker,
    }

    #[derive(Serialize)]
    struct Marker;

    #[derive(Serialize)]
    struct Wrapped(u16);

    /// Bytes that serialize with `serialize_bytes`.
    struct Bytes(Vec<u8>);

    impl Serialize for Bytes {
        fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            serializer.serialize_bytes(&self.0)
        }
    }

    #[test]
    fn every_shape_of_a_body_is_the_tree_serde_json_gives_it() {
        let body = Shapes {
            shapes: vec![
                Shape::Unit,
                Shape::Newtype(0.5),
                Shape::Tuple(-3, "x".into()),
                Shape::Struct {
                    at: [1, 2],
                    label: Some('q'),
                },
            ],
            empty: (),
            bytes: Bytes(vec![0, 255]),
            missing: None,
            wrapped: Wrapped(7),
            pair: (true, -9),
            small: (-300, 70_000, 5),
            single: 0.25,
            marker: Marker,
        };
        assert_eq!(tree_of(&body), Ok(serde_json::to_value(&body).unwrap()));
    }

    #[test]
    fn a_float_beyond_2_to_the_53_is_refused_at_its_path_whatever_its_size() {
        for number in [1e17, -1e17, 3.5e38, 1e40, -1e40, f64::MAX] {
            assert_eq!(
                tree_of(&BTreeMap::from([("scale", number)])),
                Err(Error::FloatBeyondRange {
                    path: "scale".into(),
                    number
                }),
                "{number}"
            );
        }
    }

    #[test]
    fn a_body_nested_more_than_128_levels_deep_is_refused_at_its_path() {
        let nested = |levels: usize| (0..levels).fold(json!(1), |inner, _| json!([inner]));
        assert_eq!(tree_of(&nested(128)), Ok(nested(128)));
        assert_eq!(
            tree_of(&nested(201)),
            Err(Error::NestedTooDeep {
                path: "[0]".repeat(128)
            })
        );
    }

    #[test]
    fn a_map_whose_keys_are_not_text_is_refused() {
        assert_eq!(
            tree_of(&BTreeMap::from([(1, "a")])),
            Err(Error::Fields(
                "a key of a map is a number; the keys of a document are text".into()
            ))
        );
    }

    #[test]
    fn the_tree_of_a_body_holds_its_numbers_as_parse_gives_them() {
        assert_eq!(
            tree_of(&BTreeMap::from([("a", vec![1.0, 2.5])])),
            Ok(json!({"a": [1, 2.5]}))
        );
        assert_eq!(
            tree_of(&BTreeMap::from([("n", u64::MAX)])),
            Err(Error::IntegerBeyondRange {
                path: "n".into(),
                integer: "18446744073709551615".into()
            })
        );
        assert_eq!(
            tree_of(&[0.5, f64::NAN]).unwrap_err().to_string(),
            "[1]: the number NaN is not finite; a document holds finite numbers only"
        );
    }

    #[test]
    fn a_finite_body_passes_and_a_nan_or_an_infinity_is_refused_at_its_path() {
        assert_eq!(
            tree_of(&json!({"a": [1, 2.5, "x", null]})),
            Ok(json!({"a": [1, 2.5, "x", null]}))
        );
        let nested = BTreeMap::from([("crate", vec![0.5, f64::NAN])]);
        assert_eq!(
            tree_of(&nested).unwrap_err().to_string(),
            "crate[1]: the number NaN is not finite; a document holds finite numbers only"
        );
        assert_eq!(
            tree_of(&f64::NEG_INFINITY).unwrap_err().to_string(),
            "the number is an infinity or beyond the range of a double; a document holds finite numbers only"
        );
        assert_eq!(
            tree_of(&(1u8, Some(f32::INFINITY)))
                .unwrap_err()
                .to_string(),
            "[1]: the number is an infinity or beyond the range of a double; a document holds finite numbers only"
        );
    }
}
