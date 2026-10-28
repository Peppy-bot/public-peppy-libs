//! JSON5 text to a value tree that the canonical form (RFC 8785) holds exactly.

use serde::de::{self, Deserialize, Deserializer, MapAccess, SeqAccess, Visitor};
use serde_json::{Map, Number, Value};
use std::fmt;

use crate::Error;

/// The largest magnitude of a number in a document: 2^53, the largest integer
/// an IEEE double represents without loss. The canonical form writes every
/// number as a double, so within this bound every integer is exact. An
/// integer spelling reads as that integer; a float spelling reads as the f64
/// it names, and a float that names an integer within 2^53 reads as that
/// integer.
pub(crate) const MAX_EXACT_INTEGER: i64 = 1 << 53;

/// The deepest a text nests objects and arrays: the limit serde_json sets.
/// The parse descends once per level, so the bound keeps a deep text within
/// the 2 MiB stack of a tokio worker thread.
pub(crate) const MAX_DEPTH: usize = 128;

/// The path of the root of a tree.
const ROOT: &str = "";

/// Parses JSON5 text into a value tree.
///
/// Refuses a text that nests objects and arrays more than 128 levels deep
/// before it parses it. Refuses a key twice in one object, NaN, an infinity,
/// and a number beyond 2^53 in either direction, each with its own [`Error`]
/// naming the path of the value. Numbers come out as the canonical form holds
/// them: a float that names an integer is that integer. JSON5 reads integers
/// from -2^127 to 2^128 - 1 and refuses one beyond them as [`Error::Syntax`].
pub fn parse(text: &str) -> Result<Value, Error> {
    Raw::parse(text)?.into_tree()
}

/// The path of the value under `key` of the object at `path`.
fn key_path(path: &str, key: &str) -> String {
    if path.is_empty() {
        return key.to_owned();
    }
    format!("{path}.{key}")
}

/// The path of the item at `index` of the array at `path`.
fn index_path(path: &str, index: usize) -> String {
    format!("{path}[{index}]")
}

/// The integer of sign `negative` and `magnitude`, within
/// [`MAX_EXACT_INTEGER`].
fn exact_integer(path: &str, negative: bool, magnitude: u128) -> Result<Number, Error> {
    if magnitude > u128::from(MAX_EXACT_INTEGER.unsigned_abs()) {
        return Err(Error::IntegerBeyondRange {
            path: path.to_owned(),
            integer: format!("{}{magnitude}", if negative { "-" } else { "" }),
        });
    }
    let magnitude = i64::try_from(magnitude).expect("a magnitude within 2^53 is an i64");
    Ok(Number::from(if negative { -magnitude } else { magnitude }))
}

/// A finite float within [`MAX_EXACT_INTEGER`], as the integer it names when
/// it names one.
fn exact_float(path: &str, float: f64) -> Result<Number, Error> {
    if !float.is_finite() {
        return Err(Error::NotFinite {
            path: path.to_owned(),
            number: float,
        });
    }
    if float.abs() > MAX_EXACT_INTEGER as f64 {
        return Err(Error::FloatBeyondRange {
            path: path.to_owned(),
            number: float,
        });
    }
    if float.fract() == 0.0 {
        return Ok(Number::from(float as i64));
    }
    Ok(Number::from_f64(float).expect("a finite float"))
}

/// Where a scan of JSON5 text is: in code, after a `/` in code, in a string
/// of `quote`, or in a comment.
#[derive(Clone, Copy)]
enum Scan {
    Code,
    /// A `/` in code, which begins a comment when `/` or `*` follows it.
    Slash,
    /// In a string; `escaped` when the previous character is a `\`.
    Text {
        quote: char,
        escaped: bool,
    },
    LineComment,
    /// In a block comment; `star` when the previous character is a `*`.
    BlockComment {
        star: bool,
    },
}

/// Refuses a text that nests objects and arrays more than [`MAX_DEPTH`] levels
/// deep, at the line and column of the bracket that passes the limit. A
/// bracket in a string or a comment nests nothing.
fn check_depth(text: &str) -> Result<(), Error> {
    let mut scan = Scan::Code;
    let mut depth = 0usize;
    let (mut line, mut column) = (1usize, 0usize);
    let mut after_carriage_return = false;
    for character in text.chars() {
        if ends_a_line(character) {
            if !(character == '\n' && after_carriage_return) {
                line += 1;
            }
            column = 0;
        } else {
            column += 1;
        }
        after_carriage_return = character == '\r';
        scan = match (scan, character) {
            (Scan::Slash, '/') => Scan::LineComment,
            (Scan::Slash, '*') => Scan::BlockComment { star: false },
            (Scan::Code, '/') => Scan::Slash,
            (Scan::Code | Scan::Slash, '"' | '\'') => Scan::Text {
                quote: character,
                escaped: false,
            },
            (Scan::Code | Scan::Slash, '{' | '[') => {
                depth += 1;
                if depth > MAX_DEPTH {
                    return Err(Error::TooDeep { line, column });
                }
                Scan::Code
            }
            (Scan::Code | Scan::Slash, '}' | ']') => {
                depth = depth.saturating_sub(1);
                Scan::Code
            }
            (Scan::Code | Scan::Slash, _) => Scan::Code,
            (
                Scan::Text {
                    quote,
                    escaped: false,
                },
                '\\',
            ) => Scan::Text {
                quote,
                escaped: true,
            },
            (
                Scan::Text {
                    quote,
                    escaped: false,
                },
                closing,
            ) if closing == quote => Scan::Code,
            (Scan::Text { quote, .. }, _) => Scan::Text {
                quote,
                escaped: false,
            },
            (Scan::LineComment, end) if ends_a_line(end) => Scan::Code,
            (Scan::LineComment, _) => Scan::LineComment,
            (Scan::BlockComment { star: true }, '/') => Scan::Code,
            (Scan::BlockComment { .. }, inside) => Scan::BlockComment {
                star: inside == '*',
            },
        };
    }
    Ok(())
}

/// Whether `character` ends a line of JSON5 text: a line feed, a carriage
/// return, a line separator or a paragraph separator. A carriage return and
/// the line feed after it end one line.
fn ends_a_line(character: char) -> bool {
    matches!(character, '\n' | '\r' | '\u{2028}' | '\u{2029}')
}

/// The tree as JSON5 gives it, before the refusals: objects keep every pair in
/// order, numbers keep their lexical class.
pub(crate) enum Raw {
    Null,
    Bool(bool),
    /// An integer: its sign and its magnitude, from -2^127 to 2^128 - 1 as
    /// JSON5 reads them.
    Integer {
        negative: bool,
        magnitude: u128,
    },
    Float(f64),
    String(String),
    Array(Vec<Raw>),
    Object(Vec<(String, Raw)>),
}

impl Raw {
    /// The tree of JSON5 `text`, before the refusals of [`parse`] that read the
    /// tree. Refuses a text deeper than [`MAX_DEPTH`] before JSON5 reads it.
    pub(crate) fn parse(text: &str) -> Result<Raw, Error> {
        check_depth(text)?;
        json5::from_str(text).map_err(|error| Error::Syntax(error.to_string()))
    }

    /// The value of the first pair of the root object whose key is `key`, or
    /// `None` when no pair has it. Refuses a root that is not an object.
    pub(crate) fn root_field(&self, key: &str) -> Result<Option<&Raw>, Error> {
        let Raw::Object(pairs) = self else {
            return Err(Error::NotAnObject);
        };
        Ok(pairs
            .iter()
            .find(|(name, _)| name == key)
            .map(|(_, value)| value))
    }

    /// Whether this value is the text `text`.
    pub(crate) fn is_text(&self, text: &str) -> bool {
        matches!(self, Raw::String(string) if string == text)
    }

    /// The value as a refusal names it: a text in quotes, or what kind of
    /// value it is.
    pub(crate) fn described(&self) -> String {
        match self {
            Raw::String(string) => Value::String(string.clone()).to_string(),
            Raw::Null => "null".to_owned(),
            Raw::Bool(_) => "a boolean".to_owned(),
            Raw::Integer { .. } | Raw::Float(_) => "a number".to_owned(),
            Raw::Array(_) => "an array".to_owned(),
            Raw::Object(_) => "an object".to_owned(),
        }
    }

    /// The tree of this value as the root, with the refusals of [`parse`].
    pub(crate) fn into_tree(self) -> Result<Value, Error> {
        self.into_value(ROOT, 0)
    }

    /// The tree of this value at `path`, inside `depth` objects and arrays,
    /// with the refusals of [`parse`].
    fn into_value(self, path: &str, depth: usize) -> Result<Value, Error> {
        match self {
            Raw::Null => Ok(Value::Null),
            Raw::Bool(bool) => Ok(Value::Bool(bool)),
            Raw::Integer {
                negative,
                magnitude,
            } => exact_integer(path, negative, magnitude).map(Value::Number),
            Raw::Float(float) => exact_float(path, float).map(Value::Number),
            Raw::String(string) => Ok(Value::String(string)),
            Raw::Array(items) => {
                let depth = nested(path, depth)?;
                items
                    .into_iter()
                    .enumerate()
                    .map(|(index, item)| item.into_value(&index_path(path, index), depth))
                    .collect::<Result<_, _>>()
                    .map(Value::Array)
            }
            Raw::Object(pairs) => {
                let depth = nested(path, depth)?;
                pairs
                    .into_iter()
                    .try_fold(Map::with_capacity(0), |mut fields, (key, raw)| {
                        let value = raw.into_value(&key_path(path, &key), depth)?;
                        if fields.insert(key.clone(), value).is_some() {
                            return Err(Error::DuplicateKey {
                                path: path.to_owned(),
                                key,
                            });
                        }
                        Ok(fields)
                    })
                    .map(Value::Object)
            }
        }
    }
}

/// The depth of an object or an array at `path` inside `depth` of them,
/// refused beyond [`MAX_DEPTH`].
fn nested(path: &str, depth: usize) -> Result<usize, Error> {
    let depth = depth + 1;
    if depth > MAX_DEPTH {
        return Err(Error::NestedTooDeep {
            path: path.to_owned(),
        });
    }
    Ok(depth)
}

impl<'de> Deserialize<'de> for Raw {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(RawVisitor)
    }
}

struct RawVisitor;

impl<'de> Visitor<'de> for RawVisitor {
    type Value = Raw;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a JSON5 value")
    }

    fn visit_bool<E: de::Error>(self, value: bool) -> Result<Raw, E> {
        Ok(Raw::Bool(value))
    }

    fn visit_i64<E: de::Error>(self, value: i64) -> Result<Raw, E> {
        self.visit_i128(i128::from(value))
    }

    fn visit_u64<E: de::Error>(self, value: u64) -> Result<Raw, E> {
        self.visit_u128(u128::from(value))
    }

    fn visit_i128<E: de::Error>(self, value: i128) -> Result<Raw, E> {
        Ok(Raw::Integer {
            negative: value < 0,
            magnitude: value.unsigned_abs(),
        })
    }

    fn visit_u128<E: de::Error>(self, value: u128) -> Result<Raw, E> {
        Ok(Raw::Integer {
            negative: false,
            magnitude: value,
        })
    }

    fn visit_f64<E: de::Error>(self, value: f64) -> Result<Raw, E> {
        Ok(Raw::Float(value))
    }

    fn visit_str<E: de::Error>(self, value: &str) -> Result<Raw, E> {
        Ok(Raw::String(value.to_owned()))
    }

    fn visit_string<E: de::Error>(self, value: String) -> Result<Raw, E> {
        Ok(Raw::String(value))
    }

    fn visit_unit<E: de::Error>(self) -> Result<Raw, E> {
        Ok(Raw::Null)
    }

    fn visit_none<E: de::Error>(self) -> Result<Raw, E> {
        Ok(Raw::Null)
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Raw, A::Error> {
        let mut items = Vec::new();
        while let Some(item) = seq.next_element()? {
            items.push(item);
        }
        Ok(Raw::Array(items))
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Raw, A::Error> {
        let mut pairs = Vec::new();
        while let Some(pair) = map.next_entry()? {
            pairs.push(pair);
        }
        Ok(Raw::Object(pairs))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn at_root(key: &str) -> Error {
        Error::DuplicateKey {
            path: String::new(),
            key: key.into(),
        }
    }

    fn integer_beyond(path: &str, integer: &str) -> Error {
        Error::IntegerBeyondRange {
            path: path.into(),
            integer: integer.into(),
        }
    }

    #[test]
    fn json5_syntax_parses_to_the_same_tree_as_json() {
        let tree =
            parse("{ a: 1, b: [true, null, 'x'], c: { d: 2.5 }, /* note */ e: 0x10 }").unwrap();
        assert_eq!(
            tree,
            json!({"a": 1, "b": [true, null, "x"], "c": {"d": 2.5}, "e": 16})
        );
    }

    #[test]
    fn a_key_twice_in_one_object_is_refused_by_name_at_its_path() {
        assert_eq!(parse("{ a: 1, a: 2 }"), Err(at_root("a")));
        assert_eq!(
            at_root("a").to_string(),
            "the key `a` appears twice in one object"
        );
        let nested = parse("{ objects: { a: { controls: { h: 1, h: 2 } } } }").unwrap_err();
        assert_eq!(
            nested,
            Error::DuplicateKey {
                path: "objects.a.controls".into(),
                key: "h".into()
            }
        );
        assert_eq!(
            nested.to_string(),
            "objects.a.controls: the key `h` appears twice in one object"
        );
        assert_eq!(
            parse("[0, [{ c: 1, c: 2 }]]").unwrap_err().to_string(),
            "[1][0]: the key `c` appears twice in one object"
        );
    }

    #[test]
    fn nan_and_infinities_are_refused() {
        assert_eq!(
            parse("{ a: NaN }").unwrap_err().to_string(),
            "a: the number NaN is not finite; a document holds finite numbers only"
        );
        assert_eq!(
            parse("Infinity"),
            Err(Error::NotFinite {
                path: String::new(),
                number: f64::INFINITY
            })
        );
        assert_eq!(
            parse("[-Infinity]"),
            Err(Error::NotFinite {
                path: "[0]".into(),
                number: f64::NEG_INFINITY
            })
        );
        assert_eq!(
            parse("[1e400]").unwrap_err().to_string(),
            "[0]: the number is an infinity or beyond the range of a double; a document holds finite numbers only"
        );
    }

    #[test]
    fn integers_within_2_53_pass_and_the_next_ones_are_refused() {
        assert_eq!(
            parse("[9007199254740992, -9007199254740992]").unwrap(),
            json!([MAX_EXACT_INTEGER, -MAX_EXACT_INTEGER])
        );
        assert_eq!(
            parse("[9007199254740993]"),
            Err(integer_beyond("[0]", "9007199254740993"))
        );
        assert_eq!(
            parse("{ n: -9007199254740993 }"),
            Err(integer_beyond("n", "-9007199254740993"))
        );
        assert_eq!(
            parse("[18446744073709551615]").unwrap_err().to_string(),
            "[0]: the integer 18446744073709551615 is beyond 2^53, the largest magnitude of a number in a document"
        );
    }

    #[test]
    fn an_integer_at_or_beyond_2_64_is_refused_by_the_same_rule() {
        assert_eq!(
            parse("[18446744073709551616]"),
            Err(integer_beyond("[0]", "18446744073709551616"))
        );
        assert_eq!(
            parse("[340282366920938463463374607431768211455]"),
            Err(integer_beyond(
                "[0]",
                "340282366920938463463374607431768211455"
            ))
        );
        assert_eq!(
            parse("[-170141183460469231731687303715884105728]"),
            Err(integer_beyond(
                "[0]",
                "-170141183460469231731687303715884105728"
            ))
        );
    }

    #[test]
    fn a_syntax_error_names_the_text_as_not_json5() {
        assert!(
            parse("{ a: }")
                .unwrap_err()
                .to_string()
                .starts_with("the text is not JSON5: ")
        );
    }

    #[test]
    fn a_float_that_names_an_integer_is_that_integer() {
        assert_eq!(
            parse("[3.0, -0.0, 1e2, 2.5, 9007199254740992.0]").unwrap(),
            json!([3, 0, 100, 2.5, MAX_EXACT_INTEGER])
        );
        assert!(parse("[3.0]").unwrap()[0].is_i64());
    }

    /// A float spelling reads as the f64 it names: -9007199254740993.0 names
    /// the f64 -2^53.
    #[test]
    fn a_float_spelling_reads_as_the_f64_it_names() {
        assert_eq!(
            parse("[-9007199254740993.0]").unwrap(),
            json!([-MAX_EXACT_INTEGER])
        );
    }

    #[test]
    fn an_integer_beyond_2_53_is_named_in_decimal_and_beyond_128_bits_is_syntax() {
        assert_eq!(
            parse("[0x20000000000001]"),
            Err(integer_beyond("[0]", "9007199254740993"))
        );
        for beyond_json5 in [
            "[340282366920938463463374607431768211456]",
            "[-170141183460469231731687303715884105729]",
        ] {
            assert!(
                parse(beyond_json5)
                    .unwrap_err()
                    .to_string()
                    .starts_with("the text is not JSON5: "),
                "{beyond_json5}"
            );
        }
    }

    fn nested(depth: usize, inside: &str) -> String {
        format!("{}{inside}{}", "[".repeat(depth), "]".repeat(depth))
    }

    #[test]
    fn a_text_nests_at_most_128_levels() {
        assert!(parse(&nested(MAX_DEPTH, "")).is_ok());
        assert!(parse(&format!("{{ a: {} }}", nested(MAX_DEPTH - 1, "{}"))).is_err());
        assert!(parse(&format!("{{ a: {} }}", nested(MAX_DEPTH - 2, "{}"))).is_ok());
        let refused = parse(&nested(MAX_DEPTH + 1, "")).unwrap_err();
        assert_eq!(
            refused,
            Error::TooDeep {
                line: 1,
                column: 129
            }
        );
        assert_eq!(
            refused.to_string(),
            "the text nests objects and arrays more than 128 levels deep, at line 1 column 129"
        );
        for line_end in ["\n", "\r", "\r\n", "\u{2028}", "\u{2029}"] {
            assert_eq!(
                parse(&format!("[{line_end}").repeat(MAX_DEPTH + 1)),
                Err(Error::TooDeep {
                    line: 129,
                    column: 1
                }),
                "{line_end:?}"
            );
        }
    }

    #[test]
    fn brackets_in_strings_and_comments_nest_nothing() {
        let noise = "\"[{\\\"[\", '[\\'{', // [[{\n /* {[{ */ /* * [ */";
        assert!(parse(&nested(MAX_DEPTH, noise)).is_ok(), "{noise}");
    }

    /// The depth counts again once a string or a comment closes: a deep array
    /// after either is refused as one at the start of the text is. A line
    /// comment ends at each line end JSON5 takes, and an escape takes the
    /// character after its backslash: an escaped quote leaves its string open,
    /// and the quote after an escaped backslash closes it.
    #[test]
    fn the_depth_counts_again_after_a_string_or_a_comment() {
        let line_ends = ["// c\n", "// c\r", "// c\u{2028}", "// c\u{2029}"];
        let escapes = ["\"\\\\\", ", "'\\\\', ", "\"\\\"\", "];
        let others = ["\"x\", ", "'x', ", "/* c */"];
        for before in line_ends.into_iter().chain(escapes).chain(others) {
            let text = format!("[{before}{}]", nested(MAX_DEPTH, ""));
            assert!(
                matches!(parse(&text), Err(Error::TooDeep { .. })),
                "{before:?}"
            );
        }
    }

    /// The depth is bounded before JSON5 descends into the text, so a deep text
    /// is refused on a thread with the 2 MiB stack of a tokio worker.
    #[test]
    fn a_text_10000_levels_deep_is_refused_on_a_2_mib_stack() {
        let deep = nested(10_000, "");
        let refused = std::thread::Builder::new()
            .stack_size(2 << 20)
            .spawn(move || parse(&deep))
            .unwrap()
            .join()
            .unwrap();
        assert_eq!(
            refused,
            Err(Error::TooDeep {
                line: 1,
                column: 129
            })
        );
    }

    #[test]
    fn a_float_beyond_2_53_is_refused_like_an_integer() {
        assert_eq!(
            parse("[1e20]"),
            Err(Error::FloatBeyondRange {
                path: "[0]".into(),
                number: 1e20
            })
        );
        assert_eq!(
            parse("{ objects: { a: { mass: -9007199254740994.0 } } }")
                .unwrap_err()
                .to_string(),
            "objects.a.mass: the number -9.007199254740994e15 is beyond 2^53, the largest magnitude of a number in a document"
        );
    }
}
