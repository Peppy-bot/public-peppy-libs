use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::str::FromStr;

/// The longest identifier a workflow names.
const MAX_IDENTIFIER_LEN: usize = 64;

/// The longest name peppy gives a copy or an instance.
const MAX_PEPPY_NAME_LEN: usize = 63;

/// The words of CEL that no name of a workflow takes, since an expression
/// reads each name as an identifier.
const CEL_RESERVED_WORDS: [&str; 22] = [
    "as",
    "break",
    "const",
    "continue",
    "else",
    "false",
    "for",
    "function",
    "if",
    "import",
    "in",
    "let",
    "loop",
    "namespace",
    "null",
    "package",
    "return",
    "true",
    "var",
    "void",
    "while",
    "size",
];

/// Why a text is not a name of its kind.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[error("`{text}` is not {grammar}")]
pub struct NameError {
    text: String,
    grammar: &'static str,
}

fn is_identifier(text: &str) -> bool {
    let mut chars = text.chars();
    let starts_with_a_letter = chars.next().is_some_and(|c| c.is_ascii_lowercase());
    starts_with_a_letter
        && text.len() <= MAX_IDENTIFIER_LEN
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        && !CEL_RESERVED_WORDS.contains(&text)
}

/// Lowercase letters, digits and `_`, starting with a letter, at most 64
/// bytes: the grammar of a step id, which no expression reads.
fn is_step_id(text: &str) -> bool {
    let mut chars = text.chars();
    chars.next().is_some_and(|c| c.is_ascii_lowercase())
        && text.len() <= MAX_IDENTIFIER_LEN
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
}

fn is_peppy_name(text: &str) -> bool {
    (1..=MAX_PEPPY_NAME_LEN).contains(&text.len())
        && text
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

fn is_tag(text: &str) -> bool {
    (1..=MAX_IDENTIFIER_LEN).contains(&text.len())
        && text
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || matches!(c, '_' | '-' | '.'))
}

/// A text newtype whose every value holds its grammar: it parses once, and
/// serializes and deserializes as its text.
macro_rules! text_name {
    ($(#[$doc:meta])* $name:ident, $grammar:literal, $holds:expr) => {
        $(#[$doc])*
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn parse(text: &str) -> Result<$name, NameError> {
                if !($holds)(text) {
                    return Err(NameError { text: text.to_owned(), grammar: $grammar });
                }
                Ok($name(text.to_owned()))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl FromStr for $name {
            type Err = NameError;
            fn from_str(text: &str) -> Result<$name, NameError> {
                $name::parse(text)
            }
        }

        impl Serialize for $name {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                serializer.serialize_str(&self.0)
            }
        }

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<$name, D::Error> {
                let text = String::deserialize(deserializer)?;
                $name::parse(&text).map_err(serde::de::Error::custom)
            }
        }
    };
}

text_name!(
    /// A name that a workflow gives: an alias, a role, an observation, a
    /// table, a signal, a result, a loop element, a parameter, an assumption
    /// or a lane group. An expression reads it as an identifier.
    Name,
    "a name: 1 to 64 lowercase letters, digits and `_`, starting with a letter, and no word of CEL",
    is_identifier
);

text_name!(
    /// The id of a step, unique within the trees of its role.
    StepId,
    "a step id: 1 to 64 lowercase letters, digits and `_`, starting with a letter",
    is_step_id
);

text_name!(
    /// The name of a robot of the fleet listing: the name of its copy.
    RobotName,
    "a robot name: 1 to 63 letters, digits, `_` and `-`",
    |text: &str| is_peppy_name(text) && text != "self"
);

text_name!(
    /// The name of an instance within its robot, such as `wrist` for a
    /// camera, as the launcher fragment of the robot names it.
    InstanceName,
    "an instance name: 1 to 63 letters, digits, `_` and `-`",
    is_peppy_name
);

text_name!(
    /// The sha256 of the bytes of a pinned file, as `sha256sum` prints it.
    Sha256,
    "a sha256: 64 lowercase hex digits",
    |text: &str| documents_core::Digest::parse(text).is_some()
);

/// A contract or a node by its name and tag, written `name:tag`, such as
/// `postures:v1`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NameTag {
    pub name: Name,
    tag: String,
}

impl NameTag {
    const GRAMMAR: &'static str = "`name:tag`, such as `postures:v1`: a lowercase name, `:`, and a tag of lowercase letters, digits, `_`, `-` and `.`";

    pub fn parse(text: &str) -> Result<NameTag, NameError> {
        let refused = || NameError {
            text: text.to_owned(),
            grammar: NameTag::GRAMMAR,
        };
        let (name, tag) = text.split_once(':').ok_or_else(refused)?;
        let name = Name::parse(name).map_err(|_| refused())?;
        if !is_tag(tag) {
            return Err(refused());
        }
        Ok(NameTag {
            name,
            tag: tag.to_owned(),
        })
    }

    pub fn tag(&self) -> &str {
        &self.tag
    }
}

impl fmt::Display for NameTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.name, self.tag)
    }
}

impl FromStr for NameTag {
    type Err = NameError;
    fn from_str(text: &str) -> Result<NameTag, NameError> {
        NameTag::parse(text)
    }
}

impl Serialize for NameTag {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for NameTag {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<NameTag, D::Error> {
        let text = String::deserialize(deserializer)?;
        NameTag::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// A member or a topic of a contract that a workflow names by its alias,
/// written `alias.member`, such as `postures.move_to_ready`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MemberRef {
    pub alias: Name,
    pub member: Name,
}

impl MemberRef {
    const GRAMMAR: &'static str = "`alias.member`, such as `postures.move_to_ready`: an alias of `contracts`, `.`, and a member of its contract";

    pub fn parse(text: &str) -> Result<MemberRef, NameError> {
        let refused = || NameError {
            text: text.to_owned(),
            grammar: MemberRef::GRAMMAR,
        };
        let (alias, member) = text.split_once('.').ok_or_else(refused)?;
        Ok(MemberRef {
            alias: Name::parse(alias).map_err(|_| refused())?,
            member: Name::parse(member).map_err(|_| refused())?,
        })
    }
}

impl fmt::Display for MemberRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.alias, self.member)
    }
}

impl Serialize for MemberRef {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for MemberRef {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<MemberRef, D::Error> {
        let text = String::deserialize(deserializer)?;
        MemberRef::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// The longest bound a workflow gives a step, a poll or a topic age: one day.
const MAX_BOUND_S: f64 = 86_400.0;

/// A span of time in a workflow, given in seconds: finite, more than 0 and at
/// most a day. It is kept in nanoseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Bound {
    nanos: u64,
}

impl Bound {
    pub fn from_seconds(seconds: f64) -> Result<Bound, String> {
        if !(seconds.is_finite() && seconds > 0.0 && seconds <= MAX_BOUND_S) {
            return Err(format!(
                "{seconds} s is no bound: give a number of seconds more than 0 and at most {MAX_BOUND_S}"
            ));
        }
        let nanos = (seconds * 1e9).round() as u64;
        if nanos == 0 {
            return Err(format!(
                "{seconds} s is less than a nanosecond: give a bound of 1e-9 s or more"
            ));
        }
        Ok(Bound { nanos })
    }

    pub fn nanos(self) -> u64 {
        self.nanos
    }

    pub fn seconds(self) -> f64 {
        self.nanos as f64 / 1e9
    }
}

impl Serialize for Bound {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_f64(self.seconds())
    }
}

impl<'de> Deserialize<'de> for Bound {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Bound, D::Error> {
        let seconds = f64::deserialize(deserializer)?;
        Bound::from_seconds(seconds).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_name_is_a_lowercase_identifier_and_no_word_of_cel() {
        assert!(Name::parse("clear_table_2").is_ok());
        for refused in ["", "2a", "Clear", "a-b", "in", "size", &"a".repeat(65)] {
            assert!(Name::parse(refused).is_err(), "{refused}");
        }
    }

    #[test]
    fn a_name_tag_and_a_member_ref_parse_and_print_back() {
        let contract = NameTag::parse("postures:v1").unwrap();
        assert_eq!((contract.name.as_str(), contract.tag()), ("postures", "v1"));
        assert_eq!(contract.to_string(), "postures:v1");
        assert!(NameTag::parse("postures").is_err());
        assert!(NameTag::parse("postures:").is_err());
        let member = MemberRef::parse("postures.move_to_ready").unwrap();
        assert_eq!(member.to_string(), "postures.move_to_ready");
        assert!(MemberRef::parse("postures").is_err());
    }

    #[test]
    fn a_robot_name_takes_the_peppy_grammar_but_not_self() {
        assert!(RobotName::parse("arm-left_2").is_ok());
        assert!(RobotName::parse("self").is_err());
        assert!(RobotName::parse("a b").is_err());
    }

    #[test]
    fn a_bound_is_finite_positive_and_at_most_a_day() {
        assert_eq!(Bound::from_seconds(1.5).unwrap().nanos(), 1_500_000_000);
        for refused in [0.0, -1.0, 1e-10, f64::NAN, f64::INFINITY, MAX_BOUND_S + 1.0] {
            assert!(Bound::from_seconds(refused).is_err(), "{refused}");
        }
    }

    #[test]
    fn a_sha256_is_64_lowercase_hex_digits() {
        assert!(Sha256::parse(&"a1".repeat(32)).is_ok());
        assert!(Sha256::parse(&"A1".repeat(32)).is_err());
        assert!(Sha256::parse("abc").is_err());
    }
}
