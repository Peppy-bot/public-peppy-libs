use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;

use crate::Error;

/// A name of an object, a scene object or a location: lowercase letters,
/// digits and `_`, at least one character.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(transparent)]
pub struct Name(String);

impl Name {
    pub fn parse(text: &str) -> Result<Name, Error> {
        let well_formed = !text.is_empty()
            && text
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_');
        if !well_formed {
            return Err(Error::Name(text.to_owned()));
        }
        Ok(Name(text.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for Name {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for Name {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Name, D::Error> {
        let text = String::deserialize(deserializer)?;
        Name::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// The longest name peppy gives a copy or a core node.
pub(crate) const MAX_PEPPY_NAME_LEN: usize = 63;

/// The first of `items` that an earlier item names too, as `same` compares
/// them: the first item named twice.
pub(crate) fn first_named_twice<T>(items: &[T], same: impl Fn(&T, &T) -> bool) -> Option<&T> {
    items
        .iter()
        .enumerate()
        .find(|(index, item)| items[..*index].iter().any(|earlier| same(earlier, item)))
        .map(|(_, item)| item)
}

/// The name peppy reserves for copies and core nodes: it names the daemon a
/// launch or join targets.
pub(crate) const RESERVED_PEPPY_NAME: &str = "self";

/// Why [`RESERVED_PEPPY_NAME`] names no robot and no core node.
pub(crate) const RESERVED_REASON: &str =
    "is reserved: it names the daemon a launch or join targets";

/// The grammar of a peppy name, as a refusal says it.
pub(crate) fn peppy_name_grammar() -> String {
    format!("1 to {MAX_PEPPY_NAME_LEN} letters, digits, `_` and `-`")
}

/// Whether `text` holds the grammar peppy gives the name of a copy or a core
/// node, `self` aside: 1 to 63 letters, digits, `_` and `-`.
pub(crate) fn is_peppy_name(text: &str) -> bool {
    (1..=MAX_PEPPY_NAME_LEN).contains(&text.len())
        && text
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// The name of a robot on the stack, as `peppy stack join` names a copy: 1 to
/// 63 letters, digits, `_` and `-`, and not `self`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(transparent)]
pub struct RobotName(String);

impl RobotName {
    pub fn parse(text: &str) -> Result<RobotName, Error> {
        if text == RESERVED_PEPPY_NAME {
            return Err(Error::ReservedRobotName);
        }
        if !is_peppy_name(text) {
            return Err(Error::RobotName(text.to_owned()));
        }
        Ok(RobotName(text.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// Whether this name followed by `_` begins `other`.
    pub(crate) fn prefixes(&self, other: &RobotName) -> bool {
        other.0.len() > self.0.len()
            && other.0.starts_with(&self.0)
            && other.0.as_bytes()[self.0.len()] == b'_'
    }
}

impl fmt::Display for RobotName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for RobotName {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<RobotName, D::Error> {
        let text = String::deserialize(deserializer)?;
        RobotName::parse(&text).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_name_is_lowercase_letters_digits_and_underscores() {
        assert_eq!(
            Name::parse("mantel_clock_01").unwrap().as_str(),
            "mantel_clock_01"
        );
        for bad in ["", "Crate", "box-1", "box 1", "crate/1"] {
            assert_eq!(Name::parse(bad), Err(Error::Name(bad.to_owned())), "{bad}");
        }
    }

    #[test]
    fn a_robot_name_is_1_to_63_letters_digits_underscores_and_dashes() {
        for good in ["bravo_2", "Arm-7", "a", &"n".repeat(63)] {
            assert_eq!(RobotName::parse(good).unwrap().as_str(), good, "{good}");
            assert_eq!(
                serde_json::from_str::<RobotName>(&format!("{good:?}")).unwrap(),
                RobotName::parse(good).unwrap()
            );
        }
        for bad in ["", " ", "\t", "bravo 2", "arm/1", "a@b", &"n".repeat(64)] {
            assert_eq!(
                RobotName::parse(bad),
                Err(Error::RobotName(bad.to_owned())),
                "{bad:?}"
            );
            assert!(serde_json::from_str::<RobotName>(&format!("{bad:?}")).is_err());
        }
        assert_eq!(
            RobotName::parse("").unwrap_err().to_string(),
            "the robot name `` is not 1 to 63 letters, digits, `_` and `-`"
        );
    }

    #[test]
    fn self_is_reserved_and_other_spellings_of_it_are_names() {
        assert_eq!(RobotName::parse("self"), Err(Error::ReservedRobotName));
        assert_eq!(
            Error::ReservedRobotName.to_string(),
            "the robot name `self` is reserved: it names the daemon a launch or join targets; give the robot another name"
        );
        assert!(serde_json::from_str::<RobotName>(r#""self""#).is_err());
        for name in ["Self", "self_2", "myself"] {
            assert_eq!(RobotName::parse(name).unwrap().as_str(), name);
        }
    }

    #[test]
    fn a_robot_name_prefixes_another_only_across_an_underscore() {
        let robot = |text| RobotName::parse(text).unwrap();
        let bravo = robot("bravo");
        assert!(bravo.prefixes(&robot("bravo_2")));
        assert!(!bravo.prefixes(&robot("bravo2")));
        assert!(!bravo.prefixes(&robot("bravo")));
        assert!(!robot("bravo_2").prefixes(&bravo));
    }
}
