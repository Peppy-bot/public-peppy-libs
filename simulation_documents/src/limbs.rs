//! The limbs of a robot of a world: where each arm and each gripper stands,
//! by the limb names of the robot (limb_state).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::Error;

/// How far a joint may stand from the same joint of another world for the
/// two limbs to stand alike: radians for a revolute joint, metres for a
/// prismatic one. A measured joint moves by a hair from one read to the next.
pub const JOINT_SLACK: f64 = 0.01;

/// How far a gripper opening may stand from the same opening of another
/// world for the two grippers to stand alike, as a fraction of full jaw
/// travel.
pub const OPENING_SLACK: f64 = 0.02;

/// The limbs of a robot as they stand: the joint positions of each arm, in
/// the joint order of the arm, and the opening of each gripper, each by its
/// limb name. At least one arm or one gripper.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "LimbsFields", into = "LimbsFields")]
pub struct Limbs {
    pub arms: BTreeMap<String, Vec<f64>>,
    pub grippers: BTreeMap<String, Opening>,
}

#[derive(Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "the limbs of a robot, { arms, grippers }"
)]
struct LimbsFields {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    arms: BTreeMap<String, Vec<f64>>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    grippers: BTreeMap<String, Opening>,
}

impl TryFrom<LimbsFields> for Limbs {
    type Error = Error;

    fn try_from(fields: LimbsFields) -> Result<Limbs, Error> {
        Limbs::new(fields.arms, fields.grippers)
    }
}

impl From<Limbs> for LimbsFields {
    fn from(limbs: Limbs) -> LimbsFields {
        LimbsFields {
            arms: limbs.arms,
            grippers: limbs.grippers,
        }
    }
}

impl Limbs {
    /// The limbs of `arms` and `grippers`. Limbs that name no limb, a blank
    /// limb name, and an arm without joint positions are refused.
    pub fn new(
        arms: BTreeMap<String, Vec<f64>>,
        grippers: BTreeMap<String, Opening>,
    ) -> Result<Limbs, Error> {
        if arms.is_empty() && grippers.is_empty() {
            return Err(Error::rule(
                "the limbs name no arm and no gripper; name each arm with its joint positions and each gripper with its opening, or leave limbs out",
            ));
        }
        let names = arms.keys().chain(grippers.keys());
        if let Some(blank) = names.into_iter().find(|name| name.trim().is_empty()) {
            return Err(Error::rule(format!(
                "the limb name `{blank}` is blank; name each limb as the robot names it"
            )));
        }
        if let Some((arm, _)) = arms.iter().find(|(_, joints)| joints.is_empty()) {
            return Err(Error::rule(format!(
                "the arm `{arm}` has no joint position; give one position per joint of the arm, in its joint order"
            )));
        }
        Ok(Limbs { arms, grippers })
    }

    /// Whether `other` names the same arms, each with as many joints, and the
    /// same grippers, with each joint within [`JOINT_SLACK`] and each opening
    /// within [`OPENING_SLACK`] of the other's.
    pub fn alike(&self, other: &Limbs) -> bool {
        let arms_alike = self.arms.len() == other.arms.len()
            && self.arms.iter().zip(&other.arms).all(
                |((name, joints), (other_name, other_joints))| {
                    name == other_name
                        && joints.len() == other_joints.len()
                        && joints
                            .iter()
                            .zip(other_joints)
                            .all(|(joint, other_joint)| (joint - other_joint).abs() <= JOINT_SLACK)
                },
            );
        let grippers_alike = self.grippers.len() == other.grippers.len()
            && self.grippers.iter().zip(&other.grippers).all(
                |((name, opening), (other_name, other_opening))| {
                    name == other_name
                        && (opening.value() - other_opening.value()).abs() <= OPENING_SLACK
                },
            );
        arms_alike && grippers_alike
    }
}

/// The opening of a gripper: a fraction of full jaw travel, from 0 (closed)
/// to 1 (fully open).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "f64", into = "f64")]
pub struct Opening(f64);

impl Opening {
    /// A closed gripper.
    pub const CLOSED: Opening = Opening(0.0);

    /// `value` as an opening; a value outside 0 to 1, or not a number, is
    /// refused.
    pub fn new(value: f64) -> Result<Opening, Error> {
        if !(0.0..=1.0).contains(&value) {
            return Err(Error::rule(format!(
                "the opening {value} is not from 0 to 1; write a fraction of full jaw travel, 0 closed and 1 fully open"
            )));
        }
        Ok(Opening(value))
    }

    pub fn value(self) -> f64 {
        self.0
    }
}

impl TryFrom<f64> for Opening {
    type Error = Error;

    fn try_from(value: f64) -> Result<Opening, Error> {
        Opening::new(value)
    }
}

impl From<Opening> for f64 {
    fn from(opening: Opening) -> f64 {
        opening.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::text;

    fn limbs(text: &str) -> Limbs {
        text::from_json5(text).unwrap()
    }

    fn refusal(text: &str) -> String {
        text::from_json5::<Limbs>(text).unwrap_err().to_string()
    }

    #[test]
    fn limbs_read_the_joints_of_each_arm_and_the_opening_of_each_gripper() {
        let read = limbs("{ arms: { left_arm: [0.5, -1.25] }, grippers: { left_gripper: 0.25 } }");
        assert_eq!(read.arms["left_arm"], [0.5, -1.25]);
        assert_eq!(read.grippers["left_gripper"].value(), 0.25);
        assert_eq!(limbs(&text::to_json5(&read).unwrap()), read);
    }

    #[test]
    fn limbs_with_arms_alone_or_grippers_alone_write_no_empty_field() {
        let arms = limbs("{ arms: { arm: [0] } }");
        assert!(arms.grippers.is_empty());
        assert!(!text::to_json5(&arms).unwrap().contains("grippers"));
        let grippers = limbs("{ grippers: { gripper: 1 } }");
        assert!(grippers.arms.is_empty());
        assert!(!text::to_json5(&grippers).unwrap().contains("arms"));
    }

    #[test]
    fn limbs_that_name_nothing_a_blank_limb_or_an_arm_without_joints_are_refused() {
        let cases = [
            ("{}", "the limbs name no arm and no gripper"),
            (
                "{ arms: {}, grippers: {} }",
                "the limbs name no arm and no gripper",
            ),
            ("{ arms: { ' ': [0] } }", "the limb name ` ` is blank"),
            ("{ grippers: { '': 0 } }", "the limb name `` is blank"),
            (
                "{ arms: { left_arm: [] } }",
                "the arm `left_arm` has no joint position",
            ),
        ];
        for (text, expected) in cases {
            let refused = refusal(text);
            assert!(refused.contains(expected), "{text}: {refused}");
        }
    }

    #[test]
    fn new_limbs_take_the_rules_of_written_limbs() {
        let arms = BTreeMap::from([("arm".to_owned(), vec![0.5])]);
        let made = Limbs::new(arms, BTreeMap::new()).unwrap();
        assert_eq!(made, limbs("{ arms: { arm: [0.5] } }"));
        let empty_arm = BTreeMap::from([("arm".to_owned(), Vec::new())]);
        assert_eq!(
            Limbs::new(empty_arm, BTreeMap::new())
                .unwrap_err()
                .to_string(),
            refusal("{ arms: { arm: [] } }")
        );
    }

    #[test]
    fn an_opening_is_from_0_to_1() {
        for value in [0.0, 0.5, 1.0] {
            assert_eq!(Opening::new(value).unwrap().value(), value);
        }
        for value in [-0.001, 1.001, f64::NAN, f64::INFINITY] {
            let refused = Opening::new(value).unwrap_err().to_string();
            assert!(refused.contains("is not from 0 to 1"), "{value}: {refused}");
        }
        assert_eq!(Opening::CLOSED, Opening::new(0.0).unwrap());
        let refused = refusal("{ grippers: { gripper: 1.5 } }");
        assert!(
            refused.contains("the opening 1.5 is not from 0 to 1"),
            "{refused}"
        );
    }

    #[test]
    fn limbs_stand_alike_within_the_slack_of_each_joint_and_each_opening() {
        let base = limbs("{ arms: { a: [0.5, 1] }, grippers: { g: 0.5 } }");
        let within = limbs(&format!(
            "{{ arms: {{ a: [{}, {}] }}, grippers: {{ g: {} }} }}",
            0.5 + JOINT_SLACK * 0.9,
            1.0 - JOINT_SLACK * 0.9,
            0.5 - OPENING_SLACK * 0.9
        ));
        assert!(base.alike(&within) && within.alike(&base));
        let unlike = [
            format!(
                "{{ arms: {{ a: [{}, 1] }}, grippers: {{ g: 0.5 }} }}",
                0.5 + JOINT_SLACK * 1.5
            ),
            format!(
                "{{ arms: {{ a: [0.5, 1] }}, grippers: {{ g: {} }} }}",
                0.5 + OPENING_SLACK * 1.5
            ),
            "{ arms: { a: [0.5, 1, 0] }, grippers: { g: 0.5 } }".to_owned(),
            "{ arms: { b: [0.5, 1] }, grippers: { g: 0.5 } }".to_owned(),
            "{ arms: { a: [0.5, 1] }, grippers: { h: 0.5 } }".to_owned(),
            "{ arms: { a: [0.5, 1] } }".to_owned(),
            "{ arms: { a: [0.5, 1], b: [0] }, grippers: { g: 0.5 } }".to_owned(),
        ];
        for text in unlike {
            let other = limbs(&text);
            assert!(!base.alike(&other) && !other.alike(&base), "{text}");
        }
    }
}
