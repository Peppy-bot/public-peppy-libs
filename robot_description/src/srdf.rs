//! An SRDF's elements as its text gives them, before they are held to a URDF. An element
//! or an attribute this reader does not read stops the read: a part of the description
//! that goes unread is one the robot runs without.

use roxmltree::{Document, Node};

use crate::error::DescriptionError;
use crate::xml::{attributes, elements, unread};
use crate::{DisabledPair, DisabledReason};

/// The elements of an SRDF.
pub(crate) struct Srdf {
    pub robot_name: String,
    pub groups: Vec<Group>,
    pub states: Vec<GroupState>,
    pub end_effectors: Vec<EndEffector>,
    pub passive_joints: Vec<String>,
    pub disabled_pairs: Vec<DisabledPair>,
}

/// A `<group>`: what it holds, each kind in the order the file gives it.
pub(crate) struct Group {
    pub name: String,
    /// Each `<chain>`, its base link then its tip link.
    pub chains: Vec<(String, String)>,
    pub joints: Vec<String>,
    pub links: Vec<String>,
    pub subgroups: Vec<String>,
}

/// A `<group_state>`: each joint it names with the value it gives, as text.
pub(crate) struct GroupState {
    pub name: String,
    pub group: String,
    pub joints: Vec<(String, String)>,
}

/// An `<end_effector>`.
pub(crate) struct EndEffector {
    pub name: String,
    pub parent_link: String,
    pub group: String,
    pub parent_group: Option<String>,
}

impl Srdf {
    pub fn parse(text: &str) -> Result<Self, DescriptionError> {
        let document =
            Document::parse(text).map_err(|error| DescriptionError::Srdf(error.to_string()))?;
        let root = document.root_element();
        if root.tag_name().name() != "robot" {
            return Err(DescriptionError::Srdf(format!(
                "its root element is <{}>, and an SRDF's is <robot>",
                root.tag_name().name()
            )));
        }
        let mut srdf = Srdf {
            robot_name: attributes(root, &["name"], &[])?.required("name")?,
            groups: Vec::new(),
            states: Vec::new(),
            end_effectors: Vec::new(),
            passive_joints: Vec::new(),
            disabled_pairs: Vec::new(),
        };
        for element in elements(root) {
            match element.tag_name().name() {
                "group" => srdf.groups.push(group(element)?),
                "group_state" => srdf.states.push(group_state(element)?),
                "end_effector" => {
                    let read = attributes(
                        element,
                        &["name", "parent_link", "group"],
                        &["parent_group"],
                    )?;
                    srdf.end_effectors.push(EndEffector {
                        name: read.required("name")?,
                        parent_link: read.required("parent_link")?,
                        group: read.required("group")?,
                        parent_group: read.optional("parent_group"),
                    });
                }
                "passive_joint" => {
                    let read = attributes(element, &["name"], &[])?;
                    srdf.passive_joints.push(read.required("name")?);
                }
                "disable_collisions" => {
                    let read = attributes(element, &["link1", "link2", "reason"], &[])?;
                    let reason = read.required("reason")?;
                    srdf.disabled_pairs.push(DisabledPair {
                        link1: read.required("link1")?,
                        link2: read.required("link2")?,
                        reason: DisabledReason::parse(&reason)
                            .ok_or(DescriptionError::Reason { reason })?,
                    });
                }
                other => return Err(unread(other, "robot", &ROBOT_HOLDS)),
            }
        }
        Ok(srdf)
    }
}

fn group(element: Node) -> Result<Group, DescriptionError> {
    let mut group = Group {
        name: attributes(element, &["name"], &[])?.required("name")?,
        chains: Vec::new(),
        joints: Vec::new(),
        links: Vec::new(),
        subgroups: Vec::new(),
    };
    for member in elements(element) {
        match member.tag_name().name() {
            "chain" => {
                let read = attributes(member, &["base_link", "tip_link"], &[])?;
                group
                    .chains
                    .push((read.required("base_link")?, read.required("tip_link")?));
            }
            "joint" => group
                .joints
                .push(attributes(member, &["name"], &[])?.required("name")?),
            "link" => group
                .links
                .push(attributes(member, &["name"], &[])?.required("name")?),
            "group" => group
                .subgroups
                .push(attributes(member, &["name"], &[])?.required("name")?),
            other => return Err(unread(other, "group", &GROUP_HOLDS)),
        }
    }
    Ok(group)
}

fn group_state(element: Node) -> Result<GroupState, DescriptionError> {
    let read = attributes(element, &["name", "group"], &[])?;
    let mut state = GroupState {
        name: read.required("name")?,
        group: read.required("group")?,
        joints: Vec::new(),
    };
    for member in elements(element) {
        match member.tag_name().name() {
            "joint" => {
                let read = attributes(member, &["name", "value"], &[])?;
                state
                    .joints
                    .push((read.required("name")?, read.required("value")?));
            }
            other => return Err(unread(other, "group_state", &["joint"])),
        }
    }
    Ok(state)
}

/// The elements this reader reads in `<robot>`.
const ROBOT_HOLDS: [&str; 5] = [
    "group",
    "group_state",
    "end_effector",
    "passive_joint",
    "disable_collisions",
];

/// The elements this reader reads in `<group>`.
const GROUP_HOLDS: [&str; 4] = ["chain", "joint", "link", "group"];
