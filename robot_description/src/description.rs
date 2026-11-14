//! A URDF and an SRDF held to each other: every name the SRDF gives is one the URDF
//! has, and what a consumer reads, the arms, their grippers, the postures, the disabled
//! pairs and the cameras, is worked out here once.

use std::collections::{HashMap, HashSet};

use urdf_rs::{Joint, JointType, Robot};

use crate::error::DescriptionError;
use crate::gazebo::{self, DepthSensor};
use crate::srdf::{self, Srdf};
use crate::states;

/// Why the collisions of a pair of links are disabled, as MoveIt's setup assistant
/// names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisabledReason {
    /// One joint joins the two links.
    Adjacent,
    /// The robot's joints never bring the two links together.
    Never,
    /// The two links touch where the robot stands by default.
    Default,
    /// The two links touch wherever the robot stands.
    Always,
    /// Its author's own reason.
    User,
}

impl DisabledReason {
    /// Every reason, in the order MoveIt's setup assistant lists them.
    pub(crate) const ALL: [Self; 5] = [
        Self::Adjacent,
        Self::Never,
        Self::Default,
        Self::Always,
        Self::User,
    ];

    /// The reason's name, as an SRDF writes it.
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Adjacent => "Adjacent",
            Self::Never => "Never",
            Self::Default => "Default",
            Self::Always => "Always",
            Self::User => "User",
        }
    }

    pub(crate) fn parse(reason: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|known| known.name() == reason)
    }
}

/// A pair of links whose collisions are disabled, in the order the SRDF names them.
#[derive(Debug, Clone, PartialEq)]
pub struct DisabledPair {
    pub link1: String,
    pub link2: String,
    pub reason: DisabledReason,
}

/// An arm: a group of one chain whose tip an end effector stands on.
#[derive(Debug, Clone, PartialEq)]
pub struct Arm {
    /// The group's name.
    pub name: String,
    /// The first link of the chain.
    pub base_link: String,
    /// The tip of the chain: the grasp point its end effector stands on.
    pub grasp_link: String,
    /// The joints of the chain that move of their own, base to tip.
    pub joints: Vec<String>,
    /// Each joint's [`position_limits`], `[lower, upper]` in the joint's unit, in the order
    /// of `joints`.
    pub limits: Vec<[f64; 2]>,
    /// The end effector's group.
    pub gripper: Gripper,
}

/// An arm's gripper: the group of its end effector.
#[derive(Debug, Clone, PartialEq)]
pub struct Gripper {
    /// The group's name.
    pub name: String,
    /// Each joint of the group that moves, in the group's order.
    pub fingers: Vec<Finger>,
}

/// One moving joint of a gripper and the link it carries.
#[derive(Debug, Clone, PartialEq)]
pub struct Finger {
    pub joint: String,
    pub link: String,
    /// The joint's position where the gripper's `closed` state puts it.
    pub closed: f64,
    /// The joint's position where the gripper's `open` state puts it.
    pub open: f64,
    /// The finger joint this one follows, as its URDF `<mimic>` gives it; none for a joint
    /// that moves of its own.
    pub mimic: Option<Mimic>,
}

/// How a finger joint follows another finger joint of its gripper, one that moves of its
/// own: it stands at `multiplier` times that joint's position plus `offset`, the URDF
/// `<mimic>`'s, 1 and 0 where it gives none.
#[derive(Debug, Clone, PartialEq)]
pub struct Mimic {
    /// The joint followed.
    pub joint: String,
    pub multiplier: f64,
    /// In the joints' unit, radians or meters.
    pub offset: f64,
}

/// The optical frame of a camera the robot carries: a link named
/// `<name>_optical_frame`, fixed to `parent_link`, with `+x` to the right of the image,
/// `+y` down it and `+z` along the view.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraFrame {
    /// The camera's name on the robot.
    pub name: String,
    /// The link the frame is fixed to.
    pub parent_link: String,
    /// The frame's origin in the parent link's frame, meters.
    pub xyz: [f64; 3],
    /// The frame's roll, pitch and yaw in the parent link's frame, radians, as a URDF
    /// origin gives them.
    pub rpy: [f64; 3],
    /// The depths the camera measures, as the depth sensor the URDF's Gazebo extension
    /// gives on its optical frame; none for a camera the URDF gives no depth sensor.
    pub depth_range: Option<DepthRange>,
}

/// The depths a camera measures, meters along its view (`+z` of its optical frame): the
/// `<near>` and `<far>` of its depth sensor's `<clip>`, finite, the nearest above 0 and
/// below the farthest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthRange {
    near: f64,
    far: f64,
}

impl DepthRange {
    /// The range from `near` to `far`, or none unless `0 < near < far`, both finite.
    pub(crate) fn new(near: f64, far: f64) -> Option<Self> {
        (near > 0.0 && near < far && far.is_finite()).then_some(Self { near, far })
    }

    /// The nearest depth the camera measures, meters.
    pub fn near(self) -> f64 {
        self.near
    }

    /// The farthest depth the camera measures, meters.
    pub fn far(self) -> f64 {
        self.far
    }
}

/// The suffix of the link that is a camera's optical frame.
const OPTICAL_FRAME_SUFFIX: &str = "_optical_frame";

/// A robot's description: its URDF and its SRDF, each name of the SRDF found in the URDF.
#[derive(Debug, Clone)]
pub struct RobotDescription {
    urdf: String,
    srdf: String,
    robot: Robot,
    arms: Vec<Arm>,
    /// Each group that holds other groups and nothing else, with them, in the SRDF's order.
    group_sets: Vec<(String, Vec<String>)>,
    states: Vec<State>,
    disabled_pairs: Vec<DisabledPair>,
    cameras: Vec<CameraFrame>,
}

/// A group's state: a value for each joint of the group that moves of its own, in the
/// group's order.
#[derive(Debug, Clone)]
struct State {
    group: String,
    name: String,
    values: Vec<f64>,
}

impl RobotDescription {
    /// The description `urdf` and `srdf` make. A URDF whose links and joints are not one
    /// tree of joints that each take one position (chain_kinematics' [`TreeError`]), a
    /// `<gazebo>` element of the URDF that is not one camera's depth sensor, and an SRDF
    /// that names a link, a joint or a group that does not exist, that this reader reads
    /// in part, or that describes an arm, a gripper, a state or a disabled pair the URDF
    /// contradicts, are refused.
    ///
    /// [`TreeError`]: chain_kinematics::TreeError
    pub fn parse(urdf: &str, srdf: &str) -> Result<Self, DescriptionError> {
        let robot = urdf_rs::read_from_string(urdf)
            .map_err(|error| DescriptionError::Urdf(error.to_string()))?;
        let depth_sensors = gazebo::depth_sensors(urdf)?;
        let semantics = Srdf::parse(srdf)?;
        if semantics.robot_name != robot.name {
            return Err(DescriptionError::RobotName {
                srdf: semantics.robot_name,
                urdf: robot.name,
            });
        }
        let tree = Tree::of(&robot)?;
        mesh_names(&robot)?;
        let groups = Groups::resolve(&semantics.groups, &tree)?;
        let states = states(&semantics.states, &groups, &tree)?;
        let arms = arms(&semantics, &groups, &states, &tree)?;
        for joint in &semantics.passive_joints {
            tree.joint("passive_joint", joint)?;
        }
        let group_sets = semantics
            .groups
            .iter()
            .filter(|group| {
                !group.subgroups.is_empty()
                    && group.chains.is_empty()
                    && group.joints.is_empty()
                    && group.links.is_empty()
            })
            .map(|group| (group.name.clone(), group.subgroups.clone()))
            .collect();
        let disabled_pairs = disabled_pairs(semantics.disabled_pairs, &tree)?;
        let cameras = cameras(&robot, &tree, &depth_sensors)?;
        Ok(Self {
            urdf: urdf.to_string(),
            srdf: srdf.to_string(),
            robot,
            arms,
            group_sets,
            states,
            disabled_pairs,
            cameras,
        })
    }

    /// The URDF's text.
    pub fn urdf(&self) -> &str {
        &self.urdf
    }

    /// The SRDF's text.
    pub fn srdf(&self) -> &str {
        &self.srdf
    }

    /// The URDF, parsed.
    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// The robot's arms, in the order the SRDF gives their groups.
    pub fn arms(&self) -> &[Arm] {
        &self.arms
    }

    /// The arms of the group `name`, in the order a state of the group holds their joints:
    /// the group itself when it is an arm, else the groups it holds, each an arm, in the
    /// SRDF's order of them. None for a group that is neither, or for no group.
    pub fn group_arms(&self, name: &str) -> Option<Vec<&Arm>> {
        if let Some(arm) = self.arms.iter().find(|arm| arm.name == name) {
            return Some(vec![arm]);
        }
        let (_, subgroups) = self.group_sets.iter().find(|(group, _)| group == name)?;
        subgroups
            .iter()
            .map(|subgroup| self.arms.iter().find(|arm| arm.name == *subgroup))
            .collect()
    }

    /// The state `name` of `group`: a position for each joint of the group that moves of
    /// its own, in the group's order, which for an arm is the order of [`Arm::joints`].
    pub fn state(&self, group: &str, name: &str) -> Option<&[f64]> {
        self.states
            .iter()
            .find(|state| state.group == group && state.name == name)
            .map(|state| state.values.as_slice())
    }

    /// The pairs of links whose collisions are disabled, each of two links with collision
    /// geometry, each pair once.
    pub fn disabled_pairs(&self) -> &[DisabledPair] {
        &self.disabled_pairs
    }

    /// The optical frames of the cameras the robot carries, in the URDF's order, each with
    /// the depths it measures where the URDF gives it a depth sensor.
    pub fn cameras(&self) -> &[CameraFrame] {
        &self.cameras
    }
}

/// The URDF's links and joints by name, and the joint each link hangs on.
struct Tree<'a> {
    links: HashSet<&'a str>,
    with_geometry: HashSet<&'a str>,
    joints: HashMap<&'a str, &'a Joint>,
    parent_joint: HashMap<&'a str, &'a Joint>,
}

impl<'a> Tree<'a> {
    /// The tree of `robot`'s links, refused when chain_kinematics refuses it
    /// ([`chain_kinematics::Tree::from_robot`]): each link and each joint named once,
    /// each joint naming two of the URDF's links and taking one position, and every link
    /// but one root hanging on one joint, with no loop.
    fn of(robot: &'a Robot) -> Result<Self, DescriptionError> {
        chain_kinematics::Tree::from_robot(robot)?;
        Ok(Self {
            links: robot.links.iter().map(|link| link.name.as_str()).collect(),
            with_geometry: robot
                .links
                .iter()
                .filter(|link| !link.collision.is_empty())
                .map(|link| link.name.as_str())
                .collect(),
            joints: robot
                .joints
                .iter()
                .map(|joint| (joint.name.as_str(), joint))
                .collect(),
            parent_joint: robot
                .joints
                .iter()
                .map(|joint| (joint.child.link.as_str(), joint))
                .collect(),
        })
    }

    fn link(&self, element: &'static str, link: &str) -> Result<(), DescriptionError> {
        if self.links.contains(link) {
            return Ok(());
        }
        Err(DescriptionError::NoSuchLink {
            element,
            link: link.to_string(),
        })
    }

    fn joint(&self, element: &'static str, joint: &str) -> Result<&'a Joint, DescriptionError> {
        self.joints
            .get(joint)
            .copied()
            .ok_or_else(|| DescriptionError::NoSuchJoint {
                element,
                joint: joint.to_string(),
            })
    }

    /// The joints from `base` down to `tip`, or none when `tip` is not below `base`.
    fn path(&self, base: &str, tip: &str) -> Option<Vec<&'a Joint>> {
        let mut joints = Vec::new();
        let mut link = tip;
        while link != base {
            let joint = self.parent_joint.get(link)?;
            joints.push(*joint);
            link = joint.parent.link.as_str();
        }
        joints.reverse();
        Some(joints)
    }
}

/// Refuses two collision meshes of one file name from different paths: a description
/// holds its meshes by file name ([`crate::Model::meshes`]).
fn mesh_names(robot: &Robot) -> Result<(), DescriptionError> {
    let mut paths: HashMap<&str, &str> = HashMap::new();
    let meshes = robot
        .links
        .iter()
        .flat_map(|link| &link.collision)
        .filter_map(|collision| match &collision.geometry {
            urdf_rs::Geometry::Mesh { filename, .. } => Some(filename.as_str()),
            _ => None,
        });
    for path in meshes {
        let name = mesh_name(path);
        match paths.insert(name, path) {
            Some(first) if first != path => {
                return Err(DescriptionError::MeshNameTwice {
                    name: name.to_string(),
                    first: first.to_string(),
                    second: path.to_string(),
                });
            }
            _ => {}
        }
    }
    Ok(())
}

/// The file name a description holds the collision mesh at `path` by: the last part of
/// the path.
pub fn mesh_name(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

fn moves(joint: &Joint) -> bool {
    joint.joint_type != JointType::Fixed
}

fn moves_of_its_own(joint: &Joint) -> bool {
    moves(joint) && joint.mimic.is_none()
}

/// The position limits of `joint`, `[lower, upper]` in the joint's unit: the URDF's,
/// unbounded for a continuous joint.
pub fn position_limits(joint: &Joint) -> [f64; 2] {
    match joint.joint_type {
        JointType::Continuous => [f64::NEG_INFINITY, f64::INFINITY],
        _ => [joint.limit.lower, joint.limit.upper],
    }
}

/// Each group with the joints it holds.
struct Groups<'a> {
    /// In the SRDF's order.
    resolved: Vec<(&'a srdf::Group, Vec<&'a Joint>)>,
}

impl<'a> Groups<'a> {
    fn resolve(groups: &'a [srdf::Group], tree: &Tree<'a>) -> Result<Self, DescriptionError> {
        let mut by_name: HashMap<&str, &srdf::Group> = HashMap::new();
        for group in groups {
            if by_name.insert(group.name.as_str(), group).is_some() {
                return Err(DescriptionError::DuplicateGroup(group.name.clone()));
            }
        }
        let resolved = groups
            .iter()
            .map(|group| Ok((group, joints_of(group, &by_name, tree, &mut Vec::new())?)))
            .collect::<Result<_, DescriptionError>>()?;
        Ok(Self { resolved })
    }

    fn joints(&self, element: &'static str, group: &str) -> Result<&[&'a Joint], DescriptionError> {
        self.resolved
            .iter()
            .find(|(candidate, _)| candidate.name == group)
            .map(|(_, joints)| joints.as_slice())
            .ok_or_else(|| DescriptionError::NoSuchGroup {
                element,
                group: group.to_string(),
            })
    }
}

/// The joints `group` holds: those of its chains, base to tip, then the joints it names,
/// then the joint each link it names hangs on, then those of its groups. `holding` is the
/// groups this one is being resolved for, each of which it may not hold.
fn joints_of<'a>(
    group: &'a srdf::Group,
    by_name: &HashMap<&str, &'a srdf::Group>,
    tree: &Tree<'a>,
    holding: &mut Vec<&'a str>,
) -> Result<Vec<&'a Joint>, DescriptionError> {
    if holding.contains(&group.name.as_str()) {
        return Err(DescriptionError::GroupHoldsItself {
            group: group.name.clone(),
        });
    }
    let mut joints = Vec::new();
    for (base, tip) in &group.chains {
        tree.link("chain", base)?;
        tree.link("chain", tip)?;
        joints.extend(
            tree.path(base, tip)
                .ok_or_else(|| DescriptionError::TipNotBelowBase {
                    group: group.name.clone(),
                    base: base.clone(),
                    tip: tip.clone(),
                })?,
        );
    }
    for joint in &group.joints {
        joints.push(tree.joint("group", joint)?);
    }
    for link in &group.links {
        tree.link("group", link)?;
        joints.extend(tree.parent_joint.get(link.as_str()).copied());
    }
    holding.push(&group.name);
    for subgroup in &group.subgroups {
        let held = by_name
            .get(subgroup.as_str())
            .ok_or_else(|| DescriptionError::NoSuchGroup {
                element: "group",
                group: subgroup.clone(),
            })?;
        joints.extend(joints_of(held, by_name, tree, holding)?);
    }
    holding.pop();
    Ok(joints)
}

fn states(
    states: &[srdf::GroupState],
    groups: &Groups,
    tree: &Tree,
) -> Result<Vec<State>, DescriptionError> {
    let mut read: Vec<State> = Vec::new();
    for state in states {
        if read
            .iter()
            .any(|other| other.group == state.group && other.name == state.name)
        {
            return Err(DescriptionError::DuplicateState {
                group: state.group.clone(),
                state: state.name.clone(),
            });
        }
        let own: Vec<&Joint> = groups
            .joints("group_state", &state.group)?
            .iter()
            .copied()
            .filter(|joint| moves_of_its_own(joint))
            .collect();
        let mut values: Vec<Option<f64>> = vec![None; own.len()];
        for (name, text) in &state.joints {
            tree.joint("group_state", name)?;
            let Some(index) = own.iter().position(|joint| &joint.name == name) else {
                return Err(DescriptionError::StateJointOffTheGroup {
                    group: state.group.clone(),
                    state: state.name.clone(),
                    joint: name.clone(),
                    joints: own.iter().map(|joint| joint.name.clone()).collect(),
                });
            };
            let value = text
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|value| value.is_finite())
                .ok_or_else(|| DescriptionError::StateValue {
                    state: state.name.clone(),
                    joint: name.clone(),
                    value: text.clone(),
                })?;
            let [lower, upper] = position_limits(own[index]);
            if !(lower..=upper).contains(&value) {
                return Err(DescriptionError::StateOutsideLimits {
                    group: state.group.clone(),
                    state: state.name.clone(),
                    joint: name.clone(),
                    value,
                    lower,
                    upper,
                });
            }
            if values[index].replace(value).is_some() {
                return Err(DescriptionError::StateJointTwice {
                    group: state.group.clone(),
                    state: state.name.clone(),
                    joint: name.clone(),
                });
            }
        }
        let values = own
            .iter()
            .zip(values)
            .map(|(joint, value)| {
                value.ok_or_else(|| DescriptionError::StateJointMissing {
                    group: state.group.clone(),
                    state: state.name.clone(),
                    joint: joint.name.clone(),
                })
            })
            .collect::<Result<_, _>>()?;
        read.push(State {
            group: state.group.clone(),
            name: state.name.clone(),
            values,
        });
    }
    Ok(read)
}

/// The arms: each group of one chain, and nothing else, whose tip an end effector stands
/// on, in the order of the groups.
fn arms(
    semantics: &Srdf,
    groups: &Groups,
    states: &[State],
    tree: &Tree,
) -> Result<Vec<Arm>, DescriptionError> {
    // Each end effector with the group it stands on.
    let mut standing: Vec<(&srdf::EndEffector, &srdf::Group)> = Vec::new();
    for end_effector in &semantics.end_effectors {
        if standing
            .iter()
            .any(|(other, _)| other.name == end_effector.name)
        {
            return Err(DescriptionError::DuplicateEndEffector(
                end_effector.name.clone(),
            ));
        }
        tree.link("end_effector", &end_effector.parent_link)?;
        let mut chains = semantics.groups.iter().filter(|group| {
            matches!(group.chains.as_slice(), [(_, tip)] if *tip == end_effector.parent_link)
                && group.joints.is_empty()
                && group.links.is_empty()
                && group.subgroups.is_empty()
        });
        let arm = chains
            .next()
            .ok_or_else(|| DescriptionError::EndEffectorOffAnArm {
                end_effector: end_effector.name.clone(),
                parent_link: end_effector.parent_link.clone(),
            })?;
        if let Some(second) = chains.next() {
            return Err(DescriptionError::TwoArmsOfOneTip {
                first: arm.name.clone(),
                second: second.name.clone(),
                tip: end_effector.parent_link.clone(),
            });
        }
        if let Some(parent_group) = &end_effector.parent_group
            && *parent_group != arm.name
        {
            return Err(DescriptionError::EndEffectorParentGroup {
                end_effector: end_effector.name.clone(),
                arm: arm.name.clone(),
                parent_group: parent_group.clone(),
            });
        }
        if let Some((first, _)) = standing.iter().find(|(_, other)| other.name == arm.name) {
            return Err(DescriptionError::TwoEndEffectorsOnOneArm {
                arm: arm.name.clone(),
                first: first.name.clone(),
                second: end_effector.name.clone(),
            });
        }
        standing.push((end_effector, arm));
    }

    semantics
        .groups
        .iter()
        .filter_map(|group| {
            let (end_effector, _) = standing.iter().find(|(_, arm)| arm.name == group.name)?;
            Some((group, *end_effector))
        })
        .map(|(group, end_effector)| {
            let (base_link, grasp_link) = &group.chains[0];
            let moving: Vec<&Joint> = groups
                .joints("group", &group.name)?
                .iter()
                .copied()
                .filter(|joint| moves_of_its_own(joint))
                .collect();
            Ok(Arm {
                name: group.name.clone(),
                base_link: base_link.clone(),
                grasp_link: grasp_link.clone(),
                joints: moving.iter().map(|joint| joint.name.clone()).collect(),
                limits: moving.iter().map(|joint| position_limits(joint)).collect(),
                gripper: gripper(&end_effector.group, groups, states)?,
            })
        })
        .collect()
}

/// The gripper of the group `name`: each joint of the group that moves, where the
/// group's `closed` and `open` states put it. A joint that mimics another stands where
/// its leader's position puts it.
fn gripper(name: &str, groups: &Groups, states: &[State]) -> Result<Gripper, DescriptionError> {
    let moving: Vec<&Joint> = groups
        .joints("end_effector", name)?
        .iter()
        .copied()
        .filter(|joint| moves(joint))
        .collect();
    if moving.is_empty() {
        return Ok(Gripper {
            name: name.to_string(),
            fingers: Vec::new(),
        });
    }
    let own: Vec<&Joint> = moving
        .iter()
        .copied()
        .filter(|joint| moves_of_its_own(joint))
        .collect();
    let state = |state: &'static str| {
        states
            .iter()
            .find(|candidate| candidate.group == name && candidate.name == state)
            .map(|found| found.values.as_slice())
            .ok_or_else(|| DescriptionError::GripperState {
                gripper: name.to_string(),
                state,
            })
    };
    let (closed, open) = (state(states::CLOSED)?, state(states::OPEN)?);
    let fingers = moving
        .iter()
        .map(|joint| {
            let mimic = joint.mimic.as_ref().map(|mimic| Mimic {
                joint: mimic.joint.clone(),
                multiplier: mimic.multiplier.unwrap_or(1.0),
                offset: mimic.offset.unwrap_or(0.0),
            });
            let (leader, multiplier, offset) = match &mimic {
                Some(mimic) => (mimic.joint.as_str(), mimic.multiplier, mimic.offset),
                None => (joint.name.as_str(), 1.0, 0.0),
            };
            let index = own
                .iter()
                .position(|candidate| candidate.name == leader)
                .ok_or_else(|| DescriptionError::MimicOffTheGripper {
                    gripper: name.to_string(),
                    joint: joint.name.clone(),
                    leader: leader.to_string(),
                })?;
            let (closed, open) = (
                multiplier * closed[index] + offset,
                multiplier * open[index] + offset,
            );
            Ok(Finger {
                joint: joint.name.clone(),
                link: joint.child.link.clone(),
                closed,
                open,
                mimic,
            })
        })
        .collect::<Result<_, DescriptionError>>()?;
    Ok(Gripper {
        name: name.to_string(),
        fingers,
    })
}

fn disabled_pairs(
    pairs: Vec<DisabledPair>,
    tree: &Tree,
) -> Result<Vec<DisabledPair>, DescriptionError> {
    let mut seen: HashSet<(&str, &str)> = HashSet::new();
    for pair in &pairs {
        for link in [&pair.link1, &pair.link2] {
            tree.link("disable_collisions", link)?;
            if !tree.with_geometry.contains(link.as_str()) {
                return Err(DescriptionError::PairWithoutGeometry { link: link.clone() });
            }
        }
        if pair.link1 == pair.link2 {
            return Err(DescriptionError::PairOfOneLink {
                link: pair.link1.clone(),
            });
        }
        let ordered = if pair.link1 < pair.link2 {
            (pair.link1.as_str(), pair.link2.as_str())
        } else {
            (pair.link2.as_str(), pair.link1.as_str())
        };
        if !seen.insert(ordered) {
            return Err(DescriptionError::DuplicatePair {
                link1: pair.link1.clone(),
                link2: pair.link2.clone(),
            });
        }
    }
    Ok(pairs)
}

/// The optical frame of each camera of `robot`, each with the depth range of the sensor of
/// `depth_sensors` that references it. A sensor that references a link the URDF lacks or
/// no camera's optical frame, and two sensors of one camera, are refused.
fn cameras(
    robot: &Robot,
    tree: &Tree,
    depth_sensors: &[DepthSensor],
) -> Result<Vec<CameraFrame>, DescriptionError> {
    let optical_frames: Vec<(&str, &str)> = robot
        .links
        .iter()
        .filter_map(|link| {
            let name = link.name.strip_suffix(OPTICAL_FRAME_SUFFIX)?;
            (!name.is_empty()).then_some((name, link.name.as_str()))
        })
        .collect();
    let mut depth_ranges: HashMap<&str, DepthRange> = HashMap::new();
    for sensor in depth_sensors {
        tree.link("gazebo", &sensor.reference)?;
        let &(camera, _) = optical_frames
            .iter()
            .find(|(_, frame)| *frame == sensor.reference)
            .ok_or_else(|| DescriptionError::SensorOffACamera {
                reference: sensor.reference.clone(),
            })?;
        if depth_ranges.insert(camera, sensor.range).is_some() {
            return Err(DescriptionError::DepthSensorTwice {
                camera: camera.to_string(),
            });
        }
    }
    optical_frames
        .into_iter()
        .map(|(name, frame)| {
            let joint = tree.parent_joint.get(frame).ok_or_else(|| {
                DescriptionError::CameraFrameUnmounted {
                    frame: frame.to_string(),
                }
            })?;
            if moves(joint) {
                return Err(DescriptionError::CameraFrameMoves {
                    frame: frame.to_string(),
                    joint: joint.name.clone(),
                });
            }
            Ok(CameraFrame {
                name: name.to_string(),
                parent_link: joint.parent.link.clone(),
                xyz: joint.origin.xyz.0,
                rpy: joint.origin.rpy.0,
                depth_range: depth_ranges.get(name).copied(),
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const URDF: &str = r#"<robot name="mini">
  <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="upper"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="hand"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="grasp"/>
  <link name="finger_a"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="finger_b"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="eye_optical_frame"/>
  <joint name="lift" type="revolute">
    <parent link="base"/><child link="upper"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="turn" type="revolute">
    <parent link="upper"/><child link="hand"/><axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="grasp_joint" type="fixed"><parent link="hand"/><child link="grasp"/></joint>
  <joint name="pinch" type="prismatic">
    <parent link="hand"/><child link="finger_a"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" effort="1" velocity="1"/>
  </joint>
  <joint name="pinch_mirror" type="prismatic">
    <parent link="hand"/><child link="finger_b"/><axis xyz="0 1 0"/>
    <mimic joint="pinch" multiplier="-1" offset="0.01"/>
    <limit lower="-0.03" upper="0.01" effort="1" velocity="1"/>
  </joint>
  <joint name="eye" type="fixed">
    <origin xyz="0.1 0.2 0.3" rpy="0.4 0.5 0.6"/><parent link="hand"/><child link="eye_optical_frame"/>
  </joint>
  <gazebo reference="eye_optical_frame">
    <sensor name="eye" type="depth"><camera><clip><near>0.2</near><far>4.5</far></clip></camera></sensor>
  </gazebo>
</robot>"#;

    const SRDF: &str = r#"<robot name="mini">
  <group name="arm"><chain base_link="base" tip_link="grasp"/></group>
  <group name="gripper"><joint name="pinch"/><joint name="pinch_mirror"/></group>
  <group name="all"><group name="arm"/><group name="gripper"/></group>
  <group_state name="ready" group="arm">
    <joint name="lift" value="0.5"/><joint name="turn" value="-0.25"/>
  </group_state>
  <group_state name="closed" group="gripper"><joint name="pinch" value="0"/></group_state>
  <group_state name="open" group="gripper"><joint name="pinch" value="0.04"/></group_state>
  <end_effector name="ee" parent_link="grasp" group="gripper" parent_group="arm"/>
  <passive_joint name="pinch_mirror"/>
  <disable_collisions link1="base" link2="upper" reason="Adjacent"/>
  <disable_collisions link1="finger_a" link2="finger_b" reason="Never"/>
</robot>"#;

    #[test]
    fn an_arm_is_the_chain_an_end_effector_stands_on_with_its_gripper() {
        let description = RobotDescription::parse(URDF, SRDF).expect("the description");
        assert_eq!(
            description.arms(),
            [Arm {
                name: "arm".to_string(),
                base_link: "base".to_string(),
                grasp_link: "grasp".to_string(),
                joints: vec!["lift".to_string(), "turn".to_string()],
                limits: vec![[-1.0, 1.0], [-2.0, 2.0]],
                gripper: Gripper {
                    name: "gripper".to_string(),
                    fingers: vec![
                        Finger {
                            joint: "pinch".to_string(),
                            link: "finger_a".to_string(),
                            closed: 0.0,
                            open: 0.04,
                            mimic: None,
                        },
                        // The mimic's multiplier and offset place it off its leader.
                        Finger {
                            joint: "pinch_mirror".to_string(),
                            link: "finger_b".to_string(),
                            closed: 0.01,
                            open: -0.03,
                            mimic: Some(Mimic {
                                joint: "pinch".to_string(),
                                multiplier: -1.0,
                                offset: 0.01,
                            }),
                        },
                    ],
                },
            }]
        );
        assert_eq!(description.urdf(), URDF);
        assert_eq!(description.srdf(), SRDF);
        assert_eq!(description.robot().name, "mini");
    }

    // A group's arms are the group itself when it is an arm, else the arms it holds in their
    // order; a group that holds anything but arms has none.
    #[test]
    fn a_groups_arms_are_the_arm_itself_or_the_arms_it_holds() {
        let description = RobotDescription::parse(URDF, SRDF).expect("the fixture reads");
        let names = |group: &str| {
            description
                .group_arms(group)
                .map(|arms| arms.iter().map(|arm| arm.name.as_str()).collect::<Vec<_>>())
        };
        assert_eq!(names("arm"), Some(vec!["arm"]));
        for group in ["all", "gripper", "no_such_group"] {
            assert_eq!(names(group), None, "{group}");
        }
        let both = crate::model("openarm_v2")
            .expect("a bundled model")
            .description();
        let arms: Vec<&str> = both
            .group_arms("both_arms")
            .expect("both_arms holds the two arms")
            .iter()
            .map(|arm| arm.name.as_str())
            .collect();
        assert_eq!(arms, ["left_arm", "right_arm"]);
    }

    // Each OpenArm gripper's second finger follows its first one to one, as the URDF's
    // <mimic> gives it: v1's through the defaults of a <mimic> that gives no multiplier
    // and no offset, v2's written out. The SO-101's one jaw moves of its own.
    #[test]
    fn a_finger_that_mimics_another_carries_its_mimic() {
        for id in ["openarm_v1", "openarm_v2"] {
            let description = crate::model(id).expect("a bundled model").description();
            for arm in description.arms() {
                let [first, second] = arm.gripper.fingers.as_slice() else {
                    panic!("{id}: {} has two fingers", arm.gripper.name);
                };
                assert_eq!(first.mimic, None, "{id}: {}", first.joint);
                assert_eq!(
                    second.mimic,
                    Some(Mimic {
                        joint: first.joint.clone(),
                        multiplier: 1.0,
                        offset: 0.0,
                    }),
                    "{id}: {}",
                    second.joint
                );
            }
        }
        let so101 = crate::model("so101")
            .expect("a bundled model")
            .description();
        let [jaw] = so101.arms()[0].gripper.fingers.as_slice() else {
            panic!("the SO-101 has one jaw");
        };
        assert_eq!(jaw.mimic, None);
    }

    #[test]
    fn a_state_gives_the_groups_own_joints_in_the_groups_order() {
        let description = RobotDescription::parse(URDF, SRDF).expect("the description");
        assert_eq!(
            description.state("arm", "ready"),
            Some([0.5, -0.25].as_slice())
        );
        assert_eq!(
            description.state("gripper", "open"),
            Some([0.04].as_slice())
        );
        assert_eq!(description.state("arm", "open"), None);
        assert_eq!(description.state("gripper", "ready"), None);
    }

    #[test]
    fn the_disabled_pairs_and_the_cameras_are_the_files() {
        let description = RobotDescription::parse(URDF, SRDF).expect("the description");
        assert_eq!(
            description.disabled_pairs(),
            [
                DisabledPair {
                    link1: "base".to_string(),
                    link2: "upper".to_string(),
                    reason: DisabledReason::Adjacent,
                },
                DisabledPair {
                    link1: "finger_a".to_string(),
                    link2: "finger_b".to_string(),
                    reason: DisabledReason::Never,
                },
            ]
        );
        assert_eq!(
            description.cameras(),
            [CameraFrame {
                name: "eye".to_string(),
                parent_link: "hand".to_string(),
                xyz: [0.1, 0.2, 0.3],
                rpy: [0.4, 0.5, 0.6],
                depth_range: DepthRange::new(0.2, 4.5),
            }]
        );
        let depth = description.cameras()[0]
            .depth_range
            .expect("the eye measures depth");
        assert_eq!((depth.near(), depth.far()), (0.2, 4.5));
    }

    /// The fixture URDF's depth sensor: its `<gazebo>` element, on the eye's optical frame.
    fn eye_depth_sensor() -> &'static str {
        let start = URDF.find("  <gazebo").expect("the fixture's depth sensor");
        let end = URDF.find("</robot>").expect("the fixture's end");
        &URDF[start..end]
    }

    #[test]
    fn a_camera_with_no_depth_sensor_measures_no_depth() {
        let description = RobotDescription::parse(&URDF.replacen(eye_depth_sensor(), "", 1), SRDF)
            .expect("a URDF with no Gazebo extension");
        assert_eq!(description.cameras()[0].depth_range, None);
    }

    // The OpenArm v2's chest camera measures depth; its wrist cameras and the SO-101's
    // camera give color alone.
    #[test]
    fn of_the_bundled_cameras_the_v2_chest_camera_alone_measures_depth() {
        let depth_cameras = |id: &str| {
            crate::model(id)
                .expect("a bundled model")
                .description()
                .cameras()
                .iter()
                .filter_map(|camera| Some((camera.name.clone(), camera.depth_range?)))
                .map(|(name, range)| (name, range.near(), range.far()))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            depth_cameras("openarm_v2"),
            [("chest".to_string(), 0.1, 15.0)]
        );
        for id in ["openarm_v1", "so101"] {
            assert_eq!(depth_cameras(id), [], "{id}");
        }
    }

    #[test]
    fn a_gazebo_element_that_is_no_cameras_depth_sensor_is_refused() {
        let form = "a camera's depth range is <gazebo reference=\"<camera>_optical_frame\"><sensor \
             name=\"<camera>\" type=\"depth\"><camera><clip><near>NEAR</near><far>FAR</far></clip>\
             </camera></sensor></gazebo>, in meters";
        assert_eq!(
            urdf_refusal(r#"type="depth""#, r#"type="camera""#),
            format!(
                "the sensor on \"eye_optical_frame\" is of the type \"camera\", and this reader \
                 reads a sensor of the type \"depth\"; {form}"
            )
        );
        assert_eq!(
            urdf_refusal(r#"reference="eye_optical_frame""#, r#"reference="hand""#),
            format!(
                "<gazebo> references \"hand\", which is the optical frame of no camera; {form}"
            )
        );
        assert_eq!(
            urdf_refusal(
                r#"reference="eye_optical_frame""#,
                r#"reference="ear_optical_frame""#
            ),
            "<gazebo> names the link \"ear_optical_frame\", and the URDF has no such link"
        );
        assert_eq!(
            urdf_refusal(r#"<gazebo reference="eye_optical_frame">"#, "<gazebo>"),
            "<gazebo> has no attribute \"reference\""
        );
        assert_eq!(
            urdf_refusal(r#"name="eye" type="depth""#, r#"type="depth""#),
            "<sensor> has no attribute \"name\""
        );
        let sensor = eye_depth_sensor();
        assert_eq!(
            urdf_refusal(sensor, &format!("{sensor}{sensor}")),
            "the camera \"eye\" has two depth sensors; a camera measures one depth range"
        );
    }

    #[test]
    fn what_a_depth_sensor_does_not_hold_or_holds_unread_is_refused() {
        assert_eq!(
            urdf_refusal("<sensor ", "<material>Gazebo/Grey</material><sensor "),
            "<gazebo> holds the element <material>, which this reader does not read; in \
             <gazebo> it reads sensor"
        );
        assert_eq!(
            urdf_refusal("<clip>", "<horizontal_fov>1.0</horizontal_fov><clip>"),
            "<camera> holds the element <horizontal_fov>, which this reader does not read; in \
             <camera> it reads clip"
        );
        assert_eq!(
            urdf_refusal("<near>0.2</near>", "<near><value>0.2</value></near>"),
            "<near> holds the element <value>, which this reader does not read; in <near> it \
             reads no element"
        );
        assert_eq!(
            urdf_refusal("<clip>", r#"<clip unit="m">"#),
            "<clip> has the attribute \"unit\", which it does not take"
        );
        assert_eq!(urdf_refusal("<far>4.5</far>", ""), "<clip> holds no <far>");
        assert_eq!(
            urdf_refusal(
                "<camera><clip><near>0.2</near><far>4.5</far></clip></camera>",
                ""
            ),
            "<sensor> holds no <camera>"
        );
        assert_eq!(
            urdf_refusal("<far>4.5</far>", "<far>4.5</far><far>5</far>"),
            "<clip> holds <far> twice; it holds one <far>"
        );
    }

    #[test]
    fn a_clip_that_is_not_a_range_of_depths_is_refused() {
        for value in ["near", "inf", "NaN", "", "0.2 0.3"] {
            assert_eq!(
                urdf_refusal("<near>0.2</near>", &format!("<near>{value}</near>")),
                format!(
                    "the <near> of the depth sensor on \"eye_optical_frame\" is {value:?}, not one \
                     finite number of meters"
                )
            );
        }
        for (near, far) in [("4.5", "0.2"), ("0", "4.5"), ("-0.1", "4.5"), ("1", "1")] {
            assert_eq!(
                urdf_refusal(
                    "<near>0.2</near><far>4.5</far>",
                    &format!("<near>{near}</near><far>{far}</far>")
                ),
                format!(
                    "the depth sensor on \"eye_optical_frame\" clips at {near} and {far} meters; \
                     its <near> is above 0 and below its <far>"
                )
            );
        }
    }

    /// The refusal of the description whose SRDF has `from` replaced by `to`.
    fn srdf_refusal(from: &str, to: &str) -> String {
        assert!(SRDF.contains(from), "the SRDF holds {from:?}");
        RobotDescription::parse(URDF, &SRDF.replacen(from, to, 1))
            .expect_err("a refusal")
            .to_string()
    }

    /// The refusal of the description whose URDF has `from` replaced by `to`.
    fn urdf_refusal(from: &str, to: &str) -> String {
        assert!(URDF.contains(from), "the URDF holds {from:?}");
        RobotDescription::parse(&URDF.replacen(from, to, 1), SRDF)
            .expect_err("a refusal")
            .to_string()
    }

    #[test]
    fn what_the_reader_does_not_read_is_refused_by_name() {
        assert_eq!(
            srdf_refusal(
                r#"<passive_joint name="pinch_mirror"/>"#,
                r#"<virtual_joint name="v" type="fixed" parent_frame="world" child_link="base"/>"#
            ),
            "<robot> holds the element <virtual_joint>, which this reader does not read; in \
             <robot> it reads group, group_state, end_effector, passive_joint, disable_collisions"
        );
        assert_eq!(
            srdf_refusal(r#"<joint name="pinch"/>"#, r#"<sphere name="pinch"/>"#),
            "<group> holds the element <sphere>, which this reader does not read; in <group> it \
             reads chain, joint, link, group"
        );
        assert_eq!(
            srdf_refusal(
                r#"<joint name="pinch" value="0"/>"#,
                r#"<link name="hand"/>"#
            ),
            "<group_state> holds the element <link>, which this reader does not read; in \
             <group_state> it reads joint"
        );
        assert_eq!(
            srdf_refusal(r#"reason="Adjacent""#, r#"reason="Adjacent" padding="0.1""#),
            "<disable_collisions> has the attribute \"padding\", which it does not take"
        );
        assert_eq!(
            srdf_refusal(r#" parent_link="grasp""#, ""),
            "<end_effector> has no attribute \"parent_link\""
        );
        assert_eq!(
            srdf_refusal(r#"<robot name="mini">"#, r#"<semantics name="mini">"#)
                .split(':')
                .next(),
            Some("the SRDF does not parse")
        );
    }

    #[test]
    fn a_name_the_urdf_lacks_is_refused() {
        assert_eq!(
            srdf_refusal(r#"<robot name="mini">"#, r#"<robot name="maxi">"#),
            "the SRDF is of the robot \"maxi\" and the URDF of \"mini\""
        );
        assert_eq!(
            srdf_refusal(r#"tip_link="grasp""#, r#"tip_link="grip""#),
            "<chain> names the link \"grip\", and the URDF has no such link"
        );
        assert_eq!(
            srdf_refusal(
                r#"<joint name="pinch_mirror"/>"#,
                r#"<joint name="squeeze"/>"#
            ),
            "<group> names the joint \"squeeze\", and the URDF has no such joint"
        );
        assert_eq!(
            srdf_refusal(
                r#"<passive_joint name="pinch_mirror"/>"#,
                r#"<passive_joint name="idle"/>"#
            ),
            "<passive_joint> names the joint \"idle\", and the URDF has no such joint"
        );
        assert_eq!(
            srdf_refusal(r#"link2="upper""#, r#"link2="lower""#),
            "<disable_collisions> names the link \"lower\", and the URDF has no such link"
        );
        assert_eq!(
            srdf_refusal(r#"<group name="gripper"/>"#, r#"<group name="hand"/>"#),
            "<group> names the group \"hand\", and the SRDF has no such group"
        );
        assert_eq!(
            srdf_refusal(r#"name="ready" group="arm""#, r#"name="ready" group="leg""#),
            "<group_state> names the group \"leg\", and the SRDF has no such group"
        );
        assert_eq!(
            srdf_refusal(
                r#"group="gripper" parent_group"#,
                r#"group="claw" parent_group"#
            ),
            "<end_effector> names the group \"claw\", and the SRDF has no such group"
        );
    }

    #[test]
    fn a_group_that_does_not_hold_is_refused() {
        assert_eq!(
            srdf_refusal(r#"<group name="all">"#, r#"<group name="arm">"#),
            "the group \"arm\" is given twice"
        );
        assert_eq!(
            srdf_refusal(
                r#"<group name="arm"/><group name="gripper"/>"#,
                r#"<group name="all"/>"#
            ),
            "the group \"all\" holds itself through its groups"
        );
        assert_eq!(
            srdf_refusal(
                r#"base_link="base" tip_link="grasp""#,
                r#"base_link="grasp" tip_link="base""#
            ),
            "the chain of the group \"arm\" runs to \"base\", which is not below \"grasp\""
        );
    }

    #[test]
    fn a_state_that_does_not_place_its_group_is_refused() {
        assert_eq!(
            srdf_refusal(
                r#"<group_state name="open" group="gripper">"#,
                r#"<group_state name="closed" group="gripper">"#
            ),
            "the state \"closed\" of the group \"gripper\" is given twice"
        );
        assert_eq!(
            srdf_refusal(
                r#"<joint name="turn" value="-0.25"/>"#,
                r#"<joint name="pinch" value="0"/>"#
            ),
            "the state \"ready\" of the group \"arm\" names \"pinch\", which is not a joint of the \
             group that moves of its own; those are lift, turn"
        );
        assert_eq!(
            srdf_refusal(
                r#"<joint name="pinch" value="0"/>"#,
                r#"<joint name="pinch_mirror" value="0"/>"#
            ),
            "the state \"closed\" of the group \"gripper\" names \"pinch_mirror\", which is not a \
             joint of the group that moves of its own; those are pinch"
        );
        assert_eq!(
            srdf_refusal(
                r#"<joint name="turn" value="-0.25"/>"#,
                r#"<joint name="lift" value="0.5"/>"#
            ),
            "the state \"ready\" of the group \"arm\" names \"lift\" twice"
        );
        assert_eq!(
            srdf_refusal(r#"<joint name="turn" value="-0.25"/>"#, ""),
            "the state \"ready\" of the group \"arm\" gives no value for \"turn\""
        );
        assert_eq!(
            srdf_refusal(r#"value="0.5""#, r#"value="1.5""#),
            "the state \"ready\" of the group \"arm\" puts \"lift\" at 1.5, outside its limits -1 to 1"
        );
        for value in ["0.5 0.5", "half", "inf", ""] {
            assert_eq!(
                srdf_refusal(r#"value="0.5""#, &format!(r#"value="{value}""#)),
                format!(
                    "the value {value:?} of \"lift\" in the state \"ready\" is not one finite number"
                )
            );
        }
    }

    #[test]
    fn an_end_effector_that_stands_on_no_one_arm_is_refused() {
        let ee =
            r#"<end_effector name="ee" parent_link="grasp" group="gripper" parent_group="arm"/>"#;
        assert_eq!(
            srdf_refusal(ee, &format!("{ee}{ee}")),
            "the end effector \"ee\" is given twice"
        );
        assert_eq!(
            srdf_refusal(r#"parent_link="grasp""#, r#"parent_link="hand""#),
            "the end effector \"ee\" stands on \"hand\", which is the tip of no group of one \
             chain; an arm is a group of one chain to its grasp point"
        );
        assert_eq!(
            srdf_refusal(r#"parent_group="arm""#, r#"parent_group="all""#),
            "the end effector \"ee\" stands on the tip of the group \"arm\" and names \"all\" as \
             its parent group"
        );
        assert_eq!(
            srdf_refusal(
                r#"<group name="all">"#,
                r#"<group name="arm_again"><chain base_link="upper" tip_link="grasp"/></group><group name="all">"#
            ),
            "the groups \"arm\" and \"arm_again\" are both a chain to \"grasp\"; one arm ends at \
             a grasp point"
        );
        assert_eq!(
            srdf_refusal(
                ee,
                &format!("{ee}{}", ee.replace(r#"name="ee""#, r#"name="ee_again""#))
            ),
            "the end effectors \"ee\" and \"ee_again\" both stand on the arm \"arm\""
        );
    }

    #[test]
    fn a_gripper_without_its_ends_is_refused() {
        assert_eq!(
            srdf_refusal(
                r#"<group_state name="closed" group="gripper"><joint name="pinch" value="0"/></group_state>"#,
                ""
            ),
            "the gripper \"gripper\" has no state \"closed\"; a gripper with a finger gives where \
             it is \"closed\" and \"open\""
        );
        assert_eq!(
            srdf_refusal(
                r#"<group_state name="open" group="gripper"><joint name="pinch" value="0.04"/></group_state>"#,
                ""
            ),
            "the gripper \"gripper\" has no state \"open\"; a gripper with a finger gives where it \
             is \"closed\" and \"open\""
        );
        assert_eq!(
            urdf_refusal(r#"<mimic joint="pinch""#, r#"<mimic joint="turn""#),
            "the finger joint \"pinch_mirror\" of the gripper \"gripper\" mimics \"turn\", which \
             is not a joint of the gripper that moves of its own"
        );
    }

    // Each reason reads back from its name, so ALL, which the reader parses a reason
    // against, holds it. The match names every variant: a variant added to the enum does
    // not compile here until it is listed.
    #[test]
    fn every_reason_reads_back_from_its_name() {
        use DisabledReason::{Adjacent, Always, Default, Never, User};
        for reason in [Adjacent, Never, Default, Always, User] {
            match reason {
                Adjacent | Never | Default | Always | User => {}
            }
            assert_eq!(DisabledReason::parse(reason.name()), Some(reason));
        }
    }

    #[test]
    fn a_disabled_pair_that_names_no_pair_is_refused() {
        assert_eq!(
            srdf_refusal(r#"reason="Adjacent""#, r#"reason="Nearby""#),
            "<disable_collisions> gives the reason \"Nearby\"; the reasons are Adjacent, Never, \
             Default, Always, User"
        );
        assert_eq!(
            srdf_refusal(r#"link2="upper""#, r#"link2="base""#),
            "<disable_collisions> names \"base\" twice"
        );
        assert_eq!(
            srdf_refusal(r#"link2="upper""#, r#"link2="grasp""#),
            "<disable_collisions> names \"grasp\", which has no collision geometry"
        );
        assert_eq!(
            srdf_refusal(
                r#"link1="finger_a" link2="finger_b""#,
                r#"link1="upper" link2="base""#
            ),
            "the pair \"upper\" and \"base\" is disabled twice"
        );
    }

    #[test]
    fn a_camera_frame_that_is_fixed_to_no_link_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<joint name="eye" type="fixed">"#,
                r#"<joint name="eye" type="continuous">"#
            ),
            "the camera frame \"eye_optical_frame\" hangs on the joint \"eye\", which moves; an \
             optical frame is fixed to its link"
        );
        assert_eq!(
            urdf_refusal(
                r#"<parent link="hand"/><child link="eye_optical_frame"/>"#,
                r#"<parent link="eye_optical_frame"/><child link="base"/>"#
            ),
            "the camera frame \"eye_optical_frame\" hangs on no joint"
        );
    }

    #[test]
    fn a_link_on_two_joints_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<child link="eye_optical_frame"/>"#,
                r#"<child link="upper"/>"#
            ),
            "the link \"upper\" hangs on the joints \"lift\" and \"eye\"; a link hangs on one \
             joint"
        );
    }

    #[test]
    fn a_link_given_twice_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<link name="grasp"/>"#,
                r#"<link name="grasp"/><link name="grasp"/>"#
            ),
            "the link \"grasp\" is given twice"
        );
    }

    #[test]
    fn a_joint_given_twice_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<joint name="eye" type="fixed">"#,
                r#"<joint name="turn" type="fixed">"#
            ),
            "the joint \"turn\" is given twice"
        );
    }

    #[test]
    fn a_joint_naming_a_link_the_urdf_lacks_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<parent link="hand"/><child link="grasp"/>"#,
                r#"<parent link="palm"/><child link="grasp"/>"#
            ),
            "the joint \"grasp_joint\" names the link \"palm\", and the URDF has no such link"
        );
    }

    #[test]
    fn a_joint_of_more_than_one_position_is_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<joint name="eye" type="fixed">"#,
                r#"<joint name="eye" type="floating">"#
            ),
            "the joint \"eye\" is floating, which takes more than one position; a joint is \
             revolute, continuous, prismatic or fixed"
        );
    }

    #[test]
    fn links_that_hang_from_two_roots_are_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<link name="grasp"/>"#,
                r#"<link name="grasp"/><link name="spare"/>"#
            ),
            "the links base, spare hang on no joint; a URDF has one root link, and every \
             other link hangs on a joint"
        );
    }

    #[test]
    fn joints_that_join_links_in_a_loop_are_refused() {
        assert_eq!(
            urdf_refusal(
                r#"<parent link="hand"/><child link="grasp"/>"#,
                r#"<parent link="hand"/><child link="base"/>"#
            ),
            "the joints lift, grasp_joint, turn join their links in a loop; every link hangs \
             below the root link"
        );
    }

    // Meshes of one file name from different paths are refused; one mesh named twice is
    // one mesh.
    #[test]
    fn two_meshes_of_one_file_name_are_refused() {
        let meshed = |link: &str, path: &str| {
            let boxed =
                format!(r#"<link name="{link}"><collision><geometry><box size="0.1 0.1 0.1"/>"#);
            let mesh =
                format!(r#"<link name="{link}"><collision><geometry><mesh filename="{path}"/>"#);
            move |urdf: &str| {
                assert!(urdf.contains(&boxed), "the URDF holds {boxed:?}");
                urdf.replacen(&boxed, &mesh, 1)
            }
        };
        let urdf =
            meshed("hand", "package://b/part.stl")(&meshed("upper", "package://a/part.stl")(URDF));
        assert_eq!(
            RobotDescription::parse(&urdf, SRDF)
                .expect_err("a refusal")
                .to_string(),
            "the collision meshes \"package://a/part.stl\" and \"package://b/part.stl\" share the \
             file name \"part.stl\"; a description holds its meshes by file name"
        );
        let urdf =
            meshed("hand", "package://a/part.stl")(&meshed("upper", "package://a/part.stl")(URDF));
        RobotDescription::parse(&urdf, SRDF).expect("one mesh named twice");
    }

    #[test]
    fn a_urdf_that_does_not_parse_is_refused() {
        let refusal = RobotDescription::parse("<robot", SRDF).expect_err("a refusal");
        assert!(matches!(refusal, DescriptionError::Urdf(_)), "{refusal}");
    }
}
