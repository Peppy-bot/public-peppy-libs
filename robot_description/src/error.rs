//! Why a description is refused. Each variant names what its files got wrong, so a
//! message is actionable without reading this crate.

use crate::DisabledReason;
use crate::description::LEAST_FINGER_TRAVEL;

/// The ends of a finger span that [`FingerSpan::new`](crate::FingerSpan::new) refuses:
/// one is not finite, or the travel between them is not finite or at most
/// `LEAST_FINGER_TRAVEL`, too small for an opening to scale.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "a finger span stands at two finite positions more than {} apart, and {closed} and \
     {open} are not",
    LEAST_FINGER_TRAVEL
)]
pub struct FingerSpanError {
    pub closed: f64,
    pub open: f64,
}

/// A joint that a lookup by name on a [`RobotDescription`](crate::RobotDescription) is
/// asked about and the description's URDF lacks.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("the URDF has no joint named {joint:?}")]
pub struct UnknownJoint {
    pub joint: String,
}

/// A link that a lookup by name on a [`RobotDescription`](crate::RobotDescription) is
/// asked about and the description's URDF lacks.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("the URDF has no link named {link:?}")]
pub struct UnknownLink {
    pub link: String,
}

/// A group that [`RobotDescription::group_arms`](crate::RobotDescription::group_arms) is
/// asked about and that is no arm and no group of arms of the description's SRDF.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("the SRDF has no arm and no group of arms named {group:?}")]
pub struct UnknownGroup {
    pub group: String,
}

/// A state that [`RobotDescription::state`](crate::RobotDescription::state) is asked
/// about and the description's SRDF does not give the group.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("the SRDF gives the group {group:?} no state named {state:?}")]
pub struct UnknownState {
    pub group: String,
    pub state: String,
}

/// A URDF and an SRDF that do not make one description.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum DescriptionError {
    #[error("the URDF does not parse: {0}")]
    Urdf(String),
    #[error("the SRDF does not parse: {0}")]
    Srdf(String),
    #[error(
        "<{parent}> holds the element <{element}>, which this reader does not read; in \
         <{parent}> it reads {}", read_elements(.reads)
    )]
    UnreadElement {
        element: String,
        parent: &'static str,
        reads: &'static [&'static str],
    },
    #[error("<{element}> has the attribute {attribute:?}, which it does not take")]
    UnreadAttribute { element: String, attribute: String },
    #[error("<{element}> has no attribute {attribute:?}")]
    MissingAttribute {
        element: String,
        attribute: &'static str,
    },
    #[error("<{parent}> holds no <{element}>")]
    MissingElement {
        parent: &'static str,
        element: &'static str,
    },
    #[error("<{parent}> holds <{element}> twice; it holds one <{element}>")]
    ElementTwice {
        parent: &'static str,
        element: &'static str,
    },
    #[error(transparent)]
    Tree(#[from] chain_kinematics::TreeError),
    #[error(
        "the collision meshes {first:?} and {second:?} share the file name {name:?}; a \
         description holds its meshes by file name"
    )]
    MeshNameTwice {
        name: String,
        first: String,
        second: String,
    },
    #[error("the SRDF is of the robot {srdf:?} and the URDF of {urdf:?}")]
    RobotName { srdf: String, urdf: String },
    #[error("<{element}> names the link {link:?}, and the URDF has no such link")]
    NoSuchLink { element: &'static str, link: String },
    #[error("<{element}> names the joint {joint:?}, and the URDF has no such joint")]
    NoSuchJoint {
        element: &'static str,
        joint: String,
    },
    #[error("<{element}> names the group {group:?}, and the SRDF has no such group")]
    NoSuchGroup {
        element: &'static str,
        group: String,
    },
    #[error("the group {0:?} is given twice")]
    DuplicateGroup(String),
    #[error("the group {group:?} holds itself through its groups")]
    GroupHoldsItself { group: String },
    #[error("the chain of the group {group:?} runs to {tip:?}, which is not below {base:?}")]
    TipNotBelowBase {
        group: String,
        base: String,
        tip: String,
    },
    #[error("the state {state:?} of the group {group:?} is given twice")]
    DuplicateState { group: String, state: String },
    #[error(
        "the state {state:?} of the group {group:?} names {joint:?}, which is not a joint \
         of the group that moves of its own; those are {}", listed(.joints)
    )]
    StateJointOffTheGroup {
        group: String,
        state: String,
        joint: String,
        joints: Vec<String>,
    },
    #[error("the state {state:?} of the group {group:?} names {joint:?} twice")]
    StateJointTwice {
        group: String,
        state: String,
        joint: String,
    },
    #[error("the state {state:?} of the group {group:?} gives no value for {joint:?}")]
    StateJointMissing {
        group: String,
        state: String,
        joint: String,
    },
    #[error(
        "the state {state:?} of the group {group:?} puts {joint:?} at {value}, outside its \
         limits {lower} to {upper}"
    )]
    StateOutsideLimits {
        group: String,
        state: String,
        joint: String,
        value: f64,
        lower: f64,
        upper: f64,
    },
    #[error("the value {value:?} of {joint:?} in the state {state:?} is not one finite number")]
    StateValue {
        state: String,
        joint: String,
        value: String,
    },
    #[error("the end effector {0:?} is given twice")]
    DuplicateEndEffector(String),
    #[error(
        "the end effector {end_effector:?} stands on {parent_link:?}, which is the tip of no \
         group of one chain; an arm is a group of one chain to its grasp point"
    )]
    EndEffectorOffAnArm {
        end_effector: String,
        parent_link: String,
    },
    #[error(
        "the end effector {end_effector:?} stands on the tip of the group {arm:?} and names \
         {parent_group:?} as its parent group"
    )]
    EndEffectorParentGroup {
        end_effector: String,
        arm: String,
        parent_group: String,
    },
    #[error(
        "the groups {first:?} and {second:?} are both a chain to {tip:?}; one arm ends at a grasp point"
    )]
    TwoArmsOfOneTip {
        first: String,
        second: String,
        tip: String,
    },
    #[error("the end effectors {first:?} and {second:?} both stand on the arm {arm:?}")]
    TwoEndEffectorsOnOneArm {
        arm: String,
        first: String,
        second: String,
    },
    #[error(
        "the gripper {gripper:?} has no state {state:?}; a gripper with a finger gives where it is \"closed\" and \"open\""
    )]
    GripperState {
        gripper: String,
        state: &'static str,
    },
    #[error(
        "the finger joint {joint:?} of the gripper {gripper:?} mimics {leader:?}, which is not a joint of the gripper that moves of its own"
    )]
    MimicOffTheGripper {
        gripper: String,
        joint: String,
        leader: String,
    },
    #[error(
        "the states \"closed\" and \"open\" of the gripper {gripper:?} put its finger joint \
         {joint:?} where no span stands: {error}"
    )]
    FingerSpan {
        gripper: String,
        joint: String,
        error: FingerSpanError,
    },
    #[error(
        "<disable_collisions> gives the reason {reason:?}; the reasons are {}",
        reason_names()
    )]
    Reason { reason: String },
    #[error("<disable_collisions> names {link:?} twice")]
    PairOfOneLink { link: String },
    #[error("<disable_collisions> names {link:?}, which has no collision geometry")]
    PairWithoutGeometry { link: String },
    #[error("the pair {link1:?} and {link2:?} is disabled twice")]
    DuplicatePair { link1: String, link2: String },
    #[error(
        "the sensor frame {frame:?} hangs on the joint {joint:?}, which moves; a sensor's frame is fixed to its link"
    )]
    SensorFrameMoves { frame: String, joint: String },
    #[error("the sensor frame {frame:?} hangs on no joint")]
    SensorFrameUnmounted { frame: String },
    #[error(
        "the sensor on {reference:?} is of the type {kind:?}, and this reader reads a sensor of \
         the type \"depth\" or \"gpu_lidar\"; {}; {}",
        DEPTH_SENSOR_FORM,
        LIDAR_FORM
    )]
    SensorType { reference: String, kind: String },
    #[error(
        "the <{bound}> of the depth sensor on {reference:?} is {value:?}, not one finite \
         number of meters"
    )]
    ClipValue {
        reference: String,
        bound: &'static str,
        value: String,
    },
    #[error(
        "the depth sensor on {reference:?} clips at {near} and {far} meters; its <near> is \
         above 0 and below its <far>"
    )]
    ClipRange {
        reference: String,
        near: f64,
        far: f64,
    },
    #[error(
        "<gazebo> references {reference:?}, which is the optical frame of no camera; {}",
        DEPTH_SENSOR_FORM
    )]
    SensorOffACamera { reference: String },
    #[error("the camera {camera:?} has two depth sensors; a camera measures one depth range")]
    DepthSensorTwice { camera: String },
    #[error(
        "the <{bound}> of the lidar on {reference:?} is {value:?}, not one finite number of \
         meters"
    )]
    LidarRangeValue {
        reference: String,
        bound: &'static str,
        value: String,
    },
    #[error(
        "the lidar on {reference:?} measures from {min} to {max} meters; its <min> is 0 or \
         above and below its <max>"
    )]
    LidarRangeBounds {
        reference: String,
        min: f64,
        max: f64,
    },
    #[error(
        "two sensors are named {name:?}; name each camera and each lidar of the robot apart, \
         a camera by its optical frame <camera>_optical_frame and a lidar by its <sensor name>"
    )]
    SensorNameTwice { name: String },
}

/// The form of a camera's depth sensor, which a refusal of one names.
const DEPTH_SENSOR_FORM: &str = "a camera's depth range is <gazebo \
     reference=\"<camera>_optical_frame\"><sensor name=\"<camera>\" \
     type=\"depth\"><camera><clip><near>NEAR</near><far>FAR</far></clip></camera></sensor></gazebo>, \
     in meters";

/// The form of a 2-D lidar, which a refusal of one names.
const LIDAR_FORM: &str = "a 2-D lidar is <gazebo reference=\"<frame>\"><sensor name=\"<lidar>\" \
     type=\"gpu_lidar\"><lidar><range><min>MIN</min><max>MAX</max></range></lidar></sensor></gazebo>, \
     in meters";

/// The elements a reader reads in an element, listed.
fn read_elements(reads: &[&str]) -> String {
    match reads {
        [] => "no element".to_string(),
        _ => reads.join(", "),
    }
}

/// The names of the reasons of a disabled pair, as MoveIt's setup assistant writes them.
fn reason_names() -> String {
    DisabledReason::ALL.map(DisabledReason::name).join(", ")
}

fn listed(names: &[String]) -> String {
    names.join(", ")
}
