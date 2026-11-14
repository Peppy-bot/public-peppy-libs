//! The shape of a robot's arms: how many arms it has and how many joints each has. Code
//! compiled for the shapes it serves runs at the shape of a description's arms
//! ([`on_shape`]), and a node compiled for its own arms holds a bundled model to them
//! ([`NodeArms`]).

use std::fmt;

use crate::{ArmId, Driven, Model, PerArm, RobotDescription, UndrivenModel, models};

/// The joint counts of an arm of the shapes [`on_shape`] runs.
const ARM_JOINTS: [usize; 3] = [5, 6, 7];

/// The arm counts of the shapes [`on_shape`] runs.
const ARM_COUNTS: [usize; 2] = [1, 2];

/// What runs at the shape of a description's arms: `K` arms of `N` joints each, `D`
/// joints in all.
pub trait OnShape {
    type Output;

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> Self::Output;
}

/// Why [`on_shape`] runs nothing on a description.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum UnsupportedShape {
    #[error(
        "the description names no arm: an arm is an SRDF group of one chain whose tip is an \
         end effector's parent link"
    )]
    NoArm,
    #[error(
        "the arms differ in joint count: {first} has {first_joints} joints and {other} has \
         {other_joints}; the code is compiled for arms of one joint count"
    )]
    UnequalArms {
        first: String,
        first_joints: usize,
        other: String,
        other_joints: usize,
    },
    #[error(
        "the robot has {arms} arms of {joints} joints, and the code is compiled for {} arms \
         of {} joints", either(&ARM_COUNTS), either(&ARM_JOINTS)
    )]
    Uncompiled { joints: usize, arms: usize },
}

/// `counts` as a choice among them: "1 or 2", "5, 6 or 7".
fn either(counts: &[usize]) -> String {
    let named: Vec<String> = counts.iter().map(usize::to_string).collect();
    match named.split_last() {
        Some((last, [])) => last.clone(),
        Some((last, rest)) => format!("{} or {last}", rest.join(", ")),
        None => String::new(),
    }
}

/// Runs `on` at the shape of `description`'s arms. A description of no arm, of arms of
/// different joint counts, or of a shape no code is compiled for is refused, naming the
/// shapes the code is compiled for ([`UnsupportedShape`]).
pub fn on_shape<O: OnShape>(
    description: &RobotDescription,
    on: O,
) -> Result<O::Output, UnsupportedShape> {
    let arms = description.arms();
    let [first, rest @ ..] = arms else {
        return Err(UnsupportedShape::NoArm);
    };
    if let Some(other) = rest
        .iter()
        .find(|arm| arm.joints.len() != first.joints.len())
    {
        return Err(UnsupportedShape::UnequalArms {
            first: first.name.clone(),
            first_joints: first.joints.len(),
            other: other.name.clone(),
            other_joints: other.joints.len(),
        });
    }
    on_counts(first.joints.len(), arms.len(), on)
}

/// Runs `on` at `arms` arms of `joints` joints each.
fn on_counts<O: OnShape>(joints: usize, arms: usize, on: O) -> Result<O::Output, UnsupportedShape> {
    match (joints, arms) {
        (5, 1) => Ok(on.on_shape::<5, 5, 1>()),
        (5, 2) => Ok(on.on_shape::<5, 10, 2>()),
        (6, 1) => Ok(on.on_shape::<6, 6, 1>()),
        (6, 2) => Ok(on.on_shape::<6, 12, 2>()),
        (7, 1) => Ok(on.on_shape::<7, 7, 1>()),
        (7, 2) => Ok(on.on_shape::<7, 14, 2>()),
        (joints, arms) => Err(UnsupportedShape::Uncompiled { joints, arms }),
    }
}

/// The arms a node is compiled for: `K` arms of `N` joints each, in place order, each with
/// a gripper. Each arm and each gripper goes by the name the node's pairing slots carry
/// for it, the name its description's group gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeArms<const N: usize, const K: usize> {
    /// Each arm's name and its gripper's name, in place order.
    limbs: [(&'static str, &'static str); K],
}

/// One arm's shape: its name, the count of its joints and its gripper's name.
#[derive(Debug, Clone, Copy)]
struct ArmShape<'a> {
    arm: &'a str,
    joints: usize,
    gripper: &'a str,
}

/// Arms' shapes, in order, as a refusal writes them.
struct Shapes<'s, 'a>(&'s [ArmShape<'a>]);

impl fmt::Display for Shapes<'_, '_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0.is_empty() {
            return f.write_str("no arm");
        }
        let arms: Vec<String> = self
            .0
            .iter()
            .map(|shape| {
                format!(
                    "{} of {} joints with {}",
                    shape.arm, shape.joints, shape.gripper
                )
            })
            .collect();
        f.write_str(&arms.join(" and "))
    }
}

impl RobotDescription {
    /// Each arm's shape, in the order of the arms.
    fn arm_shapes(&self) -> Vec<ArmShape<'_>> {
        self.arms()
            .iter()
            .map(|arm| ArmShape {
                arm: &arm.name,
                joints: arm.joints.len(),
                gripper: &arm.gripper.name,
            })
            .collect()
    }
}

impl<const N: usize, const K: usize> NodeArms<N, K> {
    /// The arms of `limbs`, each an arm's name and its gripper's name, in place order.
    pub const fn new(limbs: [(&'static str, &'static str); K]) -> Self {
        Self { limbs }
    }

    /// The name of `arm`.
    pub fn arm_name(&self, arm: ArmId) -> &'static str {
        self.limbs[arm.place()].0
    }

    /// The name of `arm`'s gripper.
    pub fn gripper_name(&self, arm: ArmId) -> &'static str {
        self.limbs[arm.place()].1
    }

    /// The names of the arms, in place order.
    pub fn arm_names(&self) -> [&'static str; K] {
        self.limbs.map(|(arm, _)| arm)
    }

    /// The names of the grippers, in place order.
    pub fn gripper_names(&self) -> [&'static str; K] {
        self.limbs.map(|(_, gripper)| gripper)
    }

    /// The bundled model `id`, as robot_identity names it, with its description, refused
    /// unless its description's arms are these, in place order, naming both shapes and the
    /// bundled models whose arms are these.
    pub fn model(&self, id: &str) -> Result<Driven<N, K>, UndrivenModel> {
        let model = crate::model(id)?;
        let description = model.description();
        if let Some(joint_limits) = self.joint_limits_of(&description) {
            return Ok(Driven::new(model, description, *self, joint_limits));
        }
        Err(UndrivenModel::OtherArms {
            model: id.to_owned(),
            found: Shapes(&description.arm_shapes()).to_string(),
            built_for: Shapes(&self.shapes()).to_string(),
            fitting: self.models().map(Model::id).collect(),
        })
    }

    /// The bundled models whose arms are these, in the order of their ids: each model
    /// [`Self::model`] accepts.
    pub fn models(&self) -> impl Iterator<Item = &'static Model> + use<N, K> {
        let arms = *self;
        models().filter(move |model| arms.joint_limits_of(&model.description()).is_some())
    }

    /// Each arm's joint limits, `[lower, upper]` per joint, in place order, when the arms
    /// of `description` are these: none when they are not.
    fn joint_limits_of(&self, description: &RobotDescription) -> Option<PerArm<[[f64; 2]; N], K>> {
        let described = description.arms();
        if described.len() != K {
            return None;
        }
        PerArm::from_fn(|arm| {
            let (name, gripper) = self.limbs[arm.place()];
            let of = described.get(arm.place())?;
            if of.name != name || of.gripper.name != gripper {
                return None;
            }
            of.limits.as_slice().try_into().ok()
        })
        .transpose()
    }

    /// Each arm's shape, in place order.
    fn shapes(&self) -> [ArmShape<'static>; K] {
        self.limbs.map(|(arm, gripper)| ArmShape {
            arm,
            joints: N,
            gripper,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::states;

    /// The description of a robot of an arm for each count of `joints`, each arm of that
    /// many revolute joints from its base to its grasp point, with a gripper that moves no
    /// finger.
    fn robot_of_arms(joints: &[usize]) -> RobotDescription {
        let mut urdf = String::from(r#"<robot name="arms"><link name="root"/>"#);
        let mut srdf = String::from(r#"<robot name="arms">"#);
        for (arm, &count) in joints.iter().enumerate() {
            urdf += &format!(
                r#"<link name="a{arm}_l0"/><link name="a{arm}_grasp"/>
                   <joint name="a{arm}_mount" type="fixed">
                     <parent link="root"/><child link="a{arm}_l0"/>
                   </joint>
                   <joint name="a{arm}_tip" type="fixed">
                     <parent link="a{arm}_l{count}"/><child link="a{arm}_grasp"/>
                   </joint>"#
            );
            for joint in 1..=count {
                urdf += &format!(
                    r#"<link name="a{arm}_l{joint}"/>
                       <joint name="a{arm}_j{joint}" type="revolute">
                         <parent link="a{arm}_l{}"/><child link="a{arm}_l{joint}"/>
                         <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
                       </joint>"#,
                    joint - 1
                );
            }
            srdf += &format!(
                r#"<group name="arm{arm}"><chain base_link="a{arm}_l0" tip_link="a{arm}_grasp"/></group>
                   <group name="gripper{arm}"/>
                   <end_effector name="hand{arm}" parent_link="a{arm}_grasp" group="gripper{arm}"/>"#
            );
        }
        urdf += "</robot>";
        srdf += "</robot>";
        RobotDescription::parse(&urdf, &srdf).expect("the robot of arms parses")
    }

    /// The shape `on_shape` runs at: its joints per arm, its joints in all and its arms.
    struct Counts;

    impl OnShape for Counts {
        type Output = [usize; 3];

        fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> [usize; 3] {
            [N, D, K]
        }
    }

    // Each shape of the counts the refusal names runs at its own counts, its joints in all
    // the arms' joints times the arms; any other count is refused.
    #[test]
    fn each_shape_the_refusal_names_runs_at_its_counts() {
        for joints in ARM_JOINTS {
            for arms in ARM_COUNTS {
                assert_eq!(
                    on_counts(joints, arms, Counts),
                    Ok([joints, joints * arms, arms])
                );
            }
        }
        let fewest = ARM_JOINTS[0] - 1;
        let most = ARM_JOINTS[ARM_JOINTS.len() - 1] + 1;
        let arms = ARM_COUNTS[ARM_COUNTS.len() - 1] + 1;
        for (joints, arms) in [(fewest, 1), (most, 1), (ARM_JOINTS[0], arms)] {
            assert_eq!(
                on_counts(joints, arms, Counts),
                Err(UnsupportedShape::Uncompiled { joints, arms })
            );
        }
    }

    // A description runs at the shape of its arms; one of no arm, of arms of two joint
    // counts or of a shape no code is compiled for is refused, naming why.
    #[test]
    fn a_description_runs_at_the_shape_of_its_arms() {
        assert_eq!(on_shape(&robot_of_arms(&[6, 6]), Counts), Ok([6, 12, 2]));
        assert_eq!(
            on_shape(&robot_of_arms(&[]), Counts),
            Err(UnsupportedShape::NoArm)
        );
        assert_eq!(
            on_shape(&robot_of_arms(&[7, 6]), Counts)
                .unwrap_err()
                .to_string(),
            "the arms differ in joint count: arm0 has 7 joints and arm1 has 6; the code is \
             compiled for arms of one joint count"
        );
        assert_eq!(
            on_shape(&robot_of_arms(&[5, 5, 5]), Counts)
                .unwrap_err()
                .to_string(),
            "the robot has 3 arms of 5 joints, and the code is compiled for 1 or 2 arms of 5, \
             6 or 7 joints"
        );
    }

    /// A name that lives as long as the test process, as a node's table holds it.
    fn lasting(name: &str) -> &'static str {
        String::leak(name.to_owned())
    }

    /// The check of a bundled model against the arms of a node compiled for its own.
    struct DrivesANodeOfItsArms(&'static Model);

    impl OnShape for DrivesANodeOfItsArms {
        type Output = ();

        fn on_shape<const N: usize, const D: usize, const K: usize>(self) {
            let model = self.0;
            let description = model.description();
            let described = description.arms();
            let arms = NodeArms::<N, K>::new(std::array::from_fn(|place| {
                let arm = &described[place];
                (lasting(&arm.name), lasting(&arm.gripper.name))
            }));
            let id = model.id();
            let driven = arms.model(id).expect("its own arms");
            assert_eq!(driven.model().id(), id);
            assert_eq!(driven.description().urdf(), description.urdf(), "{id}");
            assert!(arms.models().any(|fitting| fitting.id() == id), "{id}");
            let limits = driven.joint_limits();
            let ready = driven.state(states::READY).expect("each arm's ready");
            for arm in PerArm::<(), K>::ids() {
                let of = &described[arm.place()];
                assert_eq!(arms.arm_name(arm), of.name, "{id}");
                assert_eq!(arms.gripper_name(arm), of.gripper.name, "{id}");
                assert_eq!(arms.arm_names()[arm.place()], of.name, "{id}");
                assert_eq!(arms.gripper_names()[arm.place()], of.gripper.name, "{id}");
                assert_eq!(limits[arm].as_slice(), of.limits, "{id}");
                assert_eq!(
                    Some(ready[arm].as_slice()),
                    description.state(&of.name, states::READY),
                    "{id}"
                );
            }
            assert_eq!(
                driven.state("no_such_state"),
                Err(arms.arm_name(ArmId::new(0))),
                "the first arm with no such state, {id}"
            );
            let written = |names: &[(&str, &str)]| {
                let arms: Vec<String> = names
                    .iter()
                    .map(|(arm, gripper)| format!("{arm} of {N} joints with {gripper}"))
                    .collect();
                arms.join(" and ")
            };
            let mut renamed = arms.limbs;
            renamed[0].0 = "no_such_arm";
            let refused = NodeArms::<N, K>::new(renamed)
                .model(id)
                .expect_err("a node of another arm")
                .to_string();
            assert_eq!(
                refused,
                format!(
                    "model {id} describes {}, and this node drives {}; the models of that \
                     shape are none",
                    written(&arms.limbs),
                    written(&renamed)
                )
            );
            assert!(matches!(
                arms.model("no_such_model"),
                Err(UndrivenModel::Unknown(_))
            ));
        }
    }

    // Each bundled model drives a node compiled for its own arms, which names its arms and
    // grippers and reads their limits and states as the description gives them, and
    // refuses a model of other arms, naming both and the models that fit.
    #[test]
    fn each_bundled_model_drives_a_node_of_its_own_arms() {
        for model in models() {
            on_shape(&model.description(), DrivesANodeOfItsArms(model))
                .unwrap_or_else(|error| panic!("{}: {error}", model.id()));
        }
    }
}
