//! Forward kinematics. The reference is MoveIt's robot state: posed at each
//! configuration of an arm of the scenario's robot, it gives the global transform of each
//! link that the arm's joints move and of the arm's grasp link.

use crate::{Kind, LinkPose, Robot, read_reference, read_robot, read_scenario, scenario_names};
use robot_description::Arm;
use serde::Deserialize;

/// The `kind` of a forward kinematics scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "forward_kinematics")]
    ForwardKinematics,
}

/// A robot and the arms of it to pose.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The robot's model directory, relative to the repository root.
    description: String,
    arms: Vec<ArmConfigurations>,
}

/// The arm `name` of the description, and the configurations of its joints to pose it at,
/// each in the order of the arm's joints.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ArmConfigurations {
    name: String,
    configurations: Vec<Vec<f64>>,
}

/// For each arm of the scenario, in its order, and each of its configurations: the global
/// transform of the child link of each joint of the arm, in the order of the joints, and
/// then of its grasp link.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    arms: Vec<ArmPoses>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ArmPoses {
    name: String,
    configurations: Vec<Vec<LinkPose>>,
}

/// A forward kinematics scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub robot: Robot,
    pub arms: Vec<ArmCase>,
}

/// An arm of the robot, the chain from the URDF root to its grasp link that its joints
/// drive in their order, and MoveIt's poses at each of its configurations.
pub struct ArmCase {
    pub arm: Arm,
    pub configurations: Vec<Configuration>,
}

/// One configuration of an arm and MoveIt's poses at it.
pub struct Configuration {
    /// One position per joint of the arm, in its order.
    pub positions: Vec<f64>,
    /// The pose of the child link of each joint, in the order of the joints.
    pub links: Vec<LinkPose>,
    /// The pose of the grasp link.
    pub tip: LinkPose,
}

impl Cases {
    /// Every forward kinematics scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: one output per arm and per configuration,
    /// holding one pose per joint and then the pose of the grasp link.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ForwardKinematics)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::ForwardKinematics,
            description,
            arms,
        } = read_scenario(&name);
        let robot = read_robot(&name, &description);
        let Reference {
            kind: ThisKind::ForwardKinematics,
            arms: outputs,
        } = read_reference(&name);
        assert_eq!(arms.len(), outputs.len(), "{name}: one output per arm");
        let arms = arms
            .into_iter()
            .zip(outputs)
            .map(|(posed, output)| {
                let arm = robot.arm(&name, &posed.name);
                pair(&name, arm, posed, output)
            })
            .collect();
        Cases { name, robot, arms }
    }
}

fn pair(name: &str, arm: Arm, posed: ArmConfigurations, output: ArmPoses) -> ArmCase {
    let context = format!("{name} {}", arm.name);
    assert_eq!(
        arm.name, output.name,
        "{context}: the arms are in one order"
    );
    assert_eq!(
        posed.configurations.len(),
        output.configurations.len(),
        "{context}: one output per configuration"
    );
    let configurations = posed
        .configurations
        .into_iter()
        .zip(output.configurations)
        .enumerate()
        .map(|(index, (positions, mut links))| {
            assert_eq!(
                positions.len(),
                arm.joints.len(),
                "{context} configuration {index}: one position per joint"
            );
            assert_eq!(
                links.len(),
                arm.joints.len() + 1,
                "{context} configuration {index}: one pose per joint and the grasp link"
            );
            let tip = links.pop().expect("the output holds the grasp link");
            assert_eq!(
                tip.link, arm.grasp_link,
                "{context} configuration {index}: the last pose is the grasp link's"
            );
            Configuration {
                positions,
                links,
                tip,
            }
        })
        .collect();
    ArmCase {
        arm,
        configurations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "inverse_kinematics", "description": "d", "arms": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
        let text = r#"{"kind": "forward_kinematics", "description": "d", "arms": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_ok());
    }

    #[test]
    fn a_field_the_scenario_does_not_name_is_refused() {
        let text = r#"{"kind": "forward_kinematics", "description": "d", "arms": [], "seed": 1}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
        let text = r#"{"kind": "forward_kinematics", "description": "d", "arms": [
            {"name": "a", "configurations": [], "seed": 1}
        ]}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    #[should_panic(expected = "one pose per joint and the grasp link")]
    fn an_output_without_the_grasp_link_does_not_pair() {
        let arm = robot_description::model("so101")
            .expect("a bundled model")
            .description()
            .arms()[0]
            .clone();
        let posed = ArmConfigurations {
            name: arm.name.clone(),
            configurations: vec![vec![0.0; arm.joints.len()]],
        };
        let link = LinkPose {
            link: "l1".into(),
            translation: [0.0; 3],
            rotation: [0.0; 9],
        };
        let output = ArmPoses {
            name: arm.name.clone(),
            configurations: vec![vec![link]],
        };
        pair("scenario", arm, posed, output);
    }

    #[test]
    fn every_forward_kinematics_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a forward kinematics scenario"
        );
        for cases in scenarios {
            assert!(!cases.arms.is_empty(), "{} names an arm", cases.name);
            for arm in &cases.arms {
                assert!(
                    !arm.configurations.is_empty(),
                    "{} {} holds a configuration",
                    cases.name,
                    arm.arm.name
                );
            }
        }
    }
}
