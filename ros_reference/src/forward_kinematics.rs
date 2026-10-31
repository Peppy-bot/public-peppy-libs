//! Forward kinematics. The reference is MoveIt's robot state: posed at each
//! configuration of a chain, it gives the global transform of each link that the chain
//! moves.

use crate::{
    Bits, Kind, read_input, read_reference, read_scenario, repository_root, scenario_names,
};
use serde::Deserialize;

/// The `kind` of a forward kinematics scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "forward_kinematics")]
    ForwardKinematics,
}

/// A robot and the chains of it to pose.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    urdf: String,
    chains: Vec<Chain>,
}

/// A serial chain from the URDF root to `tip_link`. `joints` names every movable joint
/// between them, in the order of a configuration.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Chain {
    name: String,
    joints: Vec<String>,
    tip_link: String,
    configurations: Vec<Vec<f64>>,
}

/// For each chain of the scenario, in its order, and each of its configurations: the
/// global transform of the child link of each joint, in the order of the joints, and
/// then of the tip link.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    chains: Vec<ChainPoses>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ChainPoses {
    name: String,
    configurations: Vec<Vec<LinkPose>>,
}

/// The transform of `link` in the frame of the URDF root.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LinkPose {
    pub link: String,
    pub translation: [Bits; 3],
    /// The rotation matrix, row by row.
    pub rotation: [Bits; 9],
}

/// A forward kinematics scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    pub chains: Vec<ChainCase>,
}

/// A serial chain from the URDF root to `tip_link`, driven by `joints` in that order.
pub struct ChainCase {
    pub name: String,
    pub joints: Vec<String>,
    pub tip_link: String,
    pub configurations: Vec<Configuration>,
}

/// One configuration of a chain and MoveIt's poses at it.
pub struct Configuration {
    /// One position per joint of the chain, in its order.
    pub positions: Vec<f64>,
    /// The pose of the child link of each joint, in the order of the joints.
    pub links: Vec<LinkPose>,
    /// The pose of the tip link.
    pub tip: LinkPose,
}

impl Cases {
    /// Every forward kinematics scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: one output per chain and per
    /// configuration, holding one pose per joint and then the pose of the tip link.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ForwardKinematics)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::ForwardKinematics,
            urdf,
            chains,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::ForwardKinematics,
            chains: outputs,
        } = read_reference(&name);
        assert_eq!(chains.len(), outputs.len(), "{name}: one output per chain");
        let urdf_path = repository_root().join(&urdf);
        let urdf = String::from_utf8(read_input(&urdf_path)).unwrap_or_else(|e| {
            panic!("{name}: the URDF {} is not UTF-8: {e}", urdf_path.display())
        });
        let chains = chains
            .into_iter()
            .zip(outputs)
            .map(|(chain, output)| pair(&name, chain, output))
            .collect();
        Cases { name, urdf, chains }
    }
}

fn pair(name: &str, chain: Chain, output: ChainPoses) -> ChainCase {
    let context = format!("{name} {}", chain.name);
    assert_eq!(
        chain.name, output.name,
        "{context}: the chains are in one order"
    );
    assert_eq!(
        chain.configurations.len(),
        output.configurations.len(),
        "{context}: one output per configuration"
    );
    let configurations = chain
        .configurations
        .into_iter()
        .zip(output.configurations)
        .enumerate()
        .map(|(index, (positions, mut links))| {
            assert_eq!(
                positions.len(),
                chain.joints.len(),
                "{context} configuration {index}: one position per joint"
            );
            assert_eq!(
                links.len(),
                chain.joints.len() + 1,
                "{context} configuration {index}: one pose per joint and the tip"
            );
            let tip = links.pop().expect("the output holds the tip");
            assert_eq!(
                tip.link, chain.tip_link,
                "{context} configuration {index}: the last pose is the tip"
            );
            Configuration {
                positions,
                links,
                tip,
            }
        })
        .collect();
    ChainCase {
        name: chain.name,
        joints: chain.joints,
        tip_link: chain.tip_link,
        configurations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "inverse_kinematics", "urdf": "a.urdf", "chains": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
        let text = r#"{"kind": "forward_kinematics", "urdf": "a.urdf", "chains": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_ok());
    }

    #[test]
    fn a_field_the_scenario_does_not_name_is_refused() {
        let text = r#"{"kind": "forward_kinematics", "urdf": "a.urdf", "chains": [], "seed": 1}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
        let text = r#"{"kind": "forward_kinematics", "urdf": "a.urdf", "chains": [
            {"name": "a", "joints": [], "tip_link": "t", "configurations": [], "seed": 1}
        ]}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    #[should_panic(expected = "one pose per joint and the tip")]
    fn an_output_without_the_tip_does_not_pair() {
        let chain = Chain {
            name: "arm".into(),
            joints: vec!["j1".into()],
            tip_link: "tip".into(),
            configurations: vec![vec![0.0]],
        };
        let link = LinkPose {
            link: "l1".into(),
            translation: [Bits(0.0); 3],
            rotation: [Bits(0.0); 9],
        };
        let output = ChainPoses {
            name: "arm".into(),
            configurations: vec![vec![link]],
        };
        pair("scenario", chain, output);
    }

    #[test]
    fn every_forward_kinematics_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a forward kinematics scenario"
        );
        for cases in scenarios {
            assert!(!cases.chains.is_empty(), "{} names a chain", cases.name);
            for chain in &cases.chains {
                assert!(
                    !chain.configurations.is_empty(),
                    "{} {} holds a configuration",
                    cases.name,
                    chain.name
                );
            }
        }
    }
}
