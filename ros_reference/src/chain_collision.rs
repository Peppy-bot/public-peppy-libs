//! The collision geometry of a chain. The reference is MoveIt: its robot model names the
//! links with collision geometry that the chain carries and the center it grows each of
//! their bodies about, its robot state, posed at each configuration of the chain, gives
//! the global transform of each carried link, and its robot filter, the bodies of those
//! links grown by a padding, tells whether each probe point lies inside them.

use crate::{
    Kind, LinkPose, read_mesh_directory, read_reference, read_scenario, read_urdf, scenario_names,
};
use serde::Deserialize;
use std::path::PathBuf;

/// The `kind` of a chain collision scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "chain_collision")]
    ChainCollision,
}

/// A robot, the paddings to grow its links by, and the chains of it to pose.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    urdf: String,
    /// The directory of the collision meshes, which the URDF names by their file names,
    /// relative to the repository root; absent for a URDF that names no collision mesh.
    meshes: Option<String>,
    paddings: Vec<Padding>,
    chains: Vec<Chain>,
}

/// How the robot filter grows each link: MoveIt's padding scale, and its padding offset
/// in meters.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Padding {
    pub scale: f64,
    pub offset: f64,
}

/// A serial chain from the URDF root to `tip_link`, driven by `joints` in the order of a
/// configuration.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Chain {
    name: String,
    joints: Vec<String>,
    tip_link: String,
    configurations: Vec<Posed>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Posed {
    positions: Vec<f64>,
    points: Vec<Probe>,
}

/// A probe point, in the frame of `link`.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probe {
    pub link: String,
    pub position: [f64; 3],
}

/// The paddings of the scenario as the reference read them, and for each chain of the
/// scenario, in its order, and each of its configurations: the positions and the probe
/// points as the reference read them, the global transform of each link the chain
/// carries, in the order of the links, and for each padding, in its order, whether
/// each probe point lies inside the grown links.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    paddings: Vec<Padding>,
    chains: Vec<ChainOutput>,
}

/// A chain's output: the links with collision geometry that MoveIt's chain carries, the
/// center of each of their bodies, and one output per configuration.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ChainOutput {
    name: String,
    links: Vec<String>,
    bodies: Vec<Body>,
    configurations: Vec<PosedOutput>,
}

/// A body of MoveIt's robot filter: its link, and the point in the link's frame that
/// MoveIt grows it about, the mean of a convex mesh's vertices or a primitive's origin.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub link: String,
    pub center: [f64; 3],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PosedOutput {
    positions: Vec<f64>,
    points: Vec<Probe>,
    links: Vec<LinkPose>,
    inside: Vec<Vec<bool>>,
}

/// A chain collision scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    /// The directory of the collision meshes; `None` for a URDF that names none.
    pub meshes: Option<PathBuf>,
    pub paddings: Vec<Padding>,
    pub chains: Vec<ChainCase>,
}

/// A serial chain from the URDF root to `tip_link`, driven by `joints` in that order, which
/// carries `links`, the links with collision geometry that MoveIt's chain carries, whose
/// bodies, in the order of the links and of each link's collision elements, are `bodies`.
pub struct ChainCase {
    pub name: String,
    pub joints: Vec<String>,
    pub tip_link: String,
    pub links: Vec<String>,
    pub bodies: Vec<Body>,
    pub configurations: Vec<Configuration>,
}

/// One configuration of a chain, its probe points, and MoveIt's answers at it.
pub struct Configuration {
    /// One position per joint of the chain, in its order.
    pub positions: Vec<f64>,
    /// The pose of each link the chain carries, in the order of its links.
    pub links: Vec<LinkPose>,
    pub probes: Vec<Probe>,
    /// For each padding, in its order, whether each probe lies inside the grown links.
    pub inside: Vec<Vec<bool>>,
}

impl Cases {
    /// Every chain collision scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read the scenario's
    /// paddings, each chain's output names a body of a carried link with each center, and
    /// one output per chain and per configuration, which read its positions and probe
    /// points, holds one pose per carried link and one answer per padding and per probe.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::ChainCollision)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::ChainCollision,
            urdf,
            meshes,
            paddings,
            chains,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::ChainCollision,
            paddings: paddings_read,
            chains: outputs,
        } = read_reference(&name);
        assert_eq!(
            paddings_read, paddings,
            "{name}: the reference read the paddings"
        );
        assert_eq!(chains.len(), outputs.len(), "{name}: one output per chain");
        let chains = chains
            .into_iter()
            .zip(outputs)
            .map(|(chain, output)| pair(&name, paddings.len(), chain, output))
            .collect();
        Cases {
            urdf: read_urdf(&name, &urdf),
            meshes: meshes.map(|meshes| read_mesh_directory(&name, &meshes)),
            name,
            paddings,
            chains,
        }
    }
}

fn pair(name: &str, paddings: usize, chain: Chain, output: ChainOutput) -> ChainCase {
    let context = format!("{name} {}", chain.name);
    assert_eq!(
        chain.name, output.name,
        "{context}: the chains are in one order"
    );
    let links = output.links;
    for body in &output.bodies {
        assert!(
            links.contains(&body.link),
            "{context}: the body of {} belongs to a carried link",
            body.link
        );
    }
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
        .map(|(index, (posed, output))| {
            let context = format!("{context} configuration {index}");
            assert_eq!(
                posed.positions.len(),
                chain.joints.len(),
                "{context}: one position per joint"
            );
            assert_eq!(
                output.positions, posed.positions,
                "{context}: the reference read the positions"
            );
            assert_eq!(
                output.points, posed.points,
                "{context}: the reference read the probe points"
            );
            let linked: Vec<&str> = output.links.iter().map(|pose| pose.link.as_str()).collect();
            assert_eq!(linked, links, "{context}: one pose per carried link");
            assert_eq!(
                output.inside.len(),
                paddings,
                "{context}: one answer per padding"
            );
            for answers in &output.inside {
                assert_eq!(
                    answers.len(),
                    posed.points.len(),
                    "{context}: one answer per probe"
                );
            }
            Configuration {
                positions: posed.positions,
                links: output.links,
                probes: posed.points,
                inside: output.inside,
            }
        })
        .collect();
    ChainCase {
        name: chain.name,
        joints: chain.joints,
        tip_link: chain.tip_link,
        links,
        bodies: output.bodies,
        configurations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let scenario = |kind: &str| {
            format!(
                r#"{{"kind": "{kind}", "urdf": "a.urdf", "meshes": "m", "paddings": [], "chains": []}}"#
            )
        };
        assert!(serde_json::from_str::<Scenario>(&scenario("forward_kinematics")).is_err());
        assert!(serde_json::from_str::<Scenario>(&scenario("chain_collision")).is_ok());
    }

    #[test]
    #[should_panic(expected = "one answer per probe")]
    fn an_output_missing_an_answer_does_not_pair() {
        let chain = Chain {
            name: "arm".into(),
            joints: vec![],
            tip_link: "tip".into(),
            configurations: vec![Posed {
                positions: vec![],
                points: vec![Probe {
                    link: "l".into(),
                    position: [0.0; 3],
                }],
            }],
        };
        let output = ChainOutput {
            name: "arm".into(),
            links: vec![],
            bodies: vec![],
            configurations: vec![PosedOutput {
                positions: vec![],
                points: vec![Probe {
                    link: "l".into(),
                    position: [0.0; 3],
                }],
                links: vec![],
                inside: vec![vec![]],
            }],
        };
        pair("scenario", 1, chain, output);
    }

    #[test]
    #[should_panic(expected = "the reference read the probe points")]
    fn an_output_that_read_another_probe_does_not_pair() {
        let probe = |x: f64| Probe {
            link: "l".into(),
            position: [x, 0.0, 0.0],
        };
        let chain = Chain {
            name: "arm".into(),
            joints: vec![],
            tip_link: "tip".into(),
            configurations: vec![Posed {
                positions: vec![],
                points: vec![probe(0.5)],
            }],
        };
        let output = ChainOutput {
            name: "arm".into(),
            links: vec![],
            bodies: vec![],
            configurations: vec![PosedOutput {
                positions: vec![],
                points: vec![probe(0.25)],
                links: vec![],
                inside: vec![vec![true]],
            }],
        };
        pair("scenario", 1, chain, output);
    }

    #[test]
    #[should_panic(expected = "the body of l belongs to a carried link")]
    fn a_body_of_a_link_the_chain_does_not_carry_does_not_pair() {
        let chain = Chain {
            name: "arm".into(),
            joints: vec![],
            tip_link: "tip".into(),
            configurations: vec![],
        };
        let output = ChainOutput {
            name: "arm".into(),
            links: vec![],
            bodies: vec![Body {
                link: "l".into(),
                center: [0.0; 3],
            }],
            configurations: vec![],
        };
        pair("scenario", 1, chain, output);
    }

    #[test]
    fn a_scenario_without_a_mesh_directory_reads() {
        let text = r#"{"kind": "chain_collision", "urdf": "a.urdf", "paddings": [], "chains": []}"#;
        let scenario: Scenario = serde_json::from_str(text).expect("a scenario reads");
        assert!(scenario.meshes.is_none());
    }

    #[test]
    fn every_chain_collision_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a chain collision scenario"
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
