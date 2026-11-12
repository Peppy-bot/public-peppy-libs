//! The collision geometry of an arm. The reference is MoveIt: its robot model names the
//! links with collision geometry that the arm carries and the center it grows each of
//! their bodies about, its robot state, posed at each configuration of the arm, gives the
//! global transform of each carried link, and its robot filter, the bodies of those links
//! grown by a padding, tells whether each probe point lies inside them; CHOMP's collision
//! checker gives the collision spheres of each carried link at each of the scenario's
//! sphere paddings.

use crate::{Kind, LinkPose, Robot, read_reference, read_robot, read_scenario, scenario_names};
use robot_description::Arm;
use serde::Deserialize;

/// The `kind` of a chain collision scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "chain_collision")]
    ChainCollision,
}

/// A robot, the paddings to grow its links by, the paddings of their collision spheres,
/// and the arms of it to pose.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The robot's model directory, relative to the repository root.
    description: String,
    paddings: Vec<Padding>,
    /// The paddings, in meters, that CHOMP's collision checker grows each link's bodies
    /// by before it places their collision spheres.
    sphere_paddings: Vec<f64>,
    arms: Vec<ArmPosed>,
}

/// How the robot filter grows each link: MoveIt's padding scale, and its padding offset
/// in meters.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Padding {
    pub scale: f64,
    pub offset: f64,
}

/// The arm `name` of the description, and the configurations to pose it at, each with
/// its positions in the order of the arm's joints.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ArmPosed {
    name: String,
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

/// The paddings and the sphere paddings of the scenario as the reference read them, and
/// for each arm of the scenario, in its order, and each of its configurations: the
/// positions and the probe points as the reference read them, the global transform of
/// each link the arm carries, in the order of the links, and for each padding, in its
/// order, whether each probe point lies inside the grown links.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    paddings: Vec<Padding>,
    sphere_paddings: Vec<f64>,
    arms: Vec<ArmOutput>,
}

/// An arm's output: the links with collision geometry that MoveIt's arm carries, the
/// center of each of their bodies, their collision spheres, and one output per
/// configuration.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ArmOutput {
    name: String,
    links: Vec<String>,
    bodies: Vec<Body>,
    spheres: Vec<Vec<Vec<Sphere>>>,
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

/// A collision sphere of a link, its center in the link's frame.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sphere {
    pub center: [f64; 3],
    pub radius: f64,
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
    pub robot: Robot,
    pub paddings: Vec<Padding>,
    pub sphere_paddings: Vec<f64>,
    pub arms: Vec<ArmCase>,
}

/// An arm of the robot, the chain from the URDF root to its grasp link that its joints
/// drive in their order, which carries `links`, the links with collision geometry that
/// MoveIt's arm carries, whose bodies, in the order of the links and of each link's
/// collision elements, are `bodies`.
pub struct ArmCase {
    pub arm: Arm,
    pub links: Vec<String>,
    pub bodies: Vec<Body>,
    /// For each sphere padding, in its order, the collision spheres of each link, in the
    /// order of the links.
    pub spheres: Vec<Vec<Vec<Sphere>>>,
    pub configurations: Vec<Configuration>,
}

/// One configuration of an arm, its probe points, and MoveIt's answers at it.
pub struct Configuration {
    /// One position per joint of the arm, in its order.
    pub positions: Vec<f64>,
    /// The pose of each link the arm carries, in the order of its links.
    pub links: Vec<LinkPose>,
    pub probes: Vec<Probe>,
    /// For each padding, in its order, whether each probe lies inside the grown links.
    pub inside: Vec<Vec<bool>>,
}

impl Cases {
    /// Every chain collision scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read the scenario's
    /// paddings and sphere paddings, each arm's output names a body of a carried link
    /// with each center, holds the spheres of each link for each sphere padding, and
    /// one output per arm and per configuration, which read its positions and probe
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
            description,
            paddings,
            sphere_paddings,
            arms,
        } = read_scenario(&name);
        let robot = read_robot(&name, &description);
        let Reference {
            kind: ThisKind::ChainCollision,
            paddings: paddings_read,
            sphere_paddings: sphere_paddings_read,
            arms: outputs,
        } = read_reference(&name);
        assert_eq!(
            paddings_read, paddings,
            "{name}: the reference read the paddings"
        );
        assert_eq!(
            sphere_paddings_read, sphere_paddings,
            "{name}: the reference read the sphere paddings"
        );
        assert_eq!(arms.len(), outputs.len(), "{name}: one output per arm");
        let arms = arms
            .into_iter()
            .zip(outputs)
            .map(|(posed, output)| {
                let arm = robot.arm(&name, &posed.name);
                let counts = (paddings.len(), sphere_paddings.len());
                pair(&name, counts, arm, posed, output)
            })
            .collect();
        Cases {
            name,
            robot,
            paddings,
            sphere_paddings,
            arms,
        }
    }
}

/// The arm `arm` of the scenario `name`, posed as `posed`, paired with its output: one
/// answer per padding of the scenario's `paddings` and one set of spheres per sphere
/// padding of its `sphere_paddings`.
fn pair(
    name: &str,
    (paddings, sphere_paddings): (usize, usize),
    arm: Arm,
    posed: ArmPosed,
    output: ArmOutput,
) -> ArmCase {
    let context = format!("{name} {}", arm.name);
    assert_eq!(
        arm.name, output.name,
        "{context}: the arms are in one order"
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
        output.spheres.len(),
        sphere_paddings,
        "{context}: one set of spheres per sphere padding"
    );
    for spheres in &output.spheres {
        assert_eq!(
            spheres.len(),
            links.len(),
            "{context}: the spheres of each carried link"
        );
    }
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
        .map(|(index, (posed, output))| {
            let context = format!("{context} configuration {index}");
            assert_eq!(
                posed.positions.len(),
                arm.joints.len(),
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
    ArmCase {
        arm,
        links,
        bodies: output.bodies,
        spheres: output.spheres,
        configurations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SO-101's arm, and a configuration of it posing one probe.
    fn so101_arm(probe: Probe) -> (Arm, ArmPosed) {
        let arm = robot_description::model("so101")
            .expect("a bundled model")
            .description()
            .arms()[0]
            .clone();
        let posed = ArmPosed {
            name: arm.name.clone(),
            configurations: vec![Posed {
                positions: vec![0.0; arm.joints.len()],
                points: vec![probe],
            }],
        };
        (arm, posed)
    }

    /// An output of `arm` at its configuration of `so101_arm`, which read `probe` and
    /// answers `inside` for each padding.
    fn output_of(arm: &Arm, probe: Probe, inside: Vec<Vec<bool>>) -> ArmOutput {
        ArmOutput {
            name: arm.name.clone(),
            links: vec![],
            bodies: vec![],
            spheres: vec![],
            configurations: vec![PosedOutput {
                positions: vec![0.0; arm.joints.len()],
                points: vec![probe],
                links: vec![],
                inside,
            }],
        }
    }

    fn probe(x: f64) -> Probe {
        Probe {
            link: "l".into(),
            position: [x, 0.0, 0.0],
        }
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let scenario = |kind: &str| {
            format!(
                r#"{{"kind": "{kind}", "description": "d", "paddings": [], "sphere_paddings": [], "arms": []}}"#
            )
        };
        assert!(serde_json::from_str::<Scenario>(&scenario("forward_kinematics")).is_err());
        assert!(serde_json::from_str::<Scenario>(&scenario("chain_collision")).is_ok());
    }

    #[test]
    #[should_panic(expected = "one answer per probe")]
    fn an_output_missing_an_answer_does_not_pair() {
        let (arm, posed) = so101_arm(probe(0.0));
        let output = output_of(&arm, probe(0.0), vec![vec![]]);
        pair("scenario", (1, 0), arm, posed, output);
    }

    #[test]
    #[should_panic(expected = "the reference read the probe points")]
    fn an_output_that_read_another_probe_does_not_pair() {
        let (arm, posed) = so101_arm(probe(0.5));
        let output = output_of(&arm, probe(0.25), vec![vec![true]]);
        pair("scenario", (1, 0), arm, posed, output);
    }

    #[test]
    #[should_panic(expected = "the body of l belongs to a carried link")]
    fn a_body_of_a_link_the_arm_does_not_carry_does_not_pair() {
        let (arm, posed) = so101_arm(probe(0.0));
        let output = ArmOutput {
            bodies: vec![Body {
                link: "l".into(),
                center: [0.0; 3],
            }],
            ..output_of(&arm, probe(0.0), vec![vec![true]])
        };
        pair("scenario", (1, 0), arm, posed, output);
    }

    #[test]
    fn every_chain_collision_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a chain collision scenario"
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
