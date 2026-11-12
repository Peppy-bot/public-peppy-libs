//! OpenArm v2 as robot_description bundles it: its description, its two arms, left then
//! right, each arm's collision pieces from its base link to its grasp point, and the robot
//! as the map's intake cuts it out of a depth frame: both arms, the body fixed around them,
//! and the self model, which hangs the grippers' fingers at their openings. The tests and
//! the example `scenes_from_episodes` share it.
#![allow(dead_code)]

use std::sync::LazyLock;

use chain_kinematics::{ChainSpec, JointSelection, Tree};
use collision_geometry::chain::{BodyGeometry, ChainGeometry, Padding, Rider};
use motion_planning_core::{PosedChain, RobotMask};
use robot_description::{Arm, ArmId, Model, PerArm, RobotDescription};
use self_collision_model::SelfCollisionModel;

/// The joints of each arm.
pub const JOINTS: usize = 7;
/// The arms.
pub const ARMS: usize = 2;
/// The left arm and the right, the first and the second of the description's arms.
pub const LEFT: ArmId = ArmId::new(0);
pub const RIGHT: ArmId = ArmId::new(1);

/// OpenArm v2's self model: both arms, left then right.
pub type SelfModel = SelfCollisionModel<JOINTS, ARMS>;

/// OpenArm v2's model in robot_description.
pub fn model() -> &'static Model {
    robot_description::model("openarm_v2").expect("robot_description bundles OpenArm v2")
}

/// OpenArm v2's description: its arms `left_arm` then `right_arm`, each of `JOINTS`
/// joints.
pub static DESCRIPTION: LazyLock<RobotDescription> = LazyLock::new(|| {
    let description = model().description();
    let names: Vec<&str> = description
        .arms()
        .iter()
        .map(|arm| arm.name.as_str())
        .collect();
    assert_eq!(names, ["left_arm", "right_arm"], "OpenArm v2's arms");
    for arm in description.arms() {
        assert_eq!(arm.joints.len(), JOINTS, "the joints of {}", arm.name);
    }
    description
});

/// The URDF's root link, the frame the robot stands in.
pub fn root_link() -> &'static str {
    static ROOT: LazyLock<String> = LazyLock::new(|| {
        let tree = Tree::from_robot(DESCRIPTION.robot()).expect("the URDF is one tree");
        tree.link(tree.root()).name.clone()
    });
    &ROOT
}

/// The arm `id`.
pub fn arm(id: ArmId) -> &'static Arm {
    &DESCRIPTION.arms()[id.place()]
}

/// The chain of `arm`, from its base link to its grasp point.
pub fn chain_spec(arm: &Arm) -> ChainSpec<'_> {
    ChainSpec {
        base_link: Some(&arm.base_link),
        tip_link: &arm.grasp_link,
        joints: JointSelection::PathOrder,
    }
}

/// The collision pieces of the arm `id`, from its base link to its grasp point, its
/// collision meshes in the directory `meshes`.
pub fn arm_geometry(id: ArmId, meshes: &str) -> ChainGeometry<JOINTS> {
    ChainGeometry::from_urdf(DESCRIPTION.urdf(), meshes, &chain_spec(arm(id)))
        .expect("the arm fits")
}

/// Each arm's posture `name`, as the description gives it.
pub fn posture(name: &str) -> PerArm<[f64; JOINTS], ARMS> {
    PerArm::from_fn(|id| {
        let group = &arm(id).name;
        DESCRIPTION
            .state(group, name)
            .unwrap_or_else(|| panic!("{group} has the posture {name}"))
            .try_into()
            .expect("a position for each joint of the arm")
    })
}

pub struct Robot {
    pub arms: PerArm<ChainGeometry<JOINTS>, ARMS>,
    pub body: BodyGeometry,
    pub self_model: SelfModel,
}

impl Robot {
    /// OpenArm v2, its collision meshes in the directory `meshes`.
    pub fn new(meshes: &str) -> Self {
        Self {
            arms: PerArm::from_fn(|id| arm_geometry(id, meshes)),
            body: BodyGeometry::from_urdf(DESCRIPTION.urdf(), meshes).expect("the body fits"),
            self_model: SelfCollisionModel::from_description(&DESCRIPTION, meshes)
                .expect("the self model"),
        }
    }

    /// The fingers of both arms, as riders of their links, where `model` hangs them.
    pub fn fingers(&self, model: &SelfModel) -> PerArm<Vec<Rider<'_>>, ARMS> {
        PerArm::from_fn(|id| {
            model
                .fingers(id.place(), self.arms[id].carried())
                .expect("each arm's chain carries its own fingers")
        })
    }

    /// The robot with its arms at `arms`, carrying `fingers`, grown by `padding`.
    pub fn mask<'a>(
        &'a self,
        arms: &PerArm<[f64; JOINTS], ARMS>,
        fingers: &'a PerArm<Vec<Rider<'a>>, ARMS>,
        padding: Padding,
    ) -> RobotMask<'a, JOINTS> {
        let chains: Vec<PosedChain<'a, JOINTS>> = PerArm::<(), ARMS>::ids()
            .map(|id| PosedChain {
                geometry: &self.arms[id],
                joints: arms[id],
                riders: &fingers[id],
            })
            .collect();
        RobotMask::new(&chains, &self.body, padding)
    }
}
