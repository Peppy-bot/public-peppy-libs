//! OpenArm v2 as the map's intake cuts it out of a depth frame: both arms from their base
//! links to their grasp points, the body fixed around them, and the self model, which
//! hangs the grippers' fingers at their openings. The intake test and the example
//! `scenes_from_episodes` share it.
#![allow(dead_code)]

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::{BodyGeometry, ChainGeometry, Padding, Rider};
use motion_planning_core::{PosedChain, RobotMask};
use openarm_description::{ARM_DOF, ArmPair, HardwareVersion, Side};
use self_collision_model::SelfCollisionModel;

pub struct Robot {
    pub arms: ArmPair<ChainGeometry<ARM_DOF>>,
    pub body: BodyGeometry,
    pub self_model: SelfCollisionModel,
}

impl Robot {
    /// OpenArm v2, its collision meshes in the directory `meshes`.
    pub fn new(meshes: &str) -> Self {
        let version = HardwareVersion::V2;
        let arm = |side: Side| {
            ChainGeometry::from_urdf(
                version.urdf(),
                meshes,
                &ChainSpec {
                    base_link: Some(version.base_link(side)),
                    tip_link: version.tcp_link(side),
                    joints: JointSelection::PathOrder,
                },
            )
            .expect("the arm fits")
        };
        Self {
            arms: ArmPair::from_fn(arm),
            body: BodyGeometry::from_urdf(
                version.urdf(),
                meshes,
                &[(version.torso_body(), version.torso_regions())],
            )
            .expect("the body fits"),
            self_model: self_collision_model::openarm(version, meshes).expect("the self model"),
        }
    }

    /// The fingers of both arms, as riders of their links, where `model` hangs them.
    pub fn fingers(&self, model: &SelfCollisionModel) -> ArmPair<Vec<Rider<'_>>> {
        ArmPair::from_fn(|side| {
            model
                .fingers(side, self.arms[side].carried())
                .expect("each arm's chain carries its own fingers")
        })
    }

    /// The robot with its arms at `arms`, carrying `fingers`, grown by `padding`.
    pub fn mask<'a>(
        &'a self,
        arms: &ArmPair<[f64; ARM_DOF]>,
        fingers: &'a ArmPair<Vec<Rider<'a>>>,
        padding: Padding,
    ) -> RobotMask<'a, ARM_DOF> {
        let chains: Vec<PosedChain<'a, ARM_DOF>> = Side::BOTH
            .into_iter()
            .map(|side| PosedChain {
                geometry: &self.arms[side],
                joints: arms[side],
                riders: &fingers[side],
            })
            .collect();
        RobotMask::new(&chains, &self.body, padding)
    }
}
