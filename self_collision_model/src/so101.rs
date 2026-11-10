//! SO-101's self-collision model: its one arm, standing on its base, and the jaw.

use chain_kinematics::{ChainSpec, JointSelection};
use so101_description::{
    ARM_DOF, BASE_LINK, JAW_LINK, NEVER_TOUCHING_PAIRS, TCP_LINK, URDF, YOKED_JOINTS, jaw_shut_rad,
};

use crate::{CollisionError, Finger, PairSpec, SelfCollisionModel};

/// SO-101's model: one chain.
pub type So101Model = SelfCollisionModel<ARM_DOF, 1>;

/// The self-collision model of SO-101, from its embedded URDF and the collision meshes
/// written to `meshes_dir` (`so101_description::write_meshes_to`): the arm's chain from
/// its base to its grasp point, the jaw, and the arm's yoked joints, leaving out the pairs
/// its joint limits keep apart (`so101_description::NEVER_TOUCHING_PAIRS`).
pub fn so101(meshes_dir: &str) -> Result<So101Model, CollisionError> {
    let chains = [ChainSpec {
        base_link: Some(BASE_LINK),
        tip_link: TCP_LINK,
        joints: JointSelection::PathOrder,
    }];
    let never_touching: Vec<PairSpec> = NEVER_TOUCHING_PAIRS
        .iter()
        .map(|[a, b]| PairSpec::new(*a, *b))
        .collect();
    SelfCollisionModel::builder(URDF, meshes_dir, chains, YOKED_JOINTS)
        .fingers(&[Finger {
            link: JAW_LINK,
            shut: jaw_shut_rad(),
        }])
        .exclude(&never_touching)
        .build()
}
