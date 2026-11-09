//! OpenArm's self-collision model: both arms of a hardware generation and its torso,
//! the torso decomposed into the clip regions `openarm_description` holds for it.

use chain_kinematics::{ChainSpec, JointSelection};
use openarm_description::{ARM_DOF, ARMS, HardwareVersion, Side};

use crate::{CollisionError, Finger, PairSpec, SelfCollisionModel};

/// OpenArm's model: a chain per arm, the left's first, in [`Side::BOTH`]'s order
/// ([`Side::index`]).
pub type OpenArmModel = SelfCollisionModel<ARM_DOF, ARMS>;

/// The self-collision model of OpenArm `version`, from its embedded URDF and the
/// collision meshes written to `meshes_dir`: each arm's chain from its base link to its
/// grasp point, both grippers' fingers, the torso's clip regions, and the generation's
/// yoked joints, leaving out the pairs its joint limits keep apart
/// (`HardwareVersion::never_touching_pairs`).
pub fn openarm(version: HardwareVersion, meshes_dir: &str) -> Result<OpenArmModel, CollisionError> {
    let chains = Side::BOTH.map(|side| ChainSpec {
        base_link: Some(version.base_link(side)),
        tip_link: version.tcp_link(side),
        joints: JointSelection::PathOrder,
    });
    let never_touching: Vec<PairSpec> = version
        .never_touching_pairs()
        .iter()
        .map(|[a, b]| PairSpec::new(*a, *b))
        .collect();
    SelfCollisionModel::builder(version.urdf(), meshes_dir, chains, version.yoked_joints())
        .fingers(&openarm_fingers(version))
        .regions(version.torso_body(), version.torso_regions())
        .exclude(&never_touching)
        .build()
}

/// The fingers of both grippers of OpenArm `version`, left then right, each shut where the
/// generation shuts it.
pub fn openarm_fingers(version: HardwareVersion) -> Vec<Finger<'static>> {
    Side::BOTH
        .into_iter()
        .flat_map(|side| version.finger_links(side))
        .map(|link| Finger {
            link,
            shut: version.finger_shut(),
        })
        .collect()
}
