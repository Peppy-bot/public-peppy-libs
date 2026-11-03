//! OpenArm's self-collision model: both arms of a hardware generation and its torso,
//! the torso decomposed into the clip regions `openarm_description` holds for it.

use openarm_description::{HardwareVersion, Side};

use crate::{CollisionError, PairSpec, SelfCollisionModel};

/// The self-collision model of OpenArm `version`, from its embedded URDF and the
/// collision meshes written to `meshes_dir`, leaving out the pairs the generation's
/// joint limits keep apart (`HardwareVersion::never_touching_pairs`).
pub fn openarm(
    version: HardwareVersion,
    meshes_dir: &str,
) -> Result<SelfCollisionModel, CollisionError> {
    let never_touching: Vec<PairSpec> = version
        .never_touching_pairs()
        .iter()
        .map(|[a, b]| PairSpec::new(*a, *b))
        .collect();
    SelfCollisionModel::builder(
        version.urdf(),
        meshes_dir,
        version.base_link(Side::Left),
        version.base_link(Side::Right),
    )
    .regions(version.torso_body(), version.torso_regions())
    .exclude(&never_touching)
    .build()
}
