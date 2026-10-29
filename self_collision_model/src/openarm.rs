//! OpenArm's self-collision model: both arms of a hardware generation and its torso,
//! the torso decomposed into the clip regions `openarm_description` holds for it.

use openarm_description::{HardwareVersion, Side};

use crate::{CollisionError, SelfCollisionModel};

/// The self-collision model of OpenArm `version`, from its embedded URDF and the
/// collision meshes written to `meshes_dir`.
pub fn openarm(
    version: HardwareVersion,
    meshes_dir: &str,
) -> Result<SelfCollisionModel, CollisionError> {
    SelfCollisionModel::builder(
        version.urdf(),
        meshes_dir,
        version.base_link(Side::Left),
        version.base_link(Side::Right),
    )
    .regions(version.torso_body(), version.torso_regions())
    .build()
}
