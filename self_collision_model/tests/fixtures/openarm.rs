//! The OpenArm fixture shared by the integration tests (included via `#[path]`): the
//! generation of `openarm_description` that the fixture URDF
//! describes, whose torso body and regions a caller reads from it, and the collision
//! meshes of that generation, which the fixture URDF names.

use openarm_description::HardwareVersion;

/// The generation that the fixture URDF describes.
pub const VERSION: HardwareVersion = HardwareVersion::V1;

/// The collision meshes of OpenArm v1.0, which the fixture URDF names.
pub const MESHES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../openarm_description/assets/meshes"
);
