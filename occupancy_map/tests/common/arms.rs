//! OpenArm v2's arms, fitted once, for the suites that place them.

use std::path::Path;
use std::sync::LazyLock;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use openarm_description::{ARM_DOF, ArmPair, HardwareVersion};

/// OpenArm v2's two arms, each the chain from the URDF root to its grasp
/// point, fitted to the collision meshes the description carries.
pub static OPENARM_V2_ARMS: LazyLock<ArmPair<ChainGeometry<ARM_DOF>>> = LazyLock::new(|| {
    let version = HardwareVersion::V2;
    let meshes = Path::new(env!("CARGO_TARGET_TMPDIR")).join("openarm_v2_meshes");
    version.write_meshes_to(&meshes).expect("write the meshes");
    let meshes = meshes.to_str().expect("the mesh directory is UTF-8");
    ArmPair::from_fn(|side| {
        let joints: Vec<String> = (1..=ARM_DOF)
            .map(|joint| format!("openarm_{side}_joint{joint}"))
            .collect();
        let joints: Vec<&str> = joints.iter().map(String::as_str).collect();
        ChainGeometry::from_urdf(
            version.urdf(),
            meshes,
            &ChainSpec {
                base_link: None,
                tip_link: version.tcp_link(side),
                joints: JointSelection::Named(&joints),
            },
        )
        .expect("the arm fits")
    })
});
