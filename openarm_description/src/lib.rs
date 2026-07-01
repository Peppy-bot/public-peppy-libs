//! The OpenArm v1.0 robot description: the single embedded source of truth for the
//! URDF and its collision meshes, so nodes no longer each ship their own copy.
//!
//! - [`urdf`] returns the bundled URDF string (mechanical joint limits, as vendored).
//! - [`write_meshes_to`] (feature `meshes`) materializes the embedded collision meshes
//!   for the file-based bimanual collision builder.
//! - [`ELBOW_SINGULARITY_FLOOR_RAD`] / [`ELBOW_JOINT_INDEX`] describe the elbow control
//!   margin the kinematics consumer applies (see the constant's docs).
//!
//! Pure data: this crate carries no kinematics or solver dependency. A consumer that
//! wants a kinematic model builds it from [`urdf`] and applies the margin itself (e.g.
//! `srs_model::Arm::from_urdf(urdf(), base).with_lower_floor(ELBOW_JOINT_INDEX,
//! ELBOW_SINGULARITY_FLOOR_RAD)`), so the description stays reusable by any consumer
//! (a viz tool, a sim bridge) without pulling a solver in.

/// The bundled OpenArm v1.0 URDF. Mechanical joint limits are as vendored upstream;
/// the elbow singularity margin lives in [`ELBOW_SINGULARITY_FLOOR_RAD`], not here.
pub fn urdf() -> &'static str {
    include_str!("../assets/openarm_v10.urdf")
}

/// Lower bound (rad) a kinematics consumer should impose on the elbow (j4) beyond its
/// mechanical `0.0`. At full extension the arm is at the straight-arm singularity,
/// where a closed-form arm-angle IK is undefined; this floor holds the redundancy
/// reference off it. It is a control margin, not a mechanical limit, so it lives here
/// as a constant rather than in the URDF; apply it to the built model, e.g. via
/// `srs_model::Arm::with_lower_floor([`ELBOW_JOINT_INDEX`], ELBOW_SINGULARITY_FLOOR_RAD)`.
pub const ELBOW_SINGULARITY_FLOOR_RAD: f64 = 0.05;

/// Index of the elbow joint (j4) in the 0-based `j1..j7` joint vector, for applying
/// [`ELBOW_SINGULARITY_FLOOR_RAD`].
pub const ELBOW_JOINT_INDEX: usize = 3;

/// The bundled collision meshes as `(file name, bytes)`. The bimanual collision
/// builder resolves the URDF's `package://` mesh refs by file name against a meshes
/// directory, so [`write_meshes_to`] lays these down under their bare names.
#[cfg(feature = "meshes")]
const MESHES: &[(&str, &[u8])] = &[
    (
        "body_link0_symp.stl",
        include_bytes!("../assets/meshes/body_link0_symp.stl"),
    ),
    ("finger.stl", include_bytes!("../assets/meshes/finger.stl")),
    ("hand.stl", include_bytes!("../assets/meshes/hand.stl")),
    (
        "link0_symp.stl",
        include_bytes!("../assets/meshes/link0_symp.stl"),
    ),
    (
        "link1_symp.stl",
        include_bytes!("../assets/meshes/link1_symp.stl"),
    ),
    (
        "link2_symp.stl",
        include_bytes!("../assets/meshes/link2_symp.stl"),
    ),
    (
        "link3_symp.stl",
        include_bytes!("../assets/meshes/link3_symp.stl"),
    ),
    (
        "link4_symp.stl",
        include_bytes!("../assets/meshes/link4_symp.stl"),
    ),
    (
        "link5_symp.stl",
        include_bytes!("../assets/meshes/link5_symp.stl"),
    ),
    (
        "link6_symp.stl",
        include_bytes!("../assets/meshes/link6_symp.stl"),
    ),
    (
        "link7_symp.stl",
        include_bytes!("../assets/meshes/link7_symp.stl"),
    ),
];

/// Write the embedded collision meshes into `dir` (created if absent), returning the
/// directory to pass as the collision model's `meshes_dir`. Materializing them lets
/// the existing file-based collision builder consume assets that travel with this
/// crate instead of a per-node vendored copy.
#[cfg(feature = "meshes")]
pub fn write_meshes_to(dir: &std::path::Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    for (name, bytes) in MESHES {
        std::fs::write(dir.join(name), bytes)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parsed() -> urdf_rs::Robot {
        urdf_rs::read_from_string(urdf()).expect("bundled URDF parses")
    }

    #[test]
    fn bundled_urdf_has_both_chain_bases() {
        let robot = parsed();
        for base in ["openarm_left_link0", "openarm_right_link0"] {
            assert!(
                robot.links.iter().any(|l| l.name == base),
                "missing base link {base}"
            );
        }
    }

    #[test]
    fn bundled_urdf_keeps_the_mechanical_elbow_limit() {
        // The file itself carries the vendored 0.0; the singularity margin is a
        // control policy applied by the consumer (ELBOW_SINGULARITY_FLOOR_RAD), not
        // baked into the data.
        let robot = parsed();
        for elbow in ["openarm_left_joint4", "openarm_right_joint4"] {
            let joint = robot
                .joints
                .iter()
                .find(|j| j.name == elbow)
                .unwrap_or_else(|| panic!("missing elbow joint {elbow}"));
            assert_eq!(joint.limit.lower, 0.0, "{elbow} lower limit is mechanical");
        }
    }

    #[cfg(feature = "meshes")]
    #[test]
    fn write_meshes_to_lays_down_every_mesh() {
        let dir = std::env::temp_dir().join("openarm_description_meshes_test");
        let _ = std::fs::remove_dir_all(&dir);
        write_meshes_to(&dir).expect("materialize meshes");
        for (name, _) in MESHES {
            assert!(dir.join(name).is_file(), "missing materialized mesh {name}");
        }
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }
}
