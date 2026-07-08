//! The OpenArm robot descriptions: the single embedded source of truth for the URDFs
//! and their collision meshes, so nodes no longer each ship their own copy.
//!
//! Two hardware generations are carried, selected by [`HardwareVersion`]:
//! - [`HardwareVersion::V1`]: OpenArm v1.0 (`openarm_v10`).
//! - [`HardwareVersion::V2`]: OpenArm v2.0 (`openarm_v20`).
//!
//! - [`HardwareVersion::urdf`] returns the bundled URDF string (mechanical joint limits,
//!   as vendored).
//! - [`HardwareVersion::write_meshes_to`] (feature `meshes`) materializes the embedded
//!   collision meshes for the file-based bimanual collision builder and for any non-Rust
//!   consumer (the sim) via the `emit_meshes` binary.
//! - [`HardwareVersion::elbow_singularity_floor_rad`] / [`HardwareVersion::elbow_joint_index`]
//!   describe the elbow control margin the kinematics consumer applies (see the method docs).
//! - [`HardwareVersion::joint_limits`] resolves one side's per-joint position limits from the
//!   bundled URDF with that margin applied: the single clamp source for every node that
//!   produces joint commands.
//!
//! Pure data: this crate carries no solver dependency. A consumer that wants a kinematic
//! model builds it from the URDF and applies the margin itself, e.g.
//! `srs_model::Arm::from_urdf(v.urdf(), base).with_lower_floor(v.elbow_joint_index(),
//! v.elbow_singularity_floor_rad())`, so the description stays reusable by any consumer (a
//! viz tool, a sim bridge) without pulling a solver in.

use std::fmt;
use std::str::FromStr;

/// Joints per arm (j1..j7) in both generations.
pub const ARM_DOF: usize = 7;

/// An arm side of the bimanual robot, selecting the `openarm_left_*` or
/// `openarm_right_*` chain in the bundled URDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    fn urdf_prefix(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

/// An OpenArm hardware generation. A node parses its `hardware_version` parameter into
/// this once (parse, don't validate) and then reads the bundled description through it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareVersion {
    /// OpenArm v1.0 (`openarm_v10`): prismatic parallel-jaw gripper.
    V1,
    /// OpenArm v2.0 (`openarm_v20`): reoriented arm frames, revolute pinch gripper.
    V2,
}

impl HardwareVersion {
    /// The bundled URDF for this generation. Mechanical joint limits are as vendored
    /// upstream (enactic/openarm_description); the elbow singularity margin lives in
    /// [`Self::elbow_singularity_floor_rad`], not in the file.
    pub fn urdf(self) -> &'static str {
        match self {
            Self::V1 => include_str!("../assets/openarm_v10.urdf"),
            Self::V2 => include_str!("../assets/openarm_v20.urdf"),
        }
    }

    /// Lower bound (rad) a kinematics consumer should impose on the elbow (j4) beyond its
    /// mechanical `0.0`. At full extension the arm is at the straight-arm singularity,
    /// where a closed-form arm-angle IK is undefined; this floor holds the redundancy
    /// reference off it. It is a control margin, not a mechanical limit, so it lives here
    /// rather than in the URDF. Both generations share the value today; returning it per
    /// version keeps a future divergence a data change, not a code change.
    pub fn elbow_singularity_floor_rad(self) -> f64 {
        match self {
            Self::V1 | Self::V2 => 0.05,
        }
    }

    /// Index of the elbow joint (j4) in the 0-based `j1..j7` joint vector, for applying
    /// [`Self::elbow_singularity_floor_rad`]. j4 is the elbow in both generations.
    pub fn elbow_joint_index(self) -> usize {
        match self {
            Self::V1 | Self::V2 => 3,
        }
    }

    /// Full-open jaw width (m) of the generation's gripper; closed is 0. This is
    /// the aperture the shared gripper interfaces carry: measured between the
    /// finger pad faces (the flat gripping surfaces), which is where an object's
    /// fit is decided. It is gripper-linkage data, not a URDF joint limit, so it
    /// lives here. The v2 value is the pad gap at the finger joints' full
    /// travel computed from the URDF pivots and the finger meshes' pad faces,
    /// pending confirmation by one measurement on hardware.
    pub fn jaw_open_m(self) -> f64 {
        match self {
            Self::V1 => 0.044,
            Self::V2 => 0.0697,
        }
    }

    /// Per-joint `[lower, upper]` position limits (rad) for one arm side, j1..j7, from
    /// the bundled URDF with [`Self::elbow_singularity_floor_rad`] applied to the elbow.
    /// This is the clamp range a command-producing node (operator panel, leader arm)
    /// applies before streaming; the hub and the arm clamp again on their side. Panics
    /// only if the bundled URDF is malformed, which this crate's tests rule out.
    pub fn joint_limits(self, side: Side) -> [[f64; 2]; ARM_DOF] {
        let robot = urdf_rs::read_from_string(self.urdf()).expect("bundled URDF must parse");
        let elbow = self.elbow_joint_index();
        let floor = self.elbow_singularity_floor_rad();
        std::array::from_fn(|i| {
            let name = format!("openarm_{}_joint{}", side.urdf_prefix(), i + 1);
            let joint = robot
                .joints
                .iter()
                .find(|j| j.name == name)
                .unwrap_or_else(|| panic!("bundled URDF missing joint {name}"));
            let lower = if i == elbow {
                joint.limit.lower.max(floor)
            } else {
                joint.limit.lower
            };
            [lower, joint.limit.upper]
        })
    }
}

impl fmt::Display for HardwareVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::V1 => "v1",
            Self::V2 => "v2",
        })
    }
}

impl FromStr for HardwareVersion {
    type Err = UnknownHardwareVersion;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "v1" | "V1" => Ok(Self::V1),
            "v2" | "V2" => Ok(Self::V2),
            other => Err(UnknownHardwareVersion(other.to_owned())),
        }
    }
}

/// Returned when a `hardware_version` string is neither `v1` nor `v2`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownHardwareVersion(pub String);

impl fmt::Display for UnknownHardwareVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "unknown hardware_version '{}' (expected 'v1' or 'v2')",
            self.0
        )
    }
}

impl std::error::Error for UnknownHardwareVersion {}

/// The torso mesh, shared by both generations (the bimanual stand is unchanged).
/// Embedded once and referenced from both mesh lists; a test asserts the v2.0
/// asset file stays byte-identical.
#[cfg(feature = "meshes")]
const TORSO_MESH: &[u8] = include_bytes!("../assets/meshes/body_link0_symp.stl");

/// The bundled collision meshes as `(file name, bytes)`. The bimanual collision builder
/// resolves the URDF's `package://` mesh refs by file name against a meshes directory, so
/// [`HardwareVersion::write_meshes_to`] lays these down under their bare names.
#[cfg(feature = "meshes")]
const V1_MESHES: &[(&str, &[u8])] = &[
    ("body_link0_symp.stl", TORSO_MESH),
    ("finger.stl", include_bytes!("../assets/meshes/finger.stl")),
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

/// OpenArm v2.0 collision meshes: reoriented arm links (`base_link`, `link1..link6`) and
/// the revolute pinch gripper (`ee_base_link`, `finger_inner`, `finger_outer`), plus the
/// shared torso proxy mesh.
#[cfg(feature = "meshes")]
const V2_MESHES: &[(&str, &[u8])] = &[
    ("body_link0_symp.stl", TORSO_MESH),
    (
        "base_link.stl",
        include_bytes!("../assets/meshes_v20/base_link.stl"),
    ),
    (
        "link1.stl",
        include_bytes!("../assets/meshes_v20/link1.stl"),
    ),
    (
        "link2.stl",
        include_bytes!("../assets/meshes_v20/link2.stl"),
    ),
    (
        "link3.stl",
        include_bytes!("../assets/meshes_v20/link3.stl"),
    ),
    (
        "link4.stl",
        include_bytes!("../assets/meshes_v20/link4.stl"),
    ),
    (
        "link5.stl",
        include_bytes!("../assets/meshes_v20/link5.stl"),
    ),
    (
        "link6.stl",
        include_bytes!("../assets/meshes_v20/link6.stl"),
    ),
    (
        "ee_base_link.stl",
        include_bytes!("../assets/meshes_v20/ee_base_link.stl"),
    ),
    (
        "finger_inner.stl",
        include_bytes!("../assets/meshes_v20/finger_inner.stl"),
    ),
    (
        "finger_outer.stl",
        include_bytes!("../assets/meshes_v20/finger_outer.stl"),
    ),
];

#[cfg(feature = "meshes")]
impl HardwareVersion {
    /// The embedded collision meshes for this generation.
    fn meshes(self) -> &'static [(&'static str, &'static [u8])] {
        match self {
            Self::V1 => V1_MESHES,
            Self::V2 => V2_MESHES,
        }
    }

    /// Write this generation's embedded collision meshes into `dir` (created if absent).
    /// Materializing them lets the file-based collision builder, and the sim via the
    /// `emit_meshes` binary, consume assets that travel with this crate instead of a
    /// per-node or per-image vendored copy.
    pub fn write_meshes_to(self, dir: &std::path::Path) -> std::io::Result<()> {
        std::fs::create_dir_all(dir)?;
        for (name, bytes) in self.meshes() {
            std::fs::write(dir.join(name), bytes)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parsed(v: HardwareVersion) -> urdf_rs::Robot {
        urdf_rs::read_from_string(v.urdf()).expect("bundled URDF parses")
    }

    #[test]
    fn both_urdfs_parse_and_carry_their_chain_bases() {
        // Each generation names its per-arm base link differently: v1 `link0`, v2
        // `base_link` (v2 folded the ±90° mount roll into the arm chain).
        let cases = [
            (
                HardwareVersion::V1,
                ["openarm_left_link0", "openarm_right_link0"],
            ),
            (
                HardwareVersion::V2,
                ["openarm_left_base_link", "openarm_right_base_link"],
            ),
        ];
        for (v, bases) in cases {
            let robot = parsed(v);
            for base in bases {
                assert!(
                    robot.links.iter().any(|l| l.name == base),
                    "{v}: missing base link {base}"
                );
            }
        }
    }

    #[test]
    fn both_generations_share_the_bimanual_torso() {
        for v in [HardwareVersion::V1, HardwareVersion::V2] {
            let robot = parsed(v);
            assert!(
                robot.links.iter().any(|l| l.name == "openarm_body_link0"),
                "{v}: missing shared torso link"
            );
        }
    }

    #[test]
    fn jaw_widths_are_positive_and_v2_opens_wider() {
        assert!(HardwareVersion::V1.jaw_open_m() > 0.0);
        assert!(HardwareVersion::V2.jaw_open_m() > HardwareVersion::V1.jaw_open_m());
    }

    #[cfg(feature = "meshes")]
    #[test]
    fn v2_torso_asset_stays_byte_identical_to_the_shared_embed() {
        let v2_file: &[u8] = include_bytes!("../assets/meshes_v20/body_link0_symp.stl");
        assert_eq!(TORSO_MESH, v2_file);
    }

    #[test]
    fn keeps_the_mechanical_elbow_limit_in_both() {
        // The file carries the vendored `0.0`; the singularity margin is a control policy
        // the consumer applies (elbow_singularity_floor_rad), not baked into the data.
        let cases = [
            (
                HardwareVersion::V1,
                ["openarm_left_joint4", "openarm_right_joint4"],
            ),
            (
                HardwareVersion::V2,
                ["openarm_left_joint4", "openarm_right_joint4"],
            ),
        ];
        for (v, elbows) in cases {
            let robot = parsed(v);
            for elbow in elbows {
                let joint = robot
                    .joints
                    .iter()
                    .find(|j| j.name == elbow)
                    .unwrap_or_else(|| panic!("{v}: missing elbow joint {elbow}"));
                assert_eq!(
                    joint.limit.lower, 0.0,
                    "{v}: {elbow} lower limit is mechanical"
                );
            }
        }
    }

    #[test]
    fn v2_widened_the_shoulder_pitch_limit() {
        // v1 joint2 is symmetric ±1.7453; v2 widened it to an asymmetric range with a
        // ~3.3161 rad reach. Guard the magnitude so a stale vendored URDF is caught.
        let j2 = |v: HardwareVersion| {
            parsed(v)
                .joints
                .into_iter()
                .find(|j| j.name == "openarm_left_joint2")
                .expect("joint2 present")
                .limit
        };
        let v1 = j2(HardwareVersion::V1);
        let v2 = j2(HardwareVersion::V2);
        assert!(
            (v1.upper - v1.lower) < 3.6,
            "v1 joint2 is the symmetric range"
        );
        assert!(
            (v2.upper - v2.lower) > 3.4,
            "v2 joint2 widened: got [{}, {}]",
            v2.lower,
            v2.upper
        );
    }

    #[test]
    fn joint_limits_are_well_formed_with_the_elbow_floored() {
        for v in [HardwareVersion::V1, HardwareVersion::V2] {
            let elbow = v.elbow_joint_index();
            for side in [Side::Left, Side::Right] {
                let limits = v.joint_limits(side);
                for (i, &[lo, hi]) in limits.iter().enumerate() {
                    assert!(lo < hi, "{v} {side:?} j{}: range [{lo}, {hi}]", i + 1);
                }
                assert_eq!(
                    limits[elbow][0],
                    v.elbow_singularity_floor_rad(),
                    "{v} {side:?}: elbow lower must be the singularity floor"
                );
            }
        }
    }

    #[test]
    fn joint_limits_match_the_urdf_outside_the_elbow() {
        // Only the elbow lower bound is adjusted; every other bound is the file's.
        for v in [HardwareVersion::V1, HardwareVersion::V2] {
            let robot = parsed(v);
            let elbow = v.elbow_joint_index();
            for (side, prefix) in [(Side::Left, "left"), (Side::Right, "right")] {
                let limits = v.joint_limits(side);
                for (i, &[lo, hi]) in limits.iter().enumerate() {
                    let joint = robot
                        .joints
                        .iter()
                        .find(|j| j.name == format!("openarm_{prefix}_joint{}", i + 1))
                        .expect("joint present");
                    assert_eq!(hi, joint.limit.upper);
                    if i != elbow {
                        assert_eq!(lo, joint.limit.lower);
                    }
                }
            }
        }
    }

    #[test]
    fn hardware_version_round_trips_through_str() {
        for (s, v) in [("v1", HardwareVersion::V1), ("v2", HardwareVersion::V2)] {
            assert_eq!(s.parse::<HardwareVersion>().unwrap(), v);
            assert_eq!(v.to_string(), s);
        }
        assert!("v3".parse::<HardwareVersion>().is_err());
    }

    #[cfg(feature = "meshes")]
    #[test]
    fn write_meshes_to_lays_down_every_mesh_for_both_generations() {
        for v in [HardwareVersion::V1, HardwareVersion::V2] {
            let dir = std::env::temp_dir().join(format!("openarm_description_meshes_{v}"));
            let _ = std::fs::remove_dir_all(&dir);
            v.write_meshes_to(&dir).expect("materialize meshes");
            for (name, _) in v.meshes() {
                assert!(
                    dir.join(name).is_file(),
                    "{v}: missing materialized mesh {name}"
                );
            }
            std::fs::remove_dir_all(&dir).expect("cleanup");
        }
    }
}
