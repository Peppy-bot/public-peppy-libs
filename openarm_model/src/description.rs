//! Version-aware OpenArm hardware description: the one part of this crate that
//! pins a robot revision to its data (URDF, chain link names, friction model).
//! The kinematics/dynamics modules are robot-agnostic and consume what this
//! supplies.
//!
//! Only V1.0 is wired today. Dropping in a new revision is data-only: add a
//! [`Version`] variant, and the compiler points at every `match` that must be
//! given the revision's URDF, link names, and friction constants.

use crate::dynamics::friction::Params as FrictionParams;
use crate::fk::ForwardKinematics;
use crate::model::ArmModel;

/// Per-revision friction-model constants. The tanh model and the [`Params`] type
/// are agnostic and live in [`crate::dynamics::friction`]; only the numbers are
/// revision-specific, so they live here.
pub mod friction {
    use crate::dynamics::friction::Params;

    /// OpenArm V1.0 friction constants, from openarm_teleop's
    /// `config/leader.yaml` and `config/follower.yaml`. Those two are identical
    /// (bar a 0.01 rounding on joint 6): friction is a physical property of the
    /// joints, the *same* whether the arm is the leader or the follower, and both
    /// roles run the same `ComputeFriction`. The `coef_tmp = 0.1` tanh softening
    /// that `ComputeFriction` always applies is folded into `k`, so the runtime
    /// expression is `Fo + Fv*w + Fc*tanh(k*w)`.
    ///
    /// These are the *physical* (full) friction torques. openarm's
    /// transparency/leader control mode additionally scales the whole friction
    /// term by 0.3 (`control.cpp:277`); that scale is a control-layer choice and
    /// is intentionally NOT baked in here. Fo is a non-zero static offset, so at
    /// rest the model commands a small directional bias (intentional Coulomb
    /// breakaway).
    pub const V1: Params = Params {
        fc: [0.306, 0.306, 0.40, 0.166, 0.050, 0.093, 0.172],
        fv: [0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084],
        fo: [0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059],
        k: [2.8417, 2.8417, 2.9065, 13.0038, 15.1771, 24.2287, 0.7888],
    };
}

/// Which arm of the bimanual robot. Openarm-specific (it maps to the
/// `openarm_left`/`openarm_right` URDF prefix), so it lives here rather than in
/// the robot-agnostic modules.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmSide {
    Left,
    Right,
}

impl ArmSide {
    /// Parse the `"left"`/`"right"` node parameter.
    pub fn from_param(s: &str) -> Result<Self, String> {
        match s {
            "left" => Ok(ArmSide::Left),
            "right" => Ok(ArmSide::Right),
            other => Err(format!("arm_side must be 'left' or 'right', got '{other}'")),
        }
    }

    /// URDF link-name prefix for this side, e.g. `"openarm_left"`.
    pub fn prefix(self) -> &'static str {
        match self {
            ArmSide::Left => "openarm_left",
            ArmSide::Right => "openarm_right",
        }
    }
}

/// An OpenArm hardware revision. Add a variant to support a new one; the
/// `Description` matches below then fail to compile until that variant is given
/// a URDF, link names, and friction constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Version {
    V1,
}

impl Version {
    /// Parse a node `version` parameter, e.g. `"v1"`, `"1"`, `"1.0"`.
    pub fn from_param(s: &str) -> Result<Self, String> {
        match s.trim().to_ascii_lowercase().as_str() {
            "1" | "v1" | "1.0" | "v1.0" => Ok(Version::V1),
            other => Err(format!("unsupported version '{other}' (only v1 is wired)")),
        }
    }
}

/// Embedded URDF (bimanual; a single arm chain is extracted at load time).
const URDF_V1: &str = include_str!("../urdf/openarm_v10.urdf");

/// The description of one OpenArm: its revision and side. Maps to a URDF, the
/// base/tip link names bounding its 7-DOF chain, and a friction model. This is
/// the only place those mappings live; the math modules stay agnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Description {
    pub version: Version,
    pub side: ArmSide,
}

impl Description {
    pub fn new(version: Version, side: ArmSide) -> Self {
        Self { version, side }
    }

    /// The embedded URDF for this revision (bimanual; both arms).
    pub fn urdf(&self) -> &'static str {
        match self.version {
            Version::V1 => URDF_V1,
        }
    }

    /// Base link bounding this arm's chain.
    pub fn base_link(&self) -> String {
        let prefix = self.side.prefix();
        match self.version {
            Version::V1 => format!("{prefix}_link0"),
        }
    }

    /// Tip link (joint 7's child) bounding this arm's chain.
    pub fn tip_link(&self) -> String {
        let prefix = self.side.prefix();
        match self.version {
            Version::V1 => format!("{prefix}_link7"),
        }
    }

    /// Friction-model constants for this revision.
    pub fn friction(&self) -> FrictionParams {
        match self.version {
            Version::V1 => friction::V1,
        }
    }

    /// Build the forward-kinematics chain for this arm.
    pub fn forward_kinematics(&self) -> Result<ForwardKinematics, String> {
        ForwardKinematics::from_urdf(self.urdf(), &self.base_link(), &self.tip_link())
    }

    /// Build the kinematic model (SRS geometry + PoE screw data) for this arm.
    pub fn model(&self) -> Result<ArmModel, String> {
        ArmModel::from_urdf(self.urdf(), &self.base_link(), &self.tip_link())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_side_from_param() {
        assert_eq!(ArmSide::from_param("left").unwrap(), ArmSide::Left);
        assert_eq!(ArmSide::from_param("right").unwrap(), ArmSide::Right);
        assert!(ArmSide::from_param("middle").is_err());
    }

    #[test]
    fn version_from_param() {
        assert_eq!(Version::from_param("v1").unwrap(), Version::V1);
        assert_eq!(Version::from_param("1.0").unwrap(), Version::V1);
        assert!(Version::from_param("v2").is_err()); // not wired yet
    }

    #[test]
    fn link_names_per_side() {
        let left = Description::new(Version::V1, ArmSide::Left);
        assert_eq!(left.base_link(), "openarm_left_link0");
        assert_eq!(left.tip_link(), "openarm_left_link7");
        let right = Description::new(Version::V1, ArmSide::Right);
        assert_eq!(right.base_link(), "openarm_right_link0");
        assert_eq!(right.tip_link(), "openarm_right_link7");
    }

    /// Every (version, side) must load as a clean 7-DOF SRS arm. `model()`
    /// returns `Err` if the shoulder/wrist axes are not concurrent, so this is
    /// the drop-in guard: a malformed or non-SRS URDF fails loudly here.
    #[test]
    fn every_arm_loads_as_clean_srs() {
        for side in [ArmSide::Left, ArmSide::Right] {
            let desc = Description::new(Version::V1, side);
            desc.model()
                .unwrap_or_else(|e| panic!("{side:?} not a clean SRS arm: {e}"));
        }
    }

    /// V1 geometry sanity (known constants), proving the description wires the
    /// right URDF + link names through to the agnostic model.
    #[test]
    fn v1_geometry_matches_expected() {
        use crate::nalgebra::Vector3;
        let m = Description::new(Version::V1, ArmSide::Left)
            .model()
            .unwrap();
        assert!((m.shoulder - Vector3::new(0.0, 0.0, 0.1225)).norm() < 1e-4);
        assert!((m.wrist_home - Vector3::new(0.0, 0.436, 0.1225)).norm() < 1e-4);
    }
}
