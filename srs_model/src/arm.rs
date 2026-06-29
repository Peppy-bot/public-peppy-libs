//! The library entry point: one SRS arm loaded from a URDF, exposing forward
//! kinematics, gravity/Coriolis dynamics, and inverse kinematics behind a single
//! handle. Build it once and everything hangs off it; the underlying FK chain and
//! SRS model are internal.

use k::nalgebra::Isometry3;

use crate::fk::{ForwardKinematics, Posed};
use crate::ik::{self, ArmAnglePolicy, Solution};
use crate::model::ArmModel;
use crate::{ARM_DOF, JointVec, Limit, SrsError};

/// A complete SRS arm built from a URDF: forward kinematics + gravity/Coriolis
/// dynamics + closed-form inverse kinematics. The URDF is parsed once at
/// construction; pose it with [`at`](Self::at) for FK and dynamics, and solve
/// targets with [`solve_ik`](Self::solve_ik).
pub struct Arm {
    fk: ForwardKinematics,
    model: ArmModel,
}

impl Arm {
    /// Build from a URDF string, given the link where the 7-DOF SRS chain starts
    /// (`base_link`). Returns `Err` if the chain is missing, too short, or not a
    /// clean SRS arm.
    pub fn from_urdf(urdf: &str, base_link: &str) -> Result<Self, SrsError> {
        let mut fk = ForwardKinematics::from_urdf(urdf, base_link)?;
        let model = ArmModel::from_fk(&mut fk)?;
        Ok(Self { fk, model })
    }

    /// Like [`from_urdf`](Self::from_urdf) but reads the URDF from a file path,
    /// folding the IO error into the same `Result`.
    pub fn from_urdf_file(path: &str, base_link: &str) -> Result<Self, SrsError> {
        let mut fk = ForwardKinematics::from_urdf_file(path, base_link)?;
        let model = ArmModel::from_fk(&mut fk)?;
        Ok(Self { fk, model })
    }

    /// Pose the arm at configuration `q` for forward-kinematics and dynamics
    /// reads. Takes `&mut self` not because posing requires it (`k` poses through
    /// interior mutability) but to enforce "pose, then read": the returned [`Posed`]
    /// holds exclusive access for its lifetime, so reads (EE pose, gravity, Coriolis)
    /// always follow a pose and never race a re-pose. See [`Posed::ee_pose`],
    /// [`Posed::gravity_torques`], [`Posed::coriolis_torques`].
    pub fn at(&mut self, q: &JointVec) -> Posed<'_> {
        self.fk.at(q)
    }

    /// Solve inverse kinematics for a `target` EE pose in the **arm base frame**,
    /// resolving the redundant arm angle per `arm_angle` and selecting the branch
    /// nearest `seed`. Convert a world-frame target with [`base_pose`](Self::base_pose)
    /// first. `None` if the target is unreachable or admits no in-limit solution.
    pub fn solve_ik(
        &self,
        target: &Isometry3<f64>,
        arm_angle: ArmAnglePolicy,
        seed: &JointVec,
    ) -> Option<Solution> {
        ik::solve(&self.model, target, arm_angle, seed)
    }

    /// The arm angle of configuration `q`, or `None` at the straight-arm
    /// singularity where it is geometrically undefined.
    pub fn arm_angle(&self, q: &JointVec) -> Option<f64> {
        ik::arm_angle_of(&self.model, q)
    }

    /// URDF joint position limits, j1..j7, in radians.
    pub fn limits(&self) -> [Limit; ARM_DOF] {
        self.fk.limits()
    }

    /// Convert a world/body-frame pose into the arm base frame the solver uses.
    pub fn base_pose(&self, world: &Isometry3<f64>) -> Isometry3<f64> {
        self.model.base_pose(world)
    }

    /// Convert an arm-base-frame pose (e.g. FK output) back into the world frame.
    pub fn world_pose(&self, base: &Isometry3<f64>) -> Isometry3<f64> {
        self.model.world_pose(base)
    }

    /// The fixed `world -> base` mount transform resolved from the URDF. It is
    /// identity when `base_link` is the URDF root (no mount tree above it); since
    /// gravity/Coriolis are evaluated in that frame, a caller can log/verify which
    /// frame is in play rather than assume one.
    pub fn base_from_world(&self) -> Isometry3<f64> {
        self.fk.base_from_world()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/openarm_v10.urdf"
    );

    #[test]
    fn from_urdf_file_loads_and_reports_limits() {
        let arm = Arm::from_urdf_file(FIXTURE, "openarm_left_link0").expect("load fixture");
        let limits = arm.limits();
        for (i, l) in limits.iter().enumerate() {
            assert!(l.lo <= l.hi, "joint {i}: lo {} > hi {}", l.lo, l.hi);
        }
        // j4 (elbow) is one-sided in the V1.0 URDF: lower bound at 0.
        assert!(limits[3].lo.abs() < 1e-9, "j4 lower = {}", limits[3].lo);
    }

    #[test]
    fn from_urdf_file_missing_path_errors_with_path() {
        // `Arm` is not `Debug` (the FK chain isn't), so match rather than unwrap_err.
        let err = match Arm::from_urdf_file("/no/such/file.urdf", "openarm_left_link0") {
            Ok(_) => panic!("expected an error for a missing path"),
            Err(e) => e,
        };
        assert!(
            matches!(&err, SrsError::UrdfRead { path, .. } if path == "/no/such/file.urdf"),
            "error should name the path: {err}"
        );
    }
}
