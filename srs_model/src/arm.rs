//! The library entry point: one SRS arm loaded from a URDF, exposing forward
//! kinematics, gravity/Coriolis dynamics, and inverse kinematics behind a single
//! handle. Build it once and everything hangs off it; the underlying FK chain and
//! SRS model are internal.

use k::nalgebra::{Isometry3, Vector3, Vector6};

use crate::fk::{ForwardKinematics, Posed};
use crate::ik::{self, ArmAnglePolicy, Solution};
use crate::jacobian::damped_pseudo_inverse;
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

    /// Raise the reported lower limit of joint `joint_idx` (0-based, j1..j7) to at
    /// least `floor`, returning the arm. The parsed URDF is left untouched: this is a
    /// control margin layered over the mechanical limit (e.g. holding a joint off a
    /// solver singularity), surfaced through [`limits`](Self::limits) so every
    /// consumer of the limits inherits it. Panics if `joint_idx >= ARM_DOF`.
    pub fn with_lower_floor(mut self, joint_idx: usize, floor: f64) -> Self {
        self.fk.set_lower_floor(joint_idx, floor);
        self
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

    /// One damped resolved-rate joint step at `q` toward a world-frame task
    /// increment: `dp_world` metres of end-effector translation and `dw_world`
    /// axis-angle radians of rotation, either of which may be zero to softly
    /// hold that component. The caller caps the increments to its speed budgets;
    /// this rotates them into the arm base frame, solves
    /// `dq = J⁺(λ) ξ` with the damped pseudo-inverse (bounded through
    /// singularities), scales `dq` so every joint respects its velocity budget
    /// over `dt_s` while preserving direction, and clamps the result into the
    /// position limits. The step the operator streaming jog and the backbone's
    /// guarded servo both run, shared so the two control paths cannot drift.
    pub fn rate_step(
        &mut self,
        q: &JointVec,
        dp_world: Vector3<f64>,
        dw_world: Vector3<f64>,
        max_joint_velocity_rad_s: &JointVec,
        dt_s: f64,
        lambda: f64,
    ) -> JointVec {
        let to_base = self.base_from_world().rotation;
        let dp = to_base * dp_world;
        let dw = to_base * dw_world;
        let twist = Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z);
        let jacobian = self.at(q).jacobian();
        let mut dq = damped_pseudo_inverse(&jacobian, lambda) * twist;
        let scale = (0..ARM_DOF)
            .map(|i| {
                let cap = max_joint_velocity_rad_s[i] * dt_s;
                if dq[i].abs() > cap {
                    cap / dq[i].abs()
                } else {
                    1.0
                }
            })
            .fold(1.0_f64, f64::min);
        dq *= scale;
        let limits = self.limits();
        std::array::from_fn(|i| (q[i] + dq[i]).clamp(limits[i].lo, limits[i].hi))
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
    fn with_lower_floor_raises_only_the_targeted_lower_bound() {
        let arm = Arm::from_urdf_file(FIXTURE, "openarm_left_link0").expect("load fixture");
        let base = arm.limits();
        // The fixture's elbow (j4, index 3) has a mechanical lower bound of 0.0.
        let floored = arm.with_lower_floor(3, 0.05).limits();

        assert_eq!(
            floored[3].lo, 0.05,
            "targeted joint's lower bound is raised"
        );
        assert_eq!(floored[3].hi, base[3].hi, "upper bound is untouched");
        for i in [0, 1, 2, 4, 5, 6] {
            assert_eq!(
                floored[i].lo, base[i].lo,
                "joint {i} lower bound is untouched"
            );
        }
    }

    #[test]
    fn with_lower_floor_below_the_mechanical_limit_is_a_noop() {
        let arm = Arm::from_urdf_file(FIXTURE, "openarm_left_link0").expect("load fixture");
        let base = arm.limits();
        let floored = arm.with_lower_floor(3, -10.0).limits();
        assert_eq!(
            floored[3].lo, base[3].lo,
            "a floor under the limit does not lower it"
        );
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

    const RATE_Q: JointVec = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
    const RATE_V_MAX: JointVec = [3.0; ARM_DOF];
    const RATE_DT: f64 = 0.01;
    const RATE_LAMBDA: f64 = 0.05;

    fn rate_arm() -> Arm {
        Arm::from_urdf_file(FIXTURE, "openarm_left_link0").expect("load fixture")
    }

    fn ee_world(arm: &mut Arm, q: &JointVec) -> Isometry3<f64> {
        let base = arm.at(q).ee_pose();
        arm.world_pose(&base)
    }

    #[test]
    fn rate_step_moves_the_ee_along_the_commanded_direction() {
        let mut arm = rate_arm();
        let before = ee_world(&mut arm, &RATE_Q);
        let dp = Vector3::new(3e-3, 0.0, 0.0);
        let q = arm.rate_step(
            &RATE_Q,
            dp,
            Vector3::zeros(),
            &RATE_V_MAX,
            RATE_DT,
            RATE_LAMBDA,
        );
        let after = ee_world(&mut arm, &q);
        let moved = after.translation.vector - before.translation.vector;
        assert!(
            moved.dot(&dp) / dp.norm_squared() > 0.5,
            "step must realize most of the commanded translation, got {moved:?}"
        );
        assert!(
            after.rotation.angle_to(&before.rotation) < 5e-3,
            "an untasked orientation is softly held"
        );
    }

    #[test]
    fn rate_step_respects_the_velocity_budget_and_limits() {
        let mut arm = rate_arm();
        // An absurd demand: the scaling must keep every joint inside its budget.
        let dp = Vector3::new(1.0, -1.0, 0.5);
        let q = arm.rate_step(
            &RATE_Q,
            dp,
            Vector3::zeros(),
            &RATE_V_MAX,
            RATE_DT,
            RATE_LAMBDA,
        );
        let limits = arm.limits();
        for i in 0..ARM_DOF {
            let v = (q[i] - RATE_Q[i]).abs() / RATE_DT;
            assert!(v <= RATE_V_MAX[i] * 1.0001, "joint {i} at {v:.2} rad/s");
            assert!(q[i] >= limits[i].lo && q[i] <= limits[i].hi);
        }
    }

    #[test]
    fn rate_step_holds_still_on_a_zero_task() {
        let mut arm = rate_arm();
        let q = arm.rate_step(
            &RATE_Q,
            Vector3::zeros(),
            Vector3::zeros(),
            &RATE_V_MAX,
            RATE_DT,
            RATE_LAMBDA,
        );
        assert_eq!(q, RATE_Q, "zero task must not move any joint");
    }
}
