//! Forward kinematics for a 7-DOF SRS arm over a URDF chain.
//!
//! Poses come out of [`crate::chain`] in the URDF root (world) frame. This type
//! exposes both: **base-frame** accessors ([`ee_pose`](Posed::ee_pose),
//! `axis_base`, `origin_base`), re-expressed in the arm's own mount link so they
//! line up with the SRS geometry in [`crate::model`] and the IK target frame; and
//! **world-frame** accessors used by [`crate::gravity`] / [`crate::coriolis`],
//! because gravity is a world quantity (acts along world -Z). The fixed
//! `world -> base` transform relating the two is captured once at load.
//!
//! This is also the *independent* FK used to verify IK: it composes joint
//! transforms straight from the URDF, sharing no code with the analytic solver,
//! so an IK->FK round-trip is a genuine test.
//!
//! The seven-revolute rule lives here rather than in [`crate::chain`]: the chain
//! poses any serial path through any URDF, and this layer is what insists the
//! path be an SRS arm.

use nalgebra::{Isometry3, Matrix3, Point3, Vector3};

use crate::SrsError;
use crate::chain::{Chain, JointKind};
use crate::payload::Payload;
use crate::{ARM_DOF, JointVec, Limit};

/// The arm's chain plus the per-segment immutable data (axis, mass, COM,
/// inertia) captured at load. Segment `i` is the link moved by joint `i`; its
/// inertia is in the link frame.
pub(crate) struct ForwardKinematics {
    chain: Chain,
    /// Joint indices from the URDF root down to the tip, in order. Includes the
    /// fixed mounting joints above the arm.
    path: Vec<usize>,
    /// Where each actuated joint sits in `path`, so `q[i]` and the world
    /// transforms line up without re-walking.
    actuated: [usize; ARM_DOF],
    /// The tip link: the wrist, and the end-effector frame unless a tool is
    /// mounted (see [`tool`](Self::tool)).
    tip_link: usize,
    /// `world -> base_link`; constant because every joint between them is fixed.
    base_from_world: Isometry3<f64>,
    axes_local: [Vector3<f64>; ARM_DOF],
    masses: [f64; ARM_DOF],
    coms_local: [Vector3<f64>; ARM_DOF],
    inertias_local: [Matrix3<f64>; ARM_DOF],
    /// Per-joint lower-bound floors applied on top of the URDF limits (see
    /// [`limits`](Self::limits)). Defaults to `-inf` (no-op); a caller raises one
    /// via [`set_lower_floor`](Self::set_lower_floor) to impose a control margin
    /// (e.g. holding a joint off a solver singularity) without editing the URDF.
    lower_floors: [f64; ARM_DOF],
    /// Fixed `tip -> tool` transform, placing the mounted tool's control point on
    /// the wrist. Identity until [`set_tool_link`](Self::set_tool_link), so a bare
    /// arm controls its tip.
    tool: Isometry3<f64>,
    /// World transform of each `path` joint's child link, refreshed by
    /// [`at`](Self::at). Holding it here is what makes `at` take `&mut self` and
    /// every read through [`Posed`] follow a pose.
    world: Vec<Isometry3<f64>>,
}

impl ForwardKinematics {
    /// Build the FK chain from a URDF string given only where the SRS arm
    /// *starts* (`base_link`). The wrist (tip) is found by walking exactly
    /// [`ARM_DOF`] revolute joints out from the base, so the 7-DOF SRS invariant
    /// is enforced rather than trusting a hand-entered tip that might disagree.
    /// Everything past the wrist (gripper, fingers, tools) becomes the distal
    /// payload. Agnostic to *which* 7-DOF SRS arm: any URDF + base link the
    /// caller passes (it is not a general N-DOF or non-SRS solver).
    pub fn from_urdf(urdf: &str, base_link: &str) -> Result<Self, SrsError> {
        let robot = urdf_rs::read_from_string(urdf).map_err(|e| format!("parse URDF: {e}"))?;
        Self::from_chain(Chain::from_robot(&robot)?, base_link)
    }

    /// Like [`from_urdf`](Self::from_urdf) but reads the URDF from a file path,
    /// folding the IO error into the same `Result`.
    pub fn from_urdf_file(path: &str, base_link: &str) -> Result<Self, SrsError> {
        let urdf = std::fs::read_to_string(path).map_err(|source| SrsError::UrdfRead {
            path: path.to_string(),
            source,
        })?;
        Self::from_urdf(&urdf, base_link)
    }

    /// URDF joint position limits, j1..j7, in radians. Falls back to `±π` for any
    /// joint the URDF leaves unlimited (none do for the OpenArm V1.0). Read off the
    /// parsed chain, so a consumer that only needs FK + limits (e.g. a controller
    /// computing gravity/Coriolis) never has to build an [`ArmModel`](crate::model::ArmModel).
    ///
    /// Any lower floor set via [`set_lower_floor`](Self::set_lower_floor) raises the
    /// reported lower bound (clamped to stay `<= hi`, so the range is always
    /// well-ordered).
    pub fn limits(&self) -> [Limit; ARM_DOF] {
        std::array::from_fn(|i| {
            let (lo, hi) = self
                .chain
                .joint(self.actuated_joint(i))
                .limit
                .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI));
            Limit {
                lo: lo.max(self.lower_floors[i]).min(hi),
                hi,
            }
        })
    }

    /// Raise the reported lower bound of joint `joint_idx` to at least `floor`,
    /// leaving the parsed URDF untouched. Used to impose a control margin the URDF
    /// itself should not carry (the mechanical limit stays as vendored). Panics if
    /// `joint_idx >= ARM_DOF`.
    pub(crate) fn set_lower_floor(&mut self, joint_idx: usize, floor: f64) {
        assert!(
            joint_idx < ARM_DOF,
            "joint_idx {joint_idx} out of range (ARM_DOF={ARM_DOF})"
        );
        self.lower_floors[joint_idx] = floor;
    }

    /// Mount the tool frame carried by `link_name`: [`Posed::ee_pose`] and the
    /// Jacobian then describe that link instead of the tip. The SRS geometry the IK
    /// solver is derived from stays on the tip ([`Posed::tip_pose`]), so mounting a
    /// tool never moves the wrist center the solver's closed form is built around.
    ///
    /// The link must sit below the tip on fixed joints only. Both halves are
    /// enforced: a link elsewhere in the URDF is not found, and one reached through
    /// a movable joint is rejected, because its offset would silently change with
    /// that joint.
    pub(crate) fn set_tool_link(&mut self, link_name: &str) -> Result<(), SrsError> {
        self.tool = self
            .chain
            .fixed_path_to(self.tip_link, link_name)
            .ok_or_else(|| {
                SrsError::Tool(format!(
                    "'{link_name}' is not a link fixed below the tip '{}'",
                    self.chain.link(self.tip_link).name
                ))
            })?;
        Ok(())
    }

    /// The mounted `tip -> tool` transform (identity when none is mounted).
    pub(crate) fn tool(&self) -> Isometry3<f64> {
        self.tool
    }

    /// The chain index of actuated joint `i`.
    fn actuated_joint(&self, i: usize) -> usize {
        self.path[self.actuated[i]]
    }

    fn from_chain(chain: Chain, base_link: &str) -> Result<Self, SrsError> {
        let base = chain
            .link_index(base_link)
            .ok_or_else(|| format!("URDF missing base link '{base_link}'"))?;

        // The SRS chain is implicit: walk ARM_DOF revolute joints out from the
        // base to find the wrist, then lump everything past it into the distal
        // payload.
        let tip_link = find_srs_tip(&chain, base)?;
        let payload = Payload::from_distal(&chain, tip_link);

        let path = chain.path_to(tip_link);
        let actuated = actuated_slots(&chain, &path)?;
        let axes_local = std::array::from_fn(|i| match chain.joint(path[actuated[i]]).kind {
            JointKind::Revolute { axis } => axis.into_inner(),
            _ => unreachable!("actuated_slots verified revolute"),
        });

        // Per-segment inertial data: segment i is the link rigidly attached
        // downstream of joint i.
        let mut masses = [0.0_f64; ARM_DOF];
        let mut coms_local = [Vector3::zeros(); ARM_DOF];
        let mut inertias_local = [Matrix3::zeros(); ARM_DOF];
        for i in 0..ARM_DOF {
            let link = chain.link(chain.joint(path[actuated[i]]).child);
            masses[i] = link.mass;
            coms_local[i] = link.com;
            inertias_local[i] = link.inertia;
        }

        // Fold the distal payload into the last segment. It is rigidly attached
        // to the wrist link, whose frame is segment ARM_DOF-1's frame (the tip is
        // that link), so a bigger last link and a separate payload are the same
        // rigid body. Gravity / Coriolis then carry it.
        if payload.mass > 0.0 {
            let last = ARM_DOF - 1;
            let merged =
                payload.combined_with(masses[last], coms_local[last], inertias_local[last]);
            masses[last] = merged.mass;
            coms_local[last] = merged.com;
            inertias_local[last] = merged.inertia;
        }

        let mut fk = Self {
            world: vec![Isometry3::identity(); path.len()],
            chain,
            path,
            actuated,
            tip_link,
            base_from_world: Isometry3::identity(),
            axes_local,
            masses,
            coms_local,
            inertias_local,
            lower_floors: [f64::NEG_INFINITY; ARM_DOF],
            tool: Isometry3::identity(),
        };

        // Pose at home, then read the fixed base-link world transform. Every joint
        // between the root and the base is fixed, so it does not move afterwards.
        fk.at(&[0.0; ARM_DOF]);
        let base_world = fk
            .world_of_link(base)
            .ok_or("base link has no world transform")?;
        fk.base_from_world = base_world.inverse();
        Ok(fk)
    }

    /// World transform of `link`, if it lies on the arm's path (or is the root).
    fn world_of_link(&self, link: usize) -> Option<Isometry3<f64>> {
        if link == self.chain.root() {
            return Some(Isometry3::identity());
        }
        self.path
            .iter()
            .position(|&j| self.chain.joint(j).child == link)
            .map(|slot| self.world[slot])
    }

    /// Pose the arm at configuration `q` and return a read-only view of it. The
    /// `&mut` is what makes "pose, then read" an invariant rather than a calling
    /// convention: the returned [`Posed`] holds exclusive access for its lifetime,
    /// so no accessor runs before a pose and no re-pose races a read.
    pub fn at(&mut self, q: &JointVec) -> Posed<'_> {
        let Self {
            chain, path, world, ..
        } = self;
        let mut here = Isometry3::identity();
        let mut actuated = 0;
        for (slot, &j) in path.iter().enumerate() {
            let joint = chain.joint(j);
            // Joints off the arm (the fixed mount above it) take no value.
            let value = if joint.kind.is_movable() {
                let v = q[actuated];
                actuated += 1;
                v
            } else {
                0.0
            };
            here *= joint.origin * joint.kind.motion(value);
            world[slot] = here;
        }
        Posed { fk: self }
    }

    /// The fixed `world -> base` mount transform resolved from the URDF: the world
    /// frame this arm's base sits in. Gravity / Coriolis are computed in that world
    /// frame. It is **identity** when `base_link` is the URDF root (no mount tree
    /// above it), i.e. gravity is then computed in the base frame. Exposed so a
    /// caller can log/verify which frame is in play rather than assume one.
    pub fn base_from_world(&self) -> Isometry3<f64> {
        self.base_from_world
    }
}

/// The arm posed at one configuration: an immutable, read-only view obtained from
/// [`Arm::at`](crate::Arm::at). Every pose-dependent quantity (EE pose, gravity,
/// Coriolis) is read through here, so it can only be queried after a pose.
pub struct Posed<'a> {
    fk: &'a ForwardKinematics,
}

impl Posed<'_> {
    /// End-effector pose in the arm base frame: the mounted tool's control point,
    /// or the tip itself when no tool is mounted. This is the point the Jacobian
    /// and [`Arm::solve_ik`](crate::Arm::solve_ik) work at, so a caller commands
    /// and reads one frame throughout.
    pub fn ee_pose(&self) -> Isometry3<f64> {
        self.tip_pose() * self.fk.tool
    }

    /// Tip-link (wrist) pose in the arm base frame: where the chain's 7th revolute
    /// joint leaves the flange, before any mounted tool. Equal to
    /// [`ee_pose`](Self::ee_pose) on a bare arm; the two differ by the tool.
    pub fn tip_pose(&self) -> Isometry3<f64> {
        // The tip is the child of the last joint on the path.
        self.to_base(self.fk.world[self.fk.path.len() - 1])
    }

    /// Gravity-compensation torques at this posture: the torque each joint must
    /// apply to hold the arm against gravity (distal payload included).
    pub fn gravity_torques(&self) -> JointVec {
        crate::gravity::torques(self)
    }

    /// Coriolis + centripetal torques at joint velocity `qdot` for this posture.
    pub fn coriolis_torques(&self, qdot: &JointVec) -> JointVec {
        crate::coriolis::torques(self, qdot)
    }

    /// World-frame (URDF root) pose of segment `i`'s link frame: the link moved
    /// by joint `i+1`, the frame URDF `<collision>`/`<visual>` origins of that
    /// link are relative to. Both arms of a bimanual URDF share the root frame,
    /// so poses from two `Arm`s compose directly (e.g. for collision checking).
    pub fn link_pose_world(&self, i: usize) -> Isometry3<f64> {
        self.joint_world(i)
    }

    /// URDF name of segment `i`'s link (the link moved by joint `i+1`), e.g.
    /// `openarm_left_link3`. Keys per-link data such as collision geometry.
    pub fn link_name(&self, i: usize) -> String {
        let joint = self.fk.chain.joint(self.fk.path[self.fk.actuated[i]]);
        self.fk.chain.link(joint.child).name.clone()
    }

    /// World-frame revolute axis of joint `i`, re-expressed in the base frame. A
    /// revolute axis is invariant under its own angle, so rotating the local axis
    /// by the joint's world rotation is exact.
    pub(crate) fn axis_base(&self, i: usize) -> Vector3<f64> {
        self.to_base(self.joint_world(i))
            .rotation
            .transform_vector(&self.fk.axes_local[i])
    }

    /// Origin of joint `i`'s frame in the base frame. A point *on* the joint axis
    /// (used for SRS line geometry, never joint4's offset frame origin alone).
    pub(crate) fn origin_base(&self, i: usize) -> Vector3<f64> {
        self.to_base(self.joint_world(i)).translation.vector
    }

    /// The constant `world -> arm base` transform (this arm's fixed mounting on
    /// the body). Converts world/body-frame targets into the arm base frame.
    pub(crate) fn base_from_world(&self) -> Isometry3<f64> {
        self.fk.base_from_world
    }

    /// Linear-velocity Jacobian of a point rigidly attached to `segment` (the link
    /// moved by joint `segment`), as per-joint world-frame contributions: entry `j`
    /// is the point's world linear velocity per unit rate of joint `j`,
    /// `zⱼ × (p − pⱼ)`, and is zero for joints distal to the segment (they do not
    /// move the point). `point` is in the world (URDF root) frame that
    /// [`link_pose_world`](Self::link_pose_world) returns; a `segment` past the last
    /// joint clamps to the full chain. This is the EE [`jacobian`](Self::jacobian)'s
    /// linear rows generalized to an arbitrary witness point, for collision-distance
    /// gradients.
    pub fn point_world_jacobian(
        &self,
        point: &Point3<f64>,
        segment: usize,
    ) -> [Vector3<f64>; ARM_DOF] {
        let base_from_world = self.base_from_world();
        let world_from_base = base_from_world.rotation.inverse();
        let p_base = (base_from_world * point).coords;
        let last = segment.min(ARM_DOF - 1);
        std::array::from_fn(|j| {
            if j <= last {
                world_from_base * self.axis_base(j).cross(&(p_base - self.origin_base(j)))
            } else {
                Vector3::zeros()
            }
        })
    }

    // --- World-frame accessors (gravity is world -z; used by `gravity` / `coriolis`). ---
    // These are expressed in the URDF root/world frame used for gravity and for
    // the KDL reference checks. In the bundled OpenArm fixture, that root also
    // happens to coincide with `body_link0`.

    /// Mass of segment `i` (static; URDF parse-time).
    pub(crate) fn mass(&self, i: usize) -> f64 {
        self.fk.masses[i]
    }

    /// World-frame revolute axis of joint `i`.
    pub(crate) fn axis_world(&self, i: usize) -> Vector3<f64> {
        self.joint_world(i)
            .rotation
            .transform_vector(&self.fk.axes_local[i])
    }

    /// World-frame origin of joint `i`.
    pub(crate) fn origin_world(&self, i: usize) -> Vector3<f64> {
        self.joint_world(i).translation.vector
    }

    /// World-frame COM of segment `i`.
    pub(crate) fn com_world(&self, i: usize) -> Vector3<f64> {
        self.joint_world(i)
            .transform_point(&Point3::from(self.fk.coms_local[i]))
            .coords
    }

    /// World-frame inertia tensor of segment `i` about its COM:
    /// `I_world = R · I_local · Rᵀ`.
    pub(crate) fn inertia_world(&self, i: usize) -> Matrix3<f64> {
        let r = *self.joint_world(i).rotation.to_rotation_matrix().matrix();
        r * self.fk.inertias_local[i] * r.transpose()
    }

    /// World transform of actuated joint `i`'s frame, as posed by
    /// [`ForwardKinematics::at`].
    fn joint_world(&self, i: usize) -> Isometry3<f64> {
        self.fk.world[self.fk.actuated[i]]
    }

    fn to_base(&self, world: Isometry3<f64>) -> Isometry3<f64> {
        self.fk.base_from_world * world
    }
}

/// Walk from `base` down the unique revolute-bearing path and return the link
/// reached after exactly [`ARM_DOF`] revolute joints: the SRS wrist (tip). The
/// arm is serial until the wrist, so at each step exactly one child still leads
/// to a revolute joint; a fixed sensor branch or the (prismatic) gripper is
/// skipped, and a genuine fork (two revolute branches) is rejected as not a
/// single SRS arm.
fn find_srs_tip(chain: &Chain, base: usize) -> Result<usize, SrsError> {
    let mut link = base;
    let mut revolute = 0;
    while revolute < ARM_DOF {
        let mut arm = chain.children_of(link).iter().filter(|&&j| {
            matches!(chain.joint(j).kind, JointKind::Revolute { .. })
                || chain.subtree_has_revolute(chain.joint(j).child)
        });
        let (Some(&joint), None) = (arm.next(), arm.next()) else {
            let n = chain
                .children_of(link)
                .iter()
                .filter(|&&j| {
                    matches!(chain.joint(j).kind, JointKind::Revolute { .. })
                        || chain.subtree_has_revolute(chain.joint(j).child)
                })
                .count();
            return Err(SrsError::NotSrsArm(if n == 0 {
                format!(
                    "chain from base reaches only {revolute} revolute joints; \
                     a 7-DOF SRS arm needs {ARM_DOF}"
                )
            } else {
                format!(
                    "ambiguous arm: {n} revolute-bearing branches share one link; \
                     not a single SRS chain"
                )
            }));
        };
        if matches!(chain.joint(joint).kind, JointKind::Revolute { .. }) {
            revolute += 1;
        }
        link = chain.joint(joint).child;
    }
    Ok(link)
}

/// Locate the [`ARM_DOF`] actuated joints within the root-to-tip `path`, as
/// indices into it, rejecting any chain that does not reduce to exactly that.
///
/// A clean 7-DOF SRS arm's *only* degrees of freedom are its seven revolute
/// joints. Any other movable joint (e.g. a prismatic DOF interspersed on the
/// path) is rejected: it would otherwise pass the revolute count below while
/// `base_from_world` is frozen at home, silently building the wrong model
/// instead of returning `Err`.
fn actuated_slots(chain: &Chain, path: &[usize]) -> Result<[usize; ARM_DOF], SrsError> {
    if let Some(&extra) = path
        .iter()
        .find(|&&j| matches!(chain.joint(j).kind, JointKind::Prismatic { .. }))
    {
        return Err(SrsError::NotSrsArm(format!(
            "SRS chain has a non-revolute movable joint '{}': not a 7-DOF revolute arm",
            chain.joint(extra).name
        )));
    }
    let slots: Vec<usize> = path
        .iter()
        .enumerate()
        .filter(|(_, j)| matches!(chain.joint(**j).kind, JointKind::Revolute { .. }))
        .map(|(slot, _)| slot)
        .collect();
    slots.try_into().map_err(|v: Vec<_>| {
        SrsError::NotSrsArm(format!(
            "expected {ARM_DOF} revolute joints in the SRS chain, got {}",
            v.len()
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fk() -> ForwardKinematics {
        crate::test_support::v1_fk("left")
    }

    #[test]
    fn loads_seven_revolute_chain() {
        let mut fk = fk();
        let posed = fk.at(&[0.0; ARM_DOF]);
        for i in 0..ARM_DOF {
            let n = posed.axis_base(i).norm();
            assert!((n - 1.0).abs() < 1e-9, "joint {i} axis not unit: {n}");
        }
    }

    #[test]
    fn point_world_jacobian_matches_finite_difference() {
        let h = 1e-6;
        let configs: [JointVec; 3] = [
            [0.3, -0.2, 0.5, 0.4, -0.6, 0.2, 0.1],
            [-0.5, 0.4, -0.3, 0.8, 0.5, -0.4, 0.7],
            [0.1, 0.1, 0.1, 0.3, 0.1, 0.1, 0.1],
        ];
        // A fixed offset in each link's frame, so the same material point is tracked
        // across the perturbed configurations.
        let offset = Point3::new(0.05, -0.03, 0.04);
        for side in ["left", "right"] {
            let mut fk = crate::test_support::v1_fk(side);
            for q in configs {
                for segment in 0..ARM_DOF {
                    let point = fk.at(&q).link_pose_world(segment) * offset;
                    let cols = fk.at(&q).point_world_jacobian(&point, segment);
                    for j in 0..ARM_DOF {
                        let mut qp = q;
                        let mut qm = q;
                        qp[j] += h;
                        qm[j] -= h;
                        let pp = fk.at(&qp).link_pose_world(segment) * offset;
                        let pm = fk.at(&qm).link_pose_world(segment) * offset;
                        let fd = (pp.coords - pm.coords) / (2.0 * h);
                        // For j > segment the column is zero and the point does not
                        // move (a distal joint), so both sides are ~0.
                        assert!(
                            (cols[j] - fd).norm() < 1e-5,
                            "{side} segment {segment} joint {j} off by {}",
                            (cols[j] - fd).norm()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn home_ee_is_above_shoulder() {
        let mut fk = fk();
        let ee = fk.at(&[0.0; ARM_DOF]).ee_pose();
        // At home the wrist center sits at ~(0, 0.436, 0.1225) in base frame.
        // Mainly a smoke check that the base transform applied.
        let w = ee.translation.vector;
        assert!(
            (w - Vector3::new(0.0, 0.436, 0.1225)).norm() < 1e-3,
            "home EE {w:?} not at expected wrist center",
        );
    }

    #[test]
    fn link_names_follow_fixture_naming() {
        let mut fk = fk();
        let posed = fk.at(&[0.0; ARM_DOF]);
        for i in 0..ARM_DOF {
            let expected = format!("openarm_left_link{}", i + 1);
            assert_eq!(posed.link_name(i), expected);
        }
    }

    #[test]
    fn last_link_world_pose_matches_ee_pose_via_mount() {
        // link_pose_world(6) is the tip link's world pose; ee_pose is the same
        // pose re-expressed in the base frame, so they must agree through the
        // fixed mount transform.
        let mut fk = fk();
        let q = [0.3, -0.4, 0.5, 0.6, -0.2, 0.1, 0.7];
        let posed = fk.at(&q);
        let via_mount = posed.base_from_world() * posed.link_pose_world(ARM_DOF - 1);
        let ee = posed.ee_pose();
        assert!((via_mount.translation.vector - ee.translation.vector).norm() < 1e-12);
        assert!(via_mount.rotation.angle_to(&ee.rotation) < 1e-12);
    }

    #[test]
    fn left_and_right_first_links_are_mirrored_in_world() {
        // Both chains share the URDF root frame: at home, the two shoulder
        // (link1) origins must mirror across the XZ plane at the mount offsets.
        let mut left = crate::test_support::v1_fk("left");
        let mut right = crate::test_support::v1_fk("right");
        let l = left
            .at(&[0.0; ARM_DOF])
            .link_pose_world(0)
            .translation
            .vector;
        let r = right
            .at(&[0.0; ARM_DOF])
            .link_pose_world(0)
            .translation
            .vector;
        assert!(
            (l - Vector3::new(0.0, 0.0935, 0.698)).norm() < 1e-6,
            "left shoulder at {l:?}"
        );
        assert!(
            (r - Vector3::new(0.0, -0.0935, 0.698)).norm() < 1e-6,
            "right shoulder at {r:?}"
        );
    }

    #[test]
    fn rejects_urdf_missing_arm_links() {
        // A URDF without the arm links must Err, not panic.
        let urdf = r#"<?xml version="1.0"?><robot name="x"><link name="world"/></robot>"#;
        assert!(ForwardKinematics::from_urdf(urdf, "openarm_left_link0").is_err());
    }

    #[test]
    fn rejects_malformed_urdf() {
        assert!(ForwardKinematics::from_urdf("not even xml", "a").is_err());
    }

    #[test]
    fn rejects_chain_without_seven_revolute_joints() {
        // A prismatic-only arm must Err: walking out from the base finds no
        // revolute joints to reach the wrist, so it is not a 7-DOF SRS arm. (A
        // prismatic joint *interspersed* among the 7 is caught separately by
        // `collect_revolute_nodes`.)
        let urdf = r#"<?xml version="1.0"?><robot name="x">
          <link name="base"/><link name="tip"/>
          <joint name="slide" type="prismatic">
            <parent link="base"/><child link="tip"/>
            <axis xyz="0 0 1"/><origin xyz="0 0 0"/>
            <limit lower="0" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let err = match ForwardKinematics::from_urdf(urdf, "base") {
            Ok(_) => panic!("expected Err for a prismatic joint"),
            Err(e) => e,
        };
        assert!(
            matches!(&err, SrsError::NotSrsArm(_)),
            "unexpected error: {err}"
        );
    }
}
