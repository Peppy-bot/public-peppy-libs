//! A goal: the constraints a path's last waypoint meets, and the samplers that find
//! configurations meeting them.

use chain_kinematics::{
    Aims, Chain, ChainFrame, Draws, Ends, IkSolver, Limit, Posed, SEARCH_ITERATIONS, all_within,
};
use collision_geometry::chain::Rider;
use control_core::positive_finite::PositiveFinite;
use nalgebra::{Isometry3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3};

/// The joints from which a chain holds a frame at any pose it reaches: a pose's freedoms.
const POSE_FREEDOMS: usize = 6;

/// A frame of the arm that a target names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Frame {
    /// The chain's grasp point: its tool frame.
    GraspPoint,
    /// A link of the chain, the child link of joint `segment`, at the identity `offset`,
    /// or a link fixed below it, at `offset` in that link's frame.
    Link {
        segment: usize,
        offset: Isometry3<f64>,
    },
    /// A part the child link of joint `segment` carries through a plan at `offset` in
    /// that link's frame, such as a gripper finger at the gripper's opening, whose
    /// [`Rider`] gives the frame (`Frame::from`) where the arm's clearance checks place
    /// the part.
    Part {
        segment: usize,
        offset: Isometry3<f64>,
    },
}

impl From<&Rider<'_>> for Frame {
    fn from(rider: &Rider<'_>) -> Self {
        Self::Part {
            segment: rider.segment,
            offset: rider.offset,
        }
    }
}

impl std::fmt::Display for Frame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GraspPoint => write!(f, "the grasp point"),
            Self::Link { segment, offset } if *offset == Isometry3::identity() => {
                write!(f, "the child link of joint {segment}")
            }
            Self::Link { segment, .. } => {
                write!(f, "a link fixed below the child link of joint {segment}")
            }
            Self::Part { segment, .. } => {
                write!(f, "a part the child link of joint {segment} carries")
            }
        }
    }
}

impl From<Frame> for ChainFrame {
    fn from(frame: Frame) -> Self {
        match frame {
            Frame::GraspPoint => Self::EndEffector,
            Frame::Link { segment, offset } | Frame::Part { segment, offset } => {
                Self::OnSegment { segment, offset }
            }
        }
    }
}

impl Frame {
    /// Whether MoveIt's constraint sampler solves for the frame: the tip of its solver's
    /// chain or a link fixed to it (`IKConstraintSampler::loadIKSolver`,
    /// default_constraint_samplers.cpp:386-439), the grasp point, or a link that the chain's
    /// every joint moves and that is fixed below the last joint's link.
    pub(crate) fn solved_by_moveit<const N: usize>(self, chain: &Chain<N>) -> bool {
        match self {
            Self::GraspPoint => true,
            Self::Link { .. } => ChainFrame::from(self).moved_by_every_joint(chain),
            Self::Part { .. } => false,
        }
    }

    /// What a search for the frame aims at and the ends it offers, for a goal that names
    /// `named`. A frame MoveIt solves for is solved as KDL does. Two frames depart from
    /// MoveIt, which has no inverse kinematics for them ([`IkGoalSampler`] says what MoveIt
    /// does instead): a link the chain's later joints do not move, which fewer joints than
    /// the chain's move, so it may reach no pose drawn inside a goal exactly; and a part a
    /// link carries, such as a finger, which hangs on a joint of its own that the chain does
    /// not hold fixed, so MoveIt finds no fixed transform from it to the tip
    /// (`getAssociatedFixedTransforms`, default_constraint_samplers.cpp:395). For them the
    /// descent aims only at what the goal names, and the search offers the configuration each
    /// descent ends at.
    ///
    /// A chain of fewer joints than a pose has freedoms reaches no pose drawn inside a goal
    /// exactly. A goal of a position alone takes KDL's `position_only_ik`
    /// (kdl_kinematics_plugin.cpp:344-350). A goal that names an orientation departs from
    /// MoveIt, whose sampler offers converged ends alone (`callIK` passes no options,
    /// default_constraint_samplers.cpp:687-689): the descent weighs the orientation low,
    /// [`Aims::PositionFirst`], or aims at it alone, and the search offers each end. The goal's
    /// constraints decide on each end, as `IKConstraintSampler::validate` does (:653-660).
    fn search_of<const N: usize>(self, chain: &Chain<N>, named: Aims) -> (Aims, Ends) {
        if !self.solved_by_moveit(chain) {
            return (named, Ends::Approximate);
        }
        if N >= POSE_FREEDOMS {
            return (Aims::Both, Ends::Converged);
        }
        match named {
            Aims::Position => (Aims::Position, Ends::Converged),
            Aims::Orientation => (Aims::Orientation, Ends::Approximate),
            Aims::Both | Aims::PositionFirst => (Aims::PositionFirst, Ends::Approximate),
        }
    }

    /// The inverse kinematics of the frame of `chain`, for a goal that names `named`.
    pub(crate) fn ik_solver<const N: usize>(
        self,
        chain: &Chain<N>,
        named: Aims,
    ) -> IkSolver<'_, N> {
        let (aims, ends) = self.search_of(chain, named);
        IkSolver::new(chain, self.into(), aims, ends)
    }
}

/// A joint held within `below` under and `above` over `position`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointTarget {
    pub joint: usize,
    pub position: f64,
    pub above: PositiveFinite,
    pub below: PositiveFinite,
}

/// A frame's origin held within `tolerance_m` of `position`, in the robot frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionTarget {
    pub frame: Frame,
    pub position: Vector3<f64>,
    pub tolerance_m: PositiveFinite,
}

impl PositionTarget {
    /// Whether `p` lies within the tolerance of the position.
    fn holds(&self, p: &Vector3<f64>) -> bool {
        (p - self.position).norm() <= self.tolerance_m.get()
    }

    /// A point within the tolerance of the position, drawn uniformly.
    fn draw(&self, draws: &mut Draws) -> Vector3<f64> {
        loop {
            let p = Vector3::from_fn(|_, _| draws.between(-1.0, 1.0));
            if p.norm_squared() <= 1.0 {
                return self.position + p * self.tolerance_m.get();
            }
        }
    }
}

/// An axis of a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    /// The free axis a request's constraint names: "none", "x", "y" or "z".
    ///
    /// # Errors
    ///
    /// Any other name, as a message that says what to send.
    pub fn free_named(name: &str) -> Result<Option<Self>, String> {
        match name {
            "none" => Ok(None),
            "x" => Ok(Some(Self::X)),
            "y" => Ok(Some(Self::Y)),
            "z" => Ok(Some(Self::Z)),
            other => Err(format!(
                "free_axis is \"none\", \"x\", \"y\" or \"z\", not {other:?}"
            )),
        }
    }

    fn unit(self) -> Vector3<f64> {
        match self {
            Self::X => Vector3::x(),
            Self::Y => Vector3::y(),
            Self::Z => Vector3::z(),
        }
    }
}

/// A frame's orientation held within `tolerance_rad` of `orientation`: the angle of the
/// rotation between them, or, with a free axis, the angle between that axis of the frame
/// and its direction in `orientation`, the frame free to turn about it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrientationTarget {
    pub frame: Frame,
    pub orientation: UnitQuaternion<f64>,
    pub tolerance_rad: PositiveFinite,
    pub free_axis: Option<Axis>,
}

impl OrientationTarget {
    /// How far `actual` lies from the target, in radians, as the tolerance bounds it.
    pub fn error(&self, actual: &UnitQuaternion<f64>) -> f64 {
        match self.free_axis {
            None => self.orientation.angle_to(actual),
            Some(axis) => {
                let (target, held) = (self.orientation * axis.unit(), actual * axis.unit());
                target.cross(&held).norm().atan2(target.dot(&held))
            }
        }
    }

    /// How far the target's frame of `chain` at `q` lies from the target, in radians, as
    /// the tolerance bounds it.
    pub fn error_at<const N: usize>(&self, chain: &Chain<N>, q: &[f64; N]) -> f64 {
        self.error(&frame_pose(chain, q, self.frame).rotation)
    }

    /// The unit axis, in the robot frame, about which turning `actual` grows its error by
    /// one radian per radian. None where no axis does: at no error, and with a free axis
    /// that points exactly away from its direction in the target.
    pub(crate) fn error_axis(&self, actual: &UnitQuaternion<f64>) -> Option<Vector3<f64>> {
        match self.free_axis {
            None => (self.orientation.inverse() * actual)
                .axis()
                .map(|axis| self.orientation * axis.into_inner()),
            Some(axis) => {
                let (target, held) = (self.orientation * axis.unit(), actual * axis.unit());
                Unit::try_new(target.cross(&held), 0.0).map(Unit::into_inner)
            }
        }
    }

    /// How far apart the middles of this target and `other` of the same frame stand, by the
    /// measure of the one that holds a free axis, where no orientation meets both: the error
    /// grows at most as fast as the frame turns, so an orientation within a target's tolerance
    /// of its middle lies at most that much nearer the other's. None where one may, and where
    /// the two hold different free axes, whose distance no single angle states.
    fn apart_from(&self, other: &Self) -> Option<f64> {
        let angle = match (self.free_axis, other.free_axis) {
            (Some(own), Some(theirs)) if own != theirs => return None,
            (Some(_), _) => self.error(&other.orientation),
            (None, _) => other.error(&self.orientation),
        };
        (angle > self.tolerance_rad.get() + other.tolerance_rad.get()).then_some(angle)
    }

    /// The product of the target's tolerances about the three axes, a free axis's pi, as
    /// MoveIt's `IKConstraintSampler::getSamplingVolume` multiplies them
    /// (default_constraint_samplers.cpp:346-351).
    fn sampling_volume(&self) -> f64 {
        let tolerance = self.tolerance_rad.get();
        let about_the_third = self.free_axis.map_or(tolerance, |_| std::f64::consts::PI);
        tolerance * tolerance * about_the_third
    }

    /// An orientation within the tolerance of the target, drawn inside this crate's
    /// region of it as MoveIt's `IKConstraintSampler::samplePose` draws one inside its
    /// own (moveit2 2.15.2 default_constraint_samplers.cpp:493-529): the target turned by
    /// a rotation vector drawn uniformly inside the ball of the tolerance, where MoveIt
    /// draws each component of it inside the tolerance, a box; and with a free axis, first
    /// turned about it by an angle drawn whole.
    fn draw(&self, draws: &mut Draws) -> UnitQuaternion<f64> {
        let tilt = loop {
            let v = Vector3::from_fn(|_, _| draws.between(-1.0, 1.0));
            if v.norm_squared() <= 1.0 {
                break v * self.tolerance_rad.get();
            }
        };
        let turned = match self.free_axis {
            None => self.orientation,
            Some(axis) => {
                let turn = draws.between(-std::f64::consts::PI, std::f64::consts::PI);
                self.orientation
                    * UnitQuaternion::from_axis_angle(&Unit::new_unchecked(axis.unit()), turn)
            }
        };
        turned * UnitQuaternion::from_scaled_axis(tilt)
    }
}

/// A rotation drawn uniformly, as `IKConstraintSampler::samplePose` draws the orientation
/// of a goal that names none (default_constraint_samplers.cpp:541-547): Shoemake's draw
/// of a unit quaternion from three uniform numbers.
fn uniform_rotation(draws: &mut Draws) -> UnitQuaternion<f64> {
    let share = draws.unit();
    let (first, second) = (
        std::f64::consts::TAU * draws.unit(),
        std::f64::consts::TAU * draws.unit(),
    );
    let (r1, r2) = ((1.0 - share).sqrt(), share.sqrt());
    UnitQuaternion::from_quaternion(Quaternion::new(
        second.cos() * r2,
        first.sin() * r1,
        first.cos() * r1,
        second.sin() * r2,
    ))
}

/// A goal that names nothing to meet, or names it out of range.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum GoalError {
    #[error("a goal names at least one joint, position or orientation")]
    Empty,
    #[error("a goal names joints, or positions and orientations, not both")]
    Mixed,
    #[error("joint {0} is not a joint of the arm, whose joints run from 0 to {1}")]
    Joint(usize, usize),
    #[error("a target position is finite, got {0:?}")]
    Position([f64; 3]),
    #[error("{0} is named twice in one list of the goal")]
    Repeated(String),
    #[error(
        "a goal's positions and orientations name one frame of the arm, and this one names {0} and {1}: send the position and the orientation of one of them"
    )]
    Frames(String, String),
}

/// The constraints a path's last waypoint meets: joints, or the position and the
/// orientation of one frame of the arm, either or both, and the orientation a plan's path
/// holds of that frame, when the plan takes one.
#[derive(Debug, PartialEq)]
pub struct Goal {
    joints: Vec<JointTarget>,
    position: Option<PositionTarget>,
    orientation: Option<OrientationTarget>,
    path_orientation: Option<OrientationTarget>,
}

impl Goal {
    /// The goal of an arm of `dimension` joints that meets every one of `joints`,
    /// `positions` and `orientations`, the positions and orientations of one frame.
    pub fn new(
        dimension: usize,
        joints: Vec<JointTarget>,
        positions: Vec<PositionTarget>,
        orientations: Vec<OrientationTarget>,
    ) -> Result<Self, GoalError> {
        if joints.is_empty() && positions.is_empty() && orientations.is_empty() {
            return Err(GoalError::Empty);
        }
        let names_a_pose = !positions.is_empty() || !orientations.is_empty();
        if !joints.is_empty() && names_a_pose {
            return Err(GoalError::Mixed);
        }
        let in_arm = |joint: usize| {
            (joint < dimension)
                .then_some(())
                .ok_or(GoalError::Joint(joint, dimension.saturating_sub(1)))
        };
        let named_once = |names: Vec<String>| {
            names
                .iter()
                .enumerate()
                .find(|(k, name)| names[..*k].contains(name))
                .map_or(Ok(()), |(_, name)| Err(GoalError::Repeated(name.clone())))
        };
        let frame = |frame: Frame| match frame {
            Frame::GraspPoint => Ok(()),
            Frame::Link { segment, .. } | Frame::Part { segment, .. } => in_arm(segment),
        };
        for target in &joints {
            in_arm(target.joint)?;
        }
        for target in &positions {
            frame(target.frame)?;
            if !target.position.iter().all(|c| c.is_finite()) {
                return Err(GoalError::Position(target.position.into()));
            }
        }
        for target in &orientations {
            frame(target.frame)?;
        }
        named_once(
            joints
                .iter()
                .map(|t| format!("joint {}", t.joint))
                .collect(),
        )?;
        named_once(positions.iter().map(|t| format!("{:?}", t.frame)).collect())?;
        named_once(
            orientations
                .iter()
                .map(|t| format!("{:?}", t.frame))
                .collect(),
        )?;
        let mut frames = positions
            .iter()
            .map(|t| t.frame)
            .chain(orientations.iter().map(|t| t.frame));
        if let Some(first) = frames.next()
            && let Some(other) = frames.find(|frame| *frame != first)
        {
            return Err(GoalError::Frames(first.to_string(), other.to_string()));
        }
        // One frame, named once in each list: at most one position and one orientation.
        Ok(Self {
            joints,
            position: positions.into_iter().next(),
            orientation: orientations.into_iter().next(),
            path_orientation: None,
        })
    }

    /// Whether `chain` at `q` meets every constraint of the goal.
    pub fn met<const N: usize>(&self, chain: &Chain<N>, q: &[f64; N]) -> bool {
        let pose = |frame: Frame| frame_pose(chain, q, frame);
        let held = |t: &JointTarget| {
            (t.position - t.below.get()..=t.position + t.above.get()).contains(&q[t.joint])
        };
        self.joints.iter().all(held)
            && self
                .position
                .is_none_or(|t| t.holds(&pose(t.frame).translation.vector))
            && [self.orientation, self.path_orientation]
                .into_iter()
                .flatten()
                .all(|t| t.error(&pose(t.frame).rotation) <= t.tolerance_rad.get())
    }

    /// The goal a plan samples that holds `held`, the orientations its path holds of the
    /// arm's frames, as MoveIt merges a request's goal and path constraints
    /// (`kinematic_constraints::mergeConstraints`, utils.cpp:64-131): it takes the one on
    /// the frame it names, which it meets too. A goal of joints, or of another frame, takes
    /// none, and the plan judges what it samples by the orientations.
    pub(crate) fn holding(&self, held: &[OrientationTarget]) -> Self {
        let named = self.named_frame().map(|(frame, _)| frame);
        Self {
            joints: self.joints.clone(),
            position: self.position,
            orientation: self.orientation,
            path_orientation: held.iter().find(|t| Some(t.frame) == named).copied(),
        }
    }

    /// How far apart the goal's orientation and `held` stand where `held` names the goal's frame
    /// and no orientation meets both.
    pub(crate) fn orientation_apart_from(&self, held: &OrientationTarget) -> Option<f64> {
        self.orientation
            .filter(|own| own.frame == held.frame)
            .and_then(|own| own.apart_from(held))
    }

    /// The orientation a pose drawn inside the goal draws from: of the goal's and the
    /// path's, the one of the smaller sampling volume, the path's where they tie, as MoveIt
    /// keeps a link's sampler (constraint_sampler_manager.cpp:190-197, its path constraints
    /// after its goal's in a merged list).
    fn drawn_orientation(&self) -> Option<OrientationTarget> {
        match (self.orientation, self.path_orientation) {
            (Some(own), Some(path)) if own.sampling_volume() < path.sampling_volume() => Some(own),
            (own, path) => path.or(own),
        }
    }
}

/// The pose of `frame` of `chain` at `q`, in the robot frame.
fn frame_pose<const N: usize>(chain: &Chain<N>, q: &[f64; N], frame: Frame) -> Isometry3<f64> {
    frame_pose_at(&chain.at(q), frame)
}

/// The pose of `frame` of the chain `posed` holds, in the robot frame.
pub(crate) fn frame_pose_at<const N: usize>(posed: &Posed<N>, frame: Frame) -> Isometry3<f64> {
    ChainFrame::from(frame).pose_at(posed)
}

impl Goal {
    /// The frame the goal's position and orientation name, if it names either, and which of
    /// the two it names.
    fn named_frame(&self) -> Option<(Frame, Aims)> {
        match (self.position, self.drawn_orientation()) {
            (Some(position), Some(_)) => Some((position.frame, Aims::Both)),
            (Some(position), None) => Some((position.frame, Aims::Position)),
            (None, Some(orientation)) => Some((orientation.frame, Aims::Orientation)),
            (None, None) => None,
        }
    }

    /// A configuration inside `limits`, the joints the goal names drawn inside their
    /// targets where those overlap the limits, as MoveIt's
    /// `JointConstraintSampler::sample` draws the unbounded joints and then the bounded
    /// ones (default_constraint_samplers.cpp:179-191).
    fn draw_joints<const N: usize>(&self, limits: &[Limit; N], draws: &mut Draws) -> [f64; N] {
        let mut drawn = draws.within(limits);
        for t in &self.joints {
            let low = (t.position - t.below.get()).max(limits[t.joint].lo);
            let high = (t.position + t.above.get()).min(limits[t.joint].hi);
            if low <= high {
                drawn[t.joint] = draws.between(low, high);
            }
        }
        drawn
    }

    /// A pose of `frame` of `chain` drawn inside the goal, as
    /// `IKConstraintSampler::samplePose` draws one
    /// (default_constraint_samplers.cpp:441-557): a position drawn uniformly inside the
    /// position's tolerance, or, for a goal that names none, the frame's position at a
    /// configuration drawn inside `limits`; then an orientation drawn inside the
    /// orientation's tolerance, or drawn uniformly for a goal that names none.
    fn draw_pose<const N: usize>(
        &self,
        chain: &Chain<N>,
        frame: Frame,
        limits: &[Limit; N],
        draws: &mut Draws,
    ) -> Isometry3<f64> {
        let position = match self.position {
            Some(t) => t.draw(draws),
            None => {
                frame_pose(chain, &draws.within(limits), frame)
                    .translation
                    .vector
            }
        };
        let rotation = match self.drawn_orientation() {
            Some(t) => t.draw(draws),
            None => uniform_rotation(draws),
        };
        Isometry3::from_parts(Translation3::from(position), rotation)
    }
}

/// What finds configurations that meet a goal, one goal sample at a time, as MoveIt's
/// constraint samplers find them for its `ConstrainedGoalSampler`
/// (`crate::plan::sample_goals` runs the samples).
pub trait GoalSampler<const N: usize>: Sync {
    /// One goal sample, as MoveIt's `ConstraintSampler::sample` makes one: from the
    /// working state `working`, which lies inside the joint limits, with up to `attempts`
    /// attempts, drawing from `draws`; the first configuration inside the joint limits that
    /// meets `goal` and that `valid` accepts, or none.
    fn sample_from(
        &self,
        goal: &Goal,
        working: &[f64; N],
        attempts: usize,
        draws: &mut Draws,
        valid: &dyn Fn(&[f64; N]) -> bool,
    ) -> Option<[f64; N]>;
}

/// The goal sampler of any chain, as MoveIt's samplers of a goal sample. For a goal of
/// joints, one configuration drawn inside the joint limits, the joints the goal names
/// inside their targets, whatever the attempts, as `JointConstraintSampler::sample` draws
/// one (moveit2 2.15.2 default_constraint_samplers.cpp:169-196). For a pose, as
/// `IKConstraintSampler::sample` (:580-651): each attempt draws a pose of the goal's
/// frame inside the goal (`IKConstraintSampler::samplePose`) and searches for it by the
/// inverse kinematics of MoveIt's KDL plugin ([`IkSolver`]), the first attempt from
/// the working state held to the joint limits (`callIK(..., a == 0)`, :647, whose
/// `use_as_seed` seeds from the state's joints, :670-678) and every later one from a
/// configuration drawn inside them (:674-678). The search hands each configuration it
/// finds to the goal's constraints and to `valid`, as KDL's search hands each to the
/// constraint sampler's validity callback (:595-603). The grasp point, and a link that
/// the chain's every joint moves and that is fixed below the last joint's link, are the
/// tip of MoveIt's solver or a link fixed to it, and the search solves for them as KDL
/// does. A link the chain's later joints do not move, and a part a link carries such as a
/// finger, which hangs on a joint of its own, depart from MoveIt, which has no inverse
/// kinematics for them: MoveIt builds no IK sampler for a link that is not the tip or
/// fixed to it (`IKConstraintSampler::loadIKSolver`, :386-439), so its constraint sampler
/// manager selects none (constraint_sampler_manager.cpp:395) and its goal sampler draws
/// states uniformly and keeps those that meet the goal by chance
/// (constrained_goal_sampler.cpp:71-72, :158-166). This sampler descends toward what the
/// goal names, with KDL's position-only and approximate-solution options applied, and the
/// goal's constraints decide (`Frame::search_of`).
pub struct IkGoalSampler<const N: usize> {
    chain: Chain<N>,
}

impl<const N: usize> IkGoalSampler<N> {
    pub fn new(chain: Chain<N>) -> Self {
        Self { chain }
    }
}

impl<const N: usize> GoalSampler<N> for IkGoalSampler<N> {
    fn sample_from(
        &self,
        goal: &Goal,
        working: &[f64; N],
        attempts: usize,
        draws: &mut Draws,
        valid: &dyn Fn(&[f64; N]) -> bool,
    ) -> Option<[f64; N]> {
        let limits = self.chain.limits();
        let accepted = |q: &[f64; N]| goal.met(&self.chain, q) && valid(q);
        let Some((frame, named)) = goal.named_frame() else {
            let drawn = goal.draw_joints(&limits, draws);
            return accepted(&drawn).then_some(drawn);
        };
        assert!(
            all_within(&limits, working),
            "a goal sample's working state lies inside the joint limits, and {working:?} does not"
        );
        let ik = frame.ik_solver(&self.chain, named);
        (0..attempts).find_map(|attempt| {
            let target = goal.draw_pose(&self.chain, frame, &limits, draws);
            let seed = if attempt == 0 {
                *working
            } else {
                draws.within(&limits)
            };
            ik.search(seed, &target, draws, &accepted, SEARCH_ITERATIONS)
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use chain_kinematics::{ChainSpec, JointSelection};
    use robot_description::{ArmId, RobotDescription};

    use super::*;

    /// OpenArm v2's left arm, the first of its description's arms.
    pub(crate) const LEFT: ArmId = ArmId::new(0);

    /// The bundled description of the model `id`.
    fn bundled(id: &str) -> RobotDescription {
        robot_description::model(id)
            .unwrap_or_else(|unknown| panic!("{unknown}"))
            .description()
    }

    /// OpenArm v2's description: its arms `left_arm` then `right_arm`.
    pub(crate) fn openarm_v2() -> RobotDescription {
        let description = bundled("openarm_v2");
        let names: Vec<&str> = description
            .arms()
            .iter()
            .map(|arm| arm.name.as_str())
            .collect();
        assert_eq!(names, ["left_arm", "right_arm"], "OpenArm v2's arms");
        description
    }

    /// The chain of `arm`, an arm of `description`, from its base link to its grasp point.
    fn arm_chain<const N: usize>(
        description: &RobotDescription,
        arm: &robot_description::Arm,
    ) -> Chain<N> {
        Chain::from_urdf(
            description.robot(),
            &ChainSpec {
                base_link: Some(&arm.base_link),
                tip_link: &arm.grasp_link,
                joints: JointSelection::PathOrder,
            },
        )
        .unwrap_or_else(|error| panic!("{}: {error}", arm.name))
    }

    /// OpenArm v2's left arm from its base to its grasp point.
    pub(crate) fn left_arm() -> Chain<7> {
        let description = openarm_v2();
        arm_chain(&description, &description.arms()[LEFT.place()])
    }

    /// The grasp point held within `tolerance` of the identity, free about `free_axis`.
    fn hand_within(tolerance: f64, free_axis: Option<Axis>) -> OrientationTarget {
        OrientationTarget {
            frame: Frame::GraspPoint,
            orientation: UnitQuaternion::identity(),
            tolerance_rad: PositiveFinite::parse(tolerance).expect("a tolerance above 0"),
            free_axis,
        }
    }

    fn at_a_position(orientations: Vec<OrientationTarget>) -> Goal {
        Goal::new(
            7,
            Vec::new(),
            vec![PositionTarget {
                frame: Frame::GraspPoint,
                position: Vector3::new(0.3, 0.2, 0.5),
                tolerance_m: PositiveFinite::parse(0.01).expect("a tolerance above 0"),
            }],
            orientations,
        )
        .expect("a valid goal")
    }

    // A goal takes the path's orientation of the frame it names and leaves one of another frame
    // alone; a goal of joints takes none. It draws from the smaller sampling volume, the path's
    // where they tie, and a free axis counts pi about it, as MoveIt keeps a link's sampler.
    #[test]
    fn a_goal_holding_a_path_orientation_draws_from_the_smaller_volume() {
        let path = hand_within(0.1, None);
        let other_frame = OrientationTarget {
            frame: Frame::Link {
                segment: 2,
                offset: Isometry3::identity(),
            },
            ..path
        };
        let position_alone = at_a_position(Vec::new());
        assert_eq!(
            position_alone.named_frame(),
            Some((Frame::GraspPoint, Aims::Position))
        );
        let taken = position_alone.holding(&[other_frame, path]);
        assert_eq!(taken.path_orientation, Some(path));
        assert_eq!(taken.named_frame(), Some((Frame::GraspPoint, Aims::Both)));
        assert_eq!(taken.drawn_orientation(), Some(path));
        assert_eq!(
            position_alone.holding(&[other_frame]).path_orientation,
            None
        );

        let tighter = hand_within(0.05, None);
        let of = |own: OrientationTarget, path: OrientationTarget| {
            at_a_position(vec![own])
                .holding(&[path])
                .drawn_orientation()
        };
        assert_eq!(of(tighter, path), Some(tighter));
        assert_eq!(of(path, tighter), Some(tighter));
        let tie = hand_within(0.1, None);
        assert_eq!(of(tie, path), Some(path));
        let free = hand_within(0.1, Some(Axis::Z));
        assert_eq!(of(free, path), Some(path));
        assert_eq!(of(path, free), Some(path));

        let joints = Goal::new(
            7,
            vec![JointTarget {
                joint: 0,
                position: 0.0,
                above: PositiveFinite::parse(0.1).expect("a tolerance above 0"),
                below: PositiveFinite::parse(0.1).expect("a tolerance above 0"),
            }],
            Vec::new(),
            Vec::new(),
        )
        .expect("a valid goal");
        assert_eq!(joints.holding(&[path]).path_orientation, None);
    }

    // A goal's orientation lies apart from a path orientation of its frame exactly where their
    // middles stand farther apart than their tolerances together, whichever holds a free axis; a
    // path orientation of another frame, and two different free axes, state nothing.
    #[test]
    fn a_goal_orientation_lies_apart_from_a_path_orientation_past_both_tolerances() {
        let turned = |angle: f64, tolerance: f64, free_axis| OrientationTarget {
            orientation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle),
            ..hand_within(tolerance, free_axis)
        };
        let of_goal = |own| at_a_position(vec![own]);
        let whole = turned(0.0, 0.1, None);
        assert!(
            of_goal(whole)
                .orientation_apart_from(&turned(0.25, 0.1, None))
                .is_some()
        );
        assert_eq!(
            of_goal(whole).orientation_apart_from(&turned(0.15, 0.1, None)),
            None
        );
        let about_z = turned(0.0, 0.1, Some(Axis::Z));
        assert!(
            of_goal(whole)
                .orientation_apart_from(&turned(0.5, 0.1, Some(Axis::Z)))
                .is_some()
        );
        assert!(
            of_goal(about_z)
                .orientation_apart_from(&turned(0.5, 0.1, None))
                .is_some()
        );
        assert!(
            of_goal(about_z)
                .orientation_apart_from(&turned(0.5, 0.1, Some(Axis::Z)))
                .is_some()
        );
        assert_eq!(
            of_goal(about_z).orientation_apart_from(&turned(0.15, 0.1, None)),
            None
        );
        assert_eq!(
            of_goal(about_z).orientation_apart_from(&turned(0.5, 0.1, Some(Axis::Y))),
            None
        );
        let spun = OrientationTarget {
            orientation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.5),
            ..about_z
        };
        assert_eq!(of_goal(about_z).orientation_apart_from(&spun), None);
        assert_eq!(of_goal(whole).orientation_apart_from(&spun), None);
        let other_frame = OrientationTarget {
            frame: Frame::Link {
                segment: 2,
                offset: Isometry3::identity(),
            },
            ..turned(1.0, 0.1, None)
        };
        assert_eq!(of_goal(whole).orientation_apart_from(&other_frame), None);
        assert_eq!(
            at_a_position(Vec::new()).orientation_apart_from(&whole),
            None
        );
    }

    // A constraint names a free axis by its letter, or none; any other word says what to send.
    #[test]
    fn a_free_axis_is_named_by_its_letter() {
        assert_eq!(Axis::free_named("none"), Ok(None));
        assert_eq!(Axis::free_named("x"), Ok(Some(Axis::X)));
        assert_eq!(Axis::free_named("y"), Ok(Some(Axis::Y)));
        assert_eq!(Axis::free_named("z"), Ok(Some(Axis::Z)));
        assert_eq!(
            Axis::free_named("w"),
            Err("free_axis is \"none\", \"x\", \"y\" or \"z\", not \"w\"".to_string())
        );
    }

    // A goal holding a path orientation meets it as well as its own.
    #[test]
    fn a_goal_holding_a_path_orientation_meets_both() {
        let chain = left_arm();
        let q = [0.1; 7];
        let pose = frame_pose(&chain, &q, Frame::GraspPoint);
        let at = PositionTarget {
            frame: Frame::GraspPoint,
            position: pose.translation.vector,
            tolerance_m: PositiveFinite::parse(0.01).expect("a tolerance above 0"),
        };
        let turned_by = |angle: f64, tolerance: f64| OrientationTarget {
            orientation: pose.rotation * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle),
            ..hand_within(tolerance, None)
        };
        let goal = Goal::new(7, Vec::new(), vec![at], vec![turned_by(0.0, 0.1)]).expect("a goal");
        assert!(goal.met(&chain, &q));
        assert!(goal.holding(&[turned_by(0.05, 0.1)]).met(&chain, &q));
        assert!(!goal.holding(&[turned_by(0.5, 0.1)]).met(&chain, &q));
    }

    // The grasp point, and a link fixed below the last joint's link, are solved exactly,
    // aiming at the position and the orientation whatever the goal names; a link the
    // chain's later joints do not move, and a part the last link carries, take approximate
    // ends and aim at what the goal names.
    #[test]
    fn the_tip_and_a_link_fixed_to_it_are_solved_exactly_and_an_inner_link_and_a_part_are_not() {
        let chain = left_arm();
        let offset = Isometry3::translation(0.0, 0.0, 0.1);
        let solved = |frame: Frame| frame.search_of(&chain, Aims::Position);
        let exact = (Aims::Both, Ends::Converged);
        let departing = (Aims::Position, Ends::Approximate);
        assert_eq!(solved(Frame::GraspPoint), exact);
        assert_eq!(solved(Frame::Link { segment: 6, offset }), exact);
        assert_eq!(
            solved(Frame::Link {
                segment: 3,
                offset: Isometry3::identity()
            }),
            departing
        );
        assert_eq!(solved(Frame::Part { segment: 6, offset }), departing);
    }

    // A chain of five joints solves its tip by the options a moveit_config gives it: the
    // position alone for a goal of a position, the position first and approximate ends for
    // a goal of a pose. An inner link departs from MoveIt as on any chain.
    #[test]
    fn a_chain_of_five_joints_takes_the_options_of_a_short_chain() {
        /// The joints of SO-101's arm.
        const SO101_JOINTS: usize = 5;
        let description = bundled("so101");
        let [arm] = description.arms() else {
            panic!("SO-101 has one arm");
        };
        let chain = arm_chain::<SO101_JOINTS>(&description, arm);
        let tip = |named: Aims| Frame::GraspPoint.search_of(&chain, named);
        assert_eq!(tip(Aims::Position), (Aims::Position, Ends::Converged));
        assert_eq!(tip(Aims::Both), (Aims::PositionFirst, Ends::Approximate));
        assert_eq!(
            tip(Aims::Orientation),
            (Aims::Orientation, Ends::Approximate)
        );
        let inner = Frame::Link {
            segment: 1,
            offset: Isometry3::identity(),
        };
        assert_eq!(
            inner.search_of(&chain, Aims::Both),
            (Aims::Both, Ends::Approximate)
        );
    }
}
