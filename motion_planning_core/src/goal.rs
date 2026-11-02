//! A goal: the constraints a path's last waypoint meets, and the samplers that find
//! configurations meeting them.

use std::sync::atomic::AtomicBool;
use std::time::Instant;

use chain_kinematics::{
    Chain, DEFAULT_DLS_LAMBDA, Goal as PoseGoal, Jacobian, Kinematics, clamp_into,
    damped_pseudo_inverse,
};
use collision_geometry::chain::Rider;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3, Vector6};

use crate::arm::frame_on_segment;
use crate::draws::Draws;
use crate::stop::is_set;

/// Two goal configurations closer than this in every joint, in radians or meters, are
/// one: the sampler keeps the first.
const SAME_CONFIGURATION: f64 = 1e-3;

/// A position tolerance that leaves the position free, for a frame whose goal names
/// only its orientation.
const FREE_POSITION_M: f64 = 1e3;

/// The steps a descent toward a link's pose takes at most.
const DESCENT_STEPS: usize = 200;
/// The turn of a joint (rad, or m for a prismatic joint) over which a descent takes the
/// change of a link's pose.
const DIFFERENCE_STEP: f64 = 1e-7;
/// The most one step of a descent aims to move a link: meters, and radians.
const DESCENT_STEP_M: f64 = 0.05;
const DESCENT_STEP_RAD: f64 = 0.2;
/// How near its target, in meters and radians together, a descent stops.
const DESCENT_REACHED: f64 = 1e-9;

/// A frame of the arm that a target names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Frame {
    /// The chain's grasp point: its tool frame.
    GraspPoint,
    /// A link the child link of joint `segment` carries rigidly through a plan, at
    /// `offset` in that link's frame: the child link itself at the identity, or a part
    /// it carries, such as a gripper finger at the gripper's opening, whose
    /// [`Rider`] gives the frame (`Frame::from`) where the arm's clearance checks place
    /// the part.
    Link {
        segment: usize,
        offset: Isometry3<f64>,
    },
}

impl From<&Rider<'_>> for Frame {
    fn from(rider: &Rider<'_>) -> Self {
        Self::Link {
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
                write!(f, "a part the child link of joint {segment} carries")
            }
        }
    }
}

/// A joint held within `below` under and `above` over `position`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointTarget {
    pub joint: usize,
    pub position: f64,
    pub above: f64,
    pub below: f64,
}

/// A frame's origin held within `tolerance_m` of `position`, in the robot frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionTarget {
    pub frame: Frame,
    pub position: Vector3<f64>,
    pub tolerance_m: f64,
}

impl PositionTarget {
    /// Whether `p` lies within the tolerance of the position.
    fn holds(&self, p: &Vector3<f64>) -> bool {
        (p - self.position).norm() <= self.tolerance_m
    }

    /// A point within the tolerance of the position, drawn uniformly.
    fn draw(&self, draws: &mut Draws) -> Vector3<f64> {
        loop {
            let p = Vector3::from_fn(|_, _| draws.between(-1.0, 1.0));
            if p.norm_squared() <= 1.0 {
                return self.position + p * self.tolerance_m;
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
    pub tolerance_rad: f64,
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
    #[error("a tolerance is a finite number above 0, got {0}")]
    Tolerance(f64),
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
/// orientation of one frame of the arm, either or both.
#[derive(Debug, PartialEq)]
pub struct Goal {
    joints: Vec<JointTarget>,
    position: Option<PositionTarget>,
    orientation: Option<OrientationTarget>,
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
        let tolerance = |t: f64| {
            (t.is_finite() && t > 0.0)
                .then_some(())
                .ok_or(GoalError::Tolerance(t))
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
            Frame::Link { segment, .. } => in_arm(segment),
        };
        for target in &joints {
            in_arm(target.joint)?;
            tolerance(target.above)?;
            tolerance(target.below)?;
        }
        for target in &positions {
            frame(target.frame)?;
            tolerance(target.tolerance_m)?;
            if !target.position.iter().all(|c| c.is_finite()) {
                return Err(GoalError::Position(target.position.into()));
            }
        }
        for target in &orientations {
            frame(target.frame)?;
            tolerance(target.tolerance_rad)?;
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
        })
    }

    /// Whether `chain` at `q` meets every constraint of the goal.
    pub fn met<const N: usize>(&self, chain: &Chain<N>, q: &[f64; N]) -> bool {
        let pose = |frame: Frame| frame_pose(chain, q, frame);
        self.joints
            .iter()
            .all(|t| q[t.joint] <= t.position + t.above && q[t.joint] >= t.position - t.below)
            && self
                .position
                .is_none_or(|t| t.holds(&pose(t.frame).translation.vector))
            && self
                .orientation
                .is_none_or(|t| t.error(&pose(t.frame).rotation) <= t.tolerance_rad)
    }
}

/// The pose of `frame` of `chain` at `q`, in the robot frame.
fn frame_pose<const N: usize>(chain: &Chain<N>, q: &[f64; N], frame: Frame) -> Isometry3<f64> {
    match frame {
        Frame::GraspPoint => chain.world_ee_pose(q),
        Frame::Link { segment, offset } => frame_on_segment(&chain.at(q), segment, &offset),
    }
}

/// What finds configurations that meet a goal.
pub trait GoalSampler<const N: usize>: Sync {
    /// Up to `count` configurations of the arm, inside its joint limits, that meet
    /// `goal`, each drawn from `seed`, the ones drawn before `deadline` and before `stop`,
    /// if any, is set.
    fn sample(
        &self,
        goal: &Goal,
        count: usize,
        seed: u64,
        deadline: Instant,
        stop: Option<&AtomicBool>,
    ) -> Vec<[f64; N]>;
}

/// The goal sampler of any chain: inverse kinematics descends from each of `count`
/// configurations drawn inside the joint limits, the joints a goal names drawn inside
/// their targets, toward a pose of the goal's frame drawn within its position and
/// orientation tolerances, until the deadline passes or the stop flag is set. The chain's
/// own solver reaches for its grasp point, and a damped least-squares descent on the
/// frame's Jacobian for a link.
pub struct IkGoalSampler<const N: usize> {
    chain: Chain<N>,
}

impl<const N: usize> IkGoalSampler<N> {
    pub fn new(chain: Chain<N>) -> Self {
        Self { chain }
    }
}

/// Joint positions from `seed` that bring `frame` of `chain` toward `target`, by damped
/// least squares on the frame's Jacobian, each column taken as the change of the frame's
/// pose over a small turn of its joint. The frame's position counts when `position`
/// holds, and its orientation when `orientation` holds. Each step stays inside the joint
/// limits; the descent ends at the first step that brings the frame no nearer.
fn descend_to<const N: usize>(
    chain: &Chain<N>,
    frame: Frame,
    seed: [f64; N],
    target: &Isometry3<f64>,
    position: bool,
    orientation: bool,
) -> [f64; N] {
    let limits = chain.limits();
    // How far the frame at `q` lies from the target: the translation to it, and the
    // rotation vector of the turn to it.
    let miss = |q: &[f64; N]| -> Vector6<f64> {
        let pose = frame_pose(chain, q, frame);
        let moved = if position {
            target.translation.vector - pose.translation.vector
        } else {
            Vector3::zeros()
        };
        let turned = if orientation {
            (target.rotation * pose.rotation.inverse()).scaled_axis()
        } else {
            Vector3::zeros()
        };
        Vector6::new(moved.x, moved.y, moved.z, turned.x, turned.y, turned.z)
    };
    let mut q = seed;
    let mut here = miss(&q);
    for _ in 0..DESCENT_STEPS {
        if here.norm() <= DESCENT_REACHED {
            break;
        }
        let columns: [Vector6<f64>; N] = std::array::from_fn(|j| {
            let mut turned = q;
            turned[j] += DIFFERENCE_STEP;
            (here - miss(&turned)) / DIFFERENCE_STEP
        });
        let shrink = (DESCENT_STEP_M / here.fixed_rows::<3>(0).norm())
            .min(DESCENT_STEP_RAD / here.fixed_rows::<3>(3).norm())
            .min(1.0);
        let step =
            damped_pseudo_inverse(&Jacobian::<N>::from_columns(&columns), DEFAULT_DLS_LAMBDA)
                * (here * shrink);
        let next = clamp_into(&limits, &std::array::from_fn(|i| q[i] + step[i]));
        let there = miss(&next);
        if there.norm() >= here.norm() {
            break;
        }
        q = next;
        here = there;
    }
    q
}

impl<const N: usize> GoalSampler<N> for IkGoalSampler<N> {
    fn sample(
        &self,
        goal: &Goal,
        count: usize,
        seed: u64,
        deadline: Instant,
        stop: Option<&AtomicBool>,
    ) -> Vec<[f64; N]> {
        let mut draws = Draws::new(seed);
        let limits = self.chain.limits();
        // The frame inverse kinematics reaches for: the one the goal's position and
        // orientation name.
        let posed = goal
            .position
            .map(|t| t.frame)
            .or_else(|| goal.orientation.map(|t| t.frame));
        // The chain's own solver, for its grasp point.
        let kinematics = posed.map(|frame| {
            (
                frame,
                (frame == Frame::GraspPoint).then(|| Kinematics::new(self.chain.clone())),
            )
        });
        let mut kept: Vec<[f64; N]> = Vec::new();
        for _ in 0..count {
            if Instant::now() >= deadline || is_set(stop) {
                break;
            }
            let mut start: [f64; N] =
                std::array::from_fn(|i| draws.between(limits[i].lo, limits[i].hi));
            for t in &goal.joints {
                let low = (t.position - t.below).max(limits[t.joint].lo);
                let high = (t.position + t.above).min(limits[t.joint].hi);
                if low <= high {
                    start[t.joint] = draws.between(low, high);
                }
            }
            let candidate = match &kinematics {
                None => Some(start),
                Some((frame, kinematics)) => {
                    let current = frame_pose(&self.chain, &start, *frame);
                    let (region, orientation) = (goal.position, goal.orientation);
                    let position =
                        region.map_or(current.translation.vector, |t| t.draw(&mut draws));
                    // A free axis turns the target about it by an angle drawn whole.
                    let rotation = orientation.map_or(current.rotation, |t| match t.free_axis {
                        None => t.orientation,
                        Some(axis) => {
                            let turn = draws.between(-std::f64::consts::PI, std::f64::consts::PI);
                            t.orientation
                                * UnitQuaternion::from_axis_angle(
                                    &nalgebra::Unit::new_unchecked(axis.unit()),
                                    turn,
                                )
                        }
                    });
                    let pose = Isometry3::from_parts(Translation3::from(position), rotation);
                    match kinematics {
                        None => Some(descend_to(
                            &self.chain,
                            *frame,
                            start,
                            &pose,
                            region.is_some(),
                            orientation.is_some(),
                        )),
                        Some(kinematics) => {
                            let mut target = PoseGoal::at(pose);
                            if region.is_none() {
                                target = target
                                    .position_within(FREE_POSITION_M)
                                    .expect("a positive tolerance");
                            }
                            if let Some(t) = orientation {
                                target = target
                                    .orientation_within(t.tolerance_rad)
                                    .expect("a goal's tolerance is positive");
                            }
                            kinematics.continue_to(&start, &target)
                        }
                    }
                }
            };
            let Some(q) = candidate else { continue };
            let distinct = kept
                .iter()
                .all(|k| (0..N).any(|i| (k[i] - q[i]).abs() > SAME_CONFIGURATION));
            if distinct && goal.met(&self.chain, &q) {
                kept.push(q);
            }
        }
        kept
    }
}
