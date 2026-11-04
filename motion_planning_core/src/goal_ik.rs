//! The inverse kinematics of a goal's frame, as MoveIt's KDL plugin solves a pose
//! (moveit2 2.15.2 kdl_kinematics_plugin.cpp, at the defaults of its
//! kdl_kinematics_parameters.yaml, and orocos_kdl 1.5.2). A search descends from a seed
//! toward the frame's target pose, then from configurations drawn inside the joint
//! limits, and answers the first configuration a descent converges to that its caller
//! accepts (`searchPositionIK`, :303-415), or a descent ends at, for a frame that the
//! last paragraph names. A descent is KDL's `CartToJnt` (:418-497):
//!
//! - The frame's miss is KDL's `diff` of its pose to the target: the translation to it,
//!   and the rotation vector of the turn to it in the robot frame (frames.inl:1129-1143).
//!   The descent converges where the larger of the two norms is within `EPSILON`
//!   (:441-449).
//! - A step that lowers that error is taken from the configuration it reached: the SVD
//!   solve of the frame's Jacobian for the miss (`velocity_step`). A step that does not
//!   lower it is cut, to `STEP_CUT` of itself or to the ratio of the two errors where
//!   that is less, from the configuration before it (:450-460).
//! - A step is clipped to the joint limits, and the next solve weighs each clipped joint
//!   at `CLIPPED_JOINT_WEIGHT` (`clipToJointLimits`, :470, :500-521).
//! - A step under `EPSILON` in the sum of its joints ends a descent whose step was cut
//!   under `EPSILON`; any other draws a step off the singularity, each joint in `[-1, 1]`
//!   times `WIGGLE` or the error where that is less (:472-485).
//! - A descent takes at most `MAX_SOLVER_ITERATIONS` steps (:387, :493).
//!
//! KDL's search ends on a clock: the 0.05 s of moveit_ros_planning's
//! `kinematics_solver_timeout` default (kinematics_parameters.yaml:15-18), which
//! move_group's `RobotModelLoader::loadKinematicsSolvers` sets on each group
//! (robot_model_loader.cpp:310-318) and the constraint sampler gives each inverse
//! kinematics call (default_constraint_samplers.cpp:268). A step of a descent takes a
//! near constant time, and a descent ends in a few steps or at the cap, so this search
//! ends once its descents have made `SEARCH_ITERATIONS` passes of KDL's loop, KDL's 0.05
//! s in passes, and the same seed gives the same answer on every machine.
//!
//! A frame MoveIt's constraint sampler solves for is the tip of its solver's chain or a
//! link fixed to it (`IKConstraintSampler::loadIKSolver`,
//! default_constraint_samplers.cpp:386-439): the grasp point, or a link that the chain's
//! every joint moves and that is fixed below the last joint's link. The search solves for
//! it as KDL does. Two frames depart from MoveIt, which has no inverse kinematics for
//! them (`crate::goal::IkGoalSampler` says what MoveIt does instead): a link the chain's
//! later joints do not move, which fewer joints than the chain's move, so it may reach no
//! pose drawn inside a goal exactly; and a part a link carries, such as a finger, which
//! hangs on a joint of its own that the chain does not hold fixed, so MoveIt finds no
//! fixed transform from it to the tip (`getAssociatedFixedTransforms`, :395). For them
//! the descent aims only at what the goal names, the position, the orientation or both,
//! with KDL's `position_only_ik` option generalized, which drops the orientation rows
//! (kdl_kinematics_plugin.cpp:344-350, :442-443), and the search hands the caller the
//! configuration each descent ends at, converged or not, with KDL's
//! `return_approximate_solution` option (:389).

use chain_kinematics::{Chain, JointKind, Limit, Posed};
use nalgebra::{Isometry3, Point3, SMatrix, SVector, SymmetricEigen, Vector3, Vector6};

use crate::draws::Draws;
use crate::goal::{Frame, frame_pose_at};

/// The error at which a descent converges, in meters and radians: KDL's `epsilon` default
/// (kdl_kinematics_parameters.yaml:27-31), which also bounds a stuck step (:475-477).
const EPSILON: f64 = 1e-5;
/// The steps a descent takes at most: KDL's `max_solver_iterations` default
/// (kdl_kinematics_parameters.yaml:18-22).
const MAX_SOLVER_ITERATIONS: usize = 500;
/// The most a step that raised the error keeps of itself (kdl_kinematics_plugin.cpp:455).
const STEP_CUT: f64 = 0.2;
/// The weight of a joint the step before clipped at its limit
/// (kdl_kinematics_plugin.cpp:520).
const CLIPPED_JOINT_WEIGHT: f64 = 0.01;
/// The most a step off a singularity moves a joint, in radians or meters
/// (kdl_kinematics_plugin.cpp:482).
const WIGGLE: f64 = 0.1;
/// The share of the largest singular value under which the solve sets a singular value's
/// inverse to 0: `ChainIkSolverVelMimicSVD`'s default threshold, 0.001
/// (chainiksolver_vel_mimic_svd.hpp:63), at which the plugin builds its solver
/// (kdl_kinematics_plugin.cpp:358), and Eigen 3's `rank` keeps the singular values at or
/// above it times the largest (SVDBase.h:148-157).
const SVD_THRESHOLD: f64 = 1e-3;
/// The passes of KDL's `CartToJnt` loop after which a search starts no descent: KDL's
/// 0.05 s over the mean time of a pass in the reference image, 5.39 µs. Measured on
/// jarednm (Intel Core Ultra 9 290HX Plus, amd64, glibc hwcaps masked as run.sh runs the
/// image) on 2026-11-04: 40000 single descents of MoveIt's KDL plugin, 200 from seeds
/// drawn inside the limits toward each of the 200 pose goals of the pose scenario drawn
/// at the launch clearance
/// (`draw_arm_planning.py --poses --drawn 200 --min-clearance 0.04`), took 9.159 s over
/// 1698897 passes, each descent's passes read from the plugin's debug line
/// (kdl_kinematics_plugin.cpp:493-494); per goal, 0.05 s holds 7750 to 10361 passes.
const SEARCH_ITERATIONS: usize = 9274;

/// Where a descent ends, whether it converged there, and the passes of KDL's loop it made:
/// one for each pose of the frame it evaluated, its step count `i` plus one where it
/// converges or sticks, and `MAX_SOLVER_ITERATIONS` where it reaches the cap (:493).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Descent<const N: usize> {
    end: [f64; N],
    converged: bool,
    passes: usize,
}

/// What a descent brings to the target: the frame's position, its orientation, or both.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Aims {
    pub(crate) position: bool,
    pub(crate) orientation: bool,
}

/// The inverse kinematics of one frame of a chain.
pub(crate) struct GoalIk<'a, const N: usize> {
    chain: &'a Chain<N>,
    frame: Frame,
    limits: [Limit; N],
    /// The joints that move the frame: each joint of the chain for the grasp point, and
    /// for a link or a part, the joints on the path from the root to the link that
    /// carries it.
    moving: [bool; N],
    /// What each descent brings to the target, as the module says.
    aims: Aims,
    /// Whether the search hands its caller the end of a descent that did not converge.
    approximate: bool,
}

impl<'a, const N: usize> GoalIk<'a, N> {
    /// The inverse kinematics of `frame` of `chain`, for a goal that names what `named`
    /// holds, as the module says.
    pub(crate) fn new(chain: &'a Chain<N>, frame: Frame, named: Aims) -> Self {
        let posed = chain.at(&[0.0; N]);
        let actuated = |j: usize| !matches!(posed.kind(j), JointKind::Fixed);
        let moving: [bool; N] = match frame {
            Frame::GraspPoint => std::array::from_fn(actuated),
            Frame::Link { segment, .. } | Frame::Part { segment, .. } => {
                let tree = chain.tree();
                let carrier = tree
                    .link_index(posed.link_name(segment))
                    .expect("a segment's link is a link of the chain's tree");
                let path = tree.path_to(carrier);
                std::array::from_fn(|j| {
                    actuated(j)
                        && path
                            .iter()
                            .any(|&on| tree.joint(on).name == chain.joint_name(j))
                })
            }
        };
        let moved_by_every_joint = (0..N).all(|j| moving[j] || !actuated(j));
        let exact = match frame {
            Frame::GraspPoint => true,
            Frame::Link { .. } => moved_by_every_joint,
            Frame::Part { .. } => false,
        };
        let (aims, approximate) = if exact {
            let both = Aims {
                position: true,
                orientation: true,
            };
            (both, false)
        } else {
            (named, true)
        };
        Self {
            chain,
            frame,
            limits: chain.limits(),
            moving,
            aims,
            approximate,
        }
    }

    /// The first configuration that `accept` takes of those the descents toward `target`
    /// converge to, or, for a frame that takes approximate ends as the module says, end
    /// at: of a descent from `seed`, then of each descent from a configuration drawn
    /// inside the limits, until the descents have made `SEARCH_ITERATIONS` passes, as
    /// KDL's `searchPositionIK` re-seeds after its first attempt and starts no descent
    /// once its time is out (kdl_kinematics_plugin.cpp:369-409), handing each
    /// configuration it finds to its caller's callback (:389-400).
    pub(crate) fn search(
        &self,
        seed: [f64; N],
        target: &Isometry3<f64>,
        draws: &mut Draws,
        accept: &dyn Fn(&[f64; N]) -> bool,
    ) -> Option<[f64; N]> {
        let mut from = seed;
        let mut passes = 0;
        loop {
            let descent = self.descend(from, target, draws);
            let found = (descent.converged || self.approximate)
                .then_some(descent.end)
                .filter(|q| accept(q));
            passes += descent.passes;
            if found.is_some() || passes >= SEARCH_ITERATIONS {
                return found;
            }
            from = draws.within(&self.limits);
        }
    }

    /// KDL's `CartToJnt` from `seed` toward `target`, as the module says.
    fn descend(&self, seed: [f64; N], target: &Isometry3<f64>, draws: &mut Draws) -> Descent<N> {
        let mut q = seed;
        // KDL's `q_backup` and `delta_q` start at 0 (:425).
        let mut before = [0.0; N];
        let mut step = SVector::<f64, N>::zeros();
        let mut weights = SVector::<f64, N>::repeat(1.0);
        let mut last_error = f64::MAX;
        let mut step_size = 1.0;
        for pass in 0..MAX_SOLVER_ITERATIONS {
            let posed = self.chain.at(&q);
            let miss = self.miss(&posed, target);
            let error = miss
                .fixed_rows::<3>(0)
                .norm()
                .max(miss.fixed_rows::<3>(3).norm());
            if error <= EPSILON {
                return Descent {
                    end: q,
                    converged: true,
                    passes: pass + 1,
                };
            }
            if error >= last_error {
                let cut = step_size * STEP_CUT.min(last_error / error);
                step *= cut / step_size;
                step_size = cut;
                q = before;
            } else {
                before = q;
                step_size = 1.0;
                last_error = error;
                step = self.velocity_step(&posed, &miss, &weights);
            }
            (step, weights) = self.clipped(&q, step);
            if step.lp_norm(1) < EPSILON {
                if step_size < EPSILON {
                    return Descent {
                        end: q,
                        converged: false,
                        passes: pass + 1,
                    };
                }
                last_error = f64::MAX;
                let off =
                    SVector::<f64, N>::from_fn(|_, _| draws.between(-1.0, 1.0)) * WIGGLE.min(error);
                (step, _) = self.clipped(&q, off);
                weights = SVector::repeat(1.0);
            }
            q = std::array::from_fn(|i| q[i] + step[i]);
        }
        Descent {
            end: q,
            converged: false,
            passes: MAX_SOLVER_ITERATIONS,
        }
    }

    /// KDL's `diff` of the frame's pose in `posed` to `target`: the translation to it,
    /// then the rotation vector of the turn to it, in the robot frame
    /// (frames.inl:1129-1143).
    /// A component the descent does not aim at reads 0.
    fn miss(&self, posed: &Posed<N>, target: &Isometry3<f64>) -> Vector6<f64> {
        let pose = frame_pose_at(posed, self.frame);
        let moved = target.translation.vector - pose.translation.vector;
        let turned = pose.rotation * (pose.rotation.inverse() * target.rotation).scaled_axis();
        self.aimed(moved, turned)
    }

    /// The six rows of `linear` and `angular`, those the descent does not aim at 0.
    fn aimed(&self, linear: Vector3<f64>, angular: Vector3<f64>) -> Vector6<f64> {
        let (linear, angular) = (
            if self.aims.position {
                linear
            } else {
                Vector3::zeros()
            },
            if self.aims.orientation {
                angular
            } else {
                Vector3::zeros()
            },
        );
        Vector6::new(
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
        )
    }

    /// `ChainIkSolverVelMimicSVD::CartToJnt` (chainiksolver_vel_mimic_svd.cpp:87-130):
    /// the least-squares step of least norm that moves the frame by `miss`, on the
    /// Jacobian whose columns `weights` scale, the singular values under `SVD_THRESHOLD`
    /// of the largest left out, scaled by `weights` again. The plugin weighs position and
    /// orientation alike (`orientation_vs_position` 1,
    /// kdl_kinematics_parameters.yaml:36-46, :344-350). The step is the SVD's, taken from
    /// the eigenvectors of the Jacobian times its transpose, whose eigenvalues are the
    /// squares of the singular values: each kept singular direction `u` with eigenvalue
    /// `λ` adds the Jacobian's transpose times `u (u · miss) / λ`.
    fn velocity_step(
        &self,
        posed: &Posed<N>,
        miss: &Vector6<f64>,
        weights: &SVector<f64, N>,
    ) -> SVector<f64, N> {
        let columns = self.jacobian(posed);
        let jacobian =
            SMatrix::<f64, 6, N>::from_fn(|row, column| columns[column][row] * weights[column]);
        let eigen = SymmetricEigen::new(jacobian * jacobian.transpose());
        let floor = eigen.eigenvalues.max() * SVD_THRESHOLD * SVD_THRESHOLD;
        let inner = eigen
            .eigenvalues
            .iter()
            .zip(eigen.eigenvectors.column_iter())
            .filter(|(value, _)| **value > 0.0 && **value >= floor)
            .fold(Vector6::zeros(), |sum, (value, u)| {
                sum + u * (u.dot(miss) / value)
            });
        (jacobian.transpose() * inner).component_mul(weights)
    }

    /// The Jacobian of the frame in `posed`, in the robot frame about the frame's origin,
    /// as KDL's `ChainJntToJacSolver` gives the chain's with its reference point at the
    /// tip, in the base frame (chainjnttojacsolver.hpp:45-47): a column of 0 for a joint
    /// that does not move the frame.
    fn jacobian(&self, posed: &Posed<N>) -> [Vector6<f64>; N] {
        let origin = Point3::from(frame_pose_at(posed, self.frame).translation.vector);
        let segment = match self.frame {
            Frame::GraspPoint => N,
            Frame::Link { segment, .. } | Frame::Part { segment, .. } => segment,
        };
        let linear = posed.point_world_jacobian(&origin, segment);
        std::array::from_fn(|j| {
            let turn = match posed.kind(j) {
                JointKind::Revolute { .. } if self.moving[j] => posed.axis_world(j),
                _ => Vector3::zeros(),
            };
            let along = if self.moving[j] {
                linear[j]
            } else {
                Vector3::zeros()
            };
            self.aimed(along, turn)
        })
    }

    /// `step` clipped to the joint limits from `q`, and the weight of each joint in the
    /// next solve: `CLIPPED_JOINT_WEIGHT` for a clipped one, 1 for the others
    /// (`clipToJointLimits`, kdl_kinematics_plugin.cpp:500-521).
    fn clipped(&self, q: &[f64; N], step: SVector<f64, N>) -> (SVector<f64, N>, SVector<f64, N>) {
        let limits = &self.limits;
        let clip = |j: usize| {
            let (above, below) = (limits[j].hi - q[j], limits[j].lo - q[j]);
            if step[j] > above {
                (above, CLIPPED_JOINT_WEIGHT)
            } else if step[j] < below {
                (below, CLIPPED_JOINT_WEIGHT)
            } else {
                (step[j], 1.0)
            }
        };
        (
            SVector::from_fn(|j, _| clip(j).0),
            SVector::from_fn(|j, _| clip(j).1),
        )
    }
}

#[cfg(test)]
mod tests {
    use chain_kinematics::{ChainSpec, JointSelection};
    use nalgebra::UnitQuaternion;
    use openarm_description::{HardwareVersion, Side};

    use super::*;

    /// How far the target's orientation stands turned from the seed's: 0.3 rad, thirty
    /// thousand times `EPSILON`.
    const TURN_RAD: f64 = 0.3;

    /// A configuration of OpenArm v2's left arm inside its limits, each joint off its
    /// limits and the elbow (joint 4) bent.
    const SEED: [f64; 7] = [0.3, -0.5, 0.2, 1.2, 0.1, 0.4, 0.2];

    const BOTH: Aims = Aims {
        position: true,
        orientation: true,
    };

    /// OpenArm v2's left arm from its base to its grasp point.
    fn left_arm() -> Chain<7> {
        let version = HardwareVersion::V2;
        Chain::from_urdf(
            &urdf_rs::read_from_string(version.urdf()).expect("the URDF parses"),
            &ChainSpec {
                base_link: Some(version.base_link(Side::Left)),
                tip_link: version.tcp_link(Side::Left),
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the left arm")
    }

    // A descent from a seed whose grasp point stands at the target's position, its
    // orientation turned 0.3 rad from the target's, converges on the position and the
    // orientation together: the configuration it ends at holds the frame within EPSILON
    // of the target's position and of its orientation.
    #[test]
    fn a_descent_converges_on_the_position_and_the_orientation_together() {
        let chain = left_arm();
        let ik = GoalIk::new(&chain, Frame::GraspPoint, BOTH);
        let here = chain.world_ee_pose(&SEED);
        let target = Isometry3::from_parts(
            here.translation,
            here.rotation * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), TURN_RAD),
        );
        let descent = ik.descend(SEED, &target, &mut Draws::new(1));
        let (end, reached) = (descent.end, chain.world_ee_pose(&descent.end));
        assert!(descent.converged, "{end:?}");
        assert!(
            (reached.translation.vector - target.translation.vector).norm() <= EPSILON,
            "{reached:?}"
        );
        assert!(
            reached.rotation.angle_to(&target.rotation) <= EPSILON,
            "{reached:?}"
        );
    }

    // The grasp point, and a link fixed below the last joint's link, are solved exactly,
    // aiming at the position and the orientation whatever the goal names; a link the
    // chain's later joints do not move, and a part the last link carries, take approximate
    // ends and aim at what the goal names.
    #[test]
    fn the_tip_and_a_link_fixed_to_it_are_solved_exactly_and_an_inner_link_and_a_part_are_not() {
        let chain = left_arm();
        let position_only = Aims {
            position: true,
            orientation: false,
        };
        let offset = Isometry3::translation(0.0, 0.0, 0.1);
        let solved = |frame: Frame| {
            let ik = GoalIk::new(&chain, frame, position_only);
            (ik.approximate, ik.aims.position, ik.aims.orientation)
        };
        let exact = (false, true, true);
        let departing = (true, true, false);
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

    /// Descents of MoveIt's KDL plugin, recorded as the file's provenance says.
    const KDL_DESCENTS: &str = include_str!("../tests/kdl_descents.json");

    /// How near KDL's end each descent ends, in radians in every joint: the descents run the
    /// same steps, which round apart by far less.
    const SAME_END_RAD: f64 = 1e-6;

    // Each recorded descent of MoveIt's KDL plugin, from its seed toward its goal's pose,
    // ends within 1e-6 rad of KDL's end in every joint, converged where KDL's converged and
    // after as many passes of the loop: the step cut, the weight of a clipped joint, the
    // stuck rule, the SVD solve's threshold and the cap take the same steps as KDL's.
    #[test]
    fn each_descent_follows_kdls() {
        let chain = left_arm();
        let ik = GoalIk::new(&chain, Frame::GraspPoint, BOTH);
        let recorded: serde_json::Value =
            serde_json::from_str(KDL_DESCENTS).expect("the recorded descents parse");
        let numbers = |value: &serde_json::Value| -> Vec<f64> {
            value
                .as_array()
                .expect("an array")
                .iter()
                .map(|v| v.as_f64().expect("a number"))
                .collect()
        };
        let descents = recorded["descents"].as_array().expect("the descents");
        assert!(!descents.is_empty());
        for kdl in descents {
            let configuration =
                |key: &str| -> [f64; 7] { numbers(&kdl[key]).try_into().expect("seven joints") };
            let (position, orientation) = (
                numbers(&kdl["goal"]["position"]),
                numbers(&kdl["goal"]["orientation"]),
            );
            let target = crate::pose::pose_from_parts(
                position.try_into().expect("a position"),
                orientation.try_into().expect("a quaternion"),
            )
            .expect("the goal's pose");
            let result = kdl["result"].as_i64().expect("KDL's result");
            let steps = kdl["iterations"].as_u64().expect("KDL's step count") as usize;
            let descent = ik.descend(configuration("seed"), &target, &mut Draws::new(0));
            let end = configuration("end");
            assert_eq!(descent.converged, result == 0, "{kdl}");
            let passes = if result == -3 {
                MAX_SOLVER_ITERATIONS
            } else {
                steps + 1
            };
            assert_eq!(descent.passes, passes, "{kdl}");
            assert!(
                (0..7).all(|i| (descent.end[i] - end[i]).abs() <= SAME_END_RAD),
                "{:?} against {kdl}",
                descent.end
            );
        }
    }
}
