//! The inverse kinematics of a frame of a chain, as MoveIt's KDL plugin solves a pose
//! (moveit2 2.15.2 kdl_kinematics_plugin.cpp, at the defaults of its
//! kdl_kinematics_parameters.yaml, and orocos_kdl 1.5.2). A search
//! ([`IkSolver::search`]) descends from a seed toward the frame's target pose, then from
//! configurations drawn inside the joint limits, and answers the first configuration a
//! descent converges to that its caller accepts (`searchPositionIK`, :303-415). A descent
//! is KDL's `CartToJnt` (:418-497):
//!
//! - The frame's miss is KDL's `diff` of its pose to the target: the translation to it,
//!   and the rotation vector of the turn to it in the world frame (frames.inl:1129-1143).
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
//! ends on a budget of passes of KDL's loop, [`SEARCH_ITERATIONS`] being KDL's 0.05 s in
//! passes, and the same seed gives the same answer on every machine.
//!
//! Two of KDL's options are the caller's: what a descent aims at ([`Aims`]) generalizes
//! `position_only_ik`, which drops the orientation rows, and `orientation_vs_position`,
//! which weighs them (kdl_kinematics_plugin.cpp:344-350, :442-443), and which ends a search
//! offers ([`Ends`]) is `return_approximate_solution` (:389).

use nalgebra::{Isometry3, Matrix6, Point3, SMatrix, SVector, Vector3, Vector6};

use crate::{Chain, JointKind, Limit, Posed};

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
/// The off-diagonal entry, as a share of the largest diagonal entry, at or under which a
/// Jacobi sweep leaves a pair unturned: the precision at which Eigen 3.4.0's `JacobiSVD`
/// stops (JacobiSVD.h:671-673).
const JACOBI_PRECISION: f64 = 2.0 * f64::EPSILON;
/// The passes of KDL's `CartToJnt` loop after which a search starts no descent: KDL's
/// 0.05 s over the mean time of a pass in the reference image, 5.39 µs. Measured on
/// jarednm (Intel Core Ultra 9 290HX Plus, amd64, glibc hwcaps masked as run.sh runs the
/// image) on 2026-11-04: 40000 single descents of MoveIt's KDL plugin, 200 from seeds
/// drawn inside the limits toward each of the 200 pose goals of the pose scenario drawn
/// at the launch clearance
/// (`draw_arm_planning.py --poses --drawn 200 --min-clearance 0.04`), took 9.159 s over
/// 1698897 passes, each descent's passes read from the plugin's debug line
/// (kdl_kinematics_plugin.cpp:493-494); per goal, 0.05 s holds 7750 to 10361 passes.
pub const SEARCH_ITERATIONS: usize = 9274;
/// The weight of the orientation rows of a solve that aims at [`Aims::PositionFirst`], KDL's
/// `orientation_vs_position`: the position's rows at 1. The 1e-4 on the squared orientation
/// error of so101_description's solver, on a row. Measured on SO-101's five joints, through
/// the goal sampler at a plan's attempts, of 100 goals at 1 mm: at 0.01 it meets 100 held at
/// 0.02 rad and 100 tilted 0.05 rad and held within 0.1 rad, and at 1 it meets 99 and 87.
const POSITION_FIRST_ORIENTATION_WEIGHT: f64 = 0.01;

/// Where a search draws its numbers: the configurations it restarts from and its steps off
/// a singularity.
pub trait RandomSource {
    /// A number in `[low, high)`.
    fn between(&mut self, low: f64, high: f64) -> f64;
}

/// A configuration inside `limits`, each joint drawn in turn.
pub(crate) fn draw_within<const N: usize>(
    draws: &mut dyn RandomSource,
    limits: &[Limit; N],
) -> [f64; N] {
    std::array::from_fn(|j| draws.between(limits[j].lo, limits[j].hi))
}

/// A frame of a chain.
#[derive(Debug, Clone, Copy)]
pub enum ChainFrame {
    /// The end effector: the mounted tool's control point, or the tip when none is mounted.
    EndEffector,
    /// The frame at `offset` in the link of segment `segment`, the link joint `segment` moves.
    OnSegment {
        segment: usize,
        offset: Isometry3<f64>,
    },
}

impl ChainFrame {
    /// The frame's pose in `posed`, in the world frame.
    pub fn pose_at<const N: usize>(self, posed: &Posed<N>) -> Isometry3<f64> {
        match self {
            Self::EndEffector => posed.base_from_world().inverse() * posed.ee_pose(),
            Self::OnSegment { segment, offset } => posed.link_pose_world(segment) * offset,
        }
    }

    /// Whether every joint of `chain` moves the frame.
    pub fn moved_by_every_joint<const N: usize>(self, chain: &Chain<N>) -> bool {
        self.moving_joints(chain).into_iter().all(|moving| moving)
    }

    /// The joints of `chain` that move the frame: each joint for the end effector, and for
    /// a frame on a segment, the joints on the path from the root to its link.
    pub fn moving_joints<const N: usize>(self, chain: &Chain<N>) -> [bool; N] {
        let Self::OnSegment { segment, .. } = self else {
            return [true; N];
        };
        assert!(segment < N, "segment {segment} out of range (N = {N})");
        let tree = chain.tree();
        let link = tree
            .link_index(chain.at(&[0.0; N]).link_name(segment))
            .expect("a segment's link is a link of the chain's tree");
        let path = tree.path_to(link);
        std::array::from_fn(|j| {
            path.iter()
                .any(|&on| tree.joint(on).name == chain.joint_name(j))
        })
    }
}

/// What a descent brings to the target: the rows of the miss and the Jacobian it keeps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Aims {
    Position,
    Orientation,
    Both,
    /// Both, the orientation's rows weighed low: a chain of fewer joints than a pose has
    /// freedoms reaches the position and turns as near the orientation as it can from there.
    PositionFirst,
}

/// Which ends of its descents a search offers its caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ends {
    /// The configuration each descent converges to.
    Converged,
    /// The configuration each descent ends at, converged or not.
    Approximate,
}

/// Where a descent ends, whether it converged there, and the passes of KDL's loop it made:
/// one for each pose of the frame it evaluated, its step count `i` plus one where it
/// converges or sticks, and `MAX_SOLVER_ITERATIONS` where it reaches the cap (:493).
struct Descent<const N: usize> {
    end: [f64; N],
    converged: bool,
    passes: usize,
}

/// The inverse kinematics of one frame of a chain.
pub struct IkSolver<'a, const N: usize> {
    chain: &'a Chain<N>,
    frame: ChainFrame,
    limits: [Limit; N],
    /// The joints that move the frame.
    moving: [bool; N],
    aims: Aims,
    ends: Ends,
}

impl<'a, const N: usize> IkSolver<'a, N> {
    /// The inverse kinematics of `frame` of `chain`, each descent aiming at `aims` and each
    /// search offering its `ends`.
    pub fn new(chain: &'a Chain<N>, frame: ChainFrame, aims: Aims, ends: Ends) -> Self {
        Self {
            chain,
            frame,
            limits: chain.limits(),
            moving: frame.moving_joints(chain),
            aims,
            ends,
        }
    }

    /// The first configuration that `accept` takes of the ends the search offers of its
    /// descents toward `target`, in the world frame: of a descent from `seed`, then of each
    /// descent from a configuration drawn inside the limits, until the descents have made
    /// `iterations` passes, as KDL's `searchPositionIK` re-seeds after its first attempt and
    /// starts no descent once its time is out (kdl_kinematics_plugin.cpp:369-409), handing
    /// each configuration it finds to its caller's callback (:389-400). None for a seed or a
    /// target that is not finite: the search starts no descent.
    pub fn search(
        &self,
        seed: [f64; N],
        target: &Isometry3<f64>,
        draws: &mut dyn RandomSource,
        accept: &dyn Fn(&[f64; N]) -> bool,
        iterations: usize,
    ) -> Option<[f64; N]> {
        let finite = seed.iter().all(|v| v.is_finite())
            && target.translation.vector.iter().all(|v| v.is_finite())
            && target.rotation.coords.iter().all(|v| v.is_finite());
        if !finite {
            return None;
        }
        let mut from = seed;
        let mut passes = 0;
        loop {
            let descent = self.descend(from, target, draws);
            let offered = descent.converged || self.ends == Ends::Approximate;
            let found = offered.then_some(descent.end).filter(|q| accept(q));
            passes += descent.passes;
            if found.is_some() || passes >= iterations {
                return found;
            }
            from = draw_within(draws, &self.limits);
        }
    }

    /// KDL's `CartToJnt` from `seed` toward `target`, as the module says.
    fn descend(
        &self,
        seed: [f64; N],
        target: &Isometry3<f64>,
        draws: &mut dyn RandomSource,
    ) -> Descent<N> {
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
                if cut == 0.0 {
                    // Cut to nothing: no step from `before` lowers the error.
                    return Descent {
                        end: before,
                        converged: false,
                        passes: pass + 1,
                    };
                }
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
    /// then the rotation vector of the turn to it, in the world frame
    /// (frames.inl:1129-1143).
    /// A component the descent does not aim at reads 0.
    fn miss(&self, posed: &Posed<N>, target: &Isometry3<f64>) -> Vector6<f64> {
        let pose = self.frame.pose_at(posed);
        let moved = target.translation.vector - pose.translation.vector;
        let turned = pose.rotation * (pose.rotation.inverse() * target.rotation).scaled_axis();
        self.aimed(moved, turned)
    }

    /// The six rows of `linear` and `angular`, those the descent does not aim at 0.
    fn aimed(&self, linear: Vector3<f64>, angular: Vector3<f64>) -> Vector6<f64> {
        let (linear, angular) = match self.aims {
            Aims::Position => (linear, Vector3::zeros()),
            Aims::Orientation => (Vector3::zeros(), angular),
            Aims::Both | Aims::PositionFirst => (linear, angular),
        };
        Vector6::new(
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
        )
    }

    /// `ChainIkSolverVelMimicSVD::CartToJnt` (chainiksolver_vel_mimic_svd.cpp:87-130):
    /// the least-squares step of least norm that moves the frame by `miss`, on the
    /// Jacobian whose columns `weights` scale, the singular values under `SVD_THRESHOLD`
    /// of the largest left out, scaled by `weights` again. The plugin weighs position and
    /// orientation alike (`orientation_vs_position` 1,
    /// kdl_kinematics_parameters.yaml:36-46, :344-350), and [`Aims::PositionFirst`] weighs
    /// the orientation's rows of the miss and of the Jacobian at
    /// `POSITION_FIRST_ORIENTATION_WEIGHT` (chainiksolver_vel_mimic_svd.cpp:106-112). The step
    /// is the SVD's, taken from the eigenvectors (`jacobi_eigen`) of the Jacobian times its
    /// transpose, the Jacobian divided by its largest entry as Eigen's `JacobiSVD` divides
    /// the matrix it decomposes (Eigen 3.4.0 JacobiSVD.h:678-697): the eigenvalues are the
    /// squares of the singular values, each kept singular direction `u` with eigenvalue `λ`
    /// adds the divided Jacobian's transpose times `u (u · miss) / λ`, and the sum is
    /// divided by that entry.
    fn velocity_step(
        &self,
        posed: &Posed<N>,
        miss: &Vector6<f64>,
        weights: &SVector<f64, N>,
    ) -> SVector<f64, N> {
        let columns = self.jacobian(posed);
        let row_weight = |row: usize| match self.aims {
            Aims::PositionFirst if row >= 3 => POSITION_FIRST_ORIENTATION_WEIGHT,
            _ => 1.0,
        };
        let miss = Vector6::from_fn(|row, _| miss[row] * row_weight(row));
        let jacobian = SMatrix::<f64, 6, N>::from_fn(|row, column| {
            columns[column][row] * weights[column] * row_weight(row)
        });
        let largest = jacobian.amax();
        let scale = if largest > 0.0 { largest } else { 1.0 };
        let scaled = jacobian / scale;
        // The Jacobian times its transpose, each entry the dot product of two of its rows,
        // whose products commute, so the matrix is symmetric to the bit.
        let gram = Matrix6::from_fn(|i, j| scaled.row(i).dot(&scaled.row(j)));
        let (eigenvalues, eigenvectors) = jacobi_eigen(gram);
        let floor = eigenvalues.max() * SVD_THRESHOLD * SVD_THRESHOLD;
        let inner = eigenvalues
            .iter()
            .zip(eigenvectors.column_iter())
            .filter(|(value, _)| **value > 0.0 && **value >= floor)
            .fold(Vector6::zeros(), |sum, (value, u)| {
                sum + u * (u.dot(&miss) / value)
            });
        (scaled.transpose() * inner / scale).component_mul(weights)
    }

    /// The Jacobian of the frame in `posed`, in the world frame about the frame's origin,
    /// as KDL's `ChainJntToJacSolver` gives the chain's with its reference point at the
    /// tip, in the base frame (chainjnttojacsolver.hpp:45-47): a column of 0 for a joint
    /// that does not move the frame.
    fn jacobian(&self, posed: &Posed<N>) -> [Vector6<f64>; N] {
        let origin = Point3::from(self.frame.pose_at(posed).translation.vector);
        let segment = match self.frame {
            ChainFrame::EndEffector => N,
            ChainFrame::OnSegment { segment, .. } => segment,
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

/// The eigenvalues of the symmetric `matrix` and its eigenvectors, the columns of the
/// second, by cyclic Jacobi sweeps in the order and to the stop of Eigen's `JacobiSVD`
/// (Eigen 3.4.0 JacobiSVD.h:673-676, :704-745): each sweep turns every pair whose
/// off-diagonal entry is over `JACOBI_PRECISION` times the largest diagonal entry, or over
/// the least positive normal number where that is more, by the rotation that zeroes the
/// entry, until a sweep turns none. The rotation's tangent and cosine take their square
/// roots as Eigen's `makeJacobi` takes them (Jacobi.h:109, :120). A turned entry is over
/// `JACOBI_PRECISION` times the diagonal entries, so `tau` is under `1 / JACOBI_PRECISION`
/// in size and its square is finite, and the tangent is within [-1, 1].
fn jacobi_eigen(matrix: Matrix6<f64>) -> (Vector6<f64>, Matrix6<f64>) {
    assert!(
        matrix.iter().all(|value| value.is_finite()),
        "the Jacobian of a finite configuration is finite"
    );
    assert_eq!(matrix, matrix.transpose(), "the matrix is symmetric");
    let mut diagonalized = matrix;
    let mut eigenvectors = Matrix6::identity();
    let mut largest_diagonal = matrix.diagonal().amax();
    loop {
        let mut turned = false;
        for p in 1..6 {
            for q in 0..p {
                let off = diagonalized[(p, q)];
                if off.abs() <= f64::MIN_POSITIVE.max(JACOBI_PRECISION * largest_diagonal) {
                    continue;
                }
                turned = true;
                let tau = (diagonalized[(q, q)] - diagonalized[(p, p)]) / (2.0 * off);
                let tangent = tau.signum() / (tau.abs() + (tau * tau + 1.0).sqrt());
                let cosine = 1.0 / (tangent * tangent + 1.0).sqrt();
                let rotation = (cosine, tangent * cosine);
                turn_symmetric(&mut diagonalized, p, q, rotation);
                turn_columns(&mut eigenvectors, p, q, rotation);
                largest_diagonal = largest_diagonal
                    .max(diagonalized[(p, p)].abs())
                    .max(diagonalized[(q, q)].abs());
            }
        }
        if !turned {
            return (diagonalized.diagonal(), eigenvectors);
        }
    }
}

/// The symmetric `matrix` turned by the rotation of `(cosine, sine)` in the plane of `p`
/// and `q` on both sides, the rotation's transpose times it times the rotation, with the
/// entry at `p` and `q` that the rotation zeroes set to 0. The entries outside the rows and
/// columns of `p` and `q` stand, and the matrix stays symmetric.
fn turn_symmetric(matrix: &mut Matrix6<f64>, p: usize, q: usize, (cosine, sine): (f64, f64)) {
    for i in (0..6).filter(|&i| i != p && i != q) {
        let (at_p, at_q) = (matrix[(i, p)], matrix[(i, q)]);
        let (turned_p, turned_q) = (at_p * cosine - at_q * sine, at_p * sine + at_q * cosine);
        matrix[(i, p)] = turned_p;
        matrix[(p, i)] = turned_p;
        matrix[(i, q)] = turned_q;
        matrix[(q, i)] = turned_q;
    }
    let (pp, pq, qq) = (matrix[(p, p)], matrix[(p, q)], matrix[(q, q)]);
    let (column_pp, column_qp) = (pp * cosine - pq * sine, pq * cosine - qq * sine);
    let (column_pq, column_qq) = (pp * sine + pq * cosine, pq * sine + qq * cosine);
    matrix[(p, p)] = column_pp * cosine - column_qp * sine;
    matrix[(q, q)] = column_pq * sine + column_qq * cosine;
    matrix[(p, q)] = 0.0;
    matrix[(q, p)] = 0.0;
}

/// `matrix` times the rotation of `(cosine, sine)` in the plane of columns `p` and `q`.
fn turn_columns(matrix: &mut Matrix6<f64>, p: usize, q: usize, (cosine, sine): (f64, f64)) {
    for i in 0..6 {
        let (at_p, at_q) = (matrix[(i, p)], matrix[(i, q)]);
        matrix[(i, p)] = at_p * cosine - at_q * sine;
        matrix[(i, q)] = at_p * sine + at_q * cosine;
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, Translation3, UnitQuaternion};
    use openarm_description::{HardwareVersion, Side};

    use super::*;
    use crate::{ChainSpec, JointSelection};

    /// How far the target's orientation stands turned from the seed's: 0.3 rad, thirty
    /// thousand times `EPSILON`.
    const TURN_RAD: f64 = 0.3;
    /// A configuration of OpenArm v2's left arm inside its limits, each joint off its
    /// limits and the elbow (joint 4) bent.
    const SEED: [f64; 7] = [0.3, -0.5, 0.2, 1.2, 0.1, 0.4, 0.2];
    /// Descents of MoveIt's KDL plugin, recorded as the file's provenance says.
    const KDL_DESCENTS: &str = include_str!("../tests/fixtures/kdl_descents.json");
    /// How near KDL's end each descent ends, in radians in every joint: the descents run the
    /// same steps, which round apart by far less.
    const SAME_END_RAD: f64 = 1e-6;

    /// The source of a descent that takes no step off a singularity.
    struct NoDraws;

    impl RandomSource for NoDraws {
        fn between(&mut self, low: f64, high: f64) -> f64 {
            panic!("a draw in [{low}, {high}) of a descent that steps off no singularity")
        }
    }

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

    /// The inverse kinematics of the end effector of `chain`, as KDL solves it.
    fn solver_of(chain: &Chain<7>) -> IkSolver<'_, 7> {
        IkSolver::new(chain, ChainFrame::EndEffector, Aims::Both, Ends::Converged)
    }

    // A descent from a seed whose grasp point stands at the target's position, its
    // orientation turned 0.3 rad from the target's, converges on the position and the
    // orientation together: the configuration it ends at holds the frame within EPSILON
    // of the target's position and of its orientation.
    #[test]
    fn a_descent_converges_on_the_position_and_the_orientation_together() {
        let chain = left_arm();
        let here = chain.world_ee_pose(&SEED);
        let target = Isometry3::from_parts(
            here.translation,
            here.rotation * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), TURN_RAD),
        );
        let descent = solver_of(&chain).descend(SEED, &target, &mut NoDraws);
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

    // Each recorded descent of MoveIt's KDL plugin, from its seed toward its goal's pose,
    // ends within 1e-6 rad of KDL's end in every joint, converged where KDL's converged and
    // after as many passes of the loop: the step cut, the weight of a clipped joint, the
    // stuck rule, the SVD solve's threshold and the cap take the same steps as KDL's.
    #[test]
    fn each_descent_follows_kdls() {
        let chain = left_arm();
        let solver = solver_of(&chain);
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
            let position: [f64; 3] = numbers(&kdl["goal"]["position"])
                .try_into()
                .expect("a position");
            let [x, y, z, w]: [f64; 4] = numbers(&kdl["goal"]["orientation"])
                .try_into()
                .expect("a quaternion");
            let target = Isometry3::from_parts(
                Translation3::from(position),
                UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)),
            );
            let result = kdl["result"].as_i64().expect("KDL's result");
            let steps = kdl["iterations"].as_u64().expect("KDL's step count") as usize;
            let descent = solver.descend(configuration("seed"), &target, &mut NoDraws);
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

    // A configuration draws each joint in turn, inside its limit.
    #[test]
    fn a_configuration_draws_each_joint_in_turn() {
        struct Recording(Vec<(f64, f64)>);
        impl RandomSource for Recording {
            fn between(&mut self, low: f64, high: f64) -> f64 {
                self.0.push((low, high));
                low
            }
        }
        let limits = [
            Limit { lo: -1.0, hi: 1.0 },
            Limit { lo: 0.0, hi: 2.0 },
            Limit { lo: -3.0, hi: -2.0 },
        ];
        let mut draws = Recording(Vec::new());
        assert_eq!(
            draw_within(&mut draws, &limits),
            limits.map(|limit| limit.lo)
        );
        assert_eq!(draws.0, limits.map(|limit| (limit.lo, limit.hi)));
    }
}
