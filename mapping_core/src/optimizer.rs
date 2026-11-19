// Copyright 2018 Simbe Robotics, Inc.
// Author: Steve Macenski (stevenmacenski@gmail.com)
//
// Copyright 2018 Simbe Robotics
// Author: Steve Macenski
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2023 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This file translates into Rust slam_toolbox 2.10.0's Ceres solver plugin:
// solvers/ceres_solver.cpp (Copyright 2018 Simbe Robotics, Inc.) and solvers/ceres_utils.h
// (Copyright 2018 Simbe Robotics), whose angle normalization, angle manifold and error term
// come from Ceres Solver 2.2.0's examples/slam/pose_graph_2d/normalize_angle.h,
// angle_manifold.h and pose_graph_2d_error_term.h (Copyright 2023 Google Inc.); and Ceres
// Solver 2.2.0's Levenberg-Marquardt schedule: internal/ceres/trust_region_minimizer.cc,
// levenberg_marquardt_strategy.cc, trust_region_step_evaluator.cc,
// dynamic_sparse_normal_cholesky_solver.cc, compressed_row_sparse_matrix.cc, problem_impl.cc
// and program.cc, include/ceres/types.h and solver.h (Copyright 2023 Google Inc.).
//
// This translation takes slam_toolbox's own code under the GNU Lesser General Public License,
// version 2.1, of slam_toolbox's LICENSE; as section 3 of that license allows, the GNU General
// Public License, version 3 or later, applies in its place.
//
// Rust translation by Peppy, 2026-11-13.
//
// Copyright 2026 Peppy Inc
//
// This file is part of mapping_core.
//
// mapping_core is free software: you can redistribute it and/or modify it under the terms of
// the GNU General Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// mapping_core is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// mapping_core, in COPYING. If not, see <https://www.gnu.org/licenses/>.

//! The optimization of a pose graph, as slam_toolbox 2.10.0's Ceres solver plugin runs
//! it (solvers/ceres_solver.cpp: `Configure`, `Compute`, `AddNode`, `AddConstraint`,
//! `RemoveNode`, `RemoveConstraint`; solvers/ceres_utils.h: `PoseGraph2dErrorTerm` and
//! `AngleManifold`), by the Levenberg-Marquardt schedule of Ceres 2.2.0 with the options
//! that `Configure` sets (internal/ceres/trust_region_minimizer.cc,
//! levenberg_marquardt_strategy.cc, trust_region_step_evaluator.cc,
//! dynamic_sparse_normal_cholesky_solver.cc). Each pose is three parameters, x, y and a
//! yaw, which the angle manifold moves into [-pi, pi) when a step moves it, and the first
//! pose added stays fixed.
//! Each step solves the normal equations of the Jacobi-scaled Jacobian with the LM
//! diagonal through faer's sparse Cholesky factorization after its approximate minimum
//! degree ordering, where Ceres calls CHOLMOD.

use crate::cholesky::upper_cholesky_factor;
use crate::geometry::{Matrix3, Pose2};
use crate::localized_scan::ScanId;
use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::linalg::cholesky::llt::factor::LltRegularization;
use faer::sparse::linalg::SupernodalThreshold;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, SymmetricOrdering, factorize_symbolic_cholesky,
};
use faer::sparse::{SparseColMat, Triplet};
use faer::{Conj, Mat, Par, Side};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::f64::consts::PI;

/// The options that slam_toolbox's `Configure` sets, and Ceres's own for the rest.
const FUNCTION_TOLERANCE: f64 = 1e-3;
const GRADIENT_TOLERANCE: f64 = 1e-6;
const PARAMETER_TOLERANCE: f64 = 1e-3;
const MAX_NUM_ITERATIONS: u32 = 50;
const MAX_NUM_CONSECUTIVE_INVALID_STEPS: u32 = 3;
/// slam_toolbox sets it to `max_num_consecutive_invalid_steps`.
const MAX_CONSECUTIVE_NONMONOTONIC_STEPS: u32 = MAX_NUM_CONSECUTIVE_INVALID_STEPS;
const MIN_RELATIVE_DECREASE: f64 = 1e-3;
const INITIAL_TRUST_REGION_RADIUS: f64 = 1e4;
const MAX_TRUST_REGION_RADIUS: f64 = 1e8;
const MIN_TRUST_REGION_RADIUS: f64 = 1e-16;
const MIN_LM_DIAGONAL: f64 = 1e-6;
const MAX_LM_DIAGONAL: f64 = 1e32;
/// `LevenbergMarquardtStrategy`'s first factor by which a rejected step shrinks the radius,
/// which doubles at each rejection after it.
const INITIAL_DECREASE_FACTOR: f64 = 2.0;

/// The parameters of a pose, in the order in which `AddResidualBlock` adds them.
const COMPONENTS: usize = 3;
/// The component that the angle manifold keeps.
const YAW: usize = 2;

/// ceres_utils.h's `NormalizeAngle`: `angle` moved by whole turns into [-pi, pi).
fn normalize_angle(angle: f64) -> f64 {
    let two_pi = 2.0 * PI;
    angle - two_pi * ((angle + PI) / two_pi).floor()
}

/// One scalar parameter of the problem: a component of a pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Parameter {
    node: ScanId,
    component: usize,
}

/// `PoseGraph2dErrorTerm`: the error of the pose of `target` in the frame of `source`
/// against the measured `pose_difference`, weighted by the upper Cholesky factor of the
/// measurement's information.
#[derive(Debug, Clone, PartialEq)]
struct ResidualBlock {
    source: ScanId,
    target: ScanId,
    pose_difference: Pose2,
    sqrt_information: [[f64; 3]; 3],
}

impl ResidualBlock {
    /// The parameters of the block, in Ceres's order: the source's x, y and yaw, then the
    /// target's.
    fn parameters(&self) -> [Parameter; 6] {
        let of = |node, component| Parameter { node, component };
        [
            of(self.source, 0),
            of(self.source, 1),
            of(self.source, 2),
            of(self.target, 0),
            of(self.target, 1),
            of(self.target, 2),
        ]
    }

    /// The residuals at the poses `a` (the source) and `b` (the target), and their
    /// derivatives in the six parameters.
    fn evaluate(&self, a: [f64; 3], b: [f64; 3]) -> ([f64; 3], [[f64; 6]; 3]) {
        let (sin, cos) = a[YAW].sin_cos();
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let raw = [
            (cos * dx + sin * dy) - self.pose_difference.x,
            (-sin * dx + cos * dy) - self.pose_difference.y,
            normalize_angle((b[YAW] - a[YAW]) - self.pose_difference.yaw),
        ];
        let raw_jacobian = [
            [-cos, -sin, -sin * dx + cos * dy, cos, sin, 0.0],
            [sin, -cos, -cos * dx - sin * dy, -sin, cos, 0.0],
            [0.0, 0.0, -1.0, 0.0, 0.0, 1.0],
        ];
        let s = &self.sqrt_information;
        let residuals =
            [0, 1, 2].map(|row| s[row][0] * raw[0] + s[row][1] * raw[1] + s[row][2] * raw[2]);
        let jacobian = [0, 1, 2].map(|row| {
            [0, 1, 2, 3, 4, 5].map(|column| {
                s[row][0] * raw_jacobian[0][column]
                    + s[row][1] * raw_jacobian[1][column]
                    + s[row][2] * raw_jacobian[2][column]
            })
        });
        (residuals, jacobian)
    }
}

/// How a solve ended, by Ceres's termination types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Termination {
    Convergence,
    NoConvergence,
    Failure,
}

/// The reduced program of a solve: the parameters that are not fixed, in the problem's
/// order, and the residual blocks that hold one of them.
struct Program<'a> {
    parameters: Vec<Parameter>,
    columns: HashMap<Parameter, usize>,
    blocks: Vec<&'a ResidualBlock>,
}

/// The residuals, the cost, the gradient and the Jacobian of a program at a point.
struct Evaluation {
    cost: f64,
    residuals: Vec<f64>,
    gradient: Vec<f64>,
    /// Each residual row's nonzero entries, by column.
    jacobian: Vec<Vec<(usize, f64)>>,
}

/// `TrustRegionStepEvaluator`: the step quality with nonmonotonic steps.
struct StepEvaluator {
    minimum_cost: f64,
    current_cost: f64,
    reference_cost: f64,
    candidate_cost: f64,
    accumulated_reference_model_cost_change: f64,
    accumulated_candidate_model_cost_change: f64,
    num_consecutive_nonmonotonic_steps: u32,
}

impl StepEvaluator {
    fn new(initial_cost: f64) -> Self {
        Self {
            minimum_cost: initial_cost,
            current_cost: initial_cost,
            reference_cost: initial_cost,
            candidate_cost: initial_cost,
            accumulated_reference_model_cost_change: 0.0,
            accumulated_candidate_model_cost_change: 0.0,
            num_consecutive_nonmonotonic_steps: 0,
        }
    }

    fn step_quality(&self, cost: f64, model_cost_change: f64) -> f64 {
        if cost >= f64::MAX {
            return f64::MIN;
        }
        let relative_decrease = (self.current_cost - cost) / model_cost_change;
        let historical_relative_decrease = (self.reference_cost - cost)
            / (self.accumulated_reference_model_cost_change + model_cost_change);
        relative_decrease.max(historical_relative_decrease)
    }

    fn step_accepted(&mut self, cost: f64, model_cost_change: f64) {
        self.current_cost = cost;
        self.accumulated_candidate_model_cost_change += model_cost_change;
        self.accumulated_reference_model_cost_change += model_cost_change;
        if self.current_cost < self.minimum_cost {
            self.minimum_cost = self.current_cost;
            self.num_consecutive_nonmonotonic_steps = 0;
            self.candidate_cost = self.current_cost;
            self.accumulated_candidate_model_cost_change = 0.0;
        } else {
            self.num_consecutive_nonmonotonic_steps += 1;
            if self.current_cost > self.candidate_cost {
                self.candidate_cost = self.current_cost;
                self.accumulated_candidate_model_cost_change = 0.0;
            }
        }
        if self.num_consecutive_nonmonotonic_steps == MAX_CONSECUTIVE_NONMONOTONIC_STEPS {
            self.reference_cost = self.candidate_cost;
            self.accumulated_reference_model_cost_change =
                self.accumulated_candidate_model_cost_change;
        }
    }
}

/// `LevenbergMarquardtStrategy`: the trust region radius and the LM diagonal.
struct Strategy {
    radius: f64,
    decrease_factor: f64,
    reuse_diagonal: bool,
    diagonal: Vec<f64>,
}

impl Strategy {
    fn new() -> Self {
        Self {
            radius: INITIAL_TRUST_REGION_RADIUS,
            decrease_factor: INITIAL_DECREASE_FACTOR,
            reuse_diagonal: false,
            diagonal: Vec::new(),
        }
    }

    /// `ComputeStep`: the step that solves (J'J + D'D) y = J'r with D the square root of
    /// the clamped squared column norms over the radius, negated, or none when the
    /// matrix is not positive definite or the step is not finite; a fatal linear solve
    /// fails it.
    fn compute_step(
        &mut self,
        jacobian: &[Vec<(usize, f64)>],
        residuals: &[f64],
        columns: usize,
    ) -> Result<Option<Vec<f64>>, LinearSolveFailure> {
        if !self.reuse_diagonal {
            self.diagonal = squared_column_norms(jacobian, columns)
                .into_iter()
                .map(|norm| norm.clamp(MIN_LM_DIAGONAL, MAX_LM_DIAGONAL))
                .collect();
        }
        let lm_diagonal: Vec<f64> = self
            .diagonal
            .iter()
            .map(|value| (value / self.radius).sqrt())
            .collect();
        self.reuse_diagonal = true;
        let step = match solve_normal_equations(jacobian, residuals, &lm_diagonal, columns) {
            Ok(step) => step,
            Err(LinearSolveFailure::NotPositiveDefinite) => return Ok(None),
            Err(LinearSolveFailure::Fatal) => return Err(LinearSolveFailure::Fatal),
        };
        Ok(step
            .iter()
            .all(|value| value.is_finite())
            .then(|| step.into_iter().map(|value| -value).collect()))
    }

    fn step_accepted(&mut self, step_quality: f64) {
        self.radius /= (1.0f64 / 3.0).max(1.0 - (2.0 * step_quality - 1.0).powf(3.0));
        self.radius = self.radius.min(MAX_TRUST_REGION_RADIUS);
        self.decrease_factor = INITIAL_DECREASE_FACTOR;
        self.reuse_diagonal = false;
    }

    fn step_rejected(&mut self) {
        self.radius /= self.decrease_factor;
        self.decrease_factor *= INITIAL_DECREASE_FACTOR;
        self.reuse_diagonal = true;
    }
}

/// `SquaredColumnNorm`.
fn squared_column_norms(jacobian: &[Vec<(usize, f64)>], columns: usize) -> Vec<f64> {
    let mut norms = vec![0.0; columns];
    for row in jacobian {
        for &(column, value) in row {
            norms[column] += value * value;
        }
    }
    norms
}

/// How a linear solve fails, as Ceres's sparse normal Cholesky solver tells its minimizer:
/// a fatal error ends the solve, and a failure makes the step invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LinearSolveFailure {
    /// The matrix or its symbolic factorization could not be made (`FATAL_ERROR`).
    Fatal,
    /// The matrix is not positive definite (`FAILURE`).
    NotPositiveDefinite,
}

/// The solution y of (J'J + diag(d)^2) y = J'r, through faer's simplicial Cholesky
/// factorization after its AMD ordering.
fn solve_normal_equations(
    jacobian: &[Vec<(usize, f64)>],
    residuals: &[f64],
    lm_diagonal: &[f64],
    columns: usize,
) -> Result<Vec<f64>, LinearSolveFailure> {
    let mut rhs = vec![0.0; columns];
    for (row, &residual) in jacobian.iter().zip(residuals) {
        for &(column, value) in row {
            rhs[column] += value * residual;
        }
    }
    let mut upper: BTreeMap<(usize, usize), f64> = BTreeMap::new();
    for row in jacobian {
        for &(i, a) in row {
            for &(j, b) in row {
                if i <= j {
                    *upper.entry((j, i)).or_insert(0.0) += a * b;
                }
            }
        }
    }
    for (column, d) in lm_diagonal.iter().enumerate() {
        *upper.entry((column, column)).or_insert(0.0) += d * d;
    }
    let triplets: Vec<Triplet<usize, usize, f64>> = upper
        .into_iter()
        .map(|((column, row), value)| Triplet::new(row, column, value))
        .collect();
    let matrix = SparseColMat::<usize, f64>::try_new_from_triplets(columns, columns, &triplets)
        .map_err(|_| LinearSolveFailure::Fatal)?;
    let symbolic = factorize_symbolic_cholesky(
        matrix.symbolic(),
        Side::Upper,
        SymmetricOrdering::Amd,
        CholeskySymbolicParams {
            supernodal_flop_ratio_threshold: SupernodalThreshold::FORCE_SIMPLICIAL,
            ..Default::default()
        },
    )
    .map_err(|_| LinearSolveFailure::Fatal)?;
    let mut values = vec![0.0; symbolic.len_val()];
    let mut memory = MemBuffer::new(StackReq::any_of(&[
        symbolic.factorize_numeric_llt_scratch::<f64>(Par::Seq, Default::default()),
        symbolic.solve_in_place_scratch::<f64>(1, Par::Seq),
    ]));
    let stack = MemStack::new(&mut memory);
    let llt = symbolic
        .factorize_numeric_llt(
            &mut values,
            matrix.as_ref(),
            Side::Upper,
            LltRegularization::default(),
            Par::Seq,
            stack,
            Default::default(),
        )
        .map_err(|_| LinearSolveFailure::NotPositiveDefinite)?;
    let mut solution = Mat::<f64>::from_fn(columns, 1, |row, _| rhs[row]);
    llt.solve_in_place_with_conj(Conj::No, solution.as_mut(), Par::Seq, stack);
    Ok((0..columns).map(|row| solution[(row, 0)]).collect())
}

/// slam_toolbox's `CeresSolver`: the poses of the graph's scans, the residual block of
/// each link, and the problem's order of its parameters, as Ceres keeps them.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct Optimizer {
    nodes: BTreeMap<ScanId, [f64; 3]>,
    first_node: Option<ScanId>,
    was_constant_set: bool,
    /// The nodes whose parameters Ceres holds constant.
    constant: Vec<ScanId>,
    /// The problem's parameters, in the order in which residual blocks added them, as
    /// Ceres's removal by swapping with the last one leaves them.
    parameters: Vec<Parameter>,
    /// The parameters of `parameters`, to tell at once whether it holds one.
    parameter_set: HashSet<Parameter>,
    /// The problem's residual blocks, in the order in which they were added, as Ceres's
    /// removal by swapping with the last one leaves them.
    blocks: Vec<ResidualBlock>,
    /// The pose of every node after the last usable solve, until they are taken.
    corrections: Vec<(ScanId, Pose2)>,
}

impl Optimizer {
    /// `AddNode`: the scan's corrected pose, as its node's start; the first node added
    /// stays fixed.
    pub fn add_node(&mut self, id: ScanId, pose: Pose2) {
        self.nodes.entry(id).or_insert([pose.x, pose.y, pose.yaw]);
        if self.nodes.len() == 1 {
            self.first_node = Some(id);
        }
    }

    /// `AddConstraint`: a residual block that measures `target` from `source` by
    /// `pose_difference`, with the inverse of `covariance` as its information. A link
    /// between unknown or equal nodes adds nothing.
    pub fn add_constraint(
        &mut self,
        source: ScanId,
        target: ScanId,
        pose_difference: Pose2,
        covariance: Matrix3,
    ) {
        if source == target
            || !self.nodes.contains_key(&source)
            || !self.nodes.contains_key(&target)
        {
            return;
        }
        let precision = covariance.inverse().0;
        let information = [
            [precision[0][0], precision[0][1], precision[0][2]],
            [precision[0][1], precision[1][1], precision[1][2]],
            [precision[0][2], precision[1][2], precision[2][2]],
        ];
        let block = ResidualBlock {
            source,
            target,
            pose_difference,
            sqrt_information: upper_cholesky_factor(information),
        };
        for parameter in block.parameters() {
            if self.parameter_set.insert(parameter) {
                self.parameters.push(parameter);
            }
        }
        self.blocks.push(block);
    }

    /// `RemoveConstraint`: the residual block of the link from `source` to `target`, or
    /// from `target` to `source`.
    pub fn remove_constraint(&mut self, source: ScanId, target: ScanId) {
        let position = self
            .blocks
            .iter()
            .position(|block| block.source == source && block.target == target)
            .or_else(|| {
                self.blocks
                    .iter()
                    .position(|block| block.source == target && block.target == source)
            });
        if let Some(position) = position {
            self.blocks.swap_remove(position);
        }
    }

    /// Whether the problem holds the node `id`.
    #[cfg(test)]
    pub fn holds(&self, id: ScanId) -> bool {
        self.nodes.contains_key(&id)
    }

    /// `RemoveNode`: the node, its parameters and the residual blocks that hold them. The
    /// blocks go as `RemoveParameterBlock` removes them without fast removal: from the
    /// last block to the first, each swapped with the last. slam_toolbox turns fast
    /// removal on, which takes them in the order of a hash set of their addresses.
    pub fn remove_node(&mut self, id: ScanId) {
        if self.nodes.remove(&id).is_none() {
            return;
        }
        for position in (0..self.blocks.len()).rev() {
            let block = &self.blocks[position];
            if block.source == id || block.target == id {
                self.blocks.swap_remove(position);
            }
        }
        for component in 0..COMPONENTS {
            if let Some(position) = self.parameters.iter().position(|parameter| {
                *parameter
                    == Parameter {
                        node: id,
                        component,
                    }
            }) {
                let removed = self.parameters.swap_remove(position);
                self.parameter_set.remove(&removed);
            }
        }
        self.constant.retain(|node| *node != id);
    }

    /// `Compute`: the first node fixed once the problem holds it, the problem solved,
    /// and, when the solution is usable, the pose of every node as the corrections. With
    /// no node, or when the solve fails, every node stays where it was and the
    /// corrections stay as they were.
    pub fn compute(&mut self) {
        if self.nodes.is_empty() {
            return;
        }
        if let Some(first) = self.first_node {
            let in_problem = (0..COMPONENTS).all(|component| {
                self.parameters.contains(&Parameter {
                    node: first,
                    component,
                })
            });
            if !self.was_constant_set && in_problem {
                self.constant.push(first);
                self.was_constant_set = true;
            }
        }
        let Some(solution) = self.solve() else {
            return;
        };
        for (parameter, value) in solution {
            if let Some(node) = self.nodes.get_mut(&parameter.node) {
                node[parameter.component] = value;
            }
        }
        self.corrections = self
            .nodes
            .iter()
            .map(|(&id, &[x, y, yaw])| (id, Pose2::new(x, y, yaw)))
            .collect();
    }

    /// `GetCorrections` and `Clear`: the corrections, which then empty.
    pub fn take_corrections(&mut self) -> Vec<(ScanId, Pose2)> {
        std::mem::take(&mut self.corrections)
    }

    /// The program of the parameters that are not constant and the residual blocks that
    /// hold one of them, in the problem's order.
    fn program(&self) -> Program<'_> {
        let varies = |parameter: &Parameter| !self.constant.contains(&parameter.node);
        let blocks: Vec<&ResidualBlock> = self
            .blocks
            .iter()
            .filter(|block| block.parameters().iter().any(varies))
            .collect();
        let used: HashSet<Parameter> = blocks.iter().flat_map(|block| block.parameters()).collect();
        let parameters: Vec<Parameter> = self
            .parameters
            .iter()
            .copied()
            .filter(|parameter| varies(parameter) && used.contains(parameter))
            .collect();
        let columns = parameters
            .iter()
            .enumerate()
            .map(|(column, &parameter)| (parameter, column))
            .collect();
        Program {
            parameters,
            columns,
            blocks,
        }
    }

    /// The value of `parameter`, from the state `x` of the program when it varies and
    /// from its node when it is constant.
    fn value(&self, program: &Program, x: &[f64], parameter: Parameter) -> f64 {
        program.columns.get(&parameter).map_or_else(
            || self.nodes[&parameter.node][parameter.component],
            |&column| x[column],
        )
    }

    /// The program's residuals, cost, gradient and Jacobian at `x`, or none when a value
    /// is not finite, which Ceres takes as a failed evaluation.
    fn evaluate(&self, program: &Program, x: &[f64]) -> Option<Evaluation> {
        let pose = |node: ScanId| {
            [0, 1, 2].map(|component| self.value(program, x, Parameter { node, component }))
        };
        let mut cost = 0.0;
        let mut residuals = Vec::with_capacity(program.blocks.len() * 3);
        let mut jacobian = Vec::with_capacity(program.blocks.len() * 3);
        for block in &program.blocks {
            let (block_residuals, block_jacobian) =
                block.evaluate(pose(block.source), pose(block.target));
            let finite = block_residuals.iter().all(|value| value.is_finite())
                && block_jacobian
                    .iter()
                    .flatten()
                    .all(|value| value.is_finite());
            if !finite {
                return None;
            }
            cost += 0.5 * block_residuals.iter().map(|r| r * r).sum::<f64>();
            let parameters = block.parameters();
            for (residual, row) in block_residuals.into_iter().zip(block_jacobian) {
                residuals.push(residual);
                jacobian.push(
                    parameters
                        .iter()
                        .zip(row)
                        .filter_map(|(parameter, value)| {
                            program
                                .columns
                                .get(parameter)
                                .map(|&column| (column, value))
                        })
                        .collect(),
                );
            }
        }
        // ProgramEvaluator::Evaluate: a cost that is not finite fails the evaluation.
        if !cost.is_finite() {
            return None;
        }
        let mut gradient = vec![0.0; program.parameters.len()];
        for (row, &residual) in jacobian.iter().zip(&residuals) {
            for &(column, value) in row {
                gradient[column] += value * residual;
            }
        }
        Some(Evaluation {
            cost,
            residuals,
            gradient,
            jacobian,
        })
    }

    /// `Manifold::Plus` of each parameter: x + delta, and the angle manifold's
    /// normalization for a yaw.
    fn plus(program: &Program, x: &[f64], delta: &[f64]) -> Vec<f64> {
        program
            .parameters
            .iter()
            .zip(x.iter().zip(delta))
            .map(|(parameter, (&value, &step))| {
                if parameter.component == YAW {
                    normalize_angle(value + step)
                } else {
                    value + step
                }
            })
            .collect()
    }

    /// `TrustRegionMinimizer::Minimize` on the program, from the nodes' poses: the
    /// solution, the program's parameters at the iterate of least cost, when the solve
    /// ends usable.
    fn solve(&self) -> Option<Vec<(Parameter, f64)>> {
        let program = self.program();
        if program.parameters.is_empty() {
            return Some(Vec::new());
        }
        let columns = program.parameters.len();
        let mut x: Vec<f64> = program
            .parameters
            .iter()
            .map(|&parameter| self.nodes[&parameter.node][parameter.component])
            .collect();
        let mut evaluation = self.evaluate(&program, &x)?;
        let scaling: Vec<f64> = squared_column_norms(&evaluation.jacobian, columns)
            .into_iter()
            .map(|norm| 1.0 / (1.0 + norm.sqrt()))
            .collect();
        let scale = |jacobian: &mut [Vec<(usize, f64)>]| {
            for row in jacobian.iter_mut() {
                for (column, value) in row.iter_mut() {
                    *value *= scaling[*column];
                }
            }
        };
        scale(&mut evaluation.jacobian);
        let gradient_max_norm = |x: &[f64], gradient: &[f64]| {
            let negative: Vec<f64> = gradient.iter().map(|g| -g).collect();
            Self::plus(&program, x, &negative)
                .iter()
                .zip(x)
                .map(|(moved, value)| (value - moved).abs())
                .fold(0.0, f64::max)
        };
        let mut strategy = Strategy::new();
        let mut step_evaluator = StepEvaluator::new(evaluation.cost);
        let mut best_x = x.clone();
        let mut minimum_cost = f64::MAX;
        let mut iteration = 0;
        let mut step_is_successful = true;
        let mut current_gradient_max_norm = gradient_max_norm(&x, &evaluation.gradient);
        let mut num_consecutive_invalid_steps = 0;
        let mut atleast_one_successful_step = false;
        let termination = loop {
            // FinalizeIterationAndCheckIfMinimizerCanContinue.
            if step_is_successful && evaluation.cost < minimum_cost {
                minimum_cost = evaluation.cost;
                best_x.clone_from(&x);
            }
            if iteration >= MAX_NUM_ITERATIONS {
                break Termination::NoConvergence;
            }
            if step_is_successful && current_gradient_max_norm <= GRADIENT_TOLERANCE {
                break Termination::Convergence;
            }
            if strategy.radius <= MIN_TRUST_REGION_RADIUS {
                break Termination::Convergence;
            }
            iteration += 1;
            step_is_successful = false;
            // ComputeTrustRegionStep.
            let Ok(step) =
                strategy.compute_step(&evaluation.jacobian, &evaluation.residuals, columns)
            else {
                break Termination::Failure;
            };
            let step = step.and_then(|step| {
                let mut model_residuals = vec![0.0; evaluation.residuals.len()];
                for (row, model) in evaluation.jacobian.iter().zip(model_residuals.iter_mut()) {
                    *model = row
                        .iter()
                        .map(|&(column, value)| value * step[column])
                        .sum();
                }
                let model_cost_change = -model_residuals
                    .iter()
                    .zip(&evaluation.residuals)
                    .map(|(model, residual)| model * (residual + model / 2.0))
                    .sum::<f64>();
                (model_cost_change > 0.0).then_some((step, model_cost_change))
            });
            let Some((step, model_cost_change)) = step else {
                // HandleInvalidStep.
                num_consecutive_invalid_steps += 1;
                if num_consecutive_invalid_steps >= MAX_NUM_CONSECUTIVE_INVALID_STEPS {
                    break Termination::Failure;
                }
                strategy.step_rejected();
                continue;
            };
            num_consecutive_invalid_steps = 0;
            let delta: Vec<f64> = step
                .iter()
                .zip(&scaling)
                .map(|(step, scale)| step * scale)
                .collect();
            // ComputeCandidatePointAndEvaluateCost.
            let candidate_x = Self::plus(&program, &x, &delta);
            let candidate_cost = self
                .evaluate(&program, &candidate_x)
                .map_or(f64::MAX, |candidate| candidate.cost);
            // ParameterToleranceReached.
            if atleast_one_successful_step {
                let x_norm = x.iter().map(|value| value * value).sum::<f64>().sqrt();
                let step_norm = x
                    .iter()
                    .zip(&candidate_x)
                    .map(|(value, candidate)| (value - candidate).powi(2))
                    .sum::<f64>()
                    .sqrt();
                if step_norm <= PARAMETER_TOLERANCE * (x_norm + PARAMETER_TOLERANCE) {
                    break Termination::Convergence;
                }
            }
            // FunctionToleranceReached.
            let cost_change = evaluation.cost - candidate_cost;
            if cost_change.abs() <= FUNCTION_TOLERANCE * evaluation.cost {
                break Termination::Convergence;
            }
            // IsStepSuccessful.
            let relative_decrease = step_evaluator.step_quality(candidate_cost, model_cost_change);
            if relative_decrease > MIN_RELATIVE_DECREASE {
                // HandleSuccessfulStep.
                atleast_one_successful_step = true;
                let Some(mut accepted) = self.evaluate(&program, &candidate_x) else {
                    break Termination::Failure;
                };
                scale(&mut accepted.jacobian);
                x = candidate_x;
                current_gradient_max_norm = gradient_max_norm(&x, &accepted.gradient);
                evaluation = accepted;
                step_is_successful = true;
                strategy.step_accepted(relative_decrease);
                step_evaluator.step_accepted(candidate_cost, model_cost_change);
            } else {
                strategy.step_rejected();
            }
        };
        (termination != Termination::Failure)
            .then(|| program.parameters.iter().copied().zip(best_x).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(index: u32) -> ScanId {
        ScanId(index)
    }

    #[test]
    fn a_removed_node_takes_its_blocks_from_the_last_to_the_first() {
        let mut optimizer = Optimizer::default();
        for index in 0..5 {
            optimizer.add_node(id(index), Pose2::new(f64::from(index), 0.0, 0.0));
        }
        let covariance = Matrix3::identity();
        for index in 0..4 {
            optimizer.add_constraint(
                id(index),
                id(index + 1),
                Pose2::new(1.0, 0.0, 0.0),
                covariance,
            );
        }
        optimizer.remove_node(id(1));
        let links: Vec<(ScanId, ScanId)> = optimizer
            .blocks
            .iter()
            .map(|block| (block.source, block.target))
            .collect();
        // Ceres swaps each removed block with the last, from the last block to the first.
        assert_eq!(links, vec![(id(2), id(3)), (id(3), id(4))]);
    }

    #[test]
    fn a_cost_that_is_not_finite_fails_the_evaluation() {
        let mut optimizer = Optimizer::default();
        optimizer.add_node(id(0), Pose2::new(0.0, 0.0, 0.0));
        optimizer.add_node(id(1), Pose2::new(1e160, 0.0, 0.0));
        optimizer.add_constraint(id(0), id(1), Pose2::new(0.0, 0.0, 0.0), Matrix3::identity());
        optimizer.first_node = Some(id(0));
        optimizer.constant = vec![id(0)];
        let program = optimizer.program();
        let x: Vec<f64> = program
            .parameters
            .iter()
            .map(|parameter| optimizer.nodes[&parameter.node][parameter.component])
            .collect();
        // Each residual, 1e160, is finite; its square, the cost, is not.
        assert!(optimizer.evaluate(&program, &x).is_none());
    }

    #[test]
    fn the_angle_of_the_manifold_lies_in_minus_pi_to_pi() {
        assert_eq!(normalize_angle(PI), -PI);
        assert!((normalize_angle(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-15);
    }

    #[test]
    fn a_loop_of_links_moves_the_free_poses_to_agree_with_them() {
        let mut optimizer = Optimizer::default();
        let start = [
            Pose2::new(0.0, 0.0, 0.0),
            Pose2::new(1.1, 0.1, 0.05),
            Pose2::new(0.9, 1.2, 1.6),
        ];
        for (index, pose) in start.into_iter().enumerate() {
            optimizer.add_node(id(index as u32), pose);
        }
        let covariance = Matrix3([[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.001]]);
        optimizer.add_constraint(id(0), id(1), Pose2::new(1.0, 0.0, 0.0), covariance);
        optimizer.add_constraint(id(1), id(2), Pose2::new(0.0, 1.0, PI / 2.0), covariance);
        optimizer.add_constraint(id(0), id(2), Pose2::new(1.0, 1.0, PI / 2.0), covariance);
        optimizer.compute();
        let poses: BTreeMap<ScanId, Pose2> = optimizer.take_corrections().into_iter().collect();
        assert_eq!(poses[&id(0)], start[0], "the first node stays fixed");
        let near = |pose: Pose2, x: f64, y: f64, yaw: f64| {
            (pose.x - x).abs() < 5e-3 && (pose.y - y).abs() < 5e-3 && (pose.yaw - yaw).abs() < 5e-3
        };
        assert!(near(poses[&id(1)], 1.0, 0.0, 0.0), "{:?}", poses[&id(1)]);
        assert!(
            near(poses[&id(2)], 1.0, 1.0, PI / 2.0),
            "{:?}",
            poses[&id(2)]
        );
    }

    #[test]
    fn a_removed_node_takes_its_parameters_and_its_blocks_with_it() {
        let mut optimizer = Optimizer::default();
        for index in 0..3 {
            optimizer.add_node(id(index), Pose2::new(f64::from(index), 0.0, 0.0));
        }
        let covariance = Matrix3::identity();
        optimizer.add_constraint(id(0), id(1), Pose2::new(1.0, 0.0, 0.0), covariance);
        optimizer.add_constraint(id(1), id(2), Pose2::new(1.0, 0.0, 0.0), covariance);
        optimizer.remove_constraint(id(2), id(1));
        optimizer.remove_node(id(2));
        assert_eq!(optimizer.blocks.len(), 1);
        assert_eq!(optimizer.parameters.len(), 6);
        optimizer.compute();
        assert_eq!(optimizer.take_corrections().len(), 2);
        assert!(optimizer.take_corrections().is_empty());
    }
}
