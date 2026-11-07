//! A Cartesian move: the end effector's straight line from its pose to the goal,
//! walked by `chain_kinematics`' KDL search in MoveIt's steps and within MoveIt's
//! Cartesian precision. [`plan_line`] gives the configurations of the walk, which a
//! caller times and plays as a joint path, so the joint limits hold at every point of
//! the move.

use std::cell::RefCell;

use chain_kinematics::{
    Aims, ChainFrame, Draws, Ends, IkSolver, LineSolver, LineState, PRECISION, WalkEnd,
    interpolate, line_blends, walk_line,
};
use nalgebra::Isometry3;

use crate::{ARM_DOF, Arm, JointVec};

/// The passes a step's search may make: its first descent, from the state before, ends
/// the search, so a step continues that state and draws no far configuration.
const ONE_DESCENT: usize = 1;

/// The seed of the draws a descent steps off a singularity with, so a plan is a function
/// of its arguments.
const DRAWS_SEED: u64 = 1;

/// A step of the walk: one descent from the state before.
struct Descending<'a> {
    arm: &'a Arm,
    ik: IkSolver<'a, ARM_DOF>,
    draws: RefCell<Draws>,
}

impl LineSolver<ARM_DOF> for Descending<'_> {
    fn solve(
        &self,
        pose: &Isometry3<f64>,
        from: &LineState<ARM_DOF>,
        _to: f64,
    ) -> Option<JointVec> {
        self.ik.search(
            from.q,
            pose,
            &mut *self.draws.borrow_mut(),
            &|_| true,
            ONE_DESCENT,
        )
    }

    fn pose(&self, q: &JointVec) -> Isometry3<f64> {
        self.arm.world_ee_pose(q)
    }
}

/// The configurations of the straight line from the pose at `seed` to `end`, `seed`
/// first and the end effector within `PRECISION` of the line at each and at the midpoint
/// between each two; `None` when a step has no solution continuing the one before it, or cannot keep the
/// precision. `end` is in the world frame.
pub fn plan_line(arm: &Arm, end: &Isometry3<f64>, seed: JointVec) -> Option<Vec<JointVec>> {
    let start = arm.world_ee_pose(&seed);
    let solver = Descending {
        arm,
        ik: IkSolver::new(
            arm.chain(),
            ChainFrame::EndEffector,
            Aims::Both,
            Ends::Converged,
        ),
        draws: RefCell::new(Draws::new(DRAWS_SEED)),
    };
    let mut walk = vec![seed];
    let ended = walk_line(
        &solver,
        |blend| interpolate(&start, end, blend),
        LineState {
            blend: 0.0,
            q: seed,
        },
        line_blends(&start, end),
        Some(&PRECISION),
        |_, step| {
            walk.extend(step.iter().map(|state| state.q));
            true
        },
    );
    (ended == WalkEnd::Whole).then_some(walk)
}
