//! workspace_core's reach rule for a solver written in Python: [`reach`] with nothing in the way,
//! and [`PyArmBodies::reach_over`] clear of the block under a surface.

use std::cell::RefCell;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyTuple;
use robot_description::RobotDescription;
use workspace_core::reach::{self as rule, ArmSolver, Attempt, Solves, Unobstructed};
use workspace_core::surface_block::ArmBodies;
use workspace_core::{Limits, Reach};

use crate::{PyReach, PySurfaceHeight, numbers, refusal, three, vector3};

/// What a solver meets of a grasp orientation.
#[pyclass(name = "Solves", module = "workspace_core_py", frozen, eq, eq_int)]
#[derive(Clone, Copy, PartialEq)]
pub enum PySolves {
    /// The full pose: the approach direction and the gripper's roll about it.
    #[pyo3(name = "FULL_POSES")]
    FullPoses,
    /// The approach direction alone, the gripper's roll whatever the arm takes there.
    #[pyo3(name = "APPROACHES")]
    Approaches,
}

impl From<PySolves> for Solves {
    fn from(solves: PySolves) -> Self {
        match solves {
            PySolves::FullPoses => Self::FullPoses,
            PySolves::Approaches => Self::Approaches,
        }
    }
}

/// A solver written in Python, as the rule asks it: its `solves()`, `arm_names()`,
/// `arm_base(arm)`, `reach_bound(arm, target)`, `position_shortfall(arm, target)` and
/// `reach_pose(arm, target, rotation)`,
/// which answers None or `(joints, grasp_point, approach)`, with `joints` joint positions when it
/// is given. The first error a call raises ends the calls: the rule then reads no arm as reaching,
/// and [`Self::finish`] raises the error.
struct PySolver<'py> {
    solver: Bound<'py, PyAny>,
    solves: Solves,
    names: Vec<String>,
    joints: Option<usize>,
    error: RefCell<Option<PyErr>>,
}

impl<'py> PySolver<'py> {
    fn of(solver: &Bound<'py, PyAny>, joints: Option<usize>) -> PyResult<Self> {
        let solves = *solver.call_method0("solves")?.cast::<PySolves>()?.get();
        let names: Vec<String> = solver.call_method0("arm_names")?.extract()?;
        Ok(Self {
            solver: solver.clone(),
            solves: solves.into(),
            names,
            joints,
            error: RefCell::new(None),
        })
    }

    /// The answer of a call when it has one; `fallback` once a call has raised.
    fn answer<T>(&self, answer: impl FnOnce() -> PyResult<T>, fallback: T) -> T {
        if self.error.borrow().is_some() {
            return fallback;
        }
        answer().unwrap_or_else(|error| {
            self.error.borrow_mut().replace(error);
            fallback
        })
    }

    /// The distance the solver's method `method` gives for `target`, of arm `arm`.
    fn distance(&self, method: &str, arm: usize, target: [f64; 3]) -> f64 {
        self.answer(
            || {
                self.solver
                    .call_method1(method, (arm, vector3(target)))?
                    .extract()
            },
            f64::INFINITY,
        )
    }

    /// `reach`, or the first error a call of the solver raised.
    fn finish(self, reach: Reach) -> PyResult<PyReach> {
        self.error.into_inner().map_or(Ok(PyReach(reach)), Err)
    }
}

impl ArmSolver for PySolver<'_> {
    fn solves(&self) -> Solves {
        self.solves
    }

    fn arm_names(&self) -> Vec<String> {
        self.names.clone()
    }

    fn arm_base(&self, arm: usize) -> [f64; 3] {
        self.answer(
            || {
                three(
                    &self.solver.call_method1("arm_base", (arm,))?,
                    "an arm's base",
                )
            },
            [f64::INFINITY; 3],
        )
    }

    fn reach_bound(&mut self, arm: usize, target: [f64; 3]) -> f64 {
        self.distance("reach_bound", arm, target)
    }

    fn position_shortfall(&mut self, arm: usize, target: [f64; 3]) -> f64 {
        self.distance("position_shortfall", arm, target)
    }

    fn reach_pose(&mut self, arm: usize, target: [f64; 3], rotation: [f64; 9]) -> Option<Attempt> {
        self.answer(
            || {
                let rotation = PyTuple::new(self.solver.py(), rotation)?;
                attempt(
                    &self
                        .solver
                        .call_method1("reach_pose", (arm, vector3(target), rotation))?,
                    self.joints,
                )
            },
            None,
        )
    }
}

/// The pose a solver in Python answers: None, or `(joints, grasp_point, approach)`, with `count`
/// joint positions when it is given. Raises ValueError for another count.
fn attempt(pose: &Bound<'_, PyAny>, count: Option<usize>) -> PyResult<Option<Attempt>> {
    if pose.is_none() {
        return Ok(None);
    }
    let (joints, grasp_point, approach): (Bound<'_, PyAny>, Bound<'_, PyAny>, Bound<'_, PyAny>) =
        pose.extract()?;
    let joints = numbers(&joints)?;
    if let Some(count) = count.filter(|&count| count != joints.len()) {
        return Err(PyValueError::new_err(format!(
            "reach_pose gave {} joint positions for an arm of {count} joints",
            joints.len()
        )));
    }
    Ok(Some(Attempt {
        joints,
        grasp_point: three(&grasp_point, "a grasp point")?,
        approach: three(&approach, "an approach")?,
    }))
}

/// Whether an arm of `solver` reaches `target`, three numbers in the robot frame, by
/// workspace_core's rule, with nothing in the way. Raises what a call of the solver raises.
#[pyfunction]
pub fn reach(solver: &Bound<'_, PyAny>, target: &Bound<'_, PyAny>) -> PyResult<PyReach> {
    let target = three(target, "a target")?;
    let mut solver = PySolver::of(solver, None)?;
    let reach = rule::reach(&mut solver, &mut Unobstructed, target, &Limits::STANDARD);
    solver.finish(reach)
}

/// The collision bodies of a robot's arms, from its URDF, its SRDF and the directory of its
/// collision meshes, which a surface holds a reaching arm clear of.
#[pyclass(name = "ArmBodies", module = "workspace_core_py")]
pub struct PyArmBodies(ArmBodies);

#[pymethods]
impl PyArmBodies {
    /// Raises ValueError for a description that does not parse or whose arm bodies do not build.
    #[new]
    fn new(urdf: &str, srdf: &str, meshes_dir: &str) -> PyResult<Self> {
        let description = RobotDescription::parse(urdf, srdf).map_err(refusal)?;
        ArmBodies::new(&description, meshes_dir)
            .map(Self)
            .map_err(refusal)
    }

    /// Whether an arm of `solver` reaches `target`, three numbers in the robot frame, by
    /// workspace_core's rule, in a pose clear of the block under the surface at `surface`.
    /// Raises what a call of the solver raises, and ValueError for a pose of another count of
    /// joints than the arms have.
    fn reach_over(
        &mut self,
        solver: &Bound<'_, PyAny>,
        surface: PyRef<'_, PySurfaceHeight>,
        target: &Bound<'_, PyAny>,
    ) -> PyResult<PyReach> {
        let target = three(target, "a target")?;
        let mut solver = PySolver::of(solver, Some(self.0.joints_per_arm()))?;
        let reach = rule::reach(
            &mut solver,
            &mut self.0.over(surface.0),
            target,
            &Limits::STANDARD,
        );
        solver.finish(reach)
    }
}
