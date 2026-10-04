//! The Python bindings of `workspace_core`: the extension module that
//! Python imports as `workspace_core_py`. Its types are in
//! `workspace_core_py.pyi`.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyTuple;
use workspace_core::design::{
    self, PointAnswer, Positions, PositionsAnswer, PositionsReach, ReachMemo, SurfaceAnswer,
    SurfaceHeight, SurfaceReach, ViewCheck,
};
use workspace_core::{
    ABOVE_SURFACE, GRASP_ANGLE_TOLERANCE, GraspDirection, REACH_TOLERANCE, Reach,
};

/// A point or a direction, `(x, y, z)`.
type Vector3 = (f64, f64, f64);

/// The bounds of a rectangle, `(x_min, x_max, y_min, y_max)`.
type Bounds = (f64, f64, f64, f64);

/// The Python bindings of workspace_core, for a robot's backbone in Python:
/// what it needs to give the workspace:v1 answer of a robot without a
/// perception camera, around the reach its own solver gives. The limits, the
/// grasp directions, the parsing of a request, the grid targets of a surface,
/// the reach memo, and the answers with their messages.
#[pymodule]
fn workspace_core_py(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add("REACH_TOLERANCE", REACH_TOLERANCE)?;
    module.add("GRASP_ANGLE_TOLERANCE", GRASP_ANGLE_TOLERANCE)?;
    module.add("ABOVE_SURFACE", ABOVE_SURFACE)?;
    module.add(
        "GRASP_DIRECTIONS",
        PyTuple::new(module.py(), GraspDirection::ALL.map(PyGraspDirection))?,
    )?;
    module.add_class::<PyGraspDirection>()?;
    module.add_class::<PySurfaceHeight>()?;
    module.add_class::<PyPositions>()?;
    module.add_class::<PyReach>()?;
    module.add_class::<PySurfaceReach>()?;
    module.add_class::<PyReachMemo>()?;
    module.add_class::<PySurfaceAnswer>()?;
    module.add_class::<PyPointAnswer>()?;
    module.add_class::<PyPositionsAnswer>()?;
    module.add_function(wrap_pyfunction!(angle_between, module)?)?;
    module.add_function(wrap_pyfunction!(
        describe_without_perception_camera,
        module
    )?)?;
    module.add_function(wrap_pyfunction!(check_without_perception_camera, module)?)?;
    Ok(())
}

/// A way the gripper points when it grasps, in the robot frame.
#[pyclass(
    name = "GraspDirection",
    module = "workspace_core_py",
    frozen,
    eq,
    skip_from_py_object
)]
#[derive(Clone, Copy, PartialEq)]
struct PyGraspDirection(GraspDirection);

#[pymethods]
impl PyGraspDirection {
    /// The direction's name: `down` or `forward`.
    #[getter]
    fn name(&self) -> &'static str {
        self.0.name()
    }

    /// The unit direction the gripper approaches in, in the robot frame.
    #[getter]
    fn approach(&self) -> Vector3 {
        vector3(self.0.approach())
    }

    fn __repr__(&self) -> String {
        format!("<GraspDirection {}>", self.0.name())
    }
}

/// The angle between two directions, each three numbers, in radians, from 0
/// to pi. A zero direction makes no angle: the answer is pi.
#[pyfunction]
fn angle_between(a: &Bound<'_, PyAny>, b: &Bound<'_, PyAny>) -> PyResult<f64> {
    Ok(workspace_core::angle_between(direction(a)?, direction(b)?))
}

/// The height of a surface above the robot's base point, parsed off a
/// describe_workspace request to the millimetre.
#[pyclass(
    name = "SurfaceHeight",
    module = "workspace_core_py",
    frozen,
    eq,
    hash,
    skip_from_py_object
)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct PySurfaceHeight(SurfaceHeight);

#[pymethods]
impl PySurfaceHeight {
    /// The height `metres` gives, to the millimetre. Raises ValueError with
    /// the refusal of the request when `metres` is not a finite number, or
    /// when it is more than 1000 m above or below the robot's base point.
    #[staticmethod]
    fn from_wire(metres: f64) -> PyResult<Self> {
        SurfaceHeight::from_wire(metres).map(Self).map_err(refusal)
    }

    /// The height, in metres.
    #[getter]
    fn metres(&self) -> f64 {
        self.0.metres()
    }

    /// The targets an arm reaches for on the surface, `(x, y, z)` in the
    /// robot frame: ABOVE_SURFACE above each point of the standard grid, in
    /// the grid's order.
    fn grid_targets(&self) -> Vec<Vector3> {
        self.0.grid_targets().into_iter().map(vector3).collect()
    }

    fn __repr__(&self) -> String {
        format!("SurfaceHeight.from_wire({})", self.0.metres())
    }

    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Bound<'py, PyAny>, (f64,))> {
        let from_wire = py.get_type::<Self>().getattr("from_wire")?;
        Ok((from_wire, (self.0.metres(),)))
    }
}

/// The points of a check_positions request, parsed: at least one, each three
/// finite coordinates in the robot frame, each within 1000 m of the robot's
/// base point.
#[pyclass(name = "Positions", module = "workspace_core_py", frozen)]
struct PyPositions(Positions);

#[pymethods]
impl PyPositions {
    /// The points of `values`, three numbers (x, y, z) per point, in order.
    /// Raises ValueError with the refusal of the request when they are not
    /// points of a request.
    #[staticmethod]
    fn from_wire(values: &Bound<'_, PyAny>) -> PyResult<Self> {
        Positions::from_wire(&numbers(values)?)
            .map(Self)
            .map_err(refusal)
    }

    /// The points, `(x, y, z)` in the robot frame, in the request's order.
    #[getter]
    fn points(&self) -> Vec<Vector3> {
        self.0.points().iter().copied().map(vector3).collect()
    }

    fn __len__(&self) -> usize {
        self.0.points().len()
    }
}

/// Whether an arm reaches a target: reached by the arm named, or short of it
/// by how far the closest arm stops.
#[pyclass(
    name = "Reach",
    module = "workspace_core_py",
    frozen,
    eq,
    skip_from_py_object
)]
#[derive(Clone, PartialEq)]
struct PyReach(Reach);

#[pymethods]
impl PyReach {
    /// The arm named `arm` reaches the target. Raises ValueError when the
    /// name is empty: an answer names no arm for a target no arm reaches.
    #[staticmethod]
    fn reached_by(arm: String) -> PyResult<Self> {
        if arm.is_empty() {
            return Err(PyValueError::new_err(
                "a reached target names the arm that reaches it",
            ));
        }
        Ok(Self(Reach::Reached { arm }))
    }

    /// No arm reaches the target: the closest stops `by` metres short of it,
    /// at most REACH_TOLERANCE when the arms come close enough but none can
    /// point its gripper a grasp direction there. Raises ValueError when `by`
    /// is not a finite number of at least 0.
    #[staticmethod]
    fn short(by: f64) -> PyResult<Self> {
        if !(by.is_finite() && by >= 0.0) {
            return Err(PyValueError::new_err(
                "a target out of reach is short by a finite distance of at least 0 m",
            ));
        }
        // -0.0 passes the test above; the answer carries 0.
        let by = if by == 0.0 { 0.0 } else { by };
        Ok(Self(Reach::Short { by }))
    }

    /// Whether an arm reaches the target.
    #[getter]
    fn reached(&self) -> bool {
        self.0.reached()
    }

    /// The arm that reaches the target, or `""` when none does.
    #[getter]
    fn arm(&self) -> &str {
        self.0.arm()
    }

    /// How far short of the target the closest arm stops, in metres: 0 when
    /// an arm reaches it.
    #[getter]
    fn short_by(&self) -> f64 {
        self.0.short_by()
    }

    fn __repr__(&self) -> String {
        match &self.0 {
            Reach::Reached { arm } => format!("Reach.reached_by({arm:?})"),
            Reach::Short { by } => format!("Reach.short({by})"),
        }
    }

    fn __reduce__<'py>(
        &self,
        py: Python<'py>,
    ) -> PyResult<(Bound<'py, PyAny>, Bound<'py, PyTuple>)> {
        let reach = py.get_type::<Self>();
        match &self.0 {
            Reach::Reached { arm } => Ok((
                reach.getattr("reached_by")?,
                PyTuple::new(py, [arm.as_str()])?,
            )),
            Reach::Short { by } => Ok((reach.getattr("short")?, PyTuple::new(py, [*by])?)),
        }
    }
}

/// The reach of a surface at one height: whether an arm reaches each of its
/// grid targets, in the grid's order.
#[pyclass(
    name = "SurfaceReach",
    module = "workspace_core_py",
    frozen,
    skip_from_py_object
)]
#[derive(Clone)]
struct PySurfaceReach(SurfaceReach);

#[pymethods]
impl PySurfaceReach {
    /// The reach of the surface at `height` from `reaches`: one Reach per
    /// grid target of `height`, in the grid's order. Raises ValueError when
    /// their count is not that of the grid targets.
    #[staticmethod]
    fn from_grid_order(
        height: PyRef<'_, PySurfaceHeight>,
        reaches: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        SurfaceReach::from_grid_order(height.0, reach_list(reaches)?)
            .map(Self)
            .map_err(refusal)
    }

    /// The height of the surface.
    #[getter]
    fn height(&self) -> PySurfaceHeight {
        PySurfaceHeight(self.0.height())
    }

    /// The reach of each grid target, in the grid's order.
    #[getter]
    fn reaches(&self) -> Vec<PyReach> {
        self.0.reaches().iter().cloned().map(PyReach).collect()
    }
}

/// The reach of the surfaces measured last, one per height: past 32 heights
/// it drops the height it stored first, also when a caller asked that height
/// again after it was stored.
#[pyclass(name = "ReachMemo", module = "workspace_core_py")]
#[derive(Default)]
struct PyReachMemo(ReachMemo);

#[pymethods]
impl PyReachMemo {
    /// An empty memo.
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// The reach stored for the surface at `height`, or None.
    fn get(&self, height: PyRef<'_, PySurfaceHeight>) -> Option<PySurfaceReach> {
        self.0.get(height.0).map(PySurfaceReach)
    }

    /// Stores `surface` and gives back the reach stored for its height: the
    /// one already there when another request measured the height first.
    fn insert(&mut self, surface: PyRef<'_, PySurfaceReach>) -> PySurfaceReach {
        PySurfaceReach(self.0.insert(surface.0.clone()))
    }

    fn __len__(&self) -> usize {
        self.0.len()
    }
}

/// What describe_workspace answers of one surface.
#[pyclass(name = "SurfaceAnswer", module = "workspace_core_py", frozen, get_all)]
struct PySurfaceAnswer {
    /// Whether the robot can work the surface.
    workable: bool,
    /// The area of the surface the robot can work, in square metres.
    area: f64,
    /// The largest workable rectangle, or None.
    rectangle: Option<Bounds>,
    /// The rectangle bounding the points an arm reaches, or None.
    reach: Option<Bounds>,
    /// The rectangle bounding the points the camera sees, or None.
    view: Option<Bounds>,
    /// The one-line message of the answer.
    message: String,
}

impl From<SurfaceAnswer> for PySurfaceAnswer {
    fn from(answer: SurfaceAnswer) -> Self {
        Self {
            workable: answer.workable,
            area: answer.area,
            rectangle: answer.rectangle.map(bounds),
            reach: answer.reach.map(bounds),
            view: answer.view.map(bounds),
            message: answer.message,
        }
    }
}

/// What check_positions answers of one point.
#[pyclass(
    name = "PointAnswer",
    module = "workspace_core_py",
    frozen,
    get_all,
    skip_from_py_object
)]
#[derive(Clone)]
struct PyPointAnswer {
    /// The point, `(x, y, z)` in the robot frame.
    position: Vector3,
    /// Whether an arm reaches the point.
    reach: PyReach,
    /// The view's name in an answer: `no_camera` when no view is checked.
    view: &'static str,
    /// Whether the perception camera sees the point.
    in_view: bool,
    /// Whether the robot can work the point.
    workable: bool,
    /// The one-line message of the point.
    message: String,
}

impl From<PointAnswer> for PyPointAnswer {
    fn from(answer: PointAnswer) -> Self {
        Self {
            position: vector3(answer.position),
            workable: answer.workable(),
            view: answer.view.name(),
            in_view: answer.view.seen(),
            reach: PyReach(answer.reach),
            message: answer.message,
        }
    }
}

/// What check_positions answers of every point, in the request's order.
#[pyclass(
    name = "PositionsAnswer",
    module = "workspace_core_py",
    frozen,
    get_all
)]
struct PyPositionsAnswer {
    /// The answer of each point, in the request's order.
    points: Vec<PyPointAnswer>,
    /// Whether the robot can work every point.
    all_workable: bool,
    /// The one-line message of the answer.
    message: String,
}

impl From<PositionsAnswer> for PyPositionsAnswer {
    fn from(answer: PositionsAnswer) -> Self {
        Self {
            all_workable: answer.all_workable(),
            points: answer.points.into_iter().map(PyPointAnswer::from).collect(),
            message: answer.message,
        }
    }
}

/// What describe_workspace answers of the surface whose reach is `surface`,
/// for a robot without a perception camera: its view is not checked, and
/// the message says so.
#[pyfunction]
fn describe_without_perception_camera(surface: PyRef<'_, PySurfaceReach>) -> PySurfaceAnswer {
    design::describe_surface(&surface.0, &ViewCheck::NoPerceptionCamera).into()
}

/// What check_positions answers of `positions`, each reached as `reaches`
/// says, one Reach per point in the request's order, for a robot without a
/// perception camera: no view is checked, and the message says so. Raises
/// ValueError when the count of `reaches` is not that of the points.
#[pyfunction]
fn check_without_perception_camera(
    positions: PyRef<'_, PyPositions>,
    reaches: &Bound<'_, PyAny>,
) -> PyResult<PyPositionsAnswer> {
    let reach =
        PositionsReach::from_request_order(&positions.0, reach_list(reaches)?).map_err(refusal)?;
    Ok(design::check_positions(&reach, &ViewCheck::NoPerceptionCamera).into())
}

/// The ValueError that carries `error`'s text.
fn refusal(error: impl std::fmt::Display) -> PyErr {
    PyValueError::new_err(error.to_string())
}

/// The numbers of `values`, any iterable of them: a list, a tuple, a numpy
/// array.
fn numbers(values: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    values
        .try_iter()?
        .map(|value| value?.extract::<f64>())
        .collect()
}

/// The direction `values` gives, an iterable of three numbers.
fn direction(values: &Bound<'_, PyAny>) -> PyResult<[f64; 3]> {
    <[f64; 3]>::try_from(numbers(values)?).map_err(|numbers| {
        PyValueError::new_err(format!(
            "a direction holds 3 numbers, not {}",
            numbers.len()
        ))
    })
}

/// The reaches of `reaches`, an iterable of Reach.
fn reach_list(reaches: &Bound<'_, PyAny>) -> PyResult<Vec<Reach>> {
    reaches
        .try_iter()?
        .map(|reach| Ok(reach?.cast::<PyReach>()?.get().0.clone()))
        .collect()
}

fn vector3([x, y, z]: [f64; 3]) -> Vector3 {
    (x, y, z)
}

fn bounds([x_min, x_max, y_min, y_max]: [f64; 4]) -> Bounds {
    (x_min, x_max, y_min, y_max)
}
