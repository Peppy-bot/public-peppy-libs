//! The Python bindings of `control_core`'s path timing: the extension module
//! that Python imports as `path_timing_py`. Its types are in
//! `path_timing_py.pyi`.

use control_core::path_timing::{
    JOINT_PATH_CORNER_DEVIATION, MIN_JOINT_CHANGE_RAD, RoundedPath, TimedPath,
};
use control_core::positive_finite::PositiveFinite;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyTuple;

/// The Python bindings of control_core's path timing, for a robot's backbone
/// in Python: the joint trajectory of limb_motion's execute_joint_path, its
/// corners rounded and its time-optimal timing under each joint's velocity
/// and acceleration limits, played on the caller's clock.
#[pymodule]
fn path_timing_py(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add("JOINT_PATH_CORNER_DEVIATION", JOINT_PATH_CORNER_DEVIATION)?;
    module.add("MIN_JOINT_CHANGE_RAD", MIN_JOINT_CHANGE_RAD)?;
    module.add_class::<PyJointTrajectory>()?;
    module.add_function(wrap_pyfunction!(distinct_joint_waypoints, module)?)?;
    Ok(())
}

/// The waypoints of `waypoints` a joint path plays through without resting at a
/// repeat before its end, as control_core's distinct_joint_waypoints keeps them,
/// each a tuple.
#[pyfunction]
fn distinct_joint_waypoints<'py>(
    py: Python<'py>,
    waypoints: &Bound<'py, PyAny>,
) -> PyResult<Vec<Bound<'py, PyTuple>>> {
    control_core::path_timing::distinct_joint_waypoints(&joint_waypoints(waypoints)?)
        .iter()
        .map(|waypoint| PyTuple::new(py, waypoint))
        .collect()
}

/// A path of limb_motion's execute_joint_path played from rest to rest, as
/// control_core's JointTrajectory plays it, on the caller's clock: the
/// seconds since the trajectory started.
#[pyclass(name = "JointTrajectory", module = "path_timing_py", frozen)]
struct PyJointTrajectory {
    /// The path, timed under the joints' limits.
    path: TimedPath,
    /// The first waypoint, where the path starts.
    start: Vec<f64>,
    /// The last waypoint, where the path ends.
    end: Vec<f64>,
    /// The seconds the path plays over: the longer of the request and the
    /// path's own time.
    duration_s: f64,
}

#[pymethods]
impl PyJointTrajectory {
    /// The path through `waypoints` (one or more, where the joints stand
    /// first), each corner rounded within JOINT_PATH_CORNER_DEVIATION,
    /// resting where it turns back on itself and at each waypoint within
    /// MIN_JOINT_CHANGE_RAD in every joint of the waypoint given before it,
    /// timed under `velocity_limits` and `acceleration_limits` (one per
    /// joint), and played over the longer of `requested_s` and the path's own
    /// time. Raises ValueError when there is no waypoint, when the waypoints
    /// differ in their number of joints or hold a position that is not a
    /// finite number, when the limits do not hold one finite positive number
    /// per joint, when `requested_s` is not a finite number of at least 0,
    /// and when control_core's arithmetic fails on the path or the limits
    /// (see `rounded_and_timed`). The rounding and the timing run without the
    /// GIL. Limits too small to time the path in a finite number of seconds
    /// give a duration_s of infinity.
    #[staticmethod]
    fn through(
        py: Python<'_>,
        waypoints: &Bound<'_, PyAny>,
        velocity_limits: &Bound<'_, PyAny>,
        acceleration_limits: &Bound<'_, PyAny>,
        requested_s: f64,
    ) -> PyResult<Self> {
        let waypoints = joint_waypoints(waypoints)?;
        let joints = waypoints[0].len();
        let velocity_limits = joint_limits(velocity_limits, "velocity", joints)?;
        let acceleration_limits = joint_limits(acceleration_limits, "acceleration", joints)?;
        let requested_s = requested_duration_s(requested_s)?;
        let (path, end) =
            py.detach(|| rounded_and_timed(&waypoints, &velocity_limits, &acceleration_limits))?;
        let duration_s = requested_s.max(path.duration_s());
        Ok(Self {
            path,
            start: waypoints[0].clone(),
            end,
            duration_s,
        })
    }

    /// The seconds the path plays over: the longer of the request and the
    /// path's own time under the limits.
    #[getter]
    fn duration_s(&self) -> f64 {
        self.duration_s
    }

    /// The first waypoint, where the path starts.
    #[getter]
    fn start<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, &self.start)
    }

    /// The last waypoint, where the path ends.
    #[getter]
    fn end<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, &self.end)
    }

    /// The joints `t_s` seconds after the trajectory started: the first
    /// waypoint at or before 0, the last from duration_s on, and the path
    /// between them in between. Raises ValueError when `t_s` is not a finite
    /// number.
    fn sample<'py>(&self, py: Python<'py>, t_s: f64) -> PyResult<Bound<'py, PyTuple>> {
        if !t_s.is_finite() {
            return Err(PyValueError::new_err(format!(
                "a sample time is a finite number of seconds, not {t_s}"
            )));
        }
        // A path of no time stands at its end throughout.
        let fraction = if self.duration_s == 0.0 {
            1.0
        } else {
            t_s / self.duration_s
        };
        // At or before the start, and throughout a path of infinite time,
        // the joints stand at the first waypoint.
        if fraction <= 0.0 {
            return PyTuple::new(py, &self.start);
        }
        PyTuple::new(py, self.path.at_fraction(fraction))
    }

    /// Whether the trajectory has played to its end `t_s` seconds after it
    /// started.
    fn done(&self, t_s: f64) -> bool {
        t_s >= self.duration_s
    }
}

/// The path of limb_motion's execute_joint_path through `waypoints`, timed
/// under the limits, and its last waypoint; or the ValueError that carries
/// the assertion of control_core the arithmetic fails, where a sum
/// overflows or a speed underflows: waypoints some 1e154 apart, or limits
/// near the smallest positive double.
fn rounded_and_timed(
    waypoints: &[Vec<f64>],
    velocity_limits: &[f64],
    acceleration_limits: &[f64],
) -> PyResult<(TimedPath, Vec<f64>)> {
    std::panic::catch_unwind(|| {
        let rounded = RoundedPath::joint_path(waypoints);
        let end = rounded.end().to_vec();
        let timed = TimedPath::along(rounded, velocity_limits, acceleration_limits);
        (timed, end)
    })
    .map_err(|panic| {
        let assertion = panic
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("an assertion failed");
        PyValueError::new_err(format!(
            "control_core times no path through these waypoints under these limits: {assertion}"
        ))
    })
}

/// The waypoints of `values`, an iterable of iterables of numbers: at least
/// one, each of one number of joints, every position a finite number.
fn joint_waypoints(values: &Bound<'_, PyAny>) -> PyResult<Vec<Vec<f64>>> {
    let waypoints = values
        .try_iter()?
        .map(|waypoint| numbers(&waypoint?))
        .collect::<PyResult<Vec<_>>>()?;
    let Some(joints) = waypoints.first().map(Vec::len) else {
        return Err(PyValueError::new_err(
            "a joint path has a waypoint at least: where the joints stand",
        ));
    };
    if let Some(index) = waypoints
        .iter()
        .position(|waypoint| waypoint.len() != joints)
    {
        return Err(PyValueError::new_err(format!(
            "the waypoints hold one position per joint: waypoints[0] holds {joints}, \
             waypoints[{index}] holds {}",
            waypoints[index].len()
        )));
    }
    if let Some((index, position)) = waypoints.iter().enumerate().find_map(|(index, waypoint)| {
        waypoint
            .iter()
            .find(|position| !position.is_finite())
            .map(|&position| (index, position))
    }) {
        return Err(PyValueError::new_err(format!(
            "waypoints[{index}] holds {position}, and a joint position is a finite number"
        )));
    }
    Ok(waypoints)
}

/// The limits of `values` on the `quantity` of each joint ("velocity", say),
/// an iterable of numbers: one per joint of `joints`, each finite and
/// positive.
fn joint_limits(values: &Bound<'_, PyAny>, quantity: &str, joints: usize) -> PyResult<Vec<f64>> {
    let limits = numbers(values)?;
    if limits.len() != joints {
        return Err(PyValueError::new_err(format!(
            "the {quantity} limits hold one limit per joint, {joints}, not {}",
            limits.len()
        )));
    }
    limits
        .into_iter()
        .map(|limit| {
            PositiveFinite::parse(limit)
                .map(PositiveFinite::get)
                .map_err(|error| PyValueError::new_err(format!("a {quantity} limit {error}")))
        })
        .collect()
}

/// `requested_s`, when it is a finite number of at least 0.
fn requested_duration_s(requested_s: f64) -> PyResult<f64> {
    if !(requested_s.is_finite() && requested_s >= 0.0) {
        return Err(PyValueError::new_err(format!(
            "a requested duration is a finite number of at least 0 s, not {requested_s}"
        )));
    }
    // -0.0 passes the test above; the duration carries 0.
    Ok(if requested_s == 0.0 { 0.0 } else { requested_s })
}

/// The numbers of `values`, any iterable of them: a list, a tuple, a numpy
/// array.
fn numbers(values: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    values
        .try_iter()?
        .map(|value| value?.extract::<f64>())
        .collect()
}
