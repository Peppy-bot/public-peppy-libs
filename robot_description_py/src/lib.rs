//! The Python bindings of `robot_description`: the extension module that
//! Python imports as `robot_description_py`. Its types are in
//! `robot_description_py.pyi`.

use std::path::PathBuf;

use pyo3::exceptions::{PyKeyError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyTuple};
use robot_description::{
    Arm, CameraFrame, DepthRange, Finger, Gripper, Mimic, Model, RobotDescription, position_limits,
    states,
};

/// A point or a set of angles, `(x, y, z)` or `(roll, pitch, yaw)`.
type Vector3 = (f64, f64, f64);

/// The Python bindings of robot_description, for the Python nodes and the
/// simulation engines: each bundled model by its model id, and a robot's
/// description read from its URDF and SRDF, every name of the SRDF held to
/// the URDF.
#[pymodule]
fn robot_description_py(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add("READY", states::READY)?;
    module.add("HOME", states::HOME)?;
    module.add("PARK", states::PARK)?;
    module.add("CLOSED", states::CLOSED)?;
    module.add("OPEN", states::OPEN)?;
    module.add_class::<PyModel>()?;
    module.add_class::<PyRobotDescription>()?;
    module.add_class::<PyArm>()?;
    module.add_class::<PyGripper>()?;
    module.add_class::<PyFinger>()?;
    module.add_class::<PyMimic>()?;
    module.add_class::<PyCameraFrame>()?;
    module.add_class::<PyDepthRange>()?;
    module.add_class::<PyJointLimits>()?;
    module.add_function(wrap_pyfunction!(model, module)?)?;
    module.add_function(wrap_pyfunction!(models, module)?)?;
    Ok(())
}

/// The bundled model of the id `id`, as robot_identity gives it. An id of no
/// bundled model raises ValueError, naming the models.
#[pyfunction]
fn model(id: &str) -> PyResult<PyModel> {
    robot_description::model(id)
        .map(PyModel)
        .map_err(|unknown| PyValueError::new_err(unknown.to_string()))
}

/// The bundled models, in the order of their ids.
#[pyfunction]
fn models(py: Python<'_>) -> PyResult<Bound<'_, PyTuple>> {
    PyTuple::new(
        py,
        robot_description::models().map(PyModel).collect::<Vec<_>>(),
    )
}

/// One bundled robot model: its URDF, its SRDF and its collision meshes.
#[pyclass(name = "Model", module = "robot_description_py", frozen)]
struct PyModel(&'static Model);

#[pymethods]
impl PyModel {
    /// The model's id.
    #[getter]
    fn id(&self) -> &'static str {
        self.0.id()
    }

    /// The URDF's text.
    #[getter]
    fn urdf(&self) -> &'static str {
        self.0.urdf()
    }

    /// The SRDF's text.
    #[getter]
    fn srdf(&self) -> &'static str {
        self.0.srdf()
    }

    /// The digest of the URDF, the SRDF and the collision meshes, in
    /// lowercase hex.
    #[getter]
    fn digest(&self) -> &'static str {
        self.0.digest()
    }

    /// Each collision mesh the URDF names, `(file name, bytes)`, in the order
    /// of the names.
    #[getter]
    fn meshes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(
            py,
            self.0
                .meshes()
                .iter()
                .map(|(name, bytes)| (*name, PyBytes::new(py, bytes))),
        )
    }

    /// Writes the URDF as `<id>.urdf`, the SRDF as `<id>.srdf` and the
    /// collision meshes under `meshes/` into `directory`, created if absent,
    /// for a reader that takes the files from disk.
    fn write_to(&self, directory: PathBuf) -> PyResult<()> {
        let id = self.0.id();
        let written = std::fs::create_dir_all(&directory)
            .and_then(|()| std::fs::write(directory.join(format!("{id}.urdf")), self.0.urdf()))
            .and_then(|()| std::fs::write(directory.join(format!("{id}.srdf")), self.0.srdf()))
            .and_then(|()| self.0.write_meshes_to(&directory.join("meshes")));
        written.map_err(|error| {
            PyValueError::new_err(format!(
                "the files of {id} could not be written into {}: {error}",
                directory.display()
            ))
        })
    }

    /// The model's description.
    fn description(&self) -> PyRobotDescription {
        PyRobotDescription(self.0.description())
    }

    fn __repr__(&self) -> String {
        format!("<Model {}>", self.0.id())
    }
}

/// A robot's description: its URDF and its SRDF, each name of the SRDF found
/// in the URDF.
#[pyclass(name = "RobotDescription", module = "robot_description_py", frozen)]
struct PyRobotDescription(RobotDescription);

#[pymethods]
impl PyRobotDescription {
    /// Reads `urdf` and `srdf` together. A file that does not read, an
    /// element or an attribute the reader does not read, and a name the URDF
    /// lacks raise ValueError, naming what the files got wrong.
    #[staticmethod]
    fn parse(urdf: &str, srdf: &str) -> PyResult<Self> {
        RobotDescription::parse(urdf, srdf)
            .map(Self)
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    /// The URDF's text.
    #[getter]
    fn urdf(&self) -> &str {
        self.0.urdf()
    }

    /// The SRDF's text.
    #[getter]
    fn srdf(&self) -> &str {
        self.0.srdf()
    }

    /// The robot's arms, in the SRDF's order.
    #[getter]
    fn arms<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, self.0.arms().iter().cloned().map(PyArm))
    }

    /// The state `name` of the group `group`: a position for each joint of
    /// the group that moves of its own, in the group's order, or None for a
    /// state the SRDF does not give.
    fn state(&self, group: &str, name: &str) -> Option<Vec<f64>> {
        self.0.state(group, name).map(<[f64]>::to_vec)
    }

    /// The optical frame of each camera the robot carries, with the depths
    /// it measures where the URDF gives it a depth sensor.
    #[getter]
    fn cameras<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, self.0.cameras().iter().cloned().map(PyCameraFrame))
    }

    /// The limits of the joint `joint`: its position limits, the URDF's,
    /// unbounded for a continuous joint, and the speed and effort limits the
    /// URDF gives it. A joint the URDF lacks raises KeyError.
    fn joint_limits(&self, joint: &str) -> PyResult<PyJointLimits> {
        self.0
            .robot()
            .joints
            .iter()
            .find(|candidate| candidate.name == joint)
            .map(|found| PyJointLimits {
                position: position_limits(found),
                velocity: found.limit.velocity,
                effort: found.limit.effort,
            })
            .ok_or_else(|| PyKeyError::new_err(format!("the URDF has no joint named {joint:?}")))
    }
}

/// An arm: a group of one chain from its base link to its grasp link, with
/// its gripper.
#[pyclass(name = "Arm", module = "robot_description_py", frozen)]
struct PyArm(Arm);

#[pymethods]
impl PyArm {
    /// The group's name, the name limb_motion addresses the arm by.
    #[getter]
    fn name(&self) -> &str {
        &self.0.name
    }

    /// The first link of the chain.
    #[getter]
    fn base_link(&self) -> &str {
        &self.0.base_link
    }

    /// The tip of the chain: the grasp point its end effector stands on.
    #[getter]
    fn grasp_link(&self) -> &str {
        &self.0.grasp_link
    }

    /// The joints of the chain that move of their own, base to tip.
    #[getter]
    fn joints<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, &self.0.joints)
    }

    /// The gripper the arm carries.
    #[getter]
    fn gripper(&self) -> PyGripper {
        PyGripper(self.0.gripper.clone())
    }

    fn __repr__(&self) -> String {
        format!("<Arm {}>", self.0.name)
    }
}

/// A gripper: the group of its fingers' joints.
#[pyclass(name = "Gripper", module = "robot_description_py", frozen)]
struct PyGripper(Gripper);

#[pymethods]
impl PyGripper {
    /// The group's name, the name the gripper answers to.
    #[getter]
    fn name(&self) -> &str {
        &self.0.name
    }

    /// Each joint of the group that moves, in the group's order.
    #[getter]
    fn fingers<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, self.0.fingers.iter().cloned().map(PyFinger))
    }

    fn __repr__(&self) -> String {
        format!("<Gripper {}>", self.0.name)
    }
}

/// One finger of a gripper: its joint, the link the joint moves, where the
/// gripper's `closed` and `open` states put the joint, and the finger it
/// follows where its URDF gives the joint a mimic.
#[pyclass(name = "Finger", module = "robot_description_py", frozen)]
struct PyFinger(Finger);

#[pymethods]
impl PyFinger {
    #[getter]
    fn joint(&self) -> &str {
        &self.0.joint
    }

    #[getter]
    fn link(&self) -> &str {
        &self.0.link
    }

    #[getter]
    fn closed(&self) -> f64 {
        self.0.closed
    }

    #[getter]
    fn open(&self) -> f64 {
        self.0.open
    }

    /// The finger joint this one follows, or None for a joint that moves of
    /// its own.
    #[getter]
    fn mimic(&self) -> Option<PyMimic> {
        self.0.mimic.clone().map(PyMimic)
    }

    fn __repr__(&self) -> String {
        format!("<Finger {}>", self.0.joint)
    }
}

/// How a finger joint follows another finger joint of its gripper, one that
/// moves of its own: it stands at `multiplier` times that joint's position
/// plus `offset`, in the joints' unit.
#[pyclass(name = "Mimic", module = "robot_description_py", frozen)]
struct PyMimic(Mimic);

#[pymethods]
impl PyMimic {
    /// The joint followed.
    #[getter]
    fn joint(&self) -> &str {
        &self.0.joint
    }

    #[getter]
    fn multiplier(&self) -> f64 {
        self.0.multiplier
    }

    /// Radians or meters, the joints' unit.
    #[getter]
    fn offset(&self) -> f64 {
        self.0.offset
    }

    fn __repr__(&self) -> String {
        format!("<Mimic of {}>", self.0.joint)
    }
}

/// The optical frame of a camera: +z along the view, +x to the right of the
/// picture and +y down it, fixed to its parent link.
#[pyclass(name = "CameraFrame", module = "robot_description_py", frozen)]
struct PyCameraFrame(CameraFrame);

#[pymethods]
impl PyCameraFrame {
    /// The camera's name on the robot.
    #[getter]
    fn name(&self) -> &str {
        &self.0.name
    }

    /// The link the frame is fixed to.
    #[getter]
    fn parent_link(&self) -> &str {
        &self.0.parent_link
    }

    /// The frame's origin in the parent link's frame, meters.
    #[getter]
    fn xyz(&self) -> Vector3 {
        vector3(self.0.xyz)
    }

    /// The frame's roll, pitch and yaw in the parent link's frame, radians,
    /// as a URDF origin gives them.
    #[getter]
    fn rpy(&self) -> Vector3 {
        vector3(self.0.rpy)
    }

    /// The depths the camera measures, as the depth sensor the URDF gives on
    /// its optical frame, or None for a camera the URDF gives no depth
    /// sensor.
    #[getter]
    fn depth_range(&self) -> Option<PyDepthRange> {
        self.0.depth_range.map(PyDepthRange)
    }

    fn __repr__(&self) -> String {
        format!("<CameraFrame {}>", self.0.name)
    }
}

/// The depths a camera measures, meters along its view: the near and far of
/// its depth sensor's clip, the nearest above 0 and below the farthest.
#[pyclass(name = "DepthRange", module = "robot_description_py", frozen)]
struct PyDepthRange(DepthRange);

#[pymethods]
impl PyDepthRange {
    /// The nearest depth the camera measures, meters.
    #[getter]
    fn near(&self) -> f64 {
        self.0.near()
    }

    /// The farthest depth the camera measures, meters.
    #[getter]
    fn far(&self) -> f64 {
        self.0.far()
    }

    fn __repr__(&self) -> String {
        format!("<DepthRange {} to {} m>", self.0.near(), self.0.far())
    }
}

/// The limits of a joint: its position limits, the URDF's, unbounded for a
/// continuous joint, and the speed and effort limits the URDF gives it.
#[pyclass(name = "JointLimits", module = "robot_description_py", frozen)]
struct PyJointLimits {
    position: [f64; 2],
    velocity: f64,
    effort: f64,
}

#[pymethods]
impl PyJointLimits {
    /// The lowest position, radians or meters: -inf for a continuous joint.
    #[getter]
    fn lower(&self) -> f64 {
        self.position[0]
    }

    /// The highest position, radians or meters: inf for a continuous joint.
    #[getter]
    fn upper(&self) -> f64 {
        self.position[1]
    }

    /// The highest speed, radians or meters a second.
    #[getter]
    fn velocity(&self) -> f64 {
        self.velocity
    }

    /// The highest effort, newton meters or newtons.
    #[getter]
    fn effort(&self) -> f64 {
        self.effort
    }
}

fn vector3([x, y, z]: [f64; 3]) -> Vector3 {
    (x, y, z)
}
