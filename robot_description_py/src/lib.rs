//! The Python bindings of `robot_description`: the extension module that
//! Python imports as `robot_description_py`. Its types are in
//! `robot_description_py.pyi`.

use std::collections::HashMap;
use std::path::PathBuf;

use pyo3::exceptions::{PyKeyError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyTuple};
use robot_description::chain_kinematics::Mimic;
use robot_description::{
    Arm, CameraFrame, DepthRange, Finger, FingerSpan, Gripper, LinkTransform, Model,
    RobotDescription, states,
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
    module.add("START", states::START)?;
    module.add("PARK", states::PARK)?;
    module.add("CLOSED", states::CLOSED)?;
    module.add("OPEN", states::OPEN)?;
    module.add_class::<PyModel>()?;
    module.add_class::<PyRobotDescription>()?;
    module.add_class::<PyArm>()?;
    module.add_class::<PyGripper>()?;
    module.add_class::<PyFinger>()?;
    module.add_class::<PyFingerSpan>()?;
    module.add_class::<PyMimic>()?;
    module.add_class::<PyLinkTransform>()?;
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
    /// the group that moves of its own, in the group's order. A state the
    /// SRDF does not give the group raises KeyError.
    fn state(&self, group: &str, name: &str) -> PyResult<Vec<f64>> {
        self.0
            .state(group, name)
            .map(<[f64]>::to_vec)
            .map_err(|unknown| PyKeyError::new_err(unknown.to_string()))
    }

    /// The optical frame of each camera the robot carries, with the depths
    /// it measures where the URDF gives it a depth sensor.
    #[getter]
    fn cameras<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(py, self.0.cameras().iter().cloned().map(PyCameraFrame))
    }

    /// The URDF's root link, the robot frame.
    #[getter]
    fn root_link(&self) -> &str {
        self.0.root_link()
    }

    /// The arm whose chain carries the link `link`: of the joints of every
    /// arm, the arm that holds the one nearest above the link, the first in
    /// the SRDF's order of two that hold it. None for a link that no arm's
    /// joint moves, such as one fixed to the root. A link the URDF lacks
    /// raises KeyError.
    fn arm_carrying(&self, link: &str) -> PyResult<Option<PyArm>> {
        self.0
            .arm_carrying(link)
            .map(|arm| arm.cloned().map(PyArm))
            .map_err(|error| PyKeyError::new_err(error.to_string()))
    }

    /// The limits of the joint `joint`: its position limits, the URDF's,
    /// unbounded for a continuous joint and `[0, 0]` for a fixed one, and the
    /// speed and effort limits the URDF gives it. A joint the URDF lacks raises
    /// KeyError.
    fn joint_limits(&self, joint: &str) -> PyResult<PyJointLimits> {
        let position = self
            .0
            .position_limits(joint)
            .map_err(|unknown| PyKeyError::new_err(unknown.to_string()))?;
        let found = self
            .0
            .robot()
            .joints
            .iter()
            .find(|candidate| candidate.name == joint)
            .expect("every joint of the tree is a joint of the URDF");
        Ok(PyJointLimits {
            position,
            velocity: found.limit.velocity,
            effort: found.limit.effort,
        })
    }

    /// The transform of every fixed joint of the URDF, in the URDF's order
    /// of joints.
    fn fixed_transforms<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        PyTuple::new(
            py,
            self.0.fixed_transforms().iter().map(PyLinkTransform::of),
        )
    }

    /// The transform of every joint of the URDF that moves and has a
    /// position, in the URDF's order of joints, with `positions` by joint
    /// name, radians or meters. A joint that follows another stands where its
    /// mimic puts it when `positions` holds the followed joint, and at its own
    /// position otherwise; a joint with neither has no transform.
    fn transforms_at<'py>(
        &self,
        py: Python<'py>,
        positions: HashMap<String, f64>,
    ) -> PyResult<Bound<'py, PyTuple>> {
        let positions: HashMap<&str, f64> = positions
            .iter()
            .map(|(joint, position)| (joint.as_str(), *position))
            .collect();
        PyTuple::new(
            py,
            self.0
                .transforms_at(&positions)
                .iter()
                .map(PyLinkTransform::of),
        )
    }
}

/// Where a child link stands in its parent link: a point p of the child
/// link's frame stands at R(orientation) * p + position of the parent link's.
#[pyclass(name = "LinkTransform", module = "robot_description_py", frozen)]
struct PyLinkTransform {
    parent: String,
    child: String,
    position: Vector3,
    orientation: (f64, f64, f64, f64),
}

impl PyLinkTransform {
    fn of(transform: &LinkTransform) -> Self {
        let position = transform.pose.translation.vector;
        let orientation = transform.pose.rotation;
        Self {
            parent: transform.parent.to_string(),
            child: transform.child.to_string(),
            position: (position.x, position.y, position.z),
            orientation: (orientation.i, orientation.j, orientation.k, orientation.w),
        }
    }
}

#[pymethods]
impl PyLinkTransform {
    /// The parent link.
    #[getter]
    fn parent(&self) -> &str {
        &self.parent
    }

    /// The child link.
    #[getter]
    fn child(&self) -> &str {
        &self.child
    }

    /// The child link's origin in the parent link's frame, meters.
    #[getter]
    fn position(&self) -> Vector3 {
        self.position
    }

    /// The child link's orientation in the parent link's frame, a unit
    /// quaternion `(x, y, z, w)`.
    #[getter]
    fn orientation(&self) -> (f64, f64, f64, f64) {
        self.orientation
    }

    fn __repr__(&self) -> String {
        format!("<LinkTransform {} in {}>", self.child, self.parent)
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

    /// The position of every finger joint at opening fraction `opening`, by
    /// joint name: 0 at the `closed` state and 1 at the `open` one, a finger
    /// that mimics another included.
    fn finger_positions_at(&self, opening: f64) -> HashMap<String, f64> {
        self.0
            .finger_positions_at(opening)
            .map(|(joint, position)| (joint.to_string(), position))
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("<Gripper {}>", self.0.name)
    }
}

/// One finger of a gripper: its joint, the link the joint moves, the span the
/// gripper's `closed` and `open` states give the joint, and the finger it
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

    /// Where the gripper's `closed` and `open` states put the joint.
    #[getter]
    fn span(&self) -> PyFingerSpan {
        PyFingerSpan(self.0.span)
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

/// Where a finger joint stands between its gripper's `closed` and `open`
/// states, two finite positions in the joint's unit farther apart than
/// robot_description's least finger travel: an opening
/// fraction places the joint on the line through them, 0 at `closed` and 1 at
/// `open`.
#[pyclass(name = "FingerSpan", module = "robot_description_py", frozen, eq)]
#[derive(PartialEq)]
struct PyFingerSpan(FingerSpan);

#[pymethods]
impl PyFingerSpan {
    /// The span from `closed` to `open`. Ends that are not finite, or a
    /// travel between them too small for an opening to scale, raise
    /// ValueError with robot_description's message.
    #[new]
    fn new(closed: f64, open: f64) -> PyResult<Self> {
        FingerSpan::new(closed, open)
            .map(Self)
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    #[getter]
    fn closed(&self) -> f64 {
        self.0.closed()
    }

    #[getter]
    fn open(&self) -> f64 {
        self.0.open()
    }

    /// The signed distance from `closed` to `open`, larger in magnitude than
    /// robot_description's least finger travel.
    #[getter]
    fn travel(&self) -> f64 {
        self.0.travel()
    }

    /// The joint's position at opening fraction `opening`.
    fn position_at(&self, opening: f64) -> f64 {
        self.0.position_at(opening)
    }

    /// The opening fraction at which the joint stands at `position`.
    fn opening_at(&self, position: f64) -> f64 {
        self.0.opening_at(position)
    }

    fn __repr__(&self) -> String {
        format!("<FingerSpan {} to {}>", self.0.closed(), self.0.open())
    }
}

/// How a finger joint follows another finger joint of its gripper, one that
/// moves of its own: it stands at `multiplier` times that joint's position
/// plus `offset`, in the joints' unit.
#[pyclass(name = "Mimic", module = "robot_description_py", frozen, eq)]
#[derive(PartialEq)]
struct PyMimic(Mimic);

#[pymethods]
impl PyMimic {
    /// The mimic of the joint `joint` at `multiplier` and `offset`, both
    /// finite; a value that is not raises ValueError.
    #[new]
    fn new(joint: String, multiplier: f64, offset: f64) -> PyResult<Self> {
        Mimic::new(joint, multiplier, offset)
            .map(Self)
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    /// The position of the following joint when the followed one stands at
    /// `leader`.
    fn position_from(&self, leader: f64) -> f64 {
        self.0.position_from(leader)
    }

    /// The joint followed.
    #[getter]
    fn joint(&self) -> &str {
        self.0.joint()
    }

    #[getter]
    fn multiplier(&self) -> f64 {
        self.0.multiplier()
    }

    /// Radians or meters, the joints' unit.
    #[getter]
    fn offset(&self) -> f64 {
        self.0.offset()
    }

    fn __repr__(&self) -> String {
        format!("<Mimic of {}>", self.0.joint())
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

    /// The camera's optical frame: the link `<name>_optical_frame` of the
    /// URDF.
    #[getter]
    fn frame(&self) -> String {
        self.0.frame()
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
/// continuous joint and `[0, 0]` for a fixed one, and the speed and effort
/// limits the URDF gives it.
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
