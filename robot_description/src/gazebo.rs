//! The depth sensors a URDF's Gazebo extensions give, as its text gives them, before
//! they are held to the URDF's links. urdf-rs reads no `<gazebo>` element, so this reader
//! reads them from the URDF's text. A depth sensor is the form Gazebo's URDF extension
//! gives one, on the link its `reference` names:
//!
//! ```xml
//! <gazebo reference="chest_optical_frame">
//!   <sensor name="chest" type="depth">
//!     <camera><clip><near>0.1</near><far>15</far></clip></camera>
//!   </sensor>
//! </gazebo>
//! ```
//!
//! `<clip>` gives the nearest and the farthest depth the camera measures, meters along
//! its view. An element or an attribute this reader does not read stops the read.

use roxmltree::{Document, Node};

use crate::description::DepthRange;
use crate::error::DescriptionError;
use crate::xml::{attributes, each_once, elements};

/// The one type of sensor this reader reads.
const DEPTH: &str = "depth";

/// A `<gazebo>` element's depth sensor.
pub(crate) struct DepthSensor {
    /// The link the `<gazebo>` element references.
    pub reference: String,
    pub range: DepthRange,
}

/// The depth sensor of each `<gazebo>` element of the URDF `urdf`, in the file's order.
pub(crate) fn depth_sensors(urdf: &str) -> Result<Vec<DepthSensor>, DescriptionError> {
    let document =
        Document::parse(urdf).map_err(|error| DescriptionError::Urdf(error.to_string()))?;
    elements(document.root_element())
        .filter(|element| element.tag_name().name() == "gazebo")
        .map(depth_sensor)
        .collect()
}

fn depth_sensor(gazebo: Node) -> Result<DepthSensor, DescriptionError> {
    let reference = attributes(gazebo, &["reference"], &[])?.required("reference")?;
    let [sensor] = each_once(gazebo, "gazebo", &["sensor"])?;
    let read = attributes(sensor, &["name", "type"], &[])?;
    read.required("name")?;
    let kind = read.required("type")?;
    if kind != DEPTH {
        return Err(DescriptionError::SensorType { reference, kind });
    }
    let [camera] = each_once(sensor, "sensor", &["camera"])?;
    attributes(camera, &[], &[])?;
    let [clip] = each_once(camera, "camera", &["clip"])?;
    attributes(clip, &[], &[])?;
    let [near, far] = each_once(clip, "clip", &["near", "far"])?;
    let (near, far) = (
        meters(near, "near", &reference)?,
        meters(far, "far", &reference)?,
    );
    let range = DepthRange::new(near, far).ok_or_else(|| DescriptionError::ClipRange {
        reference: reference.clone(),
        near,
        far,
    })?;
    Ok(DepthSensor { reference, range })
}

/// The number of meters the element `<bound>` of the sensor on `reference` holds: one
/// finite number, and nothing else.
fn meters(element: Node, bound: &'static str, reference: &str) -> Result<f64, DescriptionError> {
    attributes(element, &[], &[])?;
    let [] = each_once(element, bound, &[])?;
    let text = element.text().unwrap_or_default().trim();
    text.parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| DescriptionError::ClipValue {
            reference: reference.to_string(),
            bound,
            value: text.to_string(),
        })
}
