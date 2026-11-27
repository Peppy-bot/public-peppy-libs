//! The sensors a URDF's Gazebo extensions give, as its text gives them, before they are
//! held to the URDF's links. urdf-rs reads no `<gazebo>` element, so this reader reads them
//! from the URDF's text. Each is the form Gazebo's URDF extension gives a sensor, on the
//! link its `reference` names: a camera's depth sensor,
//!
//! ```xml
//! <gazebo reference="chest_optical_frame">
//!   <sensor name="chest" type="depth">
//!     <camera><clip><near>0.1</near><far>15</far></clip></camera>
//!   </sensor>
//! </gazebo>
//! ```
//!
//! whose `<clip>` gives the nearest and the farthest depth the camera measures, meters along
//! its view; or a 2-D lidar,
//!
//! ```xml
//! <gazebo reference="lidar_frame">
//!   <sensor name="lidar" type="gpu_lidar">
//!     <lidar><range><min>0.05</min><max>40</max></range></lidar>
//!   </sensor>
//! </gazebo>
//! ```
//!
//! whose `<range>` gives the nearest and the farthest range the lidar measures, meters along
//! its rays. An element or an attribute this reader does not read stops the read.

use roxmltree::{Document, Node};

use crate::description::{DepthRange, LidarRange};
use crate::error::DescriptionError;
use crate::xml::{attributes, each_once, elements};

/// The types of sensor this reader reads.
const DEPTH: &str = "depth";
const GPU_LIDAR: &str = "gpu_lidar";

/// A `<gazebo>` element's sensor.
enum Sensor {
    Depth(DepthSensor),
    Lidar(LidarSensor),
}

/// The sensors of a URDF's `<gazebo>` elements, each kind in the file's order.
#[derive(Default)]
pub(crate) struct Sensors {
    pub depth: Vec<DepthSensor>,
    pub lidars: Vec<LidarSensor>,
}

/// A camera's depth sensor.
pub(crate) struct DepthSensor {
    /// The link the `<gazebo>` element references.
    pub reference: String,
    pub range: DepthRange,
}

/// A 2-D lidar.
pub(crate) struct LidarSensor {
    /// The sensor's name.
    pub name: String,
    /// The link the `<gazebo>` element references.
    pub reference: String,
    pub range: LidarRange,
}

/// The sensors of the `<gazebo>` elements of the URDF `urdf`.
pub(crate) fn sensors(urdf: &str) -> Result<Sensors, DescriptionError> {
    let document =
        Document::parse(urdf).map_err(|error| DescriptionError::Urdf(error.to_string()))?;
    elements(document.root_element())
        .filter(|element| element.tag_name().name() == "gazebo")
        .try_fold(Sensors::default(), |mut sensors, gazebo| {
            match sensor(gazebo)? {
                Sensor::Depth(depth) => sensors.depth.push(depth),
                Sensor::Lidar(lidar) => sensors.lidars.push(lidar),
            }
            Ok(sensors)
        })
}

fn sensor(gazebo: Node) -> Result<Sensor, DescriptionError> {
    let reference = attributes(gazebo, &["reference"], &[])?.required("reference")?;
    let [sensor] = each_once(gazebo, "gazebo", &["sensor"])?;
    let read = attributes(sensor, &["name", "type"], &[])?;
    let name = read.required("name")?;
    match read.required("type")?.as_str() {
        DEPTH => depth_sensor(sensor, reference).map(Sensor::Depth),
        GPU_LIDAR => lidar_sensor(sensor, name, reference).map(Sensor::Lidar),
        kind => Err(DescriptionError::SensorType {
            reference,
            kind: kind.to_string(),
        }),
    }
}

fn depth_sensor(sensor: Node, reference: String) -> Result<DepthSensor, DescriptionError> {
    let [camera] = each_once(sensor, "sensor", &["camera"])?;
    attributes(camera, &[], &[])?;
    let [clip] = each_once(camera, "camera", &["clip"])?;
    attributes(clip, &[], &[])?;
    let [near, far] = each_once(clip, "clip", &["near", "far"])?;
    let clip_value = |element: Node, bound: &'static str| {
        meters(element, bound, |value| DescriptionError::ClipValue {
            reference: reference.clone(),
            bound,
            value,
        })
    };
    let (near, far) = (clip_value(near, "near")?, clip_value(far, "far")?);
    let range = DepthRange::new(near, far).ok_or_else(|| DescriptionError::ClipRange {
        reference: reference.clone(),
        near,
        far,
    })?;
    Ok(DepthSensor { reference, range })
}

fn lidar_sensor(
    sensor: Node,
    name: String,
    reference: String,
) -> Result<LidarSensor, DescriptionError> {
    let [lidar] = each_once(sensor, "sensor", &["lidar"])?;
    attributes(lidar, &[], &[])?;
    let [range] = each_once(lidar, "lidar", &["range"])?;
    attributes(range, &[], &[])?;
    let [min, max] = each_once(range, "range", &["min", "max"])?;
    let range_value = |element: Node, bound: &'static str| {
        meters(element, bound, |value| DescriptionError::LidarRangeValue {
            reference: reference.clone(),
            bound,
            value,
        })
    };
    let (min, max) = (range_value(min, "min")?, range_value(max, "max")?);
    let range = LidarRange::new(min, max).ok_or_else(|| DescriptionError::LidarRangeBounds {
        reference: reference.clone(),
        min,
        max,
    })?;
    Ok(LidarSensor {
        name,
        reference,
        range,
    })
}

/// The number of meters the element `<bound>` holds: one finite number, and nothing else.
/// Text that is no finite number is refused by `invalid` of the text.
fn meters(
    element: Node,
    bound: &'static str,
    invalid: impl Fn(String) -> DescriptionError,
) -> Result<f64, DescriptionError> {
    attributes(element, &[], &[])?;
    let [] = each_once(element, bound, &[])?;
    let text = element.text().unwrap_or_default().trim();
    text.parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| invalid(text.to_string()))
}
