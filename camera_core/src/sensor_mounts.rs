//! The answer of sensor_mounts:v1's `describe_sensors`, as a node reads it for one of the
//! robot's cameras: the robot frame and the camera's frame in the robot's tree.

use std::fmt;

use frame_tree::{FrameName, FrameNameError};

/// The contracts a camera streams on, as `describe_sensors` names a sensor's kind: the frame
/// of each is the camera's color optical frame.
const CAMERA_KINDS: [&str; 2] = ["rgb_camera", "rgbd_camera"];

/// The fields of an answer of `describe_sensors`, borrowed from the node's generated
/// response: `sensor_names`, `sensor_kinds`, `sensor_frames` and `carried_by` are
/// index-aligned.
#[derive(Clone, Copy, Debug)]
pub struct DescribedSensors<'a> {
    pub robot_frame: &'a str,
    pub sensor_names: &'a [String],
    pub sensor_kinds: &'a [String],
    pub sensor_frames: &'a [String],
    pub carried_by: &'a [String],
}

/// The robot frame and the frame of one camera in the robot's tree, as `describe_sensors`
/// names them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CameraFrames {
    pub robot_frame: FrameName,
    pub camera_frame: FrameName,
}

/// An answer of `describe_sensors` that places no camera of the name asked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SensorMountsError {
    /// The arrays do not give each sensor one kind, one frame and one carrier.
    Counts {
        names: usize,
        kinds: usize,
        frames: usize,
        carriers: usize,
    },
    NoSuchSensor {
        name: String,
    },
    NotACamera {
        name: String,
        kind: String,
    },
    /// The robot frame is empty.
    RobotFrame(FrameNameError),
    /// The camera's entry of `sensor_frames` is empty.
    SensorFrame {
        name: String,
        error: FrameNameError,
    },
}

impl fmt::Display for SensorMountsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SensorMountsError::Counts {
                names,
                kinds,
                frames,
                carriers,
            } => write!(
                f,
                "sensor_mounts' describe_sensors names {names} sensors and gives {kinds} \
                 sensor_kinds, {frames} sensor_frames and {carriers} carried_by: one of each for \
                 each sensor"
            ),
            SensorMountsError::NoSuchSensor { name } => write!(
                f,
                "sensor_mounts' describe_sensors names no sensor {name:?}: name a camera it lists \
                 in sensor_names"
            ),
            SensorMountsError::NotACamera { name, kind } => write!(
                f,
                "sensor_mounts' describe_sensors names {name:?} a sensor of {kind:?}: name a \
                 camera, a sensor of {}",
                CAMERA_KINDS.join(" or ")
            ),
            SensorMountsError::RobotFrame(error) => {
                write!(
                    f,
                    "sensor_mounts' describe_sensors gives a robot_frame: {error}"
                )
            }
            SensorMountsError::SensorFrame { name, error } => write!(
                f,
                "sensor_mounts' describe_sensors gives {name:?} a sensor_frames entry: {error}"
            ),
        }
    }
}

impl std::error::Error for SensorMountsError {}

impl CameraFrames {
    /// The robot frame and the frame of the camera `name` that `sensors` describes.
    pub fn of(sensors: &DescribedSensors<'_>, name: &str) -> Result<Self, SensorMountsError> {
        let names = sensors.sensor_names.len();
        if [
            sensors.sensor_kinds.len(),
            sensors.sensor_frames.len(),
            sensors.carried_by.len(),
        ] != [names; 3]
        {
            return Err(SensorMountsError::Counts {
                names,
                kinds: sensors.sensor_kinds.len(),
                frames: sensors.sensor_frames.len(),
                carriers: sensors.carried_by.len(),
            });
        }
        let index = sensors
            .sensor_names
            .iter()
            .position(|named| named == name)
            .ok_or_else(|| SensorMountsError::NoSuchSensor {
                name: name.to_owned(),
            })?;
        let kind = &sensors.sensor_kinds[index];
        if !CAMERA_KINDS.contains(&kind.as_str()) {
            return Err(SensorMountsError::NotACamera {
                name: name.to_owned(),
                kind: kind.clone(),
            });
        }
        Ok(Self {
            robot_frame: FrameName::parse(sensors.robot_frame)
                .map_err(SensorMountsError::RobotFrame)?,
            camera_frame: FrameName::parse(&sensors.sensor_frames[index]).map_err(|error| {
                SensorMountsError::SensorFrame {
                    name: name.to_owned(),
                    error,
                }
            })?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    /// A chest camera, a wrist camera on the left arm and a lidar on the base.
    struct Answer {
        names: Vec<String>,
        kinds: Vec<String>,
        frames: Vec<String>,
        carriers: Vec<String>,
    }

    fn answer() -> Answer {
        Answer {
            names: strings(&["chest", "wrist_left", "lidar"]),
            kinds: strings(&["rgbd_camera", "rgb_camera", "lidar_2d"]),
            frames: strings(&[
                "chest_color_optical_frame",
                "wrist_left_color_optical_frame",
                "lidar_link",
            ]),
            carriers: strings(&["", "left", ""]),
        }
    }

    impl Answer {
        fn described<'a>(&'a self, robot_frame: &'a str) -> DescribedSensors<'a> {
            DescribedSensors {
                robot_frame,
                sensor_names: &self.names,
                sensor_kinds: &self.kinds,
                sensor_frames: &self.frames,
                carried_by: &self.carriers,
            }
        }
    }

    fn frame(name: &str) -> FrameName {
        FrameName::parse(name).expect("a frame name")
    }

    #[test]
    fn a_camera_of_either_kind_gives_its_frame_and_the_robot_frame() {
        let given = answer();
        let sensors = given.described("world");
        assert_eq!(
            CameraFrames::of(&sensors, "chest"),
            Ok(CameraFrames {
                robot_frame: frame("world"),
                camera_frame: frame("chest_color_optical_frame"),
            })
        );
        assert_eq!(
            CameraFrames::of(&sensors, "wrist_left").map(|frames| frames.camera_frame),
            Ok(frame("wrist_left_color_optical_frame"))
        );
    }

    #[test]
    fn an_answer_that_places_no_camera_of_the_name_is_refused_with_the_reason() {
        let given = answer();
        let sensors = given.described("world");
        assert_eq!(
            CameraFrames::of(&sensors, "head"),
            Err(SensorMountsError::NoSuchSensor {
                name: "head".into()
            })
        );
        let lidar = CameraFrames::of(&sensors, "lidar").unwrap_err();
        assert_eq!(
            lidar.to_string(),
            "sensor_mounts' describe_sensors names \"lidar\" a sensor of \"lidar_2d\": name a \
             camera, a sensor of rgb_camera or rgbd_camera"
        );
        assert_eq!(
            CameraFrames::of(&given.described(""), "chest"),
            Err(SensorMountsError::RobotFrame(FrameNameError))
        );
        let mut unframed = answer();
        unframed.frames[0].clear();
        let error = CameraFrames::of(&unframed.described("world"), "chest").unwrap_err();
        assert_eq!(
            error,
            SensorMountsError::SensorFrame {
                name: "chest".into(),
                error: FrameNameError,
            }
        );
        assert!(
            error.to_string().starts_with(
                "sensor_mounts' describe_sensors gives \"chest\" a sensor_frames entry: "
            ),
            "{error}"
        );
    }

    #[test]
    fn arrays_that_do_not_line_up_are_refused_whatever_the_name() {
        let mut short = answer();
        short.carriers.pop();
        let error = CameraFrames::of(&short.described("world"), "chest").unwrap_err();
        assert_eq!(
            error,
            SensorMountsError::Counts {
                names: 3,
                kinds: 3,
                frames: 3,
                carriers: 2,
            }
        );
        assert_eq!(
            error.to_string(),
            "sensor_mounts' describe_sensors names 3 sensors and gives 3 sensor_kinds, 3 \
             sensor_frames and 2 carried_by: one of each for each sensor"
        );
    }
}
