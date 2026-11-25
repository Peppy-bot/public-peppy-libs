//! The sensor a node runs as, as sensor_mounts:v1 places it: the name it runs under and the
//! frame it names in each measurement. With its sensor_mounts link bound, that frame is the one
//! `describe_sensors` gives the sensor of that name; with the link vacant, it is a frame of the
//! sensor's own name, and a consumer whose tree holds no frame of that name drops the
//! measurement with `unknown_frame`. A sensor node reads it once at its start through
//! [`read_sensor`], before it opens anything, with the [`SensorClass`] of the sensors it
//! streams as: a camera with [`CAMERA`].

use std::fmt::{self, Display};
use std::future::Future;
use std::time::Duration;

use control_core::asking;
use frame_tree::{FrameName, FrameNameError};

/// How long one ask of describe_sensors waits for its answer; an ask that runs out is asked
/// again.
pub const DESCRIBE_SENSORS_TIMEOUT: Duration = Duration::from_secs(2);

/// A class of sensors a node streams as: the contracts it streams on, as `describe_sensors`
/// names a sensor's kind, and the noun the refusals of the reader call such a sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorClass {
    pub noun: &'static str,
    pub kinds: &'static [&'static str],
}

/// The cameras: a sensor that streams on rgb_camera or rgbd_camera, whose frame is its colour
/// optical frame.
pub const CAMERA: SensorClass = SensorClass {
    noun: "camera",
    kinds: &["rgb_camera", "rgbd_camera"],
};

/// The fields of an answer of `describe_sensors` that place each sensor, which a node's poll
/// takes from its generated response field by field: `sensor_names`, `sensor_kinds`,
/// `sensor_frames` and `carried_by` are index-aligned.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DescribedSensors {
    pub sensor_names: Vec<String>,
    pub sensor_kinds: Vec<String>,
    pub sensor_frames: Vec<String>,
    pub carried_by: Vec<String>,
}

/// One sensor of an answer of `describe_sensors`: its entry in each of the four arrays.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DescribedSensor<'a> {
    pub name: &'a str,
    pub kind: &'a str,
    pub frame: &'a str,
    pub carried_by: &'a str,
}

impl DescribedSensors {
    /// The sensors of the answer, in its order, one for each index of its arrays; refused
    /// when the arrays do not give each sensor one kind, one frame and one carrier.
    pub fn sensors(&self) -> Result<Vec<DescribedSensor<'_>>, SensorMountsError> {
        let names = self.sensor_names.len();
        let [kinds, frames, carriers] = [
            self.sensor_kinds.len(),
            self.sensor_frames.len(),
            self.carried_by.len(),
        ];
        if [kinds, frames, carriers] != [names; 3] {
            return Err(SensorMountsError::Counts {
                names,
                kinds,
                frames,
                carriers,
            });
        }
        Ok(self
            .sensor_names
            .iter()
            .zip(&self.sensor_kinds)
            .zip(&self.sensor_frames)
            .zip(&self.carried_by)
            .map(|(((name, kind), frame), carried_by)| DescribedSensor {
                name,
                kind,
                frame,
                carried_by,
            })
            .collect())
    }
}

/// A sensor as sensor_mounts places it: the class it was read as, the name it runs under and
/// the frame each of its measurements names.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MountedSensor {
    pub class: SensorClass,
    pub name: String,
    pub frame: FrameName,
}

/// Why a sensor of `class` has no frame to name in its measurements.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SensorMountsError {
    /// The instance id is not the copy's name, an underscore and a sensor's name.
    NotInCopy {
        class: SensorClass,
        instance_id: String,
        copy: String,
    },
    /// The arrays do not give each sensor one kind, one frame and one carrier.
    Counts {
        names: usize,
        kinds: usize,
        frames: usize,
        carriers: usize,
    },
    /// The answer names no sensor of the sensor's name; `names` are the names it lists.
    NoSuchSensor {
        class: SensorClass,
        name: String,
        names: Vec<String>,
    },
    /// The answer names a sensor of a kind that is none of `class`'s.
    OtherKind {
        class: SensorClass,
        name: String,
        kind: String,
    },
    /// The sensor's entry of `sensor_frames` is empty.
    SensorFrame { name: String, error: FrameNameError },
}

impl Display for SensorMountsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SensorMountsError::NotInCopy {
                class,
                instance_id,
                copy,
            } => write!(
                f,
                "the instance id {instance_id:?} names no {noun} of copy {copy:?}: write it as \
                 `{copy}_` followed by the {noun}'s name",
                noun = class.noun
            ),
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
            SensorMountsError::NoSuchSensor { class, name, names } => {
                let listed = if names.is_empty() {
                    "none".to_owned()
                } else {
                    names.join(", ")
                };
                write!(
                    f,
                    "sensor_mounts' describe_sensors names no sensor {name:?} (its sensor_names: \
                     {listed}): run the {noun} under one of those names, or bind its \
                     sensor_mounts link to the robot that carries it, whose robot_identity \
                     implementer serves sensor_mounts",
                    noun = class.noun
                )
            }
            SensorMountsError::OtherKind { class, name, kind } => write!(
                f,
                "sensor_mounts' describe_sensors names {name:?} a sensor of {kind:?}: run the \
                 {noun} under the name of a {noun}, a sensor of {kinds}",
                noun = class.noun,
                kinds = class.kinds.join(" or ")
            ),
            SensorMountsError::SensorFrame { name, error } => write!(
                f,
                "sensor_mounts' describe_sensors gives {name:?} a sensor_frames entry: {error}"
            ),
        }
    }
}

impl std::error::Error for SensorMountsError {}

/// The sensor of `class` that runs as `instance_id` in `copy`, as sensor_mounts places it: its
/// name, its instance id in its copy, and the frame it names. With `describe_sensors`, the node's
/// poll of its bound sensor_mounts link with [`DESCRIBE_SENSORS_TIMEOUT`], the frame is the one
/// `describe_sensors` gives that name, asked until it answers, and `None` once `cancelled`
/// resolves first. With none, for a vacant link, the frame is the name, and nothing is asked. An
/// instance id that names no sensor in its copy is refused before the first ask, and an answer
/// that places no sensor of `class` under the name is refused with what to run.
///
/// # Panics
///
/// For a vacant link and an empty `instance_id` outside a copy, which names no frame: peppy's
/// instance ids are never empty.
pub async fn read_sensor<E: Display>(
    class: SensorClass,
    copy: Option<&str>,
    instance_id: &str,
    describe_sensors: Option<impl AsyncFnMut() -> Result<DescribedSensors, E>>,
    cancelled: impl Future<Output = ()>,
) -> Result<Option<MountedSensor>, SensorMountsError> {
    let name = sensor_name(class, copy, instance_id)?;
    let Some(describe_sensors) = describe_sensors else {
        let frame = FrameName::parse(&name).expect("a peppy instance id is never empty");
        return Ok(Some(MountedSensor { class, name, frame }));
    };
    let Some(sensors) = asking::ask("describe_sensors", cancelled, describe_sensors).await else {
        return Ok(None);
    };
    let frame = sensor_frame(class, &sensors, &name)?;
    Ok(Some(MountedSensor { class, name, frame }))
}

/// A value a simulation relays in a frame other than its sensor's, which the relay drops: the
/// simulation names `simulated` where sensor_mounts gives `sensor` its frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrameMismatch {
    pub simulated: String,
    pub sensor: MountedSensor,
}

impl Display for FrameMismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Self { simulated, sensor } = self;
        write!(
            f,
            "the simulation names the frame {simulated:?} where sensor_mounts gives {noun} {name:?} \
             the frame {frame:?}, so the relay drops it: pair the relay with the simulation's {noun} \
             {name:?}, or bind the relay's sensor_mounts link to the robot the simulation renders",
            noun = sensor.class.noun,
            name = sensor.name,
            frame = sensor.frame.as_str(),
        )
    }
}

/// `simulated`, the frame a simulation names in a value a relay forwards, held to `sensor`'s
/// frame: the pairing carries a frame across unchanged, so another frame is a relay paired with
/// another sensor or placed on another robot.
pub fn check_relayed_frame(simulated: &str, sensor: &MountedSensor) -> Result<(), FrameMismatch> {
    if simulated == sensor.frame.as_str() {
        return Ok(());
    }
    Err(FrameMismatch {
        simulated: simulated.to_owned(),
        sensor: sensor.clone(),
    })
}

/// The name a sensor of `class` runs under: its instance id in its copy, the name sensor_mounts
/// and a simulation give it. A launch mints a copy's instance ids as `<copy>_<id>`, so camera
/// `alpha_wrist_left` of copy `alpha` runs under `wrist_left`, and a sensor launched outside a
/// copy runs under the id the launcher wrote.
fn sensor_name(
    class: SensorClass,
    copy: Option<&str>,
    instance_id: &str,
) -> Result<String, SensorMountsError> {
    let Some(copy) = copy else {
        return Ok(instance_id.to_owned());
    };
    instance_id
        .strip_prefix(copy)
        .and_then(|rest| rest.strip_prefix('_'))
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
        .ok_or_else(|| SensorMountsError::NotInCopy {
            class,
            instance_id: instance_id.to_owned(),
            copy: copy.to_owned(),
        })
}

/// The frame that `sensors`, sensor_mounts' answer, gives the sensor of `class` running under
/// `name`.
fn sensor_frame(
    class: SensorClass,
    sensors: &DescribedSensors,
    name: &str,
) -> Result<FrameName, SensorMountsError> {
    let sensor = sensors
        .sensors()?
        .into_iter()
        .find(|sensor| sensor.name == name)
        .ok_or_else(|| SensorMountsError::NoSuchSensor {
            class,
            name: name.to_owned(),
            names: sensors.sensor_names.clone(),
        })?;
    if !class.kinds.contains(&sensor.kind) {
        return Err(SensorMountsError::OtherKind {
            class,
            name: name.to_owned(),
            kind: sensor.kind.to_owned(),
        });
    }
    FrameName::parse(sensor.frame).map_err(|error| SensorMountsError::SensorFrame {
        name: name.to_owned(),
        error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::{Cell, RefCell};

    /// The refusals camera_core_py's sensor_mounts gives in the same words: a call, its
    /// arguments and its message on each line, tab-separated.
    const REFUSALS: &str = include_str!("../tests/fixtures/sensor_mounts_refusals.tsv");

    /// `field` with each `\u{X}` in it the character of code point X, as an argument of the
    /// table writes a character it does not show.
    fn unescaped(field: &str) -> String {
        let mut text = String::new();
        let mut rest = field;
        while let Some(start) = rest.find("\\u{") {
            let (before, escape) = rest.split_at(start);
            let (code, after) = escape["\\u{".len()..]
                .split_once('}')
                .expect("a code point closed by }");
            let code = u32::from_str_radix(code, 16).expect("a code point in hex");
            text.push_str(before);
            text.push(char::from_u32(code).expect("the code point of a character"));
            rest = after;
        }
        text.push_str(rest);
        text
    }

    /// The items of a list of the table, separated by commas: none for an empty field.
    fn items(field: &str) -> Vec<String> {
        if field.is_empty() {
            return Vec::new();
        }
        field.split(',').map(unescaped).collect()
    }

    /// The refusal of the call a line of the table names, on its arguments.
    fn refusal(call: &str, arguments: &[&str]) -> SensorMountsError {
        match (call, arguments) {
            ("sensor_name", [copy, instance_id]) => {
                sensor_name(CAMERA, Some(&unescaped(copy)), &unescaped(instance_id)).unwrap_err()
            }
            ("sensor_frame", [name, names, kinds, frames, carriers]) => {
                let sensors = DescribedSensors {
                    sensor_names: items(names),
                    sensor_kinds: items(kinds),
                    sensor_frames: items(frames),
                    carried_by: items(carriers),
                };
                sensor_frame(CAMERA, &sensors, &unescaped(name)).unwrap_err()
            }
            _ => panic!(
                "the table names no call {call} of {} arguments",
                arguments.len()
            ),
        }
    }

    // Each refusal of the shared table gives its message byte for byte, as camera_core_py's
    // sensor_mounts gives it: a name in quotes as `{:?}` writes it, a character that is not
    // ASCII as it is, a control character escaped.
    #[test]
    fn each_refusal_of_the_shared_table_gives_its_message() {
        let cases = REFUSALS
            .lines()
            .filter(|line| !line.is_empty() && !line.starts_with('#'));
        for line in cases {
            let fields: Vec<&str> = line.split('\t').collect();
            let [call, arguments @ .., message] = fields.as_slice() else {
                panic!("a call and a message: {line}");
            };
            assert_eq!(refusal(call, arguments).to_string(), *message, "{line}");
        }
    }

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    /// A chest camera, a wrist camera on the left arm and a lidar on the base.
    fn answer() -> DescribedSensors {
        DescribedSensors {
            sensor_names: strings(&["chest", "wrist_left", "lidar"]),
            sensor_kinds: strings(&["rgbd_camera", "rgb_camera", "lidar_2d"]),
            sensor_frames: strings(&[
                "chest_color_optical_frame",
                "wrist_left_color_optical_frame",
                "lidar_link",
            ]),
            carried_by: strings(&["", "left", ""]),
        }
    }

    fn frame(name: &str) -> FrameName {
        FrameName::parse(name).expect("a frame name")
    }

    /// What one ask of sensor_mounts gives: its answer, or the failure of the ask.
    type Asked = Result<DescribedSensors, &'static str>;

    /// The answers of sensor_mounts, played in order, and how many asks reached it. An ask
    /// past the end of the script fails the test.
    struct Script {
        answers: RefCell<Vec<Asked>>,
        asks: Cell<usize>,
    }

    impl Script {
        fn of(answers: Vec<Asked>) -> Self {
            Self {
                answers: RefCell::new(answers.into_iter().rev().collect()),
                asks: Cell::new(0),
            }
        }

        fn next(&self) -> Asked {
            self.asks.set(self.asks.get() + 1);
            self.answers
                .borrow_mut()
                .pop()
                .expect("an ask past the end of the script")
        }
    }

    /// The camera that runs as `instance_id` in `copy`, read from `script` with a cancel that
    /// never comes.
    async fn read(
        copy: Option<&str>,
        instance_id: &str,
        script: &Script,
    ) -> Result<Option<MountedSensor>, SensorMountsError> {
        read_sensor(
            CAMERA,
            copy,
            instance_id,
            Some(async || script.next()),
            std::future::pending(),
        )
        .await
    }

    /// The camera that runs as `instance_id` in `copy` with its sensor_mounts link vacant, read
    /// with a cancel that never comes.
    async fn read_vacant(
        copy: Option<&str>,
        instance_id: &str,
    ) -> Result<Option<MountedSensor>, SensorMountsError> {
        let vacant: Option<fn() -> std::future::Ready<Asked>> = None;
        read_sensor(CAMERA, copy, instance_id, vacant, std::future::pending()).await
    }

    // With its link vacant, a camera names a frame of its own name, the name it runs under,
    // and asks nothing: the read ends though its cancel never comes. An id that names no
    // camera of its copy is refused as with a bound link.
    #[tokio::test]
    async fn a_camera_with_a_vacant_link_names_a_frame_of_its_name_and_asks_nothing() {
        for (copy, instance_id, name) in [
            (Some("alpha"), "alpha_wrist_left", "wrist_left"),
            (None, "chest", "chest"),
        ] {
            assert_eq!(
                read_vacant(copy, instance_id).await,
                Ok(Some(MountedSensor {
                    class: CAMERA,
                    name: name.to_owned(),
                    frame: frame(name),
                }))
            );
        }
        assert!(matches!(
            read_vacant(Some("alpha"), "bravo_wrist_left").await,
            Err(SensorMountsError::NotInCopy { .. })
        ));
    }

    #[tokio::test]
    async fn a_camera_of_either_kind_names_the_frame_sensor_mounts_gives_its_name() {
        for (name, expected) in [
            ("chest", "chest_color_optical_frame"),
            ("wrist_left", "wrist_left_color_optical_frame"),
        ] {
            let script = Script::of(vec![Ok(answer())]);
            assert_eq!(
                read(None, name, &script).await,
                Ok(Some(MountedSensor {
                    class: CAMERA,
                    name: name.to_owned(),
                    frame: frame(expected),
                }))
            );
            assert_eq!(script.asks.get(), 1);
        }
    }

    /// On a paused clock that moves only when every task waits: the pause between asks is the
    /// clock's, not the host's.
    #[tokio::test(start_paused = true)]
    async fn sensor_mounts_is_asked_until_it_answers() {
        let script = Script::of(vec![Err("no reply"), Err("no reply"), Ok(answer())]);
        let camera = read(None, "chest", &script).await;
        assert_eq!(
            camera.map(|camera| camera.map(|camera| camera.frame)),
            Ok(Some(frame("chest_color_optical_frame")))
        );
        assert_eq!(script.asks.get(), 3);
    }

    #[tokio::test]
    async fn a_stop_before_sensor_mounts_answers_gives_no_camera() {
        let script = Script::of(Vec::new());
        let camera = read_sensor(
            CAMERA,
            None,
            "chest",
            Some(async || script.next()),
            std::future::ready(()),
        )
        .await;
        assert_eq!((camera, script.asks.get()), (Ok(None), 0));
    }

    #[tokio::test]
    async fn a_camera_in_a_copy_runs_under_its_id_after_the_copy() {
        let script = Script::of(vec![Ok(answer())]);
        let camera = read(Some("alpha"), "alpha_wrist_left", &script).await;
        assert_eq!(
            camera.map(|camera| camera.map(|camera| camera.name)),
            Ok(Some("wrist_left".to_owned()))
        );
    }

    #[tokio::test]
    async fn an_id_that_is_no_name_in_its_copy_is_refused_before_sensor_mounts_is_asked() {
        let script = Script::of(Vec::new());
        let refusal = read(Some("alpha"), "bravo_wrist_left", &script)
            .await
            .unwrap_err();
        assert!(
            matches!(refusal, SensorMountsError::NotInCopy { .. }),
            "{refusal:?}"
        );
        assert_eq!(script.asks.get(), 0);
    }

    #[tokio::test]
    async fn a_name_sensor_mounts_does_not_list_is_refused_naming_the_names_it_lists() {
        let script = Script::of(vec![Ok(answer())]);
        let refusal = read(None, "head", &script).await.unwrap_err();
        assert_eq!(
            refusal,
            SensorMountsError::NoSuchSensor {
                class: CAMERA,
                name: "head".to_owned(),
                names: strings(&["chest", "wrist_left", "lidar"]),
            }
        );
    }

    #[test]
    fn a_class_takes_its_own_kinds_and_its_refusals_use_its_noun() {
        const LIDAR: SensorClass = SensorClass {
            noun: "lidar",
            kinds: &["lidar_2d"],
        };
        assert_eq!(
            sensor_frame(LIDAR, &answer(), "lidar"),
            Ok(frame("lidar_link"))
        );
        assert_eq!(
            sensor_frame(LIDAR, &answer(), "chest")
                .unwrap_err()
                .to_string(),
            "sensor_mounts' describe_sensors names \"chest\" a sensor of \"rgbd_camera\": run the \
             lidar under the name of a lidar, a sensor of lidar_2d"
        );
    }

    #[test]
    fn a_value_in_the_cameras_frame_passes_and_one_in_another_names_both_and_the_fix() {
        let camera = MountedSensor {
            class: CAMERA,
            name: "chest".to_owned(),
            frame: frame_tree::FrameName::parse("chest_color_optical_frame").unwrap(),
        };
        assert_eq!(
            check_relayed_frame("chest_color_optical_frame", &camera),
            Ok(())
        );
        let mismatch = check_relayed_frame("wrist_left_color_optical_frame", &camera).unwrap_err();
        assert_eq!(
            mismatch.to_string(),
            "the simulation names the frame \"wrist_left_color_optical_frame\" where sensor_mounts \
             gives camera \"chest\" the frame \"chest_color_optical_frame\", so the relay drops it: \
             pair the relay with the simulation's camera \"chest\", or bind the relay's \
             sensor_mounts link to the robot the simulation renders"
        );
        assert!(check_relayed_frame("", &camera).is_err());
    }

    #[test]
    fn a_relayed_value_of_another_class_names_that_class() {
        let lidar = MountedSensor {
            class: SensorClass {
                noun: "lidar",
                kinds: &["lidar_2d"],
            },
            name: "base".to_owned(),
            frame: frame("base_lidar_link"),
        };
        assert_eq!(
            check_relayed_frame("world", &lidar)
                .unwrap_err()
                .to_string(),
            "the simulation names the frame \"world\" where sensor_mounts gives lidar \"base\" \
             the frame \"base_lidar_link\", so the relay drops it: pair the relay with the \
             simulation's lidar \"base\", or bind the relay's sensor_mounts link to the robot the \
             simulation renders"
        );
    }

    #[test]
    fn an_empty_sensor_frame_is_refused() {
        let mut unframed = answer();
        unframed.sensor_frames[0].clear();
        let error = sensor_frame(CAMERA, &unframed, "chest").unwrap_err();
        assert_eq!(
            error,
            SensorMountsError::SensorFrame {
                name: "chest".into(),
                error: FrameNameError,
            }
        );
    }

    #[test]
    fn arrays_that_do_not_line_up_are_refused_whatever_the_name() {
        let mut short = answer();
        short.carried_by.pop();
        let error = sensor_frame(CAMERA, &short, "chest").unwrap_err();
        assert_eq!(
            error,
            SensorMountsError::Counts {
                names: 3,
                kinds: 3,
                frames: 3,
                carriers: 2,
            }
        );
    }

    #[test]
    fn an_answer_reads_as_its_sensors_in_its_order() {
        let answer = answer();
        let sensors = answer.sensors().expect("arrays that line up");
        assert_eq!(
            sensors.last(),
            Some(&DescribedSensor {
                name: "lidar",
                kind: "lidar_2d",
                frame: "lidar_link",
                carried_by: "",
            })
        );
        assert_eq!(
            sensors.iter().map(|sensor| sensor.name).collect::<Vec<_>>(),
            ["chest", "wrist_left", "lidar"]
        );
    }

    #[test]
    fn only_the_copys_own_prefix_comes_off_a_name() {
        // A camera may carry the copy's name in its own.
        assert_eq!(
            sensor_name(CAMERA, Some("alpha"), "alpha_alpha_front"),
            Ok("alpha_front".to_owned())
        );
    }

    #[test]
    fn a_camera_outside_a_copy_runs_under_its_id() {
        assert_eq!(
            sensor_name(CAMERA, None, "wrist_left"),
            Ok("wrist_left".to_owned())
        );
        assert_eq!(
            sensor_name(CAMERA, None, "alpha_wrist_left"),
            Ok("alpha_wrist_left".to_owned())
        );
    }

    #[test]
    fn an_id_that_is_no_name_in_its_copy_names_no_camera() {
        // Another copy's prefix, the copy's name run into the camera's, the
        // copy's name alone, and the bare prefix.
        for instance_id in ["bravo_wrist_left", "alphawrist_left", "alpha", "alpha_"] {
            assert_eq!(
                sensor_name(CAMERA, Some("alpha"), instance_id),
                Err(SensorMountsError::NotInCopy {
                    class: CAMERA,
                    instance_id: instance_id.to_owned(),
                    copy: "alpha".to_owned(),
                })
            );
        }
    }
}
