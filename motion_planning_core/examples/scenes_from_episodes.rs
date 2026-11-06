//! Turns episodes of OpenArm v2 recorded in simulation, each one plan request of an arm,
//! into scenes of an arm planning scenario of the ROS reference harness, and writes them on
//! stdout as JSON with the chain of the arm they plan, the map they stand in and the
//! episodes it drops.
//! `ros_reference/tools/draw_arm_planning.py --episodes` runs it, plans the scenes with the
//! example `plan_arm_scenes`, and writes the scenario.
//!
//! ```text
//! scenes_from_episodes <episodes directory> <launcher fragment of the motion planner>
//! ```
//!
//! It reads on stdin the scenario's header without its chains and its map: its one chain
//! is the arm the episodes plan, and the map is the motion planner's, whose arguments the
//! launcher fragment gives. Each subdirectory of the episodes directory that holds an
//! `episode.json`, in the format the harness README gives, is an episode, taken in the
//! order of their names. An episode becomes a scene at the planner's arguments:
//!
//! - The boxes: the occupied voxels of the map that the episode's depth frames build at the
//!   planner's voxel size, bounds and range, adjacent voxels along x joined into one box.
//!   The example takes the frames in the order of their stamps, each but one stamped within
//!   the planner's frame interval after the last one it took and one stamped where the
//!   limb_states do not reach, and inserts each with the robot cut out where the
//!   limb_states put it at the frame's stamp: both arms, the fingers at the grippers'
//!   openings and the body, grown by the planner's padding. An episode holds the frames
//!   that the planner's map took in between the recording's clear of the map and its plan
//!   request, so the map holds the frames the planner planned on.
//! - The start: the measured start, each joint outside a limit of the scenario's arm by no
//!   more than the planner's start tolerance moved onto it (`start_inside_limits`). The
//!   arm planning kind refuses any start outside a limit, so an episode whose start lies
//!   farther out is dropped.
//! - The goal: the first goal that the goal sampling of the scenario's arm finds for the
//!   episode's pose goal (`sample_goals`, at the planner's goal samples and the
//!   scenario's time limit), seeded by the scene's seed, its validity the scenario's
//!   clearance from the boxes. An episode with none is dropped.
//! - The seed: the episode's index plus one, as OMPL takes no seed of 0.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;
#[path = "../tests/openarm_v2/mod.rs"]
mod openarm_v2;

use std::collections::HashMap;
use std::num::{NonZeroU32, NonZeroUsize};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime};

use camera_core::encoding::DEPTH_ENCODING;
use camera_core::pinhole::{DepthModel, Intrinsics, Pinhole};
use camera_core::pose::Pose;
use collision_geometry::chain::Padding;
use control_core::positive_finite::PositiveFinite;
use motion_planning_core::{
    Arm, ArmGoal, Arms, DepthCamera, Frame, Goal, IkGoalSampler, JointHistory, OrientationTarget,
    PositionTarget, Surroundings, insert_cut_out, sample_goals, start_inside_limits,
    term_clearances,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use occupancy_map::{Bounds, DistanceField, Frame as MapFrame, Scan, VoxelMap};
use openarm_description::{ARM_DOF, ArmPair, HardwareVersion, Side};
use ros_reference::arm_planning::{Scene as ReferenceScene, SceneGoal, Scenes};
use serde::Deserialize;
use serde_json::{Value, json};

use arm_scenes::{boxes_of, map_of, on_the_arms};
use openarm_v2::Robot;

/// The node whose deployment in the launcher fragment gives the planner's arguments.
const PLANNER_NODE: &str = "motion_planner";
/// The values of one measurement as the replay's history keeps them: the left arm's
/// joints, the right arm's, then the left and the right gripper's openings.
const VALUES: usize = 2 * ARM_DOF + 2;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let [_, episodes, fragment] = args.as_slice() else {
        panic!(
            "usage: scenes_from_episodes <episodes directory> <launcher fragment of the motion planner>, \
             the scenario's header on stdin"
        );
    };
    let header: serde_json::Map<String, Value> =
        serde_json::from_reader(std::io::stdin()).expect("the scenario's header on stdin parses");
    let planner = Planner::from_fragment(Path::new(fragment));
    let recorded = Recorded::read(Path::new(episodes));
    let repository = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    let meshes = repository.join(
        header
            .get("meshes")
            .and_then(Value::as_str)
            .expect("the header names the meshes"),
    );
    let robot = Robot::new(meshes.to_str().expect("a UTF-8 mesh directory"));
    let version = HardwareVersion::V2;
    let side = recorded.side;
    let chain = robot.arms[side].chain();
    let arms = json!({
        "chains": [{
            "name": side.arm_name(),
            "base_link": version.base_link(side),
            "tip_link": version.tcp_link(side),
            "joints": (0..ARM_DOF).map(|i| chain.joint_name(i)).collect::<Vec<_>>(),
        }],
    });
    let map = json!({
        "voxel_size": planner.voxel_size.get(),
        "map_half_side": planner.map_half_side(),
    });
    let mut whole = header;
    for part in [&arms, &map] {
        whole.extend(part.as_object().expect("an object").clone());
    }
    whole.insert("scenes".into(), json!([]));
    let scenes: Scenes = serde_json::from_value(Value::Object(whole)).expect("the scenario parses");
    let urdf = std::fs::read_to_string(repository.join(&scenes.urdf)).expect("the URDF reads");
    assert!(
        urdf == version.urdf(),
        "the scenario's URDF {} is OpenArm v2's, the robot of the episodes",
        scenes.urdf
    );
    let answers: Vec<(String, Result<Value, Dropped>)> =
        on_the_arms(&scenes, &urdf, &meshes, |kind_arms, _| {
            let [kind_arm] = kind_arms else {
                panic!("the scenario of the episodes plans the arm they record");
            };
            let sampler = IkGoalSampler::new(kind_arm.geometry().chain().clone());
            recorded
                .episodes
                .iter()
                .map(|(name, episode)| {
                    let scene = scene_of(&planner, &robot, kind_arm, &sampler, &scenes, episode);
                    (name.clone(), scene)
                })
                .collect()
        });
    let output = json!({
        "arms": arms,
        "map": map,
        "scenes": answers
            .iter()
            .filter_map(|(_, answer)| answer.as_ref().ok())
            .collect::<Vec<_>>(),
        "dropped": answers
            .iter()
            .filter_map(|(name, answer)| {
                let Dropped { reason, detail } = answer.as_ref().err()?;
                Some(json!({ "episode": name, "reason": reason, "detail": detail }))
            })
            .collect::<Vec<_>>(),
    });
    println!("{output}");
}

/// Why an episode makes no scene: `reason` names the case, `detail` the episode's numbers.
struct Dropped {
    reason: &'static str,
    detail: String,
}

/// The motion planner's arguments that the replay reads, as the launcher fragment gives
/// them; the fragment's other arguments do not change a map or a goal.
#[derive(Deserialize)]
struct PlannerArguments {
    start_tolerance_rad: f64,
    padding_scale: f64,
    padding_offset_m: f64,
    voxel_size_m: f64,
    map_min_x_m: f64,
    map_min_y_m: f64,
    map_min_z_m: f64,
    map_max_x_m: f64,
    map_max_y_m: f64,
    map_max_z_m: f64,
    max_range_m: f64,
    max_update_rate_hz: f64,
    goal_samples: NonZeroUsize,
}

/// A launcher fragment, as far as the replay reads it: its deployments, each its node and
/// its instances' arguments.
#[derive(Deserialize)]
struct Fragment {
    deployments: Vec<Deployment>,
}

#[derive(Deserialize)]
struct Deployment {
    source: Source,
    instances: Vec<Instance>,
}

#[derive(Deserialize)]
struct Source {
    name: String,
}

#[derive(Deserialize)]
struct Instance {
    arguments: Value,
}

/// The motion planner's map and requests, from its arguments.
struct Planner {
    start_tolerance: f64,
    padding: Padding,
    voxel_size: PositiveFinite,
    bounds: Bounds,
    max_range_m: PositiveFinite,
    /// The least time between two depth frames the map takes in: 1 / max_update_rate_hz.
    frame_interval: Duration,
    goal_samples: NonZeroUsize,
}

impl Planner {
    /// The planner of the one instance of the motion planner that the launcher fragment at
    /// `path` deploys.
    fn from_fragment(path: &Path) -> Self {
        let text = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("the launcher fragment {}: {e}", path.display()));
        let fragment: Fragment = json5::from_str(&text)
            .unwrap_or_else(|e| panic!("the launcher fragment {}: {e}", path.display()));
        let instances: Vec<Instance> = fragment
            .deployments
            .into_iter()
            .filter(|d| d.source.name == PLANNER_NODE)
            .flat_map(|d| d.instances)
            .collect();
        let [instance] = <[Instance; 1]>::try_from(instances).unwrap_or_else(|found| {
            panic!(
                "{} deploys {} instances of {PLANNER_NODE}; the replay reads the arguments of one",
                path.display(),
                found.len()
            )
        });
        let a: PlannerArguments = serde_json::from_value(instance.arguments)
            .unwrap_or_else(|e| panic!("the {PLANNER_NODE} arguments of {}: {e}", path.display()));
        let positive = |name: &str, value: f64| {
            PositiveFinite::parse(value)
                .unwrap_or_else(|_| panic!("{name} is {value}; it is a finite number above 0"))
        };
        let rate = positive("max_update_rate_hz", a.max_update_rate_hz);
        Self {
            start_tolerance: positive("start_tolerance_rad", a.start_tolerance_rad).get(),
            padding: Padding::new(a.padding_scale, a.padding_offset_m)
                .expect("padding_scale and padding_offset_m make a padding"),
            voxel_size: positive("voxel_size_m", a.voxel_size_m),
            bounds: Bounds {
                min: [a.map_min_x_m, a.map_min_y_m, a.map_min_z_m],
                max: [a.map_max_x_m, a.map_max_y_m, a.map_max_z_m],
            },
            max_range_m: positive("max_range_m", a.max_range_m),
            frame_interval: Duration::from_secs_f64(1.0 / rate.get()),
            goal_samples: a.goal_samples,
        }
    }

    /// The half side of the cube about the robot's origin that holds the planner's map:
    /// the farthest of its bounds from the origin on any axis. Keys floor a coordinate, so
    /// the cube's voxels on each axis run from at or below the map's first to at or above
    /// its last.
    fn map_half_side(&self) -> f64 {
        self.bounds
            .min
            .iter()
            .chain(&self.bounds.max)
            .fold(0.0, |far: f64, c| far.max(c.abs()))
    }
}

/// The episodes of a directory, each by its subdirectory's name, and the arm they plan.
struct Recorded {
    episodes: Vec<(String, Episode)>,
    side: Side,
}

impl Recorded {
    /// The episodes under `directory`, in the order of their names; refuses a directory
    /// with none, or with episodes of both arms, which one scenario cannot plan.
    fn read(directory: &Path) -> Self {
        let mut names: Vec<PathBuf> = std::fs::read_dir(directory)
            .unwrap_or_else(|e| panic!("the episodes directory {}: {e}", directory.display()))
            .map(|entry| entry.expect("an entry of the episodes directory").path())
            .filter(|path| path.join("episode.json").is_file())
            .collect();
        names.sort();
        let episodes: Vec<(String, Episode)> = names
            .iter()
            .map(|path| {
                let name = path
                    .file_name()
                    .and_then(|n| n.to_str())
                    .expect("a UTF-8 episode name")
                    .to_string();
                (name, Episode::read(path))
            })
            .collect();
        let side = episodes
            .first()
            .unwrap_or_else(|| panic!("{} holds no episode.json", directory.display()))
            .1
            .side;
        if let Some((name, other)) = episodes.iter().find(|(_, e)| e.side != side) {
            panic!(
                "{name} plans the {} arm and the episodes before it the {} arm; a scenario plans one",
                other.side, side
            );
        }
        Self { episodes, side }
    }
}

/// An episode as `episode.json` holds it; its other members are for the reader.
#[derive(Deserialize)]
struct EpisodeFile {
    index: u32,
    arm: String,
    start: [f64; ARM_DOF],
    goal: PoseGoal,
    camera: CameraFile,
    limb_states: Vec<LimbSample>,
    frames: Vec<FrameFile>,
}

/// The request's pose goal: `link`'s origin within `tolerance_m` of `position`, turned
/// within `tolerance_rad` of `orientation`, a quaternion [x, y, z, w].
#[derive(Deserialize)]
struct PoseGoal {
    link: String,
    position: [f64; 3],
    orientation: [f64; 4],
    tolerance_m: f64,
    tolerance_rad: f64,
}

#[derive(Deserialize)]
struct PoseFile {
    position: [f64; 3],
    orientation: [f64; 4],
}

#[derive(Deserialize)]
struct IntrinsicsFile {
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    distortion_model: String,
    distortion: Vec<f64>,
    depth_model: String,
}

/// The depth camera as the planner reads it: its colour optical frame in the robot frame,
/// its depth optical frame in the colour one, its depth intrinsics, its meters per depth
/// count and the alignment the intrinsics describe.
#[derive(Deserialize)]
struct CameraFile {
    color_in_robot: PoseFile,
    depth_in_color: PoseFile,
    intrinsics: IntrinsicsFile,
    depth_unit: f32,
    align_mode: String,
}

/// One limb_states message: its stamp in seconds, each arm's joints and each gripper's
/// opening by name.
#[derive(Deserialize)]
struct LimbSample {
    at: f64,
    joints: HashMap<String, [f64; ARM_DOF]>,
    openings: HashMap<String, f64>,
}

/// One depth frame: its stamp in seconds, its encoding, size and alignment, and the file of
/// its samples, relative to the episode's directory.
#[derive(Deserialize)]
struct FrameFile {
    at: f64,
    encoding: String,
    width: u32,
    height: u32,
    align_mode: String,
    samples: PathBuf,
}

/// An episode, parsed.
struct Episode {
    index: u32,
    /// Both planners' seed: the index plus one, as OMPL takes no seed of 0.
    seed: NonZeroU32,
    side: Side,
    start: [f64; ARM_DOF],
    goal: Goal,
    camera: DepthCamera,
    /// The measurements in the order of their stamps.
    measured: Vec<(SystemTime, [f64; VALUES])>,
    /// The frames' stamps and the paths of their samples, in the order of their stamps.
    frames: Vec<(SystemTime, PathBuf)>,
}

impl Episode {
    /// The episode in `directory`; a member that does not parse, or a frame the camera
    /// could not have sent the planner, stops the replay, naming the episode.
    fn read(directory: &Path) -> Self {
        let what = directory.display();
        let path = directory.join("episode.json");
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{what}: {e}"));
        let file: EpisodeFile =
            serde_json::from_str(&text).unwrap_or_else(|e| panic!("{what}: episode.json: {e}"));
        let side = Side::from_arm_name(&file.arm)
            .unwrap_or_else(|e| panic!("{what}: the arm {:?}: {e}", file.arm));
        let tcp = HardwareVersion::V2.tcp_link(side);
        assert_eq!(
            file.goal.link, tcp,
            "{what}: the goal names the grasp point, {tcp}"
        );
        let goal = pose_goal(&file.goal).unwrap_or_else(|e| panic!("{what}: the goal: {e}"));
        let camera = &file.camera;
        let lens = Pinhole::parse(&Intrinsics {
            width: camera.intrinsics.width,
            height: camera.intrinsics.height,
            fx: camera.intrinsics.fx,
            fy: camera.intrinsics.fy,
            cx: camera.intrinsics.cx,
            cy: camera.intrinsics.cy,
            distortion_model: &camera.intrinsics.distortion_model,
            distortion: &camera.intrinsics.distortion,
        })
        .unwrap_or_else(|e| panic!("{what}: the depth intrinsics: {e}"));
        let depth_model = DepthModel::parse(&camera.intrinsics.depth_model)
            .unwrap_or_else(|e| panic!("{what}: the depth model: {e}"));
        let pose = |p: &PoseFile, name: &str| {
            Pose::new(p.position, p.orientation).unwrap_or_else(|e| panic!("{what}: {name}: {e}"))
        };
        let depth = DepthCamera::new(
            &pose(&camera.color_in_robot, "color_in_robot"),
            &pose(&camera.depth_in_color, "depth_in_color"),
            lens,
            depth_model,
            camera.depth_unit,
        );
        let mut measured: Vec<(SystemTime, [f64; VALUES])> = file
            .limb_states
            .iter()
            .map(|sample| (stamp(sample.at, &what), measurement(sample, &what)))
            .collect();
        measured.sort_by_key(|(at, _)| *at);
        let mut frames: Vec<(SystemTime, PathBuf)> = file
            .frames
            .iter()
            .map(|frame| {
                assert_eq!(
                    frame.encoding, DEPTH_ENCODING,
                    "{what}: a frame encoded otherwise than the planner reads"
                );
                assert_eq!(
                    frame.align_mode, camera.align_mode,
                    "{what}: a frame aligned otherwise than the camera's geometry describes"
                );
                assert_eq!(
                    [frame.width, frame.height],
                    [camera.intrinsics.width, camera.intrinsics.height],
                    "{what}: a frame of another size than the depth intrinsics'"
                );
                (stamp(frame.at, &what), directory.join(&frame.samples))
            })
            .collect();
        frames.sort_by_key(|(at, _)| *at);
        let seed = file
            .index
            .checked_add(1)
            .and_then(NonZeroU32::new)
            .unwrap_or_else(|| panic!("{what}: its index {} plus one is no u32", file.index));
        Self {
            index: file.index,
            seed,
            side,
            start: file.start,
            goal,
            camera: depth,
            measured,
            frames,
        }
    }
}

/// The instant `seconds` after the epoch of the recording's clock.
fn stamp(seconds: f64, what: &impl std::fmt::Display) -> SystemTime {
    let since = Duration::try_from_secs_f64(seconds)
        .unwrap_or_else(|e| panic!("{what}: a stamp of {seconds} s: {e}"));
    SystemTime::UNIX_EPOCH + since
}

/// A limb_states message as the replay's history keeps it, both arms and both grippers
/// read by their names.
fn measurement(sample: &LimbSample, what: &impl std::fmt::Display) -> [f64; VALUES] {
    let arms = ArmPair::from_fn(|side| {
        *sample
            .joints
            .get(side.arm_name())
            .unwrap_or_else(|| panic!("{what}: a limb state without {}", side.arm_name()))
    });
    let openings = ArmPair::from_fn(|side| {
        *sample
            .openings
            .get(side.gripper_name())
            .unwrap_or_else(|| panic!("{what}: a limb state without {}", side.gripper_name()))
    });
    std::array::from_fn(|i| match i {
        i if i < ARM_DOF => arms[Side::Left][i],
        i if i < 2 * ARM_DOF => arms[Side::Right][i - ARM_DOF],
        i if i == 2 * ARM_DOF => openings[Side::Left],
        _ => openings[Side::Right],
    })
}

/// The arms' joints and the grippers' openings of a measurement the history keeps.
fn arms_and_openings(values: &[f64; VALUES]) -> (ArmPair<[f64; ARM_DOF]>, ArmPair<f64>) {
    (
        ArmPair::new(
            std::array::from_fn(|i| values[i]),
            std::array::from_fn(|i| values[ARM_DOF + i]),
        ),
        ArmPair::new(values[2 * ARM_DOF], values[2 * ARM_DOF + 1]),
    )
}

/// The goal of the grasp point that `goal` asks.
fn pose_goal(goal: &PoseGoal) -> Result<Goal, String> {
    let tolerance = |name: &str, value: f64| {
        PositiveFinite::parse(value).map_err(|_| format!("{name} is {value}; it is above 0"))
    };
    let [x, y, z, w] = goal.orientation;
    Goal::new(
        ARM_DOF,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            position: Vector3::from(goal.position),
            tolerance_m: tolerance("tolerance_m", goal.tolerance_m)?,
        }],
        vec![OrientationTarget {
            frame: Frame::GraspPoint,
            orientation: UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)),
            tolerance_rad: tolerance("tolerance_rad", goal.tolerance_rad)?,
            free_axis: None,
        }],
    )
    .map_err(|e| e.to_string())
}

/// The map the planner builds of `episode`'s frames, and how many it took.
fn map_of_episode(planner: &Planner, robot: &Robot, episode: &Episode) -> (VoxelMap, usize) {
    let tree = robot.arms[Side::Left].chain().tree();
    let root = MapFrame::parse(&tree.link(tree.root()).name).expect("the URDF root names a frame");
    let mut map =
        VoxelMap::new(root.clone(), planner.voxel_size, planner.bounds).expect("the planner's map");
    // The replay keeps the whole recording, so every frame's stamp the limb_states reach
    // lies inside its history.
    let mut history = JointHistory::<VALUES>::new(Duration::MAX);
    for (at, values) in &episode.measured {
        history.push(*at, *values);
    }
    let mut model = robot.self_model.clone();
    let mut last_taken: Option<SystemTime> = None;
    let mut taken = 0;
    for (at, samples) in &episode.frames {
        let recent = last_taken.is_some_and(|t| {
            at.duration_since(t)
                .is_ok_and(|d| d < planner.frame_interval)
        });
        if recent {
            continue;
        }
        let Some(values) = history.at(*at) else {
            continue;
        };
        let (arms, openings) = arms_and_openings(&values);
        model.set_gripper_openings(openings[Side::Left], openings[Side::Right]);
        let fingers = robot.fingers(&model);
        let bytes = std::fs::read(samples)
            .unwrap_or_else(|e| panic!("the samples {}: {e}", samples.display()));
        let points = episode
            .camera
            .points(&bytes)
            .unwrap_or_else(|e| panic!("the samples {}: {e}", samples.display()));
        insert_cut_out(
            &mut map,
            &Scan {
                frame: &root,
                sensor: episode.camera.sensor,
                points: &points,
                max_range_m: planner.max_range_m,
            },
            &robot.mask(&arms, &fingers, planner.padding),
        )
        .expect("the map and every scan stand in the robot frame");
        last_taken = Some(*at);
        taken += 1;
    }
    (map, taken)
}

/// The scene of `episode` on the scenario's arm `arm`, whose goal sampler is `sampler`, or
/// why it makes none.
fn scene_of(
    planner: &Planner,
    robot: &Robot,
    arm: &Arm<ARM_DOF>,
    sampler: &IkGoalSampler<ARM_DOF>,
    scenes: &Scenes,
    episode: &Episode,
) -> Result<Value, Dropped> {
    let (map, taken) = map_of_episode(planner, robot, episode);
    if taken == 0 {
        return Err(Dropped {
            reason: "no_depth_frame",
            detail: format!(
                "none of its {} frames is stamped where its limb_states reach",
                episode.frames.len()
            ),
        });
    }
    let chain = arm.geometry().chain();
    let start = start_inside_limits(chain, &episode.start, &[planner.start_tolerance; ARM_DOF])
        .map_err(|fault| Dropped {
            reason: "start_outside_limits",
            detail: fault.to_string(),
        })?;
    let seed = episode.seed;
    let boxes = boxes_of(&map);
    let scene = ReferenceScene {
        boxes,
        objects: Vec::new(),
        attached: Vec::new(),
        allowed: Vec::new(),
        start: start.to_vec(),
        goal: SceneGoal::Joints(Vec::new()),
        seed,
        ours: (),
    };
    let kind_map = map_of(scenes, &scene);
    let field = DistanceField::of(&kind_map);
    let surroundings = Surroundings::new(&kind_map, &field, term_clearances(scenes.min_clearance));
    let deadline = Instant::now() + Duration::from_secs_f64(scenes.time_limit);
    let alone = Arms::<ARM_DOF, ARM_DOF>::new(vec![arm.clone()], None);
    let sampled = sample_goals(
        &[ArmGoal {
            goal: &episode.goal,
            sampler,
        }],
        &start,
        planner.goal_samples,
        u64::from(seed.get()),
        &|| Instant::now() >= deadline,
        &|q, _| surroundings.clear(&alone, q),
    );
    let goal = sampled.first().ok_or_else(|| Dropped {
        reason: "no_free_goal",
        detail: format!(
            "the goal sampling found no configuration that meets the goal and keeps {} m from \
             the boxes by its end",
            scenes.min_clearance
        ),
    })?;
    eprintln!(
        "episode {}: {taken} of {} frames taken, {} boxes",
        episode.index,
        episode.frames.len(),
        scene.boxes.len()
    );
    Ok(json!({
        "boxes": scene
            .boxes
            .iter()
            .map(|b| json!({ "first": b.first, "last": b.last }))
            .collect::<Vec<_>>(),
        "objects": [],
        "attached": [],
        "allowed": [],
        "start": scene.start,
        "goal": goal.to_vec(),
        "seed": seed.get(),
        "ours": null,
    }))
}
