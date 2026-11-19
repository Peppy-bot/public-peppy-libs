//! Room dataset. The reference is slam_toolbox, which maps a room and localizes in the map
//! of a mapping scenario, and Nav2's AMCL, which localizes globally in it, on 2-D scans: of
//! a TUM RGB-D sequence, those that pointcloud_to_laserscan projects from its depth frames,
//! with odometry made from its ground truth and seeded noise; of a lidar drive of `drives/`,
//! the scans that the reference casts, every ray at its scan's stamp, with odometry made
//! from the drive's route and seeded noise.
//!
//! The reference reads its sequences from the host's dataset cache, so the kind runs
//! locally only, through `run.sh --datasets`: the reference program writes the prepared
//! inputs and the poses of each run under the results directory, `target/results/` of the
//! harness, and [`write_statistics`] scores each run with [`crate::metrics`] and writes
//! the statistics of each seed as the scenario's reference output, which is committed.
//! [`Statistics`] reads the committed statistics. An estimate of AMCL is a pose that it
//! published on `amcl_pose`. A parity test of a local-only kind reads the prepared inputs of
//! a scenario under [`crate::results_directory`] with [`read_prepared`] and
//! [`read_odometry`], and scores its own runs with [`trajectory_errors_of`],
//! [`map_from_truth`] and [`localization_errors`], by which the statistics score the
//! reference's.

use crate::laser_projection::{LaserScanParameters, RangeRead, range_of, single};
use crate::metrics::{
    self, Deviations, GlobalEstimate, GlobalOutcome, LocalizationLimits, LocalizedWithin,
    PlanarPose, best_alignment, global_outcome, trajectory_errors,
};
use crate::{
    Kind, json_file, parse, read_bytes, read_reference, scenario_directory, scenario_names,
    write_json,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write as _;
use std::path::{Path, PathBuf};

/// The repository files of the kind's own code: the reference program's sources of the kind,
/// the shared reference programs it runs (the Nav2 node, whose `set_ros_time` sets each
/// node's clock, the AMCL node, the laser projection, whose projector turns each depth
/// frame's cloud into a scan, and the point clouds), and the crate's statistics. Each output
/// lists them among its inputs, with the files that the reference read, so
/// `INPUTS.SHA256SUMS` holds their digests, and a change to one fails the routine tests
/// until `run.sh --datasets` writes the outputs again.
const SOURCES: [&str; 18] = [
    "ros_reference/reference_programs/amcl_map_cspace.cpp",
    "ros_reference/reference_programs/amcl_map_cspace.hpp",
    "ros_reference/reference_programs/amcl_node.cpp",
    "ros_reference/reference_programs/amcl_node.hpp",
    "ros_reference/reference_programs/laser_projection.cpp",
    "ros_reference/reference_programs/laser_projection.hpp",
    "ros_reference/reference_programs/nav2_node.cpp",
    "ros_reference/reference_programs/nav2_node.hpp",
    "ros_reference/reference_programs/point_cloud.cpp",
    "ros_reference/reference_programs/point_cloud.hpp",
    "ros_reference/reference_programs/room_dataset.cpp",
    "ros_reference/reference_programs/room_dataset.hpp",
    "ros_reference/reference_programs/room_dataset_files.cpp",
    "ros_reference/reference_programs/room_dataset_files.hpp",
    "ros_reference/reference_programs/room_dataset_runs.cpp",
    "ros_reference/src/bin/statistics.rs",
    "ros_reference/src/metrics.rs",
    "ros_reference/src/room_dataset.rs",
];

/// The `kind` of a room dataset scenario and of its reference output, which refuses every
/// other kind.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub enum ThisKind {
    #[serde(rename = "room_dataset")]
    RoomDataset,
}

/// The parameters that a ROS node of the reference reads, by name, as the reference
/// program sets them.
pub type RosParameters = BTreeMap<String, Value>;

/// A room dataset scenario: the source of its scans, the seeds of its runs, the odometry
/// noise, slam_toolbox's parameters, and its task.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub kind: ThisKind,
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub source: Source,
    pub seeds: Vec<i64>,
    pub odometry_noise: OdometryNoise,
    pub slam_toolbox: RosParameters,
    pub task: Task,
}

/// Where the scans of a scenario come from.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Source {
    /// A TUM RGB-D sequence of `datasets.txt`, its depth frames projected to scans.
    TumRgbd(TumRgbd),
    /// A lidar drive of `drives/`, which the README describes.
    LidarDrive(LidarDrive),
}

/// A TUM RGB-D sequence: its name in `datasets.txt`, and how the reference keeps and
/// projects its depth frames.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TumRgbd {
    pub sequence: String,
    /// The time between the depth frames that the scans come from (s).
    pub scan_period_s: f64,
    /// The longest gap of the ground truth across which a scan's pose is interpolated (s).
    pub max_ground_truth_gap_s: f64,
    pub camera: Camera,
    pub laser_scan: LaserScanParameters,
}

/// A lidar drive: its file, by its path from the repository root, and the scans that the
/// scenario keeps, `[first, end)` by their index in the drive.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarDrive {
    drive: String,
    scans: [usize; 2],
}

/// The member of a lidar drive's file that its readers take beyond the prepared scans.
#[derive(Debug, Deserialize)]
struct DriveRead {
    lidar: DriveLidarRead,
}

#[derive(Debug, Deserialize)]
struct DriveLidarRead {
    time_increment_s: f64,
}

impl LidarDrive {
    /// The time (s) from each ray of the drive's scans to the next, at which the drive
    /// cast it.
    pub fn time_increment_s(&self) -> f64 {
        let path = crate::repository_root().join(&self.drive);
        let drive: DriveRead = parse(&path, &crate::read_input(&path));
        drive.lidar.time_increment_s
    }
}

/// The depth camera's pinhole intrinsics (px) and the depth sample of one meter.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub depth_counts_per_meter: f64,
}

/// The variance coefficients of AMCL's differential motion model, by which the reference
/// draws the odometry noise of each motion between two scans: of a turn, from the turn
/// (`alpha1`) and from the move (`alpha2`); of the move, from the move (`alpha3`) and from
/// the turns (`alpha4`).
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OdometryNoise {
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub alpha4: f64,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Task {
    /// slam_toolbox maps the room once a seed, and saves each seed's map.
    Mapping { map_saver: MapSaver },
    /// Once a seed, slam_toolbox localizes in the map that the same seed of `map_scenario`
    /// made, and AMCL localizes globally in it. An estimate of AMCL is localized when it
    /// lies `localized_within` the truth and, by AMCL's convergence test, the test holds or,
    /// by `search_limits`, the deviations of AMCL's whole filter lie within them.
    Localization {
        map_scenario: String,
        amcl: RosParameters,
        localized_within: LocalizedWithin,
        search_limits: LocalizationLimits,
    },
}

/// nav2_map_server's thresholds of a free and an occupied cell.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapSaver {
    pub free_thresh: f64,
    pub occupied_thresh: f64,
}

/// The scenarios of the kind, in the order of their names, read from `scenarios/`
/// without their digests, which `run.sh` writes after the statistics.
pub fn scenarios() -> Vec<Scenario> {
    scenario_names(Kind::RoomDataset)
        .into_iter()
        .map(|name| {
            let path = json_file(&scenario_directory(), &name);
            let scenario: Scenario = parse(&path, &read_bytes(&path));
            assert_eq!(
                scenario.name, name,
                "scenarios/{name}.json names itself {}",
                scenario.name
            );
            scenario
        })
        .collect()
}

/// The camera's mount on the base that the reference fitted from the ground truth.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MountRead {
    pub floor_normal: [f64; 3],
    /// The camera's height above the floor (m).
    pub height: f64,
    /// The tilt of the optical axis below the floor (rad).
    pub pitch: f64,
    pub roll: f64,
}

/// What the reference read of a TUM RGB-D sequence: its name, the camera's mount fitted
/// from its ground truth, its depth frames, and the stamps (ns) of the depth frames kept a
/// scan period that lay in a gap of the ground truth, which it dropped.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TumRgbdRead {
    pub sequence: String,
    pub mount: MountRead,
    pub depth_frames: usize,
    pub dropped_scans: Vec<i64>,
}

/// What the reference read of a scenario's source.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum SourceRead {
    TumRgbd(TumRgbdRead),
    /// A lidar drive, of which the reference measures nothing beyond its scans.
    LidarDrive {},
}

impl SourceRead {
    /// Whether this is what the reference reads of `source`.
    fn read_from(&self, source: &Source) -> bool {
        match (self, source) {
            (SourceRead::TumRgbd(read), Source::TumRgbd(tum)) => read.sequence == tum.sequence,
            (SourceRead::LidarDrive {}, Source::LidarDrive(_)) => true,
            _ => false,
        }
    }
}

/// What the reference program wrote as its output for a scenario: what it read and
/// prepared, and the runs it ran.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RunsRead {
    kind: ThisKind,
    source: SourceRead,
    task: String,
    inputs: Vec<String>,
    datasets_read: Vec<String>,
    scans: usize,
    seeds: Vec<i64>,
    runs: Vec<String>,
}

/// A kept scan as the reference prepared it: its stamp, the base's pose in the ground
/// truth's frame, and its ranges, each widened from single precision.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PreparedScanRead {
    stamp_ns: i64,
    truth: [f64; 3],
    ranges: Vec<RangeRead>,
}

/// The scan parameters that pointcloud_to_laserscan wrote into each scan, each widened
/// from single precision.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScanParametersRead {
    angle_min: f64,
    angle_max: f64,
    angle_increment: f64,
    range_min: f64,
    range_max: f64,
    scan_time: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PreparedRead {
    laser_scan: ScanParametersRead,
    scans: Vec<PreparedScanRead>,
}

/// The scan parameters that pointcloud_to_laserscan wrote into each scan of a scenario,
/// in the single precision of the message: its angles (rad), its range limits (m) and its
/// time (s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanParameters {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub scan_time: f32,
}

/// A kept scan: its stamp (ns), the base's pose in the ground truth's frame, and the
/// ranges (m) that pointcloud_to_laserscan projected, in single precision.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedScan {
    pub stamp_ns: i64,
    pub truth: PlanarPose,
    pub ranges: Vec<f32>,
}

/// The inputs that the reference prepared for a scenario, which a run reads: the scan
/// parameters and each kept scan.
#[derive(Debug, Clone, PartialEq)]
pub struct Prepared {
    pub laser_scan: ScanParameters,
    pub scans: Vec<PreparedScan>,
}

/// The pose of a kept scan that a run recorded, by the index of the scan among the kept
/// scans.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanPose {
    pub scan: usize,
    pub pose: PlanarPose,
}

/// The pose of a scan that a run recorded, by the index of the scan among the kept scans.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RecordedPose {
    scan: usize,
    pose: [f64; 3],
}

impl RecordedPose {
    fn scan_pose(&self) -> ScanPose {
        ScanPose {
            scan: self.scan,
            pose: planar(self.pose),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingRun {
    seed: i64,
    poses: Vec<RecordedPose>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LocalizationRun {
    seed: i64,
    /// The pose in the map from which slam_toolbox localizes.
    start: [f64; 3],
    poses: Vec<RecordedPose>,
}

/// The covariance that a pose carries: of x and y (m^2), and of the heading: of AMCL's whole
/// filter, as `amcl_pose` carries it, its heading term the circular variance, or of
/// slam_toolbox's match, as its pose topic carries it.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PoseCovariance {
    xx: f64,
    xy: f64,
    yy: f64,
    yaw: f64,
}

impl PoseCovariance {
    /// The standard deviations of the position along its least certain direction, the
    /// square root of the larger eigenvalue of the position block, and of the heading; a
    /// variance that rounding leaves below 0 reads as 0. Panics on a NaN.
    pub(crate) fn deviations(&self) -> Deviations {
        let half_trace = 0.5 * (self.xx + self.yy);
        let larger = half_trace + (0.5 * (self.xx - self.yy)).hypot(self.xy);
        let deviation = |variance: f64| {
            assert!(
                !variance.is_nan(),
                "a pose's covariance holds a NaN variance: {self:?}"
            );
            if variance < 0.0 { 0.0 } else { variance.sqrt() }
        };
        Deviations {
            position_std_m: deviation(larger),
            yaw_std_rad: deviation(self.yaw),
        }
    }
}

/// A pose that AMCL published on `amcl_pose` after the kept scan `scan`, with the
/// covariance it carries, and whether the filter's convergence test held then.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct GlobalPose {
    scan: usize,
    pose: [f64; 3],
    covariance: PoseCovariance,
    converged: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct GlobalLocalizationRun {
    seed: i64,
    poses: Vec<GlobalPose>,
}

fn planar([x, y, yaw]: [f64; 3]) -> PlanarPose {
    PlanarPose { x, y, yaw }
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> T {
    parse(path, &read_bytes(path))
}

/// The results of the scenario `name` in the results directory `results`.
fn results_of(results: &Path, name: &str) -> PathBuf {
    results.join(name)
}

/// The inputs that the reference prepared for the scenario `name` under `results`.
pub fn read_prepared(results: &Path, name: &str) -> Prepared {
    let read: PreparedRead = read_json(&results_of(results, name).join("prepared.json"));
    let ScanParametersRead {
        angle_min,
        angle_max,
        angle_increment,
        range_min,
        range_max,
        scan_time,
    } = read.laser_scan;
    Prepared {
        laser_scan: ScanParameters {
            angle_min: single(name, "a scan parameter", angle_min),
            angle_max: single(name, "a scan parameter", angle_max),
            angle_increment: single(name, "a scan parameter", angle_increment),
            range_min: single(name, "a scan parameter", range_min),
            range_max: single(name, "a scan parameter", range_max),
            scan_time: single(name, "a scan parameter", scan_time),
        },
        scans: read
            .scans
            .into_iter()
            .map(|scan| PreparedScan {
                stamp_ns: scan.stamp_ns,
                truth: planar(scan.truth),
                ranges: scan
                    .ranges
                    .into_iter()
                    .map(|range| range_of(name, "a range", range))
                    .collect(),
            })
            .collect(),
    }
}

/// The odometry that the reference drew for `seed` of the scenario `name` under
/// `results`: the base's pose in the odometry frame at each kept scan.
pub fn read_odometry(results: &Path, name: &str, seed: i64) -> Vec<PlanarPose> {
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct OdometryRead {
        seed: i64,
        poses: Vec<[f64; 3]>,
    }
    let read: OdometryRead =
        read_json(&results_of(results, name).join(format!("odometry_{seed}.json")));
    assert_eq!(read.seed, seed, "{name}: the odometry of seed {seed}");
    read.poses.into_iter().map(planar).collect()
}

fn run_file(results: &Path, name: &str, run: &str, seed: i64) -> PathBuf {
    results_of(results, name).join(format!("{run}_{seed}.json"))
}

/// The ground truth poses of the scans of `poses`, and their poses.
fn paired(prepared: &Prepared, poses: &[ScanPose]) -> (Vec<PlanarPose>, Vec<PlanarPose>) {
    poses
        .iter()
        .map(|recorded| (prepared.scans[recorded.scan].truth, recorded.pose))
        .unzip()
}

/// The trajectory error of a map whose scans `poses` hold, against their ground truth:
/// the errors after the rigid motion that best aligns the ground truth with the poses.
/// None when no alignment exists.
pub fn trajectory_errors_of(prepared: &Prepared, poses: &[ScanPose]) -> Option<metrics::Errors> {
    let (truths, poses) = paired(prepared, poses);
    trajectory_errors(&poses, &truths)
}

/// The rigid motion that takes the ground truth into the frame of a map whose scans
/// `poses` hold, in which its localizations estimate poses. None when no alignment exists.
pub fn map_from_truth(prepared: &Prepared, poses: &[ScanPose]) -> Option<metrics::PlanarMotion> {
    let (truths, poses) = paired(prepared, poses);
    best_alignment(&truths, &poses)
}

/// The errors of localized `poses` against the ground truth carried into the map by
/// `map_from_truth`. None for no pose.
pub fn localization_errors(
    prepared: &Prepared,
    poses: &[ScanPose],
    map_from_truth: metrics::PlanarMotion,
) -> Option<metrics::Errors> {
    if poses.is_empty() {
        return None;
    }
    let truths: Vec<PlanarPose> = poses
        .iter()
        .map(|pose| map_from_truth.apply(prepared.scans[pose.scan].truth))
        .collect();
    let estimates: Vec<PlanarPose> = poses.iter().map(|pose| pose.pose).collect();
    Some(metrics::errors(&estimates, &truths))
}

/// The pose of each scan that slam_toolbox added to its map of `seed` of the mapping
/// scenario `name` under `results`, after the map's last optimization.
pub fn reference_mapping_poses(results: &Path, name: &str, seed: i64) -> Vec<ScanPose> {
    let run: MappingRun = read_json(&run_file(results, name, "mapping", seed));
    assert_eq!(run.seed, seed, "{name}: the mapping run of seed {seed}");
    run.poses.iter().map(RecordedPose::scan_pose).collect()
}

/// A scan that slam_toolbox's localization added on a lidar drive spliced from two places: its
/// index among the spliced scans, its pose in the map, and the deviations of its match.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplicedAnswer {
    pub scan: usize,
    pub pose: PlanarPose,
    pub deviations: Deviations,
}

/// slam_toolbox's localization of a seed of a lidar drive spliced from two places, as the
/// reference splices it: the index of the first scan after the splice, the index of the kept
/// scan it is, and each answer.
#[derive(Debug, Clone, PartialEq)]
pub struct SplicedLocalization {
    pub splice: usize,
    pub far: usize,
    pub answers: Vec<SplicedAnswer>,
}

/// slam_toolbox's localization of `seed` on the drive of the scenario `name` spliced from
/// two places.
pub fn reference_spliced_localization(
    results: &Path,
    name: &str,
    seed: i64,
) -> SplicedLocalization {
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct AnswerRead {
        scan: usize,
        pose: [f64; 3],
        covariance: PoseCovariance,
    }
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct SplicedRead {
        seed: i64,
        #[serde(rename = "start")]
        _start: [f64; 3],
        splice: usize,
        far: usize,
        poses: Vec<AnswerRead>,
    }
    let read: SplicedRead = read_json(&run_file(results, name, "spliced_localization", seed));
    assert_eq!(
        read.seed, seed,
        "{name}: the spliced localization run of seed {seed}"
    );
    SplicedLocalization {
        splice: read.splice,
        far: read.far,
        answers: read
            .poses
            .iter()
            .map(|answer| SplicedAnswer {
                scan: answer.scan,
                pose: planar(answer.pose),
                deviations: answer.covariance.deviations(),
            })
            .collect(),
    }
}

/// The rigid motion that takes the ground truth of the map scenario `map_name` into the
/// frame of the map that its seed `map_seed` made.
pub fn reference_map_from_truth(
    results: &Path,
    map_name: &str,
    map_seed: i64,
) -> metrics::PlanarMotion {
    let prepared = read_prepared(results, map_name);
    map_from_truth(
        &prepared,
        &reference_mapping_poses(results, map_name, map_seed),
    )
    .unwrap_or_else(|| panic!("{map_name}: the map of seed {map_seed} aligns with no truth"))
}

/// `x` as a JSON number. Panics on a NaN or an infinity, which JSON has no number for.
fn number(x: f64) -> Value {
    serde_json::Number::from_f64(x).map_or_else(
        || panic!("a statistic is {x}, which JSON has no number for"),
        Value::Number,
    )
}

fn mapping_statistics(results: &Path, scenario: &Scenario, prepared: &Prepared) -> Value {
    let seeds: Vec<Value> = scenario
        .seeds
        .iter()
        .map(|&seed| {
            let poses = reference_mapping_poses(results, &scenario.name, seed);
            let errors = trajectory_errors_of(prepared, &poses).unwrap_or_else(|| {
                panic!(
                    "{}: the map of seed {seed} aligns with no truth",
                    scenario.name
                )
            });
            json!({
                "seed": seed,
                "processed_scans": poses.len(),
                "trajectory_rmse_m": number(errors.position_rmse_m),
                "trajectory_max_m": number(errors.position_max_m),
                "trajectory_yaw_rmse_rad": number(errors.yaw_rmse_rad),
            })
        })
        .collect();
    json!({ "mapping": seeds })
}

/// How near (m, rad) the start of a localization lies to the first scan's ground truth in
/// the map, which the reference program and this crate each align.
const START_AGREEMENT: f64 = 1e-9;

fn localization_statistics(
    results: &Path,
    scenario: &Scenario,
    prepared: &Prepared,
    map_scenario: &str,
    within: LocalizedWithin,
    search_limits: LocalizationLimits,
) -> Value {
    let map_seeds = scenarios()
        .into_iter()
        .find(|candidate| candidate.name == map_scenario)
        .map(|candidate| candidate.seeds)
        .unwrap_or_else(|| panic!("{}: no scenario {map_scenario} maps", scenario.name));
    let motion_of = |seed: i64| {
        assert!(
            map_seeds.contains(&seed),
            "{}: {map_scenario} maps no seed {seed}",
            scenario.name
        );
        reference_map_from_truth(results, map_scenario, seed)
    };
    let first_stamp_ns = prepared.scans[0].stamp_ns;
    let slam_toolbox: Vec<Value> = scenario
        .seeds
        .iter()
        .map(|&seed| {
            let run: LocalizationRun =
                read_json(&run_file(results, &scenario.name, "localization", seed));
            assert_eq!(
                run.seed, seed,
                "{}: the localization run of seed {seed}",
                scenario.name
            );
            // The reference started from the first scan's ground truth in the map, by the
            // same alignment, which Eigen's umeyama computed there.
            let motion = motion_of(seed);
            let (start_position, start_yaw) =
                metrics::pose_error(planar(run.start), motion.apply(prepared.scans[0].truth));
            assert!(
                start_position < START_AGREEMENT && start_yaw < START_AGREEMENT,
                "{}: the localization of seed {seed} started {start_position} m and \
                 {start_yaw} rad from the ground truth in the map",
                scenario.name
            );
            let poses: Vec<ScanPose> = run.poses.iter().map(RecordedPose::scan_pose).collect();
            let errors = localization_errors(prepared, &poses, motion);
            let error = |select: fn(&metrics::Errors) -> f64| {
                errors
                    .as_ref()
                    .map_or(Value::Null, |errors| number(select(errors)))
            };
            json!({
                "seed": seed,
                "localized_scans": poses.len(),
                "localization_rmse_m": error(|e| e.position_rmse_m),
                "localization_max_m": error(|e| e.position_max_m),
                "localization_yaw_rmse_rad": error(|e| e.yaw_rmse_rad),
            })
        })
        .collect();
    let amcl: Vec<Value> = scenario
        .seeds
        .iter()
        .map(|&seed| {
            let run: GlobalLocalizationRun = read_json(&run_file(
                results,
                &scenario.name,
                "global_localization",
                seed,
            ));
            assert_eq!(
                run.seed, seed,
                "{}: the AMCL run of seed {seed}",
                scenario.name
            );
            assert!(
                !run.poses.is_empty(),
                "{}: AMCL estimated no pose",
                scenario.name
            );
            let estimates: Vec<GlobalEstimate> = run
                .poses
                .iter()
                .map(|p| GlobalEstimate {
                    time_s: (prepared.scans[p.scan].stamp_ns - first_stamp_ns) as f64 * 1e-9,
                    pose: planar(p.pose),
                    converged: p.converged,
                    deviations: p.covariance.deviations(),
                })
                .collect();
            let motion = motion_of(seed);
            let truths: Vec<PlanarPose> = run
                .poses
                .iter()
                .map(|p| motion.apply(prepared.scans[p.scan].truth))
                .collect();
            let GlobalOutcome {
                estimates,
                converged_estimates,
                within_limits_estimates,
                least_deviations,
                localized_after_s,
                localized_by_limits_after_s,
                final_position_error_m,
                final_yaw_error_rad,
            } = global_outcome(&estimates, &truths, within, search_limits);
            json!({
                "seed": seed,
                "estimates": estimates,
                "converged_estimates": converged_estimates,
                "within_limits_estimates": within_limits_estimates,
                "least_position_std_m": number(least_deviations.position_std_m),
                "least_yaw_std_rad": number(least_deviations.yaw_std_rad),
                "localized_after_s": localized_after_s.map_or(Value::Null, number),
                "localized_by_limits_after_s": localized_by_limits_after_s.map_or(Value::Null, number),
                "final_position_error_m": number(final_position_error_m),
                "final_yaw_error_rad": number(final_yaw_error_rad),
            })
        })
        .collect();
    json!({ "localization": { "slam_toolbox": slam_toolbox, "amcl": amcl } })
}

/// Scores the runs of every room dataset scenario that the reference program wrote under
/// `results`, writes the statistics of each seed as the scenario's reference output under
/// `outputs`, and appends the path of each of the kind's [`SOURCES`], one a line, to the
/// inputs list `inputs_list`.
pub fn write_statistics(results: &Path, outputs: &Path, inputs_list: &Path) {
    let mut list = std::fs::OpenOptions::new()
        .append(true)
        .open(inputs_list)
        .unwrap_or_else(|e| panic!("cannot append to {}: {e}", inputs_list.display()));
    for source in SOURCES {
        writeln!(list, "{source}")
            .unwrap_or_else(|e| panic!("cannot append to {}: {e}", inputs_list.display()));
    }
    for scenario in scenarios() {
        let runs: RunsRead = read_json(&results.join(format!("{}.json", scenario.name)));
        let RunsRead {
            kind: ThisKind::RoomDataset,
            ..
        } = runs;
        assert!(
            runs.source.read_from(&scenario.source) && runs.seeds == scenario.seeds,
            "{}: the reference ran the scenario's source and seeds",
            scenario.name
        );
        let task_name = match scenario.task {
            Task::Mapping { .. } => "mapping",
            Task::Localization { .. } => "localization",
        };
        assert_eq!(
            runs.task, task_name,
            "{}: the reference ran the scenario's task",
            scenario.name
        );
        let prepared = read_prepared(results, &scenario.name);
        assert_eq!(
            prepared.scans.len(),
            runs.scans,
            "{}: the scans",
            scenario.name
        );
        let task = match &scenario.task {
            Task::Mapping { .. } => mapping_statistics(results, &scenario, &prepared),
            Task::Localization {
                map_scenario,
                localized_within,
                search_limits,
                ..
            } => localization_statistics(
                results,
                &scenario,
                &prepared,
                map_scenario,
                *localized_within,
                *search_limits,
            ),
        };
        let inputs: BTreeSet<&str> = runs
            .inputs
            .iter()
            .map(String::as_str)
            .chain(SOURCES)
            .collect();
        let output = json!({
            "kind": "room_dataset",
            "source": serde_json::to_value(&runs.source).expect("the source serializes"),
            "task": task,
            "inputs": inputs,
            "datasets_read": runs.datasets_read,
            "scans": runs.scans,
            "runs": runs.runs,
        });
        write_json(&outputs.join(format!("{}.json", scenario.name)), &output);
    }
}

/// The statistics of one mapping seed: the scans that slam_toolbox added to the map, and
/// the errors of their poses after the rigid motion that best aligns the ground truth
/// with them.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappingSeed {
    pub seed: i64,
    pub processed_scans: usize,
    pub trajectory_rmse_m: f64,
    pub trajectory_max_m: f64,
    pub trajectory_yaw_rmse_rad: f64,
}

/// The statistics of one slam_toolbox localization seed: the scans it localized, and
/// their errors against the ground truth in the map's frame, null when it localized none.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LocalizationSeed {
    pub seed: i64,
    pub localized_scans: usize,
    #[serde(deserialize_with = "Option::deserialize")]
    pub localization_rmse_m: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    pub localization_max_m: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    pub localization_yaw_rmse_rad: Option<f64>,
}

/// The statistics of one AMCL global localization seed, as [`GlobalOutcome`] defines them,
/// each time null when the run did not localize for good.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GlobalLocalizationSeed {
    pub seed: i64,
    pub estimates: usize,
    pub converged_estimates: usize,
    pub within_limits_estimates: usize,
    pub least_position_std_m: f64,
    pub least_yaw_std_rad: f64,
    #[serde(deserialize_with = "Option::deserialize")]
    pub localized_after_s: Option<f64>,
    #[serde(deserialize_with = "Option::deserialize")]
    pub localized_by_limits_after_s: Option<f64>,
    pub final_position_error_m: f64,
    pub final_yaw_error_rad: f64,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LocalizationStatistics {
    pub slam_toolbox: Vec<LocalizationSeed>,
    pub amcl: Vec<GlobalLocalizationSeed>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum TaskStatistics {
    Mapping(Vec<MappingSeed>),
    Localization(LocalizationStatistics),
}

/// The committed statistics of a room dataset scenario.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Statistics {
    #[serde(skip)]
    pub name: String,
    pub kind: ThisKind,
    pub source: SourceRead,
    pub task: TaskStatistics,
    /// The repository files that the reference read and the kind's sources, whose digests
    /// `INPUTS.SHA256SUMS` holds.
    pub inputs: Vec<String>,
    /// Each file of the dataset cache that the reference read, `<sequence>/<path>` of a TUM
    /// sequence.
    pub datasets_read: Vec<String>,
    /// The kept scans.
    pub scans: usize,
    pub runs: Vec<String>,
}

impl Statistics {
    /// The committed statistics of every room dataset scenario, in the order of their
    /// names, each checked against its digest and against its scenario's seeds.
    pub fn load_all() -> Vec<Self> {
        scenarios()
            .into_iter()
            .map(|scenario| {
                let mut statistics: Statistics = read_reference(&scenario.name);
                statistics.name = scenario.name.clone();
                let seeds: Vec<i64> = match &statistics.task {
                    TaskStatistics::Mapping(seeds) => seeds.iter().map(|s| s.seed).collect(),
                    TaskStatistics::Localization(statistics) => {
                        assert_eq!(
                            statistics
                                .slam_toolbox
                                .iter()
                                .map(|s| s.seed)
                                .collect::<Vec<_>>(),
                            statistics.amcl.iter().map(|s| s.seed).collect::<Vec<_>>(),
                            "{}: slam_toolbox and AMCL ran the same seeds",
                            scenario.name
                        );
                        statistics.slam_toolbox.iter().map(|s| s.seed).collect()
                    }
                };
                assert_eq!(
                    seeds, scenario.seeds,
                    "{}: the statistics of each seed",
                    scenario.name
                );
                statistics
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_room_dataset_scenario_parses_with_its_task() {
        let scenarios = scenarios();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a room dataset scenario"
        );
        for scenario in &scenarios {
            assert!(!scenario.seeds.is_empty(), "{} names seeds", scenario.name);
            if let Task::Localization { map_scenario, .. } = &scenario.task {
                assert!(
                    scenarios
                        .iter()
                        .any(|candidate| &candidate.name == map_scenario
                            && matches!(candidate.task, Task::Mapping { .. })),
                    "{} localizes in {map_scenario}, which maps",
                    scenario.name
                );
            }
        }
    }

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let path = json_file(&scenario_directory(), "room_dataset_pioneer_slam");
        let text = String::from_utf8(read_bytes(&path)).expect("UTF-8");
        let of_kind = |kind: &str| {
            serde_json::from_str::<Scenario>(
                &text.replace(r#""kind": "room_dataset""#, &format!(r#""kind": "{kind}""#)),
            )
        };
        assert!(of_kind("occupancy").is_err());
        assert!(of_kind("room_dataset").is_ok());
    }

    /// The covariance of a position whose deviations are `major` (m) along `angle` (rad) and
    /// `minor` across it, and of a heading of variance `yaw`.
    fn rotated(major: f64, minor: f64, angle: f64, yaw: f64) -> PoseCovariance {
        let (sin, cos) = angle.sin_cos();
        let (a, b) = (major * major, minor * minor);
        PoseCovariance {
            xx: a * cos * cos + b * sin * sin,
            xy: (a - b) * sin * cos,
            yy: a * sin * sin + b * cos * cos,
            yaw,
        }
    }

    #[test]
    fn the_position_deviation_is_the_root_of_the_larger_eigenvalue() {
        for (major, minor, angle) in [(0.4, 0.1, 0.6), (0.4, 0.1, 2.2), (0.25, 0.25, 1.0)] {
            let deviations = rotated(major, minor, angle, 0.09).deviations();
            assert!(
                (deviations.position_std_m - major).abs() < 1e-12,
                "{major} {minor} {angle}: {deviations:?}"
            );
            assert!((deviations.yaw_std_rad - 0.3).abs() < 1e-12);
        }
    }

    #[test]
    fn a_variance_that_rounding_leaves_below_zero_reads_as_zero() {
        let deviations = PoseCovariance {
            xx: -1e-18,
            xy: 0.0,
            yy: -2e-18,
            yaw: -1e-17,
        }
        .deviations();
        assert_eq!(
            (deviations.position_std_m, deviations.yaw_std_rad),
            (0.0, 0.0)
        );
    }

    #[test]
    #[should_panic(expected = "NaN variance")]
    fn a_nan_variance_is_refused() {
        rotated(0.4, 0.1, 0.6, f64::NAN).deviations();
    }

    #[test]
    fn the_sources_are_every_file_of_the_kinds_code() {
        let root = crate::repository_root();
        for source in SOURCES {
            assert!(root.join(source).is_file(), "{source} is a file");
        }
        let programs = root.join("ros_reference/reference_programs");
        for entry in std::fs::read_dir(&programs).expect("reference_programs/ reads") {
            let name = entry
                .expect("an entry")
                .file_name()
                .into_string()
                .expect("UTF-8");
            if [
                "room_dataset",
                "amcl_map_cspace",
                "amcl_node",
                "laser_projection",
                "nav2_node",
                "point_cloud",
            ]
            .iter()
            .any(|prefix| name.starts_with(prefix))
            {
                let path = format!("ros_reference/reference_programs/{name}");
                assert!(SOURCES.contains(&path.as_str()), "SOURCES lists {path}");
            }
        }
    }

    #[test]
    fn the_committed_statistics_pair_with_their_scenarios() {
        for statistics in Statistics::load_all() {
            assert!(statistics.scans > 1, "{} keeps scans", statistics.name);
        }
    }
}
