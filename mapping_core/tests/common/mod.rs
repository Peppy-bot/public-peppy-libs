//! What the integration suites share.
//!
//! Each integration test is its own crate, so items unused by one suite are expected; the
//! allow keeps clippy's -D warnings gate meaningful for the rest.
#![allow(dead_code)]

pub mod properties;
#[path = "../../src/slam_toolbox_parameters.rs"]
mod slam_toolbox_parameters;

use camera_core::laser_scan::{LaserScan, Sweep};
use camera_core::pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;
use mapping_core::{
    AmclParameters, LocalizationLimits, LocalizationParameters, Mapping, MappingParameters,
    PlacedScan, RoomMap,
};
use properties::Room;
use ros_reference::room_dataset::RosParameters;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

/// slam_toolbox's parameters of `mapper_params_online_sync.yaml`, with the laser ranges of
/// the drawn scans.
pub fn sync_parameters() -> MappingParameters {
    slam_toolbox_parameters::mapper_params_online_sync(0.05, properties::SCAN_RANGE_MAX)
}

/// slam_toolbox's parameters of `mapper_params_localization.yaml`, with the laser ranges
/// of the drawn scans: those of [`sync_parameters`] with a shorter buffer and chains.
pub fn slam_toolbox_localization() -> MappingParameters {
    MappingParameters {
        scan_buffer_size: 3,
        loop_match_minimum_chain_size: 3,
        ..sync_parameters()
    }
}

/// The largest standard deviations that hold the base of the drawn rooms localized: a
/// tenth of a meter and of a radian, twice and three times the map's cell and the
/// matcher's coarse angle step.
pub fn matching_limits() -> LocalizationLimits {
    LocalizationLimits {
        position_std_m: PositiveFinite::parse(0.1).expect("positive"),
        yaw_std_rad: PositiveFinite::parse(0.1).expect("positive"),
    }
}

/// How far a localized pose may lie from the truth (m): two cells of the map's grid.
pub const LOCALIZED_POSITION: f64 = 0.1;

/// slam_toolbox's localization mode with [`matching_limits`].
pub fn localization_parameters() -> LocalizationParameters {
    LocalizationParameters {
        slam_toolbox: slam_toolbox_localization(),
        matching_limits: matching_limits(),
    }
}

/// When the maps of the tests started (ns since the Unix epoch).
pub const CREATED_AT_UNIX_NS: i64 = 1_793_404_800_000_000_000;

/// The time between two scans of a drawn drive (ns).
pub const SCAN_PERIOD_NS: i64 = 1_000_000_000;

/// The map of a drive through `room`, with each scan at its pose in `drive` and the base at
/// `odometry` by odometry.
pub fn map_of(
    room: &Room,
    drive: &[PlanarPose],
    odometry: &[PlanarPose],
    parameters: MappingParameters,
) -> Mapping {
    let mut mapping = Mapping::start(parameters, PlanarPose::IDENTITY).expect("valid parameters");
    for (index, (truth, odometry)) in drive.iter().zip(odometry).enumerate() {
        mapping
            .add(&placed(
                index as i64 * SCAN_PERIOD_NS,
                *odometry,
                &room.scan(*truth),
            ))
            .expect("a scan");
    }
    mapping
}

/// The map of a drive through `room`, as [`map_of`] makes it, finished.
pub fn room_map_of(
    room: &Room,
    drive: &[PlanarPose],
    odometry: &[PlanarPose],
    parameters: MappingParameters,
) -> RoomMap {
    map_of(room, drive, odometry, parameters)
        .finish(CREATED_AT_UNIX_NS, localization_parameters())
        .expect("valid parameters")
        .expect("the drive maps the room")
        .0
}

/// A directory of its own under the integration tests' scratch directory.
pub fn scratch_directory(name: &str) -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let index = NEXT.fetch_add(1, Ordering::Relaxed);
    let directory = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("{name}-{}-{index}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory);
    directory
}

/// `scan` at `stamp_ns`, taken by a still laser at the base's origin facing ahead, the
/// base at `odometry` in the odometry frame.
pub fn placed(stamp_ns: i64, odometry: PlanarPose, scan: &LaserScan) -> PlacedScan {
    let still = Sweep {
        first_ray: odometry,
        last_ray: odometry,
    };
    PlacedScan::new(stamp_ns, scan, &still, odometry).expect("a still laser's scan")
}

/// AMCL's parameters of the node that the search does not read: its beam model's and its beam
/// skipping's, which the likelihood field model leaves unread, its transform tolerance, and
/// its recovery rates, which the search holds at 0.
pub const AMCL_UNREAD: [&str; 11] = [
    "beam_skip_distance",
    "beam_skip_error_threshold",
    "beam_skip_threshold",
    "do_beamskip",
    "lambda_short",
    "laser_model_type",
    "recovery_alpha_fast",
    "recovery_alpha_slow",
    "transform_tolerance",
    "z_max",
    "z_short",
];

/// AMCL's parameters `amcl` of the scenario `name`, as the search takes them; panics on a parameter
/// that the search neither reads nor lists in [`AMCL_UNREAD`], and on a model or a
/// recovery rate that the search does not run.
pub fn amcl_parameters(name: &str, amcl: &RosParameters) -> AmclParameters {
    let number = |key: &str| {
        amcl.get(key)
            .and_then(serde_json::Value::as_f64)
            .unwrap_or_else(|| panic!("{name}: AMCL's {key} is a number"))
    };
    let count = |key: &str| {
        amcl.get(key)
            .and_then(serde_json::Value::as_u64)
            .and_then(|value| u32::try_from(value).ok())
            .unwrap_or_else(|| panic!("{name}: AMCL's {key} is a count"))
    };
    assert_eq!(
        amcl.get("laser_model_type"),
        Some(&serde_json::json!("likelihood_field")),
        "{name}: AMCL weighs by the likelihood field model"
    );
    assert_eq!(
        amcl.get("do_beamskip"),
        Some(&serde_json::json!(false)),
        "{name}: AMCL skips no beam"
    );
    for recovery in ["recovery_alpha_fast", "recovery_alpha_slow"] {
        assert_eq!(number(recovery), 0.0, "{name}: AMCL's {recovery} is 0");
    }
    let parameters = AmclParameters {
        alpha1: number("alpha1"),
        alpha2: number("alpha2"),
        alpha3: number("alpha3"),
        alpha4: number("alpha4"),
        alpha5: number("alpha5"),
        laser_likelihood_max_dist: number("laser_likelihood_max_dist"),
        laser_max_range: number("laser_max_range"),
        laser_min_range: number("laser_min_range"),
        max_beams: count("max_beams"),
        max_particles: count("max_particles"),
        min_particles: count("min_particles"),
        pf_err: number("pf_err"),
        pf_z: number("pf_z"),
        resample_interval: count("resample_interval"),
        robot_model_type: serde_json::from_value(amcl["robot_model_type"].clone())
            .unwrap_or_else(|e| panic!("{name}: AMCL's robot_model_type: {e}")),
        sigma_hit: number("sigma_hit"),
        update_min_a: number("update_min_a"),
        update_min_d: number("update_min_d"),
        z_hit: number("z_hit"),
        z_rand: number("z_rand"),
    };
    const READ: [&str; 20] = [
        "alpha1",
        "alpha2",
        "alpha3",
        "alpha4",
        "alpha5",
        "laser_likelihood_max_dist",
        "laser_max_range",
        "laser_min_range",
        "max_beams",
        "max_particles",
        "min_particles",
        "pf_err",
        "pf_z",
        "resample_interval",
        "robot_model_type",
        "sigma_hit",
        "update_min_a",
        "update_min_d",
        "z_hit",
        "z_rand",
    ];
    for key in amcl.keys() {
        assert!(
            READ.contains(&key.as_str()) || AMCL_UNREAD.contains(&key.as_str()),
            "{name}: the search reads AMCL's {key}"
        );
    }
    parameters
}
