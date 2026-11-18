//! Parity with slam_toolbox 2.10.0 on the room dataset scenarios of the lidar drive, on
//! the scans and the seeded odometries that the reference prepared, which both sides
//! read. slam_toolbox reads each heading of the odometry through tf2's buffer, whose
//! rounding this crate does not reproduce to the last bit, and a match whose candidates lie
//! on the edges of cells can take the neighboring cell from a difference in the last bit, so
//! each comparison is of the statistics of the ten seeds, scored by `ros_reference::metrics`
//! as the committed statistics are: the mean of ours lies at most twice the standard error
//! of the reference's ten values above their mean. On each side, a run of a seed localizes
//! in the map that the same seed made, so the seeds sample the maps too. Each test prints
//! every seed's values and their difference.
//!
//! The tests that compare with a reference on the same scans take every ray at its scan's
//! stamp, as the references take it: the trajectory error of mapping, the error of
//! localization in each seed's map of the mapping scenario, and on the drive spliced
//! from two places of the map the error and the position deviation of the localization
//! after the splice. The other test takes each ray at the time the drive cast it, as the
//! library runs on a base: the first seed's map, saved and loaded, closes its loop.
//!
//! The scenarios of the room dataset kind run on a developer's machine alone, through
//! `ros_reference/run.sh --datasets`, which names the results directory of the references
//! in `ROS_REFERENCE_RESULTS`.

use camera_core::laser_scan::{LaserScan, Sweep};
use camera_core::pose::PlanarPose;
use mapping_core::{
    Localization, LocalizationParameters, Mapping, MappingParameters, PlacedScan, RoomMap,
};
use ros_reference::metrics::PlanarPose as HarnessPose;
use ros_reference::metrics::{self, Mean, PlanarMotion};
use ros_reference::results_directory;
use ros_reference::room_dataset::{
    LidarDrive, LocalizationSeed, MappingSeed, Prepared, PreparedScan, ScanPose, Scenario, Source,
    Statistics, Task, TaskStatistics, localization_errors, map_from_truth, read_odometry,
    read_prepared, reference_map_from_truth, reference_mapping_poses,
    reference_spliced_localization, scenarios, trajectory_errors_of,
};
use std::collections::BTreeMap;
use std::path::Path;

mod common;
use common::properties::{compose, relative};
use common::{localization_parameters, matching_limits, placed, scratch_directory};

/// The parameters of slam_toolbox's node that name its ROS interfaces, its solver plugin
/// and its logging, each with the value that the port follows, or none for any value.
const ROS_PARAMETERS: [(&str, Option<&str>); 13] = [
    ("solver_plugin", Some("\"solver_plugins::CeresSolver\"")),
    ("ceres_linear_solver", Some("\"SPARSE_NORMAL_CHOLESKY\"")),
    ("ceres_preconditioner", None),
    ("ceres_trust_strategy", Some("\"LEVENBERG_MARQUARDT\"")),
    ("ceres_dogleg_type", None),
    ("ceres_loss_function", Some("\"None\"")),
    ("scan_topic", None),
    ("debug_logging", None),
    ("restamp_tf", None),
    ("transform_timeout", None),
    ("tf_buffer_duration", None),
    ("enable_interactive_mode", Some("false")),
    ("use_scan_matching", Some("true")),
];

/// The largest gap (s) between a scan period and the sweep of the drive's scans, by which
/// the next scan's odometry is the base's at a scan's last ray: a microsecond, above the
/// single precision rounding of the drive's time increment over its rays, at most a scan
/// period x 2^-24, 6e-9 s at 10 Hz.
const SWEEP_TOLERANCE_S: f64 = 1e-6;

/// slam_toolbox's parameters of `scenario`, but for those of [`ROS_PARAMETERS`], which hold
/// the values that the port follows.
fn slam_toolbox_parameters(scenario: &Scenario) -> MappingParameters {
    let mut parameters = scenario.slam_toolbox.clone();
    for (name, expected) in ROS_PARAMETERS {
        let value = parameters
            .remove(name)
            .unwrap_or_else(|| panic!("{}: slam_toolbox's {name} is set", scenario.name));
        if let Some(expected) = expected {
            assert_eq!(
                value.to_string(),
                expected,
                "{}: slam_toolbox's {name} is the port's",
                scenario.name
            );
        }
    }
    let object = parameters.into_iter().collect::<serde_json::Map<_, _>>();
    serde_json::from_value(serde_json::Value::Object(object))
        .unwrap_or_else(|e| panic!("{}: slam_toolbox's parameters: {e}", scenario.name))
}

/// A pose of the harness as this crate holds it.
fn ours(pose: HarnessPose) -> PlanarPose {
    PlanarPose::new(pose.x, pose.y, pose.yaw).expect("a finite pose of the harness")
}

/// A pose of this crate as the harness holds it.
fn harness(pose: PlanarPose) -> HarnessPose {
    HarnessPose {
        x: pose.x(),
        y: pose.y(),
        yaw: pose.yaw(),
    }
}

/// The drive of a scenario on the lidar drive.
fn drive_of(scenario: &Scenario) -> Option<&LidarDrive> {
    match &scenario.source {
        Source::LidarDrive(drive) => Some(drive),
        Source::TumRgbd(_) => None,
    }
}

/// When the rays of a scan stand: every ray at its scan's stamp, as the references take
/// them, or each ray at the time the drive cast it, the drive's time increment (s) after
/// the one before.
#[derive(Debug, Clone, Copy)]
enum RayTimes {
    AtTheStamp,
    AsCast(f64),
}

impl RayTimes {
    fn as_cast(scenario: &Scenario) -> RayTimes {
        RayTimes::AsCast(
            drive_of(scenario)
                .expect("a scenario of the drive")
                .time_increment_s(),
        )
    }
}

/// The scans of `prepared` with the base at `odometry`, the lidar above the base's origin
/// facing ahead, their rays standing as `rays` gives: every ray at the stamp, every scan
/// taken by a still lidar; or each ray as cast, the lidar at a scan's last ray where the
/// next scan's odometry puts it, the drive sweeping a scan period, so every scan but the
/// last.
fn placed_scans(prepared: &Prepared, odometry: &[HarnessPose], rays: RayTimes) -> Vec<PlacedScan> {
    let p = prepared.laser_scan;
    let laser_scan = |scan: &PreparedScan, time_increment_s: f32| {
        LaserScan::parse(
            p.angle_min,
            p.angle_increment,
            time_increment_s,
            p.range_min,
            p.range_max,
            &scan.ranges,
        )
        .expect("a prepared scan")
    };
    match rays {
        RayTimes::AtTheStamp => prepared
            .scans
            .iter()
            .zip(odometry)
            .map(|(scan, &pose)| placed(scan.stamp_ns, ours(pose), &laser_scan(scan, 0.0)))
            .collect(),
        RayTimes::AsCast(time_increment_s) => prepared
            .scans
            .windows(2)
            .zip(odometry.windows(2))
            .map(|(scans, poses)| {
                let scan = laser_scan(&scans[0], time_increment_s as f32);
                let period_s = (scans[1].stamp_ns - scans[0].stamp_ns) as f64 * 1e-9;
                assert!(
                    (scan.sweep_s() - period_s).abs() <= SWEEP_TOLERANCE_S,
                    "the drive sweeps {} s a scan, {period_s} s apart",
                    scan.sweep_s()
                );
                let sweep = Sweep {
                    first_ray: ours(poses[0]),
                    last_ray: ours(poses[1]),
                };
                PlacedScan::new(scans[0].stamp_ns, &scan, &sweep, ours(poses[0]))
                    .expect("a scan of finite poses")
            })
            .collect(),
    }
}

/// The map of one seed of a mapping scenario, and the kept scan of each of its scans.
fn map_seed(
    scenario: &Scenario,
    prepared: &Prepared,
    odometry: &[HarnessPose],
    rays: RayTimes,
) -> (Mapping, Vec<ScanPose>) {
    let name = &scenario.name;
    let mut mapping = Mapping::start(slam_toolbox_parameters(scenario), PlanarPose::IDENTITY)
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    let mut kept = Vec::new();
    for (index, scan) in placed_scans(prepared, odometry, rays).iter().enumerate() {
        if mapping
            .add(scan)
            .unwrap_or_else(|e| panic!("{name}: scan {index}: {e}"))
            .is_some()
        {
            kept.push(index);
        }
    }
    // The graph's ids follow the order the scans were added in.
    let poses = mapping
        .poses()
        .into_iter()
        .zip(kept)
        .map(|((_, pose), scan)| ScanPose {
            scan,
            pose: harness(pose),
        })
        .collect();
    (mapping, poses)
}

/// The map of a mapping, finished, made at no time in particular.
fn finished(name: &str, mapping: Mapping) -> RoomMap {
    mapping
        .finish(0, localization_parameters())
        .unwrap_or_else(|e| panic!("{name}: {e}"))
        .unwrap_or_else(|| panic!("{name}: a map"))
        .0
}

/// This crate's map of each seed of each mapping scenario of the drive, by the scenario's
/// name and the seed, its rays standing as `rays_of` gives for the scenario, saved and
/// loaded, and the rigid motion that carries the scenario's ground truth into it.
fn our_maps(
    results: &Path,
    all: &[Scenario],
    rays_of: fn(&Scenario) -> RayTimes,
) -> BTreeMap<(String, i64), (RoomMap, PlanarMotion)> {
    all.iter()
        .filter(|scenario| {
            matches!(scenario.task, Task::Mapping { .. }) && drive_of(scenario).is_some()
        })
        .flat_map(|scenario| {
            let name = &scenario.name;
            let prepared = read_prepared(results, name);
            scenario.seeds.iter().map(move |&seed| {
                let (mapping, poses) = map_seed(
                    scenario,
                    &prepared,
                    &read_odometry(results, name, seed),
                    rays_of(scenario),
                );
                let directory = scratch_directory(&format!("{name}_map_{seed}"));
                finished(name, mapping)
                    .save(&directory)
                    .expect("the map saves");
                let map = RoomMap::load(&directory).expect("the map loads");
                let motion = map_from_truth(&prepared, &poses).expect("the map aligns");
                ((name.clone(), seed), (map, motion))
            })
        })
        .collect()
}

/// The map of `maps` that a run of `seed` of a localization scenario localizes in: the one
/// that the same seed of its map scenario made.
fn map_named<'a>(
    maps: &'a BTreeMap<(String, i64), (RoomMap, PlanarMotion)>,
    map_scenario: &str,
    seed: i64,
) -> &'a (RoomMap, PlanarMotion) {
    maps.get(&(map_scenario.to_owned(), seed))
        .unwrap_or_else(|| panic!("{map_scenario} maps no seed {seed} of the drive"))
}

/// The localization parameters of a scenario: its slam_toolbox's, and the matching limits
/// of room_map.
fn localization_parameters_of(scenario: &Scenario) -> LocalizationParameters {
    LocalizationParameters {
        slam_toolbox: slam_toolbox_parameters(scenario),
        matching_limits: matching_limits(),
    }
}

fn statistics_of(name: &str) -> TaskStatistics {
    Statistics::load_all()
        .into_iter()
        .find(|statistics| statistics.name == name)
        .unwrap_or_else(|| panic!("{name} has committed statistics"))
        .task
}

/// The reference that a statistic of ours is compared with.
#[derive(Debug, Clone, Copy)]
enum Reference {
    SlamToolbox,
}

impl Reference {
    fn name(self) -> &'static str {
        match self {
            Reference::SlamToolbox => "slam_toolbox",
        }
    }
}

/// The comparison of one statistic over the seeds: each seed's value here and in
/// `reference`, and whether our mean lies at most twice the reference's standard error
/// above its mean.
fn compare(reference: Reference, name: &str, statistic: &str, rows: &[(i64, f64, f64)]) -> bool {
    let ours: Vec<f64> = rows.iter().map(|&(_, ours, _)| ours).collect();
    let theirs: Vec<f64> = rows.iter().map(|&(.., theirs)| theirs).collect();
    let mean_of = |values: &[f64]| metrics::mean(values).expect("ten seeds");
    let (
        Mean {
            mean: our_mean,
            standard_error: our_error,
        },
        Mean {
            mean: their_mean,
            standard_error: their_error,
        },
    ) = (mean_of(&ours), mean_of(&theirs));
    println!("{name}: {statistic}");
    let reference = reference.name();
    println!("| Seed | mapping_core | {reference} | Difference |");
    println!("|---|---|---|---|");
    for &(seed, ours, theirs) in rows {
        println!(
            "| {seed} | {ours:.3} | {theirs:.3} | {:+.3} |",
            ours - theirs
        );
    }
    println!(
        "| Mean ± standard error | {our_mean:.3} ± {our_error:.3} | {their_mean:.3} ± \
         {their_error:.3} | {:+.3} |",
        our_mean - their_mean
    );
    let bound = their_mean + 2.0 * their_error;
    println!(
        "our mean {our_mean:.4} against {reference}'s bound {bound:.4}, its mean and twice \
         its standard error\n"
    );
    our_mean <= bound
}

/// Fails the test naming each comparison of `held` that does not hold.
fn assert_all_hold(held: &[(String, &str, bool)], what: &str) {
    assert!(!held.is_empty(), "a scenario of the drive {what}");
    let worse: Vec<_> = held
        .iter()
        .filter(|(.., holds)| !holds)
        .map(|(name, statistic, _)| format!("{name}: {statistic}"))
        .collect();
    assert!(
        worse.is_empty(),
        "mapping_core is worse than the reference on {worse:?}"
    );
}

#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn mapping_is_no_worse_than_slam_toolboxs() {
    let results = results_directory();
    let mut held = Vec::new();
    for scenario in scenarios() {
        let (Task::Mapping { .. }, Some(_)) = (&scenario.task, drive_of(&scenario)) else {
            continue;
        };
        let prepared = read_prepared(&results, &scenario.name);
        let TaskStatistics::Mapping(reference) = statistics_of(&scenario.name) else {
            panic!("{} maps", scenario.name);
        };
        let rows: Vec<(i64, f64, f64)> = scenario
            .seeds
            .iter()
            .zip(&reference)
            .map(
                |(
                    &seed,
                    MappingSeed {
                        trajectory_rmse_m, ..
                    },
                )| {
                    let odometry = read_odometry(&results, &scenario.name, seed);
                    let (_, poses) =
                        map_seed(&scenario, &prepared, &odometry, RayTimes::AtTheStamp);
                    let errors = trajectory_errors_of(&prepared, &poses)
                        .unwrap_or_else(|| panic!("{} seed {seed}: the map aligns", scenario.name));
                    (seed, errors.position_rmse_m, *trajectory_rmse_m)
                },
            )
            .collect();
        let statistic = "trajectory RMSE (m)";
        held.push((
            scenario.name.clone(),
            statistic,
            compare(Reference::SlamToolbox, &scenario.name, statistic, &rows),
        ));
    }
    assert_all_hold(&held, "maps");
}

/// The largest distance (m) between the hits of the scan `late` placed at its pose in the
/// map and at `expected`.
fn wall_displacement(prepared: &Prepared, late: &ScanPose, expected: HarnessPose) -> f64 {
    let p = prepared.laser_scan;
    prepared.scans[late.scan]
        .ranges
        .iter()
        .enumerate()
        .filter(|&(_, &range)| range >= p.range_min && range < p.range_max)
        .map(|(beam, &range)| {
            let bearing = f64::from(p.angle_min) + beam as f64 * f64::from(p.angle_increment);
            let range = f64::from(range);
            let place = |pose: HarnessPose| {
                let angle = pose.yaw + bearing;
                (pose.x + range * angle.cos(), pose.y + range * angle.sin())
            };
            let (a, b) = (place(late.pose), place(expected));
            (a.0 - b.0).hypot(a.1 - b.1)
        })
        .fold(0.0, f64::max)
}

/// The scan of a map that revisits its first: the last scan, after half of the scans, whose
/// ground truth lies within 0.5 m of the first scan's.
fn revisit<'a>(prepared: &Prepared, poses: &'a [ScanPose]) -> Option<&'a ScanPose> {
    let first = prepared.scans[poses.first()?.scan].truth;
    poses[poses.len() / 2..].iter().rev().find(|pose| {
        let (position, _) = metrics::pose_error(prepared.scans[pose.scan].truth, first);
        position <= 0.5
    })
}

/// The gap of the loop at the scan that revisits the first: the largest distance (m) between
/// the hits of that scan placed at its pose in the map and placed where the first scan's
/// pose in the map and the ground truth's motion from the first scan to it put it.
fn loop_gap(prepared: &Prepared, poses: &[ScanPose]) -> Option<(usize, f64)> {
    let first = poses.first()?;
    let late = revisit(prepared, poses)?;
    let truth_motion = relative(
        ours(prepared.scans[first.scan].truth),
        ours(prepared.scans[late.scan].truth),
    );
    let expected = harness(compose(ours(first.pose), truth_motion));
    Some((late.scan, wall_displacement(prepared, late, expected)))
}

#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn the_first_seeds_saved_map_of_the_drive_closes_its_loop() {
    let results = results_directory();
    let mut open = Vec::new();
    let mut maps = 0;
    for scenario in scenarios() {
        let (Task::Mapping { .. }, Some(_)) = (&scenario.task, drive_of(&scenario)) else {
            continue;
        };
        maps += 1;
        let prepared = read_prepared(&results, &scenario.name);
        let seed = scenario.seeds[0];
        let (mapping, poses) = map_seed(
            &scenario,
            &prepared,
            &read_odometry(&results, &scenario.name, seed),
            RayTimes::as_cast(&scenario),
        );
        let map = finished(&scenario.name, mapping);
        let directory = scratch_directory(&scenario.name);
        map.save(&directory).expect("the map saves");
        let loaded = RoomMap::load(&directory).expect("the map loads");
        assert_eq!(
            loaded, map,
            "{}: the saved map loads as itself",
            scenario.name
        );
        let resolution = slam_toolbox_parameters(&scenario).resolution;
        let reference_poses = reference_mapping_poses(&results, &scenario.name, seed);
        let ours = loop_gap(&prepared, &poses);
        let theirs = loop_gap(&prepared, &reference_poses);
        println!(
            "{} seed {seed}: the loop's gap at the scan that revisits the first (scan, m) \
             {ours:?} here and {theirs:?} in slam_toolbox's map, against a resolution of \
             {resolution} m",
            scenario.name
        );
        if !ours.is_some_and(|(_, gap)| gap <= resolution) {
            open.push(scenario.name.clone());
        }
    }
    assert!(maps > 0, "a room dataset scenario maps the drive");
    assert!(
        open.is_empty(),
        "the first seed's map leaves its loop open on {open:?}"
    );
}

/// The prepared scans of a scenario and their odometry spliced from two places, as
/// [`spliced`] splices them: the index of the first scan after the splice, `splice`, and of
/// the scenario's scan it is, `far`, and how far apart the two places lie (m).
struct Spliced {
    prepared: Prepared,
    odometry: Vec<HarnessPose>,
    splice: usize,
    far: usize,
    distance_m: f64,
}

/// The prepared scans of `prepared` and their odometry, spliced from two places: the first
/// half of the scans, then the scans from the one, among the first half of the rest, whose
/// ground truth lies farthest from the last of the first half, a scan period after it, their
/// odometry moved rigidly to go on from it, as a base carried to another place reads its
/// odometry.
fn spliced(prepared: &Prepared, odometry: &[HarnessPose]) -> Spliced {
    let half = prepared.scans.len() / 2;
    let last = &prepared.scans[half - 1];
    let rest = prepared.scans.len() - half;
    let (far, distance) = (half..half + rest / 2)
        .map(|index| {
            let (distance, _) = metrics::pose_error(prepared.scans[index].truth, last.truth);
            (index, distance)
        })
        .fold((half, 0.0), |best, candidate| {
            if candidate.1 > best.1 {
                candidate
            } else {
                best
            }
        });
    let period_ns = prepared.scans[1].stamp_ns - prepared.scans[0].stamp_ns;
    let carried = ours(odometry[half - 1]);
    let from = ours(odometry[far]);
    let scans: Vec<PreparedScan> = prepared.scans[..half]
        .iter()
        .cloned()
        .chain(prepared.scans[far..].iter().map(|scan| PreparedScan {
            stamp_ns: last.stamp_ns + period_ns + (scan.stamp_ns - prepared.scans[far].stamp_ns),
            ..scan.clone()
        }))
        .collect();
    let spliced_odometry: Vec<HarnessPose> = odometry[..half]
        .iter()
        .copied()
        .chain(
            odometry[far..]
                .iter()
                .map(|pose| harness(compose(carried, relative(from, ours(*pose))))),
        )
        .collect();
    Spliced {
        prepared: Prepared {
            laser_scan: prepared.laser_scan,
            scans,
        },
        odometry: spliced_odometry,
        splice: half,
        far,
        distance_m: distance,
    }
}

/// The mean position error (m) against `truths` and the mean position deviation (m) of the
/// answers of `answers` from the scan `splice` on, each a scan's index and its pose and
/// position deviation.
fn after_the_splice(
    name: &str,
    answers: &[(usize, PlanarPose, f64)],
    splice: usize,
    truths: &[PlanarPose],
) -> (f64, f64) {
    let after: Vec<&(usize, PlanarPose, f64)> = answers
        .iter()
        .filter(|(index, ..)| *index >= splice)
        .collect();
    assert!(
        !after.is_empty(),
        "{name}: the localization answers after the splice"
    );
    let count = after.len() as f64;
    let error = after
        .iter()
        .map(|(index, pose, _)| {
            (pose.x() - truths[*index].x()).hypot(pose.y() - truths[*index].y())
        })
        .sum::<f64>()
        / count;
    let deviation = after.iter().map(|(.., deviation)| deviation).sum::<f64>() / count;
    (error, deviation)
}

/// The ground truth of `scans` carried into a map by `motion`.
fn truths_of(scans: &[PreparedScan], motion: PlanarMotion) -> Vec<PlanarPose> {
    scans
        .iter()
        .map(|scan| ours(motion.apply(scan.truth)))
        .collect()
}

// Through the splice, the drive's localization is scan matching alone: `Localization` holds
// no search to start.
#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn after_the_splice_localization_keeps_its_estimate_as_slam_toolboxs_does() {
    let results = results_directory();
    let all = scenarios();
    let maps = our_maps(&results, &all, |_| RayTimes::AtTheStamp);
    let mut held = Vec::new();
    for scenario in &all {
        let (Task::Localization { map_scenario, .. }, Some(_)) =
            (&scenario.task, drive_of(scenario))
        else {
            continue;
        };
        let prepared = read_prepared(&results, &scenario.name);
        let mut errors = Vec::new();
        let mut deviations = Vec::new();
        for &seed in &scenario.seeds {
            let (map, motion) = map_named(&maps, map_scenario, seed);
            let motion = *motion;
            let reference_motion = reference_map_from_truth(&results, map_scenario, seed);
            let Spliced {
                prepared: spliced,
                odometry,
                splice,
                far,
                distance_m,
            } = spliced(&prepared, &read_odometry(&results, &scenario.name, seed));
            assert!(
                distance_m >= 1.0,
                "{}: the places of the splice lie {distance_m} m apart",
                scenario.name
            );
            let reference = reference_spliced_localization(&results, &scenario.name, seed);
            assert_eq!(
                (reference.splice, reference.far),
                (splice, far),
                "{} seed {seed}: the reference splices from the scans this test does",
                scenario.name
            );
            let mut localization = Localization::from_estimate(
                map,
                localization_parameters_of(scenario),
                ours(motion.apply(spliced.scans[0].truth)),
                ours(odometry[0]),
            )
            .unwrap_or_else(|e| panic!("{}: {e}", scenario.name));
            let answers: Vec<(usize, PlanarPose, f64)> =
                placed_scans(&spliced, &odometry, RayTimes::AtTheStamp)
                    .iter()
                    .enumerate()
                    .filter_map(|(index, scan)| {
                        localization
                            .add(scan)
                            .unwrap_or_else(|e| panic!("{} scan {index}: {e}", scenario.name))
                            .map(|pose| (index, pose.pose, pose.deviations.position_std_m))
                    })
                    .collect();
            let reference_answers: Vec<(usize, PlanarPose, f64)> = reference
                .answers
                .iter()
                .map(|answer| {
                    (
                        answer.scan,
                        ours(answer.pose),
                        answer.deviations.position_std_m,
                    )
                })
                .collect();
            let ours_after = after_the_splice(
                &scenario.name,
                &answers,
                splice,
                &truths_of(&spliced.scans, motion),
            );
            let theirs_after = after_the_splice(
                &scenario.name,
                &reference_answers,
                reference.splice,
                &truths_of(&spliced.scans, reference_motion),
            );
            errors.push((seed, ours_after.0, theirs_after.0));
            deviations.push((seed, ours_after.1, theirs_after.1));
        }
        for (statistic, rows) in [
            ("mean position error after the splice (m)", &errors),
            ("mean position deviation after the splice (m)", &deviations),
        ] {
            held.push((
                scenario.name.clone(),
                statistic,
                compare(Reference::SlamToolbox, &scenario.name, statistic, rows),
            ));
        }
    }
    assert_all_hold(&held, "localizes");
}

#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn localization_is_no_worse_than_slam_toolboxs() {
    let results = results_directory();
    let all = scenarios();
    let maps = our_maps(&results, &all, |_| RayTimes::AtTheStamp);
    let mut held = Vec::new();
    for scenario in &all {
        let (Task::Localization { map_scenario, .. }, Some(_)) =
            (&scenario.task, drive_of(scenario))
        else {
            continue;
        };
        let prepared = read_prepared(&results, &scenario.name);
        let TaskStatistics::Localization(reference) = statistics_of(&scenario.name) else {
            panic!("{} localizes", scenario.name);
        };
        let rows: Vec<(i64, f64, f64)> = scenario
            .seeds
            .iter()
            .zip(&reference.slam_toolbox)
            .map(
                |(
                    &seed,
                    LocalizationSeed {
                        localization_rmse_m,
                        ..
                    },
                )| {
                    let (map, motion) = map_named(&maps, map_scenario, seed);
                    let odometry = read_odometry(&results, &scenario.name, seed);
                    let mut localization = Localization::from_estimate(
                        map,
                        localization_parameters_of(scenario),
                        ours(motion.apply(prepared.scans[0].truth)),
                        ours(odometry[0]),
                    )
                    .unwrap_or_else(|e| panic!("{}: {e}", scenario.name));
                    let poses: Vec<ScanPose> =
                        placed_scans(&prepared, &odometry, RayTimes::AtTheStamp)
                            .iter()
                            .enumerate()
                            .filter_map(|(index, scan)| {
                                localization
                                    .add(scan)
                                    .unwrap_or_else(|e| {
                                        panic!("{} scan {index}: {e}", scenario.name)
                                    })
                                    .map(|localized| ScanPose {
                                        scan: index,
                                        pose: harness(localized.pose),
                                    })
                            })
                            .collect();
                    let ours = localization_errors(&prepared, &poses, *motion)
                        .unwrap_or_else(|| panic!("{} seed {seed} localizes", scenario.name));
                    let theirs = localization_rmse_m.unwrap_or_else(|| {
                        panic!("{} seed {seed}: slam_toolbox localized", scenario.name)
                    });
                    (seed, ours.position_rmse_m, theirs)
                },
            )
            .collect();
        let statistic = "localization RMSE (m)";
        held.push((
            scenario.name.clone(),
            statistic,
            compare(Reference::SlamToolbox, &scenario.name, statistic, &rows),
        ));
    }
    assert_all_hold(&held, "localizes");
}
