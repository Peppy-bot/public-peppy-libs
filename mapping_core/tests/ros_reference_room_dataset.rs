//! Parity with slam_toolbox 2.10.0 and Nav2 1.5.2's AMCL on the room dataset scenarios of
//! the lidar drive, on the scans and the seeded odometries that the reference prepared,
//! which both sides read. slam_toolbox reads each heading of the odometry through tf2's
//! buffer, whose rounding this crate does not reproduce to the last bit, and a match whose
//! candidates lie on the edges of cells can take the neighboring cell from a difference in
//! the last bit, so each comparison is of the statistics of the ten seeds, scored by
//! `ros_reference::metrics` as the committed statistics are: the mean of ours lies at most
//! twice the standard error of the reference's ten values above their mean. On each side, a
//! run of a seed localizes in the map that the same seed made, so the seeds sample the maps
//! too. Each test prints every seed's values and their difference.
//!
//! The tests that compare with a reference on the same scans take every ray at its scan's
//! stamp, as the references take it: the trajectory error of mapping, the error of
//! localization in each seed's map of the mapping scenario, on the drive spliced from
//! two places of the map the error and the position deviation of the localization after the
//! splice, and, from the first scan of each localization scenario, the search's
//! convergence, its time to converge and its final error. The other tests take each ray at
//! the time the drive cast it, as the library runs on a base: the first seed's map, saved
//! and loaded, closes its loop; and a search started at the first scan of each localization
//! scenario, and one started at the splice, localize the base, their final errors within
//! the largest of AMCL's ten runs.
//!
//! The scenarios of the room dataset kind run on a developer's machine alone, through
//! `ros_reference/run.sh --datasets`, which names the results directory of the references
//! in `ROS_REFERENCE_RESULTS`.

use camera_core::laser_scan::{LaserScan, Sweep};
use control_core::planar_pose::PlanarPose;
use mapping_core::{
    Drand48, GlobalLocalization, GlobalLocalizationParameters, Localization, LocalizationLimits,
    LocalizationParameters, MapPose, Mapping, MappingParameters, PlacedScan, RoomMap, SearchStep,
};
use ros_reference::metrics::PlanarPose as HarnessPose;
use ros_reference::metrics::{self, Mean, PlanarMotion};
use ros_reference::results_directory;
use ros_reference::room_dataset::{
    GlobalLocalizationSeed, LidarDrive, LocalizationSeed, MappingSeed, Prepared, PreparedScan,
    ScanPose, Scenario, Source, Statistics, Task, TaskStatistics, localization_errors,
    map_from_truth, read_odometry, read_prepared, reference_map_from_truth,
    reference_mapping_poses, reference_spliced_localization, scenarios, trajectory_errors_of,
};
use std::collections::BTreeMap;
use std::path::Path;

mod common;
use common::properties::{compose, relative};
use common::{
    amcl_parameters, localization_parameters, matching_limits, placed, scratch_directory,
};
use control_core::positive_finite::PositiveFinite;

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
    let mut mapping =
        Mapping::start(slam_toolbox_parameters(scenario)).unwrap_or_else(|e| panic!("{name}: {e}"));
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
    Amcl,
}

impl Reference {
    fn name(self) -> &'static str {
        match self {
            Reference::SlamToolbox => "slam_toolbox",
            Reference::Amcl => "AMCL",
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

/// A limit of a scenario as the search takes it.
fn limits_of(limits: metrics::LocalizationLimits) -> LocalizationLimits {
    let limit = |value: f64| PositiveFinite::parse(value).expect("a positive limit");
    LocalizationLimits {
        position_std_m: limit(limits.position_std_m),
        yaw_std_rad: limit(limits.yaw_std_rad),
    }
}

/// A localization scenario of the drive, whose first scan a search starts at, with what the
/// search on it reads: our map of each seed of its map scenario, by the seed, its rays
/// standing as the maps' do, the scenario's search parameters and AMCL's committed
/// statistics, every one of whose ten runs localized by the search limits.
struct Searched<'a> {
    scenario: &'a Scenario,
    maps: BTreeMap<i64, (RoomMap, PlanarMotion)>,
    prepared: Prepared,
    rays: RayTimes,
    parameters: GlobalLocalizationParameters,
    localized_within: metrics::LocalizedWithin,
    amcl: Vec<GlobalLocalizationSeed>,
}

/// The localization scenarios of the drive, each searched with its rays standing as
/// `rays_of` gives for it.
fn searched_starts<'a>(
    results: &Path,
    all: &'a [Scenario],
    rays_of: fn(&Scenario) -> RayTimes,
) -> Vec<Searched<'a>> {
    let maps = our_maps(results, all, rays_of);
    all.iter()
        .filter_map(|scenario| {
            let (
                Task::Localization {
                    map_scenario,
                    amcl,
                    localized_within,
                    search_limits,
                },
                Some(_),
            ) = (&scenario.task, drive_of(scenario))
            else {
                return None;
            };
            let maps_of_seeds = scenario
                .seeds
                .iter()
                .map(|&seed| (seed, map_named(&maps, map_scenario, seed).clone()))
                .collect();
            let TaskStatistics::Localization(statistics) = statistics_of(&scenario.name) else {
                panic!("{} localizes", scenario.name);
            };
            assert!(
                statistics
                    .amcl
                    .iter()
                    .all(|seed| seed.localized_by_limits_after_s.is_some()),
                "{}: AMCL localizes the base from its first scan in each of its runs",
                scenario.name
            );
            Some(Searched {
                scenario,
                maps: maps_of_seeds,
                prepared: read_prepared(results, &scenario.name),
                rays: rays_of(scenario),
                parameters: GlobalLocalizationParameters {
                    amcl: amcl_parameters(&scenario.name, amcl),
                    search_limits: limits_of(*search_limits),
                    localization: localization_parameters_of(scenario),
                },
                localized_within: *localized_within,
                amcl: statistics.amcl,
            })
        })
        .collect()
}

impl Searched<'_> {
    /// Our map that a run of `seed` searches, and the motion that carries the ground truth
    /// into it.
    fn map_of(&self, seed: i64) -> &(RoomMap, PlanarMotion) {
        self.maps
            .get(&seed)
            .unwrap_or_else(|| panic!("{}: no map of seed {seed}", self.scenario.name))
    }
}

/// A search started at the scan `start`, then the localization it ends in: the answer to
/// each scan, by its index, and the index of the scan at which the search ended.
struct SearchRun {
    answers: Vec<(usize, MapPose)>,
    found_at: Option<usize>,
}

fn search_run(
    searched: &Searched,
    prepared: &Prepared,
    odometry: &[HarnessPose],
    start: usize,
    seed: i64,
) -> SearchRun {
    let name = &searched.scenario.name;
    let (map, _) = searched.map_of(seed);
    let draw_seed = u32::try_from(seed).expect("a seed of the drive fits the search's seed");
    // The harness seeds AMCL's stream right before it asks for each search.
    let mut stream = Drand48::seeded(draw_seed);
    let mut search = Some(
        GlobalLocalization::start(map, searched.parameters, &mut stream)
            .unwrap_or_else(|e| panic!("{name}: {e}")),
    );
    let mut localization: Option<Localization> = None;
    let mut found_at = None;
    let mut answers = Vec::new();
    let scans = placed_scans(prepared, odometry, searched.rays);
    for (index, scan) in scans.iter().enumerate().skip(start) {
        if let Some(kept) = localization.as_mut() {
            if let Some(pose) = kept
                .add(scan)
                .unwrap_or_else(|e| panic!("{name} scan {index}: {e}"))
            {
                answers.push((index, pose));
            }
            continue;
        }
        match search.take().expect("a search").add(scan, &mut stream) {
            SearchStep::Searching(going_on, pose) => {
                search = Some(going_on);
                answers.extend(pose.map(|pose| (index, pose)));
            }
            SearchStep::Found(kept, pose) => {
                localization = Some(kept);
                found_at = Some(index);
                answers.push((index, pose));
            }
        }
    }
    SearchRun { answers, found_at }
}

impl SearchRun {
    /// The position (m) and heading (rad) errors of the last answer against `truths`.
    fn final_errors(&self, truths: &[PlanarPose]) -> (f64, f64) {
        let (index, pose) = self.answers.last().expect("an answer");
        metrics::pose_error(harness(pose.pose), harness(truths[*index]))
    }

    /// The index of the scan of the first answer from which every answer is localized and
    /// lies within `within` of `truths`, by `metrics`' rule of a run localized for good; none
    /// when the last answer does not.
    fn localized_from(
        &self,
        truths: &[PlanarPose],
        within: metrics::LocalizedWithin,
    ) -> Option<usize> {
        let localized: Vec<bool> = self
            .answers
            .iter()
            .map(|(_, pose)| pose.localized)
            .collect();
        let near: Vec<bool> = self
            .answers
            .iter()
            .map(|(index, pose)| within.holds(harness(pose.pose), harness(truths[*index])))
            .collect();
        metrics::localized_for_good_from(&localized, &near).map(|first| self.answers[first].0)
    }
}

/// The largest final position and heading errors of AMCL's ten runs.
fn amcl_largest_final_errors(searched: &Searched) -> (f64, f64) {
    searched
        .amcl
        .iter()
        .fold((0.0, 0.0), |(position, yaw), seed| {
            (
                f64::max(position, seed.final_position_error_m),
                f64::max(yaw, seed.final_yaw_error_rad),
            )
        })
}

/// Whether the search `run` localizes the base for good against `truths`, its final errors
/// within AMCL's largest from the same scan, printing what it found.
fn localizes_within_amcls_errors(
    searched: &Searched,
    run: &SearchRun,
    truths: &[PlanarPose],
    from: &str,
) -> bool {
    let name = &searched.scenario.name;
    let localized = run.localized_from(truths, searched.localized_within);
    let (position, yaw) = run.final_errors(truths);
    let (largest_position, largest_yaw) = amcl_largest_final_errors(searched);
    println!(
        "{name}: the search from {from} ended at scan {:?}, localized from scan \
         {localized:?}, final errors {position:.3} m and {yaw:.3} rad against AMCL's largest \
         {largest_position:.3} m and {largest_yaw:.3} rad",
        run.found_at
    );
    localized.is_some() && position <= largest_position && yaw <= largest_yaw
}

// On the map of the drive, a search started at the first scan of each localization
// scenario, from which AMCL localizes the base in each of its ten runs, localizes the base
// for good by the scenario's limits before the drive ends, its final errors within the
// largest of AMCL's ten runs.
#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn from_each_start_a_search_localizes_the_base_within_amcls_largest_errors() {
    let results = results_directory();
    let all = scenarios();
    let searched = searched_starts(&results, &all, RayTimes::as_cast);
    assert!(
        !searched.is_empty(),
        "a room dataset scenario localizes on the drive"
    );
    let missed: Vec<&String> = searched
        .iter()
        .filter(|start| {
            let seed = start.scenario.seeds[0];
            let odometry = read_odometry(&results, &start.scenario.name, seed);
            let run = search_run(start, &start.prepared, &odometry, 0, seed);
            let truths = truths_of(&start.prepared.scans, start.map_of(seed).1);
            !localizes_within_amcls_errors(start, &run, &truths, "its first scan")
        })
        .map(|start| &start.scenario.name)
        .collect();
    assert!(
        missed.is_empty(),
        "the search does not localize the base on {missed:?}"
    );
}

// On the drive's whole second lap, the longest localization scenario, spliced from two
// places of its map, a search started at the splice localizes the base at the new place
// before the drive ends, its final errors within the largest of AMCL's ten runs.
#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn a_search_started_at_the_splice_localizes_the_base_at_the_new_place() {
    let results = results_directory();
    let all = scenarios();
    let searched = searched_starts(&results, &all, RayTimes::as_cast);
    assert!(
        !searched.is_empty(),
        "a room dataset scenario localizes on the drive"
    );
    let lap = searched
        .iter()
        .max_by_key(|start| start.prepared.scans.len())
        .expect("a localization scenario of the drive");
    let seed = lap.scenario.seeds[0];
    let Spliced {
        prepared,
        odometry,
        splice,
        ..
    } = spliced(
        &lap.prepared,
        &read_odometry(&results, &lap.scenario.name, seed),
    );
    let run = search_run(lap, &prepared, &odometry, splice, seed);
    let truths = truths_of(&prepared.scans, lap.map_of(seed).1);
    assert!(
        localizes_within_amcls_errors(lap, &run, &truths, &format!("the splice, scan {splice}")),
        "{}: the search from the splice does not localize the base at the new place",
        lap.scenario.name
    );
}

// From the first scan of each localization scenario of the drive, over the ten seeds, a run
// counting as converged on either side when it localizes the base for good by the scenario's
// limits: every run of the search converges, as every run of AMCL does, and its mean time to
// converge and its mean final error are no higher than AMCL's mean plus twice the standard
// error of its ten values; every seed's difference is printed.
#[test]
#[ignore = "reads the local datasets: run.sh --datasets runs it"]
fn from_each_start_the_search_converges_as_amcl_does() {
    let results = results_directory();
    let all = scenarios();
    let searched = searched_starts(&results, &all, |_| RayTimes::AtTheStamp);
    let mut held = Vec::new();
    for start in &searched {
        let name = &start.scenario.name;
        let scans = &start.prepared.scans;
        let mut times = Vec::new();
        let mut errors = Vec::new();
        let mut failed = Vec::new();
        for (&seed, amcl) in start.scenario.seeds.iter().zip(&start.amcl) {
            assert_eq!(
                amcl.seed, seed,
                "{name}: AMCL's statistics pair with the seeds"
            );
            let odometry = read_odometry(&results, name, seed);
            let run = search_run(start, &start.prepared, &odometry, 0, seed);
            let truths = truths_of(scans, start.map_of(seed).1);
            let Some(localized) = run.localized_from(&truths, start.localized_within) else {
                failed.push(seed);
                continue;
            };
            let time_s = (scans[localized].stamp_ns - scans[0].stamp_ns) as f64 * 1e-9;
            let amcl_time_s = amcl
                .localized_by_limits_after_s
                .expect("AMCL localized in each run");
            times.push((seed, time_s, amcl_time_s));
            errors.push((
                seed,
                run.final_errors(&truths).0,
                amcl.final_position_error_m,
            ));
        }
        held.push((name.clone(), "every run converges", failed.is_empty()));
        if !failed.is_empty() {
            println!("{name}: the search does not converge with the seeds {failed:?}");
            continue;
        }
        for (statistic, rows) in [
            ("time to converge (s)", &times),
            ("final position error (m)", &errors),
        ] {
            held.push((
                name.clone(),
                statistic,
                compare(Reference::Amcl, name, statistic, rows),
            ));
        }
    }
    assert_all_hold(&held, "localizes");
}
