//! The report of ROBOTCTRL-60, `reports/ROBOTCTRL-60.md`, against the room dataset
//! scenarios and their statistics: the committed ones, or, as for every parity test, those
//! of the directory that `ROS_REFERENCE_OUTPUTS` names, where `run.sh` puts the arm64
//! image's. Its sections "What the numbers say" and "The numbers" are the ones that
//! [`summary_section`] and [`numbers_section`] render from them, so each number there is
//! the statistic of its sequence and of its rule. Its other sections state no number: they
//! hold digits only in code spans and in names such as ROBOTCTRL-62, 2-D or a version, and
//! no count in letters. A run of `run.sh --datasets` that changes a statistic fails this
//! test until the report holds the sections that the new statistics render.

use ros_reference::metrics::{self, LocalizationLimits, LocalizedWithin, Mean};
use ros_reference::room_dataset::{
    Camera, GlobalLocalizationSeed, LocalizationSeed, LocalizationStatistics, MappingSeed,
    Scenario, Statistics, Task, TaskStatistics, scenarios,
};
use std::fmt::Write as _;

const REPORT: &str = include_str!("../reports/ROBOTCTRL-60.md");
const SUMMARY_HEADING: &str = "## What the numbers say\n";
const NUMBERS_HEADING: &str = "## The numbers\n";

fn meters(value: f64) -> String {
    format!("{value:.3}")
}

fn degrees(value: f64) -> String {
    format!("{:.1}", value.to_degrees())
}

fn seconds(value: f64) -> String {
    format!("{value:.1}")
}

/// A mean count of scans.
fn scans(value: f64) -> String {
    format!("{value:.1}")
}

/// `select` of each seed of `seeds`.
fn values<Seed>(seeds: &[Seed], select: fn(&Seed) -> f64) -> Vec<f64> {
    seeds.iter().map(select).collect()
}

/// `select` of each seed of `seeds` that has it.
fn present_values<Seed>(seeds: &[Seed], select: fn(&Seed) -> Option<f64>) -> Vec<f64> {
    seeds.iter().filter_map(select).collect()
}

/// The least and the largest of `values`, which hold a value at least.
fn least_and_largest(values: &[f64]) -> (f64, f64) {
    let extreme = |pick: fn(f64, f64) -> f64| {
        values
            .iter()
            .copied()
            .reduce(pick)
            .expect("the extremes of no value")
    };
    (extreme(f64::min), extreme(f64::max))
}

/// "the N of M seeds that localized a scan" when some seeds of `seeds` localized no scan in
/// slam_toolbox's localization and so have no errors, and `None` when every seed has them.
fn localizing_subset(seeds: &[LocalizationSeed]) -> Option<String> {
    let localizing = present_values(seeds, |s| s.localization_rmse_m).len();
    (localizing < seeds.len()).then(|| {
        format!(
            "the {localizing} of {} seeds that localized a scan",
            seeds.len()
        )
    })
}

fn mean_of(values: &[f64], format: fn(f64) -> String) -> String {
    metrics::mean(values).map_or(
        "none".into(),
        |Mean {
             mean,
             standard_error,
         }| { format!("{} ± {}", format(mean), format(standard_error)) },
    )
}

fn mapping_table(out: &mut String, seeds: &[MappingSeed]) {
    out.push_str(
        "| Seed | Scans in the map | Trajectory RMSE (m) | Largest error (m) | Heading RMSE \
         (deg) |\n",
    );
    out.push_str("|---|---|---|---|---|\n");
    for seed in seeds {
        writeln!(
            out,
            "| {} | {} | {} | {} | {} |",
            seed.seed,
            seed.processed_scans,
            meters(seed.trajectory_rmse_m),
            meters(seed.trajectory_max_m),
            degrees(seed.trajectory_yaw_rmse_rad)
        )
        .unwrap();
    }
    writeln!(
        out,
        "| Mean ± standard error | {} | {} | {} | {} |",
        mean_of(&values(seeds, |s| s.processed_scans as f64), scans),
        mean_of(&values(seeds, |s| s.trajectory_rmse_m), meters),
        mean_of(&values(seeds, |s| s.trajectory_max_m), meters),
        mean_of(&values(seeds, |s| s.trajectory_yaw_rmse_rad), degrees)
    )
    .unwrap();
}

fn localization_table(out: &mut String, seeds: &[LocalizationSeed]) {
    out.push_str(
        "| Seed | Scans localized | RMSE (m) | Largest error (m) | Heading RMSE (deg) |\n",
    );
    out.push_str("|---|---|---|---|---|\n");
    let optional =
        |value: Option<f64>, format: fn(f64) -> String| value.map_or("none".into(), format);
    for seed in seeds {
        writeln!(
            out,
            "| {} | {} | {} | {} | {} |",
            seed.seed,
            seed.localized_scans,
            optional(seed.localization_rmse_m, meters),
            optional(seed.localization_max_m, meters),
            optional(seed.localization_yaw_rmse_rad, degrees)
        )
        .unwrap();
    }
    let errors_over =
        localizing_subset(seeds).map_or(String::new(), |some| format!(", the errors over {some}"));
    writeln!(
        out,
        "| Mean ± standard error{errors_over} | {} | {} | {} | {} |",
        mean_of(&values(seeds, |s| s.localized_scans as f64), scans),
        mean_of(&present_values(seeds, |s| s.localization_rmse_m), meters),
        mean_of(&present_values(seeds, |s| s.localization_max_m), meters),
        mean_of(
            &present_values(seeds, |s| s.localization_yaw_rmse_rad),
            degrees
        )
    )
    .unwrap();
}

/// The table of AMCL's global localization over the seeds `seeds`.
fn global_localization_table(out: &mut String, seeds: &[GlobalLocalizationSeed]) {
    out.push_str(
        "| Seed | amcl_pose messages | Convergence test held | Within the search limits | \
         Least position deviation (m) | Least heading deviation (deg) | Localized by the test \
         after (s) | Localized by the limits after (s) | Final position error (m) | Final \
         heading error (deg) |\n",
    );
    out.push_str("|---|---|---|---|---|---|---|---|---|---|\n");
    let after = |time: Option<f64>| time.map_or("never".into(), seconds);
    for seed in seeds {
        writeln!(
            out,
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
            seed.seed,
            seed.estimates,
            seed.converged_estimates,
            seed.within_limits_estimates,
            meters(seed.least_position_std_m),
            degrees(seed.least_yaw_std_rad),
            after(seed.localized_after_s),
            after(seed.localized_by_limits_after_s),
            meters(seed.final_position_error_m),
            degrees(seed.final_yaw_error_rad)
        )
        .unwrap();
    }
}

/// The largest final position and heading errors of `seeds`, which hold a seed at least.
fn largest_final_error(seeds: &[&GlobalLocalizationSeed]) -> String {
    let largest =
        |select: fn(&&GlobalLocalizationSeed) -> f64| least_and_largest(&values(seeds, select)).1;
    format!(
        "{} m and {} deg",
        meters(largest(|s| s.final_position_error_m)),
        degrees(largest(|s| s.final_yaw_error_rad))
    )
}

/// The smallest and the largest of `select` over `seeds`, as `format` writes them.
fn span(
    seeds: &[GlobalLocalizationSeed],
    select: fn(&GlobalLocalizationSeed) -> f64,
    format: fn(f64) -> String,
) -> String {
    let (least, largest) = least_and_largest(&values(seeds, select));
    format!("{} to {}", format(least), format(largest))
}

/// The horizontal field of view (deg) of a pinhole camera of the image width
/// `2 (cx + 0.5)`.
fn field_of_view_degrees(camera: &Camera) -> f64 {
    2.0 * (camera.cx + 0.5).atan2(camera.fx).to_degrees()
}

/// What every room dataset scenario shares, which the report states once.
fn setup_paragraph(scenarios: &[Scenario]) -> String {
    let first = scenarios
        .first()
        .expect("scenarios/ holds a room dataset scenario");
    for scenario in scenarios {
        assert!(
            scenario.seeds == first.seeds
                && scenario.scan_period_s == first.scan_period_s
                && scenario.max_ground_truth_gap_s == first.max_ground_truth_gap_s
                && scenario.camera == first.camera
                && scenario.laser_scan == first.laser_scan
                && scenario.odometry_noise == first.odometry_noise,
            "{} keeps, projects and drives as {} does, as the report states once",
            scenario.name,
            first.name
        );
    }
    let number = |name: &str| {
        first.laser_scan[name]
            .as_f64()
            .unwrap_or_else(|| panic!("the scan parameter {name} is a number"))
    };
    let camera = &first.camera;
    let field_of_view = field_of_view_degrees(camera);
    let noise = &first.odometry_noise;
    format!(
        "Every sequence runs {} seeds. It keeps the first depth frame at or after each {} s, \
         drops a frame whose ground truth has a gap over {} s, and projects each depth pixel \
         with fx {}, fy {}, cx {}, cy {} and {} counts a meter, a horizontal field of view \
         of {field_of_view:.1} deg, where a 2-D lidar sees 360 deg. The scan holds the \
         points {} m to {} m above the floor, {} m to {} m away, in {} rad steps from {} \
         rad. The odometry noise of each motion between two kept frames is the noise of AMCL's \
         differential motion model with alpha1 {}, alpha2 {}, alpha3 {} and alpha4 {}.\n",
        first.seeds.len(),
        first.scan_period_s,
        first.max_ground_truth_gap_s,
        camera.fx,
        camera.fy,
        camera.cx,
        camera.cy,
        camera.depth_counts_per_meter,
        number("min_height"),
        number("max_height"),
        number("range_min"),
        number("range_max"),
        number("angle_increment"),
        number("angle_min"),
        noise.alpha1,
        noise.alpha2,
        noise.alpha3,
        noise.alpha4
    )
}

/// The section "The numbers" of the report, rendered from the scenarios and their
/// statistics.
fn numbers_section(scenarios: &[Scenario], all: &[Statistics]) -> String {
    let mut out = format!("{NUMBERS_HEADING}\n{}", setup_paragraph(scenarios));
    for (scenario, statistics) in paired(scenarios, all) {
        let mount = &statistics.mount;
        writeln!(
            out,
            "\n### {}\n\n{} depth frames, {} scans kept, {} dropped in gaps of the ground \
             truth. Fitted mount: camera {} m above the floor, pitched {} deg down, rolled \
             {} deg.\n",
            statistics.sequence,
            statistics.depth_frames,
            statistics.scans,
            statistics.dropped_scans.len(),
            meters(mount.height),
            degrees(mount.pitch),
            degrees(mount.roll)
        )
        .unwrap();
        match &statistics.task {
            TaskStatistics::Mapping(seeds) => {
                out.push_str("slam_toolbox mapping:\n\n");
                mapping_table(&mut out, seeds);
            }
            TaskStatistics::Localization(localization) => {
                let Task::Localization { map_scenario, .. } = &scenario.task else {
                    panic!("{} holds the statistics of a localization", scenario.name);
                };
                writeln!(
                    out,
                    "slam_toolbox localization in the map of {map_scenario}'s first seed:\n"
                )
                .unwrap();
                localization_table(&mut out, &localization.slam_toolbox);
                out.push_str("\nAMCL global localization in that map:\n\n");
                global_localization_table(&mut out, &localization.amcl);
            }
        }
    }
    out
}

/// Each scenario with its statistics.
fn paired<'a>(
    scenarios: &'a [Scenario],
    all: &'a [Statistics],
) -> impl Iterator<Item = (&'a Scenario, &'a Statistics)> {
    assert_eq!(
        scenarios.len(),
        all.len(),
        "each scenario has its statistics"
    );
    scenarios.iter().zip(all).inspect(|(scenario, statistics)| {
        assert_eq!(
            scenario.name, statistics.name,
            "the statistics pair with their scenarios"
        );
    })
}

/// A localization scenario's statistics, the bound within which an estimate of AMCL counts
/// as localized, the search limits, and the scenario of the map it localizes in.
struct Localization<'a> {
    statistics: &'a Statistics,
    results: &'a LocalizationStatistics,
    within: LocalizedWithin,
    limits: LocalizationLimits,
    map_scenario: &'a str,
}

/// The localization scenarios, in the order of their names.
fn localizations<'a>(scenarios: &'a [Scenario], all: &'a [Statistics]) -> Vec<Localization<'a>> {
    paired(scenarios, all)
        .filter_map(
            |(scenario, statistics)| match (&scenario.task, &statistics.task) {
                (
                    Task::Localization {
                        map_scenario,
                        localized_within,
                        search_limits,
                        ..
                    },
                    TaskStatistics::Localization(results),
                ) => Some(Localization {
                    statistics,
                    results,
                    within: *localized_within,
                    limits: *search_limits,
                    map_scenario,
                }),
                _ => None,
            },
        )
        .collect()
}

/// The bound within which an estimate of AMCL counts as localized, which every localization
/// scenario shares, as the error bound states one bound for the seeds of every sequence.
fn shared_localized_within(localizations: &[Localization]) -> LocalizedWithin {
    let first = localizations
        .first()
        .expect("scenarios/ holds a localization scenario");
    for localization in localizations {
        assert_eq!(
            localization.within, first.within,
            "{} counts a seed localized within the bound of {}, as the error bound states one \
             bound for the seeds of every sequence",
            localization.statistics.name, first.statistics.name
        );
    }
    first.within
}

/// The bound `within`, as the report states it.
fn bound_text(within: LocalizedWithin) -> String {
    format!(
        "{} m and {} deg",
        within.position_m,
        degrees(within.yaw_rad)
    )
}

/// The sentence of slam_toolbox's localization of `sequence` in the map of `map_scenario`'s
/// first seed over `seeds`: the mean errors over the seeds that localized a scan, or that
/// none did.
fn slam_toolbox_localization_sentence(
    sequence: &str,
    map_scenario: &str,
    seeds: &[LocalizationSeed],
) -> String {
    let localizes = format!(
        "On {sequence}, slam_toolbox localizes in the map of {map_scenario}'s first seed, from \
         the ground truth carried into that map,"
    );
    let errors = present_values(seeds, |s| s.localization_rmse_m);
    if errors.is_empty() {
        return format!("{localizes} no scan in any of its {} seeds.", seeds.len());
    }
    format!(
        "{localizes} with an error of {} m and {} deg RMS of heading, mean ± standard error over \
         {}.",
        mean_of(&errors, meters),
        mean_of(
            &present_values(seeds, |s| s.localization_yaw_rmse_rad),
            degrees
        ),
        localizing_subset(seeds).unwrap_or_else(|| format!("its {} seeds", seeds.len()))
    )
}

/// The error bound by a rule over `every` seed: the largest final error of the seeds that
/// `localized_after_s` gives a time, each within `within`, or none.
fn error_bound(
    rule: &str,
    every: &[&GlobalLocalizationSeed],
    within: LocalizedWithin,
    localized_after_s: fn(&GlobalLocalizationSeed) -> Option<f64>,
) -> String {
    let localized = localized_seeds(every, localized_after_s);
    if localized.is_empty() {
        return format!("{rule}, none: it localized 0 of {} seeds", every.len());
    }
    format!(
        "{rule}, {} over the {} of {} seeds that it localized, each within the {} of the truth \
         that count a seed localized",
        largest_final_error(&localized),
        localized.len(),
        every.len(),
        bound_text(within)
    )
}

/// The seeds of `seeds` that a rule localized, which `localized_after_s` gives a time.
fn localized_seeds<'a>(
    seeds: &[&'a GlobalLocalizationSeed],
    localized_after_s: fn(&GlobalLocalizationSeed) -> Option<f64>,
) -> Vec<&'a GlobalLocalizationSeed> {
    seeds
        .iter()
        .copied()
        .filter(|s| localized_after_s(s).is_some())
        .collect()
}

/// The seeds of `seeds` that a rule localized, by `localized_after_s`: "<rule> in N of M
/// seeds", and when there are any, their mean time to localize with its standard error and
/// their largest final error.
fn localized_by(
    rule: &str,
    seeds: &[&GlobalLocalizationSeed],
    localized_after_s: fn(&GlobalLocalizationSeed) -> Option<f64>,
) -> String {
    let localized = localized_seeds(seeds, localized_after_s);
    let count = format!("{rule} in {} of {} seeds", localized.len(), seeds.len());
    if localized.is_empty() {
        return count;
    }
    let times: Vec<f64> = localized
        .iter()
        .filter_map(|s| localized_after_s(s))
        .collect();
    format!(
        "{count}, after {} s, with a largest final error of {}",
        mean_of(&times, seconds),
        largest_final_error(&localized)
    )
}

/// The section "What the numbers say" of the report, rendered from the scenarios and the
/// statistics: what each sequence gives under each measure.
fn summary_section(scenarios: &[Scenario], all: &[Statistics]) -> String {
    let camera = &scenarios
        .first()
        .expect("scenarios/ holds a room dataset scenario")
        .camera;
    let mut out = format!(
        "{SUMMARY_HEADING}\nThe depth camera's scans hold the bearings within {:.1} deg \
         ahead of the base, where a 2-D lidar sees 360 deg.\n\n",
        field_of_view_degrees(camera)
    );
    for (_, statistics) in paired(scenarios, all) {
        let TaskStatistics::Mapping(seeds) = &statistics.task else {
            continue;
        };
        writeln!(
            out,
            "On {}, slam_toolbox maps with a trajectory error of {} m and a heading error of \
             {} deg RMS, mean ± standard error over its {} seeds.\n",
            statistics.sequence,
            mean_of(&values(seeds, |s| s.trajectory_rmse_m), meters),
            mean_of(&values(seeds, |s| s.trajectory_yaw_rmse_rad), degrees),
            seeds.len()
        )
        .unwrap();
    }
    let localizations = localizations(scenarios, all);
    for Localization {
        statistics,
        results,
        map_scenario,
        ..
    } in &localizations
    {
        writeln!(
            out,
            "{}\n",
            slam_toolbox_localization_sentence(
                &statistics.sequence,
                map_scenario,
                &results.slam_toolbox
            )
        )
        .unwrap();
    }
    for Localization {
        statistics,
        results,
        within,
        limits,
        ..
    } in &localizations
    {
        let seeds = &results.amcl;
        let every: Vec<&GlobalLocalizationSeed> = seeds.iter().collect();
        let total =
            |select: fn(&GlobalLocalizationSeed) -> usize| seeds.iter().map(select).sum::<usize>();
        let messages = total(|s| s.estimates);
        writeln!(
            out,
            "On {}, AMCL localizes, within {} of the truth from some amcl_pose to the end, {}, \
             and {}. Its convergence test holds at {} of {messages} amcl_pose \
             messages, and the deviations of its whole filter lie within the search limits, {} \
             m and {} deg, at {} of {messages}. The least position deviation of a seed is {} \
             m, and its least heading deviation {} deg. The last amcl_pose of a seed lies at \
             most {} from the truth.\n",
            statistics.sequence,
            bound_text(*within),
            localized_by("by its convergence test", &every, |s| s.localized_after_s),
            localized_by("by the search limits", &every, |s| s
                .localized_by_limits_after_s),
            total(|s| s.converged_estimates),
            limits.position_std_m,
            degrees(limits.yaw_std_rad),
            total(|s| s.within_limits_estimates),
            span(seeds, |s| s.least_position_std_m, meters),
            span(seeds, |s| s.least_yaw_std_rad, degrees),
            largest_final_error(&every)
        )
        .unwrap();
    }
    let every: Vec<&GlobalLocalizationSeed> = localizations
        .iter()
        .flat_map(|localization| &localization.results.amcl)
        .collect();
    let within = shared_localized_within(&localizations);
    writeln!(
        out,
        "The error bound for ROBOTCTRL-62 and ROBOTCTRL-65, the largest final error of the \
         seeds that AMCL localized on the localization sequences: {}; {}.\n",
        error_bound("by its convergence test", &every, within, |s| s
            .localized_after_s),
        error_bound("by the search limits", &every, within, |s| s
            .localized_by_limits_after_s)
    )
    .unwrap();
    out
}

/// The section of `report` under `heading`, up to the next section's heading.
fn section<'a>(report: &'a str, heading: &str) -> &'a str {
    let start = report
        .find(heading)
        .unwrap_or_else(|| panic!("the report holds a section {heading:?}"));
    let end = report[start + heading.len()..]
        .find("\n## ")
        .map_or(report.len(), |end| start + heading.len() + end + 1);
    &report[start..end]
}

/// `text` without its code blocks and code spans, which quote files and code.
fn outside_code(text: &str) -> String {
    let outside_blocks = text.split("```").step_by(2).collect::<Vec<_>>().join(" ");
    outside_blocks
        .split('`')
        .step_by(2)
        .collect::<Vec<_>>()
        .join(" ")
}

/// The numbers that `text` states outside code: each token, stripped of the punctuation
/// around it, that holds a digit and either no letter or a digit as its first letter or
/// digit, such as 0.5, 8m, 3rd, **8m** or ±0.5m, and is neither a version such as 1.5.2 nor
/// a dimension such as 2-D. A token whose first letter or digit is a letter is a name, such
/// as ROBOTCTRL-62 or amd64.
fn bare_numbers(text: &str) -> Vec<String> {
    let first_alphanumeric = |token: &str| {
        token
            .trim_start_matches(|c: char| !c.is_ascii_alphanumeric())
            .to_owned()
    };
    let is_dimension = |token: &str| {
        let token = first_alphanumeric(token);
        token.starts_with(|c: char| c.is_ascii_digit()) && token[1..].starts_with("-D")
    };
    outside_code(text)
        .split_whitespace()
        .map(|token| token.trim_matches(|c: char| "()[]{},;:!?\"'.".contains(c)))
        .filter(|token| {
            token.chars().any(|c| c.is_ascii_digit())
                && (!token.chars().any(|c| c.is_ascii_alphabetic())
                    || first_alphanumeric(token).starts_with(|c: char| c.is_ascii_digit()))
                && token.matches('.').count() < 2
                && !is_dimension(token)
        })
        .map(str::to_owned)
        .collect()
}

/// The words that state a count or a share in letters, which the report's prose holds none
/// of: its counts are those that its rendered sections give.
const NUMBER_WORDS: &str = "none zero one two three four five six seven eight nine ten \
     eleven twelve thirteen fourteen fifteen sixteen seventeen eighteen nineteen twenty \
     thirty forty fifty hundred thousand million once twice thrice both single half halves \
     third thirds quarter quarters fifth fifths sixth sixths tenth tenths dozen";

/// The words of `text` outside code that state a count or a share in letters.
fn number_words(text: &str) -> Vec<String> {
    outside_code(text)
        .split(|c: char| !c.is_ascii_alphabetic())
        .map(str::to_ascii_lowercase)
        .filter(|word| NUMBER_WORDS.split_whitespace().any(|number| number == word))
        .collect()
}

#[test]
fn the_report_holds_the_sections_that_the_statistics_render() {
    let (scenarios, all) = (scenarios(), Statistics::load_all());
    for (heading, rendered) in [
        (SUMMARY_HEADING, summary_section(&scenarios, &all)),
        (NUMBERS_HEADING, numbers_section(&scenarios, &all)),
    ] {
        assert_eq!(
            section(REPORT, heading),
            rendered,
            "the section {heading:?} of reports/ROBOTCTRL-60.md differs from what the \
             scenarios and their statistics render, which is on the right"
        );
    }
}

#[test]
fn the_other_sections_of_the_report_state_no_number() {
    let prose = [SUMMARY_HEADING, NUMBERS_HEADING]
        .into_iter()
        .fold(REPORT.to_owned(), |text, heading| {
            text.replace(section(REPORT, heading), "")
        });
    assert_eq!(
        bare_numbers(&prose),
        Vec::<String>::new(),
        "the prose of reports/ROBOTCTRL-60.md states numbers outside its rendered sections"
    );
    assert_eq!(
        number_words(&prose),
        Vec::<String>::new(),
        "the prose of reports/ROBOTCTRL-60.md states counts in letters"
    );
}

#[test]
fn bare_numbers_are_the_tokens_of_digits_outside_code_and_names() {
    assert_eq!(
        bare_numbers(
            "over 10 seeds (0.5 m), Nav2 1.5.2's AMCL, ROBOTCTRL-62, `pf.c:72`, 2-D and 3-D's \
             amd64 on 2026-11-13, 8m away, the 3rd seed, a **8m** turn, ±0.5m.\n```sh\nrun.sh \
             4\n```\n"
        ),
        ["10", "0.5", "2026-11-13", "8m", "3rd", "**8m**", "±0.5m"]
    );
}

#[test]
fn number_words_are_found_whatever_their_case() {
    assert_eq!(
        number_words("None of the twenty runs, Four seeds, one map, Both runs, `one`, often"),
        ["none", "twenty", "four", "one", "both"]
    );
}

/// A seed of slam_toolbox's localization whose position error is `rmse_m` (m) and heading
/// error a tenth of it (rad), or that localized no scan.
fn localization_seed(seed: i64, rmse_m: Option<f64>) -> LocalizationSeed {
    LocalizationSeed {
        seed,
        localized_scans: if rmse_m.is_some() { 5 } else { 0 },
        localization_rmse_m: rmse_m,
        localization_max_m: rmse_m.map(|rmse| 2.0 * rmse),
        localization_yaw_rmse_rad: rmse_m.map(|rmse| rmse / 10.0),
    }
}

/// A seed of AMCL's global localization that localized by its convergence test after
/// `localized_after_s`, or did not, and ended at the final errors given.
fn global_seed(
    seed: i64,
    localized_after_s: Option<f64>,
    final_position_error_m: f64,
    final_yaw_error_rad: f64,
) -> GlobalLocalizationSeed {
    GlobalLocalizationSeed {
        seed,
        estimates: 10,
        converged_estimates: 0,
        within_limits_estimates: 0,
        least_position_std_m: 0.4,
        least_yaw_std_rad: 0.3,
        localized_after_s,
        localized_by_limits_after_s: None,
        final_position_error_m,
        final_yaw_error_rad,
    }
}

#[test]
fn the_localization_sentence_counts_the_seeds_that_localized_a_scan() {
    let every = [
        localization_seed(0, Some(1.0)),
        localization_seed(1, Some(2.0)),
    ];
    let some = [
        localization_seed(0, Some(1.0)),
        localization_seed(1, None),
        localization_seed(2, Some(2.0)),
    ];
    let none = [localization_seed(0, None), localization_seed(1, None)];
    let opening = "On s, slam_toolbox localizes in the map of m's first seed, from the ground \
                   truth carried into that map,";
    assert_eq!(
        slam_toolbox_localization_sentence("s", "m", &every),
        format!(
            "{opening} with an error of 1.500 ± 0.500 m and 8.6 ± 2.9 deg RMS of heading, mean \
             ± standard error over its 2 seeds."
        )
    );
    assert_eq!(
        slam_toolbox_localization_sentence("s", "m", &some),
        format!(
            "{opening} with an error of 1.500 ± 0.500 m and 8.6 ± 2.9 deg RMS of heading, mean \
             ± standard error over the 2 of 3 seeds that localized a scan."
        )
    );
    assert_eq!(
        slam_toolbox_localization_sentence("s", "m", &none),
        format!("{opening} no scan in any of its 2 seeds.")
    );
}

#[test]
fn the_localization_table_labels_a_mean_over_the_seeds_that_localized_a_scan() {
    let mut table = String::new();
    localization_table(
        &mut table,
        &[localization_seed(0, Some(1.0)), localization_seed(1, None)],
    );
    assert!(
        table.contains(
            "\n| Mean ± standard error, the errors over the 1 of 2 seeds that localized a scan |"
        ),
        "{table}"
    );
}

#[test]
fn the_error_bound_is_the_largest_final_error_of_the_seeds_that_localized() {
    let within = LocalizedWithin {
        position_m: 0.5,
        yaw_rad: 0.35,
    };
    let seeds = [
        global_seed(0, Some(3.0), 0.2, 0.1),
        global_seed(1, None, 0.9, 0.3),
        global_seed(2, Some(4.0), 0.4, 0.05),
    ];
    let every: Vec<&GlobalLocalizationSeed> = seeds.iter().collect();
    assert_eq!(
        error_bound("by its convergence test", &every, within, |s| s
            .localized_after_s),
        "by its convergence test, 0.400 m and 5.7 deg over the 2 of 3 seeds that it localized, \
         each within the 0.5 m and 20.1 deg of the truth that count a seed localized"
    );
    assert_eq!(
        error_bound("by the search limits", &every, within, |s| s
            .localized_by_limits_after_s),
        "by the search limits, none: it localized 0 of 3 seeds"
    );
}
