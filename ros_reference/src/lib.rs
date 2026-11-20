//! The scenarios of the ROS reference harness and their reference outputs.
//!
//! A scenario under `scenarios/` is an input that a parity test and a ROS reference
//! both read. `run.sh` runs the reference of every scenario in the reference image and
//! writes its output under `reference/`, with the SHA-256 digest of each output in
//! `reference/SHA256SUMS` and of each file of the repository that an output depends on
//! (each scenario and the files it names, which the references read, and the sources of
//! each local-only kind) in `reference/INPUTS.SHA256SUMS`, and all of them are committed.
//! A parity test loads a scenario and its reference output through this crate and
//! compares its own output with the reference, with no ROS on the machine.
//! Reading a scenario, another input or a reference output checks it against its digest,
//! so a parity test fails on an output or an input that differs from the files `run.sh`
//! wrote and read. A change of the digests is a change of the reference, which
//! `run.sh --check` proves.
//!
//! The parity tests read the reference outputs from `reference/`, or from the directory
//! that the environment variable `ROS_REFERENCE_OUTPUTS` names, which then holds its own
//! digests. `run.sh` names the outputs of the arm64 image there. [`Outputs`] tells a
//! test which of the two it reads. The planning kind also reads the committed outputs of
//! `reference/`, the amd64 image's, wherever that variable points.
//!
//! Each kind of scenario is a module that lists the scenarios of its kind, reads each with
//! its reference output, and pairs them for the parity test. The README says how to add a
//! scenario and a kind.

pub mod amcl_global_localization;
pub mod arm_planning;
pub mod cartesian_paths;
pub mod chain_collision;
pub mod chomp_refinement;
pub mod collision_monitor;
pub mod diff_drive_odometry;
pub mod forward_kinematics;
pub mod global_costmap;
pub mod inflation;
pub mod laser_projection;
pub mod map_export;
pub mod metrics;
pub mod obstacle_projection;
pub mod occupancy;
pub mod orientation_constraints;
pub mod path_timing;
pub mod planning;
pub mod planning_scene;
pub mod pure_pursuit;
pub mod room_dataset;
pub mod self_distance;
pub mod smac_2d;
pub mod transform_lookup;
pub mod velocity_smoother;

use robot_description::{Arm, RobotDescription};
use serde::Deserialize;
use serde::de::DeserializeOwned;
use sha2::{Digest as _, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// The environment variable that names a directory of reference outputs, read in place of
/// `reference/`.
const REFERENCE_DIRECTORY_VARIABLE: &str = "ROS_REFERENCE_OUTPUTS";

/// The file of a directory of reference outputs that holds the SHA-256 digest of each
/// output, by file name.
const OUTPUT_DIGEST_FILE: &str = "SHA256SUMS";

/// The file of a directory of reference outputs that holds the SHA-256 digest of each
/// file of the repository that an output depends on, by its path from the repository root.
const INPUT_DIGEST_FILE: &str = "INPUTS.SHA256SUMS";

/// The kind of a scenario: the reference that runs on it, and the module of this crate
/// that reads it.
#[derive(PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Kind {
    AmclGlobalLocalization,
    ArmPlanning,
    CartesianPaths,
    ChainCollision,
    ChompRefinement,
    CollisionMonitor,
    DiffDriveOdometry,
    ForwardKinematics,
    GlobalCostmap,
    Inflation,
    LaserProjection,
    MapExport,
    ObstacleProjection,
    Occupancy,
    OrientationConstraints,
    PathTiming,
    Planning,
    PlanningScene,
    PurePursuit,
    RoomDataset,
    SelfDistance,
    #[serde(rename = "smac_2d")]
    Smac2d,
    TransformLookup,
    VelocitySmoother,
}

/// The directory of the harness, which holds `scenarios/` and `reference/`.
fn harness_directory() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// The root of this repository. The paths inside a scenario are relative to it.
fn repository_root() -> &'static Path {
    harness_directory()
        .parent()
        .expect("the harness is a directory of the repository root")
}

fn scenario_directory() -> PathBuf {
    harness_directory().join("scenarios")
}

/// The reference outputs that the parity tests read.
pub enum Outputs {
    /// `reference/`: the committed outputs, which the amd64 image wrote.
    Committed,
    /// The directory that `ROS_REFERENCE_OUTPUTS` names, which holds its own digests:
    /// `run.sh` on arm64 names the outputs of the arm64 image there.
    Named(PathBuf),
}

impl Outputs {
    /// The outputs that `ROS_REFERENCE_OUTPUTS` names, or the committed ones when it names
    /// none.
    pub fn from_environment() -> Self {
        std::env::var_os(REFERENCE_DIRECTORY_VARIABLE).map_or(Outputs::Committed, |directory| {
            Outputs::Named(PathBuf::from(directory))
        })
    }

    fn directory(self) -> PathBuf {
        match self {
            Outputs::Committed => harness_directory().join("reference"),
            Outputs::Named(directory) => directory,
        }
    }
}

/// The environment variable that names the results directory of the local-only
/// references, which `run.sh --datasets` sets for the parity tests of the local-only kinds.
const RESULTS_VARIABLE: &str = "ROS_REFERENCE_RESULTS";

/// The results directory of the local-only references, which `ROS_REFERENCE_RESULTS`
/// names: the inputs that they prepared and the runs that they ran, under each scenario's
/// name. Panics when it names none: `run.sh --datasets` runs the tests that read it.
pub fn results_directory() -> PathBuf {
    std::env::var_os(RESULTS_VARIABLE)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            panic!(
                "{RESULTS_VARIABLE} names no results directory; ros_reference/run.sh --datasets \
             runs the references and then this test with it"
            )
        })
}

/// The directory of the reference outputs that the parity tests read.
fn reference_directory() -> PathBuf {
    Outputs::from_environment().directory()
}

/// The file `<name>.json` of `directory`.
fn json_file(directory: &Path, name: &str) -> PathBuf {
    directory.join(format!("{name}.json"))
}

/// The names of the JSON files of `directory`, without `.json`, in order.
fn json_names(directory: &Path) -> Vec<String> {
    let entries = std::fs::read_dir(directory)
        .unwrap_or_else(|e| panic!("cannot list {}: {e}", directory.display()));
    let mut names: Vec<String> = entries
        .map(|entry| entry.expect("a directory entry reads").path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension == "json")
        })
        .map(|path| {
            path.file_stem()
                .and_then(|stem| stem.to_str())
                .expect("a file name is UTF-8")
                .to_owned()
        })
        .collect();
    names.sort();
    names
}

fn read_bytes(path: &Path) -> Vec<u8> {
    std::fs::read(path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

fn parse<T: DeserializeOwned>(path: &Path, bytes: &[u8]) -> T {
    serde_json::from_slice(bytes)
        .unwrap_or_else(|e| panic!("{} does not parse: {e}", path.display()))
}

/// The names of the scenarios of `kind`, in order. Panics on a scenario of a kind that
/// this crate has no module for.
fn scenario_names(kind: Kind) -> Vec<String> {
    #[derive(Deserialize)]
    struct KindOf {
        kind: Kind,
    }
    let directory = scenario_directory();
    json_names(&directory)
        .into_iter()
        .filter(|name| {
            let path = json_file(&directory, name);
            parse::<KindOf>(&path, &read_bytes(&path)).kind == kind
        })
        .collect()
}

/// The path of `path` from the repository root, the name of an input in
/// `INPUTS.SHA256SUMS`.
fn repository_path(path: &Path) -> &str {
    path.strip_prefix(repository_root())
        .ok()
        .and_then(Path::to_str)
        .unwrap_or_else(|| panic!("{} is outside the repository", path.display()))
}

/// Reads the input at `path`, after checking it against the digest of the file that the
/// references read.
fn read_input(path: &Path) -> Vec<u8> {
    let bytes = read_bytes(path);
    Digests::read(&reference_directory().join(INPUT_DIGEST_FILE))
        .check(repository_path(path), &bytes);
    bytes
}

/// The robot a scenario poses: the description of the model directory the scenario names,
/// and the directory of its collision meshes, whose files the URDF names by their file
/// names.
#[derive(Debug)]
pub struct Robot {
    pub description: RobotDescription,
    pub meshes: PathBuf,
}

/// The robot of the model directory `path`, relative to the repository root, that the
/// scenario `name` names, each file of it read with [`read_input`] and its mesh directory
/// with [`read_mesh_directory`].
fn read_robot(name: &str, path: &str) -> Robot {
    robot_from(name, path, read_input, |meshes| {
        read_mesh_directory(name, meshes)
    })
}

/// The robot of the model directory `path`, relative to the repository root, that the
/// scenario `name` names: the description that `<dir>.urdf` and `<dir>.srdf` of it make,
/// `<dir>` the directory's last part, each read with `read`, and its mesh directory
/// `meshes/`, given by `meshes`.
fn robot_from(
    name: &str,
    path: &str,
    read: impl Fn(&Path) -> Vec<u8>,
    meshes: impl FnOnce(&Path) -> PathBuf,
) -> Robot {
    let directory = repository_root().join(path);
    let model = directory
        .file_name()
        .and_then(|model| model.to_str())
        .unwrap_or_else(|| panic!("{name}: the description {path} names a directory"));
    let text = |extension: &str| {
        let file = directory.join(format!("{model}.{extension}"));
        String::from_utf8(read(&file))
            .unwrap_or_else(|e| panic!("{name}: {} is not UTF-8: {e}", file.display()))
    };
    let description = RobotDescription::parse(&text("urdf"), &text("srdf"))
        .unwrap_or_else(|e| panic!("{name}: the description {path}: {e}"));
    Robot {
        description,
        meshes: meshes(&directory.join("meshes")),
    }
}

impl Robot {
    /// The robot of the model directory `description`, relative to the repository root, as
    /// its files stand, with no digest checked: what an example reads that answers a
    /// scenario a tool of the harness draws before the references run on it.
    pub fn of(description: &str) -> Self {
        robot_from(description, description, read_bytes, Path::to_owned)
    }

    /// The arm `arm` of the robot, which the scenario `name` names.
    pub fn arm(&self, name: &str, arm: &str) -> Arm {
        self.description
            .arms()
            .iter()
            .find(|candidate| candidate.name == arm)
            .unwrap_or_else(|| panic!("{name}: the robot has no arm {arm}"))
            .clone()
    }

    /// The arms of the group `group` of the robot, which the scenario `name` names, in the
    /// order a state of the group holds their joints.
    pub fn group_arms(&self, name: &str, group: &str) -> Vec<Arm> {
        self.description
            .group_arms(group)
            .unwrap_or_else(|unknown| panic!("{name}: {unknown}"))
            .into_iter()
            .cloned()
            .collect()
    }

    /// The pairs of the robot's own links that a plan of the group `group`, which the
    /// scenario `name` names, checks, each pair's names in order: where the group holds every
    /// arm of the robot, each pair of links with collision geometry but those whose
    /// collisions the SRDF disables, as MoveIt's collision matrix of the SRDF checks them;
    /// none where it holds some.
    pub fn checked_pairs(&self, name: &str, group: &str) -> BTreeSet<[String; 2]> {
        if self.group_arms(name, group).len() < self.description.arms().len() {
            return BTreeSet::new();
        }
        let disabled: BTreeSet<[&str; 2]> = self
            .description
            .disabled_pairs()
            .iter()
            .map(|pair| ordered([pair.link1.as_str(), pair.link2.as_str()]))
            .collect();
        let links: Vec<&str> = self
            .description
            .robot()
            .links
            .iter()
            .filter(|link| !link.collision.is_empty())
            .map(|link| link.name.as_str())
            .collect();
        links
            .iter()
            .enumerate()
            .flat_map(|(i, a)| links[i + 1..].iter().map(move |b| ordered([*a, *b])))
            .filter(|pair| !disabled.contains(pair))
            .map(|pair| pair.map(str::to_owned))
            .collect()
    }
}

/// The names `pair`, in order.
fn ordered([a, b]: [&str; 2]) -> [&str; 2] {
    if a <= b { [a, b] } else { [b, a] }
}

/// The mesh directory `directory` that the scenario `name` names, after reading with
/// [`read_input`] each mesh of it that the references read, as `INPUTS.SHA256SUMS` lists
/// them.
fn read_mesh_directory(name: &str, directory: &Path) -> PathBuf {
    let meshes: Vec<PathBuf> = Digests::read(&reference_directory().join(INPUT_DIGEST_FILE))
        .by_file
        .into_keys()
        .map(|file| repository_root().join(file))
        .filter(|file| file.parent() == Some(directory))
        .collect();
    assert!(
        !meshes.is_empty(),
        "{name}: the references read no mesh of {}",
        repository_path(directory)
    );
    for mesh in &meshes {
        read_input(mesh);
    }
    directory.to_owned()
}

fn read_scenario<T: DeserializeOwned>(name: &str) -> T {
    let path = json_file(&scenario_directory(), name);
    parse(&path, &read_input(&path))
}

/// Reads the reference output `name`, after checking it against its digest.
fn read_reference<T: DeserializeOwned>(name: &str) -> T {
    read_reference_in(&reference_directory(), name)
}

/// Reads the committed reference output `name` from `reference/`, after checking it
/// against its digest.
fn read_committed_reference<T: DeserializeOwned>(name: &str) -> T {
    read_reference_in(&Outputs::Committed.directory(), name)
}

/// Reads the reference output `name` from `directory`, after checking it against its
/// digest there.
fn read_reference_in<T: DeserializeOwned>(directory: &Path, name: &str) -> T {
    let path = json_file(directory, name);
    let bytes = read_bytes(&path);
    Digests::read(&directory.join(OUTPUT_DIGEST_FILE)).check(&format!("{name}.json"), &bytes);
    parse(&path, &bytes)
}

/// The SHA-256 digest of each file of a digest file that `sha256sum` wrote, by the name
/// that it gives the file.
struct Digests {
    path: PathBuf,
    by_file: BTreeMap<String, String>,
}

impl Digests {
    fn read(path: &Path) -> Self {
        let text = String::from_utf8(read_bytes(path))
            .unwrap_or_else(|e| panic!("{} is not UTF-8: {e}", path.display()));
        let by_file = Self::parse(&text).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        Digests {
            path: path.to_owned(),
            by_file,
        }
    }

    /// Reads the lines that `sha256sum` writes: 64 lowercase hexadecimal digits, two
    /// spaces and a file name, one file a line.
    fn parse(text: &str) -> Result<BTreeMap<String, String>, String> {
        text.lines().try_fold(BTreeMap::new(), |mut by_file, line| {
            let (digest, file) = line
                .split_once("  ")
                .ok_or_else(|| format!("{line:?} is not a digest, two spaces and a file name"))?;
            if digest.len() != 64
                || !digest
                    .bytes()
                    .all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
            {
                return Err(format!("{digest:?} is not 64 lowercase hexadecimal digits"));
            }
            match by_file.insert(file.to_owned(), digest.to_owned()) {
                Some(_) => Err(format!("{file} has two digests")),
                None => Ok(by_file),
            }
        })
    }

    /// Panics unless `bytes`, the content of `file`, has the digest that the digest file
    /// gives it.
    fn check(&self, file: &str, bytes: &[u8]) {
        let expected = self.by_file.get(file).unwrap_or_else(|| {
            panic!(
                "{} holds no digest of {file}; {RUN_SH_WRITES_DIGESTS}",
                self.path.display()
            )
        });
        let actual: String = Sha256::digest(bytes)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        assert_eq!(
            &actual,
            expected,
            "{file} differs from the file whose digest {} holds; {RUN_SH_WRITES_DIGESTS}",
            self.path.display()
        );
    }
}

/// What a developer runs to write a digest that [`Digests::check`] refuses.
const RUN_SH_WRITES_DIGESTS: &str = "run ros_reference/run.sh, or ros_reference/run.sh \
     --datasets when an output of a local-only kind lists the file under its inputs, which \
     writes the reference outputs and the digests of them and of their inputs";

/// Writes `document` to `path` with a final newline, by the rule that lays out every
/// scenario and every reference output: a scalar, an array of scalars and a record, an
/// object whose members are each a scalar or an array of scalars, go on one line; every
/// other array or object spreads over lines, one member a line, indented by two spaces.
/// Each float goes as [`shortest_decimal`] writes it, as the reference program writes it.
fn write_json(path: &Path, document: &serde_json::Value) {
    let text = format!("{}\n", layout(document, ""));
    std::fs::write(path, text).unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));
}

fn is_flat(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Array(members) => {
            members.iter().all(|m| !m.is_array() && !m.is_object())
        }
        serde_json::Value::Object(_) => false,
        _ => true,
    }
}

fn is_one_line(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Object(members) => members.values().all(is_flat),
        _ => is_flat(value),
    }
}

/// The two decimals of the finite `x` that `std::to_chars` writes the shorter of: its
/// shortest digits with a point, and in scientific notation, with a sign and at least two
/// digits of exponent.
fn shortest_decimals(x: f64) -> [String; 2] {
    let scientific = format!("{x:e}");
    let (digits, exponent) = scientific
        .split_once('e')
        .expect("a finite double writes an exponent");
    let exponent: i32 = exponent.parse().expect("the exponent is an integer");
    let sign = if exponent < 0 { '-' } else { '+' };
    [
        format!("{x}"),
        format!("{digits}e{sign}{:02}", exponent.unsigned_abs()),
    ]
}

/// The decimal of `x` that `std::to_chars` writes, the shorter of [`shortest_decimals`],
/// the one with a point when both are as long. Panics on a NaN or an infinity, which JSON
/// has no number for.
fn shortest_decimal(x: f64) -> String {
    assert!(
        x.is_finite(),
        "an output holds the non-finite value {x}, which JSON has no number for"
    );
    let [point, scientific] = shortest_decimals(x);
    if scientific.len() < point.len() {
        scientific
    } else {
        point
    }
}

/// The text of `value` on one line, as nlohmann::json dumps it, each float as
/// [`shortest_decimal`] writes it.
fn one_line(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::Number(number) => match number.as_f64() {
            Some(x) if number.is_f64() => shortest_decimal(x),
            _ => number.to_string(),
        },
        serde_json::Value::Array(members) => format!(
            "[{}]",
            members.iter().map(one_line).collect::<Vec<_>>().join(",")
        ),
        serde_json::Value::Object(members) => format!(
            "{{{}}}",
            members
                .iter()
                .map(|(key, member)| format!(
                    "{}:{}",
                    serde_json::Value::from(key.as_str()),
                    one_line(member)
                ))
                .collect::<Vec<_>>()
                .join(",")
        ),
        _ => value.to_string(),
    }
}

/// The text of `value` by the rule of [`write_json`], its later lines indented by `indent`.
fn layout(value: &serde_json::Value, indent: &str) -> String {
    if is_one_line(value) {
        return one_line(value);
    }
    let inner = format!("{indent}  ");
    let (open, members, close): (char, Vec<String>, char) = match value {
        serde_json::Value::Object(members) => (
            '{',
            members
                .iter()
                .map(|(key, member)| {
                    format!(
                        "{inner}{}: {}",
                        serde_json::Value::from(key.as_str()),
                        layout(member, &inner)
                    )
                })
                .collect(),
            '}',
        ),
        serde_json::Value::Array(members) => (
            '[',
            members
                .iter()
                .map(|member| format!("{inner}{}", layout(member, &inner)))
                .collect(),
            ']',
        ),
        _ => unreachable!("a scalar goes on one line"),
    };
    format!("{open}\n{}\n{indent}{close}", members.join(",\n"))
}

/// The transform of `link` in the frame of the URDF root.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LinkPose {
    pub link: String,
    pub translation: [f64; 3],
    /// The rotation matrix, row by row.
    pub rotation: [f64; 9],
}

/// A cost grid as a scenario of a Nav2 kind gives it: its size in cells, the side of a
/// cell in meters, and the corner of its first cell.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Grid {
    pub size_x: u32,
    pub size_y: u32,
    pub resolution: f64,
    pub origin: [f64; 2],
}

/// Where a sensor stands in a scenario's frame: its position, and its orientation as a unit
/// quaternion [x, y, z, w].
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorTransform {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// `value`, a number a reference holds in single precision, which its output carries as
/// the double that holds it, as that single precision number. Panics on a double no single
/// precision number holds: an output that carries such a number is not the reference's.
fn read_single(value: f64) -> f32 {
    let single = value as f32;
    assert_eq!(
        f64::from(single).to_bits(),
        value.to_bits(),
        "{value} is no single precision number"
    );
    single
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    const DIGEST: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

    /// The doubles at the edges of the format, by their bits, and the decimal that
    /// `std::to_chars` writes for each, as `check_shortest_decimals` of the reference
    /// program holds them: the smallest and the largest subnormal, the smallest normal,
    /// the largest finite value and its negation, and both zeros.
    const EDGE_DECIMALS: [(u64, &str); 7] = [
        (0x0000_0000_0000_0001, "5e-324"),
        (0x000f_ffff_ffff_ffff, "2.225073858507201e-308"),
        (0x0010_0000_0000_0000, "2.2250738585072014e-308"),
        (0x7fef_ffff_ffff_ffff, "1.7976931348623157e+308"),
        (0xffef_ffff_ffff_ffff, "-1.7976931348623157e+308"),
        (0x0000_0000_0000_0000, "0"),
        (0x8000_0000_0000_0000, "-0"),
    ];

    /// Reads `text` as a reference output reads a number.
    fn read_number(text: &str) -> serde_json::Result<f64> {
        serde_json::from_str(text)
    }

    /// The file names of the outputs `names`.
    fn output_files(names: &[String]) -> BTreeSet<String> {
        names.iter().map(|name| format!("{name}.json")).collect()
    }

    /// Panics unless `digests` names exactly the files `files`.
    fn assert_digests_name(digests: &Digests, files: &BTreeSet<String>) {
        assert_eq!(
            digests
                .by_file
                .keys()
                .cloned()
                .collect::<BTreeSet<String>>(),
            *files,
            "{} names the files on the left, and they are those on the right",
            digests.path.display()
        );
    }

    /// The bytes of `bytes` with their first digit changed.
    fn with_one_digit_changed(mut bytes: Vec<u8>) -> Vec<u8> {
        let digit = bytes
            .iter_mut()
            .find(|byte| byte.is_ascii_digit())
            .expect("the text holds a digit");
        *digit = if *digit == b'0' { b'1' } else { b'0' };
        bytes
    }

    #[test]
    fn the_layout_is_that_of_the_reference_program() {
        let directory = reference_directory();
        let path = json_file(&directory, &json_names(&directory)[0]);
        let text = String::from_utf8(read_bytes(&path)).expect("UTF-8");
        let document: serde_json::Value = parse(&path, text.as_bytes());
        assert_eq!(format!("{}\n", layout(&document, "")), text);
    }

    #[hegel::test]
    fn every_finite_double_reads_back_from_its_shortest_decimals(tc: TestCase) {
        let x = f64::from_bits(tc.draw(gs::integers::<u64>()));
        tc.assume(x.is_finite());
        for decimal in shortest_decimals(x) {
            let read = read_number(&decimal).unwrap_or_else(|e| panic!("{decimal}: {e}"));
            assert_eq!(read.to_bits(), x.to_bits(), "{decimal}");
        }
    }

    #[test]
    fn the_decimal_of_each_double_at_the_edges_reads_back_to_its_bits() {
        for (bits, decimal) in EDGE_DECIMALS {
            let x = f64::from_bits(bits);
            assert!(
                shortest_decimals(x).contains(&decimal.to_owned()),
                "{decimal} is a shortest decimal of {x:e}"
            );
            let read = read_number(decimal).unwrap_or_else(|e| panic!("{decimal}: {e}"));
            assert_eq!(read.to_bits(), bits, "{decimal}");
        }
    }

    // A double that holds a single precision number reads back to it; any other double
    // is refused.
    #[test]
    fn a_single_precision_number_reads_back_from_the_double_that_holds_it() {
        assert_eq!(read_single(f64::from(0.1_f32)), 0.1_f32);
        assert_eq!(read_single(-2.5), -2.5_f32);
        assert!(std::panic::catch_unwind(|| read_single(0.1)).is_err());
    }

    #[test]
    fn each_double_writes_as_std_to_chars_writes_it() {
        let written = [
            (0.1, "0.1"),
            (0.001, "0.001"),
            (1e-5, "1e-05"),
            (1e-7, "1e-07"),
            (100.0, "100"),
            (123_456_789_012.0, "123456789012"),
            (1e21, "1e+21"),
            (-2.5, "-2.5"),
        ];
        for (x, decimal) in written {
            assert_eq!(shortest_decimal(x), decimal, "{x:e}");
        }
        for (bits, decimal) in EDGE_DECIMALS {
            assert_eq!(shortest_decimal(f64::from_bits(bits)), decimal);
        }
    }

    #[hegel::test]
    fn the_written_decimal_is_the_shorter_of_the_two_and_reads_back(tc: TestCase) {
        let x = f64::from_bits(tc.draw(gs::integers::<u64>()));
        tc.assume(x.is_finite());
        let written = shortest_decimal(x);
        let [point, scientific] = shortest_decimals(x);
        assert_eq!(
            written.len(),
            point.len().min(scientific.len()),
            "{written}"
        );
        let read = read_number(&written).unwrap_or_else(|e| panic!("{written}: {e}"));
        assert_eq!(read.to_bits(), x.to_bits(), "{written}");
    }

    #[test]
    fn a_line_holds_its_members_as_nlohmann_dumps_them() {
        let value: serde_json::Value =
            serde_json::from_str(r#"{"b": [1, 2.5, 1e-7], "a": "x\"y", "c": null}"#)
                .expect("parses");
        assert_eq!(
            one_line(&value),
            r#"{"a":"x\"y","b":[1,2.5,1e-07],"c":null}"#
        );
    }

    #[test]
    #[should_panic(expected = "which JSON has no number for")]
    fn a_non_finite_value_is_refused_by_the_writer() {
        shortest_decimal(f64::INFINITY);
    }

    #[test]
    fn a_number_that_is_not_a_finite_double_is_refused() {
        for text in ["NaN", "Infinity", "-Infinity", "1e309", "-1e309"] {
            assert!(read_number(text).is_err(), "{text} was read");
        }
    }

    #[test]
    fn the_lines_of_sha256sum_read_as_digests() {
        let text = format!("{DIGEST}  a.json\n{DIGEST}  b.json\n");
        let by_file = Digests::parse(&text).expect("the lines read");
        assert_eq!(by_file.keys().collect::<Vec<_>>(), ["a.json", "b.json"]);
        assert_eq!(by_file["a.json"], DIGEST);
    }

    #[test]
    fn a_line_that_is_not_a_digest_and_a_file_name_is_refused() {
        let uppercase = DIGEST.to_uppercase();
        let short = &DIGEST[1..];
        for text in [
            format!("{DIGEST} a.json"),
            format!("{uppercase}  a.json"),
            format!("{short}  a.json"),
            format!("{DIGEST}  a.json\n{DIGEST}  a.json"),
        ] {
            assert!(Digests::parse(&text).is_err(), "{text:?} was read");
        }
    }

    #[test]
    fn digests_name_outputs_whose_names_sort_apart_from_their_file_names() {
        let digests = Digests {
            path: PathBuf::from("SHA256SUMS"),
            by_file: Digests::parse(&format!("{DIGEST}  x-left.json\n{DIGEST}  x.json\n"))
                .expect("the lines read"),
        };
        assert_digests_name(&digests, &output_files(&["x".into(), "x-left".into()]));
    }

    #[test]
    fn every_scenario_has_a_reference_output() {
        let outputs: BTreeSet<String> = json_names(&reference_directory()).into_iter().collect();
        for name in json_names(&scenario_directory()) {
            assert!(
                outputs.contains(&name),
                "scenarios/{name}.json has no reference output"
            );
        }
    }

    #[test]
    fn every_reference_output_has_a_scenario() {
        let scenarios: BTreeSet<String> = json_names(&scenario_directory()).into_iter().collect();
        for name in json_names(&reference_directory()) {
            assert!(
                scenarios.contains(&name),
                "the reference output {name}.json has no scenario"
            );
        }
    }

    #[test]
    fn the_digests_name_every_reference_output_and_match_it() {
        let directory = reference_directory();
        let digests = Digests::read(&directory.join(OUTPUT_DIGEST_FILE));
        let files = output_files(&json_names(&directory));
        assert_digests_name(&digests, &files);
        for file in &files {
            digests.check(file, &read_bytes(&directory.join(file)));
        }
    }

    #[test]
    fn every_scenario_and_input_matches_its_digest() {
        let digests = Digests::read(&reference_directory().join(INPUT_DIGEST_FILE));
        for name in json_names(&scenario_directory()) {
            read_input(&json_file(&scenario_directory(), &name));
        }
        for file in digests.by_file.keys() {
            digests.check(file, &read_bytes(&repository_root().join(file)));
        }
    }

    #[test]
    #[should_panic(expected = "holds no digest of README.md")]
    fn a_file_that_no_reference_read_is_refused_as_an_input() {
        read_input(&repository_root().join("README.md"));
    }

    // A plan of a group that holds every arm of its robot checks each pair of the robot's
    // links with collision geometry but those the SRDF disables, each pair once, its names
    // in order; a plan of one arm of two checks none.
    #[test]
    fn a_group_of_every_arm_checks_each_pair_the_srdf_leaves_checked() {
        for (path, every_arm) in [
            ("robot_description/models/openarm_v2", "both_arms"),
            ("robot_description/models/so101", "arm"),
        ] {
            let robot = Robot::of(path);
            let description = &robot.description;
            let checked = robot.checked_pairs(path, every_arm);
            let links = description
                .robot()
                .links
                .iter()
                .filter(|link| !link.collision.is_empty())
                .count();
            let disabled: BTreeSet<[&str; 2]> = description
                .disabled_pairs()
                .iter()
                .map(|pair| ordered([pair.link1.as_str(), pair.link2.as_str()]))
                .collect();
            assert_eq!(
                checked.len(),
                links * (links - 1) / 2 - disabled.len(),
                "{path}"
            );
            assert!(
                checked
                    .iter()
                    .all(|[a, b]| a < b && !disabled.contains(&[a.as_str(), b.as_str()])),
                "{path}"
            );
        }
        let openarm = Robot::of("robot_description/models/openarm_v2");
        assert!(openarm.checked_pairs("scenario", "left_arm").is_empty());
    }

    #[test]
    #[should_panic(expected = "the references read no mesh of ros_reference/tools")]
    fn a_mesh_directory_that_no_reference_read_is_refused() {
        read_mesh_directory("scenario", &repository_root().join("ros_reference/tools"));
    }

    #[test]
    #[should_panic(expected = "run ros_reference/run.sh")]
    fn a_reference_output_with_one_digit_changed_fails_its_digest() {
        let directory = reference_directory();
        let file = format!("{}.json", json_names(&directory)[0]);
        let bytes = with_one_digit_changed(read_bytes(&directory.join(&file)));
        Digests::read(&directory.join(OUTPUT_DIGEST_FILE)).check(&file, &bytes);
    }

    #[test]
    #[should_panic(expected = "run ros_reference/run.sh")]
    fn a_scenario_with_one_byte_changed_fails_its_digest() {
        let path = json_file(&scenario_directory(), &json_names(&scenario_directory())[0]);
        let mut bytes = read_bytes(&path);
        bytes.push(b'\n');
        Digests::read(&reference_directory().join(INPUT_DIGEST_FILE))
            .check(repository_path(&path), &bytes);
    }
}
