//! The scenarios of the ROS reference harness and their reference outputs.
//!
//! A scenario under `scenarios/` is an input that a parity test and a ROS reference
//! both read. `run.sh` runs the reference of every scenario in the reference image and
//! writes its output under `reference/`, with the SHA-256 digest of each output in
//! `reference/SHA256SUMS` and of each file of the repository that the references read
//! (each scenario and the files it names) in `reference/INPUTS.SHA256SUMS`, and all of
//! them are committed. A parity test loads a scenario and its reference output through
//! this crate and compares its own output with the reference, with no ROS on the machine.
//! Reading a scenario, another input or a reference output checks it against its digest,
//! so a parity test fails on an output or an input that differs from the files `run.sh`
//! wrote and read. A change of the digests is a change of the reference, which
//! `run.sh --check` proves.
//!
//! The parity tests read the reference outputs from `reference/`, or from the directory
//! that the environment variable `ROS_REFERENCE_OUTPUTS` names, which then holds its own
//! digests. `run.sh` names the outputs of the arm64 image there.
//!
//! Each kind of scenario is a module that lists the scenarios of its kind, reads each with
//! its reference output, and pairs them for the parity test. The README says how to add a
//! scenario and a kind.

pub mod forward_kinematics;

use serde::Deserialize;
use serde::de::DeserializeOwned;
use sha2::{Digest as _, Sha256};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// The environment variable that names a directory of reference outputs, read in place of
/// `reference/`.
const REFERENCE_DIRECTORY_VARIABLE: &str = "ROS_REFERENCE_OUTPUTS";

/// The file of a directory of reference outputs that holds the SHA-256 digest of each
/// output, by file name.
const OUTPUT_DIGEST_FILE: &str = "SHA256SUMS";

/// The file of a directory of reference outputs that holds the SHA-256 digest of each
/// input that the references read, by its path from the repository root.
const INPUT_DIGEST_FILE: &str = "INPUTS.SHA256SUMS";

/// The kind of a scenario: the reference that runs on it, and the module of this crate
/// that reads it.
#[derive(PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Kind {
    ForwardKinematics,
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

/// The directory of the reference outputs: the one that `ROS_REFERENCE_OUTPUTS` names, or
/// `reference/`.
fn reference_directory() -> PathBuf {
    std::env::var_os(REFERENCE_DIRECTORY_VARIABLE)
        .map_or_else(|| harness_directory().join("reference"), PathBuf::from)
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

fn read_scenario<T: DeserializeOwned>(name: &str) -> T {
    let path = json_file(&scenario_directory(), name);
    parse(&path, &read_input(&path))
}

/// Reads the reference output `name`, after checking it against its digest.
fn read_reference<T: DeserializeOwned>(name: &str) -> T {
    let directory = reference_directory();
    let path = json_file(&directory, name);
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
                "{} holds no digest of {file}; run ros_reference/run.sh, which writes the \
                 reference outputs and the digests of them and of their inputs",
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
            "{file} differs from the file whose digest {} holds; run ros_reference/run.sh, \
             which writes the reference outputs and the digests of them and of their inputs",
            self.path.display()
        );
    }
}

/// A float that a reference output carries as its IEEE-754 bit pattern: `0x` and 16
/// hexadecimal digits. A parity test then compares the reference's own bits.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(try_from = "String")]
pub struct Bits(pub f64);

impl TryFrom<String> for Bits {
    type Error = String;

    fn try_from(text: String) -> Result<Self, String> {
        let digits = text
            .strip_prefix("0x")
            .filter(|digits| digits.len() == 16 && digits.bytes().all(|b| b.is_ascii_hexdigit()))
            .ok_or_else(|| format!("{text:?} is not 0x and 16 hexadecimal digits"))?;
        let pattern = u64::from_str_radix(digits, 16).map_err(|e| format!("{text:?}: {e}"))?;
        Ok(Bits(f64::from_bits(pattern)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    const DIGEST: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

    fn bits(text: &str) -> Result<Bits, String> {
        Bits::try_from(text.to_owned())
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

    /// The bytes of `bytes` with the first hexadecimal digit after a `0x` changed.
    fn with_one_digit_changed(mut bytes: Vec<u8>) -> Vec<u8> {
        let digit = bytes
            .windows(2)
            .position(|pair| pair == b"0x")
            .expect("the text holds a bit pattern")
            + 2;
        bytes[digit] = if bytes[digit] == b'0' { b'1' } else { b'0' };
        bytes
    }

    #[test]
    fn a_bit_pattern_reads_as_its_float() {
        assert_eq!(bits("0x3ff0000000000000"), Ok(Bits(1.0)));
        assert_eq!(bits("0xC000000000000000"), Ok(Bits(-2.0)));
        let negative_zero = bits("0x8000000000000000").unwrap().0;
        assert_eq!(negative_zero.to_bits(), (-0.0f64).to_bits());
    }

    #[test]
    fn a_text_that_is_not_a_bit_pattern_is_refused() {
        for text in [
            "3ff0000000000000",
            "0x3ff000000000000",
            "0x3ff00000000000000",
            "0x+ff0000000000000",
            "0x3ff000000000000g",
            "1.0",
        ] {
            assert!(bits(text).is_err(), "{text} was read");
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
