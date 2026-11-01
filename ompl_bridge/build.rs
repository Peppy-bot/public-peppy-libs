//! Builds OMPL's library sources and the bridge into one static library. The sources
//! come from the source archive of OMPL's release tag: the copy that
//! `OMPL_SOURCE_TARBALL` names, or the one in the download cache that
//! `PEPPY_DOWNLOAD_CACHE` names, or else downloaded from GitHub and stored in that
//! cache; each is checked against the archive's SHA-256 digest and extracted into
//! `OUT_DIR`. OMPL needs Boost's headers, from the compiler's own include path, its
//! Boost.Serialization library, from the linker's, and Eigen 3's headers, from
//! pkg-config.

mod download_cache;

use std::env;
use std::fmt;
use std::fs::{self, File};
use std::io::{self, Read};
use std::path::{Path, PathBuf};
use std::time::Duration;

use flate2::read::GzDecoder;

/// The OMPL release that the build compiles.
const OMPL_VERSION: (u32, u32, u32) = (2, 0, 2);

/// The SHA-256 digest of the source archive of the release tag.
const OMPL_ARCHIVE_SHA256: &str =
    "d867190a46a7f730923475a68af2e09fc9e796017a59be395a4bf3900d0c3ae4";

/// The size of the source archive in bytes. The build reads at most this much of a
/// download, of a local copy or of the download cache's copy.
const OMPL_ARCHIVE_BYTES: u64 = 77_135_911;

/// How long the download may take to connect to GitHub.
const DOWNLOAD_CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// How long the whole download may take, its connection included: the archive's
/// 77,135,911 bytes within 600 s, 128.6 kB/s on average.
const DOWNLOAD_TIMEOUT: Duration = Duration::from_secs(600);

/// The environment variable that names a local copy of the source archive, read in
/// place of the download.
const ARCHIVE_VARIABLE: &str = "OMPL_SOURCE_TARBALL";

/// The environment variable that names the download cache of a peppy container build,
/// which holds the archive under its digest once a build has downloaded it.
const DOWNLOAD_CACHE_VARIABLE: &str = "PEPPY_DOWNLOAD_CACHE";

/// The library sources the build compiles, under the release's `src/ompl`: the
/// utilities, the data structures, the base module less the state spaces that no other
/// source of it uses, the geometric core and the crate's eleven planners, the
/// configuration tools, and the control core with RRT and KPIECE1, which the
/// configuration tools name.
const OMPL_SOURCES: &[&str] = &[
    "base/goals/src/GoalLazySamples.cpp",
    "base/goals/src/GoalRegion.cpp",
    "base/goals/src/GoalSpace.cpp",
    "base/goals/src/GoalState.cpp",
    "base/goals/src/GoalStates.cpp",
    "base/objectives/src/ControlDurationObjective.cpp",
    "base/objectives/src/MaximizeMinClearanceObjective.cpp",
    "base/objectives/src/MechanicalWorkOptimizationObjective.cpp",
    "base/objectives/src/MinimaxObjective.cpp",
    "base/objectives/src/MinimizeArrivalTime.cpp",
    "base/objectives/src/PathLengthOptimizationObjective.cpp",
    "base/objectives/src/StateCostIntegralObjective.cpp",
    "base/samplers/deterministic/src/HaltonSequence.cpp",
    "base/samplers/deterministic/src/PrecomputedSequence.cpp",
    "base/samplers/informed/src/OrderedInfSampler.cpp",
    "base/samplers/informed/src/PathLengthDirectInfSampler.cpp",
    "base/samplers/informed/src/RejectionInfSampler.cpp",
    "base/samplers/src/BridgeTestValidStateSampler.cpp",
    "base/samplers/src/ConditionalStateSampler.cpp",
    "base/samplers/src/DeterministicStateSampler.cpp",
    "base/samplers/src/GaussianValidStateSampler.cpp",
    "base/samplers/src/InformedStateSampler.cpp",
    "base/samplers/src/MaximizeClearanceValidStateSampler.cpp",
    "base/samplers/src/MinimumClearanceValidStateSampler.cpp",
    "base/samplers/src/ObstacleBasedValidStateSampler.cpp",
    "base/samplers/src/UniformValidStateSampler.cpp",
    "base/spaces/constraint/src/AtlasChart.cpp",
    "base/spaces/constraint/src/AtlasStateSpace.cpp",
    "base/spaces/constraint/src/ConstrainedStateSpace.cpp",
    "base/spaces/constraint/src/ProjectedStateSpace.cpp",
    "base/spaces/constraint/src/TangentBundleStateSpace.cpp",
    "base/spaces/src/DubinsStateSpace.cpp",
    "base/spaces/src/OwenStateSpace.cpp",
    "base/spaces/src/RealVectorBounds.cpp",
    "base/spaces/src/RealVectorStateProjections.cpp",
    "base/spaces/src/RealVectorStateSpace.cpp",
    "base/spaces/src/ReedsSheppStateSpace.cpp",
    "base/spaces/src/SE2StateSpace.cpp",
    "base/spaces/src/SO2StateSpace.cpp",
    "base/spaces/src/SpaceTimeStateSpace.cpp",
    "base/spaces/src/TimeStateSpace.cpp",
    "base/spaces/src/TrochoidStateSpace.cpp",
    "base/spaces/src/VanaOwenStateSpace.cpp",
    "base/spaces/src/VanaStateSpace.cpp",
    "base/spaces/src/WrapperStateSpace.cpp",
    "base/src/Constraint.cpp",
    "base/src/Cost.cpp",
    "base/src/DiscreteMotionValidator.cpp",
    "base/src/GenericParam.cpp",
    "base/src/Goal.cpp",
    "base/src/OptimizationObjective.cpp",
    "base/src/Planner.cpp",
    "base/src/PlannerData.cpp",
    "base/src/PlannerDataStorage.cpp",
    "base/src/PlannerStatus.cpp",
    "base/src/PlannerTerminationCondition.cpp",
    "base/src/PrecomputedStateSampler.cpp",
    "base/src/ProblemDefinition.cpp",
    "base/src/ProjectionEvaluator.cpp",
    "base/src/SpaceInformation.cpp",
    "base/src/StateSampler.cpp",
    "base/src/StateSpace.cpp",
    "base/src/StateStorage.cpp",
    "base/src/ValidStateSampler.cpp",
    "base/terminationconditions/src/CostConvergenceTerminationCondition.cpp",
    "base/terminationconditions/src/IterationTerminationCondition.cpp",
    "control/planners/kpiece/src/KPIECE1.cpp",
    "control/planners/rrt/src/RRT.cpp",
    "control/spaces/src/DiscreteControlSpace.cpp",
    "control/spaces/src/RealVectorControlSpace.cpp",
    "control/src/ControlSampler.cpp",
    "control/src/ControlSpace.cpp",
    "control/src/PathControl.cpp",
    "control/src/PlannerData.cpp",
    "control/src/PlannerDataStorage.cpp",
    "control/src/SimpleDirectedControlSampler.cpp",
    "control/src/SimpleSetup.cpp",
    "control/src/SpaceInformation.cpp",
    "datastructures/src/AdjacencyList.cpp",
    "geometric/planners/est/src/EST.cpp",
    "geometric/planners/kpiece/src/BKPIECE1.cpp",
    "geometric/planners/kpiece/src/KPIECE1.cpp",
    "geometric/planners/kpiece/src/LBKPIECE1.cpp",
    "geometric/planners/prm/src/PRM.cpp",
    "geometric/planners/prm/src/PRMstar.cpp",
    "geometric/planners/rrt/src/BiTRRT.cpp",
    "geometric/planners/rrt/src/RRT.cpp",
    "geometric/planners/rrt/src/RRTConnect.cpp",
    "geometric/planners/rrt/src/RRTstar.cpp",
    "geometric/planners/sbl/src/SBL.cpp",
    "geometric/src/GeneticSearch.cpp",
    "geometric/src/HillClimbing.cpp",
    "geometric/src/PathGeometric.cpp",
    "geometric/src/PathHybridization.cpp",
    "geometric/src/PathSimplifier.cpp",
    "geometric/src/SimpleSetup.cpp",
    "tools/config/src/SelfConfig.cpp",
    "util/src/Console.cpp",
    "util/src/GeometricEquations.cpp",
    "util/src/PPM.cpp",
    "util/src/ProlateHyperspheroid.cpp",
    "util/src/RandomNumbers.cpp",
    "util/src/String.cpp",
    "util/src/Time.cpp",
];

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    let release = extracted_release(&out);
    let sources = release.join("src");
    let generated = out.join("include");
    write_config(&sources, &generated.join("ompl"));
    let eigen = pkg_config::Config::new()
        .cargo_metadata(false)
        .probe("eigen3")
        .unwrap_or_else(|error| {
            panic!(
                "ompl_bridge needs Eigen 3's headers (Debian and Ubuntu: libeigen3-dev): {error}"
            )
        });
    probe_boost(&out);
    cxx_build::bridge("src/ffi.rs")
        .file("src/bridge.cpp")
        .files(
            OMPL_SOURCES
                .iter()
                .map(|source| sources.join("ompl").join(source)),
        )
        .include(&generated)
        .include(&sources)
        .includes(&eigen.include_paths)
        .std("c++17")
        // OMPL's CMake builds Release by default, which defines it and compiles out
        // OMPL's assertions.
        .define("NDEBUG", None)
        // OMPL's CMake defines it in every build: Boost.Math computes in double, never
        // in long double.
        .define("BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS", None)
        // Each product and each sum of OMPL's sources and the bridge rounds on its own,
        // as on the x86-64 baseline, in a build for a CPU with fused multiply-add too.
        // Eigen's vectorized kernels fuse by intrinsics, which the flag does not reach.
        .flag("-ffp-contract=off")
        .flag_if_supported("-Wno-deprecated-declarations")
        .warnings(false)
        .compile("ompl_bridge");
    // OMPL's planner data writes and reads itself through Boost.Serialization, which
    // OMPL's CMake links publicly.
    println!("cargo:rustc-link-lib=boost_serialization");
    println!("cargo:rerun-if-changed=src/bridge.cpp");
    println!("cargo:rerun-if-changed=src/bridge.h");
}

/// The release as OMPL tags it.
fn ompl_release() -> String {
    let (major, minor, patch) = OMPL_VERSION;
    format!("{major}.{minor}.{patch}")
}

/// The source archive of the release tag on GitHub.
fn archive_url() -> String {
    format!(
        "https://github.com/ompl/ompl/archive/refs/tags/{}.tar.gz",
        ompl_release()
    )
}

/// The release's tree in `out`, which holds its `LICENSE` and its `src/ompl`. The build
/// extracts it from the source archive, and reuses an extraction whose marker holds the
/// archive's digest. It checks a local copy that `OMPL_SOURCE_TARBALL` names on each
/// run, and reads the download cache or downloads the archive only to extract it.
fn extracted_release(out: &Path) -> PathBuf {
    println!("cargo:rerun-if-env-changed={ARCHIVE_VARIABLE}");
    println!("cargo:rerun-if-env-changed={DOWNLOAD_CACHE_VARIABLE}");
    let local_archive =
        env::var_os(ARCHIVE_VARIABLE).map(|path| read_local_archive(Path::new(&path)));
    let root = format!("ompl-{}", ompl_release());
    let release = out.join(&root);
    let marker = out.join(format!("{root}.sha256"));
    if fs::read_to_string(&marker).is_ok_and(|digest| digest == OMPL_ARCHIVE_SHA256) {
        return release;
    }
    let archive = local_archive.unwrap_or_else(cached_or_downloaded_archive);
    if release.exists() {
        fs::remove_dir_all(&release).expect("the removal of an earlier extraction");
    }
    extract(&archive, &root, out);
    fs::write(&marker, OMPL_ARCHIVE_SHA256).expect("the extraction's digest marker");
    release
}

/// The local copy of the source archive at `path`, checked against the archive's digest.
fn read_local_archive(path: &Path) -> Vec<u8> {
    assert!(
        path.is_absolute(),
        "{ARCHIVE_VARIABLE} names {}, a relative path; name the archive by its absolute path, \
         as in {ARCHIVE_VARIABLE}=\"$PWD/ompl-{}.tar.gz\"",
        path.display(),
        ompl_release()
    );
    println!("cargo:rerun-if-changed={}", path.display());
    let file = File::open(path).unwrap_or_else(|error| {
        panic!(
            "{ARCHIVE_VARIABLE} names {}, which the build cannot open: {error}",
            path.display()
        )
    });
    let source = format!(
        "{}, which {ARCHIVE_VARIABLE} names in place of {},",
        path.display(),
        archive_url()
    );
    let archive =
        read_archive(file).unwrap_or_else(|error| panic!("{source} could not be read: {error}"));
    checked(archive, &source)
}

/// The source archive from the download cache that `PEPPY_DOWNLOAD_CACHE` names, when the
/// cache holds it with its digest; or else downloaded, and stored in that cache in place
/// of a file of another digest. A cache that cannot take the archive leaves a warning,
/// and the build goes on.
fn cached_or_downloaded_archive() -> Vec<u8> {
    let Some(cache) = download_cache_directory() else {
        return download_archive();
    };
    if let Some(archive) = download_cache::cached(&cache, OMPL_ARCHIVE_SHA256, OMPL_ARCHIVE_BYTES) {
        return archive;
    }
    let archive = download_archive();
    if let Err(error) = download_cache::store(&cache, OMPL_ARCHIVE_SHA256, &archive) {
        println!(
            "cargo:warning=the OMPL source archive is not stored in the download cache {}: \
             {error}",
            cache.display()
        );
    }
    archive
}

/// The directory that `PEPPY_DOWNLOAD_CACHE` names, when it names an absolute path. A
/// relative path leaves a warning and no cache.
fn download_cache_directory() -> Option<PathBuf> {
    let directory = PathBuf::from(env::var_os(DOWNLOAD_CACHE_VARIABLE)?);
    if directory.as_os_str().is_empty() {
        return None;
    }
    if !directory.is_absolute() {
        println!(
            "cargo:warning={DOWNLOAD_CACHE_VARIABLE} names {}, a relative path, so the build \
             uses no download cache",
            directory.display()
        );
        return None;
    }
    Some(directory)
}

/// The source archive of the release tag, downloaded from GitHub within the download's
/// timeouts and checked against the archive's digest.
fn download_archive() -> Vec<u8> {
    let url = archive_url();
    let agent = ureq::Agent::new_with_config(
        ureq::Agent::config_builder()
            .timeout_connect(Some(DOWNLOAD_CONNECT_TIMEOUT))
            .timeout_global(Some(DOWNLOAD_TIMEOUT))
            .build(),
    );
    let mut response = agent
        .get(&url)
        .call()
        .unwrap_or_else(|error| download_failed(&url, &error));
    let archive = read_archive(response.body_mut().as_reader())
        .unwrap_or_else(|error| download_failed(&url, &error));
    checked(archive, &url)
}

/// Stops the build on a failed download from `url`, and says where the build reads a
/// copy of the archive downloaded by hand.
fn download_failed(url: &str, error: &dyn fmt::Display) -> ! {
    panic!(
        "ompl_bridge downloads OMPL's source archive from {url}, which failed: {error}; \
         download it once, and name it by its absolute path in {ARCHIVE_VARIABLE}, or, in \
         a peppy container build, store it in the download cache that \
         {DOWNLOAD_CACHE_VARIABLE} names, under the name {OMPL_ARCHIVE_SHA256}"
    )
}

/// What `reader` reads, up to the archive's size.
fn read_archive(reader: impl Read) -> io::Result<Vec<u8>> {
    let mut archive = Vec::new();
    reader.take(OMPL_ARCHIVE_BYTES).read_to_end(&mut archive)?;
    Ok(archive)
}

/// `archive`, read from `source`, once it has the archive's digest.
fn checked(archive: Vec<u8>, source: &str) -> Vec<u8> {
    let actual = download_cache::sha256_hex(&archive);
    assert!(
        actual == OMPL_ARCHIVE_SHA256,
        "{source} is not the source archive of OMPL {}: expected SHA-256 digest \
         {OMPL_ARCHIVE_SHA256}, actual {actual}",
        ompl_release()
    );
    archive
}

/// Unpacks the release's `LICENSE` and its `src/ompl`, under the archive's `root`, into
/// `out`.
fn extract(archive: &[u8], root: &str, out: &Path) {
    let root = Path::new(root);
    let license = root.join("LICENSE");
    let library = root.join("src/ompl");
    let mut tarball = tar::Archive::new(GzDecoder::new(archive));
    for entry in tarball.entries().expect("the entries of the archive") {
        let mut entry = entry.expect("an entry of the archive");
        let path = entry.path().expect("the path of an entry").into_owned();
        if path == license || path.starts_with(&library) {
            entry
                .unpack_in(out)
                .expect("an entry of the archive unpacked into OUT_DIR");
        }
    }
}

/// Stops the build with an install hint when the compiler finds no Boost headers or
/// the linker no Boost.Serialization library.
fn probe_boost(out: &Path) {
    let hint = "ompl_bridge needs Boost's headers and Boost.Serialization (Debian and Ubuntu: libboost-serialization-dev)";
    let probe = out.join("boost_probe.cpp");
    fs::write(
        &probe,
        "#include <boost/serialization/version.hpp>\n#include <boost/version.hpp>\n",
    )
    .expect("the Boost probe");
    if let Err(error) = cc::Build::new()
        .cpp(true)
        .file(&probe)
        .cargo_metadata(false)
        .try_compile("boost_probe")
    {
        panic!("{hint}: {error}");
    }
    // The compiler prints a library's path when its linker search path holds it, and
    // the bare name otherwise.
    let compiler = cc::Build::new().cpp(true).get_compiler();
    let libraries = [
        "libboost_serialization.so",
        "libboost_serialization.a",
        "libboost_serialization.dylib",
    ];
    let found = libraries.iter().any(|library| {
        let printed = compiler
            .to_command()
            .arg(format!("-print-file-name={library}"))
            .output()
            .expect("the C++ compiler runs");
        String::from_utf8_lossy(&printed.stdout).trim() != *library
    });
    if !found {
        panic!("{hint}: the linker finds no Boost.Serialization library");
    }
}

/// OMPL's `config.h` in `dir`, as its CMake writes it from the `config.h.in` of the
/// release's `sources` for a build without FLANN, Spot or NumPy.
fn write_config(sources: &Path, dir: &Path) {
    let (major, minor, patch) = OMPL_VERSION;
    let template =
        fs::read_to_string(sources.join("ompl/config.h.in")).expect("the release's config.h.in");
    let config = template
        .replace("@PROJECT_VERSION@", &ompl_release())
        .replace("@PROJECT_VERSION_MAJOR@", &major.to_string())
        .replace("@PROJECT_VERSION_MINOR@", &minor.to_string())
        .replace("@PROJECT_VERSION_PATCH@", &patch.to_string())
        .replace(
            "#cmakedefine01 OMPL_HAVE_FLANN",
            "#define OMPL_HAVE_FLANN 0",
        )
        .replace("#cmakedefine01 OMPL_HAVE_SPOT", "#define OMPL_HAVE_SPOT 0")
        .replace(
            "#cmakedefine01 OMPL_HAVE_NUMPY",
            "#define OMPL_HAVE_NUMPY 0",
        );
    assert!(
        !config.contains('@') && !config.contains("#cmakedefine"),
        "config.h.in holds a variable the build does not set"
    );
    fs::create_dir_all(dir).expect("the generated include directory");
    fs::write(dir.join("config.h"), config).expect("the generated config.h");
}
