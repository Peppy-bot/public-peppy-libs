//! A fit prints nothing to stdout or stderr: Qhull writes its report of each hull, the
//! check of the points and the report of a failed pass, to memory. A test binary of its
//! own, with no test harness, which runs the fits in a child process of itself and reads
//! what the child prints.

use std::process::Command;

use chain_kinematics::{ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use collision_geometry::hull::exact_hull;
use collision_geometry::nalgebra::Point3;

/// The environment variable under which this binary runs the fits.
const FITS: &str = "COLLISION_GEOMETRY_RUN_THE_FITS";

fn main() {
    if std::env::var_os(FITS).is_some() {
        run_the_fits();
        return;
    }
    let child = Command::new(std::env::current_exe().expect("the test binary has a path"))
        .env(FITS, "1")
        .output()
        .expect("the test binary runs as a child");
    let (stdout, stderr) = (
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr),
    );
    assert!(
        child.status.success(),
        "the fits failed ({}):\n{stderr}",
        child.status
    );
    assert!(
        stdout.is_empty() && stderr.is_empty(),
        "the fits printed\nto stdout:\n{stdout}\nto stderr:\n{stderr}"
    );
}

/// A hull that Qhull's first pass builds and checks, a cloud that it cannot hull, and the
/// pieces of every link of an arm.
fn run_the_fits() {
    let cube: Vec<Point3<f64>> = (0..8)
        .map(|i| Point3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64))
        .collect();
    let flat: Vec<Point3<f64>> = (0..9)
        .map(|i| Point3::new((i % 3) as f64, (i / 3) as f64, 0.0))
        .collect();
    assert_eq!(exact_hull(&cube).expect("a cube hulls").vertices.len(), 8);
    exact_hull(&flat).expect_err("a flat cloud has no hull");
    // The first arm of OpenArm v2, whose links the child fits.
    let model =
        robot_description::model("openarm_v2").expect("robot_description bundles OpenArm v2");
    let meshes = model.directory().join("meshes");
    let description = model.description();
    let first = &description.arms()[0];
    let joints: Vec<&str> = first.joints.iter().map(String::as_str).collect();
    let arm = ChainGeometry::<7>::from_urdf(
        description.urdf(),
        meshes.to_str().expect("the model's directory is UTF-8"),
        &ChainSpec {
            base_link: None,
            tip_link: &first.grasp_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .expect("the arm's geometry fits");
    assert!(!arm.links().is_empty(), "the arm carries a link");
}
