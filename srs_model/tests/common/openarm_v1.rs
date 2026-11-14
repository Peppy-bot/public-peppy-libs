//! The fixture arm every suite checks against: the OpenArm v1.0 as robot_description
//! bundles it, gripper fingers included, so the distal-payload path is exercised by
//! loading it. `side` is `"left"` or `"right"`, the side of the arm group `<side>_arm`.
//! The unit tests include this file as `openarm_v1` under `cfg(test)`, the integration
//! tests reach it as `common::openarm_v1`, and the benchmarks include it too; each
//! includer reads some of it.
#![allow(dead_code)]

use std::sync::LazyLock;

use robot_description::{Model, RobotDescription};

/// The fixture's bundled model.
fn model() -> &'static Model {
    robot_description::model("openarm_v1").expect("robot_description bundles the fixture")
}

/// The path of the fixture's URDF, for a reader that loads a file.
pub fn urdf_path() -> String {
    let model = model();
    model
        .directory()
        .join(format!("{}.urdf", model.id()))
        .to_str()
        .expect("the model's directory is UTF-8")
        .to_owned()
}

/// The fixture's description, read once per test binary.
fn description() -> &'static RobotDescription {
    static DESCRIPTION: LazyLock<RobotDescription> = LazyLock::new(|| model().description());
    &DESCRIPTION
}

/// The fixture's URDF.
pub fn urdf() -> &'static str {
    description().urdf()
}

/// The link where the fixture's 7-DOF chain on `side` starts: its arm group's base link.
pub fn base(side: &str) -> &'static str {
    let group = format!("{side}_arm");
    &description()
        .arms()
        .iter()
        .find(|arm| arm.name == group)
        .unwrap_or_else(|| panic!("the fixture's arms are \"left\" and \"right\", not {side:?}"))
        .base_link
}
