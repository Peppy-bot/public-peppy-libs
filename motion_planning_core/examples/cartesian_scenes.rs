//! Computes every path of a Cartesian paths scenario of the ROS reference harness, which it
//! reads on stdin, and writes on stdout this crate's answer to each case, the pairs of the
//! robot's own links its paths check and the step of its lines, as JSON.
//! `ros_reference/tools/draw_cartesian_paths.py` draws the cases, runs it, and writes its
//! answers into the scenario.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;
#[path = "../tests/cartesian_scenes/mod.rs"]
mod cartesian_scenes;

use std::path::Path;

use chain_kinematics::CARTESIAN_STEP_M;
use ros_reference::cartesian_paths::{Ours, Paths};
use serde_json::json;

use arm_scenes::{JOINTS, on_the_arms};
use cartesian_scenes::path_answer;

fn main() {
    let paths: Paths =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let repository = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    let urdf = std::fs::read_to_string(repository.join(&paths.urdf)).expect("the URDF reads");
    let answers = on_the_arms::<JOINTS, _>(
        &paths.chains,
        &urdf,
        &repository.join(&paths.meshes),
        |arms, standing| {
            let ours: Vec<Ours> = paths
                .cases
                .iter()
                .map(|request| path_answer(arms, standing, &paths, request))
                .collect();
            json!({
                "checked_pairs": standing.checked_pairs(),
                "step": CARTESIAN_STEP_M,
                "ours": ours,
            })
        },
    );
    println!("{answers}");
}
