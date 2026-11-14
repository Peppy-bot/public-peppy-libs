//! Computes every path of a Cartesian paths scenario of the ROS reference harness, which it
//! reads on stdin, and writes on stdout this crate's answer to each case and the step of its
//! lines, as JSON.
//! `ros_reference/tools/draw_cartesian_paths.py` draws the cases, runs it, and writes its
//! answers into the scenario.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;
#[path = "../tests/cartesian_scenes/mod.rs"]
mod cartesian_scenes;
#[path = "../tests/openarm_v2/mod.rs"]
mod openarm_v2;

use chain_kinematics::CARTESIAN_STEP_M;
use ros_reference::Robot;
use ros_reference::cartesian_paths::{Ours, Paths};
use serde_json::json;

use arm_scenes::on_the_arms;
use cartesian_scenes::path_answer;
use openarm_v2::{ARMS, JOINTS};

fn main() {
    let paths: Paths =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let robot = Robot::of(&paths.description);
    let arm = robot.arm("the scenario", &paths.group);
    let answers =
        on_the_arms::<JOINTS, ARMS, _>(std::slice::from_ref(&arm), &robot, |arms, standing| {
            let ours: Vec<Ours> = paths
                .cases
                .iter()
                .map(|request| path_answer(arms, standing, &paths, request))
                .collect();
            json!({
                "step": CARTESIAN_STEP_M,
                "ours": ours,
            })
        });
    println!("{answers}");
}
