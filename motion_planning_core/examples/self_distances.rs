//! Reads an arm planning scenario of both arms of the ROS reference harness on stdin, and
//! writes on stdout, as JSON, the least distance between the robot's own bodies in each of
//! its scenes, as this crate's self model of a plan of both arms measures it: at the
//! scene's start, at its goal, and along the straight joint line between them.
//! `ros_reference/tools/draw_arm_planning.py --both-arms` runs it and keeps the scenes whose
//! start it holds apart and whose goal it holds at least the scenario's `min_clearance`
//! apart, and the committed scene whose straight joint line runs through the robot's own
//! body was picked by its least distance along the line.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;

use std::path::Path;

use ros_reference::arm_planning::Scenes;
use serde_json::json;

use arm_scenes::{joint_goal, least_self_clearance_along, on_the_arms};

fn main() {
    let scenes: Scenes =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let repository = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    let urdf = std::fs::read_to_string(repository.join(&scenes.urdf)).expect("the URDF reads");
    let distances = on_the_arms(
        &scenes.chains,
        &urdf,
        &repository.join(&scenes.meshes),
        |_, standing| {
            scenes
                .scenes
                .iter()
                .map(|scene| {
                    let goal = joint_goal(&scene.goal)
                        .expect("a scene the drawing tool measures reaches for a configuration");
                    json!({
                        "start": standing.self_clearance(&scene.start),
                        "goal": standing.self_clearance(goal),
                        "line": least_self_clearance_along(standing, &scene.start, goal),
                    })
                })
                .collect::<Vec<_>>()
        },
    );
    println!("{}", json!(distances));
}
