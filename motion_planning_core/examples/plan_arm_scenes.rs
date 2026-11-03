//! Plans every scene of an arm planning scenario of the ROS reference harness, which it
//! reads on stdin, and writes on stdout this crate's answer to each scene and the spacing
//! at which the reference probes a path, as JSON. `ros_reference/tools/draw_arm_planning.py`
//! draws the scenes, runs it, and writes its answers into the scenario.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;

use std::path::Path;

use motion_planning_core::IkGoalSampler;
use ros_reference::arm_planning::Scenes;
use serde_json::json;

use arm_scenes::{on_the_arm, plan_scene, probe_spacing};

fn main() {
    let scenes: Scenes =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let repository = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    let urdf = std::fs::read_to_string(repository.join(&scenes.urdf)).expect("the URDF reads");
    let answers = on_the_arm(&scenes, &urdf, &repository.join(&scenes.meshes), |arm| {
        let sampler = IkGoalSampler::new(arm.geometry().chain().clone());
        let ours: Vec<_> = scenes
            .scenes
            .iter()
            .map(|scene| plan_scene(arm, &sampler, &scenes, scene))
            .collect();
        json!({
            "probe_spacing": probe_spacing(arm, &scenes),
            "ours": ours,
        })
    });
    println!("{answers}");
}
