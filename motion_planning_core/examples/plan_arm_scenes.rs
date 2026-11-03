//! Plans every scene of an arm planning scenario of the ROS reference harness, which it
//! reads on stdin, and writes on stdout this crate's answer to each scene and the spacing
//! at which the reference probes a path, as JSON, and on stderr how many of its plans that
//! found no path ended at the planning time and how many before it.
//! `ros_reference/tools/draw_arm_planning.py` draws the scenes, runs it, and writes its
//! answers into the scenario.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;

use std::path::Path;

use motion_planning_core::IkGoalSampler;
use ros_reference::arm_planning::{Ours, Scenes};
use serde_json::json;

use arm_scenes::{Planned, on_the_arm, plan_scene, probe_spacing};

fn main() {
    let scenes: Scenes =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let repository = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    let urdf = std::fs::read_to_string(repository.join(&scenes.urdf)).expect("the URDF reads");
    let answers = on_the_arm(
        &scenes,
        &urdf,
        &repository.join(&scenes.meshes),
        |arm, standing| {
            let sampler = IkGoalSampler::new(arm.geometry().chain().clone());
            let planned: Vec<Planned> = scenes
                .scenes
                .iter()
                .map(|scene| plan_scene(arm, standing, &sampler, &scenes, scene))
                .collect();
            let no_path: Vec<&Planned> = planned
                .iter()
                .filter(|p| matches!(&p.ours, Ours::Failure { failure } if failure == "no_path"))
                .collect();
            let at_the_deadline = no_path.iter().filter(|p| p.at_the_deadline).count();
            eprintln!(
                "{} plans found no path: {at_the_deadline} ended at the planning time, {} \
                 before it",
                no_path.len(),
                no_path.len() - at_the_deadline
            );
            let ours: Vec<Ours> = planned.into_iter().map(|p| p.ours).collect();
            json!({
                "probe_spacing": probe_spacing(arm, &scenes),
                "ours": ours,
            })
        },
    );
    println!("{answers}");
}
