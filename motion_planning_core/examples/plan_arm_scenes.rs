//! Plans every scene of an arm planning scenario of the ROS reference harness, which it
//! reads on stdin, and writes on stdout this crate's answer to each scene and the spacing at
//! which the reference probes a path, as JSON, and on stderr how many of its plans that found no path ended at the
//! planning time and how many before it.
//! `ros_reference/tools/draw_arm_planning.py` draws the scenes, runs it, and writes its
//! answers into the scenario.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;

use robot_description::{Arm, OnShape, on_shape};
use ros_reference::Robot;
use ros_reference::arm_planning::{Ours, Scenes};
use serde_json::json;

use arm_scenes::{Planned, on_the_arms, plan_scene, probe_spacing, samplers};

/// The answers to every scene of `scenes`, by `arms`, the arms of `robot` its group holds.
struct Answers<'a> {
    scenes: &'a Scenes,
    robot: &'a Robot,
    arms: &'a [Arm],
}

impl OnShape for Answers<'_> {
    type Output = serde_json::Value;

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> serde_json::Value {
        let Self {
            scenes,
            robot,
            arms: planned,
        } = self;
        on_the_arms::<N, K, _>(planned, robot, |arms, standing| {
            let samplers = samplers(arms);
            let planned: Vec<Planned> = scenes
                .scenes
                .iter()
                .map(|scene| plan_scene::<N, D, K, _>(arms, standing, &samplers, scenes, scene))
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
                "probe_spacing": probe_spacing::<N, D>(arms, scenes),
                "ours": ours,
            })
        })
    }
}

fn main() {
    let scenes: Scenes =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let robot = Robot::of(&scenes.description);
    let arms = robot.group_arms("the scenario", &scenes.group);
    let answers = on_shape(
        &robot.description,
        Answers {
            scenes: &scenes,
            robot: &robot,
            arms: &arms,
        },
    )
    .unwrap_or_else(|error| panic!("{}: {error}", scenes.description));
    println!("{answers}");
}
