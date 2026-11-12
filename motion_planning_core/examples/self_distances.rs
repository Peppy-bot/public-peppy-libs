//! Reads an arm planning scenario of the ROS reference harness that plans every arm of its
//! robot on stdin, and writes on stdout, as JSON, the least distance between the robot's own
//! bodies in each of its scenes, as this crate's self model of a plan of every arm measures
//! it: at the scene's start, at its goal, and along the straight joint line
//! between them. `ros_reference/tools/draw_arm_planning.py --both-arms` and
//! `--robot so101` run it and keep the scenes whose start it holds apart and whose goal it
//! holds at least the scenario's `min_clearance` apart, and the committed scene whose
//! straight joint line runs through the robot's own body was picked by its least distance
//! along the line.

#[path = "../tests/arm_scenes/mod.rs"]
mod arm_scenes;

use robot_description::{OnShape, on_shape};
use ros_reference::Robot;
use ros_reference::arm_planning::Scenes;
use serde_json::json;

use arm_scenes::{Standing, joint_goal, least_self_clearance_along};

/// The least distances of each scene of `scenes`, a scenario of `robot`.
struct Distances<'a> {
    scenes: &'a Scenes,
    robot: &'a Robot,
}

impl OnShape for Distances<'_> {
    type Output = Vec<serde_json::Value>;

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) -> Self::Output {
        let Self { scenes, robot } = self;
        let standing =
            Standing::<N, K>::new(robot, &robot.group_arms("the scenario", &scenes.group));
        scenes
            .scenes
            .iter()
            .map(|scene| {
                let goal = joint_goal(&scene.goal)
                    .expect("a scene the drawing tool measures reaches for a configuration");
                json!({
                    "start": standing.self_clearance(&scene.start),
                    "goal": standing.self_clearance(goal),
                    "line": least_self_clearance_along(&standing, &scene.start, goal),
                })
            })
            .collect()
    }
}

fn main() {
    let scenes: Scenes =
        serde_json::from_reader(std::io::stdin()).expect("the scenario on stdin parses");
    let robot = Robot::of(&scenes.description);
    let distances = on_shape(
        &robot.description,
        Distances {
            scenes: &scenes,
            robot: &robot,
        },
    )
    .unwrap_or_else(|error| panic!("{}: {error}", scenes.description));
    println!("{}", json!(distances));
}
