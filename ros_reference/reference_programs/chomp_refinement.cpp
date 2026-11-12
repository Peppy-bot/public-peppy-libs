#include "chomp_refinement.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include "arm_scene.hpp"

namespace ros_reference {
namespace {

// The input path of the scene `s`: the arm's joints at each of its waypoints, at least two,
// the scene's start first.
std::vector<Eigen::VectorXd> input_path(const nlohmann::json& s) {
  const auto& path = s.at("path");
  if (path.size() < 2 || numbers(path.front()) != numbers(s.at("start"))) {
    throw std::runtime_error("a scene's path holds at least two waypoints, the scene's start "
                             "first");
  }
  return points_of(path);
}

// Our refinement of the input path of the scene `s`: the arm's joints at each of its
// waypoints, from the scene's start to the input path's end.
const nlohmann::json& our_refinement(const nlohmann::json& s) {
  const auto& refined = s.at("refined");
  if (refined.size() < 2 || numbers(refined.front()) != numbers(s.at("start")) ||
      numbers(refined.back()) != numbers(s.at("path").back())) {
    throw std::runtime_error("a scene's refinement runs from the scene's start to its path's "
                             "end");
  }
  return refined;
}

// `points`, the arm's joints at each waypoint, as MoveIt's trajectory of the group, each
// waypoint the state `start` with the arm at its joints.
robot_trajectory::RobotTrajectoryPtr trajectory_of(const moveit::core::RobotState& start,
                                                   const moveit::core::JointModelGroup& arm,
                                                   const std::vector<Eigen::VectorXd>& points) {
  auto trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(start.getRobotModel(), arm.getName());
  for (const auto& q : points) {
    moveit::core::RobotState state(start);
    state.setJointGroupPositions(&arm, q);
    state.update();
    trajectory->addSuffixWayPoint(state, 0.0);
  }
  return trajectory;
}

}  // namespace

nlohmann::json chomp_refinement(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = arm_model(scenario, inputs);
  const ArmGroup group = group_of(*model, scenario);
  const auto& arm = *model->getJointModelGroup(group.name);
  const Probing probing = probing_of(scenario, group, *model);
  const double voxel_size = scenario.at("voxel_size").get<double>();
  // MoveIt's collision check keeps the scenario's clearance as the padding of the robot's
  // links.
  const double padding = scenario.at("min_clearance").get<double>();

  auto answers = nlohmann::json::array();
  for (const auto& s : scenario.at("scenes")) {
    const auto scene = scene_of(model, s, voxel_size, group, padding);
    const moveit::core::RobotState start(scene->getCurrentState());
    const auto points = input_path(s);
    const auto refined =
        chomp_refined(*scene, trajectory_of(start, arm, points), s.at("seed").get<std::uint32_t>());
    moveit::core::RobotState state(start);
    nlohmann::json answer = {
        {"status", refined.status},
        {"waypoints", nullptr},
        {"refined", nullptr},
        {"input", polyline_measured(*scene, state, arm, points, probing)},
        {"ours", ours_probed(*scene, model, our_refinement(s), probing)},
    };
    if (refined.path) {
      auto waypoints = nlohmann::json::array();
      for (const auto& q : *refined.path) {
        waypoints.push_back(std::vector<double>(q.data(), q.data() + q.size()));
      }
      answer["waypoints"] = waypoints;
      answer["refined"] = polyline_measured(*scene, state, arm, *refined.path, probing);
    }
    answers.push_back(answer);
  }
  return {{"kind", "chomp_refinement"}, {"scenes", answers}};
}

}  // namespace ros_reference
