#include "cartesian_paths.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "arm_scene.hpp"
#include "robot.hpp"

namespace ros_reference {
namespace {

// The pose `offset` names from `from`: its translation added in the model frame, its
// rotation vector applied in `from`'s frame.
Eigen::Isometry3d moved_by(const Eigen::Isometry3d& from, const nlohmann::json& offset) {
  const auto translation = numbers(offset.at("translation"));
  const auto rotation = numbers(offset.at("rotation"));
  const Eigen::Vector3d turn(rotation.at(0), rotation.at(1), rotation.at(2));
  Eigen::Isometry3d pose = from;
  pose.translation() += Eigen::Vector3d(translation.at(0), translation.at(1), translation.at(2));
  const double angle = turn.norm();
  if (angle > 0.0) {
    pose.linear() = from.linear() * Eigen::AngleAxisd(angle, turn / angle).toRotationMatrix();
  }
  return pose;
}

// The state validity check of move_group's compute_cartesian_path service where a request
// avoids collisions (moveit2 2.15.2 cartesian_path_service_capability.cpp:59-67), and none
// where it does not (:160-175).
moveit::core::GroupStateValidityCallbackFn state_check(const planning_scene::PlanningScene& scene,
                                                       bool avoid_collisions) {
  if (!avoid_collisions) {
    return {};
  }
  return [&scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* joints,
                  const double* positions) {
    state->setJointGroupPositions(joints, positions);
    state->update();
    return !scene.isStateColliding(*state, joints->getName());
  };
}

// MoveIt's Cartesian path of the case `path` in `scene`, the arm at the scene's state: as the
// service calls the interpolator, in the model frame, at CartesianPrecision's defaults
// (cartesian_path_service_capability.cpp:186-188), the case seeding KDL's restarts, which
// draw from a generator of MoveIt's, and its steps off a singularity.
nlohmann::json moveit_path(const moveit::core::RobotModel& model,
                           const planning_scene::PlanningScene& scene, const ArmGroup& group,
                           const nlohmann::json& path, double step) {
  const auto* arm = model.getJointModelGroup(group.name);
  const auto link_name = path.at("link").get<std::string>();
  const auto* link = model.getLinkModel(link_name);
  moveit::core::RobotState start(scene.getCurrentState());
  const Eigen::Isometry3d from = start.getGlobalLinkTransform(link);
  EigenSTL::vector_Isometry3d waypoints;
  for (const auto& offset : path.at("offsets")) {
    waypoints.push_back(moved_by(from, offset));
  }
  seed_moveit(path.at("seed").get<std::uint32_t>());
  std::vector<moveit::core::RobotStatePtr> states;
  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &start, arm, states, link, waypoints, true, moveit::core::MaxEEFStep(step),
      moveit::core::CartesianPrecision{},
      state_check(scene, path.at("avoid_collisions").get<bool>()));

  auto poses = nlohmann::json::array();
  auto joints = nlohmann::json::array();
  for (const auto& state : states) {
    state->update();
    poses.push_back(link_pose(*state, link_name));
    std::vector<double> positions;
    state->copyJointGroupPositions(arm, positions);
    joints.push_back(positions);
  }
  return {{"fraction", fraction}, {"poses", poses}, {"states", joints}};
}

}  // namespace

nlohmann::json cartesian_paths(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = arm_model(scenario, inputs);
  const ArmGroup group = group_of(*model, scenario);
  const double voxel_size = scenario.at("voxel_size").get<double>();
  // MoveIt keeps the paths' clearance as the padding of the robot's links.
  const double padding = scenario.at("min_clearance").get<double>();
  const double step = scenario.at("step").get<double>();

  auto answers = nlohmann::json::array();
  for (const auto& path : scenario.at("cases")) {
    // Each case loads MoveIt's inverse kinematics as a solver of its own: its random restarts
    // draw from a generator that no case before it drew from.
    load_kdl(model, group.solver_groups());
    const auto scene = scene_of(model, path, voxel_size, group, padding);
    answers.push_back(moveit_path(*model, *scene, group, path, step));
  }
  return {{"kind", "cartesian_paths"}, {"cases", answers}};
}

}  // namespace ros_reference
