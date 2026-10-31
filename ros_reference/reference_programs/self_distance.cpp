#include "self_distance.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

nlohmann::json self_distance(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = robot_model_with_meshes(inputs, scenario.at("urdf").get<std::string>(),
                                             scenario.at("meshes").get<std::string>());
  collision_detection::CollisionEnvFCL environment(model);
  // An empty matrix allows no pair, so every pair of links is measured.
  const collision_detection::AllowedCollisionMatrix allowed;
  moveit::core::RobotState state(model);
  std::vector<std::pair<std::string, std::string>> pairs;
  auto configurations = nlohmann::json::array();
  for (const auto& configuration : scenario.at("configurations")) {
    state.setToDefaultValues();
    auto echoed_positions = nlohmann::json::object();
    for (const auto& [joint, position] : configuration.items()) {
      state.setVariablePosition(joint, position.get<double>());
      echoed_positions[joint] = position.get<double>();
    }
    state.update();
    collision_detection::DistanceRequest request;
    request.type = collision_detection::DistanceRequestType::SINGLE;
    request.enable_signed_distance = true;
    request.acm = &allowed;
    collision_detection::DistanceResult result;
    environment.distanceSelf(request, result, state);
    std::vector<std::pair<std::string, std::string>> measured;
    auto nearest = nlohmann::json::array();
    for (const auto& [links, distances] : result.distances) {
      measured.push_back(links);
      double distance = std::numeric_limits<double>::infinity();
      for (const auto& data : distances) {
        distance = std::min(distance, data.distance);
      }
      nearest.push_back(distance);
    }
    if (configurations.empty()) {
      pairs = measured;
    } else if (measured != pairs) {
      throw std::runtime_error("FCL measured another set of pairs at another configuration");
    }
    configurations.push_back({{"positions", echoed_positions}, {"distances", nearest}});
  }
  auto names = nlohmann::json::array();
  for (const auto& [a, b] : pairs) {
    names.push_back({a, b});
  }
  return {{"kind", "self_distance"}, {"pairs", names}, {"configurations", configurations}};
}

}  // namespace ros_reference
