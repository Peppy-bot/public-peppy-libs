#include "forward_kinematics.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/robot_state/robot_state.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

nlohmann::json forward_kinematics(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = description_model(inputs, scenario.at("description").get<std::string>());
  moveit::core::RobotState state(model);
  auto arms = nlohmann::json::array();
  for (const auto& posed : scenario.at("arms")) {
    const auto arm = the_arm(*model, posed.at("name").get<std::string>());
    std::vector<std::string> links;
    for (const auto& joint : arm.joints) {
      links.push_back(child_link(*model, joint));
    }
    require_named_path(*model, arm.tip_link, arm.joints);
    links.push_back(arm.tip_link);
    auto configurations = nlohmann::json::array();
    for (const auto& configuration : posed.at("configurations")) {
      const auto positions = configuration.get<std::vector<double>>();
      if (positions.size() != arm.joints.size()) {
        throw std::runtime_error("a configuration of " + arm.name +
                                 " does not hold one position per joint");
      }
      state.setToDefaultValues();
      for (std::size_t i = 0; i < arm.joints.size(); ++i) {
        state.setVariablePosition(arm.joints[i], positions[i]);
      }
      state.update();
      auto poses = nlohmann::json::array();
      for (const auto& link : links) {
        poses.push_back(link_pose(state, link));
      }
      configurations.push_back(poses);
    }
    arms.push_back({{"name", arm.name}, {"configurations", configurations}});
  }
  return {{"kind", "forward_kinematics"}, {"arms", arms}};
}

}  // namespace ros_reference
