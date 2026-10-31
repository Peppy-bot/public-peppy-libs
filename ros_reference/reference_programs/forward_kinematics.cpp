#include "forward_kinematics.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/robot_state/robot_state.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

nlohmann::json forward_kinematics(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = robot_model(inputs.read(scenario.at("urdf").get<std::string>()));
  moveit::core::RobotState state(model);
  auto chains = nlohmann::json::array();
  for (const auto& chain : scenario.at("chains")) {
    const auto joints = chain.at("joints").get<std::vector<std::string>>();
    std::vector<std::string> links;
    for (const auto& joint : joints) {
      links.push_back(child_link(*model, joint));
    }
    const auto tip_link = chain.at("tip_link").get<std::string>();
    require_named_path(*model, tip_link, joints);
    links.push_back(tip_link);
    auto configurations = nlohmann::json::array();
    for (const auto& configuration : chain.at("configurations")) {
      const auto positions = configuration.get<std::vector<double>>();
      if (positions.size() != joints.size()) {
        throw std::runtime_error("a configuration of " + chain.at("name").get<std::string>() +
                                 " does not hold one position per joint");
      }
      state.setToDefaultValues();
      for (std::size_t i = 0; i < joints.size(); ++i) {
        state.setVariablePosition(joints[i], positions[i]);
      }
      state.update();
      auto poses = nlohmann::json::array();
      for (const auto& link : links) {
        poses.push_back(link_pose(state, link));
      }
      configurations.push_back(poses);
    }
    chains.push_back({{"name", chain.at("name")}, {"configurations", configurations}});
  }
  return {{"kind", "forward_kinematics"}, {"chains", chains}};
}

}  // namespace ros_reference
