#include "forward_kinematics.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <srdfdom/model.h>
#include <urdf/model.hpp>

#include "scenario.hpp"

namespace ros_reference {
namespace {

// The robot model of a URDF, with an SRDF that names no group and no virtual joint, so
// the URDF root stands fixed at the origin of the model frame.
moveit::core::RobotModelPtr robot_model(const std::string& urdf_text) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(urdf_text)) {
    throw std::runtime_error("the URDF does not parse");
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, "<robot name=\"" + urdf->getName() + "\"/>")) {
    throw std::runtime_error("the empty SRDF does not parse");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

std::string child_link(const moveit::core::RobotModel& model, const std::string& joint) {
  const auto* joint_model = model.getJointModel(joint);
  if (joint_model == nullptr) {
    throw std::runtime_error("the URDF has no joint " + joint);
  }
  return joint_model->getChildLinkModel()->getName();
}

// Refuses a chain whose path from the URDF root to `tip_link` holds a movable joint that
// `joints` does not name: MoveIt stands such a joint at its default position, which the
// parity test does not know.
void require_named_path(const moveit::core::RobotModel& model, const std::string& tip_link,
                        const std::vector<std::string>& joints) {
  const auto* link = model.getLinkModel(tip_link);
  if (link == nullptr) {
    throw std::runtime_error("the URDF has no link " + tip_link);
  }
  for (const auto* joint = link->getParentJointModel(); joint != nullptr;
       joint = joint->getParentLinkModel() == nullptr
                   ? nullptr
                   : joint->getParentLinkModel()->getParentJointModel()) {
    const bool movable = joint->getType() != moveit::core::JointModel::FIXED;
    const bool named = std::find(joints.begin(), joints.end(), joint->getName()) != joints.end();
    if (movable && !named) {
      throw std::runtime_error("the path to " + tip_link + " holds the movable joint " +
                               joint->getName() + ", which the chain does not name");
    }
  }
}

nlohmann::json link_pose(const moveit::core::RobotState& state, const std::string& link) {
  const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(link);
  auto translation = nlohmann::json::array();
  for (int i = 0; i < 3; ++i) {
    translation.push_back(bits(transform.translation()(i)));
  }
  auto rotation = nlohmann::json::array();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation.push_back(bits(transform.linear()(row, column)));
    }
  }
  return {{"link", link}, {"translation", translation}, {"rotation", rotation}};
}

}  // namespace

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
