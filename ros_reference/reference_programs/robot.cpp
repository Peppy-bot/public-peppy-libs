#include "robot.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>

#include <srdfdom/model.h>
#include <urdf/model.hpp>

namespace ros_reference {
namespace {

std::shared_ptr<urdf::Model> parsed(const std::string& urdf_text) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(urdf_text)) {
    throw std::runtime_error("the URDF does not parse");
  }
  return urdf;
}

moveit::core::RobotModelPtr model_of(const std::shared_ptr<urdf::Model>& urdf) {
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, "<robot name=\"" + urdf->getName() + "\"/>")) {
    throw std::runtime_error("the empty SRDF does not parse");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

}  // namespace

moveit::core::RobotModelPtr robot_model(const std::string& urdf_text) {
  return model_of(parsed(urdf_text));
}

moveit::core::RobotModelPtr robot_model_with_meshes(
    Inputs& inputs, const std::string& urdf,
    const std::optional<std::string>& mesh_directory) {
  auto model = parsed(inputs.read(urdf));
  for (const auto& [name, link] : model->links_) {
    for (const auto& collision : link->collision_array) {
      if (!collision->geometry || collision->geometry->type != urdf::Geometry::MESH) {
        continue;
      }
      auto& mesh = static_cast<urdf::Mesh&>(*collision->geometry);
      if (!mesh_directory) {
        throw std::runtime_error("the link " + name + " names the collision mesh " +
                                 mesh.filename + ", and the scenario names no mesh directory");
      }
      const auto file = mesh.filename.substr(mesh.filename.rfind('/') + 1);
      mesh.filename = inputs.file_uri(*mesh_directory + "/" + file);
    }
  }
  return model_of(model);
}

std::string child_link(const moveit::core::RobotModel& model, const std::string& joint) {
  const auto* joint_model = model.getJointModel(joint);
  if (joint_model == nullptr) {
    throw std::runtime_error("the URDF has no joint " + joint);
  }
  return joint_model->getChildLinkModel()->getName();
}

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

}  // namespace ros_reference
