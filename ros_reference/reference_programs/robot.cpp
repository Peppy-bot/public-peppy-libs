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

// The arms of `srdf`: each group of one chain, and nothing else, whose tip an end effector
// stands on.
std::vector<const srdf::Model::Group*> arms_of(const srdf::Model& srdf) {
  std::vector<const srdf::Model::Group*> arms;
  for (const auto& group : srdf.getGroups()) {
    if (group.chains_.size() != 1 || !group.joints_.empty() || !group.links_.empty() ||
        !group.subgroups_.empty()) {
      continue;
    }
    const auto& tip = group.chains_.front().second;
    const auto& end_effectors = srdf.getEndEffectors();
    if (std::any_of(end_effectors.begin(), end_effectors.end(),
                    [&tip](const auto& end_effector) { return end_effector.parent_link_ == tip; })) {
      arms.push_back(&group);
    }
  }
  return arms;
}

Arm arm_of(const moveit::core::RobotModel& model, const srdf::Model::Group& group) {
  return {group.name_, model.getJointModelGroup(group.name_)->getActiveJointModelNames(),
          group.chains_.front().second};
}

}  // namespace

moveit::core::RobotModelPtr description_model(Inputs& inputs, const std::string& description) {
  const auto name = description.substr(description.rfind('/') + 1);
  auto urdf = parsed(inputs.read(description + "/" + name + ".urdf"));
  for (const auto& [link_name, link] : urdf->links_) {
    for (const auto& collision : link->collision_array) {
      if (!collision->geometry || collision->geometry->type != urdf::Geometry::MESH) {
        continue;
      }
      auto& mesh = static_cast<urdf::Mesh&>(*collision->geometry);
      const auto file = mesh.filename.substr(mesh.filename.rfind('/') + 1);
      mesh.filename = inputs.file_uri(description + "/meshes/" + file);
    }
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, inputs.read(description + "/" + name + ".srdf"))) {
    throw std::runtime_error("the SRDF of " + description + " does not parse");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

std::vector<std::string> ArmGroup::solver_groups() const {
  std::vector<std::string> names;
  for (const auto& arm : arms) {
    names.push_back(arm.name);
  }
  return names;
}

ArmGroup arm_group(const moveit::core::RobotModel& model, const std::string& name) {
  const auto& srdf = *model.getSRDF();
  const auto arms = arms_of(srdf);
  const auto& groups = srdf.getGroups();
  const auto group = std::find_if(groups.begin(), groups.end(),
                                  [&name](const auto& candidate) { return candidate.name_ == name; });
  if (group == groups.end()) {
    throw std::runtime_error("the SRDF has no group " + name);
  }
  const auto arm_named = [&arms](const std::string& arm) {
    return std::find_if(arms.begin(), arms.end(),
                        [&arm](const auto* candidate) { return candidate->name_ == arm; });
  };
  ArmGroup held{name, {}, false};
  if (arm_named(name) != arms.end()) {
    held.arms.push_back(arm_of(model, *group));
  } else {
    if (group->subgroups_.empty() || !group->chains_.empty() || !group->joints_.empty() ||
        !group->links_.empty()) {
      throw std::runtime_error("the group " + name + " is neither an arm nor a group of arms");
    }
    for (const auto& subgroup : group->subgroups_) {
      const auto arm = arm_named(subgroup);
      if (arm == arms.end()) {
        throw std::runtime_error("the group " + name + " holds " + subgroup + ", which is no arm");
      }
      held.arms.push_back(arm_of(model, **arm));
    }
  }
  held.holds_every_arm = held.arms.size() == arms.size();
  return held;
}

Arm the_arm(const moveit::core::RobotModel& model, const std::string& name) {
  auto group = arm_group(model, name);
  if (group.arms.size() != 1 || group.arms.front().name != name) {
    throw std::runtime_error("the group " + name + " is no arm");
  }
  return group.arms.front();
}

bool disabled_pair(const moveit::core::RobotModel& model, const std::string& a,
                   const std::string& b) {
  const auto& pairs = model.getSRDF()->getDisabledCollisionPairs();
  return std::any_of(pairs.begin(), pairs.end(), [&a, &b](const auto& pair) {
    return (pair.link1_ == a && pair.link2_ == b) || (pair.link1_ == b && pair.link2_ == a);
  });
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
    translation.push_back(transform.translation()(i));
  }
  auto rotation = nlohmann::json::array();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation.push_back(transform.linear()(row, column));
    }
  }
  return {{"link", link}, {"translation", translation}, {"rotation", rotation}};
}

}  // namespace ros_reference
