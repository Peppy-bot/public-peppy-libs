// The constraint sampler allocator that the arm planning reference loads into MoveIt's OMPL
// planner through its constraint_samplers parameter (moveit2 2.15.2
// constraint_sampler_manager_loader.cpp:55-86), for a goal or a path of poses of a group
// that holds no inverse kinematics of its own, both of OpenArm v2's arms. It builds what
// ConstraintSamplerManager::selectDefaultSampler builds for such a group, the union of each
// subgroup's default sampler (constraint_sampler_manager.cpp:322-383), and builds the
// subgroups' samplers in the order of the subgroups' names. selectDefaultSampler walks them
// in the order of their JointModelGroups' addresses (joint_model_group.hpp:106), which
// follows the heap's layout, and each sampler constructs its random generator when it is
// built, which the reference seeds in the order of construction.

#include <algorithm>
#include <cstddef>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.hpp>
#include <moveit/constraint_samplers/union_constraint_sampler.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ros_reference {
namespace {

// The constraints of `constraints` on links of `subgroup` that no subgroup before it took,
// each of them added to `taken` (constraint_sampler_manager.cpp:336-359).
template <class Constraint>
std::vector<Constraint> taken_by(const moveit::core::JointModelGroup& subgroup,
                                 const std::vector<Constraint>& constraints,
                                 std::set<std::size_t>& taken) {
  std::vector<Constraint> of_subgroup;
  for (std::size_t k = 0; k < constraints.size(); ++k) {
    if (subgroup.hasLinkModel(constraints[k].link_name) && taken.insert(k).second) {
      of_subgroup.push_back(constraints[k]);
    }
  }
  return of_subgroup;
}

}  // namespace

class UnionInNameOrder : public constraint_samplers::ConstraintSamplerAllocator {
 public:
  bool canService(const planning_scene::PlanningSceneConstPtr& scene,
                  const std::string& group_name,
                  const moveit_msgs::msg::Constraints& constraints) const override {
    const auto* group = scene->getRobotModel()->getJointModelGroup(group_name);
    if (group == nullptr) {
      return false;
    }
    const auto& [solver, subgroup_solvers] = group->getGroupKinematics();
    const bool names_poses = !constraints.position_constraints.empty() ||
                             !constraints.orientation_constraints.empty();
    return !solver && !subgroup_solvers.empty() && names_poses;
  }

  constraint_samplers::ConstraintSamplerPtr alloc(
      const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
      const moveit_msgs::msg::Constraints& constraints) override {
    if (!constraints.joint_constraints.empty()) {
      throw std::runtime_error("the arm planning reference samples no constraints of " +
                               group_name + " that name both joints and poses");
    }
    const auto& group = *scene->getRobotModel()->getJointModelGroup(group_name);
    std::vector<const moveit::core::JointModelGroup*> subgroups;
    for (const auto& [subgroup, solver] : group.getGroupKinematics().second) {
      subgroups.push_back(subgroup);
    }
    std::ranges::sort(subgroups, {}, &moveit::core::JointModelGroup::getName);
    std::set<std::size_t> taken_positions;
    std::set<std::size_t> taken_orientations;
    std::vector<constraint_samplers::ConstraintSamplerPtr> samplers;
    for (const auto* subgroup : subgroups) {
      moveit_msgs::msg::Constraints of_subgroup;
      of_subgroup.position_constraints =
          taken_by(*subgroup, constraints.position_constraints, taken_positions);
      of_subgroup.orientation_constraints =
          taken_by(*subgroup, constraints.orientation_constraints, taken_orientations);
      if (of_subgroup.position_constraints.empty() &&
          of_subgroup.orientation_constraints.empty()) {
        continue;
      }
      auto sampler = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(
          scene, subgroup->getName(), of_subgroup);
      if (sampler) {
        samplers.push_back(std::move(sampler));
      }
    }
    if (samplers.empty()) {
      return nullptr;
    }
    return std::make_shared<constraint_samplers::UnionConstraintSampler>(scene, group_name,
                                                                         samplers);
  }
};

}  // namespace ros_reference

PLUGINLIB_EXPORT_CLASS(ros_reference::UnionInNameOrder,
                       constraint_samplers::ConstraintSamplerAllocator)
