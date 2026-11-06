#include "arm_scene.hpp"

#include <algorithm>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <octomap/OcTree.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

#include "planning_scene.hpp"

namespace ros_reference {
namespace {

// The generator of the seeds of MoveIt's own random generators, which a reference reseeds
// from each scene's seed before it runs MoveIt on the scene, and the lock under which a
// generator of MoveIt's, on the planner's thread or the goal sampler's, draws its seed.
std::mt19937& moveit_seeds() {
  static std::mt19937 seeds;
  return seeds;
}

std::mutex& moveit_seeds_lock() {
  static std::mutex lock;
  return lock;
}

}  // namespace

std::atomic<std::uint64_t>& moveit_generators() {
  static std::atomic<std::uint64_t> built{0};
  return built;
}

}  // namespace ros_reference

// MoveIt's state sampler and its constraint samplers each draw from a
// random_numbers::RandomNumberGenerator that they construct by its default constructor,
// which random_numbers seeds from the clock. The reference program defines that
// constructor, and its link exports it (CMakeLists.txt), so MoveIt's libraries construct
// every such generator through it, seeded from `moveit_seeds`, and a plan repeats.
random_numbers::RandomNumberGenerator::RandomNumberGenerator()
    : RandomNumberGenerator([] {
        const std::lock_guard<std::mutex> held(ros_reference::moveit_seeds_lock());
        return static_cast<std::uint32_t>(ros_reference::moveit_seeds()());
      }()) {
  ++ros_reference::moveit_generators();
}

namespace ros_reference {
namespace {

// OctoMap's key of the voxel at the origin of each axis.
constexpr int tree_max_val = 32768;
// The acceleration bound of each joint of the arm, in rad/s^2, for the pipeline's time
// parameterization: it times the path and leaves its shape to the path tolerance.
constexpr double max_acceleration = 5.0;

// Whether `link` moves with the group's joints: one of them lies on its way up to the
// root, as the hand's fingers hang below the hand.
bool moves_with_group(const moveit::core::LinkModel* link,
                      const moveit::core::JointModelGroup& arm) {
  for (const moveit::core::JointModel* joint = link->getParentJointModel(); joint != nullptr;) {
    if (arm.hasJointModel(joint->getName())) {
      return true;
    }
    const moveit::core::LinkModel* parent = joint->getParentLinkModel();
    joint = parent == nullptr ? nullptr : parent->getParentJointModel();
  }
  return false;
}

}  // namespace

void seed_moveit(std::uint32_t seed) {
  {
    const std::lock_guard<std::mutex> held(moveit_seeds_lock());
    moveit_seeds().seed(seed);
  }
  // KDL's step off a singularity draws from Eigen's Random, which reads std::rand.
  std::srand(seed);
}

ChainGroup chains_of(const nlohmann::json& scenario) {
  ChainGroup chains{group, {}};
  for (const auto& chain : scenario.at("chains")) {
    chains.chains.push_back({chain.at("name").get<std::string>(),
                             chain.at("base_link").get<std::string>(),
                             chain.at("tip_link").get<std::string>()});
  }
  return chains;
}

moveit::core::RobotModelPtr arm_model(const nlohmann::json& scenario, const ChainGroup& chains,
                                      Inputs& inputs) {
  std::vector<std::string> joints;
  for (const auto& chain : scenario.at("chains")) {
    const auto named = chain.at("joints").get<std::vector<std::string>>();
    joints.insert(joints.end(), named.begin(), named.end());
  }
  auto model = robot_model_with_meshes(inputs, scenario.at("urdf").get<std::string>(),
                                       scenario.at("meshes").get<std::string>(), chains);
  if (model->getJointModelGroup(group)->getActiveJointModelNames() != joints) {
    throw std::runtime_error(
        "the group of the scenario's chains holds other joints than the chains' joints, or "
        "holds them in another order");
  }
  // A URDF bounds no acceleration, and the pipeline's time parameterization needs one,
  // as a moveit_config's joint_limits.yaml gives it.
  for (const auto* joint : model->getJointModelGroup(group)->getActiveJointModels()) {
    auto* writable = model->getJointModel(joint->getName());
    for (const auto& variable : writable->getVariableNames()) {
      moveit::core::VariableBounds bounds = writable->getVariableBounds(variable);
      bounds.acceleration_bounded_ = true;
      bounds.min_acceleration_ = -max_acceleration;
      bounds.max_acceleration_ = max_acceleration;
      writable->setVariableBounds(variable, bounds);
    }
  }
  return model;
}

void load_kdl(const moveit::core::RobotModelPtr& model,
              const std::vector<std::string>& solver_groups) {
  start_ros();
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  std::vector<rclcpp::Parameter> solvers;
  for (const auto& name : solver_groups) {
    solvers.emplace_back("robot_description_kinematics." + name + ".kinematics_solver",
                         "kdl_kinematics_plugin/KDLKinematicsPlugin");
  }
  options.parameter_overrides(solvers);
  auto node = std::make_shared<rclcpp::Node>("arm_planning_kinematics", options);
  auto loader = std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>(node);
  const auto load = loader->getLoaderFunction(std::make_shared<srdf::Model>(*model->getSRDF()));
  // The model calls the loader for each solver it allocates after this, as a pose space does for
  // its own, so the loader lives as long as the allocators, as RobotModelLoader keeps it.
  const moveit::core::SolverAllocatorFn allocator =
      [loader, load](const moveit::core::JointModelGroup* group) { return load(group); };
  std::map<std::string, moveit::core::SolverAllocatorFn> allocators;
  for (const auto& name : loader->getKnownGroups()) {
    allocators[name] = allocator;
  }
  model->setKinematicsAllocators(allocators);
  for (const auto& [name, timeout] : loader->getIKTimeout()) {
    if (model->hasJointModelGroup(name)) {
      model->getJointModelGroup(name)->setDefaultIKTimeout(timeout);
    }
  }
  for (const auto& name : solver_groups) {
    if (!model->getJointModelGroup(name)->getSolverInstance()) {
      throw std::runtime_error("MoveIt's KDL plugin loads no inverse kinematics for " + name);
    }
  }
}

std::vector<double> numbers(const nlohmann::json& values) {
  return values.get<std::vector<double>>();
}

planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& s, double voxel_size,
                                         const nlohmann::json& checked_pairs, double padding) {
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCollisionEnvNonConst()->setPadding(padding);
  auto tree = std::make_shared<octomap::OcTree>(voxel_size);
  for (const auto& box : s.at("boxes")) {
    const auto first = box.at("first").get<std::vector<int>>();
    const auto last = box.at("last").get<std::vector<int>>();
    for (int x = first[0]; x <= last[0]; ++x) {
      for (int y = first[1]; y <= last[1]; ++y) {
        for (int z = first[2]; z <= last[2]; ++z) {
          tree->updateNode(octomap::OcTreeKey(static_cast<uint16_t>(x + tree_max_val),
                                              static_cast<uint16_t>(y + tree_max_val),
                                              static_cast<uint16_t>(z + tree_max_val)),
                           true);
        }
      }
    }
  }
  scene->getWorldNonConst()->addToObject(planning_scene::PlanningScene::OCTOMAP_NS,
                                         std::make_shared<const shapes::OcTree>(tree),
                                         Eigen::Isometry3d::Identity());
  const auto frame = model->getModelFrame();
  for (const auto& entry : s.at("objects")) {
    if (!scene->processCollisionObjectMsg(object_added(entry, frame))) {
      throw std::runtime_error("MoveIt refused the object " + entry.dump());
    }
  }
  auto& state = scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setJointGroupPositions(group, numbers(s.at("start")));
  state.update();
  const auto attach = [&](const nlohmann::json& held, const std::string& link) {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = link;
    attached.object = object_added(held.at("object"), link);
    attached.touch_links = held.at("touch_links").get<std::vector<std::string>>();
    if (!scene->processAttachedCollisionObjectMsg(attached)) {
      throw std::runtime_error("MoveIt refused the attached object " + held.dump());
    }
  };
  for (const auto& held : s.at("attached")) {
    attach(held, held.at("link").get<std::string>());
  }
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  const auto& links = model->getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(links, links, true);
  const auto& arm = *model->getJointModelGroup(group);
  std::vector<std::string> standing;
  for (const auto* link : model->getLinkModelsWithCollisionGeometry()) {
    if (!moves_with_group(link, arm)) {
      acm.setDefaultEntry(link->getName(), true);
      standing.push_back(link->getName());
    }
  }
  for (const auto& held : s.at("attached")) {
    const auto id = held.at("object").at("id").get<std::string>();
    const auto link = held.at("link").get<std::string>();
    if (moves_with_group(model->getLinkModel(link), arm)) {
      // An object an arm of the group holds meets the links that stand still, as it meets
      // the parts of the robot motion_planning_core's plan does not move.
      acm.setEntry(id, standing, false);
    } else {
      // An object an arm outside the group holds stands still with that arm: it meets the
      // arms the plan moves and what they hold, as an obstacle such an arm holds meets them.
      acm.setEntry(id, scene->getWorld()->getObjectIds(), true);
      acm.setEntry(id, standing, true);
    }
  }
  const auto& bodies = model->getLinkModelNamesWithCollisionGeometry();
  for (const auto& pair : checked_pairs) {
    const auto a = pair.at(0).get<std::string>();
    const auto b = pair.at(1).get<std::string>();
    for (const auto& name : {a, b}) {
      if (std::find(bodies.begin(), bodies.end(), name) == bodies.end()) {
        throw std::runtime_error("the checked pair " + a + ", " + b + " names " + name +
                                 ", no link of the robot with collision geometry");
      }
    }
    acm.setEntry(a, b, false);
  }
  for (const auto& pair : s.at("allowed")) {
    acm.setEntry(pair.at(0).get<std::string>(), pair.at(1).get<std::string>(), true);
  }
  return scene;
}

}  // namespace ros_reference
